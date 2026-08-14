#!/usr/bin/env python3
"""USB camera capture tuned for OCR rather than for video.

Two things here are not obvious and both were measured on the target hardware
(Raspberry Pi 5, Microdia "Vitade AF" webcam on ``/dev/video0``):

**The V4L2 buffer queue is not one frame deep.** ``CAP_PROP_BUFFERSIZE = 1`` is
accepted — ``cap.get`` even returns 1.0 — but the kernel queue is typically 4
and the setting is a no-op on this backend. With capture at ~15 fps (64 ms) and
OCR at a few hundred ms, every ``read()`` after a recognition pass hands back a
frame queued hundreds of milliseconds ago. Moving the paper then appears to do
nothing for seconds, which reads as "the camera is broken". :meth:`Camera.grab`
drains the queue with ``grab()`` and only decodes the last one.

**Exposure must be set through ``v4l2-ctl``, not OpenCV.** The UVC value
mapping behind ``CAP_PROP_AUTO_EXPOSURE`` is inconsistent across drivers and
silently no-ops here. Shelling out is ugly and it works. The camera's defaults
are actively hostile to OCR: aperture-priority auto exposure trades frame rate
for light (hence 15 fps and motion blur), and the anti-flicker filter defaults
to 50 Hz on 60 Hz mains, which bands the image under LED lighting.

This webcam exposes **no focus controls at all** despite the "AF" in its name,
so focus cannot be locked. :func:`focus_score` exists instead: a human watches
the number and slides the paper until it peaks.
"""

from __future__ import annotations

import shutil
import subprocess
from dataclasses import dataclass, field

import cv2
import numpy as np

# Applied at start-up. 60 Hz because the robot is in Korea, and manual exposure
# because the camera's own metering averages the whole scene — a white wall with
# white paper on it reads as "bright", so it exposes for the average and drives
# the plate's thin print into saturation.
#
# The starting value only has to be in the right neighbourhood; :mod:`exposure`
# meters the plate region every frame and moves it, so the robot keeps reading
# when the lighting changes. Set ``exposure.mode`` to ``camera`` to hand control
# back to the sensor, or ``manual`` to pin whatever is set here.
DEFAULT_V4L2_CONTROLS = (
    "power_line_frequency=2",  # 0=off, 1=50Hz, 2=60Hz
    "auto_exposure=1",  # 1=manual, 3=aperture priority
    "exposure_time_absolute=60",  # units of 100 us -> 6 ms; a starting point
    "exposure_dynamic_framerate=0",
    "backlight_compensation=0",
    "gain=0",
)

# Substituted for DEFAULT_V4L2_CONTROLS when the exposure mode is "camera".
CAMERA_AUTO_EXPOSURE_CONTROLS = (
    "power_line_frequency=2",
    "auto_exposure=3",  # aperture priority: the sensor decides
    "backlight_compensation=0",
)

EXPOSURE_CONTROL = "exposure_time_absolute"


class CameraError(RuntimeError):
    """The camera could not be opened or produced no frame."""


@dataclass
class CameraSettings:
    device: str = "/dev/video0"
    # 1080p, not 720p. Measured, a frame costs the same 64 ms either way — the
    # bottleneck is exposure, not pixels — and this camera's 720p mode is a
    # narrower crop that left the plate's glyphs at ~20 px, under Tesseract's
    # 30-40 px working range. At 1080p the same plate reads at ~55 px.
    width: int = 1920
    height: int = 1080
    fps: int = 30
    fourcc: str = "MJPG"
    rotate: int = 0  # 0, 90, 180 or 270 degrees, clockwise
    drain_frames: int = 4  # queued frames to discard before each capture
    v4l2_controls: tuple[str, ...] = field(default_factory=lambda: DEFAULT_V4L2_CONTROLS)

    def __post_init__(self) -> None:
        if self.rotate not in (0, 90, 180, 270):
            raise ValueError(f"rotate must be 0, 90, 180 or 270, got {self.rotate}")
        if len(self.fourcc) != 4:
            raise ValueError(f"fourcc must be 4 characters, got {self.fourcc!r}")
        if self.width <= 0 or self.height <= 0:
            raise ValueError(f"invalid frame size {self.width}x{self.height}")
        if self.drain_frames < 0:
            raise ValueError(f"drain_frames must be >= 0, got {self.drain_frames}")


_ROTATIONS = {
    90: cv2.ROTATE_90_CLOCKWISE,
    180: cv2.ROTATE_180,
    270: cv2.ROTATE_90_COUNTERCLOCKWISE,
}


def apply_v4l2_controls(device: str, controls: tuple[str, ...] | list[str]) -> list[str]:
    """Push ``name=value`` controls with ``v4l2-ctl``. Returns what went wrong.

    Failure is never fatal: a control that this camera does not expose is worth
    a warning, not a dead node.
    """
    if not controls:
        return []
    if shutil.which("v4l2-ctl") is None:
        return ["v4l2-ctl not found: sudo apt install v4l-utils"]

    problems: list[str] = []
    for control in controls:
        completed = subprocess.run(
            ["v4l2-ctl", "-d", device, "-c", control],
            capture_output=True,
            check=False,
        )
        if completed.returncode != 0:
            detail = completed.stderr.decode("utf-8", "replace").strip()
            problems.append(f"{control}: {detail or 'failed'}")
    return problems


def set_exposure(device: str, value: int) -> str | None:
    """Set ``exposure_time_absolute``. Returns a message on failure."""
    problems = apply_v4l2_controls(device, (f"{EXPOSURE_CONTROL}={value}",))
    return problems[0] if problems else None


def focus_score(gray: np.ndarray) -> float:
    """Variance of the Laplacian — higher is sharper.

    Only comparable against itself on the same scene, which is exactly the use:
    the number is printed on the debug image so a person can move the paper
    until it peaks. Single digits to ~50 is blurry; a sharp plate reads in the
    hundreds.
    """
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


class Camera:
    """A V4L2 capture that hands out *fresh* frames."""

    def __init__(self, settings: CameraSettings) -> None:
        self.settings = settings
        self.control_problems: list[str] = apply_v4l2_controls(
            settings.device, settings.v4l2_controls
        )

        self._capture = cv2.VideoCapture(settings.device, cv2.CAP_V4L2)
        if not self._capture.isOpened():
            raise CameraError(f"cannot open {settings.device}")
        # Order matters: FOURCC before the frame size, or the driver may pick a
        # YUYV mode that caps 720p at 10 fps.
        self._capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*settings.fourcc))
        self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, settings.width)
        self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, settings.height)
        self._capture.set(cv2.CAP_PROP_FPS, settings.fps)

    def set_exposure(self, value: int) -> str | None:
        """Change the exposure mid-run, for the adaptive loop."""
        return set_exposure(self.settings.device, value)

    @property
    def actual_size(self) -> tuple[int, int]:
        return (
            int(self._capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
            int(self._capture.get(cv2.CAP_PROP_FRAME_HEIGHT)),
        )

    def grab(self) -> np.ndarray:
        """Discard the queued frames, then decode and return the newest one."""
        for _ in range(self.settings.drain_frames):
            self._capture.grab()
        ok, frame = self._capture.retrieve() if self.settings.drain_frames else self._capture.read()
        if not ok or frame is None:
            raise CameraError(f"no frame from {self.settings.device}")
        rotation = _ROTATIONS.get(self.settings.rotate)
        return cv2.rotate(frame, rotation) if rotation is not None else frame

    def close(self) -> None:
        self._capture.release()

    def __enter__(self) -> Camera:
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.close()
