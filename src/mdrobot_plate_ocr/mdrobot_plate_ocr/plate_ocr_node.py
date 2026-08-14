#!/usr/bin/env python3
"""ROS 2 node: read a Korean licence plate from a USB camera, publish the text.

Deliberately publishes **no images**. The robot is reached over SSH, and
streaming camera frames to a remote RViz was too slow to see anything — which
is the whole reason this node exists. Look at ``debug_dir/latest.jpg`` in an
editor instead; it is written atomically and carries the focus score, the glyph
height and the raw OCR text as an overlay.

Interface
---------

Parameters (see ``config/plate_ocr.yaml`` for the full annotated set):
    device, width, height, fps, rotate, v4l2_controls
        Camera. ``v4l2_controls`` is applied by shelling out to ``v4l2-ctl``
        because OpenCV's ``CAP_PROP_AUTO_EXPOSURE`` silently no-ops on this
        UVC driver. A control this camera lacks is a warning, not an error.
    exposure_mode, exposure_start, exposure_target_mean, exposure_min/max
        ``adaptive`` (default) meters the region the OCR reads and tracks the
        lighting, so a fixed value measured on the bench does not stop working
        in a darker room. ``camera`` hands metering back to the sensor — which
        overexposes a white plate, because it averages the whole scene.
        ``manual`` pins ``exposure_start``.
    hfov_deg
        Horizontal field of view, used only to turn the plate's pixel offset
        into an angle.
    ocr_rate
        Hz. One iteration measured ~180 ms end to end in the probe, but 4.0 Hz
        overran its 250 ms budget on the node's slower frames, so the default is
        3.0. An overrun logs a throttled warning.
    lang, digit_lang, ocr_backend, psm
        ``digit_lang`` must stay ``eng``. With ``kor`` and a digit whitelist
        Tesseract returns *nothing* — the Korean LSTM's preferred path is
        Hangul and whitelisting prunes it away.
    detector, strategy, syllable_source, roi, upscale, preprocess
        Recognition pipeline; see :mod:`reader`.
    plate_regex, require_legal_syllable, min_confidence, min_syllable_margin
        Validation.
    confirm_count, history_size, repeat_interval_s
        Publish debouncing; see :mod:`debounce`.
    debug_dir, debug_ring_size, debug_keep_hits, publish_detail

Publishers:
    ~/plate (std_msgs/String)
        A confirmed plate, and nothing else — silence means nothing was read.
        **Latched** (transient local, depth 1): a requirement of "publish
        nothing when nothing is recognised" is that someone who runs
        ``ros2 topic echo`` after a successful read sees an empty screen and
        concludes the node is broken. A late subscriber gets the last plate.
    ~/plate_offset (geometry_msgs/Point)
        Where the plate sits relative to the centre of the frame, published on
        every accepted read rather than on debounced confirmation, because a
        control loop wants it at frame rate. ``x`` and ``y`` are normalised to
        [-1, 1] — ``x`` positive means the plate is to the **right** of centre,
        ``y`` positive means **below** — so they can be used directly as an
        error signal. ``z`` is the plate's width as a fraction of the frame's,
        a crude proxy for range. Pixel offsets and the estimated bearing are in
        ``~/plate_detail``.
    ~/plate_detail (std_msgs/String, JSON)
        Every OCR attempt including the rejects: raw and normalised text,
        confidence, region, segment count, template match, focus, glyph height,
        exposure, centre offset, elapsed time, reject reason. Echoing this
        answers "is it alive and what is it looking at" over SSH without
        opening a JPEG.

JSON inside a String rather than a custom message: a ``.msg`` needs ``rosidl``,
which would force this package to ``ament_cmake`` and break the workspace's
``ament_python`` convention for Python nodes. Please do not "fix" it.

No motor is involved. The repository's hardware safety protocol does not apply.
"""

from __future__ import annotations

import json

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from std_msgs.msg import String

from .camera import (
    CAMERA_AUTO_EXPOSURE_CONTROLS,
    DEFAULT_V4L2_CONTROLS,
    Camera,
    CameraError,
    CameraSettings,
)
from .debounce import Debouncer
from .debug import DebugWriter
from .exposure import ExposureController, ExposureSettings
from .normalize import DEFAULT_PLATE_PATTERN
from .ocr import DEFAULT_LANG, available_languages, make_engine
from .reader import DEFAULT_HFOV_DEG, PlateReader, ReadResult, ReadSettings

UNSET_ROI = [-1, -1, -1, -1]


class PlateOcrNode(Node):
    """Camera -> OCR -> plate string on a topic."""

    def __init__(self) -> None:
        super().__init__("mdrobot_plate_ocr")

        # --- parameters ------------------------------------------------------
        self.declare_parameter("device", "/dev/video0")
        self.declare_parameter("width", 1920)
        self.declare_parameter("height", 1080)
        self.declare_parameter("fps", 30)
        self.declare_parameter("rotate", 0)
        self.declare_parameter("v4l2_controls", list(DEFAULT_V4L2_CONTROLS))
        self.declare_parameter("exposure_mode", "adaptive")
        self.declare_parameter("exposure_start", 60)
        self.declare_parameter("exposure_min", 4)
        self.declare_parameter("exposure_max", 1500)
        self.declare_parameter("exposure_target_mean", 150.0)
        self.declare_parameter("hfov_deg", DEFAULT_HFOV_DEG)
        self.declare_parameter("ocr_rate", 3.0)
        self.declare_parameter("ocr_backend", "auto")
        self.declare_parameter("lang", DEFAULT_LANG)
        self.declare_parameter("digit_lang", "eng")
        self.declare_parameter("psm", 7)
        self.declare_parameter("detector", "textband")
        self.declare_parameter("strategy", "split")
        self.declare_parameter("syllable_source", "template")
        self.declare_parameter("preprocess", "none")
        self.declare_parameter("upscale", 3.0)
        # [-1, -1, -1, -1] means "no region of interest"; the detector searches
        # the whole frame. A real box narrows the search before it runs.
        self.declare_parameter("roi", UNSET_ROI)
        self.declare_parameter("plate_regex", DEFAULT_PLATE_PATTERN)
        self.declare_parameter("require_legal_syllable", True)
        self.declare_parameter("min_confidence", 0.0)
        self.declare_parameter("min_syllable_margin", 0.0)
        self.declare_parameter("confirm_count", 3)
        self.declare_parameter("history_size", 5)
        self.declare_parameter("repeat_interval_s", 3.0)
        self.declare_parameter("debug_dir", "")  # "" disables debug images
        self.declare_parameter("debug_ring_size", 30)
        self.declare_parameter("debug_keep_hits", True)
        self.declare_parameter("publish_detail", True)

        rate = float(self.get_parameter("ocr_rate").value)
        if rate <= 0.0:
            raise ValueError(f"ocr_rate must be > 0, got {rate}")
        lang = str(self.get_parameter("lang").value)
        digit_lang = str(self.get_parameter("digit_lang").value)
        backend = str(self.get_parameter("ocr_backend").value)

        installed = available_languages()
        for name in {lang, digit_lang}:
            missing = [part for part in name.split("+") if part not in installed]
            if missing:
                raise ValueError(
                    f"Tesseract language {'+'.join(missing)!r} is not installed "
                    f"(have: {', '.join(installed) or 'none'}); "
                    f"try: sudo apt install tesseract-ocr-{missing[0]}"
                )

        roi_values = [int(value) for value in self.get_parameter("roi").value]
        roi = None if roi_values == UNSET_ROI else tuple(roi_values)
        if roi is not None and len(roi) != 4:
            raise ValueError(f"roi needs four values [x, y, w, h], got {roi_values}")

        # ReadSettings and CameraSettings validate their own enums and ranges
        # and raise ValueError, which is what this node wants anyway.
        self._settings = ReadSettings(
            roi=roi,  # type: ignore[arg-type]
            detector=str(self.get_parameter("detector").value),
            strategy=str(self.get_parameter("strategy").value),
            syllable_source=str(self.get_parameter("syllable_source").value),
            preprocess=str(self.get_parameter("preprocess").value),
            upscale=float(self.get_parameter("upscale").value),
            psm=int(self.get_parameter("psm").value),
            pattern=str(self.get_parameter("plate_regex").value),
            min_confidence=float(self.get_parameter("min_confidence").value),
            require_legal_syllable=bool(self.get_parameter("require_legal_syllable").value),
            min_syllable_margin=float(self.get_parameter("min_syllable_margin").value),
            hfov_deg=float(self.get_parameter("hfov_deg").value),
        )

        self._exposure = ExposureController(
            ExposureSettings(
                mode=str(self.get_parameter("exposure_mode").value),
                start=int(self.get_parameter("exposure_start").value),
                minimum=int(self.get_parameter("exposure_min").value),
                maximum=int(self.get_parameter("exposure_max").value),
                target_mean=float(self.get_parameter("exposure_target_mean").value),
            )
        )
        self._camera_settings = CameraSettings(
            device=str(self.get_parameter("device").value),
            width=int(self.get_parameter("width").value),
            height=int(self.get_parameter("height").value),
            fps=int(self.get_parameter("fps").value),
            rotate=int(self.get_parameter("rotate").value),
            v4l2_controls=self._exposure_controls(),
        )

        # --- publishers ------------------------------------------------------
        # Transient local so a subscriber that attaches after the read still
        # sees the plate; without it "no message means nothing was recognised"
        # is indistinguishable from "the node is dead".
        self._plate_pub = self.create_publisher(
            String,
            "~/plate",
            QoSProfile(
                depth=1,
                history=HistoryPolicy.KEEP_LAST,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self._publish_detail = bool(self.get_parameter("publish_detail").value)
        self._detail_pub = (
            self.create_publisher(String, "~/plate_detail", 10) if self._publish_detail else None
        )
        # Best-effort depth 10, not latched: an offset goes stale the moment the
        # robot moves, so replaying an old one to a late subscriber would be
        # worse than saying nothing.
        self._offset_pub = self.create_publisher(Point, "~/plate_offset", 10)

        # --- pipeline --------------------------------------------------------
        self._engine = make_engine(backend, lang)
        self._digit_engine = make_engine(backend, digit_lang)
        self._reader = PlateReader(self._engine, self._settings, self._digit_engine)
        self._debouncer = Debouncer(
            confirm_count=int(self.get_parameter("confirm_count").value),
            history_size=int(self.get_parameter("history_size").value),
            repeat_interval_s=float(self.get_parameter("repeat_interval_s").value),
        )

        debug_dir = str(self.get_parameter("debug_dir").value)
        self._debug = (
            DebugWriter(
                debug_dir,
                ring_size=int(self.get_parameter("debug_ring_size").value),
                keep_hits=bool(self.get_parameter("debug_keep_hits").value),
            )
            if debug_dir
            else None
        )

        self._camera = Camera(self._camera_settings)
        for problem in self._camera.control_problems:
            self.get_logger().warn(f"v4l2 {problem}")

        self._frames = 0
        self._period = 1.0 / rate
        self._timer = self.create_timer(self._period, self._tick)
        self.get_logger().info(
            f"reading {self._camera.actual_size[0]}x{self._camera.actual_size[1]} "
            f"from {self._camera_settings.device} at {rate} Hz "
            f"(backend {self._engine.name}, lang {lang}, digits {digit_lang}, "
            f"detector {self._settings.detector}, strategy {self._settings.strategy})"
        )
        if self._debug is not None:
            self.get_logger().info(f"debug images -> {self._debug.directory}/latest.jpg")

    # --- main loop -----------------------------------------------------------

    def _tick(self) -> None:
        started = self.get_clock().now()
        try:
            frame = self._camera.grab()
        except CameraError as exc:
            self.get_logger().error(f"capture failed: {exc}", throttle_duration_sec=2.0)
            self._debouncer.miss()
            return

        result = self._reader.read(frame)
        self._frames += 1
        now = started.nanoseconds * 1e-9

        accepted = result.accepted
        published = (
            self._debouncer.offer(accepted.candidate.text, now)
            if accepted is not None
            else (self._debouncer.miss(), None)[1]
        )
        if published:
            self._plate_pub.publish(String(data=published))
            self.get_logger().info(f"plate {published}")
        if accepted is not None and result.offset is not None:
            offset = result.offset
            self._offset_pub.publish(
                Point(x=offset.dx_norm, y=offset.dy_norm, z=offset.width_ratio)
            )
        if self._detail_pub is not None:
            self._detail_pub.publish(String(data=self._detail(result, published)))
        if self._debug is not None:
            self._debug.write(frame, result, f"#{self._frames} exp {self._exposure.value}")

        self._track_exposure(result)

        elapsed_s = (self.get_clock().now() - started).nanoseconds * 1e-9
        if elapsed_s > self._period:
            self.get_logger().warn(
                f"iteration took {elapsed_s * 1e3:.0f}ms, over the "
                f"{self._period * 1e3:.0f}ms budget; lower ocr_rate",
                throttle_duration_sec=5.0,
            )

    def _exposure_controls(self) -> tuple[str, ...]:
        """Start-up v4l2 controls for the configured exposure mode."""
        if self._exposure.settings.mode == "camera":
            return CAMERA_AUTO_EXPOSURE_CONTROLS
        declared = tuple(str(c) for c in self.get_parameter("v4l2_controls").value)
        start = self._exposure.settings.start
        return tuple(
            f"exposure_time_absolute={start}"
            if control.startswith("exposure_time_absolute")
            else control
            for control in declared
        )

    def _track_exposure(self, result: ReadResult) -> None:
        """Meter the OCR region and move the exposure if the lighting drifted.

        Metering the region rather than the frame is the point: the camera's own
        metering averages a mostly-white scene and blows out the plate's print.
        """
        best = result.best
        if best is None:
            metered = result.gray
        else:
            x, y, w, h = best.region
            metered = result.gray[y : y + h, x : x + w]

        value = self._exposure.update(np.asarray(metered))
        if value is None:
            return
        problem = self._camera.set_exposure(value)
        if problem:
            self.get_logger().warn(f"exposure {problem}", throttle_duration_sec=10.0)
        else:
            self.get_logger().info(
                f"exposure -> {value} ({self._exposure.reason})", throttle_duration_sec=5.0
            )

    def _detail(self, result: ReadResult, published: str | None) -> str:
        """One JSON line describing the frame, rejects included."""
        best = result.best
        detail: dict[str, object] = {
            "frame": self._frames,
            "published": published,
            "focus": round(result.focus, 1),
            "glyph_height": round(result.glyph_height, 1),
            "elapsed_ms": round(result.elapsed_ms, 1),
            "attempts": len(result.attempts),
            "exposure": self._exposure.value,
            "exposure_reason": self._exposure.reason,
        }
        if result.offset is not None:
            offset = result.offset
            detail["offset"] = {
                "dx_px": round(offset.dx_px, 1),
                "dy_px": round(offset.dy_px, 1),
                "dx_norm": round(offset.dx_norm, 3),
                "dy_norm": round(offset.dy_norm, 3),
                "bearing_deg": round(offset.bearing_deg, 2),
                "elevation_deg": round(offset.elevation_deg, 2),
                "width_ratio": round(offset.width_ratio, 3),
                "centre_px": [round(v, 1) for v in offset.centre_px],
            }
        if best is not None:
            detail.update(
                raw=best.candidate.cleaned,
                text=best.candidate.text,
                accepted=best.accepted,
                reason=best.reason,
                confidence=round(best.ocr.confidence, 1),
                region=list(best.region),
                segments=best.segments,
                tesseract_syllable=best.tesseract_syllable,
            )
            if best.syllable is not None:
                detail.update(
                    syllable=best.syllable.syllable,
                    syllable_score=round(best.syllable.score, 3),
                    syllable_margin=round(best.syllable.margin, 3),
                    syllable_runner_up=best.syllable.runner_up,
                )
        return json.dumps(detail, ensure_ascii=False)

    # --- shutdown ------------------------------------------------------------

    def shutdown(self) -> None:
        """Release the camera and the OCR engines, each independently."""
        for name, close in (
            ("camera", getattr(self, "_camera", None)),
            ("ocr engine", getattr(self, "_engine", None)),
            ("digit engine", getattr(self, "_digit_engine", None)),
        ):
            if close is None:
                continue
            try:
                close.close()
            except Exception as exc:  # noqa: BLE001 - shutdown races
                self.get_logger().warn(f"closing {name} failed: {type(exc).__name__}: {exc}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = PlateOcrNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
