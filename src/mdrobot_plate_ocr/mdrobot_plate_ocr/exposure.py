#!/usr/bin/env python3
"""Keep the plate readable as the lighting changes, by metering the plate.

A fixed exposure is only right for the room it was measured in. 6 ms suited the
bench; the same value in a darker corridor produces a black frame and the OCR
simply stops working, with nothing in the log to say why.

The camera's own auto exposure is not the answer either — it is what produced
the first blown-out frames here. It meters the whole scene, and a scene that is
mostly white wall and white paper reads as "bright", so it exposes for the
average and drives the plate's thin grey print into saturation. Sweeping this
plate showed contrast peaking between 2 and 8 ms and collapsing at the camera's
own 15.6 ms choice: standard deviation 28 against 15.

So this meters **the region the OCR actually reads** and moves the exposure a
step at a time towards a target. Two guards, because mean brightness alone is
not enough:

* a saturation limit, since a plate can be blown out while the frame's mean
  still looks reasonable — and clipped pixels are unrecoverable;
* a settle delay, because a UVC exposure change takes a few frames to appear
  and reacting to stale frames oscillates.

Nothing here is ROS-specific and nothing captures: it is handed a grayscale
region and returns the new exposure, so it can be unit-tested without hardware.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

MODES = ("adaptive", "manual", "camera")


@dataclass
class ExposureSettings:
    """How the exposure loop behaves. Values are v4l2 units of 100 us."""

    mode: str = "adaptive"
    start: int = 60  # 6 ms — where the bench measurement landed
    minimum: int = 4  # 0.4 ms; below this the sensor gives noise
    maximum: int = 1500  # 150 ms; beyond this motion blur dominates
    target_mean: float = 150.0  # of the metered region, 0-255
    tolerance: float = 25.0  # dead band, so it stops hunting
    saturation_limit: float = 0.03  # fraction of pixels at 250 or above
    step: float = 1.3  # multiplicative; ~3 frames to correct a stop
    settle_frames: int = 3  # frames to wait after a change

    def __post_init__(self) -> None:
        if self.mode not in MODES:
            raise ValueError(f"exposure mode must be one of {MODES}, got {self.mode!r}")
        if not 0 < self.minimum <= self.maximum:
            raise ValueError(f"invalid exposure range {self.minimum}..{self.maximum}")
        if not self.minimum <= self.start <= self.maximum:
            raise ValueError(
                f"start {self.start} is outside {self.minimum}..{self.maximum}"
            )
        if self.step <= 1.0:
            raise ValueError(f"step must be > 1, got {self.step}")
        if self.settle_frames < 0:
            raise ValueError(f"settle_frames must be >= 0, got {self.settle_frames}")


class ExposureController:
    """Nudges the exposure towards a target measured on the OCR region."""

    def __init__(self, settings: ExposureSettings | None = None) -> None:
        self.settings = settings or ExposureSettings()
        self.value = self.settings.start
        self.reason = "start"
        self._settling = 0

    @property
    def active(self) -> bool:
        return self.settings.mode == "adaptive"

    def update(self, region: np.ndarray) -> int | None:
        """Meter one region. Returns a new exposure value, or ``None`` to hold.

        ``region`` should be the crop the OCR reads — metering the whole frame
        reintroduces exactly the averaging problem this exists to avoid.
        """
        if not self.active or region.size == 0:
            return None
        if self._settling > 0:
            self._settling -= 1
            self.reason = "settling"
            return None

        settings = self.settings
        saturated = float((region >= 250).mean())
        mean = float(region.mean())

        if saturated > settings.saturation_limit:
            # Clipped pixels carry no detail, so this outranks the mean: a plate
            # can be blown out while the average still looks about right.
            proposed = self.value / settings.step
            reason = f"saturated {saturated * 100:.1f}%"
        elif mean > settings.target_mean + settings.tolerance:
            proposed = self.value / settings.step
            reason = f"bright (mean {mean:.0f})"
        elif mean < settings.target_mean - settings.tolerance:
            proposed = self.value * settings.step
            reason = f"dark (mean {mean:.0f})"
        else:
            self.reason = f"ok (mean {mean:.0f})"
            return None

        new_value = int(round(min(max(proposed, settings.minimum), settings.maximum)))
        if new_value == self.value:
            self.reason = f"{reason}, at the {'low' if new_value == settings.minimum else 'high'} limit"
            return None

        self.value = new_value
        self.reason = reason
        self._settling = settings.settle_frames
        return new_value
