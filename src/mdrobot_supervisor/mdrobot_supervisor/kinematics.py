"""Mecanum inverse kinematics. Pure math — no I/O, no ROS.

Convention (ROS REP-103, right-handed, z up): +x forward, +y LEFT, +wz
counter-clockwise seen from above. Wheel order is fixed everywhere:

    0 = front_left   1 = front_right   2 = rear_left   3 = rear_right

`omega_i` is rad/s at the WHEEL, positive when that wheel's rotation would drive
the robot forward. Motor mounting direction and gear ratio are applied later, by
the caller, after the clamp — one concept per layer.

Derivation
----------
Wheel *i* at body position ``(x_i, y_i)`` has ground-contact velocity::

    v_ix = vx - wz * y_i
    v_iy = vy + wz * x_i

A mecanum wheel's passive rollers slide freely perpendicular to their own axle,
so the only no-slip constraint is *along* the roller axle. With
``d_i = +/-1`` for 45-degree rollers::

    r * omega_i = v_ix + d_i * v_iy = vx + d_i * vy + wz * (d_i * x_i - y_i)

With ``lx = wheelbase / 2`` and ``ly = track / 2``, the wz coefficient
``(d_i * x_i - y_i)`` evaluates to ``+/-(lx + ly)`` — the terms ADD rather than
cancel — only for the handedness pattern::

    d = (FL, FR, RL, RR) = (-1, +1, +1, -1)

The other pattern ``(+1, -1, -1, +1)`` yields a wz coefficient of ``(lx - ly)``:
a square-footprint base built that way cannot rotate at all, because the wz
column is then identically zero. That is a build error, not a setting — the left
and right wheels have to be physically swapped — and it shows up on the floor as
"+wz barely turns and the tyres scrub".

roller_layout
-------------
One bit that flips the **vy column only** (``sy = +1`` for "x", ``-1`` for "o");
vx and wz are untouched. Its real meaning is "does +vy strafe left or right".

It cannot be determined by eye: rotating a roller 180 degrees about the wheel
axle maps its projected direction ``(a, b) -> (-a, b)``, so the top roller you
see is the mirror of the ground-contact roller that does the work. "unknown" is
therefore a first-class value — it computes as "x" so the robot is drivable, but
the strafe DIRECTION is unverified until a floor test settles it.

This module restates the convention used by the earlier `mdrobot_mecanum`
package, whose sources are no longer in the repository, and by the wheel/roller
fields of `mecanum.yaml`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

WHEEL_NAMES = ("front_left", "front_right", "rear_left", "rear_right")

# Roller handedness d_i, in WHEEL_NAMES order. See the derivation above: this is
# the only pattern that lets a square base yaw. It is also the vy column.
ROLLER_HANDEDNESS = (-1.0, +1.0, +1.0, -1.0)

# The wz column, ``(d_i * x_i - y_i) / (lx + ly)``. NOT the same vector as
# ROLLER_HANDEDNESS: evaluating the coefficient wheel by wheel gives
#   FL (+lx, +ly, d=-1) -> -(lx+ly)      FR (+lx, -ly, d=+1) -> +(lx+ly)
#   RL (-lx, +ly, d=+1) -> -(lx+ly)      RR (-lx, -ly, d=-1) -> +(lx+ly)
# i.e. both LEFT wheels negative and both RIGHT wheels positive, which is what
# yaw has to look like. Reusing d_i here would flip the two rear wheels and the
# base would scrub instead of turning.
WZ_COLUMN = (-1.0, +1.0, -1.0, +1.0)

ROLLER_LAYOUTS = ("x", "o", "unknown")
PROVISIONAL_LAYOUT = "unknown"


def layout_sign(roller_layout: str) -> float:
    """Return the vy-column sign for a layout name. "unknown" computes as "x"."""
    if roller_layout in ("x", PROVISIONAL_LAYOUT):
        return 1.0
    if roller_layout == "o":
        return -1.0
    raise ValueError(
        f"roller_layout must be one of {ROLLER_LAYOUTS}, got {roller_layout!r}"
    )


@dataclass(frozen=True)
class MecanumGeometry:
    """Everything the kinematics needs about the base, in metres."""

    wheel_radius: float
    track: float  # left-right wheel centre distance
    wheelbase: float  # front-rear wheel centre distance
    roller_layout: str = PROVISIONAL_LAYOUT

    def __post_init__(self) -> None:
        for name in ("wheel_radius", "track", "wheelbase"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be positive and finite, got {value}")
        layout_sign(self.roller_layout)  # validates

    @property
    def lxy(self) -> float:
        """Half-wheelbase plus half-track — the wz lever arm."""
        return (self.wheelbase + self.track) / 2.0

    @property
    def layout_is_provisional(self) -> bool:
        """True while the strafe direction has not been settled on the floor."""
        return self.roller_layout == PROVISIONAL_LAYOUT


def inverse(vx: float, vy: float, wz: float,
            geom: MecanumGeometry) -> tuple[float, float, float, float]:
    """Body twist -> wheel angular velocities.

    Args:
        vx: forward velocity, m/s (+x forward).
        vy: lateral velocity, m/s (+y LEFT).
        wz: yaw rate, rad/s (+ counter-clockwise).
        geom: robot geometry.

    Returns:
        (front_left, front_right, rear_left, rear_right) in rad/s at the wheel,
        positive = drives the robot forward. Gear ratio is NOT applied.
    """
    sy = layout_sign(geom.roller_layout)
    lxy = geom.lxy
    r = geom.wheel_radius
    return tuple(  # type: ignore[return-value]
        (vx + d * sy * vy + w * lxy * wz) / r
        for d, w in zip(ROLLER_HANDEDNESS, WZ_COLUMN)
    )


def scale_to_limit(values, limit: float) -> tuple[list[float], float]:
    """Scale a wheel-speed vector down uniformly so no element exceeds `limit`.

    Returns:
        (scaled, k) with k in (0, 1]. k == 1.0 means nothing was clamped.

    Why uniform, not per-wheel clipping: the inverse kinematics is LINEAR, so
    multiplying all four wheel speeds by k is exactly identical to having asked
    for k * (vx, vy, wz). The robot therefore follows the SAME path, just
    slower. Clipping wheels individually changes the ratios between them, which
    changes the DIRECTION — a commanded straight line becomes an arc and a
    commanded strafe picks up an unwanted yaw, precisely when the operator has
    asked for the most speed and has the least margin.
    """
    if not math.isfinite(limit) or limit <= 0:
        raise ValueError(f"limit must be a positive finite number, got {limit}")
    values = [float(v) for v in values]
    if any(not math.isfinite(v) for v in values):
        raise ValueError(f"wheel speeds must be finite, got {values}")
    peak = max((abs(v) for v in values), default=0.0)
    if peak <= limit:
        return values, 1.0
    k = limit / peak
    return [v * k for v in values], k


def rad_s_to_motor_rpm(omega: float, gear_ratio: float) -> float:
    """Wheel rad/s -> rpm at the MOTOR shaft, which is what the controller takes."""
    return omega * 60.0 / (2.0 * math.pi) * gear_ratio
