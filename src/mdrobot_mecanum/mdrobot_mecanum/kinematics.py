"""Mecanum inverse / forward kinematics. Pure math — no I/O, no `mdrobot` import.

Everything here works at the WHEEL. Motor mounting direction (`sign`) and gear ratio
are applied by `base.py` after the clamp; keeping them out makes this module exactly
testable and keeps one concept per layer.

Convention (ROS REP-103, right-handed, z up): **+x forward, +y LEFT, +wz
counter-clockwise seen from above**. Wheel order is fixed everywhere in this package:

    0 = front_left   1 = front_right   2 = rear_left   3 = rear_right

`omega_i` is rad/s at the wheel, positive when that wheel's rotation would drive the
robot forward.

Derivation
----------
Wheel *i* at body position ``(x_i, y_i)`` has ground-contact velocity::

    v_ix = vx - wz * y_i
    v_iy = vy + wz * x_i

A mecanum wheel's passive rollers slide freely perpendicular to their own axle, so
the only no-slip constraint is *along* the roller axle. With the axle direction
``a_i`` in the wheel frame and ``d_i = a_iy / a_ix = +/-1`` for 45-degree rollers::

    r * omega_i = v_ix + d_i * v_iy
                = vx + d_i * vy + wz * (d_i * x_i - y_i)

With ``lx = wheelbase / 2`` and ``ly = track / 2``, the wz coefficient
``(d_i * x_i - y_i)`` evaluates to ``+/-(lx + ly)`` — the terms ADD rather than
cancel — only for the handedness pattern::

    d = (FL, FR, RL, RR) = (-1, +1, +1, -1)

The most important consequence in this package
----------------------------------------------
The other handedness pattern ``(+1, -1, -1, +1)`` yields a wz coefficient of
``(lx - ly)``. **A square-footprint base built that way cannot rotate at all** — the
wz column of the IK matrix is identically zero, so no combination of wheel speeds
produces yaw without scrubbing.

That is a BUILD ERROR, not a configuration option, and no amount of sign-flipping in
software fixes it: the left and right wheels have to be physically swapped. The usual
"X or O, pick one" framing hides this diagnosis forever. It shows up on the floor as
"+wz barely turns and the tyres scrub".

What `roller_layout` actually is
--------------------------------
It is one bit that flips the **vy column only** (`sy = +1` for "x", `-1` for "o");
the vx and wz columns are untouched. The result is fully consistent — `slip_residual`
stays 0 — and its real meaning is "does +vy strafe left or right".

Its job is to absorb the fact that the layout **cannot be determined by eye**:
rotating a roller 180 degrees about the wheel axle maps its projected direction
``(a, b) -> (-a, b)``, so the top roller you see is the mirror of the ground-contact
roller that actually does the work. Half of the mecanum diagrams in circulation are
wrong for exactly this reason.

`"unknown"` is therefore a first-class value: it computes as "x" so the robot is
drivable and every in-the-air check still passes (those compare measured wheel speeds
against what the IK dispatched, which is layout-independent), but callers should
surface `layout_is_provisional` so the operator knows the strafe DIRECTION is not yet
verified. Only a floor test can settle it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

# Fixed wheel order. Index with the FL/FR/RL/RR constants, never with a literal.
WHEEL_NAMES = ("front_left", "front_right", "rear_left", "rear_right")
#: Two-letter labels for compact output, in the same order. Do not derive these from
#: WHEEL_NAMES by truncation: "front_left"[:2] and "front_right"[:2] are both "fr".
WHEEL_ABBREV = ("FL", "FR", "RL", "RR")
FL, FR, RL, RR = 0, 1, 2, 3

#: Layout that computes as "x" but marks the strafe direction as unverified.
PROVISIONAL_LAYOUT = "unknown"
ROLLER_LAYOUTS = ("x", "o", PROVISIONAL_LAYOUT)

#: Roller handedness that makes the wz terms add to (lx + ly). See the module docstring.
HANDEDNESS = (-1.0, +1.0, +1.0, -1.0)


def _layout_sign(roller_layout: str) -> float:
    """Return the vy-column sign for a layout name. "unknown" computes as "x"."""
    if roller_layout == "x" or roller_layout == PROVISIONAL_LAYOUT:
        return +1.0
    if roller_layout == "o":
        return -1.0
    raise ValueError(
        f"roller_layout must be one of {ROLLER_LAYOUTS}, got {roller_layout!r}")


@dataclass(frozen=True)
class MecanumGeometry:
    """Robot geometry. Distances in metres.

    `gear_ratio` is carried here because it belongs to the robot's build, but
    `inverse` and `forward` never apply it — they work at the wheel. `base.py`
    applies it when converting wheel rad/s to controller rpm.
    """

    wheel_radius: float
    track: float           #: LEFT-RIGHT wheel centre distance  -> ly = track / 2
    wheelbase: float       #: FRONT-REAR wheel centre distance  -> lx = wheelbase / 2
    roller_layout: str = PROVISIONAL_LAYOUT
    gear_ratio: float = 1.0

    def __post_init__(self) -> None:
        problems = []
        for name in ("wheel_radius", "track", "wheelbase", "gear_ratio"):
            value = getattr(self, name)
            if not isinstance(value, (int, float)) or isinstance(value, bool):
                problems.append(f"{name} must be a number, got {value!r}")
            elif not math.isfinite(value):
                problems.append(f"{name} must be finite, got {value!r}")
            elif value <= 0.0:
                problems.append(f"{name} must be positive, got {value!r}")
        if self.roller_layout not in ROLLER_LAYOUTS:
            problems.append(
                f"roller_layout must be one of {ROLLER_LAYOUTS}, got {self.roller_layout!r}")
        if problems:
            raise ValueError("invalid geometry: " + "; ".join(problems))

    @property
    def lxy(self) -> float:
        """`lx + ly` — the rotation lever arm. This is THE wz gain (see module docstring)."""
        return 0.5 * (self.wheelbase + self.track)

    @property
    def layout_is_provisional(self) -> bool:
        """True while `roller_layout` is unverified: the strafe DIRECTION is a guess."""
        return self.roller_layout == PROVISIONAL_LAYOUT

    def with_layout(self, roller_layout: str) -> "MecanumGeometry":
        """Copy with a different roller layout (validated by __post_init__)."""
        return replace(self, roller_layout=roller_layout)

    def mirrored(self) -> "MecanumGeometry":
        """Copy with the strafe direction flipped — the fix when +vy goes the wrong way.

        A provisional layout resolves to "o", since it was computing as "x".
        """
        return self.with_layout("o" if _layout_sign(self.roller_layout) > 0 else "x")


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
    sy = _layout_sign(geom.roller_layout)
    r = geom.wheel_radius
    y = sy * vy
    k = geom.lxy * wz
    return ((vx - y - k) / r,    # front_left
            (vx + y + k) / r,    # front_right
            (vx + y - k) / r,    # rear_left
            (vx - y + k) / r)    # rear_right


def forward(omega: tuple[float, float, float, float] | list[float],
            geom: MecanumGeometry) -> tuple[float, float, float]:
    """Wheel angular velocities -> body twist. Exact least-squares inverse of `inverse`.

    Args:
        omega: four wheel speeds in rad/s, in WHEEL_NAMES order.
        geom: robot geometry.

    Returns:
        (vx, vy, wz). For any `omega` produced by `inverse` this returns the original
        twist exactly. For an inconsistent set (see `slip_residual`) it returns the
        least-squares best fit — the motion the robot actually makes while the wheels
        fight each other.
    """
    fl, fr, rl, rr = omega
    sy = _layout_sign(geom.roller_layout)
    r = geom.wheel_radius
    return (r * (fl + fr + rl + rr) / 4.0,
            r * sy * (-fl + fr + rl - rr) / 4.0,
            r * (-fl + fr - rl + rr) / (4.0 * geom.lxy))


def slip_residual(omega: tuple[float, float, float, float] | list[float]) -> float:
    """0 for any physically consistent wheel-speed set; nonzero = the wheels fight.

    This is the null direction of the 3x4 IK matrix: the one combination of wheel
    speeds that produces no body motion at all, only scrubbing. Independent of
    geometry and of roller layout, which is why it is a useful runtime diagnostic on
    measured wheel speeds — a persistently nonzero residual while driving means the
    commanded and actual wheel speeds have diverged (a slipping wheel, a saturated
    controller, or a wrong wheel map).
    """
    fl, fr, rl, rr = omega
    return fl + fr - rl - rr


def scale_to_limit(values, limit: float) -> tuple[list[float], float]:
    """Scale a wheel-speed vector down uniformly so no element exceeds `limit`.

    Returns:
        (scaled, k) with k in (0, 1]. k == 1.0 means nothing was clamped.

    Why uniform, not per-wheel clipping: the inverse kinematics is LINEAR, so
    multiplying all four wheel speeds by k is exactly identical to having asked for
    k * (vx, vy, wz). The robot therefore follows the SAME path, just slower. Clipping
    wheels individually changes the ratios between them, which changes the DIRECTION —
    a commanded straight line becomes an arc and a commanded strafe picks up an
    unwanted yaw, precisely at the moment the operator has asked for the most speed
    and has the least margin.
    """
    if not isinstance(limit, (int, float)) or isinstance(limit, bool) \
            or not math.isfinite(limit) or limit <= 0.0:
        raise ValueError(f"limit must be a positive finite number, got {limit!r}")
    values = list(values)
    # Check every element, not just the peak: max() compares with `>`, and every
    # comparison against NaN is False, so a NaN in the middle of the vector is simply
    # stepped over and the peak comes back finite. A NaN wheel speed must never reach
    # a motor command.
    if any(not math.isfinite(v) for v in values):
        raise ValueError(f"wheel speeds must be finite, got {values!r}")
    peak = max((abs(v) for v in values), default=0.0)
    if peak <= limit or peak == 0.0:
        return values, 1.0
    k = limit / peak
    return [v * k for v in values], k
