"""Verify the whole dispatch path — wheel map, signs, gear ratio, clamp.

Commands a pure `+vx`, then a pure `+vy`, then a pure `+wz`, and after each one
compares what every wheel actually did against what the kinematics asked it to do.
The comparison is a straight one between numbers, with no human judgement in it.

Designed for the wheels off the ground, but equally usable on the floor with
`only=` to take one motion at a time while somebody watches where the robot goes.

What it can and cannot prove, stated plainly:

- **Can prove**: every wheel is wired to the channel the config says, every `sign` is
  right relative to the others, the clamp preserves ratios, and no channel is dead or
  un-armed (commanded nonzero, measured zero).
- **Cannot prove**: which way the robot actually travels. This compares wheels against
  the kinematics that dispatched them, and that comparison is identical under either
  `roller_layout` — so whether `+vy` strafes left or right is invisible to it by
  construction. Only a human watching a floor run settles that.

Measurement is by POSITION DELTA, never by the instantaneous speed register: a 4-pole
hall gives only a handful of edges per second at identification speeds, so single
speed samples swing from 0 to 3x the commanded value while position counts stay
truthful.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

from .base import MecanumBase
from .kinematics import WHEEL_ABBREV, WHEEL_NAMES

#: Fraction of each configured limit to command. High enough to clear the low-speed
#: region where the motor cannot hold the commanded speed, low enough to stay gentle.
DEFAULT_EFFORT = 0.5

#: Seconds to hold each motion. The command is refreshed throughout — a velocity
#: command is not a latch (see base.COMMAND_WATCHDOG_S).
DEFAULT_HOLD_S = 6.0

#: A wheel whose |delta| is under this share of the largest is treated as "stationary"
#: when checking the pattern, rather than having its noise interpreted as a direction.
STATIONARY_SHARE = 0.25


@dataclass(frozen=True)
class MotionCheck:
    """One commanded motion and what the wheels did."""

    name: str
    twist: tuple[float, float, float]
    commanded: tuple[float, float, float, float]   #: wheel rad/s, forward-positive
    measured: tuple[int, int, int, int]            #: position delta, forward-positive
    problems: tuple[str, ...]

    @property
    def ok(self) -> bool:
        return not self.problems


def _direction(value: float, threshold: float) -> int:
    if abs(value) < threshold:
        return 0
    return 1 if value > 0 else -1


def check_motion(base: MecanumBase, name: str, twist: tuple[float, float, float],
                 hold_s: float, *, settle_s: float = 0.5) -> MotionCheck:
    """Command one twist, hold it, and compare each wheel's travel against the plan."""
    before = base.read_wheel_positions()
    result = base.drive(*twist)
    deadline = time.monotonic() + hold_s
    while time.monotonic() < deadline:
        base.drive(*twist)          # refresh: the controller cuts drive on bus silence
        time.sleep(0.1)
    base.stop()
    if settle_s > 0:
        time.sleep(settle_s)        # let the last counts land before reading
    after = base.read_wheel_positions()
    measured = tuple(a - b for a, b in zip(after, before))

    commanded = result.wheel_rad_s
    problems = []

    # Compare DIRECTIONS, and only for wheels the command actually asked to move.
    # Magnitudes cannot be compared directly without knowing counts-per-rev, but the
    # pattern of signs is exactly what a wheel-map or sign error corrupts.
    biggest = max((abs(v) for v in measured), default=0)
    threshold = max(1.0, STATIONARY_SHARE * biggest)
    for name_, want, got in zip(WHEEL_NAMES, commanded, measured):
        want_dir = _direction(want, 1e-9)
        got_dir = _direction(got, threshold)
        if want_dir == 0:
            continue
        if got_dir == 0:
            problems.append(f"{name_}: commanded to turn but did not move "
                            f"({got:+d} counts) — dead or un-armed channel?")
        elif got_dir != want_dir:
            problems.append(f"{name_}: turned the wrong way "
                            f"(wanted {'+' if want_dir > 0 else '-'}, got {got:+d} counts)")

    # Wheels the command asked to turn at equal speed must travel about equally, or the
    # robot is not doing the motion it thinks it is.
    moving = [(n, w, m) for n, w, m in zip(WHEEL_NAMES, commanded, measured)
              if abs(w) > 1e-9]
    if moving and biggest > 0:
        scale = biggest / max(abs(w) for _, w, _ in moving)
        for name_, want, got in moving:
            expected = abs(want) * scale
            if expected > 0 and abs(abs(got) - expected) > max(3.0, 0.35 * expected):
                problems.append(f"{name_}: travelled {abs(got)} counts, expected about "
                                f"{expected:.0f} for its share of the motion")
    return MotionCheck(name, twist, commanded, measured, tuple(problems))


#: Motion names accepted by `--motion`, in the order they should be tried on the floor.
MOTION_NAMES = ("forward", "strafe", "rotate")


def motions(base: MecanumBase, effort: float,
            only: str | None = None) -> list[tuple[str, tuple[float, float, float]]]:
    """The pure motions to test, scaled to a fraction of the configured limits."""
    limits = base.config.limits
    all_motions = [
        ("forward  (+vx)", (limits.max_linear_x * effort, 0.0, 0.0)),
        ("strafe   (+vy)", (0.0, limits.max_linear_y * effort, 0.0)),
        ("rotate   (+wz)", (0.0, 0.0, limits.max_angular_z * effort)),
    ]
    if only is None:
        return all_motions
    if only not in MOTION_NAMES:
        raise ValueError(f"motion must be one of {MOTION_NAMES}, got {only!r}")
    return [all_motions[MOTION_NAMES.index(only)]]


def run(base: MecanumBase, *, effort: float = DEFAULT_EFFORT,
        hold_s: float = DEFAULT_HOLD_S, settle_s: float = 0.5, pause_s: float = 1.0,
        only: str | None = None, printer=print) -> list[MotionCheck]:
    """Run the checks, printing progress. Returns the results."""
    checks = []
    for name, twist in motions(base, effort, only):
        travel = max(abs(twist[0]), abs(twist[1])) * hold_s
        printer(f"\n  {name}  vx={twist[0]:+.3f} vy={twist[1]:+.3f} wz={twist[2]:+.3f}"
                f"  for {hold_s:.0f}s"
                + (f"  -> about {travel:.2f} m of travel" if travel else ""))
        check = check_motion(base, name, twist, hold_s, settle_s=settle_s)
        checks.append(check)
        printer("    commanded: " + "   ".join(
            f"{n} {'+' if w > 1e-9 else '-' if w < -1e-9 else '0'}"
            for n, w in zip(WHEEL_ABBREV, check.commanded)))
        printer("    measured : " + "   ".join(
            f"{n}{m:+5d}" for n, m in zip(WHEEL_ABBREV, check.measured)))
        for problem in check.problems:
            printer(f"    FAIL {problem}")
        if check.ok:
            printer("    ok")
        if pause_s > 0:
            time.sleep(pause_s)
    return checks


def fault_test(base: MecanumBase, *, effort: float = DEFAULT_EFFORT,
               window_s: float = 30.0, printer=print) -> int:
    """Drive all four wheels and wait for the operator to pull a controller off the bus.

    The property under test: when one controller stops answering, the OTHER one must
    stop too. Two mecanum wheels driving while two are dead does not merely limp — it
    slews the robot in a direction nobody asked for.

    The repository lists this exact scenario as unit-tested only for twin mode
    (root README, "Tested drivers & firmware"), so a real result here is worth
    recording there.
    """
    limits = base.config.limits
    twist = (limits.max_linear_x * effort, 0.0, 0.0)
    printer(f"  driving forward at {twist[0]:.3f} m/s. PULL THE RS485 LINE to one "
            f"controller at any point in the next {window_s:.0f}s.")

    deadline = time.monotonic() + window_s
    ticks = 0
    try:
        while time.monotonic() < deadline:
            base.drive(*twist)
            ticks += 1
            time.sleep(0.1)
    except Exception as exc:
        elapsed = window_s - (deadline - time.monotonic())
        printer(f"\n  bus fault detected after {elapsed:.1f}s and {ticks} ticks:")
        printer(f"    {type(exc).__name__}: {exc}")
        printer(f"    stop confirmed on every reachable controller: "
                f"{'no' if not base.last_stop_ok else 'yes'}")
        printer("\n  Now check by hand: are ALL FOUR wheels stopped, including the two")
        printer("  on the controller that is still connected? That is the property.")
        return 0

    base.stop()
    printer(f"\n  the {window_s:.0f}s window passed with no fault — nothing was unplugged,")
    printer("  so this proves nothing. Re-run and pull the line while it is driving.")
    return 1


def summarise(checks: list[MotionCheck], base: MecanumBase, printer=print) -> int:
    """Print the verdict and return a process exit code."""
    printer("\n" + "=" * 70)
    for check in checks:
        printer(f"  {check.name}   {'ok' if check.ok else 'FAILED'}")
    printer("=" * 70)

    failed = [c for c in checks if not c.ok]
    if failed:
        printer("\nThe dispatch path is wrong somewhere. This check compares wheels")
        printer("against the kinematics that dispatched them, so it is a wiring or config")
        printer("error — never a roller-layout question:")
        printer("  - re-check which wheel each (slave_id, channel) drives")
        printer("  - re-check the per-wheel sign")
        printer("  - a wheel that never moves is a dead or un-armed channel")
        return 1

    printer("\nWheel map, signs, gear ratio and clamp all check out.")
    if base.config.geometry.layout_is_provisional:
        printer("\nSTILL UNVERIFIED: roller_layout. This check cannot see it — drive a")
        printer("pure strafe on the floor and watch. Goes the way you asked: keep the")
        printer("setting. Goes the other way: flip roller_layout between x and o.")
    return 0
