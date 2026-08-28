"""The autonomous approach sequence, as a pure state machine.

No ROS, no I/O: it takes the current observations and returns what to command.
That makes every transition testable without a robot, which matters here because
the sequence ends by running a drill under a burning car.

The sequence
------------
The operator drives to the front of the vehicle by hand and flips the switch to
autonomous. From there:

    WAIT_PLATE  hold still until the camera has a plate to work from
    ALIGN       creep forward while strafing to put the plate on centre
    ENTER       the plate has gone out of view under the car; keep going blind
                for entry_distance, measured on the wheel encoders
    DRILL       stop, run the drill for drill_seconds
    DONE        hold still; the operator takes it from here

ABORT is entered instead of any of the above when a guard trips, and like DONE
it commands nothing. Both are terminal: the operator has to leave autonomous and
come back, which is exactly the deliberate act that should be needed to re-arm a
drill.

Why the encoder distance is the weak point
------------------------------------------
Once the plate is out of view there is nothing left to correct against, so ENTER
is dead reckoning. Mecanum wheels slip more than most — the rollers are meant to
— so the distance travelled is an estimate, not a measurement, and it decides
where a hole gets drilled. Keep entry_distance short, keep entry_speed low, and
treat max_entry_seconds as a real guard rather than a formality. An IMU would
help and is not fitted yet.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class Phase(Enum):
    WAIT_PLATE = "wait_plate"
    ALIGN = "align"
    ENTER = "enter"
    DRILL = "drill"
    DONE = "done"
    ABORT = "abort"


TERMINAL = (Phase.DONE, Phase.ABORT)


@dataclass(frozen=True)
class AutonomousConfig:
    """Tuning for the sequence. Distances in metres, speeds in m/s."""

    plate_timeout: float = 0.5  # s without a plate reading before it counts as lost
    align_gain: float = 0.4  # strafe m/s per unit of normalised plate offset
    align_tolerance: float = 0.08  # |offset.x| this small counts as centred
    approach_speed: float = 0.08  # m/s forward while aligning
    entry_distance: float = 1.2  # m to travel blind after losing the plate
    entry_speed: float = 0.08  # m/s forward while entering
    drill_seconds: float = 5.0  # how long the drill runs once in position
    max_align_seconds: float = 60.0
    max_entry_seconds: float = 60.0

    def __post_init__(self) -> None:
        for name in ("plate_timeout", "align_gain", "approach_speed",
                     "entry_distance", "entry_speed", "drill_seconds",
                     "max_align_seconds", "max_entry_seconds"):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be positive, got {getattr(self, name)}")
        if self.align_tolerance <= 0 or self.align_tolerance >= 1:
            raise ValueError(
                f"align_tolerance must be within (0, 1), got {self.align_tolerance}"
            )


@dataclass(frozen=True)
class Observation:
    """What the supervisor knows this tick."""

    now: float  # monotonic seconds
    plate_offset_x: float | None  # normalised [-1, 1]; positive = plate right of centre
    plate_age: float | None  # seconds since the last plate reading, None if never
    distance: float  # forward travel from the wheel encoders, metres, monotonic-ish


@dataclass(frozen=True)
class Action:
    """What to command this tick."""

    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    drill: int = 0
    phase: Phase = Phase.WAIT_PLATE
    message: str = ""


class AutonomousSequence:
    """Runs the approach. One instance, reset whenever autonomous is re-entered."""

    def __init__(self, config: AutonomousConfig) -> None:
        self.config = config
        self.reset()

    def reset(self) -> None:
        self.phase = Phase.WAIT_PLATE
        self._phase_started = 0.0
        self._entry_mark = 0.0
        self._message = "waiting for a plate"

    def abort(self, why: str) -> None:
        """Stop for good. Only leaving and re-entering autonomous clears this."""
        self.phase = Phase.ABORT
        self._message = why

    def _enter(self, phase: Phase, now: float, message: str) -> None:
        self.phase = phase
        self._phase_started = now
        self._message = message

    def step(self, obs: Observation) -> Action:
        cfg = self.config
        if self._phase_started == 0.0:
            self._phase_started = obs.now

        have_plate = obs.plate_age is not None and obs.plate_offset_x is not None
        plate_fresh = have_plate and obs.plate_age <= cfg.plate_timeout
        elapsed = obs.now - self._phase_started

        if self.phase is Phase.WAIT_PLATE:
            if plate_fresh:
                self._enter(Phase.ALIGN, obs.now, "plate acquired; aligning")
            return Action(phase=self.phase, message=self._message)

        if self.phase is Phase.ALIGN:
            if elapsed > cfg.max_align_seconds:
                self.abort(f"align exceeded {cfg.max_align_seconds:.0f} s")
                return Action(phase=self.phase, message=self._message)
            if not plate_fresh:
                # Losing the plate IS the trigger to go under: it drops out of
                # view exactly as the machine reaches the car.
                self._entry_mark = obs.distance
                self._enter(Phase.ENTER, obs.now,
                            f"plate lost; entering {cfg.entry_distance:.2f} m")
                return Action(vx=cfg.entry_speed, phase=self.phase,
                              message=self._message)
            offset = obs.plate_offset_x
            assert offset is not None  # plate_fresh guarantees it
            # +y is LEFT and a positive offset means the plate sits to the RIGHT,
            # so the machine has to strafe right: negate.
            vy = 0.0 if abs(offset) <= cfg.align_tolerance else -cfg.align_gain * offset
            centred = "centred" if abs(offset) <= cfg.align_tolerance else "aligning"
            self._message = f"{centred}, offset {offset:+.3f}"
            return Action(vx=cfg.approach_speed, vy=vy, phase=self.phase,
                          message=self._message)

        if self.phase is Phase.ENTER:
            travelled = obs.distance - self._entry_mark
            if elapsed > cfg.max_entry_seconds:
                self.abort(
                    f"entry exceeded {cfg.max_entry_seconds:.0f} s after "
                    f"{travelled:.2f} m"
                )
                return Action(phase=self.phase, message=self._message)
            if travelled >= cfg.entry_distance:
                self._enter(Phase.DRILL, obs.now,
                            f"in position after {travelled:.2f} m; drilling")
                return Action(drill=1, phase=self.phase, message=self._message)
            self._message = (
                f"entering blind {travelled:.2f}/{cfg.entry_distance:.2f} m"
            )
            return Action(vx=cfg.entry_speed, phase=self.phase, message=self._message)

        if self.phase is Phase.DRILL:
            if elapsed >= cfg.drill_seconds:
                self._enter(Phase.DONE, obs.now, "drill finished")
                return Action(phase=self.phase, message=self._message)
            self._message = f"drilling {elapsed:.1f}/{cfg.drill_seconds:.1f} s"
            return Action(drill=1, phase=self.phase, message=self._message)

        # DONE and ABORT both hold still.
        return Action(phase=self.phase, message=self._message)
