"""The autonomous approach sequence, as a pure state machine.

No ROS, no I/O: it takes the current observations and returns what to command.
That makes every transition testable without a robot, which matters here because
the sequence ends by running a drill under a burning car.

The sequence
------------
The operator drives to the front of the vehicle by hand and flips the switch to
autonomous. From there:

    WAIT_PLATE  hold still until the front camera has a plate to work from
    ALIGN       creep forward while strafing to put the plate on centre
    ENTER       the plate has gone out of view under the car; keep going blind
                for entry_distance, measured on the wheel encoders
    DRILL       stop, run the drill for drill_seconds

The rest runs only with hole_stage on. It needs an upward-facing camera that is
not fitted yet, so by default the sequence finishes at the drill.

    FIND_HOLE   wait for the upward camera to pick out the hole just drilled
    ALIGN_HOLE  shuffle in both axes to bring the hole over the actuator
    RAISE       drive the actuator up into the hole for actuator_seconds
    SPRAY       open the solenoid for spray_seconds; water goes through the hole
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
    FIND_HOLE = "find_hole"
    ALIGN_HOLE = "align_hole"
    RAISE = "raise"
    SPRAY = "spray"
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

    # Hole alignment, off the upward-facing camera. Offsets are normalised
    # [-1, 1] against the frame; the target is where the ACTUATOR sits in that
    # frame, which is not usually the centre.
    hole_timeout: float = 0.5  # s without a hole reading before it counts as lost
    hole_target_x: float = 0.0
    hole_target_y: float = 0.0
    hole_tolerance: float = 0.05  # both axes within this counts as lined up
    # Signed: which way the machine must move to reduce an offset depends on how
    # the camera is mounted, and it is not fitted yet. Verify both before use.
    hole_gain_x: float = -0.3  # m/s of vy per unit of x offset
    hole_gain_y: float = -0.3  # m/s of vx per unit of y offset
    hole_max_speed: float = 0.05  # m/s cap while shuffling under the car

    actuator_seconds: float = 3.0  # how long to drive the actuator up
    spray_seconds: float = 10.0  # how long the solenoid stays open
    # The upward camera is not fitted, so nothing publishes a hole offset. With
    # this off the sequence finishes at the drill instead of stalling in
    # FIND_HOLE until the timeout. Turn it on when the camera and its detector
    # exist.
    hole_stage: bool = False

    max_align_seconds: float = 60.0
    max_entry_seconds: float = 60.0
    max_find_hole_seconds: float = 30.0
    max_hole_align_seconds: float = 60.0

    def __post_init__(self) -> None:
        for name in ("plate_timeout", "align_gain", "approach_speed",
                     "entry_distance", "entry_speed", "drill_seconds",
                     "hole_timeout", "hole_max_speed", "actuator_seconds",
                     "spray_seconds", "max_align_seconds", "max_entry_seconds",
                     "max_find_hole_seconds", "max_hole_align_seconds"):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be positive, got {getattr(self, name)}")
        for name in ("align_tolerance", "hole_tolerance"):
            value = getattr(self, name)
            if value <= 0 or value >= 1:
                raise ValueError(f"{name} must be within (0, 1), got {value}")
        for name in ("hole_target_x", "hole_target_y"):
            value = getattr(self, name)
            if not -1.0 <= value <= 1.0:
                raise ValueError(f"{name} must be within [-1, 1], got {value}")


@dataclass(frozen=True)
class Observation:
    """What the supervisor knows this tick."""

    now: float  # monotonic seconds
    plate_offset_x: float | None  # normalised [-1, 1]; positive = plate right of centre
    plate_age: float | None  # seconds since the last plate reading, None if never
    distance: float  # forward travel from the wheel encoders, metres, monotonic-ish
    # Upward camera: where the drilled hole sits in the frame, normalised [-1, 1].
    hole_offset_x: float | None = None
    hole_offset_y: float | None = None
    hole_age: float | None = None


@dataclass(frozen=True)
class Action:
    """What to command this tick."""

    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    drill: int = 0
    actuator: int = 0  # -1 down, 0 hold, +1 up
    solenoid: int = 0
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
                if cfg.hole_stage:
                    self._enter(Phase.FIND_HOLE, obs.now,
                                "drill finished; looking for the hole")
                else:
                    self._enter(Phase.DONE, obs.now,
                                "drill finished; hole stage disabled")
                return Action(phase=self.phase, message=self._message)
            self._message = f"drilling {elapsed:.1f}/{cfg.drill_seconds:.1f} s"
            return Action(drill=1, phase=self.phase, message=self._message)

        seen = (obs.hole_offset_x, obs.hole_offset_y, obs.hole_age)
        have_hole = all(v is not None for v in seen)
        hole_fresh = have_hole and obs.hole_age <= cfg.hole_timeout

        if self.phase is Phase.FIND_HOLE:
            if elapsed > cfg.max_find_hole_seconds:
                self.abort(
                    f"the hole was not found within {cfg.max_find_hole_seconds:.0f} s"
                )
                return Action(phase=self.phase, message=self._message)
            if hole_fresh:
                self._enter(Phase.ALIGN_HOLE, obs.now, "hole found; lining up the actuator")
            return Action(phase=self.phase, message=self._message)

        if self.phase is Phase.ALIGN_HOLE:
            if elapsed > cfg.max_hole_align_seconds:
                self.abort(
                    f"hole alignment exceeded {cfg.max_hole_align_seconds:.0f} s"
                )
                return Action(phase=self.phase, message=self._message)
            if not hole_fresh:
                # Losing sight of the hole here is not a cue to carry on: the
                # actuator would come up through whatever is above it.
                self.abort("lost sight of the hole while lining up")
                return Action(phase=self.phase, message=self._message)
            ex = obs.hole_offset_x - cfg.hole_target_x
            ey = obs.hole_offset_y - cfg.hole_target_y
            if abs(ex) <= cfg.hole_tolerance and abs(ey) <= cfg.hole_tolerance:
                self._enter(Phase.RAISE, obs.now,
                            f"lined up (dx {ex:+.3f}, dy {ey:+.3f}); raising")
                return Action(actuator=1, phase=self.phase, message=self._message)
            cap = cfg.hole_max_speed
            vy = max(-cap, min(cap, cfg.hole_gain_x * ex))
            vx = max(-cap, min(cap, cfg.hole_gain_y * ey))
            self._message = f"lining up dx {ex:+.3f} dy {ey:+.3f}"
            return Action(vx=vx, vy=vy, phase=self.phase, message=self._message)

        if self.phase is Phase.RAISE:
            if elapsed >= cfg.actuator_seconds:
                self._enter(Phase.SPRAY, obs.now, "actuator up; opening the valve")
                return Action(solenoid=1, phase=self.phase, message=self._message)
            self._message = f"raising {elapsed:.1f}/{cfg.actuator_seconds:.1f} s"
            return Action(actuator=1, phase=self.phase, message=self._message)

        if self.phase is Phase.SPRAY:
            if elapsed >= cfg.spray_seconds:
                self._enter(Phase.DONE, obs.now, "spray finished")
                return Action(phase=self.phase, message=self._message)
            self._message = f"spraying {elapsed:.1f}/{cfg.spray_seconds:.1f} s"
            # The actuator is left at 0, not driven: it has reached the hole, and
            # holding +1 against a hard stop would stall it for the whole spray.
            return Action(solenoid=1, phase=self.phase, message=self._message)

        # DONE and ABORT both hold still.
        return Action(phase=self.phase, message=self._message)
