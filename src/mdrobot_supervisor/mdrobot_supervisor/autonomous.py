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
    DRILL       stop driving. The bit turns for drill_seconds; the lift pushes
                it up into the underbody for the first lift_up_seconds of that,
                then holds while the drill finishes
    LIFT_DOWN   drill off, lift back down for lift_down_seconds. Nothing rises
                again until it is clear
    RAISE       actuator up into the hole for actuator_seconds
    SPRAY       solenoid open for spray_seconds, actuator still held up; water
                through the hole
    RETRACT     valve shut, actuator back down for retract_seconds
    DONE        hold still; the operator takes it from here

FIND_HOLE and ALIGN_HOLE sit between LIFT_DOWN and RAISE when hole_stage is on.
They need the upward-facing camera, which is not fitted, so they are skipped by
default and the actuator goes up where the drill just was.

Every phase that moves the machine moves it mecanum-style. wz is never set:
autonomous strafes and drives straight, and never yaws.

ABORT is entered instead of any of the above when a guard trips, and like DONE
it commands nothing. Both are terminal: the operator has to leave autonomous and
come back, which is exactly the deliberate act that should be needed to re-arm a
drill.

Why the encoder distance is the weak point
------------------------------------------
Once the plate is out of view there is nothing left to correct against, so ENTER
is dead reckoning, and the distance travelled decides where a hole gets drilled.

Measured on this machine: a commanded 1.000 m drove very close to 1 m on the
floor, so the slip these rollers were expected to bring is small on that
surface. That is the number the whole phase rests on, and it is worth
re-checking on the surface the machine will actually work on. Keep
entry_distance short, keep entry_speed low, and treat max_entry_seconds as a
real guard rather than a formality. An IMU would help and is not fitted yet.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class Phase(Enum):
    WAIT_PLATE = "wait_plate"
    ALIGN = "align"
    ENTER = "enter"
    DRILL = "drill"
    LIFT_DOWN = "lift_down"
    FIND_HOLE = "find_hole"
    ALIGN_HOLE = "align_hole"
    RAISE = "raise"
    SPRAY = "spray"
    RETRACT = "retract"
    DONE = "done"
    ABORT = "abort"


TERMINAL = (Phase.DONE, Phase.ABORT)

# Phases that command no wheel motion: the machine is parked with the drill in
# a hole and only the equipment is running. Losing the RC link during one of
# these says nothing about whether it is safe to keep holding.
STATIONARY = (
    Phase.DRILL,
    Phase.LIFT_DOWN,
    Phase.RAISE,
    Phase.SPRAY,
    Phase.RETRACT,
)


@dataclass(frozen=True)
class AutonomousConfig:
    """Tuning for the sequence. Distances in metres, speeds in m/s."""

    plate_timeout: float = 0.5  # s without a plate reading before it counts as lost
    # Losing the plate means "we are under the car" only if the plate had got
    # close first. The detector drops out for seconds at a time on a plate that
    # is plainly in frame, and every one of those dropouts used to read as an
    # arrival and fire the drill wherever the machine happened to be. A plate
    # that vanishes while still narrow has not been reached, it has been lost.
    min_approach_width: float = 0.45
    align_gain: float = 0.4  # strafe m/s per unit of normalised plate offset
    align_tolerance: float = 0.08  # |offset.x| this small counts as centred
    approach_speed: float = 0.08  # m/s forward while aligning
    entry_distance: float = 1.2  # m to travel blind after losing the plate
    entry_speed: float = 0.08  # m/s forward while entering
    # The working sequence, once the machine is under the car. The lift and the
    # drill start together: the drill spins while the lift pushes it up into the
    # underbody. Timed for now — the limit switches that should end the up
    # stroke are not fitted.
    drill_seconds: float = 20.0  # how long the bit turns, all told
    lift_up_seconds: float = 10.0  # of that, how long the lift keeps pushing up
    lift_down_seconds: float = 10.0  # bringing the lift back down afterwards

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

    actuator_seconds: float = 12.0  # actuator up, into the hole — its full stroke
    spray_seconds: float = 30.0  # solenoid open, water through the hole
    retract_seconds: float = 12.0  # actuator back down once the valve is shut
    # The actuator does not hold position unpowered — the moment the command
    # goes to 0 it comes straight back down — so the spray has to keep it up or
    # the water leaves the hole it was just placed in. But driving it
    # continuously against its end stop made it hunt: it sagged and drove and
    # sagged again, two or three times, which is a stalled motor browning out
    # and recovering (or its own end-of-travel protection cycling).
    #
    # So hold it in pulses instead: drive for hold_pulse_on out of every
    # hold_pulse_period. It settles a little between pulses rather than fighting
    # the stop, and draws a fraction of the current.
    # hold_pulse_period 0 holds it continuously, which is what manual driving
    # does and what the actuator turned out to be fine with once the run guard
    # stopped cutting it off. Set a period to pulse instead if it starts hunting
    # against its end stop again.
    hold_actuator_during_spray: bool = True
    hold_pulse_on: float = 0.4
    hold_pulse_period: float = 0.0
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
        if self.hold_pulse_period and self.hold_pulse_on > self.hold_pulse_period:
            raise ValueError(
                f"hold_pulse_on {self.hold_pulse_on} exceeds hold_pulse_period "
                f"{self.hold_pulse_period}; use a period of 0 to hold continuously"
            )
        for name in ("plate_timeout", "min_approach_width", "align_gain",
                     "approach_speed",
                     "entry_distance", "entry_speed", "drill_seconds",
                     "lift_up_seconds", "lift_down_seconds",
                     "hole_timeout", "hole_max_speed",
                     "actuator_seconds", "spray_seconds", "retract_seconds",
                     "max_align_seconds", "max_entry_seconds",
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
    # The plate's width as a fraction of the frame: the only range proxy there
    # is, and what tells an arrival apart from a dropped detection.
    plate_width: float | None = None
    # Upward camera: where the drilled hole sits in the frame, normalised [-1, 1].
    hole_offset_x: float | None = None
    hole_offset_y: float | None = None
    hole_age: float | None = None


@dataclass(frozen=True)
class Action:
    """What to command this tick."""

    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0  # always 0: autonomous drives mecanum-style, it never yaws
    lift: int = 0  # signed speed for the up/down motor, -lift_speed..+lift_speed
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
        # Where the machine was when the plate was last actually seen. The blind
        # entry is measured from there, not from where the timeout happened to
        # expire, so plate_timeout can be as long as detection needs without
        # moving the point the machine stops at.
        self._last_seen_at = 0.0
        self._widest = 0.0
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
                self._last_seen_at = obs.distance
                self._enter(Phase.ALIGN, obs.now, "plate acquired; aligning")
            return Action(phase=self.phase, message=self._message)

        if self.phase is Phase.ALIGN:
            if elapsed > cfg.max_align_seconds:
                self.abort(f"align exceeded {cfg.max_align_seconds:.0f} s")
                return Action(phase=self.phase, message=self._message)
            if not plate_fresh:
                # Losing the plate IS the trigger to go under — but only once it
                # has got close. Check how wide it was when last seen: a plate
                # that was still narrow is a dropped detection, not an arrival.
                if self._widest < cfg.min_approach_width:
                    self._message = (
                        f"plate lost at width {self._widest:.2f} < "
                        f"{cfg.min_approach_width:.2f}; too far to be under the "
                        f"car, waiting for it to come back"
                    )
                    # Keep closing straight ahead. There is no offset to steer
                    # on, so do not strafe on a stale one.
                    return Action(vx=cfg.approach_speed, phase=self.phase,
                                  message=self._message)
                # Measure from where it was last SEEN: the machine has been
                # driving through the whole timeout, and counting from here
                # would add that distance to every entry.
                self._entry_mark = self._last_seen_at or obs.distance
                drifted = obs.distance - self._entry_mark
                self._enter(Phase.ENTER, obs.now,
                            f"plate lost at width {self._widest:.2f}, "
                            f"{drifted:.2f} m ago; "
                            f"entering {cfg.entry_distance:.2f} m from there")
                return Action(vx=cfg.entry_speed, phase=self.phase,
                              message=self._message)
            offset = obs.plate_offset_x
            assert offset is not None  # plate_fresh guarantees it
            self._last_seen_at = obs.distance
            if obs.plate_width is not None:
                self._widest = max(self._widest, obs.plate_width)
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
                # Lift and drill start on the same tick, as they do for the rest
                # of the phase.
                return Action(lift=1, drill=1, phase=self.phase,
                              message=self._message)
            self._message = (
                f"entering blind {travelled:.2f}/{cfg.entry_distance:.2f} m"
            )
            return Action(vx=cfg.entry_speed, phase=self.phase, message=self._message)

        if self.phase is Phase.DRILL:
            # The bit turns for the whole phase; the lift only pushes up for the
            # first lift_up_seconds of it and then holds while the drill
            # finishes. Limit switches should end the stroke instead of a clock,
            # and are not fitted.
            if elapsed >= cfg.drill_seconds:
                self._enter(Phase.LIFT_DOWN, obs.now, "hole cut; lowering the lift")
                return Action(lift=-1, phase=self.phase, message=self._message)
            rising = elapsed < cfg.lift_up_seconds
            self._message = (
                f"drilling {elapsed:.1f}/{cfg.drill_seconds:.1f} s"
                f"{', lift rising' if rising else ', lift held'}"
            )
            return Action(lift=1 if rising else 0, drill=1,
                          phase=self.phase, message=self._message)

        if self.phase is Phase.LIFT_DOWN:
            # The drill is off from here. Nothing goes up again until the lift is
            # all the way down, or the actuator would rise into it.
            if elapsed >= cfg.lift_down_seconds:
                if cfg.hole_stage:
                    self._enter(Phase.FIND_HOLE, obs.now,
                                "lift down; looking for the hole")
                else:
                    self._enter(Phase.RAISE, obs.now,
                                "lift down; raising the actuator")
                    return Action(actuator=1, phase=self.phase, message=self._message)
                return Action(phase=self.phase, message=self._message)
            self._message = f"lift lowering {elapsed:.1f}/{cfg.lift_down_seconds:.1f} s"
            return Action(lift=-1, phase=self.phase, message=self._message)

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
                self._enter(Phase.RETRACT, obs.now, "valve shut; lowering the actuator")
                return Action(actuator=-1, phase=self.phase, message=self._message)
            # Pulsed, not continuous: the actuator falls the instant it is not
            # driven, but holding it against the stop for thirty seconds made it
            # hunt. A short push every couple of seconds keeps it there without
            # sitting stalled.
            pushing = False
            if cfg.hold_actuator_during_spray:
                pushing = (
                    True if cfg.hold_pulse_period <= 0
                    else (elapsed % cfg.hold_pulse_period) < cfg.hold_pulse_on
                )
            self._message = (
                f"spraying {elapsed:.1f}/{cfg.spray_seconds:.1f} s"
                f"{', holding' if pushing else ''}"
            )
            return Action(solenoid=1, actuator=1 if pushing else 0,
                          phase=self.phase, message=self._message)

        if self.phase is Phase.RETRACT:
            # Valve already shut — solenoid is 0 from here, so the water stops
            # before the actuator starts moving out of the hole.
            if elapsed >= cfg.retract_seconds:
                self._enter(Phase.DONE, obs.now, "actuator down; sequence complete")
                return Action(phase=self.phase, message=self._message)
            self._message = f"actuator lowering {elapsed:.1f}/{cfg.retract_seconds:.1f} s"
            return Action(actuator=-1, phase=self.phase, message=self._message)

        # DONE and ABORT both hold still.
        return Action(phase=self.phase, message=self._message)
