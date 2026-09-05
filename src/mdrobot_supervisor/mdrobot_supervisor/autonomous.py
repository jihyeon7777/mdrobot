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

Every phase that moves the machine moves it mecanum-style: it strafes and
drives straight rather than turning to face things. wz is therefore never asked
for as a *heading command* — but it is used to CANCEL yaw the machine did not
ask for, which is what yaw_hold does. See "Why the machine turns anyway".

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
real guard rather than a formality.

An IMU does NOT fix this. Integrating its accelerometer twice over a 15 s entry
gives about a metre of error even calibrated — worse than the encoders. The
distance stays on the wheels.

Why the machine turns anyway
----------------------------
The inverse kinematics assumes the wheels hold. Mecanum rollers on a smooth
floor do not, and not equally, so a commanded pure translation comes out as a
translation plus a rotation nobody asked for. Until the IMU was fitted nothing
measured that, so nothing could correct it or even report it afterwards.

What that costs is not the hole alignment — the hole search is a visual servo
closed in the body frame, and a rigidly mounted camera and actuator keep their
relationship whatever the machine's heading. What it costs is everything
geometric: the mast sweeping sideways under a car, the hole drifting out of the
upward camera's view, and above all the DRILL phase, where the wheels are
commanded to zero while a bit cuts into steel and the reaction torque acts on a
machine standing on rollers. A machine that turns with the bit in the hole
breaks the bit.

So yaw_hold corrects where correcting is safe (the phases that are already
driving: ALIGN, ENTER, FIND_HOLE, ALIGN_HOLE) and only WATCHES where it is not
(the phases with the bit or the actuator in the hole), aborting instead.

ALIGN is the one that needs it most and was the last to get it. It strafes at
up to align_gain m/s, eight times the hole search, and sideways is the
direction mecanum rollers give up in first.

Measured on this machine, 2026-09-03, before any of this was built:

* driven for 90 s and returned to marks on the floor, the reported heading came
  back 1.75 deg off — so the estimate drifts at roughly 0.02 deg/s;
* a 60 s shuffle at hole-search speed accumulated several degrees of real yaw;
* 30 s through a real drill cycle moved the heading 0.42 deg, against the
  0.6 deg that drift alone accounts for over that window. The reaction torque
  turning the machine — the thing this was most wanted for — did not happen on
  that floor. So the stationary limit is tight (4 deg) because it can be, and
  the driving one is loose (15 deg) because nobody has measured it yet.

The signal is bigger than the drift, which is what makes the correction worth
more than the error it brings with it. But only just, and only over short
windows — hence a deadband above the drift, a reference re-taken at the start of
the hole search rather than carried through the whole sequence, and a hard cap
on how much authority any of this gets.
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

# Phases that start a FRESH heading reference instead of inheriting one. The
# estimate drifts at about 0.02 deg/s, so every window it is trusted over has to
# be kept short.
#
# Two phases deliberately inherit rather than re-zero, because each is the
# second half of something that started earlier:
#
#   ENTER inherits ALIGN's. The approach is one continuous run on one heading
#   from the moment the plate is acquired to the moment the drill goes in, and
#   re-zeroing at the plate-loss would adopt whatever heading the machine had
#   drifted to during the timeout — up to plate_timeout of driving with no
#   offset to steer on. Together they are well under a minute in practice, so
#   the drift stays inside the deadband.
#
#   ALIGN_HOLE inherits FIND_HOLE's, because the two are one continuous search
#   and re-zeroing halfway would hide the drift between them.
REFERENCE_PHASES = (Phase.ALIGN, Phase.FIND_HOLE) + STATIONARY


def wrap_deg(angle: float) -> float:
    """Fold a heading difference into [-180, 180).

    Half-open at the top, so a given attitude only ever produces one of +180 or
    -180 and a threshold test cannot see it as inside the limit one tick and
    outside it the next.
    """
    return (angle + 180.0) % 360.0 - 180.0


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
    # A reading this much narrower than the widest one seen is not the plate.
    #
    # ~/plate_offset is published for any plate-SHAPED band, with no OCR to
    # confirm it is a plate — finding the band is far more reliable than
    # reading it, and a control loop does not need the number. The cost is that
    # when the real plate is rejected, something else wins and the offset
    # points at it. Measured by the detector across thirteen frames:
    #
    #     the plate         0.35 to 0.63
    #     other candidates  0.10  0.14  0.15  0.16
    #
    # and the detector's own width_ratio_range tops out at 0.75. So closing on
    # the plate eventually pushes it PAST that ceiling, the real plate drops
    # out, and one of the 0.1-something candidates takes over — from somewhere
    # else in the frame, which reads as the plate having jumped and steers the
    # machine at it. The apparent width of a plate being approached only grows;
    # a reading that collapses is a different object.
    #
    # Rejected readings count as no plate at all, not as a plate somewhere new.
    # If the real plate has genuinely gone out of view because the machine is
    # under the car, that is exactly right: the sequence goes to ENTER on the
    # width it had reached, rather than steering at whatever replaced it.
    plate_shrink_ratio: float = 0.5
    align_gain: float = 0.4  # strafe m/s per unit of normalised plate offset
    # Cap on the strafe, the way hole_max_speed caps the hole search. Without
    # one a plate at the edge of frame asks for align_gain m/s sideways, which
    # is where mecanum rollers give up first and the phase that most needs the
    # heading held is the one sliding hardest. Capping rather than lowering the
    # gain keeps a small offset closing briskly and only slows the big ones.
    align_max_speed: float = 0.06
    # How old a plate reading may be and still be STEERED on. Separate from
    # plate_timeout, which answers a different question on a different
    # timescale:
    #
    #   "has the plate gone?"        plate_timeout, deliberately generous, so
    #                                a dropout is not mistaken for an arrival
    #   "may I strafe on this?"      this, about one detector period
    #
    # Measured 2026-09-05: detection ran at 1.9 Hz and then stopped for good
    # two seconds before the timeout expired. ALIGN went on strafing at
    # align_max_speed on that last reading for the whole two seconds -- 10 cm
    # sideways, long after the offset it was chasing had been closed. The
    # machine ended up beside the plate rather than on it, and the strafe put
    # the yaw into the handover as well.
    #
    # A lateral offset two seconds old describes where the plate was, not
    # where it is. Forward motion is safe to continue on -- it closes on the
    # car either way -- so only the strafe is gated.
    align_offset_max_age: float = 0.6
    align_tolerance: float = 0.08  # |offset.x| this small counts as centred
    # The detector, not the wheels, sets how fast this can usefully go:
    # measured at 0.43 Hz with gaps up to 5.5 s, so at 0.08 m/s the machine
    # covers 44 cm between sightings, open-loop. At 0.05 that is 28 cm.
    approach_speed: float = 0.05  # m/s forward while aligning
    entry_distance: float = 1.2  # m to travel blind after losing the plate
    entry_speed: float = 0.05  # m/s forward while entering
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

    # Yaw hold, off the IMU (mdrobot_imu). OFF by default: it needs the sensor
    # fitted AND its signs verified by turning the machine, and with it off
    # every phase behaves exactly as it did before.
    yaw_hold: bool = False
    yaw_timeout: float = 0.5  # s without a heading before it counts as lost
    # Do nothing inside this. The measured 90 s closure error is 1.75 deg, so a
    # deadband much below 2 spends its time chasing the sensor's own drift —
    # rotating the machine to match an error that is not there.
    yaw_deadband_deg: float = 0.5
    yaw_gain: float = 0.01  # rad/s of wz per degree of heading error
    # About a fifth of what a full stick deflection asks for (max_angular_z is
    # 0.37 rad/s). This runs under a car; it corrects, it does not manoeuvre.
    yaw_max_wz: float = 0.08
    # Past this, the machine and the estimate disagree by more than slip
    # explains — a wheel is jammed, the sensor has come loose, or the estimate
    # has run away. Correcting harder is the wrong answer. Stop.
    #
    # This is the DRIVING figure and it is still a placeholder: how far the
    # heading strays while the correction is actually working has not been
    # measured, because it has not been run.
    yaw_abort_deg: float = 15.0
    # The STATIONARY figure, for the phases where the wheels are commanded to
    # zero and the bit or the actuator is in the hole. Much tighter, and it can
    # be, because it was measured: 30 s through a real drill cycle moved the
    # heading 0.42 deg — which is what 0.02 deg/s of drift alone would give, so
    # the reaction torque's contribution is not distinguishable from zero.
    # Anything approaching this figure is therefore a genuine fault, and this
    # is the phase where carrying on breaks the bit.
    yaw_stationary_abort_deg: float = 4.0
    # How long a stationary phase is given to actually become stationary before
    # that tight limit is enforced.
    #
    # A stationary phase takes its reference the instant it BEGINS, and at that
    # instant the machine is still rolling at entry_speed — the tick that
    # enters DRILL is the first one to command zero, and the stop happens
    # afterwards. So the yaw of coasting to a halt was being measured against a
    # 4 deg limit meant for a bit already buried in steel, and a real run
    # aborted on -4.1 deg before the drill had done anything.
    #
    # The 0.42 deg that justified 4 deg was measured on a machine that was
    # ALREADY stopped, and with the motor bus misaddressed so the wheels were
    # never commanded at all. It never contained a deceleration.
    #
    # During this window the reference tracks the machine instead, so what the
    # guard finally holds is the heading it settled at.
    yaw_settle_seconds: float = 1.5

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
        if self.yaw_stationary_abort_deg > self.yaw_abort_deg:
            raise ValueError(
                f"yaw_stationary_abort_deg {self.yaw_stationary_abort_deg} is "
                f"looser than yaw_abort_deg {self.yaw_abort_deg}; the phases "
                f"with the bit in the hole are the ones that need the TIGHTER "
                f"limit"
            )
        if self.yaw_abort_deg <= self.yaw_deadband_deg:
            raise ValueError(
                f"yaw_abort_deg {self.yaw_abort_deg} must exceed "
                f"yaw_deadband_deg {self.yaw_deadband_deg}, or the sequence "
                f"aborts on headings it was told to ignore"
            )
        if not 0.0 < self.plate_shrink_ratio < 1.0:
            raise ValueError(
                f"plate_shrink_ratio must be within (0, 1), got "
                f"{self.plate_shrink_ratio}; 1 or more would reject the plate "
                f"itself the moment it stopped growing"
            )
        for name in ("plate_timeout", "min_approach_width", "align_gain",
                     "approach_speed",
                     "align_max_speed", "align_offset_max_age",
                     "entry_distance", "entry_speed", "drill_seconds",
                     "lift_up_seconds", "lift_down_seconds",
                     "hole_timeout", "hole_max_speed",
                     "actuator_seconds", "spray_seconds", "retract_seconds",
                     "yaw_timeout", "yaw_deadband_deg", "yaw_gain",
                     "yaw_max_wz", "yaw_abort_deg", "yaw_stationary_abort_deg",
                     "yaw_settle_seconds",
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
    # Lift limit switches, already resolved for polarity and for whether gating
    # is enabled at all. False means "not there or not trusted", which leaves
    # the clock in charge exactly as before.
    at_top: bool = False
    at_bottom: bool = False
    # Upward camera: where the drilled hole sits in the frame, normalised [-1, 1].
    hole_offset_x: float | None = None
    hole_offset_y: float | None = None
    hole_age: float | None = None
    # Heading from the IMU, degrees, and how old the reading is. Absolute value
    # is irrelevant — only the change since the reference was taken is used, so
    # it does not matter what the sensor calls north. Ignored unless yaw_hold.
    yaw: float | None = None
    yaw_age: float | None = None


@dataclass(frozen=True)
class Action:
    """What to command this tick."""

    vx: float = 0.0
    vy: float = 0.0
    # Never a heading command — autonomous strafes rather than turning to face
    # things. Non-zero only when yaw_hold is cancelling rotation the machine
    # did not ask for.
    wz: float = 0.0
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
        self._plate_shrank = False
        self._top_seen = False
        self._lift_stopped_by = ""
        # The heading being held, in the sensor's own degrees. Taken afresh at
        # the start of ENTER and again at the start of FIND_HOLE: the estimate
        # drifts at about 0.02 deg/s, so carrying one reference across the whole
        # sequence would spend the deadband on drift before the search began.
        self._yaw_ref: float | None = None
        self._message = "waiting for a plate"

    def abort(self, why: str) -> None:
        """Stop for good. Only leaving and re-entering autonomous clears this."""
        self.phase = Phase.ABORT
        self._message = why

    def _enter(self, phase: Phase, obs: "Observation", message: str) -> None:
        """Move to a phase, and give it a fresh heading reference if it takes one.

        The marking lives here rather than at each transition because there are
        eleven of them and missing one would leave a phase silently watching a
        reference that is minutes old.
        """
        self.phase = phase
        self._phase_started = obs.now
        self._message = message
        if phase in REFERENCE_PHASES:
            self._mark_yaw(obs)

    def _yaw_fresh(self, obs: Observation) -> bool:
        return (
            self.config.yaw_hold
            and obs.yaw is not None
            and obs.yaw_age is not None
            and obs.yaw_age <= self.config.yaw_timeout
        )

    def _mark_yaw(self, obs: Observation) -> None:
        """Take the current heading as the one to hold from here on."""
        if self._yaw_fresh(obs):
            self._yaw_ref = obs.yaw

    def _yaw_error(self, obs: Observation) -> float | None:
        """Degrees the machine has turned since the reference, or None."""
        if self._yaw_ref is None or not self._yaw_fresh(obs):
            return None
        assert obs.yaw is not None  # _yaw_fresh guarantees it
        return wrap_deg(obs.yaw - self._yaw_ref)

    def _yaw_guard(self, obs: Observation, elapsed: float) -> str | None:
        """Reasons to stop rather than steer. None when all is well.

        Aborting on a heading that has simply gone missing looks harsh, but
        yaw_hold is opt-in: switching it on says the guard is wanted, and a
        guard that quietly stops guarding is worse than one that never existed.
        """
        cfg = self.config
        if not cfg.yaw_hold:
            return None
        if obs.yaw is None or obs.yaw_age is None:
            return "yaw_hold is on but nothing is publishing a heading"
        if obs.yaw_age > cfg.yaw_timeout:
            return (
                f"heading is {obs.yaw_age:.1f} s stale, past "
                f"{cfg.yaw_timeout:.1f} s"
            )
        error = self._yaw_error(obs)
        if error is None:
            return None
        # The wheels are commanded to zero in the stationary phases and the bit
        # or the actuator is in the hole, so the machine has no business turning
        # at all — measured, it does not. A much tighter limit therefore costs
        # nothing and catches the failure that is worth catching.
        stationary = self.phase in STATIONARY
        if stationary and elapsed < cfg.yaw_settle_seconds:
            # Still coming to a halt. Track the machine rather than judging it:
            # what the guard ends up holding is the heading it settles at, not
            # the one it was carrying while still rolling.
            self._mark_yaw(obs)
            return None
        limit = cfg.yaw_stationary_abort_deg if stationary else cfg.yaw_abort_deg
        if abs(error) > limit:
            why = (
                "with the wheels stopped and the hole occupied"
                if stationary else "more than slip explains"
            )
            return (
                f"turned {error:+.1f} deg off the held heading, past "
                f"{limit:.0f} deg — {why}"
            )
        return None

    def _hold_yaw(self, obs: Observation) -> tuple[float, str]:
        """The wz that steers back onto the held heading, and what to say.

        Correction only. A positive error means the machine has turned
        anticlockwise past its reference, so it needs a clockwise wz to come
        back: hence the negated gain.
        """
        cfg = self.config
        if not cfg.yaw_hold:
            return 0.0, ""
        error = self._yaw_error(obs)
        if error is None:
            # Not yet referenced. The guard above has already established that
            # a heading is arriving, so this is only the window before the
            # first _mark_yaw.
            return 0.0, ""
        if abs(error) <= cfg.yaw_deadband_deg:
            return 0.0, f", yaw {error:+.1f} deg"
        wz = -cfg.yaw_gain * error
        wz = max(-cfg.yaw_max_wz, min(cfg.yaw_max_wz, wz))
        return wz, f", correcting yaw {error:+.1f} deg"

    def step(self, obs: Observation) -> Action:
        cfg = self.config
        if self._phase_started == 0.0:
            self._phase_started = obs.now

        have_plate = obs.plate_age is not None and obs.plate_offset_x is not None
        plate_fresh = have_plate and obs.plate_age <= cfg.plate_timeout
        # A candidate far narrower than the widest seen is a different object,
        # not the plate having moved. See plate_shrink_ratio.
        self._plate_shrank = False
        if (plate_fresh and obs.plate_width is not None and self._widest > 0.0
                and obs.plate_width < cfg.plate_shrink_ratio * self._widest):
            plate_fresh = False
            self._plate_shrank = True
        elapsed = obs.now - self._phase_started

        # Before any phase acts. A heading that has run away or gone missing is
        # a reason to stop wherever the machine is, including with the bit in
        # the hole — carrying on is what breaks it.
        if self.phase not in TERMINAL:
            why = self._yaw_guard(obs, elapsed)
            if why is not None:
                self.abort(why)
                return Action(phase=self.phase, message=self._message)

        if self.phase is Phase.WAIT_PLATE:
            if plate_fresh:
                self._last_seen_at = obs.distance
                self._enter(Phase.ALIGN, obs, "plate acquired; aligning")
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
                        + (" (a narrower candidate is being ignored)"
                           if self._plate_shrank else "")
                    )
                    # Keep closing straight ahead. There is no offset to steer
                    # on, so do not strafe on a stale one — but DO hold the
                    # heading, which is the one thing still measured while the
                    # detector is blind.
                    wz, _ = self._hold_yaw(obs)
                    return Action(vx=cfg.approach_speed, wz=wz, phase=self.phase,
                                  message=self._message)
                # Measure from where it was last SEEN: the machine has been
                # driving through the whole timeout, and counting from here
                # would add that distance to every entry.
                self._entry_mark = self._last_seen_at or obs.distance
                drifted = obs.distance - self._entry_mark
                # _enter takes the reference: the last moment the machine was
                # aligned to something real, and the heading the hole will be
                # drilled at.
                self._enter(Phase.ENTER, obs,
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
            vy = max(-cfg.align_max_speed, min(cfg.align_max_speed, vy))
            # Steer on it only while it is recent. Between sightings the
            # machine is still moving, so a reading a detector-period old
            # describes where the plate WAS. Keep closing forwards regardless:
            # that is true whatever the lateral offset does.
            stale = obs.plate_age is not None and obs.plate_age > cfg.align_offset_max_age
            if stale:
                vy = 0.0
            # This is the phase that needs the heading held most. It strafes at
            # up to align_gain m/s — eight times the hole search — and sideways
            # is where mecanum rollers give up first. A machine that yaws while
            # it slides sees the plate move because the CAMERA turned, not
            # because the body did, and it enters the car crooked.
            wz, note = self._hold_yaw(obs)
            centred = "centred" if abs(offset) <= cfg.align_tolerance else "aligning"
            age = "" if not stale else f", offset {obs.plate_age:.1f}s old — not strafing"
            self._message = f"{centred}, offset {offset:+.3f}{age}{note}"
            return Action(vx=cfg.approach_speed, vy=vy, wz=wz, phase=self.phase,
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
                self._enter(Phase.DRILL, obs,
                            f"in position after {travelled:.2f} m; drilling")
                # Lift and drill start on the same tick, as they do for the rest
                # of the phase.
                return Action(lift=1, drill=1, phase=self.phase,
                              message=self._message)
            wz, note = self._hold_yaw(obs)
            self._message = (
                f"entering blind {travelled:.2f}/{cfg.entry_distance:.2f} m{note}"
            )
            return Action(vx=cfg.entry_speed, wz=wz, phase=self.phase,
                          message=self._message)

        if self.phase is Phase.DRILL:
            # The bit turns for the whole phase; the lift pushes up into the
            # underbody until the upper limit switch closes, and then holds
            # while the drill finishes. lift_up_seconds is only the backstop for
            # a switch that never closes — a broken wire must not mean pushing
            # until the phase ends.
            if obs.at_top and not self._top_seen:
                self._top_seen = True
                self._lift_stopped_by = f"upper limit at {elapsed:.1f} s"
            elif not self._top_seen and elapsed >= cfg.lift_up_seconds:
                # Fault timeout, not the stroke length. A switch that never
                # closes leaves a shallower hole, which is worth carrying on
                # with — unlike the lower one, where an unknown position means
                # raising the actuator into the lift.
                self._top_seen = True
                self._lift_stopped_by = (
                    f"NO upper limit in {cfg.lift_up_seconds:.0f} s"
                )
            rising = not self._top_seen
            # The bit turns for drill_seconds and no longer. The lift still runs
            # to its switch, so on the rare stroke that outlasts the drill the
            # phase waits for it with the bit already off.
            cutting = elapsed < cfg.drill_seconds
            if not cutting and not rising:
                self._enter(Phase.LIFT_DOWN, obs,
                            f"hole cut, lift stopped by {self._lift_stopped_by}; "
                            f"lowering the lift")
                return Action(lift=-1, phase=self.phase, message=self._message)
            self._message = (
                (f"drilling {elapsed:.1f}/{cfg.drill_seconds:.1f} s"
                 if cutting else "drill done")
                + (", lift rising to the upper limit" if rising
                   else f", lift held — stopped by {self._lift_stopped_by}")
            )
            return Action(lift=1 if rising else 0, drill=1 if cutting else 0,
                          phase=self.phase, message=self._message)

        if self.phase is Phase.LIFT_DOWN:
            # The drill is off from here. Nothing goes up again until the lift is
            # all the way down, or the actuator would rise into it.
            if not obs.at_bottom and elapsed >= cfg.lift_down_seconds:
                # The switch, not the clock, ends this stroke. Reaching the
                # clock means the switch never closed on a stroke that takes
                # about a fifth of it — so the lift's position is unknown, and
                # raising the actuator into a lift that may still be up is the
                # one thing this phase exists to prevent.
                self.abort(
                    f"lower limit switch never closed in "
                    f"{cfg.lift_down_seconds:.0f} s; lift position unknown"
                )
                return Action(phase=self.phase, message=self._message)
            if obs.at_bottom:
                why = "lower limit"
                self._message = f"lift down ({why})"
                if cfg.hole_stage:
                    # _enter re-takes the reference rather than carrying
                    # ENTER's: the drill and lift phases between here and there
                    # can run for a minute and a half, which at 0.02 deg/s
                    # would eat the deadband before the search started.
                    self._enter(Phase.FIND_HOLE, obs,
                                f"lift down ({why}); looking for the hole")
                else:
                    self._enter(Phase.RAISE, obs,
                                f"lift down ({why}); raising the actuator")
                    return Action(actuator=1, phase=self.phase, message=self._message)
                return Action(phase=self.phase, message=self._message)
            self._message = (
                f"lift lowering {elapsed:.1f} s, waiting for the lower limit "
                f"(gives up at {cfg.lift_down_seconds:.0f} s)")
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
                self._enter(Phase.ALIGN_HOLE, obs, "hole found; lining up the actuator")
            wz, note = self._hold_yaw(obs)
            if note:
                self._message = f"{self._message}{note}"
            return Action(wz=wz, phase=self.phase, message=self._message)

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
                self._enter(Phase.RAISE, obs,
                            f"lined up (dx {ex:+.3f}, dy {ey:+.3f}); raising")
                return Action(actuator=1, phase=self.phase, message=self._message)
            cap = cfg.hole_max_speed
            vy = max(-cap, min(cap, cfg.hole_gain_x * ex))
            vx = max(-cap, min(cap, cfg.hole_gain_y * ey))
            # The servo itself does not need this — it is closed in the body
            # frame, so it converges whatever the heading. Staying square is
            # what keeps the mast off the underbody and the hole inside the
            # upward camera's view while it converges.
            wz, note = self._hold_yaw(obs)
            self._message = f"lining up dx {ex:+.3f} dy {ey:+.3f}{note}"
            return Action(vx=vx, vy=vy, wz=wz, phase=self.phase,
                          message=self._message)

        if self.phase is Phase.RAISE:
            if elapsed >= cfg.actuator_seconds:
                self._enter(Phase.SPRAY, obs, "actuator up; opening the valve")
                return Action(solenoid=1, phase=self.phase, message=self._message)
            self._message = f"raising {elapsed:.1f}/{cfg.actuator_seconds:.1f} s"
            return Action(actuator=1, phase=self.phase, message=self._message)

        if self.phase is Phase.SPRAY:
            if elapsed >= cfg.spray_seconds:
                self._enter(Phase.RETRACT, obs, "valve shut; lowering the actuator")
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
                self._enter(Phase.DONE, obs, "actuator down; sequence complete")
                return Action(phase=self.phase, message=self._message)
            self._message = f"actuator lowering {elapsed:.1f}/{cfg.retract_seconds:.1f} s"
            return Action(actuator=-1, phase=self.phase, message=self._message)

        # DONE and ABORT both hold still.
        return Action(phase=self.phase, message=self._message)
