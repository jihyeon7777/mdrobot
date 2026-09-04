"""Yaw hold: the correction, its limits, and the guard that stops instead.

The sequence ends by running a drill under a car, so the interesting cases here
are the ones where the right answer is to stop rather than to steer harder.

Numbers used in the assertions come from the machine, 2026-09-03: driven for
90 s and returned to marks on the floor, the reported heading came back 1.75
deg off, which is why the deadband sits at 2 deg.
"""

import pytest

from mdrobot_supervisor.autonomous import (
    AutonomousConfig,
    AutonomousSequence,
    Observation,
    Phase,
    wrap_deg,
)

DRIFT_MEASURED_DEG = 1.75


def config(**overrides) -> AutonomousConfig:
    base = dict(
        yaw_hold=True,
        entry_distance=1.0,
        entry_speed=0.08,
        min_approach_width=0.45,
        drill_seconds=2.0,
        lift_up_seconds=1.0,
        lift_down_seconds=1.0,
        hole_stage=True,
    )
    base.update(overrides)
    return AutonomousConfig(**base)


def obs(now, *, yaw=0.0, yaw_age=0.0, distance=0.0, **kw) -> Observation:
    fields = dict(
        now=now, plate_offset_x=None, plate_age=None, distance=distance,
        yaw=yaw, yaw_age=yaw_age,
    )
    fields.update(kw)
    return Observation(**fields)


def drive_to_enter(seq, *, yaw=0.0, t=0.0):
    """Walk the machine from WAIT_PLATE into ENTER and return the tick time.

    Three ticks, not two: the width is only recorded on a tick spent IN align
    with a fresh plate, and the tick that enters align does not record one. A
    plate that was never seen wide reads as a dropped detection rather than an
    arrival, and the machine rightly stays put.
    """
    seq.step(obs(t, plate_offset_x=0.0, plate_age=0.0, plate_width=0.6, yaw=yaw))
    assert seq.phase is Phase.ALIGN
    seq.step(obs(t + 0.5, plate_offset_x=0.0, plate_age=0.0, plate_width=0.6,
                 yaw=yaw))
    # Losing a plate that had got wide is what counts as arriving at the car.
    seq.step(obs(t + 1.0, plate_age=99.0, yaw=yaw))
    assert seq.phase is Phase.ENTER
    return t + 1.0


def drive_to_find_hole(seq, cfg, *, yaw=0.0, drifted_to=None):
    """Walk the machine all the way to FIND_HOLE.

    ``drifted_to`` is the heading it reads by the time the drill and lift are
    done — the estimate drifts while those run, and the search is supposed to
    re-zero on arrival rather than inherit that.
    """
    later = yaw if drifted_to is None else drifted_to
    t = drive_to_enter(seq, yaw=yaw)
    seq.step(obs(t + 1.0, distance=cfg.entry_distance + 0.1, yaw=yaw))
    assert seq.phase is Phase.DRILL
    seq.step(obs(t + 2.0, at_top=True, yaw=yaw))
    t += 2.0 + cfg.drill_seconds
    seq.step(obs(t, at_top=True, yaw=later))
    assert seq.phase is Phase.LIFT_DOWN
    seq.step(obs(t + 0.1, at_bottom=True, yaw=later))
    assert seq.phase is Phase.FIND_HOLE
    return t + 0.1


# --- the feature is opt-in --------------------------------------------------

def test_with_yaw_hold_off_nothing_yaws_however_far_the_heading_moves():
    seq = AutonomousSequence(config(yaw_hold=False))
    t = drive_to_enter(seq, yaw=0.0)
    action = seq.step(obs(t + 1.0, yaw=45.0, distance=0.1))
    assert action.wz == 0.0
    assert seq.phase is Phase.ENTER


def test_with_yaw_hold_off_a_missing_heading_is_not_a_fault():
    # The sensor is not required until it is asked for.
    seq = AutonomousSequence(config(yaw_hold=False))
    t = drive_to_enter(seq)
    action = seq.step(obs(t + 1.0, yaw=None, yaw_age=None, distance=0.1))
    assert seq.phase is Phase.ENTER
    assert action.wz == 0.0


# --- the correction ---------------------------------------------------------

def test_drift_sized_error_is_ignored():
    # The measured 90 s closure error. Correcting this would rotate the machine
    # to match the sensor's own error.
    seq = AutonomousSequence(config())
    t = drive_to_enter(seq, yaw=10.0)
    action = seq.step(obs(t + 1.0, yaw=10.0 + DRIFT_MEASURED_DEG, distance=0.1))
    assert action.wz == 0.0
    assert "yaw +1.8 deg" in action.message


def test_an_anticlockwise_error_is_corrected_clockwise():
    seq = AutonomousSequence(config(yaw_gain=0.01, yaw_max_wz=1.0))
    t = drive_to_enter(seq, yaw=0.0)
    action = seq.step(obs(t + 1.0, yaw=5.0, distance=0.1))
    # +5 deg means it has turned anticlockwise past the reference, so it needs
    # a clockwise (negative) wz to come back.
    assert action.wz == pytest.approx(-0.05)
    assert "correcting yaw +5.0 deg" in action.message


def test_a_clockwise_error_is_corrected_anticlockwise():
    seq = AutonomousSequence(config(yaw_gain=0.01, yaw_max_wz=1.0))
    t = drive_to_enter(seq, yaw=0.0)
    action = seq.step(obs(t + 1.0, yaw=-5.0, distance=0.1))
    assert action.wz == pytest.approx(0.05)


def test_the_correction_is_capped():
    # This runs under a car. A large error must not become a large manoeuvre.
    seq = AutonomousSequence(config(yaw_gain=0.01, yaw_max_wz=0.08))
    t = drive_to_enter(seq, yaw=0.0)
    action = seq.step(obs(t + 1.0, yaw=14.0, distance=0.1))
    assert action.wz == pytest.approx(-0.08)


def test_correcting_does_not_disturb_the_forward_speed():
    cfg = config()
    seq = AutonomousSequence(cfg)
    t = drive_to_enter(seq, yaw=0.0)
    action = seq.step(obs(t + 1.0, yaw=6.0, distance=0.1))
    assert action.vx == cfg.entry_speed
    assert action.vy == 0.0


def test_the_error_is_measured_the_short_way_round_the_circle():
    # A machine at -179 that was referenced at +179 has turned 2 deg, not 358.
    seq = AutonomousSequence(config(yaw_deadband_deg=5.0))
    t = drive_to_enter(seq, yaw=179.0)
    action = seq.step(obs(t + 1.0, yaw=-179.0, distance=0.1))
    assert action.wz == 0.0
    assert "yaw +2.0 deg" in action.message


# --- where the reference is taken -------------------------------------------

def test_the_search_re_zeroes_instead_of_inheriting_the_entry_reference():
    # The estimate drifts at about 0.02 deg/s and the phases in between can run
    # for a minute and a half. Carrying the entry's reference through would
    # spend the whole deadband on drift before the search began.
    cfg = config()
    seq = AutonomousSequence(cfg)
    t = drive_to_find_hole(seq, cfg, yaw=0.0, drifted_to=3.0)
    assert seq._yaw_ref == pytest.approx(3.0)
    # 3 deg from where entry started, but 0 from where the search did, so the
    # search neither corrects for it nor trips on it.
    action = seq.step(obs(t + 0.1, yaw=3.0))
    assert seq.phase is Phase.FIND_HOLE
    assert action.wz == 0.0


def test_every_stationary_phase_takes_its_own_reference():
    # Otherwise drift accumulated across drill and lift -- which together can
    # run for a minute and a half -- would arrive at the tight stationary limit
    # as though the machine had turned.
    cfg = config(yaw_stationary_abort_deg=4.0)
    seq = AutonomousSequence(cfg)
    t = drive_to_enter(seq, yaw=0.0)
    seq.step(obs(t + 1.0, distance=cfg.entry_distance + 0.1, yaw=0.0))
    assert seq.phase is Phase.DRILL and seq._yaw_ref == pytest.approx(0.0)
    # 3 deg of drift through the drill: inside the limit, not a fault.
    seq.step(obs(t + 1.5, at_top=True, yaw=3.0))
    assert seq.phase is Phase.DRILL
    seq.step(obs(t + 1.0 + cfg.drill_seconds, at_top=True, yaw=3.0))
    assert seq.phase is Phase.LIFT_DOWN
    # Lift down starts afresh from 3, so another 3 is still not a fault -- where
    # 6 measured from the drill's reference would have been.
    assert seq._yaw_ref == pytest.approx(3.0)
    # Well inside lift_down_seconds: this must fail on the heading or not at
    # all, not on the lower-limit clock.
    seq.step(obs(t + 1.1 + cfg.drill_seconds, yaw=6.0))
    assert seq.phase is Phase.LIFT_DOWN


# --- the stationary limit is the tighter one --------------------------------

def test_a_stationary_phase_stops_at_an_error_a_driving_one_would_correct():
    # Measured: 30 s through a real drill cycle moved the heading 0.42 deg,
    # which is drift alone. With the wheels stopped the machine has no business
    # turning, so the limit there can be — and is — far tighter.
    cfg = config(yaw_abort_deg=15.0, yaw_stationary_abort_deg=4.0)

    driving = AutonomousSequence(cfg)
    t = drive_to_enter(driving, yaw=0.0)
    action = driving.step(obs(t + 1.0, yaw=6.0, distance=0.1))
    assert driving.phase is Phase.ENTER
    assert action.wz != 0.0  # corrected

    stopped = AutonomousSequence(cfg)
    t = drive_to_enter(stopped, yaw=0.0)
    stopped.step(obs(t + 1.0, distance=cfg.entry_distance + 0.1, yaw=0.0))
    assert stopped.phase is Phase.DRILL
    stopped.step(obs(t + 1.5, yaw=6.0))
    assert stopped.phase is Phase.ABORT
    assert "wheels stopped and the hole occupied" in stopped._message


def test_the_reference_is_taken_when_the_plate_is_acquired():
    # ALIGN takes it, not ENTER: the approach is one continuous run on one
    # heading from the plate coming into view to the drill going in.
    seq = AutonomousSequence(config())
    seq.step(obs(0.0, plate_offset_x=0.0, plate_age=0.0, plate_width=0.6, yaw=30.0))
    assert seq.phase is Phase.ALIGN
    assert seq._yaw_ref == pytest.approx(30.0)


def test_the_entry_inherits_the_alignment_reference():
    # Re-zeroing at the plate-loss would adopt whatever heading the machine had
    # drifted to during plate_timeout -- seconds of driving with no offset to
    # steer on, which is exactly when it is most likely to have wandered.
    seq = AutonomousSequence(config())
    drive_to_enter(seq, yaw=30.0)
    assert seq.phase is Phase.ENTER
    assert seq._yaw_ref == pytest.approx(30.0)


def test_find_hole_holds_the_heading_while_it_waits():
    cfg = config(yaw_gain=0.01, yaw_max_wz=1.0)
    seq = AutonomousSequence(cfg)
    t = drive_to_find_hole(seq, cfg, yaw=0.0)
    action = seq.step(obs(t + 0.1, yaw=5.0))
    assert seq.phase is Phase.FIND_HOLE
    assert action.wz == pytest.approx(-0.05)


def test_align_hole_stays_square_while_it_translates_onto_the_hole():
    cfg = config(yaw_gain=0.01, yaw_max_wz=1.0, hole_gain_x=-0.3, hole_gain_y=-0.3)
    seq = AutonomousSequence(cfg)
    t = drive_to_find_hole(seq, cfg, yaw=0.0)
    seq.step(obs(t + 0.1, yaw=0.0, hole_offset_x=0.5, hole_offset_y=0.0, hole_age=0.0))
    assert seq.phase is Phase.ALIGN_HOLE
    action = seq.step(obs(t + 0.2, yaw=5.0,
                          hole_offset_x=0.5, hole_offset_y=0.0, hole_age=0.0))
    assert action.wz == pytest.approx(-0.05)
    assert action.vy != 0.0  # still servoing onto the hole


# --- the guard: stopping rather than steering harder ------------------------

def test_a_runaway_heading_aborts_rather_than_correcting_harder():
    seq = AutonomousSequence(config(yaw_abort_deg=15.0))
    t = drive_to_enter(seq, yaw=0.0)
    action = seq.step(obs(t + 1.0, yaw=20.0, distance=0.1))
    assert seq.phase is Phase.ABORT
    assert action.wz == 0.0
    assert "more than slip explains" in action.message


def test_the_machine_stops_if_it_turns_with_the_bit_in_the_hole():
    # The reason the guard runs in DRILL at all: the wheels are commanded to
    # zero while a bit cuts into steel, and the reaction torque acts on a
    # machine standing on rollers. Turning now is what breaks the bit — and
    # nothing steers it back, because steering it back turns it too.
    cfg = config(yaw_stationary_abort_deg=4.0)
    seq = AutonomousSequence(cfg)
    t = drive_to_enter(seq, yaw=0.0)
    seq.step(obs(t + 1.0, distance=cfg.entry_distance + 0.1, yaw=0.0))
    assert seq.phase is Phase.DRILL
    action = seq.step(obs(t + 1.5, yaw=9.0))
    assert seq.phase is Phase.ABORT
    assert action.drill == 0
    assert action.lift == 0
    assert action.wz == 0.0


def test_a_heading_that_goes_missing_aborts_when_the_guard_was_asked_for():
    seq = AutonomousSequence(config())
    t = drive_to_enter(seq, yaw=0.0)
    seq.step(obs(t + 1.0, yaw=None, yaw_age=None, distance=0.1))
    assert seq.phase is Phase.ABORT
    assert "nothing is publishing a heading" in seq._message


def test_a_stale_heading_aborts():
    seq = AutonomousSequence(config(yaw_timeout=0.5))
    t = drive_to_enter(seq, yaw=0.0)
    seq.step(obs(t + 1.0, yaw=0.0, yaw_age=2.0, distance=0.1))
    assert seq.phase is Phase.ABORT
    assert "stale" in seq._message


def test_an_abort_is_terminal_and_commands_nothing():
    seq = AutonomousSequence(config())
    t = drive_to_enter(seq, yaw=0.0)
    seq.step(obs(t + 1.0, yaw=90.0, distance=0.1))
    assert seq.phase is Phase.ABORT
    action = seq.step(obs(t + 2.0, yaw=0.0, distance=0.2))
    assert seq.phase is Phase.ABORT
    assert (action.vx, action.vy, action.wz, action.drill) == (0.0, 0.0, 0.0, 0)


# --- configuration ----------------------------------------------------------

def test_an_abort_threshold_inside_the_deadband_is_rejected():
    # Otherwise the sequence aborts on headings it was told to ignore.
    with pytest.raises(ValueError, match="must exceed"):
        AutonomousConfig(yaw_deadband_deg=5.0, yaw_abort_deg=3.0,
                         yaw_stationary_abort_deg=3.0)


def test_a_stationary_limit_looser_than_the_driving_one_is_rejected():
    # The phases with the bit in the hole are the ones that need the tighter
    # limit; the other way round reads as a typo, not a policy.
    with pytest.raises(ValueError, match="looser"):
        AutonomousConfig(yaw_abort_deg=5.0, yaw_stationary_abort_deg=10.0)


@pytest.mark.parametrize(
    "name", ["yaw_timeout", "yaw_gain", "yaw_max_wz",
             "yaw_stationary_abort_deg"])
def test_non_positive_yaw_tuning_is_rejected(name):
    with pytest.raises(ValueError, match=name):
        AutonomousConfig(**{name: 0.0})


@pytest.mark.parametrize(
    "angle, folded",
    [(0.0, 0.0), (179.9, 179.9), (180.0, -180.0), (181.0, -179.0), (-181.0, 179.0)],
)
def test_wrap_deg_folds_into_a_half_open_turn(angle, folded):
    assert wrap_deg(angle) == pytest.approx(folded)


# --- ALIGN: the phase that slides hardest -----------------------------------

def test_align_holds_the_heading_while_it_strafes():
    # Sideways is where mecanum rollers give up first, and this phase strafes
    # at up to align_max_speed. A machine that yaws while it slides sees the
    # plate move because the CAMERA turned, and enters the car crooked.
    cfg = config(yaw_gain=0.01, yaw_max_wz=1.0)
    seq = AutonomousSequence(cfg)
    seq.step(obs(0.0, plate_offset_x=0.0, plate_age=0.0, plate_width=0.6, yaw=0.0))
    assert seq.phase is Phase.ALIGN
    action = seq.step(obs(0.5, plate_offset_x=0.5, plate_age=0.0,
                          plate_width=0.6, yaw=5.0))
    assert action.wz == pytest.approx(-0.05)
    assert action.vy != 0.0  # still strafing onto the plate
    assert "correcting yaw +5.0 deg" in action.message


def test_the_strafe_is_capped():
    # A plate at the edge of frame asks for align_gain m/s sideways; the cap is
    # what stops the phase that most needs the heading held from being the one
    # sliding hardest.
    cfg = config(align_gain=0.4, align_max_speed=0.1)
    seq = AutonomousSequence(cfg)
    seq.step(obs(0.0, plate_offset_x=0.0, plate_age=0.0, plate_width=0.6))
    action = seq.step(obs(0.5, plate_offset_x=1.0, plate_age=0.0, plate_width=0.6))
    assert action.vy == pytest.approx(-0.1)
    action = seq.step(obs(1.0, plate_offset_x=-1.0, plate_age=0.0, plate_width=0.6))
    assert action.vy == pytest.approx(0.1)


def test_a_small_offset_is_not_slowed_by_the_cap():
    # Capping rather than lowering the gain is the point: close offsets still
    # close at the gain's speed.
    cfg = config(align_gain=0.4, align_max_speed=0.1)
    seq = AutonomousSequence(cfg)
    seq.step(obs(0.0, plate_offset_x=0.0, plate_age=0.0, plate_width=0.6))
    action = seq.step(obs(0.5, plate_offset_x=0.15, plate_age=0.0, plate_width=0.6))
    assert action.vy == pytest.approx(-0.4 * 0.15)


def test_the_heading_is_held_even_while_the_detector_is_blind():
    # A plate that vanishes while still narrow is a dropped detection, so the
    # machine keeps closing with nothing to steer on. The heading is the one
    # thing still measured.
    cfg = config(yaw_gain=0.01, yaw_max_wz=1.0, min_approach_width=0.45)
    seq = AutonomousSequence(cfg)
    seq.step(obs(0.0, plate_offset_x=0.0, plate_age=0.0, plate_width=0.2, yaw=0.0))
    seq.step(obs(0.5, plate_offset_x=0.0, plate_age=0.0, plate_width=0.2, yaw=0.0))
    action = seq.step(obs(1.0, plate_age=99.0, yaw=5.0))
    assert seq.phase is Phase.ALIGN  # too narrow to count as an arrival
    assert action.vy == 0.0          # no offset to steer on
    assert action.wz == pytest.approx(-0.05)   # but the heading is still held
