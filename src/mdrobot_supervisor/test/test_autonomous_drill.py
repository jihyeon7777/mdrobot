"""DRILL: the upper limit switch stops the bit and the lift together.

The bit used to turn on a clock of its own (drill_seconds, 20 s) while the
lift ran to its switch, which closes around 53 s. So the bit stopped a third
of the way from the top and the lift spent the rest of the stroke pressing a
stationary bit into the underbody. A clock never knew when the hole was
through; the switch does.

lift_up_seconds is an OPTIONAL backstop for a switch that never closes, and
it is off by default. Drilling through takes as long as it takes, so any clock
running alongside the switch can only stop the bit short of the hole it was
cutting; a run that has gone wrong is stopped by the operator's brake, which
works in every phase.
"""

import pytest

from mdrobot_supervisor.autonomous import (
    AutonomousConfig,
    AutonomousSequence,
    Observation,
    Phase,
)

# Measured on the machine: a full lift stroke runs about this long, and the
# 20 s clock this replaced cut the bit at well under half of it.
STROKE_SECONDS = 53.0
FAULT_BACKSTOP = 70.0


def config(**overrides) -> AutonomousConfig:
    base = dict(
        yaw_hold=False,
        entry_distance=1.0,
        entry_speed=0.08,
        min_approach_width=0.45,
        lift_up_seconds=0.0,   # no backstop, as shipped
        lift_down_seconds=70.0,
    )
    base.update(overrides)
    return AutonomousConfig(**base)


def obs(now, **kw) -> Observation:
    fields = dict(now=now, plate_offset_x=None, plate_age=None, distance=0.0)
    fields.update(kw)
    return Observation(**fields)


def drilling(cfg):
    """A sequence stopped under the car with the bit turning. Returns (seq, t)."""
    seq = AutonomousSequence(cfg)
    seq.step(obs(0.0, plate_offset_x=0.0, plate_age=0.0, plate_width=0.6))
    seq.step(obs(0.5, plate_offset_x=0.0, plate_age=0.0, plate_width=0.6))
    seq.step(obs(1.0, plate_age=99.0))              # lost wide -> ENTER
    assert seq.phase is Phase.ENTER
    seq.step(obs(2.0, distance=cfg.entry_distance + 0.1))
    assert seq.phase is Phase.DRILL
    return seq, 2.0


def test_the_bit_turns_while_the_lift_rises():
    cfg = config()
    seq, t = drilling(cfg)
    act = seq.step(obs(t + 5.0))
    assert act.drill == 1
    assert act.lift == 1, "the bit is only cutting while it is being pushed up"


def test_the_bit_is_still_turning_where_the_old_clock_would_have_stopped_it():
    """20 s was the old drill_seconds. Most of the stroke is past it."""
    cfg = config()
    seq, t = drilling(cfg)
    act = seq.step(obs(t + 25.0))
    assert seq.phase is Phase.DRILL
    assert act.drill == 1


def test_the_upper_limit_switch_stops_the_bit():
    cfg = config()
    seq, t = drilling(cfg)
    act = seq.step(obs(t + STROKE_SECONDS, at_top=True))
    assert act.drill == 0


def test_the_switch_stops_the_bit_on_the_same_tick_as_the_lift():
    """Not a tick later: the reading that stops the push stops the cut."""
    cfg = config()
    seq, t = drilling(cfg)
    act = seq.step(obs(t + STROKE_SECONDS, at_top=True))
    assert act.drill == 0 and act.lift == -1


def test_the_switch_ends_the_phase():
    cfg = config()
    seq, t = drilling(cfg)
    seq.step(obs(t + STROKE_SECONDS, at_top=True))
    assert seq.phase is Phase.LIFT_DOWN


def test_the_message_says_the_switch_stopped_it():
    cfg = config()
    seq, t = drilling(cfg)
    act = seq.step(obs(t + STROKE_SECONDS, at_top=True))
    assert "upper limit" in act.message


def test_a_switch_that_never_closes_stops_the_bit_IF_a_backstop_is_set():
    """The opt-in fault case."""
    cfg = config(lift_up_seconds=FAULT_BACKSTOP)
    seq, t = drilling(cfg)
    act = seq.step(obs(t + FAULT_BACKSTOP + 0.1))
    assert act.drill == 0
    assert seq.phase is Phase.LIFT_DOWN
    assert "NO upper limit" in act.message


def test_the_fault_backstop_bounds_the_bit_not_a_separate_clock():
    """There is one number governing how long the bit may turn, and it is the
    one that governs how long the lift may push. If these ever came apart
    again, one of them would be pressing against the other."""
    cfg = config(lift_up_seconds=8.0)
    seq, t = drilling(cfg)
    assert seq.step(obs(t + 7.9)).drill == 1
    assert seq.step(obs(t + 8.1)).drill == 0


# --- no clock at all, which is how it ships ---------------------------------

def test_as_shipped_there_is_no_backstop():
    assert config().lift_up_seconds == 0


def test_with_no_backstop_the_bit_turns_until_the_switch_however_long():
    """Well past the 53 s stroke and past the 70 s backstop that was tried and
    removed: neither number means anything any more."""
    cfg = config()
    seq, t = drilling(cfg)
    for elapsed in (STROKE_SECONDS, FAULT_BACKSTOP, 300.0, 3600.0):
        act = seq.step(obs(t + elapsed))
        assert seq.phase is Phase.DRILL, f"stopped on its own at {elapsed} s"
        assert act.drill == 1 and act.lift == 1


def test_with_no_backstop_the_switch_still_ends_it():
    """Removing the clock must not remove the thing that actually stops it."""
    cfg = config()
    seq, t = drilling(cfg)
    seq.step(obs(t + 600.0))
    act = seq.step(obs(t + 601.0, at_top=True))
    assert act.drill == 0 and act.lift == -1
    assert seq.phase is Phase.LIFT_DOWN


def test_with_no_backstop_the_message_promises_no_clock():
    cfg = config()
    seq, t = drilling(cfg)
    act = seq.step(obs(t + 100.0))
    assert "backstop" not in act.message


def test_a_negative_backstop_is_refused():
    """0 is off and positive is a timeout; a negative would silently read as a
    timeout that has already expired."""
    with pytest.raises(ValueError, match="lift_up_seconds"):
        config(lift_up_seconds=-1.0)


def test_nothing_turns_the_bit_after_the_phase():
    cfg = config()
    seq, t = drilling(cfg)
    seq.step(obs(t + STROKE_SECONDS, at_top=True))
    act = seq.step(obs(t + STROKE_SECONDS + 1.0))
    assert act.drill == 0 and seq.phase is Phase.LIFT_DOWN


def test_config_has_no_drill_clock_left():
    """A parameter still named in the yaml but no longer read would be applied
    silently to nothing -- which has bitten this package before."""
    assert not hasattr(config(), "drill_seconds")
