"""DRILL: the upper limit switch stops the bit and the lift together.

The bit used to turn on a clock of its own (drill_seconds, 20 s) while the
lift ran to its switch, which closes around 53 s. So the bit stopped a third
of the way from the top and the lift spent the rest of the stroke pressing a
stationary bit into the underbody. A clock never knew when the hole was
through; the switch does.

The fault case still has to be bounded: a switch that never closes must not
mean a bit that turns until the phase is abandoned. lift_up_seconds is that
backstop, and it now bounds the bit as well as the lift.
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
        lift_up_seconds=FAULT_BACKSTOP,
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


def test_a_switch_that_never_closes_still_stops_the_bit():
    """The fault case. A broken wire must not leave the bit turning."""
    cfg = config()
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
