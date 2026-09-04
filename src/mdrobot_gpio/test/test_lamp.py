"""The hold. It is the only thing in a lamp worth getting wrong."""

import pytest

from mdrobot_gpio.lamp import Latch


def test_it_starts_dark():
    assert Latch(1.0).lit(0.0) is False


def test_a_signal_lights_it():
    lamp = Latch(1.0)
    lamp.signal(10.0)
    assert lamp.lit(10.0) is True


def test_it_holds_through_a_dropout():
    # The detector goes quiet for a beat at a time on a plate in plain view.
    # Without this the lamp flickers and says less than the topic it replaced.
    lamp = Latch(1.0)
    lamp.signal(10.0)
    assert lamp.lit(10.9) is True


def test_it_goes_dark_once_the_hold_runs_out():
    lamp = Latch(1.0)
    lamp.signal(10.0)
    assert lamp.lit(11.01) is False


def test_the_boundary_is_inclusive():
    lamp = Latch(1.0)
    lamp.signal(10.0)
    assert lamp.lit(11.0) is True


def test_each_signal_restarts_the_hold():
    lamp = Latch(1.0)
    lamp.signal(10.0)
    lamp.signal(10.8)
    assert lamp.lit(11.7) is True


def test_clear_goes_dark_at_once():
    # For a signal that is a STATE rather than an event: a brake reported as 0
    # is known to be off, where a detector going quiet only means nothing has
    # arrived yet.
    lamp = Latch(1.0)
    lamp.signal(10.0)
    lamp.clear()
    assert lamp.lit(10.0) is False


def test_a_reading_from_the_future_does_not_latch_it_on():
    # A clock that jumps backwards would otherwise leave the lamp lit for good,
    # which is the one failure an indicator must not have: saying yes forever.
    lamp = Latch(1.0)
    lamp.signal(100.0)
    assert lamp.lit(10.0) is False


def test_a_non_positive_hold_is_rejected():
    with pytest.raises(ValueError, match="hold must be positive"):
        Latch(0.0)
