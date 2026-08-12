"""Tests for the keyboard teleop. No hardware, no terminal.

The latched state machine and the ramp are pure logic, so they are tested directly.
The loop is tested against a fake base and a scripted key reader.
"""

from __future__ import annotations

import time

import pytest

from mdrobot_mecanum import teleop_keyboard as teleop
from mdrobot_mecanum.base import DriveError, DriveResult
from mdrobot_mecanum.config import MecanumConfig
from mdrobot_mecanum.kinematics import inverse


def make_config(**overrides) -> MecanumConfig:
    data = {
        "version": 1,
        "port": "/dev/null",
        "wheels": {
            "front_left": {"slave_id": 1, "channel": 1, "sign": -1},
            "front_right": {"slave_id": 1, "channel": 2, "sign": 1},
            "rear_left": {"slave_id": 2, "channel": 2, "sign": -1},
            "rear_right": {"slave_id": 2, "channel": 1, "sign": 1},
        },
        "roller_layout": "x",
        "geometry": {"wheel_radius": 0.0625, "track": 0.575, "wheelbase": 0.5,
                     "gear_ratio": 20.0},
        "limits": {"max_motor_rpm": 600, "max_linear_x": 0.2, "max_linear_y": 0.2,
                   "max_angular_z": 0.4, "accel_linear": 0.4, "decel_linear": 0.8,
                   "accel_angular": 0.8, "decel_angular": 1.6},
        # Fast loop so the wall-clock-dependent tests stay quick.
        "runtime": {"loop_hz": 50.0},
    }
    for key, value in overrides.items():
        if isinstance(value, dict) and isinstance(data.get(key), dict):
            data[key] = {**data[key], **value}
        else:
            data[key] = value
    return MecanumConfig.from_dict(data)


def state(**kw) -> teleop.TeleopState:
    return teleop.TeleopState(make_config(), **kw)


# --- the ramp -----------------------------------------------------------------------

def test_ramp_uses_accel_when_growing_and_decel_when_shrinking():
    assert teleop.ramp_toward(0.0, 1.0, accel=2.0, decel=8.0, dt=0.1) == pytest.approx(0.2)
    assert teleop.ramp_toward(1.0, 0.0, accel=2.0, decel=8.0, dt=0.1) == pytest.approx(0.2)


def test_ramp_snaps_to_the_target_rather_than_overshooting():
    assert teleop.ramp_toward(0.95, 1.0, accel=2.0, decel=2.0, dt=1.0) == 1.0
    assert teleop.ramp_toward(-0.95, -1.0, accel=2.0, decel=2.0, dt=1.0) == -1.0


def test_ramp_is_a_no_op_at_the_target():
    assert teleop.ramp_toward(0.5, 0.5, accel=1.0, decel=1.0, dt=1.0) == 0.5


def test_reversing_direction_counts_as_decelerating_first():
    """Crossing zero is a stop then an acceleration, so it must use the decel rate."""
    assert teleop.ramp_toward(1.0, -1.0, accel=1.0, decel=10.0, dt=0.1) == pytest.approx(0.0)


def test_ramping_up_from_zero_uses_accel():
    assert teleop.ramp_toward(0.0, -1.0, accel=2.0, decel=9.0, dt=0.1) == pytest.approx(-0.2)


# --- latched targets ----------------------------------------------------------------

def test_a_translation_key_latches_until_something_changes_it():
    s = state()
    s.handle_key("i")
    assert s.target.vx == pytest.approx(0.2)
    for _ in range(5):
        s.step(0.1)
    assert s.target.vx == pytest.approx(0.2)      # still latched


def test_translation_keys_leave_rotation_alone_and_vice_versa():
    s = state()
    s.handle_key("q")                              # rotate left
    s.handle_key("i")                              # then translate forward
    assert s.target.wz > 0 and s.target.vx > 0

    s2 = state()
    s2.handle_key("i")
    s2.handle_key("q")
    assert s2.target.vx > 0 and s2.target.wz > 0


@pytest.mark.parametrize("key,expected", [
    ("i", (+1, 0)), (",", (-1, 0)), ("j", (0, +1)), ("l", (0, -1)),
    ("u", (+1, +1)), ("o", (+1, -1)), ("m", (-1, +1)), (".", (-1, -1)),
    ("k", (0, 0)),
])
def test_the_translation_pad(key, expected):
    s = state()
    s.handle_key(key)
    fx, fy = expected
    assert s.target.vx == pytest.approx(fx * 0.2)
    assert s.target.vy == pytest.approx(fy * 0.2)


def test_wasd_aliases_match_the_pad_and_a_d_are_strafe_not_turn():
    for alias, pad in (("w", "i"), ("s", ","), ("a", "j"), ("d", "l")):
        a, b = state(), state()
        a.handle_key(alias)
        b.handle_key(pad)
        assert a.target.as_tuple() == b.target.as_tuple()

    strafing = state()
    strafing.handle_key("a")
    assert strafing.target.vy > 0 and strafing.target.wz == 0.0


@pytest.mark.parametrize("key,sign", [("q", +1), ("e", -1), ("r", 0)])
def test_rotation_keys(key, sign):
    s = state()
    s.handle_key(key)
    assert s.target.wz == pytest.approx(sign * 0.4)


# --- stops --------------------------------------------------------------------------

def test_space_is_a_hard_stop_that_zeroes_the_ramp_too():
    s = state()
    s.handle_key("i")
    s.step(1.0)
    assert s.command.vx > 0
    s.handle_key(" ")
    assert s.target.is_zero and s.command.is_zero and s.hard_stop


def test_an_unmapped_key_stops_everything():
    """A startled operator mashing the keyboard must not be a no-op."""
    s = state()
    s.handle_key("i")
    s.handle_key("q")
    s.handle_key("Z")
    assert s.target.is_zero


def test_a_soft_stop_leaves_the_ramp_to_decelerate():
    s = state()
    s.handle_key("i")
    s.step(1.0)
    moving = s.command.vx
    s.stop_soft()
    assert s.target.is_zero and s.command.vx == moving   # ramp not touched yet
    s.step(0.05)
    assert 0 < s.command.vx < moving


@pytest.mark.parametrize("key", ["\x03", "\x1c"])
def test_ctrl_c_and_ctrl_backslash_quit_and_stop_hard(key):
    s = state()
    s.handle_key("i")
    s.step(1.0)
    s.handle_key(key)
    assert s.quit and s.command.is_zero and s.target.is_zero


# --- speed scale --------------------------------------------------------------------

def test_scale_keys_adjust_and_re_apply_to_the_latched_direction():
    s = state()
    s.handle_key("i")
    full = s.target.vx
    s.handle_key("z")                              # linear -10%
    assert s.target.vx == pytest.approx(full * 0.9)


def test_scale_never_reaches_zero():
    """At zero the robot would refuse to move with no explanation."""
    s = state()
    for _ in range(30):
        s.handle_key("z")
    assert s.linear_scale == pytest.approx(teleop.MIN_SCALE)


def test_scale_is_capped_at_one():
    s = state()
    for _ in range(30):
        s.handle_key("c")
    assert s.linear_scale == pytest.approx(1.0)


def test_linear_and_angular_scales_are_independent():
    s = state()
    s.handle_key("z")
    assert s.linear_scale < 1.0 and s.angular_scale == pytest.approx(1.0)
    s.handle_key("b")
    assert s.angular_scale == pytest.approx(1.0)    # already at the cap


@pytest.mark.parametrize("key,value", [("1", 0.2), ("3", 0.6), ("5", 1.0)])
def test_preset_keys_set_both_scales(key, value):
    s = state()
    s.handle_key(key)
    assert s.linear_scale == pytest.approx(value)
    assert s.angular_scale == pytest.approx(value)


def test_the_starting_scale_is_applied():
    s = state(linear_scale=0.2, angular_scale=0.2)
    s.handle_key("i")
    assert s.target.vx == pytest.approx(0.2 * 0.2)


# --- arrows -------------------------------------------------------------------------

@pytest.mark.parametrize("code,check", [
    ("A", lambda s: s.target.vx > 0), ("B", lambda s: s.target.vx < 0),
    ("D", lambda s: s.target.wz > 0), ("C", lambda s: s.target.wz < 0),
])
def test_arrow_keys(code, check):
    s = state()
    s.handle_arrow(code)
    assert check(s)


def test_an_unknown_escape_code_stops():
    s = state()
    s.handle_key("i")
    s.handle_arrow("Z")
    assert s.target.is_zero


# --- the loop -----------------------------------------------------------------------

class FakeBase:
    def __init__(self, config, fail_after=None):
        self.config = config
        self.commands = []
        self._fail_after = fail_after

    def drive(self, vx, vy, wz):
        self.commands.append((vx, vy, wz))
        if self._fail_after is not None and len(self.commands) > self._fail_after:
            raise DriveError("injected")
        omega = inverse(vx, vy, wz, self.config.geometry)
        return DriveResult(twist=(vx, vy, wz), wheel_rad_s=omega,
                           wheel_rpm=(0, 0, 0, 0), scale=1.0)


class ScriptedReader:
    """Hands out a fixed list of events, then None (a timed-out poll) forever.

    A timed-out poll actually waits, as the real one does — otherwise the loop spins
    with no wall-clock passing and anything time-based (the ramp, the idle watchdog)
    is never exercised.
    """

    def __init__(self, events):
        self._events = list(events)
        self.polls = 0

    def poll(self, timeout):
        self.polls += 1
        if self._events:
            return self._events.pop(0)
        time.sleep(timeout)
        return None


def run_loop(base, events, ticks=6, printer=lambda *a: None):
    return teleop.run(base, ScriptedReader(events), printer=printer, max_ticks=ticks)


def test_the_loop_writes_every_tick_even_when_nothing_changes():
    """The controller cuts drive after ~2 s of bus silence: re-assertion is mandatory."""
    base = FakeBase(make_config())
    run_loop(base, [], ticks=5)
    assert len(base.commands) == 5
    assert all(c == (0.0, 0.0, 0.0) for c in base.commands)


def test_the_loop_ramps_toward_a_latched_target():
    base = FakeBase(make_config())
    run_loop(base, [("key", "i")], ticks=6)
    forward = [c[0] for c in base.commands]
    assert forward[0] < forward[-1]
    assert all(a <= b for a, b in zip(forward, forward[1:]))


def test_a_hard_stop_reaches_the_wheels_on_the_same_tick():
    base = FakeBase(make_config())
    run_loop(base, [("key", "i"), None, None, ("key", " ")], ticks=5)
    assert base.commands[-1] == (0.0, 0.0, 0.0)


def test_quitting_writes_a_zero_before_returning():
    base = FakeBase(make_config())
    assert run_loop(base, [("quit", "")], ticks=10) == 0
    assert base.commands[-1] == (0.0, 0.0, 0.0)


def test_the_loop_aborts_after_max_consecutive_comm_errors():
    config = make_config(runtime={"max_comm_errors": 3})
    base = FakeBase(config, fail_after=0)
    messages = []
    assert run_loop(base, [], ticks=20, printer=messages.append) == 1
    assert len(base.commands) == 3
    assert any("too many consecutive failures" in m for m in messages)


def test_a_single_comm_error_does_not_abort():
    config = make_config(runtime={"max_comm_errors": 3})

    class Flaky(FakeBase):
        def drive(self, vx, vy, wz):
            self.commands.append((vx, vy, wz))
            if len(self.commands) == 2:
                raise DriveError("one bad tick")
            return DriveResult(twist=(vx, vy, wz), wheel_rad_s=(0, 0, 0, 0),
                               wheel_rpm=(0, 0, 0, 0), scale=1.0)

    assert run_loop(Flaky(config), [], ticks=6) == 0


def test_the_idle_watchdog_stops_a_latched_motion():
    config = make_config(runtime={"idle_timeout": 0.05, "loop_hz": 100.0})
    base = FakeBase(config)
    messages = []
    run_loop(base, [("key", "i")], ticks=30, printer=messages.append)
    assert any("idle stop" in m for m in messages)
    assert base.commands[-1] == pytest.approx((0.0, 0.0, 0.0))


def test_the_idle_watchdog_never_fires_while_already_stopped():
    config = make_config(runtime={"idle_timeout": 0.05, "loop_hz": 100.0})
    messages = []
    run_loop(FakeBase(config), [], ticks=20, printer=messages.append)
    assert not any("idle stop" in m for m in messages)


def test_the_idle_watchdog_can_be_disabled():
    config = make_config(runtime={"idle_timeout": 0.0, "loop_hz": 100.0})
    messages = []
    run_loop(FakeBase(config), [("key", "i")], ticks=30, printer=messages.append)
    assert not any("idle stop" in m for m in messages)


# --- status rendering ---------------------------------------------------------------

def test_the_status_shows_a_limited_banner_only_when_clamped():
    s = state()
    clamped = DriveResult((0.1, 0, 0), (1, 1, 1, 1), (10, 10, 10, 10), scale=0.62)
    free = DriveResult((0.1, 0, 0), (1, 1, 1, 1), (10, 10, 10, 10), scale=1.0)
    assert "LIMITED" in teleop.render(s, clamped, 0, 100.0, None)
    assert "LIMITED" not in teleop.render(s, free, 0, 100.0, None)


def test_the_status_labels_all_four_wheels_distinctly():
    s = state()
    result = DriveResult((0.1, 0, 0), (1, 1, 1, 1), (1, 2, 3, 4), scale=1.0)
    text = teleop.render(s, result, 0, 100.0, None)
    for label in ("FL", "FR", "RL", "RR"):
        assert label in text


def test_the_status_shows_comm_errors_and_the_idle_countdown_when_present():
    s = state()
    assert "comm errors" not in teleop.render(s, None, 0, 100.0, None)
    assert "comm errors 2" in teleop.render(s, None, 2, 100.0, None)
    assert "IDLE STOP" in teleop.render(s, None, 0, 100.0, 2.0)


def test_the_status_survives_a_tick_with_no_drive_result():
    assert "target" in teleop.render(state(), None, 0, 100.0, None)


# --- key legend ---------------------------------------------------------------------

def test_the_legend_documents_that_a_and_d_strafe():
    assert "STRAFE" in teleop.KEY_LEGEND


def test_the_legend_documents_the_hard_stop():
    assert "HARD STOP" in teleop.KEY_LEGEND


# --- loop timing --------------------------------------------------------------------

def test_the_loop_holds_its_configured_period_despite_the_work_it_does():
    """Polling a whole period THEN writing would make the real period period+work.

    A loop_hz of 10 would quietly run at 8, and raising it to 15 would quietly give 10.
    """
    config = make_config(runtime={"loop_hz": 20.0})       # 50 ms period

    class SlowBase(FakeBase):
        def drive(self, vx, vy, wz):
            time.sleep(0.02)                              # 20 ms of "bus writes"
            return super().drive(vx, vy, wz)

    base = SlowBase(config)
    started = time.monotonic()
    run_loop(base, [], ticks=10)
    elapsed = time.monotonic() - started
    assert elapsed == pytest.approx(10 * 0.05, rel=0.35)


def test_a_hard_stop_does_not_wait_out_the_rest_of_the_period():
    """Measure the gap between SPACE arriving and the zero reaching the wheels."""
    config = make_config(runtime={"loop_hz": 2.0})        # 500 ms period

    class TimedBase(FakeBase):
        def __init__(self, config):
            super().__init__(config)
            self.times = []

        def drive(self, vx, vy, wz):
            self.times.append(time.monotonic())
            return super().drive(vx, vy, wz)

    class TimedReader(ScriptedReader):
        def __init__(self, events):
            super().__init__(events)
            self.space_at = None

        def poll(self, timeout):
            event = super().poll(timeout)
            if event == ("key", " "):
                self.space_at = time.monotonic()
            return event

    base, reader = TimedBase(config), TimedReader([("key", "i"), ("key", " ")])
    teleop.run(base, reader, printer=lambda *a: None, max_ticks=2)

    # The drive that follows SPACE must land far sooner than the 500 ms period.
    after_space = [t for t in base.times if t >= reader.space_at]
    assert after_space, "no command was issued after the hard stop"
    assert after_space[0] - reader.space_at < 0.1
    assert base.commands[-1] == (0.0, 0.0, 0.0)


def test_an_overrunning_tick_still_leaves_a_window_for_keys():
    """Losing key control of a moving robot when the loop struggles is unacceptable."""
    config = make_config(runtime={"loop_hz": 50.0})       # 20 ms period

    class TooSlow(FakeBase):
        def drive(self, vx, vy, wz):
            time.sleep(0.05)                              # every tick overruns
            return super().drive(vx, vy, wz)

    base = TooSlow(config)
    reader = ScriptedReader([None, ("key", "i")])
    teleop.run(base, reader, printer=lambda *a: None, max_ticks=4)
    assert any(c[0] > 0 for c in base.commands)
