"""MecanumBase unit tests against a fake two-controller bus. No hardware.

The fake follows the same shape as `src/mdrobot/test/test_device.py:17-61` — respond
per function code, record every request frame — extended in the one way this layer
needs: registers are keyed by `(slave_id, pid)`, so the two controllers can hold
different monitor data and the interleaved bus traffic can be asserted exactly.
"""

from __future__ import annotations

import math
import time

import pytest
from mdrobot import frame, registers as reg
from mdrobot.crc import append_crc
from mdrobot.device import DualMotorDriver
from mdrobot.exceptions import MdrobotError
from mdrobot.protocol import ModbusClient

from mdrobot_mecanum.base import (
    COMMAND_WATCHDOG_S,
    INT16_MAX,
    DriveError,
    MecanumBase,
    borrow_single,
)
from mdrobot_mecanum.config import MecanumConfig
from mdrobot_mecanum.kinematics import WHEEL_NAMES

R, TRACK, WHEELBASE = 0.05, 0.30, 0.28


#: `fail_on` value meaning "keep failing this register for the rest of the test".
FAIL_FOREVER = -1


class FakeBus:
    """Fake transport carrying two controllers. Records every request frame."""

    def __init__(self, registers=None, fail_on=None) -> None:
        # {(slave_id, pid): [words]}
        self.registers = {key: list(words) for key, words in (registers or {}).items()}
        self.frames: list[bytes] = []
        #: {(slave_id, pid): how many further writes to reject; FAIL_FOREVER = all}
        self.fail_on = dict(fail_on or {})
        self.closed = 0
        self._rx = bytearray()

    # -- transport interface --
    def write(self, data: bytes) -> int:
        data = bytes(data)
        self.frames.append(data)
        slave, func = data[0], data[1]
        pid = (data[2] << 8) | data[3]
        if func in (0x06, 0x10):
            budget = self.fail_on.get((slave, pid), 0)
            if budget:
                if budget > 0:
                    self.fail_on[(slave, pid)] = budget - 1
                raise MdrobotError(f"injected write failure at id {slave} pid {pid}")
        self._rx += self._respond(data)
        return len(data)

    def read(self, size: int) -> bytes:
        chunk = bytes(self._rx[:size])
        del self._rx[:size]
        return chunk

    def flush_input(self) -> None:
        pass

    def close(self) -> None:
        self.closed += 1

    # -- responses --
    def _respond(self, req: bytes) -> bytes:
        slave, func = req[0], req[1]
        pid = (req[2] << 8) | req[3]
        if func == 0x03:
            count = (req[4] << 8) | req[5]
            words = self.registers.get((slave, pid), [])
            words = (words + [0] * count)[:count]
            body = bytearray((slave, 0x03, 2 * count))
            for word in words:
                body.append((word >> 8) & 0xFF)
                body.append(word & 0xFF)
            return append_crc(bytes(body))
        if func == 0x06:
            self.registers[(slave, pid)] = [(req[4] << 8) | req[5]]
            return req
        if func == 0x10:
            return append_crc(req[:6])
        raise AssertionError(f"unexpected func 0x{func:02x}")

    # -- assertions helpers --
    def writes(self) -> list[bytes]:
        return [f for f in self.frames if f[1] in (0x06, 0x10)]


def w1(slave_id: int, pid: int, word: int) -> bytes:
    return frame.build_write_single_request(slave_id, pid, word)


def wn(slave_id: int, pid: int, words) -> bytes:
    return frame.build_write_multiple_request(slave_id, pid, list(words))


def vel(rpm: int) -> int:
    """Two's-complement wire word for a signed rpm."""
    return rpm & 0xFFFF


def make_config(**overrides) -> MecanumConfig:
    data = {
        "version": 1,
        "port": "/dev/null",
        "wheels": {
            "front_left": {"slave_id": 1, "channel": 1, "sign": 1},
            "front_right": {"slave_id": 1, "channel": 2, "sign": 1},
            "rear_left": {"slave_id": 2, "channel": 1, "sign": 1},
            "rear_right": {"slave_id": 2, "channel": 2, "sign": 1},
        },
        "roller_layout": "x",
        "geometry": {"wheel_radius": R, "track": TRACK, "wheelbase": WHEELBASE},
        "limits": {"max_motor_rpm": 100},
    }
    for key, value in overrides.items():
        if isinstance(value, dict) and isinstance(data.get(key), dict):
            data[key] = {**data[key], **value}
        else:
            data[key] = value
    return MecanumConfig.from_dict(data)


def make_base(config=None, bus=None) -> tuple[MecanumBase, FakeBus]:
    config = config or make_config()
    bus = bus or FakeBus()
    drivers = {sid: DualMotorDriver(ModbusClient(bus, slave_id=sid))
               for sid in config.slave_ids}
    return MecanumBase(config, bus, drivers), bus


def rpm_for(vx: float, cfg: MecanumConfig) -> int:
    """Motor rpm each wheel gets for a pure +vx command (all four are equal)."""
    return round(vx / cfg.geometry.wheel_radius * 60.0 / (2 * math.pi)
                 * cfg.geometry.gear_ratio)


# --- wiring -------------------------------------------------------------------------

def test_one_driver_per_controller():
    base, _ = make_base()
    assert base.slave_ids == (1, 2)
    assert base.driver(1) is not base.driver(2)


def test_driver_rejects_an_unconfigured_slave_id():
    base, _ = make_base()
    with pytest.raises(ValueError, match="no controller at slave id 5"):
        base.driver(5)


def test_wheel_at_maps_a_motor_output_back_to_its_wheel():
    base, _ = make_base()
    assert base.wheel_at(1, 1) == "front_left"
    assert base.wheel_at(2, 2) == "rear_right"
    assert base.wheel_at(3, 1) is None


# --- drive dispatch -----------------------------------------------------------------

def test_pure_forward_writes_the_same_rpm_to_all_four_channels():
    cfg = make_config()
    base, bus = make_base(cfg)
    result = base.drive(0.2, 0.0, 0.0)

    n = rpm_for(0.2, cfg)
    assert result.wheel_rpm == (n, n, n, n)
    assert bus.writes() == [
        w1(1, reg.PID_VEL_CMD, vel(n)),
        w1(1, reg.PID_VEL_CMD2, vel(n)),
        w1(2, reg.PID_VEL_CMD, vel(n)),
        w1(2, reg.PID_VEL_CMD2, vel(n)),
    ]


def test_strafe_writes_the_minus_plus_plus_minus_pattern():
    base, bus = make_base()
    result = base.drive(0.0, 0.1, 0.0)
    fl, fr, rl, rr = result.wheel_rpm
    assert (fl, fr, rl, rr) == (-fr, fr, fr, -fr) and fr > 0
    assert bus.writes() == [
        w1(1, reg.PID_VEL_CMD, vel(fl)), w1(1, reg.PID_VEL_CMD2, vel(fr)),
        w1(2, reg.PID_VEL_CMD, vel(rl)), w1(2, reg.PID_VEL_CMD2, vel(rr)),
    ]


def test_rotation_writes_the_minus_plus_minus_plus_pattern():
    base, _ = make_base()
    fl, fr, rl, rr = base.drive(0.0, 0.0, 0.8).wheel_rpm
    assert fl < 0 and fr > 0 and rl < 0 and rr > 0
    assert (fl, rl) == (-fr, -rr)


def test_zero_twist_still_writes_zero_to_every_channel():
    """Re-asserting zero is what makes a yanked cable detectable instead of ignored."""
    base, bus = make_base()
    base.drive(0.0, 0.0, 0.0)
    assert bus.writes() == [
        w1(1, reg.PID_VEL_CMD, 0), w1(1, reg.PID_VEL_CMD2, 0),
        w1(2, reg.PID_VEL_CMD, 0), w1(2, reg.PID_VEL_CMD2, 0),
    ]


def test_negative_rpm_is_written_as_a_twos_complement_word():
    base, bus = make_base()
    base.drive(-0.2, 0.0, 0.0)
    word = (bus.writes()[0][4] << 8) | bus.writes()[0][5]
    assert word > 0x8000 and word == vel(base.drive(-0.2, 0.0, 0.0).wheel_rpm[0])


# --- per-wheel sign -----------------------------------------------------------------

def test_sign_inverts_only_the_wheels_it_is_set_on():
    cfg = make_config().with_signs([1, -1, 1, -1])
    base, _ = make_base(cfg)
    fl, fr, rl, rr = base.drive(0.2, 0.0, 0.0).wheel_rpm
    assert fl > 0 and rl > 0 and fr == -fl and rr == -fl


def test_sign_is_applied_after_the_clamp_so_it_cannot_change_magnitudes():
    fast = make_config(limits={"max_motor_rpm": 40})
    plain, _ = make_base(fast)
    flipped, _ = make_base(fast.with_signs([-1, -1, -1, -1]))
    a = plain.drive(2.0, 1.0, 3.0)
    b = flipped.drive(2.0, 1.0, 3.0)
    assert b.wheel_rpm == tuple(-v for v in a.wheel_rpm)
    assert b.scale == a.scale


# --- proportional clamp -------------------------------------------------------------

def test_an_over_limit_twist_is_scaled_not_clipped():
    cfg = make_config(limits={"max_motor_rpm": 40})
    base, _ = make_base(cfg)
    result = base.drive(2.0, 1.0, 3.0)

    assert result.clamped and 0.0 < result.scale < 1.0
    assert max(abs(v) for v in result.wheel_rpm) == 40


def test_the_clamp_preserves_the_wheel_speed_ratios():
    """This is the property that keeps a clamped straight line straight."""
    slow, _ = make_base(make_config(limits={"max_motor_rpm": 10_000}))
    fast, _ = make_base(make_config(limits={"max_motor_rpm": 40}))
    unclamped = slow.drive(2.0, 1.0, 3.0)
    clamped = fast.drive(2.0, 1.0, 3.0)
    for a, b in zip(unclamped.wheel_rpm, clamped.wheel_rpm):
        assert b == pytest.approx(a * clamped.scale, abs=1.0)


def test_effective_twist_reports_the_motion_the_robot_will_actually_make():
    base, _ = make_base(make_config(limits={"max_motor_rpm": 40}))
    result = base.drive(2.0, 1.0, 3.0)
    assert result.effective_twist == pytest.approx(
        (2.0 * result.scale, 1.0 * result.scale, 3.0 * result.scale))


def test_a_twist_within_the_limit_is_not_scaled_at_all():
    base, _ = make_base()
    result = base.drive(0.1, 0.0, 0.0)
    assert result.scale == 1.0 and not result.clamped


def test_every_written_rpm_stays_inside_the_int16_wire_limit():
    """word_from_int16 is a bare & 0xFFFF: past 32767 the sign would silently flip."""
    base, _ = make_base(make_config(limits={"max_motor_rpm": 32767}))
    for rpm in base.drive(500.0, 500.0, 500.0).wheel_rpm:
        assert -INT16_MAX <= rpm <= INT16_MAX


# --- gear ratio ---------------------------------------------------------------------

def test_gear_ratio_scales_the_motor_rpm_but_not_the_wheel_speed():
    geared = make_config(geometry={"wheel_radius": R, "track": TRACK,
                                   "wheelbase": WHEELBASE, "gear_ratio": 10.0},
                         limits={"max_motor_rpm": 10_000})
    plain = make_config(limits={"max_motor_rpm": 10_000})
    a, _ = make_base(geared)
    b, _ = make_base(plain)
    # rel, not abs: both sides are rounded to whole rpm, so a x10 comparison inherits
    # ten times the rounding slack.
    assert a.drive(0.2, 0, 0).wheel_rpm[0] == pytest.approx(
        b.drive(0.2, 0, 0).wheel_rpm[0] * 10, rel=0.02)
    assert a.drive(0.2, 0, 0).wheel_rad_s == pytest.approx(b.drive(0.2, 0, 0).wheel_rad_s)


# --- partial failure ----------------------------------------------------------------

def test_a_failure_on_one_controller_takes_drive_off_the_other_too():
    """Two mecanum wheels driving while two are dead slews the robot unpredictably."""
    bus = FakeBus(fail_on={(2, reg.PID_VEL_CMD): 1})   # one transient failure
    base, _ = make_base(bus=bus)
    with pytest.raises(DriveError, match="controller 2"):
        base.drive(0.2, 0.0, 0.0)

    assert bus.writes()[-4:] == [
        w1(1, reg.PID_VEL_CMD, 0), w1(1, reg.PID_VEL_CMD2, 0),
        w1(2, reg.PID_VEL_CMD, 0), w1(2, reg.PID_VEL_CMD2, 0),
    ]


def test_a_failure_on_the_second_CHANNEL_of_one_board_also_stops_both():
    """set_velocities is two 0x06 frames: channel 1 can land while channel 2 fails."""
    bus = FakeBus(fail_on={(1, reg.PID_VEL_CMD2): 1})
    base, _ = make_base(bus=bus)
    with pytest.raises(DriveError, match="controller 1"):
        base.drive(0.2, 0.0, 0.0)
    assert bus.writes()[-4:] == [
        w1(1, reg.PID_VEL_CMD, 0), w1(1, reg.PID_VEL_CMD2, 0),
        w1(2, reg.PID_VEL_CMD, 0), w1(2, reg.PID_VEL_CMD2, 0),
    ]


def test_drive_error_keeps_the_original_exception_as_its_cause():
    bus = FakeBus(fail_on={(2, reg.PID_VEL_CMD): FAIL_FOREVER})
    base, _ = make_base(bus=bus)
    with pytest.raises(DriveError) as excinfo:
        base.drive(0.2, 0.0, 0.0)
    assert isinstance(excinfo.value.__cause__, MdrobotError)


def test_the_emergency_stop_falls_back_to_torque_off_when_stop_also_fails():
    """A controller that will not take a zero-velocity command still gets its drive cut.

    last_stop_ok stays True: torque-off landed, so the motor is no longer driven. It
    is a coast rather than a controlled stop, but drive really was removed.
    """
    bus = FakeBus(fail_on={(2, reg.PID_VEL_CMD): FAIL_FOREVER})
    base, _ = make_base(bus=bus)
    with pytest.raises(DriveError):
        base.drive(0.2, 0.0, 0.0)
    assert w1(2, reg.PID_PNT_TQ_OFF, 0x0101) in bus.writes()
    assert base.last_stop_ok is True


def test_last_stop_ok_goes_false_when_a_controller_cannot_be_stopped_at_all():
    bus = FakeBus(fail_on={(2, reg.PID_VEL_CMD): FAIL_FOREVER,
                           (2, reg.PID_PNT_TQ_OFF): FAIL_FOREVER})
    base, _ = make_base(bus=bus)
    with pytest.raises(DriveError):
        base.drive(0.2, 0.0, 0.0)
    assert base.last_stop_ok is False


def test_a_healthy_bus_leaves_last_stop_ok_true():
    base, _ = make_base()
    base.drive(0.1, 0.0, 0.0)
    assert base.last_stop_ok is True


def test_spin_one_also_stops_everything_on_failure():
    bus = FakeBus(fail_on={(2, reg.PID_VEL_CMD): 1})
    base, _ = make_base(bus=bus)
    with pytest.raises(DriveError, match="spin_one"):
        base.spin_one(2, 1, 30)
    assert w1(1, reg.PID_VEL_CMD, 0) in bus.writes()


# --- spin_one -----------------------------------------------------------------------

def test_spin_one_writes_a_single_channel():
    base, bus = make_base()
    base.spin_one(2, 2, 30)
    assert bus.writes() == [w1(2, reg.PID_VEL_CMD2, 30)]


def test_spin_one_clamps_to_max_motor_rpm():
    base, bus = make_base(make_config(limits={"max_motor_rpm": 50}))
    base.spin_one(1, 1, 9000)
    assert bus.writes() == [w1(1, reg.PID_VEL_CMD, 50)]


def test_spin_one_does_not_apply_the_wheel_sign():
    """Identification needs the RAW motor direction; sign is what it is measuring."""
    base, bus = make_base(make_config().with_signs([-1, -1, -1, -1]))
    base.spin_one(1, 1, 30)
    assert bus.writes() == [w1(1, reg.PID_VEL_CMD, 30)]


# --- drive_wheels -------------------------------------------------------------------

def test_drive_wheels_bypasses_the_kinematics_but_applies_sign_and_clamp():
    base, bus = make_base(make_config().with_signs([1, -1, 1, -1]))
    result = base.drive_wheels([30, 30, 30, 30])
    assert result.wheel_rpm == (30, -30, 30, -30)
    assert bus.writes() == [
        w1(1, reg.PID_VEL_CMD, 30), w1(1, reg.PID_VEL_CMD2, vel(-30)),
        w1(2, reg.PID_VEL_CMD, 30), w1(2, reg.PID_VEL_CMD2, vel(-30)),
    ]


def test_drive_wheels_rejects_a_wrong_length():
    base, _ = make_base()
    with pytest.raises(ValueError, match=f"{len(WHEEL_NAMES)} wheel speeds"):
        base.drive_wheels([30, 30])


def test_drive_wheels_reports_the_twist_those_speeds_imply():
    base, _ = make_base()
    result = base.drive_wheels([40, 40, 40, 40])
    vx, vy, wz = result.twist
    assert vx > 0 and vy == pytest.approx(0.0) and wz == pytest.approx(0.0)


# --- batched write ------------------------------------------------------------------

def test_batched_mode_sends_one_multi_register_frame_per_controller():
    cfg = make_config(runtime={"use_batched_velocity": True})
    base, bus = make_base(cfg)
    n = rpm_for(0.2, cfg)
    base.drive(0.2, 0.0, 0.0)
    assert bus.writes() == [
        wn(1, reg.PID_PNT_VEL_CMD, [vel(n), vel(n)]),
        wn(2, reg.PID_PNT_VEL_CMD, [vel(n), vel(n)]),
    ]


def test_batched_mode_halves_the_frame_count():
    plain, bus_a = make_base()
    batched, bus_b = make_base(make_config(runtime={"use_batched_velocity": True}))
    plain.drive(0.2, 0, 0)
    batched.drive(0.2, 0, 0)
    assert len(bus_a.writes()) == 4 and len(bus_b.writes()) == 2


# --- stop / torque off --------------------------------------------------------------

def test_stop_zeroes_every_channel():
    base, bus = make_base()
    base.stop()
    assert bus.writes() == [
        w1(1, reg.PID_VEL_CMD, 0), w1(1, reg.PID_VEL_CMD2, 0),
        w1(2, reg.PID_VEL_CMD, 0), w1(2, reg.PID_VEL_CMD2, 0),
    ]


def test_torque_off_frees_both_channels_on_both_controllers():
    base, bus = make_base()
    base.torque_off()
    assert bus.writes() == [
        w1(1, reg.PID_PNT_TQ_OFF, 0x0101),
        w1(2, reg.PID_PNT_TQ_OFF, 0x0101),
    ]


def test_torque_off_tries_every_controller_before_reporting_failure():
    """A failure on the first controller must not skip the second."""
    bus = FakeBus(fail_on={(1, reg.PID_PNT_TQ_OFF): FAIL_FOREVER})
    base, _ = make_base(bus=bus)
    with pytest.raises(DriveError, match="id 1"):
        base.torque_off()
    assert w1(2, reg.PID_PNT_TQ_OFF, 0x0101) in bus.writes()


# --- shutdown -----------------------------------------------------------------------

def test_shutdown_stops_twice_then_cuts_torque():
    base, bus = make_base()
    assert base.shutdown() is True
    velocity_writes = [f for f in bus.writes() if f[1] == 0x06
                       and ((f[2] << 8) | f[3]) in (reg.PID_VEL_CMD, reg.PID_VEL_CMD2)]
    assert len(velocity_writes) == 8          # two stops x four channels
    assert bus.writes()[-2:] == [w1(1, reg.PID_PNT_TQ_OFF, 0x0101),
                                 w1(2, reg.PID_PNT_TQ_OFF, 0x0101)]


def test_shutdown_retries_a_transient_failure():
    """A SIGINT mid-transaction leaves a stale tail on the wire; the retry clears it."""
    bus = FakeBus(fail_on={(1, reg.PID_VEL_CMD): 1})   # fail once, then recover
    base, _ = make_base(bus=bus)
    assert base.shutdown(attempts=3, delay=0.0) is True
    assert base.last_stop_ok is True


def test_shutdown_reports_failure_when_the_bus_is_really_gone():
    class DeadBus(FakeBus):
        def write(self, data):
            raise MdrobotError("bus is gone")

    base, bus = make_base(bus=DeadBus())
    assert base.shutdown(attempts=2, delay=0.0) is False
    assert base.last_stop_ok is False


# --- lifecycle ----------------------------------------------------------------------

def test_close_closes_the_shared_transport_exactly_once():
    """Every driver shares this port: closing per driver pulls it out from the others."""
    base, bus = make_base()
    base.close()
    base.close()
    assert bus.closed == 1


def test_context_manager_stops_torque_offs_and_closes():
    base, bus = make_base()
    with base:
        base.drive(0.2, 0.0, 0.0)
    assert bus.writes()[-2:] == [w1(1, reg.PID_PNT_TQ_OFF, 0x0101),
                                 w1(2, reg.PID_PNT_TQ_OFF, 0x0101)]
    assert bus.closed == 1


def test_the_port_is_closed_even_when_stopping_raises():
    class DeadBus(FakeBus):
        def write(self, data):
            raise MdrobotError("bus is gone")

    base, bus = make_base(bus=DeadBus())
    with base:
        pass
    assert bus.closed == 1


# --- feedback -----------------------------------------------------------------------

def monitor_words(speed1: int, speed2: int) -> list[int]:
    """A PID_PNT_MONITOR(216) payload: [spd1, pos1L, pos1H, spd2, pos2L, pos2H, status]."""
    return [speed1 & 0xFFFF, 0, 0, speed2 & 0xFFFF, 0, 0, 0]


def test_read_wheel_rpm_maps_channels_to_wheels():
    bus = FakeBus({(1, reg.PID_PNT_MONITOR): monitor_words(10, 20),
                   (2, reg.PID_PNT_MONITOR): monitor_words(30, 40)})
    base, _ = make_base(bus=bus)
    assert base.read_wheel_rpm() == (10, 20, 30, 40)


def test_read_wheel_rpm_undoes_the_wheel_sign():
    """Feedback comes back in the same "+ drives forward" frame the commands use."""
    bus = FakeBus({(1, reg.PID_PNT_MONITOR): monitor_words(10, -20),
                   (2, reg.PID_PNT_MONITOR): monitor_words(30, -40)})
    base, _ = make_base(make_config().with_signs([1, -1, 1, -1]), bus=bus)
    assert base.read_wheel_rpm() == (10, 20, 30, 40)


def test_read_wheel_rad_s_undoes_the_gear_ratio():
    bus = FakeBus({(1, reg.PID_PNT_MONITOR): monitor_words(600, 600),
                   (2, reg.PID_PNT_MONITOR): monitor_words(600, 600)})
    cfg = make_config(geometry={"wheel_radius": R, "track": TRACK,
                                "wheelbase": WHEELBASE, "gear_ratio": 10.0})
    base, _ = make_base(cfg, bus=bus)
    assert base.read_wheel_rad_s() == pytest.approx((2 * math.pi,) * 4)


def test_odometry_twist_recovers_a_pure_forward_motion():
    bus = FakeBus({(1, reg.PID_PNT_MONITOR): monitor_words(60, 60),
                   (2, reg.PID_PNT_MONITOR): monitor_words(60, 60)})
    base, _ = make_base(bus=bus)
    vx, vy, wz = base.odometry_twist()
    assert vx == pytest.approx(2 * math.pi * R) and vy == pytest.approx(0.0)
    assert wz == pytest.approx(0.0)


# --- preflight ----------------------------------------------------------------------

def preflight_registers(enc_ppr=0, limit_sw=0, max_rpm=1800):
    registers = {}
    for sid in (1, 2):
        registers[(sid, reg.PID_ENC_PPR)] = [enc_ppr]
        registers[(sid, reg.PID_USE_LIMIT_SW)] = [limit_sw]
        registers[(sid, reg.PID_USE_LIMIT_SW2)] = [limit_sw]
        registers[(sid, reg.PID_MAX_RPM)] = [max_rpm]
    return registers


def test_preflight_writes_nothing_when_everything_is_already_correct():
    bus = FakeBus(preflight_registers())
    base, _ = make_base(make_config(runtime={"controller_ramp_s": None}), bus=bus)
    log = base.preflight()
    assert bus.writes() == []
    assert any("already 0" in line for line in log)


def test_preflight_fixes_a_wrong_limit_switch_setting_and_reads_it_back():
    bus = FakeBus(preflight_registers(limit_sw=1))
    base, _ = make_base(make_config(runtime={"controller_ramp_s": None}), bus=bus)
    base.preflight()
    assert w1(1, reg.PID_USE_LIMIT_SW, 0) in bus.writes()
    assert w1(2, reg.PID_USE_LIMIT_SW2, 0) in bus.writes()


def test_preflight_leaves_the_limit_switch_alone_when_configured_to():
    bus = FakeBus(preflight_registers(limit_sw=1))
    base, _ = make_base(make_config(runtime={"use_limit_sw": -1,
                                             "controller_ramp_s": None}), bus=bus)
    base.preflight()
    assert bus.writes() == []


def test_preflight_reports_but_does_not_change_encoder_mode():
    """Writing ENC_PPR can reinitialise the controller — never do it unasked."""
    bus = FakeBus(preflight_registers(enc_ppr=1000))
    base, _ = make_base(make_config(runtime={"controller_ramp_s": None}), bus=bus)
    log = base.preflight()
    assert not any(((f[2] << 8) | f[3]) == reg.PID_ENC_PPR for f in bus.writes())
    assert any("encoder mode" in line for line in log)


def test_preflight_warns_when_the_config_exceeds_the_controller_cap():
    bus = FakeBus(preflight_registers(max_rpm=50))
    base, _ = make_base(make_config(limits={"max_motor_rpm": 100},
                                    runtime={"controller_ramp_s": None}), bus=bus)
    assert any("EXCEEDS THE CONTROLLER CAP" in line for line in base.preflight())


def test_preflight_sets_the_controller_ramps_on_all_four_channels():
    bus = FakeBus(preflight_registers())
    base, _ = make_base(make_config(runtime={"controller_ramp_s": 0.2}), bus=bus)
    base.preflight()
    ramp_pids = {reg.PID_SLOW_START1, reg.PID_SLOW_START2,
                 reg.PID_SLOW_DOWN1, reg.PID_SLOW_DOWN2}
    written = {((f[2] << 8) | f[3]) for f in bus.writes()}
    assert ramp_pids <= written


# --- enable -------------------------------------------------------------------------

def test_enable_arms_every_controller():
    base, bus = make_base()
    base.enable(settle=0)
    for sid in (1, 2):
        assert w1(sid, reg.PID_UI_COM, 1) in bus.writes()
        assert w1(sid, reg.PID_START_STOP, 1) in bus.writes()


def test_enable_waits_for_the_controller_to_settle():
    """Commanding immediately after arming looks like a dead channel for ~2 s."""
    base, _ = make_base()
    started = time.monotonic()
    base.enable(settle=0.2)
    assert time.monotonic() - started >= 0.2


def test_enable_can_skip_the_settle_when_the_caller_does_its_own_waiting():
    base, _ = make_base()
    started = time.monotonic()
    base.enable(settle=0)
    assert time.monotonic() - started < 0.2


def test_the_watchdog_constant_matches_what_was_measured_on_hardware():
    """1.5 s gaps survived and 2.0 s gaps cut the motor on PNT50 DL=19."""
    assert COMMAND_WATCHDOG_S == 2.0


def test_borrow_single_shares_the_clients_slave_id():
    base, _ = make_base()
    borrowed = borrow_single(base.driver(2))
    assert borrowed.client is base.driver(2).client
    assert borrowed.client.slave_id == 2
