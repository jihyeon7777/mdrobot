"""SingleMotorDriver / DualMotorDriver unit tests (fake device, no hardware).

`FakeDevice` looks at the function code of each write request and produces an
echo/ack response like a real controller would, so both the frames the driver
sends and the read decoding can be verified.
"""

import pytest

from mdrobot import registers as reg
from mdrobot.crc import append_crc
from mdrobot.device import DualMotorDriver, SingleMotorDriver
from mdrobot.exceptions import MdrobotError
from mdrobot.protocol import ModbusClient


class FakeDevice:
    """Fake transport that auto-generates a response per function code.

    - 0x03 read : take words from registers[pid] (0 if missing).
    - 0x06 write: echo the request (8 bytes).
    - 0x10 write: start-address/count echo (8 bytes).
    Write frames are recorded as full request bytes in `self.frames`.
    """

    def __init__(self, registers=None) -> None:
        self.registers = {pid: list(words) for pid, words in (registers or {}).items()}
        self.frames: list[bytes] = []
        self._rx = bytearray()

    def write(self, data: bytes) -> int:
        data = bytes(data)
        self.frames.append(data)
        self._rx += self._respond(data)
        return len(data)

    def read(self, size: int) -> bytes:
        chunk = bytes(self._rx[:size])
        del self._rx[:size]
        return chunk

    def flush_input(self) -> None:
        pass

    def _respond(self, req: bytes) -> bytes:
        slave, func = req[0], req[1]
        pid = (req[2] << 8) | req[3]
        if func == 0x03:
            count = (req[4] << 8) | req[5]
            words = self.registers.get(pid, [])
            words = (words + [0] * count)[:count]
            body = bytearray((slave, 0x03, 2 * count))
            for w in words:
                body.append((w >> 8) & 0xFF)
                body.append(w & 0xFF)
            return append_crc(bytes(body))
        if func == 0x06:
            return req  # echo
        if func == 0x10:
            return append_crc(req[:6])
        raise AssertionError(f"unexpected func 0x{func:02x}")


def make_single(registers=None):
    dev = FakeDevice(registers)
    return SingleMotorDriver(ModbusClient(dev, slave_id=1)), dev


def make_dual(registers=None):
    dev = FakeDevice(registers)
    return DualMotorDriver(ModbusClient(dev, slave_id=1)), dev


def w1(pid, word):
    from mdrobot import frame
    return frame.build_write_single_request(1, pid, word)


# --- shared / enable -----------------------------------------------------------------

def test_enable_writes_uicom_comtar_then_start():
    drv, dev = make_single()
    drv.enable()
    assert dev.frames == [
        w1(reg.PID_UI_COM, 1),
        w1(reg.PID_COM_TAR_SPEED, 0),
        w1(reg.PID_START_STOP, 1),
    ]


def test_disable_clears_start_stop():
    drv, dev = make_single()
    drv.disable()
    assert dev.frames == [w1(reg.PID_START_STOP, 0)]


def test_reset_alarm_uses_command_gateway():
    drv, dev = make_single()
    drv.reset_alarm()
    assert dev.frames == [w1(reg.PID_COMMAND, reg.CMD_ALARM_RESET)]


def test_ping_true_false():
    drv, _ = make_single({reg.PID_VERSION: [0x2D]})
    assert drv.ping() is True
    drv2, _ = make_single()  # version reply 0 -> communication itself still succeeds
    assert drv2.ping() is True


# --- SingleMotorDriver ---------------------------------------------------------------

def test_single_set_velocity_positive():
    drv, dev = make_single()
    drv.set_velocity(100)
    assert dev.frames == [w1(reg.PID_VEL_CMD, 100)]


def test_single_set_velocity_negative_twos_complement():
    drv, dev = make_single()
    drv.set_velocity(-100)
    assert dev.frames == [w1(reg.PID_VEL_CMD, 0xFF9C)]


def test_single_stop_brake_torque_off():
    drv, dev = make_single()
    drv.stop()
    drv.brake()
    drv.torque_off()
    assert dev.frames == [
        w1(reg.PID_VEL_CMD, 0),
        w1(reg.PID_BRAKE, 0),
        w1(reg.PID_TQ_OFF, 0),
    ]


def test_single_get_speed_signed():
    drv, _ = make_single({reg.PID_INT_RPM_DATA: [0xFF9C]})  # -100
    assert drv.get_speed() == -100


def test_single_get_voltage_and_current():
    drv, _ = make_single({reg.PID_VOLT_IN: [239], reg.PID_TQ_DATA: [12]})
    assert drv.get_voltage() == pytest.approx(23.9)
    assert drv.get_current() == pytest.approx(1.2)


def test_single_get_position_long():
    drv, _ = make_single({reg.PID_POSI_DATA: [0x5678, 0x1234]})  # low word first
    assert drv.get_position() == 0x12345678


def test_single_read_monitor():
    drv, _ = make_single({reg.PID_MONITOR: [0x0064, 0x000C, 0x0000, 0x0005, 0x0000, 0x0000]})
    mon = drv.read_monitor()
    assert mon.speed_rpm == 100
    assert mon.position == 5


def wN(pid, words):
    from mdrobot import frame
    return frame.build_write_multiple_request(1, pid, words)


def test_single_move_to_encodes_position_low_word_first_plus_speed():
    drv, dev = make_single()
    drv.move_to(0x12345678, speed=50)
    # position low word first [0x5678, 0x1234] + speed word
    assert dev.frames == [wN(reg.PID_POSI_VEL_CMD, [0x5678, 0x1234, 50])]


def test_single_move_to_small_target():
    drv, dev = make_single()
    drv.move_to(80, speed=50)
    assert dev.frames == [wN(reg.PID_POSI_VEL_CMD, [80, 0, 50])]


def test_single_move_by_negative_delta():
    drv, dev = make_single()
    drv.move_by(-2, speed=40)
    # -2 -> 0xFFFFFFFE -> low 0xFFFE, high 0xFFFF
    assert dev.frames == [wN(reg.PID_INC_POSI_VEL_CMD, [0xFFFE, 0xFFFF, 40])]


def test_single_get_in_position():
    drv, _ = make_single({reg.PID_IN_POSITION_OK: [1]})
    assert drv.get_in_position() is True
    drv2, _ = make_single({reg.PID_IN_POSITION_OK: [0]})
    assert drv2.get_in_position() is False


# --- encoder velocity feedback -------------------------------------------------------
# `settle=0.0` keeps the tests fast; on hardware the default delay covers the controller
# reinitialising after the write.

def test_get_encoder_ppr_reads_register():
    drv, _ = make_single({reg.PID_ENC_PPR: [1000]})
    assert drv.get_encoder_ppr() == 1000


def test_set_encoder_ppr_writes_then_verifies():
    # the fake keeps registers static, so seed the value the write is expected to land on
    drv, dev = make_single({reg.PID_ENC_PPR: [1000]})
    drv.set_encoder_ppr(1000, settle=0.0)
    assert dev.frames[0] == w1(reg.PID_ENC_PPR, 1000)


def test_disable_encoder_writes_zero():
    drv, dev = make_single()
    drv.disable_encoder(settle=0.0)
    assert dev.frames[0] == w1(reg.PID_ENC_PPR, 0)


def test_set_encoder_ppr_raises_when_not_applied():
    drv, _ = make_single()  # reads back 0, not the 1000 that was written
    with pytest.raises(MdrobotError):
        drv.set_encoder_ppr(1000, settle=0.0)


def test_set_encoder_ppr_can_skip_verification():
    drv, dev = make_single()
    drv.set_encoder_ppr(1000, settle=0.0, verify=False)
    assert dev.frames == [w1(reg.PID_ENC_PPR, 1000)]


def test_set_encoder_ppr_rejects_out_of_range():
    drv, dev = make_single()
    with pytest.raises(ValueError):
        drv.set_encoder_ppr(70000)
    assert dev.frames == []


# --- encoder position source (USE_EPOSI) ---------------------------------------------
# Enabling first reads ENC_PPR (the ENC_PPR=0 guard), so the 46 write is frames[1].

def test_get_use_encoder_position_reads_register():
    drv, _ = make_single({reg.PID_USE_EPOSI: [1]})
    assert drv.get_use_encoder_position() is True
    drv2, _ = make_single()
    assert drv2.get_use_encoder_position() is False


def test_set_use_encoder_position_writes_then_verifies():
    # the fake keeps registers static, so seed the value the write is expected to land on
    drv, dev = make_single({reg.PID_USE_EPOSI: [1], reg.PID_ENC_PPR: [1000]})
    drv.set_use_encoder_position(True, settle=0.0)
    assert dev.frames[1] == w1(reg.PID_USE_EPOSI, 1)


def test_set_use_encoder_position_false_writes_zero():
    drv, dev = make_single()  # reads back 0 = the written value; no ENC_PPR guard read
    drv.set_use_encoder_position(False, settle=0.0)
    assert dev.frames[0] == w1(reg.PID_USE_EPOSI, 0)


def test_set_use_encoder_position_raises_when_not_applied():
    # read-back stays 0 although 1 was written (e.g. firmware ignoring the register)
    drv, _ = make_single({reg.PID_ENC_PPR: [1000]})
    with pytest.raises(MdrobotError):
        drv.set_use_encoder_position(True, settle=0.0)


def test_set_use_encoder_position_can_skip_verification():
    drv, dev = make_single({reg.PID_ENC_PPR: [1000]})
    drv.set_use_encoder_position(True, settle=0.0, verify=False)
    assert dev.frames[1] == w1(reg.PID_USE_EPOSI, 1)
    assert len(dev.frames) == 2  # guard read + write, no read-back


def test_set_use_encoder_position_requires_encoder_ppr():
    drv, dev = make_single()  # ENC_PPR reads 0 -> refused before any write
    with pytest.raises(MdrobotError):
        drv.set_use_encoder_position(True, settle=0.0)
    assert w1(reg.PID_USE_EPOSI, 1) not in dev.frames


# --- DualMotorDriver -----------------------------------------------------------------

def test_dual_set_velocities_uses_two_legacy_pids():
    drv, dev = make_dual()
    drv.set_velocities(40, -40)
    assert dev.frames == [w1(reg.PID_VEL_CMD, 40), w1(reg.PID_VEL_CMD2, 0xFFD8)]


def test_dual_set_velocity_channel():
    drv, dev = make_dual()
    drv.set_velocity(1, 30)
    drv.set_velocity(2, 30)
    assert dev.frames == [w1(reg.PID_VEL_CMD, 30), w1(reg.PID_VEL_CMD2, 30)]


def test_dual_set_velocity_bad_channel():
    drv, _ = make_dual()
    with pytest.raises(ValueError):
        drv.set_velocity(3, 10)


def test_dual_brake_and_torque_off_flags():
    drv, dev = make_dual()
    drv.brake_both()
    drv.brake(1)
    drv.brake(2)
    drv.torque_off_both()
    drv.torque_off(1)
    drv.torque_off(2)
    assert dev.frames == [
        w1(reg.PID_PNT_BRAKE, 0x0101),
        w1(reg.PID_PNT_BRAKE, 0x0001),
        w1(reg.PID_PNT_BRAKE, 0x0100),
        w1(reg.PID_PNT_TQ_OFF, 0x0101),
        w1(reg.PID_PNT_TQ_OFF, 0x0001),
        w1(reg.PID_PNT_TQ_OFF, 0x0100),
    ]


def test_dual_stop_sends_both_zero():
    drv, dev = make_dual()
    drv.stop()
    assert dev.frames == [w1(reg.PID_VEL_CMD, 0), w1(reg.PID_VEL_CMD2, 0)]


def test_dual_read_monitor_and_get_speed():
    regs = {reg.PID_PNT_MONITOR: [0x0064, 0x0005, 0x0000, 0xFFCE, 0xFFFF, 0xFFFF, 0x0000]}
    drv, _ = make_dual(regs)
    dm = drv.read_monitor()
    assert dm.motor1.speed_rpm == 100
    assert dm.motor2.speed_rpm == -50
    assert drv.get_speed(1) == 100
    assert drv.get_speed(2) == -50


def test_dual_read_main_data_has_current():
    regs = {reg.PID_PNT_MAIN_DATA: [0x0064, 0x000C, 0x0000, 0x0000, 0x0000, 0x0006, 0x0000, 0x0000, 0x0000]}
    drv, _ = make_dual(regs)
    dm = drv.read_main_data()
    assert dm.motor1.current_a == pytest.approx(1.2)
    assert dm.motor2.current_a == pytest.approx(0.6)


def test_dual_move_to_both_matches_golden_vector():
    # pos1=0x12345678 spd1=0x9ABC pos2=0x34567890 spd2=0x0123
    # -> words [0x5678, 0x1234, 0x9ABC, 0x7890, 0x3456, 0x0123]
    drv, dev = make_dual()
    drv.move_to_both(0x12345678, 0x34567890, speed1=0x9ABC, speed2=0x0123)
    assert dev.frames == [wN(reg.PID_PNT_POSI_VEL_CMD, [0x5678, 0x1234, 0x9ABC, 0x7890, 0x3456, 0x0123])]


def test_dual_move_to_both_default_speed2_follows_speed1():
    drv, dev = make_dual()
    drv.move_to_both(50, 80, speed1=60)
    assert dev.frames == [wN(reg.PID_PNT_POSI_VEL_CMD, [50, 0, 60, 80, 0, 60])]


def test_dual_move_by_both_negative_delta():
    drv, dev = make_dual()
    drv.move_by_both(20, -20, speed1=50)
    # -20 -> 0xFFFFFFEC -> low 0xFFEC, high 0xFFFF
    assert dev.frames == [wN(reg.PID_PNT_INC_POSI_VEL_CMD, [20, 0, 50, 0xFFEC, 0xFFFF, 50])]


def test_dual_get_positions_and_reset():
    regs = {reg.PID_PNT_MONITOR: [0x0000, 0x0032, 0x0000, 0x0000, 0x0050, 0x0000, 0x0000]}
    drv, dev = make_dual(regs)
    assert drv.get_positions() == (50, 80)
    assert drv.get_position(2) == 80
    drv.reset_position()
    # get_positions/get_position also send read requests, so check only the last write.
    assert dev.frames[-1] == w1(reg.PID_POSI_RESET, 0)


# --- slow-start / slow-down (NOT hardware-verified) ---------------------------------

def test_single_set_slow_start_full_scale():
    drv, dev = make_single()
    drv.set_slow_start(15)  # default full scale 15 s -> raw 1023
    assert dev.frames == [w1(reg.PID_SLOW_START, 1023)]


def test_single_set_slow_down_zero():
    drv, dev = make_single()
    drv.set_slow_down(0)
    assert dev.frames == [w1(reg.PID_SLOW_DOWN, 0)]


def test_single_get_slow_start_seconds():
    drv, _ = make_single({reg.PID_SLOW_START: [1023]})
    assert drv.get_slow_start() == pytest.approx(15.0)


def test_single_position_slow_start_down():
    drv, dev = make_single()
    drv.set_position_slow_start(15)
    drv.set_position_slow_down(15)
    assert dev.frames == [w1(reg.PID_POSI_SS, 1023), w1(reg.PID_POSI_SD, 1023)]


def test_single_clear_slow_uses_command_gateway():
    drv, dev = make_single()
    drv.clear_slow_start()
    drv.clear_slow_down()
    assert dev.frames == [
        w1(reg.PID_COMMAND, reg.CMD_SLOW_START_OFF),
        w1(reg.PID_COMMAND, reg.CMD_SLOW_DOWN_OFF),
    ]


def test_dual_set_slow_start_per_channel():
    drv, dev = make_dual()
    drv.set_slow_start(1, 15)
    drv.set_slow_start(2, 0)
    assert dev.frames == [w1(reg.PID_SLOW_START1, 1023), w1(reg.PID_SLOW_START2, 0)]


def test_dual_set_slow_down_per_channel():
    drv, dev = make_dual()
    drv.set_slow_down(1, 7.5)
    assert dev.frames == [w1(reg.PID_SLOW_DOWN1, round(7.5 / 15 * 1023))]


def test_dual_slow_bad_channel():
    drv, _ = make_dual()
    with pytest.raises(ValueError):
        drv.set_slow_start(3, 10)


def test_dual_get_slow_down_seconds():
    drv, _ = make_dual({reg.PID_SLOW_DOWN2: [1023]})
    assert drv.get_slow_down(2) == pytest.approx(15.0)


def test_dual_position_slow_per_channel():
    drv, dev = make_dual()
    drv.set_position_slow_start(1, 15)
    drv.set_position_slow_down(2, 15)
    assert dev.frames == [w1(reg.PID_POSI_SS1, 1023), w1(reg.PID_POSI_SD2, 1023)]
