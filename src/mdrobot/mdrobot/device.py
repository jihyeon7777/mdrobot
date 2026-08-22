"""High-level device drivers: SingleMotorDriver / DualMotorDriver.

This layer puts an intuitive motor API on top of `ModbusClient` (raw primitives).
Raw access is always available via `self.client`.

Drive model (verified on hardware):

- A serial velocity command works only after `PID_UI_COM(78)=1` (serial only) and
  `PID_START_STOP(100)=1` (run-latch arm). Without these two, the command echoes
  fine but the motor reference stays at 0. `enable()` performs both writes.
- Velocity source: motor 1 = `PID_VEL_CMD(130)`, motor 2 (dual) = `PID_VEL_CMD2(131)`.
  signed rpm; + = increasing position (CCW), - = decreasing position (CW).
- Some dual-channel controllers take ~1 s to start turning after a command; sending
  0 immediately misses the motion.

Safety: `set_velocity*` turns the motor immediately. Call `enable()` first, and on
exit call `stop()` then `torque_off*()` or `disable()`.
"""

from __future__ import annotations

import time

from . import registers as reg
from .codec import int16, split_i32_low_word_first, word_from_int16
from .constants import DEFAULT_BAUDRATE, DEFAULT_SLAVE_ID, DEFAULT_TIMEOUT
from .exceptions import MdrobotError
from .protocol import ModbusClient
from .status import (
    DualMonitor,
    Monitor,
    StatusBits,
    decode_monitor,
    decode_pnt_main_data,
    decode_pnt_monitor,
)
from .units import SLOW_DEFAULT_FULL_SCALE_S, slow_raw_to_seconds, slow_seconds_to_raw

# PID_UI_COM(78) value: serial-only control (ignore CTRL I/O).
UI_COM_SERIAL = 1
# PID_START_STOP(100) values.
START = 1
STOP = 0
# Writing PID_ENC_PPR(156) can reinitialise the controller; wait this long before reading back.
ENCODER_SETTLE_S = 2.5
# PID_USE_EPOSI(46) applies immediately without reinitialising, but the first read-back can
# still return the OLD value; wait this long before each read-back attempt.
EPOSI_SETTLE_S = 0.2
# How many read-back attempts set_use_encoder_position makes before giving up.
EPOSI_VERIFY_TRIES = 5


class _DriverBase:
    """Shared single/dual: connection management, version/voltage/status, enable/disable, alarm reset."""

    def __init__(self, client: ModbusClient) -> None:
        self.client = client

    @classmethod
    def open(
        cls,
        port: str | None = None,
        baudrate: int = DEFAULT_BAUDRATE,
        *,
        slave_id: int = DEFAULT_SLAVE_ID,
        timeout: float = DEFAULT_TIMEOUT,
    ):
        """Open a port and build the driver (convenience constructor). Close with `close()`.

        With `port` omitted, the MDROBOT_PORT environment variable is used
        (see `mdrobot.resolve_port`); if neither is set, ValueError is raised.
        """
        from .transport import SerialTransport, resolve_port

        transport = SerialTransport(resolve_port(port), baudrate, timeout=timeout)
        return cls(ModbusClient(transport, slave_id=slave_id))

    def close(self) -> None:
        """Close the underlying transport (if any)."""
        close = getattr(self.client.transport, "close", None)
        if callable(close):
            close()

    def __enter__(self):
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    # --- shared reads ------------------------------------------------------------------
    def get_version(self) -> int:
        """firmware/protocol version code (DL byte)."""
        return self.client.read_register(reg.PID_VERSION) & 0xFF

    def get_voltage(self) -> float:
        """Supply voltage (V); raw is in 0.1 V units."""
        return self.client.read_register(reg.PID_VOLT_IN) / 10.0

    def get_status(self) -> StatusBits:
        """status1 bits from `PID_CTRL_STATUS(34)`."""
        return StatusBits.from_byte(self.client.read_register(reg.PID_CTRL_STATUS) & 0xFF)

    def ping(self) -> bool:
        """Check whether communication works by reading the version."""
        try:
            self.get_version()
            return True
        except MdrobotError:
            return False

    # --- enable / safety ---------------------------------------------------------------
    def enable(self) -> None:
        """Serial-only control + run-latch arm. Must be called before velocity commands.

        Verified on hardware: without these writes (`PID_UI_COM=1`, `PID_START_STOP=1`)
        `set_velocity` echoes but the motor does not turn. The speed source is set to 0
        first so arming does not spin from a leftover `COM_TAR_SPEED` (velocity drive
        uses VEL_CMD, so this is harmless). Position control only needs UI_COM=1, but
        calling enable() is fine too.
        """
        self.client.write_register(reg.PID_UI_COM, UI_COM_SERIAL)
        self.client.write_register(reg.PID_COM_TAR_SPEED, 0)
        self.client.write_register(reg.PID_START_STOP, START)

    def disable(self) -> None:
        """Release the run-latch (START_STOP=0); the motor reference is cut."""
        self.client.write_register(reg.PID_START_STOP, STOP)

    def reset_alarm(self) -> None:
        """Reset the alarm (`CMD_ALARM_RESET`)."""
        self.client.command(reg.CMD_ALARM_RESET)

    # --- acceleration / deceleration: slow-start / slow-down ---------------------------
    # SPEED slow-start/down hardware-verified 2026-06-22 (Phase 12, MD400 + PNT50):
    # a raw value 0-1023 maps to 0..PID_MAX_SS_TIME seconds (full scale 15 s on tested
    # devices). POSITION slow and the CMD_*_OFF clears are still protocol-doc-based
    # (not separately hardware-verified). Subclasses expose the per-channel / single
    # setters and getters; clearing is shared (global CMDs).
    def _set_slow(self, pid: int, seconds: float, full_scale_s: float) -> None:
        self.client.write_register(pid, slow_seconds_to_raw(seconds, full_scale_s))

    def _get_slow(self, pid: int, full_scale_s: float) -> float:
        return slow_raw_to_seconds(self.client.read_register(pid), full_scale_s)

    def clear_slow_start(self) -> None:
        """Erase the speed slow-start value (`CMD_SLOW_START_OFF`). NOT hardware-verified."""
        self.client.command(reg.CMD_SLOW_START_OFF)

    def clear_slow_down(self) -> None:
        """Erase the speed slow-down value (`CMD_SLOW_DOWN_OFF`). NOT hardware-verified."""
        self.client.command(reg.CMD_SLOW_DOWN_OFF)

    def clear_position_slow_start(self) -> None:
        """Stop using the position slow-start parameter (`CMD_POSI_SS_OFF`). NOT hardware-verified."""
        self.client.command(reg.CMD_POSI_SS_OFF)

    def clear_position_slow_down(self) -> None:
        """Stop using the position slow-down parameter (`CMD_POSI_SD_OFF`). NOT hardware-verified."""
        self.client.command(reg.CMD_POSI_SD_OFF)


class SingleMotorDriver(_DriverBase):
    """Single-channel motor driver."""

    def set_velocity(self, rpm: int) -> None:
        """WARNING: turns the motor immediately. Requires `enable()` first.

        rpm is signed. Verified on hardware: + = increasing position (CCW), - = CW.
        """
        self.client.write_register(reg.PID_VEL_CMD, word_from_int16(rpm))

    def stop(self) -> None:
        """Decelerate to a stop with a zero-speed command (closed loop)."""
        self.client.write_register(reg.PID_VEL_CMD, 0)

    def brake(self) -> None:
        """WARNING: apply the electric brake (`PID_BRAKE`)."""
        self.client.write_register(reg.PID_BRAKE, 0)

    def torque_off(self) -> None:
        """Put the motor in free state (`PID_TQ_OFF`): output cut, coasts to a stop."""
        self.client.write_register(reg.PID_TQ_OFF, 0)

    def reset_position(self) -> None:
        """Reset the position count to 0 (`PID_POSI_RESET`); the position reference is lost."""
        self.client.write_register(reg.PID_POSI_RESET, 0)

    def get_speed(self) -> int:
        """Measured speed (signed rpm)."""
        return int16(self.client.read_register(reg.PID_INT_RPM_DATA))

    def get_current(self) -> float:
        """Current (A); raw is in 0.1 A units. Sign to be reconfirmed under load."""
        return self.client.read_register(reg.PID_TQ_DATA) / 10.0

    def get_position(self) -> int:
        """Position count (signed long)."""
        return self.client.read_long(reg.PID_POSI_DATA)

    def read_monitor(self) -> Monitor:
        """One `PID_MONITOR(196)` read: speed/current/output/position/status."""
        return decode_monitor(self.client.read_registers(reg.PID_MONITOR, 6))

    # --- position control (verified: needs only UI_COM=1, no START_STOP arm) -----------
    def _write_posi_vel(self, pid: int, position: int, speed: int) -> None:
        """6-byte command: position (long, low word first) + max speed (word)."""
        low, high = split_i32_low_word_first(position)
        self.client.write_registers(pid, [low, high, speed & 0xFFFF])

    def move_to(self, position: int, speed: int = 100) -> None:
        """WARNING: moves to an absolute position immediately. Requires `enable()` first.

        position is in counts, speed is the max speed (positive rpm). Verified on
        hardware: stops on arrival and `get_in_position()` becomes True. A + target =
        increasing-position direction.
        """
        self._write_posi_vel(reg.PID_POSI_VEL_CMD, position, speed)

    def move_by(self, delta: int, speed: int = 100) -> None:
        """WARNING: relative move from the current position (`PID_INC_POSI_VEL_CMD`). Requires `enable()` first."""
        self._write_posi_vel(reg.PID_INC_POSI_VEL_CMD, delta, speed)

    def get_in_position(self) -> bool:
        """Whether the position target has been reached (`PID_IN_POSITION_OK`)."""
        return bool(self.client.read_register(reg.PID_IN_POSITION_OK))

    def wait_in_position(self, timeout: float = 10.0, poll: float = 0.1) -> bool:
        """Wait until `get_in_position()` is True; return False on timeout."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.get_in_position():
                return True
            time.sleep(poll)
        return False

    # --- encoder velocity feedback -----------------------------------------------------
    # Hardware-verified 2026-08-04 (2x MD400 v8.6, 1000 PPR encoders wired, one bus):
    # an attached encoder feeds the VELOCITY closed loop only. Reported position stays on
    # the hall counter (counts/rev = 3 x poles) whether the encoder is on or off, so
    # `counts_per_rev` for odometry does NOT change when you enable the encoder.
    def get_encoder_ppr(self) -> int:
        """Encoder pulses-per-rev setting (`PID_ENC_PPR`); 0 = encoder off (hall closed-loop)."""
        return self.client.read_register(reg.PID_ENC_PPR)

    def set_encoder_ppr(self, ppr: int, *, settle: float = ENCODER_SETTLE_S,
                        verify: bool = True) -> None:
        """Use an attached encoder for velocity feedback; `ppr` is its rated pulses-per-rev.

        WARNING: the controller trusts this number as one revolution, so a value that does
        not match the encoder scales the real speed by (configured / actual). A value that
        is too LARGE therefore makes the motor turn faster than commanded while the
        reported rpm still looks correct - there is no software symptom. Pass the value
        printed on the encoder, and when unsure start low (too small only runs slow).

        The encoder must be physically wired: with a nonzero PPR and no encoder signal the
        controller trips `ENC_FAIL` shortly after the motor starts. Writing this register
        can reinitialise the controller, so the write response may be lost - that is why
        the write is followed by a settle delay and a read-back (`verify`). The setting is
        stored in EEPROM and survives a power cycle.
        """
        if not 0 <= ppr <= 0xFFFF:
            raise ValueError(f"encoder PPR must be 0..65535, got {ppr}")
        try:
            self.client.write_register(reg.PID_ENC_PPR, ppr)
        except MdrobotError:
            if not verify:
                raise
            # Reinitialisation can swallow the response; the read-back below decides.
        time.sleep(settle)
        if verify:
            readback = self._read_encoder_ppr_retrying()
            if readback != ppr:
                raise MdrobotError(
                    f"PID_ENC_PPR not applied: wrote {ppr}, read back {readback}")

    def disable_encoder(self, *, settle: float = ENCODER_SETTLE_S, verify: bool = True) -> None:
        """Turn the encoder off (`PID_ENC_PPR = 0`) and run hall closed-loop instead."""
        self.set_encoder_ppr(0, settle=settle, verify=verify)

    def _read_encoder_ppr_retrying(self, tries: int = 4, delay: float = 0.3) -> int:
        """Read `PID_ENC_PPR`, retrying while the controller finishes reinitialising."""
        for attempt in range(tries):
            try:
                return self.get_encoder_ppr()
            except MdrobotError:
                if attempt == tries - 1:
                    raise
                time.sleep(delay)
        raise AssertionError("unreachable")  # pragma: no cover

    # --- encoder position source (USE_EPOSI) -------------------------------------------
    # Hardware-verified 2026-08-22 (2x MD400 v8.6, 1000 PPR encoders). PID_USE_EPOSI(46)
    # comes from the RS485 spec V6.55; it is absent from the 2021 Modbus protocol PDF.
    def get_use_encoder_position(self) -> bool:
        """Whether reported position and position control use the encoder (`PID_USE_EPOSI`)."""
        return bool(self.client.read_register(reg.PID_USE_EPOSI))

    def set_use_encoder_position(self, enabled: bool, *, settle: float = EPOSI_SETTLE_S,
                                 verify: bool = True) -> None:
        """Switch the position source between the hall counter and an attached encoder.

        With True, reported position (`get_position`, the monitor) and position-control
        targets (`move_to` / `move_by`) are in ENCODER counts: counts/rev = 4 x the
        `ENC_PPR` setting (quadrature counts - a 1000 PPR encoder gives 4000 counts/rev,
        0.09 deg resolution). With False (the factory default) they are hall counts
        (3 x poles per rev). The switch takes effect immediately, is stored in EEPROM
        (survives a power cycle), and updates `PID_POS_SEN_TYPE(26)` automatically in
        both directions. Switch only while the motor is stopped, and call
        `reset_position()` afterwards so the counter reference is unambiguous.

        WARNING (verified on MD400 v8.6):

        - The physical direction convention flips versus hall mode: with the encoder
          source, + commands and increasing position turned the verified motors CW,
          where hall-mode + is CCW. Remap signs in any layer that assumes the hall
          convention (odometry, `counts_per_rev` users, direction logic).
        - The `IN_POSITION_OK` arrival window is in counts, so at 133x finer resolution
          arrival becomes much stricter: the motor creeps toward the target and the flag
          can take many seconds - or not latch at all - while the physical error is well
          under a degree. Always give `wait_in_position()` a timeout.
        - Position is a 32-bit count; at 4 x PPR resolution it overflows ~133x sooner
          than in hall mode (the vendor spec warns about this explicitly).

        Enabling requires a nonzero `ENC_PPR` (a wired, configured encoder); enabling
        with `ENC_PPR = 0` is refused because that combination is undefined.
        The write itself is acknowledged normally (no controller reinitialisation), but
        the first read-back can still return the old value - `verify` therefore retries
        the read-back (up to ``EPOSI_VERIFY_TRIES`` times, waiting `settle` before each)
        and raises `MdrobotError` if the value never sticks (e.g. older firmware
        silently ignoring the register).
        """
        if enabled and self.get_encoder_ppr() == 0:
            raise MdrobotError(
                "cannot enable encoder position with ENC_PPR = 0 - "
                "call set_encoder_ppr() with the encoder's rated PPR first")
        value = 1 if enabled else 0
        self.client.write_register(reg.PID_USE_EPOSI, value)
        if not verify:
            return
        readback = None
        for _ in range(EPOSI_VERIFY_TRIES):
            time.sleep(settle)
            readback = self.client.read_register(reg.PID_USE_EPOSI)
            if readback == value:
                return
        raise MdrobotError(
            f"PID_USE_EPOSI not applied: wrote {value}, read back {readback}")

    # --- slow-start / slow-down (acceleration/deceleration ramp) -----------------------
    # Speed slow hardware-verified (Phase 12); position slow still doc-based.
    def set_slow_start(self, seconds: float, *, full_scale_s: float = SLOW_DEFAULT_FULL_SCALE_S) -> None:
        """Set the speed slow-start (acceleration ramp) time in seconds (`PID_SLOW_START`).

        Hardware-verified (Phase 12). Full scale (default 15 s) is set by
        PID_MAX_SS_TIME on the controller.
        """
        self._set_slow(reg.PID_SLOW_START, seconds, full_scale_s)

    def get_slow_start(self, *, full_scale_s: float = SLOW_DEFAULT_FULL_SCALE_S) -> float:
        """Read the speed slow-start time in seconds (`PID_SLOW_START`)."""
        return self._get_slow(reg.PID_SLOW_START, full_scale_s)

    def set_slow_down(self, seconds: float, *, full_scale_s: float = SLOW_DEFAULT_FULL_SCALE_S) -> None:
        """Set the speed slow-down (deceleration ramp) time in seconds (`PID_SLOW_DOWN`). Hardware-verified (Phase 12)."""
        self._set_slow(reg.PID_SLOW_DOWN, seconds, full_scale_s)

    def get_slow_down(self, *, full_scale_s: float = SLOW_DEFAULT_FULL_SCALE_S) -> float:
        """Read the speed slow-down time in seconds (`PID_SLOW_DOWN`)."""
        return self._get_slow(reg.PID_SLOW_DOWN, full_scale_s)

    def set_position_slow_start(self, seconds: float, *, full_scale_s: float = SLOW_DEFAULT_FULL_SCALE_S) -> None:
        """Set the position slow-start time in seconds (`PID_POSI_SS`). NOT hardware-verified."""
        self._set_slow(reg.PID_POSI_SS, seconds, full_scale_s)

    def get_position_slow_start(self, *, full_scale_s: float = SLOW_DEFAULT_FULL_SCALE_S) -> float:
        """Read the position slow-start time in seconds (`PID_POSI_SS`)."""
        return self._get_slow(reg.PID_POSI_SS, full_scale_s)

    def set_position_slow_down(self, seconds: float, *, full_scale_s: float = SLOW_DEFAULT_FULL_SCALE_S) -> None:
        """Set the position slow-down time in seconds (`PID_POSI_SD`). NOT hardware-verified."""
        self._set_slow(reg.PID_POSI_SD, seconds, full_scale_s)

    def get_position_slow_down(self, *, full_scale_s: float = SLOW_DEFAULT_FULL_SCALE_S) -> float:
        """Read the position slow-down time in seconds (`PID_POSI_SD`)."""
        return self._get_slow(reg.PID_POSI_SD, full_scale_s)


class DualMotorDriver(_DriverBase):
    """Dual-channel motor driver. channel is 1 or 2.

    Verified on hardware: motor 1 = `PID_VEL_CMD(130)`, motor 2 = `PID_VEL_CMD2(131)`,
    driven independently (the two PIDs are independent, so changing one does not affect
    the other's target). Some controllers take ~1 s to start turning, so do not send 0
    immediately after a command.
    """

    _CH_VEL_PID = {1: reg.PID_VEL_CMD, 2: reg.PID_VEL_CMD2}
    # slow-start / slow-down register per channel (NOT hardware-verified)
    _CH_SLOW_START = {1: reg.PID_SLOW_START1, 2: reg.PID_SLOW_START2}
    _CH_SLOW_DOWN = {1: reg.PID_SLOW_DOWN1, 2: reg.PID_SLOW_DOWN2}
    _CH_POSI_SS = {1: reg.PID_POSI_SS1, 2: reg.PID_POSI_SS2}
    _CH_POSI_SD = {1: reg.PID_POSI_SD1, 2: reg.PID_POSI_SD2}

    @staticmethod
    def _ch_flag_word(channel: int) -> int:
        """Channel flag word for brake/tq-off. ch1 -> 0x0001 (DL), ch2 -> 0x0100 (DH)."""
        if channel == 1:
            return 0x0001
        if channel == 2:
            return 0x0100
        raise ValueError(f"channel must be 1 or 2, got {channel}")

    def _vel_pid(self, channel: int) -> int:
        try:
            return self._CH_VEL_PID[channel]
        except KeyError:
            raise ValueError(f"channel must be 1 or 2, got {channel}") from None

    @staticmethod
    def _ch_pid(mapping: dict[int, int], channel: int) -> int:
        try:
            return mapping[channel]
        except KeyError:
            raise ValueError(f"channel must be 1 or 2, got {channel}") from None

    def set_velocities(self, rpm1: int, rpm2: int) -> None:
        """WARNING: turns both motors immediately. Requires `enable()` first; mind the ~1 s delay.

        signed rpm. + = increasing-position direction. Motor 1 and motor 2 are independent.
        """
        self.client.write_register(reg.PID_VEL_CMD, word_from_int16(rpm1))
        self.client.write_register(reg.PID_VEL_CMD2, word_from_int16(rpm2))

    def set_velocity(self, channel: int, rpm: int) -> None:
        """WARNING: turns only the given channel (1/2) immediately; leaves the other as-is."""
        self.client.write_register(self._vel_pid(channel), word_from_int16(rpm))

    def stop(self) -> None:
        """Zero speed on both motors."""
        self.set_velocities(0, 0)

    def stop_channel(self, channel: int) -> None:
        """Zero speed on the given channel."""
        self.client.write_register(self._vel_pid(channel), 0)

    def brake_both(self) -> None:
        """WARNING: electric brake on both motors (`PID_PNT_BRAKE`=0x0101)."""
        self.client.write_register(reg.PID_PNT_BRAKE, 0x0101)

    def brake(self, channel: int) -> None:
        """WARNING: electric brake on the given channel only."""
        self.client.write_register(reg.PID_PNT_BRAKE, self._ch_flag_word(channel))

    def torque_off_both(self) -> None:
        """Free state on both motors (`PID_PNT_TQ_OFF`=0x0101)."""
        self.client.write_register(reg.PID_PNT_TQ_OFF, 0x0101)

    def torque_off(self, channel: int) -> None:
        """Free state on the given channel only."""
        self.client.write_register(reg.PID_PNT_TQ_OFF, self._ch_flag_word(channel))

    def read_monitor(self) -> DualMonitor:
        """`PID_PNT_MONITOR(216)`: speed/position/status for both motors (no current)."""
        return decode_pnt_monitor(self.client.read_registers(reg.PID_PNT_MONITOR, 7))

    def read_main_data(self) -> DualMonitor:
        """`PID_PNT_MAIN_DATA(210)`: speed/current/position/status for both motors."""
        return decode_pnt_main_data(self.client.read_registers(reg.PID_PNT_MAIN_DATA, 9))

    def get_speed(self, channel: int) -> int:
        """Measured speed of the given channel (signed rpm), from the dual monitor."""
        mon = self.read_monitor()
        if channel == 1:
            return mon.motor1.speed_rpm
        if channel == 2:
            return mon.motor2.speed_rpm
        raise ValueError(f"channel must be 1 or 2, got {channel}")

    def get_current(self, channel: int) -> float:
        """Motor current (A) of the given channel, from PNT_MAIN_DATA (PNT_MONITOR has none)."""
        mon = self.read_main_data()
        if channel == 1:
            return mon.motor1.current_a or 0.0
        if channel == 2:
            return mon.motor2.current_a or 0.0
        raise ValueError(f"channel must be 1 or 2, got {channel}")

    def get_positions(self) -> tuple[int, int]:
        """(motor1 position, motor2 position), from the dual monitor."""
        mon = self.read_monitor()
        return mon.motor1.position, mon.motor2.position

    def get_position(self, channel: int) -> int:
        """Position count of the given channel."""
        p1, p2 = self.get_positions()
        if channel == 1:
            return p1
        if channel == 2:
            return p2
        raise ValueError(f"channel must be 1 or 2, got {channel}")

    def reset_position(self) -> None:
        """Reset both motors' position counts to 0 (`PID_POSI_RESET`; resets both at once)."""
        self.client.write_register(reg.PID_POSI_RESET, 0)

    # --- position control (verified: needs only UI_COM=1, no START_STOP arm) -----------
    def _write_pnt_posi_vel(self, pid: int, pos1: int, spd1: int, pos2: int, spd2: int) -> None:
        """12-byte command: [pos1 long, spd1 word, pos2 long, spd2 word]."""
        l1, h1 = split_i32_low_word_first(pos1)
        l2, h2 = split_i32_low_word_first(pos2)
        self.client.write_registers(pid, [l1, h1, spd1 & 0xFFFF, l2, h2, spd2 & 0xFFFF])

    def move_to_both(self, pos1: int, pos2: int, speed1: int = 100, speed2: int | None = None) -> None:
        """WARNING: moves both motors to absolute positions immediately (`PID_PNT_POSI_VEL_CMD`). Requires `enable()`.

        If speed2 is omitted, speed1 is used. Verified on hardware: independent targets,
        both directions reached.
        """
        self._write_pnt_posi_vel(
            reg.PID_PNT_POSI_VEL_CMD, pos1, speed1, pos2, speed1 if speed2 is None else speed2
        )

    def move_by_both(self, delta1: int, delta2: int, speed1: int = 100, speed2: int | None = None) -> None:
        """WARNING: relative move of both motors (`PID_PNT_INC_POSI_VEL_CMD`). Requires `enable()`."""
        self._write_pnt_posi_vel(
            reg.PID_PNT_INC_POSI_VEL_CMD, delta1, speed1, delta2, speed1 if speed2 is None else speed2
        )

    # --- slow-start / slow-down per channel --------------------------------------------
    # Speed slow hardware-verified (Phase 12, PNT50: PID 108/109/111/112); position slow doc-based.
    def set_slow_start(self, channel: int, seconds: float, *, full_scale_s: float = SLOW_DEFAULT_FULL_SCALE_S) -> None:
        """Set MOT{channel} speed slow-start (acceleration ramp) time in seconds. Hardware-verified (Phase 12)."""
        self._set_slow(self._ch_pid(self._CH_SLOW_START, channel), seconds, full_scale_s)

    def get_slow_start(self, channel: int, *, full_scale_s: float = SLOW_DEFAULT_FULL_SCALE_S) -> float:
        """Read MOT{channel} speed slow-start time in seconds."""
        return self._get_slow(self._ch_pid(self._CH_SLOW_START, channel), full_scale_s)

    def set_slow_down(self, channel: int, seconds: float, *, full_scale_s: float = SLOW_DEFAULT_FULL_SCALE_S) -> None:
        """Set MOT{channel} speed slow-down (deceleration ramp) time in seconds. Hardware-verified (Phase 12)."""
        self._set_slow(self._ch_pid(self._CH_SLOW_DOWN, channel), seconds, full_scale_s)

    def get_slow_down(self, channel: int, *, full_scale_s: float = SLOW_DEFAULT_FULL_SCALE_S) -> float:
        """Read MOT{channel} speed slow-down time in seconds."""
        return self._get_slow(self._ch_pid(self._CH_SLOW_DOWN, channel), full_scale_s)

    def set_position_slow_start(self, channel: int, seconds: float, *, full_scale_s: float = SLOW_DEFAULT_FULL_SCALE_S) -> None:
        """Set MOT{channel} position slow-start time in seconds. NOT hardware-verified."""
        self._set_slow(self._ch_pid(self._CH_POSI_SS, channel), seconds, full_scale_s)

    def get_position_slow_start(self, channel: int, *, full_scale_s: float = SLOW_DEFAULT_FULL_SCALE_S) -> float:
        """Read MOT{channel} position slow-start time in seconds."""
        return self._get_slow(self._ch_pid(self._CH_POSI_SS, channel), full_scale_s)

    def set_position_slow_down(self, channel: int, seconds: float, *, full_scale_s: float = SLOW_DEFAULT_FULL_SCALE_S) -> None:
        """Set MOT{channel} position slow-down time in seconds. NOT hardware-verified."""
        self._set_slow(self._ch_pid(self._CH_POSI_SD, channel), seconds, full_scale_s)

    def get_position_slow_down(self, channel: int, *, full_scale_s: float = SLOW_DEFAULT_FULL_SCALE_S) -> float:
        """Read MOT{channel} position slow-down time in seconds."""
        return self._get_slow(self._ch_pid(self._CH_POSI_SD, channel), full_scale_s)
