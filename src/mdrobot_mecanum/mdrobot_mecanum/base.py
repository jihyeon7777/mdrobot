"""`MecanumBase` — the four-wheel base over two dual-channel controllers on one bus.

This is the only module in the package that talks to hardware. It owns exactly one
`SerialTransport` and one `ModbusClient` per controller, because the transport is what
holds the Modbus t3.5 inter-frame gap for the whole bus (`mdrobot/transport.py`) —
giving each controller its own transport would break that and corrupt frames.

**A velocity command is not a latch.** The controller cuts drive when the bus goes
quiet for about two seconds (`COMMAND_WATCHDOG_S`), so sustained motion requires a
control loop that keeps talking. Commanding a speed and then sleeping produces a
twitch, not motion. See the constant for the measurements.

Safety properties this layer adds on top of `mdrobot`:

- **Nothing reaches a motor unclamped.** `word_from_int16()` in the library is a bare
  `& 0xFFFF`, so a command past +/-32767 silently flips sign. Every rpm here goes
  through the proportional clamp and an int16 guard.
- **A partial failure stops everything.** Two mecanum wheels driving while two are
  dead is worse than a diff-drive half-failure: the robot slews unpredictably. Any
  write error triggers a best-effort stop of BOTH controllers before the exception
  propagates. This mirrors the twin policy in `mdrobot_system.cpp:551-560`.
- **`__exit__` stops and cuts torque**, unlike `mdrobot._DriverBase.__exit__` which
  only closes the port. A motor does not stop because a program exited.

Tolerance is deliberately NOT here: the base always both-stops and raises, and the
caller decides how many consecutive failures to absorb. Generic layer strict,
application layer decides — the same split the rest of the repository uses.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

from mdrobot import (
    DualMotorDriver,
    MdrobotError,
    ModbusClient,
    SerialTransport,
    SingleMotorDriver,
    rad_s_to_rpm,
    registers as reg,
    resolve_port,
)

from .config import MecanumConfig
from .kinematics import WHEEL_NAMES, forward, inverse, scale_to_limit

#: Widest signed value a controller register can carry.
INT16_MAX = 32767

#: The controller cuts motor drive when the BUS goes quiet for roughly this long.
#: Measured on PNT50 DL=19, 2026-08-12: gaps of 0.5 / 1.0 / 1.5 s were survived, 2.0 s
#: and 3.0 s cut the motor. It is a traffic watchdog, not a command watchdog — a plain
#: register READ refreshes it just as well as a velocity write.
#:
#: Treat it as a safety feature: a control program that crashes stops sending, and the
#: robot stops by itself within ~2 s. But it also means a command is NOT a latch. Code
#: that commands a speed and then sleeps without touching the bus will silently get a
#: brief twitch instead of sustained motion.
COMMAND_WATCHDOG_S = 2.0

#: Time to leave between `enable()` and the first velocity command.
#: Measured on PNT50 DL=19, 2026-08-12: commanding immediately after enable() took
#: 1.93 s to produce motion, while commanding 2 s later took 0.75 s. The ~1.2 s
#: difference is the controller settling after the run-latch arm, and it is paid once.
ENABLE_SETTLE_S = 1.5

#: How long a motor takes to start turning once armed and commanded (low rpm, no load).
START_DELAY_S = 0.8


class DriveError(MdrobotError):
    """A wheel command failed. Every controller has been sent a best-effort stop."""


@dataclass(frozen=True)
class DriveResult:
    """What one `drive()` call actually commanded, for display and for tests."""

    twist: tuple[float, float, float]          #: (vx, vy, wz) as requested
    wheel_rad_s: tuple[float, float, float, float]  #: after the clamp, at the wheel
    wheel_rpm: tuple[int, int, int, int]       #: signed motor rpm actually written
    scale: float                               #: clamp factor, 1.0 = untouched

    @property
    def clamped(self) -> bool:
        return self.scale < 1.0

    @property
    def effective_twist(self) -> tuple[float, float, float]:
        """The twist the robot will actually follow — the request times the clamp.

        Because the kinematics is linear, clamping cannot bend the path; it only
        slows the robot along it.
        """
        return tuple(c * self.scale for c in self.twist)


class MecanumBase:
    """Four mecanum wheels on two dual-channel controllers sharing one RS485 bus."""

    def __init__(self, config: MecanumConfig, transport, drivers: dict[int, DualMotorDriver]):
        self.config = config
        self._transport = transport
        self._drivers = drivers
        self._closed = False
        #: Set False when a shutdown stop/torque-off could not be confirmed.
        self.last_stop_ok = True

    @classmethod
    def from_config(cls, config: MecanumConfig) -> "MecanumBase":
        """Open the bus and build one driver per controller."""
        port = resolve_port(config.port)
        transport = SerialTransport(port, config.baudrate, timeout=config.timeout)
        try:
            # One transport, one client per slave id. Never DualMotorDriver.open() —
            # that builds a second transport and the bus-wide t3.5 gap is lost.
            drivers = {sid: DualMotorDriver(ModbusClient(transport, slave_id=sid))
                       for sid in config.slave_ids}
        except Exception:
            transport.close()
            raise
        return cls(config, transport, drivers)

    # --- introspection ---------------------------------------------------------------

    @property
    def slave_ids(self) -> tuple[int, ...]:
        return self.config.slave_ids

    def driver(self, slave_id: int) -> DualMotorDriver:
        try:
            return self._drivers[slave_id]
        except KeyError:
            raise ValueError(
                f"no controller at slave id {slave_id}; configured: {self.slave_ids}") from None

    def wheel_at(self, slave_id: int, channel: int) -> str | None:
        """Which wheel a motor output drives, or None if it is not in the config."""
        for name, spec in self.config.wheels.items():
            if spec.address == (slave_id, channel):
                return name
        return None

    # --- preflight -------------------------------------------------------------------

    def identify(self) -> dict[int, dict]:
        """Read version / voltage / status from every controller. No writes."""
        report = {}
        for sid, drv in self._drivers.items():
            report[sid] = {
                "version": drv.get_version(),
                "voltage": drv.get_voltage(),
                "status": drv.get_status(),
            }
        return report

    def preflight(self) -> list[str]:
        """Bring every controller into a state where serial velocity drive works.

        Returns a human-readable log of what was checked and what was changed.

        Every write is conditional on a read-back showing the value is wrong. That
        matters twice over: it keeps a correctly configured controller untouched, and
        it makes the destructive-looking `ENC_PPR` write a no-op on hardware that does
        not need it (writing that register can reinitialise the controller).
        """
        log: list[str] = []
        for sid, drv in self._drivers.items():
            client = drv.client

            # ENC_PPR: 0 = hall closed loop. Nonzero with no encoder wired makes the
            # first command lurch ~0.6 s and then alarm.
            ppr = client.read_register(reg.PID_ENC_PPR)
            if ppr == 0:
                log.append(f"id {sid}: ENC_PPR already 0 (hall closed loop) — no write")
            else:
                log.append(f"id {sid}: ENC_PPR = {ppr} — encoder mode. NOT changed "
                           f"automatically; set it to 0 only if no encoder is wired "
                           f"(SingleMotorDriver(drv.client).disable_encoder())")

            # USE_LIMIT_SW / USE_LIMIT_SW2: the CTRL pins gate motion per direction.
            # The manual records these resetting after a power cycle, so re-assert.
            want = self.config.runtime.use_limit_sw
            if want >= 0:
                for pid, label in ((reg.PID_USE_LIMIT_SW, "USE_LIMIT_SW"),
                                   (reg.PID_USE_LIMIT_SW2, "USE_LIMIT_SW2")):
                    current = client.read_register(pid)
                    if current == want:
                        log.append(f"id {sid}: {label} already {want} — no write")
                    else:
                        client.write_register(pid, want)
                        readback = client.read_register(pid)
                        log.append(f"id {sid}: {label} {current} -> {readback}"
                                   + ("" if readback == want else "  WRITE DID NOT STICK"))

            # Short controller ramps protect the drive stage from a step command. The
            # real shaping is the software twist ramp: the four channels ramp
            # independently here, so a long ramp would break the wheel-speed ratio.
            ramp = self.config.runtime.controller_ramp_s
            if ramp is not None:
                for channel in (1, 2):
                    drv.set_slow_start(channel, ramp)
                    drv.set_slow_down(channel, ramp)
                log.append(f"id {sid}: controller ramps set to {ramp}s on both channels")

            cap = client.read_register(reg.PID_MAX_RPM)
            limit = self.config.limits.max_motor_rpm
            log.append(f"id {sid}: MAX_RPM {cap} rpm, config max_motor_rpm {limit} rpm"
                       + ("  CONFIG EXCEEDS THE CONTROLLER CAP" if limit > cap else ""))
        return log

    def enable(self, *, settle: float = ENABLE_SETTLE_S) -> None:
        """Arm every controller for serial velocity commands. Required before motion.

        Sleeps `settle` afterwards. The controller needs about 1.2 s after the
        run-latch arm before a velocity command takes effect promptly; without the
        pause the first command appears to be ignored for ~2 s, which reads as a dead
        channel. Pass `settle=0` only when the caller does its own waiting.
        """
        for drv in self._drivers.values():
            drv.enable()
        if settle > 0:
            time.sleep(settle)

    def disable(self) -> None:
        for drv in self._drivers.values():
            drv.disable()

    # --- motion ----------------------------------------------------------------------

    def spin_one(self, slave_id: int, channel: int, rpm: int) -> None:
        """Turn ONE motor output and leave the others as they are.

        For wheel identification only — `drive()` is the interface for actually moving
        the robot. `rpm` is clamped to `max_motor_rpm` and to int16.
        """
        rpm = self._clamp_rpm(rpm)
        try:
            self.driver(slave_id).set_velocity(channel, rpm)
        except MdrobotError as exc:
            self._emergency_stop_all()
            raise DriveError(
                f"spin_one(id={slave_id}, ch={channel}, rpm={rpm}) failed: "
                f"{type(exc).__name__}: {exc}") from exc

    def drive(self, vx: float, vy: float, wz: float) -> DriveResult:
        """Command a body twist. Returns what was actually written.

        Args:
            vx: forward, m/s. vy: left, m/s. wz: counter-clockwise, rad/s.
        """
        geom = self.config.geometry
        omega = inverse(vx, vy, wz, geom)

        # Clamp in MOTOR rpm, not wheel rad/s: max_motor_rpm is a property of the
        # motor, and with a gearbox the two differ by the gear ratio.
        raw_rpm = [rad_s_to_rpm(w) * geom.gear_ratio for w in omega]
        scaled, scale = scale_to_limit(raw_rpm, float(self.config.limits.max_motor_rpm))

        wheel_rpm = tuple(self._clamp_rpm(round(value) * spec.sign)
                          for value, spec in zip(scaled, self.config.wheel_specs))
        self._write_all(wheel_rpm)
        return DriveResult(twist=(vx, vy, wz),
                           wheel_rad_s=tuple(w * scale for w in omega),
                           wheel_rpm=wheel_rpm,
                           scale=scale)

    def drive_wheels(self, wheel_rpm) -> DriveResult:
        """Command four motor rpm directly, bypassing the kinematics.

        Used by the identification and verification paths, where the point is to
        exercise a known wheel pattern rather than a body twist. Signs are applied
        here too, so the caller thinks in "positive drives the robot forward".
        """
        wheel_rpm = list(wheel_rpm)
        if len(wheel_rpm) != len(WHEEL_NAMES):
            raise ValueError(f"need {len(WHEEL_NAMES)} wheel speeds, got {len(wheel_rpm)}")
        scaled, scale = scale_to_limit(wheel_rpm, float(self.config.limits.max_motor_rpm))
        applied = tuple(self._clamp_rpm(round(value) * spec.sign)
                        for value, spec in zip(scaled, self.config.wheel_specs))
        self._write_all(applied)
        geom = self.config.geometry
        omega = tuple(value / 60.0 * 6.283185307179586 / geom.gear_ratio for value in scaled)
        return DriveResult(twist=forward(omega, geom), wheel_rad_s=omega,
                           wheel_rpm=applied, scale=scale)

    def stop(self) -> None:
        """Zero velocity on all four motors. Raises if any controller refuses."""
        self._write_all((0, 0, 0, 0))

    def torque_off(self) -> None:
        """Free all four motors. Best effort across controllers; raises at the end."""
        failures = []
        for sid, drv in self._drivers.items():
            try:
                drv.torque_off_both()
            except MdrobotError as exc:
                failures.append(f"id {sid}: {type(exc).__name__}: {exc}")
        if failures:
            raise DriveError("torque_off failed on " + "; ".join(failures))

    # --- feedback --------------------------------------------------------------------

    def read_wheel_rpm(self) -> tuple[int, int, int, int]:
        """Measured motor rpm per wheel, sign-corrected so + means "drives forward"."""
        monitors = {sid: drv.read_monitor() for sid, drv in self._drivers.items()}
        values = []
        for spec in self.config.wheel_specs:
            monitor = monitors[spec.slave_id]
            channel = monitor.motor1 if spec.channel == 1 else monitor.motor2
            values.append(channel.speed_rpm * spec.sign)
        return tuple(values)

    def read_wheel_positions(self) -> tuple[int, int, int, int]:
        """Per-wheel position counts, sign-corrected so + means "drove forward".

        Prefer this over `read_wheel_rpm` for anything that has to be right. The
        instantaneous speed register is badly quantised at low speed — a 4-pole hall
        produces only a few edges per second, so single samples swing wildly — while
        position counts accumulate and cannot lie about which way a wheel went.
        """
        monitors = {sid: drv.read_monitor() for sid, drv in self._drivers.items()}
        values = []
        for spec in self.config.wheel_specs:
            monitor = monitors[spec.slave_id]
            channel = monitor.motor1 if spec.channel == 1 else monitor.motor2
            values.append(channel.position * spec.sign)
        return tuple(values)

    def read_wheel_rad_s(self) -> tuple[float, float, float, float]:
        """Measured wheel angular velocity, undoing sign and gear ratio."""
        ratio = self.config.geometry.gear_ratio
        return tuple(rpm / 60.0 * 6.283185307179586 / ratio for rpm in self.read_wheel_rpm())

    def odometry_twist(self) -> tuple[float, float, float]:
        """(vx, vy, wz) implied by the measured wheel speeds."""
        return forward(self.read_wheel_rad_s(), self.config.geometry)

    # --- lifecycle -------------------------------------------------------------------

    def shutdown(self, *, attempts: int = 3, delay: float = 0.05) -> bool:
        """Stop, settle, stop again, then cut torque. Returns True if all of it landed.

        Two stops with a pause between them because a controller can take about a
        second to act on a command: a single stop issued immediately after a drive
        command can be overtaken by the motion that command started. Each step is
        retried because a SIGINT landing mid-transaction leaves a stale response tail
        on the wire that the next `flush_input()` cannot catch — the first retry
        clears it (the pattern in `motor_driver_node.py:336`).
        """
        ok = self._retry("stop", self.stop, attempts, delay)
        time.sleep(0.3)
        ok = self._retry("stop", self.stop, attempts, delay) and ok
        ok = self._retry("torque_off", self.torque_off, attempts, delay) and ok
        self.last_stop_ok = ok
        return ok

    def close(self) -> None:
        """Close the shared transport. Idempotent.

        Closes the TRANSPORT once, not each driver: every driver shares this port, so
        `drv.close()` would pull it out from under the others.
        """
        if self._closed:
            return
        self._closed = True
        self._transport.close()

    def __enter__(self) -> "MecanumBase":
        return self

    def __exit__(self, *exc: object) -> None:
        try:
            self.shutdown()
        finally:
            self.close()

    # --- internals -------------------------------------------------------------------

    def _clamp_rpm(self, rpm) -> int:
        """Round, cap at max_motor_rpm, and guard the int16 wire limit."""
        limit = self.config.limits.max_motor_rpm
        value = int(round(rpm))
        value = max(-limit, min(limit, value))
        return max(-INT16_MAX, min(INT16_MAX, value))

    def _write_all(self, wheel_rpm) -> None:
        """Write all four speeds, controller by controller. Both-stops on any failure."""
        by_slave: dict[int, dict[int, int]] = {}
        for spec, rpm in zip(self.config.wheel_specs, wheel_rpm):
            by_slave.setdefault(spec.slave_id, {})[spec.channel] = rpm

        for sid in self.slave_ids:
            channels = by_slave[sid]
            try:
                self._write_pair(sid, channels[1], channels[2])
            except MdrobotError as exc:
                # A partial write is the dangerous case: set_velocities is two separate
                # 0x06 frames, so channel 1 can land while channel 2 fails on the same
                # board. Stop everything before anyone hears about the error.
                self._emergency_stop_all()
                raise DriveError(
                    f"velocity write to controller {sid} failed "
                    f"({type(exc).__name__}: {exc}); all controllers sent a stop"
                ) from exc

    def _write_pair(self, slave_id: int, rpm1: int, rpm2: int) -> None:
        drv = self.driver(slave_id)
        if self.config.runtime.use_batched_velocity:
            # One 0x10 frame instead of two 0x06 frames. UNVERIFIED word order — only
            # reachable after the batched probe passes on this hardware.
            from mdrobot import word_from_int16
            drv.client.write_registers(
                reg.PID_PNT_VEL_CMD, [word_from_int16(rpm1), word_from_int16(rpm2)])
        else:
            drv.set_velocities(rpm1, rpm2)

    def _emergency_stop_all(self) -> None:
        """Best-effort stop of every controller. Never raises — it is the last resort."""
        for drv in self._drivers.values():
            try:
                drv.stop()
                continue
            except MdrobotError:
                pass
            try:
                drv.torque_off_both()
            except MdrobotError:
                self.last_stop_ok = False

    @staticmethod
    def _retry(what: str, action, attempts: int, delay: float) -> bool:
        for attempt in range(attempts):
            try:
                action()
                return True
            except MdrobotError:
                if attempt < attempts - 1:
                    time.sleep(delay)
        return False


def borrow_single(driver: DualMotorDriver) -> SingleMotorDriver:
    """A `SingleMotorDriver` view of a dual controller, for the channel-free registers.

    `disable_encoder()` / `set_encoder_ppr()` live on `SingleMotorDriver`, but
    `PID_ENC_PPR(156)` is a controller-wide register with no channel variant, so
    borrowing the method on the same `ModbusClient` is legitimate and costs no
    duplication and no change to the released library.
    """
    return SingleMotorDriver(driver.client)
