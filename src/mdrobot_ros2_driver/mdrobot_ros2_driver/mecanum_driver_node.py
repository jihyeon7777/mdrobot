#!/usr/bin/env python3
"""Drive four mecanum wheels on two dual-channel MD controllers, one RS485 bus.

Why this exists rather than two motor_driver_node instances: both controllers
sit on the SAME serial port. Two nodes would each open /dev/ttyUSB0 and their
Modbus frames would interleave on the wire, so exactly one process has to own
the bus. This node opens the transport once and addresses both slave ids
through it, which is what the mdrobot library's split between SerialTransport
and ModbusClient is for.

It is a hardware layer and decides nothing: rpm in, rpm on the motors, encoder
positions out. Kinematics lives above it, in mdrobot_supervisor.

Interface
---------
Parameters:
  port (str='/dev/ttyUSB0')     one port for both controllers
  baudrate (int=19200)
  timeout (float=0.3)           serial timeout, seconds
  wheel_slave_ids (int[])       controller per wheel, in front_left, front_right,
                                rear_left, rear_right order
  wheel_channels (int[])        1 or 2 within that controller, same order
  counts_per_rev (double[])     per wheel, at the MOTOR shaft. Set it and
                                joint_states carries radians; leave it 0 and it
                                carries raw counts and says so once, loudly.
                                Measure with examples/calibrate_counts_per_rev.py
  joint_names (str[])           defaults to the four wheel names
  max_rpm (float=600)           hard cap applied per wheel before writing
  command_timeout (float=0.5)   stop the motors if no command arrives. 0 disables
  publish_rate (float=20.0)     Hz, joint_states
  diag_rate (float=2.0)         Hz, diagnostics
  auto_enable (bool=True)       enable both controllers on startup

Subscriptions:
  ~/cmd_wheel_rpm (std_msgs/Float64MultiArray)
      [front_left, front_right, rear_left, rear_right], motor-shaft rpm, already
      signed for mounting direction by the layer above.

Publishers:
  ~/joint_states (sensor_msgs/JointState)   four wheels, position and velocity
  ~/diagnostics  (diagnostic_msgs/DiagnosticArray)

Services (std_srvs/Trigger):
  ~/enable ~/disable ~/stop ~/brake ~/torque_off ~/reset_position

Bus budget: one 19200-baud line carries both controllers, and a transaction
costs roughly 12 ms. Every command message writes to both, and every
joint_states tick reads from both, so the combined rate of ~/cmd_wheel_rpm and
publish_rate is what has to fit inside a second. 10 Hz each is 48%, the figure
mecanum.yaml records as working; 50 Hz of commands alone would want 120% and
the transactions would simply queue.

Safety: with command_timeout > 0 the wheels stop when commands stop arriving.
Bus access is serialised on a single-threaded executor, so reads and writes
never interleave.
"""

from __future__ import annotations

import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

from mdrobot import DualMotorDriver
from mdrobot.exceptions import MdrobotError
from mdrobot.protocol import ModbusClient
from mdrobot.transport import SerialTransport, resolve_port
from mdrobot.units import rpm_to_rad_s

WHEEL_NAMES = ("front_left", "front_right", "rear_left", "rear_right")


class MecanumDriverNode(Node):
    def __init__(self) -> None:
        super().__init__("mdrobot_mecanum_driver")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 19200)
        self.declare_parameter("timeout", 0.3)
        self.declare_parameter("wheel_slave_ids", [1, 1, 2, 2])
        self.declare_parameter("wheel_channels", [1, 2, 2, 1])
        self.declare_parameter("counts_per_rev", [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("joint_names", list(WHEEL_NAMES))
        self.declare_parameter("max_rpm", 600.0)
        self.declare_parameter("command_timeout", 0.5)
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("diag_rate", 2.0)
        self.declare_parameter("auto_enable", True)

        self.wheel_slave_ids = [int(v) for v in self.get_parameter("wheel_slave_ids").value]
        self.wheel_channels = [int(v) for v in self.get_parameter("wheel_channels").value]
        self.joint_names = [str(v) for v in self.get_parameter("joint_names").value]
        self.max_rpm = float(self.get_parameter("max_rpm").value)
        self.command_timeout = float(self.get_parameter("command_timeout").value)
        self._validate()

        cpr = [float(v) for v in self.get_parameter("counts_per_rev").value]
        if len(cpr) == 4 and all(v > 0 for v in cpr):
            self.counts_per_rev = cpr
            self.publish_si = True
        else:
            self.counts_per_rev = None
            self.publish_si = False
            self.get_logger().warn(
                f"counts_per_rev is {cpr}: joint_states will carry RAW COUNTS, not "
                "radians. Anything computing distance from it must be told so. "
                "Measure with examples/calibrate_counts_per_rev.py."
            )

        # One transport, one client per controller. This is the whole point of
        # the node: the bus is opened exactly once.
        port = resolve_port(str(self.get_parameter("port").value))
        self.transport = SerialTransport(
            port,
            int(self.get_parameter("baudrate").value),
            timeout=float(self.get_parameter("timeout").value),
        )
        self.drivers = {
            slave: DualMotorDriver(ModbusClient(self.transport, slave_id=slave))
            for slave in self.controller_ids
        }

        self._command = [0.0] * 4
        self._command_wall = 0.0
        self._stopped = True
        self._errors = 0
        self._last_positions: list[float] | None = None

        self.create_subscription(
            Float64MultiArray, "~/cmd_wheel_rpm", self._on_command, 10)
        self.pub_joints = self.create_publisher(JointState, "~/joint_states", 10)
        self.pub_diag = self.create_publisher(DiagnosticArray, "~/diagnostics", 1)

        for name, action in (
            ("~/enable", lambda d: d.enable()),
            ("~/disable", lambda d: d.disable()),
            ("~/stop", lambda d: d.stop()),
            ("~/brake", lambda d: d.brake_both()),
            ("~/torque_off", lambda d: d.torque_off_both()),
            ("~/reset_position", lambda d: d.reset_position()),
        ):
            self._make_service(name, action)

        if bool(self.get_parameter("auto_enable").value):
            self._for_each(lambda d: d.enable(), "enable")

        self.create_timer(1.0 / float(self.get_parameter("publish_rate").value),
                          self._publish_joints)
        self.create_timer(1.0 / float(self.get_parameter("diag_rate").value),
                          self._publish_diag)
        if self.command_timeout > 0:
            self.create_timer(self.command_timeout / 2.0, self._check_watchdog)

        self.get_logger().info(
            f"{port} @ {self.get_parameter('baudrate').value} baud, controllers "
            f"{self.controller_ids}, wheels "
            f"{list(zip(self.wheel_slave_ids, self.wheel_channels))}, "
            f"cap {self.max_rpm} rpm"
        )

    def _validate(self) -> None:
        for name, values in (("wheel_slave_ids", self.wheel_slave_ids),
                             ("wheel_channels", self.wheel_channels),
                             ("joint_names", self.joint_names)):
            if len(values) != 4:
                raise ValueError(f"{name} needs 4 entries {WHEEL_NAMES}, got {values}")
        if set(self.wheel_channels) - {1, 2}:
            raise ValueError(f"wheel_channels entries must be 1 or 2, got {self.wheel_channels}")
        slots = list(zip(self.wheel_slave_ids, self.wheel_channels))
        if len(set(slots)) != 4:
            raise ValueError(f"each wheel needs its own (slave_id, channel); got {slots}")
        self.controller_ids = sorted(set(self.wheel_slave_ids))
        if len(self.controller_ids) != 2:
            raise ValueError(
                f"expected exactly 2 controllers, got slave ids {self.controller_ids}"
            )
        if self.max_rpm <= 0:
            raise ValueError(f"max_rpm must be positive, got {self.max_rpm}")

    # ── bus helpers ─────────────────────────────────────────────────────────
    def _for_each(self, action, what: str) -> bool:
        ok = True
        for slave, driver in self.drivers.items():
            try:
                action(driver)
            except MdrobotError as exc:
                ok = False
                self._errors += 1
                self.get_logger().error(
                    f"{what} failed on controller {slave}: {type(exc).__name__}: {exc}"
                )
        return ok

    def _make_service(self, name: str, action) -> None:
        def handler(_request, response):
            response.success = self._for_each(action, name)
            response.message = "ok" if response.success else "see log"
            return response
        self.create_service(Trigger, name, handler)

    # ── command path ────────────────────────────────────────────────────────
    def _on_command(self, msg: Float64MultiArray) -> None:
        if len(msg.data) != 4:
            self.get_logger().warn(
                f"cmd_wheel_rpm needs 4 values {WHEEL_NAMES}, got {len(msg.data)}",
                throttle_duration_sec=2.0,
            )
            return
        self._command = [
            max(-self.max_rpm, min(self.max_rpm, float(v))) for v in msg.data
        ]
        self._command_wall = time.monotonic()
        self._stopped = False
        self._write_command(self._command)

    def _write_command(self, wheel_rpm: list[float]) -> None:
        """One write per controller, both channels at once."""
        for slave, driver in self.drivers.items():
            pair = [0, 0]
            for i in range(4):
                if self.wheel_slave_ids[i] == slave:
                    pair[self.wheel_channels[i] - 1] = int(round(wheel_rpm[i]))
            try:
                driver.set_velocities(pair[0], pair[1])
            except MdrobotError as exc:
                self._errors += 1
                self.get_logger().error(
                    f"set_velocities failed on controller {slave}: "
                    f"{type(exc).__name__}: {exc}",
                    throttle_duration_sec=1.0,
                )

    def _check_watchdog(self) -> None:
        if self._stopped or not self._command_wall:
            return
        if time.monotonic() - self._command_wall > self.command_timeout:
            self._stopped = True
            self._command = [0.0] * 4
            self.get_logger().warn(
                f"no cmd_wheel_rpm for {self.command_timeout:.2f} s; stopping"
            )
            self._for_each(lambda d: d.stop(), "stop")

    # ── feedback ────────────────────────────────────────────────────────────
    def _read_positions(self) -> tuple[list[float], list[float]] | None:
        """(positions, rpms) per wheel, straight off the controllers."""
        counts: dict[int, tuple[int, int]] = {}
        rpms: dict[int, tuple[int, int]] = {}
        for slave, driver in self.drivers.items():
            try:
                mon = driver.read_monitor()
            except MdrobotError as exc:
                self._errors += 1
                self.get_logger().warn(
                    f"monitor read failed on controller {slave}: "
                    f"{type(exc).__name__}: {exc}",
                    throttle_duration_sec=2.0,
                )
                return None
            counts[slave] = (mon.motor1.position, mon.motor2.position)
            rpms[slave] = (mon.motor1.speed_rpm, mon.motor2.speed_rpm)
        positions, speeds = [], []
        for i in range(4):
            slave, channel = self.wheel_slave_ids[i], self.wheel_channels[i]
            positions.append(float(counts[slave][channel - 1]))
            speeds.append(float(rpms[slave][channel - 1]))
        return positions, speeds

    def _publish_joints(self) -> None:
        if not rclpy.ok():
            return
        read = self._read_positions()
        if read is None:
            return
        counts, rpms = read
        self._last_positions = counts

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(self.joint_names)
        if self.publish_si:
            js.position = [c / cpr * 2.0 * math.pi
                           for c, cpr in zip(counts, self.counts_per_rev)]
        else:
            js.position = list(counts)
        js.velocity = [rpm_to_rad_s(r) for r in rpms] if self.publish_si else list(rpms)
        try:
            self.pub_joints.publish(js)
        except Exception:  # noqa: BLE001 - invalid context during shutdown
            pass

    def _publish_diag(self) -> None:
        status = DiagnosticStatus()
        status.name = "mdrobot_mecanum_driver: bus"
        status.hardware_id = str(self.get_parameter("port").value)
        if self._errors:
            status.level = DiagnosticStatus.WARN
            status.message = f"{self._errors} bus errors so far"
        elif self._stopped:
            status.level = DiagnosticStatus.OK
            status.message = "idle"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "driving"
        status.values = [
            KeyValue(key="errors", value=str(self._errors)),
            KeyValue(key="units", value="rad" if self.publish_si else "count"),
            KeyValue(key="command", value=",".join(f"{v:.0f}" for v in self._command)),
        ]
        if self._last_positions is not None:
            status.values += [
                KeyValue(key=n, value=f"{p:.1f}")
                for n, p in zip(self.joint_names, self._last_positions)
            ]
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [status]
        self.pub_diag.publish(msg)

    def destroy_node(self) -> bool:
        # Each step is tried on its own: a controller that fails to stop must not
        # stop the others from being told to.
        self._for_each(lambda d: d.stop(), "stop")
        self._for_each(lambda d: d.torque_off_both(), "torque_off")
        self._for_each(lambda d: d.disable(), "disable")
        try:
            self.transport.close()
        except Exception:  # noqa: BLE001
            pass
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = MecanumDriverNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
