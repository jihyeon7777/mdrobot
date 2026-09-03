#!/usr/bin/env python3
"""Generic MDROBOT motor driver ROS 2 node.

Handles single-channel and dual-channel controllers through the `device_type`
parameter. No robot kinematics — it exposes per-motor velocity/position commands
and motor state only. Uses only standard messages (std_msgs / std_srvs /
sensor_msgs / diagnostic_msgs).

Interface
---------
Parameters:
  port (str)                   serial port, e.g. /dev/ttyUSB0
  baudrate (int=19200)
  motor_id (int=1)
  device_type (str)            'single' | 'dual'
  command_timeout (float=0.5)  velocity-command watchdog, seconds. 0 disables it
  publish_rate (float=20.0)    joint_states rate, Hz
  diag_rate (float=2.0)        diagnostics rate, Hz
  position_max_rpm (int=100)   max speed for position commands
  joint_names (str[])          empty -> auto by device_type (motor1[, motor2])
  auto_enable (bool=True)      call enable() on startup
  counts_per_rev (double[])    counts per ONE revolution of the MOTOR shaft, per
                  channel (the controller measures position and speed at the motor).
                  If set, joint_states is published in SI (rad, rad/s); if unset / 0 /
                  wrong length, raw units (count, rpm) are published and a warning is
                  logged once. Measure it — turn the motor shaft exactly N revolutions
                  and divide (examples/calibrate_counts_per_rev.py); the datasheet is
                  not enough (hall ~ 3 x pole count, encoder 4 x PPR). Gear ratio is NOT
                  applied here: counts_per_rev scales position only, while velocity is
                  rpm -> rad/s regardless. Measuring at a geared output shaft would make
                  position the wheel angle but leave velocity at the motor rate (the two
                  then disagree by the gear ratio) — keep it at the motor and handle the
                  gearbox in the robot layer above.

Subscriptions (std_msgs/Float64MultiArray):
  ~/cmd_velocity  data=[rpm]            (single) | [rpm1, rpm2] (dual)
  ~/cmd_position  data=[count]          (single) | [count1, count2] (dual)
                  (max speed is position_max_rpm)

Publishers:
  ~/joint_states (sensor_msgs/JointState)
      counts_per_rev set: position=rad, velocity=rad/s (SI)
      otherwise:          position=count, velocity=rpm  (raw)
  ~/diagnostics  (diagnostic_msgs/DiagnosticArray)  voltage / status bits / alarm

Services (std_srvs/Trigger):
  ~/enable ~/disable ~/stop ~/brake ~/torque_off ~/reset_alarm ~/reset_position

Safety: with command_timeout > 0 the motor stops if no new ~/cmd_velocity arrives
within that time. Callbacks run sequentially on a single-threaded executor, so
serial-port access never overlaps.
"""

from __future__ import annotations

import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

from mdrobot import DualMotorDriver, SingleMotorDriver
from mdrobot import registers as reg
from mdrobot.exceptions import MdrobotError
from mdrobot.units import counts_to_rad, rpm_to_rad_s


class MotorDriverNode(Node):
    def __init__(self) -> None:
        super().__init__("mdrobot_motor_driver")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 19200)
        self.declare_parameter("motor_id", 1)
        self.declare_parameter("device_type", "single")
        self.declare_parameter("command_timeout", 0.5)
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("diag_rate", 2.0)
        self.declare_parameter("position_max_rpm", 100)
        self.declare_parameter("joint_names", [""])
        self.declare_parameter("auto_enable", True)
        # Per-channel counts_per_rev. If set, publish joint_states in SI (rad, rad/s).
        # Default [0.0] = unset -> raw (count, rpm). The value differs per motor, so we
        # never hard-code a default.
        self.declare_parameter("counts_per_rev", [0.0])
        # USE_LIMIT_SW policy: -1 = leave device setting (default), 0 = disable, 1 = enable.
        # Some controllers need 0 for serial drive; connecting an encoder can make this
        # mandatory (encoder A/B share pins with the limit inputs).
        self.declare_parameter("use_limit_sw", -1)

        self.port = self.get_parameter("port").value
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.motor_id = int(self.get_parameter("motor_id").value)
        self.device_type = str(self.get_parameter("device_type").value).lower()
        self.command_timeout = float(self.get_parameter("command_timeout").value)
        self.position_max_rpm = int(self.get_parameter("position_max_rpm").value)

        if self.device_type not in ("single", "dual"):
            raise ValueError(f"device_type must be 'single' or 'dual', got {self.device_type!r}")
        self.channels = 1 if self.device_type == "single" else 2

        joint_names = [n for n in self.get_parameter("joint_names").value if n]
        if not joint_names:
            joint_names = ["motor1"] if self.channels == 1 else ["motor1", "motor2"]
        if len(joint_names) != self.channels:
            raise ValueError(f"joint_names {joint_names} must have {self.channels} entries")
        self.joint_names = joint_names

        # counts_per_rev: publish SI if the length matches the channel count and all > 0,
        # otherwise raw + a warning.
        cpr = [float(v) for v in self.get_parameter("counts_per_rev").value]
        if len(cpr) == self.channels and all(v > 0 for v in cpr):
            self.counts_per_rev = cpr
            self.publish_si = True
        else:
            self.counts_per_rev = None
            self.publish_si = False

        # Connect. The first transaction right after open can be noisy before the adapter
        # settles, so retry with ping.
        driver_cls = SingleMotorDriver if self.channels == 1 else DualMotorDriver
        self.driver = driver_cls.open(self.port, self.baudrate, slave_id=self.motor_id)
        for attempt in range(5):
            if self.driver.ping():
                break
            self.get_logger().warn(f"initial comms retry {attempt + 1}/5 ({self.port})")
            time.sleep(0.2)
        else:
            self.driver.close()
            raise RuntimeError(f"{self.port} initial communication failed — check baudrate / port / wiring")
        self.get_logger().info(
            f"opened: {self.port} @ {self.baudrate}, id={self.motor_id}, "
            f"type={self.device_type}, version={self.driver.get_version()}, "
            f"voltage={self.driver.get_voltage()}V"
        )
        self._apply_use_limit_sw()
        if bool(self.get_parameter("auto_enable").value):
            self.driver.enable()
            self.get_logger().info("enable() done (UI_COM=1 + START_STOP arm)")

        self._last_vel_time = None  # time of the last velocity command (monotonic ns)

        # subscriptions
        self.create_subscription(Float64MultiArray, "~/cmd_velocity", self._on_cmd_velocity, 10)
        self.create_subscription(Float64MultiArray, "~/cmd_position", self._on_cmd_position, 10)

        # publishers
        self._joint_pub = self.create_publisher(JointState, "~/joint_states", 10)
        self._diag_pub = self.create_publisher(DiagnosticArray, "~/diagnostics", 10)

        # timers
        rate = max(1.0, float(self.get_parameter("publish_rate").value))
        diag_rate = max(0.2, float(self.get_parameter("diag_rate").value))
        self.create_timer(1.0 / rate, self._publish_joint_states)
        self.create_timer(1.0 / diag_rate, self._publish_diagnostics)
        if self.command_timeout > 0:
            self.create_timer(min(0.1, self.command_timeout / 2.0), self._watchdog)

        # services
        self._make_service("~/enable", lambda: self.driver.enable())
        self._make_service("~/disable", lambda: self.driver.disable())
        self._make_service("~/stop", lambda: self.driver.stop())
        self._make_service("~/torque_off", self._svc_torque_off)
        self._make_service("~/brake", self._svc_brake)
        self._make_service("~/reset_alarm", lambda: self.driver.reset_alarm())
        self._make_service("~/reset_position", lambda: self.driver.reset_position())

        if self.publish_si:
            self.get_logger().info(
                f"joint_states units=SI (rad, rad/s), counts_per_rev={self.counts_per_rev}"
            )
        else:
            self.get_logger().warn(
                "joint_states units=raw (position=count, velocity=rpm). "
                f"To publish SI (rad), set counts_per_rev to {self.channels} positive value(s) "
                "(e.g. counts_per_rev:=[24.0]). Measure it with examples/calibrate_counts_per_rev.py."
            )
        self.get_logger().info("mdrobot_motor_driver ready")

    def _apply_use_limit_sw(self) -> None:
        """Apply PID_USE_LIMIT_SW (and the dual PID 29) per the use_limit_sw parameter.

        If -1, leave the device setting untouched and just log the current value.
        """
        want = int(self.get_parameter("use_limit_sw").value)
        try:
            cur = self.driver.client.read_register(reg.PID_USE_LIMIT_SW)
        except MdrobotError:
            cur = None
        if want < 0:
            self.get_logger().info(f"USE_LIMIT_SW left as-is (current={cur})")
            return
        val = 1 if want else 0
        try:
            self.driver.client.write_register(reg.PID_USE_LIMIT_SW, val)
            if self.channels == 2:
                self.driver.client.write_register(reg.PID_USE_LIMIT_SW2, val)
            self.get_logger().info(f"USE_LIMIT_SW {cur} -> {val} (param use_limit_sw={want})")
        except MdrobotError as exc:
            self.get_logger().error(f"failed to set USE_LIMIT_SW: {type(exc).__name__}: {exc}")

    # --- service helpers ---------------------------------------------------------------
    def _make_service(self, name: str, action) -> None:
        def cb(_req, resp):
            try:
                action()
                resp.success = True
                resp.message = "ok"
            except MdrobotError as exc:
                resp.success = False
                resp.message = f"{type(exc).__name__}: {exc}"
                self.get_logger().error(f"{name} failed: {resp.message}")
            return resp

        self.create_service(Trigger, name, cb)

    def _svc_torque_off(self) -> None:
        if self.channels == 1:
            self.driver.torque_off()
        else:
            self.driver.torque_off_both()

    def _svc_brake(self) -> None:
        if self.channels == 1:
            self.driver.brake()
        else:
            self.driver.brake_both()

    # --- command callbacks -------------------------------------------------------------
    def _on_cmd_velocity(self, msg: Float64MultiArray) -> None:
        data = list(msg.data)
        if len(data) != self.channels:
            self.get_logger().warn(f"cmd_velocity needs {self.channels} value(s), got: {data}")
            return
        try:
            if self.channels == 1:
                self.driver.set_velocity(int(round(data[0])))
            else:
                self.driver.set_velocities(int(round(data[0])), int(round(data[1])))
            self._last_vel_time = self.get_clock().now().nanoseconds
        except MdrobotError as exc:
            self.get_logger().error(f"cmd_velocity failed: {type(exc).__name__}: {exc}")

    def _on_cmd_position(self, msg: Float64MultiArray) -> None:
        data = list(msg.data)
        if len(data) != self.channels:
            self.get_logger().warn(f"cmd_position needs {self.channels} value(s), got: {data}")
            return
        try:
            if self.channels == 1:
                self.driver.move_to(int(round(data[0])), self.position_max_rpm)
            else:
                self.driver.move_to_both(
                    int(round(data[0])), int(round(data[1])), self.position_max_rpm
                )
            self._last_vel_time = None  # entering position mode clears the velocity watchdog
        except MdrobotError as exc:
            self.get_logger().error(f"cmd_position failed: {type(exc).__name__}: {exc}")

    # --- watchdog ----------------------------------------------------------------------
    def _watchdog(self) -> None:
        if self._last_vel_time is None:
            return
        elapsed = (self.get_clock().now().nanoseconds - self._last_vel_time) / 1e9
        if elapsed >= self.command_timeout:
            try:
                self.driver.stop()
            except MdrobotError as exc:
                self.get_logger().error(f"watchdog stop failed: {exc}")
            self._last_vel_time = None
            self.get_logger().warn(f"command_timeout ({self.command_timeout}s) exceeded -> stop")

    # --- publishers --------------------------------------------------------------------
    def _publish_joint_states(self) -> None:
        if not rclpy.ok():  # do not publish to an invalid context during shutdown
            return
        try:
            mon = self.driver.read_monitor()
        except MdrobotError as exc:
            self.get_logger().warn(f"monitor read failed: {type(exc).__name__}: {exc}", throttle_duration_sec=2.0)
            return
        if self.channels == 1:
            counts = [mon.position]
            rpms = [mon.speed_rpm]
        else:
            counts = [mon.motor1.position, mon.motor2.position]
            rpms = [mon.motor1.speed_rpm, mon.motor2.speed_rpm]

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(self.joint_names)
        if self.publish_si:
            # Only position needs counts_per_rev; speed is rpm->rad/s regardless of source.
            js.position = [counts_to_rad(c, cpr) for c, cpr in zip(counts, self.counts_per_rev)]
            js.velocity = [rpm_to_rad_s(r) for r in rpms]
        else:
            js.position = [float(c) for c in counts]
            js.velocity = [float(r) for r in rpms]
        try:
            self._joint_pub.publish(js)
        except Exception:  # noqa: BLE001 - ignore invalid-context races during shutdown teardown
            pass

    def _publish_diagnostics(self) -> None:
        if not rclpy.ok():
            return
        status = DiagnosticStatus()
        status.name = "mdrobot/motor_driver"
        status.hardware_id = f"{self.port}#{self.motor_id}"
        try:
            voltage = self.driver.get_voltage()
            bits = self.driver.get_status()
            status.level = DiagnosticStatus.ERROR if bits.alarm else DiagnosticStatus.OK
            status.message = "ALARM" if bits.alarm else "OK"
            status.values = [
                KeyValue(key="voltage_V", value=f"{voltage:.1f}"),
                KeyValue(key="status1", value=",".join(bits.active) or "none"),
                KeyValue(key="device_type", value=self.device_type),
            ]
        except MdrobotError as exc:
            status.level = DiagnosticStatus.ERROR
            status.message = f"read fail: {type(exc).__name__}"
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [status]
        try:
            self._diag_pub.publish(arr)
        except Exception:  # noqa: BLE001 - ignore invalid-context races during shutdown teardown
            pass

    # --- shutdown ----------------------------------------------------------------------
    def _retry_on_shutdown(self, what: str, action, attempts: int = 3) -> bool:
        """Run a stop action, retrying a desynced frame. Never raises.

        A SIGINT can land while the publish timer is mid-transaction, so the tail of
        that response is still on the wire. `_transact` flushes the input buffer before
        every request, but bytes arriving *after* that flush get parsed as the head of
        the next response (IncompleteResponseError / CrcError) even though the link is
        healthy. Sleeping lets the stale frame finish arriving; the next attempt's flush
        then clears it and the frames realign.
        """
        last = None
        for attempt in range(attempts):
            try:
                action()
                return True
            except Exception as exc:  # noqa: BLE001 - shutdown must always proceed
                last = exc
                if attempt + 1 < attempts:
                    time.sleep(0.05)
        self.get_logger().error(
            f"shutdown {what} failed after {attempts} tries: {type(last).__name__}: {last}"
        )
        return False

    def shutdown(self) -> None:
        """Stop (stop + torque_off), then close the port. Absorb all exceptions so a
        wedged serial line cannot block shutdown (write_timeout keeps serial writes from
        blocking forever)."""
        try:
            # Independent attempts: torque_off stops the motor on its own (it coasts),
            # so a failing stop() must not take it down with it.
            stopped = self._retry_on_shutdown("stop", self.driver.stop)
            freed = self._retry_on_shutdown("torque_off", self._svc_torque_off)
            done = [n for n, ok in (("stop", stopped), ("torque_off", freed)) if ok]
            if done:
                self.get_logger().info(f"shutdown: {' + '.join(done)}")
            else:
                self.get_logger().error(
                    "SHUTDOWN COULD NOT STOP THE MOTOR — cut power / e-stop, "
                    "or reconnect and stop it"
                )
        finally:
            try:
                self.driver.close()
            except Exception:  # noqa: BLE001
                pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = MotorDriverNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Normal exit on Ctrl-C (SIGINT) or SIGTERM — finish without a traceback.
        pass
    except RuntimeError:
        # A signal tears the context down while a timer callback may already be
        # inside publish(), and rclpy raises RCLError -- a RuntimeError with no
        # public import path -- straight out of spin(). That is the shutdown
        # arriving, not a fault: without this the process exits 1 on a clean
        # stop and launch reports "process has died", which sends you looking
        # for a crash that never happened. Anything raised while the context is
        # still up is a real error and goes on up.
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
