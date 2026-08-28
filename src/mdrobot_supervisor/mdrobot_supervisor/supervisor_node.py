#!/usr/bin/env python3
"""The decision layer: operator input in, drive and equipment commands out.

    transmitter --RF--> STM32 board --> rc_bridge_node --> THIS NODE
                                             ^                 |
                     equipment  ~/command ---+                 |
                                                               v
                                        two motor_driver_node instances (MD, ttyUSB0)

Everything the machine does is decided here. The bridge below only reports and
relays; the motor driver below only turns rpm into RS485 writes.

Drive
-----
Four mecanum wheels on two dual-channel MD controllers. Operator steer/throttle
become a body twist, the twist becomes four wheel speeds, and those are split
across the two controllers by the wheel map.

Throttle always means forward/back. What the steer stick means is the whole
difference between the two manual modes — see Modes below.

Wheel speeds are clamped by scaling all four together, never per wheel: the
kinematics is linear, so uniform scaling is exactly "the same path, slower",
while clipping wheels one by one bends a straight line into an arc.

Interface
---------
Parameters (see config/supervisor.yaml for the full annotated set):
  rate (float=50.0)          Hz; decision and publish rate
  rc_timeout (float=0.3)     s without RC before everything is commanded to stop
  max_linear_x/max_linear_y/max_angular_z  what full stick deflection asks for.
                             max_linear_y is used by mecanum mode, max_angular_z
                             by base mode
  max_motor_rpm (float=600)  the hard cap, applied at the wheel
  wheel_radius/track/wheelbase/gear_ratio/roller_layout   base geometry
  wheel_slave_ids/wheel_channels/wheel_signs  where each wheel lives, in
                             front_left, front_right, rear_left, rear_right order
  lift_input (str='tristate') how the operator's lift channel reads:
                             'tristate' (-1/0/+1) or 'pwm' (a pulse width). It
                             was never observed moving, so 'tristate' is an
                             assumption; a tristate channel reporting anything
                             else holds lift at 0 rather than guessing
  limit_gating (bool=False)  stop lift at the limit switches. OFF by default
                             because the switch polarity is not yet known — a
                             gate with the polarity backwards either blocks lift
                             forever or never fires. Turn it on once measured.
  limit_active_value (int=1) what a TRIGGERED limit switch reports

Subscriptions:
  ~/rc (std_msgs/Int32MultiArray)   the bridge's ~/channels, ten operator channels
  ~/plate_offset (geometry_msgs/Point)  plate position from mdrobot_plate_ocr;
                                    x normalised [-1, 1], positive = right of centre
  ~/hole_offset (geometry_msgs/Point)   drilled hole in the upward camera's frame,
                                    x/y normalised [-1, 1]. No publisher yet
  ~/joint_states_1, ~/joint_states_2 (sensor_msgs/JointState)  wheel positions,
                                    for the blind entry distance

Publishers:
  ~/cmd_velocity_1, ~/cmd_velocity_2 (std_msgs/Float64MultiArray)
      [channel1_rpm, channel2_rpm] for each MD controller, motor-shaft rpm
  ~/command (std_msgs/Int32MultiArray)
      [lift, brake, drill, actuator, solenoid] for the bridge to relay
  ~/mode (std_msgs/String)
      which of base / mecanum / autonomous the operator's switch selects
  ~/auto_phase (std_msgs/String)
      the sequence's current phase, empty when not in autonomous
  ~/diagnostics (diagnostic_msgs/DiagnosticArray)

Modes
-----
The operator's three-position switch reports -1, 0 or +1:

  -1  base        manual driving the conventional way: throttle is forward/back,
                  steer YAWS the machine left and right
   0  mecanum     throttle is forward/back as before, but steer STRAFES — pull
                  left and the machine slides left without changing heading
   1  autonomous  runs the whole sequence in autonomous.py: wait for a plate,
                  creep forward while strafing onto it, keep going blind once it
                  drops out of view, DRILL, then find the hole with the upward
                  camera, line the actuator up under it, raise it and SPRAY

Autonomous drives the machine under a car and runs a drill with no further
operator input. Selecting the mode is the arming action. It aborts on brake, on
losing the RC link, and if wheel odometry is missing — without encoders there is
no way to know how far under the car it has gone. An abort is terminal: the
operator has to leave autonomous and come back, which is the deliberate act that
should be needed to re-arm a drill.

Subscriptions it needs: ~/plate_offset from mdrobot_plate_ocr,
~/joint_states_1 / ~/joint_states_2 from the two motor drivers, and ~/hole_offset
from an upward-facing camera — which does not exist yet, so the sequence stops
at find_hole and times out.

Safety
------
Nothing here replaces the board's own failsafe or a physical e-stop. What the
node does guarantee: with rc_timeout exceeded, drive goes to zero rpm and
equipment to idle; a brake command zeroes drive in the same tick it is seen;
out-of-range values never reach the hardware, because the bridge clamps them.
"""

from __future__ import annotations

import math
import threading
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Int32MultiArray, String

from mdrobot_rc_bridge.rc_reader import CHANNEL_NAMES, NUM_CHANNELS
from mdrobot_supervisor.autonomous import (
    AutonomousConfig,
    AutonomousSequence,
    Observation,
    TERMINAL,
)
from mdrobot_supervisor.kinematics import (
    MecanumGeometry,
    WHEEL_NAMES,
    inverse,
    rad_s_to_motor_rpm,
    scale_to_limit,
)

CH = {name: i for i, name in enumerate(CHANNEL_NAMES)}
IDLE_COMMAND = [0, 0, 0, 0, 0]  # lift, brake, drill, actuator, solenoid


class SupervisorNode(Node):
    def __init__(self) -> None:
        super().__init__("mdrobot_supervisor")

        self.declare_parameter("rate", 50.0)
        self.declare_parameter("rc_timeout", 0.3)
        self.declare_parameter("pwm_min", 1000)
        self.declare_parameter("pwm_mid", 1500)
        self.declare_parameter("pwm_max", 2000)
        self.declare_parameter("deadband", 15)
        self.declare_parameter("invert_steer", False)
        self.declare_parameter("invert_throttle", False)
        self.declare_parameter("max_linear_x", 0.2)
        self.declare_parameter("max_linear_y", 0.2)
        self.declare_parameter("max_angular_z", 0.37)
        self.declare_parameter("max_motor_rpm", 600.0)
        self.declare_parameter("wheel_radius", 0.0625)
        self.declare_parameter("track", 0.575)
        self.declare_parameter("wheelbase", 0.5)
        self.declare_parameter("gear_ratio", 20.0)
        self.declare_parameter("roller_layout", "unknown")
        self.declare_parameter("wheel_slave_ids", [1, 1, 2, 2])
        self.declare_parameter("wheel_channels", [1, 2, 2, 1])
        self.declare_parameter("wheel_signs", [-1, 1, -1, 1])
        self.declare_parameter("lift_speed", 60)
        self.declare_parameter("lift_input", "tristate")
        self.declare_parameter("limit_gating", False)
        self.declare_parameter("limit_active_value", 1)
        self.declare_parameter("mode_names", ["base", "mecanum", "autonomous"])
        # Wheel odometry. 0.0 means ~/joint_states already carries radians, which
        # is what the driver publishes once its own counts_per_rev is set.
        self.declare_parameter("counts_per_rev", 0.0)
        self.declare_parameter("auto_plate_timeout", 0.5)
        self.declare_parameter("auto_align_gain", 0.4)
        self.declare_parameter("auto_align_tolerance", 0.08)
        self.declare_parameter("auto_approach_speed", 0.08)
        self.declare_parameter("auto_entry_distance", 1.2)
        self.declare_parameter("auto_entry_speed", 0.08)
        self.declare_parameter("auto_drill_seconds", 5.0)
        self.declare_parameter("auto_max_align_seconds", 60.0)
        self.declare_parameter("auto_max_entry_seconds", 60.0)
        self.declare_parameter("auto_hole_timeout", 0.5)
        self.declare_parameter("auto_hole_target_x", 0.0)
        self.declare_parameter("auto_hole_target_y", 0.0)
        self.declare_parameter("auto_hole_tolerance", 0.05)
        self.declare_parameter("auto_hole_gain_x", -0.3)
        self.declare_parameter("auto_hole_gain_y", -0.3)
        self.declare_parameter("auto_hole_max_speed", 0.05)
        self.declare_parameter("auto_actuator_seconds", 3.0)
        self.declare_parameter("auto_spray_seconds", 10.0)
        self.declare_parameter("auto_max_find_hole_seconds", 30.0)
        self.declare_parameter("auto_max_hole_align_seconds", 60.0)
        # Odometry sanity: the drivers publish raw counts unless their own
        # counts_per_rev is set, and counts read as radians look like enormous
        # travel. Abort if the measured speed exceeds what was ever commanded by
        # more than this factor.
        self.declare_parameter("odom_max_speed_factor", 4.0)

        self.rc_timeout = float(self.get_parameter("rc_timeout").value)
        self.pwm_min = int(self.get_parameter("pwm_min").value)
        self.pwm_mid = int(self.get_parameter("pwm_mid").value)
        self.pwm_max = int(self.get_parameter("pwm_max").value)
        self.deadband = int(self.get_parameter("deadband").value)
        if not self.pwm_min < self.pwm_mid < self.pwm_max:
            raise ValueError(
                f"need pwm_min < pwm_mid < pwm_max, got "
                f"{self.pwm_min} / {self.pwm_mid} / {self.pwm_max}"
            )
        self.invert_steer = bool(self.get_parameter("invert_steer").value)
        self.invert_throttle = bool(self.get_parameter("invert_throttle").value)
        self.max_linear_x = float(self.get_parameter("max_linear_x").value)
        self.max_linear_y = float(self.get_parameter("max_linear_y").value)
        self.max_angular_z = float(self.get_parameter("max_angular_z").value)
        self.max_motor_rpm = float(self.get_parameter("max_motor_rpm").value)
        self.gear_ratio = float(self.get_parameter("gear_ratio").value)
        if self.gear_ratio <= 0:
            raise ValueError(f"gear_ratio must be positive, got {self.gear_ratio}")

        self.geom = MecanumGeometry(
            wheel_radius=float(self.get_parameter("wheel_radius").value),
            track=float(self.get_parameter("track").value),
            wheelbase=float(self.get_parameter("wheelbase").value),
            roller_layout=str(self.get_parameter("roller_layout").value),
        )

        self.wheel_slave_ids = [int(v) for v in self.get_parameter("wheel_slave_ids").value]
        self.wheel_channels = [int(v) for v in self.get_parameter("wheel_channels").value]
        self.wheel_signs = [int(v) for v in self.get_parameter("wheel_signs").value]
        self._validate_wheel_map()

        self.lift_speed = int(self.get_parameter("lift_speed").value)
        self.lift_input = str(self.get_parameter("lift_input").value).lower()
        if self.lift_input not in ("tristate", "pwm"):
            raise ValueError(
                f"lift_input must be 'tristate' or 'pwm', got {self.lift_input!r}"
            )
        self.limit_gating = bool(self.get_parameter("limit_gating").value)
        self.limit_active_value = int(self.get_parameter("limit_active_value").value)
        self.mode_names = [str(n) for n in self.get_parameter("mode_names").value]
        if len(self.mode_names) != 3:
            raise ValueError(f"mode_names needs 3 entries, got {self.mode_names}")
        # The switch reports -1, 0, +1 and mode_names lists them in that order.
        self.mode_base, self.mode_mecanum, self.mode_autonomous = self.mode_names

        self.counts_per_rev = float(self.get_parameter("counts_per_rev").value)
        if self.counts_per_rev < 0:
            raise ValueError(f"counts_per_rev cannot be negative, got {self.counts_per_rev}")
        self.auto_config = AutonomousConfig(
            plate_timeout=float(self.get_parameter("auto_plate_timeout").value),
            align_gain=float(self.get_parameter("auto_align_gain").value),
            align_tolerance=float(self.get_parameter("auto_align_tolerance").value),
            approach_speed=float(self.get_parameter("auto_approach_speed").value),
            entry_distance=float(self.get_parameter("auto_entry_distance").value),
            entry_speed=float(self.get_parameter("auto_entry_speed").value),
            drill_seconds=float(self.get_parameter("auto_drill_seconds").value),
            max_align_seconds=float(self.get_parameter("auto_max_align_seconds").value),
            max_entry_seconds=float(self.get_parameter("auto_max_entry_seconds").value),
            hole_timeout=float(self.get_parameter("auto_hole_timeout").value),
            hole_target_x=float(self.get_parameter("auto_hole_target_x").value),
            hole_target_y=float(self.get_parameter("auto_hole_target_y").value),
            hole_tolerance=float(self.get_parameter("auto_hole_tolerance").value),
            hole_gain_x=float(self.get_parameter("auto_hole_gain_x").value),
            hole_gain_y=float(self.get_parameter("auto_hole_gain_y").value),
            hole_max_speed=float(self.get_parameter("auto_hole_max_speed").value),
            actuator_seconds=float(self.get_parameter("auto_actuator_seconds").value),
            spray_seconds=float(self.get_parameter("auto_spray_seconds").value),
            max_find_hole_seconds=float(
                self.get_parameter("auto_max_find_hole_seconds").value),
            max_hole_align_seconds=float(
                self.get_parameter("auto_max_hole_align_seconds").value),
        )
        self.odom_max_speed_factor = float(self.get_parameter("odom_max_speed_factor").value)
        if self.odom_max_speed_factor <= 1.0:
            raise ValueError(
                f"odom_max_speed_factor must exceed 1, got {self.odom_max_speed_factor}"
            )
        self.sequence = AutonomousSequence(self.auto_config)

        self.pub_drive = [
            self.create_publisher(Float64MultiArray, "~/cmd_velocity_1", 10),
            self.create_publisher(Float64MultiArray, "~/cmd_velocity_2", 10),
        ]
        self.pub_command = self.create_publisher(Int32MultiArray, "~/command", 10)
        self.pub_mode = self.create_publisher(String, "~/mode", 10)
        self.pub_diag = self.create_publisher(DiagnosticArray, "~/diagnostics", 1)
        self.pub_phase = self.create_publisher(String, "~/auto_phase", 10)
        self.create_subscription(Int32MultiArray, "~/rc", self._on_rc, 10)
        self.create_subscription(Point, "~/plate_offset", self._on_plate, 10)
        self.create_subscription(Point, "~/hole_offset", self._on_hole, 10)
        self.create_subscription(
            JointState, "~/joint_states_1", lambda m: self._on_joints(0, m), 10)
        self.create_subscription(
            JointState, "~/joint_states_2", lambda m: self._on_joints(1, m), 10)

        self._lock = threading.Lock()
        self._rc: list[int] | None = None
        self._rc_wall = 0.0
        self._stopped = True  # nothing has been commanded yet
        self._last_mode = ""
        self._clamp_k = 1.0
        self._rejected = 0
        self._plate_x: float | None = None
        self._plate_wall = 0.0
        self._hole_x: float | None = None
        self._hole_y: float | None = None
        self._hole_wall = 0.0
        # Previous odometry sample, for the plausibility check.
        self._odom_prev: tuple[float, float] | None = None
        # Motor-shaft position per controller, [channel1, channel2]; None until seen.
        self._joints: list[list[float] | None] = [None, None]
        self._was_autonomous = False
        self._last_phase = ""
        self._auto_actuator = 0
        self._auto_solenoid = 0

        self.create_timer(1.0 / float(self.get_parameter("rate").value), self._on_tick)
        self.create_timer(0.5, self._on_diag)

        self.get_logger().info(
            f"mecanum base r={self.geom.wheel_radius} track={self.geom.track} "
            f"wheelbase={self.geom.wheelbase} layout={self.geom.roller_layout} "
            f"gear={self.gear_ratio} cap={self.max_motor_rpm} rpm"
        )
        if self.geom.layout_is_provisional:
            self.get_logger().warn(
                "roller_layout is 'unknown': computed as 'x' so the base drives, but "
                "the STRAFE DIRECTION is unverified. Only a floor test settles it."
            )
        if not self.limit_gating:
            self.get_logger().warn(
                "limit_gating is off: limit switches are reported but do NOT stop the "
                "lift. Measure the switch polarity, set limit_active_value, then enable."
            )

    def _validate_wheel_map(self) -> None:
        for name, values in (("wheel_slave_ids", self.wheel_slave_ids),
                             ("wheel_channels", self.wheel_channels),
                             ("wheel_signs", self.wheel_signs)):
            if len(values) != 4:
                raise ValueError(f"{name} needs 4 entries {WHEEL_NAMES}, got {values}")
        if set(self.wheel_signs) - {-1, 1}:
            raise ValueError(f"wheel_signs entries must be -1 or +1, got {self.wheel_signs}")
        if set(self.wheel_channels) - {1, 2}:
            raise ValueError(f"wheel_channels entries must be 1 or 2, got {self.wheel_channels}")
        slots = list(zip(self.wheel_slave_ids, self.wheel_channels))
        if len(set(slots)) != 4:
            raise ValueError(
                f"each wheel needs its own (slave_id, channel); got {slots}"
            )
        ids = sorted(set(self.wheel_slave_ids))
        if len(ids) != 2:
            raise ValueError(f"expected exactly 2 controllers, got slave ids {ids}")
        self.controller_ids = ids

    # ── input ───────────────────────────────────────────────────────────────
    def _on_rc(self, msg: Int32MultiArray) -> None:
        if len(msg.data) != NUM_CHANNELS:
            self._rejected += 1
            self.get_logger().warn(
                f"ignoring ~/rc with {len(msg.data)} channels, expected {NUM_CHANNELS}",
                throttle_duration_sec=2.0,
            )
            return
        with self._lock:
            self._rc = [int(v) for v in msg.data]
            self._rc_wall = time.monotonic()

    def _on_plate(self, msg: Point) -> None:
        """Latch where the plate sits, as a normalised error signal."""
        self._plate_x = float(msg.x)
        self._plate_wall = time.monotonic()

    def _on_hole(self, msg: Point) -> None:
        """Latch where the drilled hole sits in the upward camera's frame."""
        self._hole_x = float(msg.x)
        self._hole_y = float(msg.y)
        self._hole_wall = time.monotonic()

    def _on_joints(self, controller: int, msg: JointState) -> None:
        """Latch motor-shaft positions for one controller, in [ch1, ch2] order."""
        if len(msg.position) < 2:
            return  # a single-channel driver has nothing to say about four wheels
        self._joints[controller] = [float(msg.position[0]), float(msg.position[1])]

    def _distance(self) -> float | None:
        """Forward travel in metres, averaged over the four wheels.

        None until every wheel has reported. Positions come from the motor
        shaft, so the gear ratio divides out; wheel_signs undo the mirrored
        mounting so all four agree on which way is forward.
        """
        if any(j is None for j in self._joints):
            return None
        total = 0.0
        for i in range(4):
            controller = self.controller_ids.index(self.wheel_slave_ids[i])
            position = self._joints[controller][self.wheel_channels[i] - 1]
            if self.counts_per_rev > 0:
                # Driver is publishing raw counts; turn them into motor radians.
                position = position / self.counts_per_rev * 2.0 * math.pi
            total += position * self.wheel_signs[i]
        mean_motor_rad = total / 4.0
        return mean_motor_rad / self.gear_ratio * self.geom.wheel_radius

    def _axis(self, value: int, invert: bool) -> float:
        """Pulse width in microseconds -> -1..+1, with a deadband at centre."""
        if abs(value - self.pwm_mid) <= self.deadband:
            out = 0.0
        elif value >= self.pwm_mid:
            out = (value - self.pwm_mid) / (self.pwm_max - self.pwm_mid)
        else:
            out = (value - self.pwm_mid) / (self.pwm_mid - self.pwm_min)
        out = max(-1.0, min(1.0, out))
        return -out if invert else out

    def _mode_name(self, raw: int) -> str:
        # The switch reports -1, 0 or +1; anything else means "no reading".
        if raw in (-1, 0, 1):
            return self.mode_names[raw + 1]
        return "unknown"

    # ── decision ────────────────────────────────────────────────────────────
    def _twist(self, rc: list[int], mode: str, brake: int) -> tuple[float, float, float]:
        """Operator sticks -> body twist, for the two MANUAL modes.

        Throttle always means forward/back. What the steer stick means is the
        whole difference between them: in base it yaws the machine, in mecanum
        it slides it sideways without changing heading.
        """
        # Brake wins over the sticks in the same tick it is seen.
        if brake:
            return 0.0, 0.0, 0.0

        vx = self._axis(rc[CH["throttle"]], self.invert_throttle) * self.max_linear_x
        steer = self._axis(rc[CH["steer"]], self.invert_steer)
        if mode == self.mode_mecanum:
            # +y is LEFT, so pulling the stick right (positive) strafes right.
            return vx, -steer * self.max_linear_y, 0.0
        # Base mode, and anything unrecognised: yaw. +wz is counter-clockwise,
        # so a stick pulled right has to negate.
        return vx, 0.0, -steer * self.max_angular_z

    def _wheel_rpm(self, vx: float, vy: float, wz: float) -> tuple[list[float], float]:
        """Body twist -> motor rpm per wheel, capped by scaling all four together."""
        omega = inverse(vx, vy, wz, self.geom)
        rpm = [rad_s_to_motor_rpm(w, self.gear_ratio) for w in omega]
        rpm, k = scale_to_limit(rpm, self.max_motor_rpm)
        return [r * s for r, s in zip(rpm, self.wheel_signs)], k

    def _split_by_controller(self, wheel_rpm: list[float]) -> list[list[float]]:
        """Lay the four wheel speeds out as [channel1, channel2] per controller."""
        out = [[0.0, 0.0] for _ in self.controller_ids]
        for i, rpm in enumerate(wheel_rpm):
            controller = self.controller_ids.index(self.wheel_slave_ids[i])
            out[controller][self.wheel_channels[i] - 1] = rpm
        return out

    def _on_tick(self) -> None:
        now = time.monotonic()
        with self._lock:
            rc = self._rc
            age = now - self._rc_wall if self._rc_wall else None

        if rc is None or age is None or age > self.rc_timeout:
            if not self._stopped:
                self._stopped = True
                self.get_logger().warn(
                    "no ~/rc within rc_timeout; commanding stop and idle equipment"
                )
            if self._was_autonomous:
                self.sequence.reset()
                self._was_autonomous = False
                self._publish_phase("")
            self._publish(self._split_by_controller([0.0] * 4), IDLE_COMMAND, "unknown")
            self._clamp_k = 1.0
            return

        if self._stopped:
            self._stopped = False
            self.get_logger().info("~/rc live; resuming")

        mode = self._mode_name(rc[CH["mode"]])
        brake = 1 if rc[CH["brake"]] else 0
        auto_drill = 0

        self._auto_actuator = 0
        self._auto_solenoid = 0
        if mode == self.mode_autonomous:
            vx, vy, wz, auto_drill = self._autonomous(now, brake)
        else:
            if self._was_autonomous:
                # Leaving autonomous re-arms it: the sequence restarts from
                # scratch next time, so a finished or aborted run never resumes
                # on its own.
                self.sequence.reset()
                self._was_autonomous = False
                self._publish_phase("")
            vx, vy, wz = self._twist(rc, mode, brake)

        wheel_rpm, k = self._wheel_rpm(vx, vy, wz)
        self._clamp_k = k

        lift = self._gated_lift(rc)
        command = [
            lift,
            brake,
            # The operator's drill switch and the sequence's request are both
            # honoured; either one alone turns it on.
            1 if (rc[CH["drill"]] or auto_drill) else 0,
            # The operator can always drive the actuator; the sequence takes it
            # only when the operator has left it alone.
            max(-1, min(1, rc[CH["actuator"]] or self._auto_actuator)),
            1 if (rc[CH["solenoid"]] or self._auto_solenoid) else 0,
        ]
        self._publish(self._split_by_controller(wheel_rpm), command, mode)

    def _autonomous(self, now: float, brake: int) -> tuple[float, float, float, int]:
        """Run one tick of the approach sequence.

        Every hardware-facing guard lives here rather than in the state machine,
        so the machine stays pure: brake and missing odometry abort it, and the
        abort is terminal until the operator leaves autonomous and comes back.
        """
        if not self._was_autonomous:
            self.sequence.reset()
            self._was_autonomous = True
            self.get_logger().warn(
                "autonomous armed: the machine will drive itself under the vehicle "
                "and RUN THE DRILL. Brake or switch modes to stop it."
            )

        if brake:
            if self.sequence.phase not in TERMINAL:
                self.sequence.abort("brake pressed")
                self.get_logger().warn("autonomous aborted: brake")
            self._publish_phase(self.sequence.phase.value)
            return 0.0, 0.0, 0.0, 0

        distance = self._distance()
        if distance is None:
            # ENTER measures travel on the encoders. Without them the machine
            # would drive under the car with no idea how far it had gone, and
            # then drill wherever it happened to be.
            if self.sequence.phase not in TERMINAL:
                self.sequence.abort("no wheel odometry")
                self.get_logger().error(
                    "autonomous aborted: no ~/joint_states from both controllers, "
                    "so the entry distance cannot be measured"
                )
            self._publish_phase(self.sequence.phase.value)
            return 0.0, 0.0, 0.0, 0

        implausible = self._odometry_implausible(now, distance)
        if implausible is not None:
            if self.sequence.phase not in TERMINAL:
                self.sequence.abort(implausible)
                self.get_logger().error(f"autonomous aborted: {implausible}")
            self._publish_phase(self.sequence.phase.value)
            return 0.0, 0.0, 0.0, 0

        plate_age = (now - self._plate_wall) if self._plate_wall else None
        hole_age = (now - self._hole_wall) if self._hole_wall else None
        action = self.sequence.step(Observation(
            now=now,
            plate_offset_x=self._plate_x,
            plate_age=plate_age,
            distance=distance,
            hole_offset_x=self._hole_x,
            hole_offset_y=self._hole_y,
            hole_age=hole_age,
        ))
        if action.phase.value != self._last_phase:
            self.get_logger().info(
                f"autonomous: {action.phase.value} - {action.message}"
            )
        self._publish_phase(action.phase.value)
        self._auto_actuator = action.actuator
        self._auto_solenoid = action.solenoid
        return action.vx, action.vy, action.wz, action.drill

    def _odometry_implausible(self, now: float, distance: float) -> str | None:
        """Catch odometry that cannot be what it claims, and say why.

        The drivers publish raw encoder counts unless their own counts_per_rev
        is set. Counts read as radians look like tens of metres of travel per
        second, which would carry the sequence through ENTER in a single tick
        and drill on the spot. Anything moving faster than the machine was ever
        commanded to move is not a reading worth acting on.
        """
        previous = self._odom_prev
        self._odom_prev = (now, distance)
        if previous is None:
            return None
        dt = now - previous[0]
        if dt <= 0:
            return None
        speed = abs(distance - previous[1]) / dt
        ceiling = max(self.max_linear_x, self.max_linear_y) * self.odom_max_speed_factor
        if speed > ceiling:
            return (
                f"odometry reports {speed:.1f} m/s, over {ceiling:.1f} m/s. "
                f"The drivers are probably publishing raw counts: set "
                f"counts_per_rev on them, or on this node."
            )
        return None

    def _publish_phase(self, phase: str) -> None:
        self._last_phase = phase
        self.pub_phase.publish(String(data=phase))

    def _gated_lift(self, rc: list[int]) -> int:
        """Operator lift request, stopped at whichever limit switch is closed."""
        raw = rc[CH["lift"]]
        if self.lift_input == "pwm":
            fraction = self._axis(raw, False)
        elif -1 <= raw <= 1:
            fraction = float(raw)
        else:
            # Configured as a -1/0/+1 switch but reporting something else — most
            # likely it is really a pulse width. Refuse to move rather than
            # scaling a 1500 into full-speed lift.
            self.get_logger().error(
                f"lift channel reported {raw}, outside -1..1 with "
                f"lift_input='tristate'. Holding lift at 0 — set lift_input='pwm' "
                f"if the channel carries a pulse width.",
                throttle_duration_sec=5.0,
            )
            return 0
        lift = int(round(max(-1.0, min(1.0, fraction)) * self.lift_speed))
        if not self.limit_gating:
            return lift
        at_top = rc[CH["limit_up"]] == self.limit_active_value
        at_bottom = rc[CH["limit_down"]] == self.limit_active_value
        if lift > 0 and at_top:
            return 0
        if lift < 0 and at_bottom:
            return 0
        return lift

    # ── output ──────────────────────────────────────────────────────────────
    def _publish(self, per_controller: list[list[float]],
                 command: list[int], mode: str) -> None:
        for pub, values in zip(self.pub_drive, per_controller):
            msg = Float64MultiArray()
            msg.data = [float(v) for v in values]
            pub.publish(msg)

        cmd = Int32MultiArray()
        cmd.data = [int(v) for v in command]
        self.pub_command.publish(cmd)

        self.pub_mode.publish(String(data=mode))
        if mode != self._last_mode:
            self.get_logger().info(f"mode -> {mode}")
            self._last_mode = mode

    def _on_diag(self) -> None:
        with self._lock:
            rc = self._rc
            age = time.monotonic() - self._rc_wall if self._rc_wall else None

        distance = self._distance()
        status = DiagnosticStatus()
        status.name = "mdrobot_supervisor: decision"
        status.hardware_id = "mecanum"
        if age is None:
            status.level = DiagnosticStatus.ERROR
            status.message = "no ~/rc received yet"
        elif age > self.rc_timeout:
            status.level = DiagnosticStatus.ERROR
            status.message = f"~/rc stale for {age:.2f} s; stopped"
        elif self._clamp_k < 1.0:
            status.level = DiagnosticStatus.WARN
            status.message = f"wheel speeds scaled to {self._clamp_k:.2f} of request"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "ok"
        status.values = [
            KeyValue(key="mode", value=self._last_mode or "unknown"),
            KeyValue(key="clamp_k", value=f"{self._clamp_k:.3f}"),
            KeyValue(key="rc_rejected", value=str(self._rejected)),
            KeyValue(key="limit_gating", value=str(self.limit_gating)),
            KeyValue(key="roller_layout", value=self.geom.roller_layout),
            KeyValue(key="auto_phase", value=self._last_phase or "-"),
            KeyValue(key="distance_m", value=(
                "n/a" if distance is None else f"{distance:.3f}")),
        ]
        if rc is not None:
            status.values += [KeyValue(key=n, value=str(v))
                              for n, v in zip(CHANNEL_NAMES, rc)]

        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [status]
        self.pub_diag.publish(msg)


def main() -> None:
    rclpy.init()
    node = SupervisorNode()
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
