#!/usr/bin/env python3
"""The decision layer: operator input in, drive and equipment commands out.

    transmitter --RF--> STM32 board --> rc_bridge_node --> THIS NODE
                                             ^                 |
                     equipment  ~/command ---+                 |
                                                               v
                                        mecanum_driver_node (MD x2 on one RS485 bus)

Everything the machine does is decided here. The bridge below only reports and
relays; the motor driver below only turns rpm into RS485 writes.

Drive
-----
Four mecanum wheels. Operator steer/throttle become a body twist and the twist
becomes four wheel speeds, published as one vector. Which controller and channel
each wheel hangs off is the drive node's business.

Throttle always means forward/back. What the steer stick means is the whole
difference between the two manual modes — see Modes below.

Wheel speeds are clamped by scaling all four together, never per wheel: the
kinematics is linear, so uniform scaling is exactly "the same path, slower",
while clipping wheels one by one bends a straight line into an arc.

Interface
---------
Parameters (see config/supervisor.yaml for the full annotated set):
  rate (float=50.0)          Hz; decision and publish rate
  rc_timeout (float=0.3)     s without RC before the wheels are commanded to stop
  rc_grace (float=5.0)       extra s of RC silence a running sequence rides out
                             before it aborts, and that manual keeps holding the
                             equipment the operator's switch is still asking for.
                             The wheels stop at rc_timeout regardless. Recorded
                             gaps run 3.3-3.5 s, arriving 10.8 s after the
                             actuator starts, so aborting on one threw away
                             every run about a second into the spray.
  rc_grace_stationary        the same for phases that command no wheel motion,
    (float=20.0)             where the machine is parked with the drill in a
                             hole and the link says nothing about whether it is
                             safe to keep holding.
  max_linear_x/max_linear_y/max_angular_z  what full stick deflection asks for.
                             max_linear_y is used by mecanum mode, max_angular_z
                             by base mode
  max_motor_rpm (float=600)  the hard cap, applied at the wheel
  wheel_radius/track/wheelbase/gear_ratio/roller_layout   base geometry
  wheel_signs (int[])        +1 when a POSITIVE rpm drives that wheel forward, in
                             front_left, front_right, rear_left, rear_right order.
                             Which controller and channel each wheel hangs off is
                             the driver node's business, not this one's
  lift_input (str='speed')   how the operator's lift channel reads: 'speed' (the
                             -60..60 the board actually sends, confirmed on the
                             hardware), 'tristate' (-1/0/+1), or 'pwm'. A
                             tristate channel reporting anything else holds lift
                             at 0 rather than guessing
  wheel_position_units (str='unset')  what ~/joint_states carries: 'rad' or
                             'count' (with counts_per_rev). Autonomous refuses to
                             run while this is 'unset', because counts taken for
                             radians would end the blind entry in a single tick
  lift_max_run (float=10.0)     seconds the lift may drive one way before it is
                             stopped. 0 disables
  actuator_max_run (float=8.0)  the same for the actuator. Neither has limit
                             switches or feedback, so holding the switch parks
                             them stalled against an end stop, and a stalled
                             motor's current draw is enough to brown out a
                             controller on this supply
  limit_gating (bool=False)  stop the lift at the limit switches. OFF because
                             the switches are not fitted yet; with it off nothing
                             knows where the travel ends and the carriage drives
                             into its hard stop. Turn it on once they are wired.
  limit_active_value (int=1) what a TRIGGERED limit switch reports. Both
                             switches reading it at once is impossible, so that
                             is treated as a wiring or polarity fault and the
                             lift is held at 0.
  limit_active_value (int=1) what a TRIGGERED limit switch reports

Subscriptions:
  ~/rc (std_msgs/Int32MultiArray)   the bridge's ~/channels, ten operator channels
  ~/plate_offset (geometry_msgs/Point)  plate position from mdrobot_plate_ocr;
                                    x normalised [-1, 1], positive = right of centre
  ~/hole_offset (geometry_msgs/Point)   drilled hole in the upward camera's frame,
                                    x/y normalised [-1, 1]. No publisher yet
  ~/joint_states (sensor_msgs/JointState)  four wheel positions, for the blind
                                    entry distance
  ~/imu (sensor_msgs/Imu)           attitude from mdrobot_imu. Only the heading
                                    is used, and only the change in it since a
                                    reference — what the sensor calls zero does
                                    not matter. Required when auto_yaw_hold

Publishers:
  ~/cmd_wheel_rpm (std_msgs/Float64MultiArray)
      [front_left, front_right, rear_left, rear_right] motor-shaft rpm
  ~/command (std_msgs/Int32MultiArray)
      [lift, brake, drill, actuator, solenoid] for the bridge to relay
  ~/mode (std_msgs/String)
      which of base / mecanum / autonomous the operator's switch selects
  ~/auto_phase (std_msgs/String)
      the sequence's current phase, empty when not in autonomous
  ~/auto_detail (std_msgs/String)
      the same phase plus the twist it is commanding and its own account of
      why, every tick. This is what tells a strafe onto a plate the camera saw
      move apart from a turn the heading hold asked for: one is vy, the other
      is wz. Echo it while an approach runs.
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

Subscriptions it needs: ~/plate_offset from mdrobot_plate_ocr and ~/joint_states
from the drive node. The hole stage additionally needs ~/hole_offset from an
upward-facing camera, which is not fitted — auto_hole_stage is off by default and
the sequence finishes at the drill.

auto_yaw_hold adds ~/imu from mdrobot_imu. It is off by default, and with it off
nothing about the sequence changes. On, the machine cancels the yaw that wheel
slip gives it while it is driving blind and while it is shuffling under the car,
and STOPS — rather than correcting harder — if the heading runs away, goes stale
or disappears. It never corrects while the bit is in the hole; there it only
watches, to a much tighter limit, because turning a machine with a drill engaged
is what breaks the bit.

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
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray, Int32MultiArray, String

from mdrobot_rc_bridge.rc_reader import CHANNEL_NAMES, NUM_CHANNELS
from mdrobot_supervisor.autonomous import (
    AutonomousConfig,
    AutonomousSequence,
    Observation,
    STATIONARY,
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
        self.declare_parameter("rc_grace", 5.0)
        self.declare_parameter("rc_grace_stationary", 20.0)
        self.declare_parameter("pwm_min", 1000)
        self.declare_parameter("pwm_mid", 1500)
        self.declare_parameter("pwm_max", 2000)
        self.declare_parameter("deadband", 15)
        self.declare_parameter("pwm_tolerance", 200)
        self.declare_parameter("require_neutral_start", True)
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
        self.declare_parameter("wheel_signs", [-1, 1, -1, 1])
        self.declare_parameter("lift_speed", 60)
        self.declare_parameter("lift_input", "speed")
        self.declare_parameter("lift_max_run", 15.0)
        self.declare_parameter("actuator_max_run", 10.0)
        self.declare_parameter("limit_gating", False)
        self.declare_parameter("limit_active_value", 1)
        self.declare_parameter("mode_names", ["base", "mecanum", "autonomous"])
        # Wheel odometry. 0.0 means ~/joint_states already carries radians, which
        # is what the driver publishes once its own counts_per_rev is set.
        self.declare_parameter("counts_per_rev", 0.0)
        self.declare_parameter("wheel_position_units", "unset")
        self.declare_parameter("auto_plate_timeout", 0.5)
        self.declare_parameter("auto_min_approach_width", 0.45)
        self.declare_parameter("auto_plate_shrink_ratio", 0.5)
        self.declare_parameter("auto_align_gain", 0.4)
        self.declare_parameter("auto_align_max_speed", 0.10)
        self.declare_parameter("auto_align_tolerance", 0.08)
        self.declare_parameter("auto_approach_speed", 0.05)
        self.declare_parameter("auto_entry_distance", 0.3)
        self.declare_parameter("auto_entry_speed", 0.05)
        self.declare_parameter("auto_drill_seconds", 20.0)
        self.declare_parameter("auto_lift_up_seconds", 10.0)
        self.declare_parameter("auto_lift_down_seconds", 10.0)
        self.declare_parameter("auto_retract_seconds", 12.0)
        self.declare_parameter("auto_max_align_seconds", 60.0)
        self.declare_parameter("auto_max_entry_seconds", 60.0)
        # Yaw hold, off ~/imu (mdrobot_imu). OFF by default: it needs the
        # sensor fitted and its signs verified by turning the machine, and with
        # it off every phase behaves exactly as it did before.
        #
        # Switching it on also makes the heading REQUIRED: the sequence aborts
        # if it goes missing or stale, because a guard that quietly stops
        # guarding is worse than one that was never asked for.
        self.declare_parameter("auto_yaw_hold", False)
        self.declare_parameter("auto_yaw_timeout", 0.5)
        self.declare_parameter("auto_yaw_deadband_deg", 2.0)
        self.declare_parameter("auto_yaw_gain", 0.01)
        self.declare_parameter("auto_yaw_max_wz", 0.08)
        self.declare_parameter("auto_yaw_abort_deg", 15.0)
        self.declare_parameter("auto_yaw_stationary_abort_deg", 4.0)
        self.declare_parameter("auto_yaw_settle_seconds", 1.5)
        self.declare_parameter("auto_hole_stage", False)
        self.declare_parameter("auto_hole_timeout", 0.5)
        self.declare_parameter("auto_hole_target_x", 0.0)
        self.declare_parameter("auto_hole_target_y", 0.0)
        self.declare_parameter("auto_hole_tolerance", 0.05)
        self.declare_parameter("auto_hole_gain_x", -0.3)
        self.declare_parameter("auto_hole_gain_y", -0.3)
        self.declare_parameter("auto_hole_max_speed", 0.05)
        self.declare_parameter("auto_actuator_seconds", 12.0)
        self.declare_parameter("auto_hold_actuator_during_spray", True)
        self.declare_parameter("auto_hold_pulse_on", 0.4)
        self.declare_parameter("auto_hold_pulse_period", 2.5)
        self.declare_parameter("auto_spray_seconds", 30.0)
        self.declare_parameter("auto_max_find_hole_seconds", 30.0)
        self.declare_parameter("auto_max_hole_align_seconds", 60.0)
        # Odometry sanity: the drivers publish raw counts unless their own
        # counts_per_rev is set, and counts read as radians look like enormous
        # travel. Abort if the measured speed exceeds what was ever commanded by
        # more than this factor.
        self.declare_parameter("odom_max_speed_factor", 4.0)

        self.rc_timeout = float(self.get_parameter("rc_timeout").value)
        self.rc_grace = float(self.get_parameter("rc_grace").value)
        self.rc_grace_stationary = float(
            self.get_parameter("rc_grace_stationary").value)
        self.pwm_min = int(self.get_parameter("pwm_min").value)
        self.pwm_mid = int(self.get_parameter("pwm_mid").value)
        self.pwm_max = int(self.get_parameter("pwm_max").value)
        self.deadband = int(self.get_parameter("deadband").value)
        self.require_neutral_start = bool(
            self.get_parameter("require_neutral_start").value)
        self.pwm_tolerance = int(self.get_parameter("pwm_tolerance").value)
        if self.pwm_tolerance < 0:
            raise ValueError(f"pwm_tolerance cannot be negative, got {self.pwm_tolerance}")
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

        self.wheel_signs = [int(v) for v in self.get_parameter("wheel_signs").value]
        if len(self.wheel_signs) != 4:
            raise ValueError(f"wheel_signs needs 4 entries {WHEEL_NAMES}, got {self.wheel_signs}")
        if set(self.wheel_signs) - {-1, 1}:
            raise ValueError(f"wheel_signs entries must be -1 or +1, got {self.wheel_signs}")

        self.lift_speed = int(self.get_parameter("lift_speed").value)
        self.lift_max_run = float(self.get_parameter("lift_max_run").value)
        self.actuator_max_run = float(self.get_parameter("actuator_max_run").value)
        self.lift_input = str(self.get_parameter("lift_input").value).lower()
        if self.lift_input not in ("speed", "tristate", "pwm"):
            raise ValueError(
                f"lift_input must be 'speed', 'tristate' or 'pwm', "
                f"got {self.lift_input!r}"
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
        self.position_units = str(self.get_parameter("wheel_position_units").value).lower()
        if self.position_units not in ("rad", "count", "unset"):
            raise ValueError(
                f"wheel_position_units must be 'rad', 'count' or 'unset', "
                f"got {self.position_units!r}"
            )
        if self.position_units == "count" and self.counts_per_rev <= 0:
            raise ValueError(
                "wheel_position_units is 'count', so counts_per_rev must be positive"
            )
        self.auto_config = AutonomousConfig(
            plate_timeout=float(self.get_parameter("auto_plate_timeout").value),
            min_approach_width=float(
                self.get_parameter("auto_min_approach_width").value),
            plate_shrink_ratio=float(
                self.get_parameter("auto_plate_shrink_ratio").value),
            align_gain=float(self.get_parameter("auto_align_gain").value),
            align_max_speed=float(
                self.get_parameter("auto_align_max_speed").value),
            align_tolerance=float(self.get_parameter("auto_align_tolerance").value),
            approach_speed=float(self.get_parameter("auto_approach_speed").value),
            entry_distance=float(self.get_parameter("auto_entry_distance").value),
            entry_speed=float(self.get_parameter("auto_entry_speed").value),
            drill_seconds=float(self.get_parameter("auto_drill_seconds").value),
            lift_up_seconds=float(self.get_parameter("auto_lift_up_seconds").value),
            lift_down_seconds=float(self.get_parameter("auto_lift_down_seconds").value),
            retract_seconds=float(self.get_parameter("auto_retract_seconds").value),
            hold_actuator_during_spray=bool(
                self.get_parameter("auto_hold_actuator_during_spray").value),
            hold_pulse_on=float(self.get_parameter("auto_hold_pulse_on").value),
            hold_pulse_period=float(
                self.get_parameter("auto_hold_pulse_period").value),
            max_align_seconds=float(self.get_parameter("auto_max_align_seconds").value),
            max_entry_seconds=float(self.get_parameter("auto_max_entry_seconds").value),
            yaw_hold=bool(self.get_parameter("auto_yaw_hold").value),
            yaw_timeout=float(self.get_parameter("auto_yaw_timeout").value),
            yaw_deadband_deg=float(
                self.get_parameter("auto_yaw_deadband_deg").value),
            yaw_gain=float(self.get_parameter("auto_yaw_gain").value),
            yaw_max_wz=float(self.get_parameter("auto_yaw_max_wz").value),
            yaw_abort_deg=float(self.get_parameter("auto_yaw_abort_deg").value),
            yaw_stationary_abort_deg=float(
                self.get_parameter("auto_yaw_stationary_abort_deg").value),
            yaw_settle_seconds=float(
                self.get_parameter("auto_yaw_settle_seconds").value),
            hole_stage=bool(self.get_parameter("auto_hole_stage").value),
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

        self.pub_drive = self.create_publisher(Float64MultiArray, "~/cmd_wheel_rpm", 10)
        self.pub_command = self.create_publisher(Int32MultiArray, "~/command", 10)
        self.pub_mode = self.create_publisher(String, "~/mode", 10)
        self.pub_diag = self.create_publisher(DiagnosticArray, "~/diagnostics", 1)
        self.pub_phase = self.create_publisher(String, "~/auto_phase", 10)
        # The phase NAME alone cannot answer the question that actually comes
        # up while watching an approach: is it strafing because the camera saw
        # the plate move, or turning because the heading hold decided it had
        # drifted? One is vy, the other is wz, and until this existed neither
        # was visible -- the phase message carrying them was logged only when
        # the phase CHANGED, and align is one phase for the whole approach.
        self.pub_detail = self.create_publisher(String, "~/auto_detail", 10)
        self.create_subscription(Int32MultiArray, "~/rc", self._on_rc, 10)
        self.create_subscription(Point, "~/plate_offset", self._on_plate, 10)
        self.create_subscription(Point, "~/hole_offset", self._on_hole, 10)
        self.create_subscription(JointState, "~/joint_states", self._on_joints, 10)
        self.create_subscription(Imu, "~/imu", self._on_imu, 10)

        self._lock = threading.Lock()
        self._rc: list[int] | None = None
        self._rc_wall = 0.0
        self._stopped = True  # nothing has been commanded yet
        self._last_mode = ""
        self._clamp_k = 1.0
        self._rejected = 0
        self._plate_x: float | None = None
        self._plate_w: float | None = None
        self._last_equipment: list[int] | None = None
        self._limit_top = False
        self._limit_bottom = False
        self._plate_wall = 0.0
        self._hole_x: float | None = None
        self._hole_y: float | None = None
        self._hole_wall = 0.0
        self._yaw: float | None = None
        self._yaw_wall = 0.0
        # Previous odometry sample, for the plausibility check.
        self._odom_prev: tuple[float, float] | None = None
        # Motor-shaft position per wheel, in WHEEL_NAMES order; None until seen.
        self._wheel_positions: list[float] | None = None
        self._was_autonomous = False
        self._last_phase = ""
        self._auto_lift = 0
        self._auto_actuator = 0
        self._auto_solenoid = 0
        self._sticks_bad = False
        self._at_top = False
        self._at_bottom = False
        # Per-output run guards: start time, direction, and whether the output is
        # blocked until the operator lets go.
        self._run_since: dict[str, float] = {}
        self._run_dir: dict[str, int] = {}
        self._run_blocked: dict[str, bool] = {}
        self._armed = not self.require_neutral_start

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
        # z is the plate's width as a fraction of the frame — the range proxy
        # the sequence uses to tell arriving under the car apart from the
        # detector simply dropping out.
        self._plate_w = float(msg.z)
        self._plate_wall = time.monotonic()

    def _on_hole(self, msg: Point) -> None:
        """Latch where the drilled hole sits in the upward camera's frame."""
        self._hole_x = float(msg.x)
        self._hole_y = float(msg.y)
        self._hole_wall = time.monotonic()

    def _on_joints(self, msg: JointState) -> None:
        """Latch motor-shaft positions, one per wheel in WHEEL_NAMES order."""
        if len(msg.position) != 4:
            self.get_logger().warn(
                f"~/joint_states carries {len(msg.position)} positions, expected 4 "
                f"{WHEEL_NAMES}",
                throttle_duration_sec=5.0,
            )
            return
        self._wheel_positions = [float(p) for p in msg.position]

    def _on_imu(self, msg: Imu) -> None:
        """Keep the heading, in degrees, from the IMU's orientation.

        Only the CHANGE since a reference matters to the sequence, so what the
        sensor calls zero is irrelevant — which is just as well, because under a
        car it is not north and not stable.

        Yaw comes out of the quaternion directly rather than through a transform
        library: it is one atan2, and the alternative is a dependency on
        tf_transformations for a single line.
        """
        q = msg.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        with self._lock:
            self._yaw = math.degrees(math.atan2(siny, cosy))
            self._yaw_wall = time.monotonic()

    def _distance(self) -> float | None:
        """Forward travel in metres, averaged over the four wheels.

        None until every wheel has reported. Positions come from the motor
        shaft, so the gear ratio divides out; wheel_signs undo the mirrored
        mounting so all four agree on which way is forward.

        Only meaningful once wheel_position_units says what ~/joint_states
        actually carries — see _autonomous, which refuses to run without it.
        """
        if self._wheel_positions is None:
            return None
        total = 0.0
        for i, position in enumerate(self._wheel_positions):
            if self.position_units == "count":
                # Driver is publishing raw counts; turn them into motor radians.
                position = position / self.counts_per_rev * 2.0 * math.pi
            total += position * self.wheel_signs[i]
        mean_motor_rad = total / 4.0
        return mean_motor_rad / self.gear_ratio * self.geom.wheel_radius

    def _sticks_usable(self, rc: list[int]) -> bool:
        """Are the stick channels carrying something that could be a pulse width?

        Out-of-range must never mean full deflection. Normalising, say, 42819 as
        a pulse width and clamping the result gives exactly +1.0 — the machine
        reads a value it cannot interpret and drives away at full speed. The
        board has already been seen changing what a channel carries (lift turned
        out to be a speed, not a switch), so the range is checked rather than
        assumed.
        """
        low = self.pwm_min - self.pwm_tolerance
        high = self.pwm_max + self.pwm_tolerance
        bad = [
            f"{CHANNEL_NAMES[c]}={rc[c]}"
            for c in (CH["steer"], CH["throttle"])
            if not low <= rc[c] <= high
        ]
        if bad:
            self.get_logger().error(
                f"stick channels outside {low}..{high}: {', '.join(bad)}. "
                f"Refusing to drive — an unreadable stick must not become full "
                f"deflection. Check what the board is sending with "
                f"examples/read_rc_bridge.py.",
                throttle_duration_sec=2.0,
            )
            return False
        return True

    def _run_guard(self, name: str, value: int, limit: float, now: float) -> int:
        """Stop an output that has been driving one way for too long.

        Neither the lift nor the actuator has limit switches or position
        feedback, so holding the switch drives them into their end stop and
        leaves them stalled there. A stalled motor pulls far more current than a
        moving one, and this machine already sits at 11.5 V — the sag is enough
        to brown out a motor controller, which is what the comms dropping out
        during a run looks like.

        Once blocked the output stays blocked until the operator returns the
        switch to neutral, so a guard that trips cannot be ridden through by
        holding the switch harder.
        """
        if limit <= 0:
            return value
        direction = (value > 0) - (value < 0)
        if direction == 0:
            if self._run_blocked.get(name):
                self.get_logger().info(f"{name} released; run guard cleared")
            self._run_blocked[name] = False
            self._run_dir[name] = 0
            return 0
        if self._run_blocked.get(name):
            return 0
        if self._run_dir.get(name) != direction:
            # A genuine change of direction restarts the clock.
            self._run_dir[name] = direction
            self._run_since[name] = now
            return value
        if now - self._run_since.get(name, now) > limit:
            self._run_blocked[name] = True
            self.get_logger().warn(
                f"{name} has been driving {'up' if direction > 0 else 'down'} for "
                f"{limit:.0f} s and is probably against its end stop. Stopping it "
                f"— return the switch to neutral to use it again."
            )
            return 0
        return value

    def _at_neutral(self, rc: list[int]) -> bool:
        """Both sticks resting at centre, within the deadband."""
        return all(abs(rc[c] - self.pwm_mid) <= self.deadband
                   for c in (CH["steer"], CH["throttle"]))

    def _axis(self, value: int, invert: bool) -> float:
        """Pulse width in microseconds -> -1..+1, with a deadband at centre.

        Callers must have checked the value with _sticks_usable first: the clamp
        here turns anything far out of range into full deflection.
        """
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

    def _on_tick(self) -> None:
        now = time.monotonic()
        with self._lock:
            rc = self._rc
            age = now - self._rc_wall if self._rc_wall else None

        if rc is None or age is None or age > self.rc_timeout:
            # The wheels stop the instant the link goes quiet, always. A running
            # sequence is treated separately: the board's USB drops for a few
            # tenths of a second fairly often, and killing a drill-and-spray run
            # over one of those — dropping the actuator mid-stroke with it — is
            # worse than riding it out with the wheels held still.
            running = (
                self._was_autonomous and self.sequence.phase not in TERMINAL
            )
            # Phases that command no wheel motion do not need the link at all:
            # the machine is parked with the drill in a hole. Losing RC there is
            # a reason to keep holding what is already holding, not to let go of
            # it. The driving phases keep the short grace, because a machine
            # that is moving and cannot hear the operator has to give up sooner.
            static = running and self.sequence.phase in STATIONARY
            grace = self.rc_grace_stationary if static else self.rc_grace
            riding = running and age is not None and age <= self.rc_timeout + grace
            if not self._stopped:
                self._stopped = True
                self.get_logger().warn(
                    "no ~/rc within rc_timeout; wheels stopped"
                    + (f"; {self.sequence.phase.value} riding out the gap "
                       f"({grace:.0f} s)" if riding else " and equipment idle")
                )
            if riding:
                self._auto_lift = 0
                self._auto_actuator = 0
                self._auto_solenoid = 0
                # brake reads 0: a pressed brake would have aborted the run
                # before the link went quiet.
                _, _, _, auto_drill = self._autonomous(now, 0)
                lift = self._run_guard(
                    "lift", self._auto_lift * self.lift_speed,
                    self.lift_max_run, now)
                actuator = self._run_guard(
                    "actuator", max(-1, min(1, self._auto_actuator)),
                    self.actuator_max_run, now)
                self._publish(
                    [0.0] * 4,
                    [lift, 0, 1 if auto_drill else 0, actuator,
                     1 if self._auto_solenoid else 0],
                    "rc-gap",
                )
                self._clamp_k = 1.0
                return
            if running:
                # Abort, do NOT reset. Resetting here re-armed the sequence the
                # moment the link came back, so a dropout silently ran the whole
                # thing again from the top, drill included. Aborting is terminal:
                # it takes the operator leaving autonomous and coming back to
                # start another.
                self.sequence.abort("RC link lost")
                self.get_logger().error(
                    f"autonomous aborted: no RC for {age:.1f} s, past the "
                    f"{grace:.1f} s grace. It will not restart on "
                    "its own — switch out of autonomous and back to run it again."
                )
                self._publish_phase(self.sequence.phase.value)
            if (not running and self._last_equipment is not None
                    and age is not None and age <= self.rc_timeout + self.rc_grace):
                # Manual, mid-gap. The operator is still holding the switch —
                # the link dropped, not the switch. Letting go of the actuator
                # here is what makes it sag and climb back over and over while
                # they hold it steady. Wheels still stop.
                self._publish([0.0] * 4, self._last_equipment, "rc-gap")
                self._clamp_k = 1.0
                return
            self._last_equipment = None
            self._armed = not self.require_neutral_start
            self._publish([0.0] * 4, IDLE_COMMAND, "unknown")
            self._clamp_k = 1.0
            return

        if self._stopped:
            self._stopped = False
            self.get_logger().info("~/rc live; resuming")

        if not self._sticks_usable(rc):
            # Same response as a lost link: stop, and let go of the equipment.
            # A frame we cannot read is not one to run a drill from either.
            if self._was_autonomous:
                self.sequence.reset()
                self._was_autonomous = False
                self._publish_phase("")
            self._sticks_bad = True
            self._armed = not self.require_neutral_start
            self._publish([0.0] * 4, IDLE_COMMAND, "unusable")
            self._clamp_k = 1.0
            return
        if self._sticks_bad:
            self._sticks_bad = False
            self.get_logger().info("stick channels back in range")

        if not self._armed:
            # Arming interlock: hold still until both sticks have been seen at
            # centre at least once. It catches a launch with the sticks already
            # pushed, and no-link garbage that happens to land inside the range
            # check — the board sends no failsafe neutral, and what it does send
            # with the transmitter off is not even consistent.
            if self._at_neutral(rc):
                self._armed = True
                self.get_logger().info("sticks seen at neutral; drive armed")
            else:
                self.get_logger().warn(
                    f"waiting for neutral sticks before driving: "
                    f"steer={rc[CH['steer']]} throttle={rc[CH['throttle']]}, "
                    f"want {self.pwm_mid} +/- {self.deadband}",
                    throttle_duration_sec=3.0,
                )
                self._publish([0.0] * 4, IDLE_COMMAND, "disarmed")
                self._clamp_k = 1.0
                return

        mode = self._mode_name(rc[CH["mode"]])
        brake = 1 if rc[CH["brake"]] else 0
        auto_drill = 0

        self._auto_lift = 0
        self._auto_actuator = 0
        self._auto_solenoid = 0
        self._limit_top, self._limit_bottom = self._limits(rc)
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

        # The sequence drives the lift too, and it is the only thing that does
        # while autonomous is running — the operator's channel reads 0 then.
        # Either source alone moves it.
        lift_request = self._gated_lift(rc)
        if lift_request == 0 and self._auto_lift:
            lift_request = self._auto_lift * self.lift_speed
        lift = self._run_guard("lift", lift_request, self.lift_max_run, now)
        # The operator can always drive the actuator; the sequence takes it only
        # when the operator has left it alone.
        actuator = max(-1, min(1, rc[CH["actuator"]] or self._auto_actuator))
        actuator = self._run_guard("actuator", actuator, self.actuator_max_run, now)
        command = [
            lift,
            brake,
            # The operator's drill switch and the sequence's request are both
            # honoured; either one alone turns it on.
            1 if (rc[CH["drill"]] or auto_drill) else 0,
            actuator,
            1 if (rc[CH["solenoid"]] or self._auto_solenoid) else 0,
        ]
        self._last_equipment = list(command)
        self._publish(wheel_rpm, command, mode)

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

        if self.position_units == "unset":
            # The drivers publish raw counts unless their own counts_per_rev is
            # set, and counts taken for radians overstate travel enormously —
            # ENTER would finish in one tick and the drill would fire at the
            # entry point. Refuse to guess which it is.
            if self.sequence.phase not in TERMINAL:
                self.sequence.abort("wheel_position_units is unset")
                self.get_logger().error(
                    "autonomous refused: wheel_position_units is 'unset', so the "
                    "entry distance cannot be trusted. Measure the encoders with "
                    "examples/calibrate_counts_per_rev.py, then either set "
                    "counts_per_rev on the drivers and put 'rad' here, or put "
                    "'count' here with the measured counts_per_rev."
                )
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
        # None, not a huge age, when nothing has ever arrived: the sequence
        # tells "no sensor" apart from "sensor gone quiet" and says which.
        yaw_age = (now - self._yaw_wall) if self._yaw_wall else None
        action = self.sequence.step(Observation(
            now=now,
            plate_offset_x=self._plate_x,
            plate_width=self._plate_w,
            at_top=self._limit_top,
            at_bottom=self._limit_bottom,
            plate_age=plate_age,
            distance=distance,
            hole_offset_x=self._hole_x,
            hole_offset_y=self._hole_y,
            hole_age=hole_age,
            yaw=self._yaw,
            yaw_age=yaw_age,
        ))
        detail = (
            f"{action.phase.value} | vx {action.vx:+.3f} vy {action.vy:+.3f} "
            f"wz {action.wz:+.3f} | {action.message}"
        )
        self.pub_detail.publish(String(data=detail))
        if action.phase.value != self._last_phase:
            self.get_logger().info(f"autonomous: {detail}")
        elif action.phase not in TERMINAL:
            # Throttled, so an approach that takes a minute leaves a trail
            # without burying the log. Terminal phases say it once: they never
            # change again, and repeating it every second scrolls away the run
            # that led there, which is the part worth reading.
            self.get_logger().info(f"autonomous: {detail}",
                                   throttle_duration_sec=1.0)
        self._publish_phase(action.phase.value)
        self._auto_lift = action.lift
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
        if self.lift_input == "speed":
            # The board sends this channel as the speed itself, in the same
            # -60..60 the downlink takes. Observed on the hardware: it reports
            # 60, not a switch position and not a pulse width.
            lift = int(max(-self.lift_speed, min(self.lift_speed, raw)))
        else:
            if self.lift_input == "pwm":
                fraction = self._axis(raw, False)
            elif -1 <= raw <= 1:
                fraction = float(raw)
            else:
                # Configured as a -1/0/+1 switch but reporting something else.
                # Refuse to move rather than scaling a stray value into
                # full-speed lift.
                self.get_logger().error(
                    f"lift channel reported {raw}, outside -1..1 with "
                    f"lift_input='tristate'. Holding lift at 0 — the hardware "
                    f"was seen sending a speed, so try lift_input='speed'.",
                    throttle_duration_sec=5.0,
                )
                return 0
            lift = int(round(max(-1.0, min(1.0, fraction)) * self.lift_speed))
        if not self.limit_gating:
            return lift
        at_top = rc[CH["limit_up"]] == self.limit_active_value
        at_bottom = rc[CH["limit_down"]] == self.limit_active_value
        if at_top and at_bottom:
            # The carriage cannot be at both ends. Either the polarity is
            # inverted (both read "triggered" at rest) or a switch has failed.
            # Either way the gate is meaningless, so stop rather than drive into
            # a hard stop on the strength of a reading we do not trust.
            self.get_logger().error(
                f"both limit switches read {self.limit_active_value} at once — "
                f"the carriage cannot be at both ends. Holding lift at 0. Check "
                f"limit_active_value (currently {self.limit_active_value}) and "
                f"the switch wiring.",
                throttle_duration_sec=5.0,
            )
            return 0
        if lift > 0 and at_top:
            if not self._at_top:
                self.get_logger().info("upper limit reached; lift stopped")
            self._at_top = True
            return 0
        self._at_top = False
        if lift < 0 and at_bottom:
            if not self._at_bottom:
                self.get_logger().info("lower limit reached; lift stopped")
            self._at_bottom = True
            return 0
        self._at_bottom = False
        return lift

    def _limits(self, rc: list[int]) -> tuple[bool, bool]:
        """(at_top, at_bottom), or (False, False) when they cannot be trusted.

        False means "the clock is in charge", which is what the sequence did
        before the switches were fitted. Both closed at once is a wiring or
        polarity fault, not a position, so it reports neither.
        """
        if not self.limit_gating:
            return False, False
        at_top = rc[CH["limit_up"]] == self.limit_active_value
        at_bottom = rc[CH["limit_down"]] == self.limit_active_value
        if at_top and at_bottom:
            return False, False
        return at_top, at_bottom

    # ── output ──────────────────────────────────────────────────────────────
    def _publish(self, wheel_rpm: list[float], command: list[int], mode: str) -> None:
        msg = Float64MultiArray()
        msg.data = [float(v) for v in wheel_rpm]
        self.pub_drive.publish(msg)

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
            KeyValue(key="sticks_usable", value=str(not self._sticks_bad)),
            KeyValue(key="armed", value=str(self._armed)),
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
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
