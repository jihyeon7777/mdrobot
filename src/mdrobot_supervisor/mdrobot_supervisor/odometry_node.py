"""Where the machine is and which way it is pointing, for RViz and for TF.

This exists because once the machine is under a vehicle nobody can see it. It
publishes the two things a viewer needs — a pose and the wheel angles — and it
is honest about which parts of that are measured and which are guessed.

    heading   MEASURED, by the IMU. Real.
    x, y      DEAD RECKONED from the wheels. A guess.
    wheel     MEASURED, by the encoders. Real.
      angles

The distinction matters more here than on most robots. Mecanum rollers slip,
and slip is invisible to the encoders by construction: the pattern where the
front pair drives forward against the rear pair sits in the null space of the
kinematics, so the wheels can scrub without the arithmetic noticing. The
position this publishes will therefore drift, confidently and without warning.
Do not read a distance off RViz and believe it.

The heading does not drift the same way, because it is not inferred from the
wheels at all — the IMU measures the machine. That is the whole reason the
sensor is fitted, and it is why ``use_imu_heading`` defaults to true. With it
off, the heading falls back to integrating the wheels' own disagreement, which
is exactly the quantity that is wrong.

Topics
------
Subscribes:
  ~/joint_states (sensor_msgs/JointState)   four wheels, MOTOR-shaft radians
  ~/imu (sensor_msgs/Imu)                   attitude, for the heading
Publishes:
  ~/odom (nav_msgs/Odometry)                pose and twist in the odom frame
  ~/wheel_joint_states (sensor_msgs/JointState)
      the same four wheels in WHEEL radians with the mounting signs undone —
      what robot_state_publisher needs to turn the model's wheels. Feeding it
      the raw motor angles would spin them by the gear ratio, twenty times too
      far.
  TF odom -> base_link

The odom frame is anchored wherever the machine happened to be when this node
started, and the heading zero is whatever the IMU first reported. Both are
arbitrary by design: odom is a relative frame, and what the sensor calls north
is meaningless under a car anyway.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from tf2_ros import TransformBroadcaster

from mdrobot_supervisor.kinematics import (
    MecanumGeometry,
    WHEEL_NAMES,
    forward,
)


def yaw_from_quaternion(q) -> float:
    """Heading in radians from a geometry_msgs Quaternion."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def wrap(angle: float) -> float:
    """Fold radians into [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class OdometryNode(Node):
    def __init__(self) -> None:
        super().__init__("mdrobot_odometry")
        self.declare_parameter("wheel_radius", 0.0625)
        self.declare_parameter("track", 0.575)
        self.declare_parameter("wheelbase", 0.5)
        self.declare_parameter("roller_layout", "unknown")
        self.declare_parameter("gear_ratio", 20.0)
        # +1 when a POSITIVE reported angle means that wheel drove the machine
        # forward. The same convention, and the same default, as the supervisor.
        self.declare_parameter("wheel_signs", [-1, 1, -1, 1])
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("joint_names", list(WHEEL_NAMES))
        # Off only to see what the wheels alone would have said, which is worth
        # looking at once: the gap between the two IS the slip.
        self.declare_parameter("use_imu_heading", True)
        self.declare_parameter("publish_tf", True)

        self.geom = MecanumGeometry(
            wheel_radius=float(self.get_parameter("wheel_radius").value),
            track=float(self.get_parameter("track").value),
            wheelbase=float(self.get_parameter("wheelbase").value),
            roller_layout=str(self.get_parameter("roller_layout").value),
        )
        self.gear_ratio = float(self.get_parameter("gear_ratio").value)
        if self.gear_ratio <= 0:
            raise ValueError(f"gear_ratio must be positive, got {self.gear_ratio}")
        self.wheel_signs = [int(v) for v in self.get_parameter("wheel_signs").value]
        if len(self.wheel_signs) != 4 or set(self.wheel_signs) - {-1, 1}:
            raise ValueError(
                f"wheel_signs needs 4 entries of -1 or +1 {WHEEL_NAMES}, "
                f"got {self.wheel_signs}"
            )
        self.joint_names = [str(n) for n in self.get_parameter("joint_names").value]
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.use_imu_heading = bool(self.get_parameter("use_imu_heading").value)

        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self._wheel_heading = 0.0  # what the wheels alone would have said
        self._prev: list[float] | None = None
        self._prev_stamp: float | None = None
        self._imu_yaw: float | None = None
        self._imu_zero: float | None = None
        self._warned_imu = False

        self.pub_odom = self.create_publisher(Odometry, "~/odom", 10)
        self.pub_joints = self.create_publisher(
            JointState, "~/wheel_joint_states", 10)
        self.tf = (
            TransformBroadcaster(self)
            if bool(self.get_parameter("publish_tf").value) else None
        )
        self.create_subscription(JointState, "~/joint_states", self._on_joints, 10)
        self.create_subscription(Imu, "~/imu", self._on_imu, 10)
        self.get_logger().info(
            f"odometry: heading from "
            f"{'the IMU' if self.use_imu_heading else 'THE WHEELS (imu off)'}, "
            f"position dead reckoned. Position drifts — do not measure with it."
        )

    def _on_imu(self, msg: Imu) -> None:
        yaw = yaw_from_quaternion(msg.orientation)
        if self._imu_zero is None:
            # odom is a relative frame, so the machine starts pointing along
            # +x whatever the sensor happens to call that.
            self._imu_zero = yaw
        self._imu_yaw = wrap(yaw - self._imu_zero)

    def _on_joints(self, msg: JointState) -> None:
        try:
            index = [msg.name.index(n) for n in self.joint_names]
        except ValueError:
            self.get_logger().warn(
                f"joint_states does not carry {self.joint_names}, got "
                f"{list(msg.name)}",
                throttle_duration_sec=5.0)
            return
        if len(msg.position) < max(index) + 1:
            return
        # Motor-shaft radians -> wheel radians, mounting undone.
        angles = [
            float(msg.position[i]) * s / self.gear_ratio
            for i, s in zip(index, self.wheel_signs)
        ]
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._publish_joints(msg.header.stamp, angles)

        if self._prev is None or self._prev_stamp is None:
            self._prev, self._prev_stamp = angles, stamp
            return
        deltas = [a - p for a, p in zip(angles, self._prev)]
        dt = stamp - self._prev_stamp
        self._prev, self._prev_stamp = angles, stamp
        if dt <= 0.0:
            return

        # forward() is linear, so feeding it ANGLE deltas rather than rates
        # returns the body DISPLACEMENT over the interval directly.
        dx_body, dy_body, dtheta = forward(deltas, self.geom)
        self._wheel_heading = wrap(self._wheel_heading + dtheta)

        previous = self.heading
        if self.use_imu_heading and self._imu_yaw is not None:
            self.heading = self._imu_yaw
        else:
            if self.use_imu_heading and not self._warned_imu:
                self._warned_imu = True
                self.get_logger().warn(
                    "no ~/imu yet — heading is coming from the wheels, which is "
                    "the reading the IMU was fitted to replace")
            self.heading = self._wheel_heading

        # Rotate the body increment into odom using the MIDPOINT heading. Using
        # either end biases every turn to one side, and the machine turns while
        # it strafes, which is the whole problem.
        mid = previous + wrap(self.heading - previous) * 0.5
        cos, sin = math.cos(mid), math.sin(mid)
        self.x += dx_body * cos - dy_body * sin
        self.y += dx_body * sin + dy_body * cos
        self._publish_odom(msg.header.stamp, (dx_body / dt, dy_body / dt,
                                              wrap(self.heading - previous) / dt))

    def _publish_joints(self, stamp, angles: list[float]) -> None:
        js = JointState()
        js.header.stamp = stamp
        js.name = list(self.joint_names)
        js.position = angles
        self.pub_joints.publish(js)

    def _publish_odom(self, stamp, twist: tuple[float, float, float]) -> None:
        half = self.heading * 0.5
        qz, qw = math.sin(half), math.cos(half)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        # Position is dead reckoning on wheels that slip: say so in the
        # covariance rather than letting a consumer read it as surveyed.
        odom.pose.covariance[0] = 0.25
        odom.pose.covariance[7] = 0.25
        odom.pose.covariance[35] = (
            math.radians(2.0) ** 2 if self.use_imu_heading else 0.5
        )
        odom.twist.twist.linear.x = twist[0]
        odom.twist.twist.linear.y = twist[1]
        odom.twist.twist.angular.z = twist[2]
        self.pub_odom.publish(odom)

        if self.tf is not None:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf.sendTransform(t)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
