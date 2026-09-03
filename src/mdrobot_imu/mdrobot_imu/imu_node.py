"""ROS 2 node publishing the HWT901B as sensor_msgs/Imu.

Why this sensor is on this robot
--------------------------------
The machine drives under a vehicle and drills upward through the underbody.
Once it is under there, nothing can see it. Two things then have no feedback
path at all:

* **Yaw.** The autonomous sequence commands ``wz = 0`` throughout and the
  mecanum inverse kinematics assumes the wheels do not slip. On a smooth or
  oily floor they do, unevenly, and a commanded pure translation comes out as a
  translation plus a rotation. Nothing measures that today, so nothing can
  correct it or even report it.
* **Drill reaction torque.** During the drilling phase the wheels are commanded
  to zero while a bit cuts into steel. The reaction torque acts on a machine
  sitting on rollers. If it turns the robot while the bit is in the hole, the
  bit is what gives.

This node exists to make both visible. It publishes attitude only — it is not
a position sensor and must never be used as one. Integrating this
accelerometer twice over the ~15 s blind entry gives roughly a metre of error
even after calibration, which is worse than the wheel odometry it would be
replacing.

Topics
------
``~/data`` (sensor_msgs/Imu)
    Orientation, angular velocity (rad/s) and linear acceleration (m/s^2), in
    the REP-103 body frame, for anything that consumes a standard IMU.
``~/attitude_deg`` (geometry_msgs/Vector3Stamped)
    The same attitude as plain roll/pitch/yaw degrees. Redundant on purpose:
    this is the topic you record and plot to answer "how far did it actually
    twist", and degrees in a scalar topic are far easier to read off an
    ``ros2 topic echo`` over SSH than a quaternion.

Mounting — READ THIS BEFORE TRUSTING A SIGN
-------------------------------------------
The sensor reports roll about its X, pitch about its Y and yaw about its Z,
with Z up (a horizontal sensor reads +1 g on Z — confirmed on the fitted unit).
ROS REP-103 wants X forward, Y left, Z up, all right-handed.

All three signs on this robot were settled by moving it (2026-09-03), and two
of the three defaults were wrong: ``yaw_sign`` +1 and ``roll_sign`` -1 were
both expected the other way round. That is why they are parameters and why the
defaults were flagged as reasoning rather than fact. After any remount, or on
any other machine, do the three checks again:

    turn the robot to its LEFT and confirm yaw INCREASES,
    tip its NOSE UP and confirm pitch INCREASES,
    roll it to the RIGHT and confirm roll INCREASES.

Getting a sign wrong here does not produce a wobble, it produces a controller
that drives the error the wrong way — under a car, with a drill.

``mount_yaw_offset_deg`` handles a sensor bolted on rotated about Z. A sensor
mounted on its side or upside down needs a full rotation matrix, which this
node does not do; remount it flat instead.

9-axis vs 6-axis
----------------
Set the sensor to **6-axis** before using it here. In 9-axis it fuses the
magnetometer, and this robot's job is to park inside a steel box next to its
own BLDC motors and a running drill. The fitted unit already reads a badly
skewed field standing still (mx +1399, my +1844, mz -5012 on 2026-09-03), and
that is before the car. 6-axis yaw is gyro-integrated, so it drifts — but the
sequence only needs it to hold a reference for tens of seconds, and 6-axis
also unlocks "Reset Z-axis angle", which is exactly the zero-at-the-start-of-a
-phase behaviour wanted here.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from sensor_msgs.msg import Imu

from .frames import quaternion_from_rpy, to_ros_attitude
from .protocol import G_TO_M_S2
from .reader import BAUDRATE, ImuReader, Sample

# Datasheet typical values (HWT901B V260209), used as the default covariance
# diagonals. They are honest starting points, not measurements of this unit.
DEFAULT_TILT_STDDEV_DEG = 0.1  # "Inclination accuracy"
DEFAULT_YAW_STDDEV_DEG = 1.0  # heading, 9-axis with the field calibrated
DEFAULT_RATE_STDDEV_DPS = 0.07  # gyro RMS noise, bandwidth 100 Hz
DEFAULT_ACCEL_STDDEV_G = 0.001  # accelerometer RMS noise, bandwidth 100 Hz


class ImuNode(Node):
    def __init__(self) -> None:
        super().__init__("mdrobot_imu")
        self.declare_parameter("port", "/dev/ttyUSB1")
        self.declare_parameter("baudrate", BAUDRATE)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("poll_rate", 100.0)
        # Watchdog. The sensor free-runs, so silence means the port died, the
        # baud rate is wrong, or a content type this driver needs was switched
        # off in the sensor's own configuration.
        self.declare_parameter("min_sample_rate", 5.0)
        # See the module docstring: signs are reasoned, not measured. Verify.
        self.declare_parameter("roll_sign", -1.0)
        self.declare_parameter("pitch_sign", 1.0)
        self.declare_parameter("yaw_sign", 1.0)
        self.declare_parameter("mount_yaw_offset_deg", 0.0)
        self.declare_parameter("tilt_stddev_deg", DEFAULT_TILT_STDDEV_DEG)
        self.declare_parameter("yaw_stddev_deg", DEFAULT_YAW_STDDEV_DEG)
        self.declare_parameter("rate_stddev_dps", DEFAULT_RATE_STDDEV_DPS)
        self.declare_parameter("accel_stddev_g", DEFAULT_ACCEL_STDDEV_G)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.signs = (
            float(self.get_parameter("roll_sign").value),
            float(self.get_parameter("pitch_sign").value),
            float(self.get_parameter("yaw_sign").value),
        )
        for name, sign in zip(("roll_sign", "pitch_sign", "yaw_sign"), self.signs):
            if sign not in (1.0, -1.0):
                raise ValueError(f"{name} must be +1 or -1, got {sign}")
        self.yaw_offset = float(self.get_parameter("mount_yaw_offset_deg").value)
        self.min_sample_rate = float(self.get_parameter("min_sample_rate").value)
        if self.min_sample_rate <= 0:
            raise ValueError("min_sample_rate must be positive")

        self._orientation_cov = self._diag(
            math.radians(float(self.get_parameter("tilt_stddev_deg").value)),
            math.radians(float(self.get_parameter("tilt_stddev_deg").value)),
            math.radians(float(self.get_parameter("yaw_stddev_deg").value)),
        )
        rate_sd = math.radians(float(self.get_parameter("rate_stddev_dps").value))
        self._rate_cov = self._diag(rate_sd, rate_sd, rate_sd)
        accel_sd = float(self.get_parameter("accel_stddev_g").value) * G_TO_M_S2
        self._accel_cov = self._diag(accel_sd, accel_sd, accel_sd)

        self.imu_pub = self.create_publisher(Imu, "~/data", 10)
        self.attitude_pub = self.create_publisher(Vector3Stamped, "~/attitude_deg", 10)

        port = str(self.get_parameter("port").value)
        self.reader = ImuReader(port, int(self.get_parameter("baudrate").value))
        self._last_sample = self.get_clock().now()
        self._dropped_reported = 0
        self._count = 0

        poll_rate = float(self.get_parameter("poll_rate").value)
        if poll_rate <= 0:
            raise ValueError("poll_rate must be positive")
        self.create_timer(1.0 / poll_rate, self._poll)
        self.create_timer(1.0, self._watchdog)
        self.get_logger().info(
            f"HWT901B on {port} at {self.get_parameter('baudrate').value} baud; "
            f"signs roll {self.signs[0]:+.0f} pitch {self.signs[1]:+.0f} "
            f"yaw {self.signs[2]:+.0f}, mount offset {self.yaw_offset:+.1f} deg. "
            f"VERIFY those signs by hand before anything steers on them."
        )

    @staticmethod
    def _diag(sx: float, sy: float, sz: float) -> list[float]:
        return [sx * sx, 0.0, 0.0, 0.0, sy * sy, 0.0, 0.0, 0.0, sz * sz]

    def _poll(self) -> None:
        for sample in self.reader.poll():
            self._publish(sample)
            self._last_sample = self.get_clock().now()
            self._count += 1

    def _publish(self, sample: Sample) -> None:
        roll_sign, pitch_sign, yaw_sign = self.signs
        roll, pitch, yaw = to_ros_attitude(
            sample.angle.roll, sample.angle.pitch, sample.angle.yaw,
            self.signs, self.yaw_offset,
        )

        stamp = self.get_clock().now().to_msg()
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        x, y, z, w = quaternion_from_rpy(
            math.radians(roll), math.radians(pitch), math.radians(yaw)
        )
        msg.orientation.x, msg.orientation.y = x, y
        msg.orientation.z, msg.orientation.w = z, w
        msg.orientation_covariance = self._orientation_cov
        # The rates carry the same sign flips as the angles they differentiate;
        # a yaw that increases anticlockwise needs a wz that does too, or a
        # controller reading both sees them disagree.
        msg.angular_velocity.x = math.radians(roll_sign * sample.gyro.x)
        msg.angular_velocity.y = math.radians(pitch_sign * sample.gyro.y)
        msg.angular_velocity.z = math.radians(yaw_sign * sample.gyro.z)
        msg.angular_velocity_covariance = self._rate_cov
        msg.linear_acceleration.x = roll_sign * sample.accel.x
        msg.linear_acceleration.y = pitch_sign * sample.accel.y
        msg.linear_acceleration.z = sample.accel.z
        msg.linear_acceleration_covariance = self._accel_cov
        self.imu_pub.publish(msg)

        attitude = Vector3Stamped()
        attitude.header.stamp = stamp
        attitude.header.frame_id = self.frame_id
        attitude.vector.x, attitude.vector.y, attitude.vector.z = roll, pitch, yaw
        self.attitude_pub.publish(attitude)

    def _watchdog(self) -> None:
        age = (self.get_clock().now() - self._last_sample).nanoseconds / 1e9
        if age > 1.0 / self.min_sample_rate:
            self.get_logger().error(
                f"no IMU sample for {age:.1f} s. Check the port, the baud rate, "
                f"and that acceleration, angular velocity AND angle are all "
                f"enabled in the sensor's own output configuration — this "
                f"driver needs all three to make one sample.",
                throttle_duration_sec=5.0,
            )
        dropped = self.reader.dropped
        if dropped > self._dropped_reported + 100:
            self._dropped_reported = dropped
            self.get_logger().warn(
                f"{dropped} bytes dropped as unframeable. A steadily climbing "
                f"count means the line is too full for the output rate: raise "
                f"the baud rate or turn off content types you do not use.",
                throttle_duration_sec=10.0,
            )

    def destroy_node(self) -> bool:
        self.reader.close()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ImuNode()
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
