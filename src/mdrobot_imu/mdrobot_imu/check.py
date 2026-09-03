"""Live attitude readout, for settling the mounting signs by hand.

Use THIS and not ``imu_survey`` for sign checks. The survey reads the port
directly and prints the sensor's own raw degrees, with no mounting applied —
which is what you want when measuring drift, and exactly wrong when the
question is whether the node's output moves the right way.

This subscribes to ``~/attitude_deg``, so what it prints is what the rest of
the robot sees: signs applied, mount offset applied, REP-103.

    ros2 launch mdrobot_imu imu.launch.py     # one terminal
    ros2 run mdrobot_imu imu_check            # another

Then move the machine and read the arrows. Each axis shows the value now and
how far it has moved from where it started, so a slow tilt is visible without
having to remember a number.
"""

from __future__ import annotations

import sys

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node

# What each axis must do, per REP-103. Getting one of these backwards does not
# produce a wobble, it produces a controller that drives the error the wrong
# way — under a car, with a drill.
CHECKS = (
    ("roll", "roll it RIGHT (right side down)"),
    ("pitch", "tip the NOSE UP"),
    ("yaw", "turn it LEFT"),
)


class CheckNode(Node):
    def __init__(self) -> None:
        super().__init__("mdrobot_imu_check")
        self.start: tuple[float, float, float] | None = None
        self.seen = 0
        self.create_subscription(
            Vector3Stamped, "/mdrobot_imu/attitude_deg", self._on_attitude, 10
        )
        print()
        print("  Move the machine. Every axis must go UP.")
        print()
        for axis, action in CHECKS:
            print(f"    {axis:<6} {action:<32} -> must go UP")
        print()
        print("  An axis that goes DN has its sign backwards: flip <axis>_sign")
        print("  in config/imu.yaml, rebuild, and check it again.")
        print("  Ctrl-C when done.")
        print()

    def _on_attitude(self, msg: Vector3Stamped) -> None:
        now = (msg.vector.x, msg.vector.y, msg.vector.z)
        if self.start is None:
            self.start = now
        self.seen += 1
        cells = []
        for (axis, _), value, ref in zip(CHECKS, now, self.start):
            delta = value - ref
            arrow = "  " if abs(delta) < 0.15 else (" UP" if delta > 0 else " DN")
            cells.append(f"{axis} {value:+8.2f} ({delta:+7.2f}{arrow})")
        sys.stdout.write("\r  " + "   ".join(cells) + "  ")
        sys.stdout.flush()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CheckNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print()
        if node.seen == 0:
            print("nothing arrived on /mdrobot_imu/attitude_deg — is the node "
                  "running?  ros2 launch mdrobot_imu imu.launch.py")
        else:
            print(f"{node.seen} readings. Flip any axis that moved the wrong "
                  f"way in config/imu.yaml, rebuild, and check it again.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
