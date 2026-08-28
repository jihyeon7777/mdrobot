"""Bring up the whole machine: RC bridge, drive, and the decision layer.

  ros2 launch mdrobot_supervisor bringup.launch.py

Three nodes, one per concern:

  mdrobot_rc_bridge       ttyACM0  reports the operator, relays equipment commands
  mdrobot_mecanum_driver  ttyUSB0  owns the RS485 bus, turns rpm into motor writes
  mdrobot_supervisor               decides everything, talks to both

Each keeps its own parameter file; pass rc_config, drive_config or
supervisor_config to override one. The plate reader is NOT started here — run
mdrobot_plate_ocr separately, since it is only needed for autonomous mode.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    rc_share = get_package_share_directory("mdrobot_rc_bridge")
    drive_share = get_package_share_directory("mdrobot_ros2_driver")
    sup_share = get_package_share_directory("mdrobot_supervisor")

    args = [
        DeclareLaunchArgument(
            "rc_config",
            default_value=os.path.join(rc_share, "config", "rc_bridge.yaml")),
        DeclareLaunchArgument(
            "drive_config",
            default_value=os.path.join(drive_share, "config", "mecanum.yaml")),
        DeclareLaunchArgument(
            "supervisor_config",
            default_value=os.path.join(sup_share, "config", "supervisor.yaml")),
    ]

    rc_bridge = Node(
        package="mdrobot_rc_bridge",
        executable="rc_bridge_node",
        name="mdrobot_rc_bridge",
        output="screen",
        parameters=[LaunchConfiguration("rc_config")],
    )
    drive = Node(
        package="mdrobot_ros2_driver",
        executable="mecanum_driver_node",
        name="mdrobot_mecanum_driver",
        output="screen",
        parameters=[LaunchConfiguration("drive_config")],
    )
    supervisor = Node(
        package="mdrobot_supervisor",
        executable="supervisor_node",
        name="mdrobot_supervisor",
        output="screen",
        parameters=[LaunchConfiguration("supervisor_config")],
        remappings=[
            ("~/rc", "/mdrobot_rc_bridge/channels"),
            ("~/command", "/mdrobot_rc_bridge/command"),
            ("~/cmd_wheel_rpm", "/mdrobot_mecanum_driver/cmd_wheel_rpm"),
            ("~/joint_states", "/mdrobot_mecanum_driver/joint_states"),
            ("~/plate_offset", "/mdrobot_plate_ocr/plate_offset"),
            # No publisher yet; the upward camera is not fitted.
            ("~/hole_offset", "/mdrobot_hole_detector/hole_offset"),
        ],
    )
    return LaunchDescription(args + [rc_bridge, drive, supervisor])
