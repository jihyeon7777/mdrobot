"""Launch the decision layer.

All options live in config/supervisor.yaml — edit that file instead of passing
them on the command line:
  ros2 launch mdrobot_supervisor supervisor.launch.py
Use a different parameter file with:
  ros2 launch mdrobot_supervisor supervisor.launch.py config:=/path/to/my.yaml

This launches the decision layer only. It expects mdrobot_rc_bridge and
mdrobot_ros2_driver's mecanum_driver_node to be running. For the whole machine
at once use mdrobot_supervisor's bringup.launch.py.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_config = os.path.join(
        get_package_share_directory("mdrobot_supervisor"), "config", "supervisor.yaml"
    )
    args = [
        DeclareLaunchArgument(
            "config", default_value=default_config,
            description="Parameter YAML file. Edit it (or pass your own) instead of CLI options.",
        ),
        DeclareLaunchArgument("namespace", default_value=""),
    ]
    node = Node(
        package="mdrobot_supervisor",
        executable="supervisor_node",
        name="mdrobot_supervisor",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[LaunchConfiguration("config")],
        remappings=[
            ("~/rc", "/mdrobot_rc_bridge/channels"),
            ("~/command", "/mdrobot_rc_bridge/command"),
            ("~/cmd_wheel_rpm", "/mdrobot_mecanum_driver/cmd_wheel_rpm"),
            ("~/joint_states", "/mdrobot_mecanum_driver/joint_states"),
            ("~/plate_offset", "/mdrobot_plate_ocr/plate_offset"),
            # No node publishes this yet; the upward camera is not fitted.
            ("~/hole_offset", "/mdrobot_hole_detector/hole_offset"),
        ],
    )
    return LaunchDescription(args + [node])
