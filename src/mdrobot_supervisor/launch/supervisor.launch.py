"""Launch the decision layer.

All options live in config/supervisor.yaml — edit that file instead of passing
them on the command line:
  ros2 launch mdrobot_supervisor supervisor.launch.py
Use a different parameter file with:
  ros2 launch mdrobot_supervisor supervisor.launch.py config:=/path/to/my.yaml

This launches the decision layer only. It expects mdrobot_rc_bridge below it and
two mdrobot_ros2_driver nodes (one per MD controller) above the RS485 bus, with
the remappings wired to match.
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
            ("~/cmd_velocity_1", "/md1/cmd_velocity"),
            ("~/cmd_velocity_2", "/md2/cmd_velocity"),
        ],
    )
    return LaunchDescription(args + [node])
