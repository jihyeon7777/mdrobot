"""Launch the four-wheel mecanum drive node.

Both MD controllers share one RS485 bus, so this is a single node rather than
one per controller — two processes on the same port would interleave their
Modbus frames on the wire.

  ros2 launch mdrobot_ros2_driver mecanum.launch.py
  ros2 launch mdrobot_ros2_driver mecanum.launch.py config:=/path/to/my.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_config = os.path.join(
        get_package_share_directory("mdrobot_ros2_driver"), "config", "mecanum.yaml"
    )
    args = [
        DeclareLaunchArgument(
            "config", default_value=default_config,
            description="Parameter YAML file. Edit it (or pass your own) instead of CLI options.",
        ),
        DeclareLaunchArgument("namespace", default_value=""),
    ]
    node = Node(
        package="mdrobot_ros2_driver",
        executable="mecanum_driver_node",
        name="mdrobot_mecanum_driver",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[LaunchConfiguration("config")],
    )
    return LaunchDescription(args + [node])
