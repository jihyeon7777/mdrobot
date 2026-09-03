"""Launch the HWT901B attitude sensor node.

All options live in config/imu.yaml — edit that file instead of passing them on
the command line:
  ros2 launch mdrobot_imu imu.launch.py
Use a different parameter file with:
  ros2 launch mdrobot_imu imu.launch.py config:=/path/to/my.yaml

Watch it with:
  ros2 topic echo /mdrobot_imu/attitude_deg   # roll/pitch/yaw degrees, readable
  ros2 topic echo /mdrobot_imu/data           # the full sensor_msgs/Imu
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_config = os.path.join(
        get_package_share_directory("mdrobot_imu"), "config", "imu.yaml"
    )
    args = [
        DeclareLaunchArgument(
            "config", default_value=default_config,
            description="Parameter YAML file. Edit it (or pass your own) instead of CLI options.",
        ),
        DeclareLaunchArgument("namespace", default_value=""),
    ]
    node = Node(
        package="mdrobot_imu",
        executable="imu_node",
        name="mdrobot_imu",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[LaunchConfiguration("config")],
    )
    return LaunchDescription(args + [node])
