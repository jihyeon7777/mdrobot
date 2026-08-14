"""Launch the licence plate OCR node.

All options live in config/plate_ocr.yaml — edit that file instead of passing them
on the command line:
  ros2 launch mdrobot_plate_ocr plate_ocr.launch.py
Use a different parameter file with:
  ros2 launch mdrobot_plate_ocr plate_ocr.launch.py config:=/path/to/my.yaml

Read the results with:
  ros2 topic echo /mdrobot_plate_ocr/plate          # confirmed plates (latched)
  ros2 topic echo /mdrobot_plate_ocr/plate_detail   # every attempt, rejects too
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_config = os.path.join(
        get_package_share_directory("mdrobot_plate_ocr"), "config", "plate_ocr.yaml"
    )
    args = [
        DeclareLaunchArgument(
            "config", default_value=default_config,
            description="Parameter YAML file. Edit it (or pass your own) instead of CLI options.",
        ),
        DeclareLaunchArgument("namespace", default_value=""),
    ]
    node = Node(
        package="mdrobot_plate_ocr",
        executable="plate_ocr_node",
        name="mdrobot_plate_ocr",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[LaunchConfiguration("config")],
    )
    return LaunchDescription(args + [node])
