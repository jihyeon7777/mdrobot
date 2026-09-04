"""Indicator LEDs, driven from the topics that already exist.

Run alongside bringup and the plate reader:

    ros2 launch mdrobot_supervisor bringup.launch.py
    ros2 launch mdrobot_plate_ocr plate_ocr.launch.py
    ros2 launch mdrobot_gpio led.launch.py

Options live in config/led.yaml. Nothing else depends on this node, and
nothing stops working without it — a lamp is an indicator, not an interlock.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_config = os.path.join(
        get_package_share_directory("mdrobot_gpio"), "config", "led.yaml"
    )
    args = [
        DeclareLaunchArgument("config", default_value=default_config),
    ]
    node = Node(
        package="mdrobot_gpio",
        executable="led_node",
        name="mdrobot_led",
        output="both",
        parameters=[LaunchConfiguration("config")],
        remappings=[
            ("~/plate_offset", "/mdrobot_plate_ocr/plate_offset"),
            ("~/command", "/mdrobot_rc_bridge/command"),
        ],
    )
    return LaunchDescription(args + [node])
