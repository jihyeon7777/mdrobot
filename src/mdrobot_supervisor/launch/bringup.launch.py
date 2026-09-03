"""Bring up the whole machine: RC bridge, drive, and the decision layer.

  ros2 launch mdrobot_supervisor bringup.launch.py

Four nodes, one per concern:

  mdrobot_rc_bridge       ttyACM   reports the operator, relays equipment commands
  mdrobot_mecanum_driver  FTDI     owns the RS485 bus, turns rpm into motor writes
  mdrobot_imu             CH340    attitude, so the machine knows which way it is
                                   pointing once it is out of sight under a car
  mdrobot_supervisor               decides everything, talks to all three

The two USB-serial adapters share the /dev/ttyUSB numbering space, so both the
drive and the IMU configs address theirs by a by-id path. Getting them the wrong
way round points the IMU driver at the motor controllers.

Every node runs with output="both", so what it prints goes to the terminal AND
to ~/.ros/log/<run>/<node>-stdout.log. With "screen" a node that dies takes its
reason with it the moment the terminal scrolls -- and this machine is reached
over SSH, where that is most of the time. The launch log records only that a
process died and its exit code, never why.

Each keeps its own parameter file; pass rc_config, drive_config, imu_config or
supervisor_config to override one. The plate reader is NOT started here — run
mdrobot_plate_ocr separately, since it is only needed for autonomous mode.

The IMU is started unconditionally but USED only when auto_yaw_hold is on in
supervisor.yaml: publishing attitude nobody reads costs nothing, and it means
the survey tool and the diagnostics have a live sensor to look at without a
second launch file.
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
    imu_share = get_package_share_directory("mdrobot_imu")
    sup_share = get_package_share_directory("mdrobot_supervisor")

    args = [
        DeclareLaunchArgument(
            "rc_config",
            default_value=os.path.join(rc_share, "config", "rc_bridge.yaml")),
        DeclareLaunchArgument(
            "drive_config",
            default_value=os.path.join(drive_share, "config", "mecanum.yaml")),
        DeclareLaunchArgument(
            "imu_config",
            default_value=os.path.join(imu_share, "config", "imu.yaml")),
        DeclareLaunchArgument(
            "supervisor_config",
            default_value=os.path.join(sup_share, "config", "supervisor.yaml")),
    ]

    rc_bridge = Node(
        package="mdrobot_rc_bridge",
        executable="rc_bridge_node",
        name="mdrobot_rc_bridge",
        output="both",
        parameters=[LaunchConfiguration("rc_config")],
    )
    drive = Node(
        package="mdrobot_ros2_driver",
        executable="mecanum_driver_node",
        name="mdrobot_mecanum_driver",
        output="both",
        parameters=[LaunchConfiguration("drive_config")],
    )
    imu = Node(
        package="mdrobot_imu",
        executable="imu_node",
        name="mdrobot_imu",
        output="both",
        parameters=[LaunchConfiguration("imu_config")],
    )
    supervisor = Node(
        package="mdrobot_supervisor",
        executable="supervisor_node",
        name="mdrobot_supervisor",
        output="both",
        parameters=[LaunchConfiguration("supervisor_config")],
        remappings=[
            ("~/rc", "/mdrobot_rc_bridge/channels"),
            ("~/command", "/mdrobot_rc_bridge/command"),
            ("~/cmd_wheel_rpm", "/mdrobot_mecanum_driver/cmd_wheel_rpm"),
            ("~/joint_states", "/mdrobot_mecanum_driver/joint_states"),
            ("~/plate_offset", "/mdrobot_plate_ocr/plate_offset"),
            ("~/imu", "/mdrobot_imu/data"),
            # No publisher yet; the upward camera is not fitted.
            ("~/hole_offset", "/mdrobot_hole_detector/hole_offset"),
        ],
    )
    return LaunchDescription(args + [rc_bridge, drive, imu, supervisor])
