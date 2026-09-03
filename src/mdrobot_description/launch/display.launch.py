"""Show the machine in RViz, driven by the real hardware.

Run this ALONGSIDE bringup, not instead of it — it starts no drive node, no RC
bridge and no IMU, only the pieces that turn what those publish into a picture:

  robot_state_publisher   URDF + wheel angles -> the model's own TF
  odometry_node           joint_states + IMU  -> odom -> base_link, wheel angles
  rviz2                   the view

    ros2 launch mdrobot_supervisor bringup.launch.py     # one terminal
    ros2 launch mdrobot_description display.launch.py    # another

What you are looking at is not all equally true:

  heading, roll, pitch   MEASURED by the IMU. Real.
  wheel angles           MEASURED by the encoders. Real.
  x, y position          DEAD RECKONED. Mecanum rollers slip, and the slip
                         pattern where the front pair fights the rear sits in
                         the null space of the kinematics — the wheels can
                         scrub without the arithmetic noticing. The machine
                         will drift away from where RViz draws it. Do not
                         measure a distance off this picture.

Arguments:
  body_mesh, wheel_mesh
               package:// paths to the meshes. Empty string for either falls
               back to a primitive, which is useful for telling a mesh problem
               apart from a transform problem.
  mesh_scale   default "0.001 0.001 0.001": the CAD exports millimetres and
               URDF reads metres.
  wheelbase, track, wheel_radius
               default to supervisor.yaml, which is what the machine BELIEVES.
               The CAD says 0.365 / 0.395 / 0.0644 and the difference is not
               settled — see the URDF. To see whether the CAD numbers fit the
               body better:

                 ros2 launch mdrobot_description display.launch.py \
                     wheelbase:=0.365 track:=0.395

  rviz         false to publish the TF and the model without opening a window,
               which is what you want over a slow SSH link.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("mdrobot_description")
    sup_share = get_package_share_directory("mdrobot_supervisor")

    args = [
        DeclareLaunchArgument(
            "body_mesh",
            default_value="package://mdrobot_description/meshes/body.stl"),
        DeclareLaunchArgument(
            "wheel_mesh",
            default_value="package://mdrobot_description/meshes/wheel.stl"),
        DeclareLaunchArgument("mesh_scale", default_value="0.001 0.001 0.001"),
        # Settled by driving: with body_yaw_deg 0 the model crabbed sideways
        # while the machine drove straight. Add 180 if it drives backwards.
        DeclareLaunchArgument("body_yaw_deg", default_value="90"),
        DeclareLaunchArgument("wheel_yaw_deg", default_value="0"),
        # Defaults track supervisor.yaml. The CAD disagrees; the URDF says why.
        DeclareLaunchArgument("wheelbase", default_value="0.5"),
        DeclareLaunchArgument("track", default_value="0.575"),
        DeclareLaunchArgument("wheel_radius", default_value="0.0625"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument(
            "model",
            default_value=os.path.join(share, "urdf", "mdrobot.urdf.xacro")),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(share, "rviz", "mdrobot.rviz")),
        DeclareLaunchArgument(
            "supervisor_config",
            default_value=os.path.join(sup_share, "config", "supervisor.yaml")),
    ]

    # ParameterValue(..., value_type=str) or the URDF arrives as something
    # robot_state_publisher will not parse.
    robot_description = ParameterValue(
        Command([
            "xacro ", LaunchConfiguration("model"),
            " body_mesh:=", LaunchConfiguration("body_mesh"),
            " wheel_mesh:=", LaunchConfiguration("wheel_mesh"),
            " mesh_scale:='", LaunchConfiguration("mesh_scale"), "'",
            " body_yaw_deg:=", LaunchConfiguration("body_yaw_deg"),
            " wheel_yaw_deg:=", LaunchConfiguration("wheel_yaw_deg"),
            " wheelbase:=", LaunchConfiguration("wheelbase"),
            " track:=", LaunchConfiguration("track"),
            " wheel_radius:=", LaunchConfiguration("wheel_radius"),
        ]),
        value_type=str,
    )

    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
        # The wheel angles have to be the ones with the gear ratio taken out.
        remappings=[("joint_states", "/mdrobot_odometry/wheel_joint_states")],
    )
    odometry = Node(
        package="mdrobot_supervisor",
        executable="odometry_node",
        name="mdrobot_odometry",
        output="screen",
        # Geometry, gear ratio and wheel signs come from the SAME file the
        # supervisor reads, so the model cannot drift out of step with the
        # machine's own kinematics.
        parameters=[LaunchConfiguration("supervisor_config")],
        remappings=[
            ("~/joint_states", "/mdrobot_mecanum_driver/joint_states"),
            ("~/imu", "/mdrobot_imu/data"),
        ],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )
    return LaunchDescription(args + [state_publisher, odometry, rviz])
