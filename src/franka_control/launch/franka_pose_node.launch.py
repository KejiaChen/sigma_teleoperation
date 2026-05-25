from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    use_rviz = LaunchConfiguration("use_rviz")
    command_mode = LaunchConfiguration("command_mode")

    rviz_file = os.path.join(
        get_package_share_directory("franka_description"),
        "rviz",
        "visualize_franka.rviz",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="Logging level",
            ),
            DeclareLaunchArgument(
                "robot_ip",
                default_value="192.168.5.11",
                description="Hostname or IP address of the robot.",
            ),
            DeclareLaunchArgument(
                "command_mode",
                default_value="pose",
                description="Command mode: pose or velocity.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="Visualize the robot in Rviz",
            ),
            Node(
                package="franka_control",
                executable="franka_pose_node",
                namespace="franka_control1",
                name="franka_pose",
                output="screen",
                arguments=[
                    "--robot-ip",
                    robot_ip,
                    "--ros-args",
                    "--log-level",
                    LaunchConfiguration("log_level"),
                ],
                parameters=[{"command_mode": command_mode}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["--display-config", rviz_file],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
