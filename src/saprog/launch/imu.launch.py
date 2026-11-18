"""Run the C++ IMU backend, optionally with the Python desktop display."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("device", default_value=os.environ.get("SAPROG_IMU_DEVICE", "")),
            DeclareLaunchArgument("baud_rate", default_value="115200"),
            DeclareLaunchArgument("auto_connect", default_value="true"),
            Node(
                package="saprog",
                executable="imu_node",
                output="screen",
                parameters=[
                    {
                        "device": ParameterValue(LaunchConfiguration("device"), value_type=str),
                        "baud_rate": ParameterValue(
                            LaunchConfiguration("baud_rate"), value_type=int
                        ),
                        "auto_connect": ParameterValue(
                            LaunchConfiguration("auto_connect"), value_type=bool
                        ),
                        "usb_helper": PathJoinSubstitution(
                            [
                                FindPackageShare("saprog"),
                                "scripts",
                                "ensure_imu_usb.bash",
                            ]
                        ),
                    }
                ],
            ),
            Node(
                package="saprog",
                executable="imu_monitor",
                output="screen",
                condition=IfCondition(LaunchConfiguration("gui")),
            ),
        ]
    )
