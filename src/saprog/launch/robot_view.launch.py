"""Launch the OpenGL RViz2 view of Saprog's live robot state."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description_share = Path(get_package_share_directory("saprog_description"))
    saprog_share = Path(get_package_share_directory("saprog"))
    urdf_path = description_share / "urdf" / "URDF.urdf"
    rviz_path = saprog_share / "config" / "robot_view.rviz"
    robot_description = urdf_path.read_text(encoding="utf-8")

    embedded_renderer = Node(
        package="saprog",
        executable="robot_view_embed",
        name="saprog_embedded_rviz",
        output="screen",
        arguments=[
            LaunchConfiguration("parent_window"),
            LaunchConfiguration("width"),
            LaunchConfiguration("height"),
            str(rviz_path),
            LaunchConfiguration("ready_file"),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("parent_window", default_value="0"),
            DeclareLaunchArgument("width", default_value="1"),
            DeclareLaunchArgument("height", default_value="1"),
            DeclareLaunchArgument("ready_file", default_value=""),
            SetEnvironmentVariable("QT_QPA_PLATFORM", "xcb"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description, "use_sim_time": False}],
            ),
            Node(
                package="saprog",
                executable="robot_view_bridge",
                name="saprog_robot_view_bridge",
                output="screen",
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=embedded_renderer,
                    on_exit=[EmitEvent(event=Shutdown(reason="embedded RViz renderer exited"))],
                )
            ),
            embedded_renderer,
        ]
    )
