"""Load the installed URDF into a paused, hardware-independent physics world."""

import math
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start_world(context):
    share = Path(get_package_share_directory("saprog_simulation"))
    description = Path(get_package_share_directory("saprog_description"))
    # Gazebo resolves package://saprog_description against its resource paths.
    # Import the authoritative installed URDF directly, without rewriting it.
    resource_path = os.pathsep.join(filter(None, [
        str(description.parent), os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
    ]))
    command = ["gz", "sim", "-v", "3", str(share / "worlds" / "inspection.sdf")]
    if LaunchConfiguration("gui").perform(context).lower() == "false":
        command.append("-s")
    simulator = ExecuteProcess(cmd=command, output="screen")
    spawn = Node(
        package="ros_gz_sim", executable="create", namespace="saprog_simulation",
        output="screen", arguments=[
            "-world", "saprog_simulation", "-name", "saprog",
            "-file", str(description / "urdf" / "URDF.urdf"),
            "-z", "0.5", "-P", str(math.pi),
        ],
    )

    def spawned(event, context):
        if event.returncode:
            return [EmitEvent(event=Shutdown(reason="Robot spawn failed; check Gazebo output"))]
        marker = LaunchConfiguration("ready_file").perform(context)
        if marker:
            Path(marker).touch()
        return []

    return [
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        RegisterEventHandler(OnProcessExit(target_action=simulator, on_exit=[
            EmitEvent(event=Shutdown(reason="Gazebo closed")),
        ])),
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=spawned)),
        simulator, spawn,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("ready_file", default_value=""),
        OpaqueFunction(function=start_world),
    ])
