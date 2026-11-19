"""Inspect one leg with the source model's approximate knee coupling."""

from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def single_leg_description(path):
    """Extract the base and front-right branch; retain source frames and limits."""
    source = ET.parse(path).getroot()
    names = {
        "base", "front_right_11", "front_right_12",
        "front_right_joint_13", "front_right_leg",
    }
    robot = ET.Element("robot", name="saprog_front_right_inspection")
    found = set()
    for link in source.findall("link"):
        if link.get("name") in names:
            robot.append(deepcopy(link))
            found.add(link.get("name"))
    if found != names:
        raise ValueError(f"Description is missing required links: {sorted(names - found)}")
    children = set()
    for joint in source.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            raise ValueError("Description contains a joint without parent/child")
        if child.get("link") not in names - {"base"}:
            continue
        if parent.get("link") not in names:
            raise ValueError(f"Leg joint {joint.get('name')} has an unexpected parent")
        copied = deepcopy(joint)
        robot.append(copied)
        children.add(child.get("link"))
    if children != names - {"base"}:
        raise ValueError("Description does not contain the complete front-right branch")
    return ET.tostring(robot, encoding="unicode")


def launch_inspector(context):
    description_path = Path(LaunchConfiguration("description").perform(context))
    description = ParameterValue(single_leg_description(description_path), value_type=str)
    share = Path(get_package_share_directory("saprog_leg_lab"))
    namespace = "/saprog/leg_lab"
    transforms = [("/tf", f"{namespace}/tf"), ("/tf_static", f"{namespace}/tf_static")]
    viewer = Node(
        package="rviz2", executable="rviz2", name="viewer", namespace=namespace,
        arguments=["-d", str(share / "config" / "leg.rviz")],
        remappings=transforms, output="screen",
    )
    sliders = Node(
        package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
        name="joint_sliders", namespace=namespace,
        parameters=[{"robot_description": description, "rate": 30}], output="screen",
    )
    return [
        LogInfo(msg=(
            "LEG GEOMETRY INSPECTOR: three driven joints with the source model's "
            "approximate knee coupling. Joint limits use calibrated servo radians. "
            "No gravity, ground contact, or hardware commands."
        )),
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            name="state_publisher", namespace=namespace,
            parameters=[{"robot_description": description}],
            remappings=transforms, output="screen",
        ),
        RegisterEventHandler(OnProcessExit(
            target_action=viewer,
            on_exit=[EmitEvent(event=Shutdown(reason="Leg viewer closed"))],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=sliders,
            on_exit=[EmitEvent(event=Shutdown(reason="Leg controls closed"))],
        )),
        sliders,
        viewer,
    ]


def generate_launch_description():
    source_share = Path(get_package_share_directory("saprog_description"))
    return LaunchDescription([
        DeclareLaunchArgument(
            "description", default_value=str(source_share / "urdf" / "URDF.urdf"),
            description="URDF to inspect; source geometry and limits are preserved",
        ),
        OpaqueFunction(function=launch_inspector),
    ])
