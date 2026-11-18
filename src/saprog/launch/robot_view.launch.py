"""Launch the OpenGL RViz2 view of Saprog's live robot state."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    description_share = Path(get_package_share_directory("saprog_description"))
    saprog_share = Path(get_package_share_directory("saprog"))
    urdf_path = description_share / "urdf" / "URDF.urdf"
    rviz_path = saprog_share / "config" / "robot_view.rviz"
    robot_description = urdf_path.read_text(encoding="utf-8")

    return LaunchDescription([
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            name="robot_state_publisher", output="screen",
            parameters=[{"robot_description": robot_description, "use_sim_time": False}],
        ),
        Node(
            package="saprog", executable="robot_view_bridge",
            name="saprog_robot_view_bridge", output="screen",
        ),
        Node(
            package="rviz2", executable="rviz2", name="rviz2", output="screen",
            arguments=["-d", str(rviz_path)],
        ),
    ])
