"""Data types shared by the desktop ROS manager components."""

from dataclasses import dataclass


@dataclass(frozen=True)
class NodeRequirement:
    """ROS package executable that can provide a requested topic."""

    package: str
    executable: str


@dataclass(frozen=True)
class TopicRequest:
    """One widget's request for a ROS topic."""

    topic: str
    message_type: str
    requirement: NodeRequirement
