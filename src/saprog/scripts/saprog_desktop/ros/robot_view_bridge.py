#!/usr/bin/env python3
"""Adapt Saprog servo and IMU topics to robot_state_publisher and RViz2."""

import math

import rclpy
from geometry_msgs.msg import QuaternionStamped, TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

from saprog_desktop.robot_view_model import (
    SERVO_JOINT_NAMES,
    OrientationReference,
)


class RobotViewBridge(Node):
    def __init__(self):
        super().__init__("saprog_robot_view_bridge")
        self.joint_publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.orientation = OrientationReference()
        # Complete display pose even before the first (possibly partial) readback.
        self.positions = dict.fromkeys(SERVO_JOINT_NAMES.values(), 0.0)
        # Lowest zero-pose mesh vertex is -0.18281 m in the supplied description.
        self.base_height = float(self.declare_parameter("base_height", 0.183).value)
        if not math.isfinite(self.base_height) or self.base_height < 0:
            raise ValueError("base_height must be finite and nonnegative (metres)")
        self.create_subscription(JointState, "/saprog/joint_states", self.on_joints, 10)
        self.create_subscription(
            QuaternionStamped,
            "/saprog/imu/orientation",
            self.on_orientation,
            qos_profile_sensor_data,
        )
        self.create_timer(0.05, self.publish_base_transform)

    def on_joints(self, message):
        for index, name in enumerate(message.name):
            if not name.startswith("servo_"):
                continue
            try:
                servo_id = int(name.removeprefix("servo_"))
            except ValueError:
                continue
            urdf_name = SERVO_JOINT_NAMES.get(servo_id)
            if urdf_name is None or index >= len(message.position):
                continue
            position = message.position[index]
            if math.isfinite(position):
                # Logical servo radians use the same convention as the URDF.
                self.positions[urdf_name] = position

    def on_orientation(self, message):
        q = message.quaternion
        self.orientation.update((q.x, q.y, q.z, q.w))

    def publish_base_transform(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "world"
        transform.child_frame_id = "base"
        transform.transform.translation.z = self.base_height
        q = transform.transform.rotation
        q.x, q.y, q.z, q.w = self.orientation.current()
        self.tf_broadcaster.sendTransform(transform)
        # Refresh all TF branches at the same time as the base, including mimic
        # joints computed by robot_state_publisher. This is display state only.
        result = JointState()
        result.header.stamp = transform.header.stamp
        result.name = list(self.positions)
        result.position = list(self.positions.values())
        self.joint_publisher.publish(result)


def main():
    rclpy.init()
    node = RobotViewBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
