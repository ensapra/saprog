#!/usr/bin/env python3
"""Adapt Saprog servo and IMU topics to robot_state_publisher and RViz2."""

import rclpy
from geometry_msgs.msg import QuaternionStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

from saprog_desktop.robot_view_model import OrientationReference, SERVO_JOINT_NAMES


class RobotViewBridge(Node):
    def __init__(self):
        super().__init__("saprog_robot_view_bridge")
        self.joint_publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.orientation = OrientationReference()
        self.create_subscription(JointState, "/saprog/joint_states", self.on_joints, 10)
        self.create_subscription(
            QuaternionStamped,
            "/saprog/imu/orientation",
            self.on_orientation,
            qos_profile_sensor_data,
        )
        self.create_timer(0.05, self.publish_base_transform)

    def on_joints(self, message):
        result = JointState()
        result.header = message.header
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
            result.name.append(urdf_name)
            result.position.append(message.position[index])
            if len(message.velocity) == len(message.name):
                result.velocity.append(message.velocity[index])
            if len(message.effort) == len(message.name):
                result.effort.append(message.effort[index])
        if result.name:
            self.joint_publisher.publish(result)

    def on_orientation(self, message):
        q = message.quaternion
        self.orientation.update((q.x, q.y, q.z, q.w))

    def publish_base_transform(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "world"
        transform.child_frame_id = "base"
        q = transform.transform.rotation
        q.x, q.y, q.z, q.w = self.orientation.current()
        self.tf_broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = RobotViewBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
