#!/usr/bin/env python3
"""Adapt Saprog servo and IMU topics to robot_state_publisher and RViz2."""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from saprog_desktop.robot_view_model import (
    SERVO_JOINT_NAMES,
    OrientationReference,
    apply_imu_mounting_rotation,
)


class RobotViewBridge(Node):
    def __init__(self):
        super().__init__("saprog_robot_view_bridge")
        self.joint_publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.compass_publisher = self.create_publisher(
            MarkerArray, "/saprog/robot_view/compass", 10
        )
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
            Imu,
            "/saprog/imu/data",
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
        if message.orientation_covariance[0] < 0:
            return
        q = message.orientation
        quaternion = (q.x, q.y, q.z, q.w)
        self.orientation.update(quaternion)

    def publish_compass(self):
        array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        # A fixed ground reference beside the robot; its arrow points magnetic north.
        center_x, center_y = 0.33, -0.33

        def marker(marker_id, marker_type, x, y, z, scale, color):
            item = Marker()
            item.header.stamp = stamp
            item.header.frame_id = "world"
            item.ns = "magnetic_compass"
            item.id = marker_id
            item.type = marker_type
            item.action = Marker.ADD
            item.pose.position.x = x
            item.pose.position.y = y
            item.pose.position.z = z
            item.pose.orientation.w = 1.0
            item.scale.x, item.scale.y, item.scale.z = scale
            item.color.r, item.color.g, item.color.b = color
            item.color.a = 1.0
            return item

        obsolete_ring = marker(
            0, Marker.LINE_STRIP, center_x, center_y, 0.006,
            (0.0, 0.0, 0.0), (0.0, 0.0, 0.0),
        )
        obsolete_ring.action = Marker.DELETE
        array.markers.append(obsolete_ring)

        # North is magnetic world +X; this thin arrow lies flat on the ground.
        array.markers.append(marker(
            1, Marker.ARROW, center_x, center_y, 0.004,
            (0.20, 0.010, 0.008), (0.25, 0.78, 0.95),
        ))
        obsolete_label = marker(
            2, Marker.TEXT_VIEW_FACING, center_x, center_y, 0.0,
            (0.0, 0.0, 0.0), (0.0, 0.0, 0.0),
        )
        obsolete_label.action = Marker.DELETE
        array.markers.append(obsolete_label)
        for marker_id in range(3, 8):
            obsolete = marker(marker_id, Marker.ARROW, center_x, center_y, 0.0,
                              (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
            obsolete.action = Marker.DELETE
            array.markers.append(obsolete)
        self.compass_publisher.publish(array)

    def publish_base_transform(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "world"
        transform.child_frame_id = "base"
        transform.transform.translation.z = self.base_height
        q = transform.transform.rotation
        q.x, q.y, q.z, q.w = apply_imu_mounting_rotation(self.orientation.current())
        self.tf_broadcaster.sendTransform(transform)
        self.publish_compass()
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
