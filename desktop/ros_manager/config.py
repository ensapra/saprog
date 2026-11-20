"""Shared ROS topic and provider configuration for desktop widgets."""

from dataclasses import dataclass


@dataclass(frozen=True)
class TopicConfig:
    """Describe one topic and the package executable that provides it."""

    topic: str
    message_type: str
    package: str
    executable: str


IMU_RAW = TopicConfig(
    topic="/saprog/imu/data_raw",
    message_type="sensor_msgs/msg/Imu",
    package="saprog_imu",
    executable="imu_publisher_node",
)
IMU_MAGNETOMETER = TopicConfig(
    topic="/saprog/imu/mag",
    message_type="sensor_msgs/msg/MagneticField",
    package="saprog_imu",
    executable="imu_publisher_node",
)
IMU_ORIENTATION = TopicConfig(
    topic="/saprog/imu/orientation",
    message_type="geometry_msgs/msg/QuaternionStamped",
    package="saprog_imu",
    executable="imu_publisher_node",
)

IMU_TOPICS = (IMU_RAW, IMU_MAGNETOMETER, IMU_ORIENTATION)
