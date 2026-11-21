"""Shared ROS topic and provider configuration for desktop widgets."""

from dataclasses import dataclass


@dataclass(frozen=True)
class TopicConfig:
    """Describe one topic and the package executable that provides it."""

    topic: str
    message_type: str
    package: str
    executable: str
    data_type: str = ""


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

DYNAMIXEL_TEMPERATURE = TopicConfig(
    "/saprog/temperatures", "sensor_msgs/msg/Temperature", "saprog_dynamixel",
    "dynamixel_control_node", "temperature",
)
DYNAMIXEL_VOLTAGE = TopicConfig(
    "/saprog/voltages", "sensor_msgs/msg/JointState", "saprog_dynamixel",
    "dynamixel_control_node", "voltage",
)
DYNAMIXEL_SPEED = TopicConfig(
    "/saprog/speeds", "sensor_msgs/msg/JointState", "saprog_dynamixel",
    "dynamixel_control_node", "speed",
)
DYNAMIXEL_TOPICS = (DYNAMIXEL_TEMPERATURE, DYNAMIXEL_VOLTAGE, DYNAMIXEL_SPEED)

SIMULATION_STATUS = TopicConfig(
    "/saprog/simulation/status", "std_msgs/msg/String", "saprog_simulation", "simulation_node",
)

LEG_TEST_STATUS = TopicConfig(
    "/saprog/leg_test/status", "std_msgs/msg/String", "saprog_leg_test", "leg_test_node",
)
