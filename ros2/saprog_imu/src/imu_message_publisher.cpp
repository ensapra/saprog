#include "saprog_imu/imu_message_publisher.hpp"

namespace saprog_imu
{
ImuMessagePublisher::ImuMessagePublisher(rclcpp::Node &node)
{
  imu_publisher_ = node.create_publisher<sensor_msgs::msg::Imu>(
    "/saprog/imu/data_raw", rclcpp::SensorDataQoS());
  mag_publisher_ = node.create_publisher<sensor_msgs::msg::MagneticField>(
    "/saprog/imu/mag", rclcpp::SensorDataQoS());
  orientation_publisher_ = node.create_publisher<geometry_msgs::msg::QuaternionStamped>(
    "/saprog/imu/orientation", rclcpp::SensorDataQoS());
}

void ImuMessagePublisher::publish(
  const ImuSample &sample, const std::string &frame_id,
  const builtin_interfaces::msg::Time &stamp,
  const std::array<double, 4> &orientation, bool orientation_valid)
{
  sensor_msgs::msg::Imu imu;
  imu.header.stamp = stamp;
  imu.header.frame_id = frame_id;
  imu.orientation_covariance[0] = -1.0;
  constexpr double gravity = 9.80665;
  constexpr double degrees_to_radians = 0.017453292519943295;
  imu.linear_acceleration.x = sample.acceleration[0] * gravity;
  imu.linear_acceleration.y = sample.acceleration[1] * gravity;
  imu.linear_acceleration.z = sample.acceleration[2] * gravity;
  imu.angular_velocity.x = sample.angular_velocity[0] * degrees_to_radians;
  imu.angular_velocity.y = sample.angular_velocity[1] * degrees_to_radians;
  imu.angular_velocity.z = sample.angular_velocity[2] * degrees_to_radians;
  imu_publisher_->publish(imu);

  sensor_msgs::msg::MagneticField mag;
  mag.header = imu.header;
  constexpr double microtesla_to_tesla = 1e-6;
  mag.magnetic_field.x = sample.magnetic_field[0] * microtesla_to_tesla;
  mag.magnetic_field.y = sample.magnetic_field[1] * microtesla_to_tesla;
  mag.magnetic_field.z = sample.magnetic_field[2] * microtesla_to_tesla;
  mag_publisher_->publish(mag);

  if (orientation_valid) {
    geometry_msgs::msg::QuaternionStamped orientation_message;
    orientation_message.header.stamp = stamp;
    orientation_message.header.frame_id = "imu_world";
    orientation_message.quaternion.w = orientation[0];
    orientation_message.quaternion.x = orientation[1];
    orientation_message.quaternion.y = orientation[2];
    orientation_message.quaternion.z = orientation[3];
    orientation_publisher_->publish(orientation_message);
  }
}
}  // namespace saprog_imu
