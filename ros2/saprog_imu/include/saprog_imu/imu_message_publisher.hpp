#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "saprog_imu/imu_sample.hpp"

namespace saprog_imu
{
class ImuMessagePublisher
{
public:
  explicit ImuMessagePublisher(rclcpp::Node &node);
  void publish(
    const ImuSample &sample, const std::string &frame_id,
    const builtin_interfaces::msg::Time &stamp,
    const std::array<double, 4> &orientation, bool orientation_valid);

private:
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr orientation_publisher_;
};
}  // namespace saprog_imu
