#pragma once

#include <vector>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "saprog_dynamixel/servo_bus.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/temperature.hpp"

namespace saprog_dynamixel
{
struct ServoCalibration
{
  uint16_t zero_position{};
  int direction{1};
};

class ServoReadingsPublisher
{
public:
  explicit ServoReadingsPublisher(rclcpp::Node &node);
  void publish(
      const std::vector<ServoReading> &readings,
      const std::unordered_map<uint8_t, ServoCalibration> &calibration,
      const rclcpp::Time &stamp, bool publish_temperatures);

private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr voltage_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr speed_publisher_;
};
}  // namespace saprog_dynamixel
