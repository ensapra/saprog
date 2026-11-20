#include "saprog_dynamixel/servo_readings_publisher.hpp"

#include <string>

namespace saprog_dynamixel
{
ServoReadingsPublisher::ServoReadingsPublisher(rclcpp::Node &node)
: joint_state_publisher_(node.create_publisher<sensor_msgs::msg::JointState>(
      "/saprog/joint_states", rclcpp::QoS(1))),
  temperature_publisher_(node.create_publisher<sensor_msgs::msg::Temperature>(
      "/saprog/temperatures", rclcpp::QoS(100))),
  voltage_publisher_(node.create_publisher<sensor_msgs::msg::JointState>(
      "/saprog/voltages", rclcpp::QoS(10))),
  speed_publisher_(node.create_publisher<sensor_msgs::msg::JointState>(
      "/saprog/speeds", rclcpp::QoS(10)))
{}

void ServoReadingsPublisher::publish(const std::vector<ServoReading> &readings,
    const std::unordered_map<uint8_t, ServoCalibration> &calibration,
    const rclcpp::Time &stamp, bool publish_temperatures)
{
  if (readings.empty()) {
    return;
  }
  sensor_msgs::msg::JointState joint_states;
  sensor_msgs::msg::JointState voltages;
  sensor_msgs::msg::JointState speeds;
  joint_states.header.stamp = stamp;
  voltages.header.stamp = stamp;
  speeds.header.stamp = stamp;
  for (const auto &reading : readings) {
    const auto calibration_entry = calibration.find(reading.id);
    if (calibration_entry == calibration.end()) {
      continue;
    }
    const auto servo_name = "servo_" + std::to_string(reading.id);
    joint_states.name.push_back(servo_name);
    joint_states.position.push_back(
        calibration_entry->second.direction *
        (static_cast<int>(reading.position) - calibration_entry->second.zero_position) *
        (5.0 * 3.14159265358979323846 / 3.0) / 1023.0);
    joint_states.effort.push_back(reading.torque_enabled ? 1.0 : 0.0);
    voltages.name.push_back(servo_name);
    voltages.position.push_back(static_cast<double>(reading.voltage) / 10.0);
    speeds.name.push_back(servo_name);
    const auto speed_magnitude = static_cast<double>(reading.speed & 0x3ff);
    speeds.velocity.push_back((reading.speed & 0x400) ? -speed_magnitude : speed_magnitude);

    if (publish_temperatures) {
      sensor_msgs::msg::Temperature temperature;
      temperature.header.stamp = stamp;
      temperature.header.frame_id = servo_name;
      temperature.temperature = reading.temperature;
      temperature_publisher_->publish(temperature);
    }
  }
  if (!joint_states.name.empty()) {
    joint_state_publisher_->publish(joint_states);
    voltage_publisher_->publish(voltages);
    speed_publisher_->publish(speeds);
  }
}
}  // namespace saprog_dynamixel
