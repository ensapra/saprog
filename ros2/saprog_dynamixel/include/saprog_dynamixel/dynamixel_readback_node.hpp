#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "saprog_dynamixel/servo_bus.hpp"
#include "saprog_dynamixel/servo_readings_publisher.hpp"

namespace saprog_dynamixel
{
class DynamixelReadbackNode final : public rclcpp::Node
{
public:
  explicit DynamixelReadbackNode(const rclcpp::NodeOptions &options);

private:
  void read_next_servo();

  std::string device_name_;
  int baud_rate_;
  double protocol_version_;
  std::vector<int64_t> servo_ids_;
  std::vector<int64_t> zero_positions_;
  std::vector<int64_t> directions_;
  std::unordered_map<uint8_t, ServoCalibration> calibration_;
  double poll_rate_;
  std::unique_ptr<ServoBus> bus_;
  ServoReadingsPublisher publisher_;
  std::vector<ServoReading> sweep_;
  rclcpp::Time sweep_stamp_;
  std::size_t next_servo_{0};
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::steady_clock::time_point last_connect_attempt_{};
  std::chrono::steady_clock::time_point next_temperature_publish_{};
};
}  // namespace saprog_dynamixel
