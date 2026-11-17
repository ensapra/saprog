#pragma once

#include <cstddef>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

struct SaprogConfig
{
  std::string device_name;
  int baud_rate;
  double protocol_version;
  double poll_rate;
  int moving_speed;
  int torque_limit;
  double target_tolerance;
  int status_return_level;
  bool verify_goal_registers;
  std::vector<uint8_t> servo_ids;
  std::vector<uint16_t> zero_positions;
  std::vector<int> directions;
  std::vector<uint16_t> lower_limits;
  std::vector<uint16_t> upper_limits;
};

inline SaprogConfig read_config(rclcpp::Node & node)
{
  SaprogConfig config;
  config.device_name = node.get_parameter("device_name").as_string();
  const auto baud_rate = node.get_parameter("baud_rate").as_int();
  config.protocol_version = node.get_parameter("protocol_version").as_double();
  config.poll_rate = node.get_parameter("poll_rate").as_double();
  const auto moving_speed = node.get_parameter("moving_speed").as_int();
  const auto torque_limit = node.get_parameter("torque_limit").as_int();
  config.target_tolerance = node.get_parameter("target_tolerance").as_double();
  const auto status_return_level = node.get_parameter("status_return_level").as_int();
  config.verify_goal_registers = node.get_parameter("verify_goal_registers").as_bool();
  const auto ids = node.get_parameter("servo_ids").as_integer_array();
  const auto zeros = node.get_parameter("zero_positions").as_integer_array();
  const auto directions = node.get_parameter("directions").as_integer_array();

  if (config.device_name.empty() || baud_rate <= 0 ||
    baud_rate > std::numeric_limits<int>::max() || config.protocol_version != 1.0 ||
    config.poll_rate <= 0.0 || moving_speed < 0 || moving_speed > 1023 ||
    torque_limit < 0 || torque_limit > 1023 || !std::isfinite(config.target_tolerance) ||
    config.target_tolerance < 0.0 || config.target_tolerance > 5.0 * 3.14159265358979323846 / 3.0 || ids.empty() ||
    (status_return_level != 1 && status_return_level != 2) ||
    zeros.size() != ids.size() || directions.size() != ids.size())
  {
    throw std::invalid_argument("Invalid DYNAMIXEL settings in config/saprog.yaml");
  }
  config.baud_rate = static_cast<int>(baud_rate);
  config.moving_speed = static_cast<int>(moving_speed);
  config.torque_limit = static_cast<int>(torque_limit);
  config.status_return_level = static_cast<int>(status_return_level);

  for (size_t i = 0; i < ids.size(); ++i) {
    if (ids[i] < 0 || ids[i] > 253 || zeros[i] < 0 || zeros[i] > 1023 ||
      (directions[i] != -1 && directions[i] != 1))
    {
      throw std::invalid_argument("Check servo_ids, zero_positions, and directions in config/saprog.yaml");
    }
    config.servo_ids.push_back(static_cast<uint8_t>(ids[i]));
    config.zero_positions.push_back(static_cast<uint16_t>(zeros[i]));
    config.directions.push_back(static_cast<int>(directions[i]));
  }
  return config;
}
