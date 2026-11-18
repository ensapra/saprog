#pragma once

#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "saprog/config.hpp"

// ROS parameter decoding lives at the adapter boundary; SaprogConfig and its
// validation remain usable by pure tests and non-ROS command planning.
inline SaprogConfig read_config_parameters(rclcpp::Node &node)
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

  if (baud_rate <= 0 || baud_rate > std::numeric_limits<int>::max() || ids.empty() ||
      zeros.size() != ids.size() || directions.size() != ids.size()) {
    throw std::invalid_argument("Invalid DYNAMIXEL settings in config/saprog.yaml");
  }
  config.baud_rate = static_cast<int>(baud_rate);
  config.moving_speed = static_cast<int>(moving_speed);
  config.torque_limit = static_cast<int>(torque_limit);
  config.status_return_level = static_cast<int>(status_return_level);
  for (size_t i = 0; i < ids.size(); ++i) {
    if (ids[i] < 0 || ids[i] > 253 || zeros[i] < 0 || zeros[i] > 1023 ||
        (directions[i] != -1 && directions[i] != 1)) {
      throw std::invalid_argument(
          "Check unique servo_ids, zero_positions, and directions in config/saprog.yaml");
    }
    config.servos.push_back({static_cast<uint8_t>(ids[i]), static_cast<uint16_t>(zeros[i]),
                             static_cast<int>(directions[i])});
  }
  validate_config(config);
  return config;
}

inline SaprogConfig read_config(rclcpp::Node &node) { return read_config_parameters(node); }
