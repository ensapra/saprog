#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "saprog/servo_units.hpp"

struct ServoCalibration
{
  uint8_t id;
  uint16_t zero_position;
  int direction;
  uint16_t lower_limit{0};
  uint16_t upper_limit{1023};
};

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
  // Ordered once from startup parameters; measured limits are filled by the
  // controller after it reads them from the hardware.
  std::vector<ServoCalibration> servos;
};

inline void validate_config(const SaprogConfig &config)
{
  const double poll_period = 1.0 / config.poll_rate;
  const double maximum_period =
      std::chrono::duration<double>(std::chrono::nanoseconds::max()).count();
  if (config.device_name.empty() || config.baud_rate <= 0 || config.protocol_version != 1.0 ||
      !std::isfinite(config.poll_rate) || config.poll_rate <= 0.0 || !std::isfinite(poll_period) ||
      poll_period < 1e-9 || poll_period >= maximum_period || config.moving_speed < 0 ||
      config.moving_speed > 1023 || config.torque_limit < 0 || config.torque_limit > 1023 ||
      !std::isfinite(config.target_tolerance) || config.target_tolerance < 0.0 ||
      config.target_tolerance > saprog::ax::position_range_radians || config.servos.empty() ||
      (config.status_return_level != 1 && config.status_return_level != 2)) {
    throw std::invalid_argument("Invalid DYNAMIXEL settings in config/saprog.yaml");
  }
  std::vector<bool> seen(254, false);
  for (const auto &servo : config.servos) {
    if (servo.id > 253 || (servo.direction != -1 && servo.direction != 1) ||
        servo.zero_position > 1023 || seen[servo.id] || servo.lower_limit >= servo.upper_limit ||
        servo.upper_limit > 1023) {
      throw std::invalid_argument(
          "Check unique servo_ids, zero_positions, and directions in config/saprog.yaml");
    }
    seen[servo.id] = true;
  }
}

inline std::vector<uint8_t> configured_servo_ids(const SaprogConfig &config)
{
  std::vector<uint8_t> ids;
  ids.reserve(config.servos.size());
  for (const auto &servo : config.servos) {
    ids.push_back(servo.id);
  }
  return ids;
}

inline bool is_startup_only_parameter(std::string_view name)
{
  constexpr std::string_view names[] = {"device_name",       "baud_rate", "protocol_version",
                                        "poll_rate",         "moving_speed", "torque_limit",
                                        "target_tolerance",  "status_return_level",
                                        "verify_goal_registers", "servo_ids", "zero_positions",
                                        "directions"};
  for (const auto candidate : names) {
    if (name == candidate) {
      return true;
    }
  }
  return false;
}

inline size_t configured_servo_index(const SaprogConfig &config, std::string_view joint_name)
{
  for (size_t i = 0; i < config.servos.size(); ++i) {
    const std::string expected = "servo_" + std::to_string(config.servos[i].id);
    if (joint_name == expected) {
      return i;
    }
  }
  return config.servos.size();
}
