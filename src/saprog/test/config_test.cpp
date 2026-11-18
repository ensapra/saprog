#include <cmath>
#include <limits>
#include <stdexcept>

#include "saprog/config.hpp"

namespace
{
void check(bool condition)
{
  if (!condition) {
    throw std::runtime_error("Configuration regression check failed");
  }
}

void read_with(double poll_rate, const std::vector<int64_t> &ids)
{
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  options.parameter_overrides({
      rclcpp::Parameter("device_name", "unused"),
      rclcpp::Parameter("baud_rate", 1000000),
      rclcpp::Parameter("protocol_version", 1.0),
      rclcpp::Parameter("poll_rate", poll_rate),
      rclcpp::Parameter("moving_speed", 1023),
      rclcpp::Parameter("torque_limit", 1023),
      rclcpp::Parameter("target_tolerance", 0.1),
      rclcpp::Parameter("status_return_level", 1),
      rclcpp::Parameter("verify_goal_registers", false),
      rclcpp::Parameter("servo_ids", ids),
      rclcpp::Parameter("zero_positions", std::vector<int64_t>(ids.size(), 512)),
      rclcpp::Parameter("directions", std::vector<int64_t>(ids.size(), -1)),
  });
  rclcpp::Node node("config_test", options);
  const auto config = read_config(node);
  check(config.servo_ids.size() == ids.size());
}

void rejects(double rate, const std::vector<int64_t> &ids)
{
  try {
    read_with(rate, ids);
  } catch (const std::invalid_argument &) {
    return;
  }
  throw std::runtime_error("Invalid configuration was accepted");
}
} // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  read_with(1.0, {11, 12});
  rejects(1.0, {11, 11});
  for (const double rate : {0.0, -1.0, 1e-300, 1e300, std::numeric_limits<double>::quiet_NaN(),
                            std::numeric_limits<double>::infinity()}) {
    rejects(rate, {11, 12});
  }
  // Exercise both calibration directions, including encoder extremes.
  for (const int direction : {-1, 1}) {
    for (const uint16_t raw : {0, 350, 512, 1023}) {
      const double radians = saprog::ax::to_radians(raw, 512, direction);
      check(std::abs(saprog::ax::to_raw(radians, 512, direction) - raw) < 1e-9);
    }
  }
  rclcpp::shutdown();
  return 0;
}
