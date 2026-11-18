#include "saprog/readback_node.hpp"
#include "saprog/config_parameters.hpp"

#include <chrono>
#include <string>

ReadbackNode::ReadbackNode(const rclcpp::NodeOptions &options)
    : Node("saprog_readback", options), config_(read_config_parameters(*this))
{
  parameter_callback_ = add_on_set_parameters_callback(
      [](const std::vector<rclcpp::Parameter> &parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto &parameter : parameters) {
          if (is_startup_only_parameter(parameter.get_name())) {
            result.successful = false;
            result.reason = "Readback configuration is startup-only; restart the node to change it";
            return result;
          }
        }
        return result;
      });
  bus_ = std::make_shared<DynamixelBus>(config_.device_name, config_.baud_rate,
                                        config_.protocol_version, config_.status_return_level,
                                        config_.verify_goal_registers);
  state_pub_ = create_publisher<sensor_msgs::msg::JointState>("/saprog/joint_states", 10);
  temperature_pub_ = create_publisher<sensor_msgs::msg::Temperature>("/saprog/temperatures", 100);
  timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / config_.poll_rate),
                             [this]() { read_servos(); });
  RCLCPP_INFO(get_logger(), "Reading %zu configured DYNAMIXEL IDs", config_.servos.size());
}

std::shared_ptr<DynamixelBus> ReadbackNode::bus() const
{
  return bus_;
}

void ReadbackNode::read_servos()
{
  sensor_msgs::msg::JointState states;
  states.header.stamp = now();

  for (const auto &servo : config_.servos) {
    const auto id = servo.id;
    std::array<uint8_t, 8> data{};
    std::string error;
    if (!bus_->read(id, data, error)) {
      RCLCPP_WARN(get_logger(), "Servo %u: %s", id, error.c_str());
      continue;
    }

    const uint16_t raw_position = static_cast<uint16_t>(data[0] | (data[1] << 8));
    const double position = saprog::ax::to_radians(raw_position, servo.zero_position, servo.direction);
    const uint16_t speed = static_cast<uint16_t>(data[2] | (data[3] << 8));
    const uint16_t load = static_cast<uint16_t>(data[4] | (data[5] << 8));
    const double voltage = data[6] / 10.0;
    sensor_msgs::msg::Temperature temperature;
    temperature.header.stamp = states.header.stamp;
    temperature.header.frame_id = "servo_" + std::to_string(id);
    temperature.temperature = data[7];
    temperature_pub_->publish(temperature);

    states.name.push_back("servo_" + std::to_string(id));
    states.position.push_back(position);
    bool torque_enabled = false;
    if (bus_->get_torque(id, torque_enabled, error)) {
      states.effort.push_back(torque_enabled ? 1.0 : 0.0);
    } else {
      states.effort.push_back(-1.0);
      RCLCPP_WARN(get_logger(), "Servo %u torque state: %s", id, error.c_str());
    }
    RCLCPP_DEBUG(get_logger(),
                 "ID %u: position=%.3f rad (raw %u) speed=%u load=%u voltage=%.1fV temperature=%uC",
                 id, position, raw_position, speed, load, voltage, data[7]);
  }
  state_pub_->publish(states);
}
