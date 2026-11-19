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
  state_pub_ = create_publisher<sensor_msgs::msg::JointState>("/saprog/joint_states", 1);
  temperature_pub_ = create_publisher<sensor_msgs::msg::Temperature>("/saprog/temperatures", 100);
  timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / (config_.poll_rate * config_.servos.size())),
                             [this]() { read_servos(); });
  RCLCPP_INFO(get_logger(), "Reading %zu configured DYNAMIXEL IDs", config_.servos.size());
}

std::shared_ptr<DynamixelBus> ReadbackNode::bus() const
{
  return bus_;
}

void ReadbackNode::read_servos()
{
  // Yield to command/service callbacks after every servo, including a timeout.
  // Publish only readings collected in this sweep; never reuse a failed read.
  if (next_servo_ == 0) {
    sweep_ = sensor_msgs::msg::JointState{};
    sweep_.header.stamp = now();
    const auto started = std::chrono::steady_clock::now();
    publish_temperatures_ = started >= next_temperature_;
    if (publish_temperatures_) {
      next_temperature_ = started + std::chrono::seconds(1);
    }
  }
  const auto &servo = config_.servos[next_servo_];
  std::array<uint8_t, 8> data{};
  std::string error;
  bool torque_enabled = false;
  if (bus_->read_state(servo.id, data, torque_enabled, error)) {
    const uint16_t raw_position = static_cast<uint16_t>(data[0] | (data[1] << 8));
    sweep_.name.push_back("servo_" + std::to_string(servo.id));
    sweep_.position.push_back(
        saprog::ax::to_radians(raw_position, servo.zero_position, servo.direction));
    sweep_.effort.push_back(torque_enabled ? 1.0 : 0.0);
    // Temperature changes slowly; keep its publication at 1 Hz per servo
    // rather than flooding desktop callbacks with 240 temperature messages/s.
    if (publish_temperatures_) {
      sensor_msgs::msg::Temperature temperature;
      temperature.header.stamp = now();
      temperature.header.frame_id = sweep_.name.back();
      temperature.temperature = data[7];
      temperature_pub_->publish(temperature);
    }
  } else {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Servo %u: %s", servo.id, error.c_str());
  }
  if (++next_servo_ == config_.servos.size()) {
    next_servo_ = 0;
    state_pub_->publish(sweep_);
  }
  // Do not run an overdue timer continuously when USB replies time out.
  // Leave a scheduling gap so services and position commands remain runnable.
  if (timer_->time_until_trigger().count() <= 0) {
    timer_->reset();
  }
}
