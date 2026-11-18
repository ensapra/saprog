#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "saprog/config.hpp"
#include "saprog/dynamixel_bus.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/temperature.hpp"

class ReadbackNode : public rclcpp::Node
{
public:
  explicit ReadbackNode(const rclcpp::NodeOptions &options);
  std::shared_ptr<DynamixelBus> bus() const;

private:
  void read_servos();

  SaprogConfig config_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
  std::shared_ptr<DynamixelBus> bus_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
