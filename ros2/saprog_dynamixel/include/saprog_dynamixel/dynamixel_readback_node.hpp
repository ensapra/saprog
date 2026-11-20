#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "saprog_dynamixel/servo_bus.hpp"
#include "saprog_dynamixel/servo_readings_publisher.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace saprog_dynamixel
{
class DynamixelReadbackNode final : public rclcpp::Node
{
public:
  explicit DynamixelReadbackNode(const rclcpp::NodeOptions &options);

private:
  friend struct DynamixelReadbackNodeTestPeer;

  void read_next_servo();
  void process_torque_requests();
  void command_positions(const sensor_msgs::msg::JointState::ConstSharedPtr message);
  void set_servo_torque(
      uint8_t id,
      int64_t source_timestamp,
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void release_all_torque(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  bool ensure_connected(std::string &error);

  std::string device_name_;
  int baud_rate_;
  double protocol_version_;
  std::vector<int64_t> servo_ids_;
  std::vector<int64_t> zero_positions_;
  std::vector<int64_t> directions_;
  std::unordered_map<uint8_t, ServoCalibration> calibration_;
  double poll_rate_;
  int moving_speed_;
  int torque_limit_;
  std::unique_ptr<ServoBus> bus_;
  ServoReadingsPublisher publisher_;
  std::vector<ServoReading> sweep_;
  rclcpp::Time sweep_stamp_;
  std::size_t next_servo_{0};
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr position_subscription_;
  std::vector<rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr> torque_services_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr release_all_service_;
  rclcpp::Time last_release_stamp_;
  int64_t last_release_system_stamp_{0};
  bool has_release_stamp_{false};
  std::chrono::steady_clock::time_point last_connect_attempt_{};
  std::chrono::steady_clock::time_point next_temperature_publish_{};
};
}  // namespace saprog_dynamixel
