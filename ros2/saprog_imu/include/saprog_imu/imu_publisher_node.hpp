#pragma once

#include <chrono>
#include <string>

#include "saprog_imu/imu_message_publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "saprog_imu/imu_orientation_estimator.hpp"
#include "saprog_imu/serial_imu_reader.hpp"

namespace saprog_imu
{
class ImuPublisherNode : public rclcpp::Node
{
public:
  ImuPublisherNode();

private:
  void poll_serial();

  std::string device_;
  std::string frame_id_;
  int baud_rate_{};
  SerialImuReader serial_reader_;
  ImuOrientationEstimator orientation_estimator_;
  ImuMessagePublisher publisher_;
  std::chrono::steady_clock::time_point last_open_attempt_{};
  std::chrono::steady_clock::time_point connection_opened_{};
  std::chrono::steady_clock::time_point last_valid_sample_{};
  std::chrono::steady_clock::time_point last_received_data_{};
  bool resume_toggle_sent_{false};
  bool orientation_stale_{false};
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace saprog_imu
