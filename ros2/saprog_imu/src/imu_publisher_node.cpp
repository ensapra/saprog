#include "saprog_imu/imu_publisher_node.hpp"

#include <cstdlib>
#include <memory>
#include <vector>

#include "saprog_imu/imu_csv_parser.hpp"

namespace saprog_imu
{
ImuPublisherNode::ImuPublisherNode()
: Node("saprog_imu_publisher"),
  device_([this]() {
    const char *environment_device = std::getenv("SAPROG_IMU_DEVICE");
    return declare_parameter<std::string>("device", environment_device ? environment_device : "/dev/ttyACM0");
  }()),
  frame_id_(declare_parameter<std::string>("frame_id", "imu_link")),
  baud_rate_(declare_parameter<int>("baud_rate", 115200)),
  serial_reader_(device_, baud_rate_),
  orientation_estimator_(declare_parameter<bool>("use_magnetometer", false)),
  publisher_(*this)
{
  timer_ = create_wall_timer(std::chrono::milliseconds(5), [this]() { poll_serial(); });
  RCLCPP_INFO(get_logger(), "Publishing raw IMU data from %s", device_.c_str());
}

void ImuPublisherNode::poll_serial()
{
  if (!serial_reader_.connected()) {
    const auto steady_now = std::chrono::steady_clock::now();
    if (steady_now - last_open_attempt_ < std::chrono::seconds(2)) {
      return;
    }
    last_open_attempt_ = steady_now;
    std::string error;
    if (!serial_reader_.connect(error)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Cannot open IMU serial device %s: %s", device_.c_str(), error.c_str());
      return;
    }
    RCLCPP_INFO(get_logger(), "Connected to IMU serial device %s", device_.c_str());
    connection_opened_ = steady_now;
    last_valid_sample_ = steady_now;
    last_received_data_ = steady_now;
    resume_toggle_sent_ = false;
    orientation_stale_ = false;
    orientation_estimator_.reset();
  }

  std::vector<std::string> lines;
  std::string error;
  if (!serial_reader_.read_lines(lines, error)) {
    RCLCPP_WARN(get_logger(), "IMU serial read failed: %s", error.c_str());
    return;
  }
  const auto steady_now = std::chrono::steady_clock::now();
  if (serial_reader_.received_data_this_read()) {
    last_received_data_ = steady_now;
  }
  const bool valid_sample_stale = steady_now - last_valid_sample_ > std::chrono::seconds(3);
  if (valid_sample_stale && !orientation_stale_) {
    orientation_estimator_.reset();
    orientation_stale_ = true;
  }
  if (steady_now - last_received_data_ > std::chrono::seconds(3))
  {
    RCLCPP_WARN(get_logger(), "No IMU data received; reconnecting");
    serial_reader_.disconnect();
    orientation_estimator_.reset();
    return;
  }
  if (!resume_toggle_sent_ &&
    steady_now - connection_opened_ > std::chrono::milliseconds(1500) &&
    steady_now - last_received_data_ > std::chrono::milliseconds(1500))
  {
    resume_toggle_sent_ = true;
    std::string toggle_error;
    if (!serial_reader_.send_resume_toggle(toggle_error)) {
      RCLCPP_WARN(get_logger(), "Could not send IMU resume toggle: %s", toggle_error.c_str());
    }
  }
  if (!error.empty()) {
    RCLCPP_WARN(get_logger(), "Discarding %s", error.c_str());
  }
  for (const auto &line : lines) {
    ImuSample sample;
    if (parse_imu_csv(line, sample)) {
      last_valid_sample_ = std::chrono::steady_clock::now();
      orientation_stale_ = false;
      orientation_estimator_.update(sample);
      publisher_.publish(sample, frame_id_, now(),
        orientation_estimator_.orientation(), orientation_estimator_.valid());
    }
  }
}
}  // namespace saprog_imu

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<saprog_imu::ImuPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
