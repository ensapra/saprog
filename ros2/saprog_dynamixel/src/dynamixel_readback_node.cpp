#include "saprog_dynamixel/dynamixel_readback_node.hpp"

#include <cmath>
#include <set>
#include <stdexcept>

namespace saprog_dynamixel
{
namespace
{
constexpr auto kReconnectDelay = std::chrono::seconds(2);
}

DynamixelReadbackNode::DynamixelReadbackNode(const rclcpp::NodeOptions &options)
: Node("saprog_dynamixel_readback", options),
  device_name_(get_parameter("device_name").as_string()),
  baud_rate_(static_cast<int>(get_parameter("baud_rate").as_int())),
  protocol_version_(get_parameter("protocol_version").as_double()),
  servo_ids_(get_parameter("servo_ids").as_integer_array()),
  zero_positions_(get_parameter("zero_positions").as_integer_array()),
  directions_(get_parameter("directions").as_integer_array()),
  poll_rate_(get_parameter("poll_rate").as_double()),
  publisher_(*this)
{
  if (device_name_.empty() || baud_rate_ <= 0 || !std::isfinite(protocol_version_) ||
      protocol_version_ <= 0.0 || !std::isfinite(poll_rate_) || poll_rate_ <= 0.0 ||
      poll_rate_ > 1000.0 || servo_ids_.empty() || zero_positions_.size() != servo_ids_.size() ||
      directions_.size() != servo_ids_.size()) {
    throw std::invalid_argument("Invalid DYNAMIXEL settings or servo calibration arrays");
  }

  std::set<int> unique_ids;
  for (size_t index = 0; index < servo_ids_.size(); ++index) {
    const auto id = servo_ids_[index];
    const auto zero = zero_positions_[index];
    const auto direction = directions_[index];
    if (id < 0 || id > 253 || !unique_ids.insert(id).second || zero < 0 || zero > 1023 ||
        (direction != -1 && direction != 1)) {
      throw std::invalid_argument("Check unique servo IDs, zero positions, and directions");
    }
    calibration_.emplace(static_cast<uint8_t>(id), ServoCalibration{
        static_cast<uint16_t>(zero), static_cast<int>(direction)});
  }

  bus_ = std::make_unique<ServoBus>(device_name_, baud_rate_, protocol_version_);
  sweep_.reserve(servo_ids_.size());
  const auto interval = std::chrono::duration<double>(
      1.0 / (poll_rate_ * static_cast<double>(servo_ids_.size())));
  timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(interval),
      [this]() { read_next_servo(); });
  RCLCPP_INFO(get_logger(), "Reading %zu DYNAMIXEL servos on %s at %.1f sweeps/s",
      servo_ids_.size(), device_name_.c_str(), poll_rate_);
}

void DynamixelReadbackNode::read_next_servo()
{
  if (!bus_->connected()) {
    const auto steady_now = std::chrono::steady_clock::now();
    if (steady_now - last_connect_attempt_ < kReconnectDelay) {
      return;
    }
    last_connect_attempt_ = steady_now;
    std::string error;
    if (!bus_->connect(error)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", error.c_str());
      return;
    }
    RCLCPP_INFO(get_logger(), "Connected to DYNAMIXEL bus %s at %d baud",
        device_name_.c_str(), baud_rate_);
  }

  if (next_servo_ == 0) {
    sweep_.clear();
    sweep_stamp_ = now();
  }

  ServoReading reading;
  std::string error;
  const auto id = static_cast<uint8_t>(servo_ids_[next_servo_]);
  if (bus_->read(id, reading, error)) {
    sweep_.push_back(reading);
  } else {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Servo %u read failed: %s", static_cast<unsigned>(id), error.c_str());
  }

  if (++next_servo_ == servo_ids_.size()) {
    next_servo_ = 0;
    const auto steady_now = std::chrono::steady_clock::now();
    const bool publish_temperatures = steady_now >= next_temperature_publish_;
    if (publish_temperatures) {
      next_temperature_publish_ = steady_now + std::chrono::seconds(1);
    }
    publisher_.publish(sweep_, calibration_, sweep_stamp_, publish_temperatures);
    if (sweep_.empty()) {
      bus_->disconnect();
    }
  }
}
}  // namespace saprog_dynamixel
