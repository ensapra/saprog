#include "saprog_dynamixel/dynamixel_readback_node.hpp"

#include <cmath>
#include <algorithm>
#include <unordered_set>
#include <set>
#include <stdexcept>
#include <sstream>

#include "rcl/service.h"

namespace saprog_dynamixel
{
namespace
{
constexpr auto kReconnectDelay = std::chrono::seconds(2);
constexpr double kTickRadians = (5.0 * 3.14159265358979323846 / 3.0) / 1023.0;
constexpr double kSpeedUnitRadians = 0.111 * 2.0 * 3.14159265358979323846 / 60.0;
}

DynamixelReadbackNode::DynamixelReadbackNode(const rclcpp::NodeOptions &options)
: Node("saprog_dynamixel_control", options),
  device_name_(get_parameter("device_name").as_string()),
  baud_rate_(static_cast<int>(get_parameter("baud_rate").as_int())),
  protocol_version_(get_parameter("protocol_version").as_double()),
  servo_ids_(get_parameter("servo_ids").as_integer_array()),
  zero_positions_(get_parameter("zero_positions").as_integer_array()),
  directions_(get_parameter("directions").as_integer_array()),
  poll_rate_(get_parameter("poll_rate").as_double()),
  moving_speed_(static_cast<int>(get_parameter("moving_speed").as_int())),
  torque_limit_(static_cast<int>(get_parameter("torque_limit").as_int())),
  publisher_(*this)
{
  if (device_name_.empty() || baud_rate_ <= 0 || !std::isfinite(protocol_version_) ||
      protocol_version_ <= 0.0 || !std::isfinite(poll_rate_) || poll_rate_ <= 0.0 ||
      poll_rate_ > 1000.0 || servo_ids_.empty() || zero_positions_.size() != servo_ids_.size() ||
      directions_.size() != servo_ids_.size() || moving_speed_ < 0 || moving_speed_ > 1023 ||
      torque_limit_ < 0 || torque_limit_ > 1023) {
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

  const auto status_return_level = static_cast<int>(get_parameter("status_return_level").as_int());
  bus_ = std::make_unique<ServoBus>(device_name_, baud_rate_, protocol_version_, status_return_level);
  position_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      "/saprog/position_command", rclcpp::QoS(10),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr message) {
        command_positions(message);
      });
  for (const auto id : servo_ids_) {
    // Take these requests ourselves between reads: rclcpp service callbacks
    // expose only the request ID and discard its middleware source timestamp.
    // Do not register a second request consumer with the executor.
    rclcpp::AnyServiceCallback<std_srvs::srv::SetBool> callback;
    auto service_options = rcl_service_get_default_options();
    torque_services_.push_back(std::make_shared<rclcpp::Service<std_srvs::srv::SetBool>>(
        get_node_base_interface()->get_shared_rcl_node_handle(),
        "/saprog/servo_" + std::to_string(id) + "/torque",
        callback, service_options));
  }
  release_all_service_ = create_service<std_srvs::srv::Trigger>(
      "/saprog/release_all_torque",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        release_all_torque(request, response);
      });
  limits_service_ = create_service<std_srvs::srv::Trigger>(
      "/saprog/read_servo_limits",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        read_servo_limits(response);
      });
  sweep_.reserve(servo_ids_.size());
  const auto interval = std::chrono::duration<double>(
      1.0 / (poll_rate_ * static_cast<double>(servo_ids_.size())));
  timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(interval),
      [this]() { read_next_servo(); });
  RCLCPP_INFO(get_logger(), "Reading and controlling %zu DYNAMIXEL servos on %s at %.1f sweeps/s",
      servo_ids_.size(), device_name_.c_str(), poll_rate_);
}

bool DynamixelReadbackNode::ensure_connected(std::string &error)
{
  if (bus_->connected()) {
    return true;
  }
  if (!bus_->connect(error)) {
    return false;
  }
  RCLCPP_INFO(get_logger(), "Connected to DYNAMIXEL bus %s at %d baud",
      device_name_.c_str(), baud_rate_);
  return true;
}

void DynamixelReadbackNode::command_positions(
    const sensor_msgs::msg::JointState::ConstSharedPtr message)
{
  if (message->header.stamp.sec < 0 || message->header.stamp.nanosec >= 1000000000u) {
    RCLCPP_WARN(get_logger(), "Discarding position command with an invalid timestamp");
    return;
  }
  if (has_release_stamp_) {
    const rclcpp::Time command_stamp(message->header.stamp, get_clock()->get_clock_type());
    if (command_stamp <= last_release_stamp_) {
      RCLCPP_INFO(get_logger(), "Discarding position command queued before Release all torque");
      return;
    }
  }
  if (message->name.empty() || message->name.size() != message->position.size() ||
      (!message->velocity.empty() && message->velocity.size() != message->name.size()) ||
      (!message->effort.empty() && message->effort.size() != message->name.size())) {
    RCLCPP_WARN(get_logger(), "Position command requires matching joint, position, speed, and torque arrays");
    return;
  }

  std::vector<uint8_t> ids;
  std::vector<uint16_t> positions;
  std::vector<uint16_t> speeds;
  std::vector<uint16_t> torque_limits;
  std::unordered_set<int64_t> seen;
  for (std::size_t index = 0; index < message->name.size(); ++index) {
    const auto &name = message->name[index];
    if (name.rfind("servo_", 0) != 0) {
      RCLCPP_WARN(get_logger(), "Unknown joint '%s'", name.c_str());
      return;
    }
    int64_t id = -1;
    try {
      std::size_t parsed = 0;
      id = std::stoll(name.substr(6), &parsed);
      if (parsed != name.size() - 6) {
        throw std::invalid_argument("invalid servo name");
      }
    } catch (const std::exception &) {
      RCLCPP_WARN(get_logger(), "Invalid joint name '%s'", name.c_str());
      return;
    }
    const auto configured = std::find(servo_ids_.begin(), servo_ids_.end(), id);
    if (configured == servo_ids_.end() || !seen.insert(id).second ||
        !std::isfinite(message->position[index])) {
      RCLCPP_WARN(get_logger(), "Unknown, duplicate, or invalid position for joint '%s'", name.c_str());
      return;
    }
    const auto servo_index = static_cast<std::size_t>(configured - servo_ids_.begin());
    const double raw = static_cast<double>(zero_positions_[servo_index]) +
        static_cast<double>(directions_[servo_index]) * message->position[index] / kTickRadians;
    if (!std::isfinite(raw) || raw < 0.0 || raw > 1023.0) {
      RCLCPP_WARN(get_logger(), "Position for %s exceeds the AX raw position range", name.c_str());
      return;
    }
    int speed = moving_speed_;
    if (!message->velocity.empty()) {
      const double requested = message->velocity[index] / kSpeedUnitRadians;
      if (!std::isfinite(requested) || requested < 1.0 - 1e-9 || requested > 1023.0 + 1e-9) {
        RCLCPP_WARN(get_logger(), "Speed for %s must map to AX speed 1..1023", name.c_str());
        return;
      }
      speed = static_cast<int>(std::lround(requested));
    }
    int torque = torque_limit_;
    if (!message->effort.empty()) {
      const double requested = message->effort[index];
      if (!std::isfinite(requested) || requested < 0.0 || requested > 1023.0) {
        RCLCPP_WARN(get_logger(), "Torque strength for %s must be 0..1023", name.c_str());
        return;
      }
      torque = static_cast<int>(std::lround(requested));
    }
    ids.push_back(static_cast<uint8_t>(id));
    positions.push_back(static_cast<uint16_t>(std::lround(raw)));
    speeds.push_back(static_cast<uint16_t>(speed));
    torque_limits.push_back(static_cast<uint16_t>(torque));
  }

  std::string error;
  if (!ensure_connected(error) || !bus_->set_positions(ids, positions, speeds, torque_limits, error)) {
    RCLCPP_ERROR(get_logger(), "Could not send DYNAMIXEL position command: %s", error.c_str());
    return;
  }
  RCLCPP_INFO(get_logger(), "Commanded %zu DYNAMIXEL servo(s)", ids.size());
}

void DynamixelReadbackNode::set_servo_torque(
    uint8_t id,
    int64_t source_timestamp,
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (request->data && has_release_stamp_ && source_timestamp <= last_release_system_stamp_) {
    response->success = false;
    response->message = "Torque enable cancelled by Release all torque; send a new request";
    return;
  }
  std::string error;
  response->success = ensure_connected(error) && bus_->set_torque(id, request->data, error);
  response->message = response->success
      ? std::string("Torque ") + (request->data ? "enabled" : "disabled")
      : error;
  if (!response->success) {
    RCLCPP_ERROR(get_logger(), "Could not set servo_%u torque: %s", id, error.c_str());
  }
}

void DynamixelReadbackNode::process_torque_requests()
{
  // Limit each service to one request per poll so a busy client cannot prevent
  // the executor from handling Release all torque or position commands.
  for (std::size_t index = 0; index < torque_services_.size(); ++index) {
    const auto &service = torque_services_[index];
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    rmw_service_info_t info{};
    const auto result = rcl_take_request_with_info(
        service->get_service_handle().get(), &info, request.get());
    if (result == RCL_RET_SERVICE_TAKE_FAILED) {
      continue;
    }
    if (result != RCL_RET_OK) {
      RCLCPP_ERROR(get_logger(), "Could not take torque request: %s", rcl_get_error_string().str);
      rcl_reset_error();
      continue;
    }
    auto response = std::make_shared<std_srvs::srv::SetBool::Response>();
    set_servo_torque(static_cast<uint8_t>(servo_ids_[index]), info.source_timestamp, request, response);
    service->send_response(info.request_id, *response);
  }
}

void DynamixelReadbackNode::release_all_torque(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  // The timestamp barrier also rejects position messages that were already
  // published but had not reached this node when Release all torque was called.
  last_release_stamp_ = now();
  // DDS source timestamps use system time, even when this node uses ROS time.
  last_release_system_stamp_ = rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds();
  has_release_stamp_ = true;
  std::vector<uint8_t> ids;
  ids.reserve(servo_ids_.size());
  for (const auto id : servo_ids_) {
    ids.push_back(static_cast<uint8_t>(id));
  }
  std::string error;
  response->success = ensure_connected(error) && bus_->set_torques(ids, false, error);
  response->message = response->success
      ? "Queued commands cancelled and torque disabled for all configured servos"
      : error;
  if (!response->success) {
    RCLCPP_ERROR(get_logger(), "Could not release all DYNAMIXEL torque: %s", error.c_str());
  } else {
    RCLCPP_INFO(get_logger(), "Released torque for all %zu configured servos", ids.size());
  }
}

void DynamixelReadbackNode::read_servo_limits(
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  std::string error;
  if (!ensure_connected(error)) {
    response->message = error;
    return;
  }
  // This callback shares the polling callback group: only the port owner reads
  // the EEPROM, and no register writes or torque changes are needed.
  std::ostringstream report;
  report << "{\"servos\":[";
  for (std::size_t index = 0; index < servo_ids_.size(); ++index) {
    const auto id = static_cast<uint8_t>(servo_ids_[index]);
    uint16_t lower = 0, upper = 0;
    if (!bus_->read_limits(id, lower, upper, error)) {
      response->message = "servo_" + std::to_string(id) + ": " + error;
      return;
    }
    if (index != 0) { report << ','; }
    report << "{\"servo_id\":" << static_cast<int>(id)
           << ",\"zero_position\":" << zero_positions_[index]
           << ",\"direction\":" << directions_[index]
           << ",\"raw_lower\":" << lower << ",\"raw_upper\":" << upper << '}';
  }
  report << "]}";
  response->message = report.str();
  response->success = true;
}

void DynamixelReadbackNode::read_next_servo()
{
  process_torque_requests();
  if (!bus_->connected()) {
    const auto steady_now = std::chrono::steady_clock::now();
    if (steady_now - last_connect_attempt_ < kReconnectDelay) {
      return;
    }
    last_connect_attempt_ = steady_now;
    std::string error;
    if (!ensure_connected(error)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", error.c_str());
      return;
    }
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
