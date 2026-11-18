#include "saprog/dynamixel_bus.hpp"
#include "saprog/dynamixel_registers.hpp"
#include "saprog/servo_units.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

DynamixelBus::DynamixelBus(const std::string &device_name, int baud_rate, double protocol_version,
                           int status_return_level, bool verify_goal_registers)
    : verify_goal_registers_(verify_goal_registers),
      transport_(device_name, baud_rate, protocol_version, status_return_level)
{
}

DynamixelBus::~DynamixelBus() = default;

std::unique_lock<std::timed_mutex> DynamixelBus::lock_bus()
{
  std::unique_lock<std::mutex> priority_lock(timeline_priority_mutex_);
  timeline_priority_cv_.wait(priority_lock, [this]() { return !timeline_priority_requested_; });
  return std::unique_lock<std::timed_mutex>(bus_mutex_);
}

DynamixelBus::TimelinePriorityGuard::TimelinePriorityGuard(DynamixelBus &bus) : bus_(bus)
{
  std::lock_guard<std::mutex> lock(bus_.timeline_priority_mutex_);
  bus_.timeline_priority_requested_ = true;
}

DynamixelBus::TimelinePriorityGuard::~TimelinePriorityGuard()
{
  {
    std::lock_guard<std::mutex> lock(bus_.timeline_priority_mutex_);
    bus_.timeline_priority_requested_ = false;
  }
  bus_.timeline_priority_cv_.notify_all();
}

bool DynamixelBus::read(uint8_t id, std::array<uint8_t, 8> &data, std::string &error)
{
  auto lock = lock_bus();
  if (!transport_.read(id, saprog::ax_register::present_position.address,
                       static_cast<uint16_t>(data.size()), data.data(), error)) {
    settings_.erase(id);
    return false;
  }
  return true;
}

bool DynamixelBus::get_torque(uint8_t id, bool &enabled, std::string &error)
{
  auto lock = lock_bus();
  uint8_t value = 0;
  if (!transport_.read(id, saprog::ax_register::torque_enable.address,
                       saprog::ax_register::torque_enable.width, &value, error)) {
    settings_.erase(id);
    return false;
  }
  enabled = value != 0;
  auto &settings = settings_[id];
  if (settings.torque_enabled == 1 && !enabled) {
    settings.torque_limit = -1;
    settings.speed = -1;
  }
  settings.torque_enabled = enabled ? 1 : 0;
  return true;
}

bool DynamixelBus::get_limits(uint8_t id, uint16_t &lower, uint16_t &upper, std::string &error)
{
  auto lock = lock_bus();
  error.clear();
  std::array<uint8_t, 4> data{};
  if (!transport_.read(id, saprog::ax_register::angle_limits.address,
                       saprog::ax_register::angle_limits.width, data.data(), error)) {
    return false;
  }
  lower = static_cast<uint16_t>(data[0] | (data[1] << 8));
  upper = static_cast<uint16_t>(data[2] | (data[3] << 8));
  if (lower >= upper || upper > 1023) {
    error = "Invalid joint limits or wheel mode";
    return false;
  }
  return true;
}

bool DynamixelBus::set_position(uint8_t id, uint16_t position, int speed, int torque_limit,
                                std::string &error)
{
  auto lock = lock_bus();
  if (!set_setting(id, saprog::ax_register::torque_limit.address,
                   static_cast<uint16_t>(torque_limit), saprog::ax_register::torque_limit.width,
                   error) ||
      !set_setting(id, saprog::ax_register::moving_speed.address, static_cast<uint16_t>(speed),
                   saprog::ax_register::moving_speed.width, error) ||
      !write_register(id, saprog::ax_register::goal_position.address, position,
                      saprog::ax_register::goal_position.width, error)) {
    return false;
  }
  return set_setting(id, saprog::ax_register::torque_enable.address, 1,
                     saprog::ax_register::torque_enable.width, error);
}

bool DynamixelBus::set_torque(uint8_t id, bool enabled, std::string &error)
{
  auto lock = lock_bus();
  return set_setting(id, saprog::ax_register::torque_enable.address, enabled ? 1 : 0,
                     saprog::ax_register::torque_enable.width, error);
}

bool DynamixelBus::set_positions(const std::vector<uint8_t> &ids,
                                 const std::vector<uint16_t> &positions, int speed,
                                 const std::vector<int> &torque_limits, double tolerance_rad,
                                 std::string &error)
{
  return set_positions(ids, positions, std::vector<int>(ids.size(), speed), torque_limits,
                       tolerance_rad, error);
}

bool DynamixelBus::set_positions(const std::vector<uint8_t> &ids,
                                 const std::vector<uint16_t> &positions,
                                 const std::vector<int> &speeds,
                                 const std::vector<int> &torque_limits, double tolerance_rad,
                                 std::string &error)
{
  auto lock = lock_bus();
  return set_positions_locked(ids, positions, speeds, torque_limits, tolerance_rad, error);
}

bool DynamixelBus::set_positions_before(const std::vector<uint8_t> &ids,
                                        const std::vector<uint16_t> &positions,
                                        const std::vector<int> &speeds,
                                        const std::vector<int> &torque_limits, double tolerance_rad,
                                        std::chrono::steady_clock::time_point latest_start,
                                        std::string &error,
                                        const std::function<bool()> &cancellation_requested)
{
  TimelinePriorityGuard priority(*this);
  std::unique_lock<std::timed_mutex> lock(bus_mutex_, std::defer_lock);
  if (!lock.try_lock_until(latest_start) || std::chrono::steady_clock::now() > latest_start) {
    error = "Serial bus remained busy past the timeline command bus-wait deadline";
    return false;
  }
  if (cancellation_requested && cancellation_requested()) {
    error = "Timeline canceled while waiting for the serial bus";
    return false;
  }
  return set_positions_locked(ids, positions, speeds, torque_limits, tolerance_rad, error);
}

bool DynamixelBus::set_positions_locked(const std::vector<uint8_t> &ids,
                                        const std::vector<uint16_t> &positions,
                                        const std::vector<int> &speeds,
                                        const std::vector<int> &torque_limits, double tolerance_rad,
                                        std::string &error)
{
  if (ids.empty() || positions.size() != ids.size() || speeds.size() != ids.size() ||
      torque_limits.size() != ids.size()) {
    error = "Synchronized position command has mismatched servo data";
    return false;
  }
  if (std::any_of(positions.begin(), positions.end(),
                  [](uint16_t value) { return value > 1023; }) ||
      std::any_of(speeds.begin(), speeds.end(),
                  [](int value) { return value < 0 || value > 1023; }) ||
      std::any_of(torque_limits.begin(), torque_limits.end(),
                  [](int value) { return value < 0 || value > 1023; })) {
    error = "Synchronized position, speed, and torque values must be from 0 to 1023";
    return false;
  }
  std::vector<uint16_t> speed_values(speeds.begin(), speeds.end());
  if (!set_settings(ids, std::vector<uint16_t>(torque_limits.begin(), torque_limits.end()),
                    saprog::ax_register::torque_limit.address,
                    saprog::ax_register::torque_limit.width, error) ||
      !set_settings(ids, speed_values, saprog::ax_register::moving_speed.address,
                    saprog::ax_register::moving_speed.width, error)) {
    return false;
  }

  if (!transport_.sync_write(ids, positions, saprog::ax_register::goal_position.address,
                             saprog::ax_register::goal_position.width, error)) {
    return false;
  }
  if (!set_settings(ids, std::vector<uint16_t>(ids.size(), 1),
                    saprog::ax_register::torque_enable.address,
                    saprog::ax_register::torque_enable.width, error)) {
    return false;
  }

  if (verify_goal_registers_) {
    for (size_t i = 0; i < ids.size(); ++i) {
      std::array<uint8_t, 2> data{};
      std::string read_error;
      if (!transport_.read(ids[i], saprog::ax_register::goal_position.address,
                           saprog::ax_register::goal_position.width, data.data(), read_error)) {
        error = "servo_" + std::to_string(ids[i]) +
                " synchronized goal sent, but readback failed: " + read_error;
        return false;
      }
      const uint16_t observed = static_cast<uint16_t>(data[0] | (data[1] << 8));
      const double tolerance_raw = tolerance_rad / saprog::ax::tick_radians;
      if (std::abs(static_cast<int>(observed) - static_cast<int>(positions[i])) > tolerance_raw) {
        error = "servo_" + std::to_string(ids[i]) +
                " synchronized goal readback mismatch (expected " + std::to_string(positions[i]) +
                ", got " + std::to_string(observed) + ", tolerance " +
                std::to_string(tolerance_raw) + " raw ticks)";
        return false;
      }
    }
  }
  error.clear();
  return true;
}

bool DynamixelBus::set_torques(const std::vector<uint8_t> &ids, bool enabled, std::string &error)
{
  auto lock = lock_bus();
  if (ids.empty()) {
    error = "Synchronized torque command needs at least one servo";
    return false;
  }
  return set_settings(ids, std::vector<uint16_t>(ids.size(), enabled ? 1 : 0),
                      saprog::ax_register::torque_enable.address,
                      saprog::ax_register::torque_enable.width, error);
}

bool DynamixelBus::set_torques_before(const std::vector<uint8_t> &ids, bool enabled,
                                      std::chrono::steady_clock::time_point latest_start,
                                      std::string &error,
                                      const std::function<bool()> &cancellation_requested)
{
  TimelinePriorityGuard priority(*this);
  std::unique_lock<std::timed_mutex> lock(bus_mutex_, std::defer_lock);
  if (!lock.try_lock_until(latest_start) || std::chrono::steady_clock::now() > latest_start) {
    error = "Serial bus remained busy past the timeline torque-action bus-wait deadline";
    return false;
  }
  if (cancellation_requested && cancellation_requested()) {
    error = "Timeline canceled while waiting for the serial bus";
    return false;
  }
  if (ids.empty()) {
    error = "Synchronized torque command needs at least one servo";
    return false;
  }
  return set_settings(ids, std::vector<uint16_t>(ids.size(), enabled ? 1 : 0),
                      saprog::ax_register::torque_enable.address,
                      saprog::ax_register::torque_enable.width, error);
}

int &DynamixelBus::cached_setting(uint8_t id, uint16_t address)
{
  auto &settings = settings_[id];
  switch (address) {
  case saprog::ax_register::torque_enable.address:
    return settings.torque_enabled;
  case saprog::ax_register::moving_speed.address:
    return settings.speed;
  case saprog::ax_register::torque_limit.address:
    return settings.torque_limit;
  default:
    throw std::invalid_argument("Unsupported cached DYNAMIXEL setting register");
  }
}

void DynamixelBus::invalidate_setting(uint8_t id, uint16_t address)
{
  cached_setting(id, address) = -1;
}

bool DynamixelBus::set_setting(uint8_t id, uint16_t address, uint16_t value, uint16_t size,
                               std::string &error)
{
  auto &cached = cached_setting(id, address);
  if (cached == value) {
    error.clear();
    return true;
  }
  if (!write_register(id, address, value, size, error)) {
    cached = -1;
    return false;
  }
  cached = value;
  return true;
}

bool DynamixelBus::set_settings(const std::vector<uint8_t> &ids,
                                const std::vector<uint16_t> &values, uint16_t address,
                                uint16_t size, std::string &error)
{
  if (ids.size() != values.size() || (size != 1 && size != 2)) {
    error = "Synchronized register write has mismatched servo data";
    return false;
  }
  std::vector<uint8_t> changed_ids;
  std::vector<uint16_t> changed_values;
  for (size_t i = 0; i < ids.size(); ++i) {
    if (cached_setting(ids[i], address) != values[i]) {
      changed_ids.push_back(ids[i]);
      changed_values.push_back(values[i]);
    }
  }
  if (changed_ids.empty()) {
    error.clear();
    return true;
  }

  if (!transport_.sync_write(changed_ids, changed_values, address, size, error)) {
    for (const auto id : changed_ids) {
      invalidate_setting(id, address);
    }
    return false;
  }
  for (const auto id : changed_ids) {
    invalidate_setting(id, address);
  }
  for (size_t i = 0; i < changed_ids.size(); ++i) {
    if (!verify_register(changed_ids[i], address, changed_values[i], size, error)) {
      return false;
    }
    cached_setting(changed_ids[i], address) = changed_values[i];
  }
  error.clear();
  return true;
}

bool DynamixelBus::verify_register(uint8_t id, uint16_t address, uint16_t value, uint16_t size,
                                   std::string &error)
{
  std::array<uint8_t, 2> data{};
  const std::string context =
      "servo_" + std::to_string(id) + " register " + std::to_string(address) + ": ";
  if (!transport_.read(id, address, size, data.data(), error)) {
    const auto device_error = error.find("device error: ");
    error = context + (device_error == std::string::npos ? "readback failed: "
                                                         : "readback device error: ") +
            (device_error == std::string::npos ? error : error.substr(device_error + 14));
    return false;
  }
  const uint16_t observed = static_cast<uint16_t>(data[0] | (data[1] << 8));
  if (observed != value) {
    error = context + "readback mismatch (expected " + std::to_string(value) + ", got " +
            std::to_string(observed) + ").";
    return false;
  }
  return true;
}

bool DynamixelBus::write_register(uint8_t id, uint16_t address, uint16_t value, uint16_t size,
                                  std::string &error)
{
  return transport_.write(id, address, value, size, error);
}
