#include "saprog/dynamixel_bus.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk/group_sync_write.h"
#include "saprog/servo_units.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

DynamixelBus::DynamixelBus(const std::string &device_name, int baud_rate, double protocol_version,
                           int status_return_level, bool verify_goal_registers)
    : status_return_level_(status_return_level), verify_goal_registers_(verify_goal_registers)
{
  if (status_return_level_ != 1 && status_return_level_ != 2) {
    throw std::invalid_argument("DYNAMIXEL status_return_level must be 1 or 2");
  }
  port_handler_ = dynamixel::PortHandler::getPortHandler(device_name.c_str());
  packet_handler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version);
  if (!port_handler_->openPort()) {
    throw std::runtime_error("Could not open serial port " + device_name);
  }
  if (!port_handler_->setBaudRate(baud_rate)) {
    port_handler_->closePort();
    throw std::runtime_error("Could not set baud rate on " + device_name);
  }
}

DynamixelBus::~DynamixelBus()
{
  if (port_handler_ != nullptr) {
    port_handler_->closePort();
  }
}

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
  uint8_t device_error = 0;
  const int result = packet_handler_->readTxRx(
      port_handler_, id, 36, static_cast<uint16_t>(data.size()), data.data(), &device_error);
  if (result != COMM_SUCCESS) {
    settings_.erase(id);
    error = packet_handler_->getTxRxResult(result);
    return false;
  }
  if (device_error != 0) {
    settings_.erase(id);
    error = packet_handler_->getRxPacketError(device_error);
    return false;
  }
  return true;
}

bool DynamixelBus::get_torque(uint8_t id, bool &enabled, std::string &error)
{
  auto lock = lock_bus();
  uint8_t value = 0;
  uint8_t device_error = 0;
  const int result = packet_handler_->read1ByteTxRx(port_handler_, id, 24, &value, &device_error);
  if (result != COMM_SUCCESS) {
    settings_.erase(id);
    error = packet_handler_->getTxRxResult(result);
    return false;
  }
  if (device_error != 0) {
    settings_.erase(id);
    error = packet_handler_->getRxPacketError(device_error);
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
  uint8_t device_error = 0;
  const int result = packet_handler_->readTxRx(port_handler_, id, 6, 4, data.data(), &device_error);
  if (result != COMM_SUCCESS || device_error != 0) {
    error = result != COMM_SUCCESS ? packet_handler_->getTxRxResult(result)
                                   : packet_handler_->getRxPacketError(device_error);
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
  if (!set_setting(id, 34, static_cast<uint16_t>(torque_limit), 2, error) ||
      !set_setting(id, 32, static_cast<uint16_t>(speed), 2, error) ||
      !write_register(id, 30, position, 2, error)) {
    return false;
  }
  return set_setting(id, 24, 1, 1, error);
}

bool DynamixelBus::set_torque(uint8_t id, bool enabled, std::string &error)
{
  auto lock = lock_bus();
  return set_setting(id, 24, enabled ? 1 : 0, 1, error);
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
                                        std::string &error)
{
  TimelinePriorityGuard priority(*this);
  std::unique_lock<std::timed_mutex> lock(bus_mutex_, std::defer_lock);
  if (!lock.try_lock_until(latest_start) || std::chrono::steady_clock::now() > latest_start) {
    error = "Serial bus remained busy past the timeline command bus-wait deadline";
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
  if (!set_settings(ids, std::vector<uint16_t>(torque_limits.begin(), torque_limits.end()), 34, 2,
                    error) ||
      !set_settings(ids, speed_values, 32, 2, error)) {
    return false;
  }

  dynamixel::GroupSyncWrite group(port_handler_, packet_handler_, 30, 2);
  for (size_t i = 0; i < ids.size(); ++i) {
    uint8_t data[2] = {static_cast<uint8_t>(positions[i] & 0xff),
                       static_cast<uint8_t>((positions[i] >> 8) & 0xff)};
    if (!group.addParam(ids[i], data)) {
      error = "Could not add servo " + std::to_string(ids[i]) + " to synchronized position command";
      group.clearParam();
      return false;
    }
  }
  const int result = group.txPacket();
  group.clearParam();
  if (result != COMM_SUCCESS) {
    error = packet_handler_->getTxRxResult(result);
    return false;
  }
  if (!set_settings(ids, std::vector<uint16_t>(ids.size(), 1), 24, 1, error)) {
    return false;
  }

  if (verify_goal_registers_) {
    for (size_t i = 0; i < ids.size(); ++i) {
      std::array<uint8_t, 2> data{};
      uint8_t device_error = 0;
      const int read_result =
          packet_handler_->readTxRx(port_handler_, ids[i], 30, 2, data.data(), &device_error);
      if (read_result != COMM_SUCCESS || device_error != 0) {
        error = "servo_" + std::to_string(ids[i]) +
                " synchronized goal sent, but readback failed: " +
                (read_result != COMM_SUCCESS ? packet_handler_->getTxRxResult(read_result)
                                             : packet_handler_->getRxPacketError(device_error));
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
  return set_settings(ids, std::vector<uint16_t>(ids.size(), enabled ? 1 : 0), 24, 1, error);
}

bool DynamixelBus::set_torques_before(const std::vector<uint8_t> &ids, bool enabled,
                                      std::chrono::steady_clock::time_point latest_start,
                                      std::string &error)
{
  TimelinePriorityGuard priority(*this);
  std::unique_lock<std::timed_mutex> lock(bus_mutex_, std::defer_lock);
  if (!lock.try_lock_until(latest_start) || std::chrono::steady_clock::now() > latest_start) {
    error = "Serial bus remained busy past the timeline torque-action bus-wait deadline";
    return false;
  }
  if (ids.empty()) {
    error = "Synchronized torque command needs at least one servo";
    return false;
  }
  return set_settings(ids, std::vector<uint16_t>(ids.size(), enabled ? 1 : 0), 24, 1, error);
}

int &DynamixelBus::cached_setting(uint8_t id, uint16_t address)
{
  auto &settings = settings_[id];
  switch (address) {
  case 24:
    return settings.torque_enabled;
  case 32:
    return settings.speed;
  case 34:
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

  std::vector<std::array<uint8_t, 2>> parameters(changed_ids.size());
  dynamixel::GroupSyncWrite group(port_handler_, packet_handler_, address, size);
  for (size_t i = 0; i < changed_ids.size(); ++i) {
    parameters[i][0] = static_cast<uint8_t>(changed_values[i] & 0xff);
    parameters[i][1] = static_cast<uint8_t>((changed_values[i] >> 8) & 0xff);
    if (!group.addParam(changed_ids[i], parameters[i].data())) {
      group.clearParam();
      for (const auto id : changed_ids) {
        invalidate_setting(id, address);
      }
      error = "Could not add servo " + std::to_string(changed_ids[i]) +
              " to synchronized register write";
      return false;
    }
  }

  const int result = group.txPacket();
  group.clearParam();
  if (result != COMM_SUCCESS) {
    for (const auto id : changed_ids) {
      invalidate_setting(id, address);
    }
    error = packet_handler_->getTxRxResult(result);
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
  uint8_t device_error = 0;
  const int result =
      packet_handler_->readTxRx(port_handler_, id, address, size, data.data(), &device_error);
  const std::string context =
      "servo_" + std::to_string(id) + " register " + std::to_string(address) + ": ";
  if (result != COMM_SUCCESS) {
    error = context + "readback failed: " + packet_handler_->getTxRxResult(result);
    return false;
  }
  if (device_error != 0) {
    error = context + "readback device error: " + packet_handler_->getRxPacketError(device_error);
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
  error.clear();
  const std::string context =
      "servo_" + std::to_string(id) + " register " + std::to_string(address) + ": ";
  if (status_return_level_ == 1) {
    const int result = size == 1
                           ? packet_handler_->write1ByteTxOnly(port_handler_, id, address,
                                                               static_cast<uint8_t>(value))
                           : packet_handler_->write2ByteTxOnly(port_handler_, id, address, value);
    if (result != COMM_SUCCESS) {
      error = context + packet_handler_->getTxRxResult(result);
      return false;
    }
    return verify_register(id, address, value, size, error);
  }

  uint8_t device_error = 0;
  const int result =
      size == 1 ? packet_handler_->write1ByteTxRx(port_handler_, id, address,
                                                  static_cast<uint8_t>(value), &device_error)
                : packet_handler_->write2ByteTxRx(port_handler_, id, address, value, &device_error);
  // An acknowledged write can still time out; verify without repeating it.
  if (result == COMM_RX_TIMEOUT) {
    return verify_register(id, address, value, size, error);
  }
  if (result != COMM_SUCCESS) {
    error = context + packet_handler_->getTxRxResult(result);
    return false;
  }
  if (device_error != 0) {
    error = context + packet_handler_->getRxPacketError(device_error);
    return false;
  }
  return true;
}
