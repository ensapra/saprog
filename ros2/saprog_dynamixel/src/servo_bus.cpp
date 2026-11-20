#include "saprog_dynamixel/servo_bus.hpp"

#include <cstddef>
#include <array>
#include <algorithm>
#include <stdexcept>
#include <vector>
#include <utility>
#include "dynamixel_sdk/group_sync_write.h"

namespace saprog_dynamixel
{
namespace
{
constexpr uint16_t kTelemetryStartAddress = 24;
constexpr uint16_t kTelemetryLength = 20;
constexpr uint16_t kTorqueEnableAddress = 24;
constexpr uint16_t kGoalPositionAddress = 30;
constexpr uint16_t kMovingSpeedAddress = 32;
constexpr uint16_t kTorqueLimitAddress = 34;

uint16_t word_at(const std::vector<uint8_t> &data, std::size_t offset)
{
  return static_cast<uint16_t>(data[offset] | (static_cast<uint16_t>(data[offset + 1]) << 8));
}

bool write_register(dynamixel::PortHandler *port, dynamixel::PacketHandler *packet,
                    int status_return_level, uint8_t id, uint16_t address,
                    uint16_t value, uint16_t width, std::string &error)
{
  int result = COMM_TX_FAIL;
  uint8_t device_error = 0;
  if (status_return_level == 1) {
    result = width == 1
        ? packet->write1ByteTxOnly(port, id, address, static_cast<uint8_t>(value))
        : packet->write2ByteTxOnly(port, id, address, value);
  } else {
    result = width == 1
        ? packet->write1ByteTxRx(port, id, address, static_cast<uint8_t>(value), &device_error)
        : packet->write2ByteTxRx(port, id, address, value, &device_error);
  }
  if (result != COMM_SUCCESS) {
    error = packet->getTxRxResult(result);
    return false;
  }
  if (device_error != 0) {
    error = packet->getRxPacketError(device_error);
    return false;
  }
  error.clear();
  return true;
}

bool sync_write(dynamixel::PortHandler *port, dynamixel::PacketHandler *packet,
                uint16_t address, uint16_t width, const std::vector<uint8_t> &ids,
                const std::vector<uint16_t> &values, std::string &error)
{
  dynamixel::GroupSyncWrite group(port, packet, address, width);
  std::vector<std::array<uint8_t, 2>> parameters(ids.size());
  for (std::size_t index = 0; index < ids.size(); ++index) {
    parameters[index][0] = static_cast<uint8_t>(values[index] & 0xff);
    parameters[index][1] = static_cast<uint8_t>((values[index] >> 8) & 0xff);
    if (!group.addParam(ids[index], parameters[index].data())) {
      group.clearParam();
      error = "Could not add servo " + std::to_string(ids[index]) +
              " to synchronized register write";
      return false;
    }
  }
  const int result = group.txPacket();
  group.clearParam();
  if (result != COMM_SUCCESS) {
    error = packet->getTxRxResult(result);
    return false;
  }
  error.clear();
  return true;
}
}  // namespace

ServoBus::ServoBus(std::string device_name, int baud_rate, double protocol_version,
                   int status_return_level)
: device_name_(std::move(device_name)),
  baud_rate_(baud_rate),
  status_return_level_(status_return_level),
  port_(dynamixel::PortHandler::getPortHandler(device_name_.c_str())),
  packet_(dynamixel::PacketHandler::getPacketHandler(protocol_version))
{
  if (status_return_level_ != 1 && status_return_level_ != 2) {
    throw std::invalid_argument("status_return_level must be 1 or 2");
  }
}

ServoBus::~ServoBus()
{
  disconnect();
}

bool ServoBus::connected() const
{
  return connected_;
}

bool ServoBus::connect(std::string &error)
{
  if (connected_) {
    return true;
  }
  if (!port_->openPort()) {
    error = "Could not open DYNAMIXEL device " + device_name_;
    return false;
  }
  if (!port_->setBaudRate(baud_rate_)) {
    port_->closePort();
    error = "Could not set DYNAMIXEL baud rate to " + std::to_string(baud_rate_);
    return false;
  }
  connected_ = true;
  error.clear();
  return true;
}

bool ServoBus::read(uint8_t id, ServoReading &reading, std::string &error)
{
  std::vector<uint8_t> data(kTelemetryLength);
  uint8_t device_error = 0;
  const int result = packet_->readTxRx(
      port_, id, kTelemetryStartAddress, kTelemetryLength, data.data(), &device_error);
  if (result != COMM_SUCCESS) {
    error = packet_->getTxRxResult(result);
    return false;
  }
  if (device_error != 0) {
    error = packet_->getRxPacketError(device_error);
    return false;
  }

  reading.id = id;
  reading.torque_enabled = data[0] != 0;
  reading.position = word_at(data, 12);
  reading.speed = word_at(data, 14);
  reading.load = word_at(data, 16);
  reading.voltage = data[18];
  reading.temperature = data[19];
  error.clear();
  return true;
}

bool ServoBus::read_limits(uint8_t id, uint16_t &lower, uint16_t &upper, std::string &error)
{
  std::vector<uint8_t> data(4);
  uint8_t device_error = 0;
  const int result = packet_->readTxRx(port_, id, 6, 4, data.data(), &device_error);
  if (result != COMM_SUCCESS || device_error != 0) {
    error = result != COMM_SUCCESS ? packet_->getTxRxResult(result)
                                   : packet_->getRxPacketError(device_error);
    return false;
  }
  const auto low = word_at(data, 0);
  const auto high = word_at(data, 2);
  if (low >= high || high > 1023) {
    error = "Invalid angle limits (wheel mode, locked range, or out of range)";
    return false;
  }
  lower = low;
  upper = high;
  error.clear();
  return true;
}

bool ServoBus::set_positions(const std::vector<uint8_t> &ids,
                             const std::vector<uint16_t> &positions,
                             const std::vector<uint16_t> &speeds,
                             const std::vector<uint16_t> &torque_limits,
                             std::string &error)
{
  if (ids.empty() || positions.size() != ids.size() || speeds.size() != ids.size() ||
      torque_limits.size() != ids.size() ||
      std::any_of(positions.begin(), positions.end(), [](auto value) { return value > 1023; }) ||
      std::any_of(speeds.begin(), speeds.end(), [](auto value) { return value > 1023; }) ||
      std::any_of(torque_limits.begin(), torque_limits.end(), [](auto value) { return value > 1023; })) {
    error = "Position, speed, and torque arrays must match and contain AX values from 0 to 1023";
    return false;
  }
  if (!sync_write(port_, packet_, kTorqueLimitAddress, 2, ids, torque_limits, error) ||
      !sync_write(port_, packet_, kMovingSpeedAddress, 2, ids, speeds, error) ||
      !sync_write(port_, packet_, kGoalPositionAddress, 2, ids, positions, error)) {
    return false;
  }
  return sync_write(port_, packet_, kTorqueEnableAddress, 1, ids,
                    std::vector<uint16_t>(ids.size(), 1), error);
}

bool ServoBus::set_torque(uint8_t id, bool enabled, std::string &error)
{
  return write_register(port_, packet_, status_return_level_, id, kTorqueEnableAddress,
                        enabled ? 1 : 0, 1, error);
}

bool ServoBus::set_torques(const std::vector<uint8_t> &ids, bool enabled, std::string &error)
{
  if (ids.empty()) {
    error = "Torque command requires at least one servo";
    return false;
  }
  return sync_write(port_, packet_, kTorqueEnableAddress, 1, ids,
                    std::vector<uint16_t>(ids.size(), enabled ? 1 : 0), error);
}

void ServoBus::disconnect()
{
  if (connected_) {
    port_->closePort();
    connected_ = false;
  }
}
}  // namespace saprog_dynamixel
