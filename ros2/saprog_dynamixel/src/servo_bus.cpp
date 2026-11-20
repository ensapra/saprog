#include "saprog_dynamixel/servo_bus.hpp"

#include <cstddef>
#include <vector>
#include <utility>

namespace saprog_dynamixel
{
namespace
{
constexpr uint16_t kTelemetryStartAddress = 24;
constexpr uint16_t kTelemetryLength = 20;

uint16_t word_at(const std::vector<uint8_t> &data, std::size_t offset)
{
  return static_cast<uint16_t>(data[offset] | (static_cast<uint16_t>(data[offset + 1]) << 8));
}
}  // namespace

ServoBus::ServoBus(std::string device_name, int baud_rate, double protocol_version)
: device_name_(std::move(device_name)),
  baud_rate_(baud_rate),
  port_(dynamixel::PortHandler::getPortHandler(device_name_.c_str())),
  packet_(dynamixel::PacketHandler::getPacketHandler(protocol_version))
{}

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

void ServoBus::disconnect()
{
  if (connected_) {
    port_->closePort();
    connected_ = false;
  }
}
}  // namespace saprog_dynamixel
