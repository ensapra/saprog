#include "saprog/dynamixel_register_transport.hpp"

#include <array>
#include <algorithm>
#include <stdexcept>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk/group_sync_write.h"

DynamixelRegisterTransport::DynamixelRegisterTransport(const std::string &device_name, int baud_rate,
                                                       double protocol_version,
                                                       int status_return_level)
    : status_return_level_(status_return_level)
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

DynamixelRegisterTransport::~DynamixelRegisterTransport()
{
  if (port_handler_ != nullptr) {
    port_handler_->closePort();
  }
}

bool DynamixelRegisterTransport::read(uint8_t id, uint16_t address, uint16_t width, uint8_t *data,
                                      std::string &error)
{
  if ((width == 0 || width > 8) || data == nullptr) {
    error = "Register read has an invalid width or destination";
    return false;
  }
  uint8_t device_error = 0;
  const int result = packet_handler_->readTxRx(port_handler_, id, address, width, data, &device_error);
  if (result != COMM_SUCCESS) {
    error = packet_handler_->getTxRxResult(result);
    return false;
  }
  if (device_error != 0) {
    error = std::string("device error: ") + packet_handler_->getRxPacketError(device_error);
    return false;
  }
  error.clear();
  return true;
}

bool DynamixelRegisterTransport::write(uint8_t id, uint16_t address, uint16_t value, uint16_t width,
                                       std::string &error)
{
  const std::string context = "servo_" + std::to_string(id) + " register " +
                              std::to_string(address) + ": ";
  if (width != 1 && width != 2) {
    error = context + "unsupported register width";
    return false;
  }
  if (width == 1 && value > 0xff) {
    error = context + "value does not fit the register width";
    return false;
  }
  if (status_return_level_ == 1) {
    const int result = width == 1
                           ? packet_handler_->write1ByteTxOnly(port_handler_, id, address,
                                                               static_cast<uint8_t>(value))
                           : packet_handler_->write2ByteTxOnly(port_handler_, id, address, value);
    if (result != COMM_SUCCESS) {
      error = context + packet_handler_->getTxRxResult(result);
      return false;
    }
    error.clear();
    return true;
  }

  uint8_t device_error = 0;
  const int result = width == 1
                         ? packet_handler_->write1ByteTxRx(port_handler_, id, address,
                                                           static_cast<uint8_t>(value), &device_error)
                         : packet_handler_->write2ByteTxRx(port_handler_, id, address, value,
                                                           &device_error);
  if (result == COMM_RX_TIMEOUT) {
    return verify(id, address, value, width, error);
  }
  if (result != COMM_SUCCESS) {
    error = context + packet_handler_->getTxRxResult(result);
    return false;
  }
  if (device_error != 0) {
    error = context + packet_handler_->getRxPacketError(device_error);
    return false;
  }
  error.clear();
  return true;
}

bool DynamixelRegisterTransport::sync_write(const std::vector<uint8_t> &ids,
                                            const std::vector<uint16_t> &values,
                                            uint16_t address, uint16_t width, std::string &error)
{
  if (ids.empty() || ids.size() != values.size() || (width != 1 && width != 2)) {
    error = "Synchronized register write has mismatched servo data";
    return false;
  }
  if (width == 1 && std::any_of(values.begin(), values.end(), [](uint16_t value) {
        return value > 0xff;
      })) {
    error = "Synchronized register value does not fit the register width";
    return false;
  }
  dynamixel::GroupSyncWrite group(port_handler_, packet_handler_, address, width);
  std::vector<std::array<uint8_t, 2>> parameters(ids.size());
  for (size_t i = 0; i < ids.size(); ++i) {
    parameters[i][0] = static_cast<uint8_t>(values[i] & 0xff);
    parameters[i][1] = static_cast<uint8_t>((values[i] >> 8) & 0xff);
    if (!group.addParam(ids[i], parameters[i].data())) {
      group.clearParam();
      error = "Could not add servo " + std::to_string(ids[i]) + " to synchronized register write";
      return false;
    }
  }
  const int result = group.txPacket();
  group.clearParam();
  if (result != COMM_SUCCESS) {
    error = packet_handler_->getTxRxResult(result);
    return false;
  }
  error.clear();
  return true;
}

bool DynamixelRegisterTransport::verify(uint8_t id, uint16_t address, uint16_t value, uint16_t width,
                                        std::string &error)
{
  std::array<uint8_t, 2> data{};
  if (!read(id, address, width, data.data(), error)) {
    const std::string context = "servo_" + std::to_string(id) + " register " +
                                std::to_string(address) + ": ";
    const auto device_error = error.find("device error: ");
    error = context + (device_error == std::string::npos ? "readback failed: "
                                                         : "readback device error: ") +
            (device_error == std::string::npos ? error : error.substr(device_error + 14));
    return false;
  }
  const uint16_t observed = static_cast<uint16_t>(data[0] | (data[1] << 8));
  if (observed != value) {
    error = "servo_" + std::to_string(id) + " register " + std::to_string(address) +
            ": readback mismatch (expected " + std::to_string(value) + ", got " +
            std::to_string(observed) + ").";
    return false;
  }
  error.clear();
  return true;
}
