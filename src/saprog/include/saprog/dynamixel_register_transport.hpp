#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace dynamixel
{
class PacketHandler;
class PortHandler;
} // namespace dynamixel

// Owns the single SDK port and translates register-level SDK calls into
// checked reads, writes and synchronized batches. Bus arbitration/cache policy
// stays in DynamixelBus.
class DynamixelRegisterTransport
{
public:
  DynamixelRegisterTransport(const std::string &device_name, int baud_rate, double protocol_version,
                             int status_return_level);
  ~DynamixelRegisterTransport();
  DynamixelRegisterTransport(const DynamixelRegisterTransport &) = delete;
  DynamixelRegisterTransport &operator=(const DynamixelRegisterTransport &) = delete;

  bool read(uint8_t id, uint16_t address, uint16_t width, uint8_t *data, std::string &error);
  bool write(uint8_t id, uint16_t address, uint16_t value, uint16_t width, std::string &error);
  bool sync_write(const std::vector<uint8_t> &ids, const std::vector<uint16_t> &values,
                  uint16_t address, uint16_t width, std::string &error);

private:
  bool verify(uint8_t id, uint16_t address, uint16_t value, uint16_t width, std::string &error);
  dynamixel::PortHandler *port_handler_{nullptr};
  dynamixel::PacketHandler *packet_handler_{nullptr};
  int status_return_level_;
};
