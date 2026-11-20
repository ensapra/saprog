#pragma once

#include <cstdint>
#include <string>

#include "dynamixel_sdk/dynamixel_sdk.h"

namespace saprog_dynamixel
{
struct ServoReading
{
  uint8_t id{};
  uint16_t position{};
  uint16_t speed{};
  uint16_t load{};
  uint8_t voltage{};
  uint8_t temperature{};
  bool torque_enabled{false};
};

class ServoBus
{
public:
  ServoBus(std::string device_name, int baud_rate, double protocol_version);
  ~ServoBus();
  ServoBus(const ServoBus &) = delete;
  ServoBus &operator=(const ServoBus &) = delete;

  bool connected() const;
  bool connect(std::string &error);
  bool read(uint8_t id, ServoReading &reading, std::string &error);
  void disconnect();

private:
  std::string device_name_;
  int baud_rate_;
  dynamixel::PortHandler *port_;
  dynamixel::PacketHandler *packet_;
  bool connected_{false};
};
}  // namespace saprog_dynamixel
