#pragma once

#include <cstdint>
#include <string>
#include <vector>

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
  ServoBus(std::string device_name, int baud_rate, double protocol_version,
           int status_return_level);
  ~ServoBus();
  ServoBus(const ServoBus &) = delete;
  ServoBus &operator=(const ServoBus &) = delete;

  bool connected() const;
  bool connect(std::string &error);
  bool read(uint8_t id, ServoReading &reading, std::string &error);
  bool set_positions(const std::vector<uint8_t> &ids,
                     const std::vector<uint16_t> &positions,
                     const std::vector<uint16_t> &speeds,
                     const std::vector<uint16_t> &torque_limits,
                     std::string &error);
  bool set_torque(uint8_t id, bool enabled, std::string &error);
  bool set_torques(const std::vector<uint8_t> &ids, bool enabled, std::string &error);
  void disconnect();

private:
  std::string device_name_;
  int baud_rate_;
  int status_return_level_;
  dynamixel::PortHandler *port_;
  dynamixel::PacketHandler *packet_;
  bool connected_{false};
};
}  // namespace saprog_dynamixel
