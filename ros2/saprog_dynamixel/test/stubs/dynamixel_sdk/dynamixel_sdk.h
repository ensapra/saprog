#pragma once

#include <algorithm>
#include <cstdint>
#include <utility>
#include <vector>

constexpr int COMM_SUCCESS = 0;
constexpr int COMM_TX_FAIL = -1001;

namespace dynamixel
{
class PortHandler
{
public:
  static PortHandler *getPortHandler(const char *) { static PortHandler port; return &port; }
  bool openPort() { return true; }
  bool setBaudRate(int) { return true; }
  void closePort() {}
};

class PacketHandler
{
public:
  std::vector<std::pair<uint16_t, uint16_t>> writes;
  static PacketHandler *getPacketHandler(double) { static PacketHandler packet; return &packet; }
  int write1ByteTxOnly(PortHandler *, uint8_t, uint16_t address, uint8_t value)
  { writes.emplace_back(address, value); return COMM_SUCCESS; }
  int write2ByteTxOnly(PortHandler *, uint8_t, uint16_t address, uint16_t value)
  { writes.emplace_back(address, value); return COMM_SUCCESS; }
  int write1ByteTxRx(PortHandler *port, uint8_t id, uint16_t address, uint8_t value, uint8_t *error)
  { *error = 0; return write1ByteTxOnly(port, id, address, value); }
  int write2ByteTxRx(PortHandler *port, uint8_t id, uint16_t address, uint16_t value, uint8_t *error)
  { *error = 0; return write2ByteTxOnly(port, id, address, value); }
  int readTxRx(PortHandler *, uint8_t, uint16_t, uint16_t size, uint8_t *data, uint8_t *error)
  { std::fill_n(data, size, 0); *error = 0; return COMM_SUCCESS; }
  const char *getTxRxResult(int) { return "fake communication error"; }
  const char *getRxPacketError(uint8_t) { return "fake device error"; }
};

class GroupSyncWrite
{
public:
  GroupSyncWrite(PortHandler *, PacketHandler *packet, uint16_t address, uint16_t width)
  : packet_(packet), address_(address), width_(width) {}
  bool addParam(uint8_t, uint8_t *data)
  {
    values_.push_back(data[0] | (width_ == 2 ? static_cast<uint16_t>(data[1]) << 8 : 0));
    return true;
  }
  int txPacket()
  {
    for (auto value : values_) { packet_->writes.emplace_back(address_, value); }
    return COMM_SUCCESS;
  }
  void clearParam() { values_.clear(); }
private:
  PacketHandler *packet_;
  uint16_t address_;
  uint16_t width_;
  std::vector<uint16_t> values_;
};
}  // namespace dynamixel
