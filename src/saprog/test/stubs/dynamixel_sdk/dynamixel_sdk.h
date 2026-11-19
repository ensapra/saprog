#pragma once
#include <chrono>
#include <cstdint>
#include <map>
#include <thread>
#include <utility>
#include <vector>

constexpr int COMM_SUCCESS = 0;
constexpr int COMM_RX_TIMEOUT = -3001;
constexpr int COMM_TX_FAIL = -1001;

namespace dynamixel {
class PortHandler {
public:
  static PortHandler * getPortHandler(const char *) { static PortHandler port; return &port; }
  bool openPort() { return true; }
  bool setBaudRate(int) { return true; }
  void closePort() {}
};

class PacketHandler {
public:
  int write_result = COMM_SUCCESS;
  int tx_only_result = COMM_SUCCESS;
  int group_result = COMM_SUCCESS;
  int read_result = COMM_SUCCESS;
  uint8_t write_error = 0;
  uint8_t read_error = 0;
  bool apply_write = true;
  bool apply_group_write = true;
  int reads = 0;
  std::chrono::milliseconds read_delay{0};
  std::map<std::pair<uint8_t, uint16_t>, uint16_t> registers;
  std::vector<uint16_t> writes;
  std::vector<uint16_t> tx_only_writes;
  std::vector<uint16_t> group_writes;
  static PacketHandler * getPacketHandler(double) { static PacketHandler packet; return &packet; }
  int write1ByteTxRx(PortHandler *, uint8_t id, uint16_t address, uint8_t value, uint8_t * error) {
    return write(id, address, value, error);
  }
  int write2ByteTxRx(PortHandler *, uint8_t id, uint16_t address, uint16_t value, uint8_t * error) {
    return write(id, address, value, error);
  }
  int write1ByteTxOnly(PortHandler *, uint8_t id, uint16_t address, uint8_t value) {
    return write_only(id, address, value);
  }
  int write2ByteTxOnly(PortHandler *, uint8_t id, uint16_t address, uint16_t value) {
    return write_only(id, address, value);
  }
  int write(uint8_t id, uint16_t address, uint16_t value, uint8_t * error) {
    writes.push_back(address);
    if (apply_write) registers[{id, address}] = value;
    *error = write_error;
    return write_result;
  }
  int write_only(uint8_t id, uint16_t address, uint16_t value) {
    tx_only_writes.push_back(address);
    if (apply_write) registers[{id, address}] = value;
    return tx_only_result;
  }
  int readTxRx(PortHandler *, uint8_t id, uint16_t address, uint16_t size, uint8_t * data, uint8_t * error) {
    ++reads;
    std::this_thread::sleep_for(read_delay);
    *error = read_error;
    data[0] = registers[{id, address}] & 0xff;
    if (size > 1) data[1] = registers[{id, address}] >> 8;
    for (uint16_t offset = 2; offset < size; ++offset) {
      data[offset] = registers[{id, static_cast<uint16_t>(address + offset)}] & 0xff;
    }
    // Word registers are represented as words in this fake.
    for (uint16_t offset = 2; offset + 1 < size; offset += 2) {
      data[offset + 1] = registers[{id, static_cast<uint16_t>(address + offset)}] >> 8;
    }
    return read_result;
  }
  int read1ByteTxRx(PortHandler * port, uint8_t id, uint16_t address, uint8_t * value, uint8_t * error) {
    return readTxRx(port, id, address, 1, value, error);
  }
  const char * getTxRxResult(int) { return "communication error"; }
  const char * getRxPacketError(uint8_t) { return "device error"; }
};

class GroupSyncWrite {
public:
  GroupSyncWrite(PortHandler *, PacketHandler * packet, uint16_t address, uint16_t size)
  : packet_(packet), address_(address), size_(size) {}
  bool addParam(uint8_t id, uint8_t * data) {
    std::vector<uint8_t> value(data, data + size_);
    return parameters_.emplace(id, std::move(value)).second;
  }
  int txPacket() {
    packet_->group_writes.push_back(address_);
    if (packet_->group_result == COMM_SUCCESS && packet_->apply_group_write) {
      for (const auto & parameter : parameters_) {
        uint16_t value = parameter.second[0];
        if (size_ > 1) value |= static_cast<uint16_t>(parameter.second[1] << 8);
        packet_->registers[{parameter.first, address_}] = value;
      }
    }
    return packet_->group_result;
  }
  void clearParam() { parameters_.clear(); }
private:
  PacketHandler * packet_;
  uint16_t address_;
  uint16_t size_;
  std::map<uint8_t, std::vector<uint8_t>> parameters_;
};
}
