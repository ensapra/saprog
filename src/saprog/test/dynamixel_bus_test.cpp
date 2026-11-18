#include "dynamixel_sdk/dynamixel_sdk.h"
#include "saprog/dynamixel_bus.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>

void check(bool condition)
{
  if (!condition)
    throw std::runtime_error("Bus regression check failed");
}

int main()
{
  DynamixelBus bus("fake", 1000000, 1.0, 1, false);
  auto &sdk = *dynamixel::PacketHandler::getPacketHandler(1.0);
  std::string error = "old error";
  uint16_t lower = 0, upper = 0;
  sdk.registers[{13, 6}] = 350;
  sdk.registers[{13, 8}] = 440;
  check(bus.get_limits(13, lower, upper, error));
  check(lower == 350 && upper == 440 && error.empty() && sdk.writes.empty());
  sdk.registers[{13, 6}] = 0;
  sdk.registers[{13, 8}] = 0;
  check(!bus.get_limits(13, lower, upper, error));
  sdk.read_result = COMM_RX_TIMEOUT;
  check(!bus.get_limits(13, lower, upper, error));
  sdk = {};
  check(bus.set_torque(12, false, error));
  check(error.empty() && sdk.reads == 1 && sdk.writes.empty());
  check(sdk.tx_only_writes == std::vector<uint16_t>({24}));

  sdk = {};
  DynamixelBus position_bus("fake", 1000000, 1.0, 1, false);
  check(position_bus.set_position(12, 700, 50, 512, error));
  check(error.empty() && sdk.reads == 4);
  check(sdk.tx_only_writes == std::vector<uint16_t>({34, 32, 30, 24}));
  check(sdk.registers[{12, 30}] == 700 && sdk.registers[{12, 24}] == 1);
  check(position_bus.set_torque(12, false, error));
  check(sdk.registers[{12, 24}] == 0);
  const auto writes_before_cached_torque = sdk.tx_only_writes.size();
  check(position_bus.set_torque(12, false, error));
  check(sdk.tx_only_writes.size() == writes_before_cached_torque);

  sdk = {};
  DynamixelBus mismatch_bus("fake", 1000000, 1.0, 1, false);
  sdk.apply_write = false;
  check(!mismatch_bus.set_position(12, 700, 50, 512, error));
  check(error.find("readback mismatch") != std::string::npos);
  check(sdk.tx_only_writes == std::vector<uint16_t>({34})); // Never command a goal after failure.

  sdk = {};
  DynamixelBus timeout_bus("fake", 1000000, 1.0, 1, false);
  sdk.read_result = COMM_RX_TIMEOUT;
  check(!timeout_bus.set_torque(12, false, error));
  check(error.find("readback failed") != std::string::npos);
  check(sdk.tx_only_writes.size() == 1 && sdk.reads == 1); // No command retries.

  sdk = {};
  DynamixelBus device_error_bus("fake", 1000000, 1.0, 1, false);
  sdk.read_error = 4;
  check(!device_error_bus.set_torque(12, true, error));
  check(error.find("readback device error") != std::string::npos);

  sdk = {};
  DynamixelBus transmit_error_bus("fake", 1000000, 1.0, 1, false);
  sdk.tx_only_result = COMM_TX_FAIL;
  check(!transmit_error_bus.set_torque(12, true, error));
  check(sdk.reads == 0);

  sdk = {};
  DynamixelBus acknowledged_bus("fake", 1000000, 1.0, 2, false);
  check(acknowledged_bus.set_torque(12, false, error));
  check(sdk.writes == std::vector<uint16_t>({24}) && sdk.reads == 0);
  sdk = {};
  sdk.write_result = COMM_RX_TIMEOUT;
  check(acknowledged_bus.set_torque(12, true, error));
  check(sdk.writes == std::vector<uint16_t>({24}) && sdk.reads == 1);
  sdk = {};
  DynamixelBus device_ack_bus("fake", 1000000, 1.0, 2, false);
  sdk.write_error = 4;
  check(!device_ack_bus.set_torque(12, true, error));
  check(sdk.reads == 0);

  sdk = {};
  DynamixelBus streaming_bus("fake", 1000000, 1.0, 1, false);
  const std::vector<uint8_t> ids{11, 12};
  const std::vector<int> torque_limits{500, 600};
  check(streaming_bus.set_positions(ids, {100, 200}, 50, torque_limits, 0.1, error));
  check(sdk.group_writes == std::vector<uint16_t>({34, 32, 30, 24}));
  check(sdk.reads == 6); // Verify changed settings; skip per-command goal reads.
  check(streaming_bus.set_positions(ids, {110, 210}, 50, torque_limits, 0.1, error));
  check(sdk.group_writes == std::vector<uint16_t>({34, 32, 30, 24, 30}));
  check(sdk.reads == 6); // Reuse cached settings and keep the high-rate path lean.
  check(streaming_bus.set_positions(ids, {120, 220}, 60, {510, 610}, 0.1, error));
  check(sdk.group_writes == std::vector<uint16_t>({34, 32, 30, 24, 30, 34, 32, 30}));
  check(sdk.reads == 10); // Reverify only settings that changed.

  sdk = {};
  DynamixelBus failed_group_bus("fake", 1000000, 1.0, 1, false);
  sdk.group_result = COMM_TX_FAIL;
  check(!failed_group_bus.set_positions(ids, {100, 200}, 50, torque_limits, 0.1, error));
  check(sdk.group_writes == std::vector<uint16_t>({34}) && sdk.reads == 0);

  sdk = {};
  DynamixelBus verified_bus("fake", 1000000, 1.0, 1, true);
  check(verified_bus.set_positions({11, 12}, {100, 200}, 50, {500, 600}, 0.1, error));
  check(sdk.reads == 8); // Six changed-setting reads plus two goal-register reads.

  sdk = {};
  DynamixelBus per_joint_bus("fake", 1000000, 1.0, 1, false);
  check(per_joint_bus.set_positions(ids, {100, 200}, {50, 75}, torque_limits, 0.1, error));
  check(sdk.group_writes == std::vector<uint16_t>({34, 32, 30, 24}));
  check(sdk.registers[{11, 32}] == 50 && sdk.registers[{12, 32}] == 75);
  check(sdk.registers[{11, 30}] == 100 && sdk.registers[{12, 30}] == 200);
  check(per_joint_bus.set_torques(ids, false, error));
  check(sdk.registers[{11, 24}] == 0 && sdk.registers[{12, 24}] == 0);
  check(sdk.group_writes.back() == 24);

  sdk = {};
  DynamixelBus deadline_bus("fake", 1000000, 1.0, 1, false);
  check(!deadline_bus.set_positions_before(
      ids, {100, 200}, {50, 75}, torque_limits, 0.1,
      std::chrono::steady_clock::now() - std::chrono::milliseconds(1), error));
  check(error.find("deadline") != std::string::npos && sdk.group_writes.empty());
  std::array<uint8_t, 8> post_deadline_read{};
  check(deadline_bus.read(11, post_deadline_read, error));

  sdk = {};
  DynamixelBus busy_bus("fake", 1000000, 1.0, 1, false);
  check(busy_bus.set_positions(ids, {90, 190}, {50, 75}, torque_limits, 0.1, error));
  sdk.read_delay = std::chrono::milliseconds(100);
  std::atomic_bool read_started{false};
  std::atomic_bool read_succeeded{false};
  std::thread background_read([&]() {
    std::array<uint8_t, 8> data{};
    std::string read_error;
    read_started = true;
    read_succeeded = busy_bus.read(11, data, read_error);
  });
  while (!read_started) {
    std::this_thread::yield();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  check(busy_bus.set_positions_before(
      ids, {100, 200}, {50, 75}, torque_limits, 0.1,
      std::chrono::steady_clock::now() + std::chrono::milliseconds(250), error));
  background_read.join();
  check(read_succeeded);

  sdk = {};
  DynamixelBus late_preparation_bus("fake", 1000000, 1.0, 1, false);
  sdk.read_delay = std::chrono::milliseconds(130);
  check(late_preparation_bus.set_positions_before(
      {11}, {100}, {50}, {500}, 0.1,
      std::chrono::steady_clock::now() + std::chrono::milliseconds(250), error));
  check(std::find(sdk.group_writes.begin(), sdk.group_writes.end(), 30) != sdk.group_writes.end());

  sdk = {};
  DynamixelBus canceled_wait_bus("fake", 1000000, 1.0, 1, false);
  sdk.read_delay = std::chrono::milliseconds(100);
  std::atomic_bool cancel_read_started{false};
  std::thread cancel_read([&]() {
    std::array<uint8_t, 8> data{};
    std::string read_error;
    cancel_read_started = true;
    (void)canceled_wait_bus.read(11, data, read_error);
  });
  while (!cancel_read_started) {
    std::this_thread::yield();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  const auto writes_before_canceled_command = sdk.group_writes.size();
  check(!canceled_wait_bus.set_positions_before(
      ids, {101, 201}, {50, 75}, torque_limits, 0.1,
      std::chrono::steady_clock::now() + std::chrono::seconds(1), error, []() { return true; }));
  check(error.find("canceled") != std::string::npos);
  check(sdk.group_writes.size() == writes_before_canceled_command);
  cancel_read.join();

  sdk = {};
  DynamixelBus deadline_torque_bus("fake", 1000000, 1.0, 1, false);
  check(!deadline_torque_bus.set_torques_before(
      ids, false, std::chrono::steady_clock::now() - std::chrono::milliseconds(1), error));
  check(error.find("deadline") != std::string::npos && sdk.group_writes.empty());

  sdk = {};
  DynamixelBus scheduled_torque_bus("fake", 1000000, 1.0, 1, false);
  check(scheduled_torque_bus.set_torques_before(
      ids, false, std::chrono::steady_clock::now() + std::chrono::seconds(1), error));
  check(sdk.group_writes == std::vector<uint16_t>({24}));
  check(scheduled_torque_bus.read(11, post_deadline_read, error));
  std::cout << "All bus regression checks passed\n";
}
