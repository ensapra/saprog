#pragma once

#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "saprog/dynamixel_register_transport.hpp"

// Owns the shared serial connection. Public methods lock the bus; *_locked
// helpers require that lock. Cached settings describe verified register values.
class DynamixelBus
{
public:
  DynamixelBus(const std::string &device_name, int baud_rate, double protocol_version,
               int status_return_level = 1, bool verify_goal_registers = false);
  ~DynamixelBus();

  bool read(uint8_t id, std::array<uint8_t, 8> &data, std::string &error);
  bool read_state(uint8_t id, std::array<uint8_t, 8> &data, bool &enabled, std::string &error);
  bool get_torque(uint8_t id, bool &enabled, std::string &error);
  bool get_limits(uint8_t id, uint16_t &lower, uint16_t &upper, std::string &error);
  bool set_position(uint8_t id, uint16_t position, int speed, int torque_limit, std::string &error);
  bool set_positions(const std::vector<uint8_t> &ids, const std::vector<uint16_t> &positions,
                     int speed, const std::vector<int> &torque_limits, double tolerance_rad,
                     std::string &error);
  bool set_positions(const std::vector<uint8_t> &ids, const std::vector<uint16_t> &positions,
                     const std::vector<int> &speeds, const std::vector<int> &torque_limits,
                     double tolerance_rad, std::string &error);
  bool set_positions_before(const std::vector<uint8_t> &ids, const std::vector<uint16_t> &positions,
                            const std::vector<int> &speeds, const std::vector<int> &torque_limits,
                            double tolerance_rad,
                            std::chrono::steady_clock::time_point latest_start, std::string &error,
                            const std::function<bool()> &cancellation_requested = {});
  bool set_torque(uint8_t id, bool enabled, std::string &error);
  bool set_torques(const std::vector<uint8_t> &ids, bool enabled, std::string &error);
  bool set_torques_before(const std::vector<uint8_t> &ids, bool enabled,
                          std::chrono::steady_clock::time_point latest_start, std::string &error,
                          const std::function<bool()> &cancellation_requested = {});

private:
  class TimelinePriorityGuard
  {
  public:
    explicit TimelinePriorityGuard(DynamixelBus &bus);
    ~TimelinePriorityGuard();

    TimelinePriorityGuard(const TimelinePriorityGuard &) = delete;
    TimelinePriorityGuard &operator=(const TimelinePriorityGuard &) = delete;

  private:
    DynamixelBus &bus_;
  };

  struct ServoSettings
  {
    // -1 means the register value is unknown and must be written.
    int torque_limit{-1};
    int speed{-1};
    int torque_enabled{-1};
  };

  void observe_torque(uint8_t id, bool enabled);
  bool set_setting(uint8_t id, uint16_t address, uint16_t value, uint16_t size, std::string &error);
  bool set_settings(const std::vector<uint8_t> &ids, const std::vector<uint16_t> &values,
                    uint16_t address, uint16_t size, std::string &error);
  int &cached_setting(uint8_t id, uint16_t address);
  void invalidate_setting(uint8_t id, uint16_t address);
  bool write_register(uint8_t id, uint16_t address, uint16_t value, uint16_t size,
                      std::string &error);
  bool set_positions_locked(const std::vector<uint8_t> &ids, const std::vector<uint16_t> &positions,
                            const std::vector<int> &speeds, const std::vector<int> &torque_limits,
                            double tolerance_rad, std::string &error);
  std::unique_lock<std::timed_mutex> lock_bus();
  bool verify_goal_registers_;
  DynamixelRegisterTransport transport_;
  std::timed_mutex bus_mutex_;
  std::mutex timeline_priority_mutex_;
  std::condition_variable timeline_priority_cv_;
  bool timeline_priority_requested_{false};
  std::map<uint8_t, ServoSettings> settings_;
};
