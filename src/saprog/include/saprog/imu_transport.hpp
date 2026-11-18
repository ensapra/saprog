#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <sys/types.h>

namespace saprog::imu
{
struct SerialDevices
{
  std::vector<std::string> ports;
  std::string preferred;
};

// Owns the exclusively locked IMU serial descriptor and the Linux USB identity scan.
class SerialTransport
{
public:
  SerialTransport() = default;
  ~SerialTransport();
  SerialTransport(const SerialTransport &) = delete;
  SerialTransport &operator=(const SerialTransport &) = delete;

  static SerialDevices discover();
  bool open(const std::string &device, unsigned baud, std::string &error);
  void close();
  bool connected() const { return fd_ >= 0; }
  ssize_t read(char *buffer, std::size_t size) const;
  bool write_space(std::string &error) const;
  bool failed() const;

private:
  int fd_ = -1;
};

enum class HelperState
{
  running,
  succeeded,
  failed,
  timed_out
};

struct HelperResult
{
  HelperState state = HelperState::running;
  std::string device;
  std::string diagnostics;
};

// Incrementally frames CR/LF-delimited input with a strict per-line memory bound.
class LineFramer
{
public:
  explicit LineFramer(std::size_t maximum_line_size = 16384)
  : maximum_line_size_(maximum_line_size)
  {
  }
  std::vector<std::string> append(const char *data, std::size_t size);
  std::uint64_t rejected_lines() const { return rejected_lines_; }
  void clear()
  {
    pending_.clear();
    discarding_ = false;
  }

private:
  std::size_t maximum_line_size_;
  std::string pending_;
  std::uint64_t rejected_lines_ = 0;
  bool discarding_ = false;
};

// Owns a USB setup subprocess group and collects its path separately from diagnostics.
class UsbHelperProcess
{
public:
  UsbHelperProcess() = default;
  ~UsbHelperProcess();
  UsbHelperProcess(const UsbHelperProcess &) = delete;
  UsbHelperProcess &operator=(const UsbHelperProcess &) = delete;

  bool start(const std::string &script, std::string &error);
  HelperResult poll(std::chrono::steady_clock::time_point now);
  void stop();
  bool running() const { return pid_ > 0; }

private:
  void drain(int fd, std::string &output);
  pid_t pid_ = -1;
  int stdout_fd_ = -1;
  int stderr_fd_ = -1;
  std::chrono::steady_clock::time_point started_{};
  std::string device_output_;
  std::string diagnostics_;
};
} // namespace saprog::imu
