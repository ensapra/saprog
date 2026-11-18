#include "saprog/imu_transport.hpp"

#include <cerrno>
#include <algorithm>
#include <csignal>
#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <poll.h>
#include <stdexcept>
#include <sys/file.h>
#include <sys/wait.h>
#include <termios.h>
#include <unistd.h>
#include <utility>

namespace saprog::imu
{
namespace
{
speed_t baud_speed(unsigned baud)
{
  switch (baud) {
  case 9600:
    return B9600;
  case 19200:
    return B19200;
  case 38400:
    return B38400;
  case 57600:
    return B57600;
  case 115200:
    return B115200;
  case 230400:
    return B230400;
  default:
    return 0;
  }
}

std::string read_word(const std::filesystem::path &path)
{
  std::ifstream file(path);
  std::string word;
  file >> word;
  return word;
}

void close_fd(int &fd)
{
  if (fd >= 0) {
    ::close(fd);
    fd = -1;
  }
}

void make_pipe(int (&fds)[2])
{
  if (pipe(fds) < 0) {
    throw std::runtime_error("Could not create USB helper pipe: " + std::string(std::strerror(errno)));
  }
  for (int fd : fds) {
    fcntl(fd, F_SETFD, FD_CLOEXEC);
  }
}
} // namespace

SerialTransport::~SerialTransport()
{
  close();
}

SerialDevices SerialTransport::discover()
{
  SerialDevices result;
  std::vector<std::string> matches;
  std::error_code error;
  std::filesystem::directory_iterator entry("/sys/class/tty", error), end;
  for (; !error && entry != end; entry.increment(error)) {
    const auto name = entry->path().filename().string();
    if (name.rfind("ttyACM", 0) != 0 && name.rfind("ttyUSB", 0) != 0) {
      continue;
    }
    const std::string device = "/dev/" + name;
    result.ports.push_back(device);
    error.clear();
    auto parent = std::filesystem::canonical(entry->path() / "device", error);
    if (error) {
      error.clear();
      continue;
    }
    while (parent != parent.root_path() && !parent.empty()) {
      if (read_word(parent / "idVendor") == "1b4f" &&
          read_word(parent / "idProduct") == "9d0f") {
        matches.push_back(device);
        break;
      }
      parent = parent.parent_path();
    }
  }
  std::sort(result.ports.begin(), result.ports.end());
  if (matches.size() == 1) {
    result.preferred = matches.front();
  }
  return result;
}

bool SerialTransport::open(const std::string &device, unsigned baud, std::string &error)
{
  close();
  const speed_t speed = baud_speed(baud);
  if (!speed) {
    error = "Unsupported baud rate.";
    return false;
  }
  const int fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd < 0) {
    error = "Could not open " + device + ": " + std::strerror(errno);
    return false;
  }
  if (flock(fd, LOCK_EX | LOCK_NB) < 0) {
    ::close(fd);
    error = "Serial port is already owned by another process.";
    return false;
  }
  termios settings{};
  if (tcgetattr(fd, &settings) < 0) {
    ::close(fd);
    error = "Could not read serial settings.";
    return false;
  }
  cfmakeraw(&settings);
  settings.c_cflag |= CLOCAL | CREAD;
  settings.c_cflag &= ~CRTSCTS;
  settings.c_cc[VMIN] = 0;
  settings.c_cc[VTIME] = 0;
  cfsetispeed(&settings, speed);
  cfsetospeed(&settings, speed);
  if (tcsetattr(fd, TCSANOW, &settings) < 0) {
    ::close(fd);
    error = "Could not configure serial port.";
    return false;
  }
  fd_ = fd;
  return true;
}

void SerialTransport::close()
{
  close_fd(fd_);
}

ssize_t SerialTransport::read(char *buffer, std::size_t size) const
{
  return fd_ < 0 ? -1 : ::read(fd_, buffer, size);
}

bool SerialTransport::write_space(std::string &error) const
{
  if (fd_ < 0) {
    error = "Not connected.";
    return false;
  }
  if (::write(fd_, " ", 1) != 1) {
    error = "Could not send Space toggle: " + std::string(std::strerror(errno));
    return false;
  }
  return true;
}

bool SerialTransport::failed() const
{
  if (fd_ < 0) {
    return true;
  }
  pollfd descriptor{fd_, POLLIN, 0};
  return ::poll(&descriptor, 1, 0) > 0 && (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL));
}

UsbHelperProcess::~UsbHelperProcess()
{
  stop();
}

bool UsbHelperProcess::start(const std::string &script, std::string &error)
{
  if (running()) {
    error = "USB helper is already running.";
    return false;
  }
  int output_pipe[2], error_pipe[2];
  try {
    make_pipe(output_pipe);
    try {
      make_pipe(error_pipe);
    } catch (...) {
      ::close(output_pipe[0]);
      ::close(output_pipe[1]);
      throw;
    }
  } catch (const std::exception &exception) {
    error = exception.what();
    return false;
  }
  pid_ = fork();
  if (pid_ == 0) {
    setpgid(0, 0);
    ::close(output_pipe[0]);
    ::close(error_pipe[0]);
    dup2(output_pipe[1], STDOUT_FILENO);
    dup2(error_pipe[1], STDERR_FILENO);
    ::close(output_pipe[1]);
    ::close(error_pipe[1]);
    execl("/bin/bash", "bash", script.c_str(), static_cast<char *>(nullptr));
    _exit(127);
  }
  ::close(output_pipe[1]);
  ::close(error_pipe[1]);
  if (pid_ < 0) {
    ::close(output_pipe[0]);
    ::close(error_pipe[0]);
    error = "Could not start USB helper: " + std::string(std::strerror(errno));
    return false;
  }
  setpgid(pid_, pid_);
  stdout_fd_ = output_pipe[0];
  stderr_fd_ = error_pipe[0];
  fcntl(stdout_fd_, F_SETFL, O_NONBLOCK);
  fcntl(stderr_fd_, F_SETFL, O_NONBLOCK);
  started_ = std::chrono::steady_clock::now();
  device_output_.clear();
  diagnostics_.clear();
  return true;
}

void UsbHelperProcess::drain(int fd, std::string &output)
{
  if (fd < 0) {
    return;
  }
  char buffer[1024];
  ssize_t count;
  while ((count = ::read(fd, buffer, sizeof(buffer))) > 0) {
    if (output.size() < 8192) {
      output.append(buffer, std::min<std::size_t>(count, 8192 - output.size()));
    }
  }
}

HelperResult UsbHelperProcess::poll(std::chrono::steady_clock::time_point now)
{
  HelperResult result;
  if (!running()) {
    result.state = HelperState::failed;
    return result;
  }
  drain(stdout_fd_, device_output_);
  drain(stderr_fd_, diagnostics_);
  int status = 0;
  const pid_t exited = waitpid(pid_, &status, WNOHANG);
  if (exited == pid_) {
    pid_ = -1;
    close_fd(stdout_fd_);
    close_fd(stderr_fd_);
    if (WIFEXITED(status) && WEXITSTATUS(status) == 0) {
      const auto first = device_output_.find_first_not_of(" \r\n\t");
      const auto last = device_output_.find_last_not_of(" \r\n\t");
      if (first != std::string::npos) {
        result.device = device_output_.substr(first, last - first + 1);
      }
      result.state = result.device.empty() ? HelperState::failed : HelperState::succeeded;
    } else {
      result.state = HelperState::failed;
    }
    result.diagnostics = diagnostics_;
  } else if (exited < 0 || now - started_ > std::chrono::seconds(35)) {
    stop();
    result.state = exited < 0 ? HelperState::failed : HelperState::timed_out;
    result.diagnostics = diagnostics_;
  }
  return result;
}

void UsbHelperProcess::stop()
{
  if (pid_ > 0) {
    kill(-pid_, SIGKILL);
    waitpid(pid_, nullptr, 0);
    pid_ = -1;
  }
  close_fd(stdout_fd_);
  close_fd(stderr_fd_);
}

std::vector<std::string> LineFramer::append(const char *data, std::size_t size)
{
  std::vector<std::string> lines;
  for (std::size_t i = 0; i < size; ++i) {
    const char value = data[i];
    if (value == '\r' || value == '\n') {
      if (!discarding_ && !pending_.empty()) {
        lines.push_back(std::move(pending_));
        pending_.clear();
      }
      if (discarding_) {
        discarding_ = false;
      }
      continue;
    }
    if (discarding_) {
      continue;
    }
    if (pending_.size() == maximum_line_size_) {
      pending_.clear();
      discarding_ = true;
      ++rejected_lines_;
      continue;
    }
    pending_ += value;
  }
  return lines;
}
} // namespace saprog::imu
