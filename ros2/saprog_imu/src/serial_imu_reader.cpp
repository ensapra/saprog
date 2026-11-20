#include "saprog_imu/serial_imu_reader.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <stdexcept>
#include <termios.h>
#include <unistd.h>
#include <utility>

namespace saprog_imu
{
namespace
{
speed_t baud_constant(int baud_rate)
{
  switch (baud_rate) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    default: throw std::runtime_error("Unsupported baud_rate; use 9600, 19200, 38400, 57600, or 115200");
  }
}
}  // namespace

SerialImuReader::SerialImuReader(std::string device, int baud_rate)
: device_(std::move(device)), baud_rate_(baud_rate)
{
  (void)baud_constant(baud_rate_);
}

SerialImuReader::~SerialImuReader()
{
  disconnect();
}

bool SerialImuReader::connected() const
{
  return file_descriptor_ >= 0;
}

bool SerialImuReader::connect(std::string &error)
{
  const int fd = ::open(device_.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd < 0) {
    error = std::strerror(errno);
    return false;
  }

  termios options{};
  if (tcgetattr(fd, &options) != 0) {
    error = std::strerror(errno);
    ::close(fd);
    return false;
  }
  cfmakeraw(&options);
  const speed_t baud = baud_constant(baud_rate_);
  cfsetispeed(&options, baud);
  cfsetospeed(&options, baud);
  options.c_cflag |= (CLOCAL | CREAD);
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  if (tcsetattr(fd, TCSANOW, &options) != 0) {
    error = std::strerror(errno);
    ::close(fd);
    return false;
  }

  file_descriptor_ = fd;
  pending_.clear();
  return true;
}

bool SerialImuReader::read_lines(std::vector<std::string> &lines, std::string &error)
{
  char buffer[512];
  const ssize_t bytes_read = ::read(file_descriptor_, buffer, sizeof(buffer));
  if (bytes_read < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
      return true;
    }
    error = std::strerror(errno);
    disconnect();
    return false;
  }
  if (bytes_read == 0) {
    return true;
  }

  pending_.append(buffer, static_cast<std::size_t>(bytes_read));
  if (pending_.size() > 4096) {
    pending_.clear();
    error = "serial input line exceeded 4096 bytes";
    return true;
  }

  std::size_t newline = 0;
  while ((newline = pending_.find('\n')) != std::string::npos) {
    std::string line = pending_.substr(0, newline);
    pending_.erase(0, newline + 1);
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    lines.push_back(std::move(line));
  }
  return true;
}

void SerialImuReader::disconnect()
{
  if (file_descriptor_ >= 0) {
    ::close(file_descriptor_);
    file_descriptor_ = -1;
  }
  pending_.clear();
}
}  // namespace saprog_imu
