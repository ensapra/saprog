#include "saprog/imu_transport.hpp"

#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>

using namespace std::chrono_literals;
using saprog::imu::HelperState;
using saprog::imu::LineFramer;
using saprog::imu::SerialTransport;
using saprog::imu::UsbHelperProcess;

static void check(bool condition, const char *message)
{
  if (!condition) {
    std::cerr << message << '\n';
    std::exit(1);
  }
}

static std::string temporary_file()
{
  char path[] = "/tmp/saprog_imu_transport_XXXXXX";
  const int fd = mkstemp(path);
  check(fd >= 0, "could not create temporary helper script");
  close(fd);
  return path;
}

static void write_script(const std::string &path, const std::string &body)
{
  const int fd = open(path.c_str(), O_WRONLY | O_TRUNC);
  check(fd >= 0, "could not open temporary helper script");
  const auto written = write(fd, body.data(), body.size());
  close(fd);
  check(written == static_cast<ssize_t>(body.size()), "could not write helper script");
  check(chmod(path.c_str(), 0700) == 0, "could not make helper script executable");
}

int main()
{
  LineFramer framer(4);
  const std::string first = "ab";
  const std::string second = "cd\n12345ignored\nok\r\n";
  auto lines = framer.append(first.data(), first.size());
  check(lines.empty(), "partial frame emitted early");
  lines = framer.append(second.data(), second.size());
  check(lines.size() == 2 && lines[0] == "abcd" && lines[1] == "ok",
        "incremental CR/LF framing changed");
  check(framer.rejected_lines() == 1, "oversized line was not discarded as one frame");

  const int master = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
  check(master >= 0 && grantpt(master) == 0 && unlockpt(master) == 0,
        "could not create PTY for serial lifecycle test");
  const char *slave = ptsname(master);
  check(slave != nullptr, "could not resolve PTY slave path");
  SerialTransport serial;
  std::string error;
  check(serial.open(slave, 115200, error), "serial transport could not open PTY");
  SerialTransport second_owner;
  check(!second_owner.open(slave, 115200, error), "serial lock allowed a second owner");
  check(serial.write_space(error), "serial transport could not write resume toggle");
  char buffer[32]{};
  const auto count = read(master, buffer, sizeof(buffer));
  check(count == 1 && buffer[0] == ' ', "resume toggle was not sent over serial");
  check(write(master, "sample\n", 7) == 7, "PTY sample write failed");
  check(serial.read(buffer, sizeof(buffer)) == 7 && std::string(buffer, 7) == "sample\n",
        "serial transport did not read from PTY");
  serial.close();
  close(master);

  const auto script = temporary_file();
  write_script(script,
               "#!/bin/sh\nprintf '/dev/ttyACM7\\n'\nprintf 'diagnostic\\n' >&2\n"
               "head -c 20000 /dev/zero >&2\n");
  UsbHelperProcess helper;
  check(helper.start(script, error), "USB helper process did not start");
  auto result = helper.poll(std::chrono::steady_clock::now() + 10ms);
  for (int attempt = 0; attempt < 100 && result.state == HelperState::running; ++attempt) {
    std::this_thread::sleep_for(5ms);
    result = helper.poll(std::chrono::steady_clock::now());
  }
  check(result.state == HelperState::succeeded && result.device == "/dev/ttyACM7" &&
            result.diagnostics.find("diagnostic") != std::string::npos &&
            result.diagnostics.size() <= 8192,
        "USB helper did not preserve the path and stderr contracts");
  unlink(script.c_str());

  const auto slow_script = temporary_file();
  write_script(slow_script, "#!/bin/sh\nsleep 60\n");
  check(helper.start(slow_script, error), "slow USB helper process did not start");
  helper.stop();
  check(!helper.running(), "USB helper process group survived cleanup");
  unlink(slow_script.c_str());

  std::cout << "IMU transport PTY, framing and helper lifecycle checks passed\n";
}
