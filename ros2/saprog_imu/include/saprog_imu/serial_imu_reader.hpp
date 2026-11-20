#pragma once

#include <string>
#include <vector>

namespace saprog_imu
{
class SerialImuReader
{
public:
  SerialImuReader(std::string device, int baud_rate);
  ~SerialImuReader();

  SerialImuReader(const SerialImuReader &) = delete;
  SerialImuReader &operator=(const SerialImuReader &) = delete;

  bool connected() const;
  bool connect(std::string &error);
  bool read_lines(std::vector<std::string> &lines, std::string &error);
  bool received_data_this_read() const;
  bool send_resume_toggle(std::string &error);
  void disconnect();

private:
  std::string device_;
  int baud_rate_;
  int file_descriptor_{-1};
  std::string pending_;
  bool received_data_this_read_{false};
};
}  // namespace saprog_imu
