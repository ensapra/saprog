#include "saprog_imu/imu_csv_parser.hpp"

#include <array>
#include <cmath>
#include <sstream>

namespace saprog_imu
{
bool parse_imu_csv(const std::string &line, ImuSample &sample)
{
  std::istringstream input(line);
  std::array<double, 10> values{};
  std::string field;
  std::size_t count = 0;
  while (std::getline(input, field, ',')) {
    if (count == values.size()) {
      return false;
    }
    try {
      std::size_t consumed = 0;
      const double value = std::stod(field, &consumed);
      if (!std::isfinite(value) || field.find_first_not_of(" \t\r", consumed) != std::string::npos) {
        return false;
      }
      values[count++] = value;
    } catch (...) {
      return false;
    }
  }
  if (count != values.size() || values[0] < 0 || (!line.empty() && line.back() == ',')) {
    return false;
  }

  sample = {values[0], {values[1], values[2], values[3]},
            {values[4], values[5], values[6]}, {values[7], values[8], values[9]}};
  return true;
}
}  // namespace saprog_imu
