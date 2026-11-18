#pragma once

#include <array>
#include <cmath>
#include <sstream>
#include <string>

#include "saprog/imu_math.hpp"

namespace saprog::imu
{
// Firmware calculated output in g, degrees/s, and microtesla before startup scaling.
struct Sample
{
  double time_ms{};
  Vec accel{}, gyro{}, mag{};
};

// Accept only the documented ten-column calculated format; extra fields are ambiguous.
inline bool parse_sample(const std::string &line, Sample &sample)
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
      std::size_t end;
      const double value = std::stod(field, &end);
      if (!std::isfinite(value) || field.find_first_not_of(" \t\r", end) != std::string::npos) {
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
  sample = {values[0], {values[1], values[2], values[3]}, {values[4], values[5], values[6]},
            {values[7], values[8], values[9]}};
  return true;
}
} // namespace saprog::imu
