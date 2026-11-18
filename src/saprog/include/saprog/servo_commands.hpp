#pragma once

#include <cmath>
#include <cstdint>
#include <stdexcept>

#include "saprog/servo_units.hpp"

namespace saprog
{
namespace commands
{
// Convert one radian target to the nearest AX tick after checking the measured
// raw-register limits. The caller chooses whether a command must cover every
// configured joint or may submit a subset.
inline uint16_t position_tick(double radians, uint16_t zero, int direction, uint16_t lower,
                              uint16_t upper)
{
  if (!std::isfinite(radians) || lower >= upper || upper > static_cast<uint16_t>(ax::position_max) ||
      (direction != -1 && direction != 1)) {
    throw std::invalid_argument("Invalid servo position or calibration");
  }
  const double raw = ax::to_raw(radians, zero, direction);
  if (!std::isfinite(raw) || raw < lower - 1e-9 || raw > upper + 1e-9) {
    throw std::out_of_range("Position is outside measured servo limits");
  }
  return static_cast<uint16_t>(std::lround(raw));
}
} // namespace commands
} // namespace saprog
