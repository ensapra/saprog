#pragma once

#include <cstdint>

// AX servos encode 300 degrees as 0..1023. ROS positions use calibrated radians.
// Keep this conversion identical in commands, telemetry, and arrival checks.
namespace saprog::ax
{
constexpr double position_range_radians = 5.0 * 3.14159265358979323846 / 3.0;
constexpr double position_max = 1023.0;
constexpr double tick_radians = position_range_radians / position_max;
constexpr double speed_unit_radians = 0.111 * 2.0 * 3.14159265358979323846 / 60.0;

inline double to_radians(uint16_t raw, uint16_t zero, int direction)
{
  return direction * (static_cast<int>(raw) - static_cast<int>(zero)) * tick_radians;
}

inline double to_raw(double radians, uint16_t zero, int direction)
{
  // Return the unrounded value so callers can check limits before rounding.
  return zero + direction * radians * position_max / position_range_radians;
}
} // namespace saprog::ax
