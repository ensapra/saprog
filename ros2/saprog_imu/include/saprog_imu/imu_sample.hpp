#pragma once

#include <array>

namespace saprog_imu
{
struct ImuSample
{
  double time_ms{};
  std::array<double, 3> acceleration{};  // g
  std::array<double, 3> angular_velocity{};  // degrees/s
  std::array<double, 3> magnetic_field{};  // microtesla
};
}  // namespace saprog_imu
