#pragma once

#include <cstdint>

namespace saprog::ax_register
{
struct Register
{
  uint16_t address;
  uint16_t width;
};

constexpr Register cw_angle_limit{6, 2};
constexpr Register angle_limits{6, 4};
constexpr Register torque_enable{24, 1};
constexpr Register goal_position{30, 2};
constexpr Register moving_speed{32, 2};
constexpr Register torque_limit{34, 2};
constexpr Register present_position{36, 2};
} // namespace saprog::ax_register
