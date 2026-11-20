#pragma once

#include <string>

#include "saprog_imu/imu_sample.hpp"

namespace saprog_imu
{
bool parse_imu_csv(const std::string &line, ImuSample &sample);
}  // namespace saprog_imu
