#pragma once

#include <array>

#include "saprog_imu/imu_sample.hpp"

namespace saprog_imu
{
class ImuOrientationEstimator
{
public:
  // Magnetic measurements must use the same body frame as accel/gyro.
  explicit ImuOrientationEstimator(bool use_magnetometer = true);
  void reset();
  void update(const ImuSample &sample);
  bool valid() const;
  const std::array<double, 4> &orientation() const;

private:
  bool use_magnetometer_;
  std::array<double, 4> orientation_{1.0, 0.0, 0.0, 0.0};  // w, x, y, z
  bool valid_{false};
  double last_time_ms_{-1.0};
  double tilt_resume_ms_{0.0};
};
}  // namespace saprog_imu
