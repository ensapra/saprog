#include "saprog_imu/imu_orientation_estimator.hpp"

#include <algorithm>
#include <cmath>

namespace saprog_imu
{
namespace
{
using Vector = std::array<double, 3>;
// Quaternion layout is w, x, y, z; radians are used internally.
using Quaternion = std::array<double, 4>;
constexpr double radians = 3.14159265358979323846 / 180.0;

double norm(const Vector &v)
{
  return std::hypot(v[0], v[1], v[2]);
}

Quaternion multiply(const Quaternion &a, const Quaternion &b)
{
  const auto [w, x, y, z] = a;
  const auto [v, i, j, k] = b;
  return {w * v - x * i - y * j - z * k, w * i + x * v + y * k - z * j,
          w * j - x * k + y * v + z * i, w * k + x * j - y * i + z * v};
}

Quaternion normalize(Quaternion q)
{
  const double magnitude = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  for (auto &value : q) {
    value /= magnitude;
  }
  return q;
}

Quaternion from_euler(double roll, double pitch, double yaw)
{
  const double cr = std::cos(roll / 2), sr = std::sin(roll / 2);
  const double cp = std::cos(pitch / 2), sp = std::sin(pitch / 2);
  const double cy = std::cos(yaw / 2), sy = std::sin(yaw / 2);
  return {cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
          cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy};
}

Vector rotate(const Quaternion &q, const Vector &v)
{
  const Quaternion conjugate{q[0], -q[1], -q[2], -q[3]};
  const auto rotated = multiply(multiply(q, {0.0, v[0], v[1], v[2]}), conjugate);
  return {rotated[1], rotated[2], rotated[3]};
}
}  // namespace

void ImuOrientationEstimator::update(const ImuSample &sample)
{
  // Firmware acceleration is in g. Its direction gives tilt, but not rotation
  // around gravity (yaw).
  const double acceleration_magnitude = norm(sample.acceleration);
  if (acceleration_magnitude < 1e-9) {
    return;
  }
  const Vector acceleration{
    sample.acceleration[0] / acceleration_magnitude,
    sample.acceleration[1] / acceleration_magnitude,
    sample.acceleration[2] / acceleration_magnitude};
    
  const double roll = std::atan2(acceleration[1], acceleration[2]);
  const double pitch = std::atan2(-acceleration[0], std::hypot(acceleration[1], acceleration[2]));
  const double hx = sample.magnetic_field[0] * std::cos(pitch) +
    sample.magnetic_field[1] * std::sin(roll) * std::sin(pitch) +
    sample.magnetic_field[2] * std::cos(roll) * std::sin(pitch);
  const double hy = sample.magnetic_field[1] * std::cos(roll) -
    sample.magnetic_field[2] * std::sin(roll);
  const double heading = std::atan2(-hy, hx);

  if (!valid_) {
    // Start from measured tilt and, when available, magnetic heading.
    orientation_ = from_euler(roll, pitch, norm(sample.magnetic_field) > 1e-9 ? heading : 0.0);
    valid_ = true;
    tilt_resume_ms_ = sample.time_ms + 250.0;
  } else {
    // Integrate gyro rates (degrees/s) over the firmware sample-time interval.
    const double elapsed = (sample.time_ms - last_time_ms_) / 1000.0;
    const double dt = elapsed > 0.0 && elapsed <= 1.0 ? elapsed : 0.0;
    if (dt > 0.0) {
      const Vector omega{
        sample.angular_velocity[0] * radians,
        sample.angular_velocity[1] * radians,
        sample.angular_velocity[2] * radians};
      const double rate = norm(omega);
      const double half_angle = rate * dt / 2.0;
      const double factor = rate > 1e-12 ? std::sin(half_angle) / rate : dt / 2.0;
      orientation_ = normalize(multiply(
        orientation_, {std::cos(half_angle), omega[0] * factor, omega[1] * factor, omega[2] * factor}));

      constexpr double gravity_g = 1.0;
      // Trust acceleration as a gravity direction only when its magnitude and
      // the current rotation rate suggest little linear motion.
      const bool reliable_acceleration =
        std::abs(acceleration_magnitude / gravity_g - 1.0) < 0.08 && rate < 45.0 * radians;
      if (!reliable_acceleration) {
        tilt_resume_ms_ = sample.time_ms + 250.0;
      }
      if (reliable_acceleration && sample.time_ms >= tilt_resume_ms_) {
        // Slowly correct accumulated gyro tilt drift using measured gravity.
        const auto world_acceleration = rotate(orientation_, acceleration);
        const double horizontal = std::hypot(world_acceleration[0], world_acceleration[1]);
        if (horizontal > 1e-9) {
          const double weight = 1.0 - std::exp(-dt / 0.5);
          const double correction = std::atan2(horizontal, world_acceleration[2]) * weight / 2.0;
          const double factor_tilt = std::sin(correction) / horizontal;
          orientation_ = normalize(multiply(
            {std::cos(correction), world_acceleration[1] * factor_tilt,
             -world_acceleration[0] * factor_tilt, 0.0}, orientation_));
        }
      }

      if (norm(sample.magnetic_field) > 1e-9) {
        // Rotate the magnetic measurement into world space and slowly align
        // its horizontal component with the estimator's north axis.
        const auto magnetic_world = rotate(orientation_, sample.magnetic_field);
        if (std::hypot(magnetic_world[0], magnetic_world[1]) > 1e-9) {
          const double error = std::atan2(-magnetic_world[1], magnetic_world[0]);
          const double weight = 1.0 - std::exp(-dt / 0.5);
          orientation_ = normalize(multiply(
            {std::cos(error * weight / 2.0), 0.0, 0.0, std::sin(error * weight / 2.0)},
            orientation_));
        }
      }
    } else {
      tilt_resume_ms_ = sample.time_ms + 250.0;
    }
  }
  last_time_ms_ = sample.time_ms;
}

bool ImuOrientationEstimator::valid() const
{
  return valid_;
}

const std::array<double, 4> &ImuOrientationEstimator::orientation() const
{
  return orientation_;
}
}  // namespace saprog_imu
