#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "saprog/imu_sample.hpp"

namespace saprog::imu
{

class Filter
{
public:
  Quat attitude{1, 0, 0, 0};
  Vec bias{0, 0, 0};
  bool valid = false;
  bool zeroed = false;
  bool magnetic_heading = false;
  bool calibrating = false;
  bool calibrated = false;
  bool correcting_tilt = false;
  double last_time_ms = -1;
  double sample_hz = 0;
  double peak_gyro_dps = 0;
  std::uint64_t time_gaps = 0;
  std::string calibration_status = "Not calibrated; stationary calibration reduces gyro bias.";

  Quat relative() const
  {
    return zeroed ? normalize(multiply(conjugate(reference_), attitude)) : attitude;
  }
  void zero()
  {
    if (valid) {
      reference_ = attitude;
      zeroed = true;
    }
  }
  void clear_zero()
  {
    zeroed = false;
  }
  void set_heading(bool enabled)
  {
    magnetic_heading = enabled;
    valid = false;
    zeroed = false;
    cancel_calibration("Heading mode changed; set zero again.");
  }
  void start_calibration()
  {
    samples_.clear();
    calibrating = true;
    calibration_status = "Keep flat and completely still for four seconds.";
  }
  void cancel_calibration(const std::string &reason)
  {
    calibrating = false;
    samples_.clear();
    calibration_status = reason;
  }

  void update(const Sample &s)
  {
    const auto g = subtract(s.gyro, bias);
    for (double v : s.gyro) {
      peak_gyro_dps = std::max(peak_gyro_dps, std::abs(v));
    }
    const double magnitude = norm(s.accel);
    const Vec accel = magnitude > 1e-9 ? Vec{s.accel[0] / magnitude, s.accel[1] / magnitude,
                                             s.accel[2] / magnitude}
                                       : Vec{0, 0, 0};
    const double roll = std::atan2(accel[1], accel[2]);
    const double pitch = std::atan2(-accel[0], std::hypot(accel[1], accel[2]));
    const double hx = s.mag[0] * std::cos(pitch) + s.mag[1] * std::sin(roll) * std::sin(pitch) +
                      s.mag[2] * std::cos(roll) * std::sin(pitch);
    const double hy = s.mag[1] * std::cos(roll) - s.mag[2] * std::sin(roll);
    const double heading = std::atan2(-hy, hx);
    correcting_tilt = false;
    if (!valid) {
      if (magnitude < 1e-9) {
        return;
      }
      attitude = from_euler(roll, pitch, magnetic_heading ? heading : 0.0);
      valid = true;
      tilt_resume_ms_ = s.time_ms + 250;
    } else {
      const double elapsed = (s.time_ms - last_time_ms) / 1000;
      const double dt = elapsed > 0 && elapsed <= 1 ? elapsed : 0;
      if (dt == 0 || (sample_hz > 0 && elapsed > 2.5 / sample_hz)) {
        ++time_gaps;
      }
      if (dt > 0) {
        sample_hz = sample_hz == 0 ? 1 / dt : 0.95 * sample_hz + 0.05 / dt;
      }
      const Vec omega{g[0] * radians, g[1] * radians, g[2] * radians};
      const double rate = norm(omega), angle = rate * dt / 2;
      const double factor = rate > 1e-12 ? std::sin(angle) / rate : dt / 2;
      attitude = normalize(multiply(
          attitude, {std::cos(angle), omega[0] * factor, omega[1] * factor, omega[2] * factor}));
      const bool reliable = std::abs(magnitude / gravity_ - 1) < 0.08 && rate < 45 * radians;
      if (!reliable || dt == 0) {
        tilt_resume_ms_ = s.time_ms + 250;
      }
      if (reliable && s.time_ms >= tilt_resume_ms_) {
        correcting_tilt = true;
        const double weight = 1 - std::exp(-dt / 0.5);
        const auto a = rotate(attitude, accel);
        const double horizontal = std::hypot(a[0], a[1]);
        const double correction = std::atan2(horizontal, a[2]) * weight / 2;
        if (horizontal > 1e-9) {
          const double k = std::sin(correction) / horizontal;
          attitude = multiply({std::cos(correction), a[1] * k, -a[0] * k, 0}, attitude);
        }
        attitude = normalize(attitude);
      }
      // Magnetometer yaw is an independent observation. Keep applying it while
      // the accelerometer is unreliable or the robot is turning; otherwise the
      // heading can freeze precisely while the magnetic vector is changing.
      // Project the measured field through the current sensor-to-world attitude
      // so this correction does not depend on a potentially contaminated accel.
      if (magnetic_heading && norm(s.mag) > 1e-9 && dt > 0) {
        const auto magnetic_world = rotate(attitude, s.mag);
        if (std::hypot(magnetic_world[0], magnetic_world[1]) > 1e-9) {
          const double error = std::atan2(-magnetic_world[1], magnetic_world[0]);
          const double weight = 1 - std::exp(-dt / 0.5);
          attitude = normalize(multiply(
              {std::cos(error * weight / 2), 0, 0, std::sin(error * weight / 2)}, attitude));
        }
      }
    }
    last_time_ms = s.time_ms;
    if (calibrating) {
      collect(s);
    }
  }

private:
  Quat reference_{1, 0, 0, 0};
  double gravity_ = 1;
  double tilt_resume_ms_ = 0;
  std::vector<Sample> samples_;

  void collect(const Sample &s)
  {
    if (!samples_.empty() &&
        (s.time_ms <= samples_.back().time_ms || s.time_ms - samples_.back().time_ms > 1000)) {
      cancel_calibration("Calibration rejected: non-contiguous timestamps.");
      return;
    }
    samples_.push_back(s);
    if (s.time_ms - samples_.front().time_ms < 4000 || samples_.size() < 20) {
      return;
    }
    Vec accel{}, gyro{}, mag{};
    for (const auto &sample : samples_) {
      for (int i = 0; i < 3; ++i) {
        accel[i] += sample.accel[i] / samples_.size();
        gyro[i] += sample.gyro[i] / samples_.size();
        mag[i] += sample.mag[i] / samples_.size();
      }
    }
    const double gravity = norm(accel);
    if (gravity < 0.75 || gravity > 1.25) {
      cancel_calibration(
          "Calibration rejected: acceleration must be near 1 g; check serial units.");
      return;
    }
    for (const auto &sample : samples_) {
      if (norm(subtract(sample.accel, accel)) > 0.05 * gravity ||
          norm(subtract(sample.gyro, gyro)) > 1 || norm(gyro) > 5) {
        cancel_calibration(
            "Calibration rejected: motion or excessive noise; keep still and retry.");
        return;
      }
    }
    bias = gyro;
    gravity_ = gravity;
    calibrated = true;
    calibrating = false;
    valid = false;
    samples_.clear();
    update({s.time_ms, accel, gyro, mag});
    zero();
    calibration_status = "Gyro bias calibrated and zero saved. Relative yaw may still drift.";
  }
};
} // namespace saprog::imu
