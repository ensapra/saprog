#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

namespace saprog::imu
{
constexpr double pi = 3.14159265358979323846;
constexpr double radians = pi / 180.0;
using Vec = std::array<double, 3>;
using Quat = std::array<double, 4>; // w, x, y, z; sensor -> world

inline double norm(const Vec &v)
{
  return std::hypot(v[0], v[1], v[2]);
}
inline Vec subtract(const Vec &a, const Vec &b)
{
  return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}
inline Quat conjugate(const Quat &q)
{
  return {q[0], -q[1], -q[2], -q[3]};
}
inline Quat multiply(const Quat &a, const Quat &b)
{
  const auto [w, x, y, z] = a;
  const auto [v, i, j, k] = b;
  return {w * v - x * i - y * j - z * k, w * i + x * v + y * k - z * j,
          w * j - x * k + y * v + z * i, w * k + x * j - y * i + z * v};
}
inline Quat normalize(Quat q)
{
  const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  for (auto &v : q) {
    v /= n;
  }
  return q;
}
inline Vec rotate(const Quat &q, const Vec &v)
{
  const auto r = multiply(multiply(q, {0, v[0], v[1], v[2]}), conjugate(q));
  return {r[1], r[2], r[3]};
}
inline Quat from_euler(double roll, double pitch, double yaw)
{
  const double cr = std::cos(roll / 2), sr = std::sin(roll / 2);
  const double cp = std::cos(pitch / 2), sp = std::sin(pitch / 2);
  const double cy = std::cos(yaw / 2), sy = std::sin(yaw / 2);
  return {cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
          cr * cp * sy - sr * sp * cy};
}
inline double yaw(const Quat &q)
{
  const auto [w, x, y, z] = q;
  return std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}

struct Sample
{
  double time_ms{};
  Vec accel{}, gyro{}, mag{}; // firmware: g, degrees/s, microtesla
};

// Only the documented calculated 10-column format is accepted. Extra enabled
// firmware fields must be disabled; guessing their layout can produce false poses.
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
  sample = {values[0],
            {values[1], values[2], values[3]},
            {values[4], values[5], values[6]},
            {values[7], values[8], values[9]}};
  return true;
}

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
        if (magnetic_heading && norm(s.mag) > 1e-9) {
          const double error = std::remainder(heading - yaw(attitude), 2 * pi);
          attitude = multiply({std::cos(error * weight / 2), 0, 0, std::sin(error * weight / 2)},
                              attitude);
        }
        attitude = normalize(attitude);
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
