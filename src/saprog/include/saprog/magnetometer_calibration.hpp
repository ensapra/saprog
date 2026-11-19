#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

#include "saprog/imu_math.hpp"

namespace saprog::imu
{
// Hard-iron offset in microtesla, plus diagonal (axis-aligned) scale correction.
// This deliberately does not model cross-axis soft-iron distortion.
struct MagneticCorrection
{
  Vec offset{0, 0, 0};
  Vec scale{1, 1, 1};

  bool valid() const
  {
    for (int i = 0; i < 3; ++i) {
      if (!std::isfinite(offset[i]) || std::abs(offset[i]) > 10000 ||
          !std::isfinite(scale[i]) || scale[i] < 0.25 || scale[i] > 4) {
        return false;
      }
    }
    return true;
  }

  Vec apply(const Vec &field) const
  {
    return {(field[0] - offset[0]) * scale[0], (field[1] - offset[1]) * scale[1],
            (field[2] - offset[2]) * scale[2]};
  }
};

class MagnetometerCalibration
{
public:
  bool collecting = false;
  bool calibrated = false;
  MagneticCorrection correction;
  std::string message = "Not calibrated. Rotate through all orientations during collection.";

  void start()
  {
    samples_.clear();
    collecting = true;
    message = "Rotate slowly through ALL orientations, including upside down; then Save.";
  }

  void cancel(const std::string &reason)
  {
    collecting = false;
    samples_.clear();
    message = reason;
  }

  std::size_t sample_count() const { return samples_.size(); }

  void observe(const Vec &field)
  {
    if (!collecting || samples_.size() >= 12000) {
      return;
    }
    if (std::all_of(field.begin(), field.end(), [](double v) { return std::isfinite(v); })) {
      samples_.push_back(field);
    }
  }

  // Compute a candidate without replacing the active correction. The owner must
  // persist it successfully before committing it, preserving old state on errors.
  bool estimate(MagneticCorrection &candidate, std::string &error) const
  {
    if (!collecting || samples_.size() < 200) {
      error = "Need at least 200 samples; keep rotating through all orientations.";
      return false;
    }
    Vec low = samples_.front(), high = low, radius{};
    for (const auto &field : samples_) {
      for (int i = 0; i < 3; ++i) {
        low[i] = std::min(low[i], field[i]);
        high[i] = std::max(high[i], field[i]);
      }
    }
    double mean_radius = 0;
    for (int i = 0; i < 3; ++i) {
      candidate.offset[i] = (high[i] + low[i]) / 2;
      radius[i] = (high[i] - low[i]) / 2;
      if (radius[i] < 5 || radius[i] > 1000) {
        error = "Insufficient rotation or implausible field range; turn every axis away from metal.";
        return false;
      }
      mean_radius += radius[i] / 3;
    }
    for (int i = 0; i < 3; ++i) {
      candidate.scale[i] = mean_radius / radius[i];
    }
    if (!candidate.valid()) {
      error = "Excessive axis distortion; move away from magnetic interference and retry.";
      return false;
    }
    std::array<unsigned, 8> octants{};
    std::array<unsigned, 6> faces{};
    double residual = 0;
    for (const auto &field : samples_) {
      const auto corrected = candidate.apply(field);
      const double relative_norm = norm(corrected) / mean_radius;
      residual += (relative_norm - 1) * (relative_norm - 1);
      unsigned octant = 0;
      for (int i = 0; i < 3; ++i) {
        if (corrected[i] > 0) {
          octant |= 1u << i;
        }
        if (std::abs(corrected[i]) > 0.8 * mean_radius) {
          ++faces[2 * i + (corrected[i] > 0 ? 1 : 0)];
        }
      }
      ++octants[octant];
    }
    if (*std::min_element(octants.begin(), octants.end()) < 3 ||
        *std::min_element(faces.begin(), faces.end()) < 3) {
      error = "Incomplete coverage: turn each face up and rotate upside down, then Save again.";
      return false;
    }
    if (std::sqrt(residual / samples_.size()) > 0.15) {
      error = "Field changed or distortion is too complex; retry away from motors and metal.";
      return false;
    }
    return true;
  }

  void commit(const MagneticCorrection &candidate)
  {
    correction = candidate;
    calibrated = true;
    cancel("Magnetometer calibration saved; magnetic heading now uses the correction.");
  }

private:
  std::vector<Vec> samples_;
};
} // namespace saprog::imu
