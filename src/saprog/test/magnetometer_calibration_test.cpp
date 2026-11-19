#include "saprog/magnetometer_calibration.hpp"
#include <cstdlib>
#include <iostream>
#include <limits>

using namespace saprog::imu;

static void check(bool condition, const char *message)
{
  if (!condition) {
    std::cerr << message << '\n';
    std::exit(1);
  }
}

int main()
{
  MagnetometerCalibration calibration;
  MagneticCorrection candidate;
  std::string error;
  calibration.start();
  for (int i = 0; i < 1200; ++i) {
    // Uniform spherical coverage with known hard iron and unequal axis scales.
    const double z = 1 - 2 * (i + 0.5) / 1200;
    const double angle = i * pi * (3 - std::sqrt(5.0));
    const double radial = std::sqrt(1 - z * z);
    calibration.observe({12 + 40 * radial * std::cos(angle),
                         -8 + 50 * radial * std::sin(angle), 20 + 60 * z});
  }
  check(calibration.estimate(candidate, error), error.c_str());
  check(!calibration.calibrated, "estimate committed before persistence");
  check(std::abs(candidate.offset[0] - 12) < 0.2 &&
        std::abs(candidate.offset[1] + 8) < 0.2 &&
        std::abs(candidate.offset[2] - 20) < 0.2, "wrong hard-iron offsets");
  check(std::abs(candidate.scale[0] - 1.25) < 0.01 &&
        std::abs(candidate.scale[1] - 1) < 0.01 &&
        std::abs(candidate.scale[2] - 50.0 / 60) < 0.01, "wrong axis scale");
  calibration.commit(candidate);
  const auto previous = calibration.correction;
  check(calibration.calibrated && !calibration.collecting, "commit lifecycle");
  check(norm(previous.apply({52, -8, 20})) > 49, "correction not applied");

  calibration.start();
  for (int i = 0; i < 500; ++i) {
    const double angle = 2 * pi * i / 500;
    calibration.observe({40 * std::cos(angle), 40 * std::sin(angle), 1});
  }
  check(!calibration.estimate(candidate, error), "flat rotation accepted");
  check(calibration.correction.offset == previous.offset, "failure replaced correction");
  calibration.cancel("cancelled");
  check(calibration.calibrated && calibration.sample_count() == 0, "cancel erased calibration");

  calibration.start();
  for (int i = 0; i < 13000; ++i) { calibration.observe({1, 2, 3}); }
  check(calibration.sample_count() == 12000, "unbounded sample collection");
  check(!calibration.estimate(candidate, error), "stationary data accepted");
  calibration.start();
  calibration.observe({std::numeric_limits<double>::quiet_NaN(), 0, 0});
  check(calibration.sample_count() == 0, "nonfinite sample accepted");
  check(!calibration.estimate(candidate, error), "empty collection accepted");
  candidate.scale[0] = -1;
  check(!candidate.valid(), "invalid persisted scale accepted");
  candidate.scale[0] = std::numeric_limits<double>::infinity();
  check(!candidate.valid(), "infinite persisted scale accepted");
  return 0;
}
