#include "saprog_imu/imu_orientation_estimator.hpp"

#include <algorithm>
#include <cmath>

#include <gtest/gtest.h>

namespace saprog_imu
{
namespace
{
constexpr double pi = 3.14159265358979323846;

void expect_axis_rotation(const ImuOrientationEstimator &estimator, int axis, double angle)
{
  ASSERT_TRUE(estimator.valid());
  const auto &q = estimator.orientation();
  const double dot = q[0] * std::cos(angle / 2.0) + q[axis + 1] * std::sin(angle / 2.0);
  // q and -q represent the same rotation; also check normalization.
  EXPECT_NEAR(std::abs(dot), 1.0, 1e-9);
  EXPECT_NEAR(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3], 1.0, 1e-12);
}

ImuSample axis_sample(int axis, double angle)
{
  ImuSample sample;
  sample.acceleration = {0.0, 0.0, 1.0};
  if (axis == 0) {
    sample.acceleration = {0.0, std::sin(angle), std::cos(angle)};
  } else if (axis == 1) {
    sample.acceleration = {-std::sin(angle), 0.0, std::cos(angle)};
  }
  // Deliberately stuck magnetic reading: it must not override relative attitude.
  sample.magnetic_field = {30.0, 10.0, -40.0};
  return sample;
}
}  // namespace

TEST(ImuOrientationEstimator, HoldsRealRotationsWithUntrustedMagneticReadings)
{
  for (int axis = 0; axis < 3; ++axis) {
    for (const double direction : {-1.0, 1.0}) {
      SCOPED_TRACE(axis);
      SCOPED_TRACE(direction);
      ImuOrientationEstimator estimator;
      estimator.update(axis_sample(axis, 0.0));
      for (int step = 1; step <= 100; ++step) {
        auto sample = axis_sample(axis, direction * step * pi / 200.0);
        sample.time_ms = step * 10.0;
        sample.angular_velocity[axis] = direction * 90.0;
        estimator.update(sample);
      }
      expect_axis_rotation(estimator, axis, direction * pi / 2.0);
      for (int step = 101; step <= 1100; ++step) {
        auto sample = axis_sample(axis, direction * pi / 2.0);
        sample.time_ms = step * 10.0;
        estimator.update(sample);
      }
      expect_axis_rotation(estimator, axis, direction * pi / 2.0);
    }
  }
}

TEST(ImuOrientationEstimator, IgnoresMagneticHeadingAtStartupAndAfterResetByDefault)
{
  ImuOrientationEstimator estimator;
  const auto sample = axis_sample(2, 0.0);
  estimator.update(sample);
  expect_axis_rotation(estimator, 2, 0.0);
  estimator.reset();
  EXPECT_FALSE(estimator.valid());
  estimator.update(sample);
  expect_axis_rotation(estimator, 2, 0.0);
}

TEST(ImuOrientationEstimator, IntegratesHeadingWithoutMagnetometerData)
{
  ImuOrientationEstimator estimator;
  auto sample = axis_sample(2, 0.0);
  sample.magnetic_field = {};
  estimator.update(sample);
  sample.angular_velocity[2] = 90.0;
  for (int step = 1; step <= 100; ++step) {
    sample.time_ms = step * 10.0;
    estimator.update(sample);
  }
  expect_axis_rotation(estimator, 2, pi / 2.0);
}

TEST(ImuOrientationEstimator, RetainsGravityDriftCorrectionWithoutMagneticHeading)
{
  ImuOrientationEstimator estimator;
  auto sample = axis_sample(0, 0.0);
  estimator.update(sample);
  // Simulate a gyro error while the sensor is physically level.
  sample.time_ms = 100.0;
  sample.angular_velocity[0] = 90.0;
  estimator.update(sample);
  expect_axis_rotation(estimator, 0, pi / 20.0);
  sample.angular_velocity = {};
  for (int step = 11; step <= 1010; ++step) {
    sample.time_ms = step * 10.0;
    estimator.update(sample);
  }
  expect_axis_rotation(estimator, 0, 0.0);
}

TEST(ImuOrientationEstimator, ExplicitMagneticModeTracksConsistentHeading)
{
  ImuOrientationEstimator estimator(true);
  auto sample = axis_sample(2, 0.0);
  sample.magnetic_field = {30.0, 0.0, -40.0};
  estimator.update(sample);
  for (int step = 1; step <= 1100; ++step) {
    const double angle = std::min(step, 100) * pi / 200.0;
    sample.time_ms = step * 10.0;
    sample.angular_velocity[2] = step <= 100 ? 90.0 : 0.0;
    sample.magnetic_field = {30.0 * std::cos(angle), -30.0 * std::sin(angle), -40.0};
    estimator.update(sample);
  }
  expect_axis_rotation(estimator, 2, pi / 2.0);
}

TEST(ImuOrientationEstimator, ExplicitMagneticModeCorrectsDriftAndSurvivesReset)
{
  ImuOrientationEstimator estimator(true);
  auto sample = axis_sample(2, 0.0);
  sample.magnetic_field = {30.0, 0.0, -40.0};
  estimator.update(sample);
  // A changed field implies a quarter turn; the absolute mode must follow it.
  sample.magnetic_field = {0.0, -30.0, -40.0};
  for (int step = 1; step <= 1000; ++step) {
    sample.time_ms = step * 10.0;
    estimator.update(sample);
  }
  expect_axis_rotation(estimator, 2, pi / 2.0);
  estimator.reset();
  estimator.update(sample);
  expect_axis_rotation(estimator, 2, pi / 2.0);
}
}  // namespace saprog_imu
