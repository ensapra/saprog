#include "saprog/imu_message_adapter.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

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
  Sample sample{123, {1, -2, 0.5}, {180, -90, 45}, {12, -34, 56}};
  Filter filter;
  filter.valid = true;
  filter.attitude = {0.5, 0.1, 0.2, 0.3};
  filter.bias = {1, 2, 3};
  sensor_msgs::msg::Imu raw, data;
  sensor_msgs::msg::MagneticField mag;
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 7;
  stamp.nanosec = 11;
  make_ros_messages(sample, filter, "imu_link", stamp, raw, data, mag);

  check(raw.header.frame_id == "imu_link" && raw.header.stamp.sec == 7,
        "raw message header was not mapped");
  check(std::abs(raw.linear_acceleration.x - 9.80665) < 1e-9 &&
            std::abs(raw.linear_acceleration.y + 19.6133) < 1e-9,
        "acceleration was not converted from g to m/s^2");
  check(std::abs(raw.angular_velocity.x - pi) < 1e-12 &&
            std::abs(raw.angular_velocity.y + pi / 2) < 1e-12,
        "raw gyro was not converted from degrees/s to radians/s");
  check(raw.orientation_covariance[0] == -1 && data.orientation_covariance[0] == 0,
        "orientation availability covariance changed");
  check(data.orientation.w == 0.5 && data.orientation.x == 0.1 && data.orientation.y == 0.2 &&
            data.orientation.z == 0.3,
        "wxyz filter quaternion was not mapped to ROS xyzw fields");
  check(std::abs(data.angular_velocity.x - (179 * radians)) < 1e-12 &&
            std::abs(data.angular_velocity.z - (42 * radians)) < 1e-12,
        "gyro bias was not subtracted before SI conversion");
  check(std::abs(mag.magnetic_field.x - 12e-6) < 1e-15 &&
            std::abs(mag.magnetic_field.y + 34e-6) < 1e-15,
        "magnetic field was not converted from microtesla to tesla");
  std::cout << "IMU ROS unit and quaternion mapping checks passed\n";
}
