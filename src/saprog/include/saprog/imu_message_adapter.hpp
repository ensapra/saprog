#pragma once

#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "saprog/imu_filter.hpp"

namespace saprog::imu
{
inline geometry_msgs::msg::Quaternion to_ros_quaternion(const Quat &source)
{
  geometry_msgs::msg::Quaternion result;
  result.w = source[0];
  result.x = source[1];
  result.y = source[2];
  result.z = source[3];
  return result;
}

inline void to_ros_vector(geometry_msgs::msg::Vector3 &destination, const Vec &source, double scale)
{
  destination.x = source[0] * scale;
  destination.y = source[1] * scale;
  destination.z = source[2] * scale;
}

// Map firmware-unit samples to ROS SI messages at the reception timestamp.
inline void make_ros_messages(const Sample &sample, const Filter &filter, const std::string &frame,
                              const builtin_interfaces::msg::Time &stamp,
                              sensor_msgs::msg::Imu &raw, sensor_msgs::msg::Imu &data,
                              sensor_msgs::msg::MagneticField &mag)
{
  raw.header.stamp = stamp;
  raw.header.frame_id = frame;
  to_ros_vector(raw.linear_acceleration, sample.accel, 9.80665);
  to_ros_vector(raw.angular_velocity, sample.gyro, radians);
  raw.orientation_covariance[0] = -1;

  data = raw;
  data.orientation = to_ros_quaternion(filter.attitude);
  data.orientation_covariance[0] = filter.valid ? 0 : -1;
  to_ros_vector(data.angular_velocity, subtract(sample.gyro, filter.bias), radians);

  mag.header = raw.header;
  to_ros_vector(mag.magnetic_field, sample.mag, 1e-6);
}
} // namespace saprog::imu
