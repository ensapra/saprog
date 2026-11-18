#pragma once

#include <array>
#include <cmath>

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
} // namespace saprog::imu
