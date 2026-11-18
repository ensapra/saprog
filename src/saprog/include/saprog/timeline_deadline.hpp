#pragma once

#include <chrono>
#include <cmath>

namespace saprog::timeline
{
// Convert a duration to a steady-clock deadline without overflowing either
// the duration representation or the supplied start time.
inline bool checked_deadline(double seconds, std::chrono::steady_clock::time_point start,
                             std::chrono::steady_clock::time_point &deadline)
{
  using Clock = std::chrono::steady_clock;
  if (!std::isfinite(seconds) || seconds < 0.0) {
    return false;
  }
  const std::chrono::duration<long double> requested{static_cast<long double>(seconds)};
  const std::chrono::duration<long double> remaining{Clock::time_point::max() - start};
  // Keep one tick of headroom so floating-point rounding near the maximum
  // integral duration cannot turn a representable-looking value into overflow.
  if (requested >= remaining) {
    return false;
  }
  deadline = start + std::chrono::duration_cast<Clock::duration>(requested);
  return true;
}
} // namespace saprog::timeline
