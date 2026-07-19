#pragma once

#include <algorithm>
#include <cmath>

namespace mm::characterization {

inline float updatePeakVelocityMagnitude(const float previous_peak,
                                         const float measured_velocity_rad_s) {
  return std::max(previous_peak, std::fabs(measured_velocity_rad_s));
}

}  // namespace mm::characterization
