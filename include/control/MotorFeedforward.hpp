#pragma once

#include <algorithm>
#include <cmath>

namespace mm {

inline float motorFeedforwardDuty(const float desired_velocity_rad_s,
                                  const float breakaway_duty,
                                  const float maximum_velocity_rad_s,
                                  const float maximum_duty) {
  if (!(desired_velocity_rad_s > 0.0F) || !(maximum_velocity_rad_s > 0.0F)) {
    return 0.0F;
  }
  const float fraction = std::clamp(
      desired_velocity_rad_s / std::fabs(maximum_velocity_rad_s), 0.0F, 1.0F);
  return std::clamp(breakaway_duty + fraction * (maximum_duty - breakaway_duty),
                    0.0F, maximum_duty);
}

}  // namespace mm
