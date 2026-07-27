#pragma once

#include <algorithm>
#include <cmath>

#include "core/Types.hpp"

namespace mm {

constexpr float kMotorCommandZeroThreshold = 0.0001F;

inline float compensateMotorDeadband(
    float signed_duty, const MotorCharacteristics& characteristics,
    const SafetyConfiguration& safety) {
  signed_duty =
      std::clamp(signed_duty, -safety.max_duty, safety.max_duty);
  if (std::fabs(signed_duty) < kMotorCommandZeroThreshold) {
    return 0.0F;
  }
  const bool forward = signed_duty > 0.0F;
  const float minimum = forward ? characteristics.start_duty_forward
                                : characteristics.start_duty_reverse;
  const float compensated =
      minimum + std::fabs(signed_duty) * (1.0F - minimum);
  return std::copysign(std::clamp(compensated, 0.0F, safety.max_duty),
                       signed_duty);
}

}  // namespace mm
