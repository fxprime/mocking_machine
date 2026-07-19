#pragma once

#include <algorithm>
#include <cmath>

#include "core/Types.hpp"

namespace mm::characterization {

inline bool prepareCharacterizedSettings(const MachineSettings& current,
                                          const MotorCharacteristics& measured,
                                          MachineSettings& candidate) {
  if (!std::isfinite(measured.max_velocity_forward_rad_s) ||
      !std::isfinite(measured.max_velocity_reverse_rad_s) ||
      measured.max_velocity_forward_rad_s <= 0.0F ||
      measured.max_velocity_reverse_rad_s <= 0.0F) {
    return false;
  }

  candidate = current;
  candidate.motor = measured;
  const float detected_velocity_limit = std::min(
      measured.max_velocity_forward_rad_s, measured.max_velocity_reverse_rad_s);
  candidate.safety.max_velocity_rad_s =
      std::min(current.safety.max_velocity_rad_s, detected_velocity_limit);
  candidate.safety.encoder_timeout_velocity_rad_s = std::min(
      current.safety.encoder_timeout_velocity_rad_s,
      candidate.safety.max_velocity_rad_s);

  for (uint8_t index = 0U; index < candidate.profile_count; ++index) {
    auto& profile = candidate.profiles[index];
    const float limit = candidate.safety.max_velocity_rad_s;
    profile.target_velocity_rad_s = std::min(profile.target_velocity_rad_s, limit);

    const float sine_peak = profile.sine_mean_rad_s +
                            std::fabs(profile.sine_amplitude_rad_s);
    if (sine_peak > limit && sine_peak > 0.0F) {
      const float scale = limit / sine_peak;
      profile.sine_mean_rad_s *= scale;
      profile.sine_amplitude_rad_s *= scale;
    }

    for (uint8_t point = 0U; point < profile.point_count; ++point) {
      profile.points[point].velocity_rad_s =
          std::min(profile.points[point].velocity_rad_s, limit);
    }
  }
  return true;
}

}  // namespace mm::characterization
