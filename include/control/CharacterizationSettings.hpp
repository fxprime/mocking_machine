#pragma once

#include <algorithm>
#include <cmath>

#include "core/Types.hpp"

namespace mm::characterization {

inline float recommendedAccelerationRadS2(
    const CharacterizationDynamicsResult& measured, const float safety_factor) {
  return safety_factor * std::min(measured.acceleration_forward_rad_s2,
                                  measured.acceleration_reverse_rad_s2);
}

inline float recommendedJerkRadS3(
    const CharacterizationDynamicsResult& measured, const float safety_factor) {
  return safety_factor * std::min(measured.jerk_forward_rad_s3,
                                  measured.jerk_reverse_rad_s3);
}

inline void constrainWaypointAcceleration(VelocityProfileConfiguration& profile,
                                          const float maximum_acceleration_rad_s2) {
  if (profile.kind != ProfileKind::Waypoints || profile.point_count < 2U) return;
  for (uint8_t index = 1U; index < profile.point_count; ++index) {
    const float dt_s = static_cast<float>(
        profile.points[index].time_ms - profile.points[index - 1U].time_ms) * 0.001F;
    profile.points[index].velocity_rad_s = std::min(
        profile.points[index].velocity_rad_s,
        profile.points[index - 1U].velocity_rad_s + maximum_acceleration_rad_s2 * dt_s);
  }
  for (uint8_t index = profile.point_count - 1U; index > 0U; --index) {
    const float dt_s = static_cast<float>(
        profile.points[index].time_ms - profile.points[index - 1U].time_ms) * 0.001F;
    profile.points[index - 1U].velocity_rad_s = std::min(
        profile.points[index - 1U].velocity_rad_s,
        profile.points[index].velocity_rad_s + maximum_acceleration_rad_s2 * dt_s);
  }
}

inline bool applyRecommendedDynamics(
    const CharacterizationDynamicsResult& measured, const float safety_factor,
    const bool apply_acceleration, const bool apply_jerk, MachineSettings& candidate) {
  const float acceleration = recommendedAccelerationRadS2(measured, safety_factor);
  const float jerk = recommendedJerkRadS3(measured, safety_factor);
  if ((apply_acceleration && (!std::isfinite(acceleration) || acceleration <= 0.0F)) ||
      (apply_jerk && (!std::isfinite(jerk) || jerk <= 0.0F))) {
    return false;
  }
  if (apply_acceleration) {
    candidate.safety.max_acceleration_rad_s2 =
        std::min(candidate.safety.max_acceleration_rad_s2, acceleration);
    for (uint8_t index = 0U; index < candidate.profile_count; ++index) {
      constrainWaypointAcceleration(candidate.profiles[index],
                                    candidate.safety.max_acceleration_rad_s2);
    }
  }
  if (apply_jerk) {
    candidate.safety.max_jerk_rad_s3 =
        std::min(candidate.safety.max_jerk_rad_s3, jerk);
  }
  return true;
}

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
