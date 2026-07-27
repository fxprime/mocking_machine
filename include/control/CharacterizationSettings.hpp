#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

#include "core/Types.hpp"

namespace mm::characterization {

class BreakawayTrialAccumulator {
 public:
  void reset() {
    count_ = 0U;
    maximum_duty_ = 0.0F;
  }

  bool add(const float duty) {
    if (!std::isfinite(duty) || duty < 0.0F ||
        count_ == std::numeric_limits<uint8_t>::max()) {
      return false;
    }
    maximum_duty_ = std::max(maximum_duty_, duty);
    ++count_;
    return true;
  }

  bool complete(const uint8_t required_trials) const {
    return required_trials > 0U && count_ >= required_trials;
  }

  uint8_t count() const { return count_; }
  float maximumDuty() const { return maximum_duty_; }

 private:
  uint8_t count_ = 0U;
  float maximum_duty_ = 0.0F;
};

class FullDutyTrialAccumulator {
 public:
  void reset() {
    count_ = 0U;
    maximum_velocity_rad_s_ = 0.0F;
    maximum_acceleration_rad_s2_ = 0.0F;
    maximum_jerk_rad_s3_ = 0.0F;
    model_gain_rad_s_per_duty_ = 0.0F;
    model_time_constant_s_ = 0.0F;
  }

  bool add(const float peak_velocity_rad_s,
           const float acceleration_rad_s2, const float jerk_rad_s3,
           const float model_gain_rad_s_per_duty,
           const float model_time_constant_s) {
    if (!std::isfinite(peak_velocity_rad_s) ||
        !std::isfinite(acceleration_rad_s2) ||
        !std::isfinite(jerk_rad_s3) ||
        !std::isfinite(model_gain_rad_s_per_duty) ||
        !std::isfinite(model_time_constant_s) ||
        peak_velocity_rad_s <= 0.0F || acceleration_rad_s2 < 0.0F ||
        jerk_rad_s3 < 0.0F || model_gain_rad_s_per_duty <= 0.0F ||
        model_time_constant_s <= 0.0F ||
        count_ == std::numeric_limits<uint8_t>::max()) {
      return false;
    }
    if (count_ == 0U ||
        peak_velocity_rad_s > maximum_velocity_rad_s_) {
      maximum_velocity_rad_s_ = peak_velocity_rad_s;
      model_gain_rad_s_per_duty_ = model_gain_rad_s_per_duty;
      model_time_constant_s_ = model_time_constant_s;
    }
    maximum_acceleration_rad_s2_ = std::max(
        maximum_acceleration_rad_s2_, acceleration_rad_s2);
    maximum_jerk_rad_s3_ =
        std::max(maximum_jerk_rad_s3_, jerk_rad_s3);
    ++count_;
    return true;
  }

  bool complete(const uint8_t required_trials) const {
    return required_trials > 0U && count_ >= required_trials;
  }

  uint8_t count() const { return count_; }
  float maximumVelocityRadS() const {
    return maximum_velocity_rad_s_;
  }
  float maximumAccelerationRadS2() const {
    return maximum_acceleration_rad_s2_;
  }
  float maximumJerkRadS3() const { return maximum_jerk_rad_s3_; }
  float modelGainRadSPerDuty() const {
    return model_gain_rad_s_per_duty_;
  }
  float modelTimeConstantS() const {
    return model_time_constant_s_;
  }

 private:
  uint8_t count_ = 0U;
  float maximum_velocity_rad_s_ = 0.0F;
  float maximum_acceleration_rad_s2_ = 0.0F;
  float maximum_jerk_rad_s3_ = 0.0F;
  float model_gain_rad_s_per_duty_ = 0.0F;
  float model_time_constant_s_ = 0.0F;
};

inline bool breakawayVelocityMatchesDirection(
    const float velocity_rad_s, const int8_t commanded_direction,
    const float motion_threshold_rad_s) {
  const int8_t normalized_direction = commanded_direction >= 0 ? 1 : -1;
  return velocity_rad_s * static_cast<float>(normalized_direction) >=
      motion_threshold_rad_s;
}

inline uint64_t breakawayDirectedTravelTicks(
    const int64_t start_count, const int64_t current_count,
    const int8_t commanded_direction, const int8_t encoder_direction) {
  const int8_t expected_raw_direction =
      (commanded_direction >= 0 ? 1 : -1) *
      (encoder_direction >= 0 ? 1 : -1);
  if ((expected_raw_direction > 0 && current_count < start_count) ||
      (expected_raw_direction < 0 && current_count > start_count)) {
    return 0U;
  }
  return current_count >= start_count
      ? static_cast<uint64_t>(current_count) -
            static_cast<uint64_t>(start_count)
      : static_cast<uint64_t>(start_count) -
            static_cast<uint64_t>(current_count);
}

inline bool breakawayTravelComplete(
    const int64_t start_count, const int64_t current_count,
    const int8_t commanded_direction, const EncoderConfiguration& encoder) {
  return encoder.counts_per_output_revolution > 0U &&
      breakawayDirectedTravelTicks(
          start_count, current_count, commanded_direction,
          encoder.direction) >= encoder.counts_per_output_revolution;
}

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
  candidate.position_control.max_velocity_rad_s =
      std::min(current.position_control.max_velocity_rad_s,
               candidate.safety.max_velocity_rad_s);
  candidate.position_control.settle_velocity_rad_s =
      std::min(current.position_control.settle_velocity_rad_s,
               candidate.position_control.max_velocity_rad_s);
  candidate.position_control.minimum_velocity_forward_rad_s =
      std::min(current.position_control.minimum_velocity_forward_rad_s,
               candidate.position_control.max_velocity_rad_s);
  candidate.position_control.minimum_velocity_reverse_rad_s =
      std::min(current.position_control.minimum_velocity_reverse_rad_s,
               candidate.position_control.max_velocity_rad_s);

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
