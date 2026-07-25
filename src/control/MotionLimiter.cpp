#include "control/MotionLimiter.hpp"

#include <algorithm>
#include <cmath>

namespace mm {

void MotionLimiter::reset(const float velocity) {
  velocity_rad_s_ = velocity;
  acceleration_rad_s2_ = 0.0F;
  previous_target_velocity_rad_s_ = velocity;
  target_initialized_ = false;
}

float MotionLimiter::update(float target, const float dt_s) {
  if (!(dt_s > 0.0F)) {
    return velocity_rad_s_;
  }
  target = std::clamp(target, -configuration_.max_velocity_rad_s,
                      configuration_.max_velocity_rad_s);
  const float maximum_acceleration =
      configuration_.max_acceleration_rad_s2;
  const float maximum_jerk = configuration_.max_jerk_rad_s3;
  if (!(maximum_acceleration > 0.0F)) {
    acceleration_rad_s2_ = 0.0F;
    previous_target_velocity_rad_s_ = target;
    target_initialized_ = true;
    return velocity_rad_s_;
  }
  if (!configuration_.jerk_limit_enabled) {
    const float previous_velocity = velocity_rad_s_;
    const float maximum_velocity_change =
        maximum_acceleration * dt_s;
    velocity_rad_s_ += std::clamp(
        target - velocity_rad_s_, -maximum_velocity_change,
        maximum_velocity_change);
    acceleration_rad_s2_ =
        (velocity_rad_s_ - previous_velocity) / dt_s;
    previous_target_velocity_rad_s_ = target;
    target_initialized_ = true;
    return velocity_rad_s_;
  }
  if (!(maximum_jerk > 0.0F)) {
    acceleration_rad_s2_ = 0.0F;
    previous_target_velocity_rad_s_ = target;
    target_initialized_ = true;
    return velocity_rad_s_;
  }

  const float target_acceleration = target_initialized_
      ? std::clamp(
            (target - previous_target_velocity_rad_s_) / dt_s,
            -maximum_acceleration, maximum_acceleration)
      : 0.0F;
  previous_target_velocity_rad_s_ = target;
  target_initialized_ = true;

  const float error = target - velocity_rad_s_;
  const float relative_acceleration =
      acceleration_rad_s2_ - target_acceleration;
  const float maximum_acceleration_change = maximum_jerk * dt_s;
  const float capture_error =
      std::max(std::fabs(acceleration_rad_s2_),
               std::fabs(target_acceleration)) *
          dt_s +
      0.5F * maximum_jerk * dt_s * dt_s;
  if (std::fabs(error) <= capture_error &&
      std::fabs(relative_acceleration) <=
          maximum_acceleration_change) {
    velocity_rad_s_ = target;
    acceleration_rad_s2_ = target_acceleration;
    return velocity_rad_s_;
  }

  const float relative_stopping_error =
      relative_acceleration * std::fabs(relative_acceleration) /
      (2.0F * maximum_jerk);
  const float switching_error = error - relative_stopping_error;
  const float acceleration_change =
      switching_error >= 0.0F ? maximum_acceleration_change
                              : -maximum_acceleration_change;
  acceleration_rad_s2_ = std::clamp(
      acceleration_rad_s2_ + acceleration_change,
      -maximum_acceleration, maximum_acceleration);
  velocity_rad_s_ = std::clamp(
      velocity_rad_s_ + acceleration_rad_s2_ * dt_s,
      -configuration_.max_velocity_rad_s,
      configuration_.max_velocity_rad_s);
  return velocity_rad_s_;
}

}  // namespace mm
