#include "control/MotionLimiter.hpp"

#include <algorithm>
#include <cmath>

namespace mm {

void MotionLimiter::reset(const float velocity) {
  velocity_rad_s_ = velocity;
  acceleration_rad_s2_ = 0.0F;
}

float MotionLimiter::update(float target, const float dt_s) {
  if (!(dt_s > 0.0F)) {
    return velocity_rad_s_;
  }
  target = std::clamp(target, -configuration_.max_velocity_rad_s,
                      configuration_.max_velocity_rad_s);
  const float error = target - velocity_rad_s_;
  const float desired_acceleration = std::clamp(
      error / dt_s, -configuration_.max_acceleration_rad_s2,
      configuration_.max_acceleration_rad_s2);
  const float maximum_acceleration_change = configuration_.max_jerk_rad_s3 * dt_s;
  acceleration_rad_s2_ += std::clamp(desired_acceleration - acceleration_rad_s2_,
                                    -maximum_acceleration_change,
                                    maximum_acceleration_change);

  const float step = acceleration_rad_s2_ * dt_s;
  if (std::fabs(step) >= std::fabs(error)) {
    velocity_rad_s_ = target;
    acceleration_rad_s2_ = 0.0F;
  } else {
    velocity_rad_s_ += step;
  }
  return velocity_rad_s_;
}

}  // namespace mm

