#pragma once

#include "core/Types.hpp"

namespace mm {

class MotionLimiter {
 public:
  void configure(const SafetyConfiguration& configuration) { configuration_ = configuration; }
  void reset(float velocity = 0.0F);
  float update(float target_velocity_rad_s, float dt_s);
  float velocity() const { return velocity_rad_s_; }
  float acceleration() const { return acceleration_rad_s2_; }

 private:
  SafetyConfiguration configuration_{};
  float velocity_rad_s_ = 0.0F;
  float acceleration_rad_s2_ = 0.0F;
};

}  // namespace mm

