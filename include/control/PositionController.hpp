#pragma once

#include "core/Types.hpp"

namespace mm {

class PositionController {
 public:
  void configure(const PositionControlConfiguration& configuration) {
    configuration_ = configuration;
  }
  void reset();
  float update(float target_position_deg, float measured_position_deg,
               float dt_s);
  float errorDegrees() const { return error_deg_; }

  static float shortestErrorDegrees(float target_position_deg,
                                    float measured_position_deg);
  static float applyVelocityDeadband(
      float velocity_rad_s, float position_error_deg,
      const PositionControlConfiguration& configuration);

 private:
  PositionControlConfiguration configuration_{};
  float integral_error_rad_s_ = 0.0F;
  float previous_error_rad_ = 0.0F;
  float error_deg_ = 0.0F;
  bool initialized_ = false;
};

}  // namespace mm
