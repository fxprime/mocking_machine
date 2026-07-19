#include "control/IncrementalVelocityController.hpp"

#include <algorithm>
#include <cmath>

namespace mm {

void IncrementalVelocityController::configure(const ControlConfiguration& configuration) {
  configuration_ = configuration;
  reset();
}

void IncrementalVelocityController::reset(const float output) {
  output_ = std::clamp(output, configuration_.output_min, configuration_.output_max);
  error_1_ = 0.0F;
  error_2_ = 0.0F;
  proportional_term_ = 0.0F;
  integral_term_ = 0.0F;
  derivative_term_ = 0.0F;
  initialized_ = false;
}

float IncrementalVelocityController::update(const float desired_rad_s,
                                            const float measured_rad_s,
                                            const float dt_s) {
  if (!(dt_s > 0.0F) || !std::isfinite(dt_s)) {
    proportional_term_ = 0.0F;
    integral_term_ = 0.0F;
    derivative_term_ = 0.0F;
    return output_;
  }

  float error = desired_rad_s - measured_rad_s;
  if (std::fabs(error) < configuration_.error_deadband_rad_s) {
    error = 0.0F;
  }
  if (!initialized_) {
    error_1_ = error;
    error_2_ = error;
    initialized_ = true;
  }

  proportional_term_ = configuration_.kp * (error - error_1_);
  integral_term_ = configuration_.ki * dt_s * error;
  derivative_term_ =
      configuration_.kd * (error - 2.0F * error_1_ + error_2_) / dt_s;
  output_ = std::clamp(output_ + proportional_term_ + integral_term_ + derivative_term_,
                       configuration_.output_min, configuration_.output_max);

  error_2_ = error_1_;
  error_1_ = error;
  return output_;
}

}  // namespace mm
