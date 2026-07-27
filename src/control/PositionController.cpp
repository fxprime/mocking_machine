#include "control/PositionController.hpp"

#include <algorithm>
#include <cmath>

namespace mm {
namespace {

constexpr float kDegreesToRadians = 0.01745329251994329577F;

}  // namespace

void PositionController::reset() {
  integral_error_rad_s_ = 0.0F;
  previous_error_rad_ = 0.0F;
  error_deg_ = 0.0F;
  initialized_ = false;
}

float PositionController::shortestErrorDegrees(
    const float target_position_deg, const float measured_position_deg) {
  float error = std::fmod(target_position_deg - measured_position_deg, 360.0F);
  if (error >= 180.0F) {
    error -= 360.0F;
  } else if (error < -180.0F) {
    error += 360.0F;
  }
  return error;
}

float PositionController::applyVelocityDeadband(
    const float velocity_rad_s, const float position_error_deg,
    const PositionControlConfiguration& configuration) {
  if (std::fabs(position_error_deg) <= configuration.tolerance_deg) {
    return 0.0F;
  }
  if (velocity_rad_s > 0.0F) {
    return std::max(velocity_rad_s,
                    configuration.minimum_velocity_forward_rad_s);
  }
  if (velocity_rad_s < 0.0F) {
    return std::min(velocity_rad_s,
                    -configuration.minimum_velocity_reverse_rad_s);
  }
  return position_error_deg > 0.0F
             ? configuration.minimum_velocity_forward_rad_s
             : -configuration.minimum_velocity_reverse_rad_s;
}

float PositionController::update(const float target_position_deg,
                                 const float measured_position_deg,
                                 const float dt_s) {
  error_deg_ =
      shortestErrorDegrees(target_position_deg, measured_position_deg);
  if (std::fabs(error_deg_) <= configuration_.tolerance_deg) {
    integral_error_rad_s_ = 0.0F;
    previous_error_rad_ = 0.0F;
    initialized_ = false;
    return 0.0F;
  }
  const float error_rad = error_deg_ * kDegreesToRadians;
  if (!(dt_s > 0.0F)) {
    const float bounded =
        std::clamp(configuration_.kp * error_rad,
                   -configuration_.max_velocity_rad_s,
                   configuration_.max_velocity_rad_s);
    return applyVelocityDeadband(bounded, error_deg_, configuration_);
  }

  float derivative = 0.0F;
  if (initialized_) {
    const float error_delta_deg =
        shortestErrorDegrees(error_deg_, previous_error_rad_ / kDegreesToRadians);
    derivative = error_delta_deg * kDegreesToRadians / dt_s;
  }
  previous_error_rad_ = error_rad;
  initialized_ = true;

  integral_error_rad_s_ += error_rad * dt_s;
  if (configuration_.ki > 0.0F) {
    const float integral_limit =
        configuration_.max_velocity_rad_s / configuration_.ki;
    integral_error_rad_s_ =
        std::clamp(integral_error_rad_s_, -integral_limit, integral_limit);
  } else {
    integral_error_rad_s_ = 0.0F;
  }

  const float output = configuration_.kp * error_rad +
                       configuration_.ki * integral_error_rad_s_ +
                       configuration_.kd * derivative;
  const float bounded =
      std::clamp(output, -configuration_.max_velocity_rad_s,
                 configuration_.max_velocity_rad_s);
  return applyVelocityDeadband(bounded, error_deg_, configuration_);
}

}  // namespace mm
