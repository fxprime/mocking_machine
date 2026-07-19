#pragma once

#include "core/Types.hpp"

namespace mm {

class IncrementalVelocityController {
 public:
  void configure(const ControlConfiguration& configuration);
  void reset(float output = 0.0F);
  float update(float desired_rad_s, float measured_rad_s, float dt_s);
  float output() const { return output_; }
  float error() const { return error_1_; }
  float proportionalTerm() const { return proportional_term_; }
  float integralTerm() const { return integral_term_; }
  float derivativeTerm() const { return derivative_term_; }

 private:
  ControlConfiguration configuration_{};
  float output_ = 0.0F;
  float error_1_ = 0.0F;
  float error_2_ = 0.0F;
  float proportional_term_ = 0.0F;
  float integral_term_ = 0.0F;
  float derivative_term_ = 0.0F;
  bool initialized_ = false;
};

}  // namespace mm
