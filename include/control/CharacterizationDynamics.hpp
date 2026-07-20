#pragma once

#include <cmath>
#include <cstdint>

#include "control/P2QuantileEstimator.hpp"

namespace mm::characterization {

class DynamicsEstimator {
 public:
  void configure(float cutoff_hz, float quantile) {
    cutoff_hz_ = cutoff_hz;
    filter_dt_s_ = 0.0F;
    filter_alpha_ = 0.0F;
    acceleration_quantile_.configure(quantile);
    jerk_quantile_.configure(quantile);
    reset();
  }

  void reset() {
    initialized_ = false;
    acceleration_initialized_ = false;
    previous_velocity_rad_s_ = 0.0F;
    filtered_acceleration_rad_s2_ = 0.0F;
    previous_filtered_acceleration_rad_s2_ = 0.0F;
    filtered_jerk_rad_s3_ = 0.0F;
    acceleration_quantile_.reset();
    jerk_quantile_.reset();
  }

  void update(float velocity_rad_s, float dt_s) {
    if (!std::isfinite(velocity_rad_s) || !std::isfinite(dt_s) || dt_s <= 0.0F) return;
    if (!initialized_) {
      previous_velocity_rad_s_ = velocity_rad_s;
      initialized_ = true;
      return;
    }
    if (dt_s != filter_dt_s_) {
      constexpr float kTwoPi = 6.2831853071795864769F;
      filter_alpha_ = 1.0F - std::exp(-kTwoPi * cutoff_hz_ * dt_s);
      filter_dt_s_ = dt_s;
    }
    const float raw_acceleration =
        (velocity_rad_s - previous_velocity_rad_s_) / dt_s;
    if (!acceleration_initialized_) {
      filtered_acceleration_rad_s2_ = raw_acceleration;
      previous_filtered_acceleration_rad_s2_ = raw_acceleration;
      previous_velocity_rad_s_ = velocity_rad_s;
      acceleration_quantile_.add(std::fabs(raw_acceleration));
      acceleration_initialized_ = true;
      return;
    }
    filtered_acceleration_rad_s2_ +=
        filter_alpha_ * (raw_acceleration - filtered_acceleration_rad_s2_);
    const float raw_jerk = (filtered_acceleration_rad_s2_ -
                            previous_filtered_acceleration_rad_s2_) / dt_s;
    filtered_jerk_rad_s3_ += filter_alpha_ * (raw_jerk - filtered_jerk_rad_s3_);
    acceleration_quantile_.add(std::fabs(filtered_acceleration_rad_s2_));
    jerk_quantile_.add(std::fabs(filtered_jerk_rad_s3_));
    previous_velocity_rad_s_ = velocity_rad_s;
    previous_filtered_acceleration_rad_s2_ = filtered_acceleration_rad_s2_;
  }

  bool valid() const { return acceleration_quantile_.count() >= 20U; }
  float accelerationRadS2() const { return acceleration_quantile_.value(); }
  float jerkRadS3() const { return jerk_quantile_.value(); }

 private:
  float cutoff_hz_ = 20.0F;
  float filter_dt_s_ = 0.0F;
  float filter_alpha_ = 0.0F;
  bool initialized_ = false;
  bool acceleration_initialized_ = false;
  float previous_velocity_rad_s_ = 0.0F;
  float filtered_acceleration_rad_s2_ = 0.0F;
  float previous_filtered_acceleration_rad_s2_ = 0.0F;
  float filtered_jerk_rad_s3_ = 0.0F;
  P2QuantileEstimator acceleration_quantile_{};
  P2QuantileEstimator jerk_quantile_{};
};

}  // namespace mm::characterization
