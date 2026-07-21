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

// Allocation-free least-squares identification of v[k+1] = a*v[k] + b*u[k].
// Characterization feeds velocity and applied-duty magnitudes from one direction.
class FirstOrderMotorIdentifier {
 public:
  void reset() {
    initialized_ = false;
    count_ = 0U;
    previous_velocity_ = 0.0F;
    previous_duty_ = 0.0F;
    sum_dt_s_ = 0.0F;
    mean_velocity_ = 0.0F;
    mean_next_velocity_ = 0.0F;
    mean_duty_ = 0.0F;
    velocity_variance_sum_ = 0.0F;
    velocity_cross_sum_ = 0.0F;
  }

  void update(float velocity_rad_s, float applied_duty, float dt_s) {
    velocity_rad_s = std::fabs(velocity_rad_s);
    applied_duty = std::fabs(applied_duty);
    if (!std::isfinite(velocity_rad_s) || !std::isfinite(applied_duty) ||
        !std::isfinite(dt_s) || dt_s <= 0.0F || applied_duty < 0.01F) {
      return;
    }
    if (!initialized_) {
      previous_velocity_ = velocity_rad_s;
      previous_duty_ = applied_duty;
      initialized_ = true;
      return;
    }
    ++count_;
    const float inverse_count = 1.0F / static_cast<float>(count_);
    const float velocity_delta = previous_velocity_ - mean_velocity_;
    mean_velocity_ += velocity_delta * inverse_count;
    const float next_velocity_delta = velocity_rad_s - mean_next_velocity_;
    mean_next_velocity_ += next_velocity_delta * inverse_count;
    velocity_variance_sum_ += velocity_delta * (previous_velocity_ - mean_velocity_);
    velocity_cross_sum_ += velocity_delta * (velocity_rad_s - mean_next_velocity_);
    mean_duty_ += (previous_duty_ - mean_duty_) * inverse_count;
    sum_dt_s_ += dt_s;
    previous_velocity_ = velocity_rad_s;
    previous_duty_ = applied_duty;
  }

  bool result(float& velocity_gain_rad_s_per_duty, float& time_constant_s) const {
    if (count_ < 20U || sum_dt_s_ <= 0.0F) return false;
    if (!std::isfinite(velocity_variance_sum_) || velocity_variance_sum_ <= 1.0e-6F ||
        mean_duty_ < 0.01F) {
      return false;
    }
    const float a = velocity_cross_sum_ / velocity_variance_sum_;
    const float intercept = mean_next_velocity_ - a * mean_velocity_;
    const float b = intercept / mean_duty_;
    const float dt_s = sum_dt_s_ / static_cast<float>(count_);
    if (!std::isfinite(a) || !std::isfinite(b) || a <= 0.0F || a >= 0.99999F ||
        b <= 0.0F) {
      return false;
    }
    time_constant_s = -dt_s / std::log(a);
    velocity_gain_rad_s_per_duty = b / (1.0F - a);
    return std::isfinite(time_constant_s) && std::isfinite(velocity_gain_rad_s_per_duty) &&
        time_constant_s >= 0.005F && time_constant_s <= 5.0F &&
        velocity_gain_rad_s_per_duty >= 1.0F && velocity_gain_rad_s_per_duty <= 1000.0F;
  }

 private:
  bool initialized_ = false;
  uint32_t count_ = 0U;
  float previous_velocity_ = 0.0F;
  float previous_duty_ = 0.0F;
  float sum_dt_s_ = 0.0F;
  float mean_velocity_ = 0.0F;
  float mean_next_velocity_ = 0.0F;
  float mean_duty_ = 0.0F;
  float velocity_variance_sum_ = 0.0F;
  float velocity_cross_sum_ = 0.0F;
};

}  // namespace mm::characterization
