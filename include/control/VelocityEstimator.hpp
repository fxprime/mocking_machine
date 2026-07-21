#pragma once

#include <cstdint>

#include "core/Types.hpp"

namespace mm {

class VelocityEstimator {
 public:
  void configure(const EncoderConfiguration& encoder, float filter_tau_s);
  void configure(const EncoderConfiguration& encoder, float filter_tau_s,
                 const MotorModelConfiguration& model);
  void configure(const EncoderConfiguration& encoder, float filter_tau_s,
                 const MotorModelConfiguration& model, VelocityEstimatorMethod method,
                 float maximum_acceleration_rad_s2,
                 uint32_t acceleration_window_samples = 5U);
  void reset(int64_t count, uint64_t timestamp_us);
  float update(int64_t count, uint64_t timestamp_us);
  float update(int64_t count, uint64_t timestamp_us, float applied_duty,
               bool disarmed = false);
  float velocityRadPerSecond() const { return filtered_velocity_rad_s_; }
  bool usingMotorModel() const { return active_method_ == VelocityEstimatorMethod::Kalman; }
  VelocityEstimatorMethod method() const { return active_method_; }

 private:
  EncoderConfiguration encoder_{};
  float filter_tau_s_ = 0.025F;
  int64_t previous_count_ = 0;
  uint64_t previous_timestamp_us_ = 0;
  float filtered_velocity_rad_s_ = 0.0F;
  MotorModelConfiguration model_{};
  bool model_enabled_ = false;
  VelocityEstimatorMethod active_method_ = VelocityEstimatorMethod::LowPass;
  float maximum_acceleration_rad_s2_ = 10000.0F;
  int64_t measurement_count_ = 0;
  uint64_t measurement_timestamp_us_ = 0;
  float disturbance_rad_s2_ = 0.0F;
  float covariance_velocity_ = 100.0F;
  float covariance_cross_ = 0.0F;
  float covariance_disturbance_ = 100.0F;
  float window_acceleration_rad_s2_ = 0.0F;
  float window_velocity_origin_rad_s_ = 0.0F;
  static constexpr uint8_t kMaximumAccelerationWindowSamples = 32U;
  uint8_t acceleration_window_samples_ = 5U;
  uint8_t acceleration_window_head_ = 0U;
  uint8_t acceleration_window_count_ = 0U;
  float acceleration_velocity_window_[kMaximumAccelerationWindowSamples]{};
  uint64_t acceleration_timestamp_window_us_[kMaximumAccelerationWindowSamples]{};
};

}  // namespace mm
