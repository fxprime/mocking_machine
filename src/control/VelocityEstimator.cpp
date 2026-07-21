#include "control/VelocityEstimator.hpp"

#include <algorithm>
#include <cmath>

namespace mm {
namespace {
constexpr float kTwoPi = 6.2831853071795864769F;
}

void VelocityEstimator::configure(const EncoderConfiguration& encoder, const float filter_tau_s) {
  configure(encoder, filter_tau_s, MotorModelConfiguration{},
            VelocityEstimatorMethod::LowPass, 10000.0F, 5U);
}

void VelocityEstimator::configure(const EncoderConfiguration& encoder, const float filter_tau_s,
                                  const MotorModelConfiguration& model) {
  configure(encoder, filter_tau_s, model, VelocityEstimatorMethod::Kalman, 10000.0F, 5U);
}

void VelocityEstimator::configure(const EncoderConfiguration& encoder, const float filter_tau_s,
                                  const MotorModelConfiguration& model,
                                  const VelocityEstimatorMethod method,
                                  const float maximum_acceleration_rad_s2,
                                  const uint32_t acceleration_window_samples) {
  encoder_ = encoder;
  filter_tau_s_ = filter_tau_s;
  model_ = model;
  model_enabled_ = std::isfinite(model.velocity_gain_forward_rad_s_per_duty) &&
      std::isfinite(model.velocity_gain_reverse_rad_s_per_duty) &&
      std::isfinite(model.time_constant_forward_s) &&
      std::isfinite(model.time_constant_reverse_s) &&
      model.velocity_gain_forward_rad_s_per_duty > 0.0F &&
      model.velocity_gain_reverse_rad_s_per_duty > 0.0F &&
      model.time_constant_forward_s > 0.0F && model.time_constant_reverse_s > 0.0F;
  active_method_ = method == VelocityEstimatorMethod::Kalman && !model_enabled_
      ? VelocityEstimatorMethod::LowPass
      : method;
  maximum_acceleration_rad_s2_ = maximum_acceleration_rad_s2;
  acceleration_window_samples_ = static_cast<uint8_t>(std::clamp<uint32_t>(
      acceleration_window_samples, 2U, kMaximumAccelerationWindowSamples));
}

void VelocityEstimator::reset(const int64_t count, const uint64_t timestamp_us) {
  previous_count_ = count;
  previous_timestamp_us_ = timestamp_us;
  measurement_count_ = count;
  measurement_timestamp_us_ = timestamp_us;
  filtered_velocity_rad_s_ = 0.0F;
  disturbance_rad_s2_ = 0.0F;
  covariance_velocity_ = 100.0F;
  covariance_cross_ = 0.0F;
  covariance_disturbance_ = 100.0F;
  window_acceleration_rad_s2_ = 0.0F;
  window_velocity_origin_rad_s_ = 0.0F;
  acceleration_window_head_ = 0U;
  acceleration_window_count_ = 0U;
}

float VelocityEstimator::update(const int64_t count, const uint64_t timestamp_us) {
  return update(count, timestamp_us, 0.0F, false);
}

float VelocityEstimator::update(const int64_t count, const uint64_t timestamp_us,
                                const float applied_duty, const bool disarmed) {
  if (timestamp_us <= previous_timestamp_us_ || encoder_.counts_per_output_revolution == 0U) {
    return filtered_velocity_rad_s_;
  }
  const float dt_s = static_cast<float>(timestamp_us - previous_timestamp_us_) * 1.0e-6F;
  const int64_t delta_count = count - previous_count_;
  previous_count_ = count;
  previous_timestamp_us_ = timestamp_us;
  const float radians_per_count =
      kTwoPi / static_cast<float>(encoder_.counts_per_output_revolution);
  if (active_method_ == VelocityEstimatorMethod::Kalman && disarmed) {
    const float raw = static_cast<float>(delta_count) * radians_per_count / dt_s *
                      static_cast<float>(encoder_.direction);
    const float alpha = filter_tau_s_ <= 0.0F
        ? 1.0F : dt_s / (filter_tau_s_ + dt_s);
    filtered_velocity_rad_s_ += alpha * (raw - filtered_velocity_rad_s_);
    measurement_count_ = count;
    measurement_timestamp_us_ = timestamp_us;
    disturbance_rad_s2_ = 0.0F;
    covariance_velocity_ = 100.0F;
    covariance_cross_ = 0.0F;
    covariance_disturbance_ = 100.0F;
    return filtered_velocity_rad_s_;
  }
  if (active_method_ == VelocityEstimatorMethod::WindowedAccelerationPrediction) {
    const int64_t window_count_delta = count - measurement_count_;
    const uint64_t window_us = timestamp_us - measurement_timestamp_us_;
    filtered_velocity_rad_s_ = window_velocity_origin_rad_s_ +
        window_acceleration_rad_s2_ * static_cast<float>(window_us) * 1.0e-6F;
    const uint64_t count_magnitude = window_count_delta < 0
        ? static_cast<uint64_t>(-(window_count_delta + 1)) + 1U
        : static_cast<uint64_t>(window_count_delta);
    const bool enough_counts = count_magnitude >= encoder_.estimator_min_counts;
    const bool maximum_window = window_count_delta != 0 &&
        window_us >= encoder_.estimator_max_window_us;
    const bool stale = window_us >= encoder_.estimator_stale_timeout_us;
    if (window_us > 0U && (enough_counts || maximum_window || stale)) {
      const float measurement_dt_s = static_cast<float>(window_us) * 1.0e-6F;
      const float measured_velocity = static_cast<float>(window_count_delta) *
          radians_per_count / measurement_dt_s * static_cast<float>(encoder_.direction);
      if (stale && window_count_delta == 0) {
        filtered_velocity_rad_s_ = 0.0F;
        window_velocity_origin_rad_s_ = 0.0F;
        window_acceleration_rad_s2_ = 0.0F;
        acceleration_window_head_ = 0U;
        acceleration_window_count_ = 0U;
      } else {
        acceleration_velocity_window_[acceleration_window_head_] = measured_velocity;
        acceleration_timestamp_window_us_[acceleration_window_head_] = timestamp_us;
        acceleration_window_head_ = static_cast<uint8_t>(
            (acceleration_window_head_ + 1U) % acceleration_window_samples_);
        if (acceleration_window_count_ < acceleration_window_samples_) {
          ++acceleration_window_count_;
        }
        const uint8_t oldest = acceleration_window_count_ == acceleration_window_samples_
            ? acceleration_window_head_ : 0U;
        float velocity_sum = 0.0F;
        float acceleration_sum = 0.0F;
        uint8_t acceleration_count = 0U;
        for (uint8_t sample = 0U; sample < acceleration_window_count_; ++sample) {
          const uint8_t index = static_cast<uint8_t>(
              (oldest + sample) % acceleration_window_samples_);
          velocity_sum += acceleration_velocity_window_[index];
          if (sample > 0U) {
            const uint8_t previous_index = static_cast<uint8_t>(
                (oldest + sample - 1U) % acceleration_window_samples_);
            const uint64_t pair_us = acceleration_timestamp_window_us_[index] -
                acceleration_timestamp_window_us_[previous_index];
            if (pair_us > 0U) {
              acceleration_sum +=
                  (acceleration_velocity_window_[index] -
                   acceleration_velocity_window_[previous_index]) /
                  (static_cast<float>(pair_us) * 1.0e-6F);
              ++acceleration_count;
            }
          }
        }
        window_velocity_origin_rad_s_ =
            velocity_sum / static_cast<float>(acceleration_window_count_);
        window_acceleration_rad_s2_ = acceleration_count == 0U
            ? 0.0F
            : std::clamp(acceleration_sum / static_cast<float>(acceleration_count),
                         -maximum_acceleration_rad_s2_, maximum_acceleration_rad_s2_);
        filtered_velocity_rad_s_ = window_velocity_origin_rad_s_;
      }
      measurement_count_ = count;
      measurement_timestamp_us_ = timestamp_us;
    }
    return filtered_velocity_rad_s_;
  }
  if (active_method_ == VelocityEstimatorMethod::Kalman) {
    const bool reverse = applied_duty < 0.0F ||
        (applied_duty == 0.0F && filtered_velocity_rad_s_ < 0.0F);
    const float time_constant_s = reverse ? model_.time_constant_reverse_s
                                          : model_.time_constant_forward_s;
    const float velocity_gain = reverse
        ? model_.velocity_gain_reverse_rad_s_per_duty
        : model_.velocity_gain_forward_rad_s_per_duty;
    const float a = std::exp(-dt_s / time_constant_s);
    const float predicted_velocity = a * filtered_velocity_rad_s_ +
        velocity_gain * (1.0F - a) * applied_duty + dt_s * disturbance_rad_s2_;

    const float old_velocity_covariance = covariance_velocity_;
    const float old_cross_covariance = covariance_cross_;
    covariance_velocity_ = a * a * old_velocity_covariance +
        2.0F * a * dt_s * old_cross_covariance +
        dt_s * dt_s * covariance_disturbance_ +
        model_.velocity_process_noise_rad_s2 * dt_s;
    covariance_cross_ = a * old_cross_covariance + dt_s * covariance_disturbance_;
    covariance_disturbance_ += model_.disturbance_process_noise_rad_s3 * dt_s;
    filtered_velocity_rad_s_ = predicted_velocity;

    const int64_t window_count_delta = count - measurement_count_;
    const uint64_t window_us = timestamp_us - measurement_timestamp_us_;
    const uint64_t count_magnitude = window_count_delta < 0
        ? static_cast<uint64_t>(-(window_count_delta + 1)) + 1U
        : static_cast<uint64_t>(window_count_delta);
    const bool enough_counts = count_magnitude >= encoder_.estimator_min_counts;
    const bool maximum_window = window_count_delta != 0 &&
        window_us >= encoder_.estimator_max_window_us;
    const bool stale = window_us >= encoder_.estimator_stale_timeout_us;
    if (window_us > 0U && (enough_counts || maximum_window || stale)) {
      const float measurement_dt_s = static_cast<float>(window_us) * 1.0e-6F;
      const float measured_velocity = static_cast<float>(window_count_delta) *
          radians_per_count / measurement_dt_s * static_cast<float>(encoder_.direction);
      const float velocity_per_noise_count = radians_per_count /
          measurement_dt_s * model_.encoder_measurement_noise_counts;
      const float measurement_variance = velocity_per_noise_count *
          velocity_per_noise_count + 1.0e-6F;
      const float innovation_variance = covariance_velocity_ + measurement_variance;
      if (std::isfinite(innovation_variance) && innovation_variance > 0.0F) {
        const float prior_cross = covariance_cross_;
        const float velocity_gain_k = covariance_velocity_ / innovation_variance;
        const float disturbance_gain_k = prior_cross / innovation_variance;
        const float innovation = measured_velocity - filtered_velocity_rad_s_;
        filtered_velocity_rad_s_ += velocity_gain_k * innovation;
        disturbance_rad_s2_ += disturbance_gain_k * innovation;
        covariance_velocity_ *= 1.0F - velocity_gain_k;
        covariance_cross_ = prior_cross * (1.0F - velocity_gain_k);
        covariance_disturbance_ -= disturbance_gain_k * prior_cross;
      }
      measurement_count_ = count;
      measurement_timestamp_us_ = timestamp_us;
    }
    if (!std::isfinite(filtered_velocity_rad_s_) ||
        !std::isfinite(disturbance_rad_s2_)) {
      reset(count, timestamp_us);
    }
    return filtered_velocity_rad_s_;
  }
  const float raw = static_cast<float>(delta_count) * radians_per_count / dt_s *
                    static_cast<float>(encoder_.direction);
  const float alpha = filter_tau_s_ <= 0.0F
                          ? 1.0F
                          : dt_s / (filter_tau_s_ + dt_s);
  filtered_velocity_rad_s_ += alpha * (raw - filtered_velocity_rad_s_);
  return filtered_velocity_rad_s_;
}

}  // namespace mm
