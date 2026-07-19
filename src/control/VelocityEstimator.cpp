#include "control/VelocityEstimator.hpp"

#include <algorithm>
#include <cmath>

namespace mm {
namespace {
constexpr float kTwoPi = 6.2831853071795864769F;
}

void VelocityEstimator::configure(const EncoderConfiguration& encoder, const float filter_tau_s) {
  encoder_ = encoder;
  filter_tau_s_ = filter_tau_s;
}

void VelocityEstimator::reset(const int64_t count, const uint64_t timestamp_us) {
  previous_count_ = count;
  accumulated_counts_ = 0;
  previous_timestamp_us_ = timestamp_us;
  window_start_us_ = timestamp_us;
  count_velocity_rad_s_ = 0.0F;
  filtered_velocity_rad_s_ = 0.0F;
}

float VelocityEstimator::update(const int64_t count, const uint64_t timestamp_us,
                                const uint64_t last_edge_timestamp_us,
                                const uint32_t edge_period_us,
                                const int8_t edge_direction) {
  if (timestamp_us <= previous_timestamp_us_ || encoder_.counts_per_output_revolution == 0U) {
    return filtered_velocity_rad_s_;
  }
  const float update_dt_s = static_cast<float>(timestamp_us - previous_timestamp_us_) * 1.0e-6F;
  accumulated_counts_ += count - previous_count_;
  previous_count_ = count;
  previous_timestamp_us_ = timestamp_us;

  const uint64_t elapsed_us = timestamp_us - window_start_us_;
  if (std::llabs(accumulated_counts_) >= encoder_.estimator_min_counts ||
      elapsed_us >= encoder_.estimator_max_window_us) {
    const float window_dt_s = static_cast<float>(elapsed_us) * 1.0e-6F;
    const float radians_per_count =
        kTwoPi / static_cast<float>(encoder_.counts_per_output_revolution);
    count_velocity_rad_s_ = static_cast<float>(accumulated_counts_) * radians_per_count /
                            window_dt_s * static_cast<float>(encoder_.direction);
    accumulated_counts_ = 0;
    window_start_us_ = timestamp_us;
  }

  const float radians_per_count =
      kTwoPi / static_cast<float>(encoder_.counts_per_output_revolution);
  float raw = count_velocity_rad_s_;
  const bool edge_is_fresh = last_edge_timestamp_us > 0U &&
      timestamp_us >= last_edge_timestamp_us &&
      timestamp_us - last_edge_timestamp_us <= encoder_.estimator_stale_timeout_us;
  if (edge_is_fresh && edge_period_us > 0U && edge_direction != 0) {
    const float period_velocity = radians_per_count /
        (static_cast<float>(edge_period_us) * 1.0e-6F) *
        static_cast<float>(edge_direction) * static_cast<float>(encoder_.direction);
    const float expected_counts = static_cast<float>(encoder_.estimator_max_window_us) /
                                  static_cast<float>(edge_period_us);
    const float period_weight = std::clamp(
        1.0F - expected_counts / static_cast<float>(encoder_.estimator_min_counts),
        0.0F, 1.0F);
    raw += period_weight * (period_velocity - raw);
  } else if (last_edge_timestamp_us > 0U && !edge_is_fresh) {
    raw = 0.0F;
    count_velocity_rad_s_ = 0.0F;
  }
  const float alpha = filter_tau_s_ <= 0.0F
                          ? 1.0F
                          : update_dt_s / (filter_tau_s_ + update_dt_s);
  filtered_velocity_rad_s_ += alpha * (raw - filtered_velocity_rad_s_);
  return filtered_velocity_rad_s_;
}

}  // namespace mm
