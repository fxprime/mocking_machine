#include "control/VelocityEstimator.hpp"

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
  previous_timestamp_us_ = timestamp_us;
  filtered_velocity_rad_s_ = 0.0F;
}

float VelocityEstimator::update(const int64_t count, const uint64_t timestamp_us) {
  if (timestamp_us <= previous_timestamp_us_ || encoder_.counts_per_output_revolution == 0U) {
    return filtered_velocity_rad_s_;
  }
  const float dt_s = static_cast<float>(timestamp_us - previous_timestamp_us_) * 1.0e-6F;
  const int64_t delta_count = count - previous_count_;
  previous_count_ = count;
  previous_timestamp_us_ = timestamp_us;
  const float radians_per_count =
      kTwoPi / static_cast<float>(encoder_.counts_per_output_revolution);
  const float raw = static_cast<float>(delta_count) * radians_per_count / dt_s *
                    static_cast<float>(encoder_.direction);
  const float alpha = filter_tau_s_ <= 0.0F
                          ? 1.0F
                          : dt_s / (filter_tau_s_ + dt_s);
  filtered_velocity_rad_s_ += alpha * (raw - filtered_velocity_rad_s_);
  return filtered_velocity_rad_s_;
}

}  // namespace mm
