#include "profile/VelocityProfile.hpp"

#include <algorithm>
#include <cmath>

namespace mm {
namespace {
constexpr float kTwoPi = 6.2831853071795864769F;
}

void VelocityProfile::select(const VelocityProfileConfiguration* configuration,
                             const uint64_t start_us) {
  configuration_ = configuration;
  start_us_ = start_us;
}

void VelocityProfile::stop() { configuration_ = nullptr; }

bool VelocityProfile::finished(const uint64_t timestamp_us) const {
  return configuration_ != nullptr && configuration_->duration_ms != 0U &&
      timestamp_us >= start_us_ + static_cast<uint64_t>(configuration_->duration_ms) * 1000ULL;
}

float VelocityProfile::target(const uint64_t timestamp_us) const {
  if (configuration_ == nullptr || timestamp_us < start_us_) {
    return 0.0F;
  }
  const uint64_t elapsed_us = timestamp_us - start_us_;
  const float elapsed_s = static_cast<float>(elapsed_us) * 1.0e-6F;
  if (configuration_->duration_ms != 0U && elapsed_us >=
      static_cast<uint64_t>(configuration_->duration_ms) * 1000ULL) {
    return 0.0F;
  }

  switch (configuration_->kind) {
    case ProfileKind::Ramp:
      return configuration_->target_velocity_rad_s;
    case ProfileKind::Sine:
      return configuration_->sine_mean_rad_s + configuration_->sine_amplitude_rad_s *
             std::sin(kTwoPi * configuration_->sine_frequency_hz * elapsed_s);
    case ProfileKind::Waypoints:
      break;
  }

  const uint8_t count = std::min<uint8_t>(configuration_->point_count,
                                          static_cast<uint8_t>(kMaximumProfilePoints));
  if (count == 0U) {
    return 0.0F;
  }
  const uint32_t elapsed_ms = static_cast<uint32_t>(elapsed_us / 1000ULL);
  if (elapsed_ms <= configuration_->points[0].time_ms) {
    return configuration_->points[0].velocity_rad_s;
  }
  for (uint8_t index = 1; index < count; ++index) {
    const auto& previous = configuration_->points[index - 1U];
    const auto& next = configuration_->points[index];
    if (elapsed_ms <= next.time_ms) {
      const uint32_t span_ms = next.time_ms - previous.time_ms;
      if (span_ms == 0U) {
        return next.velocity_rad_s;
      }
      const float blend = static_cast<float>(elapsed_ms - previous.time_ms) /
                          static_cast<float>(span_ms);
      return previous.velocity_rad_s + blend *
             (next.velocity_rad_s - previous.velocity_rad_s);
    }
  }
  return configuration_->points[count - 1U].velocity_rad_s;
}

}  // namespace mm
