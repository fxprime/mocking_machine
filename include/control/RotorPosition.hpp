#pragma once

#include <cstdint>

namespace mm {

__attribute__((always_inline)) inline bool shouldAcceptZeroIndexRise(
    const uint64_t timestamp_us, const uint64_t last_accepted_timestamp_us,
    const uint32_t minimum_interval_us) {
  return last_accepted_timestamp_us == 0U || timestamp_us < last_accepted_timestamp_us ||
         timestamp_us - last_accepted_timestamp_us >= minimum_interval_us;
}

inline float calculateRotorPositionDegrees(const int64_t encoder_count,
                                           const int64_t zero_encoder_count,
                                           const uint32_t counts_per_revolution,
                                           const int8_t encoder_direction) {
  if (counts_per_revolution == 0U) {
    return 0.0F;
  }
  const int64_t direction = encoder_direction >= 0 ? 1 : -1;
  const int64_t revolution_counts = static_cast<int64_t>(counts_per_revolution);
  const int64_t current_modulo = encoder_count % revolution_counts;
  const int64_t zero_modulo = zero_encoder_count % revolution_counts;
  int64_t position_counts = ((current_modulo - zero_modulo) * direction) %
                            revolution_counts;
  if (position_counts < 0) {
    position_counts += revolution_counts;
  }
  return static_cast<float>(position_counts) * 360.0F /
         static_cast<float>(counts_per_revolution);
}

}  // namespace mm
