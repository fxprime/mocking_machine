#pragma once

#include <cstdint>

#include "core/Types.hpp"

namespace mm {

class VelocityEstimator {
 public:
  void configure(const EncoderConfiguration& encoder, float filter_tau_s);
  void reset(int64_t count, uint64_t timestamp_us);
  float update(int64_t count, uint64_t timestamp_us,
               uint64_t last_edge_timestamp_us = 0,
               uint32_t edge_period_us = 0,
               int8_t edge_direction = 0);
  float velocityRadPerSecond() const { return filtered_velocity_rad_s_; }

 private:
  EncoderConfiguration encoder_{};
  float filter_tau_s_ = 0.025F;
  int64_t previous_count_ = 0;
  int64_t accumulated_counts_ = 0;
  uint64_t previous_timestamp_us_ = 0;
  uint64_t window_start_us_ = 0;
  float count_velocity_rad_s_ = 0.0F;
  float filtered_velocity_rad_s_ = 0.0F;
};

}  // namespace mm
