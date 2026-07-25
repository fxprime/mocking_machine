#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace mm {

constexpr uint32_t zeroIndexCalibrationEncoderTimeoutMs(
    const uint32_t encoder_timeout_ms,
    const uint32_t reversal_pause_ms) {
  return std::max(encoder_timeout_ms, reversal_pause_ms);
}

class EncoderActivityWatchdog {
 public:
  void reset() { demand_start_us_ = 0; }

  bool update(uint64_t timestamp_us, float desired_velocity_rad_s,
              uint64_t last_edge_us, bool running, uint32_t timeout_ms,
              float demand_threshold_rad_s,
              bool force_motion_expected = false) {
    const bool motion_expected = running &&
        (force_motion_expected ||
         std::fabs(desired_velocity_rad_s) > demand_threshold_rad_s);
    if (!motion_expected) {
      reset();
      return false;
    }
    if (demand_start_us_ == 0U) {
      demand_start_us_ = timestamp_us;
      return false;
    }
    const uint64_t activity_us = std::max(demand_start_us_, last_edge_us);
    const uint64_t timeout_us = static_cast<uint64_t>(timeout_ms) * 1000ULL;
    return timestamp_us > activity_us && timestamp_us - activity_us > timeout_us;
  }

 private:
  uint64_t demand_start_us_ = 0;
};

}  // namespace mm
