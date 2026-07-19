#pragma once

#include <cstddef>
#include <cstdint>

namespace mm {

// Keeps a short history of valid quadrature edges. Measuring the interval
// across several edges reduces timestamp jitter without adding a control-loop
// filter delay. All methods are allocation-free and safe to inline in an ISR.
class EncoderPeriodAverager {
 public:
  static constexpr uint8_t kSampleCount = 5U;

  __attribute__((always_inline)) inline void addEdge(const uint64_t timestamp_us,
                                                      const int8_t direction) {
    if (direction == 0) {
      return;
    }
    if (direction != direction_) {
      sample_count_ = 0U;
      next_index_ = 0U;
      direction_ = direction;
    }
    timestamps_us_[next_index_] = timestamp_us;
    next_index_ = static_cast<uint8_t>((next_index_ + 1U) % kSampleCount);
    if (sample_count_ < kSampleCount) {
      ++sample_count_;
    }
  }

  __attribute__((always_inline)) inline uint32_t averagePeriodUs() const {
    if (sample_count_ < 2U) {
      return 0U;
    }
    const uint8_t newest_index =
        static_cast<uint8_t>((next_index_ + kSampleCount - 1U) % kSampleCount);
    const uint8_t oldest_index =
        static_cast<uint8_t>((next_index_ + kSampleCount - sample_count_) % kSampleCount);
    const uint64_t span_us = timestamps_us_[newest_index] - timestamps_us_[oldest_index];
    const uint64_t average_us = span_us / static_cast<uint64_t>(sample_count_ - 1U);
    return average_us <= UINT32_MAX ? static_cast<uint32_t>(average_us) : UINT32_MAX;
  }

 private:
  uint64_t timestamps_us_[kSampleCount]{};
  uint8_t sample_count_ = 0U;
  uint8_t next_index_ = 0U;
  int8_t direction_ = 0;
};

}  // namespace mm
