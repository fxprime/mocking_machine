#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace mm {

constexpr size_t kMaximumVelocityTestLevels = 16U;

class VelocityStepSequence {
 public:
  void start(const std::array<float, kMaximumVelocityTestLevels>& levels,
             uint8_t level_count, uint32_t hold_ms, uint64_t start_us) {
    levels_ = levels;
    level_count_ = level_count <= kMaximumVelocityTestLevels
                       ? level_count
                       : static_cast<uint8_t>(kMaximumVelocityTestLevels);
    hold_us_ = static_cast<uint64_t>(hold_ms) * 1000ULL;
    start_us_ = start_us;
    active_ = level_count_ > 0U && hold_us_ > 0U;
  }

  void stop() { active_ = false; }

  float target(uint64_t timestamp_us) const {
    if (!active_ || timestamp_us < start_us_) return 0.0F;
    const uint64_t level_index = (timestamp_us - start_us_) / hold_us_;
    return level_index < level_count_ ? levels_[level_index] : 0.0F;
  }

  bool finished(uint64_t timestamp_us) const {
    return active_ && timestamp_us >=
        start_us_ + static_cast<uint64_t>(level_count_) * hold_us_;
  }

  bool active() const { return active_; }
  uint8_t levelCount() const { return level_count_; }

 private:
  std::array<float, kMaximumVelocityTestLevels> levels_{};
  uint8_t level_count_ = 0U;
  uint64_t hold_us_ = 0U;
  uint64_t start_us_ = 0U;
  bool active_ = false;
};

}  // namespace mm
