#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace mm {

class P2QuantileEstimator {
 public:
  void configure(float quantile) {
    quantile_ = std::clamp(quantile, 0.5F, 0.999F);
    reset();
  }

  void reset() {
    count_ = 0U;
    initial_ = {};
    heights_ = {};
    positions_ = {};
    desired_positions_ = {};
  }

  void add(float sample) {
    if (!std::isfinite(sample)) return;
    if (count_ < initial_.size()) {
      initial_[count_++] = sample;
      if (count_ == initial_.size()) initializeMarkers();
      return;
    }

    size_t cell = 0U;
    if (sample < heights_[0]) {
      heights_[0] = sample;
    } else if (sample < heights_[1]) {
      cell = 0U;
    } else if (sample < heights_[2]) {
      cell = 1U;
    } else if (sample < heights_[3]) {
      cell = 2U;
    } else if (sample <= heights_[4]) {
      cell = 3U;
    } else {
      heights_[4] = sample;
      cell = 3U;
    }
    for (size_t index = cell + 1U; index < positions_.size(); ++index) {
      positions_[index] += 1.0F;
    }
    const std::array<float, 5> increments{
        0.0F, quantile_ * 0.5F, quantile_, (1.0F + quantile_) * 0.5F, 1.0F};
    for (size_t index = 0U; index < desired_positions_.size(); ++index) {
      desired_positions_[index] += increments[index];
    }
    adjustMarkers();
    ++count_;
  }

  uint32_t count() const { return count_; }

  float value() const {
    if (count_ == 0U) return 0.0F;
    if (count_ >= initial_.size()) return heights_[2];
    auto sorted = initial_;
    std::sort(sorted.begin(), sorted.begin() + count_);
    const size_t index = static_cast<size_t>(
        std::ceil(quantile_ * static_cast<float>(count_ - 1U)));
    return sorted[index];
  }

 private:
  void initializeMarkers() {
    std::sort(initial_.begin(), initial_.end());
    heights_ = initial_;
    positions_ = {1.0F, 2.0F, 3.0F, 4.0F, 5.0F};
    desired_positions_ = {
        1.0F, 1.0F + 2.0F * quantile_, 1.0F + 4.0F * quantile_,
        3.0F + 2.0F * quantile_, 5.0F};
  }

  void adjustMarkers() {
    for (size_t index = 1U; index < 4U; ++index) {
      const float delta = desired_positions_[index] - positions_[index];
      const bool move_up = delta >= 1.0F &&
                           positions_[index + 1U] - positions_[index] > 1.0F;
      const bool move_down = delta <= -1.0F &&
                             positions_[index - 1U] - positions_[index] < -1.0F;
      if (!move_up && !move_down) continue;
      const float direction = delta > 0.0F ? 1.0F : -1.0F;
      const float lower_span = positions_[index] - positions_[index - 1U];
      const float upper_span = positions_[index + 1U] - positions_[index];
      const float total_span = positions_[index + 1U] - positions_[index - 1U];
      const float parabolic = heights_[index] + direction / total_span *
          ((lower_span + direction) *
               (heights_[index + 1U] - heights_[index]) / upper_span +
           (upper_span - direction) *
               (heights_[index] - heights_[index - 1U]) / lower_span);
      if (parabolic > heights_[index - 1U] && parabolic < heights_[index + 1U]) {
        heights_[index] = parabolic;
      } else {
        const size_t neighbor = direction > 0.0F ? index + 1U : index - 1U;
        heights_[index] += direction *
            (heights_[neighbor] - heights_[index]) /
            (positions_[neighbor] - positions_[index]);
      }
      positions_[index] += direction;
    }
  }

  float quantile_ = 0.95F;
  uint32_t count_ = 0U;
  std::array<float, 5> initial_{};
  std::array<float, 5> heights_{};
  std::array<float, 5> positions_{};
  std::array<float, 5> desired_positions_{};
};

}  // namespace mm
