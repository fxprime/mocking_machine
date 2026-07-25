#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace mm {

__attribute__((always_inline)) inline bool zeroIndexMinimumIntervalElapsed(
    const uint64_t timestamp_us, const uint64_t last_accepted_timestamp_us,
    const uint32_t minimum_interval_us) {
  return last_accepted_timestamp_us == 0U ||
      timestamp_us < last_accepted_timestamp_us ||
      timestamp_us - last_accepted_timestamp_us >= minimum_interval_us;
}

__attribute__((always_inline)) inline bool shouldAcceptZeroIndexEvent(
    const uint64_t timestamp_us, const uint64_t last_accepted_timestamp_us,
    const uint32_t minimum_interval_us, const int64_t encoder_count,
    const int64_t last_accepted_encoder_count,
    const uint32_t minimum_separation_counts) {
  if (!zeroIndexMinimumIntervalElapsed(
          timestamp_us, last_accepted_timestamp_us, minimum_interval_us)) {
    return false;
  }
  if (last_accepted_timestamp_us == 0U) {
    return true;
  }
  const uint64_t separation = encoder_count >= last_accepted_encoder_count
      ? static_cast<uint64_t>(encoder_count) -
            static_cast<uint64_t>(last_accepted_encoder_count)
      : static_cast<uint64_t>(last_accepted_encoder_count) -
            static_cast<uint64_t>(encoder_count);
  return separation >= minimum_separation_counts;
}

inline uint32_t zeroIndexMinimumSeparationCounts(
    const uint32_t counts_per_revolution,
    const float minimum_separation_revolutions) {
  if (counts_per_revolution == 0U || minimum_separation_revolutions <= 0.0F) {
    return 0U;
  }
  const uint32_t separation = static_cast<uint32_t>(
      static_cast<float>(counts_per_revolution) * minimum_separation_revolutions);
  return separation == 0U ? 1U : separation;
}

class RotorPhaseTracker {
 public:
  void configure(const uint32_t counts_per_revolution, const int8_t encoder_direction,
                 const float zero_correction_gain,
                 const uint32_t zero_position_offset_ticks = 0U) {
    const int8_t normalized_direction = encoder_direction >= 0 ? 1 : -1;
    const float bounded_gain = std::clamp(zero_correction_gain, 0.0F, 1.0F);
    const uint32_t normalized_offset_ticks =
        wrapPositiveTickOffset(zero_position_offset_ticks, counts_per_revolution);
    const bool tracking_configuration_unchanged =
        counts_per_revolution_ == counts_per_revolution &&
        encoder_direction_ == normalized_direction &&
        zero_correction_gain_ == bounded_gain;
    if (tracking_configuration_unchanged && referenced_) {
      phase_offset_counts_ = wrapSignedCounts(
          phase_offset_counts_ -
          (static_cast<float>(normalized_offset_ticks) -
           static_cast<float>(zero_position_offset_ticks_)));
    }
    zero_position_offset_ticks_ = normalized_offset_ticks;
    if (tracking_configuration_unchanged) {
      return;
    }
    counts_per_revolution_ = counts_per_revolution;
    encoder_direction_ = normalized_direction;
    zero_correction_gain_ = bounded_gain;
    reset();
  }

  void reset() {
    phase_offset_counts_ = 0.0F;
    last_zero_sequence_ = 0U;
    referenced_ = false;
  }

  float update(const int64_t encoder_count, const int64_t zero_encoder_count,
               const uint32_t zero_sequence) {
    if (counts_per_revolution_ == 0U) {
      return 0.0F;
    }
    if (zero_sequence != 0U && zero_sequence != last_zero_sequence_) {
      const float zero_encoder_phase = directedModuloCounts(zero_encoder_count);
      const float correction_target_counts =
          -static_cast<float>(zero_position_offset_ticks_);
      if (!referenced_) {
        phase_offset_counts_ = wrapSignedCounts(
            correction_target_counts - zero_encoder_phase);
        referenced_ = true;
      } else {
        const float phase_error_counts =
            wrapSignedCounts(zero_encoder_phase + phase_offset_counts_ -
                             correction_target_counts);
        phase_offset_counts_ = wrapSignedCounts(
            phase_offset_counts_ - zero_correction_gain_ * phase_error_counts);
      }
      last_zero_sequence_ = zero_sequence;
    }
    if (!referenced_) {
      return 0.0F;
    }
    const float position_counts = wrapPositiveCounts(
        directedModuloCounts(encoder_count) + phase_offset_counts_);
    return position_counts * 360.0F /
           static_cast<float>(counts_per_revolution_);
  }

  bool referenced() const { return referenced_; }
  uint32_t zeroPositionOffsetTicks() const {
    return zero_position_offset_ticks_;
  }

  uint32_t positionTicksFromZeroIndex(const int64_t encoder_count,
                                      const int64_t zero_encoder_count) const {
    if (counts_per_revolution_ == 0U) {
      return 0U;
    }
    const int64_t revolution_counts =
        static_cast<int64_t>(counts_per_revolution_);
    int64_t relative_counts =
        ((encoder_count % revolution_counts) -
         (zero_encoder_count % revolution_counts)) *
        static_cast<int64_t>(encoder_direction_);
    relative_counts %= revolution_counts;
    if (relative_counts < 0) {
      relative_counts += revolution_counts;
    }
    return static_cast<uint32_t>(relative_counts);
  }

  void synchronizeToZeroIndex(const int64_t zero_encoder_count,
                              const uint32_t zero_sequence) {
    if (counts_per_revolution_ == 0U || zero_sequence == 0U) {
      return;
    }
    phase_offset_counts_ =
        wrapSignedCounts(-directedModuloCounts(zero_encoder_count) -
                         static_cast<float>(zero_position_offset_ticks_));
    last_zero_sequence_ = zero_sequence;
    referenced_ = true;
  }

 private:
  static uint32_t wrapPositiveTickOffset(
      const uint32_t ticks, const uint32_t counts_per_revolution) {
    if (counts_per_revolution == 0U) {
      return 0U;
    }
    return ticks % counts_per_revolution;
  }

  float directedModuloCounts(const int64_t encoder_count) const {
    const int64_t revolution_counts = static_cast<int64_t>(counts_per_revolution_);
    return static_cast<float>(encoder_count % revolution_counts) *
           static_cast<float>(encoder_direction_);
  }

  float wrapPositiveCounts(const float counts) const {
    const float revolution_counts = static_cast<float>(counts_per_revolution_);
    float wrapped = std::fmod(counts, revolution_counts);
    if (wrapped < 0.0F) {
      wrapped += revolution_counts;
    }
    return wrapped;
  }

  float wrapSignedCounts(const float counts) const {
    const float revolution_counts = static_cast<float>(counts_per_revolution_);
    float wrapped = wrapPositiveCounts(counts);
    if (wrapped >= revolution_counts * 0.5F) {
      wrapped -= revolution_counts;
    }
    return wrapped;
  }

  uint32_t counts_per_revolution_ = 0U;
  int8_t encoder_direction_ = 1;
  float zero_correction_gain_ = 0.10F;
  uint32_t zero_position_offset_ticks_ = 0U;
  float phase_offset_counts_ = 0.0F;
  uint32_t last_zero_sequence_ = 0U;
  bool referenced_ = false;
};

}  // namespace mm
