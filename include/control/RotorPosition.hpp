#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace mm {

__attribute__((always_inline)) inline bool shouldAcceptZeroIndexRise(
    const uint64_t timestamp_us, const uint64_t last_accepted_timestamp_us,
    const uint32_t minimum_interval_us, const int64_t encoder_count,
    const int64_t last_accepted_encoder_count,
    const uint32_t minimum_separation_counts) {
  if (last_accepted_timestamp_us == 0U) {
    return true;
  }
  if (timestamp_us >= last_accepted_timestamp_us &&
      timestamp_us - last_accepted_timestamp_us < minimum_interval_us) {
    return false;
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
                 const float zero_correction_gain) {
    const int8_t normalized_direction = encoder_direction >= 0 ? 1 : -1;
    const float bounded_gain = std::clamp(zero_correction_gain, 0.0F, 1.0F);
    if (counts_per_revolution_ == counts_per_revolution &&
        encoder_direction_ == normalized_direction &&
        zero_correction_gain_ == bounded_gain) {
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
      if (!referenced_) {
        phase_offset_counts_ = wrapSignedCounts(-zero_encoder_phase);
        referenced_ = true;
      } else {
        const float phase_error_counts =
            wrapSignedCounts(zero_encoder_phase + phase_offset_counts_);
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
    return position_counts * 360.0F / static_cast<float>(counts_per_revolution_);
  }

  bool referenced() const { return referenced_; }

 private:
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
  float phase_offset_counts_ = 0.0F;
  uint32_t last_zero_sequence_ = 0U;
  bool referenced_ = false;
};

}  // namespace mm
