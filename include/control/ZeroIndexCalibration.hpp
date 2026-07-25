#pragma once

#include <array>
#include <cstdint>

#include "core/Types.hpp"

namespace mm {

constexpr float zeroIndexCalibrationTargetVelocityRadS(
    const int8_t directed_rotation, const float speed_rpm) {
  return static_cast<float>(directed_rotation >= 0 ? 1 : -1) *
      speed_rpm * EncoderConfiguration::kRadiansPerSecondPerRpm;
}

enum class ZeroIndexEdge : uint8_t {
  Falling = 0,
  Rising = 1,
};

enum class ZeroIndexReferenceSide : uint8_t {
  ClockwiseRising = 0,
  ClockwiseFalling = 1,
};

inline bool zeroIndexEdgeMatches(
    const int8_t directed_rotation, const ZeroIndexEdge edge,
    const ZeroIndexReferenceSide reference_side) {
  if (directed_rotation == 0) {
    return false;
  }
  const bool clockwise = directed_rotation > 0;
  const bool rising = edge == ZeroIndexEdge::Rising;
  if (reference_side == ZeroIndexReferenceSide::ClockwiseRising) {
    return clockwise == rising;
  }
  return clockwise != rising;
}

inline int64_t applyZeroIndexDirectionCorrection(
    const int64_t encoder_count, const int8_t directed_rotation,
    const int8_t encoder_direction, const int32_t counterclockwise_correction_ticks) {
  if (directed_rotation >= 0) {
    return encoder_count;
  }
  const int8_t normalized_encoder_direction = encoder_direction >= 0 ? 1 : -1;
  return encoder_count +
      static_cast<int64_t>(counterclockwise_correction_ticks) *
          static_cast<int64_t>(normalized_encoder_direction);
}

struct ZeroIndexHysteresisResult {
  int32_t clockwise_rising_correction_ticks = 0;
  int32_t clockwise_falling_correction_ticks = 0;
  uint16_t maximum_residual_ticks = 0;
};

class ZeroIndexHysteresisCalibration {
 public:
  static constexpr uint8_t kRequiredPasses = 5U;

  void configure(const uint32_t counts_per_revolution,
                 const int8_t encoder_direction,
                 const uint16_t maximum_error_ticks) {
    counts_per_revolution_ = counts_per_revolution;
    encoder_direction_ = encoder_direction >= 0 ? 1 : -1;
    maximum_error_ticks_ = maximum_error_ticks;
    reset();
  }

  void reset() {
    clockwise_count_ = 0U;
    counterclockwise_count_ = 0U;
  }

  bool addPass(const int8_t directed_rotation, const int64_t rising_encoder_count,
               const int64_t falling_encoder_count) {
    if (counts_per_revolution_ == 0U || directed_rotation == 0) {
      return false;
    }
    auto& count = directed_rotation > 0 ? clockwise_count_ : counterclockwise_count_;
    if (count >= kRequiredPasses) {
      return false;
    }
    auto& rising = directed_rotation > 0 ? clockwise_rising_ : counterclockwise_rising_;
    auto& falling = directed_rotation > 0 ? clockwise_falling_ : counterclockwise_falling_;
    const int64_t directed_rising =
        rising_encoder_count * static_cast<int64_t>(encoder_direction_);
    const int64_t directed_falling =
        falling_encoder_count * static_cast<int64_t>(encoder_direction_);
    if (count > 0U) {
      const int64_t expected_delta =
          static_cast<int64_t>(directed_rotation > 0 ? 1 : -1) *
          static_cast<int64_t>(counts_per_revolution_);
      if (absolute((directed_rising - rising[count - 1U]) - expected_delta) >
              maximum_error_ticks_ ||
          absolute((directed_falling - falling[count - 1U]) - expected_delta) >
              maximum_error_ticks_) {
        return false;
      }
    }
    rising[count] = directed_rising;
    falling[count] = directed_falling;
    ++count;
    return true;
  }

  uint8_t clockwiseCount() const { return clockwise_count_; }
  uint8_t counterclockwiseCount() const { return counterclockwise_count_; }
  bool complete() const {
    return clockwise_count_ == kRequiredPasses &&
        counterclockwise_count_ == kRequiredPasses;
  }

  bool result(ZeroIndexHysteresisResult& result) const {
    if (!complete()) {
      return false;
    }
    const int64_t clockwise_rising = circularMean(clockwise_rising_);
    const int64_t clockwise_falling = circularMean(clockwise_falling_);
    const int64_t counterclockwise_rising = circularMean(counterclockwise_rising_);
    const int64_t counterclockwise_falling = circularMean(counterclockwise_falling_);
    const int64_t rising_side_correction =
        wrapSigned(clockwise_rising - counterclockwise_falling);
    const int64_t falling_side_correction =
        wrapSigned(clockwise_falling - counterclockwise_rising);
    if (rising_side_correction < INT32_MIN || rising_side_correction > INT32_MAX ||
        falling_side_correction < INT32_MIN || falling_side_correction > INT32_MAX) {
      return false;
    }
    result.clockwise_rising_correction_ticks =
        static_cast<int32_t>(rising_side_correction);
    result.clockwise_falling_correction_ticks =
        static_cast<int32_t>(falling_side_correction);
    uint64_t maximum_residual = 0U;
    maximum_residual = maximumResidual(
        clockwise_rising_, clockwise_rising, maximum_residual);
    maximum_residual = maximumResidual(
        clockwise_falling_, clockwise_falling, maximum_residual);
    maximum_residual = maximumResidual(
        counterclockwise_rising_, counterclockwise_rising, maximum_residual);
    maximum_residual = maximumResidual(
        counterclockwise_falling_, counterclockwise_falling, maximum_residual);
    if (maximum_residual > maximum_error_ticks_ || maximum_residual > UINT16_MAX) {
      return false;
    }
    result.maximum_residual_ticks = static_cast<uint16_t>(maximum_residual);
    return true;
  }

 private:
  static uint64_t absolute(const int64_t value) {
    return value < 0 ? static_cast<uint64_t>(-(value + 1)) + 1U
                     : static_cast<uint64_t>(value);
  }

  int64_t wrapSigned(int64_t value) const {
    const int64_t revolution = static_cast<int64_t>(counts_per_revolution_);
    value %= revolution;
    const int64_t half = revolution / 2;
    if (value > half) {
      value -= revolution;
    } else if (value < -half) {
      value += revolution;
    }
    return value;
  }

  int64_t circularMean(
      const std::array<int64_t, kRequiredPasses>& samples) const {
    const int64_t anchor = samples[0];
    int64_t residual_sum = 0;
    for (const int64_t sample : samples) {
      residual_sum += wrapSigned(sample - anchor);
    }
    const int64_t rounded_residual =
        residual_sum >= 0
            ? (residual_sum + kRequiredPasses / 2) / kRequiredPasses
            : (residual_sum - kRequiredPasses / 2) / kRequiredPasses;
    return anchor + rounded_residual;
  }

  uint64_t maximumResidual(
      const std::array<int64_t, kRequiredPasses>& samples, const int64_t mean,
      uint64_t maximum) const {
    for (const int64_t sample : samples) {
      const uint64_t residual = absolute(wrapSigned(sample - mean));
      if (residual > maximum) {
        maximum = residual;
      }
    }
    return maximum;
  }

  uint32_t counts_per_revolution_ = 0U;
  int8_t encoder_direction_ = 1;
  uint16_t maximum_error_ticks_ = 0U;
  std::array<int64_t, kRequiredPasses> clockwise_rising_{};
  std::array<int64_t, kRequiredPasses> clockwise_falling_{};
  std::array<int64_t, kRequiredPasses> counterclockwise_rising_{};
  std::array<int64_t, kRequiredPasses> counterclockwise_falling_{};
  uint8_t clockwise_count_ = 0U;
  uint8_t counterclockwise_count_ = 0U;
};

}  // namespace mm
