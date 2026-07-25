#pragma once

#include <Arduino.h>

#include <cstdint>

#include "control/ZeroIndexCalibration.hpp"
#include "drivers/QuadratureEncoder.hpp"

namespace mm {

struct ZeroIndexEdgeCapture {
  uint64_t timestamp_us = 0;
  int64_t encoder_count = 0;
  uint32_t sequence = 0;
  int8_t directed_rotation = 0;
};

struct ZeroIndexCapture {
  uint64_t timestamp_us = 0;
  int64_t encoder_count = 0;
  uint32_t sequence = 0;
  uint32_t rejected_count = 0;
  int8_t directed_rotation = 0;
  ZeroIndexEdge edge = ZeroIndexEdge::Rising;
  ZeroIndexEdgeCapture rising{};
  ZeroIndexEdgeCapture falling{};
};

class ZeroIndexSensor {
 public:
  bool begin(uint8_t pin, QuadratureEncoder& encoder, uint32_t minimum_interval_us,
             uint32_t minimum_separation_counts);
  void setMinimumIntervalUs(uint32_t minimum_interval_us);
  void setMinimumSeparationCounts(uint32_t minimum_separation_counts);
  void configureDirectionSelection(
      int8_t encoder_direction, ZeroIndexReferenceSide reference_side,
      int32_t clockwise_rising_correction_ticks,
      int32_t clockwise_falling_correction_ticks);
  ZeroIndexCapture snapshot() const;

 private:
  static void IRAM_ATTR interruptThunk(void* argument);
  void IRAM_ATTR handleChange();

  QuadratureEncoder* encoder_ = nullptr;
  uint8_t pin_ = 0U;
  volatile uint64_t timestamp_us_ = 0;
  volatile int64_t encoder_count_ = 0;
  volatile uint32_t sequence_ = 0;
  volatile uint32_t rejected_count_ = 0;
  volatile int8_t directed_rotation_ = 0;
  volatile ZeroIndexEdge edge_ = ZeroIndexEdge::Rising;
  volatile ZeroIndexEdgeCapture rising_{};
  volatile ZeroIndexEdgeCapture falling_{};
  volatile int8_t encoder_direction_ = 1;
  volatile ZeroIndexReferenceSide reference_side_ =
      ZeroIndexReferenceSide::ClockwiseRising;
  volatile int32_t clockwise_rising_correction_ticks_ = 0;
  volatile int32_t clockwise_falling_correction_ticks_ = 0;
  volatile uint32_t minimum_interval_us_ = 0U;
  volatile uint32_t minimum_separation_counts_ = 0U;
  mutable portMUX_TYPE mutex_ = portMUX_INITIALIZER_UNLOCKED;
};

}  // namespace mm
