#pragma once

#include <Arduino.h>

#include <cstdint>

#include "drivers/EncoderPeriodAverager.hpp"

namespace mm {

struct EncoderEdgeTiming {
  uint64_t timestamp_us = 0;
  uint32_t period_us = 0;
  int8_t direction = 0;
};

class QuadratureEncoder {
 public:
  bool begin(uint8_t pin_a, uint8_t pin_b);
  int64_t count() const;
  int64_t IRAM_ATTR countFromIsr() const { return count_; }
  uint64_t lastEdgeTimestampUs() const;
  EncoderEdgeTiming edgeTiming() const;
  void reset(int64_t value = 0);

 private:
  static void IRAM_ATTR interruptThunk(void* argument);
  void IRAM_ATTR handleEdge();

  uint8_t pin_a_ = 0;
  uint8_t pin_b_ = 0;
  volatile int64_t count_ = 0;
  volatile uint64_t last_edge_us_ = 0;
  volatile uint32_t edge_period_us_ = 0;
  volatile int8_t edge_direction_ = 0;
  volatile uint8_t previous_state_ = 0;
  EncoderPeriodAverager period_averager_{};
  mutable portMUX_TYPE mutex_ = portMUX_INITIALIZER_UNLOCKED;
};

}  // namespace mm
