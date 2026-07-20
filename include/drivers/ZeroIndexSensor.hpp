#pragma once

#include <Arduino.h>

#include <cstdint>

#include "drivers/QuadratureEncoder.hpp"

namespace mm {

struct ZeroIndexCapture {
  uint64_t timestamp_us = 0;
  int64_t encoder_count = 0;
  uint32_t sequence = 0;
  uint32_t rejected_count = 0;
};

class ZeroIndexSensor {
 public:
  bool begin(uint8_t pin, QuadratureEncoder& encoder, uint32_t minimum_interval_us,
             uint32_t minimum_separation_counts);
  void setMinimumIntervalUs(uint32_t minimum_interval_us);
  void setMinimumSeparationCounts(uint32_t minimum_separation_counts);
  ZeroIndexCapture snapshot() const;

 private:
  static void IRAM_ATTR interruptThunk(void* argument);
  void IRAM_ATTR handleRise();

  QuadratureEncoder* encoder_ = nullptr;
  volatile uint64_t timestamp_us_ = 0;
  volatile int64_t encoder_count_ = 0;
  volatile uint32_t sequence_ = 0;
  volatile uint32_t rejected_count_ = 0;
  volatile uint32_t minimum_interval_us_ = 0U;
  volatile uint32_t minimum_separation_counts_ = 0U;
  mutable portMUX_TYPE mutex_ = portMUX_INITIALIZER_UNLOCKED;
};

}  // namespace mm
