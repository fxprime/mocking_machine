#pragma once

#include <Arduino.h>

#include <cstdint>

#include "drivers/QuadratureEncoder.hpp"

namespace mm {

class ZeroIndexSensor {
 public:
  bool begin(uint8_t pin, QuadratureEncoder& encoder);
  uint64_t timestampUs() const;
  int64_t encoderCount() const;
  uint32_t sequence() const;

 private:
  static void IRAM_ATTR interruptThunk(void* argument);
  void IRAM_ATTR handleRise();

  QuadratureEncoder* encoder_ = nullptr;
  volatile uint64_t timestamp_us_ = 0;
  volatile int64_t encoder_count_ = 0;
  volatile uint32_t sequence_ = 0;
  mutable portMUX_TYPE mutex_ = portMUX_INITIALIZER_UNLOCKED;
};

}  // namespace mm

