#include "drivers/QuadratureEncoder.hpp"

#include <driver/gpio.h>
#include <esp_timer.h>

namespace mm {
namespace {
DRAM_ATTR const int8_t kTransition[16] = {
    0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0,
};
}

bool QuadratureEncoder::begin(const uint8_t pin_a, const uint8_t pin_b) {
  pin_a_ = pin_a;
  pin_b_ = pin_b;
  pinMode(pin_a_, INPUT_PULLUP);
  pinMode(pin_b_, INPUT_PULLUP);
  previous_state_ = static_cast<uint8_t>((gpio_get_level(static_cast<gpio_num_t>(pin_a_)) << 1) |
                                         gpio_get_level(static_cast<gpio_num_t>(pin_b_)));
  last_edge_us_ = static_cast<uint64_t>(esp_timer_get_time());
  attachInterruptArg(pin_a_, &QuadratureEncoder::interruptThunk, this, CHANGE);
  attachInterruptArg(pin_b_, &QuadratureEncoder::interruptThunk, this, CHANGE);
  return true;
}

void IRAM_ATTR QuadratureEncoder::interruptThunk(void* const argument) {
  static_cast<QuadratureEncoder*>(argument)->handleEdge();
}

void IRAM_ATTR QuadratureEncoder::handleEdge() {
  const uint8_t state = static_cast<uint8_t>(
      (gpio_get_level(static_cast<gpio_num_t>(pin_a_)) << 1) |
      gpio_get_level(static_cast<gpio_num_t>(pin_b_)));
  portENTER_CRITICAL_ISR(&mutex_);
  const int8_t delta = kTransition[(previous_state_ << 2U) | state];
  count_ += delta;
  previous_state_ = state;
  if (delta != 0) {
    const uint64_t now = static_cast<uint64_t>(esp_timer_get_time());
    last_edge_us_ = now;
  }
  portEXIT_CRITICAL_ISR(&mutex_);
}

int64_t QuadratureEncoder::count() const {
  portENTER_CRITICAL(&mutex_);
  const int64_t result = count_;
  portEXIT_CRITICAL(&mutex_);
  return result;
}

uint64_t QuadratureEncoder::lastEdgeTimestampUs() const {
  portENTER_CRITICAL(&mutex_);
  const uint64_t result = last_edge_us_;
  portEXIT_CRITICAL(&mutex_);
  return result;
}

void QuadratureEncoder::reset(const int64_t value) {
  portENTER_CRITICAL(&mutex_);
  count_ = value;
  portEXIT_CRITICAL(&mutex_);
}

}  // namespace mm
