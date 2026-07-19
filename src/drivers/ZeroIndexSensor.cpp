#include "drivers/ZeroIndexSensor.hpp"

#include <esp_timer.h>

namespace mm {

bool ZeroIndexSensor::begin(const uint8_t pin, QuadratureEncoder& encoder) {
  encoder_ = &encoder;
  pinMode(pin, INPUT_PULLUP);
  attachInterruptArg(pin, &ZeroIndexSensor::interruptThunk, this, RISING);
  return true;
}

void IRAM_ATTR ZeroIndexSensor::interruptThunk(void* const argument) {
  static_cast<ZeroIndexSensor*>(argument)->handleRise();
}

void IRAM_ATTR ZeroIndexSensor::handleRise() {
  portENTER_CRITICAL_ISR(&mutex_);
  timestamp_us_ = static_cast<uint64_t>(esp_timer_get_time());
  encoder_count_ = encoder_->countFromIsr();
  ++sequence_;
  portEXIT_CRITICAL_ISR(&mutex_);
}

uint64_t ZeroIndexSensor::timestampUs() const {
  portENTER_CRITICAL(&mutex_);
  const uint64_t result = timestamp_us_;
  portEXIT_CRITICAL(&mutex_);
  return result;
}

int64_t ZeroIndexSensor::encoderCount() const {
  portENTER_CRITICAL(&mutex_);
  const int64_t result = encoder_count_;
  portEXIT_CRITICAL(&mutex_);
  return result;
}

uint32_t ZeroIndexSensor::sequence() const {
  portENTER_CRITICAL(&mutex_);
  const uint32_t result = sequence_;
  portEXIT_CRITICAL(&mutex_);
  return result;
}

}  // namespace mm
