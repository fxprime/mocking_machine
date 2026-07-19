#include "drivers/ZeroIndexSensor.hpp"

#include <esp_timer.h>

#include "control/RotorPosition.hpp"

namespace mm {

bool ZeroIndexSensor::begin(const uint8_t pin, QuadratureEncoder& encoder,
                            const uint32_t minimum_interval_us) {
  encoder_ = &encoder;
  minimum_interval_us_ = minimum_interval_us;
  pinMode(pin, INPUT_PULLUP);
  attachInterruptArg(pin, &ZeroIndexSensor::interruptThunk, this, RISING);
  return true;
}

void IRAM_ATTR ZeroIndexSensor::interruptThunk(void* const argument) {
  static_cast<ZeroIndexSensor*>(argument)->handleRise();
}

void IRAM_ATTR ZeroIndexSensor::handleRise() {
  portENTER_CRITICAL_ISR(&mutex_);
  const uint64_t timestamp_us = static_cast<uint64_t>(esp_timer_get_time());
  if (!shouldAcceptZeroIndexRise(timestamp_us, timestamp_us_, minimum_interval_us_)) {
    ++rejected_count_;
    portEXIT_CRITICAL_ISR(&mutex_);
    return;
  }
  timestamp_us_ = timestamp_us;
  encoder_count_ = encoder_->countFromIsr();
  ++sequence_;
  portEXIT_CRITICAL_ISR(&mutex_);
}

void ZeroIndexSensor::setMinimumIntervalUs(const uint32_t minimum_interval_us) {
  portENTER_CRITICAL(&mutex_);
  minimum_interval_us_ = minimum_interval_us;
  portEXIT_CRITICAL(&mutex_);
}

ZeroIndexCapture ZeroIndexSensor::snapshot() const {
  portENTER_CRITICAL(&mutex_);
  const ZeroIndexCapture result{timestamp_us_, encoder_count_, sequence_, rejected_count_};
  portEXIT_CRITICAL(&mutex_);
  return result;
}

}  // namespace mm
