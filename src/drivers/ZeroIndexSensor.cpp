#include "drivers/ZeroIndexSensor.hpp"

#include <esp_timer.h>

#include "control/RotorPosition.hpp"

namespace mm {

bool ZeroIndexSensor::begin(const uint8_t pin, QuadratureEncoder& encoder,
                            const uint32_t minimum_interval_us,
                            const uint32_t minimum_separation_counts) {
  encoder_ = &encoder;
  minimum_interval_us_ = minimum_interval_us;
  minimum_separation_counts_ = minimum_separation_counts;
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
  const int64_t encoder_count = encoder_->countFromIsr();
  if (!shouldAcceptZeroIndexRise(timestamp_us, timestamp_us_, minimum_interval_us_,
                                 encoder_count, encoder_count_,
                                 minimum_separation_counts_)) {
    ++rejected_count_;
    portEXIT_CRITICAL_ISR(&mutex_);
    return;
  }
  timestamp_us_ = timestamp_us;
  encoder_count_ = encoder_count;
  ++sequence_;
  portEXIT_CRITICAL_ISR(&mutex_);
}

void ZeroIndexSensor::setMinimumIntervalUs(const uint32_t minimum_interval_us) {
  portENTER_CRITICAL(&mutex_);
  minimum_interval_us_ = minimum_interval_us;
  portEXIT_CRITICAL(&mutex_);
}

void ZeroIndexSensor::setMinimumSeparationCounts(
    const uint32_t minimum_separation_counts) {
  portENTER_CRITICAL(&mutex_);
  minimum_separation_counts_ = minimum_separation_counts;
  portEXIT_CRITICAL(&mutex_);
}

ZeroIndexCapture ZeroIndexSensor::snapshot() const {
  portENTER_CRITICAL(&mutex_);
  const ZeroIndexCapture result{timestamp_us_, encoder_count_, sequence_, rejected_count_};
  portEXIT_CRITICAL(&mutex_);
  return result;
}

}  // namespace mm
