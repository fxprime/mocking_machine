#include "drivers/ZeroIndexSensor.hpp"

#include <driver/gpio.h>
#include <esp_timer.h>

#include "control/RotorPosition.hpp"

namespace mm {

bool ZeroIndexSensor::begin(const uint8_t pin, QuadratureEncoder& encoder,
                            const uint32_t minimum_interval_us,
                            const uint32_t minimum_separation_counts) {
  encoder_ = &encoder;
  pin_ = pin;
  minimum_interval_us_ = minimum_interval_us;
  minimum_separation_counts_ = minimum_separation_counts;
  pinMode(pin, INPUT_PULLUP);
  attachInterruptArg(pin, &ZeroIndexSensor::interruptThunk, this, CHANGE);
  return true;
}

void IRAM_ATTR ZeroIndexSensor::interruptThunk(void* const argument) {
  static_cast<ZeroIndexSensor*>(argument)->handleChange();
}

void IRAM_ATTR ZeroIndexSensor::handleChange() {
  const uint64_t timestamp_us = static_cast<uint64_t>(esp_timer_get_time());
  const int64_t encoder_count = encoder_->countFromIsr();
  const int8_t encoder_rotation = encoder_->directionFromIsr();
  const int8_t directed_rotation =
      static_cast<int8_t>(encoder_rotation * encoder_direction_);
  const ZeroIndexEdge edge =
      gpio_get_level(static_cast<gpio_num_t>(pin_)) != 0
          ? ZeroIndexEdge::Rising
          : ZeroIndexEdge::Falling;

  portENTER_CRITICAL_ISR(&mutex_);
  volatile ZeroIndexEdgeCapture& raw =
      edge == ZeroIndexEdge::Rising ? rising_ : falling_;
  const bool raw_direction_changed =
      directed_rotation != 0 &&
      directed_rotation != raw.directed_rotation;
  const bool raw_accepted =
      zeroIndexMinimumIntervalElapsed(
          timestamp_us, raw.timestamp_us, minimum_interval_us_) &&
      (raw_direction_changed ||
       shouldAcceptZeroIndexEvent(
           timestamp_us, raw.timestamp_us, minimum_interval_us_, encoder_count,
           raw.encoder_count, minimum_separation_counts_));
  if (raw_accepted) {
    raw.timestamp_us = timestamp_us;
    raw.encoder_count = encoder_count;
    raw.directed_rotation = directed_rotation;
    ++raw.sequence;
  }
  if (!zeroIndexEdgeMatches(directed_rotation, edge, reference_side_)) {
    portEXIT_CRITICAL_ISR(&mutex_);
    return;
  }
  const bool selected_direction_changed =
      directed_rotation != 0 && directed_rotation != directed_rotation_;
  if (!raw_accepted ||
      !zeroIndexMinimumIntervalElapsed(
          timestamp_us, timestamp_us_, minimum_interval_us_) ||
      (!selected_direction_changed &&
       !shouldAcceptZeroIndexEvent(
           timestamp_us, timestamp_us_, minimum_interval_us_, encoder_count,
           encoder_count_, minimum_separation_counts_))) {
    ++rejected_count_;
    portEXIT_CRITICAL_ISR(&mutex_);
    return;
  }
  const int32_t correction_ticks =
      reference_side_ == ZeroIndexReferenceSide::ClockwiseRising
          ? clockwise_rising_correction_ticks_
          : clockwise_falling_correction_ticks_;
  timestamp_us_ = timestamp_us;
  encoder_count_ = applyZeroIndexDirectionCorrection(
      encoder_count, directed_rotation, encoder_direction_, correction_ticks);
  directed_rotation_ = directed_rotation;
  edge_ = edge;
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

void ZeroIndexSensor::configureDirectionSelection(
    const int8_t encoder_direction,
    const ZeroIndexReferenceSide reference_side,
    const int32_t clockwise_rising_correction_ticks,
    const int32_t clockwise_falling_correction_ticks) {
  portENTER_CRITICAL(&mutex_);
  encoder_direction_ = encoder_direction >= 0 ? 1 : -1;
  reference_side_ = reference_side;
  clockwise_rising_correction_ticks_ = clockwise_rising_correction_ticks;
  clockwise_falling_correction_ticks_ = clockwise_falling_correction_ticks;
  timestamp_us_ = 0U;
  directed_rotation_ = 0;
  portEXIT_CRITICAL(&mutex_);
}

ZeroIndexCapture ZeroIndexSensor::snapshot() const {
  portENTER_CRITICAL(&mutex_);
  ZeroIndexCapture result{};
  result.timestamp_us = timestamp_us_;
  result.encoder_count = encoder_count_;
  result.sequence = sequence_;
  result.rejected_count = rejected_count_;
  result.directed_rotation = directed_rotation_;
  result.edge = edge_;
  result.rising = {
      rising_.timestamp_us, rising_.encoder_count, rising_.sequence,
      rising_.directed_rotation};
  result.falling = {
      falling_.timestamp_us, falling_.encoder_count, falling_.sequence,
      falling_.directed_rotation};
  portEXIT_CRITICAL(&mutex_);
  return result;
}

}  // namespace mm
