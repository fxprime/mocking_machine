#pragma once

#include <cmath>

namespace mm {

struct CurrentCalibrationPoint {
  float sense_voltage_v = 0.0F;
  float reference_current_a = 0.0F;
};

struct CurrentCalibrationResult {
  float gain_a_per_v = 0.0F;
  float offset_v = 0.0F;
};

class CurrentCalibrationAccumulator {
 public:
  static constexpr uint8_t kRequiredSamples = 64U;

  void start(const float reference_current_a) {
    reference_current_a_ = reference_current_a;
    voltage_sum_v_ = 0.0F;
    sample_count_ = 0U;
    active_ = true;
  }

  bool addSample(const float sense_voltage_v, float& average_voltage_v) {
    if (!active_ || !std::isfinite(sense_voltage_v)) {
      return false;
    }
    voltage_sum_v_ += sense_voltage_v;
    ++sample_count_;
    if (sample_count_ < kRequiredSamples) {
      return false;
    }
    average_voltage_v = voltage_sum_v_ / static_cast<float>(sample_count_);
    active_ = false;
    return true;
  }

  void cancel() { active_ = false; }
  bool active() const { return active_; }
  uint8_t sampleCount() const { return sample_count_; }
  float referenceCurrentA() const { return reference_current_a_; }

 private:
  float reference_current_a_ = 0.0F;
  float voltage_sum_v_ = 0.0F;
  uint8_t sample_count_ = 0U;
  bool active_ = false;
};

inline bool calculateCurrentCalibration(const CurrentCalibrationPoint& low,
                                        const CurrentCalibrationPoint& high,
                                        CurrentCalibrationResult& result) {
  constexpr float kMinimumVoltageSpanV = 0.001F;
  constexpr float kMinimumCurrentSpanA = 0.01F;
  const float voltage_span = high.sense_voltage_v - low.sense_voltage_v;
  const float current_span = high.reference_current_a - low.reference_current_a;
  if (!std::isfinite(low.sense_voltage_v) ||
      !std::isfinite(low.reference_current_a) ||
      !std::isfinite(high.sense_voltage_v) ||
      !std::isfinite(high.reference_current_a) ||
      voltage_span < kMinimumVoltageSpanV ||
      current_span < kMinimumCurrentSpanA) {
    return false;
  }
  result.gain_a_per_v = current_span / voltage_span;
  result.offset_v = low.sense_voltage_v -
                    low.reference_current_a / result.gain_a_per_v;
  return std::isfinite(result.gain_a_per_v) && result.gain_a_per_v > 0.0F &&
         result.gain_a_per_v <= 10000.0F && std::isfinite(result.offset_v);
}

}  // namespace mm
