#pragma once

#include <Arduino.h>

#include "core/Types.hpp"

namespace mm {

class Vnh2sp30MotorDriver {
 public:
  bool begin(const MachineSettings& settings);
  void command(float signed_duty);
  void commandRaw(float signed_duty);
  void stop();
  void disable();
  float currentAmperes() const;
  float currentAmperesFromVoltage(float sense_voltage_v) const;
  float currentSenseVoltage(uint16_t sample_count = 1U) const;
  float supplyVoltage() const;
  float supplySenseVoltage(uint16_t sample_count = 1U) const;
  void setCurrentCalibration(float gain_a_per_v, float offset_v);
  void setCharacteristics(const MotorCharacteristics& characteristics);
  void setSupplyVoltageCalibration(float divider_gain, float input_offset_v);
  void setDiagnosticEnabled(bool enabled);
  void setSafety(const SafetyConfiguration& safety) { safety_ = safety; }
  void setMotorDirection(int8_t direction) { motor_direction_ = direction >= 0 ? 1 : -1; }
  bool diagnosticFault() const;
  float appliedDuty() const { return applied_duty_; }

 private:
  void writeDirection(bool forward);

  PinConfiguration pins_{};
  MotorCharacteristics characteristics_{};
  VoltageSenseConfiguration supply_voltage_{};
  SafetyConfiguration safety_{};
  StopMode stop_mode_ = StopMode::BrakeToGround;
  int8_t motor_direction_ = 1;
  bool diagnostic_enabled_ = false;
  float applied_duty_ = 0.0F;
  static constexpr uint8_t kPwmChannel = 0;
  static constexpr uint16_t kPwmFrequencyHz = 20000;
  // 80 MHz APB / (20 kHz * 2^11) leaves a valid LEDC divider. 12-bit at
  // 20 kHz requires a divider below the ESP32 hardware minimum.
  static constexpr uint8_t kPwmResolutionBits = 11;
  static constexpr uint16_t kPwmMaximum = (1U << kPwmResolutionBits) - 1U;
  static constexpr uint32_t kLedcClockHz = 80000000U;
  static_assert(kLedcClockHz >=
                    static_cast<uint32_t>(kPwmFrequencyHz) * (1UL << kPwmResolutionBits),
                "PWM frequency/resolution requires an invalid sub-unity LEDC divider");
};

}  // namespace mm
