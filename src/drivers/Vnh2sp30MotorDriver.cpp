#include "drivers/Vnh2sp30MotorDriver.hpp"

#include <algorithm>
#include <cmath>

#include "control/MotorDeadband.hpp"

namespace mm {

bool Vnh2sp30MotorDriver::begin(const MachineSettings& settings) {
  pins_ = settings.pins;
  characteristics_ = settings.motor;
  supply_voltage_ = settings.supply_voltage;
  safety_ = settings.safety;
  stop_mode_ = settings.stop_mode;
  motor_direction_ = settings.motor_direction >= 0 ? 1 : -1;
  diagnostic_enabled_ = settings.safety.driver_diagnostic_enabled;

  pinMode(pins_.motor_ina, OUTPUT);
  pinMode(pins_.motor_inb, OUTPUT);
  if (diagnostic_enabled_) {
    pinMode(pins_.driver_diag, INPUT);  // External 5 V-to-3.3 V protection is required.
  }
  pinMode(pins_.current_sense, INPUT);
  pinMode(pins_.supply_voltage_sense, INPUT);
  analogSetPinAttenuation(pins_.current_sense, ADC_11db);
  analogSetPinAttenuation(pins_.supply_voltage_sense, ADC_11db);
  if (ledcSetup(kPwmChannel, kPwmFrequencyHz, kPwmResolutionBits) == 0.0) {
    return false;
  }
  ledcAttachPin(pins_.motor_pwm, kPwmChannel);
  stop();
  return true;
}

void Vnh2sp30MotorDriver::writeDirection(const bool forward) {
  digitalWrite(pins_.motor_ina, forward ? HIGH : LOW);
  digitalWrite(pins_.motor_inb, forward ? LOW : HIGH);
}

void Vnh2sp30MotorDriver::command(float signed_duty) {
  signed_duty =
      compensateMotorDeadband(signed_duty, characteristics_, safety_);
  if (signed_duty == 0.0F) {
    stop();
    return;
  }

  const bool logical_forward = signed_duty > 0.0F;
  const bool electrical_forward = motor_direction_ > 0 ? logical_forward : !logical_forward;
  writeDirection(electrical_forward);

  const float magnitude = std::fabs(signed_duty);
  const auto pwm = static_cast<uint32_t>(
      std::lround(magnitude * kPwmMaximum));
  ledcWrite(kPwmChannel, pwm);
  applied_duty_ = signed_duty;
}

void Vnh2sp30MotorDriver::commandRaw(float signed_duty) {
  signed_duty = std::clamp(signed_duty, -safety_.max_duty, safety_.max_duty);
  if (std::fabs(signed_duty) < 0.0001F) {
    stop();
    return;
  }

  const bool logical_forward = signed_duty > 0.0F;
  const bool electrical_forward = motor_direction_ > 0 ? logical_forward : !logical_forward;
  writeDirection(electrical_forward);
  const float magnitude = std::fabs(signed_duty);
  const auto pwm = static_cast<uint32_t>(std::lround(magnitude * kPwmMaximum));
  ledcWrite(kPwmChannel, pwm);
  applied_duty_ = logical_forward ? magnitude : -magnitude;
}

void Vnh2sp30MotorDriver::stop() {
  switch (stop_mode_) {
    case StopMode::Coast:
      ledcWrite(kPwmChannel, 0U);
      digitalWrite(pins_.motor_ina, LOW);
      digitalWrite(pins_.motor_inb, LOW);
      break;
    case StopMode::BrakeToGround:
      digitalWrite(pins_.motor_ina, LOW);
      digitalWrite(pins_.motor_inb, LOW);
      ledcWrite(kPwmChannel, kPwmMaximum);
      break;
    case StopMode::BrakeToSupply:
      digitalWrite(pins_.motor_ina, HIGH);
      digitalWrite(pins_.motor_inb, HIGH);
      ledcWrite(kPwmChannel, kPwmMaximum);
      break;
  }
  applied_duty_ = 0.0F;
}

void Vnh2sp30MotorDriver::disable() {
  ledcWrite(kPwmChannel, 0U);
  digitalWrite(pins_.motor_ina, LOW);
  digitalWrite(pins_.motor_inb, LOW);
  applied_duty_ = 0.0F;
}

float Vnh2sp30MotorDriver::currentAmperes() const {
  return currentAmperesFromVoltage(currentSenseVoltage());
}

float Vnh2sp30MotorDriver::currentAmperesFromVoltage(
    const float sense_voltage_v) const {
  return std::max(0.0F, (sense_voltage_v - characteristics_.current_offset_v) *
                            characteristics_.current_gain_a_per_v);
}

float Vnh2sp30MotorDriver::currentSenseVoltage(uint16_t sample_count) const {
  sample_count = std::max<uint16_t>(sample_count, 1U);
  uint32_t millivolt_sum = 0U;
  for (uint16_t sample = 0; sample < sample_count; ++sample) {
    millivolt_sum += analogReadMilliVolts(pins_.current_sense);
  }
  return static_cast<float>(millivolt_sum) /
         (1000.0F * static_cast<float>(sample_count));
}

float Vnh2sp30MotorDriver::supplySenseVoltage(uint16_t sample_count) const {
  sample_count = std::max<uint16_t>(sample_count, 1U);
  uint32_t millivolt_sum = 0U;
  for (uint16_t sample = 0; sample < sample_count; ++sample) {
    millivolt_sum += analogReadMilliVolts(pins_.supply_voltage_sense);
  }
  return static_cast<float>(millivolt_sum) /
         (1000.0F * static_cast<float>(sample_count));
}

float Vnh2sp30MotorDriver::supplyVoltage() const {
  return std::max(0.0F, supplySenseVoltage() * supply_voltage_.divider_gain +
                            supply_voltage_.input_offset_v);
}

void Vnh2sp30MotorDriver::setCurrentCalibration(const float gain_a_per_v,
                                                const float offset_v) {
  characteristics_.current_gain_a_per_v = gain_a_per_v;
  characteristics_.current_offset_v = offset_v;
}

void Vnh2sp30MotorDriver::setCharacteristics(
    const MotorCharacteristics& characteristics) {
  characteristics_ = characteristics;
}

void Vnh2sp30MotorDriver::setSupplyVoltageCalibration(const float divider_gain,
                                                      const float input_offset_v) {
  supply_voltage_.divider_gain = divider_gain;
  supply_voltage_.input_offset_v = input_offset_v;
}

void Vnh2sp30MotorDriver::setDiagnosticEnabled(const bool enabled) {
  diagnostic_enabled_ = enabled;
  if (enabled) {
    pinMode(pins_.driver_diag, INPUT);
  }
}

bool Vnh2sp30MotorDriver::diagnosticFault() const {
  return diagnostic_enabled_ && digitalRead(pins_.driver_diag) == LOW;
}

}  // namespace mm
