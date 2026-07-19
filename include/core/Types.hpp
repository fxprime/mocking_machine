#pragma once

#include <array>
#include <cstdint>

namespace mm {

enum class StopMode : uint8_t { Coast = 0, BrakeToGround = 1, BrakeToSupply = 2 };
enum class RunState : uint8_t { Disarmed = 0, Armed = 1, Running = 2, Fault = 3 };
enum class ProfileKind : uint8_t { Ramp = 0, Sine = 1, Waypoints = 2 };

enum Fault : uint32_t {
  FaultNone = 0,
  FaultControlOverrun = 1U << 0U,
  FaultDriverDiagnostic = 1U << 1U,
  FaultOverCurrent = 1U << 2U,
  FaultEncoderTimeout = 1U << 3U,
  FaultInvalidConfiguration = 1U << 4U,
  FaultUnderVoltage = 1U << 5U,
  FaultOverVoltage = 1U << 6U,
};

struct PinConfiguration {
  uint8_t encoder_a = 32;
  uint8_t encoder_b = 33;
  uint8_t zero_index = 13;
  uint8_t motor_ina = 25;
  uint8_t motor_inb = 26;
  uint8_t motor_pwm = 27;
  uint8_t current_sense = 34;
  uint8_t driver_diag = 35;
  uint8_t supply_voltage_sense = 36;
};

struct ControlConfiguration {
  float kp = 0.12F;
  float ki = 0.8F;
  float kd = 0.0F;
  float output_min = -1.0F;
  float output_max = 1.0F;
  float error_deadband_rad_s = 0.2F;
  float velocity_filter_tau_s = 0.025F;
  uint32_t period_us = 2000;
  float max_feedback_correction = 0.10F;
};

struct MotorCharacteristics {
  float start_duty_forward = 0.18F;
  float start_duty_reverse = 0.18F;
  float max_velocity_forward_rad_s = 166.5F;
  float max_velocity_reverse_rad_s = 166.5F;
  float current_gain_a_per_v = 6.5F;
  float current_offset_v = 0.0F;
  float current_filter_cutoff_hz = 20.0F;
};

struct VoltageSenseConfiguration {
  static constexpr float kTopResistorOhm = 6800.0F;
  static constexpr float kBottomResistorOhm = 1000.0F;
  static constexpr float kNominalDividerGain =
      (kTopResistorOhm + kBottomResistorOhm) / kBottomResistorOhm;
  // Keep calibration below the nonlinear/saturated top of the ESP32 ADC range.
  static constexpr float kMaximumCalibrationSenseV = 2.8F;

  float divider_gain = kNominalDividerGain;
  float input_offset_v = 0.0F;
};

struct SafetyConfiguration {
  float max_velocity_rad_s = 150.0F;
  float max_acceleration_rad_s2 = 120.0F;
  float max_jerk_rad_s3 = 800.0F;
  float max_current_a = 5.0F;
  float min_supply_voltage_v = 5.5F;
  float max_supply_voltage_v = 16.0F;
  float max_duty = 0.90F;
  uint32_t command_timeout_ms = 1000;
  uint32_t encoder_timeout_ms = 250;
  float encoder_timeout_velocity_rad_s = 1.0F;
  bool current_sense_enabled = false;
  bool driver_diagnostic_enabled = false;
};

struct SerialConfiguration {
  uint32_t baud = 115200;
  uint16_t stream_rate_hz = 100;
};

struct EncoderConfiguration {
  uint32_t counts_per_output_revolution = 184;
  int8_t direction = 1;
  uint8_t estimator_min_counts = 4;
  uint32_t estimator_max_window_us = 20000;
  uint32_t estimator_stale_timeout_us = 100000;
};

struct CharacterizationConfiguration {
  float duty_step = 0.01F;
  float motion_threshold_rad_s = 1.0F;
  uint16_t settle_ms = 300;
  uint16_t reversal_pause_ms = 1000;
  uint16_t maximum_hold_ms = 2000;
  uint8_t consecutive_motion_samples = 2;
};

struct LoadEntry {
  uint8_t load_id = 0;
  uint16_t position_deg = 0;
  float strength = 0.0F;
};

constexpr size_t kMaximumLoads = 8;
struct MachineLoadSetting {
  uint8_t setting_id = 0;
  uint8_t count = 0;
  std::array<LoadEntry, kMaximumLoads> loads{};
};

struct ProfilePoint {
  uint32_t time_ms = 0;
  float velocity_rad_s = 0.0F;
};

constexpr size_t kMaximumProfilePoints = 16;
constexpr size_t kMaximumProfiles = 8;
struct VelocityProfileConfiguration {
  uint16_t profile_id = 0;
  ProfileKind kind = ProfileKind::Ramp;
  char name[16] = "default";
  float target_velocity_rad_s = 50.0F;
  float sine_mean_rad_s = 50.0F;
  float sine_amplitude_rad_s = 10.0F;
  float sine_frequency_hz = 1.0F;
  uint32_t duration_ms = 10000;
  uint8_t point_count = 0;
  std::array<ProfilePoint, kMaximumProfilePoints> points{};
};

struct MachineSettings {
  static constexpr uint32_t kSchemaVersion = 7;
  uint32_t schema_version = kSchemaVersion;
  PinConfiguration pins{};
  ControlConfiguration control{};
  MotorCharacteristics motor{};
  VoltageSenseConfiguration supply_voltage{};
  SafetyConfiguration safety{};
  SerialConfiguration serial{};
  EncoderConfiguration encoder{};
  CharacterizationConfiguration characterization{};
  MachineLoadSetting load_setting{};
  uint8_t profile_count = 1;
  uint16_t selected_profile_id = 0;
  std::array<VelocityProfileConfiguration, kMaximumProfiles> profiles{};
  int8_t motor_direction = 1;
  StopMode stop_mode = StopMode::BrakeToGround;
};

struct TelemetrySample {
  uint64_t timestamp_us = 0;
  uint64_t last_zero_timestamp_us = 0;
  int64_t encoder_count = 0;
  int64_t last_zero_encoder_count = 0;
  float desired_velocity_rad_s = 0.0F;
  float measured_velocity_rad_s = 0.0F;
  float controller_output = 0.0F;
  float controller_proportional_term = 0.0F;
  float controller_integral_term = 0.0F;
  float controller_derivative_term = 0.0F;
  float current_a = 0.0F;
  float supply_voltage_v = 0.0F;
  uint32_t faults = FaultNone;
  uint16_t profile_id = 0;
  uint8_t load_setting_id = 0;
  RunState state = RunState::Disarmed;
};

}  // namespace mm
