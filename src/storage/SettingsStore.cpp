#include "storage/SettingsStore.hpp"

#include <Preferences.h>

#include <cstddef>
#include <cmath>
#include <cstring>

#include "protocol/Crc16.hpp"

namespace mm {
namespace {
constexpr uint32_t kSettingsMagic = 0x4D4D4346U;  // MMCF
constexpr char kNamespace[] = "mmachine";
constexpr char kBlobKey[] = "settings";

struct LegacyMachineLoadSettingV11 {
  uint8_t setting_id;
  uint8_t count;
  std::array<LoadEntry, 8> loads;
};

struct LegacyCharacterizationConfigurationV12 {
  float duty_step;
  float motion_threshold_rad_s;
  uint16_t settle_ms;
  uint16_t reversal_pause_ms;
  uint16_t maximum_hold_ms;
  uint8_t consecutive_motion_samples;
};

struct PersistedSettings {
  uint32_t magic = kSettingsMagic;
  uint32_t schema_version = MachineSettings::kSchemaVersion;
  uint32_t payload_size = sizeof(MachineSettings);
  MachineSettings payload{};
  uint16_t crc = 0;
};

static_assert(sizeof(SafetyConfiguration) == 44U &&
                  offsetof(SafetyConfiguration, jerk_limit_enabled) == 42U,
              "Schema-24 jerk flag must occupy schema-23 safety padding");

struct LegacyControlConfigurationV5 {
  float kp;
  float ki;
  float kd;
  float output_min;
  float output_max;
  float error_deadband_rad_s;
  float velocity_filter_tau_s;
  uint32_t period_us;
};

struct LegacyEncoderConfigurationV5 {
  uint32_t counts_per_output_revolution;
  int8_t direction;
  uint8_t estimator_min_counts;
  uint32_t estimator_max_window_us;
};

struct LegacyEncoderConfigurationV7 {
  uint32_t counts_per_output_revolution;
  int8_t direction;
  uint8_t estimator_min_counts;
  uint32_t estimator_max_window_us;
  uint32_t estimator_stale_timeout_us;
};

struct LegacyEncoderConfigurationV8 {
  uint32_t counts_per_output_revolution;
  int8_t direction;
  uint8_t estimator_min_counts;
  uint32_t estimator_max_window_us;
  uint32_t estimator_stale_timeout_us;
  uint32_t zero_index_min_interval_us;
};

struct LegacyEncoderConfigurationV9 {
  uint32_t counts_per_output_revolution;
  int8_t direction;
  uint8_t estimator_min_counts;
  uint32_t estimator_max_window_us;
  uint32_t estimator_stale_timeout_us;
  uint32_t zero_index_min_interval_us;
  float zero_index_correction_gain;
};

struct LegacyEncoderConfigurationV16 {
  uint32_t counts_per_output_revolution;
  int8_t direction;
  uint8_t estimator_min_counts;
  uint32_t estimator_max_window_us;
  uint32_t estimator_stale_timeout_us;
  uint32_t zero_index_min_interval_us;
  float zero_index_correction_gain;
  float zero_index_minimum_separation_revolutions;
};

struct LegacyMotorCharacteristicsV6 {
  float start_duty_forward;
  float start_duty_reverse;
  float max_velocity_forward_rad_s;
  float max_velocity_reverse_rad_s;
  float current_gain_a_per_v;
  float current_offset_v;
};

struct LegacySafetyConfigurationV4 {
  float max_velocity_rad_s;
  float max_acceleration_rad_s2;
  float max_jerk_rad_s3;
  float max_current_a;
  float min_supply_voltage_v;
  float max_supply_voltage_v;
  float max_duty;
  uint32_t command_timeout_ms;
  uint32_t encoder_timeout_ms;
  bool current_sense_enabled;
  bool driver_diagnostic_enabled;
};

struct LegacyMachineSettingsV4 {
  uint32_t schema_version;
  PinConfiguration pins;
  LegacyControlConfigurationV5 control;
  LegacyMotorCharacteristicsV6 motor;
  VoltageSenseConfiguration supply_voltage;
  LegacySafetyConfigurationV4 safety;
  SerialConfiguration serial;
  LegacyEncoderConfigurationV5 encoder;
  LegacyCharacterizationConfigurationV12 characterization;
  LegacyMachineLoadSettingV11 load_setting;
  uint8_t profile_count;
  uint16_t selected_profile_id;
  std::array<VelocityProfileConfiguration, kMaximumProfiles> profiles;
  int8_t motor_direction;
  StopMode stop_mode;
};

struct LegacyPersistedSettingsV4 {
  uint32_t magic;
  uint32_t schema_version;
  uint32_t payload_size;
  LegacyMachineSettingsV4 payload;
  uint16_t crc;
};

struct LegacyMachineSettingsV5 {
  uint32_t schema_version;
  PinConfiguration pins;
  LegacyControlConfigurationV5 control;
  LegacyMotorCharacteristicsV6 motor;
  VoltageSenseConfiguration supply_voltage;
  SafetyConfiguration safety;
  SerialConfiguration serial;
  LegacyEncoderConfigurationV5 encoder;
  LegacyCharacterizationConfigurationV12 characterization;
  LegacyMachineLoadSettingV11 load_setting;
  uint8_t profile_count;
  uint16_t selected_profile_id;
  std::array<VelocityProfileConfiguration, kMaximumProfiles> profiles;
  int8_t motor_direction;
  StopMode stop_mode;
};

struct LegacyPersistedSettingsV5 {
  uint32_t magic;
  uint32_t schema_version;
  uint32_t payload_size;
  LegacyMachineSettingsV5 payload;
  uint16_t crc;
};

struct LegacyMachineSettingsV6 {
  uint32_t schema_version;
  PinConfiguration pins;
  ControlConfiguration control;
  LegacyMotorCharacteristicsV6 motor;
  VoltageSenseConfiguration supply_voltage;
  SafetyConfiguration safety;
  SerialConfiguration serial;
  LegacyEncoderConfigurationV7 encoder;
  LegacyCharacterizationConfigurationV12 characterization;
  LegacyMachineLoadSettingV11 load_setting;
  uint8_t profile_count;
  uint16_t selected_profile_id;
  std::array<VelocityProfileConfiguration, kMaximumProfiles> profiles;
  int8_t motor_direction;
  StopMode stop_mode;
};

struct LegacyPersistedSettingsV6 {
  uint32_t magic;
  uint32_t schema_version;
  uint32_t payload_size;
  LegacyMachineSettingsV6 payload;
  uint16_t crc;
};

struct LegacyMachineSettingsV7 {
  uint32_t schema_version;
  PinConfiguration pins;
  ControlConfiguration control;
  MotorCharacteristics motor;
  VoltageSenseConfiguration supply_voltage;
  SafetyConfiguration safety;
  SerialConfiguration serial;
  LegacyEncoderConfigurationV7 encoder;
  LegacyCharacterizationConfigurationV12 characterization;
  LegacyMachineLoadSettingV11 load_setting;
  uint8_t profile_count;
  uint16_t selected_profile_id;
  std::array<VelocityProfileConfiguration, kMaximumProfiles> profiles;
  int8_t motor_direction;
  StopMode stop_mode;
};

struct LegacyPersistedSettingsV7 {
  uint32_t magic;
  uint32_t schema_version;
  uint32_t payload_size;
  LegacyMachineSettingsV7 payload;
  uint16_t crc;
};

struct LegacyMachineSettingsV8 {
  uint32_t schema_version;
  PinConfiguration pins;
  ControlConfiguration control;
  MotorCharacteristics motor;
  VoltageSenseConfiguration supply_voltage;
  SafetyConfiguration safety;
  SerialConfiguration serial;
  LegacyEncoderConfigurationV8 encoder;
  LegacyCharacterizationConfigurationV12 characterization;
  LegacyMachineLoadSettingV11 load_setting;
  uint8_t profile_count;
  uint16_t selected_profile_id;
  std::array<VelocityProfileConfiguration, kMaximumProfiles> profiles;
  int8_t motor_direction;
  StopMode stop_mode;
};

struct LegacyPersistedSettingsV8 {
  uint32_t magic;
  uint32_t schema_version;
  uint32_t payload_size;
  LegacyMachineSettingsV8 payload;
  uint16_t crc;
};

struct LegacyMachineSettingsV9 {
  uint32_t schema_version;
  PinConfiguration pins;
  ControlConfiguration control;
  MotorCharacteristics motor;
  VoltageSenseConfiguration supply_voltage;
  SafetyConfiguration safety;
  SerialConfiguration serial;
  LegacyEncoderConfigurationV9 encoder;
  LegacyCharacterizationConfigurationV12 characterization;
  LegacyMachineLoadSettingV11 load_setting;
  uint8_t profile_count;
  uint16_t selected_profile_id;
  std::array<VelocityProfileConfiguration, kMaximumProfiles> profiles;
  int8_t motor_direction;
  StopMode stop_mode;
};

struct LegacyPersistedSettingsV9 {
  uint32_t magic;
  uint32_t schema_version;
  uint32_t payload_size;
  LegacyMachineSettingsV9 payload;
  uint16_t crc;
};

struct LegacyMachineSettingsV11 {
  uint32_t schema_version;
  PinConfiguration pins;
  ControlConfiguration control;
  MotorCharacteristics motor;
  VoltageSenseConfiguration supply_voltage;
  SafetyConfiguration safety;
  SerialConfiguration serial;
  LegacyEncoderConfigurationV16 encoder;
  LegacyCharacterizationConfigurationV12 characterization;
  LegacyMachineLoadSettingV11 load_setting;
  uint8_t profile_count;
  uint16_t selected_profile_id;
  std::array<VelocityProfileConfiguration, kMaximumProfiles> profiles;
  int8_t motor_direction;
  StopMode stop_mode;
};

struct LegacyPersistedSettingsV11 {
  uint32_t magic;
  uint32_t schema_version;
  uint32_t payload_size;
  LegacyMachineSettingsV11 payload;
  uint16_t crc;
};

struct LegacyMachineSettingsV12 {
  uint32_t schema_version;
  PinConfiguration pins;
  ControlConfiguration control;
  MotorCharacteristics motor;
  VoltageSenseConfiguration supply_voltage;
  SafetyConfiguration safety;
  SerialConfiguration serial;
  LegacyEncoderConfigurationV16 encoder;
  LegacyCharacterizationConfigurationV12 characterization;
  MachineLoadSetting load_setting;
  uint8_t profile_count;
  uint16_t selected_profile_id;
  std::array<VelocityProfileConfiguration, kMaximumProfiles> profiles;
  int8_t motor_direction;
  StopMode stop_mode;
};

struct LegacyPersistedSettingsV12 {
  uint32_t magic;
  uint32_t schema_version;
  uint32_t payload_size;
  LegacyMachineSettingsV12 payload;
  uint16_t crc;
};

struct LegacyMachineSettingsV13 {
  uint32_t schema_version;
  PinConfiguration pins;
  ControlConfiguration control;
  MotorCharacteristics motor;
  VoltageSenseConfiguration supply_voltage;
  SafetyConfiguration safety;
  SerialConfiguration serial;
  LegacyEncoderConfigurationV16 encoder;
  CharacterizationConfiguration characterization;
  MachineLoadSetting load_setting;
  uint8_t profile_count;
  uint16_t selected_profile_id;
  std::array<VelocityProfileConfiguration, kMaximumProfiles> profiles;
  int8_t motor_direction;
  StopMode stop_mode;
};

struct LegacyPersistedSettingsV13 {
  uint32_t magic;
  uint32_t schema_version;
  uint32_t payload_size;
  LegacyMachineSettingsV13 payload;
  uint16_t crc;
};

struct LegacyMachineSettingsV14 {
  uint32_t schema_version;
  PinConfiguration pins;
  ControlConfiguration control;
  MotorCharacteristics motor;
  VoltageSenseConfiguration supply_voltage;
  SafetyConfiguration safety;
  SerialConfiguration serial;
  LegacyEncoderConfigurationV16 encoder;
  CharacterizationConfiguration characterization;
  MachineLoadSetting load_setting;
  uint8_t profile_count;
  uint16_t selected_profile_id;
  std::array<VelocityProfileConfiguration, kMaximumProfiles> profiles;
  int8_t motor_direction;
  StopMode stop_mode;
  MotorModelConfiguration motor_model;
};

struct LegacyPersistedSettingsV14 {
  uint32_t magic;
  uint32_t schema_version;
  uint32_t payload_size;
  LegacyMachineSettingsV14 payload;
  uint16_t crc;
};

struct LegacyMachineSettingsV15 {
  uint32_t schema_version;
  PinConfiguration pins;
  ControlConfiguration control;
  MotorCharacteristics motor;
  VoltageSenseConfiguration supply_voltage;
  SafetyConfiguration safety;
  SerialConfiguration serial;
  LegacyEncoderConfigurationV16 encoder;
  CharacterizationConfiguration characterization;
  MachineLoadSetting load_setting;
  uint8_t profile_count;
  uint16_t selected_profile_id;
  std::array<VelocityProfileConfiguration, kMaximumProfiles> profiles;
  int8_t motor_direction;
  StopMode stop_mode;
  MotorModelConfiguration motor_model;
  VelocityEstimatorMethod velocity_estimator_method;
};

struct LegacyPersistedSettingsV15 {
  uint32_t magic;
  uint32_t schema_version;
  uint32_t payload_size;
  LegacyMachineSettingsV15 payload;
  uint16_t crc;
};

struct LegacyMachineSettingsV16 {
  uint32_t schema_version;
  PinConfiguration pins;
  ControlConfiguration control;
  MotorCharacteristics motor;
  VoltageSenseConfiguration supply_voltage;
  SafetyConfiguration safety;
  SerialConfiguration serial;
  LegacyEncoderConfigurationV16 encoder;
  CharacterizationConfiguration characterization;
  MachineLoadSetting load_setting;
  uint8_t profile_count;
  uint16_t selected_profile_id;
  std::array<VelocityProfileConfiguration, kMaximumProfiles> profiles;
  int8_t motor_direction;
  StopMode stop_mode;
  MotorModelConfiguration motor_model;
  VelocityEstimatorMethod velocity_estimator_method;
  uint32_t velocity_acceleration_window_samples;
};

struct LegacyPersistedSettingsV16 {
  uint32_t magic;
  uint32_t schema_version;
  uint32_t payload_size;
  LegacyMachineSettingsV16 payload;
  uint16_t crc;
};

struct LegacyEncoderConfigurationV21 {
  uint32_t counts_per_output_revolution;
  int8_t direction;
  uint8_t estimator_min_counts;
  uint32_t estimator_max_window_us;
  uint32_t estimator_stale_timeout_us;
  uint32_t zero_index_min_interval_us;
  float zero_index_correction_gain;
  float zero_index_minimum_separation_revolutions;
  uint32_t zero_position_offset_ticks;
};

struct LegacyMachineSettingsV21 {
  uint32_t schema_version;
  PinConfiguration pins;
  ControlConfiguration control;
  MotorCharacteristics motor;
  VoltageSenseConfiguration supply_voltage;
  SafetyConfiguration safety;
  SerialConfiguration serial;
  LegacyEncoderConfigurationV21 encoder;
  CharacterizationConfiguration characterization;
  MachineLoadSetting load_setting;
  uint8_t profile_count;
  uint16_t selected_profile_id;
  std::array<VelocityProfileConfiguration, kMaximumProfiles> profiles;
  int8_t motor_direction;
  StopMode stop_mode;
  MotorModelConfiguration motor_model;
  VelocityEstimatorMethod velocity_estimator_method;
  uint32_t velocity_acceleration_window_samples;
};

struct LegacyPersistedSettingsV21 {
  uint32_t magic;
  uint32_t schema_version;
  uint32_t payload_size;
  LegacyMachineSettingsV21 payload;
  uint16_t crc;
};

struct LegacyEncoderConfigurationV22 {
  uint32_t counts_per_output_revolution;
  int8_t direction;
  uint8_t estimator_min_counts;
  uint32_t estimator_max_window_us;
  uint32_t estimator_stale_timeout_us;
  uint32_t zero_index_min_interval_us;
  float zero_index_correction_gain;
  float zero_index_minimum_separation_revolutions;
  uint32_t zero_position_offset_ticks;
  uint8_t zero_index_reference_side;
  uint8_t zero_index_hysteresis_calibrated;
  int32_t clockwise_rising_correction_ticks;
  int32_t clockwise_falling_correction_ticks;
  float zero_index_calibration_duty;
  uint32_t zero_index_calibration_timeout_ms;
  uint16_t zero_index_calibration_reversal_pause_ms;
  uint16_t zero_index_calibration_maximum_error_ticks;
};

struct LegacyMachineSettingsV22 {
  uint32_t schema_version;
  PinConfiguration pins;
  ControlConfiguration control;
  MotorCharacteristics motor;
  VoltageSenseConfiguration supply_voltage;
  SafetyConfiguration safety;
  SerialConfiguration serial;
  LegacyEncoderConfigurationV22 encoder;
  CharacterizationConfiguration characterization;
  MachineLoadSetting load_setting;
  uint8_t profile_count;
  uint16_t selected_profile_id;
  std::array<VelocityProfileConfiguration, kMaximumProfiles> profiles;
  int8_t motor_direction;
  StopMode stop_mode;
  MotorModelConfiguration motor_model;
  VelocityEstimatorMethod velocity_estimator_method;
  uint32_t velocity_acceleration_window_samples;
};

struct LegacyPersistedSettingsV22 {
  uint32_t magic;
  uint32_t schema_version;
  uint32_t payload_size;
  LegacyMachineSettingsV22 payload;
  uint16_t crc;
};

static_assert(sizeof(LegacyMachineSettingsV4) + sizeof(float) ==
                  sizeof(LegacyMachineSettingsV5),
              "Schema-v4 to v5 migration layout assumption changed");
static_assert(sizeof(LegacyMachineSettingsV5) + sizeof(float) + sizeof(uint32_t) ==
                  sizeof(LegacyMachineSettingsV6),
              "Schema-v5 to v6 migration layout assumption changed");
static_assert(sizeof(LegacyMachineSettingsV6) + sizeof(float) ==
                  sizeof(LegacyMachineSettingsV7),
              "Schema-v6 to v7 migration layout assumption changed");
static_assert(sizeof(LegacyMachineSettingsV7) + sizeof(uint32_t) ==
                  sizeof(LegacyMachineSettingsV8),
              "Schema-v7 to v8 migration layout assumption changed");
static_assert(sizeof(LegacyMachineSettingsV8) + sizeof(float) ==
                  sizeof(LegacyMachineSettingsV9),
              "Schema-v8 to v9 migration layout assumption changed");
static_assert(sizeof(LegacyMachineSettingsV9) + sizeof(float) ==
                  sizeof(LegacyMachineSettingsV11),
              "Schema-v9 to v10 migration layout assumption changed");
static_assert(sizeof(LegacyMachineSettingsV11) + 4U * sizeof(LoadEntry) ==
                  sizeof(LegacyMachineSettingsV12),
              "Schema-v11 to v12 migration layout assumption changed");
static_assert(sizeof(LegacyMachineSettingsV12) + 3U * sizeof(float) ==
                  sizeof(LegacyMachineSettingsV13),
              "Schema-v12 to v13 migration layout assumption changed");
static_assert(sizeof(LegacyMachineSettingsV13) + sizeof(MotorModelConfiguration) ==
                  sizeof(LegacyMachineSettingsV14),
              "Schema-v13 to v14 migration layout assumption changed");
static_assert(sizeof(LegacyMachineSettingsV14) + sizeof(uint32_t) ==
                  sizeof(LegacyMachineSettingsV15),
              "Schema-v14 to v15 migration layout assumption changed");
static_assert(sizeof(LegacyMachineSettingsV15) + sizeof(uint32_t) ==
                  sizeof(LegacyMachineSettingsV16),
              "Schema-v15 to v16 migration layout assumption changed");
static_assert(sizeof(LegacyMachineSettingsV16) + sizeof(float) ==
                  sizeof(LegacyMachineSettingsV21),
              "Schema-v16 to v17 migration layout assumption changed");

void copyLegacyControl(const LegacyControlConfigurationV5& source,
                       ControlConfiguration& target) {
  target.kp = source.kp;
  target.ki = source.ki;
  target.kd = source.kd;
  target.output_min = source.output_min;
  target.output_max = source.output_max;
  target.error_deadband_rad_s = source.error_deadband_rad_s;
  target.velocity_filter_tau_s = source.velocity_filter_tau_s;
  target.period_us = source.period_us;
}

void copyLegacyEncoder(const LegacyEncoderConfigurationV5& source,
                       EncoderConfiguration& target) {
  target.counts_per_output_revolution = source.counts_per_output_revolution;
  target.direction = source.direction;
  target.estimator_min_counts = source.estimator_min_counts;
  target.estimator_max_window_us = source.estimator_max_window_us;
}

void copyLegacyEncoder(const LegacyEncoderConfigurationV7& source,
                       EncoderConfiguration& target) {
  target.counts_per_output_revolution = source.counts_per_output_revolution;
  target.direction = source.direction;
  target.estimator_min_counts = source.estimator_min_counts;
  target.estimator_max_window_us = source.estimator_max_window_us;
  target.estimator_stale_timeout_us = source.estimator_stale_timeout_us;
}

void copyLegacyEncoder(const LegacyEncoderConfigurationV8& source,
                       EncoderConfiguration& target) {
  target.counts_per_output_revolution = source.counts_per_output_revolution;
  target.direction = source.direction;
  target.estimator_min_counts = source.estimator_min_counts;
  target.estimator_max_window_us = source.estimator_max_window_us;
  target.estimator_stale_timeout_us = source.estimator_stale_timeout_us;
  target.zero_index_min_interval_us = source.zero_index_min_interval_us;
}

void copyLegacyEncoder(const LegacyEncoderConfigurationV9& source,
                       EncoderConfiguration& target) {
  target.counts_per_output_revolution = source.counts_per_output_revolution;
  target.direction = source.direction;
  target.estimator_min_counts = source.estimator_min_counts;
  target.estimator_max_window_us = source.estimator_max_window_us;
  target.estimator_stale_timeout_us = source.estimator_stale_timeout_us;
  target.zero_index_min_interval_us = source.zero_index_min_interval_us;
  target.zero_index_correction_gain = source.zero_index_correction_gain;
}

void copyLegacyEncoder(const LegacyEncoderConfigurationV16& source,
                       EncoderConfiguration& target) {
  target.counts_per_output_revolution = source.counts_per_output_revolution;
  target.direction = source.direction;
  target.estimator_min_counts = source.estimator_min_counts;
  target.estimator_max_window_us = source.estimator_max_window_us;
  target.estimator_stale_timeout_us = source.estimator_stale_timeout_us;
  target.zero_index_min_interval_us = source.zero_index_min_interval_us;
  target.zero_index_correction_gain = source.zero_index_correction_gain;
  target.zero_index_minimum_separation_revolutions =
      source.zero_index_minimum_separation_revolutions;
}

void copyLegacyEncoder(const LegacyEncoderConfigurationV21& source,
                       EncoderConfiguration& target) {
  target.counts_per_output_revolution = source.counts_per_output_revolution;
  target.direction = source.direction;
  target.estimator_min_counts = source.estimator_min_counts;
  target.estimator_max_window_us = source.estimator_max_window_us;
  target.estimator_stale_timeout_us = source.estimator_stale_timeout_us;
  target.zero_index_min_interval_us = source.zero_index_min_interval_us;
  target.zero_index_correction_gain = source.zero_index_correction_gain;
  target.zero_index_minimum_separation_revolutions =
      source.zero_index_minimum_separation_revolutions;
  target.zero_position_offset_ticks = source.zero_position_offset_ticks;
}

void copyLegacyEncoder(const LegacyEncoderConfigurationV22& source,
                       EncoderConfiguration& target) {
  target.counts_per_output_revolution = source.counts_per_output_revolution;
  target.direction = source.direction;
  target.estimator_min_counts = source.estimator_min_counts;
  target.estimator_max_window_us = source.estimator_max_window_us;
  target.estimator_stale_timeout_us = source.estimator_stale_timeout_us;
  target.zero_index_min_interval_us = source.zero_index_min_interval_us;
  target.zero_index_correction_gain = source.zero_index_correction_gain;
  target.zero_index_minimum_separation_revolutions =
      source.zero_index_minimum_separation_revolutions;
  target.zero_position_offset_ticks = source.zero_position_offset_ticks;
  target.zero_index_reference_side = source.zero_index_reference_side;
  target.zero_index_hysteresis_calibrated =
      source.zero_index_hysteresis_calibrated;
  target.clockwise_rising_correction_ticks =
      source.clockwise_rising_correction_ticks;
  target.clockwise_falling_correction_ticks =
      source.clockwise_falling_correction_ticks;
  target.zero_index_calibration_duty =
      source.zero_index_calibration_duty;
  target.zero_index_calibration_timeout_ms =
      source.zero_index_calibration_timeout_ms;
  target.zero_index_calibration_reversal_pause_ms =
      source.zero_index_calibration_reversal_pause_ms;
  target.zero_index_calibration_maximum_error_ticks =
      source.zero_index_calibration_maximum_error_ticks;
}

void copyLegacyMotor(const LegacyMotorCharacteristicsV6& source,
                     MotorCharacteristics& target) {
  target.start_duty_forward = source.start_duty_forward;
  target.start_duty_reverse = source.start_duty_reverse;
  target.max_velocity_forward_rad_s = source.max_velocity_forward_rad_s;
  target.max_velocity_reverse_rad_s = source.max_velocity_reverse_rad_s;
  target.current_gain_a_per_v = source.current_gain_a_per_v;
  target.current_offset_v = source.current_offset_v;
}

void copyLegacyLoad(const LegacyMachineLoadSettingV11& source,
                    MachineLoadSetting& target) {
  target = {};
  target.setting_id = source.setting_id;
  const uint8_t source_count = source.count > source.loads.size()
                                   ? static_cast<uint8_t>(source.loads.size())
                                   : source.count;
  bool occupied[kRotorSlotCount]{};
  for (uint8_t index = 0; index < source_count; ++index) {
    const auto& legacy = source.loads[index];
    if (legacy.position_deg >= 360U || legacy.position_deg % 30U != 0U ||
        !std::isfinite(legacy.strength) || legacy.strength < 1.0F ||
        legacy.strength > 10.0F) {
      continue;
    }
    const uint8_t slot = static_cast<uint8_t>(legacy.position_deg / 30U);
    if (occupied[slot]) {
      continue;
    }
    occupied[slot] = true;
    target.loads[target.count++] = {slot, legacy.position_deg, legacy.strength};
  }
}

void copyLegacyCharacterization(
    const LegacyCharacterizationConfigurationV12& source,
    CharacterizationConfiguration& target) {
  target.duty_step = source.duty_step;
  target.motion_threshold_rad_s = source.motion_threshold_rad_s;
  target.settle_ms = source.settle_ms;
  target.reversal_pause_ms = source.reversal_pause_ms;
  target.maximum_hold_ms = source.maximum_hold_ms;
  target.consecutive_motion_samples = source.consecutive_motion_samples;
}
}

MachineSettings SettingsStore::defaults() {
  MachineSettings settings{};
  settings.profiles[0].profile_id = 0;
  return settings;
}

bool SettingsStore::validate(const MachineSettings& settings) {
  const auto finite = [](const float value) { return std::isfinite(value); };
  if (settings.schema_version != MachineSettings::kSchemaVersion ||
      settings.control.period_us < 1000U || settings.control.period_us > 20000U ||
      settings.encoder.counts_per_output_revolution == 0U ||
      (settings.encoder.direction != -1 && settings.encoder.direction != 1) ||
      (settings.motor_direction != -1 && settings.motor_direction != 1) ||
      settings.profile_count == 0U || settings.profile_count > kMaximumProfiles ||
      settings.load_setting.count > kMaximumLoads ||
      settings.load_setting.broken_bearing > 1U ||
      settings.serial.stream_rate_hz == 0U ||
      settings.serial.stream_rate_hz > 500U || settings.serial.baud < 9600U ||
      settings.serial.baud > 921600U ||
      static_cast<uint8_t>(settings.velocity_estimator_method) >
          static_cast<uint8_t>(VelocityEstimatorMethod::WindowedAccelerationPrediction) ||
      settings.velocity_acceleration_window_samples < 2U ||
      settings.velocity_acceleration_window_samples > 32U) {
    return false;
  }
  if (!finite(settings.control.kp) || !finite(settings.control.ki) ||
      !finite(settings.control.kd) || !finite(settings.safety.max_velocity_rad_s) ||
      !finite(settings.control.max_feedback_correction) ||
      !finite(settings.safety.max_acceleration_rad_s2) ||
      !finite(settings.safety.max_jerk_rad_s3) || !finite(settings.safety.max_current_a) ||
      !finite(settings.safety.max_duty) || !finite(settings.motor.start_duty_forward) ||
      !finite(settings.motor.start_duty_reverse) ||
      !finite(settings.motor.current_gain_a_per_v) ||
      !finite(settings.motor.current_offset_v) ||
      !finite(settings.motor.current_filter_cutoff_hz) ||
      !finite(settings.motor_model.velocity_gain_forward_rad_s_per_duty) ||
      !finite(settings.motor_model.velocity_gain_reverse_rad_s_per_duty) ||
      !finite(settings.motor_model.time_constant_forward_s) ||
      !finite(settings.motor_model.time_constant_reverse_s) ||
      !finite(settings.motor_model.velocity_process_noise_rad_s2) ||
      !finite(settings.motor_model.disturbance_process_noise_rad_s3) ||
      !finite(settings.motor_model.encoder_measurement_noise_counts) ||
      !finite(settings.safety.encoder_timeout_velocity_rad_s) ||
      !finite(settings.encoder.zero_index_correction_gain) ||
      !finite(settings.encoder.zero_index_minimum_separation_revolutions) ||
      !finite(settings.encoder.zero_index_calibration_duty) ||
      !finite(settings.encoder.zero_index_calibration_speed_rpm) ||
      !finite(settings.supply_voltage.divider_gain) ||
      !finite(settings.supply_voltage.input_offset_v) ||
      !finite(settings.safety.min_supply_voltage_v) ||
      !finite(settings.safety.max_supply_voltage_v) ||
      settings.safety.max_velocity_rad_s <= 0.0F ||
      settings.safety.max_acceleration_rad_s2 <= 0.0F ||
      settings.control.kp < 0.0F || settings.control.ki < 0.0F ||
      settings.control.kd < 0.0F ||
      settings.control.max_feedback_correction <= 0.0F ||
      settings.control.max_feedback_correction > 1.0F ||
      settings.control.output_min >= 0.0F || settings.control.output_max <= 0.0F ||
      settings.control.output_min >= settings.control.output_max ||
      settings.supply_voltage.divider_gain < 1.0F ||
      settings.supply_voltage.divider_gain > 20.0F ||
      settings.safety.min_supply_voltage_v < 0.0F ||
      settings.safety.min_supply_voltage_v >= settings.safety.max_supply_voltage_v ||
      settings.safety.max_supply_voltage_v > 20.0F ||
      settings.safety.max_jerk_rad_s3 <= 0.0F || settings.safety.max_duty <= 0.0F ||
      settings.safety.max_duty > 1.0F || settings.safety.max_current_a <= 0.0F ||
      settings.motor.start_duty_forward < 0.0F ||
      settings.motor.start_duty_forward > settings.safety.max_duty ||
      settings.motor.start_duty_reverse < 0.0F ||
      settings.motor.start_duty_reverse > settings.safety.max_duty ||
      settings.motor.current_gain_a_per_v <= 0.0F ||
      settings.motor.current_filter_cutoff_hz < 0.1F ||
      settings.motor.current_filter_cutoff_hz > 200.0F ||
      settings.motor_model.velocity_process_noise_rad_s2 <= 0.0F ||
      settings.motor_model.velocity_process_noise_rad_s2 > 10000.0F ||
      settings.motor_model.disturbance_process_noise_rad_s3 <= 0.0F ||
      settings.motor_model.disturbance_process_noise_rad_s3 > 100000.0F ||
      settings.motor_model.encoder_measurement_noise_counts <= 0.0F ||
      settings.motor_model.encoder_measurement_noise_counts > 10.0F ||
      settings.encoder.estimator_min_counts == 0U ||
      settings.encoder.estimator_min_counts > 32U ||
      settings.encoder.estimator_max_window_us < 1000U ||
      settings.encoder.estimator_max_window_us > 500000U ||
      settings.encoder.estimator_stale_timeout_us <
          settings.encoder.estimator_max_window_us ||
      settings.encoder.estimator_stale_timeout_us > 2000000U ||
      settings.encoder.zero_index_min_interval_us <
          EncoderConfiguration::kMinimumZeroIndexMinimumIntervalUs ||
      settings.encoder.zero_index_min_interval_us >
          EncoderConfiguration::kMaximumZeroIndexMinimumIntervalUs ||
      settings.encoder.zero_index_correction_gain <
          EncoderConfiguration::kMinimumZeroIndexCorrectionGain ||
      settings.encoder.zero_index_correction_gain >
          EncoderConfiguration::kMaximumZeroIndexCorrectionGain ||
      settings.encoder.zero_index_minimum_separation_revolutions <
          EncoderConfiguration::kMinimumZeroIndexMinimumSeparationRevolutions ||
      settings.encoder.zero_index_minimum_separation_revolutions >
          EncoderConfiguration::kMaximumZeroIndexMinimumSeparationRevolutions ||
      settings.encoder.zero_position_offset_ticks >=
          settings.encoder.counts_per_output_revolution ||
      settings.encoder.zero_index_reference_side > 1U ||
      settings.encoder.zero_index_hysteresis_calibrated > 1U ||
      std::abs(static_cast<int64_t>(
          settings.encoder.clockwise_rising_correction_ticks)) >
          static_cast<int64_t>(
              settings.encoder.counts_per_output_revolution / 2U) ||
      std::abs(static_cast<int64_t>(
          settings.encoder.clockwise_falling_correction_ticks)) >
          static_cast<int64_t>(
              settings.encoder.counts_per_output_revolution / 2U) ||
      settings.encoder.zero_index_calibration_duty <
          EncoderConfiguration::kMinimumZeroIndexCalibrationDuty ||
      settings.encoder.zero_index_calibration_duty > settings.safety.max_duty ||
      settings.encoder.zero_index_calibration_timeout_ms <
          EncoderConfiguration::kMinimumZeroIndexCalibrationTimeoutMs ||
      settings.encoder.zero_index_calibration_timeout_ms >
          EncoderConfiguration::kMaximumZeroIndexCalibrationTimeoutMs ||
      settings.encoder.zero_index_calibration_reversal_pause_ms <
          EncoderConfiguration::kMinimumZeroIndexCalibrationReversalPauseMs ||
      settings.encoder.zero_index_calibration_reversal_pause_ms >
          EncoderConfiguration::kMaximumZeroIndexCalibrationReversalPauseMs ||
      settings.encoder.zero_index_calibration_maximum_error_ticks >
          EncoderConfiguration::kMaximumZeroIndexCalibrationMaximumErrorTicks ||
      settings.encoder.zero_index_calibration_maximum_error_ticks >=
          settings.encoder.counts_per_output_revolution ||
      settings.encoder.zero_index_calibration_speed_rpm <
          EncoderConfiguration::kMinimumZeroIndexCalibrationSpeedRpm ||
      settings.encoder.zero_index_calibration_speed_rpm >
          EncoderConfiguration::kMaximumZeroIndexCalibrationSpeedRpm ||
      settings.encoder.zero_index_calibration_speed_rpm *
              EncoderConfiguration::kRadiansPerSecondPerRpm >
          settings.safety.max_velocity_rad_s ||
      settings.safety.encoder_timeout_ms < 50U ||
      settings.safety.encoder_timeout_ms > 10000U ||
      settings.safety.encoder_timeout_velocity_rad_s <= 0.0F ||
      settings.safety.encoder_timeout_velocity_rad_s > settings.safety.max_velocity_rad_s ||
      settings.characterization.duty_step <= 0.0F ||
      settings.characterization.duty_step > 0.1F ||
      settings.characterization.motion_threshold_rad_s <= 0.0F ||
      settings.characterization.settle_ms < 50U ||
      settings.characterization.reversal_pause_ms < 250U ||
      settings.characterization.maximum_hold_ms < 250U ||
      settings.characterization.consecutive_motion_samples == 0U ||
      !finite(settings.characterization.dynamics_filter_cutoff_hz) ||
      settings.characterization.dynamics_filter_cutoff_hz < 0.5F ||
      settings.characterization.dynamics_filter_cutoff_hz > 100.0F ||
      !finite(settings.characterization.dynamics_quantile) ||
      settings.characterization.dynamics_quantile < 0.80F ||
      settings.characterization.dynamics_quantile > 0.99F ||
      !finite(settings.characterization.recommendation_safety_factor) ||
      settings.characterization.recommendation_safety_factor < 0.10F ||
      settings.characterization.recommendation_safety_factor > 1.0F) {
    return false;
  }
  const bool motor_model_disabled =
      settings.motor_model.velocity_gain_forward_rad_s_per_duty == 0.0F &&
      settings.motor_model.velocity_gain_reverse_rad_s_per_duty == 0.0F &&
      settings.motor_model.time_constant_forward_s == 0.0F &&
      settings.motor_model.time_constant_reverse_s == 0.0F;
  const bool motor_model_valid =
      settings.motor_model.velocity_gain_forward_rad_s_per_duty >= 1.0F &&
      settings.motor_model.velocity_gain_forward_rad_s_per_duty <= 1000.0F &&
      settings.motor_model.velocity_gain_reverse_rad_s_per_duty >= 1.0F &&
      settings.motor_model.velocity_gain_reverse_rad_s_per_duty <= 1000.0F &&
      settings.motor_model.time_constant_forward_s >= 0.005F &&
      settings.motor_model.time_constant_forward_s <= 5.0F &&
      settings.motor_model.time_constant_reverse_s >= 0.005F &&
      settings.motor_model.time_constant_reverse_s <= 5.0F;
  if (!motor_model_disabled && !motor_model_valid) return false;
  if (settings.velocity_estimator_method == VelocityEstimatorMethod::Kalman &&
      !motor_model_valid) {
    return false;
  }
  bool occupied_slots[kRotorSlotCount]{};
  for (uint8_t index = 0; index < settings.load_setting.count; ++index) {
    const auto& load = settings.load_setting.loads[index];
    if (load.load_id >= kRotorSlotCount || occupied_slots[load.load_id] ||
        load.position_deg != static_cast<uint16_t>(load.load_id * 30U) ||
        !finite(load.strength) || load.strength < 1.0F || load.strength > 10.0F) {
      return false;
    }
    occupied_slots[load.load_id] = true;
  }
  bool selected_profile_found = false;
  for (uint8_t index = 0; index < settings.profile_count; ++index) {
    const auto& profile = settings.profiles[index];
    selected_profile_found |= profile.profile_id == settings.selected_profile_id;
    if (static_cast<uint8_t>(profile.kind) > static_cast<uint8_t>(ProfileKind::Waypoints) ||
        profile.duration_ms == 0U || profile.point_count > kMaximumProfilePoints ||
        !finite(profile.target_velocity_rad_s) || !finite(profile.sine_mean_rad_s) ||
        !finite(profile.sine_amplitude_rad_s) || !finite(profile.sine_frequency_hz) ||
        profile.target_velocity_rad_s < 0.0F ||
        profile.target_velocity_rad_s > settings.safety.max_velocity_rad_s ||
        profile.sine_mean_rad_s < std::fabs(profile.sine_amplitude_rad_s) ||
        profile.sine_mean_rad_s + std::fabs(profile.sine_amplitude_rad_s) >
            settings.safety.max_velocity_rad_s) {
      return false;
    }
    if (profile.kind == ProfileKind::Sine && profile.sine_frequency_hz <= 0.0F) {
      return false;
    }
    if (profile.kind == ProfileKind::Waypoints &&
        (profile.point_count < 2U || profile.points[0].time_ms != 0U ||
         profile.points[profile.point_count - 1U].time_ms != profile.duration_ms)) {
      return false;
    }
    if (profile.point_count > 0U &&
        (!finite(profile.points[0].velocity_rad_s) ||
         profile.points[0].velocity_rad_s < 0.0F ||
         profile.points[0].velocity_rad_s > settings.safety.max_velocity_rad_s)) {
      return false;
    }
    for (uint8_t point = 1; point < profile.point_count; ++point) {
      const auto& previous = profile.points[point - 1U];
      const auto& current = profile.points[point];
      if (!finite(current.velocity_rad_s) || current.time_ms <= previous.time_ms ||
          current.time_ms > profile.duration_ms ||
          profile.points[point].velocity_rad_s < 0.0F ||
          profile.points[point].velocity_rad_s > settings.safety.max_velocity_rad_s) {
        return false;
      }
      const float span_s = static_cast<float>(current.time_ms - previous.time_ms) * 0.001F;
      if (std::fabs(current.velocity_rad_s - previous.velocity_rad_s) / span_s >
          settings.safety.max_acceleration_rad_s2 + 0.001F) {
        return false;
      }
    }
  }
  return selected_profile_found;
}

bool SettingsStore::load(MachineSettings& settings) {
  Preferences preferences;
  if (!preferences.begin(kNamespace, true)) {
    settings = defaults();
    return false;
  }
  const size_t stored_size = preferences.getBytesLength(kBlobKey);
  bool loaded = false;
  bool migrated = false;
  bool migrated_bearing_value_is_valid = false;

  // Stored sizes are schema-specific. Keep each large blob in a separate scope so
  // boot never reserves every historical MachineSettings layout on the task stack.
  if (stored_size == sizeof(PersistedSettings)) {
    PersistedSettings blob{};
    const size_t bytes = preferences.getBytes(kBlobKey, &blob, sizeof(blob));
    const uint16_t crc = protocol::crc16CcittFalse(
        reinterpret_cast<const uint8_t*>(&blob.payload), sizeof(blob.payload));
    const bool valid_blob =
        bytes == sizeof(blob) && blob.magic == kSettingsMagic &&
        blob.payload_size == sizeof(MachineSettings) && blob.crc == crc;
    const bool current_schema =
        blob.schema_version == MachineSettings::kSchemaVersion &&
        blob.payload.schema_version == MachineSettings::kSchemaVersion;
    if (valid_blob && current_schema) {
      settings = blob.payload;
      loaded = validate(settings);
    } else if (valid_blob && blob.schema_version == 23U &&
               blob.payload.schema_version == 23U) {
      settings = blob.payload;
      settings.schema_version = MachineSettings::kSchemaVersion;
      settings.safety.jerk_limit_enabled = true;
      loaded = validate(settings);
      migrated = loaded;
      migrated_bearing_value_is_valid = loaded;
    }
  } else if (stored_size == sizeof(LegacyPersistedSettingsV22)) {
    LegacyPersistedSettingsV22 blob{};
    const size_t bytes = preferences.getBytes(kBlobKey, &blob, sizeof(blob));
    const uint16_t crc = protocol::crc16CcittFalse(
        reinterpret_cast<const uint8_t*>(&blob.payload),
        sizeof(blob.payload));
    const bool compatible_schema =
        blob.schema_version == 22U &&
        blob.payload.schema_version == 22U;
    if (bytes == sizeof(blob) && blob.magic == kSettingsMagic &&
        compatible_schema && blob.payload_size == sizeof(blob.payload) &&
        blob.crc == crc) {
      settings = defaults();
      settings.pins = blob.payload.pins;
      settings.control = blob.payload.control;
      settings.motor = blob.payload.motor;
      settings.supply_voltage = blob.payload.supply_voltage;
      settings.safety = blob.payload.safety;
      settings.serial = blob.payload.serial;
      copyLegacyEncoder(blob.payload.encoder, settings.encoder);
      settings.characterization = blob.payload.characterization;
      settings.load_setting = blob.payload.load_setting;
      settings.profile_count = blob.payload.profile_count;
      settings.selected_profile_id = blob.payload.selected_profile_id;
      settings.profiles = blob.payload.profiles;
      settings.motor_direction = blob.payload.motor_direction;
      settings.stop_mode = blob.payload.stop_mode;
      settings.motor_model = blob.payload.motor_model;
      settings.velocity_estimator_method =
          blob.payload.velocity_estimator_method;
      settings.velocity_acceleration_window_samples =
          blob.payload.velocity_acceleration_window_samples;
      loaded = validate(settings);
      migrated = loaded;
      migrated_bearing_value_is_valid = loaded;
    }
  } else if (stored_size == sizeof(LegacyPersistedSettingsV21)) {
    LegacyPersistedSettingsV21 blob{};
    const size_t bytes = preferences.getBytes(kBlobKey, &blob, sizeof(blob));
    const uint16_t crc = protocol::crc16CcittFalse(
        reinterpret_cast<const uint8_t*>(&blob.payload), sizeof(blob.payload));
    const bool compatible_schema =
        blob.schema_version >= 17U && blob.schema_version <= 21U &&
        blob.payload.schema_version == blob.schema_version;
    if (bytes == sizeof(blob) && blob.magic == kSettingsMagic &&
        compatible_schema && blob.payload_size == sizeof(blob.payload) &&
        blob.crc == crc) {
      settings = defaults();
      settings.pins = blob.payload.pins;
      settings.control = blob.payload.control;
      settings.motor = blob.payload.motor;
      settings.supply_voltage = blob.payload.supply_voltage;
      settings.safety = blob.payload.safety;
      settings.serial = blob.payload.serial;
      copyLegacyEncoder(blob.payload.encoder, settings.encoder);
      if (blob.schema_version < 20U) {
        settings.encoder.zero_position_offset_ticks = 0U;
      }
      settings.characterization = blob.payload.characterization;
      settings.load_setting = blob.payload.load_setting;
      if (blob.schema_version < 21U) {
        settings.load_setting.broken_bearing = false;
      }
      settings.profile_count = blob.payload.profile_count;
      settings.selected_profile_id = blob.payload.selected_profile_id;
      settings.profiles = blob.payload.profiles;
      settings.motor_direction = blob.payload.motor_direction;
      settings.stop_mode = blob.payload.stop_mode;
      settings.motor_model = blob.payload.motor_model;
      settings.velocity_estimator_method = blob.payload.velocity_estimator_method;
      settings.velocity_acceleration_window_samples =
          blob.payload.velocity_acceleration_window_samples;
      loaded = validate(settings);
      migrated = loaded;
      migrated_bearing_value_is_valid =
          loaded && blob.schema_version >= 21U;
    }
  } else if (stored_size == sizeof(LegacyPersistedSettingsV16)) {
    LegacyPersistedSettingsV16 blob{};
    const size_t bytes = preferences.getBytes(kBlobKey, &blob, sizeof(blob));
    const uint16_t crc = protocol::crc16CcittFalse(
        reinterpret_cast<const uint8_t*>(&blob.payload), sizeof(blob.payload));
    if (bytes == sizeof(blob) && blob.magic == kSettingsMagic &&
        blob.schema_version == 16U && blob.payload.schema_version == 16U &&
        blob.payload_size == sizeof(blob.payload) && blob.crc == crc) {
      settings = defaults();
      settings.pins = blob.payload.pins;
      settings.control = blob.payload.control;
      settings.motor = blob.payload.motor;
      settings.supply_voltage = blob.payload.supply_voltage;
      settings.safety = blob.payload.safety;
      settings.serial = blob.payload.serial;
      copyLegacyEncoder(blob.payload.encoder, settings.encoder);
      settings.characterization = blob.payload.characterization;
      settings.load_setting = blob.payload.load_setting;
      settings.profile_count = blob.payload.profile_count;
      settings.selected_profile_id = blob.payload.selected_profile_id;
      settings.profiles = blob.payload.profiles;
      settings.motor_direction = blob.payload.motor_direction;
      settings.stop_mode = blob.payload.stop_mode;
      settings.motor_model = blob.payload.motor_model;
      settings.velocity_estimator_method = blob.payload.velocity_estimator_method;
      settings.velocity_acceleration_window_samples =
          blob.payload.velocity_acceleration_window_samples;
      loaded = validate(settings);
      migrated = loaded;
    }
  } else if (stored_size == sizeof(LegacyPersistedSettingsV15)) {
    LegacyPersistedSettingsV15 blob{};
    const size_t bytes = preferences.getBytes(kBlobKey, &blob, sizeof(blob));
    const uint16_t crc = protocol::crc16CcittFalse(
        reinterpret_cast<const uint8_t*>(&blob.payload), sizeof(blob.payload));
    if (bytes == sizeof(blob) && blob.magic == kSettingsMagic &&
        blob.schema_version == 15U && blob.payload.schema_version == 15U &&
        blob.payload_size == sizeof(blob.payload) && blob.crc == crc) {
      settings = defaults();
      settings.pins = blob.payload.pins;
      settings.control = blob.payload.control;
      settings.motor = blob.payload.motor;
      settings.supply_voltage = blob.payload.supply_voltage;
      settings.safety = blob.payload.safety;
      settings.serial = blob.payload.serial;
      copyLegacyEncoder(blob.payload.encoder, settings.encoder);
      settings.characterization = blob.payload.characterization;
      settings.load_setting = blob.payload.load_setting;
      settings.profile_count = blob.payload.profile_count;
      settings.selected_profile_id = blob.payload.selected_profile_id;
      settings.profiles = blob.payload.profiles;
      settings.motor_direction = blob.payload.motor_direction;
      settings.stop_mode = blob.payload.stop_mode;
      settings.motor_model = blob.payload.motor_model;
      settings.velocity_estimator_method = blob.payload.velocity_estimator_method;
      loaded = validate(settings);
      migrated = loaded;
    }
  } else if (stored_size == sizeof(LegacyPersistedSettingsV14)) {
    LegacyPersistedSettingsV14 blob{};
    const size_t bytes = preferences.getBytes(kBlobKey, &blob, sizeof(blob));
    const uint16_t crc = protocol::crc16CcittFalse(
        reinterpret_cast<const uint8_t*>(&blob.payload), sizeof(blob.payload));
    if (bytes == sizeof(blob) && blob.magic == kSettingsMagic &&
        blob.schema_version == 14U && blob.payload.schema_version == 14U &&
        blob.payload_size == sizeof(blob.payload) && blob.crc == crc) {
      settings = defaults();
      settings.pins = blob.payload.pins;
      settings.control = blob.payload.control;
      settings.motor = blob.payload.motor;
      settings.supply_voltage = blob.payload.supply_voltage;
      settings.safety = blob.payload.safety;
      settings.serial = blob.payload.serial;
      copyLegacyEncoder(blob.payload.encoder, settings.encoder);
      settings.characterization = blob.payload.characterization;
      settings.load_setting = blob.payload.load_setting;
      settings.profile_count = blob.payload.profile_count;
      settings.selected_profile_id = blob.payload.selected_profile_id;
      settings.profiles = blob.payload.profiles;
      settings.motor_direction = blob.payload.motor_direction;
      settings.stop_mode = blob.payload.stop_mode;
      settings.motor_model = blob.payload.motor_model;
      const bool had_active_motor_model =
          settings.motor_model.velocity_gain_forward_rad_s_per_duty > 0.0F &&
          settings.motor_model.velocity_gain_reverse_rad_s_per_duty > 0.0F &&
          settings.motor_model.time_constant_forward_s > 0.0F &&
          settings.motor_model.time_constant_reverse_s > 0.0F;
      settings.velocity_estimator_method = had_active_motor_model
          ? VelocityEstimatorMethod::Kalman
          : VelocityEstimatorMethod::LowPass;
      loaded = validate(settings);
      migrated = loaded;
    }
  } else if (stored_size == sizeof(LegacyPersistedSettingsV13)) {
    LegacyPersistedSettingsV13 blob{};
    const size_t bytes = preferences.getBytes(kBlobKey, &blob, sizeof(blob));
    const uint16_t crc = protocol::crc16CcittFalse(
        reinterpret_cast<const uint8_t*>(&blob.payload), sizeof(blob.payload));
    if (bytes == sizeof(blob) && blob.magic == kSettingsMagic &&
        blob.schema_version == 13U && blob.payload.schema_version == 13U &&
        blob.payload_size == sizeof(blob.payload) && blob.crc == crc) {
      settings = defaults();
      settings.pins = blob.payload.pins;
      settings.control = blob.payload.control;
      settings.motor = blob.payload.motor;
      settings.supply_voltage = blob.payload.supply_voltage;
      settings.safety = blob.payload.safety;
      settings.serial = blob.payload.serial;
      copyLegacyEncoder(blob.payload.encoder, settings.encoder);
      settings.characterization = blob.payload.characterization;
      settings.load_setting = blob.payload.load_setting;
      settings.profile_count = blob.payload.profile_count;
      settings.selected_profile_id = blob.payload.selected_profile_id;
      settings.profiles = blob.payload.profiles;
      settings.motor_direction = blob.payload.motor_direction;
      settings.stop_mode = blob.payload.stop_mode;
      loaded = validate(settings);
      migrated = loaded;
    }
  } else if (stored_size == sizeof(LegacyPersistedSettingsV12)) {
    LegacyPersistedSettingsV12 blob{};
    const size_t bytes = preferences.getBytes(kBlobKey, &blob, sizeof(blob));
    const uint16_t crc = protocol::crc16CcittFalse(
        reinterpret_cast<const uint8_t*>(&blob.payload), sizeof(blob.payload));
    if (bytes == sizeof(blob) && blob.magic == kSettingsMagic &&
        blob.schema_version == 12U && blob.payload.schema_version == 12U &&
        blob.payload_size == sizeof(blob.payload) && blob.crc == crc) {
      settings = defaults();
      settings.pins = blob.payload.pins;
      settings.control = blob.payload.control;
      settings.motor = blob.payload.motor;
      settings.supply_voltage = blob.payload.supply_voltage;
      settings.safety = blob.payload.safety;
      settings.serial = blob.payload.serial;
      copyLegacyEncoder(blob.payload.encoder, settings.encoder);
      copyLegacyCharacterization(blob.payload.characterization,
                                 settings.characterization);
      settings.load_setting = blob.payload.load_setting;
      settings.profile_count = blob.payload.profile_count;
      settings.selected_profile_id = blob.payload.selected_profile_id;
      settings.profiles = blob.payload.profiles;
      settings.motor_direction = blob.payload.motor_direction;
      settings.stop_mode = blob.payload.stop_mode;
      loaded = validate(settings);
      migrated = loaded;
    }
  } else if (stored_size == sizeof(LegacyPersistedSettingsV11)) {
    LegacyPersistedSettingsV11 blob{};
    const size_t bytes = preferences.getBytes(kBlobKey, &blob, sizeof(blob));
    const uint16_t crc = protocol::crc16CcittFalse(
        reinterpret_cast<const uint8_t*>(&blob.payload), sizeof(blob.payload));
    const bool compatible_schema =
        (blob.schema_version == 10U || blob.schema_version == 11U) &&
        blob.payload.schema_version == blob.schema_version;
    if (bytes == sizeof(blob) && blob.magic == kSettingsMagic && compatible_schema &&
        blob.payload_size == sizeof(blob.payload) && blob.crc == crc) {
      settings = defaults();
      settings.pins = blob.payload.pins;
      settings.control = blob.payload.control;
      settings.motor = blob.payload.motor;
      settings.supply_voltage = blob.payload.supply_voltage;
      settings.safety = blob.payload.safety;
      settings.serial = blob.payload.serial;
      copyLegacyEncoder(blob.payload.encoder, settings.encoder);
      copyLegacyCharacterization(blob.payload.characterization,
                                 settings.characterization);
      copyLegacyLoad(blob.payload.load_setting, settings.load_setting);
      settings.profile_count = blob.payload.profile_count;
      settings.selected_profile_id = blob.payload.selected_profile_id;
      settings.profiles = blob.payload.profiles;
      settings.motor_direction = blob.payload.motor_direction;
      settings.stop_mode = blob.payload.stop_mode;
      loaded = validate(settings);
      migrated = loaded;
    }
  } else if (stored_size == sizeof(LegacyPersistedSettingsV9)) {
    LegacyPersistedSettingsV9 blob{};
    const size_t bytes = preferences.getBytes(kBlobKey, &blob, sizeof(blob));
    const uint16_t crc = protocol::crc16CcittFalse(
        reinterpret_cast<const uint8_t*>(&blob.payload), sizeof(blob.payload));
    if (bytes == sizeof(blob) && blob.magic == kSettingsMagic &&
        blob.schema_version == 9U && blob.payload_size == sizeof(blob.payload) &&
        blob.crc == crc) {
      settings = defaults();
      settings.pins = blob.payload.pins;
      settings.control = blob.payload.control;
      settings.motor = blob.payload.motor;
      settings.supply_voltage = blob.payload.supply_voltage;
      settings.safety = blob.payload.safety;
      settings.serial = blob.payload.serial;
      copyLegacyEncoder(blob.payload.encoder, settings.encoder);
      copyLegacyCharacterization(blob.payload.characterization,
                                 settings.characterization);
      copyLegacyLoad(blob.payload.load_setting, settings.load_setting);
      settings.profile_count = blob.payload.profile_count;
      settings.selected_profile_id = blob.payload.selected_profile_id;
      settings.profiles = blob.payload.profiles;
      settings.motor_direction = blob.payload.motor_direction;
      settings.stop_mode = blob.payload.stop_mode;
      loaded = validate(settings);
      migrated = loaded;
    }
  } else if (stored_size == sizeof(LegacyPersistedSettingsV8)) {
    LegacyPersistedSettingsV8 blob{};
    const size_t bytes = preferences.getBytes(kBlobKey, &blob, sizeof(blob));
    const uint16_t crc = protocol::crc16CcittFalse(
        reinterpret_cast<const uint8_t*>(&blob.payload), sizeof(blob.payload));
    if (bytes == sizeof(blob) && blob.magic == kSettingsMagic &&
        blob.schema_version == 8U && blob.payload_size == sizeof(blob.payload) &&
        blob.crc == crc) {
      settings = defaults();
      settings.pins = blob.payload.pins;
      settings.control = blob.payload.control;
      settings.motor = blob.payload.motor;
      settings.supply_voltage = blob.payload.supply_voltage;
      settings.safety = blob.payload.safety;
      settings.serial = blob.payload.serial;
      copyLegacyEncoder(blob.payload.encoder, settings.encoder);
      copyLegacyCharacterization(blob.payload.characterization,
                                 settings.characterization);
      copyLegacyLoad(blob.payload.load_setting, settings.load_setting);
      settings.profile_count = blob.payload.profile_count;
      settings.selected_profile_id = blob.payload.selected_profile_id;
      settings.profiles = blob.payload.profiles;
      settings.motor_direction = blob.payload.motor_direction;
      settings.stop_mode = blob.payload.stop_mode;
      loaded = validate(settings);
      migrated = loaded;
    }
  } else if (stored_size == sizeof(LegacyPersistedSettingsV7)) {
    LegacyPersistedSettingsV7 blob{};
    const size_t bytes = preferences.getBytes(kBlobKey, &blob, sizeof(blob));
    const uint16_t crc = protocol::crc16CcittFalse(
        reinterpret_cast<const uint8_t*>(&blob.payload), sizeof(blob.payload));
    if (bytes == sizeof(blob) && blob.magic == kSettingsMagic &&
        blob.schema_version == 7U && blob.payload_size == sizeof(blob.payload) &&
        blob.crc == crc) {
      settings = defaults();
      settings.pins = blob.payload.pins;
      settings.control = blob.payload.control;
      settings.motor = blob.payload.motor;
      settings.supply_voltage = blob.payload.supply_voltage;
      settings.safety = blob.payload.safety;
      settings.serial = blob.payload.serial;
      copyLegacyEncoder(blob.payload.encoder, settings.encoder);
      copyLegacyCharacterization(blob.payload.characterization,
                                 settings.characterization);
      copyLegacyLoad(blob.payload.load_setting, settings.load_setting);
      settings.profile_count = blob.payload.profile_count;
      settings.selected_profile_id = blob.payload.selected_profile_id;
      settings.profiles = blob.payload.profiles;
      settings.motor_direction = blob.payload.motor_direction;
      settings.stop_mode = blob.payload.stop_mode;
      loaded = validate(settings);
      migrated = loaded;
    }
  } else if (stored_size == sizeof(LegacyPersistedSettingsV6)) {
    LegacyPersistedSettingsV6 blob{};
    const size_t bytes = preferences.getBytes(kBlobKey, &blob, sizeof(blob));
    const uint16_t crc = protocol::crc16CcittFalse(
        reinterpret_cast<const uint8_t*>(&blob.payload), sizeof(blob.payload));
    if (bytes == sizeof(blob) && blob.magic == kSettingsMagic &&
        blob.schema_version == 6U && blob.payload_size == sizeof(blob.payload) &&
        blob.crc == crc) {
      settings = defaults();
      settings.pins = blob.payload.pins;
      settings.control = blob.payload.control;
      copyLegacyMotor(blob.payload.motor, settings.motor);
      settings.supply_voltage = blob.payload.supply_voltage;
      settings.safety = blob.payload.safety;
      settings.serial = blob.payload.serial;
      copyLegacyEncoder(blob.payload.encoder, settings.encoder);
      copyLegacyCharacterization(blob.payload.characterization,
                                 settings.characterization);
      copyLegacyLoad(blob.payload.load_setting, settings.load_setting);
      settings.profile_count = blob.payload.profile_count;
      settings.selected_profile_id = blob.payload.selected_profile_id;
      settings.profiles = blob.payload.profiles;
      settings.motor_direction = blob.payload.motor_direction;
      settings.stop_mode = blob.payload.stop_mode;
      loaded = validate(settings);
      migrated = loaded;
    }
  } else if (stored_size == sizeof(LegacyPersistedSettingsV5)) {
    LegacyPersistedSettingsV5 blob{};
    const size_t bytes = preferences.getBytes(kBlobKey, &blob, sizeof(blob));
    const uint16_t crc = protocol::crc16CcittFalse(
        reinterpret_cast<const uint8_t*>(&blob.payload), sizeof(blob.payload));
    if (bytes == sizeof(blob) && blob.magic == kSettingsMagic &&
        blob.schema_version == 5U && blob.payload_size == sizeof(blob.payload) &&
        blob.crc == crc) {
      settings = defaults();
      settings.pins = blob.payload.pins;
      copyLegacyControl(blob.payload.control, settings.control);
      copyLegacyMotor(blob.payload.motor, settings.motor);
      settings.supply_voltage = blob.payload.supply_voltage;
      settings.safety = blob.payload.safety;
      settings.serial = blob.payload.serial;
      copyLegacyEncoder(blob.payload.encoder, settings.encoder);
      copyLegacyCharacterization(blob.payload.characterization,
                                 settings.characterization);
      copyLegacyLoad(blob.payload.load_setting, settings.load_setting);
      settings.profile_count = blob.payload.profile_count;
      settings.selected_profile_id = blob.payload.selected_profile_id;
      settings.profiles = blob.payload.profiles;
      settings.motor_direction = blob.payload.motor_direction;
      settings.stop_mode = blob.payload.stop_mode;
      loaded = validate(settings);
      migrated = loaded;
    }
  } else if (stored_size == sizeof(LegacyPersistedSettingsV4)) {
    LegacyPersistedSettingsV4 blob{};
    const size_t bytes = preferences.getBytes(kBlobKey, &blob, sizeof(blob));
    const uint16_t crc = protocol::crc16CcittFalse(
        reinterpret_cast<const uint8_t*>(&blob.payload), sizeof(blob.payload));
    if (bytes == sizeof(blob) && blob.magic == kSettingsMagic &&
        blob.schema_version == 4U && blob.payload_size == sizeof(blob.payload) &&
        blob.crc == crc) {
      settings = defaults();
      settings.pins = blob.payload.pins;
      copyLegacyControl(blob.payload.control, settings.control);
      copyLegacyMotor(blob.payload.motor, settings.motor);
      settings.supply_voltage = blob.payload.supply_voltage;
      settings.safety.max_velocity_rad_s = blob.payload.safety.max_velocity_rad_s;
      settings.safety.max_acceleration_rad_s2 = blob.payload.safety.max_acceleration_rad_s2;
      settings.safety.max_jerk_rad_s3 = blob.payload.safety.max_jerk_rad_s3;
      settings.safety.max_current_a = blob.payload.safety.max_current_a;
      settings.safety.min_supply_voltage_v = blob.payload.safety.min_supply_voltage_v;
      settings.safety.max_supply_voltage_v = blob.payload.safety.max_supply_voltage_v;
      settings.safety.max_duty = blob.payload.safety.max_duty;
      settings.safety.command_timeout_ms = blob.payload.safety.command_timeout_ms;
      settings.safety.encoder_timeout_ms = blob.payload.safety.encoder_timeout_ms;
      settings.safety.current_sense_enabled = blob.payload.safety.current_sense_enabled;
      settings.safety.driver_diagnostic_enabled =
          blob.payload.safety.driver_diagnostic_enabled;
      settings.serial = blob.payload.serial;
      copyLegacyEncoder(blob.payload.encoder, settings.encoder);
      copyLegacyCharacterization(blob.payload.characterization,
                                 settings.characterization);
      copyLegacyLoad(blob.payload.load_setting, settings.load_setting);
      settings.profile_count = blob.payload.profile_count;
      settings.selected_profile_id = blob.payload.selected_profile_id;
      settings.profiles = blob.payload.profiles;
      settings.motor_direction = blob.payload.motor_direction;
      settings.stop_mode = blob.payload.stop_mode;
      loaded = validate(settings);
      migrated = loaded;
    }
  }

  preferences.end();
  if (!loaded) {
    settings = defaults();
    return false;
  }
  if (migrated) {
    // Schemas 4–20 used this byte as structure padding, so never interpret its
    // stored value as a bearing label during migration.
    if (!migrated_bearing_value_is_valid) {
      settings.load_setting.broken_bearing = false;
    }
    save(settings);
  }
  return true;
}

bool SettingsStore::save(const MachineSettings& settings) {
  if (!validate(settings)) {
    return false;
  }
  PersistedSettings blob{};
  blob.payload = settings;
  blob.crc = protocol::crc16CcittFalse(reinterpret_cast<const uint8_t*>(&blob.payload),
                                       sizeof(blob.payload));
  Preferences preferences;
  if (!preferences.begin(kNamespace, false)) {
    return false;
  }
  const bool result = preferences.putBytes(kBlobKey, &blob, sizeof(blob)) == sizeof(blob);
  preferences.end();
  return result;
}

bool SettingsStore::erase() {
  Preferences preferences;
  if (!preferences.begin(kNamespace, false)) {
    return false;
  }
  const bool result = preferences.clear();
  preferences.end();
  return result;
}

}  // namespace mm
