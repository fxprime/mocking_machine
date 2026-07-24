#include "app/MachineApplication.hpp"

#include <esp_timer.h>

#include <algorithm>
#include <cmath>
#include <cstring>

#include "control/CharacterizationMetrics.hpp"
#include "control/CharacterizationSettings.hpp"
#include "control/RotorPosition.hpp"
#include "profile/ProfileCollection.hpp"
#include "protocol/SerialBandwidth.hpp"

#ifndef BUILD_VERSION
#define BUILD_VERSION "development"
#endif

namespace mm {
namespace {

#pragma pack(push, 1)
struct AckPayload {
  uint16_t request_message_id;
  uint8_t result;
};

struct HeartbeatPayload {
  uint64_t uptime_us;
  uint32_t settings_schema;
  uint32_t faults;
  uint8_t state;
  char build_version[48];
};

struct SettingsPayload {
  uint32_t schema_version;
  uint32_t baud;
  uint32_t control_period_us;
  uint32_t counts_per_revolution;
  uint16_t stream_rate_hz;
  uint16_t selected_profile_id;
  float kp;
  float ki;
  float kd;
  float max_velocity_rad_s;
  float max_acceleration_rad_s2;
  float max_jerk_rad_s3;
  float max_current_a;
  float max_duty;
  float start_duty_forward;
  float start_duty_reverse;
  uint8_t load_setting_id;
  uint8_t load_count;
  int8_t motor_direction;
  uint8_t stop_mode;
  float supply_divider_gain;
  float supply_input_offset_v;
  float min_supply_voltage_v;
  float max_supply_voltage_v;
  uint8_t supply_voltage_pin;
  float current_gain_a_per_v;
  float current_offset_v;
  uint8_t current_sense_pin;
  uint8_t current_sense_enabled;
  uint8_t driver_diagnostic_enabled;
  uint8_t driver_diagnostic_pin;
  uint32_t encoder_timeout_ms;
  float encoder_timeout_velocity_rad_s;
  float max_feedback_correction;
  uint8_t estimator_min_counts;
  uint32_t estimator_max_window_us;
  uint32_t estimator_stale_timeout_us;
  float current_filter_cutoff_hz;
  uint32_t zero_index_min_interval_us;
  float zero_index_correction_gain;
  int8_t encoder_direction;
  float zero_index_minimum_separation_revolutions;
  float characterization_dynamics_filter_cutoff_hz;
  float characterization_dynamics_quantile;
  float characterization_recommendation_safety_factor;
  float motor_model_gain_forward_rad_s_per_duty;
  float motor_model_gain_reverse_rad_s_per_duty;
  float motor_model_time_constant_forward_s;
  float motor_model_time_constant_reverse_s;
  float motor_model_velocity_process_noise_rad_s2;
  float motor_model_disturbance_process_noise_rad_s3;
  float motor_model_encoder_measurement_noise_counts;
  uint8_t velocity_estimator_method;
  uint8_t velocity_acceleration_window_samples;
  uint32_t rotor_zero_offset_ticks;
};

struct TelemetryPayload {
  uint64_t timestamp_us;
  uint64_t last_zero_timestamp_us;
  int64_t encoder_count;
  int64_t last_zero_encoder_count;
  float desired_velocity_rad_s;
  float measured_velocity_rad_s;
  float controller_output;
  float current_a;
  float supply_voltage_v;
  uint32_t faults;
  uint16_t profile_id;
  uint8_t load_setting_id;
  uint8_t state;
  float controller_proportional_term;
  float controller_integral_term;
  float controller_derivative_term;
  float rotor_position_deg;
  uint32_t zero_index_sequence;
  uint32_t zero_index_rejected_count;
};

struct ControllerPayload {
  float kp;
  float ki;
  float kd;
};

struct VelocityTestPayload {
  float target_velocity_rad_s;
  uint32_t duration_ms;
};

struct VelocitySequencePayload {
  uint32_t hold_ms;
  uint8_t level_count;
  uint8_t reserved[3];
  float levels_rad_s[kMaximumVelocityTestLevels];
};

struct DriverDiagnosticPayload {
  uint8_t enabled;
};

struct CurrentSensePayload {
  uint8_t enabled;
};

enum class ParameterId : uint16_t {
  Baud = 1,
  ControlPeriodUs,
  CountsPerRevolution,
  StreamRateHz,
  Kp,
  Ki,
  Kd,
  MaximumVelocity,
  MaximumAcceleration,
  MaximumJerk,
  MaximumCurrent,
  MaximumDuty,
  ForwardDeadband,
  ReverseDeadband,
  MotorDirection,
  SupplyDividerGain,
  SupplyInputOffset,
  MinimumSupplyVoltage,
  MaximumSupplyVoltage,
  CurrentGain,
  CurrentOffset,
  EncoderTimeoutMs,
  EncoderTimeoutVelocity,
  MaximumFeedbackCorrection,
  EstimatorMinimumCounts,
  EstimatorMaximumWindowUs,
  EstimatorStaleTimeoutUs,
  CurrentSenseEnabled,
  CurrentFilterCutoffHz,
  ZeroIndexMinimumIntervalUs,
  ZeroIndexCorrectionGain,
  ZeroIndexMinimumSeparationRevolutions,
  CharacterizationDynamicsFilterCutoffHz,
  CharacterizationDynamicsQuantile,
  CharacterizationRecommendationSafetyFactor,
  VelocityEstimatorMethod,
  VelocityAccelerationWindowSamples,
  RotorZeroOffsetTicks,
};

struct ParameterPayload {
  uint16_t parameter_id;
  float value;
  uint8_t persist;
};

struct ProfilePointPayload {
  uint32_t time_ms;
  float velocity_rad_s;
};

struct ProfilePayload {
  uint16_t profile_id;
  uint8_t kind;
  char name[16];
  float target_velocity_rad_s;
  float sine_mean_rad_s;
  float sine_amplitude_rad_s;
  float sine_frequency_hz;
  uint32_t duration_ms;
  uint8_t point_count;
  ProfilePointPayload points[kMaximumProfilePoints];
};

struct SetProfilePayload {
  uint8_t persist;
  ProfilePayload profile;
};

struct LoadEntryPayload {
  uint8_t slot_id;
  uint16_t position_deg;
  float strength;
};

struct LoadConfigurationPayload {
  uint8_t setting_id;
  uint8_t count;
  LoadEntryPayload loads[kMaximumLoads];
};

struct MotorTestPayload {
  float signed_raw_duty;
};

enum class CurrentCalibrationAction : uint8_t {
  Reset = 0,
  CapturePoint1 = 1,
  CapturePoint2 = 2,
  Save = 3,
  Cancel = 4,
  RequestStatus = 5,
};

struct CurrentCalibrationPayload {
  uint8_t action;
  float reference_current_a;
};

struct CurrentCalibrationStatusPayload {
  uint8_t captured_mask;
  uint8_t capture_point;
  uint8_t last_result;
  float point1_voltage_v;
  float point1_reference_a;
  float point2_voltage_v;
  float point2_reference_a;
  float candidate_gain_a_per_v;
  float candidate_offset_v;
};

enum class RotorZeroCalibrationAction : uint8_t {
  Capture = 0,
  Save = 1,
  Cancel = 2,
  RequestStatus = 3,
};

struct RotorZeroCalibrationPayload {
  uint8_t action;
};

struct RotorZeroCalibrationStatusPayload {
  uint8_t state;
  uint8_t capture_count;
  uint8_t last_result;
  float current_position_deg;
  uint32_t candidate_offset_ticks;
  uint32_t saved_offset_ticks;
};

struct SupplyVoltageCalibrationPayload {
  float reference_voltage_v;
};

struct CharacterizationResultPayload {
  float start_duty_forward;
  float start_duty_reverse;
  float max_velocity_forward_rad_s;
  float max_velocity_reverse_rad_s;
  float acceleration_forward_rad_s2;
  float acceleration_reverse_rad_s2;
  float jerk_forward_rad_s3;
  float jerk_reverse_rad_s3;
  float velocity_gain_forward_rad_s_per_duty;
  float velocity_gain_reverse_rad_s_per_duty;
  float time_constant_forward_s;
  float time_constant_reverse_s;
};

struct CharacterizationActionPayload {
  uint8_t flags;
};

constexpr uint8_t kCharacterizationSave = 1U << 0U;
constexpr uint8_t kCharacterizationApplyAcceleration = 1U << 1U;
constexpr uint8_t kCharacterizationApplyJerk = 1U << 2U;
constexpr uint8_t kCharacterizationActionMask = kCharacterizationSave |
    kCharacterizationApplyAcceleration | kCharacterizationApplyJerk;

struct CharacterizationStatusPayload {
  uint8_t stage;
  uint8_t result_pending;
  float applied_duty;
  float measured_velocity_rad_s;
};
#pragma pack(pop)

static_assert(sizeof(SettingsPayload) == 177U, "Update protocol and browser SETTINGS decoder");
static_assert(sizeof(CurrentCalibrationPayload) == 5U,
              "Current calibration command payload changed");
static_assert(sizeof(CurrentCalibrationStatusPayload) == 27U,
              "Current calibration status payload changed");
static_assert(sizeof(RotorZeroCalibrationPayload) == 1U,
              "Rotor-zero calibration command payload changed");
static_assert(sizeof(RotorZeroCalibrationStatusPayload) == 15U,
              "Rotor-zero calibration status payload changed");
static_assert(sizeof(TelemetryPayload) == 84U, "Update protocol and browser TELEMETRY decoder");
static_assert(sizeof(SupplyVoltageCalibrationPayload) == 4U,
              "Supply calibration payload must remain one float");
static_assert(sizeof(CharacterizationResultPayload) == 48U,
              "Characterization result payload layout changed");
static_assert(sizeof(CharacterizationStatusPayload) == 10U,
              "Characterization status payload layout changed");
static_assert(sizeof(ParameterPayload) == 7U, "Parameter payload layout changed");
static_assert(sizeof(ProfilePayload) == 168U, "Profile payload layout changed");
static_assert(sizeof(SetProfilePayload) == 169U, "Set profile payload layout changed");
static_assert(sizeof(VelocitySequencePayload) == 72U,
              "Update protocol and browser velocity-sequence encoder");
static_assert(sizeof(LoadEntryPayload) == 7U, "Load entry payload layout changed");
static_assert(sizeof(LoadConfigurationPayload) == 86U,
              "Load configuration payload layout changed");

VelocityProfileConfiguration decodeProfile(const ProfilePayload& payload) {
  VelocityProfileConfiguration profile{};
  profile.profile_id = payload.profile_id;
  profile.kind = static_cast<ProfileKind>(payload.kind);
  std::memcpy(profile.name, payload.name, sizeof(profile.name));
  profile.name[sizeof(profile.name) - 1U] = '\0';
  profile.target_velocity_rad_s = payload.target_velocity_rad_s;
  profile.sine_mean_rad_s = payload.sine_mean_rad_s;
  profile.sine_amplitude_rad_s = payload.sine_amplitude_rad_s;
  profile.sine_frequency_hz = payload.sine_frequency_hz;
  profile.duration_ms = payload.duration_ms;
  profile.point_count = payload.point_count;
  for (size_t index = 0; index < kMaximumProfilePoints; ++index) {
    profile.points[index].time_ms = payload.points[index].time_ms;
    profile.points[index].velocity_rad_s = payload.points[index].velocity_rad_s;
  }
  return profile;
}
}

MachineApplication& MachineApplication::instance() {
  static MachineApplication application;
  return application;
}

void MachineApplication::configureVelocityController() {
  ControlConfiguration configuration = settings_.control;
  configuration.output_min = -settings_.safety.max_duty;
  configuration.output_max = settings_.safety.max_duty;
  controller_.configure(configuration);
}

void MachineApplication::begin() {
  const bool loaded = settings_store_.load(settings_);
  runtime_profile_id_ = settings_.selected_profile_id;
  const uint16_t constrained_stream_rate = protocol::constrainTelemetryStreamRateHz(
      settings_.serial.baud, settings_.serial.stream_rate_hz);
  if (settings_.serial.stream_rate_hz != constrained_stream_rate) {
    settings_.serial.stream_rate_hz = constrained_stream_rate;
    settings_store_.save(settings_);
  }
  Serial.begin(settings_.serial.baud);
  serial_link_.begin(Serial, &MachineApplication::frameThunk, &MachineApplication::lineThunk, this);

  encoder_.begin(settings_.pins.encoder_a, settings_.pins.encoder_b);
  zero_index_.begin(settings_.pins.zero_index, encoder_,
                    settings_.encoder.zero_index_min_interval_us,
                    zeroIndexMinimumSeparationCounts(
                        settings_.encoder.counts_per_output_revolution,
                        settings_.encoder.zero_index_minimum_separation_revolutions));
  rotor_phase_tracker_.configure(settings_.encoder.counts_per_output_revolution,
                                 settings_.encoder.direction,
                                 settings_.encoder.zero_index_correction_gain,
                                 settings_.encoder.zero_position_offset_ticks);
  motor_initialized_ = motor_.begin(settings_);
  if (!motor_initialized_) {
    faults_ |= FaultInvalidConfiguration;
    state_ = RunState::Fault;
  }
  configureVelocityController();
  velocity_estimator_.configure(settings_.encoder, settings_.control.velocity_filter_tau_s,
                                settings_.motor_model, settings_.velocity_estimator_method,
                                settings_.safety.max_acceleration_rad_s2,
                                settings_.velocity_acceleration_window_samples);
  current_filter_.configure(settings_.motor.current_filter_cutoff_hz);
  motion_limiter_.configure(settings_.safety);
  const uint64_t now = static_cast<uint64_t>(esp_timer_get_time());
  velocity_estimator_.reset(encoder_.count(), now);
  next_control_us_ = now + settings_.control.period_us;
  last_control_us_ = now;
  next_heartbeat_us_ = now;
  next_stream_us_ = now;
  next_supply_voltage_sample_us_ = now;
  next_characterization_status_us_ = now;

  Serial.printf("mocking-machine %s, settings=%s\r\n", BUILD_VERSION,
                loaded ? "nvs" : "defaults");
  Serial.println("Type 'help'. Motor starts disarmed.");
}

void MachineApplication::runOnce() {
  uint64_t now = static_cast<uint64_t>(esp_timer_get_time());

  if (now >= next_control_us_) {
    const uint64_t lateness_us = now - next_control_us_;
    if (lateness_us > static_cast<uint64_t>(settings_.control.period_us) * 2ULL) {
      const bool motor_output_active = state_ == RunState::Running ||
          characterization_stage_ != CharacterizationStage::Idle ||
          (state_ == RunState::Armed && manual_command_expiry_us_ > now);
      if (motor_output_active) {
        faults_ |= FaultControlOverrun;
        transitionToStopped();
        state_ = RunState::Fault;
      }
      // Flash/NVS and host traffic may legitimately block while stopped. Do not
      // replay stale ticks or latch a control fault when no actuator was active.
      next_control_us_ = now + settings_.control.period_us;
      last_control_us_ = now;
    } else {
      controlTick(now);
      do {
        next_control_us_ += settings_.control.period_us;
      } while (next_control_us_ <= now);
    }
  }

  // The control deadline is evaluated before commands can change actuator state.
  // This prevents lateness accumulated while safely stopped from being attributed
  // retroactively to a motor command received during this iteration.
  serial_link_.poll();
  serial_link_.serviceTx();
  now = static_cast<uint64_t>(esp_timer_get_time());

  if (now >= next_heartbeat_us_) {
    sendHeartbeat(now);
    next_heartbeat_us_ = now + 1000000ULL;
  }
  if (stream_enabled_ && now >= next_stream_us_) {
    sendTelemetry();
    next_stream_us_ = now + 1000000ULL / settings_.serial.stream_rate_hz;
  }
  if (characterization_notification_pending_) {
    characterization_notification_pending_ =
        !sendCharacterizationResult(transmit_sequence_++);
  }
  if (characterization_stage_ != CharacterizationStage::Idle &&
      now >= next_characterization_status_us_) {
    characterization_status_pending_ = true;
    next_characterization_status_us_ = now + 100000ULL;
  }
  if (characterization_status_pending_) {
    characterization_status_pending_ =
        !sendCharacterizationStatus(transmit_sequence_++);
  }
  if (current_calibration_status_pending_) {
    current_calibration_status_pending_ =
        !sendCurrentCalibrationStatus(transmit_sequence_++);
  }
  if (rotor_zero_calibration_status_pending_) {
    rotor_zero_calibration_status_pending_ =
        !sendRotorZeroCalibrationStatus(transmit_sequence_++);
  }
  serial_link_.serviceTx();
}

void MachineApplication::controlTick(const uint64_t scheduled_us) {
  const float dt_s = static_cast<float>(scheduled_us - last_control_us_) * 1.0e-6F;
  last_control_us_ = scheduled_us;
  telemetry_.timestamp_us = scheduled_us;
  const ZeroIndexCapture zero_capture = zero_index_.snapshot();
  telemetry_.encoder_count = encoder_.count();
  telemetry_.last_zero_timestamp_us = zero_capture.timestamp_us;
  telemetry_.last_zero_encoder_count = zero_capture.encoder_count;
  telemetry_.zero_index_sequence = zero_capture.sequence;
  telemetry_.zero_index_rejected_count = zero_capture.rejected_count;
  telemetry_.rotor_position_deg = rotor_phase_tracker_.update(
      telemetry_.encoder_count, zero_capture.encoder_count, zero_capture.sequence);
  telemetry_.measured_velocity_rad_s = velocity_estimator_.update(
      telemetry_.encoder_count, scheduled_us, motor_.appliedDuty(),
      state_ == RunState::Disarmed);
  const float current_sense_voltage_v = motor_.currentSenseVoltage();
  telemetry_.current_a = current_filter_.update(
      motor_.currentAmperesFromVoltage(current_sense_voltage_v), dt_s);
  updateCurrentCalibrationCapture(current_sense_voltage_v);
  if (scheduled_us >= next_supply_voltage_sample_us_) {
    const float measured_supply_v = motor_.supplyVoltage();
    telemetry_.supply_voltage_v = supply_voltage_initialized_
                                      ? telemetry_.supply_voltage_v +
                                            0.2F * (measured_supply_v - telemetry_.supply_voltage_v)
                                      : measured_supply_v;
    supply_voltage_initialized_ = true;
    next_supply_voltage_sample_us_ = scheduled_us + 10000ULL;
  }
  updateSafety(scheduled_us, telemetry_.current_a);
  telemetry_.controller_proportional_term = 0.0F;
  telemetry_.controller_integral_term = 0.0F;
  telemetry_.controller_derivative_term = 0.0F;

  if (characterization_stage_ != CharacterizationStage::Idle && faults_ == FaultNone) {
    updateCharacterization(scheduled_us);
    telemetry_.desired_velocity_rad_s = 0.0F;
    telemetry_.controller_output = motor_.appliedDuty();
  } else if (state_ == RunState::Running && faults_ == FaultNone) {
    const bool sequence_active = velocity_step_sequence_.active();
    const float raw_target = sequence_active
                                 ? velocity_step_sequence_.target(scheduled_us)
                                 : profile_.target(scheduled_us);
    telemetry_.desired_velocity_rad_s = motion_limiter_.update(raw_target, dt_s);
    if (telemetry_.desired_velocity_rad_s > 0.0F) {
      telemetry_.controller_output = controller_.update(
          telemetry_.desired_velocity_rad_s, telemetry_.measured_velocity_rad_s, dt_s);
      telemetry_.controller_proportional_term = controller_.proportionalTerm();
      telemetry_.controller_integral_term = controller_.integralTerm();
      telemetry_.controller_derivative_term = controller_.derivativeTerm();
      motor_.command(telemetry_.controller_output);
    } else {
      controller_.reset();
      telemetry_.controller_output = 0.0F;
      motor_.stop();
    }
    const bool source_finished = sequence_active
                                     ? velocity_step_sequence_.finished(scheduled_us)
                                     : profile_.finished(scheduled_us);
    if ((!sequence_active && !profile_.active()) ||
        (source_finished && motion_limiter_.velocity() == 0.0F)) {
      transitionToStopped();
    }
  } else if (state_ == RunState::Armed && manual_command_expiry_us_ > scheduled_us) {
    telemetry_.desired_velocity_rad_s = 0.0F;
    telemetry_.controller_output = manual_duty_;
    if (manual_raw_pwm_) {
      motor_.commandRaw(manual_duty_);
    } else {
      motor_.command(manual_duty_);
    }
  } else {
    manual_duty_ = 0.0F;
    manual_raw_pwm_ = false;
    telemetry_.desired_velocity_rad_s = 0.0F;
    telemetry_.controller_output = 0.0F;
    motor_.stop();
  }

  telemetry_.faults = faults_;
  telemetry_.profile_id = velocity_step_sequence_.active() ? UINT16_MAX : profile_.id();
  telemetry_.load_setting_id = settings_.load_setting.setting_id;
  telemetry_.state = state_;
}

void MachineApplication::updateSafety(const uint64_t timestamp_us, const float current_a) {
  const bool motor_output_active = state_ == RunState::Running ||
      characterization_stage_ != CharacterizationStage::Idle ||
      (state_ == RunState::Armed && manual_command_expiry_us_ > timestamp_us);
  if (motor_.diagnosticFault()) {
    faults_ |= FaultDriverDiagnostic;
  }
  if (motor_output_active && settings_.safety.current_sense_enabled &&
      settings_.safety.max_current_a > 0.0F &&
      current_a > settings_.safety.max_current_a) {
    faults_ |= FaultOverCurrent;
  }
  if (supply_voltage_initialized_ &&
      telemetry_.supply_voltage_v > settings_.safety.max_supply_voltage_v) {
    faults_ |= FaultOverVoltage;
  }
  if (motor_output_active && supply_voltage_initialized_ &&
      telemetry_.supply_voltage_v < settings_.safety.min_supply_voltage_v) {
    faults_ |= FaultUnderVoltage;
  }
  if (encoder_watchdog_.update(
          timestamp_us, telemetry_.desired_velocity_rad_s,
          encoder_.lastEdgeTimestampUs(), state_ == RunState::Running,
          settings_.safety.encoder_timeout_ms,
          settings_.safety.encoder_timeout_velocity_rad_s)) {
    faults_ |= FaultEncoderTimeout;
  }
  if (faults_ != FaultNone && state_ != RunState::Fault) {
    transitionToStopped();
    state_ = RunState::Fault;
  }
}

void MachineApplication::transitionToStopped() {
  profile_.stop();
  velocity_step_sequence_.stop();
  controller_.reset();
  motion_limiter_.reset();
  encoder_watchdog_.reset();
  manual_duty_ = 0.0F;
  manual_raw_pwm_ = false;
  manual_command_expiry_us_ = 0;
  if (characterization_stage_ != CharacterizationStage::Idle) {
    characterization_status_pending_ = true;
  }
  characterization_stage_ = CharacterizationStage::Idle;
  motor_.stop();
  if (state_ != RunState::Fault) {
    state_ = RunState::Disarmed;
  }
}

void MachineApplication::updateCurrentCalibrationCapture(
    const float sense_voltage_v) {
  if (!current_calibration_accumulator_.active()) {
    return;
  }
  float average_voltage_v = 0.0F;
  if (!current_calibration_accumulator_.addSample(sense_voltage_v,
                                                  average_voltage_v)) {
    return;
  }
  const CurrentCalibrationPoint point{
      average_voltage_v,
      current_calibration_accumulator_.referenceCurrentA(),
  };
  if (current_calibration_capture_point_ == 1U) {
    current_calibration_points_[0] = point;
    current_calibration_points_[1] = {};
    current_calibration_candidate_ = {};
    current_calibration_captured_mask_ = 0x01U;
    current_calibration_last_result_ = protocol::ResultCode::Ok;
  } else {
    CurrentCalibrationResult result{};
    if (calculateCurrentCalibration(current_calibration_points_[0], point,
                                    result)) {
      current_calibration_points_[1] = point;
      current_calibration_candidate_ = result;
      current_calibration_captured_mask_ = 0x03U;
      current_calibration_last_result_ = protocol::ResultCode::Ok;
    } else {
      current_calibration_last_result_ = protocol::ResultCode::InvalidValue;
    }
  }
  current_calibration_capture_point_ = 0U;
  current_calibration_status_pending_ = true;
}

bool MachineApplication::clearFaultsAndRecheck() {
  if (state_ == RunState::Running ||
      characterization_stage_ != CharacterizationStage::Idle) {
    return false;
  }

  transitionToStopped();
  uint32_t active_faults = motor_initialized_ ? FaultNone : FaultInvalidConfiguration;
  telemetry_.current_a = motor_.currentAmperes();
  telemetry_.supply_voltage_v = motor_.supplySenseVoltage(64U) *
      settings_.supply_voltage.divider_gain + settings_.supply_voltage.input_offset_v;
  telemetry_.supply_voltage_v = std::max(0.0F, telemetry_.supply_voltage_v);
  supply_voltage_initialized_ = true;

  if (motor_.diagnosticFault()) {
    active_faults |= FaultDriverDiagnostic;
  }
  if (telemetry_.supply_voltage_v < settings_.safety.min_supply_voltage_v) {
    active_faults |= FaultUnderVoltage;
  }
  if (telemetry_.supply_voltage_v > settings_.safety.max_supply_voltage_v) {
    active_faults |= FaultOverVoltage;
  }

  faults_ = active_faults;
  state_ = faults_ == FaultNone ? RunState::Disarmed : RunState::Fault;
  telemetry_.faults = faults_;
  telemetry_.state = state_;
  return faults_ == FaultNone;
}

void MachineApplication::updateCharacterization(const uint64_t scheduled_us) {
  const auto pause = [this, scheduled_us](const CharacterizationStage next) {
    motor_.stop();
    characterization_stage_ = next;
    characterization_deadline_us_ = scheduled_us +
        static_cast<uint64_t>(settings_.characterization.reversal_pause_ms) * 1000ULL;
    characterization_motion_samples_ = 0;
  };

  switch (characterization_stage_) {
    case CharacterizationStage::Idle:
      return;
    case CharacterizationStage::ForwardDeadband:
    case CharacterizationStage::ReverseDeadband: {
      motor_.command(characterization_duty_);
      if (scheduled_us < characterization_deadline_us_) {
        return;
      }
      if (std::fabs(telemetry_.measured_velocity_rad_s) >=
          settings_.characterization.motion_threshold_rad_s) {
        ++characterization_motion_samples_;
      } else {
        characterization_motion_samples_ = 0;
      }
      if (characterization_motion_samples_ >=
          settings_.characterization.consecutive_motion_samples) {
        if (characterization_stage_ == CharacterizationStage::ForwardDeadband) {
          characterization_candidate_.start_duty_forward = std::fabs(characterization_duty_);
          characterization_duty_ = -settings_.characterization.duty_step;
          pause(CharacterizationStage::PauseBeforeReverseDeadband);
        } else {
          characterization_candidate_.start_duty_reverse = std::fabs(characterization_duty_);
          pause(CharacterizationStage::PauseBeforeForwardMaximum);
        }
        return;
      }
      characterization_duty_ +=
          characterization_stage_ == CharacterizationStage::ForwardDeadband
              ? settings_.characterization.duty_step
              : -settings_.characterization.duty_step;
      if (std::fabs(characterization_duty_) > settings_.safety.max_duty) {
        faults_ |= FaultInvalidConfiguration;
        transitionToStopped();
        state_ = RunState::Fault;
        return;
      }
      characterization_deadline_us_ = scheduled_us +
          static_cast<uint64_t>(settings_.characterization.settle_ms) * 1000ULL;
      return;
    }
    case CharacterizationStage::PauseBeforeReverseDeadband:
      motor_.stop();
      if (scheduled_us >= characterization_deadline_us_) {
        characterization_stage_ = CharacterizationStage::ReverseDeadband;
        characterization_deadline_us_ = scheduled_us +
            static_cast<uint64_t>(settings_.characterization.settle_ms) * 1000ULL;
      }
      return;
    case CharacterizationStage::PauseBeforeForwardMaximum:
      motor_.stop();
      if (scheduled_us >= characterization_deadline_us_) {
        characterization_stage_ = CharacterizationStage::ForwardMaximum;
        characterization_peak_velocity_ = 0.0F;
        characterization_dynamics_estimator_.reset();
        characterization_model_identifier_.reset();
        characterization_deadline_us_ = scheduled_us +
            static_cast<uint64_t>(settings_.characterization.maximum_hold_ms) * 1000ULL;
      }
      return;
    case CharacterizationStage::ForwardMaximum:
      motor_.command(settings_.safety.max_duty);
      characterization_dynamics_estimator_.update(
          telemetry_.measured_velocity_rad_s,
          static_cast<float>(settings_.control.period_us) * 0.000001F);
      characterization_model_identifier_.update(
          telemetry_.measured_velocity_rad_s, motor_.appliedDuty(),
          static_cast<float>(settings_.control.period_us) * 0.000001F);
      characterization_peak_velocity_ = characterization::updatePeakVelocityMagnitude(
          characterization_peak_velocity_, telemetry_.measured_velocity_rad_s);
      if (scheduled_us >= characterization_deadline_us_) {
        if (!characterization_dynamics_estimator_.valid()) {
          faults_ |= FaultInvalidConfiguration;
          transitionToStopped();
          state_ = RunState::Fault;
          return;
        }
        characterization_candidate_.max_velocity_forward_rad_s = characterization_peak_velocity_;
        characterization_dynamics_candidate_.acceleration_forward_rad_s2 =
            characterization_dynamics_estimator_.accelerationRadS2();
        characterization_dynamics_candidate_.jerk_forward_rad_s3 =
            characterization_dynamics_estimator_.jerkRadS3();
        if (!characterization_model_identifier_.result(
                characterization_model_candidate_.velocity_gain_forward_rad_s_per_duty,
                characterization_model_candidate_.time_constant_forward_s)) {
          faults_ |= FaultInvalidConfiguration;
          transitionToStopped();
          state_ = RunState::Fault;
          return;
        }
        pause(CharacterizationStage::PauseBeforeReverseMaximum);
      }
      return;
    case CharacterizationStage::PauseBeforeReverseMaximum:
      motor_.stop();
      if (scheduled_us >= characterization_deadline_us_) {
        characterization_stage_ = CharacterizationStage::ReverseMaximum;
        characterization_peak_velocity_ = 0.0F;
        characterization_dynamics_estimator_.reset();
        characterization_model_identifier_.reset();
        characterization_deadline_us_ = scheduled_us +
            static_cast<uint64_t>(settings_.characterization.maximum_hold_ms) * 1000ULL;
      }
      return;
    case CharacterizationStage::ReverseMaximum:
      motor_.command(-settings_.safety.max_duty);
      characterization_dynamics_estimator_.update(
          telemetry_.measured_velocity_rad_s,
          static_cast<float>(settings_.control.period_us) * 0.000001F);
      characterization_model_identifier_.update(
          telemetry_.measured_velocity_rad_s, motor_.appliedDuty(),
          static_cast<float>(settings_.control.period_us) * 0.000001F);
      characterization_peak_velocity_ = characterization::updatePeakVelocityMagnitude(
          characterization_peak_velocity_, telemetry_.measured_velocity_rad_s);
      if (scheduled_us >= characterization_deadline_us_) {
        if (!characterization_dynamics_estimator_.valid()) {
          faults_ |= FaultInvalidConfiguration;
          transitionToStopped();
          state_ = RunState::Fault;
          return;
        }
        characterization_candidate_.max_velocity_reverse_rad_s = characterization_peak_velocity_;
        characterization_dynamics_candidate_.acceleration_reverse_rad_s2 =
            characterization_dynamics_estimator_.accelerationRadS2();
        characterization_dynamics_candidate_.jerk_reverse_rad_s3 =
            characterization_dynamics_estimator_.jerkRadS3();
        if (!characterization_model_identifier_.result(
                characterization_model_candidate_.velocity_gain_reverse_rad_s_per_duty,
                characterization_model_candidate_.time_constant_reverse_s)) {
          faults_ |= FaultInvalidConfiguration;
          transitionToStopped();
          state_ = RunState::Fault;
          return;
        }
        characterization_result_pending_ = true;
        characterization_notification_pending_ = true;
        transitionToStopped();
      }
      return;
  }
}

const VelocityProfileConfiguration* MachineApplication::selectedProfile() const {
  for (uint8_t index = 0; index < settings_.profile_count; ++index) {
    if (settings_.profiles[index].profile_id == runtime_profile_id_) {
      return &settings_.profiles[index];
    }
  }
  return nullptr;
}

void MachineApplication::frameThunk(void* const context, const protocol::FrameView& frame) {
  static_cast<MachineApplication*>(context)->handleFrame(frame);
}

void MachineApplication::lineThunk(void* const context, const char* const line) {
  static_cast<MachineApplication*>(context)->handleLine(line);
}

void MachineApplication::handleFrame(const protocol::FrameView& frame) {
  using protocol::MessageId;
  using protocol::ResultCode;
  switch (frame.message_id) {
    case MessageId::GetSettings:
      sendSettings(frame.sequence);
      return;
    case MessageId::GetProfiles:
      sendProfiles(frame.sequence);
      return;
    case MessageId::GetLoadConfiguration:
      sendLoadConfiguration(frame.sequence);
      return;
    case MessageId::StartStream:
      sendSettings(frame.sequence);
      sendProfiles(frame.sequence);
      sendLoadConfiguration(frame.sequence);
      if (characterization_result_pending_) {
        sendCharacterizationResult(frame.sequence);
      }
      if (characterization_stage_ != CharacterizationStage::Idle ||
          characterization_result_pending_) {
        sendCharacterizationStatus(frame.sequence);
      }
      stream_enabled_ = true;
      sendAck(frame.sequence, frame.message_id, ResultCode::Ok);
      return;
    case MessageId::StopStream:
      stream_enabled_ = false;
      sendAck(frame.sequence, frame.message_id, ResultCode::Ok);
      return;
    case MessageId::SelectProfile: {
      if (frame.payload_size != sizeof(uint16_t)) {
        sendAck(frame.sequence, frame.message_id, ResultCode::InvalidLength);
        return;
      }
      uint16_t requested_id = 0;
      std::memcpy(&requested_id, frame.payload, sizeof(requested_id));
      const uint16_t previous_id = runtime_profile_id_;
      runtime_profile_id_ = requested_id;
      if (selectedProfile() == nullptr) {
        runtime_profile_id_ = previous_id;
        sendAck(frame.sequence, frame.message_id, ResultCode::InvalidValue);
      } else {
        sendAck(frame.sequence, frame.message_id, ResultCode::Ok);
      }
      return;
    }
    case MessageId::SetDefaultProfile: {
      if (frame.payload_size != sizeof(uint16_t) || state_ != RunState::Disarmed) {
        sendAck(frame.sequence, frame.message_id,
                frame.payload_size != sizeof(uint16_t) ? ResultCode::InvalidLength
                                                       : ResultCode::UnsafeState);
        return;
      }
      uint16_t requested_id = 0;
      std::memcpy(&requested_id, frame.payload, sizeof(requested_id));
      const uint16_t previous_runtime_id = runtime_profile_id_;
      runtime_profile_id_ = requested_id;
      if (selectedProfile() == nullptr) {
        runtime_profile_id_ = previous_runtime_id;
        sendAck(frame.sequence, frame.message_id, ResultCode::InvalidValue);
        return;
      }
      const uint16_t previous_default_id = settings_.selected_profile_id;
      settings_.selected_profile_id = requested_id;
      MachineSettings persisted = SettingsStore::defaults();
      settings_store_.load(persisted);
      bool persisted_profile_exists = false;
      for (uint8_t index = 0; index < persisted.profile_count; ++index) {
        persisted_profile_exists |= persisted.profiles[index].profile_id == requested_id;
      }
      if (!persisted_profile_exists) {
        settings_.selected_profile_id = previous_default_id;
        runtime_profile_id_ = previous_runtime_id;
        sendAck(frame.sequence, frame.message_id, ResultCode::InvalidValue);
        return;
      }
      persisted.selected_profile_id = requested_id;
      if (!SettingsStore::validate(persisted) || !settings_store_.save(persisted)) {
        settings_.selected_profile_id = previous_default_id;
        runtime_profile_id_ = previous_runtime_id;
        sendAck(frame.sequence, frame.message_id, ResultCode::StorageFailure);
        return;
      }
      sendAck(frame.sequence, frame.message_id, ResultCode::Ok);
      sendSettings(frame.sequence);
      return;
    }
    case MessageId::SetLoadConfiguration: {
      if (frame.payload_size != sizeof(LoadConfigurationPayload) ||
          state_ != RunState::Disarmed) {
        sendAck(frame.sequence, frame.message_id,
                frame.payload_size != sizeof(LoadConfigurationPayload)
                    ? ResultCode::InvalidLength
                    : ResultCode::UnsafeState);
        return;
      }
      LoadConfigurationPayload requested{};
      std::memcpy(&requested, frame.payload, sizeof(requested));
      bool valid = requested.count <= kMaximumLoads;
      bool occupied[kRotorSlotCount]{};
      MachineLoadSetting candidate{};
      candidate.setting_id = requested.setting_id;
      candidate.count = requested.count;
      for (uint8_t index = 0; valid && index < requested.count; ++index) {
        const auto& entry = requested.loads[index];
        valid = entry.slot_id < kRotorSlotCount &&
                entry.position_deg == static_cast<uint16_t>(entry.slot_id * 30U) &&
                !occupied[entry.slot_id] && std::isfinite(entry.strength) &&
                entry.strength >= 1.0F && entry.strength <= 10.0F;
        if (valid) {
          occupied[entry.slot_id] = true;
          candidate.loads[index] = {entry.slot_id, entry.position_deg, entry.strength};
        }
      }
      if (!valid) {
        sendAck(frame.sequence, frame.message_id, ResultCode::InvalidValue);
        return;
      }
      const MachineLoadSetting previous = settings_.load_setting;
      settings_.load_setting = candidate;
      MachineSettings persisted = SettingsStore::defaults();
      settings_store_.load(persisted);
      persisted.load_setting = candidate;
      if (!SettingsStore::validate(persisted) || !settings_store_.save(persisted)) {
        settings_.load_setting = previous;
        sendAck(frame.sequence, frame.message_id, ResultCode::StorageFailure);
        return;
      }
      sendAck(frame.sequence, frame.message_id, ResultCode::Ok);
      sendSettings(frame.sequence);
      sendLoadConfiguration(frame.sequence);
      return;
    }
    case MessageId::SetController: {
      if (frame.payload_size != sizeof(ControllerPayload) || state_ == RunState::Running) {
        sendAck(frame.sequence, frame.message_id,
                state_ == RunState::Running ? ResultCode::UnsafeState : ResultCode::InvalidLength);
        return;
      }
      ControllerPayload gains{};
      std::memcpy(&gains, frame.payload, sizeof(gains));
      if (!std::isfinite(gains.kp) || !std::isfinite(gains.ki) || !std::isfinite(gains.kd) ||
          gains.kp < 0.0F || gains.ki < 0.0F || gains.kd < 0.0F) {
        sendAck(frame.sequence, frame.message_id, ResultCode::InvalidValue);
        return;
      }
      settings_.control.kp = gains.kp;
      settings_.control.ki = gains.ki;
      settings_.control.kd = gains.kd;
      configureVelocityController();
      sendAck(frame.sequence, frame.message_id, ResultCode::Ok);
      return;
    }
    case MessageId::SaveController: {
      if (frame.payload_size != sizeof(ControllerPayload) || state_ != RunState::Disarmed ||
          characterization_stage_ != CharacterizationStage::Idle) {
        sendAck(frame.sequence, frame.message_id,
                frame.payload_size != sizeof(ControllerPayload) ? ResultCode::InvalidLength
                                                                 : ResultCode::UnsafeState);
        return;
      }
      ControllerPayload gains{};
      std::memcpy(&gains, frame.payload, sizeof(gains));
      MachineSettings candidate = settings_;
      candidate.control.kp = gains.kp;
      candidate.control.ki = gains.ki;
      candidate.control.kd = gains.kd;
      if (!SettingsStore::validate(candidate)) {
        sendAck(frame.sequence, frame.message_id, ResultCode::InvalidValue);
        return;
      }
      if (!settings_store_.save(candidate)) {
        sendAck(frame.sequence, frame.message_id, ResultCode::StorageFailure);
        return;
      }
      settings_ = candidate;
      configureVelocityController();
      sendAck(frame.sequence, frame.message_id, ResultCode::Ok);
      sendSettings(frame.sequence);
      return;
    }
    case MessageId::SetDriverDiagnostic: {
      if (frame.payload_size != sizeof(DriverDiagnosticPayload)) {
        sendAck(frame.sequence, frame.message_id, ResultCode::InvalidLength);
        return;
      }
      DriverDiagnosticPayload diagnostic{};
      std::memcpy(&diagnostic, frame.payload, sizeof(diagnostic));
      if (diagnostic.enabled > 1U) {
        sendAck(frame.sequence, frame.message_id, ResultCode::InvalidValue);
        return;
      }
      const bool disabling_faulted_input = state_ == RunState::Fault && diagnostic.enabled == 0U;
      if ((state_ != RunState::Disarmed && !disabling_faulted_input) ||
          characterization_stage_ != CharacterizationStage::Idle) {
        sendAck(frame.sequence, frame.message_id, ResultCode::UnsafeState);
        return;
      }
      settings_.safety.driver_diagnostic_enabled = diagnostic.enabled != 0U;
      motor_.setDiagnosticEnabled(settings_.safety.driver_diagnostic_enabled);
      if (!settings_.safety.driver_diagnostic_enabled) {
        faults_ &= ~static_cast<uint32_t>(FaultDriverDiagnostic);
        if (faults_ == FaultNone && state_ == RunState::Fault) {
          state_ = RunState::Disarmed;
        }
      }
      const ResultCode result = settings_store_.save(settings_) ? ResultCode::Ok
                                                                : ResultCode::StorageFailure;
      sendAck(frame.sequence, frame.message_id, result);
      sendSettings(frame.sequence);
      return;
    }
    case MessageId::SetCurrentSense: {
      if (frame.payload_size != sizeof(CurrentSensePayload)) {
        sendAck(frame.sequence, frame.message_id, ResultCode::InvalidLength);
        return;
      }
      CurrentSensePayload current_sense{};
      std::memcpy(&current_sense, frame.payload, sizeof(current_sense));
      if (current_sense.enabled > 1U) {
        sendAck(frame.sequence, frame.message_id, ResultCode::InvalidValue);
        return;
      }
      const bool disabling_faulted_input =
          state_ == RunState::Fault && current_sense.enabled == 0U;
      if ((state_ != RunState::Disarmed && !disabling_faulted_input) ||
          characterization_stage_ != CharacterizationStage::Idle) {
        sendAck(frame.sequence, frame.message_id, ResultCode::UnsafeState);
        return;
      }
      settings_.safety.current_sense_enabled = current_sense.enabled != 0U;
      if (!settings_.safety.current_sense_enabled) {
        faults_ &= ~static_cast<uint32_t>(FaultOverCurrent);
        if (faults_ == FaultNone && state_ == RunState::Fault) {
          state_ = RunState::Disarmed;
        }
      }
      const ResultCode result = settings_store_.save(settings_) ? ResultCode::Ok
                                                                : ResultCode::StorageFailure;
      sendAck(frame.sequence, frame.message_id, result);
      sendSettings(frame.sequence);
      return;
    }
    case MessageId::SetParameter: {
      if (frame.payload_size != sizeof(ParameterPayload) || state_ != RunState::Disarmed ||
          characterization_stage_ != CharacterizationStage::Idle) {
        sendAck(frame.sequence, frame.message_id,
                frame.payload_size != sizeof(ParameterPayload) ? ResultCode::InvalidLength
                                                                : ResultCode::UnsafeState);
        return;
      }
      ParameterPayload update{};
      std::memcpy(&update, frame.payload, sizeof(update));
      if (!std::isfinite(update.value) || update.persist > 1U) {
        sendAck(frame.sequence, frame.message_id, ResultCode::InvalidValue);
        return;
      }
      MachineSettings candidate = settings_;
      const auto whole = [](const float value) {
        return value >= 0.0F && value <= static_cast<float>(UINT32_MAX) &&
               std::floor(value) == value;
      };
      switch (static_cast<ParameterId>(update.parameter_id)) {
        case ParameterId::Baud:
          if (!whole(update.value)) { sendAck(frame.sequence, frame.message_id, ResultCode::InvalidValue); return; }
          candidate.serial.baud = static_cast<uint32_t>(update.value); break;
        case ParameterId::ControlPeriodUs:
          if (!whole(update.value)) { sendAck(frame.sequence, frame.message_id, ResultCode::InvalidValue); return; }
          candidate.control.period_us = static_cast<uint32_t>(update.value); break;
        case ParameterId::CountsPerRevolution:
          if (!whole(update.value)) { sendAck(frame.sequence, frame.message_id, ResultCode::InvalidValue); return; }
          candidate.encoder.counts_per_output_revolution = static_cast<uint32_t>(update.value); break;
        case ParameterId::StreamRateHz:
          if (!whole(update.value) || update.value > UINT16_MAX) { sendAck(frame.sequence, frame.message_id, ResultCode::InvalidValue); return; }
          candidate.serial.stream_rate_hz = static_cast<uint16_t>(update.value); break;
        case ParameterId::Kp: candidate.control.kp = update.value; break;
        case ParameterId::Ki: candidate.control.ki = update.value; break;
        case ParameterId::Kd: candidate.control.kd = update.value; break;
        case ParameterId::MaximumVelocity: candidate.safety.max_velocity_rad_s = update.value; break;
        case ParameterId::MaximumAcceleration: candidate.safety.max_acceleration_rad_s2 = update.value; break;
        case ParameterId::MaximumJerk: candidate.safety.max_jerk_rad_s3 = update.value; break;
        case ParameterId::MaximumCurrent: candidate.safety.max_current_a = update.value; break;
        case ParameterId::MaximumDuty: candidate.safety.max_duty = update.value; break;
        case ParameterId::ForwardDeadband: candidate.motor.start_duty_forward = update.value; break;
        case ParameterId::ReverseDeadband: candidate.motor.start_duty_reverse = update.value; break;
        case ParameterId::MotorDirection:
          if (update.value != -1.0F && update.value != 1.0F) { sendAck(frame.sequence, frame.message_id, ResultCode::InvalidValue); return; }
          candidate.motor_direction = static_cast<int8_t>(update.value); break;
        case ParameterId::SupplyDividerGain: candidate.supply_voltage.divider_gain = update.value; break;
        case ParameterId::SupplyInputOffset: candidate.supply_voltage.input_offset_v = update.value; break;
        case ParameterId::MinimumSupplyVoltage: candidate.safety.min_supply_voltage_v = update.value; break;
        case ParameterId::MaximumSupplyVoltage: candidate.safety.max_supply_voltage_v = update.value; break;
        case ParameterId::CurrentGain: candidate.motor.current_gain_a_per_v = update.value; break;
        case ParameterId::CurrentOffset: candidate.motor.current_offset_v = update.value; break;
        case ParameterId::EncoderTimeoutMs:
          if (!whole(update.value)) { sendAck(frame.sequence, frame.message_id, ResultCode::InvalidValue); return; }
          candidate.safety.encoder_timeout_ms = static_cast<uint32_t>(update.value); break;
        case ParameterId::EncoderTimeoutVelocity:
          candidate.safety.encoder_timeout_velocity_rad_s = update.value; break;
        case ParameterId::MaximumFeedbackCorrection:
          candidate.control.max_feedback_correction = update.value; break;
        case ParameterId::EstimatorMinimumCounts:
          if (!whole(update.value) || update.value > UINT8_MAX) { sendAck(frame.sequence, frame.message_id, ResultCode::InvalidValue); return; }
          candidate.encoder.estimator_min_counts = static_cast<uint8_t>(update.value); break;
        case ParameterId::EstimatorMaximumWindowUs:
          if (!whole(update.value)) { sendAck(frame.sequence, frame.message_id, ResultCode::InvalidValue); return; }
          candidate.encoder.estimator_max_window_us = static_cast<uint32_t>(update.value); break;
        case ParameterId::EstimatorStaleTimeoutUs:
          if (!whole(update.value)) { sendAck(frame.sequence, frame.message_id, ResultCode::InvalidValue); return; }
          candidate.encoder.estimator_stale_timeout_us = static_cast<uint32_t>(update.value); break;
        case ParameterId::CurrentSenseEnabled:
          if (update.value != 0.0F && update.value != 1.0F) { sendAck(frame.sequence, frame.message_id, ResultCode::InvalidValue); return; }
          candidate.safety.current_sense_enabled = update.value != 0.0F; break;
        case ParameterId::CurrentFilterCutoffHz:
          candidate.motor.current_filter_cutoff_hz = update.value; break;
        case ParameterId::ZeroIndexMinimumIntervalUs:
          if (!whole(update.value)) { sendAck(frame.sequence, frame.message_id, ResultCode::InvalidValue); return; }
          candidate.encoder.zero_index_min_interval_us = static_cast<uint32_t>(update.value); break;
        case ParameterId::ZeroIndexCorrectionGain:
          candidate.encoder.zero_index_correction_gain = update.value; break;
        case ParameterId::ZeroIndexMinimumSeparationRevolutions:
          candidate.encoder.zero_index_minimum_separation_revolutions = update.value; break;
        case ParameterId::CharacterizationDynamicsFilterCutoffHz:
          candidate.characterization.dynamics_filter_cutoff_hz = update.value; break;
        case ParameterId::CharacterizationDynamicsQuantile:
          candidate.characterization.dynamics_quantile = update.value; break;
        case ParameterId::CharacterizationRecommendationSafetyFactor:
          candidate.characterization.recommendation_safety_factor = update.value; break;
        case ParameterId::VelocityEstimatorMethod:
          if (!whole(update.value) || update.value > 2.0F) {
            sendAck(frame.sequence, frame.message_id, ResultCode::InvalidValue); return;
          }
          candidate.velocity_estimator_method =
              static_cast<mm::VelocityEstimatorMethod>(static_cast<uint8_t>(update.value));
          break;
        case ParameterId::VelocityAccelerationWindowSamples:
          if (!whole(update.value) || update.value < 2.0F || update.value > 32.0F) {
            sendAck(frame.sequence, frame.message_id, ResultCode::InvalidValue); return;
          }
          candidate.velocity_acceleration_window_samples =
              static_cast<uint32_t>(update.value);
          break;
        case ParameterId::RotorZeroOffsetTicks:
          if (!whole(update.value) || update.value < 0.0F ||
              update.value >= static_cast<float>(
                                  candidate.encoder.counts_per_output_revolution)) {
            sendAck(frame.sequence, frame.message_id, ResultCode::InvalidValue);
            return;
          }
          candidate.encoder.zero_position_offset_ticks =
              static_cast<uint32_t>(update.value);
          break;
        default:
          sendAck(frame.sequence, frame.message_id, ResultCode::InvalidValue); return;
      }
      if (candidate.serial.stream_rate_hz >
          protocol::maximumTelemetryStreamRateHz(candidate.serial.baud)) {
        sendAck(frame.sequence, frame.message_id, ResultCode::InvalidValue);
        return;
      }
      if (!SettingsStore::validate(candidate)) {
        sendAck(frame.sequence, frame.message_id, ResultCode::InvalidValue);
        return;
      }
      if (update.persist != 0U && !settings_store_.save(candidate)) {
        sendAck(frame.sequence, frame.message_id, ResultCode::StorageFailure);
        return;
      }
      settings_ = candidate;
      configureVelocityController();
      motion_limiter_.configure(settings_.safety);
      velocity_estimator_.configure(settings_.encoder, settings_.control.velocity_filter_tau_s,
                                    settings_.motor_model,
                                    settings_.velocity_estimator_method,
                                    settings_.safety.max_acceleration_rad_s2,
                                    settings_.velocity_acceleration_window_samples);
      velocity_estimator_.reset(encoder_.count(),
          static_cast<uint64_t>(esp_timer_get_time()));
      motor_.setSafety(settings_.safety);
      motor_.setCharacteristics(settings_.motor);
      motor_.setMotorDirection(settings_.motor_direction);
      motor_.setSupplyVoltageCalibration(settings_.supply_voltage.divider_gain,
                                         settings_.supply_voltage.input_offset_v);
      motor_.setCurrentCalibration(settings_.motor.current_gain_a_per_v,
                                   settings_.motor.current_offset_v);
      current_filter_.configure(settings_.motor.current_filter_cutoff_hz);
      zero_index_.setMinimumIntervalUs(settings_.encoder.zero_index_min_interval_us);
      zero_index_.setMinimumSeparationCounts(zeroIndexMinimumSeparationCounts(
          settings_.encoder.counts_per_output_revolution,
          settings_.encoder.zero_index_minimum_separation_revolutions));
      rotor_phase_tracker_.configure(settings_.encoder.counts_per_output_revolution,
                                     settings_.encoder.direction,
                                     settings_.encoder.zero_index_correction_gain,
                                     settings_.encoder.zero_position_offset_ticks);
      if (!settings_.safety.current_sense_enabled) {
        faults_ &= ~static_cast<uint32_t>(FaultOverCurrent);
      }
      sendAck(frame.sequence, frame.message_id, ResultCode::Ok);
      sendSettings(frame.sequence);
      return;
    }
    case MessageId::SetProfile:
    case MessageId::CreateProfile: {
      if (frame.payload_size != sizeof(SetProfilePayload) || state_ != RunState::Disarmed ||
          characterization_stage_ != CharacterizationStage::Idle) {
        sendAck(frame.sequence, frame.message_id,
                frame.payload_size != sizeof(SetProfilePayload) ? ResultCode::InvalidLength
                                                                 : ResultCode::UnsafeState);
        return;
      }
      SetProfilePayload update{};
      std::memcpy(&update, frame.payload, sizeof(update));
      if (update.persist > 1U ||
          update.profile.kind > static_cast<uint8_t>(ProfileKind::Waypoints) ||
          update.profile.point_count > kMaximumProfilePoints || update.profile.duration_ms == 0U) {
        sendAck(frame.sequence, frame.message_id, ResultCode::InvalidValue);
        return;
      }
      MachineSettings candidate = settings_;
      const bool create_only = frame.message_id == MessageId::CreateProfile;
      const ProfileUpdateResult profile_result = applyProfileUpdate(
          candidate.profiles, candidate.profile_count, decodeProfile(update.profile), create_only);
      if ((profile_result != ProfileUpdateResult::Created &&
           profile_result != ProfileUpdateResult::Replaced) ||
          !SettingsStore::validate(candidate)) {
        sendAck(frame.sequence, frame.message_id, ResultCode::InvalidValue);
        return;
      }
      if (update.persist != 0U && !settings_store_.save(candidate)) {
        sendAck(frame.sequence, frame.message_id, ResultCode::StorageFailure);
        return;
      }
      settings_ = candidate;
      sendAck(frame.sequence, frame.message_id, ResultCode::Ok);
      sendProfiles(frame.sequence);
      return;
    }
    case MessageId::Arm:
      if (frame.payload_size != 0U) {
        sendAck(frame.sequence, frame.message_id, ResultCode::InvalidLength);
      } else if (faults_ != FaultNone ||
                 (state_ != RunState::Disarmed && state_ != RunState::Armed) ||
                 characterization_stage_ != CharacterizationStage::Idle) {
        sendAck(frame.sequence, frame.message_id, ResultCode::UnsafeState);
      } else {
        state_ = RunState::Armed;
        sendAck(frame.sequence, frame.message_id, ResultCode::Ok);
      }
      return;
    case MessageId::StartRun: {
      const auto* selected = selectedProfile();
      if (state_ != RunState::Armed || selected == nullptr) {
        sendAck(frame.sequence, frame.message_id, ResultCode::UnsafeState);
        return;
      }
      const uint64_t now = static_cast<uint64_t>(esp_timer_get_time());
      velocity_step_sequence_.stop();
      profile_.select(selected, now);
      motion_limiter_.reset();
      controller_.reset();
      state_ = RunState::Running;
      sendAck(frame.sequence, frame.message_id, ResultCode::Ok);
      return;
    }
    case MessageId::StartVelocityTest: {
      if (frame.payload_size != sizeof(VelocityTestPayload) || state_ != RunState::Armed ||
          faults_ != FaultNone || characterization_stage_ != CharacterizationStage::Idle) {
        sendAck(frame.sequence, frame.message_id,
                frame.payload_size != sizeof(VelocityTestPayload) ? ResultCode::InvalidLength
                                                                   : ResultCode::UnsafeState);
        return;
      }
      VelocityTestPayload test{};
      std::memcpy(&test, frame.payload, sizeof(test));
      if (!std::isfinite(test.target_velocity_rad_s) ||
          test.target_velocity_rad_s <= 0.0F ||
          test.target_velocity_rad_s > settings_.safety.max_velocity_rad_s ||
          test.duration_ms < 100U || test.duration_ms > 3600000U) {
        sendAck(frame.sequence, frame.message_id, ResultCode::InvalidValue);
        return;
      }
      tuning_profile_ = VelocityProfileConfiguration{};
      tuning_profile_.profile_id = UINT16_MAX;
      tuning_profile_.kind = ProfileKind::Ramp;
      std::strncpy(tuning_profile_.name, "manual-step", sizeof(tuning_profile_.name) - 1U);
      tuning_profile_.target_velocity_rad_s = test.target_velocity_rad_s;
      tuning_profile_.duration_ms = test.duration_ms;
      velocity_step_sequence_.stop();
      profile_.select(&tuning_profile_, static_cast<uint64_t>(esp_timer_get_time()));
      motion_limiter_.reset();
      controller_.reset();
      state_ = RunState::Running;
      sendAck(frame.sequence, frame.message_id, ResultCode::Ok);
      return;
    }
    case MessageId::StartVelocitySequence: {
      if (frame.payload_size != sizeof(VelocitySequencePayload) ||
          state_ != RunState::Armed || faults_ != FaultNone ||
          characterization_stage_ != CharacterizationStage::Idle) {
        sendAck(frame.sequence, frame.message_id,
                frame.payload_size != sizeof(VelocitySequencePayload)
                    ? ResultCode::InvalidLength
                    : ResultCode::UnsafeState);
        return;
      }
      VelocitySequencePayload test{};
      std::memcpy(&test, frame.payload, sizeof(test));
      const uint64_t duration_ms =
          static_cast<uint64_t>(test.hold_ms) * test.level_count;
      bool valid = test.level_count >= 1U &&
                   test.level_count <= kMaximumVelocityTestLevels &&
                   test.hold_ms >= 100U && duration_ms <= 3600000ULL;
      std::array<float, kMaximumVelocityTestLevels> levels{};
      for (size_t index = 0U; valid && index < test.level_count; ++index) {
        const float level = test.levels_rad_s[index];
        valid = std::isfinite(level) && level > 0.0F &&
                level <= settings_.safety.max_velocity_rad_s;
        levels[index] = level;
      }
      if (!valid) {
        sendAck(frame.sequence, frame.message_id, ResultCode::InvalidValue);
        return;
      }
      profile_.stop();
      velocity_step_sequence_.start(
          levels, test.level_count, test.hold_ms,
          static_cast<uint64_t>(esp_timer_get_time()));
      motion_limiter_.reset();
      controller_.reset();
      state_ = RunState::Running;
      sendAck(frame.sequence, frame.message_id, ResultCode::Ok);
      return;
    }
    case MessageId::StopRun:
      transitionToStopped();
      sendAck(frame.sequence, frame.message_id, ResultCode::Ok);
      return;
    case MessageId::MotorTest: {
      if (frame.payload_size != sizeof(MotorTestPayload) || state_ != RunState::Armed ||
          faults_ != FaultNone || characterization_stage_ != CharacterizationStage::Idle) {
        sendAck(frame.sequence, frame.message_id,
                state_ != RunState::Armed || faults_ != FaultNone ||
                        characterization_stage_ != CharacterizationStage::Idle
                    ? ResultCode::UnsafeState
                    : ResultCode::InvalidLength);
        return;
      }
      MotorTestPayload test{};
      std::memcpy(&test, frame.payload, sizeof(test));
      if (!std::isfinite(test.signed_raw_duty) ||
          std::fabs(test.signed_raw_duty) > settings_.safety.max_duty) {
        sendAck(frame.sequence, frame.message_id, ResultCode::InvalidValue);
        return;
      }
      manual_duty_ = test.signed_raw_duty;
      manual_raw_pwm_ = true;
      if (std::fabs(manual_duty_) < 0.0001F) {
        manual_command_expiry_us_ = 0U;
        motor_.stop();
      } else {
        manual_command_expiry_us_ = static_cast<uint64_t>(esp_timer_get_time()) +
            static_cast<uint64_t>(settings_.safety.command_timeout_ms) * 1000ULL;
      }
      sendAck(frame.sequence, frame.message_id, ResultCode::Ok);
      return;
    }
    case MessageId::ClearFaults: {
      if (frame.payload_size != 0U) {
        sendAck(frame.sequence, frame.message_id, ResultCode::InvalidLength);
        return;
      }
      sendAck(frame.sequence, frame.message_id,
              clearFaultsAndRecheck() ? ResultCode::Ok : ResultCode::UnsafeState);
      return;
    }
    case MessageId::CurrentCalibration: {
      if (frame.payload_size != sizeof(CurrentCalibrationPayload) ||
          state_ == RunState::Running ||
          characterization_stage_ != CharacterizationStage::Idle) {
        sendAck(frame.sequence, frame.message_id,
                frame.payload_size != sizeof(CurrentCalibrationPayload)
                    ? ResultCode::InvalidLength
                    : ResultCode::UnsafeState);
        return;
      }
      CurrentCalibrationPayload calibration{};
      std::memcpy(&calibration, frame.payload, sizeof(calibration));
      if (calibration.action >
          static_cast<uint8_t>(CurrentCalibrationAction::RequestStatus)) {
        sendAck(frame.sequence, frame.message_id, ResultCode::InvalidValue);
        return;
      }
      const auto action = static_cast<CurrentCalibrationAction>(calibration.action);
      if (action == CurrentCalibrationAction::Reset ||
          action == CurrentCalibrationAction::Cancel) {
        if (action == CurrentCalibrationAction::Cancel) {
          transitionToStopped();
        }
        current_calibration_accumulator_.cancel();
        current_calibration_capture_point_ = 0U;
        current_calibration_captured_mask_ = 0U;
        current_calibration_points_[0] = {};
        current_calibration_points_[1] = {};
        current_calibration_candidate_ = {};
        current_calibration_last_result_ = protocol::ResultCode::Ok;
        sendAck(frame.sequence, frame.message_id, ResultCode::Ok);
        current_calibration_status_pending_ =
            !sendCurrentCalibrationStatus(frame.sequence);
        return;
      }
      if (action == CurrentCalibrationAction::RequestStatus) {
        sendAck(frame.sequence, frame.message_id, ResultCode::Ok);
        current_calibration_status_pending_ =
            !sendCurrentCalibrationStatus(frame.sequence);
        return;
      }
      if (action == CurrentCalibrationAction::CapturePoint1 ||
          action == CurrentCalibrationAction::CapturePoint2) {
        if (!std::isfinite(calibration.reference_current_a) ||
            calibration.reference_current_a < 0.0F ||
            calibration.reference_current_a > 50.0F) {
          sendAck(frame.sequence, frame.message_id, ResultCode::InvalidValue);
          return;
        }
        const size_t point_index =
            action == CurrentCalibrationAction::CapturePoint1 ? 0U : 1U;
        if (current_calibration_accumulator_.active() ||
            (point_index == 1U &&
             (current_calibration_captured_mask_ & 0x01U) == 0U)) {
          sendAck(frame.sequence, frame.message_id, ResultCode::InvalidValue);
          return;
        }
        current_calibration_capture_point_ = static_cast<uint8_t>(point_index + 1U);
        current_calibration_last_result_ = protocol::ResultCode::Ok;
        current_calibration_accumulator_.start(calibration.reference_current_a);
        sendAck(frame.sequence, frame.message_id, ResultCode::Ok);
        current_calibration_status_pending_ =
            !sendCurrentCalibrationStatus(frame.sequence);
        return;
      }
      if (current_calibration_accumulator_.active() ||
          current_calibration_captured_mask_ != 0x03U) {
        sendAck(frame.sequence, frame.message_id, ResultCode::InvalidValue);
        return;
      }
      MachineSettings candidate = settings_;
      candidate.motor.current_gain_a_per_v =
          current_calibration_candidate_.gain_a_per_v;
      candidate.motor.current_offset_v = current_calibration_candidate_.offset_v;
      if (!SettingsStore::validate(candidate)) {
        sendAck(frame.sequence, frame.message_id, ResultCode::InvalidValue);
        return;
      }
      // NVS writes can take multiple control periods. Stop before touching flash
      // so saving calibration can never create an active-output overrun fault.
      transitionToStopped();
      if (!settings_store_.save(candidate)) {
        sendAck(frame.sequence, frame.message_id, ResultCode::StorageFailure);
        return;
      }
      settings_ = candidate;
      motor_.setCurrentCalibration(settings_.motor.current_gain_a_per_v,
                                   settings_.motor.current_offset_v);
      current_filter_.reset();
      sendAck(frame.sequence, frame.message_id, ResultCode::Ok);
      sendSettings(frame.sequence);
      current_calibration_status_pending_ =
          !sendCurrentCalibrationStatus(frame.sequence);
      return;
    }
    case MessageId::RotorZeroCalibration: {
      if (frame.payload_size != sizeof(RotorZeroCalibrationPayload)) {
        sendAck(frame.sequence, frame.message_id, ResultCode::InvalidLength);
        return;
      }
      RotorZeroCalibrationPayload calibration{};
      std::memcpy(&calibration, frame.payload, sizeof(calibration));
      if (calibration.action >
          static_cast<uint8_t>(RotorZeroCalibrationAction::RequestStatus)) {
        sendAck(frame.sequence, frame.message_id, ResultCode::InvalidValue);
        return;
      }
      const auto action =
          static_cast<RotorZeroCalibrationAction>(calibration.action);
      if (action == RotorZeroCalibrationAction::RequestStatus) {
        sendAck(frame.sequence, frame.message_id, ResultCode::Ok);
        rotor_zero_calibration_status_pending_ =
            !sendRotorZeroCalibrationStatus(frame.sequence);
        return;
      }
      if (action == RotorZeroCalibrationAction::Cancel) {
        rotor_zero_calibration_candidate_valid_ = false;
        rotor_zero_calibration_last_result_ = ResultCode::Ok;
        sendAck(frame.sequence, frame.message_id, ResultCode::Ok);
        rotor_zero_calibration_status_pending_ =
            !sendRotorZeroCalibrationStatus(frame.sequence);
        return;
      }
      if (state_ != RunState::Disarmed ||
          characterization_stage_ != CharacterizationStage::Idle) {
        sendAck(frame.sequence, frame.message_id, ResultCode::UnsafeState);
        return;
      }
      if (action == RotorZeroCalibrationAction::Capture) {
        if (!rotor_phase_tracker_.referenced()) {
          sendAck(frame.sequence, frame.message_id, ResultCode::InvalidValue);
          return;
        }
        rotor_zero_calibration_candidate_ticks_ =
            rotor_phase_tracker_.positionTicksFromZeroIndex(
                telemetry_.encoder_count, telemetry_.last_zero_encoder_count);
        rotor_zero_calibration_candidate_valid_ = true;
        rotor_zero_calibration_last_result_ = ResultCode::Ok;
        sendAck(frame.sequence, frame.message_id, ResultCode::Ok);
        rotor_zero_calibration_status_pending_ =
            !sendRotorZeroCalibrationStatus(frame.sequence);
        return;
      }
      if (!rotor_zero_calibration_candidate_valid_) {
        sendAck(frame.sequence, frame.message_id, ResultCode::InvalidValue);
        return;
      }
      MachineSettings candidate = settings_;
      candidate.encoder.zero_position_offset_ticks =
          rotor_zero_calibration_candidate_ticks_;
      if (!SettingsStore::validate(candidate)) {
        sendAck(frame.sequence, frame.message_id, ResultCode::InvalidValue);
        return;
      }
      transitionToStopped();
      if (!settings_store_.save(candidate)) {
        rotor_zero_calibration_last_result_ = ResultCode::StorageFailure;
        sendAck(frame.sequence, frame.message_id, ResultCode::StorageFailure);
        rotor_zero_calibration_status_pending_ =
            !sendRotorZeroCalibrationStatus(frame.sequence);
        return;
      }
      settings_ = candidate;
      rotor_phase_tracker_.configure(settings_.encoder.counts_per_output_revolution,
                                     settings_.encoder.direction,
                                     settings_.encoder.zero_index_correction_gain,
                                     settings_.encoder.zero_position_offset_ticks);
      rotor_phase_tracker_.synchronizeToZeroIndex(
          telemetry_.last_zero_encoder_count, telemetry_.zero_index_sequence);
      telemetry_.rotor_position_deg = rotor_phase_tracker_.update(
          telemetry_.encoder_count, telemetry_.last_zero_encoder_count,
          telemetry_.zero_index_sequence);
      rotor_zero_calibration_candidate_valid_ = false;
      rotor_zero_calibration_last_result_ = ResultCode::Ok;
      sendAck(frame.sequence, frame.message_id, ResultCode::Ok);
      sendSettings(frame.sequence);
      rotor_zero_calibration_status_pending_ =
          !sendRotorZeroCalibrationStatus(frame.sequence);
      return;
    }
    case MessageId::SupplyVoltageCalibration: {
      if (frame.payload_size != sizeof(SupplyVoltageCalibrationPayload) ||
          state_ == RunState::Running) {
        sendAck(frame.sequence, frame.message_id,
                state_ == RunState::Running ? ResultCode::UnsafeState : ResultCode::InvalidLength);
        return;
      }
      SupplyVoltageCalibrationPayload calibration{};
      std::memcpy(&calibration, frame.payload, sizeof(calibration));
      const float sense_voltage = motor_.supplySenseVoltage(64U);
      const float corrected_reference =
          calibration.reference_voltage_v - settings_.supply_voltage.input_offset_v;
      if (!std::isfinite(calibration.reference_voltage_v) || corrected_reference <= 0.0F ||
          calibration.reference_voltage_v > 20.0F || sense_voltage < 0.01F ||
          sense_voltage > VoltageSenseConfiguration::kMaximumCalibrationSenseV) {
        sendAck(frame.sequence, frame.message_id, ResultCode::InvalidValue);
        return;
      }
      settings_.supply_voltage.divider_gain = corrected_reference / sense_voltage;
      motor_.setSupplyVoltageCalibration(settings_.supply_voltage.divider_gain,
                                         settings_.supply_voltage.input_offset_v);
      telemetry_.supply_voltage_v = calibration.reference_voltage_v;
      supply_voltage_initialized_ = true;
      transitionToStopped();
      const ResultCode result = settings_store_.save(settings_) ? ResultCode::Ok
                                                                : ResultCode::StorageFailure;
      sendAck(frame.sequence, frame.message_id, result);
      sendSettings(frame.sequence);
      return;
    }
    case MessageId::CharacterizationAction: {
      if (frame.payload_size != sizeof(CharacterizationActionPayload)) {
        sendAck(frame.sequence, frame.message_id, ResultCode::InvalidLength);
        return;
      }
      CharacterizationActionPayload action{};
      std::memcpy(&action, frame.payload, sizeof(action));
      const bool invalid_flags = (action.flags & ~kCharacterizationActionMask) != 0U ||
          (action.flags != 0U && (action.flags & kCharacterizationSave) == 0U);
      if (invalid_flags || !characterization_result_pending_ ||
          state_ != RunState::Disarmed) {
        sendAck(frame.sequence, frame.message_id,
                invalid_flags ? ResultCode::InvalidValue : ResultCode::UnsafeState);
        return;
      }
      if (action.flags == 0U) {
        characterization_result_pending_ = false;
        characterization_notification_pending_ = false;
        sendAck(frame.sequence, frame.message_id, ResultCode::Ok);
        return;
      }
      MachineSettings candidate{};
      if (!characterization::prepareCharacterizedSettings(
              settings_, characterization_candidate_, candidate) ||
          !characterization::applyRecommendedDynamics(
              characterization_dynamics_candidate_,
              settings_.characterization.recommendation_safety_factor,
              (action.flags & kCharacterizationApplyAcceleration) != 0U,
              (action.flags & kCharacterizationApplyJerk) != 0U, candidate)) {
        sendAck(frame.sequence, frame.message_id, ResultCode::InvalidValue);
        return;
      }
      candidate.motor_model = characterization_model_candidate_;
      if (!SettingsStore::validate(candidate)) {
        sendAck(frame.sequence, frame.message_id, ResultCode::InvalidValue);
        return;
      }
      if (!settings_store_.save(candidate)) {
        sendAck(frame.sequence, frame.message_id, ResultCode::StorageFailure);
        return;
      }
      settings_ = candidate;
      motor_.setCharacteristics(settings_.motor);
      motor_.setSafety(settings_.safety);
      motion_limiter_.configure(settings_.safety);
      velocity_estimator_.configure(settings_.encoder, settings_.control.velocity_filter_tau_s,
                                    settings_.motor_model, settings_.velocity_estimator_method,
                                    settings_.safety.max_acceleration_rad_s2,
                                    settings_.velocity_acceleration_window_samples);
      velocity_estimator_.reset(encoder_.count(),
          static_cast<uint64_t>(esp_timer_get_time()));
      characterization_result_pending_ = false;
      characterization_notification_pending_ = false;
      sendAck(frame.sequence, frame.message_id, ResultCode::Ok);
      sendSettings(frame.sequence);
      sendProfiles(frame.sequence);
      return;
    }
    default:
      sendAck(frame.sequence, frame.message_id, ResultCode::InvalidMessage);
      return;
  }
}

void MachineApplication::sendAck(const uint16_t sequence, const protocol::MessageId request,
                                 const protocol::ResultCode result) {
  const AckPayload payload{static_cast<uint16_t>(request), static_cast<uint8_t>(result)};
  serial_link_.send(protocol::MessageId::Ack, sequence, &payload, sizeof(payload));
}

void MachineApplication::sendHeartbeat(const uint64_t timestamp_us) {
  HeartbeatPayload payload{};
  payload.uptime_us = timestamp_us;
  payload.settings_schema = settings_.schema_version;
  payload.faults = faults_;
  payload.state = static_cast<uint8_t>(state_);
  std::strncpy(payload.build_version, BUILD_VERSION, sizeof(payload.build_version) - 1U);
  serial_link_.send(protocol::MessageId::Heartbeat, transmit_sequence_++, &payload,
                    sizeof(payload));
}

void MachineApplication::sendSettings(const uint16_t sequence) {
  const SettingsPayload payload{
      settings_.schema_version, settings_.serial.baud, settings_.control.period_us,
      settings_.encoder.counts_per_output_revolution, settings_.serial.stream_rate_hz,
      settings_.selected_profile_id, settings_.control.kp, settings_.control.ki,
      settings_.control.kd, settings_.safety.max_velocity_rad_s,
      settings_.safety.max_acceleration_rad_s2, settings_.safety.max_jerk_rad_s3,
      settings_.safety.max_current_a, settings_.safety.max_duty,
      settings_.motor.start_duty_forward, settings_.motor.start_duty_reverse,
      settings_.load_setting.setting_id, settings_.load_setting.count,
      settings_.motor_direction, static_cast<uint8_t>(settings_.stop_mode),
      settings_.supply_voltage.divider_gain, settings_.supply_voltage.input_offset_v,
      settings_.safety.min_supply_voltage_v, settings_.safety.max_supply_voltage_v,
      settings_.pins.supply_voltage_sense,
      settings_.motor.current_gain_a_per_v, settings_.motor.current_offset_v,
      settings_.pins.current_sense,
      static_cast<uint8_t>(settings_.safety.current_sense_enabled),
      static_cast<uint8_t>(settings_.safety.driver_diagnostic_enabled),
      settings_.pins.driver_diag,
      settings_.safety.encoder_timeout_ms,
      settings_.safety.encoder_timeout_velocity_rad_s,
      settings_.control.max_feedback_correction,
      settings_.encoder.estimator_min_counts,
      settings_.encoder.estimator_max_window_us,
      settings_.encoder.estimator_stale_timeout_us,
      settings_.motor.current_filter_cutoff_hz,
      settings_.encoder.zero_index_min_interval_us,
      settings_.encoder.zero_index_correction_gain,
      settings_.encoder.direction,
      settings_.encoder.zero_index_minimum_separation_revolutions,
      settings_.characterization.dynamics_filter_cutoff_hz,
      settings_.characterization.dynamics_quantile,
      settings_.characterization.recommendation_safety_factor,
      settings_.motor_model.velocity_gain_forward_rad_s_per_duty,
      settings_.motor_model.velocity_gain_reverse_rad_s_per_duty,
      settings_.motor_model.time_constant_forward_s,
      settings_.motor_model.time_constant_reverse_s,
      settings_.motor_model.velocity_process_noise_rad_s2,
      settings_.motor_model.disturbance_process_noise_rad_s3,
      settings_.motor_model.encoder_measurement_noise_counts,
      static_cast<uint8_t>(settings_.velocity_estimator_method),
      static_cast<uint8_t>(settings_.velocity_acceleration_window_samples),
      settings_.encoder.zero_position_offset_ticks,
  };
  serial_link_.send(protocol::MessageId::Settings, sequence, &payload, sizeof(payload));
}

bool MachineApplication::sendCurrentCalibrationStatus(const uint16_t sequence) {
  const CurrentCalibrationStatusPayload payload{
      current_calibration_captured_mask_,
      current_calibration_capture_point_,
      static_cast<uint8_t>(current_calibration_last_result_),
      current_calibration_points_[0].sense_voltage_v,
      current_calibration_points_[0].reference_current_a,
      current_calibration_points_[1].sense_voltage_v,
      current_calibration_points_[1].reference_current_a,
      current_calibration_candidate_.gain_a_per_v,
      current_calibration_candidate_.offset_v,
  };
  return serial_link_.send(protocol::MessageId::CurrentCalibrationStatus, sequence,
                           &payload, sizeof(payload));
}

bool MachineApplication::sendRotorZeroCalibrationStatus(
    const uint16_t sequence) {
  const uint8_t calibration_state =
      rotor_zero_calibration_candidate_valid_ ? 2U : 0U;
  const RotorZeroCalibrationStatusPayload payload{
      calibration_state,
      static_cast<uint8_t>(rotor_zero_calibration_candidate_valid_ ? 1U : 0U),
      static_cast<uint8_t>(rotor_zero_calibration_last_result_),
      telemetry_.rotor_position_deg,
      rotor_zero_calibration_candidate_ticks_,
      settings_.encoder.zero_position_offset_ticks,
  };
  return serial_link_.send(protocol::MessageId::RotorZeroCalibrationStatus,
                           sequence, &payload, sizeof(payload));
}

bool MachineApplication::sendProfile(const VelocityProfileConfiguration& profile,
                                     const uint16_t sequence) {
  ProfilePayload payload{};
  payload.profile_id = profile.profile_id;
  payload.kind = static_cast<uint8_t>(profile.kind);
  std::strncpy(payload.name, profile.name, sizeof(payload.name) - 1U);
  payload.target_velocity_rad_s = profile.target_velocity_rad_s;
  payload.sine_mean_rad_s = profile.sine_mean_rad_s;
  payload.sine_amplitude_rad_s = profile.sine_amplitude_rad_s;
  payload.sine_frequency_hz = profile.sine_frequency_hz;
  payload.duration_ms = profile.duration_ms;
  payload.point_count = profile.point_count;
  for (size_t index = 0; index < kMaximumProfilePoints; ++index) {
    payload.points[index].time_ms = profile.points[index].time_ms;
    payload.points[index].velocity_rad_s = profile.points[index].velocity_rad_s;
  }
  return serial_link_.send(protocol::MessageId::ProfileConfiguration, sequence, &payload,
                           sizeof(payload));
}

void MachineApplication::sendProfiles(const uint16_t sequence) {
  for (uint8_t index = 0; index < settings_.profile_count; ++index) {
    sendProfile(settings_.profiles[index], sequence);
  }
}

bool MachineApplication::sendLoadConfiguration(const uint16_t sequence) {
  LoadConfigurationPayload payload{};
  payload.setting_id = settings_.load_setting.setting_id;
  payload.count = settings_.load_setting.count;
  for (uint8_t index = 0; index < settings_.load_setting.count; ++index) {
    const auto& source = settings_.load_setting.loads[index];
    payload.loads[index] = {source.load_id, source.position_deg, source.strength};
  }
  return serial_link_.send(protocol::MessageId::LoadConfiguration, sequence, &payload,
                           sizeof(payload));
}

void MachineApplication::sendTelemetry() {
  const TelemetryPayload payload{
      telemetry_.timestamp_us, telemetry_.last_zero_timestamp_us, telemetry_.encoder_count,
      telemetry_.last_zero_encoder_count, telemetry_.desired_velocity_rad_s,
      telemetry_.measured_velocity_rad_s, telemetry_.controller_output, telemetry_.current_a,
      telemetry_.supply_voltage_v, telemetry_.faults, telemetry_.profile_id,
      telemetry_.load_setting_id,
      static_cast<uint8_t>(telemetry_.state),
      telemetry_.controller_proportional_term, telemetry_.controller_integral_term,
      telemetry_.controller_derivative_term,
      telemetry_.rotor_position_deg, telemetry_.zero_index_sequence,
      telemetry_.zero_index_rejected_count,
  };
  serial_link_.send(protocol::MessageId::Telemetry, transmit_sequence_++, &payload,
                    sizeof(payload));
}

bool MachineApplication::sendCharacterizationResult(const uint16_t sequence) {
  if (!characterization_result_pending_) {
    return false;
  }
  const CharacterizationResultPayload payload{
      characterization_candidate_.start_duty_forward,
      characterization_candidate_.start_duty_reverse,
      characterization_candidate_.max_velocity_forward_rad_s,
      characterization_candidate_.max_velocity_reverse_rad_s,
      characterization_dynamics_candidate_.acceleration_forward_rad_s2,
      characterization_dynamics_candidate_.acceleration_reverse_rad_s2,
      characterization_dynamics_candidate_.jerk_forward_rad_s3,
      characterization_dynamics_candidate_.jerk_reverse_rad_s3,
      characterization_model_candidate_.velocity_gain_forward_rad_s_per_duty,
      characterization_model_candidate_.velocity_gain_reverse_rad_s_per_duty,
      characterization_model_candidate_.time_constant_forward_s,
      characterization_model_candidate_.time_constant_reverse_s,
  };
  return serial_link_.send(protocol::MessageId::CharacterizationResult, sequence, &payload,
                           sizeof(payload));
}

bool MachineApplication::sendCharacterizationStatus(const uint16_t sequence) {
  const CharacterizationStatusPayload payload{
      static_cast<uint8_t>(characterization_stage_),
      static_cast<uint8_t>(characterization_result_pending_),
      motor_.appliedDuty(),
      telemetry_.measured_velocity_rad_s,
  };
  return serial_link_.send(protocol::MessageId::CharacterizationStatus, sequence, &payload,
                           sizeof(payload));
}

}  // namespace mm
