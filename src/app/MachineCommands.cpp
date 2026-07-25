#include "app/MachineApplication.hpp"

#include <esp_timer.h>

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>

#include "control/CharacterizationSettings.hpp"
#include "protocol/SerialBandwidth.hpp"

namespace mm {
namespace {

bool parseFloat(const char* const text, float& value) {
  if (text == nullptr) {
    return false;
  }
  char* end = nullptr;
  errno = 0;
  const float parsed = std::strtof(text, &end);
  if (errno != 0 || end == text || *end != '\0' || !std::isfinite(parsed)) {
    return false;
  }
  value = parsed;
  return true;
}

bool parseUnsigned(const char* const text, uint32_t& value) {
  if (text == nullptr || *text == '-') {
    return false;
  }
  char* end = nullptr;
  errno = 0;
  const unsigned long parsed = std::strtoul(text, &end, 10);
  if (errno != 0 || end == text || *end != '\0' || parsed > UINT32_MAX) {
    return false;
  }
  value = static_cast<uint32_t>(parsed);
  return true;
}
}

const MachineApplication::CommandDefinition MachineApplication::kCommands[] = {
    {"help", "help - list commands", &MachineApplication::commandHelp},
    {"status", "status - show runtime state", &MachineApplication::commandStatus},
    {"arm", "arm - permit motor output", &MachineApplication::commandArm},
    {"run", "run - start the selected profile", &MachineApplication::commandRun},
    {"stop", "stop - stop and disarm", &MachineApplication::commandStop},
    {"clear", "clear - stop, recheck safety inputs, and clear inactive faults",
     &MachineApplication::commandClearFault},
    {"stream", "stream <on|off|rate HZ>", &MachineApplication::commandStream},
    {"gain", "gain <KP> <KI> <KD>", &MachineApplication::commandGain},
    {"profile", "profile <list|select ID>", &MachineApplication::commandProfile},
    {"config", "config <show|save|defaults>", &MachineApplication::commandConfig},
    {"motor", "motor <duty|raw> <-1..1> - armed timed bench test",
     &MachineApplication::commandMotor},
    {"diagnostic", "diagnostic <status|on|off> - configure EN/DIAG fault input",
     &MachineApplication::commandDiagnostic},
    {"current", "current <read|calibrate REFERENCE_A|protection status|on|off>",
     &MachineApplication::commandCurrent},
    {"voltage", "voltage <read|calibrate REFERENCE_V>",
     &MachineApplication::commandVoltage},
    {"characterize", "characterize <start CONFIRM_UNLOADED|status|abort|save [dynamics]|discard>",
     &MachineApplication::commandCharacterize},
};

void MachineApplication::handleLine(const char* const line) {
  char buffer[160]{};
  std::strncpy(buffer, line, sizeof(buffer) - 1U);
  char* arguments[kMaximumArguments]{};
  int count = 0;
  char* save = nullptr;
  for (char* token = strtok_r(buffer, " ", &save);
       token != nullptr && count < static_cast<int>(kMaximumArguments);
       token = strtok_r(nullptr, " ", &save)) {
    arguments[count++] = token;
  }
  if (count == 0) {
    return;
  }
  for (const auto& command : kCommands) {
    if (std::strcmp(arguments[0], command.name) == 0) {
      (this->*command.handler)(count, arguments);
      return;
    }
  }
  Serial.println("ERR unknown command; type help");
}

void MachineApplication::commandHelp(int, char*[]) {
  for (const auto& command : kCommands) {
    Serial.println(command.usage);
  }
}

void MachineApplication::printStatus() const {
  Serial.printf("state=%u faults=0x%08lx profile=%u stream=%s rate=%uHz current_trip=%s\r\n",
                static_cast<unsigned>(state_), static_cast<unsigned long>(faults_),
                settings_.selected_profile_id, stream_enabled_ ? "on" : "off",
                settings_.serial.stream_rate_hz,
                settings_.safety.current_sense_enabled ? "on" : "off");
  Serial.printf("target=%.3f rad/s velocity=%.3f rad/s duty=%.3f current=%.3f A vin=%.3f V count=%lld\r\n",
                telemetry_.desired_velocity_rad_s, telemetry_.measured_velocity_rad_s,
                motor_.appliedDuty(), telemetry_.current_a, telemetry_.supply_voltage_v,
                static_cast<long long>(telemetry_.encoder_count));
  if (telemetry_.zero_index_sequence > 0U) {
    Serial.printf("rotor=%.3f deg zero_time=%llu us zero_count=%lld zero_sequence=%lu rejected=%lu\r\n",
                  telemetry_.rotor_position_deg,
                  static_cast<unsigned long long>(telemetry_.last_zero_timestamp_us),
                  static_cast<long long>(telemetry_.last_zero_encoder_count),
                  static_cast<unsigned long>(telemetry_.zero_index_sequence),
                  static_cast<unsigned long>(telemetry_.zero_index_rejected_count));
  } else {
    Serial.printf("rotor=unreferenced zero_sequence=0 rejected=%lu\r\n",
                  static_cast<unsigned long>(telemetry_.zero_index_rejected_count));
  }
}

void MachineApplication::commandStatus(int, char*[]) { printStatus(); }

void MachineApplication::commandArm(int, char*[]) {
  if (faults_ != FaultNone || (state_ != RunState::Disarmed && state_ != RunState::Armed) ||
      characterization_stage_ != CharacterizationStage::Idle ||
      zero_index_calibration_stage_ != ZeroIndexCalibrationStage::Idle) {
    Serial.println("ERR stop, clear faults, and end calibration/characterization first");
    return;
  }
  state_ = RunState::Armed;
  Serial.println("OK armed");
}

void MachineApplication::commandRun(int, char*[]) {
  const auto* selected = selectedProfile();
  if (state_ != RunState::Armed || selected == nullptr ||
      zero_index_calibration_stage_ != ZeroIndexCalibrationStage::Idle) {
    Serial.println("ERR arm and select a valid profile first");
    return;
  }
  const uint64_t now = static_cast<uint64_t>(esp_timer_get_time());
  profile_.select(selected, now);
  motion_limiter_.reset();
  controller_.reset();
  state_ = RunState::Running;
  Serial.println("OK running");
}

void MachineApplication::commandStop(int, char*[]) {
  transitionToStopped();
  Serial.println("OK stopped and disarmed");
}

void MachineApplication::commandClearFault(int, char*[]) {
  if (clearFaultsAndRecheck()) {
    Serial.printf("OK faults cleared; vin=%.3f V, machine disarmed\r\n",
                  telemetry_.supply_voltage_v);
  } else {
    Serial.printf("ERR active fault remains: 0x%08lx vin=%.3f V\r\n",
                  static_cast<unsigned long>(faults_), telemetry_.supply_voltage_v);
  }
}

void MachineApplication::commandStream(const int argc, char* argv[]) {
  if (argc == 2 && std::strcmp(argv[1], "on") == 0) {
    stream_enabled_ = true;
    sendSettings(transmit_sequence_++);
    Serial.println("OK stream on");
  } else if (argc == 2 && std::strcmp(argv[1], "off") == 0) {
    stream_enabled_ = false;
    Serial.println("OK stream off");
  } else if (argc == 3 && std::strcmp(argv[1], "rate") == 0) {
    uint32_t rate = 0;
    const uint16_t maximum_rate =
        protocol::maximumTelemetryStreamRateHz(settings_.serial.baud);
    if (!parseUnsigned(argv[2], rate) || rate == 0U || rate > maximum_rate) {
      Serial.printf("ERR rate must be 1..%u Hz at the configured baud\r\n", maximum_rate);
      return;
    }
    settings_.serial.stream_rate_hz = static_cast<uint16_t>(rate);
    Serial.println("OK (use config save to persist)");
  } else {
    Serial.println("ERR usage: stream <on|off|rate HZ>");
  }
}

void MachineApplication::commandGain(const int argc, char* argv[]) {
  float kp = 0.0F;
  float ki = 0.0F;
  float kd = 0.0F;
  if (argc != 4 || state_ == RunState::Running || !parseFloat(argv[1], kp) ||
      !parseFloat(argv[2], ki) || !parseFloat(argv[3], kd) || kp < 0.0F || ki < 0.0F ||
      kd < 0.0F) {
    Serial.println("ERR usage: gain <nonnegative KP> <KI> <KD>; stop first");
    return;
  }
  settings_.control.kp = kp;
  settings_.control.ki = ki;
  settings_.control.kd = kd;
  configureVelocityController();
  Serial.println("OK gains loaded (use config save to persist)");
}

void MachineApplication::commandProfile(const int argc, char* argv[]) {
  if (argc == 2 && std::strcmp(argv[1], "list") == 0) {
    for (uint8_t index = 0; index < settings_.profile_count; ++index) {
      const auto& profile = settings_.profiles[index];
      Serial.printf("%u %s kind=%u duration=%lums\r\n", profile.profile_id, profile.name,
                    static_cast<unsigned>(profile.kind),
                    static_cast<unsigned long>(profile.duration_ms));
    }
    return;
  }
  uint32_t id = 0;
  if (argc != 3 || std::strcmp(argv[1], "select") != 0 || !parseUnsigned(argv[2], id) ||
      id > UINT16_MAX) {
    Serial.println("ERR usage: profile <list|select ID>");
    return;
  }
  const uint16_t old_id = runtime_profile_id_;
  runtime_profile_id_ = static_cast<uint16_t>(id);
  if (selectedProfile() == nullptr) {
    runtime_profile_id_ = old_id;
    Serial.println("ERR unknown profile");
    return;
  }
  Serial.println("OK profile selected for the next run only");
}

void MachineApplication::commandConfig(const int argc, char* argv[]) {
  if (argc != 2) {
    Serial.println("ERR usage: config <show|save|defaults>");
    return;
  }
  if (std::strcmp(argv[1], "show") == 0) {
    Serial.printf("schema=%lu baud=%lu period_us=%lu cpr=%lu direction=%d encoder=%u/%u vin_adc=%u diag=%s/%u\r\n",
                  static_cast<unsigned long>(settings_.schema_version),
                  static_cast<unsigned long>(settings_.serial.baud),
                  static_cast<unsigned long>(settings_.control.period_us),
                  static_cast<unsigned long>(settings_.encoder.counts_per_output_revolution),
                  settings_.motor_direction, settings_.pins.encoder_a, settings_.pins.encoder_b,
                  settings_.pins.supply_voltage_sense,
                  settings_.safety.driver_diagnostic_enabled ? "on" : "off",
                  settings_.pins.driver_diag);
    Serial.printf("kp=%.6f ki=%.6f kd=%.6f vmax=%.3f amax=%.3f jmax=%.3f imax=%.3f vin=%.3f..%.3f V gain=%.5f\r\n",
                  settings_.control.kp, settings_.control.ki, settings_.control.kd,
                  settings_.safety.max_velocity_rad_s,
                  settings_.safety.max_acceleration_rad_s2,
                  settings_.safety.max_jerk_rad_s3, settings_.safety.max_current_a,
                  settings_.safety.min_supply_voltage_v,
                  settings_.safety.max_supply_voltage_v,
                  settings_.supply_voltage.divider_gain);
  } else if (std::strcmp(argv[1], "save") == 0) {
    if (state_ != RunState::Disarmed) {
      Serial.println("ERR stop and disarm before saving configuration");
      return;
    }
    Serial.println(settings_store_.save(settings_) ? "OK saved" : "ERR validation/storage");
  } else if (std::strcmp(argv[1], "defaults") == 0 && state_ != RunState::Running) {
    settings_ = SettingsStore::defaults();
    Serial.println("OK defaults loaded; reboot after config save");
  } else {
    Serial.println("ERR invalid or unsafe config command");
  }
}

void MachineApplication::commandMotor(const int argc, char* argv[]) {
  float duty = 0.0F;
  const bool compensated = argc == 3 && std::strcmp(argv[1], "duty") == 0;
  const bool raw = argc == 3 && std::strcmp(argv[1], "raw") == 0;
  if ((!compensated && !raw) || state_ != RunState::Armed || !parseFloat(argv[2], duty) ||
      std::fabs(duty) > (raw ? settings_.safety.max_duty : 1.0F)) {
    Serial.println("ERR usage: arm; motor <duty|raw> <signed duty within configured limit>");
    return;
  }
  manual_duty_ = duty;
  manual_raw_pwm_ = raw;
  manual_command_expiry_us_ = static_cast<uint64_t>(esp_timer_get_time()) +
                              static_cast<uint64_t>(settings_.safety.command_timeout_ms) * 1000ULL;
  Serial.println("OK duty valid until command timeout");
}

void MachineApplication::commandDiagnostic(const int argc, char* argv[]) {
  if (argc == 2 && std::strcmp(argv[1], "status") == 0) {
    Serial.printf("driver_diagnostic=%s pin=%u active=%s\r\n",
                  settings_.safety.driver_diagnostic_enabled ? "on" : "off",
                  settings_.pins.driver_diag, motor_.diagnosticFault() ? "yes" : "no");
    return;
  }
  const bool requested_off = argc == 2 && std::strcmp(argv[1], "off") == 0;
  if (argc != 2 || (state_ != RunState::Disarmed && !(state_ == RunState::Fault && requested_off)) ||
      (std::strcmp(argv[1], "on") != 0 && std::strcmp(argv[1], "off") != 0)) {
    Serial.println("ERR usage: stop; diagnostic <status|on|off>");
    return;
  }
  settings_.safety.driver_diagnostic_enabled = std::strcmp(argv[1], "on") == 0;
  motor_.setDiagnosticEnabled(settings_.safety.driver_diagnostic_enabled);
  if (!settings_.safety.driver_diagnostic_enabled) {
    faults_ &= ~static_cast<uint32_t>(FaultDriverDiagnostic);
    if (faults_ == FaultNone && state_ == RunState::Fault) {
      state_ = RunState::Disarmed;
    }
  }
  Serial.println(settings_store_.save(settings_) ? "OK diagnostic setting saved"
                                                  : "ERR storage");
}

void MachineApplication::commandCurrent(const int argc, char* argv[]) {
  if (argc == 2 && std::strcmp(argv[1], "read") == 0) {
    Serial.printf("sense=%.4f V current=%.4f A gain=%.4f A/V offset=%.4f V\r\n",
                  motor_.currentSenseVoltage(), motor_.currentAmperes(),
                  settings_.motor.current_gain_a_per_v, settings_.motor.current_offset_v);
    return;
  }
  if (argc == 3 && std::strcmp(argv[1], "protection") == 0 &&
      std::strcmp(argv[2], "status") == 0) {
    Serial.printf("current_protection=%s limit=%.3f A pin=%u\r\n",
                  settings_.safety.current_sense_enabled ? "on" : "off",
                  settings_.safety.max_current_a, settings_.pins.current_sense);
    return;
  }
  if (argc == 3 && std::strcmp(argv[1], "protection") == 0 &&
      (std::strcmp(argv[2], "on") == 0 || std::strcmp(argv[2], "off") == 0)) {
    const bool enabling = std::strcmp(argv[2], "on") == 0;
    const bool disabling_faulted_input = state_ == RunState::Fault && !enabling;
    if ((state_ != RunState::Disarmed && !disabling_faulted_input) ||
        characterization_stage_ != CharacterizationStage::Idle) {
      Serial.println("ERR stop before changing current protection");
      return;
    }
    settings_.safety.current_sense_enabled = enabling;
    if (!enabling) {
      faults_ &= ~static_cast<uint32_t>(FaultOverCurrent);
      if (faults_ == FaultNone && state_ == RunState::Fault) {
        state_ = RunState::Disarmed;
      }
    }
    Serial.println(settings_store_.save(settings_) ? "OK current protection setting saved"
                                                    : "ERR storage");
    return;
  }
  float reference = 0.0F;
  const float adjusted_voltage = motor_.currentSenseVoltage() - settings_.motor.current_offset_v;
  if (argc != 3 || std::strcmp(argv[1], "calibrate") != 0 ||
      state_ == RunState::Running || !parseFloat(argv[2], reference) || reference <= 0.0F ||
      adjusted_voltage < 0.01F) {
    Serial.println("ERR usage: current <read|calibrate REFERENCE_A|protection status|on|off>");
    return;
  }
  settings_.motor.current_gain_a_per_v = reference / adjusted_voltage;
  motor_.setCurrentCalibration(settings_.motor.current_gain_a_per_v,
                               settings_.motor.current_offset_v);
  transitionToStopped();
  Serial.println(settings_store_.save(settings_) ? "OK calibrated and saved" : "ERR storage");
}

void MachineApplication::commandVoltage(const int argc, char* argv[]) {
  if (argc == 2 && std::strcmp(argv[1], "read") == 0) {
    Serial.printf("adc=%.5f V vin=%.5f V gain=%.6f offset=%.5f V\r\n",
                  motor_.supplySenseVoltage(16U), telemetry_.supply_voltage_v,
                  settings_.supply_voltage.divider_gain,
                  settings_.supply_voltage.input_offset_v);
    return;
  }
  float reference = 0.0F;
  if (argc != 3 || std::strcmp(argv[1], "calibrate") != 0 ||
      state_ == RunState::Running || !parseFloat(argv[2], reference) || reference <= 0.0F ||
      reference > 20.0F) {
    Serial.println("ERR usage: voltage calibrate <REFERENCE_V> while stopped");
    return;
  }
  const float sense_voltage = motor_.supplySenseVoltage(64U);
  const float corrected_reference = reference - settings_.supply_voltage.input_offset_v;
  if (sense_voltage < 0.01F || corrected_reference <= 0.0F ||
      sense_voltage > VoltageSenseConfiguration::kMaximumCalibrationSenseV) {
    Serial.println("ERR VIN sense is outside the reliable ADC calibration range");
    return;
  }
  settings_.supply_voltage.divider_gain = corrected_reference / sense_voltage;
  motor_.setSupplyVoltageCalibration(settings_.supply_voltage.divider_gain,
                                     settings_.supply_voltage.input_offset_v);
  telemetry_.supply_voltage_v = reference;
  supply_voltage_initialized_ = true;
  transitionToStopped();
  Serial.println(settings_store_.save(settings_) ? "OK voltage calibrated and saved"
                                                  : "ERR storage");
}

void MachineApplication::commandCharacterize(const int argc, char* argv[]) {
  if (argc == 2 && std::strcmp(argv[1], "status") == 0) {
    Serial.printf("stage=%u pending=%s duty=%.3f velocity=%.3f start_fwd=%.3f start_rev=%.3f max_fwd=%.3f max_rev=%.3f accel_fwd=%.3f accel_rev=%.3f jerk_fwd=%.3f jerk_rev=%.3f model_gain_fwd=%.3f model_gain_rev=%.3f model_tau_fwd=%.4f model_tau_rev=%.4f\r\n",
                  static_cast<unsigned>(characterization_stage_),
                  characterization_result_pending_ ? "yes" : "no", characterization_duty_,
                  telemetry_.measured_velocity_rad_s,
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
                  characterization_model_candidate_.time_constant_reverse_s);
    return;
  }
  if (argc == 2 && std::strcmp(argv[1], "abort") == 0) {
    transitionToStopped();
    characterization_result_pending_ = false;
    characterization_notification_pending_ = false;
    Serial.println("OK characterization aborted and disarmed");
    return;
  }
  if (argc == 2 && std::strcmp(argv[1], "discard") == 0 &&
      characterization_result_pending_) {
    characterization_result_pending_ = false;
    characterization_notification_pending_ = false;
    Serial.println("OK characterization result discarded");
    return;
  }
  const bool save_characterization =
      (argc == 2 || (argc == 3 && std::strcmp(argv[2], "dynamics") == 0)) &&
      std::strcmp(argv[1], "save") == 0;
  if (save_characterization &&
      characterization_result_pending_ && state_ == RunState::Disarmed) {
    MachineSettings candidate{};
    if (!characterization::prepareCharacterizedSettings(
            settings_, characterization_candidate_, candidate) ||
        !characterization::applyRecommendedDynamics(
            characterization_dynamics_candidate_,
            settings_.characterization.recommendation_safety_factor,
            argc == 3, argc == 3, candidate)) {
      Serial.println("ERR characterization result is invalid");
      return;
    }
    candidate.motor_model = characterization_model_candidate_;
    if (!SettingsStore::validate(candidate)) {
      Serial.println("ERR characterized motor model is invalid");
      return;
    }
    if (!settings_store_.save(candidate)) {
      Serial.println("ERR characterization result remains pending; storage failed");
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
    Serial.printf("OK characterization result applied and saved; vmax=%.3f rad/s\r\n",
                  settings_.safety.max_velocity_rad_s);
    return;
  }
  if (argc != 3 || std::strcmp(argv[1], "start") != 0 ||
      std::strcmp(argv[2], "CONFIRM_UNLOADED") != 0 || state_ != RunState::Armed ||
      faults_ != FaultNone || characterization_stage_ != CharacterizationStage::Idle ||
      zero_index_calibration_stage_ != ZeroIndexCalibrationStage::Idle ||
      characterization_result_pending_) {
    Serial.println("ERR review pending result or remove load, arm, then: characterize start CONFIRM_UNLOADED");
    return;
  }
  characterization_candidate_ = settings_.motor;
  characterization_model_candidate_ = settings_.motor_model;
  characterization_dynamics_candidate_ = {};
  characterization_dynamics_estimator_.configure(
      settings_.characterization.dynamics_filter_cutoff_hz,
      settings_.characterization.dynamics_quantile);
  characterization_stage_ = CharacterizationStage::ForwardDeadband;
  characterization_status_pending_ = true;
  next_characterization_status_us_ = 0U;
  characterization_duty_ = settings_.characterization.duty_step;
  characterization_motion_samples_ = 0;
  characterization_breakaway_start_count_ = encoder_.count();
  characterization_motion_confirmed_ = false;
  characterization_breakaway_trial_pause_ = false;
  characterization_breakaway_trials_.reset();
  characterization_full_duty_trial_pause_ = false;
  characterization_full_duty_trials_.reset();
  characterization_peak_velocity_ = 0.0F;
  characterization_deadline_us_ = static_cast<uint64_t>(esp_timer_get_time()) +
      static_cast<uint64_t>(settings_.characterization.settle_ms) * 1000ULL;
  Serial.println("OK characterization started; stop/abort remains available");
}

}  // namespace mm
