#pragma once

#include <Arduino.h>

#include <cstdint>

#include "control/IncrementalVelocityController.hpp"
#include "control/CurrentCalibration.hpp"
#include "control/CharacterizationDynamics.hpp"
#include "control/EncoderActivityWatchdog.hpp"
#include "control/LowPassFilter.hpp"
#include "control/MotionLimiter.hpp"
#include "control/RotorPosition.hpp"
#include "control/VelocityEstimator.hpp"
#include "core/Types.hpp"
#include "drivers/QuadratureEncoder.hpp"
#include "drivers/Vnh2sp30MotorDriver.hpp"
#include "drivers/ZeroIndexSensor.hpp"
#include "profile/VelocityProfile.hpp"
#include "protocol/SerialLink.hpp"
#include "storage/SettingsStore.hpp"

namespace mm {

class MachineApplication final {
 public:
  static MachineApplication& instance();
  void begin();
  void runOnce();

  MachineApplication(const MachineApplication&) = delete;
  MachineApplication& operator=(const MachineApplication&) = delete;

 private:
  MachineApplication() = default;

  struct CommandDefinition {
    const char* name;
    const char* usage;
    void (MachineApplication::*handler)(int argc, char* argv[]);
  };

  static void frameThunk(void* context, const protocol::FrameView& frame);
  static void lineThunk(void* context, const char* line);
  void handleFrame(const protocol::FrameView& frame);
  void handleLine(const char* line);
  void controlTick(uint64_t scheduled_us);
  void updateSafety(uint64_t timestamp_us, float current_a);
  void sendHeartbeat(uint64_t timestamp_us);
  void sendSettings(uint16_t sequence);
  void sendProfiles(uint16_t sequence);
  bool sendLoadConfiguration(uint16_t sequence);
  bool sendProfile(const VelocityProfileConfiguration& profile, uint16_t sequence);
  void sendTelemetry();
  bool sendCharacterizationResult(uint16_t sequence);
  bool sendCharacterizationStatus(uint16_t sequence);
  bool sendCurrentCalibrationStatus(uint16_t sequence);
  void sendAck(uint16_t sequence, protocol::MessageId request, protocol::ResultCode result);
  void transitionToStopped();
  bool clearFaultsAndRecheck();
  const VelocityProfileConfiguration* selectedProfile() const;
  void printStatus() const;
  void updateCharacterization(uint64_t scheduled_us);
  void configureVelocityController();
  void updateCurrentCalibrationCapture(float sense_voltage_v);

  void commandHelp(int argc, char* argv[]);
  void commandStatus(int argc, char* argv[]);
  void commandArm(int argc, char* argv[]);
  void commandRun(int argc, char* argv[]);
  void commandStop(int argc, char* argv[]);
  void commandClearFault(int argc, char* argv[]);
  void commandStream(int argc, char* argv[]);
  void commandGain(int argc, char* argv[]);
  void commandProfile(int argc, char* argv[]);
  void commandConfig(int argc, char* argv[]);
  void commandMotor(int argc, char* argv[]);
  void commandDiagnostic(int argc, char* argv[]);
  void commandCurrent(int argc, char* argv[]);
  void commandVoltage(int argc, char* argv[]);
  void commandCharacterize(int argc, char* argv[]);

  static const CommandDefinition kCommands[];
  static constexpr size_t kMaximumArguments = 10;

  MachineSettings settings_{};
  uint16_t runtime_profile_id_ = 0;
  SettingsStore settings_store_{};
  QuadratureEncoder encoder_{};
  ZeroIndexSensor zero_index_{};
  Vnh2sp30MotorDriver motor_{};
  VelocityEstimator velocity_estimator_{};
  IncrementalVelocityController controller_{};
  MotionLimiter motion_limiter_{};
  EncoderActivityWatchdog encoder_watchdog_{};
  RotorPhaseTracker rotor_phase_tracker_{};
  LowPassFilter current_filter_{};
  VelocityProfile profile_{};
  VelocityProfileConfiguration tuning_profile_{};
  protocol::SerialLink serial_link_{};
  TelemetrySample telemetry_{};
  RunState state_ = RunState::Disarmed;
  uint32_t faults_ = FaultNone;
  uint64_t next_control_us_ = 0;
  uint64_t last_control_us_ = 0;
  uint64_t next_heartbeat_us_ = 0;
  uint64_t next_stream_us_ = 0;
  uint64_t next_supply_voltage_sample_us_ = 0;
  uint64_t next_characterization_status_us_ = 0;
  uint64_t manual_command_expiry_us_ = 0;
  float manual_duty_ = 0.0F;
  bool manual_raw_pwm_ = false;
  bool stream_enabled_ = false;
  bool supply_voltage_initialized_ = false;
  bool motor_initialized_ = false;
  uint16_t transmit_sequence_ = 0;

  enum class CharacterizationStage : uint8_t {
    Idle,
    ForwardDeadband,
    PauseBeforeReverseDeadband,
    ReverseDeadband,
    PauseBeforeForwardMaximum,
    ForwardMaximum,
    PauseBeforeReverseMaximum,
    ReverseMaximum,
  };
  CharacterizationStage characterization_stage_ = CharacterizationStage::Idle;
  uint64_t characterization_deadline_us_ = 0;
  float characterization_duty_ = 0.0F;
  float characterization_peak_velocity_ = 0.0F;
  uint8_t characterization_motion_samples_ = 0;
  MotorCharacteristics characterization_candidate_{};
  characterization::DynamicsEstimator characterization_dynamics_estimator_{};
  CharacterizationDynamicsResult characterization_dynamics_candidate_{};
  bool characterization_result_pending_ = false;
  bool characterization_notification_pending_ = false;
  bool characterization_status_pending_ = false;
  CurrentCalibrationPoint current_calibration_points_[2]{};
  CurrentCalibrationResult current_calibration_candidate_{};
  CurrentCalibrationAccumulator current_calibration_accumulator_{};
  uint8_t current_calibration_captured_mask_ = 0U;
  uint8_t current_calibration_capture_point_ = 0U;
  protocol::ResultCode current_calibration_last_result_ = protocol::ResultCode::Ok;
  bool current_calibration_status_pending_ = false;
};

}  // namespace mm
