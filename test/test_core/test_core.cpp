#include <unity.h>

#include <array>
#include <cmath>
#include <cstdint>

#include "control/IncrementalVelocityController.hpp"
#include "control/CharacterizationMetrics.hpp"
#include "control/CharacterizationDynamics.hpp"
#include "control/CharacterizationSettings.hpp"
#include "control/CurrentCalibration.hpp"
#include "control/EncoderActivityWatchdog.hpp"
#include "control/LowPassFilter.hpp"
#include "control/MotionLimiter.hpp"
#include "control/MotorDeadband.hpp"
#include "control/PositionController.hpp"
#include "control/RotorPosition.hpp"
#include "control/StatusLedPattern.hpp"
#include "control/ZeroIndexCalibration.hpp"
#include "control/VelocityEstimator.hpp"
#include "profile/VelocityProfile.hpp"
#include "profile/VelocityStepSequence.hpp"
#include "profile/ProfileCollection.hpp"
#include "protocol/Crc16.hpp"
#include "protocol/SerialBandwidth.hpp"

using namespace mm;

void test_crc_standard_vector() {
  constexpr std::array<uint8_t, 9> data{'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  TEST_ASSERT_EQUAL_HEX16(0x29B1, protocol::crc16CcittFalse(data.data(), data.size()));
}

void test_status_led_maps_machine_states_to_distinct_colors() {
  StatusLedPattern pattern;
  const StatusLedConfiguration configuration{};
  const uint8_t level = configuration.brightness;

  TEST_ASSERT_TRUE(
      (pattern.color(0U, RunState::Disarmed, FaultNone, configuration) ==
       StatusLedColor{0U, level, 0U}));
  TEST_ASSERT_TRUE(
      (pattern.color(0U, RunState::Armed, FaultNone, configuration) ==
       StatusLedColor{level, 0U, 0U}));
  TEST_ASSERT_TRUE(
      (pattern.color(0U, RunState::Running, FaultNone, configuration) ==
       StatusLedColor{level, 0U, 0U}));
}

void test_status_led_boot_pattern_identifies_pixel_order() {
  constexpr uint8_t level = 24U;
  TEST_ASSERT_TRUE(
      (StatusLedPattern::bootOrderColor(0U, level) ==
       StatusLedColor{level, 0U, 0U}));
  TEST_ASSERT_TRUE(
      (StatusLedPattern::bootOrderColor(1U, level) ==
       StatusLedColor{0U, 0U, level}));
  TEST_ASSERT_TRUE(
      (StatusLedPattern::bootOrderColor(2U, level) ==
       StatusLedColor{level, level, level}));
  TEST_ASSERT_TRUE(
      (StatusLedPattern::bootOrderColor(3U, level) == StatusLedColor{}));
  TEST_ASSERT_EQUAL_UINT16(
      5000U, StatusLedConfiguration::kBootOrderTestDurationMs);
}

void test_status_led_command_overlay_blinks_twice_then_restores_state() {
  StatusLedPattern pattern;
  const StatusLedConfiguration configuration{};
  const uint8_t level = configuration.brightness;
  pattern.notifyCommandReceived(1000000ULL);

  TEST_ASSERT_TRUE(
      (pattern.color(1000000ULL, RunState::Disarmed, FaultNone, configuration) ==
       StatusLedColor{level, level, level}));
  TEST_ASSERT_TRUE(
      (pattern.color(1060000ULL, RunState::Disarmed, FaultNone, configuration) ==
       StatusLedColor{}));
  TEST_ASSERT_TRUE(
      (pattern.color(1120000ULL, RunState::Disarmed, FaultNone, configuration) ==
       StatusLedColor{level, level, level}));
  TEST_ASSERT_TRUE(
      (pattern.color(1180000ULL, RunState::Disarmed, FaultNone, configuration) ==
       StatusLedColor{0U, level, 0U}));
}

void test_status_led_command_overlay_is_suppressed_while_armed() {
  StatusLedPattern pattern;
  const StatusLedConfiguration configuration{};
  pattern.notifyCommandReceived(0U);
  TEST_ASSERT_TRUE(
      (pattern.color(0U, RunState::Armed, FaultNone, configuration) ==
       StatusLedColor{configuration.brightness, 0U, 0U}));
}

void test_status_led_running_trail_reverses_with_motor_direction() {
  constexpr uint8_t level = 24U;
  TEST_ASSERT_TRUE(
      (StatusLedPattern::runningColor(0U, 1U, 1, level) ==
       StatusLedColor{level / 2U, 0U, 0U}));
  TEST_ASSERT_TRUE(
      (StatusLedPattern::runningColor(1U, 1U, 1, level) ==
       StatusLedColor{level, 0U, 0U}));
  TEST_ASSERT_TRUE(
      (StatusLedPattern::runningColor(3U, 1U, -1, level) ==
       StatusLedColor{level, 0U, 0U}));
  TEST_ASSERT_TRUE(
      (StatusLedPattern::runningColor(0U, 1U, -1, level) ==
       StatusLedColor{level / 2U, 0U, 0U}));
}

void test_status_led_fault_overrides_command_blink() {
  StatusLedPattern pattern;
  const StatusLedConfiguration configuration{};
  pattern.notifyCommandReceived(0U);

  TEST_ASSERT_TRUE(
      (pattern.color(0U, RunState::Fault, FaultOverCurrent, configuration) ==
       StatusLedColor{configuration.brightness, 0U, 0U}));
  TEST_ASSERT_TRUE(
      (pattern.color(250000ULL, RunState::Fault, FaultOverCurrent,
                     configuration) == StatusLedColor{}));
}

void test_characterization_dynamics_estimates_constant_acceleration() {
  characterization::DynamicsEstimator estimator;
  estimator.configure(20.0F, 0.95F);
  constexpr float dt_s = 0.002F;
  constexpr float acceleration_rad_s2 = 10.0F;
  for (uint32_t sample = 0U; sample < 1000U; ++sample) {
    estimator.update(acceleration_rad_s2 * static_cast<float>(sample) * dt_s, dt_s);
  }
  TEST_ASSERT_TRUE(estimator.valid());
  TEST_ASSERT_FLOAT_WITHIN(0.25F, acceleration_rad_s2, estimator.accelerationRadS2());
  TEST_ASSERT_LESS_THAN_FLOAT(5.0F, estimator.jerkRadS3());
}

void test_characterization_dynamics_uses_robust_quantile() {
  P2QuantileEstimator estimator;
  estimator.configure(0.95F);
  for (uint32_t sample = 0U; sample < 1000U; ++sample) estimator.add(10.0F);
  estimator.add(10000.0F);
  TEST_ASSERT_FLOAT_WITHIN(0.1F, 10.0F, estimator.value());
}

void test_breakaway_trials_retain_highest_duty() {
  characterization::BreakawayTrialAccumulator trials;
  constexpr uint8_t required_trials = 5U;
  TEST_ASSERT_TRUE(trials.add(0.18F));
  TEST_ASSERT_TRUE(trials.add(0.21F));
  TEST_ASSERT_TRUE(trials.add(0.19F));
  TEST_ASSERT_TRUE(trials.add(0.23F));
  TEST_ASSERT_FALSE(trials.complete(required_trials));
  TEST_ASSERT_TRUE(trials.add(0.20F));
  TEST_ASSERT_TRUE(trials.complete(required_trials));
  TEST_ASSERT_EQUAL_UINT8(required_trials, trials.count());
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, 0.23F, trials.maximumDuty());

  trials.reset();
  TEST_ASSERT_EQUAL_UINT8(0U, trials.count());
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, 0.0F, trials.maximumDuty());
}

void test_full_duty_trials_retain_highest_measurements() {
  characterization::FullDutyTrialAccumulator trials;
  constexpr uint8_t required_trials = 5U;
  TEST_ASSERT_TRUE(trials.add(100.0F, 12.0F, 120.0F, 110.0F, 0.20F));
  TEST_ASSERT_TRUE(trials.add(120.0F, 11.0F, 130.0F, 132.0F, 0.30F));
  TEST_ASSERT_TRUE(trials.add(110.0F, 15.0F, 125.0F, 121.0F, 0.25F));
  TEST_ASSERT_TRUE(trials.add(115.0F, 13.0F, 150.0F, 126.0F, 0.27F));
  TEST_ASSERT_FALSE(trials.complete(required_trials));
  TEST_ASSERT_TRUE(trials.add(105.0F, 14.0F, 140.0F, 115.0F, 0.22F));
  TEST_ASSERT_TRUE(trials.complete(required_trials));
  TEST_ASSERT_FLOAT_WITHIN(
      0.001F, 120.0F, trials.maximumVelocityRadS());
  TEST_ASSERT_FLOAT_WITHIN(
      0.001F, 15.0F, trials.maximumAccelerationRadS2());
  TEST_ASSERT_FLOAT_WITHIN(
      0.001F, 150.0F, trials.maximumJerkRadS3());
  TEST_ASSERT_FLOAT_WITHIN(
      0.001F, 132.0F, trials.modelGainRadSPerDuty());
  TEST_ASSERT_FLOAT_WITHIN(
      0.001F, 0.30F, trials.modelTimeConstantS());
}

void test_breakaway_requires_one_correctly_directed_output_revolution() {
  EncoderConfiguration encoder{};
  encoder.counts_per_output_revolution = 144U;
  encoder.direction = 1;

  TEST_ASSERT_TRUE(characterization::breakawayVelocityMatchesDirection(
      1.0F, 1, 1.0F));
  TEST_ASSERT_FALSE(characterization::breakawayTravelComplete(
      1000, 1143, 1, encoder));
  TEST_ASSERT_TRUE(characterization::breakawayTravelComplete(
      1000, 1144, 1, encoder));
  TEST_ASSERT_FALSE(characterization::breakawayTravelComplete(
      1000, 856, 1, encoder));
}

void test_breakaway_travel_handles_reverse_and_inverted_encoder() {
  EncoderConfiguration encoder{};
  encoder.counts_per_output_revolution = 144U;
  encoder.direction = -1;

  TEST_ASSERT_TRUE(characterization::breakawayVelocityMatchesDirection(
      -1.0F, -1, 1.0F));
  TEST_ASSERT_FALSE(characterization::breakawayVelocityMatchesDirection(
      1.0F, -1, 1.0F));
  TEST_ASSERT_TRUE(characterization::breakawayTravelComplete(
      1000, 1144, -1, encoder));
  TEST_ASSERT_FALSE(characterization::breakawayTravelComplete(
      1000, 856, -1, encoder));
}

void test_motor_identifier_recovers_first_order_step_model() {
  characterization::FirstOrderMotorIdentifier identifier;
  identifier.reset();
  constexpr float dt_s = 0.01F;
  constexpr float duty = 0.5F;
  constexpr float expected_gain = 120.0F;
  constexpr float expected_time_constant_s = 0.20F;
  const float a = std::exp(-dt_s / expected_time_constant_s);
  float velocity = 0.0F;
  for (uint32_t sample = 0U; sample < 1000U; ++sample) {
    identifier.update(velocity, duty, dt_s);
    velocity = a * velocity + expected_gain * (1.0F - a) * duty;
  }
  float gain = 0.0F;
  float time_constant_s = 0.0F;
  TEST_ASSERT_TRUE(identifier.result(gain, time_constant_s));
  TEST_ASSERT_FLOAT_WITHIN(0.5F, expected_gain, gain);
  TEST_ASSERT_FLOAT_WITHIN(0.005F, expected_time_constant_s, time_constant_s);
}

void test_create_profile_frame_crc_vector() {
  std::array<uint8_t, 8U + 169U> protected_bytes{};
  protected_bytes[0] = 1U;     // Protocol version.
  protected_bytes[2] = 0x24U;  // CREATE_PROFILE 0x0124, little endian.
  protected_bytes[3] = 0x01U;
  protected_bytes[4] = 0x34U;  // Sequence 0x1234, little endian.
  protected_bytes[5] = 0x12U;
  protected_bytes[6] = 169U;   // Bounded payload size, little endian.
  TEST_ASSERT_EQUAL_HEX16(
      0xA828U, protocol::crc16CcittFalse(protected_bytes.data(), protected_bytes.size()));
}

void test_load_configuration_frame_crc_vector() {
  std::array<uint8_t, 8U + 86U> protected_bytes{};
  protected_bytes[0] = 1U;
  protected_bytes[2] = 0x32U;  // SET_LOAD_CONFIGURATION 0x0132.
  protected_bytes[3] = 0x01U;
  protected_bytes[4] = 0x78U;
  protected_bytes[5] = 0x56U;
  protected_bytes[6] = 86U;
  TEST_ASSERT_EQUAL_HEX16(
      0xD9C3U, protocol::crc16CcittFalse(protected_bytes.data(), protected_bytes.size()));
}

void test_characterization_result_frame_crc_vector() {
  std::array<uint8_t, 8U + 48U> protected_bytes{};
  protected_bytes[0] = 1U;
  protected_bytes[2] = 0x10U;  // CHARACTERIZATION_RESULT 0x0310.
  protected_bytes[3] = 0x03U;
  protected_bytes[4] = 0xBCU;
  protected_bytes[5] = 0x9AU;
  protected_bytes[6] = 48U;
  TEST_ASSERT_EQUAL_HEX16(
      0xEA4BU, protocol::crc16CcittFalse(protected_bytes.data(), protected_bytes.size()));
}

void test_settings_schema_24_frame_crc_vector() {
  std::array<uint8_t, 8U + 204U> protected_bytes{};
  protected_bytes[0] = 1U;
  protected_bytes[2] = 0x01U;  // SETTINGS 0x0101.
  protected_bytes[3] = 0x01U;
  protected_bytes[4] = 0x34U;
  protected_bytes[5] = 0x12U;
  protected_bytes[6] = 204U;
  TEST_ASSERT_EQUAL_HEX16(
      0xC565U, protocol::crc16CcittFalse(protected_bytes.data(), protected_bytes.size()));
}

void test_settings_schema_27_frame_crc_vector() {
  std::array<uint8_t, 8U + 232U> protected_bytes{};
  protected_bytes[0] = 1U;
  protected_bytes[2] = 0x01U;  // SETTINGS 0x0101.
  protected_bytes[3] = 0x01U;
  protected_bytes[4] = 0x34U;
  protected_bytes[5] = 0x12U;
  protected_bytes[6] = 232U;
  TEST_ASSERT_EQUAL_HEX16(
      0x2F10U,
      protocol::crc16CcittFalse(protected_bytes.data(),
                                protected_bytes.size()));
}

void test_settings_schema_28_frame_crc_vector() {
  std::array<uint8_t, 8U + 240U> protected_bytes{};
  protected_bytes[0] = 1U;
  protected_bytes[2] = 0x01U;  // SETTINGS 0x0101.
  protected_bytes[3] = 0x01U;
  protected_bytes[4] = 0x34U;
  protected_bytes[5] = 0x12U;
  protected_bytes[6] = 240U;
  TEST_ASSERT_EQUAL_HEX16(
      0xF303U,
      protocol::crc16CcittFalse(protected_bytes.data(),
                                protected_bytes.size()));
}

void test_position_target_frame_crc_vector() {
  std::array<uint8_t, 8U + 4U> protected_bytes{
      1U, 0U, 0x07U, 0x02U, 0x34U, 0x12U, 4U, 0U,
      0x00U, 0x00U, 0xB4U, 0x42U};
  TEST_ASSERT_EQUAL_HEX16(
      0xFFD6U,
      protocol::crc16CcittFalse(protected_bytes.data(),
                                protected_bytes.size()));
}

void test_position_controller_uses_shortest_wrapped_error() {
  PositionController controller;
  PositionControlConfiguration configuration{};
  configuration.kp = 2.0F;
  configuration.ki = 0.0F;
  configuration.kd = 0.0F;
  configuration.max_velocity_rad_s = 10.0F;
  controller.configure(configuration);

  const float forward = controller.update(2.0F, 358.0F, 0.002F);
  controller.reset();
  const float reverse = controller.update(358.0F, 2.0F, 0.002F);

  TEST_ASSERT_FLOAT_WITHIN(0.0001F, 4.0F, controller.errorDegrees() * -1.0F);
  TEST_ASSERT_TRUE(forward > 0.0F);
  TEST_ASSERT_TRUE(reverse < 0.0F);
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, std::fabs(forward),
                           std::fabs(reverse));
}

void test_position_controller_bounds_velocity_demand() {
  PositionController controller;
  PositionControlConfiguration configuration{};
  configuration.kp = 100.0F;
  configuration.max_velocity_rad_s = 3.0F;
  controller.configure(configuration);
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, 3.0F,
                           controller.update(90.0F, 0.0F, 0.002F));
}

void test_position_controller_avoids_velocity_deadband_until_intentional_zero() {
  PositionController controller;
  PositionControlConfiguration configuration{};
  configuration.kp = 0.1F;
  configuration.ki = 0.0F;
  configuration.kd = 0.0F;
  configuration.tolerance_deg = 1.0F;
  configuration.minimum_velocity_forward_rad_s = 2.0F;
  configuration.minimum_velocity_reverse_rad_s = 3.0F;
  controller.configure(configuration);

  TEST_ASSERT_FLOAT_WITHIN(
      0.0001F, 2.0F, controller.update(2.0F, 0.0F, 0.002F));
  controller.reset();
  TEST_ASSERT_FLOAT_WITHIN(
      0.0001F, -3.0F, controller.update(358.0F, 0.0F, 0.002F));
  controller.reset();
  TEST_ASSERT_FLOAT_WITHIN(
      0.0001F, 0.0F, controller.update(0.5F, 0.0F, 0.002F));

  configuration.kp = 0.0F;
  controller.configure(configuration);
  controller.reset();
  TEST_ASSERT_FLOAT_WITHIN(
      0.0001F, 2.0F, controller.update(2.0F, 0.0F, 0.002F));
}

void test_position_velocity_output_uses_directional_motor_deadband() {
  MotorCharacteristics characteristics{};
  characteristics.start_duty_forward = 0.20F;
  characteristics.start_duty_reverse = 0.30F;
  SafetyConfiguration safety{};
  safety.max_duty = 0.90F;

  TEST_ASSERT_FLOAT_WITHIN(
      0.0001F, 0.60F,
      compensateMotorDeadband(0.50F, characteristics, safety));
  TEST_ASSERT_FLOAT_WITHIN(
      0.0001F, -0.65F,
      compensateMotorDeadband(-0.50F, characteristics, safety));
  TEST_ASSERT_EQUAL_FLOAT(
      0.0F, compensateMotorDeadband(0.0F, characteristics, safety));
}

void test_rotor_zero_calibration_frame_crc_vector() {
  std::array<uint8_t, 8U + 1U> protected_bytes{};
  protected_bytes[0] = 1U;
  protected_bytes[2] = 0x03U;  // ROTOR_ZERO_CALIBRATION 0x0303.
  protected_bytes[3] = 0x03U;
  protected_bytes[4] = 0x34U;
  protected_bytes[5] = 0x12U;
  protected_bytes[6] = 1U;
  TEST_ASSERT_EQUAL_HEX16(
      0xA124U, protocol::crc16CcittFalse(protected_bytes.data(), protected_bytes.size()));

  std::array<uint8_t, 8U + 15U> status_bytes{};
  status_bytes[0] = 1U;
  status_bytes[2] = 0x04U;  // ROTOR_ZERO_CALIBRATION_STATUS 0x0304.
  status_bytes[3] = 0x03U;
  status_bytes[4] = 0x34U;
  status_bytes[5] = 0x12U;
  status_bytes[6] = 15U;
  TEST_ASSERT_EQUAL_HEX16(
      0x75C9U, protocol::crc16CcittFalse(status_bytes.data(), status_bytes.size()));
}

void test_zero_index_hysteresis_calibration_frame_crc_vector() {
  std::array<uint8_t, 8U + 2U> protected_bytes{};
  protected_bytes[0] = 1U;
  protected_bytes[2] = 0x05U;  // ZERO_INDEX_HYSTERESIS_CALIBRATION 0x0305.
  protected_bytes[3] = 0x03U;
  protected_bytes[4] = 0x34U;
  protected_bytes[5] = 0x12U;
  protected_bytes[6] = 2U;
  TEST_ASSERT_EQUAL_HEX16(
      0x9ADCU, protocol::crc16CcittFalse(protected_bytes.data(), protected_bytes.size()));
}

void test_velocity_step_sequence_holds_each_level_and_finishes_at_zero() {
  VelocityStepSequence sequence;
  std::array<float, kMaximumVelocityTestLevels> levels{};
  levels[0] = 5.0F;
  levels[1] = 18.0F;
  levels[2] = 9.0F;
  sequence.start(levels, 3U, 500U, 1000000ULL);

  TEST_ASSERT_FLOAT_WITHIN(0.0001F, 0.0F, sequence.target(999999ULL));
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, 5.0F, sequence.target(1000000ULL));
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, 5.0F, sequence.target(1499999ULL));
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, 18.0F, sequence.target(1500000ULL));
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, 9.0F, sequence.target(2499999ULL));
  TEST_ASSERT_TRUE(sequence.finished(2500000ULL));
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, 0.0F, sequence.target(2500000ULL));
}

void test_velocity_sequence_frame_crc_vector() {
  std::array<uint8_t, 8U + 72U> protected_bytes{};
  protected_bytes[0] = 1U;
  protected_bytes[2] = 0x06U;  // START_VELOCITY_SEQUENCE 0x0206.
  protected_bytes[3] = 0x02U;
  protected_bytes[4] = 0x34U;
  protected_bytes[5] = 0x12U;
  protected_bytes[6] = 72U;
  TEST_ASSERT_EQUAL_HEX16(
      0xEC74U, protocol::crc16CcittFalse(protected_bytes.data(), protected_bytes.size()));
}

void test_telemetry_rate_respects_uart_bandwidth() {
  TEST_ASSERT_EQUAL_UINT16(84U, protocol::maximumTelemetryStreamRateHz(115200U));
  TEST_ASSERT_EQUAL_UINT16(84U, protocol::constrainTelemetryStreamRateHz(115200U, 200U));
  TEST_ASSERT_EQUAL_UINT16(50U, protocol::constrainTelemetryStreamRateHz(115200U, 50U));
  TEST_ASSERT_EQUAL_UINT16(500U, protocol::maximumTelemetryStreamRateHz(921600U));
  const SerialConfiguration defaults{};
  TEST_ASSERT_LESS_OR_EQUAL_UINT16(
      protocol::maximumTelemetryStreamRateHz(defaults.baud), defaults.stream_rate_hz);
  TEST_ASSERT_EQUAL_UINT32(28U, MachineSettings::kSchemaVersion);
}

void test_bearing_configuration_frame_crc_vector() {
  std::array<uint8_t, 8U + 2U> protected_bytes{};
  protected_bytes[0] = 1U;
  protected_bytes[2] = 0x35U;  // SET_BEARING_CONFIGURATION 0x0135.
  protected_bytes[3] = 0x01U;
  protected_bytes[4] = 0x34U;
  protected_bytes[5] = 0x12U;
  protected_bytes[6] = 2U;
  protected_bytes[8] = 7U;
  protected_bytes[9] = 1U;
  TEST_ASSERT_EQUAL_HEX16(
      0x3555U, protocol::crc16CcittFalse(protected_bytes.data(), protected_bytes.size()));
}

void test_incremental_controller_scales_integral_by_time() {
  ControlConfiguration configuration{};
  configuration.kp = 0.0F;
  configuration.ki = 2.0F;
  configuration.kd = 0.0F;
  configuration.error_deadband_rad_s = 0.0F;
  IncrementalVelocityController controller;
  controller.configure(configuration);
  for (int sample = 0; sample < 10; ++sample) {
    controller.update(1.0F, 0.0F, 0.01F);
  }
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, 0.2F, controller.output());
}

void test_incremental_controller_does_not_wind_up() {
  ControlConfiguration configuration{};
  configuration.kp = 0.0F;
  configuration.ki = 100.0F;
  configuration.kd = 0.0F;
  configuration.error_deadband_rad_s = 0.0F;
  IncrementalVelocityController controller;
  controller.configure(configuration);
  for (int sample = 0; sample < 100; ++sample) {
    controller.update(10.0F, 0.0F, 0.01F);
  }
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, 1.0F, controller.output());
  controller.update(0.0F, 10.0F, 0.01F);
  TEST_ASSERT_TRUE(controller.output() < 1.0F);
}

void test_incremental_controller_reports_each_delta_term() {
  ControlConfiguration configuration{};
  configuration.kp = 2.0F;
  configuration.ki = 3.0F;
  configuration.kd = 4.0F;
  configuration.error_deadband_rad_s = 0.0F;
  configuration.output_min = -1000.0F;
  configuration.output_max = 1000.0F;
  IncrementalVelocityController controller;
  controller.configure(configuration);
  controller.update(10.0F, 0.0F, 0.1F);
  controller.update(10.0F, 2.0F, 0.1F);
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, -4.0F, controller.proportionalTerm());
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, 2.4F, controller.integralTerm());
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, -80.0F, controller.derivativeTerm());
}

void test_low_pass_filter_uses_cutoff_and_elapsed_time() {
  LowPassFilter filter;
  filter.configure(10.0F);
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, 0.0F, filter.update(0.0F, 0.002F));
  const float first = filter.update(1.0F, 0.002F);
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, 0.1180886F, first);
  TEST_ASSERT_TRUE(filter.update(1.0F, 0.002F) > first);
  filter.reset();
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, 1.0F, filter.update(1.0F, 0.002F));
}

void test_two_point_current_calibration_calculates_gain_and_offset() {
  CurrentCalibrationResult result{};
  TEST_ASSERT_TRUE(calculateCurrentCalibration(
      {0.100F, 0.0F}, {0.200F, 1.0F}, result));
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, 10.0F, result.gain_a_per_v);
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, 0.100F, result.offset_v);
}

void test_two_point_current_calibration_rejects_insufficient_span() {
  CurrentCalibrationResult result{};
  TEST_ASSERT_FALSE(calculateCurrentCalibration(
      {0.100F, 0.0F}, {0.1005F, 1.0F}, result));
  TEST_ASSERT_FALSE(calculateCurrentCalibration(
      {0.200F, 0.0F}, {0.100F, 1.0F}, result));
}

void test_current_calibration_capture_averages_incrementally() {
  CurrentCalibrationAccumulator accumulator;
  accumulator.start(1.25F);
  float average = 0.0F;
  for (uint8_t sample = 0; sample < 63U; ++sample) {
    TEST_ASSERT_FALSE(accumulator.addSample(0.100F + static_cast<float>(sample) * 0.001F,
                                            average));
  }
  TEST_ASSERT_TRUE(accumulator.active());
  TEST_ASSERT_TRUE(accumulator.addSample(0.163F, average));
  TEST_ASSERT_FALSE(accumulator.active());
  TEST_ASSERT_EQUAL_UINT8(64U, accumulator.sampleCount());
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, 0.1315F, average);
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, 1.25F, accumulator.referenceCurrentA());
}

void test_motion_limiter_honors_acceleration_and_jerk() {
  SafetyConfiguration configuration{};
  configuration.max_velocity_rad_s = 100.0F;
  configuration.max_acceleration_rad_s2 = 10.0F;
  configuration.max_jerk_rad_s3 = 20.0F;
  MotionLimiter limiter;
  limiter.configure(configuration);
  limiter.reset();
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, 0.2F, limiter.update(100.0F, 0.1F));
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, 2.0F, limiter.acceleration());
}

void test_motion_limiter_tracks_feasible_ramp_without_ripple() {
  SafetyConfiguration configuration{};
  configuration.max_velocity_rad_s = 100.0F;
  configuration.max_acceleration_rad_s2 = 10.0F;
  configuration.max_jerk_rad_s3 = 20.0F;
  MotionLimiter limiter;
  limiter.configure(configuration);
  limiter.reset();

  constexpr float dt_s = 0.002F;
  constexpr float ramp_acceleration_rad_s2 = 5.0F;
  float previous_acceleration = limiter.acceleration();
  float maximum_settled_error = 0.0F;
  for (uint32_t sample = 0U; sample <= 2500U; ++sample) {
    const float time_s = static_cast<float>(sample) * dt_s;
    const float target =
        ramp_acceleration_rad_s2 * time_s;
    const float output = limiter.update(target, dt_s);
    TEST_ASSERT_LESS_OR_EQUAL_FLOAT(
        configuration.max_jerk_rad_s3 * dt_s + 0.0001F,
        std::fabs(limiter.acceleration() - previous_acceleration));
    previous_acceleration = limiter.acceleration();
    if (time_s >= 2.0F) {
      maximum_settled_error = std::max(
          maximum_settled_error, std::fabs(target - output));
    }
  }
  TEST_ASSERT_LESS_THAN_FLOAT(0.02F, maximum_settled_error);

  limiter.reset();
  previous_acceleration = limiter.acceleration();
  maximum_settled_error = 0.0F;
  for (uint32_t sample = 0U; sample <= 2500U; ++sample) {
    const float time_s = static_cast<float>(sample) * dt_s;
    const float target =
        -ramp_acceleration_rad_s2 * time_s;
    const float output = limiter.update(target, dt_s);
    TEST_ASSERT_LESS_OR_EQUAL_FLOAT(
        configuration.max_jerk_rad_s3 * dt_s + 0.0001F,
        std::fabs(limiter.acceleration() - previous_acceleration));
    previous_acceleration = limiter.acceleration();
    if (time_s >= 2.0F) {
      maximum_settled_error = std::max(
          maximum_settled_error, std::fabs(target - output));
    }
  }
  TEST_ASSERT_LESS_THAN_FLOAT(0.02F, maximum_settled_error);
}

void test_motion_limiter_can_disable_jerk_limit() {
  SafetyConfiguration configuration{};
  configuration.max_velocity_rad_s = 100.0F;
  configuration.max_acceleration_rad_s2 = 10.0F;
  configuration.max_jerk_rad_s3 = 20.0F;
  configuration.jerk_limit_enabled = false;
  MotionLimiter limiter;
  limiter.configure(configuration);
  limiter.reset();

  TEST_ASSERT_FLOAT_WITHIN(
      0.0001F, 1.0F, limiter.update(100.0F, 0.1F));
  TEST_ASSERT_FLOAT_WITHIN(
      0.0001F, 10.0F, limiter.acceleration());

  limiter.reset();
  TEST_ASSERT_FLOAT_WITHIN(
      0.0001F, -1.0F, limiter.update(-100.0F, 0.1F));
  TEST_ASSERT_FLOAT_WITHIN(
      0.0001F, -10.0F, limiter.acceleration());
}

void test_position_motion_can_ignore_enabled_jerk_limit() {
  SafetyConfiguration configuration{};
  configuration.max_velocity_rad_s = 100.0F;
  configuration.max_acceleration_rad_s2 = 10.0F;
  configuration.max_jerk_rad_s3 = 20.0F;
  configuration.jerk_limit_enabled = true;
  MotionLimiter limiter;
  limiter.configure(configuration);
  limiter.reset();

  TEST_ASSERT_FLOAT_WITHIN(
      0.0001F, 1.0F, limiter.update(100.0F, 0.1F, false));
  TEST_ASSERT_FLOAT_WITHIN(
      0.0001F, 10.0F, limiter.acceleration());

  limiter.reset();
  TEST_ASSERT_FLOAT_WITHIN(
      0.0001F, -1.0F, limiter.update(-100.0F, 0.1F, false));
  TEST_ASSERT_FLOAT_WITHIN(
      0.0001F, -10.0F, limiter.acceleration());
}

void test_velocity_estimator_uses_output_shaft_cpr() {
  EncoderConfiguration encoder{};
  encoder.counts_per_output_revolution = 100;
  encoder.estimator_min_counts = 1;
  encoder.estimator_max_window_us = 20000;
  VelocityEstimator estimator;
  estimator.configure(encoder, 0.0F);
  estimator.reset(0, 1000);
  const float velocity = estimator.update(10, 101000);
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, 6.2831853F, velocity);
}

void test_velocity_estimator_uses_each_control_interval_count_delta() {
  EncoderConfiguration encoder{};
  encoder.counts_per_output_revolution = 184;
  VelocityEstimator estimator;
  estimator.configure(encoder, 0.0F);
  estimator.reset(0, 1000);
  const float radians_per_count = 6.283185307F / 184.0F;
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, 0.0F, estimator.update(0, 3000));
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, radians_per_count / 0.002F,
                           estimator.update(1, 5000));
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, 0.0F, estimator.update(1, 7000));
}

void test_velocity_estimator_predicts_from_characterized_motor_model() {
  EncoderConfiguration encoder{};
  encoder.counts_per_output_revolution = 184;
  encoder.estimator_min_counts = 4;
  encoder.estimator_max_window_us = 20000;
  encoder.estimator_stale_timeout_us = 100000;
  MotorModelConfiguration model{};
  model.velocity_gain_forward_rad_s_per_duty = 100.0F;
  model.velocity_gain_reverse_rad_s_per_duty = 90.0F;
  model.time_constant_forward_s = 0.10F;
  model.time_constant_reverse_s = 0.12F;
  VelocityEstimator estimator;
  estimator.configure(encoder, 0.025F, model);
  estimator.reset(0, 1000U);
  TEST_ASSERT_TRUE(estimator.usingMotorModel());
  const float predicted = estimator.update(0, 11000U, 0.5F);
  TEST_ASSERT_FLOAT_WITHIN(0.001F, 100.0F * (1.0F - std::exp(-0.1F)) * 0.5F,
                           predicted);
}

void test_velocity_estimator_method_zero_ignores_motor_model() {
  EncoderConfiguration encoder{};
  MotorModelConfiguration model{};
  model.velocity_gain_forward_rad_s_per_duty = 100.0F;
  model.velocity_gain_reverse_rad_s_per_duty = 100.0F;
  model.time_constant_forward_s = 0.10F;
  model.time_constant_reverse_s = 0.10F;
  VelocityEstimator estimator;
  estimator.configure(encoder, 0.025F, model, VelocityEstimatorMethod::LowPass, 120.0F);
  estimator.reset(0, 1000U);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(VelocityEstimatorMethod::LowPass),
                          static_cast<uint8_t>(estimator.method()));
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, 0.0F, estimator.update(0, 11000U, 0.5F));
}

void test_velocity_estimator_predicts_from_encoder_window_acceleration() {
  EncoderConfiguration encoder{};
  encoder.counts_per_output_revolution = 100;
  encoder.estimator_min_counts = 1;
  encoder.estimator_max_window_us = 100000;
  encoder.estimator_stale_timeout_us = 100000;
  VelocityEstimator estimator;
  estimator.configure(encoder, 0.0F, MotorModelConfiguration{},
                      VelocityEstimatorMethod::WindowedAccelerationPrediction, 50.0F, 3U);
  estimator.reset(0, 0U);
  const float radians_per_count = 6.283185307F / 100.0F;
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, radians_per_count / 0.1F,
                           estimator.update(1, 100000U, 1.0F));
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, 1.5F * radians_per_count / 0.1F,
                           estimator.update(3, 200000U, -1.0F));
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, 2.0F * radians_per_count / 0.1F,
                           estimator.update(6, 300000U, 1.0F));
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, 2.1F * radians_per_count / 0.1F,
                           estimator.update(6, 310000U, 1.0F));
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, 0.0F,
                           estimator.update(6, 400000U, 1.0F));
}

void test_kalman_uses_low_pass_for_hand_motion_only_while_disarmed() {
  EncoderConfiguration encoder{};
  encoder.counts_per_output_revolution = 100;
  MotorModelConfiguration model{};
  model.velocity_gain_forward_rad_s_per_duty = 100.0F;
  model.velocity_gain_reverse_rad_s_per_duty = 100.0F;
  model.time_constant_forward_s = 0.10F;
  model.time_constant_reverse_s = 0.10F;
  VelocityEstimator estimator;
  estimator.configure(encoder, 0.0F, model, VelocityEstimatorMethod::Kalman, 120.0F, 5U);
  estimator.reset(0, 0U);
  const float hand_velocity = (6.283185307F / 100.0F) / 0.1F;
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, hand_velocity,
                           estimator.update(1, 100000U, 0.0F, true));
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, hand_velocity,
                           estimator.update(2, 200000U, 0.0F, true));
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, hand_velocity * std::exp(-0.1F),
                           estimator.update(2, 210000U, 0.0F, false));
}

void test_velocity_estimator_preserves_reverse_sign_in_all_modes() {
  EncoderConfiguration encoder{};
  encoder.counts_per_output_revolution = 100;
  encoder.estimator_min_counts = 1;
  encoder.estimator_max_window_us = 100000;
  encoder.estimator_stale_timeout_us = 100000;
  const float reverse_encoder_velocity =
      -(6.283185307F / 100.0F) / 0.1F;

  VelocityEstimator low_pass;
  low_pass.configure(
      encoder, 0.0F, MotorModelConfiguration{},
      VelocityEstimatorMethod::LowPass, 120.0F);
  low_pass.reset(0, 1000U);
  TEST_ASSERT_FLOAT_WITHIN(
      0.0001F, reverse_encoder_velocity,
      low_pass.update(-1, 101000U, -0.5F));

  VelocityEstimator windowed;
  windowed.configure(
      encoder, 0.0F, MotorModelConfiguration{},
      VelocityEstimatorMethod::WindowedAccelerationPrediction,
      120.0F, 3U);
  windowed.reset(0, 1000U);
  TEST_ASSERT_FLOAT_WITHIN(
      0.0001F, reverse_encoder_velocity,
      windowed.update(-1, 101000U, -0.5F));

  MotorModelConfiguration model{};
  model.velocity_gain_forward_rad_s_per_duty = 100.0F;
  model.velocity_gain_reverse_rad_s_per_duty = 80.0F;
  model.time_constant_forward_s = 0.10F;
  model.time_constant_reverse_s = 0.20F;
  VelocityEstimator kalman;
  kalman.configure(
      encoder, 0.025F, model, VelocityEstimatorMethod::Kalman,
      120.0F, 5U);
  kalman.reset(0, 1000U);
  const float expected_prediction =
      -80.0F * (1.0F - std::exp(-0.05F)) * 0.5F;
  TEST_ASSERT_FLOAT_WITHIN(
      0.001F, expected_prediction,
      kalman.update(0, 11000U, -0.5F));
  TEST_ASSERT_TRUE(kalman.update(-1, 101000U, -0.5F) < 0.0F);

  encoder.direction = -1;
  VelocityEstimator inverted_encoder;
  inverted_encoder.configure(
      encoder, 0.0F, MotorModelConfiguration{},
      VelocityEstimatorMethod::LowPass, 120.0F);
  inverted_encoder.reset(0, 1000U);
  TEST_ASSERT_FLOAT_WITHIN(
      0.0001F, reverse_encoder_velocity,
      inverted_encoder.update(1, 101000U, -0.5F));
}

void test_reverse_estimated_velocity_drives_negative_pid_output() {
  EncoderConfiguration encoder{};
  encoder.counts_per_output_revolution = 100;
  VelocityEstimator estimator;
  estimator.configure(encoder, 0.0F);
  estimator.reset(0, 1000U);
  const float measured =
      estimator.update(-1, 101000U, -0.5F);
  TEST_ASSERT_TRUE(measured < 0.0F);

  ControlConfiguration control{};
  IncrementalVelocityController controller;
  controller.configure(control);
  float output = 0.0F;
  for (uint8_t iteration = 0U; iteration < 20U; ++iteration) {
    output = controller.update(-2.0F, measured, 0.002F);
  }
  TEST_ASSERT_TRUE(output < 0.0F);
}

void test_velocity_control_demand_accepts_both_directions() {
  TEST_ASSERT_TRUE(velocityControlDemandActive(10.0F));
  TEST_ASSERT_TRUE(velocityControlDemandActive(-10.0F));
  TEST_ASSERT_FALSE(velocityControlDemandActive(0.0F));
}

void test_zero_index_debounce_accepts_first_and_filters_close_events() {
  TEST_ASSERT_TRUE(shouldAcceptZeroIndexEvent(
      100000U, 0U, 5000U, 100, 0, 0U));
  TEST_ASSERT_FALSE(shouldAcceptZeroIndexEvent(
      102000U, 100000U, 5000U, 100, 100, 0U));
  TEST_ASSERT_TRUE(shouldAcceptZeroIndexEvent(
      105000U, 100000U, 5000U, 100, 100, 0U));
}

void test_zero_index_rejects_late_bounce_without_rotor_travel() {
  TEST_ASSERT_EQUAL_UINT32(92U, zeroIndexMinimumSeparationCounts(184U, 0.50F));
  TEST_ASSERT_FALSE(shouldAcceptZeroIndexEvent(
      106000U, 100000U, 5000U, 101, 100, 92U));
  TEST_ASSERT_TRUE(shouldAcceptZeroIndexEvent(
      106000U, 100000U, 5000U, 192, 100, 92U));
  TEST_ASSERT_TRUE(shouldAcceptZeroIndexEvent(
      106000U, 100000U, 5000U, 8, 100, 92U));
}

void test_zero_index_reversal_still_requires_time_debounce() {
  TEST_ASSERT_FALSE(
      zeroIndexMinimumIntervalElapsed(102000U, 100000U, 5000U));
  TEST_ASSERT_TRUE(
      zeroIndexMinimumIntervalElapsed(105000U, 100000U, 5000U));
}

void test_rotor_phase_tracker_uses_encoder_after_first_zero_reference() {
  RotorPhaseTracker tracker;
  tracker.configure(184U, 1, 0.10F);
  TEST_ASSERT_FLOAT_WITHIN(0.001F, 0.0F, tracker.update(100, 100, 1U));
  TEST_ASSERT_FLOAT_WITHIN(0.001F, 90.0F, tracker.update(146, 100, 1U));

  tracker.configure(184U, -1, 0.10F);
  TEST_ASSERT_FLOAT_WITHIN(0.001F, 0.0F, tracker.update(100, 100, 1U));
  TEST_ASSERT_FLOAT_WITHIN(0.001F, 270.0F, tracker.update(146, 100, 1U));
}

void test_rotor_phase_tracker_applies_fractional_correction_once_per_zero() {
  RotorPhaseTracker tracker;
  tracker.configure(184U, 1, 0.10F);
  tracker.update(0, 0, 1U);

  // The next index arrives five counts late. A 0.10 gain removes only half a count,
  // keeping encoder motion primary instead of snapping the position to zero.
  const float corrected_at_index = tracker.update(189, 189, 2U);
  TEST_ASSERT_FLOAT_WITHIN(0.001F, 8.804348F, corrected_at_index);
  TEST_ASSERT_FLOAT_WITHIN(0.001F, corrected_at_index, tracker.update(189, 189, 2U));
  TEST_ASSERT_FLOAT_WITHIN(0.001F, 98.804348F, tracker.update(235, 189, 2U));
}

void test_rotor_phase_tracker_applies_user_zero_without_losing_reference() {
  RotorPhaseTracker tracker;
  tracker.configure(184U, 1, 0.10F, 0);
  tracker.update(0, 0, 1U);
  TEST_ASSERT_FLOAT_WITHIN(0.001F, 90.0F, tracker.update(46, 0, 1U));

  tracker.configure(184U, 1, 0.10F, 46);
  TEST_ASSERT_TRUE(tracker.referenced());
  TEST_ASSERT_EQUAL_UINT32(46U, tracker.zeroPositionOffsetTicks());
  TEST_ASSERT_FLOAT_WITHIN(0.001F, 0.0F, tracker.update(46, 0, 1U));
  TEST_ASSERT_FLOAT_WITHIN(0.001F, 270.0F, tracker.update(0, 0, 1U));
}

void test_rotor_user_zero_does_not_drift_during_later_index_correction() {
  RotorPhaseTracker tracker;
  tracker.configure(184U, 1, 0.10F, 0);
  tracker.update(0, 0, 1U);

  // The index is five counts late. Fractional correction leaves residual phase
  // error at the moment the user captures a physical point 46 counts later.
  tracker.update(189, 189, 2U);
  tracker.update(235, 189, 2U);
  const uint32_t candidate_offset_ticks =
      tracker.positionTicksFromZeroIndex(235, 189);
  TEST_ASSERT_EQUAL_UINT32(46U, candidate_offset_ticks);
  tracker.configure(184U, 1, 0.10F, candidate_offset_ticks);
  tracker.synchronizeToZeroIndex(189, 2U);
  TEST_ASSERT_FLOAT_WITHIN(0.001F, 0.0F, tracker.update(235, 189, 2U));

  // A later index correction must not move that same physical 46-count point.
  tracker.update(373, 373, 3U);
  TEST_ASSERT_FLOAT_WITHIN(0.001F, 0.0F, tracker.update(419, 373, 3U));
}

void test_rotor_zero_index_filters_wrapped_tick_toward_saved_reference() {
  RotorPhaseTracker tracker;
  tracker.configure(184U, 1, 0.10F, 46);
  TEST_ASSERT_FLOAT_WITHIN(0.001F, 270.0F, tracker.update(0, 0, 1U));

  // At the next index, the corrected phase is -41 ticks while its target is
  // -46 ticks. A 0.10 correction moves it to -41.5 ticks:
  // 0.90 * -41 + 0.10 * -46.
  TEST_ASSERT_FLOAT_WITHIN(0.001F, 278.804348F,
                           tracker.update(189, 189, 2U));
}

void test_rotor_zero_capture_uses_one_index_relative_encoder_count() {
  RotorPhaseTracker tracker;
  tracker.configure(184U, 1, 0.10F);
  TEST_ASSERT_EQUAL_UINT32(46U, tracker.positionTicksFromZeroIndex(235, 189));

  tracker.configure(184U, -1, 0.10F);
  TEST_ASSERT_EQUAL_UINT32(138U, tracker.positionTicksFromZeroIndex(235, 189));
}

void test_zero_index_reference_side_selects_complementary_edges() {
  TEST_ASSERT_TRUE(zeroIndexEdgeMatches(
      1, ZeroIndexEdge::Rising, ZeroIndexReferenceSide::ClockwiseRising));
  TEST_ASSERT_TRUE(zeroIndexEdgeMatches(
      -1, ZeroIndexEdge::Falling, ZeroIndexReferenceSide::ClockwiseRising));
  TEST_ASSERT_FALSE(zeroIndexEdgeMatches(
      1, ZeroIndexEdge::Falling, ZeroIndexReferenceSide::ClockwiseRising));

  TEST_ASSERT_TRUE(zeroIndexEdgeMatches(
      1, ZeroIndexEdge::Falling, ZeroIndexReferenceSide::ClockwiseFalling));
  TEST_ASSERT_TRUE(zeroIndexEdgeMatches(
      -1, ZeroIndexEdge::Rising, ZeroIndexReferenceSide::ClockwiseFalling));
}

void test_zero_index_hysteresis_calibration_learns_both_reference_sides() {
  ZeroIndexHysteresisCalibration calibration;
  calibration.configure(184U, 1, 2U);

  for (int64_t revolution = 0; revolution < 5; ++revolution) {
    const int64_t base = revolution * 184;
    TEST_ASSERT_TRUE(calibration.addPass(1, base + 110, base + 100));
  }
  for (int64_t revolution = 0; revolution < 5; ++revolution) {
    const int64_t base = 5 * 184 - revolution * 184;
    TEST_ASSERT_TRUE(calibration.addPass(-1, base + 98, base + 112));
  }

  ZeroIndexHysteresisResult result{};
  TEST_ASSERT_TRUE(calibration.result(result));
  TEST_ASSERT_EQUAL_INT32(-2, result.clockwise_rising_correction_ticks);
  TEST_ASSERT_EQUAL_INT32(2, result.clockwise_falling_correction_ticks);
  TEST_ASSERT_EQUAL_UINT16(0U, result.maximum_residual_ticks);

  TEST_ASSERT_EQUAL_INT64(
      1030,
      applyZeroIndexDirectionCorrection(
          1032, -1, 1, result.clockwise_rising_correction_ticks));
  TEST_ASSERT_EQUAL_INT64(
      1020,
      applyZeroIndexDirectionCorrection(
          1018, -1, 1, result.clockwise_falling_correction_ticks));
}

void test_zero_index_hysteresis_calibration_rejects_lost_counts() {
  ZeroIndexHysteresisCalibration calibration;
  calibration.configure(184U, 1, 2U);
  TEST_ASSERT_TRUE(calibration.addPass(1, 110, 100));
  TEST_ASSERT_FALSE(calibration.addPass(1, 290, 280));
  TEST_ASSERT_EQUAL_UINT8(1U, calibration.clockwiseCount());
}

void test_sine_profile_stays_one_direction() {
  VelocityProfileConfiguration configuration{};
  configuration.kind = ProfileKind::Sine;
  configuration.sine_mean_rad_s = 20.0F;
  configuration.sine_amplitude_rad_s = 5.0F;
  configuration.sine_frequency_hz = 1.0F;
  configuration.duration_ms = 2000;
  VelocityProfile profile;
  profile.select(&configuration, 1000000);
  TEST_ASSERT_FLOAT_WITHIN(0.001F, 25.0F, profile.target(1250000));
  TEST_ASSERT_TRUE(profile.target(1750000) >= 0.0F);
  TEST_ASSERT_FLOAT_WITHIN(0.001F, 0.0F, profile.target(3000000));
}

void test_waypoint_profile_interpolates_and_stops_at_duration() {
  VelocityProfileConfiguration configuration{};
  configuration.kind = ProfileKind::Waypoints;
  configuration.duration_ms = 2000;
  configuration.point_count = 3;
  configuration.points[0] = {0, 0.0F};
  configuration.points[1] = {1000, 40.0F};
  configuration.points[2] = {2000, 0.0F};
  VelocityProfile profile;
  profile.select(&configuration, 1000000);
  TEST_ASSERT_FALSE(profile.finished(1000000));
  TEST_ASSERT_FLOAT_WITHIN(0.001F, 20.0F, profile.target(1500000));
  TEST_ASSERT_FLOAT_WITHIN(0.001F, 20.0F, profile.target(2500000));
  TEST_ASSERT_FALSE(profile.finished(2999999));
  TEST_ASSERT_TRUE(profile.finished(3000000));
  TEST_ASSERT_FLOAT_WITHIN(0.001F, 0.0F, profile.target(3000000));
}

void test_profile_collection_creates_without_replacing_existing_id() {
  std::array<VelocityProfileConfiguration, kMaximumProfiles> profiles{};
  uint8_t count = 1U;
  profiles[0].profile_id = 0U;
  VelocityProfileConfiguration created{};
  created.profile_id = 1U;
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(ProfileUpdateResult::Created),
      static_cast<uint8_t>(applyProfileUpdate(profiles, count, created, true)));
  TEST_ASSERT_EQUAL_UINT8(2U, count);
  TEST_ASSERT_EQUAL_UINT16(0U, profiles[0].profile_id);
  TEST_ASSERT_EQUAL_UINT16(1U, profiles[1].profile_id);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(ProfileUpdateResult::AlreadyExists),
      static_cast<uint8_t>(applyProfileUpdate(profiles, count, created, true)));
  TEST_ASSERT_EQUAL_UINT8(2U, count);

  created.name[0] = 'x';
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(ProfileUpdateResult::Replaced),
      static_cast<uint8_t>(applyProfileUpdate(profiles, count, created, false)));
  TEST_ASSERT_EQUAL_CHAR('x', profiles[1].name[0]);
  TEST_ASSERT_EQUAL_UINT8(2U, count);
}

void test_vin_divider_nominal_gain() {
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, 7.8F,
                           VoltageSenseConfiguration::kNominalDividerGain);
  TEST_ASSERT_FLOAT_WITHIN(0.001F, 2.051282F,
                           16.0F / VoltageSenseConfiguration::kNominalDividerGain);
}

void test_driver_diagnostic_is_disabled_by_default() {
  const MachineSettings settings{};
  TEST_ASSERT_FALSE(settings.safety.driver_diagnostic_enabled);
  TEST_ASSERT_FALSE(settings.safety.current_sense_enabled);
  TEST_ASSERT_TRUE(settings.safety.jerk_limit_enabled);
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, 20.0F,
                           settings.motor.current_filter_cutoff_hz);
  TEST_ASSERT_EQUAL_UINT32(28U, settings.schema_version);
  TEST_ASSERT_TRUE(settings.status_led.enabled);
  TEST_ASSERT_EQUAL_UINT8(2U, settings.status_led.data_pin);
  TEST_ASSERT_EQUAL_UINT8(4U, settings.status_led.pixel_count);
  TEST_ASSERT_FALSE(settings.load_setting.broken_bearing);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(VelocityEstimatorMethod::LowPass),
                          static_cast<uint8_t>(settings.velocity_estimator_method));
  TEST_ASSERT_EQUAL_UINT32(5U, settings.velocity_acceleration_window_samples);
  TEST_ASSERT_EQUAL_UINT32(12U, kMaximumLoads);
  TEST_ASSERT_EQUAL_UINT32(EncoderConfiguration::kDefaultZeroIndexMinimumIntervalUs,
                           settings.encoder.zero_index_min_interval_us);
  TEST_ASSERT_FLOAT_WITHIN(0.0001F,
                           EncoderConfiguration::kDefaultZeroIndexCorrectionGain,
                           settings.encoder.zero_index_correction_gain);
  TEST_ASSERT_FLOAT_WITHIN(
      0.0001F,
      EncoderConfiguration::kDefaultZeroIndexMinimumSeparationRevolutions,
      settings.encoder.zero_index_minimum_separation_revolutions);
}

void test_encoder_watchdog_grants_fresh_motion_demand_timeout() {
  EncoderActivityWatchdog watchdog;
  constexpr uint64_t demand_start_us = 5000000ULL;
  constexpr uint64_t stale_edge_us = 1000000ULL;
  TEST_ASSERT_FALSE(watchdog.update(demand_start_us, 2.0F, stale_edge_us, true, 250U, 1.0F));
  TEST_ASSERT_FALSE(watchdog.update(demand_start_us + 250000ULL, 2.0F, stale_edge_us, true, 250U, 1.0F));
  TEST_ASSERT_TRUE(watchdog.update(demand_start_us + 250001ULL, 2.0F, stale_edge_us, true, 250U, 1.0F));
  TEST_ASSERT_FALSE(watchdog.update(demand_start_us + 260000ULL, 0.5F, stale_edge_us, true, 250U, 1.0F));
  TEST_ASSERT_FALSE(watchdog.update(demand_start_us + 300000ULL, 2.0F, stale_edge_us, true, 250U, 1.0F));
}

void test_encoder_watchdog_refreshes_on_valid_edge() {
  EncoderActivityWatchdog watchdog;
  TEST_ASSERT_FALSE(watchdog.update(1000000ULL, 2.0F, 0U, true, 250U, 1.0F));
  TEST_ASSERT_FALSE(watchdog.update(1200000ULL, 2.0F, 1190000ULL, true, 250U, 1.0F));
  TEST_ASSERT_FALSE(watchdog.update(1430000ULL, 2.0F, 1190000ULL, true, 250U, 1.0F));
  TEST_ASSERT_TRUE(watchdog.update(1440001ULL, 2.0F, 1190000ULL, true, 250U, 1.0F));
}

void test_encoder_watchdog_can_guard_fixed_duty_calibration_motion() {
  EncoderActivityWatchdog watchdog;
  TEST_ASSERT_FALSE(
      watchdog.update(100000U, 0.0F, 90000U, true, 250U, 1.0F, true));
  TEST_ASSERT_TRUE(
      watchdog.update(351000U, 0.0F, 90000U, true, 250U, 1.0F, true));
}

void test_zero_index_calibration_watchdog_allows_reversal_startup() {
  EncoderActivityWatchdog watchdog;
  const uint32_t calibration_timeout_ms =
      zeroIndexCalibrationEncoderTimeoutMs(250U, 1000U);
  TEST_ASSERT_EQUAL_UINT32(1000U, calibration_timeout_ms);
  TEST_ASSERT_FALSE(watchdog.update(
      100000U, 0.0F, 90000U, true, calibration_timeout_ms, 1.0F, true));
  TEST_ASSERT_FALSE(watchdog.update(
      351000U, 0.0F, 90000U, true, calibration_timeout_ms, 1.0F, true));
  TEST_ASSERT_TRUE(watchdog.update(
      1100001U, 0.0F, 90000U, true, calibration_timeout_ms, 1.0F, true));
}

void test_zero_index_calibration_uses_signed_fifteen_rpm_target() {
  const MachineSettings settings{};
  TEST_ASSERT_FLOAT_WITHIN(
      0.0001F, 15.0F,
      settings.encoder.zero_index_calibration_speed_rpm);
  TEST_ASSERT_FLOAT_WITHIN(
      0.0001F, 1.5707963F,
      zeroIndexCalibrationTargetVelocityRadS(
          1, settings.encoder.zero_index_calibration_speed_rpm));
  TEST_ASSERT_FLOAT_WITHIN(
      0.0001F, -1.5707963F,
      zeroIndexCalibrationTargetVelocityRadS(
          -1, settings.encoder.zero_index_calibration_speed_rpm));
}

void test_vin_calibration_ceiling_is_above_valid_sixteen_volt_input() {
  const float sense_at_safety_max = 16.0F / VoltageSenseConfiguration::kNominalDividerGain;
  TEST_ASSERT_TRUE(sense_at_safety_max < VoltageSenseConfiguration::kMaximumCalibrationSenseV);
  TEST_ASSERT_TRUE(3.139F > VoltageSenseConfiguration::kMaximumCalibrationSenseV);
}

void test_characterization_peak_is_independent_of_encoder_polarity() {
  TEST_ASSERT_FLOAT_WITHIN(
      0.0001F, 42.0F,
      characterization::updatePeakVelocityMagnitude(12.0F, -42.0F));
  TEST_ASSERT_FLOAT_WITHIN(
      0.0001F, 42.0F,
      characterization::updatePeakVelocityMagnitude(42.0F, 10.0F));
}

void test_characterization_clamps_vmax_and_dependent_settings() {
  MachineSettings current{};
  current.safety.max_velocity_rad_s = 150.0F;
  current.safety.encoder_timeout_velocity_rad_s = 120.0F;
  current.position_control.max_velocity_rad_s = 130.0F;
  current.position_control.settle_velocity_rad_s = 120.0F;
  current.position_control.minimum_velocity_forward_rad_s = 115.0F;
  current.position_control.minimum_velocity_reverse_rad_s = 118.0F;
  current.profile_count = 1U;
  current.profiles[0].target_velocity_rad_s = 140.0F;
  current.profiles[0].sine_mean_rad_s = 100.0F;
  current.profiles[0].sine_amplitude_rad_s = 30.0F;
  current.profiles[0].point_count = 2U;
  current.profiles[0].points[0].velocity_rad_s = 0.0F;
  current.profiles[0].points[1].velocity_rad_s = 140.0F;

  MotorCharacteristics measured = current.motor;
  measured.max_velocity_forward_rad_s = 125.0F;
  measured.max_velocity_reverse_rad_s = 110.0F;
  MachineSettings candidate{};
  TEST_ASSERT_TRUE(characterization::prepareCharacterizedSettings(
      current, measured, candidate));
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, 110.0F,
                           candidate.safety.max_velocity_rad_s);
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, 110.0F,
                           candidate.safety.encoder_timeout_velocity_rad_s);
  TEST_ASSERT_FLOAT_WITHIN(
      0.0001F, 110.0F, candidate.position_control.max_velocity_rad_s);
  TEST_ASSERT_FLOAT_WITHIN(
      0.0001F, 110.0F,
      candidate.position_control.settle_velocity_rad_s);
  TEST_ASSERT_FLOAT_WITHIN(
      0.0001F, 110.0F,
      candidate.position_control.minimum_velocity_forward_rad_s);
  TEST_ASSERT_FLOAT_WITHIN(
      0.0001F, 110.0F,
      candidate.position_control.minimum_velocity_reverse_rad_s);
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, 110.0F,
                           candidate.profiles[0].target_velocity_rad_s);
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, 110.0F,
                           candidate.profiles[0].sine_mean_rad_s +
                               std::fabs(candidate.profiles[0].sine_amplitude_rad_s));
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, 110.0F,
                           candidate.profiles[0].points[1].velocity_rad_s);
}

void test_characterization_never_raises_existing_vmax() {
  MachineSettings current{};
  current.safety.max_velocity_rad_s = 80.0F;
  MotorCharacteristics measured = current.motor;
  measured.max_velocity_forward_rad_s = 125.0F;
  measured.max_velocity_reverse_rad_s = 110.0F;
  MachineSettings candidate{};
  TEST_ASSERT_TRUE(characterization::prepareCharacterizedSettings(
      current, measured, candidate));
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, 80.0F,
                           candidate.safety.max_velocity_rad_s);
}

void test_characterization_dynamics_only_lowers_selected_limits() {
  MachineSettings candidate{};
  candidate.safety.max_acceleration_rad_s2 = 120.0F;
  candidate.safety.max_jerk_rad_s3 = 800.0F;
  candidate.profiles[0].kind = ProfileKind::Waypoints;
  candidate.profiles[0].duration_ms = 2000U;
  candidate.profiles[0].point_count = 3U;
  candidate.profiles[0].points[0] = {0U, 0.0F};
  candidate.profiles[0].points[1] = {1000U, 100.0F};
  candidate.profiles[0].points[2] = {2000U, 0.0F};
  CharacterizationDynamicsResult measured{};
  measured.acceleration_forward_rad_s2 = 100.0F;
  measured.acceleration_reverse_rad_s2 = 80.0F;
  measured.jerk_forward_rad_s3 = 1500.0F;
  measured.jerk_reverse_rad_s3 = 1200.0F;
  TEST_ASSERT_TRUE(characterization::applyRecommendedDynamics(
      measured, 0.70F, true, true, candidate));
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, 56.0F,
                           candidate.safety.max_acceleration_rad_s2);
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, 800.0F,
                           candidate.safety.max_jerk_rad_s3);
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, 56.0F,
                           candidate.profiles[0].points[1].velocity_rad_s);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_crc_standard_vector);
  RUN_TEST(test_status_led_maps_machine_states_to_distinct_colors);
  RUN_TEST(test_status_led_boot_pattern_identifies_pixel_order);
  RUN_TEST(test_status_led_command_overlay_blinks_twice_then_restores_state);
  RUN_TEST(test_status_led_command_overlay_is_suppressed_while_armed);
  RUN_TEST(test_status_led_running_trail_reverses_with_motor_direction);
  RUN_TEST(test_status_led_fault_overrides_command_blink);
  RUN_TEST(test_characterization_dynamics_estimates_constant_acceleration);
  RUN_TEST(test_characterization_dynamics_uses_robust_quantile);
  RUN_TEST(test_breakaway_trials_retain_highest_duty);
  RUN_TEST(test_full_duty_trials_retain_highest_measurements);
  RUN_TEST(test_breakaway_requires_one_correctly_directed_output_revolution);
  RUN_TEST(test_breakaway_travel_handles_reverse_and_inverted_encoder);
  RUN_TEST(test_motor_identifier_recovers_first_order_step_model);
  RUN_TEST(test_create_profile_frame_crc_vector);
  RUN_TEST(test_load_configuration_frame_crc_vector);
  RUN_TEST(test_characterization_result_frame_crc_vector);
  RUN_TEST(test_settings_schema_24_frame_crc_vector);
  RUN_TEST(test_settings_schema_27_frame_crc_vector);
  RUN_TEST(test_settings_schema_28_frame_crc_vector);
  RUN_TEST(test_position_target_frame_crc_vector);
  RUN_TEST(test_position_controller_uses_shortest_wrapped_error);
  RUN_TEST(test_position_controller_bounds_velocity_demand);
  RUN_TEST(test_position_controller_avoids_velocity_deadband_until_intentional_zero);
  RUN_TEST(test_position_velocity_output_uses_directional_motor_deadband);
  RUN_TEST(test_bearing_configuration_frame_crc_vector);
  RUN_TEST(test_rotor_zero_calibration_frame_crc_vector);
  RUN_TEST(test_zero_index_hysteresis_calibration_frame_crc_vector);
  RUN_TEST(test_velocity_step_sequence_holds_each_level_and_finishes_at_zero);
  RUN_TEST(test_velocity_sequence_frame_crc_vector);
  RUN_TEST(test_telemetry_rate_respects_uart_bandwidth);
  RUN_TEST(test_incremental_controller_scales_integral_by_time);
  RUN_TEST(test_incremental_controller_does_not_wind_up);
  RUN_TEST(test_incremental_controller_reports_each_delta_term);
  RUN_TEST(test_low_pass_filter_uses_cutoff_and_elapsed_time);
  RUN_TEST(test_two_point_current_calibration_calculates_gain_and_offset);
  RUN_TEST(test_two_point_current_calibration_rejects_insufficient_span);
  RUN_TEST(test_current_calibration_capture_averages_incrementally);
  RUN_TEST(test_motion_limiter_honors_acceleration_and_jerk);
  RUN_TEST(test_motion_limiter_tracks_feasible_ramp_without_ripple);
  RUN_TEST(test_motion_limiter_can_disable_jerk_limit);
  RUN_TEST(test_position_motion_can_ignore_enabled_jerk_limit);
  RUN_TEST(test_velocity_estimator_uses_output_shaft_cpr);
  RUN_TEST(test_velocity_estimator_uses_each_control_interval_count_delta);
  RUN_TEST(test_velocity_estimator_predicts_from_characterized_motor_model);
  RUN_TEST(test_velocity_estimator_method_zero_ignores_motor_model);
  RUN_TEST(test_velocity_estimator_predicts_from_encoder_window_acceleration);
  RUN_TEST(test_kalman_uses_low_pass_for_hand_motion_only_while_disarmed);
  RUN_TEST(test_velocity_estimator_preserves_reverse_sign_in_all_modes);
  RUN_TEST(test_reverse_estimated_velocity_drives_negative_pid_output);
  RUN_TEST(test_velocity_control_demand_accepts_both_directions);
  RUN_TEST(test_zero_index_debounce_accepts_first_and_filters_close_events);
  RUN_TEST(test_zero_index_rejects_late_bounce_without_rotor_travel);
  RUN_TEST(test_zero_index_reversal_still_requires_time_debounce);
  RUN_TEST(test_rotor_phase_tracker_uses_encoder_after_first_zero_reference);
  RUN_TEST(test_rotor_phase_tracker_applies_fractional_correction_once_per_zero);
  RUN_TEST(test_rotor_phase_tracker_applies_user_zero_without_losing_reference);
  RUN_TEST(test_rotor_user_zero_does_not_drift_during_later_index_correction);
  RUN_TEST(test_rotor_zero_index_filters_wrapped_tick_toward_saved_reference);
  RUN_TEST(test_rotor_zero_capture_uses_one_index_relative_encoder_count);
  RUN_TEST(test_zero_index_reference_side_selects_complementary_edges);
  RUN_TEST(test_zero_index_hysteresis_calibration_learns_both_reference_sides);
  RUN_TEST(test_zero_index_hysteresis_calibration_rejects_lost_counts);
  RUN_TEST(test_sine_profile_stays_one_direction);
  RUN_TEST(test_waypoint_profile_interpolates_and_stops_at_duration);
  RUN_TEST(test_profile_collection_creates_without_replacing_existing_id);
  RUN_TEST(test_vin_divider_nominal_gain);
  RUN_TEST(test_driver_diagnostic_is_disabled_by_default);
  RUN_TEST(test_encoder_watchdog_grants_fresh_motion_demand_timeout);
  RUN_TEST(test_encoder_watchdog_refreshes_on_valid_edge);
  RUN_TEST(test_encoder_watchdog_can_guard_fixed_duty_calibration_motion);
  RUN_TEST(test_zero_index_calibration_watchdog_allows_reversal_startup);
  RUN_TEST(test_zero_index_calibration_uses_signed_fifteen_rpm_target);
  RUN_TEST(test_vin_calibration_ceiling_is_above_valid_sixteen_volt_input);
  RUN_TEST(test_characterization_peak_is_independent_of_encoder_polarity);
  RUN_TEST(test_characterization_clamps_vmax_and_dependent_settings);
  RUN_TEST(test_characterization_never_raises_existing_vmax);
  RUN_TEST(test_characterization_dynamics_only_lowers_selected_limits);
  return UNITY_END();
}
