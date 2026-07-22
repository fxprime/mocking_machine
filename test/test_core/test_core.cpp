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
#include "control/RotorPosition.hpp"
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

void test_settings_schema_16_frame_crc_vector() {
  std::array<uint8_t, 8U + 173U> protected_bytes{};
  protected_bytes[0] = 1U;
  protected_bytes[2] = 0x01U;  // SETTINGS 0x0101.
  protected_bytes[3] = 0x01U;
  protected_bytes[4] = 0x34U;
  protected_bytes[5] = 0x12U;
  protected_bytes[6] = 173U;
  TEST_ASSERT_EQUAL_HEX16(
      0xBA2DU, protocol::crc16CcittFalse(protected_bytes.data(), protected_bytes.size()));
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
  TEST_ASSERT_EQUAL_UINT32(16U, MachineSettings::kSchemaVersion);
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

void test_zero_index_debounce_accepts_first_and_filters_close_rises() {
  TEST_ASSERT_TRUE(shouldAcceptZeroIndexRise(
      100000U, 0U, 5000U, 100, 0, 0U));
  TEST_ASSERT_FALSE(shouldAcceptZeroIndexRise(
      102000U, 100000U, 5000U, 100, 100, 0U));
  TEST_ASSERT_TRUE(shouldAcceptZeroIndexRise(
      105000U, 100000U, 5000U, 100, 100, 0U));
}

void test_zero_index_rejects_late_bounce_without_rotor_travel() {
  TEST_ASSERT_EQUAL_UINT32(92U, zeroIndexMinimumSeparationCounts(184U, 0.50F));
  TEST_ASSERT_FALSE(shouldAcceptZeroIndexRise(
      106000U, 100000U, 5000U, 101, 100, 92U));
  TEST_ASSERT_TRUE(shouldAcceptZeroIndexRise(
      106000U, 100000U, 5000U, 192, 100, 92U));
  TEST_ASSERT_TRUE(shouldAcceptZeroIndexRise(
      106000U, 100000U, 5000U, 8, 100, 92U));
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
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, 20.0F,
                           settings.motor.current_filter_cutoff_hz);
  TEST_ASSERT_EQUAL_UINT32(16U, settings.schema_version);
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
  RUN_TEST(test_characterization_dynamics_estimates_constant_acceleration);
  RUN_TEST(test_characterization_dynamics_uses_robust_quantile);
  RUN_TEST(test_motor_identifier_recovers_first_order_step_model);
  RUN_TEST(test_create_profile_frame_crc_vector);
  RUN_TEST(test_load_configuration_frame_crc_vector);
  RUN_TEST(test_characterization_result_frame_crc_vector);
  RUN_TEST(test_settings_schema_16_frame_crc_vector);
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
  RUN_TEST(test_velocity_estimator_uses_output_shaft_cpr);
  RUN_TEST(test_velocity_estimator_uses_each_control_interval_count_delta);
  RUN_TEST(test_velocity_estimator_predicts_from_characterized_motor_model);
  RUN_TEST(test_velocity_estimator_method_zero_ignores_motor_model);
  RUN_TEST(test_velocity_estimator_predicts_from_encoder_window_acceleration);
  RUN_TEST(test_kalman_uses_low_pass_for_hand_motion_only_while_disarmed);
  RUN_TEST(test_zero_index_debounce_accepts_first_and_filters_close_rises);
  RUN_TEST(test_zero_index_rejects_late_bounce_without_rotor_travel);
  RUN_TEST(test_rotor_phase_tracker_uses_encoder_after_first_zero_reference);
  RUN_TEST(test_rotor_phase_tracker_applies_fractional_correction_once_per_zero);
  RUN_TEST(test_sine_profile_stays_one_direction);
  RUN_TEST(test_waypoint_profile_interpolates_and_stops_at_duration);
  RUN_TEST(test_profile_collection_creates_without_replacing_existing_id);
  RUN_TEST(test_vin_divider_nominal_gain);
  RUN_TEST(test_driver_diagnostic_is_disabled_by_default);
  RUN_TEST(test_encoder_watchdog_grants_fresh_motion_demand_timeout);
  RUN_TEST(test_encoder_watchdog_refreshes_on_valid_edge);
  RUN_TEST(test_vin_calibration_ceiling_is_above_valid_sixteen_volt_input);
  RUN_TEST(test_characterization_peak_is_independent_of_encoder_polarity);
  RUN_TEST(test_characterization_clamps_vmax_and_dependent_settings);
  RUN_TEST(test_characterization_never_raises_existing_vmax);
  RUN_TEST(test_characterization_dynamics_only_lowers_selected_limits);
  return UNITY_END();
}
