#include <unity.h>

#include <array>
#include <cstdint>

#include "control/IncrementalVelocityController.hpp"
#include "control/CharacterizationMetrics.hpp"
#include "control/CurrentCalibration.hpp"
#include "control/EncoderActivityWatchdog.hpp"
#include "control/LowPassFilter.hpp"
#include "control/MotorFeedforward.hpp"
#include "control/MotionLimiter.hpp"
#include "control/VelocityEstimator.hpp"
#include "drivers/EncoderPeriodAverager.hpp"
#include "profile/VelocityProfile.hpp"
#include "protocol/Crc16.hpp"

using namespace mm;

void test_crc_standard_vector() {
  constexpr std::array<uint8_t, 9> data{'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  TEST_ASSERT_EQUAL_HEX16(0x29B1, protocol::crc16CcittFalse(data.data(), data.size()));
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

void test_velocity_controller_bounds_feedback_around_feedforward() {
  ControlConfiguration configuration{};
  configuration.kp = 0.2F;
  configuration.ki = 0.0F;
  configuration.kd = 0.0F;
  configuration.error_deadband_rad_s = 0.0F;
  configuration.output_min = 0.0F;
  configuration.output_max = 0.9F;
  IncrementalVelocityController controller;
  controller.configure(configuration);
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, 0.3F,
                           controller.update(10.0F, 10.0F, 0.002F, 0.3F, 0.1F));
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, 0.2F,
                           controller.update(10.0F, 20.0F, 0.002F, 0.3F, 0.1F));
  TEST_ASSERT_TRUE(controller.output() >= 0.0F);
}

void test_motor_feedforward_maps_velocity_to_physical_duty() {
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, 0.0F,
                           motorFeedforwardDuty(0.0F, 0.18F, 166.5F, 0.9F));
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, 0.18F,
                           motorFeedforwardDuty(0.001F, 0.18F, 166.5F, 0.9F));
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, 0.9F,
                           motorFeedforwardDuty(166.5F, 0.18F, 166.5F, 0.9F));
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

void test_velocity_estimator_uses_edge_period_below_count_window_threshold() {
  EncoderConfiguration encoder{};
  encoder.counts_per_output_revolution = 184;
  encoder.estimator_min_counts = 4;
  encoder.estimator_max_window_us = 20000;
  encoder.estimator_stale_timeout_us = 100000;
  VelocityEstimator estimator;
  estimator.configure(encoder, 0.0F);
  estimator.reset(0, 1000);
  constexpr uint32_t edge_period_us = 17074;
  const float velocity = estimator.update(2, 36000, 35148, edge_period_us, 1);
  TEST_ASSERT_FLOAT_WITHIN(0.02F, 2.0F, velocity);
  TEST_ASSERT_FLOAT_WITHIN(0.0001F, 0.0F,
                           estimator.update(2, 136000, 35148, edge_period_us, 1));
}

void test_encoder_period_averager_uses_recent_edge_history() {
  EncoderPeriodAverager averager;
  averager.addEdge(1000U, 1);
  averager.addEdge(9000U, 1);
  averager.addEdge(21000U, 1);
  averager.addEdge(29000U, 1);
  averager.addEdge(45000U, 1);
  TEST_ASSERT_EQUAL_UINT32(11000U, averager.averagePeriodUs());

  // Direction changes must not mix the old direction into the new estimate.
  averager.addEdge(50000U, -1);
  TEST_ASSERT_EQUAL_UINT32(0U, averager.averagePeriodUs());
  averager.addEdge(62000U, -1);
  TEST_ASSERT_EQUAL_UINT32(12000U, averager.averagePeriodUs());
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
  TEST_ASSERT_EQUAL_UINT32(7U, settings.schema_version);
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

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_crc_standard_vector);
  RUN_TEST(test_incremental_controller_scales_integral_by_time);
  RUN_TEST(test_incremental_controller_does_not_wind_up);
  RUN_TEST(test_incremental_controller_reports_each_delta_term);
  RUN_TEST(test_velocity_controller_bounds_feedback_around_feedforward);
  RUN_TEST(test_motor_feedforward_maps_velocity_to_physical_duty);
  RUN_TEST(test_low_pass_filter_uses_cutoff_and_elapsed_time);
  RUN_TEST(test_two_point_current_calibration_calculates_gain_and_offset);
  RUN_TEST(test_two_point_current_calibration_rejects_insufficient_span);
  RUN_TEST(test_current_calibration_capture_averages_incrementally);
  RUN_TEST(test_motion_limiter_honors_acceleration_and_jerk);
  RUN_TEST(test_velocity_estimator_uses_output_shaft_cpr);
  RUN_TEST(test_velocity_estimator_uses_edge_period_below_count_window_threshold);
  RUN_TEST(test_encoder_period_averager_uses_recent_edge_history);
  RUN_TEST(test_sine_profile_stays_one_direction);
  RUN_TEST(test_waypoint_profile_interpolates_and_stops_at_duration);
  RUN_TEST(test_vin_divider_nominal_gain);
  RUN_TEST(test_driver_diagnostic_is_disabled_by_default);
  RUN_TEST(test_encoder_watchdog_grants_fresh_motion_demand_timeout);
  RUN_TEST(test_encoder_watchdog_refreshes_on_valid_edge);
  RUN_TEST(test_vin_calibration_ceiling_is_above_valid_sixteen_volt_input);
  RUN_TEST(test_characterization_peak_is_independent_of_encoder_polarity);
  return UNITY_END();
}
