#include "drivers/Ws2812StatusLed.hpp"

#include <esp_err.h>
#include <esp_timer.h>

namespace mm {

bool Ws2812StatusLed::begin(const StatusLedConfiguration& configuration) {
  configuration_ = configuration;
  if (!configuration_.enabled) {
    return true;
  }

  rmt_channel_ = static_cast<rmt_channel_t>(configuration_.rmt_channel);
  rmt_config_t rmt_configuration = RMT_DEFAULT_CONFIG_TX(
      static_cast<gpio_num_t>(configuration_.data_pin), rmt_channel_);
  rmt_configuration.clk_div = configuration_.rmt_clock_divider;
  rmt_configuration.tx_config.loop_en = false;
  rmt_configuration.tx_config.carrier_en = false;
  rmt_configuration.tx_config.idle_output_en = true;
  rmt_configuration.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
  if (rmt_config(&rmt_configuration) != ESP_OK ||
      rmt_driver_install(rmt_channel_, 0U, 0U) != ESP_OK) {
    return false;
  }
  initialized_ = true;
  boot_test_started_us_ = static_cast<uint64_t>(esp_timer_get_time());
  return true;
}

void Ws2812StatusLed::notifyCommandReceived(const uint64_t timestamp_us) {
  if (configuration_.enabled) {
    pattern_.notifyCommandReceived(timestamp_us);
  }
}

void Ws2812StatusLed::update(const uint64_t timestamp_us, const RunState state,
                             const uint32_t faults,
                             const float signed_motion) {
  if (!initialized_) {
    return;
  }
  const uint64_t boot_test_duration_us =
      static_cast<uint64_t>(
          StatusLedConfiguration::kBootOrderTestDurationMs) *
      1000ULL;
  const uint64_t boot_test_elapsed_us =
      timestamp_us >= boot_test_started_us_
          ? timestamp_us - boot_test_started_us_
          : 0U;
  if (boot_test_elapsed_us < boot_test_duration_us) {
    if (!boot_test_transmitted_ &&
        rmt_wait_tx_done(rmt_channel_, 0U) == ESP_OK) {
      for (size_t pixel = 0U; pixel < configuration_.pixel_count; ++pixel) {
        pixel_colors_[pixel] = StatusLedPattern::bootOrderColor(
            pixel, configuration_.brightness);
      }
      boot_test_transmitted_ = transmit();
    }
    return;
  }
  if (signed_motion > 0.0F) {
    motion_direction_ = 1;
  } else if (signed_motion < 0.0F) {
    motion_direction_ = -1;
  }
  if (state == RunState::Running && faults == FaultNone) {
    if (!running_animation_active_) {
      running_animation_active_ = true;
      running_animation_started_us_ = timestamp_us;
      displayed_animation_frame_ = UINT8_MAX;
    }
    const uint64_t elapsed_us =
        timestamp_us >= running_animation_started_us_
            ? timestamp_us - running_animation_started_us_
            : 0U;
    const uint64_t step_us =
        static_cast<uint64_t>(
            StatusLedConfiguration::kRunningAnimationStepMs) *
        1000ULL;
    const uint8_t frame = static_cast<uint8_t>(
        step_us > 0U ? (elapsed_us / step_us) % 4U : 0U);
    if (frame == displayed_animation_frame_ &&
        motion_direction_ == displayed_motion_direction_) {
      return;
    }
    if (rmt_wait_tx_done(rmt_channel_, 0U) != ESP_OK) {
      return;
    }
    for (size_t pixel = 0U; pixel < configuration_.pixel_count; ++pixel) {
      pixel_colors_[pixel] = StatusLedPattern::runningColor(
          pixel, frame, motion_direction_, configuration_.brightness);
    }
    if (transmit()) {
      displayed_animation_frame_ = frame;
      displayed_motion_direction_ = motion_direction_;
      display_valid_ = false;
    }
    return;
  }
  running_animation_active_ = false;
  const StatusLedColor requested =
      pattern_.color(timestamp_us, state, faults, configuration_);
  if (display_valid_ && requested == displayed_) {
    return;
  }
  if (rmt_wait_tx_done(rmt_channel_, 0U) != ESP_OK) {
    return;
  }
  fillSolid(requested);
  if (transmit()) {
    displayed_ = requested;
    display_valid_ = true;
  }
}

void Ws2812StatusLed::fillSolid(const StatusLedColor& color) {
  for (size_t pixel = 0U; pixel < configuration_.pixel_count; ++pixel) {
    pixel_colors_[pixel] = color;
  }
}

bool Ws2812StatusLed::transmit() {
  const size_t item_count =
      static_cast<size_t>(configuration_.pixel_count) * kBitsPerPixel;
  for (size_t item_index = 0U; item_index < item_count; ++item_index) {
    const size_t pixel_index = item_index / kBitsPerPixel;
    const size_t pixel_bit = item_index % kBitsPerPixel;
    const StatusLedColor pixel_color = pixel_colors_[pixel_index];
    const uint32_t grb =
        (static_cast<uint32_t>(pixel_color.green) << 16U) |
        (static_cast<uint32_t>(pixel_color.red) << 8U) |
        static_cast<uint32_t>(pixel_color.blue);
    const bool one = (grb & (1UL << (23U - pixel_bit))) != 0U;
    auto& item = items_[item_index];
    item.level0 = 1U;
    item.duration0 =
        one ? configuration_.one_high_ticks : configuration_.zero_high_ticks;
    item.level1 = 0U;
    item.duration1 =
        one ? configuration_.one_low_ticks : configuration_.zero_low_ticks;
  }
  return rmt_write_items(rmt_channel_, items_.data(), item_count, false) ==
         ESP_OK;
}

}  // namespace mm
