#pragma once

#include <driver/rmt.h>

#include <array>
#include <cstddef>
#include <cstdint>

#include "control/StatusLedPattern.hpp"
#include "core/Types.hpp"

namespace mm {

class Ws2812StatusLed {
 public:
  bool begin(const StatusLedConfiguration& configuration);
  void notifyCommandReceived(uint64_t timestamp_us);
  void update(uint64_t timestamp_us, RunState state, uint32_t faults,
              float signed_motion);

 private:
  static constexpr size_t kBitsPerPixel = 24U;
  static constexpr size_t kMaximumItemCount =
      kBitsPerPixel * StatusLedConfiguration::kMaximumPixelCount;

  bool transmit();
  void fillSolid(const StatusLedColor& color);

  StatusLedConfiguration configuration_{};
  rmt_channel_t rmt_channel_ = RMT_CHANNEL_0;
  StatusLedPattern pattern_{};
  std::array<StatusLedColor, StatusLedConfiguration::kMaximumPixelCount>
      pixel_colors_{};
  std::array<rmt_item32_t, kMaximumItemCount> items_{};
  StatusLedColor displayed_{};
  uint64_t boot_test_started_us_ = 0U;
  uint64_t running_animation_started_us_ = 0U;
  bool initialized_ = false;
  bool display_valid_ = false;
  bool boot_test_transmitted_ = false;
  bool running_animation_active_ = false;
  uint8_t displayed_animation_frame_ = UINT8_MAX;
  int8_t motion_direction_ = 1;
  int8_t displayed_motion_direction_ = 0;
};

}  // namespace mm
