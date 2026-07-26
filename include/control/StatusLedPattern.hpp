#pragma once

#include <cstddef>
#include <cstdint>

#include "core/Types.hpp"

namespace mm {

struct StatusLedColor {
  uint8_t red = 0U;
  uint8_t green = 0U;
  uint8_t blue = 0U;

  bool operator==(const StatusLedColor& other) const {
    return red == other.red && green == other.green && blue == other.blue;
  }

  bool operator!=(const StatusLedColor& other) const { return !(*this == other); }
};

class StatusLedPattern {
 public:
  static StatusLedColor bootOrderColor(const size_t pixel_index,
                                       const uint8_t level) {
    switch (pixel_index) {
      case 0U:
        return {level, 0U, 0U};
      case 1U:
        return {0U, 0U, level};
      case 2U:
        return {level, level, level};
      default:
        return {};
    }
  }

  static StatusLedColor runningColor(const size_t pixel_index,
                                     const uint8_t animation_frame,
                                     const int8_t direction,
                                     const uint8_t level) {
    if (pixel_index >= 4U) {
      return {};
    }
    const uint8_t head =
        direction >= 0
            ? static_cast<uint8_t>(animation_frame % 4U)
            : static_cast<uint8_t>((4U - animation_frame % 4U) % 4U);
    const uint8_t distance =
        direction >= 0
            ? static_cast<uint8_t>((head + 4U - pixel_index) % 4U)
            : static_cast<uint8_t>((pixel_index + 4U - head) % 4U);
    switch (distance) {
      case 0U:
        return {level, 0U, 0U};
      case 1U:
        return {static_cast<uint8_t>(level / 2U), 0U, 0U};
      case 2U:
        return {static_cast<uint8_t>(level / 6U), 0U, 0U};
      default:
        return {};
    }
  }

  void notifyCommandReceived(const uint64_t timestamp_us) {
    command_started_us_ = timestamp_us;
    command_active_ = true;
  }

  StatusLedColor color(const uint64_t timestamp_us, const RunState state,
                       const uint32_t faults,
                       const StatusLedConfiguration& configuration) {
    const uint8_t level = configuration.brightness;
    if (state == RunState::Fault || faults != FaultNone) {
      command_active_ = false;
      if (!fault_active_) {
        fault_started_us_ = timestamp_us;
        fault_active_ = true;
      }
      const uint64_t interval_us =
          static_cast<uint64_t>(configuration.fault_blink_interval_ms) * 1000ULL;
      const uint64_t elapsed_us = timestamp_us >= fault_started_us_
                                      ? timestamp_us - fault_started_us_
                                      : 0U;
      return interval_us > 0U && (elapsed_us / interval_us) % 2U == 0U
                 ? StatusLedColor{level, 0U, 0U}
                 : StatusLedColor{};
    }
    fault_active_ = false;

    if (command_active_ && state != RunState::Disarmed) {
      command_active_ = false;
    }
    if (command_active_) {
      const uint64_t on_us =
          static_cast<uint64_t>(configuration.command_blink_on_ms) * 1000ULL;
      const uint64_t off_us =
          static_cast<uint64_t>(configuration.command_blink_off_ms) * 1000ULL;
      const uint64_t elapsed_us = timestamp_us >= command_started_us_
                                      ? timestamp_us - command_started_us_
                                      : 0U;
      if (elapsed_us < on_us) {
        return {level, level, level};
      }
      if (elapsed_us < on_us + off_us) {
        return {};
      }
      if (elapsed_us < on_us + off_us + on_us) {
        return {level, level, level};
      }
      command_active_ = false;
    }

    switch (state) {
      case RunState::Armed:
        return {level, 0U, 0U};
      case RunState::Running:
        return {level, 0U, 0U};
      case RunState::Disarmed:
      default:
        return {0U, level, 0U};
    }
  }

 private:
  uint64_t command_started_us_ = 0U;
  uint64_t fault_started_us_ = 0U;
  bool command_active_ = false;
  bool fault_active_ = false;
};

}  // namespace mm
