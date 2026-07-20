#pragma once

#include <algorithm>
#include <cstdint>

namespace mm::protocol {

constexpr uint16_t kMaximumConfiguredStreamRateHz = 500U;
constexpr uint16_t kTelemetryPayloadBytes = 84U;
constexpr uint16_t kFrameOverheadBytes = 12U;
constexpr uint8_t kUartBitsPerByte = 10U;
constexpr uint8_t kMaximumStreamingUtilizationPercent = 70U;

constexpr uint16_t maximumTelemetryStreamRateHz(const uint32_t baud) {
  const uint64_t usable_bits_per_second =
      static_cast<uint64_t>(baud) * kMaximumStreamingUtilizationPercent / 100U;
  const uint64_t bits_per_frame =
      static_cast<uint64_t>(kTelemetryPayloadBytes + kFrameOverheadBytes) * kUartBitsPerByte;
  const uint64_t calculated = usable_bits_per_second / bits_per_frame;
  return static_cast<uint16_t>(
      std::min<uint64_t>(kMaximumConfiguredStreamRateHz, std::max<uint64_t>(1U, calculated)));
}

constexpr uint16_t constrainTelemetryStreamRateHz(const uint32_t baud,
                                                   const uint16_t requested_rate_hz) {
  return std::min(requested_rate_hz, maximumTelemetryStreamRateHz(baud));
}

}  // namespace mm::protocol
