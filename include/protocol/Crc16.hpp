#pragma once

#include <cstddef>
#include <cstdint>

namespace mm::protocol {

uint16_t crc16CcittFalse(const uint8_t* data, size_t size, uint16_t initial = 0xFFFFU);

}  // namespace mm::protocol

