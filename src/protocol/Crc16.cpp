#include "protocol/Crc16.hpp"

namespace mm::protocol {

uint16_t crc16CcittFalse(const uint8_t* data, const size_t size, uint16_t crc) {
  for (size_t index = 0; index < size; ++index) {
    crc ^= static_cast<uint16_t>(data[index]) << 8U;
    for (uint8_t bit = 0; bit < 8U; ++bit) {
      crc = (crc & 0x8000U) != 0U ? static_cast<uint16_t>((crc << 1U) ^ 0x1021U)
                                  : static_cast<uint16_t>(crc << 1U);
    }
  }
  return crc;
}

}  // namespace mm::protocol
