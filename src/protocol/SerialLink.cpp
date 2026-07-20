#include "protocol/SerialLink.hpp"

#include <algorithm>
#include <cstring>

#include "protocol/Crc16.hpp"

namespace mm::protocol {
namespace {
uint16_t readU16(const uint8_t* const data) {
  return static_cast<uint16_t>(data[0]) |
         static_cast<uint16_t>(static_cast<uint16_t>(data[1]) << 8U);
}

void writeU16(uint8_t* const data, const uint16_t value) {
  data[0] = static_cast<uint8_t>(value & 0xFFU);
  data[1] = static_cast<uint8_t>(value >> 8U);
}
}

void SerialLink::begin(HardwareSerial& serial, const FrameHandler frame_handler,
                       const LineHandler line_handler, void* const context) {
  serial_ = &serial;
  frame_handler_ = frame_handler;
  line_handler_ = line_handler;
  context_ = context;
}

void SerialLink::poll(const size_t byte_budget) {
  if (serial_ == nullptr) {
    return;
  }
  size_t count = 0;
  while (serial_->available() > 0 && count++ < byte_budget) {
    consume(static_cast<uint8_t>(serial_->read()));
  }
}

void SerialLink::consumeAscii(const uint8_t byte) {
  if (byte == '\r') {
    return;
  }
  if (byte == '\n') {
    if (line_size_ > 0U && line_handler_ != nullptr) {
      line_[line_size_] = '\0';
      line_handler_(context_, line_.data());
    }
    line_size_ = 0;
    return;
  }
  if (byte >= 0x20U && byte <= 0x7EU && line_size_ < line_.size() - 1U) {
    line_[line_size_++] = static_cast<char>(byte);
  } else if (line_size_ >= line_.size() - 1U) {
    line_size_ = 0;
  }
}

void SerialLink::consume(const uint8_t byte) {
  switch (receive_state_) {
    case ReceiveState::Idle:
      if (byte == kSync1) {
        receive_state_ = ReceiveState::Sync2;
      } else {
        consumeAscii(byte);
      }
      break;
    case ReceiveState::Sync2:
      if (byte == kSync2) {
        receive_index_ = 0;
        receive_state_ = ReceiveState::Header;
      } else {
        receive_state_ = ReceiveState::Idle;
        consumeAscii(byte);
      }
      break;
    case ReceiveState::Header:
      header_[receive_index_++] = byte;
      if (receive_index_ == kHeaderSize) {
        payload_size_ = readU16(&header_[6]);
        receive_index_ = 0;
        if (header_[0] != kProtocolVersion || payload_size_ > kMaximumPayload) {
          receive_state_ = ReceiveState::Idle;
        } else {
          receive_state_ = payload_size_ == 0U ? ReceiveState::CrcLow
                                               : ReceiveState::Payload;
        }
      }
      break;
    case ReceiveState::Payload:
      payload_[receive_index_++] = byte;
      if (receive_index_ == payload_size_) {
        receive_state_ = ReceiveState::CrcLow;
      }
      break;
    case ReceiveState::CrcLow:
      received_crc_ = byte;
      receive_state_ = ReceiveState::CrcHigh;
      break;
    case ReceiveState::CrcHigh: {
      received_crc_ |= static_cast<uint16_t>(byte) << 8U;
      uint16_t crc = crc16CcittFalse(header_.data(), header_.size());
      crc = crc16CcittFalse(payload_.data(), payload_size_, crc);
      if (crc == received_crc_ && frame_handler_ != nullptr) {
        const FrameView frame{static_cast<MessageId>(readU16(&header_[2])), readU16(&header_[4]),
                              header_[1], payload_.data(), payload_size_};
        frame_handler_(context_, frame);
      }
      receive_state_ = ReceiveState::Idle;
      break;
    }
  }
}

bool SerialLink::enqueue(const uint8_t* const data, const size_t size) {
  const size_t used = tx_head_ >= tx_tail_ ? tx_head_ - tx_tail_
                                           : tx_.size() - tx_tail_ + tx_head_;
  if (size > tx_.size() - used - 1U) {
    return false;
  }
  for (size_t index = 0; index < size; ++index) {
    tx_[tx_head_] = data[index];
    tx_head_ = (tx_head_ + 1U) % tx_.size();
  }
  return true;
}

bool SerialLink::send(const MessageId message_id, const uint16_t sequence,
                      const void* const payload, const uint16_t payload_size,
                      const uint8_t flags) {
  if (payload_size > kMaximumPayload || (payload_size > 0U && payload == nullptr)) {
    return false;
  }
  std::array<uint8_t, 2U + kHeaderSize + kMaximumPayload + 2U> frame{};
  frame[0] = kSync1;
  frame[1] = kSync2;
  frame[2] = kProtocolVersion;
  frame[3] = flags;
  writeU16(&frame[4], static_cast<uint16_t>(message_id));
  writeU16(&frame[6], sequence);
  writeU16(&frame[8], payload_size);
  if (payload_size > 0U) {
    std::memcpy(&frame[10], payload, payload_size);
  }
  uint16_t crc = crc16CcittFalse(&frame[2], kHeaderSize + payload_size);
  writeU16(&frame[10U + payload_size], crc);
  return enqueue(frame.data(), 12U + payload_size);
}

void SerialLink::serviceTx() {
  if (serial_ == nullptr || tx_tail_ == tx_head_) {
    return;
  }
  size_t available = static_cast<size_t>(std::max(0, serial_->availableForWrite()));
  while (available > 0U && tx_tail_ != tx_head_) {
    const size_t queued_contiguous = tx_head_ > tx_tail_ ? tx_head_ - tx_tail_
                                                         : tx_.size() - tx_tail_;
    const size_t requested = std::min(available, queued_contiguous);
    const size_t written = serial_->write(tx_.data() + tx_tail_, requested);
    if (written == 0U) {
      break;
    }
    tx_tail_ = (tx_tail_ + written) % tx_.size();
    available -= written;
  }
}

}  // namespace mm::protocol
