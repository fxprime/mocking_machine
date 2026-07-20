#pragma once

#include <Arduino.h>

#include <array>
#include <cstddef>
#include <cstdint>

namespace mm::protocol {

constexpr uint8_t kSync1 = 0xB5U;
constexpr uint8_t kSync2 = 0x62U;
constexpr uint8_t kProtocolVersion = 1U;
constexpr size_t kMaximumPayload = 512;

enum class MessageId : uint16_t {
  Heartbeat = 0x0001,
  Ack = 0x0002,
  GetSettings = 0x0100,
  Settings = 0x0101,
  SetController = 0x0110,
  SetDriverDiagnostic = 0x0111,
  SetCurrentSense = 0x0112,
  SetParameter = 0x0113,
  SaveController = 0x0114,
  SelectProfile = 0x0120,
  GetProfiles = 0x0121,
  ProfileConfiguration = 0x0122,
  SetProfile = 0x0123,
  CreateProfile = 0x0124,
  StartRun = 0x0200,
  StopRun = 0x0201,
  MotorTest = 0x0202,
  ClearFaults = 0x0203,
  Arm = 0x0204,
  StartVelocityTest = 0x0205,
  StartStream = 0x0210,
  StopStream = 0x0211,
  Telemetry = 0x0220,
  CurrentCalibration = 0x0300,
  SupplyVoltageCalibration = 0x0301,
  CurrentCalibrationStatus = 0x0302,
  CharacterizationResult = 0x0310,
  CharacterizationAction = 0x0311,
  CharacterizationStatus = 0x0312,
};

enum class ResultCode : uint8_t {
  Ok = 0,
  InvalidMessage = 1,
  InvalidLength = 2,
  InvalidValue = 3,
  UnsafeState = 4,
  StorageFailure = 5,
};

struct FrameView {
  MessageId message_id{};
  uint16_t sequence = 0;
  uint8_t flags = 0;
  const uint8_t* payload = nullptr;
  uint16_t payload_size = 0;
};

using FrameHandler = void (*)(void* context, const FrameView& frame);
using LineHandler = void (*)(void* context, const char* line);

class SerialLink {
 public:
  void begin(HardwareSerial& serial, FrameHandler frame_handler, LineHandler line_handler,
             void* context);
  void poll(size_t byte_budget = 128);
  void serviceTx();
  bool send(MessageId message_id, uint16_t sequence, const void* payload, uint16_t payload_size,
            uint8_t flags = 0);

 private:
  enum class ReceiveState : uint8_t { Idle, Sync2, Header, Payload, CrcLow, CrcHigh };
  static constexpr size_t kHeaderSize = 8;
  static constexpr size_t kAsciiLineSize = 160;
  static constexpr size_t kTxCapacity = 2048;

  void consume(uint8_t byte);
  void consumeAscii(uint8_t byte);
  bool enqueue(const uint8_t* data, size_t size);

  HardwareSerial* serial_ = nullptr;
  FrameHandler frame_handler_ = nullptr;
  LineHandler line_handler_ = nullptr;
  void* context_ = nullptr;
  ReceiveState receive_state_ = ReceiveState::Idle;
  std::array<uint8_t, kHeaderSize> header_{};
  std::array<uint8_t, kMaximumPayload> payload_{};
  uint16_t receive_index_ = 0;
  uint16_t payload_size_ = 0;
  uint16_t received_crc_ = 0;
  std::array<char, kAsciiLineSize> line_{};
  size_t line_size_ = 0;
  std::array<uint8_t, kTxCapacity> tx_{};
  size_t tx_head_ = 0;
  size_t tx_tail_ = 0;
};

}  // namespace mm::protocol
