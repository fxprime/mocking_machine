import {
  MAXIMUM_PAYLOAD_SIZE,
  MessageId,
  MessageName,
  PROTOCOL_VERSION,
  ResultName,
  SYNC_1,
  SYNC_2
} from "./constants.mjs";
import { Emitter } from "./emitter.mjs";

const LITTLE_ENDIAN = true;
const textEncoder = new TextEncoder();

export function toUint8Array(value) {
  if (value === undefined || value === null) return new Uint8Array();
  if (value instanceof Uint8Array) return value;
  if (ArrayBuffer.isView(value)) {
    return new Uint8Array(value.buffer, value.byteOffset, value.byteLength);
  }
  if (value instanceof ArrayBuffer) return new Uint8Array(value);
  if (Array.isArray(value)) return Uint8Array.from(value);
  throw new TypeError("expected bytes as Uint8Array, ArrayBuffer, ArrayBufferView, or number[]");
}

export function crc16CcittFalse(value, initial = 0xffff) {
  const bytes = toUint8Array(value);
  let crc = initial & 0xffff;
  for (const byte of bytes) {
    crc ^= byte << 8;
    for (let bit = 0; bit < 8; ++bit) {
      crc = crc & 0x8000 ? ((crc << 1) ^ 0x1021) & 0xffff : (crc << 1) & 0xffff;
    }
  }
  return crc;
}

export function encodeFrame(messageId, payload = new Uint8Array(), options = {}) {
  const bytesPayload = toUint8Array(payload);
  if (!Number.isInteger(messageId) || messageId < 0 || messageId > 0xffff) {
    throw new RangeError("messageId must be a uint16");
  }
  if (bytesPayload.byteLength > MAXIMUM_PAYLOAD_SIZE) {
    throw new RangeError(`payload exceeds ${MAXIMUM_PAYLOAD_SIZE} bytes`);
  }
  const sequence = options.sequence ?? 0;
  const flags = options.flags ?? 0;
  if (!Number.isInteger(sequence) || sequence < 0 || sequence > 0xffff) {
    throw new RangeError("sequence must be a uint16");
  }
  if (!Number.isInteger(flags) || flags < 0 || flags > 0xff) {
    throw new RangeError("flags must be a uint8");
  }

  const frame = new Uint8Array(12 + bytesPayload.byteLength);
  const view = new DataView(frame.buffer);
  frame[0] = SYNC_1;
  frame[1] = SYNC_2;
  frame[2] = PROTOCOL_VERSION;
  frame[3] = flags;
  view.setUint16(4, messageId, LITTLE_ENDIAN);
  view.setUint16(6, sequence, LITTLE_ENDIAN);
  view.setUint16(8, bytesPayload.byteLength, LITTLE_ENDIAN);
  frame.set(bytesPayload, 10);
  view.setUint16(
    10 + bytesPayload.byteLength,
    crc16CcittFalse(frame.subarray(2, 10 + bytesPayload.byteLength)),
    LITTLE_ENDIAN
  );
  return frame;
}

export class FrameParser extends Emitter {
  #buffer = new Uint8Array();
  #decoder = new TextDecoder();

  push(value) {
    const incoming = toUint8Array(value);
    if (incoming.byteLength === 0) return;
    const combined = new Uint8Array(this.#buffer.byteLength + incoming.byteLength);
    combined.set(this.#buffer);
    combined.set(incoming, this.#buffer.byteLength);
    this.#buffer = combined;
    this.#parse();
  }

  reset() {
    this.#buffer = new Uint8Array();
    this.#decoder = new TextDecoder();
  }

  #emitText(bytes) {
    if (bytes.byteLength > 0) this.emit("text", this.#decoder.decode(bytes, { stream: true }));
  }

  #discard(count) {
    this.#buffer = this.#buffer.slice(count);
  }

  #syncIndex() {
    for (let index = 0; index + 1 < this.#buffer.byteLength; ++index) {
      if (this.#buffer[index] === SYNC_1 && this.#buffer[index + 1] === SYNC_2) return index;
    }
    return -1;
  }

  #parse() {
    while (this.#buffer.byteLength > 0) {
      const sync = this.#syncIndex();
      if (sync < 0) {
        const preserve = this.#buffer.at(-1) === SYNC_1 ? 1 : 0;
        this.#emitText(this.#buffer.subarray(0, this.#buffer.byteLength - preserve));
        this.#buffer = preserve ? this.#buffer.slice(-1) : new Uint8Array();
        return;
      }
      if (sync > 0) {
        this.#emitText(this.#buffer.subarray(0, sync));
        this.#discard(sync);
      }
      if (this.#buffer.byteLength < 10) return;

      const header = new DataView(
        this.#buffer.buffer,
        this.#buffer.byteOffset,
        Math.min(this.#buffer.byteLength, 10)
      );
      const version = header.getUint8(2);
      const payloadSize = header.getUint16(8, LITTLE_ENDIAN);
      if (version !== PROTOCOL_VERSION || payloadSize > MAXIMUM_PAYLOAD_SIZE) {
        const reason = version !== PROTOCOL_VERSION ? "unsupported-version" : "payload-too-large";
        this.emit("error", { type: "framing", reason, version, payloadSize });
        this.#discard(1);
        continue;
      }

      const totalSize = 12 + payloadSize;
      if (this.#buffer.byteLength < totalSize) return;
      const packet = this.#buffer.slice(0, totalSize);
      this.#discard(totalSize);
      const packetView = new DataView(packet.buffer);
      const expectedCrc = packetView.getUint16(10 + payloadSize, LITTLE_ENDIAN);
      const actualCrc = crc16CcittFalse(packet.subarray(2, 10 + payloadSize));
      if (actualCrc !== expectedCrc) {
        this.emit("error", { type: "crc", expected: expectedCrc, actual: actualCrc, packet });
        continue;
      }

      const payload = packet.slice(10, 10 + payloadSize);
      this.emit("frame", {
        version,
        flags: packetView.getUint8(3),
        messageId: packetView.getUint16(4, LITTLE_ENDIAN),
        sequence: packetView.getUint16(6, LITTLE_ENDIAN),
        payload,
        packet
      });
    }
  }
}

class PayloadReader {
  constructor(payload) {
    this.bytes = toUint8Array(payload);
    this.view = new DataView(this.bytes.buffer, this.bytes.byteOffset, this.bytes.byteLength);
  }

  has(offset, size) {
    return offset >= 0 && size >= 0 && offset + size <= this.bytes.byteLength;
  }

  require(offset, size, name) {
    if (!this.has(offset, size)) {
      throw new RangeError(`${name} payload is ${this.bytes.byteLength} bytes; need ${offset + size}`);
    }
  }

  u8(offset) { return this.view.getUint8(offset); }
  i8(offset) { return this.view.getInt8(offset); }
  u16(offset) { return this.view.getUint16(offset, LITTLE_ENDIAN); }
  u32(offset) { return this.view.getUint32(offset, LITTLE_ENDIAN); }
  i32(offset) { return this.view.getInt32(offset, LITTLE_ENDIAN); }
  u64(offset) { return this.view.getBigUint64(offset, LITTLE_ENDIAN); }
  i64(offset) { return this.view.getBigInt64(offset, LITTLE_ENDIAN); }
  f32(offset) { return this.view.getFloat32(offset, LITTLE_ENDIAN); }

  optional(offset, size, read, fallback) {
    return this.has(offset, size) ? read() : fallback;
  }

  text(offset, size) {
    this.require(offset, size, "string");
    const bytes = this.bytes.subarray(offset, offset + size);
    const end = bytes.indexOf(0);
    return new TextDecoder().decode(end < 0 ? bytes : bytes.subarray(0, end));
  }
}

class PayloadWriter {
  constructor(size) {
    this.bytes = new Uint8Array(size);
    this.view = new DataView(this.bytes.buffer);
  }

  u8(offset, value) { this.view.setUint8(offset, value); }
  i8(offset, value) { this.view.setInt8(offset, value); }
  u16(offset, value) { this.view.setUint16(offset, value, LITTLE_ENDIAN); }
  u32(offset, value) { this.view.setUint32(offset, value, LITTLE_ENDIAN); }
  i32(offset, value) { this.view.setInt32(offset, value, LITTLE_ENDIAN); }
  f32(offset, value) { this.view.setFloat32(offset, value, LITTLE_ENDIAN); }

  text(offset, size, value) {
    const encoded = textEncoder.encode(String(value ?? ""));
    if (encoded.byteLength >= size) throw new RangeError(`text must encode to fewer than ${size} bytes`);
    this.bytes.set(encoded, offset);
  }
}

function requirePayload(reader, size, name) {
  reader.require(0, size, name);
}

function decodeHeartbeat(reader) {
  requirePayload(reader, 65, "HEARTBEAT");
  return {
    uptimeUs: reader.u64(0),
    settingsSchema: reader.u32(8),
    faults: reader.u32(12),
    state: reader.u8(16),
    buildVersion: reader.text(17, 48)
  };
}

function decodeSettings(reader) {
  requirePayload(reader, 93, "SETTINGS");
  return {
    schemaVersion: reader.u32(0),
    baud: reader.u32(4),
    controlPeriodUs: reader.u32(8),
    countsPerRevolution: reader.u32(12),
    streamRateHz: reader.u16(16),
    selectedProfileId: reader.u16(18),
    kp: reader.f32(20),
    ki: reader.f32(24),
    kd: reader.f32(28),
    maximumVelocityRadS: reader.f32(32),
    maximumAccelerationRadS2: reader.f32(36),
    maximumJerkRadS3: reader.f32(40),
    maximumCurrentA: reader.f32(44),
    maximumDuty: reader.f32(48),
    startDutyForward: reader.f32(52),
    startDutyReverse: reader.f32(56),
    loadSettingId: reader.u8(60),
    loadCount: reader.u8(61),
    motorDirection: reader.i8(62),
    stopMode: reader.u8(63),
    supplyDividerGain: reader.f32(64),
    supplyInputOffsetV: reader.f32(68),
    minimumSupplyVoltageV: reader.f32(72),
    maximumSupplyVoltageV: reader.f32(76),
    supplyVoltagePin: reader.u8(80),
    currentGainAPerV: reader.f32(81),
    currentOffsetV: reader.f32(85),
    currentSensePin: reader.u8(89),
    currentSenseEnabled: reader.u8(90) !== 0,
    driverDiagnosticEnabled: reader.u8(91) !== 0,
    driverDiagnosticPin: reader.u8(92),
    encoderTimeoutMs: reader.optional(93, 4, () => reader.u32(93), 250),
    encoderTimeoutVelocityRadS: reader.optional(97, 4, () => reader.f32(97), 1),
    maximumFeedbackCorrection: reader.optional(101, 4, () => reader.f32(101), 0.1),
    estimatorMinimumCounts: reader.optional(105, 1, () => reader.u8(105), 4),
    estimatorMaximumWindowUs: reader.optional(106, 4, () => reader.u32(106), 20000),
    estimatorStaleTimeoutUs: reader.optional(110, 4, () => reader.u32(110), 100000),
    currentFilterCutoffHz: reader.optional(114, 4, () => reader.f32(114), 20),
    zeroIndexMinimumIntervalUs: reader.optional(118, 4, () => reader.u32(118), 5000),
    zeroIndexCorrectionGain: reader.optional(122, 4, () => reader.f32(122), 0.1),
    encoderDirection: reader.optional(126, 1, () => reader.i8(126), 1),
    zeroIndexMinimumSeparationRevolutions:
      reader.optional(127, 4, () => reader.f32(127), 0.5),
    characterizationDynamicsFilterCutoffHz:
      reader.optional(131, 4, () => reader.f32(131), 20),
    characterizationDynamicsQuantile:
      reader.optional(135, 4, () => reader.f32(135), 0.95),
    characterizationRecommendationSafetyFactor:
      reader.optional(139, 4, () => reader.f32(139), 0.7),
    motorModelGainForwardRadSPerDuty:
      reader.optional(143, 4, () => reader.f32(143), 0),
    motorModelGainReverseRadSPerDuty:
      reader.optional(147, 4, () => reader.f32(147), 0),
    motorModelTimeConstantForwardS:
      reader.optional(151, 4, () => reader.f32(151), 0),
    motorModelTimeConstantReverseS:
      reader.optional(155, 4, () => reader.f32(155), 0),
    motorModelVelocityProcessNoiseRadS2:
      reader.optional(159, 4, () => reader.f32(159), 25),
    motorModelDisturbanceProcessNoiseRadS3:
      reader.optional(163, 4, () => reader.f32(163), 100),
    motorModelEncoderMeasurementNoiseCounts:
      reader.optional(167, 4, () => reader.f32(167), 0.5),
    velocityEstimatorMethod: reader.optional(171, 1, () => reader.u8(171), 0),
    velocityAccelerationWindowSamples: reader.optional(172, 1, () => reader.u8(172), 5),
    rotorZeroOffsetTicks: reader.optional(173, 4, () => reader.u32(173), 0),
    zeroIndexReferenceSide: reader.optional(177, 1, () => reader.u8(177), 0),
    zeroIndexHysteresisCalibrated:
      reader.optional(178, 1, () => reader.u8(178) !== 0, false),
    clockwiseRisingCorrectionTicks: reader.optional(179, 4, () => reader.i32(179), 0),
    clockwiseFallingCorrectionTicks: reader.optional(183, 4, () => reader.i32(183), 0),
    zeroIndexCalibrationDuty: reader.optional(187, 4, () => reader.f32(187), 0.1),
    zeroIndexCalibrationTimeoutMs: reader.optional(191, 4, () => reader.u32(191), 120000),
    zeroIndexCalibrationReversalPauseMs:
      reader.optional(195, 2, () => reader.u16(195), 1000),
    zeroIndexCalibrationMaximumErrorTicks:
      reader.optional(197, 2, () => reader.u16(197), 2),
    zeroIndexCalibrationSpeedRpm: reader.optional(199, 4, () => reader.f32(199), 15),
    jerkLimitEnabled: reader.optional(203, 1, () => reader.u8(203) !== 0, true)
  };
}

export function decodeProfile(payload) {
  const reader = payload instanceof PayloadReader ? payload : new PayloadReader(payload);
  requirePayload(reader, 168, "PROFILE_CONFIGURATION");
  const pointCount = Math.min(reader.u8(39), 16);
  return {
    profileId: reader.u16(0),
    kind: reader.u8(2),
    name: reader.text(3, 16),
    targetVelocityRadS: reader.f32(19),
    sineMeanRadS: reader.f32(23),
    sineAmplitudeRadS: reader.f32(27),
    sineFrequencyHz: reader.f32(31),
    durationMs: reader.u32(35),
    points: Array.from({ length: pointCount }, (_, index) => ({
      timeMs: reader.u32(40 + index * 8),
      velocityRadS: reader.f32(44 + index * 8)
    }))
  };
}

function decodeLoadConfiguration(reader) {
  requirePayload(reader, 86, "LOAD_CONFIGURATION");
  const count = Math.min(reader.u8(1), 12);
  return {
    settingId: reader.u8(0),
    loads: Array.from({ length: count }, (_, index) => ({
      slotId: reader.u8(2 + index * 7),
      positionDeg: reader.u16(3 + index * 7),
      strength: reader.f32(5 + index * 7)
    }))
  };
}

function decodeTelemetry(reader) {
  requirePayload(reader, 60, "TELEMETRY");
  return {
    timestampUs: reader.u64(0),
    lastZeroTimestampUs: reader.u64(8),
    encoderCount: reader.i64(16),
    lastZeroEncoderCount: reader.i64(24),
    desiredVelocityRadS: reader.f32(32),
    measuredVelocityRadS: reader.f32(36),
    controllerOutput: reader.f32(40),
    currentA: reader.f32(44),
    supplyVoltageV: reader.f32(48),
    faults: reader.u32(52),
    profileId: reader.u16(56),
    loadSettingId: reader.u8(58),
    state: reader.u8(59),
    controllerProportionalTerm: reader.optional(60, 4, () => reader.f32(60), 0),
    controllerIntegralTerm: reader.optional(64, 4, () => reader.f32(64), 0),
    controllerDerivativeTerm: reader.optional(68, 4, () => reader.f32(68), 0),
    rotorPositionDeg: reader.optional(72, 4, () => reader.f32(72), Number.NaN),
    zeroIndexSequence: reader.optional(76, 4, () => reader.u32(76), 0),
    zeroIndexRejectedCount: reader.optional(80, 4, () => reader.u32(80), 0)
  };
}

const decoders = new Map([
  [MessageId.HEARTBEAT, decodeHeartbeat],
  [MessageId.ACK, reader => {
    requirePayload(reader, 3, "ACK");
    const requestMessageId = reader.u16(0);
    const result = reader.u8(2);
    return {
      requestMessageId,
      requestName: MessageName[requestMessageId] ?? "UNKNOWN",
      result,
      resultName: ResultName[result] ?? "UNKNOWN",
      ok: result === 0
    };
  }],
  [MessageId.SETTINGS, decodeSettings],
  [MessageId.PROFILE_CONFIGURATION, decodeProfile],
  [MessageId.LOAD_CONFIGURATION, decodeLoadConfiguration],
  [MessageId.BEARING_CONFIGURATION, reader => {
    requirePayload(reader, 2, "BEARING_CONFIGURATION");
    return { settingId: reader.u8(0), brokenBearing: reader.u8(1) !== 0 };
  }],
  [MessageId.TELEMETRY, decodeTelemetry],
  [MessageId.CURRENT_CALIBRATION_STATUS, reader => {
    requirePayload(reader, 27, "CURRENT_CALIBRATION_STATUS");
    return {
      capturedMask: reader.u8(0),
      capturePoint: reader.u8(1),
      lastResult: reader.u8(2),
      point1VoltageV: reader.f32(3),
      point1ReferenceA: reader.f32(7),
      point2VoltageV: reader.f32(11),
      point2ReferenceA: reader.f32(15),
      candidateGainAPerV: reader.f32(19),
      candidateOffsetV: reader.f32(23)
    };
  }],
  [MessageId.ROTOR_ZERO_CALIBRATION_STATUS, reader => {
    requirePayload(reader, 15, "ROTOR_ZERO_CALIBRATION_STATUS");
    return {
      state: reader.u8(0),
      captureCount: reader.u8(1),
      lastResult: reader.u8(2),
      currentPositionDeg: reader.f32(3),
      candidateOffsetTicks: reader.u32(7),
      savedOffsetTicks: reader.u32(11)
    };
  }],
  [MessageId.ZERO_INDEX_HYSTERESIS_CALIBRATION_STATUS, reader => {
    requirePayload(reader, 20, "ZERO_INDEX_HYSTERESIS_CALIBRATION_STATUS");
    return {
      stage: reader.u8(0),
      clockwiseCount: reader.u8(1),
      counterclockwiseCount: reader.u8(2),
      referenceSide: reader.u8(3),
      candidateValid: reader.u8(4) !== 0,
      lastResult: reader.u8(5),
      clockwiseRisingCorrectionTicks: reader.i32(6),
      clockwiseFallingCorrectionTicks: reader.i32(10),
      maximumResidualTicks: reader.u16(14),
      currentPositionDeg: reader.f32(16)
    };
  }],
  [MessageId.CHARACTERIZATION_RESULT, reader => {
    requirePayload(reader, 16, "CHARACTERIZATION_RESULT");
    return {
      startDutyForward: reader.f32(0),
      startDutyReverse: reader.f32(4),
      maximumVelocityForwardRadS: reader.f32(8),
      maximumVelocityReverseRadS: reader.f32(12),
      accelerationForwardRadS2: reader.optional(16, 4, () => reader.f32(16), Number.NaN),
      accelerationReverseRadS2: reader.optional(20, 4, () => reader.f32(20), Number.NaN),
      jerkForwardRadS3: reader.optional(24, 4, () => reader.f32(24), Number.NaN),
      jerkReverseRadS3: reader.optional(28, 4, () => reader.f32(28), Number.NaN),
      velocityGainForwardRadSPerDuty:
        reader.optional(32, 4, () => reader.f32(32), Number.NaN),
      velocityGainReverseRadSPerDuty:
        reader.optional(36, 4, () => reader.f32(36), Number.NaN),
      timeConstantForwardS: reader.optional(40, 4, () => reader.f32(40), Number.NaN),
      timeConstantReverseS: reader.optional(44, 4, () => reader.f32(44), Number.NaN)
    };
  }],
  [MessageId.CHARACTERIZATION_STATUS, reader => {
    requirePayload(reader, 10, "CHARACTERIZATION_STATUS");
    return {
      stage: reader.u8(0),
      resultPending: reader.u8(1) !== 0,
      appliedDuty: reader.f32(2),
      measuredVelocityRadS: reader.f32(6)
    };
  }]
]);

export function decodeMessage(frame) {
  const name = MessageName[frame.messageId] ?? "UNKNOWN";
  const decode = decoders.get(frame.messageId);
  let data = frame.payload;
  if (decode) data = decode(new PayloadReader(frame.payload));
  return { ...frame, name, data };
}

export function encodeController({ kp, ki, kd }) {
  const writer = new PayloadWriter(12);
  writer.f32(0, kp);
  writer.f32(4, ki);
  writer.f32(8, kd);
  return writer.bytes;
}

export function encodeEnabled(enabled) {
  return Uint8Array.of(enabled ? 1 : 0);
}

export function encodeParameter(parameterId, value, persist = false) {
  const writer = new PayloadWriter(7);
  writer.u16(0, parameterId);
  writer.f32(2, value);
  writer.u8(6, persist ? 1 : 0);
  return writer.bytes;
}

export function encodeProfile(profile) {
  if (!Array.isArray(profile.points ?? [])) throw new TypeError("profile.points must be an array");
  if ((profile.points?.length ?? 0) > 16) throw new RangeError("a profile supports at most 16 points");
  const writer = new PayloadWriter(168);
  writer.u16(0, profile.profileId);
  writer.u8(2, profile.kind);
  writer.text(3, 16, profile.name);
  writer.f32(19, profile.targetVelocityRadS ?? 0);
  writer.f32(23, profile.sineMeanRadS ?? 0);
  writer.f32(27, profile.sineAmplitudeRadS ?? 0);
  writer.f32(31, profile.sineFrequencyHz ?? 0);
  writer.u32(35, profile.durationMs);
  writer.u8(39, profile.points?.length ?? 0);
  for (const [index, point] of (profile.points ?? []).entries()) {
    writer.u32(40 + index * 8, point.timeMs);
    writer.f32(44 + index * 8, point.velocityRadS);
  }
  return writer.bytes;
}

export function encodeStoredProfile(profile, persist = true) {
  const payload = new Uint8Array(169);
  payload[0] = persist ? 1 : 0;
  payload.set(encodeProfile(profile), 1);
  return payload;
}

export function encodeLoadConfiguration({ settingId, loads }) {
  if (!Array.isArray(loads)) throw new TypeError("loads must be an array");
  if (loads.length > 12) throw new RangeError("load configuration supports at most 12 loads");
  const writer = new PayloadWriter(86);
  writer.u8(0, settingId);
  writer.u8(1, loads.length);
  for (const [index, load] of loads.entries()) {
    writer.u8(2 + index * 7, load.slotId);
    writer.u16(3 + index * 7, load.positionDeg);
    writer.f32(5 + index * 7, load.strength);
  }
  return writer.bytes;
}

export function encodeBearingConfiguration({ settingId, brokenBearing }) {
  return Uint8Array.of(settingId, brokenBearing ? 1 : 0);
}

export function encodeFloat32(value) {
  const writer = new PayloadWriter(4);
  writer.f32(0, value);
  return writer.bytes;
}

export function encodeVelocityTest({ targetVelocityRadS, durationMs }) {
  const writer = new PayloadWriter(8);
  writer.f32(0, targetVelocityRadS);
  writer.u32(4, durationMs);
  return writer.bytes;
}

export function encodeVelocitySequence({ holdMs, levelsRadS }) {
  if (!Array.isArray(levelsRadS) || levelsRadS.length === 0 || levelsRadS.length > 16) {
    throw new RangeError("levelsRadS must contain 1 to 16 values");
  }
  const writer = new PayloadWriter(72);
  writer.u32(0, holdMs);
  writer.u8(4, levelsRadS.length);
  for (const [index, level] of levelsRadS.entries()) writer.f32(8 + index * 4, level);
  return writer.bytes;
}

export function encodeCurrentCalibration(action, referenceCurrentA = 0) {
  const writer = new PayloadWriter(5);
  writer.u8(0, action);
  writer.f32(1, referenceCurrentA);
  return writer.bytes;
}

export function encodeZeroIndexCalibration(action, referenceSide = 0) {
  return Uint8Array.of(action, referenceSide);
}

export function encodeCharacterizationAction({
  save = false,
  applyAcceleration = false,
  applyJerk = false
} = {}) {
  return Uint8Array.of(
    (save ? 1 : 0) | (applyAcceleration ? 2 : 0) | (applyJerk ? 4 : 0)
  );
}

export function encodeUint16(value) {
  const writer = new PayloadWriter(2);
  writer.u16(0, value);
  return writer.bytes;
}
