import assert from "node:assert/strict";
import {
  FrameParser,
  MachineCommandError,
  MessageId,
  MockingMachineClient,
  ParameterId,
  crc16CcittFalse,
  decodeMessage,
  encodeFrame,
  encodeProfile,
  encodeVelocitySequence
} from "../lib/mocking-machine/index.mjs";

class FakeTransport {
  listeners = new Map();
  writes = [];
  responder;
  opened = false;

  on(event, listener) {
    const listeners = this.listeners.get(event) ?? new Set();
    listeners.add(listener);
    this.listeners.set(event, listeners);
    return () => listeners.delete(listener);
  }

  emit(event, value) {
    for (const listener of this.listeners.get(event) ?? []) listener(value);
  }

  async open() { this.opened = true; }
  async close() { this.opened = false; }

  async write(bytes) {
    const copy = Uint8Array.from(bytes);
    this.writes.push(copy);
    await this.responder?.(copy, this);
  }
}

function frameFields(bytes) {
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  return {
    messageId: view.getUint16(4, true),
    sequence: view.getUint16(6, true),
    payload: bytes.slice(10, -2)
  };
}

function settingsPayload() {
  const payload = new Uint8Array(204);
  const view = new DataView(payload.buffer);
  view.setUint32(0, 24, true);
  view.setUint32(4, 115200, true);
  view.setUint32(8, 2000, true);
  view.setUint32(12, 184, true);
  view.setUint16(16, 50, true);
  view.setUint16(18, 2, true);
  view.setFloat32(20, 0.5, true);
  view.setInt8(62, -1);
  view.setUint8(90, 1);
  view.setUint8(203, 1);
  return payload;
}

function ackPayload(requestMessageId, result = 0) {
  const payload = new Uint8Array(3);
  const view = new DataView(payload.buffer);
  view.setUint16(0, requestMessageId, true);
  view.setUint8(2, result);
  return payload;
}

assert.equal(
  crc16CcittFalse(new TextEncoder().encode("123456789")),
  0x29b1,
  "CRC16/CCITT-FALSE check vector"
);

{
  const parser = new FrameParser();
  const frames = [];
  const text = [];
  const errors = [];
  parser.on("frame", frame => frames.push(frame));
  parser.on("text", chunk => text.push(chunk));
  parser.on("error", error => errors.push(error));

  const packet = encodeFrame(MessageId.ARM, undefined, { sequence: 42 });
  const mixed = new Uint8Array(4 + packet.length);
  mixed.set(new TextEncoder().encode("OK\r\n"));
  mixed.set(packet, 4);
  for (const byte of mixed) parser.push(Uint8Array.of(byte));

  assert.equal(text.join(""), "OK\r\n");
  assert.equal(frames.length, 1);
  assert.equal(frames[0].messageId, MessageId.ARM);
  assert.equal(frames[0].sequence, 42);
  assert.equal(errors.length, 0);

  const corrupt = encodeFrame(MessageId.STOP_RUN, undefined, { sequence: 43 });
  corrupt[corrupt.length - 1] ^= 0xff;
  parser.push(corrupt);
  assert.equal(errors.at(-1).type, "crc");
}

{
  const heartbeat = new Uint8Array(65);
  const view = new DataView(heartbeat.buffer);
  view.setBigUint64(0, 9007199254740993n, true);
  view.setUint32(8, 24, true);
  view.setUint32(12, 0x41, true);
  view.setUint8(16, 3);
  heartbeat.set(new TextEncoder().encode("v1.2.3"), 17);
  const decoded = decodeMessage({
    messageId: MessageId.HEARTBEAT,
    sequence: 7,
    flags: 0,
    payload: heartbeat
  });
  assert.equal(decoded.data.uptimeUs, 9007199254740993n);
  assert.equal(decoded.data.buildVersion, "v1.2.3");
  assert.equal(decoded.data.faults, 0x41);
}

{
  const encoded = encodeProfile({
    profileId: 4,
    kind: 2,
    name: "two-step",
    durationMs: 1000,
    points: [
      { timeMs: 0, velocityRadS: 1 },
      { timeMs: 1000, velocityRadS: -1 }
    ]
  });
  assert.equal(encoded.byteLength, 168);
  assert.equal(new DataView(encoded.buffer).getUint8(39), 2);
  assert.equal(encodeVelocitySequence({ holdMs: 500, levelsRadS: [1, -1] }).byteLength, 72);
  assert.throws(
    () => encodeVelocitySequence({ holdMs: 500, levelsRadS: [] }),
    /1 to 16/
  );
}

{
  const transport = new FakeTransport();
  transport.responder = async (bytes, target) => {
    const request = frameFields(bytes);
    if (request.messageId === MessageId.GET_SETTINGS) {
      target.emit("data", encodeFrame(MessageId.SETTINGS, settingsPayload(), {
        sequence: request.sequence
      }));
    } else if (request.messageId === MessageId.ARM) {
      target.emit("data", encodeFrame(MessageId.ACK, ackPayload(request.messageId), {
        sequence: request.sequence
      }));
    } else if (request.messageId === MessageId.START_RUN) {
      target.emit("data", encodeFrame(MessageId.ACK, ackPayload(request.messageId, 4), {
        sequence: request.sequence
      }));
    } else if (request.messageId === MessageId.CURRENT_CALIBRATION) {
      target.emit("data", encodeFrame(MessageId.ACK, ackPayload(request.messageId), {
        sequence: request.sequence
      }));
      const status = new Uint8Array(27);
      status[0] = 3;
      target.emit("data", encodeFrame(MessageId.CURRENT_CALIBRATION_STATUS, status, {
        sequence: request.sequence
      }));
    } else if (request.messageId === MessageId.SET_PARAMETER) {
      target.emit("data", encodeFrame(MessageId.ACK, ackPayload(request.messageId), {
        sequence: request.sequence
      }));
    }
  };

  const client = new MockingMachineClient({ transport, requestTimeoutMs: 100 });
  await client.connect();
  assert.equal(client.connected, true);

  const settings = await client.getSettings();
  assert.equal(settings.schemaVersion, 24);
  assert.equal(settings.baud, 115200);
  assert.equal(settings.motorDirection, -1);
  assert.equal(settings.jerkLimitEnabled, true);

  const ack = await client.arm();
  assert.equal(ack.ok, true);
  await assert.rejects(() => client.startRun(), error =>
    error instanceof MachineCommandError && error.result === 4
  );

  const calibration = await client.requestCurrentCalibrationStatus();
  assert.equal(calibration.capturedMask, 3);

  await client.setParameter(ParameterId.KP, 0.4, { persist: true });
  const parameterRequest = transport.writes.map(frameFields)
    .find(request => request.messageId === MessageId.SET_PARAMETER);
  assert.equal(parameterRequest.payload.byteLength, 7);
  assert.equal(new DataView(parameterRequest.payload.buffer).getUint8(6), 1);

  await client.disconnect();
  assert.equal(client.connected, false);
}

console.log("machine API tests passed");
