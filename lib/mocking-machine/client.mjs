import {
  CurrentCalibrationAction,
  MessageId,
  ResultName,
  RotorZeroCalibrationAction,
  ZeroIndexCalibrationAction
} from "./constants.mjs";
import {
  FrameParser,
  decodeMessage,
  encodeBearingConfiguration,
  encodeCharacterizationAction,
  encodeController,
  encodeCurrentCalibration,
  encodeEnabled,
  encodeFloat32,
  encodeFrame,
  encodeLoadConfiguration,
  encodeParameter,
  encodeStoredProfile,
  encodeUint16,
  encodeVelocitySequence,
  encodeVelocityTest,
  encodeZeroIndexCalibration,
  toUint8Array
} from "./codec.mjs";
import { Emitter } from "./emitter.mjs";

const textEncoder = new TextEncoder();

export class MachineCommandError extends Error {
  constructor(ack) {
    const resultName = ResultName[ack.data.result] ?? `code ${ack.data.result}`;
    super(`${ack.data.requestName} rejected: ${resultName}`);
    this.name = "MachineCommandError";
    this.ack = ack;
    this.requestMessageId = ack.data.requestMessageId;
    this.result = ack.data.result;
  }
}

export class MachineTimeoutError extends Error {
  constructor(messageId, sequence, timeoutMs) {
    super(`message 0x${messageId.toString(16).padStart(4, "0")} sequence ${sequence} timed out after ${timeoutMs} ms`);
    this.name = "MachineTimeoutError";
    this.messageId = messageId;
    this.sequence = sequence;
    this.timeoutMs = timeoutMs;
  }
}

export class MockingMachineClient extends Emitter {
  #transport;
  #parser = new FrameParser();
  #sequence;
  #requestTimeoutMs;
  #pending = new Map();
  #connected = false;
  #unsubscribers = [];
  #writeChain = Promise.resolve();

  constructor({ transport, requestTimeoutMs = 3000, initialSequence = 1 } = {}) {
    super();
    if (!transport || typeof transport.write !== "function" || typeof transport.on !== "function") {
      throw new TypeError("transport must provide write(bytes) and on(event, listener)");
    }
    this.#transport = transport;
    this.#requestTimeoutMs = requestTimeoutMs;
    this.#sequence = initialSequence & 0xffff;
    this.#parser.on("frame", frame => this.#handleFrame(frame));
    this.#parser.on("text", text => this.emit("text", text));
    this.#parser.on("error", error => this.emit("protocolError", error));
  }

  get connected() {
    return this.#connected;
  }

  async connect() {
    if (this.#connected) return;
    this.#unsubscribers.push(
      this.#transport.on("data", bytes => this.#parser.push(bytes)),
      this.#transport.on("error", error => this.emit("transportError", error)),
      this.#transport.on("close", () => {
        if (this.#connected) this.#handleDisconnect(new Error("transport closed"));
      })
    );
    try {
      await this.#transport.open?.();
      this.#connected = true;
      this.emit("connect");
    } catch (error) {
      this.#unsubscribeTransport();
      throw error;
    }
  }

  async disconnect() {
    if (!this.#connected && this.#unsubscribers.length === 0) return;
    this.#connected = false;
    this.#rejectPending(new Error("client disconnected"));
    this.#unsubscribeTransport();
    await this.#transport.close?.();
    this.#parser.reset();
    this.emit("disconnect");
  }

  #unsubscribeTransport() {
    for (const unsubscribe of this.#unsubscribers.splice(0)) unsubscribe?.();
  }

  #handleDisconnect(error) {
    this.#connected = false;
    this.#rejectPending(error);
    this.#unsubscribeTransport();
    this.emit("disconnect", error);
  }

  #rejectPending(error) {
    for (const pending of this.#pending.values()) {
      clearTimeout(pending.timer);
      pending.reject(error);
    }
    this.#pending.clear();
  }

  #nextSequence() {
    const sequence = this.#sequence;
    this.#sequence = (this.#sequence + 1) & 0xffff;
    return sequence;
  }

  #write(bytes) {
    if (!this.#connected) return Promise.reject(new Error("client is not connected"));
    const operation = this.#writeChain.then(() => this.#transport.write(bytes));
    this.#writeChain = operation.catch(() => {});
    return operation;
  }

  async send(messageId, payload = new Uint8Array(), options = {}) {
    const sequence = options.sequence ?? this.#nextSequence();
    await this.#write(encodeFrame(messageId, payload, {
      sequence,
      flags: options.flags ?? 0
    }));
    return sequence;
  }

  async sendAscii(command) {
    if (typeof command !== "string" || command.includes("\n") || command.includes("\r")) {
      throw new TypeError("ASCII command must be one line");
    }
    await this.#write(textEncoder.encode(`${command}\n`));
  }

  request(messageId, payload = new Uint8Array(), options = {}) {
    const sequence = this.#nextSequence();
    const responseMessageId = options.responseMessageId ?? MessageId.ACK;
    const timeoutMs = options.timeoutMs ?? this.#requestTimeoutMs;
    const promise = new Promise((resolve, reject) => {
      const timer = setTimeout(() => {
        this.#pending.delete(sequence);
        reject(new MachineTimeoutError(messageId, sequence, timeoutMs));
      }, timeoutMs);
      this.#pending.set(sequence, {
        requestMessageId: messageId,
        responseMessageId,
        resolve,
        reject,
        timer
      });
    });
    this.send(messageId, payload, { ...options, sequence }).catch(error => {
      const pending = this.#pending.get(sequence);
      if (!pending) return;
      clearTimeout(pending.timer);
      this.#pending.delete(sequence);
      pending.reject(error);
    });
    return promise;
  }

  #handleFrame(frame) {
    let message;
    try {
      message = decodeMessage(frame);
    } catch (error) {
      this.emit("protocolError", { type: "payload", error, frame });
      return;
    }

    this.emit("frame", frame);
    this.emit("message", message);
    this.emit(message.name.toLowerCase(), message.data);

    const pending = this.#pending.get(message.sequence);
    if (!pending) return;
    if (message.messageId === MessageId.ACK) {
      if (message.data.requestMessageId !== pending.requestMessageId) return;
      if (pending.responseMessageId !== MessageId.ACK && message.data.ok) return;
    } else if (message.messageId !== pending.responseMessageId) {
      return;
    }

    clearTimeout(pending.timer);
    this.#pending.delete(message.sequence);
    if (message.messageId === MessageId.ACK && !message.data.ok) {
      pending.reject(new MachineCommandError(message));
    } else {
      pending.resolve(message.data);
    }
  }

  getSettings(options) {
    return this.request(MessageId.GET_SETTINGS, undefined, {
      ...options,
      responseMessageId: MessageId.SETTINGS
    });
  }

  async getProfiles({ settleMs = 50 } = {}) {
    const profiles = [];
    let settleTimer;
    let resolveCollection;
    const complete = new Promise(resolve => { resolveCollection = resolve; });
    const sequence = this.#nextSequence();
    const settle = () => {
      clearTimeout(settleTimer);
      settleTimer = setTimeout(() => resolveCollection(profiles), settleMs);
    };
    const unsubscribe = this.on("message", message => {
      if (message.sequence !== sequence ||
          message.messageId !== MessageId.PROFILE_CONFIGURATION) return;
      profiles.push(message.data);
      settle();
    });
    try {
      await this.send(MessageId.GET_PROFILES, undefined, { sequence });
      settle();
      return await complete;
    } finally {
      clearTimeout(settleTimer);
      unsubscribe();
    }
  }

  getLoadConfiguration(options) {
    return this.request(MessageId.GET_LOAD_CONFIGURATION, undefined, {
      ...options,
      responseMessageId: MessageId.LOAD_CONFIGURATION
    });
  }

  getBearingConfiguration(options) {
    return this.request(MessageId.GET_BEARING_CONFIGURATION, undefined, {
      ...options,
      responseMessageId: MessageId.BEARING_CONFIGURATION
    });
  }

  startStream(options) { return this.request(MessageId.START_STREAM, undefined, options); }
  stopStream(options) { return this.request(MessageId.STOP_STREAM, undefined, options); }
  arm(options) { return this.request(MessageId.ARM, undefined, options); }
  startRun(options) { return this.request(MessageId.START_RUN, undefined, options); }
  stop(options) { return this.request(MessageId.STOP_RUN, undefined, options); }
  clearFaults(options) { return this.request(MessageId.CLEAR_FAULTS, undefined, options); }

  selectProfile(profileId, options) {
    return this.request(MessageId.SELECT_PROFILE, encodeUint16(profileId), options);
  }

  setDefaultProfile(profileId, options) {
    return this.request(MessageId.SET_DEFAULT_PROFILE, encodeUint16(profileId), options);
  }

  setController(gains, options) {
    return this.request(MessageId.SET_CONTROLLER, encodeController(gains), options);
  }

  saveController(gains, options) {
    return this.request(MessageId.SAVE_CONTROLLER, encodeController(gains), options);
  }

  setDriverDiagnostic(enabled, options) {
    return this.request(MessageId.SET_DRIVER_DIAGNOSTIC, encodeEnabled(enabled), options);
  }

  setCurrentSense(enabled, options) {
    return this.request(MessageId.SET_CURRENT_SENSE, encodeEnabled(enabled), options);
  }

  setParameter(parameterId, value, { persist = false, ...options } = {}) {
    return this.request(
      MessageId.SET_PARAMETER,
      encodeParameter(parameterId, value, persist),
      options
    );
  }

  setProfile(profile, { persist = true, ...options } = {}) {
    return this.request(
      MessageId.SET_PROFILE,
      encodeStoredProfile(profile, persist),
      options
    );
  }

  createProfile(profile, { persist = true, ...options } = {}) {
    return this.request(
      MessageId.CREATE_PROFILE,
      encodeStoredProfile(profile, persist),
      options
    );
  }

  setLoadConfiguration(configuration, options) {
    return this.request(
      MessageId.SET_LOAD_CONFIGURATION,
      encodeLoadConfiguration(configuration),
      options
    );
  }

  setBearingConfiguration(configuration, options) {
    return this.request(
      MessageId.SET_BEARING_CONFIGURATION,
      encodeBearingConfiguration(configuration),
      options
    );
  }

  motorTest(signedRawDuty, options) {
    return this.request(MessageId.MOTOR_TEST, encodeFloat32(signedRawDuty), options);
  }

  startVelocityTest(test, options) {
    return this.request(MessageId.START_VELOCITY_TEST, encodeVelocityTest(test), options);
  }

  startVelocitySequence(sequence, options) {
    return this.request(
      MessageId.START_VELOCITY_SEQUENCE,
      encodeVelocitySequence(sequence),
      options
    );
  }

  currentCalibration(action, referenceCurrentA = 0, options) {
    return this.request(
      MessageId.CURRENT_CALIBRATION,
      encodeCurrentCalibration(action, referenceCurrentA),
      options
    );
  }

  requestCurrentCalibrationStatus(options) {
    return this.request(
      MessageId.CURRENT_CALIBRATION,
      encodeCurrentCalibration(CurrentCalibrationAction.REQUEST_STATUS),
      { ...options, responseMessageId: MessageId.CURRENT_CALIBRATION_STATUS }
    );
  }

  supplyVoltageCalibration(referenceVoltageV, options) {
    return this.request(
      MessageId.SUPPLY_VOLTAGE_CALIBRATION,
      encodeFloat32(referenceVoltageV),
      options
    );
  }

  rotorZeroCalibration(action, options) {
    return this.request(MessageId.ROTOR_ZERO_CALIBRATION, Uint8Array.of(action), options);
  }

  requestRotorZeroCalibrationStatus(options) {
    return this.request(
      MessageId.ROTOR_ZERO_CALIBRATION,
      Uint8Array.of(RotorZeroCalibrationAction.REQUEST_STATUS),
      { ...options, responseMessageId: MessageId.ROTOR_ZERO_CALIBRATION_STATUS }
    );
  }

  zeroIndexCalibration(action, referenceSide = 0, options) {
    return this.request(
      MessageId.ZERO_INDEX_HYSTERESIS_CALIBRATION,
      encodeZeroIndexCalibration(action, referenceSide),
      options
    );
  }

  requestZeroIndexCalibrationStatus(options) {
    return this.request(
      MessageId.ZERO_INDEX_HYSTERESIS_CALIBRATION,
      encodeZeroIndexCalibration(ZeroIndexCalibrationAction.REQUEST_STATUS),
      {
        ...options,
        responseMessageId: MessageId.ZERO_INDEX_HYSTERESIS_CALIBRATION_STATUS
      }
    );
  }

  characterizationAction(action, options) {
    return this.request(
      MessageId.CHARACTERIZATION_ACTION,
      encodeCharacterizationAction(action),
      options
    );
  }

  async startCharacterization() {
    await this.sendAscii("arm");
    await this.sendAscii("characterize start CONFIRM_UNLOADED");
  }

  abortCharacterization() {
    return this.sendAscii("characterize abort");
  }

  writeRaw(bytes) {
    return this.#write(toUint8Array(bytes));
  }
}
