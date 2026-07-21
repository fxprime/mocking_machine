import { configurationPreparation, profilePreparation, resultDescription } from "./protocol-status.mjs";
import { calculateStepMetrics, symmetricNiceLimit } from "./response-metrics.mjs";
import { estimateClosedLoopStepResponse } from "./step-response-estimator.mjs";
import { createLoadConfigurationCsv, createTelemetryCsv, defaultExportBaseName, sanitizeExportBaseName } from "./csv-export.mjs";
import { updateRotorVisualState } from "./rotor-position.mjs";
import { calculateEncoderCalibration } from "./encoder-calibration.mjs";
import { nextAvailableProfileId } from "./profile-collection.mjs";
import { maximumTelemetryStreamRateHz } from "./serial-bandwidth.mjs";
import { DeviceSynchronizer } from "./device-synchronizer.mjs";
import { normalizeRotorLoads, slotPosition, synchronizeRotorLoadDraft, ROTOR_SLOT_COUNT } from "./rotor-load.mjs";
import { machineStatusKey, shouldRefreshMachineUi } from "./machine-ui-state.mjs";
import { RunRecorder } from "./run-recorder.mjs";
import { readBaudPreference, writeBaudPreference } from "./baud-preference.mjs";
import { CommunicationMetrics } from "./communication-metrics.mjs";
import { createParameterCsv, parseParameterCsv } from "./parameter-csv.mjs";

const SYNC_1 = 0xb5;
const SYNC_2 = 0x62;
const VERSION = 1;
const MSG = { HEARTBEAT: 0x0001, ACK: 0x0002, GET_SETTINGS: 0x0100, SETTINGS: 0x0101, SET_CONTROLLER: 0x0110, SET_DRIVER_DIAGNOSTIC: 0x0111, SET_CURRENT_SENSE: 0x0112, SET_PARAMETER: 0x0113, SAVE_CONTROLLER: 0x0114, SELECT_PROFILE: 0x0120, GET_PROFILES: 0x0121, PROFILE_CONFIGURATION: 0x0122, SET_PROFILE: 0x0123, CREATE_PROFILE: 0x0124, SET_DEFAULT_PROFILE: 0x0125, GET_LOAD_CONFIGURATION: 0x0130, LOAD_CONFIGURATION: 0x0131, SET_LOAD_CONFIGURATION: 0x0132, START_RUN: 0x0200, STOP_RUN: 0x0201, MOTOR_TEST: 0x0202, CLEAR_FAULTS: 0x0203, ARM: 0x0204, START_VELOCITY_TEST: 0x0205, START_STREAM: 0x0210, STOP_STREAM: 0x0211, TELEMETRY: 0x0220, CURRENT_CALIBRATION: 0x0300, SUPPLY_VOLTAGE_CALIBRATION: 0x0301, CURRENT_CALIBRATION_STATUS: 0x0302, CHARACTERIZATION_RESULT: 0x0310, CHARACTERIZATION_ACTION: 0x0311, CHARACTERIZATION_STATUS: 0x0312 };
const CURRENT_CALIBRATION_ACTION = { RESET: 0, CAPTURE_POINT_1: 1, CAPTURE_POINT_2: 2, SAVE: 3, CANCEL: 4, REQUEST_STATUS: 5 };
const MAX_PROFILES = 8;
const PROFILE_ACTION_TIMEOUT_MS = 10000;
const deviceSynchronizer = new DeviceSynchronizer();
const runRecorder = new RunRecorder();
const communicationMetrics = new CommunicationMetrics();

let port;
let reader;
let writer;
let connected = false;
let sequence = 1;
let rx = [];
let textRx = "";
let samples = [];
let settings = {};
let profiles = [];
let editingProfile;
let selectedProfilePoint = 0;
let draggedProfilePoint = -1;
let profileAction;
let editingParameterKey;
let pendingParameterPayload;
let parameterImportDraft;
let parameterImportAction;
let profileTestActive = false;
let profileTestSawRunning = false;
let profileTestSamples = [];
let profileTestStartMs = 0;
let pendingGains;
let testedGains;
let gainDraftDirty = false;
let pendingGainSavePayload;
let tuningAction;
let tuningTestActive = false;
let tuningTestSawRunning = false;
let tuningSamples = [];
let tuningTestMode = "profile";
let tuningTargetVelocity = 0;
let stepEstimate;
let latestState = 0;
let latestFaults = 0;
let latestCurrentA = 0;
let latestEncoderCount;
let rotorVisualState;
let encoderCalibrationStartCount;
let encoderCalibrationTurns;
let encoderCalibrationCandidate;
let encoderCalibrationSavePending = false;
let motorTestActive = false;
let motorTestTimer;
let motorTestAction;
let characterizationAction;
let characterizationRunning = false;
let characterizationSamples = [];
let currentCalibrationPendingAction;
let currentCalibrationCapturedMask = 0;
let currentCalibrationLastResult = 0;
let currentCalibrationDriveActive = false;
let currentCalibrationDriveArmPending = false;
let currentCalibrationDriveTimer;
let runtimeProfileId;
let overviewRunAction;
let defaultProfilePending;
let loadConfiguration = [];
let loadConfigurationSaved = [];
let editingLoadSlot;
let loadSavePending = false;
let renderedMachineStatusKey;
let recordedLoadConfiguration = [];
let recordedLoadSettingId = 0;
const decoder = new TextDecoder();
const encoder = new TextEncoder();
const $ = id => document.getElementById(id);

function localPreferenceStorage() {
  try { return globalThis.localStorage; } catch { return undefined; }
}

function crc16(bytes) {
  let crc = 0xffff;
  for (const value of bytes) {
    crc ^= value << 8;
    for (let bit = 0; bit < 8; bit++) crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) & 0xffff : (crc << 1) & 0xffff;
  }
  return crc;
}

function frame(msgId, payload = new Uint8Array()) {
  const bytes = new Uint8Array(12 + payload.length);
  const view = new DataView(bytes.buffer);
  bytes[0] = SYNC_1; bytes[1] = SYNC_2; bytes[2] = VERSION; bytes[3] = 0;
  view.setUint16(4, msgId, true); view.setUint16(6, sequence++, true); view.setUint16(8, payload.length, true);
  bytes.set(payload, 10);
  view.setUint16(10 + payload.length, crc16(bytes.slice(2, 10 + payload.length)), true);
  return bytes;
}

async function sendFrame(msgId, payload) {
  if (!writer) return;
  const bytes = frame(msgId, payload);
  await writer.write(bytes);
  communicationMetrics.recordTxBytes(bytes.byteLength);
  communicationMetrics.recordTxMessage();
}

async function sendAscii(command) {
  if (!writer) return;
  const bytes = encoder.encode(`${command}\n`);
  await writer.write(bytes);
  communicationMetrics.recordTxBytes(bytes.byteLength);
  communicationMetrics.recordTxMessage();
}

async function requestDeviceSynchronization() {
  const now = performance.now();
  if (!connected || !writer || !deviceSynchronizer.shouldRequest(now)) return;
  deviceSynchronizer.markRequested(now);
  await sendFrame(MSG.GET_SETTINGS);
  await sendFrame(MSG.START_STREAM);
}

async function connect() {
  if (connected) return disconnect();
  if (!("serial" in navigator)) return toast("Web Serial requires Chrome/Edge on HTTPS or localhost.");
  try {
    port = await navigator.serial.requestPort();
    await port.open({ baudRate: Number($("baud").value), bufferSize: 4096 });
    writer = port.writable.getWriter();
    profiles = [];
    deviceSynchronizer.reset();
    connected = true;
    setConnected(true);
    readLoop();
    await requestDeviceSynchronization();
    toast("Connected. Waiting for firmware synchronization; motor remains disarmed.");
  } catch (error) { toast(`Connection failed: ${error.message}`); await disconnect(); }
}

async function disconnect() {
  if (writer) await stopAllManualOutputs(true);
  connected = false;
  try { await reader?.cancel(); } catch {}
  try { reader?.releaseLock(); } catch {}
  try { writer?.releaseLock(); } catch {}
  reader = undefined; writer = undefined;
  try { await port?.close(); } catch {}
  port = undefined; setConnected(false);
}

async function readLoop() {
  reader = port.readable.getReader();
  try {
    while (connected) {
      const { value, done } = await reader.read();
      if (done) break;
      communicationMetrics.recordRxBytes(value.byteLength);
      for (const byte of value) rx.push(byte);
      parseRx();
    }
  } catch (error) { if (connected) toast(`Serial read failed: ${error.message}`); }
  finally { try { reader.releaseLock(); } catch {} }
}

function parseRx() {
  while (rx.length) {
    const sync = rx.findIndex((value, index) => value === SYNC_1 && rx[index + 1] === SYNC_2);
    if (sync < 0) {
      const preserve = rx.at(-1) === SYNC_1 ? 1 : 0;
      const ascii = rx.splice(0, rx.length - preserve);
      appendTerminal(decoder.decode(new Uint8Array(ascii), { stream: true }));
      return;
    }
    if (sync > 0) appendTerminal(decoder.decode(new Uint8Array(rx.splice(0, sync)), { stream: true }));
    if (rx.length < 12) return;
    const header = new DataView(new Uint8Array(rx.slice(0, 10)).buffer);
    const length = header.getUint16(8, true);
    const total = 12 + length;
    if (length > 512) { communicationMetrics.recordFramingError(); rx.shift(); continue; }
    if (rx.length < total) return;
    const packet = new Uint8Array(rx.splice(0, total));
    const packetView = new DataView(packet.buffer);
    const expected = packetView.getUint16(10 + length, true);
    if (crc16(packet.slice(2, 10 + length)) !== expected) { communicationMetrics.recordCrcError(); continue; }
    communicationMetrics.recordRxFrame();
    handleMessage(packetView.getUint16(4, true), new DataView(packet.buffer, 10, length));
  }
}

function handleMessage(id, data) {
  if (id === MSG.HEARTBEAT) {
    const state = data.getUint8(16); latestState = state;
    updateState(state, data.getUint32(12, true));
    requestDeviceSynchronization().catch(() => {});
  } else if (id === MSG.SETTINGS) {
    settings = {
      schema: data.getUint32(0, true), baud: data.getUint32(4, true), periodUs: data.getUint32(8, true), cpr: data.getUint32(12, true),
      streamRate: data.getUint16(16, true), profileId: data.getUint16(18, true), kp: data.getFloat32(20, true), ki: data.getFloat32(24, true), kd: data.getFloat32(28, true),
      vmax: data.getFloat32(32, true), amax: data.getFloat32(36, true), jmax: data.getFloat32(40, true), imax: data.getFloat32(44, true), maxDuty: data.getFloat32(48, true),
      deadbandFwd: data.getFloat32(52, true), deadbandRev: data.getFloat32(56, true), loadSetting: data.getUint8(60), loadCount: data.getUint8(61), direction: data.getInt8(62), stopMode: data.getUint8(63),
      vinGain: data.getFloat32(64, true), vinOffset: data.getFloat32(68, true), vinMin: data.getFloat32(72, true), vinMax: data.getFloat32(76, true), vinPin: data.getUint8(80),
      currentGain: data.getFloat32(81, true), currentOffset: data.getFloat32(85, true), currentPin: data.getUint8(89), currentSenseEnabled: data.getUint8(90) !== 0, diagEnabled: data.getUint8(91) !== 0, diagPin: data.getUint8(92),
      encoderTimeoutMs: data.byteLength >= 101 ? data.getUint32(93, true) : 250,
      encoderTimeoutVelocity: data.byteLength >= 101 ? data.getFloat32(97, true) : 1,
      feedbackLimit: data.byteLength >= 114 ? data.getFloat32(101, true) : 0.1,
      estimatorMinCounts: data.byteLength >= 114 ? data.getUint8(105) : 4,
      estimatorMaxWindowUs: data.byteLength >= 114 ? data.getUint32(106, true) : 20000,
      estimatorStaleTimeoutUs: data.byteLength >= 114 ? data.getUint32(110, true) : 100000,
      currentFilterCutoffHz: data.byteLength >= 118 ? data.getFloat32(114, true) : 20,
      zeroIndexMinIntervalUs: data.byteLength >= 122 ? data.getUint32(118, true) : 5000,
      zeroIndexCorrectionGain: data.byteLength >= 126 ? data.getFloat32(122, true) : 0.1,
      encoderDirection: data.byteLength >= 127 ? data.getInt8(126) : 1,
      zeroIndexMinSeparationRevolutions: data.byteLength >= 131
          ? data.getFloat32(127, true)
          : 0.5,
      characterizationDynamicsCutoffHz: data.byteLength >= 135 ? data.getFloat32(131, true) : 20,
      characterizationDynamicsQuantile: data.byteLength >= 139 ? data.getFloat32(135, true) : 0.95,
      characterizationSafetyFactor: data.byteLength >= 143 ? data.getFloat32(139, true) : 0.70,
      motorModelGainForward: data.byteLength >= 147 ? data.getFloat32(143, true) : 0,
      motorModelGainReverse: data.byteLength >= 151 ? data.getFloat32(147, true) : 0,
      motorModelTimeConstantForward: data.byteLength >= 155 ? data.getFloat32(151, true) : 0,
      motorModelTimeConstantReverse: data.byteLength >= 159 ? data.getFloat32(155, true) : 0,
      motorModelVelocityNoise: data.byteLength >= 163 ? data.getFloat32(159, true) : 25,
      motorModelDisturbanceNoise: data.byteLength >= 167 ? data.getFloat32(163, true) : 100,
      motorModelEncoderNoiseCounts: data.byteLength >= 171 ? data.getFloat32(167, true) : 0.5,
      velocityEstimatorMethod: data.byteLength >= 172 ? data.getUint8(171) : 0,
      velocityAccelerationWindowSamples: data.byteLength >= 173 ? data.getUint8(172) : 5
    };
    parameterDefinitions.streamRate.max = maximumTelemetryStreamRateHz(settings.baud);
    if (runtimeProfileId === undefined) runtimeProfileId = settings.profileId;
    rotorVisualState = undefined;
    renderSettings();
    renderEncoderCalibration();
    renderProfiles();
    renderRunProfileDialog();
    sendCurrentCalibrationCommand(CURRENT_CALIBRATION_ACTION.REQUEST_STATUS);
  } else if (id === MSG.PROFILE_CONFIGURATION) {
    const profile = decodeProfile(data);
    const existing = profiles.findIndex(item => item.id === profile.id);
    if (existing >= 0) profiles[existing] = profile; else profiles.push(profile);
    profiles.sort((a, b) => a.id - b.id);
    renderProfiles();
    renderRunProfileDialog();
  } else if (id === MSG.LOAD_CONFIGURATION) {
    const count = Math.min(ROTOR_SLOT_COUNT, data.getUint8(1));
    const decoded = Array.from({ length: count }, (_, index) => ({
      slot: data.getUint8(2 + index * 7),
      position: data.getUint16(3 + index * 7, true),
      strength: Math.round(data.getFloat32(5 + index * 7, true))
    }));
    try {
      const synchronized = synchronizeRotorLoadDraft(
        loadConfiguration, loadConfigurationSaved, decoded);
      loadConfiguration = synchronized.draft;
      loadConfigurationSaved = synchronized.saved;
      renderRotorLoadSetup();
    } catch (error) {
      toast(`Firmware load configuration is invalid: ${error.message}`);
    }
  } else if (id === MSG.TELEMETRY) {
    deviceSynchronizer.markTelemetryReceived();
    const sample = { timestamp: Number(data.getBigUint64(0, true)), zeroTime: Number(data.getBigUint64(8, true)), count: data.getBigInt64(16, true), zeroCount: data.getBigInt64(24, true), desired: data.getFloat32(32, true), measured: data.getFloat32(36, true), output: data.getFloat32(40, true), current: data.getFloat32(44, true), supplyVoltage: data.getFloat32(48, true), faults: data.getUint32(52, true), profile: data.getUint16(56, true), load: data.getUint8(58), state: data.getUint8(59), pTerm: data.byteLength >= 72 ? data.getFloat32(60, true) : 0, iTerm: data.byteLength >= 72 ? data.getFloat32(64, true) : 0, dTerm: data.byteLength >= 72 ? data.getFloat32(68, true) : 0, rotorPosition: data.byteLength >= 76 ? data.getFloat32(72, true) : Number.NaN, zeroSequence: data.byteLength >= 80 ? data.getUint32(76, true) : 0, zeroRejected: data.byteLength >= 84 ? data.getUint32(80, true) : 0 };
    communicationMetrics.recordTelemetry(sample.timestamp, settings.streamRate);
    const recordedSampleCount = runRecorder.samples.length;
    runRecorder.consume(sample);
    if (runRecorder.samples.length !== recordedSampleCount) updateExportButton();
    samples.push(sample); if (samples.length > 12000) samples.shift(); renderTelemetry(sample);
    renderProfileTestTelemetry(sample);
    renderTuningTelemetry(sample);
    if (characterizationRunning) {
      characterizationSamples.push({ timestamp: sample.timestamp, measured: sample.measured });
      if (characterizationSamples.length > 4000) characterizationSamples.shift();
    }
  } else if (id === MSG.CHARACTERIZATION_RESULT) {
    const result = {
      startForward: data.getFloat32(0, true), startReverse: data.getFloat32(4, true),
      maxForward: data.getFloat32(8, true), maxReverse: data.getFloat32(12, true),
      accelerationForward: data.byteLength >= 32 ? data.getFloat32(16, true) : Number.NaN,
      accelerationReverse: data.byteLength >= 32 ? data.getFloat32(20, true) : Number.NaN,
      jerkForward: data.byteLength >= 32 ? data.getFloat32(24, true) : Number.NaN,
      jerkReverse: data.byteLength >= 32 ? data.getFloat32(28, true) : Number.NaN,
      modelGainForward: data.byteLength >= 48 ? data.getFloat32(32, true) : Number.NaN,
      modelGainReverse: data.byteLength >= 48 ? data.getFloat32(36, true) : Number.NaN,
      modelTimeConstantForward: data.byteLength >= 48 ? data.getFloat32(40, true) : Number.NaN,
      modelTimeConstantReverse: data.byteLength >= 48 ? data.getFloat32(44, true) : Number.NaN
    };
    renderCharacterizationResult(result);
  } else if (id === MSG.CHARACTERIZATION_STATUS) {
    renderCharacterizationStatus({ stage: data.getUint8(0), resultPending: data.getUint8(1) !== 0, duty: data.getFloat32(2, true), velocity: data.getFloat32(6, true) });
  } else if (id === MSG.CURRENT_CALIBRATION_STATUS) {
    renderCurrentCalibrationStatus({
      capturedMask: data.getUint8(0), capturePoint: data.getUint8(1), lastResult: data.getUint8(2),
      point1Voltage: data.getFloat32(3, true), point1Current: data.getFloat32(7, true),
      point2Voltage: data.getFloat32(11, true), point2Current: data.getFloat32(15, true),
      gain: data.getFloat32(19, true), offset: data.getFloat32(23, true)
    });
  } else if (id === MSG.ACK) {
    const request = data.getUint16(0, true);
    const result = data.getUint8(2);
    if (result === 0 && (request === MSG.START_RUN || request === MSG.START_VELOCITY_TEST)) {
      runRecorder.begin();
      recordedLoadConfiguration = loadConfigurationSaved.map(load => ({ ...load }));
      recordedLoadSettingId = Number(settings.loadSetting) || 0;
      updateExportButton();
    }
    if (request === MSG.ARM && overviewRunAction?.stage === "arm") {
      overviewRunAction = undefined;
      if (result === 0) {
        latestState = 1;
        toast("Motor output armed. Select a profile and press Run when the rig is safe.");
      } else toast(`Could not arm: ${resultDescription(result)}.`);
      renderRunProfileDialog();
      return;
    }
    if (request === MSG.SELECT_PROFILE && overviewRunAction) {
      const action = overviewRunAction;
      if (result !== 0) {
        overviewRunAction = undefined;
        toast(`Could not select profile: ${resultDescription(result)}.`);
      } else {
        runtimeProfileId = action.profileId;
        if (action.stage === "run-select") {
          action.stage = "run-start";
          $("runProfileStatus").textContent = "Profile selected for this run. Starting…";
          sendFrame(MSG.START_RUN);
        } else {
          overviewRunAction = undefined;
          toast("Profile selected for the next run; saved default unchanged.");
        }
      }
      renderProfiles();
      return;
    }
    if (request === MSG.START_RUN && overviewRunAction?.stage === "run-start") {
      overviewRunAction = undefined;
      if (result === 0) {
        $("runProfileDialog").close("run");
        toast("Profile run started.");
      } else {
        $("runProfileStatus").textContent = `Run rejected: ${resultDescription(result)}.`;
        toast(`Run rejected: ${resultDescription(result)}.`);
      }
      renderRunProfileDialog();
      return;
    }
    if (request === MSG.SET_DEFAULT_PROFILE && defaultProfilePending !== undefined) {
      const profileId = defaultProfilePending;
      defaultProfilePending = undefined;
      if (result === 0) {
        settings.profileId = profileId;
        runtimeProfileId = profileId;
        toast("Default profile saved to this device.");
      } else toast(`Default profile was not saved: ${resultDescription(result)}.`);
      renderProfiles();
      return;
    }
    if (request === MSG.SET_LOAD_CONFIGURATION && loadSavePending) {
      loadSavePending = false;
      if (result === 0) {
        loadConfigurationSaved = loadConfiguration.map(load => ({ ...load }));
        toast("Rotor load change saved.");
      } else {
        loadConfiguration = loadConfigurationSaved.map(load => ({ ...load }));
        toast(`Load change was rejected and rolled back: ${resultDescription(result)}.`);
      }
      renderRotorLoadSetup();
      return;
    }
    if (request === MSG.CURRENT_CALIBRATION) {
      const action = currentCalibrationPendingAction;
      currentCalibrationPendingAction = undefined;
      if (result !== 0) {
        setCurrentCalibrationBusy(false);
        toast(action === CURRENT_CALIBRATION_ACTION.CAPTURE_POINT_2
          ? "Point 2 must have at least 0.01 A and 0.001 V more span than Point 1."
          : `Current calibration rejected: ${resultDescription(result)}.`);
      } else if (action === CURRENT_CALIBRATION_ACTION.SAVE) {
        setCurrentCalibrationDriveActive(false);
        toast("Two-point current calibration applied and saved; motor disarmed.");
      } else if (action === CURRENT_CALIBRATION_ACTION.CANCEL) {
        stopCurrentCalibrationDrive(true);
        toast("Current calibration cancelled; saved values were not changed.");
      }
      return;
    }
    if (request === MSG.ARM && currentCalibrationDriveArmPending) {
      currentCalibrationDriveArmPending = false;
      if (result === 0) {
        setCurrentCalibrationDriveActive(true);
        sendCurrentCalibrationDriveDuty().catch(() => stopCurrentCalibrationDrive(false));
        currentCalibrationDriveTimer = setInterval(() => {
          if (currentCalibrationDriveActive) sendCurrentCalibrationDriveDuty()
              .catch(() => stopCurrentCalibrationDrive(false));
        }, 250);
        toast("Calibration motor running at raw PWM.");
      } else {
        setCurrentCalibrationDriveActive(false);
        toast(`Could not arm calibration drive: ${resultDescription(result)}.`);
      }
      return;
    }
    if (request === MSG.ARM && motorTestAction?.stage === "arm") {
      if (result === 0) {
        motorTestAction.stage = "start";
        $("motorTestState").textContent = "● Armed; applying raw output…";
        sendMotorTestDuty(motorTestAction.duty).catch(error => {
          motorTestAction = undefined;
          setMotorTestActive(false);
          toast(`Could not send raw motor command: ${error.message}`);
        });
      } else {
        motorTestAction = undefined;
        setMotorTestActive(false);
        toast(`Could not arm motor test: ${resultDescription(result)}.`);
      }
      return;
    }
    if (request === MSG.MOTOR_TEST) {
      if (motorTestAction?.stage === "start") {
        if (result === 0) {
          motorTestAction = undefined;
          setMotorTestActive(true);
          motorTestTimer = setInterval(() => {
            if (motorTestActive) sendMotorTestDuty(signedMotorTestDuty())
                .catch(() => stopMotorTest(false));
          }, 250);
          toast("Raw motor output started.");
        } else {
          motorTestAction = undefined;
          setMotorTestActive(false);
          toast(`Raw motor command rejected: ${resultDescription(result)}.`);
        }
        return;
      }
      if (result !== 0) {
        setMotorTestActive(false);
        setCurrentCalibrationDriveActive(false);
        toast(`Raw motor command rejected: ${resultDescription(result)}.`);
      }
      return;
    }
    if (request === MSG.CLEAR_FAULTS) {
      toast(result === 0 ? "Faults cleared. Safety inputs rechecked; machine remains disarmed." : "An active fault remains. Restore VIN and check enabled safety inputs.");
      return;
    }
    if (request === MSG.STOP_RUN && pendingGainSavePayload) {
      if (result === 0) {
        const payload = pendingGainSavePayload;
        pendingGainSavePayload = undefined;
        sendFrame(MSG.SAVE_CONTROLLER, payload);
      } else {
        pendingGainSavePayload = undefined;
        toast(`Could not disarm before saving gains: ${resultDescription(result)}.`);
      }
      return;
    }
    if (request === MSG.STOP_RUN && tuningAction?.stage === "stop") {
      if (result === 0) {
        latestState = 0;
        if (tuningAction.mode === "profile") {
          tuningAction.stage = "select";
          const payload = new Uint8Array(2);
          new DataView(payload.buffer).setUint16(0, tuningAction.profileId, true);
          sendFrame(MSG.SELECT_PROFILE, payload);
        } else {
          tuningAction.stage = "arm";
          sendFrame(MSG.ARM);
        }
        setTuningTestStatus("● Preparing", "Motor disarmed; preparing test input.", "offline");
      } else {
        failTuningAction(`Could not disarm: ${resultDescription(result)}.`);
      }
      return;
    }
    if (request === MSG.STOP_RUN && profileAction?.stage === "stop") {
      if (result === 0) {
        latestState = 0;
        profileAction.stage = "upload";
        if (profileAction.type === "run") $("profileRunCommandStatus").textContent = "Motor disarmed. Validating profile with firmware…";
        sendFrame(profileAction.createOnly ? MSG.CREATE_PROFILE : MSG.SET_PROFILE,
                  encodeProfile(profileAction.type === "save"));
      } else {
        finishProfileAction();
        $("saveProfile").disabled = false; $("runProfileTest").disabled = false;
        renderProfiles();
        toast(`Could not disarm before profile update: ${resultDescription(result)}.`);
      }
      return;
    }
    if (request === MSG.STOP_RUN && parameterImportAction?.stage === "stop") {
      if (result === 0) {
        latestState = 0;
        parameterImportAction.stage = "apply";
        sendNextParameterImport();
      } else {
        failParameterImport(`Could not disarm before import: ${resultDescription(result)}.`);
      }
      return;
    }
    if (request === MSG.STOP_RUN && pendingParameterPayload) {
      if (result === 0) {
        latestState = 0;
        const payload = pendingParameterPayload;
        pendingParameterPayload = undefined;
        sendFrame(MSG.SET_PARAMETER, payload);
      } else {
        pendingParameterPayload = undefined;
        toast(`Could not disarm before parameter update: ${resultDescription(result)}.`);
      }
      return;
    }
    if (request === MSG.STOP_RUN && tuningTestActive) {
      tuningTestActive = false;
      $("stopTuningTest").disabled = true;
      setTuningTestStatus(result === 0 ? "■ Stopped" : "▲ Stop failed",
                          result === 0 ? "Test stopped and motor disarmed." : resultDescription(result),
                          result === 0 ? "offline" : "fault");
      finalizeTuningResponse();
      return;
    }
    if (request === MSG.STOP_RUN && profileTestActive) {
      profileTestActive = false;
      setProfileTestStatus(result === 0 ? "■ Stopped" : "▲ Stop failed",
                           result === 0 ? "Profile test stopped and motor disarmed." : resultDescription(result),
                           result === 0 ? "offline" : "fault");
      $("stopProfileTest").disabled = true;
      return;
    }
    if (request === MSG.CHARACTERIZATION_ACTION) {
      const completedAction = characterizationAction;
      characterizationAction = undefined;
      $("saveCharacterization").disabled = false;
      $("discardCharacterization").disabled = false;
      if (result === 0) {
        if ($("characterizationResultDialog").open) $("characterizationResultDialog").close();
        $("characterizationProgress").classList.add("hidden");
        toast(completedAction === "save" ? "Characterization applied and saved." : "Characterization result discarded.");
      } else {
        toast(`Could not ${completedAction === "save" ? "save" : "discard"} result (code ${result}).`);
      }
      return;
    }
    if (request === MSG.SET_PARAMETER) {
      if (parameterImportAction) {
        const entry = parameterImportAction.entries[parameterImportAction.index];
        if (result !== 0) {
          failParameterImport(`Import stopped at "${entry.parameter}" after ${parameterImportAction.index} saved values: ${resultDescription(result)}.`);
          return;
        }
        ++parameterImportAction.index;
        $("parameterImportProgress").textContent =
            `${parameterImportAction.index} of ${parameterImportAction.entries.length} values applied and saved.`;
        if (parameterImportAction.index >= parameterImportAction.entries.length) {
          parameterImportAction = undefined;
          parameterImportDraft = undefined;
          $("applyParameterImport").disabled = false;
          $("cancelParameterImport").disabled = false;
          $("parameterImportDialog").close("apply");
          toast("Parameter CSV imported and saved.");
          sendFrame(MSG.GET_SETTINGS);
        } else {
          sendNextParameterImport();
        }
        return;
      }
      pendingParameterPayload = undefined;
      if (encoderCalibrationSavePending) {
        encoderCalibrationSavePending = false;
        if (result === 0) {
          encoderCalibrationStartCount = undefined;
          encoderCalibrationTurns = undefined;
          encoderCalibrationCandidate = undefined;
          $("encoderCalibrationResult").classList.add("hidden");
          setEncoderCalibrationState("✓ Saved", "online");
          toast("Encoder counts per output revolution applied and saved.");
        } else {
          setEncoderCalibrationState("▲ Save rejected", "fault");
          toast(`Encoder calibration was not saved: ${resultDescription(result)}.`);
        }
        renderEncoderCalibration();
        return;
      }
      if (result === 0) {
        if ($("parameterEditorDialog").open) $("parameterEditorDialog").close("save");
        toast("Parameter applied and saved.");
      } else {
        toast(`Parameter rejected: ${resultDescription(result)}.`);
      }
      return;
    }
    if (request === MSG.SET_CONTROLLER) {
      $("loadGains").disabled = !connected;
      if (result === 0 && pendingGains) {
        testedGains = pendingGains;
        pendingGains = undefined;
        gainDraftDirty = false;
        setGainState("● Applied for test", "online");
        $("saveGains").disabled = !connected;
        toast("Gains applied to RAM. They are not saved yet.");
      } else {
        pendingGains = undefined;
        toast(`Could not apply gains: ${resultDescription(result)}.`);
      }
      return;
    }
    if (request === MSG.SAVE_CONTROLLER) {
      if (result === 0) {
        testedGains = undefined;
        gainDraftDirty = false;
        setGainState("✓ Saved", "online");
        $("saveGains").disabled = true;
        toast("Tested gains saved to device Preferences.");
      } else {
        $("saveGains").disabled = !connected || !testedGains;
        toast(`Could not save gains: ${resultDescription(result)}.`);
      }
      return;
    }
    if (request === MSG.SET_PROFILE || request === MSG.CREATE_PROFILE) {
      const action = profileAction;
      if (result !== 0) {
        finishProfileAction();
        $("saveProfile").disabled = false; $("runProfileTest").disabled = false;
        renderProfiles();
        const reason = request === MSG.CREATE_PROFILE && result === 1
          ? "This firmware does not support creating profiles; upload the current firmware build"
          : resultDescription(result);
        $("profileRunCommandStatus").textContent = `Profile rejected: ${reason}.`;
        toast(`Profile rejected: ${reason}.`);
        return;
      }
      if (action?.type === "run") {
        action.stage = "select";
        $("profileRunCommandStatus").textContent = "Profile accepted. Selecting it for the test…";
        const payload = new Uint8Array(2); new DataView(payload.buffer).setUint16(0, action.profileId, true);
        sendFrame(MSG.SELECT_PROFILE, payload);
      } else {
        finishProfileAction();
        $("profileEditorDialog").close("save");
        $("saveProfile").disabled = false; $("runProfileTest").disabled = false;
        renderProfiles();
        toast("Profile saved to this device.");
      }
      return;
    }
    if (request === MSG.SELECT_PROFILE && profileAction?.stage === "select") {
      if (result === 0) {
        runtimeProfileId = profileAction.profileId;
        profileAction.stage = "arm";
        $("profileRunCommandStatus").textContent = "Profile selected. Confirming firmware arm state…";
        sendFrame(MSG.ARM);
        renderProfiles();
      } else {
        finishProfileAction();
        $("saveProfile").disabled = false; $("runProfileTest").disabled = false;
        renderProfiles();
        $("profileRunCommandStatus").textContent = `Could not select profile: ${resultDescription(result)}.`;
        toast(`Could not select profile: ${resultDescription(result)}.`);
      }
      return;
    }
    if (request === MSG.SELECT_PROFILE && tuningAction?.stage === "select") {
      if (result === 0) {
        runtimeProfileId = tuningAction.profileId;
        renderProfiles();
        tuningAction.stage = "arm";
        sendFrame(MSG.ARM);
      } else failTuningAction(`Could not select profile: ${resultDescription(result)}.`);
      return;
    }
    if (request === MSG.ARM && profileAction?.stage === "arm") {
      if (result === 0) {
        profileAction.stage = "start";
        $("profileRunCommandStatus").textContent = "Motor armed. Waiting for firmware run confirmation…";
        sendFrame(MSG.START_RUN);
      } else {
        finishProfileAction();
        $("saveProfile").disabled = false; $("runProfileTest").disabled = false;
        renderProfiles();
        $("profileRunCommandStatus").textContent = `Could not arm: ${resultDescription(result)}.`;
        toast(`Could not arm profile test: ${resultDescription(result)}.`);
      }
      return;
    }
    if (request === MSG.ARM && tuningAction?.stage === "arm") {
      if (result === 0) {
        tuningAction.stage = "start";
        if (tuningAction.mode === "profile") {
          sendFrame(MSG.START_RUN);
        } else {
          const payload = new Uint8Array(8), view = new DataView(payload.buffer);
          view.setFloat32(0, tuningAction.velocity, true);
          view.setUint32(4, Math.round(tuningAction.duration * 1000), true);
          sendFrame(MSG.START_VELOCITY_TEST, payload);
        }
        setTuningTestStatus("● Preparing", "Motor armed; waiting for RUNNING confirmation.", "offline");
      } else failTuningAction(`Could not arm: ${resultDescription(result)}.`);
      return;
    }
    if (request === MSG.START_RUN && profileAction?.stage === "start") {
      if (result === 0) {
        finishProfileAction();
        profileTestActive = true; profileTestSawRunning = false; profileTestSamples = [];
        profileTestStartMs = performance.now();
        $("profileEditorDialog").close("run");
        $("saveProfile").disabled = false; $("runProfileTest").disabled = false;
        renderProfiles();
        $("profileRunCommandStatus").classList.add("hidden");
        $("profileTestProgress").classList.remove("hidden");
        $("stopProfileTest").disabled = false;
        setProfileTestStatus("● Running", "Firmware confirmed RUNNING. Waiting for telemetry…", "online");
        $("profileTestProgress").scrollIntoView({ behavior: matchMedia("(prefers-reduced-motion: reduce)").matches ? "auto" : "smooth", block: "start" });
        toast("Firmware confirmed profile test is running.");
      } else {
        finishProfileAction();
        $("saveProfile").disabled = false; $("runProfileTest").disabled = false;
        renderProfiles();
        $("profileRunCommandStatus").textContent = `Run was not started: ${resultDescription(result)}.`;
        setProfileTestStatus("▲ Not started", resultDescription(result), "fault");
        toast(`Profile test did not start: ${resultDescription(result)}.`);
      }
      return;
    }
    if (request === MSG.START_RUN && tuningAction?.stage === "start") {
      confirmTuningRun(result);
      return;
    }
    if (request === MSG.START_VELOCITY_TEST && tuningAction?.stage === "start") {
      confirmTuningRun(result);
      return;
    }
    toast(result === 0 ? "Command acknowledged." : `Command rejected: ${resultDescription(result)}.`);
  }
}

function setConnected(value) {
  communicationMetrics.reset();
  resetCommunicationDisplay(value);
  $("connectionState").textContent = value ? "● Connected" : "● Disconnected";
  $("connectionState").className = `status-badge ${value ? "online" : "offline"}`;
  $("connectButton").textContent = value ? "Disconnect" : "Connect";
  for (const id of ["stopButton", "armButton", "runButton", "loadGains", "saveConfig", "characterizeButton", "openVinCalibration", "saveDriverDiagnostic", "saveCurrentSense", "startMotorTest", "stopMotorTest", "startTuningTest", "sendTerminal"]) $(id).disabled = !value;
  $("exportParameters").disabled = !value || !Number.isFinite(Number(settings.schema));
  $("importParameters").disabled = !value || !Number.isFinite(Number(settings.schema));
  $("saveProfile").disabled = !value;
  $("runProfileTest").disabled = !value;
  renderProfiles();
  if (value && !Number.isFinite(settings.schema)) {
    $("parameterRows").innerHTML = '<tr><td colspan="3">Connected; waiting for a valid SETTINGS frame…</td></tr>';
  }
  setCurrentCalibrationBusy(false);
  setCurrentCalibrationDriveActive(value && currentCalibrationDriveActive);
  if (!value) { $("machineState").textContent = "DISCONNECTED"; samples = []; profiles = []; deviceSynchronizer.reset(); runRecorder.reset(); recordedLoadConfiguration = []; recordedLoadSettingId = 0; renderedMachineStatusKey = undefined; finishProfileAction(); motorTestAction = undefined; overviewRunAction = undefined; defaultProfilePending = undefined; runtimeProfileId = undefined; loadConfiguration = []; loadConfigurationSaved = []; loadSavePending = false; latestEncoderCount = undefined; rotorVisualState = undefined; encoderCalibrationStartCount = undefined; encoderCalibrationTurns = undefined; encoderCalibrationCandidate = undefined; encoderCalibrationSavePending = false; parameterImportDraft = undefined; parameterImportAction = undefined; if ($("parameterImportDialog").open) $("parameterImportDialog").close("disconnect"); $("encoderCalibrationResult").classList.add("hidden"); latestFaults = 0; $("clearFaultButton").disabled = true; characterizationRunning = false; $("abortCharacterization").disabled = true; profileTestActive = false; tuningTestActive = false; $("stopProfileTest").disabled = true; $("stopTuningTest").disabled = true; $("saveGains").disabled = true; setMotorTestActive(false); setCurrentCalibrationDriveActive(false); renderProfiles(); }
  updateExportButton();
  renderRunProfileDialog();
  renderRotorLoadSetup();
  renderEncoderCalibration();
}

function resetCommunicationDisplay(isConnected) {
  const stateText = isConnected ? "● Waiting for data" : "● Offline";
  $("serialLinkState").textContent = stateText;
  $("serialLinkState").className = "status-badge offline";
  $("serialDialogState").textContent = stateText;
  $("serialDialogState").className = "status-badge offline";
  $("serialRxRate").textContent = "0.00";
  $("serialDialogRxRate").textContent = "0.00";
  $("serialRxFrames").textContent = "0.0 valid frames/s";
  $("serialTxRate").textContent = "0.00";
  $("serialDialogTxRate").textContent = "0.00";
  $("serialTxMessages").textContent = "0.0 messages/s";
  $("serialTelemetryRate").textContent = "0.0";
  $("serialDialogTelemetryRate").textContent = "0.0";
  $("serialFrameAge").textContent = "No valid frame received";
  $("serialDropouts").textContent = "0";
  $("serialDialogDropouts").textContent = "0";
  $("serialDropoutPercent").textContent = "0.00% estimated loss";
  $("serialIntegrityErrors").textContent = "0 CRC · 0 framing errors";
}

function renderCommunicationMetrics() {
  if (!connected) return;
  const metrics = communicationMetrics.snapshot();
  const rxRate = (metrics.rxBytesPerSecond / 1000).toFixed(2);
  const txRate = (metrics.txBytesPerSecond / 1000).toFixed(2);
  const telemetryRate = metrics.telemetryHz.toFixed(1);
  $("serialRxRate").textContent = rxRate;
  $("serialDialogRxRate").textContent = rxRate;
  $("serialRxFrames").textContent = `${metrics.rxFramesPerSecond.toFixed(1)} valid frames/s`;
  $("serialTxRate").textContent = txRate;
  $("serialDialogTxRate").textContent = txRate;
  $("serialTxMessages").textContent = `${metrics.txMessagesPerSecond.toFixed(1)} messages/s`;
  $("serialTelemetryRate").textContent = telemetryRate;
  $("serialDialogTelemetryRate").textContent = telemetryRate;
  $("serialFrameAge").textContent = metrics.lastFrameAgeMs === undefined
    ? "No valid frame received"
    : `Last valid frame ${Math.round(metrics.lastFrameAgeMs)} ms ago`;
  $("serialDropouts").textContent = String(metrics.droppedTelemetry);
  $("serialDialogDropouts").textContent = String(metrics.droppedTelemetry);
  $("serialDropoutPercent").textContent = `${metrics.dropoutPercent.toFixed(2)}% estimated loss`;
  $("serialIntegrityErrors").textContent = `${metrics.crcErrors} CRC · ${metrics.framingErrors} framing errors`;

  const stale = metrics.lastFrameAgeMs !== undefined && metrics.lastFrameAgeMs > 3000;
  const stateText = metrics.lastFrameAgeMs === undefined
    ? "● Waiting for data"
    : stale ? "▲ Data stale" : "● Live";
  const stateClass = `status-badge ${stale ? "fault" : metrics.lastFrameAgeMs === undefined ? "offline" : "online"}`;
  $("serialLinkState").textContent = stateText;
  $("serialLinkState").className = stateClass;
  $("serialDialogState").textContent = stateText;
  $("serialDialogState").className = stateClass;
}

function updateState(state, faults) {
  const refreshInteractiveState = shouldRefreshMachineUi(renderedMachineStatusKey, state, faults);
  renderedMachineStatusKey = machineStatusKey(state, faults);
  latestState = state;
  latestFaults = faults;
  if (state !== 0 && (encoderCalibrationStartCount !== undefined || encoderCalibrationCandidate)) {
    resetEncoderCalibration("▲ Measurement cancelled", "Motor must remain disarmed during manual encoder calibration.");
  }
  if (currentCalibrationDriveActive && state !== 1) setCurrentCalibrationDriveActive(false);
  const names = ["DISARMED", "ARMED", "RUNNING", "FAULT"];
  $("machineState").textContent = names[state] ?? `UNKNOWN ${state}`;
  $("faultBanner").classList.toggle("hidden", faults === 0);
  $("clearFaultButton").disabled = !connected || faults === 0;
  const faultNames = [[1, "control overrun"], [2, "driver diagnostic"], [4, "overcurrent"], [8, "encoder timeout"], [16, "invalid configuration"], [32, "VIN undervoltage"], [64, "VIN overvoltage"]];
  const activeFaults = faultNames.filter(([bit]) => faults & bit).map(([, name]) => name);
  $("faultText").textContent = activeFaults.length ? `${activeFaults.join(", ")} (0x${faults.toString(16).padStart(8, "0")})` : "";
  if (refreshInteractiveState) {
    renderProfiles();
    renderRunProfileDialog();
    renderRotorLoadSetup();
  }
}

function updateExportButton() {
  $("exportButton").disabled = !connected || runRecorder.samples.length === 0;
  $("exportButton").title = runRecorder.samples.length
    ? `${runRecorder.samples.length} samples from the latest run`
    : "Run a profile or response test before exporting.";
}

function exportBaseName() {
  return sanitizeExportBaseName($("exportBaseName").value, "mocking-machine-run");
}

function updateExportFileSummary() {
  const baseName = exportBaseName();
  const files = [`${baseName}.csv`];
  if (recordedLoadConfiguration.length) files.push(`${baseName}-loads.csv`);
  $("exportFileSummary").textContent = recordedLoadConfiguration.length
    ? `Two files will be downloaded: ${files.join(" and ")}. Your browser may ask to allow multiple downloads.`
    : `One file will be downloaded: ${files[0]}. No saved rotor loads were active for this run.`;
}

function downloadCsv(fileName, contents) {
  const link = document.createElement("a");
  const url = URL.createObjectURL(new Blob([contents], { type: "text/csv;charset=utf-8" }));
  link.href = url;
  link.download = fileName;
  document.body.append(link);
  link.click();
  link.remove();
  URL.revokeObjectURL(url);
}

function renderTelemetry(sample) {
  latestCurrentA = sample.current;
  latestEncoderCount = sample.count;
  $("desiredVelocity").textContent = sample.desired.toFixed(1);
  $("measuredVelocity").textContent = sample.measured.toFixed(1);
  $("motorCurrent").textContent = sample.current.toFixed(2);
  $("supplyVoltage").textContent = sample.supplyVoltage.toFixed(2);
  $("calibrationSupplyVoltage").textContent = sample.supplyVoltage.toFixed(3);
  $("currentDialogMeasured").textContent = sample.current.toFixed(3);
  const rotorReferenced = sample.zeroSequence > 0 || sample.zeroTime > 0;
  const rotorPosition = rotorReferenced && Number.isFinite(sample.rotorPosition)
                          ? ((sample.rotorPosition % 360) + 360) % 360
                          : Number.NaN;
  $("rotorPosition").textContent = Number.isFinite(rotorPosition)
                                     ? rotorPosition.toFixed(1)
                                     : "—";
  rotorVisualState = Number.isFinite(rotorPosition)
      ? updateRotorVisualState(rotorVisualState, rotorPosition, sample.count,
                               settings.cpr, settings.encoderDirection)
      : undefined;
  $("rotorNeedle").style.transform = `rotate(${rotorVisualState?.unwrappedAngleDeg ?? 0}deg)`;
  $("rotorDial").classList.toggle("unreferenced", !Number.isFinite(rotorPosition));
  $("rotorDial").setAttribute("aria-label", Number.isFinite(rotorPosition)
      ? `Rotor position ${rotorPosition.toFixed(1)} degrees; zero index ${sample.zeroSequence}; ${sample.zeroRejected} rejected bounce edges`
      : "Waiting for the first zero-index detection");
  $("zeroIndexStatus").textContent = rotorReferenced
      ? `Accepted zero #${sample.zeroSequence} · ${sample.zeroRejected} rejected edges`
      : "Waiting for zero index";
  renderEncoderCalibration();
  updateState(sample.state, sample.faults);
}

function setProfileTestStatus(label, detail, appearance) {
  $("profileTestState").textContent = label;
  $("profileTestState").className = `status-badge ${appearance}`;
  $("profileTestStage").textContent = detail;
}

function renderProfileTestTelemetry(sample) {
  if (!profileTestActive) return;
  $("profileTestDesired").textContent = sample.desired.toFixed(2);
  $("profileTestMeasured").textContent = sample.measured.toFixed(2);
  $("profileTestOutput").textContent = (sample.output * 100).toFixed(1);
  if (sample.state === 2) {
    profileTestSawRunning = true;
    profileTestSamples.push({ desired: sample.desired, measured: sample.measured });
    if (profileTestSamples.length > 2000) profileTestSamples.shift();
    setProfileTestStatus("● Running", `Executing profile ${sample.profile}.`, "online");
  } else if (profileTestSawRunning) {
    profileTestActive = false;
    $("stopProfileTest").disabled = true;
    setProfileTestStatus(sample.state === 3 ? "▲ Fault" : "✓ Complete",
                         sample.state === 3 ? "Profile test stopped by a firmware fault." : "Profile duration complete; motor stopped and disarmed.",
                         sample.state === 3 ? "fault" : "online");
  } else if (performance.now() - profileTestStartMs > 750) {
    profileTestActive = false;
    $("stopProfileTest").disabled = true;
    setProfileTestStatus(sample.state === 3 ? "▲ Fault" : "▲ Ended early",
                         sample.state === 3 ? "Firmware entered a fault before motion began." : "Firmware left RUNNING before producing telemetry. Upload the latest firmware and retry.",
                         "fault");
  }
}

function setGainState(label, appearance) {
  $("gainState").textContent = label;
  $("gainState").className = `status-badge ${appearance}`;
}

function readGainInputs() {
  const gains = { kp: Number($("kp").value), ki: Number($("ki").value), kd: Number($("kd").value) };
  return Object.values(gains).every(value => Number.isFinite(value) && value >= 0) ? gains : null;
}

function encodeGains(gains) {
  const payload = new Uint8Array(12), view = new DataView(payload.buffer);
  view.setFloat32(0, gains.kp, true); view.setFloat32(4, gains.ki, true); view.setFloat32(8, gains.kd, true);
  return payload;
}

function setTuningTestStatus(label, detail, appearance) {
  $("tuningTestState").textContent = label;
  $("tuningTestState").className = `status-badge ${appearance}`;
  $("tuningResultSummary").textContent = detail;
}

function failTuningAction(message) {
  tuningAction = undefined;
  $("startTuningTest").disabled = !connected;
  $("stopTuningTest").disabled = true;
  $("loadGains").disabled = !connected;
  $("saveGains").disabled = !connected || !testedGains;
  setTuningTestStatus("▲ Not started", message, "fault");
  toast(message);
}

function confirmTuningRun(result) {
  if (result !== 0) return failTuningAction(`Test did not start: ${resultDescription(result)}.`);
  tuningTestMode = tuningAction.mode;
  tuningTargetVelocity = tuningAction.mode === "manual" ? tuningAction.velocity : 0;
  tuningAction = undefined;
  tuningTestActive = true; tuningTestSawRunning = false; tuningSamples = [];
  clearStepEstimate("Test running; the estimate will be calculated after capture completes.");
  $("startTuningTest").disabled = true;
  $("stopTuningTest").disabled = false;
  $("loadGains").disabled = true;
  $("saveGains").disabled = true;
  setTuningTestStatus("● Running", "Firmware confirmed RUNNING; capturing response telemetry.", "online");
  toast("Response test running.");
}

function renderTuningTelemetry(sample) {
  if (!tuningTestActive) return;
  if (sample.state === 2) {
    tuningTestSawRunning = true;
    tuningSamples.push({ timestamp: sample.timestamp, desired: sample.desired, measured: sample.measured, output: sample.output, pTerm: sample.pTerm, iTerm: sample.iTerm, dTerm: sample.dTerm });
    if (tuningSamples.length > 12000) tuningSamples.shift();
    setTuningTestStatus("● Running", `Capturing ${tuningTestMode === "manual" ? "manual step" : "profile"} response…`, "online");
  } else if (tuningTestSawRunning) {
    tuningTestActive = false;
    $("startTuningTest").disabled = !connected;
    $("stopTuningTest").disabled = true;
    $("loadGains").disabled = !connected;
    $("saveGains").disabled = !connected || !testedGains;
    finalizeTuningResponse(sample.state === 3);
  }
}

function finalizeTuningResponse(faulted = false) {
  $("startTuningTest").disabled = !connected;
  $("stopTuningTest").disabled = true;
  $("loadGains").disabled = !connected;
  $("saveGains").disabled = !connected || !testedGains;
  if (!tuningSamples.length) {
    clearStepEstimate("No response samples were captured.", true);
    setTuningTestStatus(faulted ? "▲ Fault" : "■ Stopped", "No response samples were captured.", faulted ? "fault" : "offline");
    return;
  }
  updateStepEstimate();
  const peak = Math.max(...tuningSamples.map(sample => sample.measured));
  $("tuningPeakVelocity").textContent = formatNumber(peak, 2);
  if (tuningTestMode !== "manual" || !(tuningTargetVelocity > 0)) {
    $("tuningOvershoot").textContent = "—"; $("tuningRiseTime").textContent = "—"; $("tuningSettlingTime").textContent = "—";
    setTuningTestStatus(faulted ? "▲ Fault" : "✓ Complete", faulted ? "Profile response ended with a firmware fault." : "Profile response captured. Step metrics apply only to manual tests.", faulted ? "fault" : "online");
    return;
  }
  const metrics = calculateStepMetrics(tuningSamples, tuningTargetVelocity);
  $("tuningOvershoot").textContent = formatNumber(metrics.overshootPercent, 1);
  $("tuningRiseTime").textContent = metrics.riseTimeSeconds == null ? "—" : formatNumber(metrics.riseTimeSeconds, 3);
  $("tuningSettlingTime").textContent = metrics.settlingTimeSeconds == null ? "—" : formatNumber(metrics.settlingTimeSeconds, 3);
  setTuningTestStatus(faulted ? "▲ Fault" : "✓ Complete", faulted ? "Step response captured until the firmware fault." : "Step response captured; review metrics before saving gains.", faulted ? "fault" : "online");
}

function clearStepEstimate(message = "Run a response test to calculate an estimate.", failed = false) {
  stepEstimate = undefined;
  $("stepEstimateState").textContent = failed ? "▲ No estimate" : "● No estimate";
  $("stepEstimateState").className = `status-badge ${failed ? "fault" : "offline"}`;
  $("stepEstimateRate").textContent = "—";
  $("stepEstimateWindow").textContent = "—";
  $("stepEstimateWindows").textContent = "—";
  $("stepEstimateMessage").textContent = message;
}

function updateStepEstimate() {
  if (tuningSamples.length < 2) return clearStepEstimate();
  const result = estimateClosedLoopStepResponse(tuningSamples, {
    windowSize: Number($("stepFftWindow").value),
    responseDurationSeconds: Number($("stepResponseDuration").value),
    regularization: Number($("stepRegularization").value),
    cutoffHz: Number($("stepCutoffHz").value),
    minimumInputAmplitude: Number($("stepMinimumExcitation").value)
  });
  stepEstimate = result;
  $("stepEstimateRate").textContent = Number.isFinite(result.sampleRateHz) ? formatNumber(result.sampleRateHz, 1) : "—";
  $("stepEstimateWindow").textContent = result.windowSize ?? "—";
  $("stepEstimateWindows").textContent = result.totalWindows == null ? "—" : `${result.acceptedWindows}/${result.totalWindows}`;
  $("stepEstimateMessage").textContent = result.message;
  $("stepEstimateState").textContent = result.ok ? "✓ Estimated" : "▲ Insufficient data";
  $("stepEstimateState").className = `status-badge ${result.ok ? "online" : "fault"}`;
}

const parameterDefinitions = {
  baud: { id: 1, decimals: 0, step: 1, min: 9600, max: 921600, description: "UART rate after the next reboot" },
  periodUs: { id: 2, decimals: 0, step: 100, min: 1000, max: 20000, description: "Deterministic control-loop period in microseconds" },
  cpr: { id: 3, decimals: 0, step: 1, min: 1, max: 1000000, description: "Quadrature counts per output-shaft revolution" },
  streamRate: { id: 4, decimals: 0, step: 1, min: 1, max: 500, description: "Telemetry messages per second" },
  kp: { id: 5, decimals: 4, step: 0.001, min: 0, max: 100, description: "Incremental proportional gain" },
  ki: { id: 6, decimals: 4, step: 0.001, min: 0, max: 1000, description: "Integral gain in 1/s; firmware multiplies by Ts" },
  kd: { id: 7, decimals: 4, step: 0.001, min: 0, max: 100, description: "Incremental derivative gain" },
  vmax: { id: 8, decimals: 2, step: 0.1, min: 0.1, max: 10000, description: "Maximum feasible angular velocity in rad/s" },
  amax: { id: 9, decimals: 2, step: 0.1, min: 0.1, max: 10000, description: "Maximum angular acceleration in rad/s²" },
  jmax: { id: 10, decimals: 2, step: 1, min: 0.1, max: 100000, description: "Maximum jerk in rad/s³" },
  imax: { id: 11, decimals: 2, step: 0.1, min: 0.1, max: 100, description: "Software current trip in amperes; a fuse remains mandatory" },
  maxDuty: { id: 12, decimals: 3, step: 0.01, min: 0.01, max: 1, description: "Maximum PWM duty, expressed from 0 to 1" },
  deadbandFwd: { id: 13, decimals: 3, step: 0.001, min: 0, max: 1, description: "Measured forward breakaway duty" },
  deadbandRev: { id: 14, decimals: 3, step: 0.001, min: 0, max: 1, description: "Measured reverse breakaway duty" },
  direction: { id: 15, decimals: 0, step: 2, min: -1, max: 1, description: "Electrical motor direction multiplier; enter -1 or 1" },
  vinGain: { id: 16, decimals: 4, step: 0.001, min: 1, max: 20, description: "Calibrated reconstruction gain for the 6.8 kΩ / 1 kΩ divider" },
  vinOffset: { id: 17, decimals: 4, step: 0.001, min: -5, max: 5, description: "Calibrated VIN input offset in volts" },
  vinMin: { id: 18, decimals: 2, step: 0.1, min: 0, max: 20, description: "Undervoltage safety limit in volts" },
  vinMax: { id: 19, decimals: 2, step: 0.1, min: 0.1, max: 20, description: "Overvoltage safety limit in volts" },
  currentGain: { id: 20, decimals: 4, step: 0.001, min: 0.001, max: 10000, description: "Calibrated current-sense gain in A/V" },
  currentOffset: { id: 21, decimals: 4, step: 0.001, min: -5, max: 5, description: "Current-sense zero offset in volts" },
  encoderTimeoutMs: { id: 22, decimals: 0, step: 10, min: 50, max: 10000, description: "Maximum time without a valid encoder transition while motion is demanded, in milliseconds" },
  encoderTimeoutVelocity: { id: 23, decimals: 2, step: 0.1, min: 0.1, max: 10000, description: "Desired-velocity threshold that activates the encoder activity watchdog, in rad/s" },
  currentSenseEnabled: { id: 28, decimals: 0, step: 1, min: 0, max: 1, description: "Enable current protection: 0 = disabled, 1 = enabled" },
  currentFilterCutoffHz: { id: 29, decimals: 1, step: 0.1, min: 0.1, max: 200, description: "First-order current-sense low-pass cutoff in Hz; lower values reduce noise but delay overcurrent detection" },
  zeroIndexMinIntervalUs: { id: 30, decimals: 0, step: 100, min: 100, max: 1000000, description: "Minimum accepted interval between zero-index rising edges in microseconds; closer edges are counted as bounce" },
  zeroIndexCorrectionGain: { id: 31, decimals: 3, step: 0.01, min: 0, max: 1, description: "Fraction of zero-index phase error corrected per accepted pulse; 0 trusts encoder counts only after initial reference, 1 snaps fully to every pulse" },
  zeroIndexMinSeparationRevolutions: { id: 32, decimals: 2, step: 0.05, min: 0, max: 0.95, description: "Minimum rotor travel in revolutions before another zero edge can be accepted; 0 disables the encoder-distance bounce filter" },
  characterizationDynamicsCutoffHz: { id: 33, decimals: 1, step: 0.5, min: 0.5, max: 100, description: "Low-pass cutoff used before calculating characterization acceleration and jerk" },
  characterizationDynamicsQuantile: { id: 34, decimals: 3, step: 0.01, min: 0.80, max: 0.99, description: "Robust quantile used instead of a noise-sensitive single acceleration or jerk peak" },
  characterizationSafetyFactor: { id: 35, decimals: 2, step: 0.05, min: 0.10, max: 1, description: "Multiplier applied to the weaker measured direction when recommending acceleration and jerk limits" },
  velocityEstimatorMethod: { id: 36, decimals: 0, step: 1, min: 0, max: 2, description: "Velocity estimator: 0 = low-pass, 1 = characterized motor-model Kalman, 2 = encoder-window acceleration prediction" },
  velocityAccelerationWindowSamples: { id: 37, decimals: 0, step: 1, min: 2, max: 32, description: "Circular velocity-history length used by estimator method 2; larger windows reduce acceleration noise but add lag" },
  currentPin: { decimals: 0, description: "ADC1 input used for motor current sense" },
  diagEnabled: { decimals: 0, description: "Whether the protected EN/DIAG input can trip the machine" },
  diagPin: { decimals: 0, description: "Protected active-low driver diagnostic input" },
  vinPin: { decimals: 0, description: "ADC1 input used to read driver VIN" }
};
function formatNumber(value, decimals = 2) {
  if (typeof value === "boolean") return value ? "Enabled" : "Disabled";
  const number = Number(value);
  if (!Number.isFinite(number)) return "—";
  return Number(number.toFixed(decimals)).toString();
}
function escapeHtml(value) { return String(value).replace(/[&<>"']/g, character => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" })[character]); }
function renderSettings() {
  $("kp").value = formatNumber(settings.kp, 4); $("ki").value = formatNumber(settings.ki, 4); $("kd").value = formatNumber(settings.kd, 4);
  $("driverDiagnosticEnabled").checked = settings.diagEnabled;
  $("currentSenseEnabled").checked = settings.currentSenseEnabled;
  const maximumDuty = Math.max(0.01, Number(settings.maxDuty) || 0.9);
  $("tuningManualVelocity").max = formatNumber(settings.vmax, 2);
  if (Number($("tuningManualVelocity").value) > settings.vmax) $("tuningManualVelocity").value = formatNumber(settings.vmax, 2);
  $("motorTestDutyRange").max = maximumDuty.toFixed(2);
  $("motorTestDuty").max = maximumDuty.toFixed(2);
  if (Number($("motorTestDuty").value) > maximumDuty) setMotorTestDuty(maximumDuty);
  setCurrentCalibrationDuty(Math.min(Number($("currentCalibrationDuty").value) || 0.10, maximumDuty));
  $("parameterRows").innerHTML = Object.entries(parameterDefinitions).map(([key, definition]) => {
    const editable = definition.id !== undefined;
    const value = formatNumber(settings[key], definition.decimals);
    return `<tr><td><code>${key}</code></td><td><button class="parameter-value${editable ? "" : " readonly"}" data-parameter="${key}" ${editable && connected ? "" : "disabled"}>${value}</button></td><td>${definition.description}</td></tr>`;
  }).join("");
  const parametersReady = connected && Number.isFinite(Number(settings.schema));
  $("exportParameters").disabled = !parametersReady;
  $("importParameters").disabled = !parametersReady || Boolean(parameterImportAction);
}

function encodeParameterImportEntry(entry) {
  const payload = new Uint8Array(7);
  const view = new DataView(payload.buffer);
  view.setUint16(0, entry.id, true);
  view.setFloat32(2, entry.value, true);
  payload[6] = 1;
  return payload;
}

function failParameterImport(message) {
  parameterImportAction = undefined;
  $("applyParameterImport").disabled = false;
  $("cancelParameterImport").disabled = false;
  $("importParameters").disabled = !connected || !Number.isFinite(Number(settings.schema));
  $("parameterImportProgress").textContent = message;
  toast(message);
  if (connected) sendFrame(MSG.GET_SETTINGS);
}

function sendNextParameterImport() {
  if (!parameterImportAction || parameterImportAction.stage !== "apply") return;
  const entry = parameterImportAction.entries[parameterImportAction.index];
  $("parameterImportProgress").textContent =
      `Applying ${parameterImportAction.index + 1} of ${parameterImportAction.entries.length}: ${entry.parameter}…`;
  sendFrame(MSG.SET_PARAMETER, encodeParameterImportEntry(entry)).catch(error => {
    failParameterImport(`Could not send "${entry.parameter}": ${error.message}`);
  });
}

function exportParameterCsv() {
  if (!connected || !Number.isFinite(Number(settings.schema))) return;
  downloadCsv(`mocking-machine-parameters-schema-${settings.schema}.csv`,
              createParameterCsv(settings, parameterDefinitions));
  toast("Editable machine parameters exported.");
}

async function reviewParameterCsvFile(file) {
  if (!file) return;
  if (file.size > 65536) return toast("Parameter CSV must be 64 KiB or smaller.");
  try {
    const imported = parseParameterCsv(await file.text(), parameterDefinitions);
    parameterImportDraft = imported.filter(entry =>
      Number(settings[entry.parameter]) !== entry.value);
    if (!parameterImportDraft.length) return toast("The CSV contains no parameter changes.");
    $("parameterImportRows").innerHTML = parameterImportDraft.map(entry => {
      const definition = parameterDefinitions[entry.parameter];
      return `<tr><td><code>${escapeHtml(entry.parameter)}</code></td><td>${escapeHtml(formatNumber(settings[entry.parameter], definition.decimals))}</td><td>${escapeHtml(formatNumber(entry.value, definition.decimals))}</td></tr>`;
    }).join("");
    $("parameterImportSummary").textContent =
        `${parameterImportDraft.length} changed values from ${file.name} will be validated and saved one at a time.`;
    $("parameterImportProgress").textContent = "No values have been applied.";
    $("applyParameterImport").disabled = false;
    $("cancelParameterImport").disabled = false;
    $("parameterImportDialog").showModal();
  } catch (error) {
    parameterImportDraft = undefined;
    toast(`Could not load parameter CSV: ${error.message}`);
  }
}

function setEncoderCalibrationState(label, appearance) {
  $("encoderCalibrationState").textContent = label;
  $("encoderCalibrationState").className = `status-badge ${appearance}`;
}

function signedCountText(value) {
  const count = BigInt(value);
  return count > 0n ? `+${count}` : count.toString();
}

function renderEncoderCalibration() {
  const measuring = encoderCalibrationStartCount !== undefined && !encoderCalibrationCandidate;
  const ready = connected && latestState === 0 && latestEncoderCount !== undefined &&
                Number.isFinite(settings.cpr) && !encoderCalibrationSavePending;
  $("encoderConfiguredCpr").textContent = Number.isFinite(settings.cpr)
      ? formatNumber(settings.cpr, 0)
      : "—";
  $("encoderCalibrationLiveCount").textContent = latestEncoderCount === undefined
      ? "—"
      : latestEncoderCount.toString();
  $("encoderCalibrationLiveDelta").textContent = measuring && latestEncoderCount !== undefined
      ? signedCountText(latestEncoderCount - encoderCalibrationStartCount)
      : "—";
  $("encoderCalibrationRevolutions").disabled = measuring || encoderCalibrationSavePending ||
                                                 Boolean(encoderCalibrationCandidate);
  $("startEncoderCalibration").disabled = !ready || measuring || Boolean(encoderCalibrationCandidate);
  $("finishEncoderCalibration").disabled = !ready || !measuring;
  $("cancelEncoderCalibration").disabled = encoderCalibrationSavePending;
  $("saveEncoderCalibration").disabled = !ready || !encoderCalibrationCandidate;

  if (!connected) setEncoderCalibrationState("● Disconnected", "offline");
  else if (encoderCalibrationSavePending) setEncoderCalibrationState("● Saving…", "offline");
  else if (encoderCalibrationCandidate) setEncoderCalibrationState("● Review result", "online");
  else if (measuring) setEncoderCalibrationState("● Measuring", "online");
  else if (latestState !== 0) setEncoderCalibrationState("▲ Disarm required", "fault");
  else if (latestEncoderCount === undefined) setEncoderCalibrationState("● Waiting for telemetry", "offline");
  else setEncoderCalibrationState("● Ready", "offline");
}

function resetEncoderCalibration(label, message) {
  encoderCalibrationStartCount = undefined;
  encoderCalibrationTurns = undefined;
  encoderCalibrationCandidate = undefined;
  $("encoderCalibrationResult").classList.add("hidden");
  renderEncoderCalibration();
  if (label) setEncoderCalibrationState(label, label.startsWith("▲") ? "fault" : "offline");
  if (message) toast(message);
}

function startEncoderCalibrationMeasurement() {
  const turns = Number($("encoderCalibrationRevolutions").value);
  if (!connected || latestEncoderCount === undefined) return toast("Waiting for live encoder telemetry.");
  if (latestState !== 0) return toast("Stop and disarm the motor before encoder calibration.");
  if (!Number.isInteger(turns) || turns < 1 || turns > 10) {
    return toast("Manual revolutions must be a whole number from 1 to 10.");
  }
  encoderCalibrationStartCount = latestEncoderCount;
  encoderCalibrationTurns = turns;
  encoderCalibrationCandidate = undefined;
  $("encoderCalibrationResult").classList.add("hidden");
  renderEncoderCalibration();
  toast(`Start count captured. Turn the output shaft exactly ${turns} revolution${turns === 1 ? "" : "s"}.`);
}

function finishEncoderCalibrationMeasurement() {
  if (encoderCalibrationStartCount === undefined || latestEncoderCount === undefined) {
    return toast("Capture the start count first.");
  }
  try {
    encoderCalibrationCandidate = calculateEncoderCalibration(
        encoderCalibrationStartCount, latestEncoderCount, encoderCalibrationTurns, settings.cpr);
  } catch (error) {
    return toast(error.message);
  }
  const result = encoderCalibrationCandidate;
  $("encoderCalibrationStartCount").textContent = encoderCalibrationStartCount.toString();
  $("encoderCalibrationCountDelta").textContent = signedCountText(result.signedCountDelta);
  $("encoderCalibrationMeasuredCpr").textContent = formatNumber(result.measuredCountsPerRevolution, 3);
  $("encoderCalibrationCandidateCpr").textContent = formatNumber(result.candidateCountsPerRevolution, 0);
  $("encoderCalibrationChange").textContent = Number.isFinite(result.changePercent)
      ? `${result.changePercent >= 0 ? "+" : ""}${formatNumber(result.changePercent, 2)}%`
      : "—";
  $("encoderCalibrationResidual").textContent = `${signedCountText(BigInt(result.roundingResidualCounts))} counts`;
  $("encoderCalibrationResult").classList.remove("hidden");
  renderEncoderCalibration();
  $("encoderCalibrationResult").scrollIntoView({
    behavior: matchMedia("(prefers-reduced-motion: reduce)").matches ? "auto" : "smooth",
    block: "nearest"
  });
}

async function saveEncoderCalibrationResult() {
  if (!encoderCalibrationCandidate || encoderCalibrationSavePending) return;
  if (latestState !== 0) return toast("Motor must be disarmed before saving encoder calibration.");
  const value = encoderCalibrationCandidate.candidateCountsPerRevolution;
  const payload = new Uint8Array(7), view = new DataView(payload.buffer);
  view.setUint16(0, parameterDefinitions.cpr.id, true);
  view.setFloat32(2, value, true);
  payload[6] = 1;
  encoderCalibrationSavePending = true;
  renderEncoderCalibration();
  try {
    await sendFrame(MSG.SET_PARAMETER, payload);
  } catch (error) {
    encoderCalibrationSavePending = false;
    renderEncoderCalibration();
    toast(`Could not send encoder calibration: ${error.message}`);
  }
}

function openParameterEditor(key) {
  const definition = parameterDefinitions[key];
  if (!connected || !definition?.id) return;
  editingParameterKey = key;
  $("parameterEditorName").textContent = key;
  $("parameterEditorDescription").textContent = definition.description;
  const input = $("parameterEditorValue");
  input.value = typeof settings[key] === "boolean" ? (settings[key] ? 1 : 0) : formatNumber(settings[key], definition.decimals);
  input.step = definition.step; input.min = definition.min; input.max = definition.max;
  $("parameterEditorRange").textContent = `Allowed range: ${formatNumber(definition.min, definition.decimals)} to ${formatNumber(definition.max, definition.decimals)}`;
  $("parameterEditorDialog").showModal(); input.focus(); input.select();
}

function decodeProfile(data) {
  const nameBytes = new Uint8Array(data.buffer, data.byteOffset + 3, 16);
  const zero = nameBytes.indexOf(0);
  const pointCount = Math.min(16, data.getUint8(39));
  return {
    id: data.getUint16(0, true), kind: data.getUint8(2),
    name: decoder.decode(nameBytes.slice(0, zero < 0 ? 16 : zero)),
    target: data.getFloat32(19, true), sineMean: data.getFloat32(23, true),
    sineAmplitude: data.getFloat32(27, true), sineFrequency: data.getFloat32(31, true),
    durationMs: data.getUint32(35, true),
    points: Array.from({ length: pointCount }, (_, index) => ({
      time: data.getUint32(40 + index * 8, true) / 1000,
      velocity: data.getFloat32(44 + index * 8, true)
    }))
  };
}

const loadColors = ["#4ec3e0", "#f0b84a", "#58d69d", "#b58cff", "#ff7a90", "#77a6ff", "#e88de7", "#a9d45b", "#ff9d57", "#45d4c5", "#d8c65a", "#9e93ff"];

function renderRotorLoadSetup() {
  const loadEditingEnabled = connected && latestState === 0 && !loadSavePending;
  const bySlot = new Map(loadConfiguration.map(load => [load.slot, load]));
  const slots = Array.from({ length: ROTOR_SLOT_COUNT }, (_, slot) => {
    const point = slotPosition(slot);
    const load = bySlot.get(slot);
    const color = load ? loadColors[slot] : "#6e7c85";
    const dx = 210 - point.x, dy = 210 - point.y;
    const length = load ? 18 + load.strength * 4 : 0;
    const magnitude = Math.hypot(dx, dy) || 1;
    const arrowX = point.x + dx / magnitude * length;
    const arrowY = point.y + dy / magnitude * length;
    const badgeX = point.x - dx / magnitude * 34;
    const badgeY = point.y - dy / magnitude * 34;
    return `<g class="rotor-slot" data-slot="${slot}" tabindex="0" role="button" aria-disabled="${!loadEditingEnabled}" aria-label="${slot * 30} degree slot, ${load ? `strength ${load.strength}` : "empty"}" style="color:${color}">
      <circle class="slot-hit" cx="${point.x}" cy="${point.y}" r="34"/>
      <circle class="slot-hole" cx="${point.x}" cy="${point.y}" r="24" ${load ? `style="fill:${color}33;stroke:${color}"` : ""}/>
      ${load ? `<line class="strength-arrow" x1="${point.x}" y1="${point.y}" x2="${arrowX}" y2="${arrowY}"/><circle class="slot-badge" cx="${badgeX}" cy="${badgeY}" r="15"/><text x="${badgeX}" y="${badgeY}">${load.strength}</text>` : ""}
    </g>`;
  }).join("");
  $("rotorLoadDiagram").innerHTML = `<defs><marker id="loadArrow" markerWidth="7" markerHeight="7" refX="5" refY="3.5" orient="auto"><path d="M0,0 L7,3.5 L0,7 Z" fill="context-stroke"/></marker></defs>
    <circle cx="210" cy="210" r="196" fill="#89918f" stroke="#d5dcda" stroke-width="2"/>
    <circle cx="210" cy="210" r="92" fill="#c7c9c3" stroke="#111820" stroke-width="5"/>
    <circle cx="210" cy="210" r="50" fill="#0b1015" stroke="#6e7c85" stroke-width="3"/>
    <path d="M210 116 V304 M116 210 H304" stroke="#44515a" stroke-width="1" stroke-dasharray="5 6"/>
    ${slots}`;
  $("rotorLoadDiagram").querySelectorAll(".rotor-slot").forEach(element => {
    const open = () => openLoadSlotEditor(Number(element.dataset.slot));
    element.addEventListener("click", open);
    element.addEventListener("keydown", event => {
      if (event.key === "Enter" || event.key === " ") { event.preventDefault(); open(); }
    });
  });
  $("loadSetupStatus").textContent = loadSavePending
    ? "Saving…"
    : `${loadConfiguration.length} active slot${loadConfiguration.length === 1 ? "" : "s"}`;
  $("loadSetupStatus").className = `status-badge ${loadSavePending ? "offline" : loadConfiguration.length ? "online" : "offline"}`;
  $("loadSlotList").innerHTML = loadConfiguration.length
    ? loadConfiguration.map(load => `<span class="load-slot-chip" style="color:${loadColors[load.slot]}"><i></i>${load.position}° · strength ${load.strength}</span>`).join("")
    : '<p class="hint">No load positions configured.</p>';
}

function openLoadSlotEditor(slot) {
  if (!connected) return toast("Connect to the firmware before changing the load setup.");
  if (latestState !== 0) return toast("Disarm the machine before changing the load setup.");
  if (loadSavePending) return toast("Wait for the current load change to finish saving.");
  editingLoadSlot = slot;
  const existing = loadConfiguration.find(load => load.slot === slot);
  $("loadSlotPosition").textContent = slot * 30;
  $("loadSlotStrength").value = existing?.strength ?? 1;
  $("loadSlotStrengthValue").value = existing?.strength ?? 1;
  $("loadSlotStrengthValue").textContent = existing?.strength ?? 1;
  $("removeLoadSlot").disabled = !existing;
  $("loadSlotDialog").showModal();
}

function encodeLoadConfiguration() {
  const payload = new Uint8Array(86), view = new DataView(payload.buffer);
  view.setUint8(0, ((Number(settings.loadSetting) || 0) + 1) & 0xff);
  view.setUint8(1, loadConfiguration.length);
  loadConfiguration.forEach((load, index) => {
    view.setUint8(2 + index * 7, load.slot);
    view.setUint16(3 + index * 7, load.position, true);
    view.setFloat32(5 + index * 7, load.strength, true);
  });
  return payload;
}

async function persistLoadConfiguration() {
  if (!connected || !writer || latestState !== 0 || loadSavePending) {
    loadConfiguration = loadConfigurationSaved.map(load => ({ ...load }));
    renderRotorLoadSetup();
    return;
  }
  loadSavePending = true;
  renderRotorLoadSetup();
  try {
    await sendFrame(MSG.SET_LOAD_CONFIGURATION, encodeLoadConfiguration());
  } catch (error) {
    loadSavePending = false;
    loadConfiguration = loadConfigurationSaved.map(load => ({ ...load }));
    renderRotorLoadSetup();
    toast(`Could not save load change: ${error.message}`);
  }
}

function renderRunProfileDialog() {
  if (!$("runProfileSelect")) return;
  const previous = $("runProfileSelect").value;
  $("runProfileSelect").innerHTML = profiles.length
    ? profiles.map(profile => `<option value="${profile.id}">${escapeHtml(profile.name)}${profile.id === settings.profileId ? " · default" : ""}</option>`).join("")
    : '<option value="">No profiles available</option>';
  const preferred = profiles.some(profile => String(profile.id) === previous)
    ? previous : String(runtimeProfileId ?? settings.profileId ?? "");
  $("runProfileSelect").value = preferred;
  const armed = latestState === 1;
  $("runDialogArm").disabled = !connected || armed || latestState !== 0 || latestFaults !== 0 || Boolean(overviewRunAction);
  $("runDialogArm").textContent = armed ? "Armed" : "Arm output";
  $("runDialogArmHint").textContent = armed ? "Output is armed. Verify the guard before running." : "The machine must be armed before Run is enabled.";
  $("confirmRunProfile").disabled = !connected || !armed || !profiles.length || Boolean(overviewRunAction);
  $("setDefaultProfile").disabled = !connected || latestState !== 0 || !profiles.length || Boolean(defaultProfilePending);
}

function renderProfiles() {
  if (!$("profileRows")) return;
  const full = profiles.length >= MAX_PROFILES;
  const unavailableReason = !connected
    ? "Connect to create a profile."
    : full
      ? `Firmware profile storage is full (${MAX_PROFILES}/${MAX_PROFILES}).`
      : profileAction
        ? "Waiting for the current profile command to finish…"
        : "";
  $("profileCapacity").textContent = `${profiles.length} / ${MAX_PROFILES} profiles`;
  $("profileCapacity").className = `status-badge ${full ? "fault" : "offline"}`;
  $("newProfile").disabled = Boolean(unavailableReason);
  $("newProfile").title = unavailableReason;
  $("profileCreateReason").textContent = unavailableReason || "Ready to create another profile.";
  if (!profiles.length) {
    $("profileRows").innerHTML = `<tr><td colspan="6">${connected ? "Waiting for firmware profiles…" : "Connect to load firmware profiles."}</td></tr>`;
    $("tuningProfileSelect").innerHTML = '<option value="">No profiles available</option>';
    return;
  }
  const kinds = ["Ramp", "Sine", "Waypoints"];
  $("profileRows").innerHTML = profiles.map(profile => {
    const labels = [profile.id === settings.profileId ? "Default" : "", profile.id === runtimeProfileId ? "Next run" : ""].filter(Boolean).join(" · ") || "Available";
    const selectDisabled = profile.id === runtimeProfileId || !connected || latestState === 2 || latestState === 3 || Boolean(overviewRunAction);
    return `<tr><td>${profile.id}</td><td>${escapeHtml(profile.name)}</td><td>${kinds[profile.kind] ?? "Unknown"}</td><td>${formatNumber(profile.durationMs / 1000, 2)} s</td><td>${labels}</td><td><div class="action-row"><button class="select-profile" data-profile-id="${profile.id}" ${selectDisabled ? "disabled" : ""}>Select</button><button class="edit-profile" data-profile-id="${profile.id}">Edit</button></div></td></tr>`;
  }).join("");
  const previous = $("tuningProfileSelect").value;
  $("tuningProfileSelect").innerHTML = profiles.map(profile => `<option value="${profile.id}">${escapeHtml(profile.name)} · ${formatNumber(profile.durationMs / 1000, 2)} s</option>`).join("");
  $("tuningProfileSelect").value = profiles.some(profile => String(profile.id) === previous) ? previous : String(runtimeProfileId ?? settings.profileId);
  document.querySelectorAll(".select-profile").forEach(button => button.addEventListener("click", async () => {
    if (overviewRunAction) return;
    const profileId = Number(button.dataset.profileId);
    const payload = new Uint8Array(2); new DataView(payload.buffer).setUint16(0, profileId, true);
    button.disabled = true;
    overviewRunAction = { stage: "profile-tab-select", profileId };
    renderProfiles();
    await sendFrame(MSG.SELECT_PROFILE, payload);
  }));
  renderRunProfileDialog();
}

function finishProfileAction() {
  const action = profileAction;
  if (action?.timeoutId !== undefined) clearTimeout(action.timeoutId);
  profileAction = undefined;
  return action;
}

function armProfileActionTimeout(action) {
  action.timeoutId = setTimeout(() => {
    if (profileAction !== action) return;
    finishProfileAction();
    $("saveProfile").disabled = !connected;
    $("runProfileTest").disabled = !connected;
    renderProfiles();
    $("profileRunCommandStatus").classList.remove("hidden");
    $("profileRunCommandStatus").textContent = "Firmware did not acknowledge the profile command. The controls were unlocked; reconnect if communication remains stalled.";
    if (action.type === "run" && writer) sendFrame(MSG.STOP_RUN).catch(() => {});
    toast("Profile command timed out; controls unlocked.");
  }, PROFILE_ACTION_TIMEOUT_MS);
}

function profileToWaypoints(profile) {
  const duration = Math.max(0.1, profile.durationMs / 1000);
  if (profile.kind === 2 && profile.points.length >= 2) return profile.points.map(point => ({ ...point }));
  if (profile.kind === 1) {
    const count = Math.min(16, Math.max(8, Math.ceil(duration * Math.max(profile.sineFrequency, 0.1) * 8)));
    const points = Array.from({ length: count }, (_, index) => {
      const time = duration * index / (count - 1);
      const velocity = profile.sineMean + profile.sineAmplitude * Math.sin(2 * Math.PI * profile.sineFrequency * time);
      return { time, velocity: Math.max(0, velocity) };
    });
    points[0].velocity = 0; points.at(-1).velocity = 0;
    return points;
  }
  const target = Math.min(Number(settings.vmax) || 1, Math.max(0, profile.target));
  const rampTime = Math.min(duration / 2, target / Math.max(Number(settings.amax) || 1, 0.001));
  const points = [{ time: 0, velocity: 0 }];
  if (rampTime > 0.001) points.push({ time: rampTime, velocity: target });
  if (duration - rampTime > rampTime + 0.001) points.push({ time: duration - rampTime, velocity: target });
  points.push({ time: duration, velocity: 0 });
  return points;
}

function constrainProfilePoints() {
  if (!editingProfile) return;
  const points = editingProfile.points;
  editingProfile.duration = Math.round(editingProfile.duration * 1000) / 1000;
  const duration = editingProfile.duration;
  const vmax = Math.max(0.1, Number(settings.vmax) || 1);
  const amax = Math.max(0.1, Number(settings.amax) || 1);
  points.forEach(point => { point.time = Math.round(point.time * 1000) / 1000; });
  points.sort((a, b) => a.time - b.time);
  points[0].time = 0; points[0].velocity = 0;
  points.at(-1).time = duration; points.at(-1).velocity = 0;
  for (let index = 1; index < points.length - 1; index++) {
    const latest = duration - (points.length - 1 - index) * 0.001;
    points[index].time = Math.min(latest, Math.max(points[index - 1].time + 0.001, points[index].time));
  }
  for (let pass = 0; pass < 5; pass++) {
    points[0].velocity = 0;
    for (let index = 1; index < points.length; index++) {
      const span = Math.max(0.001, points[index].time - points[index - 1].time);
      points[index].velocity = Math.min(vmax, Math.max(0, points[index].velocity), points[index - 1].velocity + amax * span);
      points[index].velocity = Math.max(points[index].velocity, points[index - 1].velocity - amax * span);
    }
    points.at(-1).velocity = 0;
    for (let index = points.length - 2; index >= 0; index--) {
      const span = Math.max(0.001, points[index + 1].time - points[index].time);
      points[index].velocity = Math.min(points[index].velocity, points[index + 1].velocity + amax * span);
      points[index].velocity = Math.max(0, points[index].velocity, points[index + 1].velocity - amax * span);
    }
  }
  points[0].velocity = 0; points.at(-1).velocity = 0;
}

function openProfileDraft(source, createOnly) {
  editingProfile = { id: source.id, name: source.name, duration: Math.max(0.1, source.durationMs / 1000), points: profileToWaypoints(source), createOnly };
  selectedProfilePoint = 0;
  constrainProfilePoints();
  $("profileName").value = editingProfile.name;
  $("profileDuration").value = formatNumber(editingProfile.duration, 2);
  $("profileLimitVelocity").textContent = formatNumber(settings.vmax, 2);
  $("profileLimitAcceleration").textContent = formatNumber(settings.amax, 2);
  $("profileLimitJerk").textContent = formatNumber(settings.jmax, 2);
  $("profileEditorTitle").textContent = createOnly ? "Create velocity profile" : "Edit velocity profile";
  $("saveProfile").textContent = createOnly ? "Create profile" : "Save profile";
  updateProfileEditor();
  $("saveProfile").disabled = !connected;
  $("runProfileTest").disabled = !connected;
  $("profileEditorDialog").showModal();
  requestAnimationFrame(drawProfileEditorChart);
}

function openProfileEditor(profileId) {
  const source = profiles.find(profile => profile.id === profileId);
  if (source) openProfileDraft(source, false);
}

function openNewProfileEditor() {
  const id = nextAvailableProfileId(profiles, MAX_PROFILES);
  if (id === undefined) return toast(`Firmware can store at most ${MAX_PROFILES} profiles.`);
  const target = Math.min(20, Math.max(0.1, Number(settings.vmax) || 1));
  openProfileDraft({
    id,
    name: `profile-${id}`,
    kind: 0,
    durationMs: 5000,
    target
  }, true);
}

function updateProfileEditor() {
  if (!editingProfile) return;
  constrainProfilePoints();
  selectedProfilePoint = Math.max(0, Math.min(selectedProfilePoint, editingProfile.points.length - 1));
  const point = editingProfile.points[selectedProfilePoint];
  const endpoint = selectedProfilePoint === 0 || selectedProfilePoint === editingProfile.points.length - 1;
  $("profilePointTime").value = formatNumber(point.time, 3);
  $("profilePointVelocity").value = formatNumber(point.velocity, 3);
  $("profilePointTime").disabled = endpoint;
  $("profilePointVelocity").disabled = endpoint;
  $("removeProfilePoint").disabled = endpoint || editingProfile.points.length <= 2;
  $("addProfilePoint").disabled = editingProfile.points.length >= 16;
  $("profilePointCount").textContent = editingProfile.points.length;
  $("profileFeasibility").textContent = "✓ Feasible";
  $("profileFeasibility").className = "status-badge online";
  $("profileConstraintMessage").textContent = "Requested segments obey the velocity and acceleration limits. The cyan preview shows the additional firmware jerk limiting.";
  drawProfileEditorChart();
}

function desiredProfileVelocity(time) {
  const points = editingProfile.points;
  if (time <= 0) return points[0].velocity;
  for (let index = 1; index < points.length; index++) {
    if (time <= points[index].time) {
      const previous = points[index - 1], next = points[index];
      const blend = (time - previous.time) / Math.max(0.000001, next.time - previous.time);
      return previous.velocity + blend * (next.velocity - previous.velocity);
    }
  }
  return points.at(-1).velocity;
}

function constrainedProfilePreview() {
  const result = [];
  const steps = 500;
  const dt = editingProfile.duration / steps;
  const vmax = Number(settings.vmax), amax = Number(settings.amax), jmax = Number(settings.jmax);
  let velocity = 0, acceleration = 0;
  for (let index = 0; index <= steps; index++) {
    const time = index * dt;
    const target = Math.min(vmax, Math.max(0, desiredProfileVelocity(time)));
    const error = target - velocity;
    const desiredAcceleration = Math.min(amax, Math.max(-amax, error / Math.max(dt, 0.000001)));
    acceleration += Math.min(jmax * dt, Math.max(-jmax * dt, desiredAcceleration - acceleration));
    const step = acceleration * dt;
    if (Math.abs(step) >= Math.abs(error)) { velocity = target; acceleration = 0; } else velocity += step;
    result.push({ time, velocity });
  }
  return result;
}

function profileChartGeometry() {
  const canvas = $("profileEditorChart");
  const rect = canvas.getBoundingClientRect();
  return { canvas, rect, left: 56, right: 18, top: 18, bottom: 38, width: Math.max(1, rect.width - 74), height: Math.max(1, rect.height - 56) };
}

function drawProfileEditorChart() {
  if (!editingProfile || !$("profileEditorDialog").open) return;
  const g = profileChartGeometry(), ctx = g.canvas.getContext("2d"), dpr = window.devicePixelRatio || 1;
  if (g.canvas.width !== Math.round(g.rect.width * dpr) || g.canvas.height !== Math.round(g.rect.height * dpr)) { g.canvas.width = Math.round(g.rect.width * dpr); g.canvas.height = Math.round(g.rect.height * dpr); }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0); ctx.clearRect(0, 0, g.rect.width, g.rect.height);
  const vmax = Math.max(1, Number(settings.vmax));
  const xFor = time => g.left + time / editingProfile.duration * g.width;
  const yFor = velocity => g.top + (1 - velocity / vmax) * g.height;
  ctx.font = "12px system-ui"; ctx.fillStyle = "#9eb0bc"; ctx.strokeStyle = "#22303a"; ctx.lineWidth = 1;
  for (let line = 0; line <= 4; line++) {
    const y = g.top + g.height * line / 4; ctx.beginPath(); ctx.moveTo(g.left, y); ctx.lineTo(g.left + g.width, y); ctx.stroke();
    ctx.fillText(formatNumber(vmax * (1 - line / 4), 1), 6, y + 4);
  }
  for (let line = 0; line <= 4; line++) {
    const x = g.left + g.width * line / 4; ctx.beginPath(); ctx.moveTo(x, g.top); ctx.lineTo(x, g.top + g.height); ctx.stroke();
    ctx.fillText(`${formatNumber(editingProfile.duration * line / 4, 1)}s`, x - 10, g.top + g.height + 22);
  }
  const preview = constrainedProfilePreview();
  ctx.strokeStyle = "#4ec3e0"; ctx.lineWidth = 2; ctx.beginPath(); preview.forEach((point, index) => index ? ctx.lineTo(xFor(point.time), yFor(point.velocity)) : ctx.moveTo(xFor(point.time), yFor(point.velocity))); ctx.stroke();
  ctx.strokeStyle = "#f0b84a"; ctx.lineWidth = 2; ctx.beginPath(); editingProfile.points.forEach((point, index) => index ? ctx.lineTo(xFor(point.time), yFor(point.velocity)) : ctx.moveTo(xFor(point.time), yFor(point.velocity))); ctx.stroke();
  editingProfile.points.forEach((point, index) => { ctx.beginPath(); ctx.arc(xFor(point.time), yFor(point.velocity), index === selectedProfilePoint ? 7 : 5, 0, Math.PI * 2); ctx.fillStyle = index === selectedProfilePoint ? "#eef4f7" : "#f0b84a"; ctx.fill(); ctx.strokeStyle = "#0a0f14"; ctx.stroke(); });
}

function setSelectedProfilePoint(time, velocity, allowTime = true) {
  if (!editingProfile) return;
  const point = editingProfile.points[selectedProfilePoint];
  const endpoint = selectedProfilePoint === 0 || selectedProfilePoint === editingProfile.points.length - 1;
  if (allowTime && !endpoint) {
    const previous = editingProfile.points[selectedProfilePoint - 1], next = editingProfile.points[selectedProfilePoint + 1];
    point.time = Math.min(next.time - 0.01, Math.max(previous.time + 0.01, time));
  }
  if (!endpoint) point.velocity = Math.min(Number(settings.vmax), Math.max(0, velocity));
  updateProfileEditor();
}

function addProfilePoint(time, velocity) {
  if (!editingProfile || editingProfile.points.length >= 16) return toast("A profile can contain at most 16 waypoints.");
  const safeTime = Math.min(editingProfile.duration - 0.01, Math.max(0.01, time));
  const existing = editingProfile.points.findIndex(point => Math.abs(point.time - safeTime) < 0.01);
  if (existing >= 0) { selectedProfilePoint = existing; return updateProfileEditor(); }
  editingProfile.points.push({ time: safeTime, velocity: Math.min(Number(settings.vmax), Math.max(0, velocity)) });
  editingProfile.points.sort((a, b) => a.time - b.time);
  selectedProfilePoint = editingProfile.points.findIndex(point => point.time === safeTime);
  updateProfileEditor();
}

function encodeProfile(persist) {
  const payload = new Uint8Array(169), view = new DataView(payload.buffer);
  payload[0] = persist ? 1 : 0; view.setUint16(1, editingProfile.id, true); payload[3] = 2;
  payload.set(encoder.encode(editingProfile.name).slice(0, 15), 4);
  view.setFloat32(20, 0, true); view.setFloat32(24, 0, true); view.setFloat32(28, 0, true); view.setFloat32(32, 1, true);
  view.setUint32(36, Math.round(editingProfile.duration * 1000), true); payload[40] = editingProfile.points.length;
  editingProfile.points.forEach((point, index) => { view.setUint32(41 + index * 8, Math.round(point.time * 1000), true); view.setFloat32(45 + index * 8, point.velocity, true); });
  return payload;
}

async function submitProfile(type) {
  if (!connected || !editingProfile || profileAction) return;
  editingProfile.name = $("profileName").value.trim();
  if (!editingProfile.name) return toast("Profile name is required.");
  const preparation = profilePreparation(latestState, characterizationRunning);
  if (preparation.blocked) return toast(preparation.blocked);
  if (type === "run" && !await confirmSafety("This test will rotate the motor using the edited velocity path. Verify the imbalance setting, close the guard, and keep the emergency stop accessible.")) return;
  profileAction = { type, profileId: editingProfile.id, createOnly: editingProfile.createOnly,
                    stage: preparation.firstCommand };
  armProfileActionTimeout(profileAction);
  $("saveProfile").disabled = true; $("runProfileTest").disabled = true;
  $("newProfile").disabled = true;
  if (type === "run") {
    profileTestActive = false; profileTestSawRunning = false; profileTestSamples = [];
    $("profileRunCommandStatus").classList.remove("hidden");
    $("profileRunCommandStatus").textContent = "Stopping and disarming before applying the profile…";
    $("profileTestProgress").classList.remove("hidden");
    $("stopProfileTest").disabled = true;
    setProfileTestStatus("● Preparing", "Waiting for firmware command confirmations…", "offline");
  }
  if (preparation.firstCommand === "stop") {
    toast("Stopping and disarming before applying the profile…");
    await sendFrame(MSG.STOP_RUN);
  } else {
    await sendFrame(profileAction.createOnly ? MSG.CREATE_PROFILE : MSG.SET_PROFILE,
                    encodeProfile(type === "save"));
  }
}

async function startTuningResponseTest() {
  if (!connected || tuningAction || tuningTestActive) return;
  if (gainDraftDirty) return toast("Apply the draft gains before running a response test.");
  if (latestState === 3) return toast("Clear and recheck firmware faults before tuning.");
  if (characterizationRunning) return toast("Abort motor characterization before tuning.");
  const mode = document.querySelector('input[name="tuningTestMode"]:checked').value;
  const action = { mode, stage: "stop" };
  if (mode === "profile") {
    action.profileId = Number($("tuningProfileSelect").value);
    if (!profiles.some(profile => profile.id === action.profileId)) return toast("Select a valid stored profile.");
  } else {
    action.velocity = Number($("tuningManualVelocity").value);
    action.duration = Number($("tuningManualDuration").value);
    if (!(action.velocity > 0 && action.velocity <= Number(settings.vmax))) return toast(`Manual velocity must be between 0 and ${formatNumber(settings.vmax, 2)} rad/s.`);
    if (!(action.duration >= 0.1 && action.duration <= 3600)) return toast("Test duration must be between 0.1 and 3600 seconds.");
  }
  if (!await confirmSafety("The response test will rotate the machine using the currently applied controller gains. Verify the load, close the guard, and keep the emergency stop accessible.")) return;
  tuningAction = action;
  tuningSamples = [];
  $("tuningPeakVelocity").textContent = "—"; $("tuningOvershoot").textContent = "—"; $("tuningRiseTime").textContent = "—"; $("tuningSettlingTime").textContent = "—";
  $("startTuningTest").disabled = true; $("stopTuningTest").disabled = true;
  setTuningTestStatus("● Preparing", "Stopping and disarming before the response test…", "offline");
  await sendFrame(MSG.STOP_RUN);
}

function signedMotorTestDuty() {
  const direction = Number(document.querySelector('input[name="motorTestDirection"]:checked').value);
  return direction * Number($("motorTestDuty").value);
}

async function sendMotorTestDuty(duty) {
  const payload = new Uint8Array(4);
  new DataView(payload.buffer).setFloat32(0, duty, true);
  await sendFrame(MSG.MOTOR_TEST, payload);
}

function setMotorTestActive(active) {
  motorTestActive = active;
  clearInterval(motorTestTimer);
  motorTestTimer = undefined;
  $("motorTestState").textContent = active ? "● Raw output active" : "● Stopped";
  $("motorTestState").className = `status-badge ${active ? "online" : "offline"}`;
  $("startMotorTest").disabled = !connected || active || Boolean(motorTestAction);
}

async function startMotorTest() {
  const duty = signedMotorTestDuty();
  if (latestState !== 0 && latestState !== 1) return toast("Stop the active run or clear the fault before motor testing.");
  if (!(Math.abs(duty) >= 0.01 && Math.abs(duty) <= Number(settings.maxDuty))) return toast("PWM duty is outside the firmware limit.");
  if (!await confirmSafety("Direct raw PWM will rotate the motor without velocity control. Remove imbalance weights, close the guard, verify direction, and keep the emergency stop accessible.")) return;
  motorTestAction = { stage: "arm", duty };
  setMotorTestActive(false);
  $("motorTestState").textContent = "● Requesting arm…";
  try {
    await sendFrame(MSG.ARM);
  } catch (error) {
    motorTestAction = undefined;
    setMotorTestActive(false);
    toast(`Could not request motor-test arm: ${error.message}`);
  }
}

async function stopMotorTest(disarm = true) {
  const wasActive = motorTestActive;
  motorTestAction = undefined;
  setMotorTestActive(false);
  if (!writer) return;
  if (wasActive) await sendMotorTestDuty(0);
  if (disarm) await sendFrame(MSG.STOP_RUN);
}

function setMotorTestDuty(value) {
  const maximum = Number(settings.maxDuty) || 0.9;
  const duty = Math.min(maximum, Math.max(0.01, Number(value) || 0.01));
  $("motorTestDutyRange").value = duty;
  $("motorTestDuty").value = duty.toFixed(2);
  $("motorTestDuty").nextElementSibling.textContent = `${Math.round(duty * 100)}%`;
}

function encodeCurrentCalibrationCommand(action, referenceCurrentA = 0) {
  const payload = new Uint8Array(5), view = new DataView(payload.buffer);
  payload[0] = action;
  view.setFloat32(1, referenceCurrentA, true);
  return payload;
}

async function sendCurrentCalibrationCommand(action, referenceCurrentA = 0) {
  if (currentCalibrationPendingAction !== undefined) return;
  currentCalibrationPendingAction = action;
  setCurrentCalibrationBusy(true);
  try {
    await sendFrame(MSG.CURRENT_CALIBRATION,
                    encodeCurrentCalibrationCommand(action, referenceCurrentA));
  } catch (error) {
    currentCalibrationPendingAction = undefined;
    setCurrentCalibrationBusy(false);
    toast(`Could not send calibration command: ${error.message}`);
  }
}

function setCurrentCalibrationBusy(busy) {
  $("captureCurrentPoint1").disabled = busy || !connected;
  $("captureCurrentPoint2").disabled = busy || !connected || !currentCalibrationDriveActive || (currentCalibrationCapturedMask & 1) === 0;
  $("resetCurrentCalibration").disabled = busy || !connected;
  $("cancelCurrentCalibration").disabled = busy || !connected;
  $("saveCurrentCalibration").disabled = busy || !connected || (currentCalibrationCapturedMask & 2) === 0;
  if (busy) {
    $("currentCalibrationState").textContent = "● Capturing…";
    $("currentCalibrationState").className = "status-badge offline";
  }
}

function renderCurrentCalibrationStatus(result) {
  currentCalibrationCapturedMask = result.capturedMask;
  const point1Captured = (result.capturedMask & 1) !== 0;
  const point2Captured = (result.capturedMask & 2) !== 0;
  $("currentPoint1Voltage").textContent = point1Captured ? formatNumber(result.point1Voltage, 4) : "—";
  $("currentPoint2Voltage").textContent = point2Captured ? formatNumber(result.point2Voltage, 4) : "—";
  if (point1Captured) $("currentReferencePoint1").value = formatNumber(result.point1Current, 3);
  if (point2Captured) $("currentReferencePoint2").value = formatNumber(result.point2Current, 3);
  setCurrentCalibrationBusy(result.capturePoint !== 0);
  $("currentCalibrationResult").classList.toggle("hidden", !point2Captured);
  if (point2Captured) {
    $("currentCandidateGain").textContent = formatNumber(result.gain, 4);
    $("currentCandidateOffset").textContent = formatNumber(result.offset, 5);
  }
  if (result.lastResult !== 0 && result.lastResult !== currentCalibrationLastResult) {
    toast("Point 2 needs at least 0.01 A and 0.001 V more span than Point 1.");
  }
  currentCalibrationLastResult = result.lastResult;
  const label = result.capturePoint !== 0 ? `● Capturing point ${result.capturePoint}…` : point2Captured ? "✓ Ready to save" : point1Captured ? "● Point 1 captured" : "● No points";
  $("currentCalibrationState").textContent = label;
  $("currentCalibrationState").className = `status-badge ${point2Captured && result.capturePoint === 0 ? "online" : "offline"}`;
}

function currentReferenceValue(id) {
  const value = Number($(id).value);
  if (!Number.isFinite(value) || value < 0 || value > 50) {
    toast("Meter reference current must be between 0 and 50 A.");
    return undefined;
  }
  return value;
}

function setCurrentCalibrationDuty(value) {
  const maximum = Number(settings.maxDuty) || 0.9;
  const duty = Math.min(maximum, Math.max(0.01, Number(value) || 0.01));
  $("currentCalibrationDutyRange").max = maximum;
  $("currentCalibrationDuty").max = maximum;
  $("currentCalibrationDutyRange").value = duty;
  $("currentCalibrationDuty").value = duty.toFixed(2);
  $("currentCalibrationDuty").nextElementSibling.textContent = `${Math.round(duty * 100)}%`;
}

function setCurrentCalibrationDriveActive(active) {
  currentCalibrationDriveActive = active;
  clearInterval(currentCalibrationDriveTimer);
  currentCalibrationDriveTimer = undefined;
  $("currentCalibrationDriveState").textContent = active ? "● Raw output active" : "■ Stopped";
  $("startCurrentCalibrationDrive").disabled = !connected || active || currentCalibrationDriveArmPending;
  $("stopCurrentCalibrationDrive").disabled = !connected || !active;
  $("currentCalibrationDutyRange").disabled = active;
  $("currentCalibrationDuty").disabled = active;
  setCurrentCalibrationBusy(currentCalibrationPendingAction !== undefined);
}

async function sendCurrentCalibrationDriveDuty(duty = Number($("currentCalibrationDuty").value)) {
  await sendMotorTestDuty(duty);
}

async function startCurrentCalibrationDrive() {
  const duty = Number($("currentCalibrationDuty").value);
  if (latestState !== 0 && latestState !== 1) return toast("Stop the active run or clear the fault before calibration.");
  if (!(duty >= 0.01 && duty <= Number(settings.maxDuty))) return toast("Calibration PWM is outside the firmware limit.");
  if (!await confirmSafety("Current calibration will rotate the motor with direct raw PWM. Remove imbalance weights, close the guard, connect the current meter, and keep the emergency stop accessible.")) return;
  currentCalibrationDriveArmPending = true;
  setCurrentCalibrationDriveActive(false);
  await sendFrame(MSG.ARM);
}

async function stopCurrentCalibrationDrive(disarm = true) {
  const wasActive = currentCalibrationDriveActive;
  currentCalibrationDriveArmPending = false;
  setCurrentCalibrationDriveActive(false);
  if (!writer) return;
  if (wasActive) await sendCurrentCalibrationDriveDuty(0);
  if (disarm) await sendFrame(MSG.STOP_RUN);
}

async function stopAllManualOutputs(disarm = true) {
  await stopMotorTest(false);
  await stopCurrentCalibrationDrive(false);
  if (disarm && writer) await sendFrame(MSG.STOP_RUN);
}

function renderCharacterizationResult(result) {
  const rpm = radiansPerSecond => radiansPerSecond * 60 / (2 * Math.PI);
  const detectedLimit = Math.min(Math.abs(result.maxForward), Math.abs(result.maxReverse));
  const configuredVmax = Number(settings.vmax);
  const characterizedVmax = Number.isFinite(configuredVmax)
      ? Math.min(configuredVmax, detectedLimit)
      : detectedLimit;
  $("resultStartDutyForward").textContent = `${(result.startForward * 100).toFixed(1)}%`;
  $("resultStartDutyReverse").textContent = `${(result.startReverse * 100).toFixed(1)}%`;
  $("resultMaxVelocityForward").textContent = result.maxForward.toFixed(2);
  $("resultMaxVelocityReverse").textContent = result.maxReverse.toFixed(2);
  $("resultMaxRpmForward").textContent = `(${rpm(result.maxForward).toFixed(0)} rpm)`;
  $("resultMaxRpmReverse").textContent = `(${rpm(result.maxReverse).toFixed(0)} rpm)`;
  $("resultCharacterizedVmax").textContent = formatNumber(characterizedVmax, 2);
  $("resultCharacterizedVmaxDetail").textContent = Number.isFinite(configuredVmax) &&
      characterizedVmax < configuredVmax
      ? `clamped from ${formatNumber(configuredVmax, 2)} rad/s`
      : "configured vmax already lower";
  const safetyFactor = Number(settings.characterizationSafetyFactor) || 0.70;
  const recommendedAcceleration = safetyFactor * Math.min(
    Math.abs(result.accelerationForward), Math.abs(result.accelerationReverse));
  const recommendedJerk = safetyFactor * Math.min(
    Math.abs(result.jerkForward), Math.abs(result.jerkReverse));
  const accelerationValid = Number.isFinite(recommendedAcceleration) && recommendedAcceleration > 0;
  const jerkValid = Number.isFinite(recommendedJerk) && recommendedJerk > 0;
  $("resultAccelerationForward").textContent = accelerationValid ? formatNumber(result.accelerationForward, 2) : "—";
  $("resultAccelerationReverse").textContent = accelerationValid ? formatNumber(result.accelerationReverse, 2) : "—";
  $("resultJerkForward").textContent = jerkValid ? formatNumber(result.jerkForward, 1) : "—";
  $("resultJerkReverse").textContent = jerkValid ? formatNumber(result.jerkReverse, 1) : "—";
  $("resultModelGainForward").textContent = formatNumber(result.modelGainForward, 2);
  $("resultModelGainReverse").textContent = formatNumber(result.modelGainReverse, 2);
  $("resultModelTimeForward").textContent = formatNumber(result.modelTimeConstantForward, 4);
  $("resultModelTimeReverse").textContent = formatNumber(result.modelTimeConstantReverse, 4);
  $("resultRecommendedAcceleration").textContent = accelerationValid ? formatNumber(recommendedAcceleration, 2) : "—";
  $("resultRecommendedJerk").textContent = jerkValid ? formatNumber(recommendedJerk, 1) : "—";
  const accelerationWouldLower = accelerationValid && recommendedAcceleration < Number(settings.amax);
  const jerkWouldLower = jerkValid && recommendedJerk < Number(settings.jmax);
  $("applyCharacterizedAcceleration").checked = false;
  $("applyCharacterizedAcceleration").disabled = !accelerationWouldLower;
  $("applyCharacterizedJerk").checked = false;
  $("applyCharacterizedJerk").disabled = !jerkWouldLower;
  $("resultAccelerationRecommendationDetail").textContent = accelerationWouldLower
    ? `current ${formatNumber(settings.amax, 2)} rad/s² · ${formatNumber(safetyFactor * 100, 0)}% safety factor`
    : "configured acceleration is already lower";
  $("resultJerkRecommendationDetail").textContent = jerkWouldLower
    ? `current ${formatNumber(settings.jmax, 1)} rad/s³ · ${formatNumber(safetyFactor * 100, 0)}% safety factor`
    : "configured jerk is already lower";
  $("saveCharacterization").disabled = false;
  $("discardCharacterization").disabled = false;
  characterizationRunning = false;
  $("characterizationProgress").classList.remove("hidden");
  $("characterizationRunState").textContent = "● Complete";
  $("characterizationRunState").className = "status-badge online";
  $("characterizationStageText").textContent = "Measurement complete. Review the candidate values before saving.";
  $("abortCharacterization").disabled = true;
  const dialog = $("characterizationResultDialog");
  if (!dialog.open) dialog.showModal();
}

function renderCharacterizationStatus(status) {
  const stages = ["Stopped", "Finding forward breakaway PWM", "Pausing before reverse test", "Finding reverse breakaway PWM", "Pausing before forward maximum", "Measuring maximum forward velocity", "Pausing before reverse maximum", "Measuring maximum reverse velocity"];
  const wasRunning = characterizationRunning;
  characterizationRunning = status.stage !== 0;
  if (characterizationRunning && !wasRunning) characterizationSamples = [];
  const panel = $("characterizationProgress");
  panel.classList.remove("hidden");
  $("characterizationStageText").textContent = status.resultPending ? "Measurement complete. Review the candidate values before saving." : (stages[status.stage] ?? `Stage ${status.stage}`);
  $("characterizationVelocity").textContent = status.velocity.toFixed(2);
  $("characterizationDuty").textContent = (status.duty * 100).toFixed(1);
  $("characterizationRunState").textContent = characterizationRunning ? "● Running" : (status.resultPending ? "● Complete" : "■ Stopped");
  $("characterizationRunState").className = `status-badge ${characterizationRunning || status.resultPending ? "online" : "offline"}`;
  $("abortCharacterization").disabled = !connected || !characterizationRunning;
}

async function submitCharacterizationAction(action) {
  if (characterizationAction) return;
  characterizationAction = action;
  $("saveCharacterization").disabled = true;
  $("discardCharacterization").disabled = true;
  const flags = action === "save"
    ? 1 | ($("applyCharacterizedAcceleration").checked ? 2 : 0) |
        ($("applyCharacterizedJerk").checked ? 4 : 0)
    : 0;
  await sendFrame(MSG.CHARACTERIZATION_ACTION, Uint8Array.of(flags));
}

function drawChart() {
  const canvas = $("responseChart"); const ctx = canvas.getContext("2d");
  const rect = canvas.getBoundingClientRect(); const dpr = window.devicePixelRatio || 1;
  if (canvas.width !== Math.round(rect.width * dpr)) { canvas.width = Math.round(rect.width * dpr); canvas.height = Math.round(380 * dpr); }
  const w = canvas.width, h = canvas.height; ctx.clearRect(0, 0, w, h); ctx.strokeStyle = "#22303a"; ctx.lineWidth = dpr;
  for (let i = 1; i < 5; i++) { const y = h * i / 5; ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke(); }
  const recent = samples.slice(-1000); if (recent.length < 2) return requestAnimationFrame(drawChart);
  const max = Math.max(10, ...recent.flatMap(s => [Math.abs(s.desired), Math.abs(s.measured)])) * 1.1;
  const plot = (key, color) => { ctx.strokeStyle = color; ctx.lineWidth = 2 * dpr; ctx.beginPath(); recent.forEach((s, i) => { const x = i * w / (recent.length - 1); const y = h / 2 - s[key] / max * h * .45; i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); }); ctx.stroke(); };
  plot("desired", "#f0b84a"); plot("measured", "#4ec3e0"); requestAnimationFrame(drawChart);
}

function drawCharacterizationChart() {
  const canvas = $("characterizationChart"); const ctx = canvas.getContext("2d");
  const rect = canvas.getBoundingClientRect(); const dpr = window.devicePixelRatio || 1;
  if (canvas.width !== Math.round(rect.width * dpr)) { canvas.width = Math.round(rect.width * dpr); canvas.height = Math.round(300 * dpr); }
  const w = canvas.width, h = canvas.height; ctx.clearRect(0, 0, w, h); ctx.strokeStyle = "#22303a"; ctx.lineWidth = dpr;
  for (let i = 1; i < 5; i++) { const y = h * i / 5; ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke(); }
  const recent = characterizationSamples.slice(-1200);
  if (recent.length > 1) {
    const maximum = Math.max(5, ...recent.map(sample => Math.abs(sample.measured))) * 1.1;
    ctx.strokeStyle = "#4ec3e0"; ctx.lineWidth = 2 * dpr; ctx.beginPath();
    recent.forEach((sample, index) => { const x = index * w / (recent.length - 1); const y = h / 2 - sample.measured / maximum * h * .45; index ? ctx.lineTo(x, y) : ctx.moveTo(x, y); });
    ctx.stroke();
  }
  requestAnimationFrame(drawCharacterizationChart);
}

function drawProfileTestChart() {
  const canvas = $("profileTestChart"), ctx = canvas.getContext("2d");
  const rect = canvas.getBoundingClientRect(), dpr = window.devicePixelRatio || 1;
  if (canvas.width !== Math.round(rect.width * dpr)) { canvas.width = Math.round(rect.width * dpr); canvas.height = Math.round(300 * dpr); }
  const w = canvas.width, h = canvas.height; ctx.clearRect(0, 0, w, h); ctx.strokeStyle = "#22303a"; ctx.lineWidth = dpr;
  for (let index = 1; index < 5; index++) { const y = h * index / 5; ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke(); }
  const recent = profileTestSamples.slice(-1200);
  if (recent.length > 1) {
    const maximum = Math.max(5, ...recent.flatMap(sample => [Math.abs(sample.desired), Math.abs(sample.measured)])) * 1.1;
    const plot = (key, color) => { ctx.strokeStyle = color; ctx.lineWidth = 2 * dpr; ctx.beginPath(); recent.forEach((sample, index) => { const x = index * w / (recent.length - 1); const y = h / 2 - sample[key] / maximum * h * .45; index ? ctx.lineTo(x, y) : ctx.moveTo(x, y); }); ctx.stroke(); };
    plot("desired", "#f0b84a"); plot("measured", "#4ec3e0");
  }
  requestAnimationFrame(drawProfileTestChart);
}

function drawTuningChart() {
  const canvas = $("tuningChart"), ctx = canvas.getContext("2d");
  const rect = canvas.getBoundingClientRect(), dpr = window.devicePixelRatio || 1;
  if (canvas.width !== Math.round(rect.width * dpr)) { canvas.width = Math.round(rect.width * dpr); canvas.height = Math.round(380 * dpr); }
  const w = canvas.width, h = canvas.height; ctx.clearRect(0, 0, w, h); ctx.strokeStyle = "#22303a"; ctx.lineWidth = dpr;
  for (let index = 1; index < 5; index++) { const y = h * index / 5; ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke(); }
  if (tuningSamples.length > 1) {
    const maximum = Math.max(5, ...tuningSamples.flatMap(sample => [Math.abs(sample.desired), Math.abs(sample.measured)])) * 1.1;
    const plot = (key, color) => { ctx.strokeStyle = color; ctx.lineWidth = 2 * dpr; ctx.beginPath(); tuningSamples.forEach((sample, index) => { const x = index * w / (tuningSamples.length - 1); const y = h / 2 - sample[key] / maximum * h * .45; index ? ctx.lineTo(x, y) : ctx.moveTo(x, y); }); ctx.stroke(); };
    plot("desired", "#f0b84a"); plot("measured", "#4ec3e0");
  }
  requestAnimationFrame(drawTuningChart);
}

function prepareTuningCanvas(id, height = 380) {
  const canvas = $(id), ctx = canvas.getContext("2d");
  const rect = canvas.getBoundingClientRect(), dpr = window.devicePixelRatio || 1;
  if (rect.width < 2) return null;
  const width = Math.round(rect.width * dpr), pixelHeight = Math.round(height * dpr);
  if (canvas.width !== width || canvas.height !== pixelHeight) { canvas.width = width; canvas.height = pixelHeight; }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, rect.width, height);
  return { canvas, ctx, width: rect.width, height, left: 54, right: 18, top: 18, bottom: 34 };
}

function drawEmptyChartMessage(chart, message) {
  const { ctx, width, height } = chart;
  ctx.fillStyle = "#9eb0bc";
  ctx.font = "14px system-ui";
  ctx.textAlign = "center";
  ctx.fillText(message, width / 2, height / 2);
  ctx.textAlign = "start";
}

function drawEstimatedStepChart() {
  const chart = prepareTuningCanvas("estimatedStepChart");
  if (!chart) return requestAnimationFrame(drawEstimatedStepChart);
  const { ctx, width, height, left, right, top, bottom } = chart;
  const plotWidth = width - left - right, plotHeight = height - top - bottom;
  ctx.font = "12px system-ui"; ctx.lineWidth = 1; ctx.fillStyle = "#9eb0bc"; ctx.strokeStyle = "#22303a";
  for (let line = 0; line <= 4; line++) {
    const value = 2 - line * 0.5, y = top + plotHeight * line / 4;
    ctx.beginPath(); ctx.moveTo(left, y); ctx.lineTo(left + plotWidth, y); ctx.stroke();
    ctx.fillText(value.toFixed(1), 12, y + 4);
  }
  if (!stepEstimate?.ok || stepEstimate.response.length < 2) {
    drawEmptyChartMessage(chart, stepEstimate?.message ?? "Run a response test to calculate an estimate.");
    return requestAnimationFrame(drawEstimatedStepChart);
  }
  const response = stepEstimate.response;
  const duration = response.at(-1).timeSeconds;
  const xFor = time => left + time / Math.max(duration, Number.EPSILON) * plotWidth;
  const yFor = value => top + (2 - Math.max(0, Math.min(2, value))) / 2 * plotHeight;
  for (let line = 0; line <= 4; line++) {
    const x = left + plotWidth * line / 4;
    ctx.fillStyle = "#9eb0bc"; ctx.fillText(`${formatNumber(duration * line / 4, 2)}s`, x - 10, height - 10);
  }
  ctx.save(); ctx.setLineDash([6, 5]); ctx.strokeStyle = "#9eb0bc"; ctx.beginPath();
  ctx.moveTo(left, yFor(1)); ctx.lineTo(left + plotWidth, yFor(1)); ctx.stroke(); ctx.restore();
  const styles = getComputedStyle(document.documentElement);
  ctx.strokeStyle = styles.getPropertyValue("--estimated-step").trim(); ctx.lineWidth = 2.5; ctx.beginPath();
  response.forEach((point, index) => index ? ctx.lineTo(xFor(point.timeSeconds), yFor(point.value)) : ctx.moveTo(xFor(point.timeSeconds), yFor(point.value))); ctx.stroke();
  requestAnimationFrame(drawEstimatedStepChart);
}

function drawPidOutputChart() {
  const chart = prepareTuningCanvas("pidOutputChart", 520);
  if (!chart) return requestAnimationFrame(drawPidOutputChart);
  const { ctx, width, height, left, right, top, bottom } = chart;
  const styles = getComputedStyle(document.documentElement);
  const series = [
    { key: "pTerm", label: "P Δu", enabled: $("showTuningP").checked, color: styles.getPropertyValue("--controller-p").trim(), dash: [] },
    { key: "iTerm", label: "I Δu", enabled: $("showTuningI").checked, color: styles.getPropertyValue("--controller-i").trim(), dash: [8, 4] },
    { key: "dTerm", label: "D Δu", enabled: $("showTuningD").checked, color: styles.getPropertyValue("--controller-d").trim(), dash: [2, 4] },
    { key: "output", label: "TOTAL u", enabled: $("showTuningTotal").checked, color: styles.getPropertyValue("--controller-total").trim(), dash: [] }
  ].filter(item => item.enabled);
  if (!series.length) {
    drawEmptyChartMessage(chart, "Select at least one controller signal.");
    return requestAnimationFrame(drawPidOutputChart);
  }
  if (tuningSamples.length < 2) {
    drawEmptyChartMessage(chart, "Run a response test to populate controller output telemetry.");
    return requestAnimationFrame(drawPidOutputChart);
  }
  const axisLeft = 104, plotWidth = width - axisLeft - right;
  const laneGap = 8, timeAxisHeight = 26;
  const availableHeight = height - top - bottom - timeAxisHeight;
  const laneHeight = (availableHeight - laneGap * (series.length - 1)) / series.length;
  const duration = (tuningSamples.at(-1).timestamp - tuningSamples[0].timestamp) / 1e6;
  ctx.font = "12px system-ui";
  for (let line = 0; line <= 4; line++) {
    const x = axisLeft + plotWidth * line / 4;
    ctx.fillStyle = "#9eb0bc"; ctx.fillText(`${formatNumber(duration * line / 4, 2)}s`, x - 10, height - 11);
  }
  const axisText = limit => formatNumber(limit, limit < 0.001 ? 6 : limit < 0.1 ? 4 : 2);
  const xFor = index => axisLeft + index * plotWidth / (tuningSamples.length - 1);
  series.forEach((item, laneIndex) => {
    const laneTop = top + laneIndex * (laneHeight + laneGap);
    const laneBottom = laneTop + laneHeight;
    const center = (laneTop + laneBottom) / 2;
    const maximum = Math.max(...tuningSamples.map(sample => Math.abs(sample[item.key] ?? 0)));
    const limit = symmetricNiceLimit(maximum);
    const yFor = value => center - value / limit * laneHeight * 0.42;
    ctx.fillStyle = laneIndex % 2 ? "#0c1218" : "#0a0f14";
    ctx.fillRect(axisLeft, laneTop, plotWidth, laneHeight);
    ctx.strokeStyle = "#27333d"; ctx.lineWidth = 1; ctx.setLineDash([]);
    ctx.strokeRect(axisLeft, laneTop, plotWidth, laneHeight);
    for (let line = 1; line < 4; line++) {
      const x = axisLeft + plotWidth * line / 4;
      ctx.strokeStyle = "#18242d"; ctx.beginPath(); ctx.moveTo(x, laneTop); ctx.lineTo(x, laneBottom); ctx.stroke();
    }
    ctx.strokeStyle = "#27333d";
    ctx.beginPath(); ctx.moveTo(axisLeft, center); ctx.lineTo(axisLeft + plotWidth, center); ctx.stroke();
    ctx.fillStyle = item.color; ctx.font = "600 13px system-ui"; ctx.fillText(item.label, 9, center - 8);
    ctx.fillStyle = "#9eb0bc"; ctx.font = "11px ui-monospace, monospace";
    ctx.fillText(`+${axisText(limit)}`, 9, laneTop + 13);
    ctx.fillText("0", 9, center + 4);
    ctx.fillText(`−${axisText(limit)}`, 9, laneBottom - 5);
    ctx.strokeStyle = item.color; ctx.lineWidth = item.key === "output" ? 2.5 : 2; ctx.setLineDash(item.dash); ctx.beginPath();
    tuningSamples.forEach((sample, index) => index ? ctx.lineTo(xFor(index), yFor(sample[item.key] ?? 0)) : ctx.moveTo(xFor(index), yFor(sample[item.key] ?? 0)));
    ctx.stroke(); ctx.setLineDash([]);
  });
  requestAnimationFrame(drawPidOutputChart);
}

function appendTerminal(text) { textRx += text; if (textRx.length > 50000) textRx = textRx.slice(-50000); $("terminalOutput").textContent = textRx; $("terminalOutput").scrollTop = $("terminalOutput").scrollHeight; }
function toast(message) { const node = $("toast"); node.textContent = message; node.classList.add("visible"); clearTimeout(toast.timer); toast.timer = setTimeout(() => node.classList.remove("visible"), 3000); }
async function confirmSafety(message) { $("safetyMessage").textContent = message; const dialog = $("safetyDialog"); dialog.showModal(); return new Promise(resolve => dialog.addEventListener("close", () => resolve(dialog.returnValue === "confirm"), { once: true })); }

document.querySelectorAll(".tab").forEach(button => button.addEventListener("click", () => { if (motorTestActive && button.dataset.page !== "motorTest") stopMotorTest(true); if (currentCalibrationDriveActive && button.dataset.page !== "calibration") stopCurrentCalibrationDrive(true); document.querySelectorAll(".tab,.page").forEach(node => node.classList.remove("active")); button.classList.add("active"); $(button.dataset.page).classList.add("active"); }));
$("newProfile").addEventListener("click", openNewProfileEditor);
$("profileRows").addEventListener("click", event => {
  const button = event.target.closest(".edit-profile");
  if (button) openProfileEditor(Number(button.dataset.profileId));
});
$("exportParameters").addEventListener("click", exportParameterCsv);
$("importParameters").addEventListener("click", () => $("parameterCsvFile").click());
$("parameterCsvFile").addEventListener("change", async event => {
  const [file] = event.target.files;
  await reviewParameterCsvFile(file);
  event.target.value = "";
});
$("parameterImportForm").addEventListener("submit", async event => {
  if (event.submitter?.value !== "apply") return;
  event.preventDefault();
  if (!parameterImportDraft?.length || parameterImportAction) return;
  const preparation = configurationPreparation(latestState, characterizationRunning, "parameter import");
  if (preparation.blocked) return toast(preparation.blocked);
  parameterImportAction = {
    entries: parameterImportDraft,
    index: 0,
    stage: preparation.firstCommand === "stop" ? "stop" : "apply"
  };
  $("applyParameterImport").disabled = true;
  $("cancelParameterImport").disabled = true;
  $("importParameters").disabled = true;
  if (parameterImportAction.stage === "stop") {
    $("parameterImportProgress").textContent = "Stopping and disarming before import…";
    try {
      await sendFrame(MSG.STOP_RUN);
    } catch (error) {
      failParameterImport(`Could not request disarm: ${error.message}`);
    }
  } else {
    sendNextParameterImport();
  }
});
$("parameterImportDialog").addEventListener("close", () => {
  if (!parameterImportAction) parameterImportDraft = undefined;
});
$("parameterImportDialog").addEventListener("cancel", event => {
  if (parameterImportAction) event.preventDefault();
});
$("parameterRows").addEventListener("click", event => {
  const button = event.target.closest(".parameter-value");
  if (button && !button.classList.contains("readonly")) openParameterEditor(button.dataset.parameter);
});
$("parameterEditorForm").addEventListener("submit", async event => {
  if (event.submitter?.value !== "save") return;
  event.preventDefault();
  const definition = parameterDefinitions[editingParameterKey], value = Number($("parameterEditorValue").value);
  if (!definition || !Number.isFinite(value) || value < definition.min || value > definition.max || (editingParameterKey === "direction" && value !== -1 && value !== 1)) return toast("Enter a value inside the allowed range.");
  const payload = new Uint8Array(7), view = new DataView(payload.buffer);
  view.setUint16(0, definition.id, true); view.setFloat32(2, value, true); payload[6] = 1;
  const preparation = configurationPreparation(latestState, characterizationRunning, "parameters");
  if (preparation.blocked) return toast(preparation.blocked);
  if (preparation.firstCommand === "stop") {
    pendingParameterPayload = payload;
    toast("Stopping and disarming before applying the parameter…");
    await sendFrame(MSG.STOP_RUN);
  } else {
    await sendFrame(MSG.SET_PARAMETER, payload);
  }
});
$("profileDuration").addEventListener("change", () => {
  if (!editingProfile) return;
  const duration = Math.min(3600, Math.max(0.1, Number($("profileDuration").value) || editingProfile.duration));
  const scale = duration / editingProfile.duration;
  editingProfile.points.forEach(point => { point.time *= scale; });
  editingProfile.duration = duration; $("profileDuration").value = formatNumber(duration, 2); updateProfileEditor();
});
$("profilePointTime").addEventListener("change", () => setSelectedProfilePoint(Number($("profilePointTime").value), editingProfile.points[selectedProfilePoint].velocity));
$("profilePointVelocity").addEventListener("change", () => setSelectedProfilePoint(editingProfile.points[selectedProfilePoint].time, Number($("profilePointVelocity").value), false));
$("addProfilePoint").addEventListener("click", () => {
  const point = editingProfile.points[selectedProfilePoint];
  const next = editingProfile.points[Math.min(selectedProfilePoint + 1, editingProfile.points.length - 1)];
  const previous = editingProfile.points[Math.max(0, selectedProfilePoint - 1)];
  const neighbor = selectedProfilePoint < editingProfile.points.length - 1 ? next : previous;
  addProfilePoint((point.time + neighbor.time) / 2, (point.velocity + neighbor.velocity) / 2);
});
$("removeProfilePoint").addEventListener("click", () => {
  if (selectedProfilePoint <= 0 || selectedProfilePoint >= editingProfile.points.length - 1) return;
  editingProfile.points.splice(selectedProfilePoint, 1); selectedProfilePoint = Math.max(0, selectedProfilePoint - 1); updateProfileEditor();
});
$("saveProfile").addEventListener("click", () => submitProfile("save"));
$("runProfileTest").addEventListener("click", () => submitProfile("run"));
$("stopProfileTest").addEventListener("click", () => sendFrame(MSG.STOP_RUN));
$("profileEditorChart").addEventListener("pointerdown", event => {
  if (!editingProfile) return;
  const g = profileChartGeometry(), x = event.clientX - g.rect.left, y = event.clientY - g.rect.top;
  const xFor = time => g.left + time / editingProfile.duration * g.width;
  const yFor = velocity => g.top + (1 - velocity / Number(settings.vmax)) * g.height;
  const nearest = editingProfile.points.map((point, index) => ({ index, distance: Math.hypot(x - xFor(point.time), y - yFor(point.velocity)) })).sort((a, b) => a.distance - b.distance)[0];
  if (nearest?.distance <= 14) { selectedProfilePoint = nearest.index; draggedProfilePoint = nearest.index; updateProfileEditor(); }
  else {
    addProfilePoint((x - g.left) / g.width * editingProfile.duration, (1 - (y - g.top) / g.height) * Number(settings.vmax));
    draggedProfilePoint = selectedProfilePoint;
  }
  $("profileEditorChart").setPointerCapture(event.pointerId);
});
$("profileEditorChart").addEventListener("pointermove", event => {
  if (draggedProfilePoint < 0 || !editingProfile) return;
  const g = profileChartGeometry(), x = event.clientX - g.rect.left, y = event.clientY - g.rect.top;
  selectedProfilePoint = draggedProfilePoint;
  setSelectedProfilePoint((x - g.left) / g.width * editingProfile.duration, (1 - (y - g.top) / g.height) * Number(settings.vmax));
  draggedProfilePoint = selectedProfilePoint;
});
const finishProfileDrag = () => { draggedProfilePoint = -1; };
$("profileEditorChart").addEventListener("pointerup", finishProfileDrag);
$("profileEditorChart").addEventListener("pointercancel", finishProfileDrag);
$("profileEditorChart").addEventListener("keydown", event => {
  if (!editingProfile || !["ArrowLeft", "ArrowRight", "ArrowUp", "ArrowDown", "Delete"].includes(event.key)) return;
  event.preventDefault();
  if (event.key === "Delete") return $("removeProfilePoint").click();
  const point = editingProfile.points[selectedProfilePoint];
  const time = point.time + (event.key === "ArrowLeft" ? -0.01 : event.key === "ArrowRight" ? 0.01 : 0);
  const velocity = point.velocity + (event.key === "ArrowDown" ? -0.1 : event.key === "ArrowUp" ? 0.1 : 0);
  setSelectedProfilePoint(time, velocity);
});
$("connectButton").addEventListener("click", connect);
$("baud").value = String(readBaudPreference(localPreferenceStorage()));
$("baud").addEventListener("change", () => {
  writeBaudPreference(localPreferenceStorage(), $("baud").value);
});
$("stopButton").addEventListener("click", () => stopAllManualOutputs(true));
$("armButton").addEventListener("click", async () => { if (await confirmSafety("Verify the rotor guard is closed, the emergency stop works, and the area is clear. Arming permits PWM output.")) await sendAscii("arm"); });
$("runButton").addEventListener("click", () => {
  $("runProfileStatus").textContent = "";
  renderRunProfileDialog();
  $("runProfileDialog").showModal();
});
$("runDialogArm").addEventListener("click", async () => {
  if (!await confirmSafety("Verify the configured load positions match the physical rotor, the guard is closed, and the emergency stop is accessible.")) return;
  overviewRunAction = { stage: "arm" };
  renderRunProfileDialog();
  await sendFrame(MSG.ARM);
});
$("confirmRunProfile").addEventListener("click", async () => {
  const profileId = Number($("runProfileSelect").value);
  if (!Number.isInteger(profileId) || latestState !== 1 || overviewRunAction) return;
  overviewRunAction = { stage: "run-select", profileId };
  $("runProfileStatus").textContent = "Selecting profile for this run…";
  renderRunProfileDialog();
  const payload = new Uint8Array(2); new DataView(payload.buffer).setUint16(0, profileId, true);
  await sendFrame(MSG.SELECT_PROFILE, payload);
});
$("setDefaultProfile").addEventListener("click", async () => {
  const profileId = Number($("runProfileSelect").value);
  if (!Number.isInteger(profileId) || latestState !== 0 || defaultProfilePending !== undefined) return;
  defaultProfilePending = profileId;
  renderRunProfileDialog();
  const payload = new Uint8Array(2); new DataView(payload.buffer).setUint16(0, profileId, true);
  await sendFrame(MSG.SET_DEFAULT_PROFILE, payload);
});
$("runProfileSelect").addEventListener("change", () => {
  $("runProfileStatus").textContent = Number($("runProfileSelect").value) === settings.profileId
    ? "Saved default selected." : "Temporary selection; saved default is unchanged.";
});
$("loadSlotStrength").addEventListener("input", () => {
  $("loadSlotStrengthValue").value = $("loadSlotStrength").value;
  $("loadSlotStrengthValue").textContent = $("loadSlotStrength").value;
});
$("applyLoadSlot").addEventListener("click", async () => {
  const strength = Number($("loadSlotStrength").value);
  loadConfiguration = normalizeRotorLoads([
    ...loadConfiguration.filter(load => load.slot !== editingLoadSlot),
    { slot: editingLoadSlot, strength }
  ]);
  $("loadSlotDialog").close("apply");
  renderRotorLoadSetup();
  await persistLoadConfiguration();
});
$("removeLoadSlot").addEventListener("click", async () => {
  loadConfiguration = loadConfiguration.filter(load => load.slot !== editingLoadSlot);
  $("loadSlotDialog").close("remove");
  renderRotorLoadSetup();
  await persistLoadConfiguration();
});
$("loadGains").addEventListener("click", async () => {
  if (pendingGains) return;
  if (latestState === 2 || latestState === 3 || characterizationRunning) return toast("Stop the machine and clear faults before applying gains.");
  const gains = readGainInputs();
  if (!gains) return toast("Kp, Ki, and Kd must be finite non-negative values.");
  pendingGains = gains;
  $("loadGains").disabled = true;
  await sendFrame(MSG.SET_CONTROLLER, encodeGains(gains));
});
$("saveGains").addEventListener("click", async () => {
  if (!testedGains || pendingGainSavePayload || tuningTestActive || tuningAction) return;
  pendingGainSavePayload = encodeGains(testedGains);
  $("saveGains").disabled = true;
  await sendFrame(MSG.STOP_RUN);
});
["kp", "ki", "kd"].forEach(id => $(id).addEventListener("input", () => {
  gainDraftDirty = true;
  testedGains = undefined;
  $("saveGains").disabled = true;
  setGainState("● Draft — not applied", "offline");
}));
function renderTuningTestInputMode(mode) {
  const profileControls = $("tuningProfileControls");
  const manualControls = $("tuningManualControls");
  profileControls.hidden = mode !== "profile";
  manualControls.hidden = mode !== "manual";
}
document.querySelectorAll('input[name="tuningTestMode"]').forEach(input => input.addEventListener("change", () => {
  if (input.checked) renderTuningTestInputMode(input.value);
}));
renderTuningTestInputMode(document.querySelector('input[name="tuningTestMode"]:checked').value);
$("startTuningTest").addEventListener("click", startTuningResponseTest);
$("stopTuningTest").addEventListener("click", () => sendFrame(MSG.STOP_RUN));
for (const id of ["stepFftWindow", "stepResponseDuration", "stepRegularization", "stepCutoffHz", "stepMinimumExcitation"]) $(id).addEventListener("change", updateStepEstimate);
$("saveConfig").addEventListener("click", () => sendAscii("config save"));
$("saveDriverDiagnostic").addEventListener("click", async () => { const payload = Uint8Array.of($("driverDiagnosticEnabled").checked ? 1 : 0); await sendFrame(MSG.SET_DRIVER_DIAGNOSTIC, payload); });
$("saveCurrentSense").addEventListener("click", async () => { const payload = Uint8Array.of($("currentSenseEnabled").checked ? 1 : 0); await sendFrame(MSG.SET_CURRENT_SENSE, payload); });
$("clearFaultButton").addEventListener("click", async () => { await stopAllManualOutputs(true); await sendFrame(MSG.CLEAR_FAULTS); });
$("startEncoderCalibration").addEventListener("click", startEncoderCalibrationMeasurement);
$("finishEncoderCalibration").addEventListener("click", finishEncoderCalibrationMeasurement);
$("cancelEncoderCalibration").addEventListener("click", () => resetEncoderCalibration("● Measurement discarded"));
$("saveEncoderCalibration").addEventListener("click", saveEncoderCalibrationResult);
$("currentCalibrationDutyRange").addEventListener("input", event => setCurrentCalibrationDuty(event.target.value));
$("currentCalibrationDuty").addEventListener("change", event => setCurrentCalibrationDuty(event.target.value));
$("startCurrentCalibrationDrive").addEventListener("click", startCurrentCalibrationDrive);
$("stopCurrentCalibrationDrive").addEventListener("click", () => stopCurrentCalibrationDrive(true));
$("captureCurrentPoint1").addEventListener("click", () => {
  const reference = currentReferenceValue("currentReferencePoint1");
  if (reference !== undefined) sendCurrentCalibrationCommand(CURRENT_CALIBRATION_ACTION.CAPTURE_POINT_1, reference);
});
$("captureCurrentPoint2").addEventListener("click", () => {
  const reference = currentReferenceValue("currentReferencePoint2");
  if (reference !== undefined) sendCurrentCalibrationCommand(CURRENT_CALIBRATION_ACTION.CAPTURE_POINT_2, reference);
});
$("resetCurrentCalibration").addEventListener("click", () => sendCurrentCalibrationCommand(CURRENT_CALIBRATION_ACTION.RESET));
$("cancelCurrentCalibration").addEventListener("click", () => sendCurrentCalibrationCommand(CURRENT_CALIBRATION_ACTION.CANCEL));
$("saveCurrentCalibration").addEventListener("click", () => sendCurrentCalibrationCommand(CURRENT_CALIBRATION_ACTION.SAVE));
$("openVinCalibration").addEventListener("click", () => { const measured = Number($("supplyVoltage").textContent); $("vinDialogMeasured").textContent = measured.toFixed(3); $("vinReferenceVoltage").value = measured > 0 ? measured.toFixed(3) : "12.000"; $("vinCalibrationDialog").showModal(); $("vinReferenceVoltage").focus(); });
$("vinCalibrationForm").addEventListener("submit", async event => { if (event.submitter?.value !== "confirm") return; event.preventDefault(); const reference = Number($("vinReferenceVoltage").value); if (!(reference > 0 && reference <= 20)) return toast("Reference voltage must be between 0 and 20 V."); const payload = new Uint8Array(4); new DataView(payload.buffer).setFloat32(0, reference, true); await sendFrame(MSG.SUPPLY_VOLTAGE_CALIBRATION, payload); $("vinCalibrationDialog").close("confirm"); });
$("characterizeButton").addEventListener("click", async () => { if (await confirmSafety("Remove all imbalance weights and disconnect the mechanical load. The motor will sweep both directions and reach maximum configured duty.")) { characterizationSamples = []; characterizationRunning = true; $("characterizationProgress").classList.remove("hidden"); $("characterizationRunState").textContent = "● Starting"; $("characterizationStageText").textContent = "Arming and preparing characterization…"; $("abortCharacterization").disabled = false; $("characterizationProgress").scrollIntoView({ behavior: matchMedia("(prefers-reduced-motion: reduce)").matches ? "auto" : "smooth", block: "start" }); await sendAscii("arm"); await sendAscii("characterize start CONFIRM_UNLOADED"); } });
$("abortCharacterization").addEventListener("click", async () => { await sendAscii("characterize abort"); characterizationRunning = false; $("abortCharacterization").disabled = true; $("characterizationRunState").textContent = "■ Aborted"; $("characterizationRunState").className = "status-badge offline"; $("characterizationStageText").textContent = "Characterization aborted; previous values retained."; });
$("characterizationResultForm").addEventListener("submit", event => { event.preventDefault(); submitCharacterizationAction(event.submitter?.value === "save" ? "save" : "discard"); });
$("characterizationResultDialog").addEventListener("cancel", event => { event.preventDefault(); submitCharacterizationAction("discard"); });
$("motorTestDutyRange").addEventListener("input", event => setMotorTestDuty(event.target.value));
$("motorTestDuty").addEventListener("change", event => setMotorTestDuty(event.target.value));
document.querySelectorAll('input[name="motorTestDirection"]').forEach(input => input.addEventListener("change", () => {
  if (motorTestActive) {
    stopMotorTest(true);
    toast("Direction changed. Motor test stopped and disarmed; confirm safety before restarting.");
  }
}));
$("startMotorTest").addEventListener("click", startMotorTest);
$("stopMotorTest").addEventListener("click", () => stopMotorTest(true));
$("terminalForm").addEventListener("submit", async event => { event.preventDefault(); const input = $("terminalInput"); if (input.value.trim()) { appendTerminal(`> ${input.value}\n`); await sendAscii(input.value.trim()); input.value = ""; } });
$("clearTerminal").addEventListener("click", () => { textRx = ""; $("terminalOutput").textContent = ""; });
$("serialLinkBadge").addEventListener("click", () => $("serialLinkDialog").showModal());
$("exportButton").addEventListener("click", () => {
  if (!runRecorder.samples.length) return;
  $("exportBaseName").value = defaultExportBaseName();
  updateExportFileSummary();
  $("exportDialog").showModal();
  $("exportBaseName").focus();
  $("exportBaseName").select();
});
$("exportBaseName").addEventListener("input", updateExportFileSummary);
$("exportForm").addEventListener("submit", event => {
  if (event.submitter?.value !== "export") return;
  event.preventDefault();
  const baseName = exportBaseName();
  downloadCsv(`${baseName}.csv`, createTelemetryCsv(runRecorder.samples));
  if (recordedLoadConfiguration.length) {
    downloadCsv(`${baseName}-loads.csv`,
      createLoadConfigurationCsv(recordedLoadSettingId, recordedLoadConfiguration));
  }
  $("exportDialog").close("export");
  toast(recordedLoadConfiguration.length
    ? "Run data and rotor load information exported."
    : "Run data exported.");
});
window.addEventListener("beforeunload", () => { clearInterval(motorTestTimer); clearInterval(currentCalibrationDriveTimer); if (connected) sendFrame(MSG.STOP_RUN); });
setMotorTestDuty(0.10);
setCurrentCalibrationDuty(0.10);
drawChart();
drawCharacterizationChart();
drawProfileTestChart();
drawTuningChart();
drawEstimatedStepChart();
drawPidOutputChart();
renderRotorLoadSetup();
renderRunProfileDialog();
resetCommunicationDisplay(false);
window.setInterval(renderCommunicationMetrics, 1000);
