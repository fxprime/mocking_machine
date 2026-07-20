import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import { configurationPreparation, profilePreparation, resultDescription } from "../web/protocol-status.mjs";
import { calculateStepMetrics, symmetricNiceLimit } from "../web/response-metrics.mjs";

assert.equal(profilePreparation(0, false).firstCommand, "stop");
assert.equal(profilePreparation(1, false).firstCommand, "stop");
assert.equal(profilePreparation(2, false).firstCommand, "stop");
assert.match(profilePreparation(3, false).blocked, /fault/);
assert.match(configurationPreparation(0, true, "parameters").blocked, /characterization/);
assert.match(resultDescription(4), /safely disarmed/);

const appSource = await readFile(new URL("../web/app.js", import.meta.url), "utf8");
const indexSource = await readFile(new URL("../web/index.html", import.meta.url), "utf8");
const firmwareSource = await readFile(new URL("../src/app/MachineApplication.cpp", import.meta.url), "utf8");
assert.match(indexSource, /parameterRows[^>]*><tr><td colspan="3">Connect to load firmware parameters/);
assert.match(appSource, /Connected; waiting for a valid SETTINGS frame/);
assert.match(appSource, /request === MSG\.SELECT_PROFILE[\s\S]*sendFrame\(MSG\.ARM\)/);
assert.match(appSource, /request === MSG\.ARM[\s\S]*sendFrame\(MSG\.START_RUN\)/);
const startAck = appSource.indexOf("request === MSG.START_RUN && profileAction?.stage === \"start\"");
const closeAfterStart = appSource.indexOf('$("profileEditorDialog").close("run")', startAck);
assert.ok(startAck >= 0 && closeAfterStart > startAck, "Editor must close only after START_RUN ACK");
assert.match(appSource, /currentSenseEnabled:\s*\{\s*id:\s*28,[^}]*min:\s*0,[^}]*max:\s*1/);
assert.match(appSource, /currentFilterCutoffHz:\s*\{\s*id:\s*29,[^}]*min:\s*0\.1,[^}]*max:\s*200/);
assert.match(appSource, /zeroIndexCorrectionGain:\s*\{\s*id:\s*31,[^}]*min:\s*0,[^}]*max:\s*1/);
assert.match(appSource, /zeroIndexMinSeparationRevolutions:\s*\{\s*id:\s*32,[^}]*min:\s*0,[^}]*max:\s*0\.95/);
assert.match(appSource, /Accepted zero #\$\{sample\.zeroSequence\}[^`]*\$\{sample\.zeroRejected\} rejected edges/,
  "Rotor status must distinguish accepted zero events from rejected bounce edges");
assert.match(appSource, /view\.setUint16\(0, parameterDefinitions\.cpr\.id, true\)/,
  "Encoder calibration must save through the validated CPR parameter path");
assert.match(appSource, /typeof settings\[key\] === "boolean"\s*\?\s*\(settings\[key\] \? 1 : 0\)/);
assert.match(indexSource, /id="captureCurrentPoint1"/);
assert.match(indexSource, /id="captureCurrentPoint2"/);
assert.match(indexSource, /id="saveCurrentCalibration"/);
assert.match(indexSource, /id="cancelCurrentCalibration"/);
assert.match(indexSource, /id="currentCalibrationWorkspace"/);
assert.match(indexSource, /id="encoderCalibrationWorkspace"/);
assert.match(indexSource, /id="startEncoderCalibration"/);
assert.match(indexSource, /id="finishEncoderCalibration"/);
assert.match(indexSource, /id="encoderCalibrationResult"/);
assert.match(indexSource, /id="saveEncoderCalibration"/);
assert.match(indexSource, /id="resultCharacterizedVmax"/);
assert.match(appSource, /Math\.min\(configuredVmax, detectedLimit\)/,
  "Characterization review must show the clamped vmax before save");
assert.match(indexSource, /id="startCurrentCalibrationDrive"/);
assert.match(indexSource, /id="stopCurrentCalibrationDrive"/);
assert.match(indexSource, /id="rotorPosition"/, "Overview must show rotor position");
assert.match(indexSource, /id="rotorNeedle"/, "Rotor position must include a visual indicator");
assert.match(indexSource, /id="newProfile"/, "Profiles page must expose a create action");
assert.match(appSource, /CREATE_PROFILE:\s*0x0124/,
  "Profile creation must use a separate message ID instead of changing SET_PROFILE length");
assert.match(appSource, /new Uint8Array\(169\)/,
  "The established SET_PROFILE payload boundary must remain compatible");
assert.match(firmwareSource, /static_assert\(sizeof\(SetProfilePayload\) == 169U/,
  "Firmware must preserve the established profile-update payload boundary");
const motorStartSource = appSource.slice(appSource.indexOf("async function startMotorTest"), appSource.indexOf("async function stopMotorTest"));
assert.doesNotMatch(motorStartSource, /sendAscii\("arm"\)/,
  "Motor test must not mix an unacknowledged ASCII arm with a binary PWM command");
assert.match(motorStartSource, /sendFrame\(MSG\.ARM\)/,
  "Motor test must begin with an acknowledged binary ARM command");
const runOnceSource = firmwareSource.slice(firmwareSource.indexOf("void MachineApplication::runOnce()"), firmwareSource.indexOf("void MachineApplication::controlTick"));
assert.ok(runOnceSource.indexOf("if (now >= next_control_us_)") < runOnceSource.indexOf("serial_link_.poll()"),
  "The 500 Hz control deadline must be serviced before serial commands can activate output");
assert.match(appSource, /rotorPosition:\s*data\.byteLength >= 76 \? data\.getFloat32\(72, true\)/,
  "GUI must decode the appended rotor position telemetry field");
assert.match(appSource, /rotorNeedle[\s\S]*rotate\(/,
  "Rotor indicator must follow the decoded angular position");
assert.match(indexSource, /id="tuningManualControls"[^>]*\shidden(?:\s|>)/,
  "Manual response-test inputs must start hidden while Stored profile is selected");
assert.match(appSource, /function renderTuningTestInputMode[\s\S]*profileControls\.hidden = mode !== "profile"[\s\S]*manualControls\.hidden = mode !== "manual"/,
  "Response-test mode must exclusively reveal its matching input controls");
assert.doesNotMatch(indexSource, /id="currentCalibrationDialog"/);
assert.doesNotMatch(indexSource, /id="motorTestCalibrateCurrent"/);
assert.match(appSource, /CURRENT_CALIBRATION_STATUS:\s*0x0302/);
assert.match(appSource, /new Uint8Array\(5\)/);
const calibrationCase = firmwareSource.slice(firmwareSource.indexOf("case MessageId::CurrentCalibration:"), firmwareSource.indexOf("case MessageId::SupplyVoltageCalibration:"));
assert.doesNotMatch(calibrationCase, /currentSenseVoltage\(64U\)/, "Calibration capture must not block the control loop with a 64-read burst");
assert.match(calibrationCase, /SettingsStore::validate\(candidate\)[\s\S]*transitionToStopped\(\);[\s\S]*settings_store_\.save\(candidate\)/, "Calibration motor must stop before an NVS write");
const metrics = calculateStepMetrics([
  { timestamp: 0, desired: 0, measured: 0 },
  { timestamp: 100000, desired: 10, measured: 1 },
  { timestamp: 300000, desired: 10, measured: 9 },
  { timestamp: 500000, desired: 10, measured: 10.4 },
  { timestamp: 700000, desired: 10, measured: 10.1 }
], 10);
assert.equal(metrics.peak, 10.4);
assert.equal(Math.round(metrics.overshootPercent), 4);
assert.equal(metrics.riseTimeSeconds, 0.2);
assert.equal(metrics.settlingTimeSeconds, 0.5);

assert.equal(symmetricNiceLimit(0.069), 0.1);
assert.equal(symmetricNiceLimit(0.0031), 0.005);
assert.equal(symmetricNiceLimit(0), 0.000001);

console.log("Web control-flow tests passed");
