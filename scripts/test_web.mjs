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
assert.match(appSource, /typeof settings\[key\] === "boolean"\s*\?\s*\(settings\[key\] \? 1 : 0\)/);
assert.match(indexSource, /id="captureCurrentPoint1"/);
assert.match(indexSource, /id="captureCurrentPoint2"/);
assert.match(indexSource, /id="saveCurrentCalibration"/);
assert.match(indexSource, /id="cancelCurrentCalibration"/);
assert.match(indexSource, /id="currentCalibrationWorkspace"/);
assert.match(indexSource, /id="startCurrentCalibrationDrive"/);
assert.match(indexSource, /id="stopCurrentCalibrationDrive"/);
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
