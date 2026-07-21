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
const companyLogo = await readFile(new URL("../web/assets/modulemore-logo.png", import.meta.url));
assert.match(indexSource, /class="product-footer"[\s\S]*Thanabadee Bulunseechart[\s\S]*Modulemore Co\., Ltd\./,
  "GUI must display the product author and company credit");
assert.match(indexSource, /src="assets\/modulemore-logo\.png"[^>]*alt="Modulemore Co\., Ltd\. logo"/,
  "Company logo must have a stable asset path and accessible alternative text");
assert.ok(companyLogo.byteLength > 0, "Company logo asset must be packaged with the web console");
assert.match(indexSource, /parameterRows[^>]*><tr><td colspan="3">Connect to load firmware parameters/);
assert.match(appSource, /Connected; waiting for a valid SETTINGS frame/);
assert.match(appSource, /id === MSG\.HEARTBEAT[\s\S]*requestDeviceSynchronization/,
  "A valid heartbeat must retry configuration synchronization after boot-time command loss");
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
assert.match(indexSource, /id="resultRecommendedAcceleration"/);
assert.match(indexSource, /id="resultRecommendedJerk"/);
assert.match(indexSource, /id="applyCharacterizedAcceleration"/);
assert.match(indexSource, /id="applyCharacterizedJerk"/);
assert.match(appSource, /accelerationForward:\s*data\.byteLength >= 32 \? data\.getFloat32\(16, true\)/,
  "GUI must decode appended characterization acceleration results");
assert.match(appSource, /jerkReverse:\s*data\.byteLength >= 32 \? data\.getFloat32\(28, true\)/,
  "GUI must decode appended characterization jerk results");
assert.match(appSource, /applyCharacterizedAcceleration[\s\S]*\? 2 : 0[\s\S]*applyCharacterizedJerk[\s\S]*\? 4 : 0/,
  "Acceleration and jerk recommendations must be explicit opt-in action flags");
assert.match(firmwareSource, /static_assert\(sizeof\(CharacterizationResultPayload\) == 48U/,
  "Firmware characterization result payload boundary must be documented in code");
assert.match(appSource, /modelGainForward:\s*data\.byteLength >= 48 \? data\.getFloat32\(32, true\)/,
  "GUI must decode the characterized forward motor-model gain");
assert.match(appSource, /velocityEstimatorMethod:\s*data\.byteLength >= 172 \? data\.getUint8\(171\) : 0/,
  "GUI must decode the velocity-estimator selector");
assert.match(appSource, /velocityAccelerationWindowSamples:\s*data\.byteLength >= 173 \? data\.getUint8\(172\) : 5/,
  "GUI must decode the schema-16 acceleration-history length");
assert.match(appSource, /velocityEstimatorMethod:\s*\{ id: 36,[\s\S]*0 = low-pass,[\s\S]*1 = characterized motor-model Kalman,[\s\S]*2 = encoder-window acceleration prediction/,
  "GUI must expose all three velocity-estimator methods through SET_PARAMETER");
assert.match(appSource, /velocityAccelerationWindowSamples:\s*\{ id: 37,[\s\S]*Circular velocity-history length/,
  "GUI must expose the method-2 circular history length through SET_PARAMETER");
assert.match(indexSource, /id="resultModelTimeReverse"/,
  "Characterization review must display the identified motor-model time constants");
assert.match(appSource, /Math\.min\(configuredVmax, detectedLimit\)/,
  "Characterization review must show the clamped vmax before save");
assert.match(indexSource, /id="startCurrentCalibrationDrive"/);
assert.match(indexSource, /id="stopCurrentCalibrationDrive"/);
assert.match(indexSource, /id="rotorPosition"/, "Overview must show rotor position");
assert.match(indexSource, /id="rotorNeedle"/, "Rotor position must include a visual indicator");
assert.match(indexSource, /id="newProfile"/, "Profiles page must expose a create action");
assert.match(indexSource, /id="runProfileDialog"/);
assert.match(indexSource, /id="setDefaultProfile"/);
assert.match(indexSource, /id="rotorLoadDiagram"/);
assert.match(indexSource, /id="loadSlotDialog"/);
assert.match(indexSource, /class="overview-status-layout"[\s\S]*class="metric-grid overview-metrics"[\s\S]*Machine state[\s\S]*Driver VIN[\s\S]*Desired velocity[\s\S]*Measured velocity[\s\S]*Motor current[\s\S]*Rotor position[\s\S]*class="panel rotor-load-panel"/,
  "Overview must place the ordered 2x3 machine metrics beside the rotor-load panel");
assert.match(indexSource, /id="serialRxRate"/);
assert.match(indexSource, /id="serialTxRate"/);
assert.match(indexSource, /id="serialTelemetryRate"/);
assert.match(indexSource, /id="serialDropouts"/);
assert.match(indexSource, /id="serialIntegrityErrors"/);
assert.match(indexSource, /class="tab-strip"[\s\S]*class="tabs"[\s\S]*class="serial-link-badge"/,
  "Compact serial diagnostics must share the navigation row");
assert.match(indexSource, /id="serialLinkBadge"[\s\S]*serial-summary-row[\s\S]*serial-summary-row/,
  "Serial summary must remain a compact two-row disclosure control");
assert.match(indexSource, /id="serialLinkDialog"[\s\S]*id="serialRxFrames"[\s\S]*id="serialIntegrityErrors"/,
  "Detailed communication measurements must be available in a dialog");
assert.match(appSource, /serialLinkBadge[\s\S]*serialLinkDialog[\s\S]*showModal\(\)/,
  "Clicking the serial badge must open its diagnostics dialog");
assert.doesNotMatch(indexSource, /communication-panel/,
  "Serial diagnostics must not consume an Overview panel");
assert.match(indexSource, /class="product-footer"[\s\S]*width="72" height="25"/,
  "The author credit must use the compact logo treatment");
assert.match(appSource, /communicationMetrics\.recordRxBytes\(value\.byteLength\)/,
  "Serial diagnostics must measure every received byte before parsing");
assert.match(appSource, /communicationMetrics\.recordTxBytes\(bytes\.byteLength\)/,
  "Serial diagnostics must measure successful host writes");
assert.match(appSource, /communicationMetrics\.recordCrcError\(\)/);
assert.match(appSource, /communicationMetrics\.recordFramingError\(\)/);
assert.match(appSource, /communicationMetrics\.recordTelemetry\(sample\.timestamp, settings\.streamRate\)/,
  "Dropout estimation must use device timestamps and the configured telemetry rate");
assert.match(appSource, /SET_DEFAULT_PROFILE:\s*0x0125/);
assert.match(appSource, /SET_LOAD_CONFIGURATION:\s*0x0132/);
assert.match(appSource, /runtimeProfileId = action\.profileId/,
  "Temporary selection must update runtime state without overwriting the saved default");
assert.match(appSource, /shouldRefreshMachineUi\(renderedMachineStatusKey, state, faults\)/,
  "Stable telemetry must not rebuild the interactive rotor diagram");
assert.match(appSource, /runRecorder\.consume\(sample\)/,
  "Telemetry must pass through the run-session recorder");
assert.match(appSource, /createTelemetryCsv\(runRecorder\.samples\)/,
  "CSV export must use only the latest explicitly started run");
assert.match(appSource, /readBaudPreference\(localPreferenceStorage\(\)\)/,
  "The connection selector must restore the last valid browser-local baud rate");
assert.match(appSource, /writeBaudPreference\(localPreferenceStorage\(\), \$\("baud"\)\.value\)/,
  "Changing baud must persist the selection locally");
assert.match(indexSource, /id="exportDialog"/);
assert.match(indexSource, /id="exportBaseName"/);
assert.match(appSource, /createLoadConfigurationCsv\(recordedLoadSettingId, recordedLoadConfiguration\)/,
  "A recorded run with loads must export a separate load-information CSV");
assert.doesNotMatch(indexSource, /id="saveLoadSetup"/,
  "Load editing must not require a separate save button");
assert.match(appSource, /applyLoadSlot[\s\S]*persistLoadConfiguration\(\)/,
  "Applying a slot change must persist immediately");
assert.match(appSource, /removeLoadSlot[\s\S]*persistLoadConfiguration\(\)/,
  "Removing a slot must persist immediately");
assert.match(indexSource, /id="profileCreateReason"/,
  "Profiles page must explain why creation is unavailable");
assert.match(appSource, /PROFILE_ACTION_TIMEOUT_MS[\s\S]*Profile command timed out; controls unlocked/,
  "A lost firmware acknowledgement must not leave profile creation disabled forever");
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
