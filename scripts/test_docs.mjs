import assert from "node:assert/strict";
import { existsSync } from "node:fs";
import { readFile, readdir } from "node:fs/promises";
import path from "node:path";
import { fileURLToPath } from "node:url";

const root = fileURLToPath(new URL("../", import.meta.url));
const source = await readFile(path.join(root, "include/protocol/SerialLink.hpp"), "utf8");
const types = await readFile(path.join(root, "include/core/Types.hpp"), "utf8");
const application = await readFile(path.join(root, "src/app/MachineApplication.cpp"), "utf8");
const browser = await readFile(path.join(root, "web/app.js"), "utf8");
const protocol = await readFile(path.join(root, "docs/protocol.md"), "utf8");
const wiring = await readFile(path.join(root, "docs/wiring.md"), "utf8");
const readme = await readFile(path.join(root, "README.md"), "utf8");

function screamingSnake(name) {
  return name.replace(/([a-z0-9])([A-Z])/g, "$1_$2").toUpperCase();
}

const messageBlock = source.match(/enum class MessageId[^\{]*\{([\s\S]*?)\};/);
assert.ok(messageBlock, "MessageId enum must exist");
const firmwareMessages = new Map();
for (const match of messageBlock[1].matchAll(/(\w+)\s*=\s*(0x[0-9A-Fa-f]+)/g)) {
  firmwareMessages.set(screamingSnake(match[1]), Number(match[2]));
}

const documentedMessages = new Map();
for (const match of protocol.matchAll(/^\| `(0x[0-9A-Fa-f]+)` \| \[([A-Z0-9_]+)\][^|]*\|[^|]*\|\s*(\d+)\s*\|/gm)) {
  documentedMessages.set(match[2], { id: Number(match[1]), bytes: Number(match[3]) });
}
assert.equal(documentedMessages.size, firmwareMessages.size,
  "Protocol summary must contain every firmware message exactly once");
for (const [name, id] of firmwareMessages) {
  assert.equal(documentedMessages.get(name)?.id, id,
    `${name} documentation ID must match firmware`);
}

const browserMessageBlock = browser.match(/const MSG = \{([\s\S]*?)\};/);
assert.ok(browserMessageBlock, "Browser MSG table must exist");
const browserMessages = new Map();
for (const match of browserMessageBlock[1].matchAll(/([A-Z0-9_]+):\s*(0x[0-9A-Fa-f]+)/g)) {
  browserMessages.set(match[1], Number(match[2]));
  assert.equal(firmwareMessages.get(match[1]), Number(match[2]),
    `${match[1]} browser ID must match firmware`);
}
assert.equal(browserMessages.size, firmwareMessages.size,
  "Browser MSG table must contain every firmware message exactly once");

const payloadMessages = new Map([
  ["SettingsPayload", "SETTINGS"],
  ["CurrentCalibrationPayload", "CURRENT_CALIBRATION"],
  ["CurrentCalibrationStatusPayload", "CURRENT_CALIBRATION_STATUS"],
  ["TelemetryPayload", "TELEMETRY"],
  ["SupplyVoltageCalibrationPayload", "SUPPLY_VOLTAGE_CALIBRATION"],
  ["CharacterizationResultPayload", "CHARACTERIZATION_RESULT"],
  ["CharacterizationStatusPayload", "CHARACTERIZATION_STATUS"],
  ["ParameterPayload", "SET_PARAMETER"],
  ["ProfilePayload", "PROFILE_CONFIGURATION"],
  ["SetProfilePayload", "SET_PROFILE"],
  ["LoadConfigurationPayload", "LOAD_CONFIGURATION"]
]);
for (const match of application.matchAll(/static_assert\(sizeof\((\w+)\) == (\d+)U/g)) {
  const message = payloadMessages.get(match[1]);
  if (message) {
    assert.equal(documentedMessages.get(message)?.bytes, Number(match[2]),
      `${message} documented payload size must match its packed static assertion`);
  }
}
assert.equal(documentedMessages.get("CREATE_PROFILE")?.bytes,
  documentedMessages.get("SET_PROFILE")?.bytes,
  "SET_PROFILE and CREATE_PROFILE share the same packed payload");

const parameterBlock = application.match(/enum class ParameterId[^\{]*\{([\s\S]*?)\};/);
assert.ok(parameterBlock, "ParameterId enum must exist");
const firmwareParameters = new Map();
let parameterValue = 0;
for (const match of parameterBlock[1].matchAll(/(?:^|\n)\s*(\w+)(?:\s*=\s*(\d+))?,/g)) {
  parameterValue = match[2] === undefined ? parameterValue + 1 : Number(match[2]);
  firmwareParameters.set(screamingSnake(match[1]), parameterValue);
}
const documentedParameters = new Map();
const documentedParameterSection = protocol.slice(
  protocol.indexOf("## Parameter IDs"), protocol.indexOf("## Operational sequences"));
for (const match of documentedParameterSection.matchAll(/^\|\s*(\d+)\s*\| `([A-Z0-9_]+)` \|/gm)) {
  documentedParameters.set(match[2], Number(match[1]));
}
assert.equal(documentedParameters.size, firmwareParameters.size,
  "Protocol parameter table must contain every firmware parameter exactly once");
for (const [name, id] of firmwareParameters) {
  assert.equal(documentedParameters.get(name), id,
    `${name} documentation ID must match firmware`);
}

const schema = Number(types.match(/kSchemaVersion\s*=\s*(\d+)/)?.[1]);
assert.ok(schema > 0, "Machine settings schema must be discoverable");
assert.match(protocol, new RegExp(`settings schema ${schema}\\b`, "i"));
assert.match(readme, new RegExp(`schema ${schema}\\b`, "i"));

const wireVersion = Number(source.match(/kProtocolVersion\s*=\s*(\d+)U/)?.[1]);
const controlPeriod = Number(types.match(/uint32_t period_us\s*=\s*(\d+)/)?.[1]);
const defaultBaud = Number(types.match(/uint32_t baud\s*=\s*(\d+)/)?.[1]);
const defaultStreamRate = Number(types.match(/uint16_t stream_rate_hz\s*=\s*(\d+)/)?.[1]);
const defaultCpr = Number(types.match(/counts_per_output_revolution\s*=\s*(\d+)/)?.[1]);
assert.match(readme, new RegExp(`Version ${wireVersion} / schema ${schema}`));
assert.match(readme, new RegExp(controlPeriod.toLocaleString("en-US") + " µs"));
assert.match(readme, new RegExp(`${defaultBaud} bit/s / ${defaultStreamRate} Hz`));
assert.match(readme, new RegExp(`${defaultCpr} counts/rev`));

const pins = new Map([
  ["encoder_a", "Encoder A"], ["encoder_b", "Encoder B"],
  ["zero_index", "Zero-index IR sensor"], ["motor_ina", "Motor direction A"],
  ["motor_inb", "Motor direction B"], ["motor_pwm", "Motor PWM"],
  ["current_sense", "Current sense"], ["driver_diag", "Driver diagnostic"],
  ["supply_voltage_sense", "Driver VIN sense"]
]);
for (const [field, label] of pins) {
  const value = types.match(new RegExp(`uint8_t\\s+${field}\\s*=\\s*(\\d+)`))?.[1];
  assert.ok(value, `${field} default must be discoverable`);
  assert.match(wiring, new RegExp(`\\| ${label.replace(/[.*+?^${}()|[\]\\]/g, "\\$&")} \\| ${value} \\|`),
    `${label} wiring row must match firmware pin default`);
}

const markdownFiles = ["README.md", "AGENTS.md",
  ...(await readdir(path.join(root, "docs")))
    .filter(file => file.endsWith(".md")).map(file => path.join("docs", file))];
let mermaidCount = 0;
for (const relative of markdownFiles) {
  const absolute = path.join(root, relative);
  const markdown = await readFile(absolute, "utf8");
  assert.doesNotMatch(markdown, /[└├┬▼►]/,
    `${relative} must use Mermaid instead of ASCII-art diagrams`);
  for (const match of markdown.matchAll(/```mermaid\n([\s\S]*?)\n```/g)) {
    mermaidCount++;
    assert.match(match[1], /^(?:flowchart\s+(?:TB|TD|LR|RL|BT)|stateDiagram-v2)\n/,
      `${relative} Mermaid block must declare a supported diagram type`);
  }
  const headingAnchors = new Set([...markdown.matchAll(/^#{1,6}\s+(.+)$/gm)].map(match =>
    match[1].toLowerCase().replace(/[^a-z0-9 _-]/g, "").trim().replace(/ +/g, "-")));
  for (const match of markdown.matchAll(/\]\(#([^)]+)\)/g)) {
    assert.ok(headingAnchors.has(match[1]),
      `${relative} contains broken internal anchor #${match[1]}`);
  }
  for (const match of markdown.matchAll(/!?\[[^\]]*\]\(([^)]+)\)/g)) {
    const target = match[1].split("#", 1)[0];
    if (!target || /^(?:https?:|mailto:)/.test(target)) continue;
    assert.ok(existsSync(path.resolve(path.dirname(absolute), target)),
      `${relative} links to missing local file ${target}`);
  }
  for (const match of markdown.matchAll(/<img\s+[^>]*src="([^"]+)"/g)) {
    if (/^https?:/.test(match[1])) continue;
    assert.ok(existsSync(path.resolve(path.dirname(absolute), match[1])),
      `${relative} references missing image ${match[1]}`);
  }
}
assert.ok(mermaidCount > 0, "Documentation must retain its Mermaid diagrams");

console.log(`Documentation checks passed: ${firmwareMessages.size} messages, ` +
  `${firmwareParameters.size} parameters, ${pins.size} pins, ${mermaidCount} diagrams.`);
