import assert from "node:assert/strict";
import { maximumTelemetryStreamRateHz } from "../web/serial-bandwidth.mjs";

assert.equal(maximumTelemetryStreamRateHz(115200), 84);
assert.equal(maximumTelemetryStreamRateHz(921600), 500);
assert.equal(maximumTelemetryStreamRateHz(9600), 7);

console.log("Serial bandwidth tests passed");
