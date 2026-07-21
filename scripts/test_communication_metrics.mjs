import assert from "node:assert/strict";
import { CommunicationMetrics } from "../web/communication-metrics.mjs";

let nowMs = 0;
const metrics = new CommunicationMetrics(() => nowMs);
metrics.recordRxBytes(1000);
metrics.recordTxBytes(500);
metrics.recordRxFrame(100);
metrics.recordRxFrame(500);
metrics.recordTxMessage();
metrics.recordTelemetry(0, 50);
metrics.recordTelemetry(20_000, 50);
metrics.recordTelemetry(80_000, 50);
metrics.recordCrcError();
metrics.recordFramingError();
nowMs = 1000;

const first = metrics.snapshot();
assert.equal(first.rxBytesPerSecond, 1000);
assert.equal(first.txBytesPerSecond, 500);
assert.equal(first.rxFramesPerSecond, 2);
assert.equal(first.txMessagesPerSecond, 1);
assert.equal(first.telemetryHz, 3);
assert.equal(first.telemetryReceived, 3);
assert.equal(first.droppedTelemetry, 2);
assert.ok(Math.abs(first.dropoutPercent - 40) < 1e-9);
assert.equal(first.crcErrors, 1);
assert.equal(first.framingErrors, 1);
assert.equal(first.lastFrameAgeMs, 500);

metrics.recordTelemetry(5_000, 50); // Device timestamp reset must not look like a dropout.
metrics.recordTelemetry(25_000, 100); // Rate change also starts a new comparison baseline.
nowMs = 2000;
const second = metrics.snapshot();
assert.equal(second.rxBytesPerSecond, 0);
assert.equal(second.telemetryHz, 2);
assert.equal(second.telemetryReceived, 5);
assert.equal(second.droppedTelemetry, 2);
assert.equal(second.crcErrors, 1);

metrics.reset();
nowMs = 3000;
const reset = metrics.snapshot();
assert.equal(reset.telemetryReceived, 0);
assert.equal(reset.droppedTelemetry, 0);
assert.equal(reset.lastFrameAgeMs, undefined);

console.log("Communication metrics tests passed");
