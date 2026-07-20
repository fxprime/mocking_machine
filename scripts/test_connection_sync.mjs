import assert from "node:assert/strict";
import { DeviceSynchronizer } from "../web/device-synchronizer.mjs";

const synchronizer = new DeviceSynchronizer(1000);
assert.equal(synchronizer.shouldRequest(0), true, "Connection must request initial settings");
synchronizer.markRequested(0);
assert.equal(synchronizer.shouldRequest(999), false, "Retries must be rate limited");
assert.equal(synchronizer.shouldRequest(1000), true, "A lost boot-time request must retry");
synchronizer.markRequested(1000);
synchronizer.markTelemetryReceived();
assert.equal(synchronizer.shouldRequest(5000), false, "Live telemetry completes synchronization");
synchronizer.reset();
assert.equal(synchronizer.shouldRequest(5001), true, "Reconnect must start a new handshake");

console.log("Connection synchronization tests passed");
