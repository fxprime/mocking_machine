import assert from "node:assert/strict";
import { machineStatusKey, shouldRefreshMachineUi } from "../web/machine-ui-state.mjs";
import { RunRecorder } from "../web/run-recorder.mjs";

const key = machineStatusKey(0, 0);
assert.equal(shouldRefreshMachineUi(undefined, 0, 0), true);
assert.equal(shouldRefreshMachineUi(key, 0, 0), false,
  "Stable telemetry must not replace interactive rotor-slot elements");
assert.equal(shouldRefreshMachineUi(key, 1, 0), true);

const recorder = new RunRecorder(3);
recorder.consume({ state: 0, id: "before-click" });
assert.equal(recorder.samples.length, 0, "Connection telemetry must not be recorded");
recorder.begin();
recorder.consume({ state: 1, id: "armed" });
recorder.consume({ state: 2, id: "run-1" });
recorder.consume({ state: 2, id: "run-2" });
recorder.consume({ state: 0, id: "after-run" });
recorder.consume({ state: 0, id: "idle" });
assert.deepEqual(recorder.samples.map(sample => sample.id), ["run-1", "run-2"]);
recorder.begin();
assert.equal(recorder.samples.length, 0, "A new Run command starts a new recording");

console.log("Run recording and UI render-gate tests passed");
