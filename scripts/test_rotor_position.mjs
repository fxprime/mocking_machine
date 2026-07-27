import assert from "node:assert/strict";
import {
  normalizeDegrees,
  pointerAngleDegrees,
  updateRotorVisualState
} from "../web/rotor-position.mjs";
import {
  rotorPositionInteractionState
} from "../web/rotor-position-interaction.mjs";

let state = updateRotorVisualState(undefined, 350, 350n, 360, 1);
state = updateRotorVisualState(state, 2, 362n, 360, 1);
assert.equal(state.unwrappedAngleDeg, 362,
  "350° to 2° must continue clockwise rather than rewind");

state = updateRotorVisualState(state, 5, 725n, 360, 1);
assert.equal(state.unwrappedAngleDeg, 725,
  "Encoder counts must preserve complete turns skipped between telemetry frames");

let reverse = updateRotorVisualState(undefined, 10, 100n, 360, -1);
reverse = updateRotorVisualState(reverse, 358, 112n, 360, -1);
assert.equal(reverse.unwrappedAngleDeg, -2,
  "Reverse encoder motion must continue counter-clockwise through zero");

assert.equal(pointerAngleDegrees(50, 0, 50, 50), 0,
  "Pointer above the dial must select zero degrees");
assert.equal(pointerAngleDegrees(100, 50, 50, 50), 90,
  "Pointer right of the dial must select 90 degrees");
assert.equal(pointerAngleDegrees(50, 100, 50, 50), 180,
  "Pointer below the dial must select 180 degrees");
assert.equal(pointerAngleDegrees(0, 50, 50, 50), 270,
  "Pointer left of the dial must select 270 degrees");
assert.equal(normalizeDegrees(-1), 359);

assert.deepEqual(
  rotorPositionInteractionState({
    connected: true,
    referenced: true,
    machineState: 0,
    faults: 0
  }),
  {
    canDrag: true,
    canCommand: false,
    hint: "Drag to preview; arm output to move"
  },
  "A referenced disarmed rotor must allow target preview without commanding motion"
);
assert.equal(
  rotorPositionInteractionState({
    connected: true,
    referenced: true,
    machineState: 1,
    faults: 0
  }).canCommand,
  true,
  "An armed referenced rotor must send position commands"
);
assert.equal(
  rotorPositionInteractionState({
    connected: true,
    referenced: false,
    machineState: 1,
    faults: 0
  }).canDrag,
  false,
  "Position drag must remain disabled until zero reference exists"
);

console.log("Rotor position display tests passed");
