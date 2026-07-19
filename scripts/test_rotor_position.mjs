import assert from "node:assert/strict";
import { updateRotorVisualState } from "../web/rotor-position.mjs";

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

console.log("Rotor position display tests passed");
