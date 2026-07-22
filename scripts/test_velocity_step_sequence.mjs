import assert from "node:assert/strict";
import {
  createVelocityStepSequence,
  encodeVelocityStepSequence,
  MAXIMUM_VELOCITY_LEVELS
} from "../web/velocity-step-sequence.mjs";

const options = {
  minimumVelocity: 5,
  maximumVelocity: 25,
  levelCount: 10,
  holdSeconds: 0.75,
  velocityLimit: 30,
  seed: 0x12345678
};
const first = createVelocityStepSequence(options);
const repeated = createVelocityStepSequence(options);
assert.deepEqual(first, repeated, "A displayed seed must reproduce the same sequence");
assert.equal(first.levels.length, 10);
assert.equal(first.holdMs, 750);
assert.equal(first.durationSeconds, 7.5);
assert.ok(first.levels.every(level => level >= 5 && level <= 25));
assert.ok(first.levels.slice(1).every((level, index) =>
  Math.abs(level - first.levels[index]) >= 4), "Adjacent levels need useful excitation");

const payload = encodeVelocityStepSequence(first);
assert.equal(payload.byteLength, 72);
const view = new DataView(payload.buffer);
assert.equal(view.getUint32(0, true), 750);
assert.equal(payload[4], 10);
assert.ok(Math.abs(view.getFloat32(8, true) - first.levels[0]) < 1e-5);
assert.throws(() => createVelocityStepSequence({ ...options, levelCount: MAXIMUM_VELOCITY_LEVELS + 1 }), /between/);
assert.throws(() => createVelocityStepSequence({ ...options, minimumVelocity: 25 }), /range/);

console.log("Velocity step-sequence tests passed");
