import assert from "node:assert/strict";
import { estimateClosedLoopStepResponse } from "../web/step-response-estimator.mjs";

function deterministicExcitation(length) {
  let state = 0x5a;
  return Array.from({ length }, () => {
    const feedback = ((state >> 7) ^ (state >> 5) ^ (state >> 4) ^ (state >> 3)) & 1;
    state = ((state << 1) | feedback) & 0xff;
    return (state & 1) ? 1 : -1;
  });
}

const sampleRateHz = 100;
const input = deterministicExcitation(2048);
let output = 0;
const samples = input.map((desired, index) => {
  output += 0.1 * (desired - output);
  return {
    timestamp: index * 1e6 / sampleRateHz,
    desired,
    measured: output
  };
});

const estimate = estimateClosedLoopStepResponse(samples, {
  windowSize: 256,
  responseDurationSeconds: 0.5,
  cutoffHz: 25,
  regularization: 1e-5,
  minimumInputAmplitude: 0.1
});

assert.equal(estimate.ok, true, estimate.message);
assert.equal(estimate.windowSize, 256);
assert.ok(estimate.acceptedWindows > 10);
assert.ok(Math.abs(estimate.sampleRateHz - sampleRateHz) < 0.01);
assert.ok(estimate.response.length >= 49 && estimate.response.length <= 51);
assert.ok(estimate.response[0].value > 0.04 && estimate.response[0].value < 0.2);
assert.ok(estimate.response.at(-1).value > 0.85 && estimate.response.at(-1).value < 1.1);

const insufficient = estimateClosedLoopStepResponse(samples.slice(0, 20), { windowSize: 64 });
assert.equal(insufficient.ok, false);
assert.match(insufficient.message, /samples|window/i);

console.log("Step-response estimator tests passed");
