import assert from "node:assert/strict";

import { JerkLimitedVelocityLimiter } from "../web/motion-limiter.mjs";

const maximumVelocity = 100;
const maximumAcceleration = 10;
const maximumJerk = 20;
const elapsedSeconds = 0.02;
const rampAcceleration = 5;
for (const direction of [1, -1]) {
  const limiter = new JerkLimitedVelocityLimiter(
      maximumVelocity, maximumAcceleration, maximumJerk);
  let previousAcceleration = limiter.acceleration;
  let maximumSettledError = 0;
  for (let sample = 0; sample <= 250; sample++) {
    const time = sample * elapsedSeconds;
    const target = direction * rampAcceleration * time;
    const output = limiter.update(target, elapsedSeconds);
    assert.ok(
        Math.abs(limiter.acceleration - previousAcceleration) <=
            maximumJerk * elapsedSeconds + 1e-9,
        "preview acceleration must obey the configured jerk limit");
    previousAcceleration = limiter.acceleration;
    if (time >= 2) {
      maximumSettledError = Math.max(
          maximumSettledError, Math.abs(target - output));
    }
  }
  assert.ok(
      maximumSettledError < 0.02,
      "preview must track a feasible ramp without periodic ripple");
}

console.log("Jerk-limited motion preview tests passed");
