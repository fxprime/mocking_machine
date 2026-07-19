import assert from "node:assert/strict";
import { calculateEncoderCalibration } from "../web/encoder-calibration.mjs";

const exact = calculateEncoderCalibration(100n, 1940n, 10, 200);
assert.equal(exact.absoluteCountDelta, 1840n);
assert.equal(exact.measuredCountsPerRevolution, 184);
assert.equal(exact.candidateCountsPerRevolution, 184);
assert.equal(exact.roundingResidualCounts, 0);
assert.equal(exact.changePercent, -8);
assert.equal(exact.countDirection, 1);

const reverse = calculateEncoderCalibration(100n, -820n, 5, 184);
assert.equal(reverse.absoluteCountDelta, 920n);
assert.equal(reverse.candidateCountsPerRevolution, 184);
assert.equal(reverse.countDirection, -1);

const averaged = calculateEncoderCalibration(0n, 1843n, 10, 184);
assert.equal(averaged.measuredCountsPerRevolution, 184.3);
assert.equal(averaged.candidateCountsPerRevolution, 184);
assert.equal(averaged.roundingResidualCounts, 3);

assert.throws(() => calculateEncoderCalibration(0n, 184n, 0, 184), /1 to 10/);
assert.throws(() => calculateEncoderCalibration(10n, 10n, 1, 184), /No encoder movement/);

console.log("Encoder calibration tests passed");
