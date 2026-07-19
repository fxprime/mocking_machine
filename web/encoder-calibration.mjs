export function calculateEncoderCalibration(startCount, endCount, revolutions,
                                            currentCountsPerRevolution) {
  const turns = Number(revolutions);
  if (!Number.isInteger(turns) || turns < 1 || turns > 10) {
    throw new RangeError("Revolutions must be an integer from 1 to 10.");
  }

  const signedCountDelta = BigInt(endCount) - BigInt(startCount);
  const absoluteCountDelta = signedCountDelta < 0n ? -signedCountDelta : signedCountDelta;
  if (absoluteCountDelta === 0n) {
    throw new RangeError("No encoder movement was measured.");
  }
  if (absoluteCountDelta > BigInt(Number.MAX_SAFE_INTEGER)) {
    throw new RangeError("Encoder count change is too large to calculate safely.");
  }

  const measuredCountsPerRevolution = Number(absoluteCountDelta) / turns;
  const candidateCountsPerRevolution = Math.round(measuredCountsPerRevolution);
  if (candidateCountsPerRevolution < 1 || candidateCountsPerRevolution > 1_000_000) {
    throw new RangeError("Calculated counts per revolution is outside the firmware range.");
  }

  const current = Number(currentCountsPerRevolution);
  return {
    revolutions: turns,
    signedCountDelta,
    absoluteCountDelta,
    measuredCountsPerRevolution,
    candidateCountsPerRevolution,
    roundingResidualCounts: Number(absoluteCountDelta) - candidateCountsPerRevolution * turns,
    changePercent: current > 0
      ? (candidateCountsPerRevolution - current) * 100 / current
      : Number.NaN,
    countDirection: signedCountDelta > 0n ? 1 : -1
  };
}
