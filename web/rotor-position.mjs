export function normalizeDegrees(angleDeg) {
  const normalized = Number(angleDeg) % 360;
  return normalized < 0 ? normalized + 360 : normalized;
}

export function pointerAngleDegrees(clientX, clientY, centerX, centerY) {
  return normalizeDegrees(
      Math.atan2(clientX - centerX, centerY - clientY) * 180 / Math.PI);
}

function shortestAngleDelta(fromDeg, toDeg) {
  return ((normalizeDegrees(toDeg) - normalizeDegrees(fromDeg) + 540) % 360) - 180;
}

export function updateRotorVisualState(previous, rotorPositionDeg, encoderCount,
                                       countsPerRevolution, encoderDirection) {
  const normalizedPosition = normalizeDegrees(rotorPositionDeg);
  const count = BigInt(encoderCount);
  const cpr = Number(countsPerRevolution);
  const direction = Number(encoderDirection) >= 0 ? 1 : -1;
  if (!previous || !(cpr > 0)) {
    return { unwrappedAngleDeg: normalizedPosition, encoderCount: count };
  }

  const encoderDeltaDeg = Number(count - previous.encoderCount) * direction * 360 / cpr;
  const encoderPredictedAngle = previous.unwrappedAngleDeg + encoderDeltaDeg;
  const phaseCorrectionDeg = shortestAngleDelta(encoderPredictedAngle, normalizedPosition);
  return {
    unwrappedAngleDeg: encoderPredictedAngle + phaseCorrectionDeg,
    encoderCount: count
  };
}
