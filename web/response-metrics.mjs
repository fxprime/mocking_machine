export function calculateStepMetrics(samples, target) {
  if (!samples.length || !(target > 0)) return null;
  const peak = Math.max(...samples.map(sample => sample.measured));
  const ten = samples.find(sample => sample.measured >= target * 0.1);
  const ninety = samples.find(sample => sample.measured >= target * 0.9);
  let plateauEnd = -1;
  for (let index = samples.length - 1; index >= 0; index--) {
    if (samples[index].desired >= target * 0.95) { plateauEnd = index; break; }
  }
  let settling;
  if (plateauEnd >= 0) {
    for (let index = 0; index <= plateauEnd; index++) {
      if (samples[index].measured >= target * 0.95 &&
          samples.slice(index, plateauEnd + 1).every(sample =>
            Math.abs(sample.measured - target) <= target * 0.05)) {
        settling = samples[index];
        break;
      }
    }
  }
  return {
    peak,
    overshootPercent: Math.max(0, (peak - target) / target * 100),
    riseTimeSeconds: ten && ninety ? (ninety.timestamp - ten.timestamp) / 1e6 : null,
    settlingTimeSeconds: settling ? (settling.timestamp - samples[0].timestamp) / 1e6 : null
  };
}

export function symmetricNiceLimit(maximum, minimum = 1e-6) {
  const raw = Math.max(Math.abs(maximum) * 1.08, minimum);
  const magnitude = 10 ** Math.floor(Math.log10(raw));
  const normalized = raw / magnitude;
  return (normalized <= 1 ? 1 : normalized <= 2 ? 2 : normalized <= 5 ? 5 : 10) * magnitude;
}
