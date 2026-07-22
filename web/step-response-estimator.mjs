function median(values) {
  const sorted = [...values].sort((a, b) => a - b);
  const middle = Math.floor(sorted.length / 2);
  return sorted.length % 2 ? sorted[middle] : (sorted[middle - 1] + sorted[middle]) * 0.5;
}

function largestPowerOfTwoAtMost(value) {
  if (value < 1) return 0;
  return 2 ** Math.floor(Math.log2(value));
}

function fft(real, imaginary, inverse = false) {
  const length = real.length;
  for (let index = 1, reversed = 0; index < length; index++) {
    let bit = length >> 1;
    for (; reversed & bit; bit >>= 1) reversed ^= bit;
    reversed ^= bit;
    if (index < reversed) {
      [real[index], real[reversed]] = [real[reversed], real[index]];
      [imaginary[index], imaginary[reversed]] = [imaginary[reversed], imaginary[index]];
    }
  }
  for (let size = 2; size <= length; size <<= 1) {
    const angle = (inverse ? 2 : -2) * Math.PI / size;
    const stepReal = Math.cos(angle), stepImaginary = Math.sin(angle);
    for (let start = 0; start < length; start += size) {
      let twiddleReal = 1, twiddleImaginary = 0;
      for (let offset = 0; offset < size / 2; offset++) {
        const even = start + offset, odd = even + size / 2;
        const oddReal = real[odd] * twiddleReal - imaginary[odd] * twiddleImaginary;
        const oddImaginary = real[odd] * twiddleImaginary + imaginary[odd] * twiddleReal;
        real[odd] = real[even] - oddReal;
        imaginary[odd] = imaginary[even] - oddImaginary;
        real[even] += oddReal;
        imaginary[even] += oddImaginary;
        const nextReal = twiddleReal * stepReal - twiddleImaginary * stepImaginary;
        twiddleImaginary = twiddleReal * stepImaginary + twiddleImaginary * stepReal;
        twiddleReal = nextReal;
      }
    }
  }
  if (inverse) {
    for (let index = 0; index < length; index++) {
      real[index] /= length;
      imaginary[index] /= length;
    }
  }
}

function resampleUniformly(samples) {
  const valid = samples.filter(sample =>
    Number.isFinite(sample.timestamp) && Number.isFinite(sample.desired) &&
    Number.isFinite(sample.measured));
  if (valid.length < 3) return null;
  const deltas = [];
  for (let index = 1; index < valid.length; index++) {
    const delta = valid[index].timestamp - valid[index - 1].timestamp;
    if (delta > 0) deltas.push(delta);
  }
  if (!deltas.length) return null;
  const intervalUs = median(deltas);
  const firstTime = valid[0].timestamp, lastTime = valid.at(-1).timestamp;
  const count = Math.min(20000, Math.floor((lastTime - firstTime) / intervalUs) + 1);
  if (count < 3) return null;
  const desired = new Array(count), measured = new Array(count);
  let source = 0;
  for (let index = 0; index < count; index++) {
    const time = firstTime + index * intervalUs;
    while (source + 1 < valid.length && valid[source + 1].timestamp < time) source++;
    const before = valid[source], after = valid[Math.min(source + 1, valid.length - 1)];
    const span = after.timestamp - before.timestamp;
    const blend = span > 0 ? Math.min(1, Math.max(0, (time - before.timestamp) / span)) : 0;
    desired[index] = before.desired + (after.desired - before.desired) * blend;
    measured[index] = before.measured + (after.measured - before.measured) * blend;
  }
  return { desired, measured, sampleRateHz: 1e6 / intervalUs };
}

function pidReviewNoiseSpectrum(length, sampleRateHz, cutoffHz, factor) {
  const realLength = Math.floor(length / 2) + 1;
  const firstAboveCutoff = Math.min(realLength - 1,
    Math.floor(cutoffHz * length / sampleRateHz) + 1);
  const lowPassLength = Math.max(2, Math.min(realLength, firstAboveCutoff * 2 - 2));
  const radius = Math.ceil(lowPassLength * 0.5);
  const sigma = lowPassLength / 6;
  const singleSided = new Array(realLength).fill(1);
  let accumulated = 0;
  for (let index = 0; index < lowPassLength; index++) {
    accumulated += Math.exp((-0.5 / sigma ** 2) * (index - radius) ** 2);
    singleSided[index] = accumulated;
  }
  for (let index = 0; index < lowPassLength; index++) singleSided[index] /= accumulated;
  const doubleSided = [
    ...singleSided,
    ...singleSided.slice(1, realLength - 1).reverse()
  ];
  // PIDReview inverts the shaped spectrum before adding it to Pxx. Keep the
  // user factor outside that reciprocal so larger values still mean stronger
  // regularization and a factor of 1 matches PIDReview's nominal spectrum.
  return doubleSided.map(value => factor / ((1 - value + 1e-9) * 10));
}

export function estimateClosedLoopStepResponse(samples, options = {}) {
  const uniform = resampleUniformly(samples);
  if (!uniform) return { ok: false, message: "Not enough valid timestamped samples." };
  const requestedWindow = Number(options.windowSize) || 0;
  const windowSize = requestedWindow > 0
    ? requestedWindow
    : Math.min(1024, largestPowerOfTwoAtMost(uniform.desired.length));
  if (windowSize < 32 || !Number.isInteger(Math.log2(windowSize))) {
    return { ok: false, message: "FFT window must be a power of two and at least 32 samples." };
  }
  if (uniform.desired.length < windowSize) {
    return { ok: false, message: `Need at least ${windowSize} samples for the selected FFT window.` };
  }

  const responseDurationSeconds = Math.min(5, Math.max(0.05,
    Number(options.responseDurationSeconds) || 0.5));
  const cutoffHz = Math.min(uniform.sampleRateHz * 0.49, Math.max(0.1,
    Number(options.cutoffHz) || 25));
  const regularization = Math.min(100, Math.max(1e-9,
    Number(options.regularization) || 1));
  const minimumInputAmplitude = Math.max(0,
    Number(options.minimumInputAmplitude) || 0.05);
  const responseLength = Math.min(windowSize,
    Math.max(2, Math.ceil(responseDurationSeconds * uniform.sampleRateHz)));
  const hop = Math.max(1, Math.round(windowSize / 16));
  const totalWindows = Math.floor((uniform.desired.length - windowSize) / hop) + 1;
  const hann = Array.from({ length: windowSize }, (_, index) =>
    0.5 - 0.5 * Math.cos(2 * Math.PI * index / (windowSize - 1)));
  const noiseSpectrum = pidReviewNoiseSpectrum(
    windowSize, uniform.sampleRateHz, cutoffHz, regularization);
  const accumulated = new Array(responseLength).fill(0);
  let acceptedWindows = 0;

  for (let windowIndex = 0; windowIndex < totalWindows; windowIndex++) {
    const start = windowIndex * hop;
    let inputPeak = 0;
    const inputReal = new Array(windowSize), inputImaginary = new Array(windowSize).fill(0);
    const outputReal = new Array(windowSize), outputImaginary = new Array(windowSize).fill(0);
    for (let index = 0; index < windowSize; index++) {
      const desired = uniform.desired[start + index];
      inputPeak = Math.max(inputPeak, Math.abs(desired));
      // PIDReview operates on angular rates in degrees/second and scales the FFT by N.
      // Matching that unit and scale preserves the meaning of its nominal noise spectrum.
      inputReal[index] = desired * 180 / Math.PI * hann[index];
      outputReal[index] = uniform.measured[start + index] * 180 / Math.PI * hann[index];
    }
    if (inputPeak < minimumInputAmplitude) continue;
    fft(inputReal, inputImaginary);
    fft(outputReal, outputImaginary);
    for (let bin = 0; bin < windowSize; bin++) {
      inputReal[bin] /= windowSize;
      inputImaginary[bin] /= windowSize;
      outputReal[bin] /= windowSize;
      outputImaginary[bin] /= windowSize;
    }

    const transferReal = new Array(windowSize), transferImaginary = new Array(windowSize);
    for (let bin = 0; bin < windowSize; bin++) {
      const inputPower = inputReal[bin] ** 2 + inputImaginary[bin] ** 2;
      const denominator = inputPower + noiseSpectrum[bin];
      transferReal[bin] =
        (outputReal[bin] * inputReal[bin] + outputImaginary[bin] * inputImaginary[bin]) /
        denominator;
      transferImaginary[bin] =
        (outputImaginary[bin] * inputReal[bin] - outputReal[bin] * inputImaginary[bin]) /
        denominator;
    }
    fft(transferReal, transferImaginary, true);
    let step = 0;
    for (let index = 0; index < responseLength; index++) {
      step += transferReal[index];
      accumulated[index] += step;
    }
    acceptedWindows++;
  }

  if (!acceptedWindows) {
    return {
      ok: false,
      message: "No analysis window contained enough target excitation.",
      sampleRateHz: uniform.sampleRateHz,
      windowSize,
      totalWindows,
      acceptedWindows: 0
    };
  }
  return {
    ok: true,
    message: "Estimated from target-to-actual closed-loop telemetry.",
    sampleRateHz: uniform.sampleRateHz,
    windowSize,
    totalWindows,
    acceptedWindows,
    response: accumulated.map((value, index) => ({
      timeSeconds: index / uniform.sampleRateHz,
      value: value / acceptedWindows
    }))
  };
}
