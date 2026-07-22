export const MAXIMUM_VELOCITY_LEVELS = 16;

function xorshift32(seed) {
  let state = seed >>> 0 || 0x6d2b79f5;
  return () => {
    state ^= state << 13;
    state ^= state >>> 17;
    state ^= state << 5;
    return (state >>> 0) / 0x100000000;
  };
}

export function createVelocityStepSequence(options) {
  const minimumVelocity = Number(options.minimumVelocity);
  const maximumVelocity = Number(options.maximumVelocity);
  const levelCount = Number(options.levelCount);
  const holdSeconds = Number(options.holdSeconds);
  const velocityLimit = Number(options.velocityLimit);
  const seed = Number(options.seed) >>> 0;
  if (!(minimumVelocity > 0 && minimumVelocity < maximumVelocity &&
        maximumVelocity <= velocityLimit)) {
    throw new RangeError(`Velocity range must be above 0 and no greater than ${velocityLimit} rad/s.`);
  }
  if (!Number.isInteger(levelCount) || levelCount < 2 ||
      levelCount > MAXIMUM_VELOCITY_LEVELS) {
    throw new RangeError(`Random level count must be between 2 and ${MAXIMUM_VELOCITY_LEVELS}.`);
  }
  const holdMs = Math.round(holdSeconds * 1000);
  if (!(holdMs >= 100 && holdMs * levelCount <= 3600000)) {
    throw new RangeError("Hold time must be at least 0.1 seconds and total duration at most 3600 seconds.");
  }

  const random = xorshift32(seed);
  const span = maximumVelocity - minimumVelocity;
  const minimumChange = span * 0.2;
  const levels = [];
  for (let index = 0; index < levelCount; index++) {
    let level = minimumVelocity + random() * span;
    if (levels.length && Math.abs(level - levels.at(-1)) < minimumChange) {
      level = minimumVelocity + maximumVelocity - level;
      if (Math.abs(level - levels.at(-1)) < minimumChange) {
        level = levels.at(-1) < (minimumVelocity + maximumVelocity) * 0.5
          ? maximumVelocity
          : minimumVelocity;
      }
    }
    levels.push(level);
  }
  return { seed, levels, holdMs, durationSeconds: holdMs * levelCount / 1000 };
}

export function encodeVelocityStepSequence(sequence) {
  if (!sequence || !Array.isArray(sequence.levels) ||
      sequence.levels.length < 1 || sequence.levels.length > MAXIMUM_VELOCITY_LEVELS) {
    throw new RangeError("Velocity sequence must contain 1–16 levels.");
  }
  const payload = new Uint8Array(72);
  const view = new DataView(payload.buffer);
  view.setUint32(0, sequence.holdMs, true);
  payload[4] = sequence.levels.length;
  sequence.levels.forEach((level, index) => view.setFloat32(8 + index * 4, level, true));
  return payload;
}
