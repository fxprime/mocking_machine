function clamp(value, minimum, maximum) {
  return Math.min(maximum, Math.max(minimum, value));
}

export class JerkLimitedVelocityLimiter {
  constructor(maximumVelocity, maximumAcceleration, maximumJerk,
              jerkLimitEnabled = true) {
    this.maximumVelocity = maximumVelocity;
    this.maximumAcceleration = maximumAcceleration;
    this.maximumJerk = maximumJerk;
    this.jerkLimitEnabled = jerkLimitEnabled;
    this.reset();
  }

  reset(velocity = 0) {
    this.velocity = velocity;
    this.acceleration = 0;
    this.previousTarget = velocity;
    this.targetInitialized = false;
  }

  update(requestedTarget, elapsedSeconds) {
    if (!(elapsedSeconds > 0)) return this.velocity;
    const target = clamp(
        requestedTarget, -this.maximumVelocity, this.maximumVelocity);
    if (!(this.maximumAcceleration > 0)) {
      this.acceleration = 0;
      this.previousTarget = target;
      this.targetInitialized = true;
      return this.velocity;
    }
    if (!this.jerkLimitEnabled) {
      const previousVelocity = this.velocity;
      const maximumVelocityChange =
          this.maximumAcceleration * elapsedSeconds;
      this.velocity += clamp(
          target - this.velocity, -maximumVelocityChange,
          maximumVelocityChange);
      this.acceleration =
          (this.velocity - previousVelocity) / elapsedSeconds;
      this.previousTarget = target;
      this.targetInitialized = true;
      return this.velocity;
    }
    if (!(this.maximumJerk > 0)) {
      this.acceleration = 0;
      this.previousTarget = target;
      this.targetInitialized = true;
      return this.velocity;
    }

    const targetAcceleration = this.targetInitialized
      ? clamp(
          (target - this.previousTarget) / elapsedSeconds,
          -this.maximumAcceleration, this.maximumAcceleration)
      : 0;
    this.previousTarget = target;
    this.targetInitialized = true;

    const error = target - this.velocity;
    const relativeAcceleration =
        this.acceleration - targetAcceleration;
    const maximumAccelerationChange =
        this.maximumJerk * elapsedSeconds;
    const captureError =
        Math.max(Math.abs(this.acceleration),
                 Math.abs(targetAcceleration)) *
            elapsedSeconds +
        0.5 * this.maximumJerk * elapsedSeconds * elapsedSeconds;
    if (Math.abs(error) <= captureError &&
        Math.abs(relativeAcceleration) <= maximumAccelerationChange) {
      this.velocity = target;
      this.acceleration = targetAcceleration;
      return this.velocity;
    }

    const relativeStoppingError =
        relativeAcceleration * Math.abs(relativeAcceleration) /
        (2 * this.maximumJerk);
    const switchingError = error - relativeStoppingError;
    const accelerationChange = switchingError >= 0
      ? maximumAccelerationChange
      : -maximumAccelerationChange;
    this.acceleration = clamp(
        this.acceleration + accelerationChange,
        -this.maximumAcceleration, this.maximumAcceleration);
    this.velocity = clamp(
        this.velocity + this.acceleration * elapsedSeconds,
        -this.maximumVelocity, this.maximumVelocity);
    return this.velocity;
  }
}
