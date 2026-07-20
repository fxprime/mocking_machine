export class DeviceSynchronizer {
  constructor(retryIntervalMs = 1000) {
    this.retryIntervalMs = retryIntervalMs;
    this.reset();
  }

  reset() {
    this.synchronized = false;
    this.lastRequestMs = undefined;
  }

  shouldRequest(nowMs) {
    return !this.synchronized &&
      (this.lastRequestMs === undefined || nowMs - this.lastRequestMs >= this.retryIntervalMs);
  }

  markRequested(nowMs) {
    this.lastRequestMs = nowMs;
  }

  markTelemetryReceived() {
    this.synchronized = true;
  }
}
