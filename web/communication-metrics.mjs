export class CommunicationMetrics {
  constructor(now = () => performance.now()) {
    this.now = now;
    this.reset();
  }

  reset(nowMs = this.now()) {
    this.windowStartedMs = nowMs;
    this.rxBytes = 0;
    this.txBytes = 0;
    this.rxFrames = 0;
    this.txMessages = 0;
    this.telemetryFrames = 0;
    this.telemetryReceived = 0;
    this.droppedTelemetry = 0;
    this.crcErrors = 0;
    this.framingErrors = 0;
    this.lastFrameMs = undefined;
    this.lastTelemetryTimestampUs = undefined;
    this.lastExpectedIntervalUs = undefined;
  }

  recordRxBytes(count) {
    if (Number.isFinite(count) && count > 0) this.rxBytes += count;
  }

  recordTxBytes(count) {
    if (Number.isFinite(count) && count > 0) this.txBytes += count;
  }

  recordRxFrame(nowMs = this.now()) {
    this.rxFrames += 1;
    this.lastFrameMs = nowMs;
  }

  recordTxMessage() {
    this.txMessages += 1;
  }

  recordCrcError() {
    this.crcErrors += 1;
  }

  recordFramingError() {
    this.framingErrors += 1;
  }

  recordTelemetry(timestampUs, expectedRateHz) {
    this.telemetryFrames += 1;
    this.telemetryReceived += 1;

    const validTimestamp = Number.isFinite(timestampUs) && timestampUs >= 0;
    const validRate = Number.isFinite(expectedRateHz) && expectedRateHz > 0;
    if (!validTimestamp || !validRate) {
      this.lastTelemetryTimestampUs = validTimestamp ? timestampUs : undefined;
      this.lastExpectedIntervalUs = undefined;
      return;
    }

    const expectedIntervalUs = 1_000_000 / expectedRateHz;
    const rateIsStable = this.lastExpectedIntervalUs !== undefined
      && Math.abs(expectedIntervalUs - this.lastExpectedIntervalUs) <= expectedIntervalUs * 0.01;

    if (rateIsStable && this.lastTelemetryTimestampUs !== undefined
        && timestampUs > this.lastTelemetryTimestampUs) {
      const gapUs = timestampUs - this.lastTelemetryTimestampUs;
      if (gapUs > expectedIntervalUs * 1.5) {
        this.droppedTelemetry += Math.max(0, Math.round(gapUs / expectedIntervalUs) - 1);
      }
    }

    this.lastTelemetryTimestampUs = timestampUs;
    this.lastExpectedIntervalUs = expectedIntervalUs;
  }

  snapshot(nowMs = this.now()) {
    const elapsedSeconds = Math.max((nowMs - this.windowStartedMs) / 1000, 0.001);
    const totalExpectedTelemetry = this.telemetryReceived + this.droppedTelemetry;
    const result = {
      rxBytesPerSecond: this.rxBytes / elapsedSeconds,
      txBytesPerSecond: this.txBytes / elapsedSeconds,
      rxFramesPerSecond: this.rxFrames / elapsedSeconds,
      txMessagesPerSecond: this.txMessages / elapsedSeconds,
      telemetryHz: this.telemetryFrames / elapsedSeconds,
      telemetryReceived: this.telemetryReceived,
      droppedTelemetry: this.droppedTelemetry,
      dropoutPercent: totalExpectedTelemetry > 0
        ? this.droppedTelemetry / totalExpectedTelemetry * 100
        : 0,
      crcErrors: this.crcErrors,
      framingErrors: this.framingErrors,
      lastFrameAgeMs: this.lastFrameMs === undefined ? undefined : Math.max(0, nowMs - this.lastFrameMs)
    };

    this.windowStartedMs = nowMs;
    this.rxBytes = 0;
    this.txBytes = 0;
    this.rxFrames = 0;
    this.txMessages = 0;
    this.telemetryFrames = 0;
    return result;
  }
}
