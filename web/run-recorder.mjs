export class RunRecorder {
  constructor(maximumSamples = 12000) {
    this.maximumSamples = maximumSamples;
    this.reset();
  }

  begin() {
    this.samples = [];
    this.waitingForRunning = true;
    this.recording = false;
  }

  consume(sample) {
    if (this.waitingForRunning && sample.state === 2) {
      this.waitingForRunning = false;
      this.recording = true;
    }
    if (this.recording && sample.state === 2) {
      this.samples.push(sample);
      if (this.samples.length > this.maximumSamples) this.samples.shift();
    } else if (this.recording) {
      this.recording = false;
    } else if (this.waitingForRunning && (sample.state === 0 || sample.state === 3)) {
      this.waitingForRunning = false;
    }
  }

  reset() {
    this.samples = [];
    this.waitingForRunning = false;
    this.recording = false;
  }
}
