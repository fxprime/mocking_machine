# Mocking Machine JavaScript API

This dependency-free ES module lets an application communicate with the Mocking Machine firmware
without implementing serial framing, CRC, incremental parsing, payload layouts, or ACK correlation.
It works in browsers, Electron, and Node.js. The firmware remains the authority for all machine
safety checks; API motor commands still pass through its safety state machine.

Use the source module directly inside this repository, or install the library directory into
another local Node project:

```sh
npm install /path/to/mocking_machine/lib/mocking-machine
```

Node applications can then import from `@modulemore/mocking-machine-api`.

## Browser example

```js
import {
  MockingMachineClient,
  WebSerialTransport
} from "./lib/mocking-machine/index.mjs";

const transport = new WebSerialTransport({ baudRate: 115200 });
const machine = new MockingMachineClient({ transport });

machine.on("heartbeat", heartbeat => {
  console.log(heartbeat.buildVersion, heartbeat.state, heartbeat.faults);
});

machine.on("telemetry", sample => {
  // Timestamps and encoder counts are BigInt so values never lose precision.
  console.log(sample.timestampUs, sample.measuredVelocityRadS);
});

machine.on("text", text => console.log("console:", text));
machine.on("protocolError", error => console.warn("bad serial data", error));

await machine.connect();       // Opens the Web Serial device picker.
const settings = await machine.getSettings();
await machine.startStream();   // Synchronizes configuration and enables telemetry.

// Explicit user safety confirmation should happen in the application before this:
await machine.arm();
await machine.selectProfile(settings.selectedProfileId);
await machine.startRun();

// Always stop and disarm on application shutdown.
await machine.stop();
await machine.disconnect();
```

Web Serial requires Chrome/Edge and a secure context (`https:` or `localhost`).

## Node serial-library adapter

The API does not force a particular Node serial package. Wrap any package that can write bytes and
subscribe to received bytes:

```js
import {
  CallbackTransport,
  MockingMachineClient
} from "@modulemore/mocking-machine-api";
import { SerialPort } from "serialport";

const port = new SerialPort({ path: "/dev/ttyUSB0", baudRate: 115200, autoOpen: false });
const transport = new CallbackTransport({
  open: () => new Promise((resolve, reject) => port.open(error => error ? reject(error) : resolve())),
  write: bytes => new Promise((resolve, reject) =>
    port.write(bytes, error => error ? reject(error) : port.drain(resolve))),
  subscribe: (onData, onError) => {
    port.on("data", onData);
    port.on("error", onError);
    return () => {
      port.off("data", onData);
      port.off("error", onError);
    };
  },
  close: () => new Promise((resolve, reject) =>
    port.close(error => error ? reject(error) : resolve()))
});

const machine = new MockingMachineClient({ transport });
await machine.connect();
await machine.startStream();
```

`serialport` is only an example and is not a dependency of this repository.

## Transport contract

A custom transport must implement:

```js
{
  async open() {},                 // optional
  async write(bytes) {},           // required
  async close() {},                // optional
  on("data" | "error" | "close", listener) // required; returns unsubscribe()
}
```

`StreamTransport` adapts standard Web `ReadableStream`/`WritableStream` pairs.

## Main client API

Configuration and state:

- `getSettings()`, `getProfiles()`, `getLoadConfiguration()`,
  `getBearingConfiguration()`
- `setController()`, `saveController()`, `setParameter()`
- `setProfile()`, `createProfile()`, `selectProfile()`, `setDefaultProfile()`
- `setLoadConfiguration()`, `setBearingConfiguration()`
- `setDriverDiagnostic()`, `setCurrentSense()`

Machine control:

- `startStream()`, `stopStream()`
- `arm()`, `startRun()`, `stop()`, `clearFaults()`
- `motorTest()`, `startVelocityTest()`, `startVelocitySequence()`

Calibration:

- `currentCalibration()`, `requestCurrentCalibrationStatus()`
- `supplyVoltageCalibration()`
- `rotorZeroCalibration()`, `requestRotorZeroCalibrationStatus()`
- `zeroIndexCalibration()`, `requestZeroIndexCalibrationStatus()`
- `startCharacterization()`, `abortCharacterization()`, `characterizationAction()`

Low-level extension points:

- `send(messageId, payload)` writes a frame without waiting for a response.
- `request(messageId, payload, options)` correlates an ACK or direct response.
- `sendAscii(command)` sends one validated ASCII console line.
- `FrameParser`, `encodeFrame()`, `decodeMessage()`, and payload codecs can be used independently.

Command methods reject with `MachineCommandError` when firmware returns a non-OK result and with
`MachineTimeoutError` when no correlated response arrives. Subscribe to `message` for every decoded
message, or to lowercase message names such as `heartbeat`, `telemetry`, and `settings`.

See [`docs/protocol.md`](../../docs/protocol.md) for the complete wire specification and enum values.
