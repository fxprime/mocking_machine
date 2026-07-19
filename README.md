# Mocking Machine

A controlled rotating-machine fault simulator for producing labeled datasets for imbalance, bearing, and phase-analysis experiments.

The repository contains:

- `src/` and `include/`: ESP32-WROVER Arduino firmware
- `web/`: dependency-free Web Serial console
- `docs/`: wiring, protocol, architecture, and product decisions
- `test/`: host-side deterministic control/protocol tests

## Build

```sh
pio run -e esp-wrover-kit
pio test -e native
```

The build generates a UTC timestamp plus Git revision for every firmware image. The firmware starts disarmed and never starts the motor when a serial client merely connects or enables telemetry.

Serve the browser console from localhost (Web Serial requires a secure context):

```sh
python3 -m http.server 8080 -d web
```

Then open `http://localhost:8080` in Chrome or Edge.

Read [the wiring guide](docs/wiring.md) before connecting motor power.

