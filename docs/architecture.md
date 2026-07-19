# Product and firmware architecture

## Product definition

Mocking Machine is a controlled excitation rig, not merely a motor speed controller. It creates repeatable, labeled operating conditions so a separate sensing system can learn or validate abnormal-machine signatures. Every sample must therefore identify the commanded motion, measured motion, physical load configuration, profile, zero-index timing, firmware build, controller configuration, and machine configuration.

The physical machine is intentionally hazardous: a 12 V geared brushed-DC motor can rotate an adjustable imbalance at high speed. A guard, base, fuse, emergency stop, and independent power disconnect are product requirements, not optional accessories.

## Design learned from the reference projects

The lightsaber project has a useful interaction pattern: one command registry supplies dispatch and help, while Arduino `setup()` constructs services and `loop()` services them. Its implementation is unsuitable for this controller because `readStringUntil()`, `String`, `std::vector`, runtime linked-list deletion, and serial printing can block or allocate unpredictably. `CommandSystem::currentId` is also not initialized, and the run loop resets deadlines to “now,” which accumulates timing drift.

The mecanum project uses the desired incremental PI form:

```text
u[k] = u[k-1] + Kp(e[k] - e[k-1]) + Ki e[k]
```

That code calculates `deltaT` but does not use it, does not update `lastTime`, and clamps only the returned PWM while leaving the internal accumulator wound up. This project implements the time-correct form:

```text
Δu = Kp Δe + Ki Ts e + Kd Δ²e / Ts
u[k] = saturate(u[k-1] + Δu)
```

Integral contribution that would push a saturated output farther into saturation is rejected. `Kd` defaults to zero, preserving incremental PI behavior.

## Runtime ownership

`MachineApplication` is a Meyers singleton only because Arduino requires global `setup()` and `loop()` entry points. It owns concrete modules, establishes initialization order, and advances the state machine. Modules do not reach back through the singleton and remain independently testable.

```text
Arduino loop
  └─ MachineApplication::runOnce
      ├─ bounded serial RX/TX service
      ├─ fixed-deadline 500 Hz control tick
      │   ├─ encoder snapshot + adaptive velocity estimate
      │   ├─ current/diagnostic/encoder safety checks
      │   ├─ profile → jerk/acceleration/velocity limiter
      │   ├─ incremental velocity controller
      │   └─ VNH2SP30 output
      ├─ heartbeat (1 Hz)
      └─ configurable telemetry stream
```

Missed control ticks are never replayed against stale sensor data. The scheduler preserves its deadline phase, uses the actual elapsed `dt`, and latches a fault when lateness exceeds two periods. Serial parsing and transmission are bounded; no control tick builds a `String`, grows a container, writes NVS, or emits serial output.

## Time and encoder handling

- `esp_timer_get_time()` supplies monotonic 64-bit microseconds.
- Both quadrature channels use CHANGE interrupts and a 16-entry Gray-code transition table.
- The zero-index interrupt accepts the first rising edge, rejects later edges inside the
  configurable debounce interval, and atomically stores the accepted timestamp and encoder
  count. Rejected zero edges never alter quadrature counting. The first accepted index creates
  the absolute phase reference; subsequent motion comes entirely from encoder-count changes.
  Later index events apply a configurable fraction of the measured phase error to a persistent
  offset, preventing long-term drift without allowing trigger jitter to replace encoder motion.
- The estimator calculates count delta over each actual control interval and then applies the
  configured first-order velocity filter. Encoder edge timestamps remain dedicated to the
  encoder-activity watchdog.
- `counts_per_output_revolution` must include quadrature multiplication and gearbox placement. The default `184` is a placeholder from the mecanum reference and must be measured for this motor.

## Configuration model

`MachineSettings` owns pins, controller gains, encoder scale, motion constraints, current and VIN calibration, supply limits, motor characteristics, profiles, load labels, serial rate, and characterization timing. One schema-versioned NVS blob is protected by CRC16. Invalid or older layouts fall back to firmware defaults.

Profiles are fixed-capacity structures (8 profiles, 16 waypoints each). Fixed capacity prevents heap fragmentation and makes NVS and protocol limits explicit. All profiles are constrained to one logical direction; `motor_direction` maps that logical direction to electrical polarity.

## Safety state machine

```text
DISARMED → ARM command → ARMED → RUN command → RUNNING
    ↑                       │                       │
    └──────── STOP ─────────┴──────── STOP ─────────┘
                            any fault → FAULT → clear → DISARMED
```

Manual duty expires automatically. Characterization requires the literal `CONFIRM_UNLOADED`, tests both directions, pauses before reversal, measures breakaway duty and maximum velocity, then saves the motor characteristics. Current sense and DIAG are secondary protection; the physical fuse and emergency stop remain primary.

Saving a characterization result also applies a conservative velocity constraint: `vmax` can
only decrease to the lower measured forward/reverse maximum, never increase. Stored velocity
profiles and the encoder-timeout threshold are clamped in the same candidate settings copy,
then the whole structure is validated before a single Preferences write.

## Browser console

The dependency-free console targets a rectangular desktop browser around 1440×900 at desk distance, English/LTR, keyboard and pointer. It uses a compact 1.125 type scale, persistent connection/machine/fault state, visible focus, reduced-motion support, actuator confirmation dialogs, live response chart, settings table, tuning controls, terminal, calibration, and CSV export. Encoder CPR calibration is host-guided: telemetry supplies the 64-bit start/end counts, the browser averages their absolute difference over 1–10 manually entered output-shaft turns, and an explicit review action saves the rounded integer through the normal disarmed `SET_PARAMETER` path. It adds no work to the control loop.

Connecting automatically starts only the telemetry stream. It never arms or starts the motor.

## Next product slices

1. Confirm encoder CPR, gearbox ratio, motor stall current, exact WROVER board variant, and DIAG pin count on the supplied module.
2. Add parameter-descriptor and full profile/load CRUD messages so the browser edits every firmware-owned default and constraint.
3. Add the draggable feasible-profile editor and step-response experiment message (pre-trigger, test duration, automatic stop, summary metrics).
4. Add current offset sampling at zero current, multi-point gain calibration, and direction-specific sense calibration.
5. Add datastream backpressure/drop counters and a session metadata record at CSV start.
6. Validate velocity estimator and controller on a dynamometer before fitting an imbalance.
