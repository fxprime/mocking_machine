# Wiring and commissioning

## Default signal map

| Function | ESP32 GPIO | Module signal | Notes |
|---|---:|---|---|
| Encoder A | 32 | Encoder A | Interrupt input; no boot-strapping conflict |
| Encoder B | 33 | Encoder B | Interrupt input; no boot-strapping conflict |
| Zero-index IR sensor | 13 | Sensor digital output | Rising-edge interrupt; 3.3 V only |
| Motor direction A | 25 | INA | Prefer a 74AHCT125 level buffer |
| Motor direction B | 26 | INB | Prefer a 74AHCT125 level buffer |
| Motor PWM | 27 | PWM | 20 kHz, 11-bit LEDC; prefer a 74AHCT125 buffer |
| Current sense | 34 | CS | ADC1 input-only, external divider/filter required |
| Driver diagnostic | 35 | EN/DIAG | Optional and disabled by default; external 5 V-to-3.3 V protection required when enabled |
| Driver VIN sense | 36 | Motor supply V+ through divider | ADC1 input-only; 6.8 kΩ / 1 kΩ divider |

All pins are configurable in `MachineSettings`.

The zero-index input accepts the first rising edge as the rotor reference and captures both
its microsecond timestamp and the current quadrature count. Later rising edges inside
`zero_index_min_interval_us` (5,000 µs by default) are treated as sensor bounce. This filter
does not reset, suppress, or otherwise modify the GPIO32/33 encoder count. At the configured
150 rad/s maximum, one revolution is about 41.9 ms, so the 5 ms default remains comfortably
below the expected one-pulse-per-revolution interval.

## Power wiring

1. Connect the 12 V supply through an appropriately rated fuse and a latching emergency-stop/power contactor to module motor `V+`.
2. Connect supply negative, driver ground, ESP32 ground, encoder ground, and sensor ground at a deliberate common/star point.
3. Power the ESP32 and driver logic from a regulated supply. Do not power the motor from the ESP32 board.
4. Connect motor leads to OUTA/OUTB. Use `motor_direction` to define logical forward; do not infer physical direction from wire color.
5. Fit bulk electrolytic capacitance near the driver plus local ceramic bypassing. Keep the high-current motor loop short and separate from encoder/sense wiring.
6. Use twisted/shielded encoder wiring and keep it away from motor leads. Ground the shield at one end.

The module vendor rates motor power at 5.5–16 V and reports roughly 6 A without additional heat sinking, with higher sustained current requiring proper thermal design. Size wiring, fuse, connector, and emergency stop from measured motor stall current and the weakest component—not the advertised 30 A peak.

## Logic-level protection

The VNH2SP30 input-high minimum is close to a 3.3 V ESP32 output, leaving little noise margin. Use a 5 V-powered 74AHCT125 (or equivalent unidirectional logic buffer with 3.3 V-compatible inputs) for INA, INB, and PWM.

The module documentation says its EN/DIAG lines are pulled up by the board's 5 V logic supply. Do not connect a 5 V-pulled diagnostic signal directly to an ESP32 pin. Use a resistor divider, open-drain level shifter, or suitable buffer. GPIO35 has no internal pull resistor.

Firmware leaves driver diagnostic monitoring disabled by default, so EN/DIAG may remain disconnected without producing a floating-input fault. Enable it from the Parameters page or with `diagnostic on` only after the protected signal has been verified. `diagnostic off` remains available if an active diagnostic fault caused lockout.

## Current sense

CS can approach the module logic rail and is noisy under PWM. A practical starting network is:

```text
module CS ── 15 kΩ ──┬── GPIO34
                     ├── 10 kΩ ── GND
                     └── 1 µF  ── GND
```

This scales 5 V to about 2 V and low-pass filters PWM ripple. The default firmware gain is only a starting estimate for that divider. Calibrate against a trusted current meter with `current calibrate <A>` or the browser dialog. The vendor notes approximately 10% sense accuracy, poorer low-current performance, direction-dependent variation, and a need for more filtering than the module's small capacitor. Do not use CS as the sole short-circuit protection.

Current-sense fault protection is disabled by default because an unconnected or uncalibrated GPIO34 can produce false trips. ADC telemetry remains available. Enable protection from Parameters or with `current protection on` only after verifying the circuit; use `current protection off` to disable it again.

## Driver VIN measurement

Connect the divider to GPIO36:

```text
driver motor V+ ── 6.8 kΩ ──┬── GPIO36
                            ├── 1.0 kΩ ── GND
                            └── 100 nF ── GND  (recommended)
```

The nominal divider gain is `7.8`; 16 V at the driver becomes approximately 2.05 V at GPIO36. Use stable 1% resistors at minimum, or 0.1% parts when repeatability across temperature matters. The browser VIN calibration dialog averages 64 ADC readings and recalculates the actual gain from a trusted multimeter value. Firmware then applies configurable 5.5 V undervoltage and 16.0 V overvoltage limits. These software limits do not replace supply transient protection.

Firmware rejects VIN calibration when the ADC input is above 2.8 V because that region is near saturation and cannot produce a trustworthy divider calibration. At a 12 V driver supply, measure approximately 1.54 V at GPIO36 before calibrating.

## Commissioning order

1. Test firmware and serial with motor power disconnected.
2. Verify encoder GPIO32/33 count direction by turning the shaft by hand; set CPR and direction.
3. Verify zero-index timestamp/count by turning slowly by hand.
4. Verify INA/INB/PWM and DIAG levels with a scope or logic analyzer.
5. Calibrate GPIO36 VIN against a multimeter, then calibrate CS with a current-limited bench supply.
6. Secure the unloaded motor in the final guard, arm, and run `characterize start CONFIRM_UNLOADED`.
7. Tune at low maximum duty and acceleration.
8. Add a small imbalance only after unloaded behavior, emergency stop, enclosure, and mounting are validated.
