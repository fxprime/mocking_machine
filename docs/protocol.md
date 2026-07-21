# Mocking Machine serial protocol

| Property | Value |
|---|---|
| Protocol version | **1** |
| Transport | Upload UART, binary frames with optional ASCII console input |
| Byte order | **Little-endian** for every multibyte integer and floating-point value |

This document is the human-readable wire specification for the firmware protocol. Its layout follows the [MAVLink message-reference style](https://mavlink.io/en/messages/development.html): a compact message index, individual message definitions, and shared enumerations.

## Contents

- [Packet format](#packet-format)
- [Message summary](#message-summary)
- [Message definitions](#message-definitions)
- [Enumerations](#enumerations)
- [Parameter IDs](#parameter-ids)
- [Operational sequences](#operational-sequences)
- [ASCII console](#ascii-console)
- [Compatibility](#compatibility)

## Conventions

| Notation | Meaning |
|---|---|
| Host | Browser or other controlling computer |
| Device | ESP32 firmware |
| `u8`, `u16`, `u32`, `u64` | Unsigned integer of the stated width |
| `i8`, `i64` | Signed integer of the stated width |
| `f32` | IEEE-754 single-precision floating point |
| `char[N]` | Fixed-width byte string; firmware-generated strings are NUL-terminated when shorter than the field |
| Empty | Payload length is zero |

Fixed arrays always occupy their full wire size. Fields marked by a count contain valid entries only below that count; senders should zero unused entries.

## Packet format

Every binary packet has a 10-byte prefix, a bounded payload, and a 2-byte checksum.

| Offset | Size | Field | Value / description |
|---:|---:|---|---|
| 0 | 1 | `sync_1` | `0xB5` |
| 1 | 1 | `sync_2` | `0x62` |
| 2 | 1 | `version` | Protocol version, currently `1` |
| 3 | 1 | `flags` | Reserved; transmit as zero |
| 4 | 2 | `message_id` | Message identifier |
| 6 | 2 | `sequence` | Request correlation or device transmit sequence |
| 8 | 2 | `payload_size` | Payload bytes, range 0–512 |
| 10 | N | `payload` | Message-specific packed payload |
| 10 + N | 2 | `crc` | CRC16/CCITT-FALSE, least-significant byte first |

Total packet size is `12 + payload_size` bytes.

### CRC

| Parameter | Value |
|---|---|
| Algorithm | CRC16/CCITT-FALSE |
| Polynomial | `0x1021` |
| Initial value | `0xFFFF` |
| Input reflection | No |
| Output reflection | No |
| Final XOR | `0x0000` |
| Covered bytes | `version` through the final payload byte |
| Excluded bytes | Both sync bytes and the transmitted CRC |

Frames with an unsupported version, payload larger than 512 bytes, or incorrect CRC are discarded.

### Sequence handling

- The host chooses the sequence of each command.
- `ACK` echoes the request sequence and identifies the acknowledged message in its payload.
- Direct responses such as `SETTINGS`, `PROFILE_CONFIGURATION`, and calibration status use the triggering request sequence.
- Unsolicited `HEARTBEAT` and `TELEMETRY` packets use the device's incrementing 16-bit transmit sequence.
- Sequence values wrap naturally at 65535.

## Message summary

### System and configuration

| ID | Name | Direction | Bytes | Description |
|---:|---|---|---:|---|
| `0x0001` | [HEARTBEAT](#heartbeat-0x0001) | Device → host | 65 | Device identity, state, and health |
| `0x0002` | [ACK](#ack-0x0002) | Device → host | 3 | Command result |
| `0x0100` | [GET_SETTINGS](#get_settings-0x0100) | Host → device | 0 | Request current settings |
| `0x0101` | [SETTINGS](#settings-0x0101) | Device → host | 143 | Complete runtime settings snapshot |
| `0x0110` | [SET_CONTROLLER](#controller-gain-messages-0x0110-0x0114) | Host → device | 12 | Apply gains to RAM |
| `0x0111` | [SET_DRIVER_DIAGNOSTIC](#protection-enable-messages-0x0111-0x0112) | Host → device | 1 | Enable or disable EN/DIAG protection |
| `0x0112` | [SET_CURRENT_SENSE](#protection-enable-messages-0x0111-0x0112) | Host → device | 1 | Enable or disable overcurrent protection |
| `0x0113` | [SET_PARAMETER](#set_parameter-0x0113) | Host → device | 7 | Apply one parameter |
| `0x0114` | [SAVE_CONTROLLER](#controller-gain-messages-0x0110-0x0114) | Host → device | 12 | Apply and persist gains |

### Profiles and load labels

| ID | Name | Direction | Bytes | Description |
|---:|---|---|---:|---|
| `0x0120` | [SELECT_PROFILE](#profile-selection-messages-0x0120-0x0125) | Host → device | 2 | Select a profile for the next run |
| `0x0121` | [GET_PROFILES](#profile-configuration-messages-0x0121-0x0124) | Host → device | 0 | Request all stored profiles |
| `0x0122` | [PROFILE_CONFIGURATION](#profile_configuration-0x0122) | Device → host | 168 | One profile definition |
| `0x0123` | [SET_PROFILE](#set_profile-and-create_profile-0x0123-0x0124) | Host → device | 169 | Replace an existing profile |
| `0x0124` | [CREATE_PROFILE](#set_profile-and-create_profile-0x0123-0x0124) | Host → device | 169 | Add a profile |
| `0x0125` | [SET_DEFAULT_PROFILE](#profile-selection-messages-0x0120-0x0125) | Host → device | 2 | Persist the default profile |
| `0x0130` | [GET_LOAD_CONFIGURATION](#load-configuration-messages-0x0130-0x0132) | Host → device | 0 | Request the load label |
| `0x0131` | [LOAD_CONFIGURATION](#load_configuration-0x0131) | Device → host | 86 | Current load label |
| `0x0132` | [SET_LOAD_CONFIGURATION](#load-configuration-messages-0x0130-0x0132) | Host → device | 86 | Validate and persist a load label |

### Runtime and telemetry

| ID | Name | Direction | Bytes | Description |
|---:|---|---|---:|---|
| `0x0200` | [START_RUN](#runtime-control-messages-0x0200-0x0205) | Host → device | 0 | Run the selected profile |
| `0x0201` | [STOP_RUN](#runtime-control-messages-0x0200-0x0205) | Host → device | 0 | Stop and disarm |
| `0x0202` | [MOTOR_TEST](#motor_test-0x0202) | Host → device | 4 | Apply raw signed duty |
| `0x0203` | [CLEAR_FAULTS](#runtime-control-messages-0x0200-0x0205) | Host → device | 0 | Clear and recheck operational faults |
| `0x0204` | [ARM](#runtime-control-messages-0x0200-0x0205) | Host → device | 0 | Enter armed state after safety checks |
| `0x0205` | [START_VELOCITY_TEST](#start_velocity_test-0x0205) | Host → device | 8 | Run a temporary constrained step |
| `0x0210` | [START_STREAM](#stream-control-messages-0x0210-0x0211) | Host → device | 0 | Synchronize state and enable telemetry |
| `0x0211` | [STOP_STREAM](#stream-control-messages-0x0210-0x0211) | Host → device | 0 | Disable telemetry |
| `0x0220` | [TELEMETRY](#telemetry-0x0220) | Device → host | 84 | Machine measurement sample |

### Calibration and characterization

| ID | Name | Direction | Bytes | Description |
|---:|---|---|---:|---|
| `0x0300` | [CURRENT_CALIBRATION](#current_calibration-0x0300) | Host → device | 5 | Control two-point current calibration |
| `0x0301` | [SUPPLY_VOLTAGE_CALIBRATION](#supply_voltage_calibration-0x0301) | Host → device | 4 | Calibrate the VIN divider |
| `0x0302` | [CURRENT_CALIBRATION_STATUS](#current_calibration_status-0x0302) | Device → host | 27 | Current calibration progress/result |
| `0x0310` | [CHARACTERIZATION_RESULT](#characterization_result-0x0310) | Device → host | 32 | Pending motor characterization result |
| `0x0311` | [CHARACTERIZATION_ACTION](#characterization_action-0x0311) | Host → device | 1 | Save or discard the pending result |
| `0x0312` | [CHARACTERIZATION_STATUS](#characterization_status-0x0312) | Device → host | 10 | Live characterization progress |

## Message definitions

### HEARTBEAT (0x0001)

Device identity and current health. The host should retain the latest build string and settings schema with every exported dataset.

| Offset | Field name | Type | Units / values | Description |
|---:|---|---|---|---|
| 0 | `uptime_us` | `u64` | µs | Time since device boot |
| 8 | `settings_schema` | `u32` | — | Persisted settings schema understood by the firmware |
| 12 | `faults` | `u32` | [FAULT_FLAGS](#fault_flags-bitmask) | Active fault bitmask |
| 16 | `state` | `u8` | [RUN_STATE](#run_state) | Safety state-machine state |
| 17 | `build_version` | `char[48]` | UTF-8/ASCII | Build-generated version string |

### ACK (0x0002)

Explicit result for a host command. The frame sequence equals the request sequence.

| Offset | Field name | Type | Values | Description |
|---:|---|---|---|---|
| 0 | `request_message_id` | `u16` | Message ID | Command being acknowledged |
| 2 | `result` | `u8` | [RESULT_CODE](#result_code) | Command result |

### GET_SETTINGS (0x0100)

Empty payload. The device replies with `SETTINGS` using the request sequence. Query messages use their data response as completion and do not emit a separate `ACK`.

### SETTINGS (0x0101)

Complete packed settings snapshot. The field order is append-only for backward-compatible decoders; hosts should check payload length before reading fields added by newer firmware.

| Offset | Field name | Type | Units / values | Description |
|---:|---|---|---|---|
| 0 | `schema_version` | `u32` | — | Persisted settings schema |
| 4 | `baud` | `u32` | bit/s | UART rate used after reboot |
| 8 | `control_period_us` | `u32` | µs | Control-loop period |
| 12 | `counts_per_revolution` | `u32` | counts/rev | Quadrature counts per output-shaft revolution |
| 16 | `stream_rate_hz` | `u16` | Hz | Requested telemetry rate |
| 18 | `selected_profile_id` | `u16` | profile ID | Persisted default profile, not a temporary runtime selection |
| 20 | `kp` | `f32` | — | Incremental proportional gain |
| 24 | `ki` | `f32` | 1/s | Incremental integral gain; firmware applies sample time |
| 28 | `kd` | `f32` | — | Incremental derivative gain |
| 32 | `max_velocity_rad_s` | `f32` | rad/s | Velocity constraint |
| 36 | `max_acceleration_rad_s2` | `f32` | rad/s² | Acceleration constraint |
| 40 | `max_jerk_rad_s3` | `f32` | rad/s³ | Jerk constraint |
| 44 | `max_current_a` | `f32` | A | Software overcurrent threshold |
| 48 | `max_duty` | `f32` | 0–1 | Maximum PWM duty |
| 52 | `start_duty_forward` | `f32` | 0–1 | Characterized forward breakaway duty |
| 56 | `start_duty_reverse` | `f32` | 0–1 | Characterized reverse breakaway duty |
| 60 | `load_setting_id` | `u8` | — | Active load-label group |
| 61 | `load_count` | `u8` | 0–12 | Number of active load slots |
| 62 | `motor_direction` | `i8` | `-1`, `1` | Electrical motor-direction multiplier |
| 63 | `stop_mode` | `u8` | [STOP_MODE](#stop_mode) | Driver behavior at zero command |
| 64 | `supply_divider_gain` | `f32` | V/V | VIN reconstruction gain |
| 68 | `supply_input_offset_v` | `f32` | V | VIN ADC input offset |
| 72 | `min_supply_voltage_v` | `f32` | V | Undervoltage threshold |
| 76 | `max_supply_voltage_v` | `f32` | V | Overvoltage threshold |
| 80 | `supply_voltage_pin` | `u8` | GPIO | VIN ADC input |
| 81 | `current_gain_a_per_v` | `f32` | A/V | Current-sense calibration gain |
| 85 | `current_offset_v` | `f32` | V | Current-sense calibration offset |
| 89 | `current_sense_pin` | `u8` | GPIO | Current-sense ADC input |
| 90 | `current_sense_enabled` | `u8` | `0`, `1` | Whether current can trip the machine |
| 91 | `driver_diagnostic_enabled` | `u8` | `0`, `1` | Whether EN/DIAG can trip the machine |
| 92 | `driver_diagnostic_pin` | `u8` | GPIO | Protected active-low EN/DIAG input |
| 93 | `encoder_timeout_ms` | `u32` | ms | Maximum encoder-inactive time while motion is demanded |
| 97 | `encoder_timeout_velocity_rad_s` | `f32` | rad/s | Desired-speed threshold that enables the watchdog |
| 101 | `max_feedback_correction` | `f32` | — | Compatibility field; ignored by the active estimator |
| 105 | `estimator_min_counts` | `u8` | counts | Compatibility field; ignored by the active estimator |
| 106 | `estimator_max_window_us` | `u32` | µs | Compatibility field; ignored by the active estimator |
| 110 | `estimator_stale_timeout_us` | `u32` | µs | Compatibility field; ignored by the active estimator |
| 114 | `current_filter_cutoff_hz` | `f32` | Hz | Current low-pass cutoff |
| 118 | `zero_index_min_interval_us` | `u32` | µs | Time-domain index debounce interval |
| 122 | `zero_index_correction_gain` | `f32` | 0–1 | Fraction of index phase error corrected per accepted pulse |
| 126 | `encoder_direction` | `i8` | `-1`, `1` | Encoder sign multiplier |
| 127 | `zero_index_minimum_separation_revolutions` | `f32` | rev | Required rotor travel between accepted index pulses |
| 131 | `characterization_dynamics_filter_cutoff_hz` | `f32` | Hz | Dynamics-estimator low-pass cutoff |
| 135 | `characterization_dynamics_quantile` | `f32` | 0–1 | Robust acceleration/jerk quantile |
| 139 | `characterization_recommendation_safety_factor` | `f32` | 0–1 | Multiplier applied to the weaker direction |

The encoder watchdog starts when desired velocity first exceeds its configured threshold. Each valid quadrature transition refreshes activity. Dropping below the threshold or stopping resets the watchdog window.

### Controller gain messages (0x0110, 0x0114)

`SET_CONTROLLER` applies gains to RAM for testing. `SAVE_CONTROLLER` validates the complete settings structure and persists the gains; it is accepted only while disarmed.

| Offset | Field name | Type | Description |
|---:|---|---|---|
| 0 | `kp` | `f32` | Incremental proportional gain |
| 4 | `ki` | `f32` | Integral gain in 1/s |
| 8 | `kd` | `f32` | Incremental derivative gain |

### Protection enable messages (0x0111, 0x0112)

`SET_DRIVER_DIAGNOSTIC` and `SET_CURRENT_SENSE` each carry the following payload:

| Offset | Field name | Type | Values | Description |
|---:|---|---|---|---|
| 0 | `enabled` | `u8` | `0`, `1` | Disable or enable the corresponding protection |

Both are normally accepted only while disarmed. Disabling a protection is also accepted from its corresponding fault state so a disconnected or uncalibrated input cannot permanently lock out the machine. The change is applied immediately and persisted. Disabling current protection does not disable current telemetry or calibration.

### SET_PARAMETER (0x0113)

Applies one entry from the [parameter table](#parameter-ids). It is accepted only while disarmed.

| Offset | Field name | Type | Values | Description |
|---:|---|---|---|---|
| 0 | `parameter_id` | `u16` | [PARAMETER_ID](#parameter-ids) | Parameter to update |
| 2 | `value` | `f32` | Parameter-specific | Candidate value |
| 6 | `persist` | `u8` | `0`, `1` | Save to Preferences when one |

Firmware applies the value to a copy of the complete settings structure, validates all cross-parameter constraints, reconfigures affected runtime modules, and replies with `ACK` followed by `SETTINGS`. Unknown IDs and non-finite values are rejected.

### Profile selection messages (0x0120, 0x0125)

Both messages carry one `u16 profile_id`. `SELECT_PROFILE` changes only the runtime selection for the next run and never writes Preferences. `SET_DEFAULT_PROFILE` is accepted only while disarmed and persists both the default and current runtime selection. The `SETTINGS.selected_profile_id` field always denotes the persisted default.

### Profile configuration messages (0x0121-0x0124)

`GET_PROFILES` has an empty payload and emits one `PROFILE_CONFIGURATION` for every stored profile. At most eight profiles are stored.

### PROFILE_CONFIGURATION (0x0122)

| Offset | Field name | Type | Units / values | Description |
|---:|---|---|---|---|
| 0 | `profile_id` | `u16` | — | Unique profile ID |
| 2 | `kind` | `u8` | [PROFILE_KIND](#profile_kind) | Profile generator |
| 3 | `name` | `char[16]` | UTF-8/ASCII | Display name |
| 19 | `target_velocity_rad_s` | `f32` | rad/s | Ramp target |
| 23 | `sine_mean_rad_s` | `f32` | rad/s | Sine center velocity |
| 27 | `sine_amplitude_rad_s` | `f32` | rad/s | Sine amplitude |
| 31 | `sine_frequency_hz` | `f32` | Hz | Sine frequency |
| 35 | `duration_ms` | `u32` | ms | Total run duration |
| 39 | `point_count` | `u8` | 0–16 | Valid entries in `points` |
| 40 | `points` | `PROFILE_POINT[16]` | — | Fixed waypoint array |

`PROFILE_POINT` is an 8-byte packed structure:

| Relative offset | Field name | Type | Units | Description |
|---:|---|---|---|---|
| 0 | `time_ms` | `u32` | ms | Time from profile start |
| 4 | `velocity_rad_s` | `f32` | rad/s | Desired angular velocity |

### SET_PROFILE and CREATE_PROFILE (0x0123, 0x0124)

These messages prepend a persistence byte to the complete 168-byte `PROFILE_CONFIGURATION` payload.

| Offset | Field name | Type | Description |
|---:|---|---|---|
| 0 | `persist` | `u8` | `0` for a temporary test, `1` to save the collection |
| 1 | `profile` | `PROFILE_CONFIGURATION` | Profile payload without a frame header |

Both commands are accepted only while disarmed. `SET_PROFILE` requires an existing ID. `CREATE_PROFILE` requires an unused ID and free capacity. Firmware validates kind, duration, point count, chronological order, velocity limits, and waypoint acceleration feasibility before changing the collection.

### Load configuration messages (0x0130-0x0132)

`GET_LOAD_CONFIGURATION` has an empty payload. `LOAD_CONFIGURATION` and `SET_LOAD_CONFIGURATION` share the following format.

### LOAD_CONFIGURATION (0x0131)

| Offset | Field name | Type | Units / values | Description |
|---:|---|---|---|---|
| 0 | `setting_id` | `u8` | — | Load-label group ID |
| 1 | `count` | `u8` | 0–12 | Valid entries in `loads` |
| 2 | `loads` | `LOAD_ENTRY[12]` | — | Fixed load-slot array |

`LOAD_ENTRY` is a 7-byte packed structure:

| Relative offset | Field name | Type | Units / values | Description |
|---:|---|---|---|---|
| 0 | `slot_id` | `u8` | 0–11 | Physical rotor slot |
| 1 | `position_deg` | `u16` | deg | Must equal `slot_id × 30` |
| 3 | `strength` | `f32` | label 1–10 | Integer-equivalent load-strength label |

Slots must be unique. `SET_LOAD_CONFIGURATION` persists the complete configuration and is accepted only while disarmed.

### Runtime control messages (0x0200-0x0205)

- `ARM` performs all safety checks. Hosts must wait for a successful ACK before starting motion.
- `START_RUN` runs the current runtime profile and requires the armed state.
- `STOP_RUN` stops output and disarms.
- `CLEAR_FAULTS` stops and disarms, clears latched operational faults, then re-samples VIN and enabled EN/DIAG protection. Initialization faults require a successful reboot.

These messages have empty payloads except `MOTOR_TEST` and `START_VELOCITY_TEST` below.

### MOTOR_TEST (0x0202)

| Offset | Field name | Type | Units / values | Description |
|---:|---|---|---|---|
| 0 | `signed_raw_duty` | `f32` | `-max_duty`…`max_duty` | Signed direct motor command |

The command requires armed, fault-free state. It passes through the safety state machine but bypasses velocity control and deadband compensation. It expires after the configured command timeout unless refreshed. Zero stops output immediately.

### START_VELOCITY_TEST (0x0205)

Starts a temporary constrained velocity step after a successful `ARM`.

| Offset | Field name | Type | Units | Description |
|---:|---|---|---|---|
| 0 | `target_velocity_rad_s` | `f32` | rad/s | Requested step velocity |
| 4 | `duration_ms` | `u32` | ms | Test duration |

Configured velocity, acceleration, and jerk limits remain active.

### Stream control messages (0x0210, 0x0211)

Both have empty payloads. `START_STREAM` first emits `SETTINGS`, every stored `PROFILE_CONFIGURATION`, `LOAD_CONFIGURATION`, any pending `CHARACTERIZATION_RESULT`, applicable `CHARACTERIZATION_STATUS`, and then `ACK`; periodic telemetry follows. It does not arm or rotate the motor. `STOP_STREAM` disables periodic telemetry.

### TELEMETRY (0x0220)

One timestamped machine sample.

| Offset | Field name | Type | Units / values | Description |
|---:|---|---|---|---|
| 0 | `timestamp_us` | `u64` | µs | Device monotonic sample time |
| 8 | `last_zero_timestamp_us` | `u64` | µs | Time of the last accepted zero-index edge |
| 16 | `encoder_count` | `i64` | counts | Cumulative quadrature count |
| 24 | `last_zero_encoder_count` | `i64` | counts | Encoder count at the last accepted zero edge |
| 32 | `desired_velocity_rad_s` | `f32` | rad/s | Motion-limited controller setpoint |
| 36 | `measured_velocity_rad_s` | `f32` | rad/s | Encoder-derived velocity |
| 40 | `controller_output` | `f32` | `-1`…`1` | Saturated accumulated motor command |
| 44 | `current_a` | `f32` | A | Filtered calibrated current |
| 48 | `supply_voltage_v` | `f32` | V | Calibrated driver VIN |
| 52 | `faults` | `u32` | [FAULT_FLAGS](#fault_flags-bitmask) | Active fault bitmask |
| 56 | `profile_id` | `u16` | profile ID | Runtime profile represented by this sample |
| 58 | `load_setting_id` | `u8` | — | Active load-label group |
| 59 | `state` | `u8` | [RUN_STATE](#run_state) | Safety state |
| 60 | `controller_proportional_term` | `f32` | Δoutput | Latest incremental P contribution |
| 64 | `controller_integral_term` | `f32` | Δoutput | Latest incremental I contribution |
| 68 | `controller_derivative_term` | `f32` | Δoutput | Latest incremental D contribution |
| 72 | `rotor_position_deg` | `f32` | deg | Encoder-primary rotor phase in `[0, 360)` |
| 76 | `zero_index_sequence` | `u32` | count | Number of accepted index edges |
| 80 | `zero_index_rejected_count` | `u32` | count | Number of rejected bounce/too-close edges |

Incremental P/I/D terms are zero while velocity control is inactive. Rotor position becomes referenced after `zero_index_sequence` is nonzero. The first accepted index establishes the phase offset; later pulses apply only `zero_index_correction_gain × phase_error`. Rejected pulses do not change the accepted timestamp, encoder count, sequence, or phase.

### CURRENT_CALIBRATION (0x0300)

Controls non-blocking two-point current-sense calibration.

| Offset | Field name | Type | Units / values | Description |
|---:|---|---|---|---|
| 0 | `action` | `u8` | [CURRENT_CALIBRATION_ACTION](#current_calibration_action) | Calibration operation |
| 1 | `reference_current_a` | `f32` | A | External reference for capture actions |

Each capture averages 64 ADC readings, one per control tick. Point 2 must differ from Point 1 by at least 0.01 A and 0.001 V. The candidate is calculated as `gain = ΔI / ΔV` and `offset = V1 - I1 / gain`. It is neither applied nor persisted until `Save`; saving stops and disarms the motor. `Cancel` preserves the previous calibration.

### SUPPLY_VOLTAGE_CALIBRATION (0x0301)

| Offset | Field name | Type | Units | Description |
|---:|---|---|---|---|
| 0 | `reference_voltage_v` | `f32` | V | Precisely measured driver input voltage |

Firmware averages 64 raw ADC readings, calculates the divider gain, applies it immediately, persists it, and replies with `ACK` followed by `SETTINGS`.

### CURRENT_CALIBRATION_STATUS (0x0302)

| Offset | Field name | Type | Units / values | Description |
|---:|---|---|---|---|
| 0 | `captured_mask` | `u8` | bit 0: point 1; bit 1: point 2/candidate | Available calibration data |
| 1 | `capture_point` | `u8` | `0`, `1`, `2` | Zero when idle; otherwise active 64-sample capture |
| 2 | `last_result` | `u8` | [RESULT_CODE](#result_code) | Most recent calibration result |
| 3 | `point1_voltage_v` | `f32` | V | Averaged sense voltage at point 1 |
| 7 | `point1_reference_a` | `f32` | A | Reference current at point 1 |
| 11 | `point2_voltage_v` | `f32` | V | Averaged sense voltage at point 2 |
| 15 | `point2_reference_a` | `f32` | A | Reference current at point 2 |
| 19 | `candidate_gain_a_per_v` | `f32` | A/V | Calculated candidate gain |
| 23 | `candidate_offset_v` | `f32` | V | Calculated candidate offset |

### CHARACTERIZATION_RESULT (0x0310)

Pending motor characterization result. It remains in RAM and is resent when streaming starts until explicitly saved or discarded.

| Offset | Field name | Type | Units | Description |
|---:|---|---|---|---|
| 0 | `start_duty_forward` | `f32` | 0–1 | Forward breakaway duty |
| 4 | `start_duty_reverse` | `f32` | 0–1 | Reverse breakaway duty |
| 8 | `max_velocity_forward_rad_s` | `f32` | rad/s | Measured forward peak velocity magnitude |
| 12 | `max_velocity_reverse_rad_s` | `f32` | rad/s | Measured reverse peak velocity magnitude |
| 16 | `acceleration_forward_rad_s2` | `f32` | rad/s² | Filtered robust forward acceleration estimate |
| 20 | `acceleration_reverse_rad_s2` | `f32` | rad/s² | Filtered robust reverse acceleration estimate |
| 24 | `jerk_forward_rad_s3` | `f32` | rad/s³ | Filtered robust forward jerk estimate |
| 28 | `jerk_reverse_rad_s3` | `f32` | rad/s³ | Filtered robust reverse jerk estimate |

Dynamics use the configured online low-pass filter and robust quantile independently in each full-duty direction.

### CHARACTERIZATION_ACTION (0x0311)

| Offset | Field name | Type | Values | Description |
|---:|---|---|---|---|
| 0 | `flags` | `u8` | [CHARACTERIZATION_ACTION_FLAGS](#characterization_action_flags-bitmask) | Review action |

Zero discards the pending result. Saving always applies the motor deadband and velocity candidate. Acceleration and jerk recommendations are separately opt-in, use the weaker direction multiplied by the configured safety factor, and can only lower existing limits. Lowering acceleration also constrains stored waypoint slopes before complete-settings validation. Characterization never writes Preferences automatically.

### CHARACTERIZATION_STATUS (0x0312)

Emitted at 10 Hz while characterization runs and on stage transitions.

| Offset | Field name | Type | Units / values | Description |
|---:|---|---|---|---|
| 0 | `stage` | `u8` | [CHARACTERIZATION_STAGE](#characterization_stage) | Active stage |
| 1 | `result_pending` | `u8` | `0`, `1` | Whether a result awaits review |
| 2 | `applied_duty` | `f32` | `-1`…`1` | Current characterization command |
| 6 | `measured_velocity_rad_s` | `f32` | rad/s | Live encoder-derived velocity |

Aborting, stopping, producing an invalid result, or faulting before completion leaves previously saved settings unchanged.

## Enumerations

### RESULT_CODE

| Value | Name | Description |
|---:|---|---|
| 0 | `OK` | Command completed |
| 1 | `INVALID_MESSAGE` | Unsupported message or operation |
| 2 | `INVALID_LENGTH` | Payload size does not match the message |
| 3 | `INVALID_VALUE` | Field value or cross-parameter combination is invalid |
| 4 | `UNSAFE_STATE` | Safety state does not permit the operation |
| 5 | `STORAGE_FAILURE` | Preferences write failed |

### RUN_STATE

| Value | Name | Description |
|---:|---|---|
| 0 | `DISARMED` | Motor output disabled; configuration changes permitted |
| 1 | `ARMED` | Safety checks passed; waiting for a run command or accepting bounded direct-duty tests |
| 2 | `RUNNING` | Profile, velocity test, or direct motor test active |
| 3 | `FAULT` | Output inhibited by one or more faults |

### FAULT_FLAGS (bitmask)

| Value | Name | Description |
|---:|---|---|
| `0x00000000` | `NONE` | No fault |
| `0x00000001` | `CONTROL_OVERRUN` | 500 Hz control deadline exceeded |
| `0x00000002` | `DRIVER_DIAGNOSTIC` | Enabled EN/DIAG input asserted |
| `0x00000004` | `OVER_CURRENT` | Filtered current exceeded configured limit |
| `0x00000008` | `ENCODER_TIMEOUT` | No valid encoder transition while motion was demanded |
| `0x00000010` | `INVALID_CONFIGURATION` | Initialization or runtime configuration is invalid |
| `0x00000020` | `UNDER_VOLTAGE` | Driver VIN below configured limit |
| `0x00000040` | `OVER_VOLTAGE` | Driver VIN above configured limit |

Multiple bits may be set simultaneously.

### STOP_MODE

| Value | Name |
|---:|---|
| 0 | `COAST` |
| 1 | `BRAKE_TO_GROUND` |
| 2 | `BRAKE_TO_SUPPLY` |

### PROFILE_KIND

| Value | Name | Active fields |
|---:|---|---|
| 0 | `RAMP` | Target and duration |
| 1 | `SINE` | Mean, amplitude, frequency, and duration |
| 2 | `WAYPOINTS` | Point array and duration |

### CURRENT_CALIBRATION_ACTION

| Value | Name | Description |
|---:|---|---|
| 0 | `RESET` | Clear the in-RAM calibration workflow |
| 1 | `CAPTURE_POINT_1` | Capture first voltage/reference-current pair |
| 2 | `CAPTURE_POINT_2` | Capture second pair and calculate candidate |
| 3 | `SAVE` | Apply and persist the valid candidate |
| 4 | `CANCEL` | Discard candidate and retain saved calibration |
| 5 | `REQUEST_STATUS` | Emit `CURRENT_CALIBRATION_STATUS` |

### CHARACTERIZATION_ACTION_FLAGS (bitmask)

| Value | Name | Description |
|---:|---|---|
| `0x00` | `DISCARD` | Discard pending result |
| `0x01` | `SAVE` | Save deadband and velocity result |
| `0x02` | `APPLY_ACCELERATION` | Also apply recommended acceleration limit |
| `0x04` | `APPLY_JERK` | Also apply recommended jerk limit |

Bits 1 and 2 are meaningful only with `SAVE`. All other bits are invalid.

### CHARACTERIZATION_STAGE

| Value | Name |
|---:|---|
| 0 | `IDLE` |
| 1 | `FORWARD_DEADBAND` |
| 2 | `PAUSE_BEFORE_REVERSE_DEADBAND` |
| 3 | `REVERSE_DEADBAND` |
| 4 | `PAUSE_BEFORE_FORWARD_MAXIMUM` |
| 5 | `FORWARD_MAXIMUM` |
| 6 | `PAUSE_BEFORE_REVERSE_MAXIMUM` |
| 7 | `REVERSE_MAXIMUM` |

## Parameter IDs

All values are transported as `f32`, including integer and Boolean settings. Firmware rejects fractional values for integer parameters and validates the complete candidate configuration before applying it.

| ID | Name | Units / values | Description |
|---:|---|---|---|
| 1 | `BAUD` | bit/s | UART rate after reboot |
| 2 | `CONTROL_PERIOD_US` | µs | Deterministic control-loop period |
| 3 | `COUNTS_PER_REVOLUTION` | counts/rev | Output-shaft quadrature resolution |
| 4 | `STREAM_RATE_HZ` | Hz | Telemetry rate, additionally bounded by UART capacity |
| 5 | `KP` | — | Incremental proportional gain |
| 6 | `KI` | 1/s | Incremental integral gain |
| 7 | `KD` | — | Incremental derivative gain |
| 8 | `MAXIMUM_VELOCITY` | rad/s | Motion constraint |
| 9 | `MAXIMUM_ACCELERATION` | rad/s² | Motion constraint |
| 10 | `MAXIMUM_JERK` | rad/s³ | Motion constraint |
| 11 | `MAXIMUM_CURRENT` | A | Software current trip |
| 12 | `MAXIMUM_DUTY` | 0–1 | PWM constraint |
| 13 | `FORWARD_DEADBAND` | 0–1 | Forward breakaway duty |
| 14 | `REVERSE_DEADBAND` | 0–1 | Reverse breakaway duty |
| 15 | `MOTOR_DIRECTION` | `-1`, `1` | Electrical direction multiplier |
| 16 | `SUPPLY_DIVIDER_GAIN` | V/V | VIN reconstruction gain |
| 17 | `SUPPLY_INPUT_OFFSET` | V | VIN ADC offset |
| 18 | `MINIMUM_SUPPLY_VOLTAGE` | V | Undervoltage threshold |
| 19 | `MAXIMUM_SUPPLY_VOLTAGE` | V | Overvoltage threshold |
| 20 | `CURRENT_GAIN` | A/V | Current-sense gain |
| 21 | `CURRENT_OFFSET` | V | Current-sense voltage offset |
| 22 | `ENCODER_TIMEOUT_MS` | ms | Encoder watchdog duration |
| 23 | `ENCODER_TIMEOUT_VELOCITY` | rad/s | Watchdog activation threshold |
| 24 | `MAXIMUM_FEEDBACK_CORRECTION` | — | Retained for compatibility; inactive |
| 25 | `ESTIMATOR_MINIMUM_COUNTS` | counts | Retained for compatibility; inactive |
| 26 | `ESTIMATOR_MAXIMUM_WINDOW_US` | µs | Retained for compatibility; inactive |
| 27 | `ESTIMATOR_STALE_TIMEOUT_US` | µs | Retained for compatibility; inactive |
| 28 | `CURRENT_SENSE_ENABLED` | `0`, `1` | Enables software overcurrent protection |
| 29 | `CURRENT_FILTER_CUTOFF_HZ` | Hz | Current first-order low-pass cutoff; valid 0.1–200 |
| 30 | `ZERO_INDEX_MINIMUM_INTERVAL_US` | µs | Time debounce; valid 100–1,000,000 |
| 31 | `ZERO_INDEX_CORRECTION_GAIN` | 0–1 | Fractional phase correction |
| 32 | `ZERO_INDEX_MINIMUM_SEPARATION_REVOLUTIONS` | rev | Encoder-distance debounce; valid 0–0.95 |
| 33 | `CHARACTERIZATION_DYNAMICS_FILTER_CUTOFF_HZ` | Hz | Dynamics filter cutoff; valid 0.5–100 |
| 34 | `CHARACTERIZATION_DYNAMICS_QUANTILE` | 0–1 | Robust quantile; valid 0.80–0.99 |
| 35 | `CHARACTERIZATION_RECOMMENDATION_SAFETY_FACTOR` | 0–1 | Recommendation multiplier; valid 0.10–1.0 |

Lower current-filter cutoff reduces noise but delays software overcurrent detection. Hardware current limiting and a correctly sized fuse remain mandatory.

For zero indexing, the first accepted pulse always establishes absolute phase. Subsequent pulses must satisfy both time and encoder-travel debounce. A correction gain of zero trusts encoder counts entirely after the initial reference; one snaps fully to each accepted pulse.

## Operational sequences

### Initial connection

1. Open the UART at the selected baud.
2. Wait for a CRC-valid `HEARTBEAT` and verify protocol/settings compatibility.
3. Send `START_STREAM`.
4. Collect the resulting settings, profiles, load configuration, pending characterization result, and ACK.
5. Treat telemetry as active only after a successful ACK.

### Run a stored profile

1. Optionally send `SELECT_PROFILE` and wait for ACK.
2. Send `ARM` and wait for `OK`.
3. Start recording locally.
4. Send `START_RUN` and wait for `OK`/running telemetry.
5. Send `STOP_RUN` when required; the firmware also stops at profile completion.

### Recover from a fault

1. Remove the physical cause.
2. Send `CLEAR_FAULTS`.
3. An `OK` result means the resampled safety inputs are healthy and the device is disarmed.
4. `UNSAFE_STATE` means at least one active condition remains; inspect the next heartbeat or telemetry fault mask.

### Telemetry bandwidth

Firmware reserves 30% of UART capacity for heartbeats, acknowledgements, settings, and terminal traffic. It computes the maximum stream rate using the 96-byte framed telemetry packet and 10 UART bits per byte. At 115200 bit/s, the accepted telemetry range is 1–84 Hz.

UART baud cannot be reliably autodetected after boot. A browser may probe a short supported-rate list by reopening the selected port and waiting for a CRC-valid heartbeat. A persisted baud update is acknowledged at the old rate and takes effect after reboot.

## ASCII console

Binary packets and newline-terminated printable ASCII commands share the upload UART. The parser is incremental and non-blocking.

- Send `help` for the firmware-owned command list and usage text.
- Commands are limited to 159 characters and 10 whitespace-separated arguments.
- Quoting is not supported.
- No dynamic allocation occurs in command parsing.
- ASCII bytes that begin with `0xB5 0x62` are interpreted as a binary frame prefix.

The binary protocol is authoritative for the browser GUI. The ASCII console is intended for commissioning and debugging.

## Compatibility

Wire protocol version and Preferences schema are separate concepts. Protocol version 1 currently transports settings schema 13.

| Settings schema | Change |
|---:|---|
| 11 | Added telemetry-rate migration |
| 12 | Expanded rotor-load storage from 8 to 12 slots |
| 13 | Added characterization dynamics filter, quantile, and safety-factor settings |

The firmware migrates supported older Preferences layouts while preserving prior values. For append-only response changes, hosts should gate optional decoding by payload size. In particular:

- `TELEMETRY` rotor-position fields begin at byte 72.
- `CHARACTERIZATION_RESULT` dynamics fields begin at byte 16.
- The schema-13 `SETTINGS` extension begins at byte 131.

Message IDs, packed field order, fixed-array capacities, and CRC behavior are protocol contracts. Any change to them requires synchronized firmware, browser, tests, and this document.
