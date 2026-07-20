# Serial protocol v1

The upload UART carries binary application frames and newline-terminated ASCII debug commands. Parsing is incremental and non-blocking.

## Frame

All multibyte values and payload scalars are little-endian.

| Offset | Size | Field |
|---:|---:|---|
| 0 | 1 | Sync `0xB5` |
| 1 | 1 | Sync `0x62` |
| 2 | 1 | Protocol version (`1`) |
| 3 | 1 | Flags |
| 4 | 2 | Message ID |
| 6 | 2 | Sequence |
| 8 | 2 | Payload size, maximum 512 |
| 10 | N | Payload |
| 10+N | 2 | CRC16/CCITT-FALSE |

CRC parameters are polynomial `0x1021`, initial value `0xFFFF`, no reflection, no final XOR. CRC covers version through the final payload byte; sync and CRC bytes are excluded.

## Implemented message IDs

| ID | Name | Direction | Payload bytes |
|---:|---|---|---:|
| `0x0001` | HEARTBEAT | device→host | 65 |
| `0x0002` | ACK | device→host | 3 |
| `0x0100` | GET_SETTINGS | host→device | 0 |
| `0x0101` | SETTINGS | device→host | 143 |
| `0x0110` | SET_CONTROLLER | host→device | 12 |
| `0x0111` | SET_DRIVER_DIAGNOSTIC | host→device | 1 |
| `0x0112` | SET_CURRENT_SENSE | host→device | 1 |
| `0x0113` | SET_PARAMETER | host→device | 7 |
| `0x0114` | SAVE_CONTROLLER | host→device | 12 |
| `0x0120` | SELECT_PROFILE | host→device | 2 |
| `0x0121` | GET_PROFILES | host→device | 0 |
| `0x0122` | PROFILE_CONFIGURATION | device→host | 168 |
| `0x0123` | SET_PROFILE | host→device | 169 |
| `0x0124` | CREATE_PROFILE | host→device | 169 |
| `0x0125` | SET_DEFAULT_PROFILE | host→device | 2 |
| `0x0130` | GET_LOAD_CONFIGURATION | host→device | 0 |
| `0x0131` | LOAD_CONFIGURATION | device→host | 86 |
| `0x0132` | SET_LOAD_CONFIGURATION | host→device | 86 |
| `0x0200` | START_RUN | host→device | 0 |
| `0x0201` | STOP_RUN | host→device | 0 |
| `0x0202` | MOTOR_TEST | host→device | 4 |
| `0x0203` | CLEAR_FAULTS | host→device | 0 |
| `0x0204` | ARM | host→device | 0 |
| `0x0205` | START_VELOCITY_TEST | host→device | 8 |
| `0x0210` | START_STREAM | host→device | 0 |
| `0x0211` | STOP_STREAM | host→device | 0 |
| `0x0220` | TELEMETRY | device→host | 84 |
| `0x0300` | CURRENT_CALIBRATION | host→device | 5 |
| `0x0301` | SUPPLY_VOLTAGE_CALIBRATION | host→device | 4 |
| `0x0302` | CURRENT_CALIBRATION_STATUS | device→host | 27 |
| `0x0310` | CHARACTERIZATION_RESULT | device→host | 32 |
| `0x0311` | CHARACTERIZATION_ACTION | host→device | 1 |
| `0x0312` | CHARACTERIZATION_STATUS | device→host | 10 |

`START_STREAM` first emits SETTINGS, the stored profile collection, LOAD_CONFIGURATION, then ACK and telemetry. It does not arm or rotate the motor.

### HEARTBEAT payload

`u64 uptime_us, u32 settings_schema, u32 faults, u8 state, char build_version[48]`

### SETTINGS payload

`u32 schema, u32 baud, u32 control_period_us, u32 CPR, u16 stream_rate_hz, u16 selected_profile_id, f32 kp, f32 ki, f32 kd, f32 vmax, f32 amax, f32 jmax, f32 imax, f32 max_duty, f32 start_duty_forward, f32 start_duty_reverse, u8 load_setting_id, u8 load_count, i8 motor_direction, u8 stop_mode, f32 supply_divider_gain, f32 supply_input_offset_v, f32 min_supply_voltage_v, f32 max_supply_voltage_v, u8 supply_voltage_pin, f32 current_gain_a_per_v, f32 current_offset_v, u8 current_sense_pin, u8 current_sense_enabled, u8 driver_diagnostic_enabled, u8 driver_diagnostic_pin, u32 encoder_timeout_ms, f32 encoder_timeout_velocity_rad_s, f32 max_feedback_correction, u8 estimator_min_counts, u32 estimator_max_window_us, u32 estimator_stale_timeout_us, f32 current_filter_cutoff_hz, u32 zero_index_min_interval_us, f32 zero_index_correction_gain, i8 encoder_direction, f32 zero_index_minimum_separation_revolutions, f32 characterization_dynamics_filter_cutoff_hz, f32 characterization_dynamics_quantile, f32 characterization_recommendation_safety_factor`

The feedback-correction and three estimator-tuning fields are retained for packet/NVS
compatibility but are ignored while the classic count-delta velocity path is active.

The encoder watchdog begins its timeout window when desired velocity first exceeds `encoder_timeout_velocity_rad_s`; time spent stationary before a run is not counted. Each valid quadrature transition refreshes encoder activity. Dropping below the threshold or stopping resets the window.

`SET_DRIVER_DIAGNOSTIC` carries `u8 enabled` (`0` or `1`). It is accepted while disarmed. Disabling is also accepted from a diagnostic-fault state so a disconnected input cannot lock out the machine. The setting is applied immediately and saved.

`SET_CURRENT_SENSE` carries `u8 enabled` (`0` or `1`). Current telemetry and calibration remain active when disabled, but overcurrent fault detection is bypassed. Protection is disabled by default and should only be enabled after the CS circuit is connected, filtered, and calibrated.

The parameter table also exposes `current_sense_enabled` as parameter 28,
`current_filter_cutoff_hz` as parameter 29, `zero_index_min_interval_us` as parameter 30,
`zero_index_correction_gain` as parameter 31, and
`zero_index_minimum_separation_revolutions` as parameter 32. Characterization dynamics
filter cutoff, robust quantile, and recommendation safety factor are parameters 33–35.
The current cutoff accepts 0.1–200 Hz. The zero-index interval accepts 100–1,000,000 µs;
its default is 5,000 µs. Rising edges closer than this to the last accepted zero edge are
ignored as bounce and counted in telemetry. Filtered current is used for both telemetry and
software overcurrent detection, so reducing the cutoff also increases protection delay.
Hardware current limiting and a correctly sized fuse remain mandatory.

The zero correction gain accepts 0–1 and defaults to 0.10. The first accepted index always
establishes the absolute reference. After that, encoder counts remain the primary position
source and each new index corrects only `gain × phase_error`. A gain of 0 disables subsequent
phase correction; 1 reproduces full snapping to every accepted index.

The zero-index separation accepts 0–0.95 revolutions and defaults to 0.50. After the first
accepted edge, a later edge must satisfy both the time interval and encoder-travel threshold.
Rejected edges increment only `zero_index_rejected_count`; they do not update the accepted
timestamp/count, increment `zero_index_sequence`, or correct rotor phase. Set the separation
to 0 only when intentionally disabling the encoder-distance bounce filter.

`SET_PARAMETER` carries `u16 parameter_id, f32 value, u8 persist`. It is accepted only while disarmed. Firmware applies the value to a copy of the complete settings structure, validates all cross-parameter constraints, optionally saves it, reconfigures the affected runtime modules, and returns ACK followed by SETTINGS. Parameter IDs are defined by the firmware/browser parameter table; unknown IDs are rejected.

`GET_PROFILES` emits one `PROFILE_CONFIGURATION` per stored profile. Streaming also sends the complete profile list after SETTINGS. The packed profile payload is `u16 id, u8 kind, char name[16], f32 target, f32 sine_mean, f32 sine_amplitude, f32 sine_frequency_hz, u32 duration_ms, u8 point_count`, followed by 16 fixed slots of `u32 time_ms, f32 velocity_rad_s`.

`SELECT_PROFILE` carries `u16 profile_id` and changes only the runtime selection for the next run. It never writes Preferences. `SET_DEFAULT_PROFILE` carries the same ID, is accepted only while disarmed, and saves both the default and current runtime selection. The SETTINGS `selected_profile_id` field always means the persisted default.

`GET_LOAD_CONFIGURATION` emits one packed `LOAD_CONFIGURATION`: `u8 setting_id, u8 count`, followed by 12 fixed entries of `u8 slot_id, u16 position_deg, f32 strength`. Slots are fixed at 30° increments (`position_deg = slot_id × 30`), must be unique, and strength is an integer-equivalent label from 1–10. `SET_LOAD_CONFIGURATION` uses the same bounded 86-byte payload, validates every active entry, and persists only while disarmed. Schema 12 expands stored load capacity from 8 to all 12 physical slots while migrating schema 10/11 settings.

`SET_PROFILE` and `CREATE_PROFILE` both prepend `u8 persist` to the same 168-byte profile payload and are accepted only while disarmed. `SET_PROFILE` preserves its original 169-byte wire format and requires an existing ID. `CREATE_PROFILE` requires an unused ID and appends the profile if fewer than eight profiles exist; an ID collision or a full collection is rejected without modifying any profile. Firmware also rejects invalid kinds, durations, point counts, out-of-order points, velocities outside the configured limit, and waypoint slopes above the configured acceleration limit. `persist=0` supports a temporary test; `persist=1` saves the resulting collection to Preferences.

`MOTOR_TEST` carries one `f32 signed_raw_duty`. It is accepted only while armed and fault-free, is bounded by configured `max_duty`, bypasses velocity control and deadband compensation, and expires after `command_timeout_ms` unless refreshed. Zero duty stops output immediately. `STOP_RUN` stops and disarms.

`CLEAR_FAULTS` stops and disarms, clears latched operational faults, then re-samples VIN and enabled EN/DIAG protection. It returns `OK` only when no active safety condition remains; initialization faults cannot be cleared without a successful reboot.

`ARM` performs the same safety checks as the ASCII `arm` command and acknowledges the state transition. Hosts should wait for its successful ACK before sending `START_RUN`.

`SET_CONTROLLER` applies `f32 kp, f32 ki, f32 kd` to RAM for testing and never writes Preferences. `SAVE_CONTROLLER` accepts the same payload only while disarmed, validates the complete settings structure, then applies and persists the gains.

`START_VELOCITY_TEST` carries `f32 target_velocity_rad_s, u32 duration_ms`. It starts a temporary constrained step profile only after an acknowledged `ARM`; velocity, acceleration, and jerk limits remain active.

### TELEMETRY payload

`u64 timestamp_us, u64 last_zero_timestamp_us, i64 encoder_count, i64 last_zero_encoder_count, f32 desired_rad_s, f32 measured_rad_s, f32 controller_output, f32 current_a, f32 supply_voltage_v, u32 faults, u16 profile_id, u8 load_setting_id, u8 state, f32 controller_p_delta, f32 controller_i_delta, f32 controller_d_delta, f32 rotor_position_deg, u32 zero_index_sequence, u32 zero_index_rejected_count`

The controller fields are the proportional, integral, and derivative contributions to the latest incremental-controller output update. They are zero when velocity control is inactive. `controller_output` is the saturated total actuator command. Rotor position is appended at byte 72 and is valid after `zero_index_sequence` becomes nonzero. It is calculated from encoder count plus a persistent phase offset, modulo CPR and adjusted by encoder direction. The first index initializes that offset; later index events apply only the configured fractional phase correction. `zero_index_rejected_count` counts rising edges rejected by the configured minimum interval. Hosts can remain backward compatible by checking payload length.

`SUPPLY_VOLTAGE_CALIBRATION` carries one `f32 reference_voltage_v`. Firmware averages 64 raw GPIO36 ADC readings, calculates the divider gain, applies it immediately, saves it to Preferences, and returns ACK followed by SETTINGS.

`CURRENT_CALIBRATION` carries `u8 action, f32 reference_current_a`. Actions are `0 Reset`, `1 Capture Point 1`, `2 Capture Point 2`, `3 Save`, `4 Cancel`, and `5 Request Status`. Each capture averages 64 raw ADC readings incrementally—one sample per control tick—so calibration cannot block the motor-control deadline. Point 2 must exceed Point 1 by at least 0.01 A and 0.001 V. Firmware calculates `gain = ΔI / ΔV` and `offset = V1 - I1 / gain`; the candidate is neither applied nor persisted until Save. Save stops and disarms the motor. Cancel preserves the previously saved calibration.

`CURRENT_CALIBRATION_STATUS` carries `u8 captured_mask, u8 capture_point, u8 last_result, f32 point1_voltage_v, f32 point1_reference_a, f32 point2_voltage_v, f32 point2_reference_a, f32 candidate_gain_a_per_v, f32 candidate_offset_v`. Bit 0 of the mask indicates Point 1; bit 1 indicates Point 2 and a valid candidate. `capture_point` is zero when idle and 1 or 2 while the corresponding 64-sample acquisition runs. `last_result` uses the standard result-code values.

`CHARACTERIZATION_RESULT` carries `f32 start_duty_forward, f32 start_duty_reverse, f32 max_velocity_forward_rad_s, f32 max_velocity_reverse_rad_s, f32 acceleration_forward_rad_s2, f32 acceleration_reverse_rad_s2, f32 jerk_forward_rad_s3, f32 jerk_reverse_rad_s3`. Dynamics are filtered online and represented by the configured robust quantile, independently for each full-duty direction. Results remain pending in RAM and are resent when streaming starts until explicitly reviewed.

`CHARACTERIZATION_ACTION` carries a one-byte flag mask: bit 0 saves the motor/velocity candidate, bit 1 opts into the recommended acceleration limit, and bit 2 opts into the recommended jerk limit. Zero discards. Dynamics recommendations use the weaker direction multiplied by the configured safety factor and can only lower existing limits. Lowering acceleration also constrains stored waypoint slopes before complete-settings validation. Characterization never writes Preferences automatically. An abort, stop, invalid result, or fault before completion leaves the previously saved settings unchanged.

`CHARACTERIZATION_STATUS` carries `u8 stage, u8 result_pending, f32 applied_duty, f32 measured_velocity_rad_s`. Firmware emits it at 10 Hz while the routine runs and on state transitions. Stages are `0 Idle`, `1 ForwardDeadband`, `2 PauseBeforeReverseDeadband`, `3 ReverseDeadband`, `4 PauseBeforeForwardMaximum`, `5 ForwardMaximum`, `6 PauseBeforeReverseMaximum`, and `7 ReverseMaximum`.

The host should also record the most recent HEARTBEAT build string and SETTINGS record in each exported dataset.

## ASCII commands

Type `help` to receive the firmware-owned list. Commands are bounded to 159 characters and 10 whitespace-separated arguments. No quoting or dynamic allocation is used.

Changing baud rate cannot be truly autodetected by a normal UART after boot. The browser should probe a short supported-rate list by reopening the selected port and waiting for a valid CRC heartbeat. A persisted baud change must acknowledge at the old rate and take effect after reboot; that command is reserved for the full parameter protocol.

Telemetry rate is bounded by UART capacity. Firmware reserves 30% of link bandwidth for heartbeats, acknowledgements, settings, and terminal traffic, then computes the maximum from the 96-byte telemetry frame and 10 UART bits per byte. At 115200 baud the accepted range is 1–84 Hz. Schema 11 introduced this rate migration, schema 12 expanded rotor-load storage, and schema 13 adds characterization-dynamics settings while preserving prior values.
