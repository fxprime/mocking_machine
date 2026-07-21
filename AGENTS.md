# Mocking Machine engineering rules

- Keep the 500 Hz control path non-blocking and allocation-free after `setup()`.
- Use `esp_timer_get_time()`/`uint64_t` for control and sensor timestamps. Never compare absolute timestamps with signed arithmetic.
- Hardware limits live in `MachineSettings`; do not add pin, timing, gain, or safety magic numbers in source files.
- Every serial command must be registered in the static command table, validate all arguments, return an explicit result, and have a matching help description.
- Binary protocol changes require a message ID, bounded payload, CRC test, and an update to `docs/protocol.md`.
- Motor commands must pass through the safety state machine. No command handler may write PWM pins directly.
- Persisted structures require a schema-version bump when their layout or meaning changes.
- Use Mermaid for architecture, flow, state, and wiring diagrams in Markdown. Reserve text/code fences for literal source, commands, formulas, and raw output—not ASCII-art charts.
