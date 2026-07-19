# Low-speed velocity control

Velocity profiles are logically one-direction. `motor_direction` selects the physical
rotation direction, while the closed-loop controller output is constrained to non-negative
duty. Bidirectional output remains available only through the guarded raw Motor Test command.

For a positive desired velocity, the controller uses:

1. A physical-duty feedforward interpolated from the characterized forward breakaway duty to
   maximum duty at the characterized maximum velocity.
2. Incremental P/I/D feedback accumulated as a correction around that feedforward.
3. `max_feedback_correction` to bound the correction in either direction.
4. Final saturation from zero to configured maximum duty.

The final value is sent through the raw driver path because the feedforward already includes
breakaway compensation. This avoids applying the deadband offset twice and prevents a small
controller zero crossing from becoming reverse breakaway torque.

Velocity estimation blends two measurements:

- Count-over-time above `estimator_min_counts` within `estimator_max_window_us`.
- The average period across the five most recent same-direction encoder edges below that
  count-rate threshold. The history resets on reversal, so timing from opposite directions is
  never mixed. This rejects single-edge timestamp jitter without adding filter lag.

The blend changes continuously with expected counts per estimation window. If no valid edge
arrives within `estimator_stale_timeout_us`, the raw estimate becomes zero and the configured
velocity low-pass filter controls its decay.
