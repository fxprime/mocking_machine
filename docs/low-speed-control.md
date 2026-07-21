# Velocity estimation and low-speed behavior

## Current implementation

The 500 Hz control path uses a count-delta estimator:

1. Snapshot the signed 64-bit quadrature count.
2. Calculate count change over the actual elapsed control interval.
3. Convert counts to rad/s using `counts_per_output_revolution` and `encoder.direction`.
4. Apply a first-order low-pass filter with `alpha = dt / (tau + dt)`; the default `tau` is 25 ms.
5. Feed the filtered result to the incremental controller.

Encoder edge timestamps are not used to calculate velocity. They are reserved for the encoder-activity watchdog, which starts only when the machine is `RUNNING` and desired velocity exceeds `encoder_timeout_velocity_rad_s`.

## Expected low-speed limitation

When the shaft moves slowly enough that a 2 ms control interval contains no encoder count, the raw estimate is zero for that tick. A later count produces a short nonzero pulse. The low-pass filter smooths this sequence but cannot recover information that the encoder did not sample. Consequently:

- control output can be stable while the displayed velocity still looks quantized;
- incorrect CPR makes both velocity and position wrong by a constant scale factor;
- increasing filter time reduces ripple but adds feedback delay;
- raising gains to chase quantization can recreate low-speed chatter.

Calibrate output-shaft CPR first, then tune the controller using recorded desired/measured velocity. The current firmware intentionally does not blend count-delta velocity with edge-period estimates.

## Controller and driver path

Incremental P, I, and D contributions accumulate into a saturated signed command. Integral contribution that would push an already saturated output farther into saturation is rejected. The VNH2SP30 driver then maps nonzero command magnitude through the characterized forward or reverse breakaway duty. Raw motor tests bypass this deadband mapping.

The experimental low-speed estimator fields `max_feedback_correction`, `estimator_min_counts`, `estimator_max_window_us`, and `estimator_stale_timeout_us` remain in the settings packet and Preferences layout for compatibility. They are validated and can be transported by parameter IDs 24–27, but the active estimator ignores them and the GUI hides them.
