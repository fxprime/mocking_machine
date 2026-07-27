# Velocity estimation and low-speed behavior

`velocity_estimator_method` selects the active algorithm: `0` is the count-delta low-pass,
`1` is the characterized motor-model Kalman observer, and `2` is encoder-only windowed
acceleration prediction. Changing the method is accepted only while disarmed. Method `1` requires
a valid saved motor characterization. While the machine remains `DISARMED`, method `1` temporarily
uses the count-delta low-pass so hand-driven shaft motion is reported without the powered-motor
model pulling the estimate toward zero. Arming restores the unchanged Kalman path.

## Fallback implementation

Before motor characterization, the 500 Hz control path uses a count-delta estimator:

1. Snapshot the signed 64-bit quadrature count.
2. Calculate count change over the actual elapsed control interval.
3. Convert counts to rad/s using `counts_per_output_revolution` and `encoder.direction`.
4. Apply a first-order low-pass filter with `alpha = dt / (tau + dt)`; the default `tau` is 25 ms.
5. Feed the filtered result to the incremental controller.

Encoder edge timestamps are not used to calculate velocity. They are reserved for the encoder-activity watchdog, which starts only when the machine is `RUNNING` and desired velocity exceeds `encoder_timeout_velocity_rad_s`.

## Characterized motor observer

After characterization, the estimator predicts each tick from applied duty, direction-specific
motor gain/time constant, and a slowly varying load-disturbance state. It performs a Kalman encoder
correction after `estimator_min_counts`, after `estimator_max_window_us` for a non-empty window, or
after `estimator_stale_timeout_us` to establish zero motion. Count-less 2 ms ticks therefore do not
become independent zero-velocity measurements. Encoder quantization still limits the available
information at very low speed. Consequently:

- control output can be stable while the displayed velocity still looks quantized;
- incorrect CPR makes both velocity and position wrong by a constant scale factor;
- incorrect model parameters cause prediction error until encoder feedback corrects it;
- raising gains to chase quantization can recreate low-speed chatter.

Calibrate output-shaft CPR first, then tune the controller using recorded desired/measured velocity. The current firmware intentionally does not blend count-delta velocity with edge-period estimates.

## Controller and driver path

Incremental P, I, and D contributions accumulate into a saturated signed command. Integral contribution that would push an already saturated output farther into saturation is rejected. The VNH2SP30 driver then maps nonzero command magnitude through the characterized forward or reverse breakaway duty. Raw motor tests bypass this deadband mapping.

For dragged position moves, the outer position loop has a separate velocity-domain rule:
inside the configured angular tolerance it requests exactly zero; outside tolerance it
requests at least the configured minimum usable forward or reverse velocity. Position mode
keeps acceleration limiting but bypasses jerk limiting before the incremental velocity loop.

When jerk limiting is enabled, the velocity motion limiter estimates the requested profile slope and uses a jerk-bounded switching trajectory to converge its velocity and acceleration states. Feasible constant-slope waypoint segments settle to their requested acceleration without repeatedly snapping acceleration to zero, preventing periodic ripple in the desired velocity while preserving the configured velocity, acceleration, and jerk limits in both directions. Disabling jerk limiting bypasses the jerk-bounded trajectory but continues to enforce maximum velocity and acceleration.

`estimator_min_counts`, `estimator_max_window_us`, and `estimator_stale_timeout_us` control Kalman
measurement timing and the encoder samples used by method `2`. The windowed predictor stores each
accepted window velocity and timestamp in a fixed-capacity circular buffer. Once
At every accepted sample, corrected velocity is the arithmetic mean of every velocity in the
circular history. Acceleration is the mean of the timestamp-aware slopes between every adjacent
pair. The configured size is 2–32 samples (default 5); larger values reduce noise but add lag.
Between samples, prediction is always anchored to the most recent corrected mean and elapsed time
since that correction; it is replaced rather than accumulated when the next sample arrives.
Acceleration is clamped to `max_acceleration_rad_s2`, and no motor duty is used.
`max_feedback_correction` remains a compatibility field and is ignored.
