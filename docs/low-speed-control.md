# Classic velocity-control behavior

The experimental low-speed estimator and feedforward-bounded controller were rolled back.
The firmware now uses the original control behavior:

1. Velocity is calculated from the encoder count change over every actual control interval.
2. The configured velocity low-pass filter is applied to that measurement.
3. Incremental P, I, and D contributions accumulate directly into a signed controller output.
4. The motor driver applies the characterized forward or reverse deadband mapping.

Encoder edge timestamps are still used by the encoder-activity safety watchdog, but they are
not used to calculate velocity. The legacy `max_feedback_correction`,
`estimator_min_counts`, `estimator_max_window_us`, and `estimator_stale_timeout_us` fields
remain in the settings packet and NVS layout for compatibility with existing firmware data.
They are not used by the classic estimator/controller path and are hidden from the GUI.
