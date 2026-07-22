# Closed-loop step-response estimation

The PID tuning page estimates the closed-loop transfer from telemetry `desired_velocity_rad_s` to `measured_velocity_rad_s` and derives its unit-step response.

The estimator follows the method used by ArduPilot WebTools PIDReview:

1. Resample timestamped telemetry onto a uniform grid using the median sample interval.
2. Split target and actual velocity into Hann-windowed FFT blocks with 15/16 overlap.
3. Reject blocks whose target amplitude is below the configured excitation threshold.
4. Calculate the regularized Wiener transfer estimate `H = Y·conj(X) / (X·conj(X) + noise)`.
5. Inverse-transform `H` to obtain the impulse response.
6. Cumulatively sum the impulse response to obtain the unit-step response.
7. Average all accepted block estimates.

The noise spectrum uses the PIDReview Gaussian-integral shape and nominal scale. The GUI's **PIDReview noise factor** multiplies that nominal spectrum. A value of `1` matches the nominal scale; decreasing it trusts the captured input spectrum more, while increasing it suppresses poorly excited frequencies more strongly.

The estimate is not valid merely because the calculation completed. Review the accepted-window count and use a varying target such as the tuning page's repeated random steps, multisine, chirp, or PRBS. A long constant-speed plateau provides little broadband excitation. The repeated manual test generates 2–16 previewed levels, rejects adjacent changes smaller than 20% of the selected velocity span, and records the entire sequence as one run.

The implementation runs entirely in the browser after a response test; firmware does not calculate or transmit a step response. Input is limited to the latest captured tuning run. Resampling is capped at 20,000 points, automatic FFT windows are powers of two up to 1024 samples, and the displayed response horizon is constrained to 0.05–5 s. The result is an identification estimate, not a replacement for reviewing the original velocity-tracking plot.

Reference: <https://github.com/ArduPilot/WebTools/blob/main/PIDReview/PIDReview.js#L1030-L1205>
