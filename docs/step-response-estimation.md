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

The estimate is not valid merely because the calculation completed. Review the accepted-window count and use a varying target such as multisine, chirp, or PRBS. A long constant-speed plateau provides little broadband excitation.

Reference: <https://github.com/ArduPilot/WebTools/blob/main/PIDReview/PIDReview.js#L1030-L1205>
