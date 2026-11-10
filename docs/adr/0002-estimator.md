# ADR-0002: State estimator, first version

**Status:** Accepted (M1b) · **Date:** 2026-11-09 · **Design reference:** DESIGN §4.3, §4.7, §5.2, §5.4

## Context

M1b replaces the simulator's ground-truth state with an estimate made from noisy sensors. DESIGN §4.7 fixes the architecture: an error-state EKF in the style of PX4's EKF2, IMU deltas integrated at the rate loop, prediction and fusion at a fixed 100–250 Hz on a delayed time horizon, an output predictor that brings the state to the current tick, and gated innovations. It also says the equations are derived symbolically and emitted as generated Rust in f32 or f64. This ADR records how the first version meets that, and where it deliberately falls short for now.

## Decision

1. **States.** Sixteen nominal states: attitude quaternion (body→world), NED velocity and position, gyro bias and accelerometer bias in the body frame. Fifteen error states, with the attitude error a **world-frame** rotation vector, `q_true = Exp(δθ) ⊗ q` (Solà 2017, "global angular error"). Earth magnetic field, magnetometer bias and wind states are left out until the sensors that need them (S2).

2. **Equations are written by hand, not generated.** The transition matrix and the measurement Jacobians are written out in `estimator/ekf.rs` and checked against central-difference derivatives of the nominal model in f64, to 1e-6 per entry. The filter is generic over `f32`/`f64` (`Ekf<T: Real>`); the core instantiates `f32`. Code generation from one symbolic source (DESIGN §4.7) is deferred until the state grows enough for hand derivation to become a risk. The derivative tests stay as the check on whatever replaces the hand-written code.

3. **Timing.** The estimator task runs every base tick. It integrates IMU samples into deltas and propagates the output predictor. The filter step runs every `Profile::filter_period_ticks`: 250 Hz on both profiles (every 4 ticks on the F405, every 16 on the H743). The filter runs `delay_horizon_us` (200 ms) behind real time, so every measurement is fused at the time it was taken. Measurements that arrive later than that are dropped and counted.

4. **Output predictor.** A current-time state integrated with every IMU sample, pulled towards the EKF by comparing both at the same delayed time: attitude by a world-frame correction rate, velocity and position by a proportional correction applied to the stored history, all with time constant `output_tau_s` (0.25 s). EKF resets (GPS first fix, GPS timeout) shift the predictor and its history by the same amount.

5. **Measurements.**
   - **GPS:** north/east position and NED velocity, gated as groups. The first fix resets position and velocity. If GPS is rejected continuously for `gps_timeout_us` (5 s), or returns after aiding has timed out, position and velocity reset to it. Position rejections are timed on their own as well: a position jump with a consistent velocity would otherwise keep the whole-fix timer cleared for ever. M1b's GPS sample is already in the local NED frame; latitude/longitude conversion arrives with the GPS driver.
   - **Barometer:** the height reference. Height is measured from the barometer reading at alignment. GPS height is not fused yet.
   - **Magnetometer:** fused as the horizontal direction of the measured field, rotated into the world frame with the estimated attitude and compared with the configured declination. Field strength and dip are not needed. The Jacobian is the exact first-order one, `ν = δθz − m_z(m_x·δθx + m_y·δθy)/(m_x² + m_y²)`: a tilt error leaks into the heading in proportion to the field's dip (×1.7 at 60° inclination). The usual "innovation is a yaw error" simplification drops those tilt terms. In testing that made the filter overconfident in yaw (0.4° claimed, 2.5° actual) and let the error feed back into tilt and accelerometer bias whenever a gust accelerated the vehicle. Hard/soft-iron calibration and field states arrive with S2's magnetometer errors.
   - **Gates** in standard deviations: GPS 5, baro 5, heading 3 (PX4 defaults).

6. **Alignment.** The vehicle must be still for `align_time_us` (1 s). Tilt comes from the mean specific force, heading from the mean magnetometer field, and the initial gyro bias from the mean rate over the still window. Without that last step the turn-on bias drifts the heading by several degrees before the filter learns it.
   - **Heading is required.** If no usable magnetometer reading reached the still window, the filter starts without a heading, reports itself not aligned, and publishes no estimate. The first usable reading then sets the heading directly. Starting at heading 0 with an ordinary variance would instead make the gate reject every later reading.
   - **GPS fixes from before the still window are discarded**, so a vehicle that was carried and then set down does not start from a position and velocity measured while it was moving.

7. **Numerical hygiene.** Covariance is symmetrised and its diagonal clamped after every prediction and fusion. A non-finite diagonal resets the covariance and is counted. Bias states are clamped to PX4's limits (gyro 0.15 rad/s, accel 0.4 m/s²). The covariance-reset Jacobian after injection is neglected, as in PX4.

8. **Rate loop.** The rate controller uses the gyro through a second-order Butterworth low-pass (`Gains::gyro_cutoff_hz`, 80 Hz) minus the EKF's gyro bias.

9. **Accelerometer-bias prior.** `init_accel_bias_sigma` is 0.1 m/s², not PX4's 0.2. In hover a horizontal accelerometer bias cannot be told from a tilt error, so the filter's tilt uncertainty settles at about prior/g, and the estimate is free to wander that far. The simulated IMU's switch-on bias is 0.05 m/s² (a calibrated sensor); the prior allows twice that. Over 200 gust-hold seeds the worst attitude error in flight fell from 2.9° to 2.1°. What remains is a 3σ accelerometer bias seen through the magnetic dip. Yawing the vehicle or a better accelerometer calibration reduces it further.

10. **The estimate seen by controllers** exists once the filter is aligned (tilt and heading) and GPS has been fused once. Dead reckoning continues after a GPS timeout in flight; arming needs GPS to be aiding. Position-invalid handling and attitude-only flight belong to the M1c failsafes.

11. **IMU stall.** If no IMU sample arrives for 100 ms (ADR-0001's state-staleness limit), the estimator drops its solution and has to align again, which needs the vehicle still. The estimate is reported missing from that moment. Recovering an estimate in the air after an IMU gap belongs to the M1c failsafes.

## Consequences

- Prediction is a dense 15×15 `F·P·Fᵀ` at 250 Hz, about 1.7 M multiply-adds a second. That is fine for the H743 and should be measured on the F405 at bring-up. Exploiting the sparsity of F is the first optimisation if it is needed.
- The simulator's sensor models (S1: noise, bias drift, vibration, GPS latency and dropouts) exercise every path above. The M1b acceptance test flies the filter in gusts across 200 seeds on both profiles.
- PX4's EKF2 remains the benchmark reference once the PX4 SITL bridge exists (DESIGN §8).

## Deferred

- Code generation of the filter equations (DESIGN §4.7).
- GPS height fusion, antenna offset, latitude/longitude.
- 3-axis magnetometer fusion with field and bias states.
- Wind and drag-specific-force states (rotor-drag fusion as in PX4's `EKF2_MCOEF`).
- Selecting f64 on the H743 profile.
- A heading reset after sustained magnetometer rejection. It needs an independent check of heading (GPS velocity during acceleration) to be safe near magnetic disturbances.
- Keeping the covariance consistent when a bias state hits its clamp. Today the clamp is invisible to the covariance, as in PX4.
