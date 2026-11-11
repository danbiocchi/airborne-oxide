// SPDX-License-Identifier: AGPL-3.0-or-later
//! Vehicle configuration. On hardware this comes from parameters (§4.14);
//! in the simulator it is built from the airframe file.

use crate::math::Vec3;

/// Propeller spin direction, seen from above.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Spin {
    /// Counter-clockwise: reaction torque is positive yaw (nose right).
    Ccw,
    /// Clockwise: reaction torque is negative yaw.
    Cw,
}

impl Spin {
    pub const fn sign(self) -> f32 {
        match self {
            Spin::Ccw => 1.0,
            Spin::Cw => -1.0,
        }
    }
}

/// One rotor, thrusting along body −z (up).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Rotor {
    /// Position from the centre of mass, m (x forward, y right).
    pub x: f32,
    pub y: f32,
    pub spin: Spin,
    /// Maximum thrust at the nominal battery voltage, N.
    pub max_thrust_n: f32,
    /// Reaction torque per newton of thrust, m.
    pub torque_coeff_m: f32,
}

/// Controller gains, in physical units.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Gains {
    /// Rate loop: angular acceleration demand per rad/s of error, 1/s.
    pub rate_p: Vec3<f32>,
    /// Rate loop integral, 1/s².
    pub rate_i: Vec3<f32>,
    /// Rate loop derivative (on measurement), dimensionless.
    pub rate_d: Vec3<f32>,
    /// Derivative low-pass cutoff, Hz.
    pub rate_d_cutoff_hz: f32,
    /// Gyro low-pass cutoff for the rate loop (second-order Butterworth),
    /// Hz. Zero or negative turns it off.
    pub gyro_cutoff_hz: f32,
    /// Attitude loop: rate demand per rad of error, 1/s.
    pub attitude_p: Vec3<f32>,
    /// Velocity loop: acceleration demand per m/s of error, 1/s (x, y, z).
    pub velocity_p: Vec3<f32>,
    /// Velocity loop integral, 1/s².
    pub velocity_i: Vec3<f32>,
    /// Velocity loop derivative (on measurement), dimensionless.
    pub velocity_d: Vec3<f32>,
    /// Position loop: velocity demand per m of error, 1/s.
    pub position_p: Vec3<f32>,
}

impl Default for Gains {
    /// Starting gains for a small multirotor; auto-tuned per airframe later.
    ///
    /// The outer loops were chosen on a reduced model (position P → velocity
    /// PI → acceleration with a 0.12 s lag, 50 Hz, one-period delay) for
    /// ~1 % overshoot and ~1.1 s settling on a 1 m step. PX4-style defaults
    /// (0.95 / 1.8 / 0.4) leave a slow closed-loop pole near −0.2 rad/s that
    /// gives a 14 % overshoot and a 4 s tail without trajectory smoothing.
    fn default() -> Self {
        Self {
            rate_p: Vec3::new(28.0, 28.0, 12.0),
            rate_i: Vec3::new(40.0, 40.0, 10.0),
            rate_d: Vec3::new(0.3, 0.3, 0.0),
            rate_d_cutoff_hz: 60.0,
            gyro_cutoff_hz: 80.0,
            attitude_p: Vec3::new(6.5, 6.5, 3.0),
            velocity_p: Vec3::new(4.0, 4.0, 4.0),
            velocity_i: Vec3::new(0.4, 0.4, 0.8),
            velocity_d: Vec3::new(0.0, 0.0, 0.0),
            position_p: Vec3::new(1.5, 1.5, 1.5),
        }
    }
}

/// Flight limits.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Limits {
    /// Maximum tilt in position and offboard modes, rad.
    pub max_tilt_rad: f32,
    /// Maximum tilt in stabilized mode, rad.
    pub max_tilt_stabilized_rad: f32,
    /// Maximum body rates, rad/s.
    pub max_rate: Vec3<f32>,
    /// Maximum horizontal speed, m/s.
    pub max_speed_xy: f32,
    pub max_speed_up: f32,
    pub max_speed_down: f32,
    /// Yaw rate at full yaw stick, rad/s.
    pub max_yaw_rate: f32,
    /// Limit on the rate integral's contribution, rad/s².
    pub rate_integral_limit: f32,
    /// Limit on the velocity integral's contribution, m/s².
    pub velocity_integral_limit: f32,
    /// Minimum collective thrust while flying, as a fraction of hover.
    pub min_thrust_fraction_of_hover: f32,
    /// Motor command while armed with no demand (keeps props spinning).
    pub idle_command: f32,
}

impl Default for Limits {
    fn default() -> Self {
        Self {
            max_tilt_rad: 35.0_f32.to_radians(),
            max_tilt_stabilized_rad: 45.0_f32.to_radians(),
            max_rate: Vec3::new(3.8, 3.8, 3.5),
            max_speed_xy: 5.0,
            max_speed_up: 3.0,
            max_speed_down: 1.5,
            max_yaw_rate: 2.0,
            rate_integral_limit: 30.0,
            velocity_integral_limit: 3.0,
            min_thrust_fraction_of_hover: 0.1,
            idle_command: 0.05,
        }
    }
}

/// Where the state estimate comes from (DESIGN §4.7, ADR-0002).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum EstimatorSource {
    /// The `state` input is the estimate: simulator truth or motion capture.
    #[default]
    External,
    /// The error-state EKF on IMU, GPS, barometer and magnetometer.
    Ekf,
}

/// EKF tuning (ADR-0002). Noise values are 1σ. Defaults follow PX4 EKF2
/// where a counterpart exists.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EkfParams {
    /// How far the filter runs behind real time, µs. Must exceed the largest
    /// sensor latency (GPS); older measurements are dropped.
    pub delay_horizon_us: u32,
    /// Gyro process noise, rad/s (PX4 `EKF2_GYR_NOISE`).
    pub gyro_noise: f32,
    /// Accelerometer process noise, m/s² (`EKF2_ACC_NOISE`).
    pub accel_noise: f32,
    /// Gyro bias random walk, rad/s² (`EKF2_GYR_B_NOISE`).
    pub gyro_bias_noise: f32,
    /// Accelerometer bias random walk, m/s³ (`EKF2_ACC_B_NOISE`).
    pub accel_bias_noise: f32,
    /// Lower bound on GPS horizontal position noise, m. The receiver's
    /// reported accuracy is used when it is larger.
    pub gps_pos_noise_m: f32,
    /// Lower bound on GPS velocity noise, m/s.
    pub gps_vel_noise_mps: f32,
    /// Barometric height noise, m.
    pub baro_noise_m: f32,
    /// Magnetic heading noise, rad.
    pub yaw_noise_rad: f32,
    /// Innovation gates, in standard deviations.
    pub gps_gate: f32,
    pub baro_gate: f32,
    pub yaw_gate: f32,
    /// Magnetic declination, rad, east positive.
    pub mag_declination_rad: f32,
    /// Initial 1σ uncertainties after alignment.
    pub init_tilt_sigma_rad: f32,
    pub init_yaw_sigma_rad: f32,
    pub init_vel_sigma_mps: f32,
    pub init_pos_sigma_m: f32,
    pub init_gyro_bias_sigma: f32,
    /// Accelerometer switch-on bias, m/s². In hover a horizontal
    /// accelerometer bias cannot be told from a tilt error, so this prior
    /// bounds how far the tilt estimate can wander (0.1 m/s² ≈ 0.6°). The
    /// default assumes a calibrated IMU (residual about 0.05 m/s²) and
    /// allows twice that; PX4's 0.2 (`EKF2_ABIAS_INIT`) is for uncalibrated
    /// sensors.
    pub init_accel_bias_sigma: f32,
    /// The vehicle must sit still this long before the filter aligns, µs.
    pub align_time_us: u32,
    /// GPS aiding stops after this long without an accepted fix, µs. Also
    /// how long horizontal velocity stays valid (ADR-0003 §1).
    pub gps_timeout_us: u32,
    /// Horizontal position stays valid this long after the last accepted
    /// GPS fix, µs.
    pub pos_timeout_us: u32,
    /// Height stays valid this long after the last accepted barometer
    /// reading, µs.
    pub baro_timeout_us: u32,
    /// Heading stays valid this long after the last accepted magnetometer
    /// reading, µs.
    pub mag_timeout_us: u32,
    /// Output-predictor correction time constant, s.
    pub output_tau_s: f32,
}

impl Default for EkfParams {
    fn default() -> Self {
        Self {
            delay_horizon_us: 200_000,
            gyro_noise: 0.015,
            accel_noise: 0.35,
            gyro_bias_noise: 1.0e-3,
            accel_bias_noise: 3.0e-3,
            gps_pos_noise_m: 0.5,
            gps_vel_noise_mps: 0.3,
            baro_noise_m: 0.5,
            yaw_noise_rad: 0.15,
            gps_gate: 5.0,
            baro_gate: 5.0,
            yaw_gate: 3.0,
            mag_declination_rad: 0.0,
            init_tilt_sigma_rad: 0.1,
            init_yaw_sigma_rad: 0.3,
            init_vel_sigma_mps: 0.5,
            init_pos_sigma_m: 1.0,
            init_gyro_bias_sigma: 0.02,
            init_accel_bias_sigma: 0.1,
            align_time_us: 1_000_000,
            gps_timeout_us: 5_000_000,
            pos_timeout_us: 1_500_000,
            baro_timeout_us: 5_000_000,
            mag_timeout_us: 30_000_000,
            output_tau_s: 0.25,
        }
    }
}

/// Estimator selection and tuning.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct EstimatorConfig {
    pub source: EstimatorSource,
    pub ekf: EkfParams,
}

/// Failsafe timings, thresholds and flight-action settings (ADR-0003).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FailsafeParams {
    /// RC counts as lost after this long without a valid frame, µs.
    pub rc_timeout_us: u32,
    /// RC must be valid this long without a gap before pilot input counts
    /// (arming, override), s.
    pub rc_valid_min_s: f32,
    /// After RC loss in a pilot mode: hold this long, then return, s.
    pub rc_loss_hold_s: f32,
    /// The ground station counts as lost after this long without a
    /// heartbeat, µs.
    pub gcs_timeout_us: u32,
    /// Offboard with no human link: continue this long, then return, s.
    pub no_link_rtl_s: f32,
    /// The brain counts as lost when its progress counter has not advanced
    /// for this long, µs.
    pub brain_timeout_us: u32,
    /// After brain loss: hold this long, then return, s.
    pub brain_loss_rtl_s: f32,
    /// Tier-2 setpoints count as stale after this long, µs.
    pub setpoint_timeout_us: u32,
    /// After losing position with nobody flying: hold this long, then
    /// land, s.
    pub gps_loss_land_s: f32,
    /// A Hold ends when its reasons have been clear this long, s.
    pub clear_hysteresis_s: f32,
    /// Battery low and critical thresholds, V per cell, on the filtered,
    /// load-compensated voltage.
    pub battery_low_v: f32,
    pub battery_critical_v: f32,
    /// The voltage must stay below a threshold this long to latch, s.
    pub battery_debounce_s: f32,
    /// Battery voltage low-pass time constant, s.
    pub battery_filter_tau_s: f32,
    /// Geofence radius around home, m. Zero disables it.
    pub fence_radius_m: f32,
    /// Geofence height above home, m. Zero disables it.
    pub fence_altitude_m: f32,
    /// Loss of control: attitude error above this for
    /// `loc_error_s`, rad.
    pub loc_error_rad: f32,
    pub loc_error_s: f32,
    /// Loss of control: tilt above this for `loc_tilt_s`, rad.
    pub loc_tilt_rad: f32,
    pub loc_tilt_s: f32,
    /// Return altitude above home, m.
    pub rtl_altitude_m: f32,
    /// Return ground speed, m/s.
    pub rtl_speed_mps: f32,
    /// Landing descent rate, m/s.
    pub land_speed_mps: f32,
    /// Landing without a valid height: thrust as a fraction of hover
    /// thrust.
    pub land_blind_thrust: f32,
    /// Touchdown: thrust below this fraction of hover thrust…
    pub touchdown_thrust: f32,
    /// …and vertical speed below this, m/s…
    pub touchdown_speed_mps: f32,
    /// …for this long, s.
    pub touchdown_s: f32,
}

impl Default for FailsafeParams {
    fn default() -> Self {
        Self {
            rc_timeout_us: 500_000,
            rc_valid_min_s: 1.0,
            rc_loss_hold_s: 3.0,
            gcs_timeout_us: 3_000_000,
            no_link_rtl_s: 60.0,
            brain_timeout_us: 2_000_000,
            brain_loss_rtl_s: 10.0,
            setpoint_timeout_us: 500_000,
            gps_loss_land_s: 30.0,
            clear_hysteresis_s: 1.0,
            battery_low_v: 3.5,
            battery_critical_v: 3.3,
            battery_debounce_s: 2.0,
            battery_filter_tau_s: 2.0,
            fence_radius_m: 0.0,
            fence_altitude_m: 0.0,
            loc_error_rad: 45.0_f32.to_radians(),
            loc_error_s: 0.5,
            loc_tilt_rad: 75.0_f32.to_radians(),
            loc_tilt_s: 0.25,
            rtl_altitude_m: 10.0,
            rtl_speed_mps: 3.0,
            land_speed_mps: 0.7,
            land_blind_thrust: 0.9,
            touchdown_thrust: 0.3,
            touchdown_speed_mps: 0.25,
            touchdown_s: 1.0,
        }
    }
}

impl FailsafeParams {
    /// A commanded descent counts as a command to move when it is at least
    /// this many times `touchdown_speed_mps` (ground contact, ADR-0003 §2).
    /// The landing speed must reach it too, or a landing could not be told
    /// from standing on the ground.
    pub const CONTACT_DESCENT_RATIO: f32 = 2.0;

    /// The slowest commanded descent that counts as a command to move, m/s.
    pub fn contact_descent_mps(&self) -> f32 {
        Self::CONTACT_DESCENT_RATIO * self.touchdown_speed_mps
    }

    /// Shortest accepted timeout, µs: below this a single late frame at
    /// the slowest supervisor rate would count as a loss.
    pub const MIN_TIMEOUT_US: u32 = 50_000;
    /// Longest accepted timeout, µs (an hour, like the durations): a
    /// timeout that never runs out is a loss that is never detected.
    pub const MAX_TIMEOUT_US: u32 = 3_600_000_000;
    /// Longest accepted duration, s.
    pub const MAX_DURATION_S: f32 = 3600.0;
    /// Accepted battery thresholds, V per cell. The monitor treats a cell
    /// voltage outside 3.0 to 4.35 V as implausible and refuses to arm, so a
    /// threshold below 3.0 could only fire after the pack had passed the
    /// point where it could be armed. A latch clears on the ground only
    /// above the threshold plus 0.1 V, and a full cell reads 4.2 V, so a
    /// threshold above 4.0 V could latch a fresh pack for good.
    pub const BATTERY_THRESHOLD_RANGE_V: (f32, f32) = (3.0, 4.0);

    /// Checks every parameter. Returns [`ConfigError::BadFailsafe`] for a
    /// non-finite or out-of-range value, or for one that would disable a
    /// protection or make a manoeuvre unable to finish (a zero return
    /// speed, a landing too slow to tell from standing on the ground).
    pub fn validate(&self) -> Result<(), ConfigError> {
        let finite = |v: f32| v.is_finite();
        let duration = |v: f32| (0.0..=Self::MAX_DURATION_S).contains(&v);
        let positive_duration = |v: f32| v > 0.0 && v <= Self::MAX_DURATION_S;
        let (v_min, v_max) = Self::BATTERY_THRESHOLD_RANGE_V;
        let floats = [
            self.rc_valid_min_s,
            self.rc_loss_hold_s,
            self.no_link_rtl_s,
            self.brain_loss_rtl_s,
            self.gps_loss_land_s,
            self.clear_hysteresis_s,
            self.battery_low_v,
            self.battery_critical_v,
            self.battery_debounce_s,
            self.battery_filter_tau_s,
            self.fence_radius_m,
            self.fence_altitude_m,
            self.loc_error_rad,
            self.loc_error_s,
            self.loc_tilt_rad,
            self.loc_tilt_s,
            self.rtl_altitude_m,
            self.rtl_speed_mps,
            self.land_speed_mps,
            self.land_blind_thrust,
            self.touchdown_thrust,
            self.touchdown_speed_mps,
            self.touchdown_s,
        ];
        let timeouts = [
            self.rc_timeout_us,
            self.gcs_timeout_us,
            self.brain_timeout_us,
            self.setpoint_timeout_us,
        ];
        let durations = [
            self.rc_valid_min_s,
            self.rc_loss_hold_s,
            self.no_link_rtl_s,
            self.brain_loss_rtl_s,
            self.gps_loss_land_s,
            self.clear_hysteresis_s,
            self.battery_debounce_s,
            self.loc_error_s,
            self.loc_tilt_s,
        ];
        let angle = |v: f32| v > 0.0 && v <= core::f32::consts::PI;
        let ok = floats.iter().all(|v| finite(*v))
            && timeouts
                .iter()
                .all(|t| (Self::MIN_TIMEOUT_US..=Self::MAX_TIMEOUT_US).contains(t))
            && durations.iter().all(|v| duration(*v))
            && positive_duration(self.touchdown_s)
            && positive_duration(self.battery_filter_tau_s)
            && (v_min..=v_max).contains(&self.battery_low_v)
            && (v_min..=v_max).contains(&self.battery_critical_v)
            && self.battery_critical_v <= self.battery_low_v
            && self.fence_radius_m >= 0.0
            && self.fence_altitude_m >= 0.0
            && angle(self.loc_error_rad)
            && angle(self.loc_tilt_rad)
            && self.rtl_altitude_m >= 0.0
            && self.rtl_speed_mps >= 0.1
            && self.touchdown_speed_mps > 0.0
            && self.land_speed_mps >= self.contact_descent_mps()
            && self.touchdown_thrust > 0.0
            && self.touchdown_thrust < 1.0
            // A blind landing flies this fixed thrust with no vertical speed
            // known, where low thrust alone reads as landed: it must stay
            // above the touchdown thrust or the motors would stop in the air.
            && self.land_blind_thrust > self.touchdown_thrust
            && self.land_blind_thrust <= 1.0;
        if ok {
            Ok(())
        } else {
            Err(ConfigError::BadFailsafe)
        }
    }
}

/// Everything the core needs to know about the vehicle.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VehicleConfig<const N: usize> {
    pub mass_kg: f32,
    /// Principal moments of inertia, kg·m².
    pub inertia: Vec3<f32>,
    pub rotors: [Rotor; N],
    /// Thrust curve: thrust fraction = (1 − e)·u + e·u² for command u.
    pub thrust_expo: f32,
    /// Battery voltage at which `max_thrust_n` applies, V.
    pub nominal_voltage_v: f32,
    pub gains: Gains,
    pub limits: Limits,
    pub estimator: EstimatorConfig,
    pub failsafe: FailsafeParams,
    /// Battery cells in series.
    pub battery_cells: u8,
    /// Battery internal resistance, Ω, for load compensation.
    pub battery_resistance_ohm: f32,
}

/// Configuration problems found when the core is built.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConfigError {
    /// Mass or inertia not positive and finite.
    BadMassProperties,
    /// A rotor has a non-positive or non-finite thrust or position.
    BadRotor,
    /// The rotor layout cannot produce independent roll, pitch, yaw and thrust.
    UncontrollableLayout,
    /// Thrust expo outside `[0, 1]`.
    BadThrustExpo,
    /// The scheduling profile is malformed.
    BadProfile,
    /// An estimator parameter is out of range, or the delay horizon does not
    /// fit the profile's filter rate and buffer sizes.
    BadEstimator,
    /// A failsafe parameter or the battery description is out of range, or
    /// the minimum thrust is not below the touchdown thrust.
    BadFailsafe,
}

impl<const N: usize> VehicleConfig<N> {
    pub fn validate(&self) -> Result<(), ConfigError> {
        let pos = |v: f32| v.is_finite() && v > 0.0;
        if !pos(self.mass_kg)
            || !pos(self.inertia.x)
            || !pos(self.inertia.y)
            || !pos(self.inertia.z)
        {
            return Err(ConfigError::BadMassProperties);
        }
        if self.rotors.iter().any(|r| {
            !pos(r.max_thrust_n)
                || !r.x.is_finite()
                || !r.y.is_finite()
                || !r.torque_coeff_m.is_finite()
        }) {
            return Err(ConfigError::BadRotor);
        }
        if !(0.0..=1.0).contains(&self.thrust_expo) {
            return Err(ConfigError::BadThrustExpo);
        }
        if !pos(self.nominal_voltage_v) {
            return Err(ConfigError::BadMassProperties);
        }
        self.failsafe.validate()?;
        // On ground contact guidance commands the minimum thrust, and
        // touchdown needs the thrust below `touchdown_thrust`: at or above
        // it a landed vehicle would never disarm.
        if !(0.0..self.failsafe.touchdown_thrust)
            .contains(&self.limits.min_thrust_fraction_of_hover)
        {
            return Err(ConfigError::BadFailsafe);
        }
        if self.battery_cells < 1
            || !self.battery_resistance_ohm.is_finite()
            || self.battery_resistance_ohm < 0.0
        {
            return Err(ConfigError::BadFailsafe);
        }
        Ok(())
    }

    /// Total thrust available at nominal voltage, N.
    pub fn total_max_thrust_n(&self) -> f32 {
        self.rotors.iter().map(|r| r.max_thrust_n).sum()
    }

    /// Weight, N.
    pub fn weight_n(&self) -> f32 {
        self.mass_kg * crate::GRAVITY
    }
}
