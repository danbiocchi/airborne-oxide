// SPDX-License-Identifier: AGPL-3.0-or-later
//! Vehicle configuration. On hardware this comes from parameters (§4.14);
//! in the simulator it is built from the airframe file.

use crate::estimator::IMU_BUFFER;
use crate::math::Vec3;
use crate::schedule::Profile;

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

impl Gains {
    /// A gyro cut-off the profile's rate loop can realise: zero (off), or
    /// below 0.45 times the base rate. `LowPass2::new` switches itself off at
    /// or above that, which would make the parameter mean two things.
    pub fn check_against(&self, profile: &Profile) -> Result<(), ConfigError> {
        let base_hz = 1.0e6 / f64::from(profile.base_period_us.max(1));
        let cutoff = f64::from(self.gyro_cutoff_hz);
        if cutoff != 0.0 && cutoff >= 0.45 * base_hz {
            return Err(ConfigError::FilterCutoffTooHigh);
        }
        Ok(())
    }
}

impl Default for Gains {
    /// Starting gains for a small multirotor; auto-tuned per airframe later.
    /// The values are the defaults in the parameter table.
    ///
    /// The outer loops were chosen on a reduced model (position P → velocity
    /// PI → acceleration with a 0.12 s lag, 50 Hz, one-period delay) for
    /// ~1 % overshoot and ~1.1 s settling on a 1 m step. PX4-style defaults
    /// (0.95 / 1.8 / 0.4) leave a slow closed-loop pole near −0.2 rad/s that
    /// gives a 14 % overshoot and a 4 s tail without trajectory smoothing.
    fn default() -> Self {
        let z = Vec3::new(0.0, 0.0, 0.0);
        let mut g = Self {
            rate_p: z,
            rate_i: z,
            rate_d: z,
            rate_d_cutoff_hz: 0.0,
            gyro_cutoff_hz: 0.0,
            attitude_p: z,
            velocity_p: z,
            velocity_i: z,
            velocity_d: z,
            position_p: z,
        };
        crate::params::fill_defaults(&mut g);
        g
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
    /// Lowest motor command on every motor while armed (a floor, not only a
    /// command for no demand); at most half of the hover thrust of a rotor.
    pub idle_command: f32,
    /// Stick deadband: a stick within this of centre reads as centred, and
    /// the rest of the travel is rescaled to the full range.
    pub stick_deadband: f32,
}

impl Default for Limits {
    /// The defaults in the parameter table.
    fn default() -> Self {
        let mut l = Self {
            max_tilt_rad: 0.0,
            max_tilt_stabilized_rad: 0.0,
            max_rate: Vec3::new(0.0, 0.0, 0.0),
            max_speed_xy: 0.0,
            max_speed_up: 0.0,
            max_speed_down: 0.0,
            max_yaw_rate: 0.0,
            rate_integral_limit: 0.0,
            velocity_integral_limit: 0.0,
            min_thrust_fraction_of_hover: 0.0,
            idle_command: 0.0,
            stick_deadband: 0.0,
        };
        crate::params::fill_defaults(&mut l);
        l
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
    /// The defaults in the parameter table.
    fn default() -> Self {
        let mut e = Self {
            delay_horizon_us: 0,
            gyro_noise: 0.0,
            accel_noise: 0.0,
            gyro_bias_noise: 0.0,
            accel_bias_noise: 0.0,
            gps_pos_noise_m: 0.0,
            gps_vel_noise_mps: 0.0,
            baro_noise_m: 0.0,
            yaw_noise_rad: 0.0,
            gps_gate: 0.0,
            baro_gate: 0.0,
            yaw_gate: 0.0,
            mag_declination_rad: 0.0,
            init_tilt_sigma_rad: 0.0,
            init_yaw_sigma_rad: 0.0,
            init_vel_sigma_mps: 0.0,
            init_pos_sigma_m: 0.0,
            init_gyro_bias_sigma: 0.0,
            init_accel_bias_sigma: 0.0,
            align_time_us: 0,
            gps_timeout_us: 0,
            pos_timeout_us: 0,
            baro_timeout_us: 0,
            mag_timeout_us: 0,
            output_tau_s: 0.0,
        };
        crate::params::fill_defaults(&mut e);
        e
    }
}

impl EkfParams {
    /// The output predictor's time constant must be at least the delay
    /// horizon, compared in whole microseconds. Flown on `gust-hold` with
    /// both profiles, the hold fails below about 0.7 of the horizon (0.14 s
    /// at the default 0.2 s, 0.17 s at 0.248 s); the rule keeps a margin of
    /// 40 % or more over that.
    pub fn output_tau_covers_horizon(&self) -> bool {
        let tau_us = f64::from(self.output_tau_s) * 1_000_000.0 + 0.5;
        tau_us >= f64::from(self.delay_horizon_us)
    }

    /// Checks the parameters against their table ranges and the delay
    /// horizon against the profile: it must fit the IMU buffer,
    /// `ceil(delay_horizon_us / filter period) + 2 <= IMU_BUFFER`. Fails
    /// with [`ConfigError::BadEstimator`].
    pub fn validate(&self, profile: &Profile) -> Result<(), ConfigError> {
        if !crate::params::in_range(self) || !self.output_tau_covers_horizon() {
            return Err(ConfigError::BadEstimator);
        }
        let period_us = u64::from(profile.filter_period_ticks) * u64::from(profile.base_period_us);
        let needed = u64::from(self.delay_horizon_us).div_ceil(period_us.max(1)) + 2;
        if period_us == 0 || needed > IMU_BUFFER as u64 {
            return Err(ConfigError::BadEstimator);
        }
        Ok(())
    }
}

/// Estimator selection and tuning.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EstimatorConfig {
    pub source: EstimatorSource,
    pub ekf: EkfParams,
}

impl Default for EstimatorConfig {
    /// The defaults in the parameter table.
    fn default() -> Self {
        let mut c = Self {
            source: EstimatorSource::External,
            ekf: EkfParams::default(),
        };
        crate::params::fill_defaults(&mut c);
        c
    }
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
    /// The defaults in the parameter table.
    fn default() -> Self {
        let mut f = Self {
            rc_timeout_us: 0,
            rc_valid_min_s: 0.0,
            rc_loss_hold_s: 0.0,
            gcs_timeout_us: 0,
            no_link_rtl_s: 0.0,
            brain_timeout_us: 0,
            brain_loss_rtl_s: 0.0,
            setpoint_timeout_us: 0,
            gps_loss_land_s: 0.0,
            clear_hysteresis_s: 0.0,
            battery_low_v: 0.0,
            battery_critical_v: 0.0,
            battery_debounce_s: 0.0,
            battery_filter_tau_s: 0.0,
            fence_radius_m: 0.0,
            fence_altitude_m: 0.0,
            loc_error_rad: 0.0,
            loc_error_s: 0.0,
            loc_tilt_rad: 0.0,
            loc_tilt_s: 0.0,
            rtl_altitude_m: 0.0,
            rtl_speed_mps: 0.0,
            land_speed_mps: 0.0,
            land_blind_thrust: 0.0,
            touchdown_thrust: 0.0,
            touchdown_speed_mps: 0.0,
            touchdown_s: 0.0,
        };
        crate::params::fill_defaults(&mut f);
        f
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

    /// The loss-of-control tilt threshold must exceed the larger of the two
    /// tilt limits by this much, rad (10 degrees).
    pub const LOC_TILT_MARGIN_RAD: f32 = 0.175;

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
    /// value outside its range in the parameter table (non-finite included),
    /// or for one that would disable a protection or make a manoeuvre unable
    /// to finish (a landing too slow to tell from standing on the ground).
    /// The table holds the per-field ranges; the rules between two fields
    /// are here.
    pub fn validate(&self) -> Result<(), ConfigError> {
        let ok = crate::params::in_range(self)
            && self.battery_critical_v <= self.battery_low_v
            && self.land_speed_mps >= self.contact_descent_mps()
            // A blind landing flies this fixed thrust with no vertical speed
            // known, where low thrust alone reads as landed: it must stay
            // above the touchdown thrust or the motors would stop in the air.
            && self.land_blind_thrust > self.touchdown_thrust;
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
    /// A gain is outside its range in the parameter table.
    BadGains,
    /// A limit is outside its range in the parameter table.
    BadLimits,
    /// The rotors' total thrust at nominal voltage does not exceed the
    /// weight: the vehicle could not climb (mass in grams, thrust in grams-
    /// force, or a rotor that is far too small).
    CannotLift,
    /// The idle motor command gives more than half of the hover thrust per
    /// rotor: it is a floor on every motor, so the vehicle could not be held
    /// down or steered.
    IdleTooHigh,
    /// The descent speed limit is below the slowest commanded descent that
    /// counts as a command to move (twice the touchdown speed): the landing
    /// would be clamped below it and ground contact declared in mid-air.
    DescentLimitTooLow,
    /// The loss-of-control tilt threshold is not at least 10 degrees above
    /// the larger tilt limit: normal commanded flight would switch the motors
    /// off.
    LossOfControlTooTight,
    /// The gyro low-pass cut-off is at or above 0.45 times the rate-loop
    /// frequency of the profile, where the filter would silently be off.
    FilterCutoffTooHigh,
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
        // A multirotor must be able to lift itself. This also keeps the
        // minimum thrust (a fraction below one of the hover thrust) below the
        // maximum, which the controllers clamp between.
        if self.total_max_thrust_n().is_nan() || self.total_max_thrust_n() <= self.weight_n() {
            return Err(ConfigError::CannotLift);
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
        if !crate::params::in_range(&self.gains) {
            return Err(ConfigError::BadGains);
        }
        if !crate::params::in_range(&self.limits) {
            return Err(ConfigError::BadLimits);
        }
        if !crate::params::in_range(&self.estimator.ekf)
            || !crate::params::in_range(&self.estimator)
            || !self.estimator.ekf.output_tau_covers_horizon()
        {
            return Err(ConfigError::BadEstimator);
        }
        // The velocity loop clamps the descent that `land_speed_mps` commands
        // (landing, return, land-now, blind landing) to `max_speed_down`.
        // Below the speed that counts as a command to move, the landing
        // would read as standing on the ground, in mid-air. (A pilot's own
        // descent is at most `max_speed_down`, and is judged on thrust.)
        if self.limits.max_speed_down < self.failsafe.contact_descent_mps() {
            return Err(ConfigError::DescentLimitTooLow);
        }
        // Loss of control switches the motors off. Its tilt threshold must sit
        // above any tilt the controller itself commands, with a margin.
        let widest_tilt = self
            .limits
            .max_tilt_rad
            .max(self.limits.max_tilt_stabilized_rad);
        if self.failsafe.loc_tilt_rad < widest_tilt + FailsafeParams::LOC_TILT_MARGIN_RAD {
            return Err(ConfigError::LossOfControlTooTight);
        }
        // `idle_command` is a floor on every motor on every tick while armed.
        // Above about hover it pins the thrust at or above the weight and
        // overrides the differences that steer, so it must give at most half
        // of the hover thrust of a rotor (through the thrust curve).
        let hover_per_rotor_n = self.weight_n() / (N.max(1) as f32);
        let idle_fraction =
            crate::allocation::fraction_for_command(self.limits.idle_command, self.thrust_expo);
        if self
            .rotors
            .iter()
            .any(|r| idle_fraction * r.max_thrust_n > 0.5 * hover_per_rotor_n)
        {
            return Err(ConfigError::IdleTooHigh);
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

// ---- the vehicle description as a list of numbers ----

/// Rotors whose fields have names in [`VehicleConfig::vehicle_numbers`].
pub const MAX_NUMBERED_ROTORS: usize = 16;
/// Values per rotor: x, y, spin, max thrust, torque coefficient.
const ROTOR_FIELDS: usize = 5;
/// Values before the rotors.
const SCALARS: usize = 8;
const SCALAR_NAMES: [&str; SCALARS] = [
    "mass_kg",
    "inertia_x",
    "inertia_y",
    "inertia_z",
    "thrust_expo",
    "nominal_voltage_v",
    "battery_cells",
    "battery_resistance_ohm",
];
const ROTOR_NAMES: [[&str; ROTOR_FIELDS]; MAX_NUMBERED_ROTORS] = [
    [
        "rotor0_x",
        "rotor0_y",
        "rotor0_spin",
        "rotor0_max_thrust_n",
        "rotor0_torque_coeff_m",
    ],
    [
        "rotor1_x",
        "rotor1_y",
        "rotor1_spin",
        "rotor1_max_thrust_n",
        "rotor1_torque_coeff_m",
    ],
    [
        "rotor2_x",
        "rotor2_y",
        "rotor2_spin",
        "rotor2_max_thrust_n",
        "rotor2_torque_coeff_m",
    ],
    [
        "rotor3_x",
        "rotor3_y",
        "rotor3_spin",
        "rotor3_max_thrust_n",
        "rotor3_torque_coeff_m",
    ],
    [
        "rotor4_x",
        "rotor4_y",
        "rotor4_spin",
        "rotor4_max_thrust_n",
        "rotor4_torque_coeff_m",
    ],
    [
        "rotor5_x",
        "rotor5_y",
        "rotor5_spin",
        "rotor5_max_thrust_n",
        "rotor5_torque_coeff_m",
    ],
    [
        "rotor6_x",
        "rotor6_y",
        "rotor6_spin",
        "rotor6_max_thrust_n",
        "rotor6_torque_coeff_m",
    ],
    [
        "rotor7_x",
        "rotor7_y",
        "rotor7_spin",
        "rotor7_max_thrust_n",
        "rotor7_torque_coeff_m",
    ],
    [
        "rotor8_x",
        "rotor8_y",
        "rotor8_spin",
        "rotor8_max_thrust_n",
        "rotor8_torque_coeff_m",
    ],
    [
        "rotor9_x",
        "rotor9_y",
        "rotor9_spin",
        "rotor9_max_thrust_n",
        "rotor9_torque_coeff_m",
    ],
    [
        "rotor10_x",
        "rotor10_y",
        "rotor10_spin",
        "rotor10_max_thrust_n",
        "rotor10_torque_coeff_m",
    ],
    [
        "rotor11_x",
        "rotor11_y",
        "rotor11_spin",
        "rotor11_max_thrust_n",
        "rotor11_torque_coeff_m",
    ],
    [
        "rotor12_x",
        "rotor12_y",
        "rotor12_spin",
        "rotor12_max_thrust_n",
        "rotor12_torque_coeff_m",
    ],
    [
        "rotor13_x",
        "rotor13_y",
        "rotor13_spin",
        "rotor13_max_thrust_n",
        "rotor13_torque_coeff_m",
    ],
    [
        "rotor14_x",
        "rotor14_y",
        "rotor14_spin",
        "rotor14_max_thrust_n",
        "rotor14_torque_coeff_m",
    ],
    [
        "rotor15_x",
        "rotor15_y",
        "rotor15_spin",
        "rotor15_max_thrust_n",
        "rotor15_torque_coeff_m",
    ],
];

/// Why a list of vehicle numbers could not be turned into a configuration.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NumbersError {
    /// A name the vehicle needs is not in the list.
    Missing(&'static str),
    /// A name that is not one of this vehicle's numbers.
    Unknown,
    /// A name appears twice.
    Duplicate(&'static str),
    /// The value cannot be this field: a spin other than 1 or -1, or a cell
    /// count that is not a whole number from 0 to 255.
    BadValue(&'static str),
}

/// 64-bit FNV-1a, one byte at a time.
fn fnv1a_64(mut h: u64, bytes: &[u8]) -> u64 {
    for b in bytes {
        h ^= u64::from(*b);
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    }
    h
}

impl<const N: usize> VehicleConfig<N> {
    /// Every value of the vehicle description that is not one of the
    /// parameters, as `(name, value)` in a fixed order: [`SCALAR_NAMES`]
    /// (mass, the three moments of inertia, thrust expo, nominal voltage,
    /// battery cells and resistance), then for each rotor `rotor{k}_x`,
    /// `_y` (m), `_spin` (1 for counter-clockwise, -1 for clockwise),
    /// `_max_thrust_n`, `_torque_coeff_m`. Every value is exact in an `f32`
    /// (the cell count is an integer below 256). At most
    /// [`MAX_NUMBERED_ROTORS`] rotors: more does not compile.
    ///
    /// The 93 parameters are not here; they are `params::get` over the
    /// table. Together they are the whole configuration.
    pub fn vehicle_numbers(&self, mut f: impl FnMut(&'static str, f32)) {
        const { assert!(N <= MAX_NUMBERED_ROTORS) };
        let scalars = [
            self.mass_kg,
            self.inertia.x,
            self.inertia.y,
            self.inertia.z,
            self.thrust_expo,
            self.nominal_voltage_v,
            f32::from(self.battery_cells),
            self.battery_resistance_ohm,
        ];
        for (name, v) in SCALAR_NAMES.iter().zip(scalars) {
            f(name, v);
        }
        for (names, r) in ROTOR_NAMES.iter().zip(self.rotors.iter()) {
            let values = [r.x, r.y, r.spin.sign(), r.max_thrust_n, r.torque_coeff_m];
            for (name, v) in names.iter().zip(values) {
                f(name, v);
            }
        }
    }

    /// The inverse of [`Self::vehicle_numbers`]: builds the vehicle
    /// description from named values (any order). Every name must appear
    /// once; one that is not this vehicle's is [`NumbersError::Unknown`].
    ///
    /// The gains, limits, estimator and failsafe settings, which the 93
    /// parameters hold, come out at the table defaults: apply the
    /// parameters on top (`params::set_all`). The result is not validated.
    pub fn from_vehicle_numbers<'a>(
        numbers: impl IntoIterator<Item = (&'a str, f32)>,
    ) -> Result<Self, NumbersError> {
        const { assert!(N <= MAX_NUMBERED_ROTORS) };
        let mut scalars = [None::<f32>; SCALARS];
        let mut rotors = [[None::<f32>; ROTOR_FIELDS]; N];
        for (name, v) in numbers {
            let slot =
                match SCALAR_NAMES.iter().position(|n| *n == name) {
                    Some(i) => scalars.get_mut(i).map(|s| (s, SCALAR_NAMES.get(i))),
                    None => ROTOR_NAMES.iter().take(N).zip(rotors.iter_mut()).find_map(
                        |(names, slots)| {
                            let i = names.iter().position(|n| *n == name)?;
                            Some((slots.get_mut(i)?, names.get(i)))
                        },
                    ),
                };
            let Some((slot, canonical)) = slot else {
                return Err(NumbersError::Unknown);
            };
            if slot.is_some() {
                return Err(NumbersError::Duplicate(canonical.copied().unwrap_or("")));
            }
            *slot = Some(v);
        }
        let need = |v: Option<f32>, name: &'static str| v.ok_or(NumbersError::Missing(name));
        let mut s = [0.0_f32; SCALARS];
        for ((out, v), name) in s.iter_mut().zip(scalars).zip(SCALAR_NAMES) {
            *out = need(v, name)?;
        }
        let [mass, ix, iy, iz, expo, volts, cells, resistance] = s;
        let cells_u8 = {
            let c = cells as u8;
            if f32::from(c) == cells {
                c
            } else {
                return Err(NumbersError::BadValue("battery_cells"));
            }
        };
        let mut out_rotors = [Rotor {
            x: 0.0,
            y: 0.0,
            spin: Spin::Ccw,
            max_thrust_n: 0.0,
            torque_coeff_m: 0.0,
        }; N];
        for ((slots, names), rotor) in rotors
            .iter()
            .zip(ROTOR_NAMES.iter())
            .zip(out_rotors.iter_mut())
        {
            let mut v = [0.0_f32; ROTOR_FIELDS];
            for ((out, slot), name) in v.iter_mut().zip(slots).zip(names) {
                *out = need(*slot, name)?;
            }
            let [x, y, spin, max_thrust_n, torque_coeff_m] = v;
            let spin_name = names.get(2).copied().unwrap_or("");
            let spin = if spin == 1.0 {
                Spin::Ccw
            } else if spin == -1.0 {
                Spin::Cw
            } else {
                return Err(NumbersError::BadValue(spin_name));
            };
            *rotor = Rotor {
                x,
                y,
                spin,
                max_thrust_n,
                torque_coeff_m,
            };
        }
        Ok(Self {
            mass_kg: mass,
            inertia: Vec3::new(ix, iy, iz),
            rotors: out_rotors,
            thrust_expo: expo,
            nominal_voltage_v: volts,
            gains: Gains::default(),
            limits: Limits::default(),
            estimator: EstimatorConfig::default(),
            failsafe: FailsafeParams::default(),
            battery_cells: cells_u8,
            battery_resistance_ohm: resistance,
        })
    }

    /// A 64-bit hash of the whole configuration (ADR-0005 §6): FNV-1a over
    /// the bits of every [`Self::vehicle_numbers`] value in their order
    /// (each as its `f32` bits, little endian), then every parameter in table
    /// order (an `f32` as its bits, an `i32` as its two's-complement bits,
    /// little endian; parameters in seconds as `params::get` reads them),
    /// then the exact microseconds of every parameter held as whole
    /// microseconds (`params::micros`, as `u32` little endian, in table
    /// order), which `f32` seconds do not tell apart above 16 s.
    /// Two logs with the same hash and profile were flown with the same
    /// settings.
    pub fn config_hash(&self) -> u64 {
        let mut h: u64 = 0xcbf2_9ce4_8422_2325;
        self.vehicle_numbers(|_, v| h = fnv1a_64(h, &v.to_bits().to_le_bytes()));
        for i in 0..crate::params::COUNT {
            let bits = match u16::try_from(i)
                .ok()
                .and_then(|i| crate::params::get(self, crate::params::ParamId(i)))
            {
                Some(crate::params::ParamValue::F32(x)) => x.to_bits(),
                Some(crate::params::ParamValue::I32(x)) => x as u32,
                None => 0,
            };
            h = fnv1a_64(h, &bits.to_le_bytes());
        }
        for id in crate::params::micro_ids() {
            let us = crate::params::micros(self, id).unwrap_or(0);
            h = fnv1a_64(h, &us.to_le_bytes());
        }
        h
    }
}
