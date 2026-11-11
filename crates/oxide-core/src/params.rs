// SPDX-License-Identifier: AGPL-3.0-or-later
//! The parameter table (ADR-0004): every tunable value declared once, with
//! its name, type, default, range, unit, group and description.
//!
//! The configuration structs stay the storage; the table describes their
//! fields and [`get`] and [`set`] read and write them.

use crate::config::{
    ConfigError, EkfParams, EstimatorConfig, EstimatorSource, FailsafeParams, Gains, Limits,
    VehicleConfig,
};
use crate::schedule::Profile;

/// A parameter's value. `f32` and `i32` are the two types MAVLink carries
/// without loss.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ParamValue {
    F32(f32),
    I32(i32),
}

impl ParamValue {
    /// The type of this value.
    pub const fn kind(self) -> ParamKind {
        match self {
            ParamValue::F32(_) => ParamKind::F32,
            ParamValue::I32(_) => ParamKind::I32,
        }
    }
}

/// A parameter's type.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ParamKind {
    F32,
    I32,
}

/// The group a parameter belongs to; the name prefix says the same.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ParamGroup {
    /// `RATE_`: the rate loop and its filters.
    Rate,
    /// `ATT_`: the attitude loop.
    Attitude,
    /// `POS_`: the velocity and position loops.
    Position,
    /// `LIM_`: flight limits.
    Limits,
    /// `EKF_`: the state estimator.
    Estimator,
    /// `FS_`: failsafe timings and thresholds.
    Failsafe,
    /// `BAT_`: battery thresholds.
    Battery,
    /// `RTL_`: return to launch.
    Return,
    /// `LAND_`: landing and touchdown.
    Land,
}

impl ParamGroup {
    /// Every group, in table order.
    pub const ALL: [ParamGroup; 9] = [
        ParamGroup::Rate,
        ParamGroup::Attitude,
        ParamGroup::Position,
        ParamGroup::Limits,
        ParamGroup::Estimator,
        ParamGroup::Failsafe,
        ParamGroup::Battery,
        ParamGroup::Return,
        ParamGroup::Land,
    ];

    /// The name prefix of the group's parameters.
    pub const fn prefix(self) -> &'static str {
        match self {
            ParamGroup::Rate => "RATE_",
            ParamGroup::Attitude => "ATT_",
            ParamGroup::Position => "POS_",
            ParamGroup::Limits => "LIM_",
            ParamGroup::Estimator => "EKF_",
            ParamGroup::Failsafe => "FS_",
            ParamGroup::Battery => "BAT_",
            ParamGroup::Return => "RTL_",
            ParamGroup::Land => "LAND_",
        }
    }

    /// A human-readable title.
    pub const fn title(self) -> &'static str {
        match self {
            ParamGroup::Rate => "Rate loop",
            ParamGroup::Attitude => "Attitude loop",
            ParamGroup::Position => "Velocity and position loops",
            ParamGroup::Limits => "Limits",
            ParamGroup::Estimator => "State estimator",
            ParamGroup::Failsafe => "Failsafe",
            ParamGroup::Battery => "Battery",
            ParamGroup::Return => "Return to launch",
            ParamGroup::Land => "Landing",
        }
    }
}

/// A parameter: its index in [`PARAMS`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct ParamId(pub u16);

impl ParamId {
    /// The index into [`PARAMS`].
    pub const fn index(self) -> usize {
        self.0 as usize
    }
}

/// Everything about a parameter except its value.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ParamDef {
    /// At most 16 characters of `A-Z`, `0-9` and `_`.
    pub name: &'static str,
    pub kind: ParamKind,
    pub default: ParamValue,
    /// Lowest accepted value, inclusive.
    pub min: ParamValue,
    /// Highest accepted value, inclusive.
    pub max: ParamValue,
    /// `rad`, `rad/s`, `m`, `m/s`, `s`, `V` and so on; empty when the value
    /// has no unit.
    pub unit: &'static str,
    pub group: ParamGroup,
    pub description: &'static str,
    /// True if a change takes effect in flight; false if it is accepted only
    /// while disarmed.
    pub live: bool,
}

/// Why a parameter change was refused.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ParamError {
    /// No such parameter.
    Unknown,
    /// An `f32` value for an `i32` parameter or the other way round.
    WrongType,
    /// The value is NaN or infinite.
    NotFinite,
    /// The value is outside the parameter's `[min, max]`.
    OutOfRange,
    /// The value is in range but the configuration as a whole would break a
    /// rule that involves more than one field.
    Invalid(ConfigError),
    /// The parameter cannot be changed while armed.
    Armed,
}

// ---- storage access ----

/// A field's stored value, before conversion to the parameter's own unit.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum Stored {
    F(f32),
    /// Whole microseconds (parameter in seconds).
    Us(u32),
    I(i32),
}

/// A configuration struct that holds some of the parameters.
pub(crate) trait Section {
    /// The stored value of `id` if this struct holds it.
    fn load(s: &Self, id: ParamId) -> Option<Stored>;
    /// Writes `v` to the field of `id` if this struct holds it: `Ok(true)`
    /// if written, `Ok(false)` if the field is elsewhere. No range check.
    fn store(s: &mut Self, id: ParamId, v: ParamValue) -> Result<bool, ParamError>;
}

/// Seconds to whole microseconds, rounded; `None` for a negative or
/// non-finite value or one that does not fit `u32`.
fn secs_to_us(v: f32) -> Option<u32> {
    if !v.is_finite() || v < 0.0 {
        return None;
    }
    let us = f64::from(v) * 1_000_000.0 + 0.5;
    if us >= f64::from(u32::MAX) + 1.0 {
        return None;
    }
    Some(us as u32)
}

/// Whole microseconds to seconds.
fn us_to_secs(us: u32) -> f32 {
    (f64::from(us) / 1_000_000.0) as f32
}

/// The table's range of a parameter held as microseconds, as microseconds.
fn range_us(def: &ParamDef) -> Option<(u32, u32)> {
    match (def.min, def.max) {
        (ParamValue::F32(lo), ParamValue::F32(hi)) => Some((secs_to_us(lo)?, secs_to_us(hi)?)),
        _ => None,
    }
}

fn stored_in_range(def: &ParamDef, s: Stored) -> bool {
    match (s, def.min, def.max) {
        (Stored::F(v), ParamValue::F32(lo), ParamValue::F32(hi)) => {
            v.is_finite() && v >= lo && v <= hi
        }
        (Stored::Us(u), _, _) => range_us(def).is_some_and(|(lo, hi)| u >= lo && u <= hi),
        (Stored::I(v), ParamValue::I32(lo), ParamValue::I32(hi)) => v >= lo && v <= hi,
        _ => false,
    }
}

fn to_value(s: Stored) -> ParamValue {
    match s {
        Stored::F(v) => ParamValue::F32(v),
        Stored::Us(u) => ParamValue::F32(us_to_secs(u)),
        Stored::I(v) => ParamValue::I32(v),
    }
}

/// True if every parameter held by `s` is within its table range. A
/// microsecond field is compared as whole microseconds.
pub(crate) fn in_range<S: Section>(s: &S) -> bool {
    PARAMS
        .iter()
        .enumerate()
        .all(|(i, def)| match S::load(s, ParamId(i as u16)) {
            None => true,
            Some(v) => stored_in_range(def, v),
        })
}

/// Writes every default of the table held by `s`.
#[allow(dead_code)] // used by the Default impls in the next step
pub(crate) fn fill_defaults<S: Section>(s: &mut S) {
    for (i, def) in PARAMS.iter().enumerate() {
        // A default the table cannot store is a bug the pinned-defaults test
        // and the table tests catch; the field keeps its starting value.
        let _ = S::store(s, ParamId(i as u16), def.default);
    }
}

macro_rules! p_kind {
    (f32) => {
        ParamKind::F32
    };
    (secs) => {
        ParamKind::F32
    };
    (source) => {
        ParamKind::I32
    };
}

macro_rules! p_val {
    (f32, $x:expr) => {
        ParamValue::F32($x)
    };
    (secs, $x:expr) => {
        ParamValue::F32($x)
    };
    (source, $x:expr) => {
        ParamValue::I32($x)
    };
}

macro_rules! p_load {
    (f32, $s:ident, $($p:ident).+) => {
        Stored::F($s.$($p).+)
    };
    (secs, $s:ident, $($p:ident).+) => {
        Stored::Us($s.$($p).+)
    };
    (source, $s:ident, $($p:ident).+) => {
        Stored::I(match $s.$($p).+ {
            EstimatorSource::External => 0,
            EstimatorSource::Ekf => 1,
        })
    };
}

macro_rules! p_store {
    (f32, $s:ident, $v:ident, $($p:ident).+) => {
        match $v {
            ParamValue::F32(x) => {
                $s.$($p).+ = x;
                Ok(true)
            }
            ParamValue::I32(_) => Err(ParamError::WrongType),
        }
    };
    (secs, $s:ident, $v:ident, $($p:ident).+) => {
        match $v {
            ParamValue::F32(x) => match secs_to_us(x) {
                Some(us) => {
                    $s.$($p).+ = us;
                    Ok(true)
                }
                None => Err(ParamError::OutOfRange),
            },
            ParamValue::I32(_) => Err(ParamError::WrongType),
        }
    };
    (source, $s:ident, $v:ident, $($p:ident).+) => {
        match $v {
            ParamValue::I32(0) => {
                $s.$($p).+ = EstimatorSource::External;
                Ok(true)
            }
            ParamValue::I32(1) => {
                $s.$($p).+ = EstimatorSource::Ekf;
                Ok(true)
            }
            ParamValue::I32(_) => Err(ParamError::OutOfRange),
            ParamValue::F32(_) => Err(ParamError::WrongType),
        }
    };
}

/// Declares the table. Each entry is `kind NAME = field.path : default, min,
/// max, "unit", Group, live | disarmed, "description";` inside the block of
/// the configuration struct that holds the field. Kinds: `f32` (an `f32`
/// field), `secs` (a `u32` field of whole microseconds, a parameter in
/// seconds), `source` (the estimator source as `i32`).
macro_rules! declare_params {
    ($(
        $sec:ty {
            $(
                $kind:ident $name:ident = $($path:ident).+ :
                $def:expr, $min:expr, $max:expr, $unit:literal, $group:ident, $mode:ident, $desc:literal;
            )*
        }
    )*) => {
        #[allow(non_camel_case_types, clippy::upper_case_acronyms)]
        #[repr(u16)]
        enum Ix {
            $( $( $name, )* )*
        }

        impl ParamId {
            $( $(
                pub const $name: ParamId = ParamId(Ix::$name as u16);
            )* )*
        }

        /// The number of parameters.
        pub const COUNT: usize = [$( $( stringify!($name), )* )*].len();

        /// Every parameter, declared once.
        pub static PARAMS: &[ParamDef] = &[
            $( $(
                ParamDef {
                    name: stringify!($name),
                    kind: p_kind!($kind),
                    default: p_val!($kind, $def),
                    min: p_val!($kind, $min),
                    max: p_val!($kind, $max),
                    unit: $unit,
                    group: ParamGroup::$group,
                    description: $desc,
                    live: declare_params!(@live $mode),
                },
            )* )*
        ];

        $(
            impl Section for $sec {
                fn load(s: &Self, id: ParamId) -> Option<Stored> {
                    match id {
                        $( ParamId::$name => Some(p_load!($kind, s, $($path).+)), )*
                        _ => None,
                    }
                }

                fn store(s: &mut Self, id: ParamId, v: ParamValue) -> Result<bool, ParamError> {
                    match id {
                        $( ParamId::$name => p_store!($kind, s, v, $($path).+), )*
                        _ => Ok(false),
                    }
                }
            }
        )*
    };
    (@live live) => { true };
    (@live disarmed) => { false };
}

// ---- range constants ----

/// The smallest positive `f32`: the lower end of a range that excludes zero
/// (`> 0`), where the code accepted any positive value.
const TINY: f32 = f32::from_bits(1);
/// The largest `f32` below one: the upper end of a range that excludes one.
const BELOW_ONE: f32 = f32::from_bits(0x3f7f_ffff);
const PI: f32 = core::f32::consts::PI;
/// One microsecond, the shortest nonzero time held as whole microseconds.
const MIN_US: f32 = 1.0e-6;
/// Shortest and longest accepted failsafe timeouts, s.
const MIN_TIMEOUT: f32 = FailsafeParams::MIN_TIMEOUT_US as f32 / 1_000_000.0;
const MAX_TIMEOUT: f32 = FailsafeParams::MAX_TIMEOUT_US as f32 / 1_000_000.0;
/// Longest accepted duration, s.
const DUR: f32 = FailsafeParams::MAX_DURATION_S;
const BATTERY_MIN_V: f32 = FailsafeParams::BATTERY_THRESHOLD_RANGE_V.0;
const BATTERY_MAX_V: f32 = FailsafeParams::BATTERY_THRESHOLD_RANGE_V.1;

declare_params! {
    Gains {
        f32 RATE_ROLL_P = rate_p.x: 28.0, 0.0, 200.0, "1/s", Rate, live,
            "Rate loop proportional gain on roll: angular acceleration demanded per rad/s of rate error.";
        f32 RATE_PITCH_P = rate_p.y: 28.0, 0.0, 200.0, "1/s", Rate, live,
            "Rate loop proportional gain on pitch: angular acceleration demanded per rad/s of rate error.";
        f32 RATE_YAW_P = rate_p.z: 12.0, 0.0, 100.0, "1/s", Rate, live,
            "Rate loop proportional gain on yaw: angular acceleration demanded per rad/s of rate error.";
        f32 RATE_ROLL_I = rate_i.x: 40.0, 0.0, 300.0, "1/s^2", Rate, live,
            "Rate loop integral gain on roll.";
        f32 RATE_PITCH_I = rate_i.y: 40.0, 0.0, 300.0, "1/s^2", Rate, live,
            "Rate loop integral gain on pitch.";
        f32 RATE_YAW_I = rate_i.z: 10.0, 0.0, 100.0, "1/s^2", Rate, live,
            "Rate loop integral gain on yaw.";
        f32 RATE_ROLL_D = rate_d.x: 0.3, 0.0, 2.0, "", Rate, live,
            "Rate loop derivative gain on roll, applied to the measurement.";
        f32 RATE_PITCH_D = rate_d.y: 0.3, 0.0, 2.0, "", Rate, live,
            "Rate loop derivative gain on pitch, applied to the measurement.";
        f32 RATE_YAW_D = rate_d.z: 0.0, 0.0, 1.0, "", Rate, live,
            "Rate loop derivative gain on yaw, applied to the measurement.";
        f32 RATE_D_CUTOFF = rate_d_cutoff_hz: 60.0, 0.0, 500.0, "Hz", Rate, live,
            "Cut-off of the first-order low-pass on the rate loop derivative term. Zero turns it off.";
        f32 RATE_GYRO_CUTOFF = gyro_cutoff_hz: 80.0, 0.0, 500.0, "Hz", Rate, disarmed,
            "Cut-off of the second-order Butterworth low-pass on the gyro for the rate loop. Zero turns it off. The filter is rebuilt when this changes.";
        f32 ATT_ROLL_P = attitude_p.x: 6.5, 0.0, 50.0, "1/s", Attitude, live,
            "Attitude loop proportional gain on roll: rate demanded per rad of attitude error.";
        f32 ATT_PITCH_P = attitude_p.y: 6.5, 0.0, 50.0, "1/s", Attitude, live,
            "Attitude loop proportional gain on pitch: rate demanded per rad of attitude error.";
        f32 ATT_YAW_P = attitude_p.z: 3.0, 0.0, 30.0, "1/s", Attitude, live,
            "Attitude loop proportional gain on yaw: rate demanded per rad of attitude error.";
        f32 POS_VELX_P = velocity_p.x: 4.0, 0.0, 30.0, "1/s", Position, live,
            "Velocity loop proportional gain, north axis: acceleration demanded per m/s of velocity error.";
        f32 POS_VELY_P = velocity_p.y: 4.0, 0.0, 30.0, "1/s", Position, live,
            "Velocity loop proportional gain, east axis: acceleration demanded per m/s of velocity error.";
        f32 POS_VELZ_P = velocity_p.z: 4.0, 0.0, 30.0, "1/s", Position, live,
            "Velocity loop proportional gain, down axis: acceleration demanded per m/s of velocity error.";
        f32 POS_VELX_I = velocity_i.x: 0.4, 0.0, 4.0, "1/s^2", Position, live,
            "Velocity loop integral gain, north axis.";
        f32 POS_VELY_I = velocity_i.y: 0.4, 0.0, 4.0, "1/s^2", Position, live,
            "Velocity loop integral gain, east axis.";
        f32 POS_VELZ_I = velocity_i.z: 0.8, 0.0, 8.0, "1/s^2", Position, live,
            "Velocity loop integral gain, down axis.";
        f32 POS_VELX_D = velocity_d.x: 0.0, 0.0, 1.0, "", Position, live,
            "Velocity loop derivative gain, north axis, applied to the measurement.";
        f32 POS_VELY_D = velocity_d.y: 0.0, 0.0, 1.0, "", Position, live,
            "Velocity loop derivative gain, east axis, applied to the measurement.";
        f32 POS_VELZ_D = velocity_d.z: 0.0, 0.0, 1.0, "", Position, live,
            "Velocity loop derivative gain, down axis, applied to the measurement.";
        f32 POS_X_P = position_p.x: 1.5, 0.0, 10.0, "1/s", Position, live,
            "Position loop proportional gain, north axis: velocity demanded per m of position error.";
        f32 POS_Y_P = position_p.y: 1.5, 0.0, 10.0, "1/s", Position, live,
            "Position loop proportional gain, east axis: velocity demanded per m of position error.";
        f32 POS_Z_P = position_p.z: 1.5, 0.0, 10.0, "1/s", Position, live,
            "Position loop proportional gain, down axis: velocity demanded per m of position error.";
    }
    Limits {
        f32 LIM_TILT = max_tilt_rad: 35.0_f32.to_radians(), 0.01, 1.39, "rad", Limits, live,
            "Maximum tilt in the position and offboard modes.";
        f32 LIM_TILT_STAB = max_tilt_stabilized_rad: 45.0_f32.to_radians(), 0.01, 1.39, "rad", Limits, live,
            "Maximum tilt in the stabilized mode, at full stick.";
        f32 LIM_RATE_ROLL = max_rate.x: 3.8, 0.01, 35.0, "rad/s", Limits, live,
            "Maximum body rate, roll axis.";
        f32 LIM_RATE_PITCH = max_rate.y: 3.8, 0.01, 35.0, "rad/s", Limits, live,
            "Maximum body rate, pitch axis.";
        f32 LIM_RATE_YAW = max_rate.z: 3.5, 0.01, 35.0, "rad/s", Limits, live,
            "Maximum body rate, yaw axis.";
        f32 LIM_SPEED_XY = max_speed_xy: 5.0, 0.01, 30.0, "m/s", Limits, live,
            "Maximum horizontal speed.";
        f32 LIM_SPEED_UP = max_speed_up: 3.0, 0.01, 30.0, "m/s", Limits, live,
            "Maximum climb speed.";
        f32 LIM_SPEED_DOWN = max_speed_down: 1.5, 0.01, 30.0, "m/s", Limits, live,
            "Maximum descent speed.";
        f32 LIM_YAW_RATE = max_yaw_rate: 2.0, 0.01, 35.0, "rad/s", Limits, live,
            "Yaw rate at full yaw stick.";
        f32 LIM_RATE_I = rate_integral_limit: 30.0, 0.0, 1000.0, "rad/s^2", Limits, live,
            "Limit on the rate loop integral term.";
        f32 LIM_VEL_I = velocity_integral_limit: 3.0, 0.0, 30.0, "m/s^2", Limits, live,
            "Limit on the velocity loop integral term.";
        f32 LIM_MIN_THRUST = min_thrust_fraction_of_hover: 0.1, 0.0, BELOW_ONE, "", Limits, live,
            "Minimum collective thrust while flying, as a fraction of hover thrust. Must stay below LAND_TD_THRUST.";
        f32 LIM_IDLE = idle_command: 0.05, 0.0, 1.0, "", Limits, live,
            "Motor command while armed with no demand, 0 to 1.";
        f32 LIM_STICK_DB = stick_deadband: 0.05, 0.0, 0.5, "", Limits, live,
            "Stick deadband: a stick within this of centre reads as centred, and the rest of the travel is rescaled to the full range.";
    }
    EkfParams {
        secs EKF_HORIZON = delay_horizon_us: 0.2, MIN_US, 0.5, "s", Estimator, disarmed,
            "How far the filter runs behind real time. Must exceed the largest GPS latency and fit the profile's IMU buffer.";
        f32 EKF_GYR_NOISE = gyro_noise: 0.015, TINY, 1.0, "rad/s", Estimator, disarmed,
            "Gyro process noise (1 sigma).";
        f32 EKF_ACC_NOISE = accel_noise: 0.35, TINY, 10.0, "m/s^2", Estimator, disarmed,
            "Accelerometer process noise (1 sigma).";
        f32 EKF_GYR_BNOISE = gyro_bias_noise: 1.0e-3, TINY, 0.1, "rad/s^2", Estimator, disarmed,
            "Gyro bias random walk.";
        f32 EKF_ACC_BNOISE = accel_bias_noise: 3.0e-3, TINY, 1.0, "m/s^3", Estimator, disarmed,
            "Accelerometer bias random walk.";
        f32 EKF_GPS_POS_N = gps_pos_noise_m: 0.5, TINY, 1000.0, "m", Estimator, disarmed,
            "Lower bound on the GPS horizontal position noise; the receiver's own accuracy is used when larger.";
        f32 EKF_GPS_VEL_N = gps_vel_noise_mps: 0.3, TINY, 100.0, "m/s", Estimator, disarmed,
            "Lower bound on the GPS velocity noise.";
        f32 EKF_BARO_NOISE = baro_noise_m: 0.5, TINY, 1000.0, "m", Estimator, disarmed,
            "Barometric height noise.";
        f32 EKF_YAW_NOISE = yaw_noise_rad: 0.15, TINY, PI, "rad", Estimator, disarmed,
            "Magnetic heading noise.";
        f32 EKF_GPS_GATE = gps_gate: 5.0, TINY, 100.0, "sigma", Estimator, disarmed,
            "GPS innovation gate, in standard deviations.";
        f32 EKF_BARO_GATE = baro_gate: 5.0, TINY, 100.0, "sigma", Estimator, disarmed,
            "Barometer innovation gate, in standard deviations.";
        f32 EKF_YAW_GATE = yaw_gate: 3.0, TINY, 100.0, "sigma", Estimator, disarmed,
            "Heading innovation gate, in standard deviations.";
        f32 EKF_MAG_DECL = mag_declination_rad: 0.0, -PI, PI, "rad", Estimator, disarmed,
            "Magnetic declination at the site, east positive.";
        f32 EKF_INIT_TILT = init_tilt_sigma_rad: 0.1, TINY, PI, "rad", Estimator, disarmed,
            "Initial tilt uncertainty after alignment (1 sigma).";
        f32 EKF_INIT_YAW = init_yaw_sigma_rad: 0.3, TINY, PI, "rad", Estimator, disarmed,
            "Initial yaw uncertainty after alignment (1 sigma).";
        f32 EKF_INIT_VEL = init_vel_sigma_mps: 0.5, TINY, 100.0, "m/s", Estimator, disarmed,
            "Initial velocity uncertainty after alignment (1 sigma).";
        f32 EKF_INIT_POS = init_pos_sigma_m: 1.0, TINY, 1000.0, "m", Estimator, disarmed,
            "Initial position uncertainty after alignment (1 sigma).";
        f32 EKF_INIT_GBIAS = init_gyro_bias_sigma: 0.02, TINY, 1.0, "rad/s", Estimator, disarmed,
            "Initial gyro bias uncertainty (1 sigma).";
        f32 EKF_INIT_ABIAS = init_accel_bias_sigma: 0.1, TINY, 10.0, "m/s^2", Estimator, disarmed,
            "Initial accelerometer bias uncertainty (1 sigma). Bounds how far the tilt estimate can wander in hover.";
        secs EKF_ALIGN_TIME = align_time_us: 1.0, MIN_US, 60.0, "s", Estimator, disarmed,
            "The vehicle must sit still this long before the filter aligns.";
        secs EKF_GPS_TMO = gps_timeout_us: 5.0, MIN_US, DUR, "s", Estimator, disarmed,
            "GPS aiding stops after this long without an accepted fix; also how long horizontal velocity stays valid.";
        secs EKF_POS_TMO = pos_timeout_us: 1.5, MIN_US, DUR, "s", Estimator, disarmed,
            "Horizontal position stays valid this long after the last accepted GPS fix.";
        secs EKF_BARO_TMO = baro_timeout_us: 5.0, MIN_US, DUR, "s", Estimator, disarmed,
            "Height stays valid this long after the last accepted barometer reading.";
        secs EKF_MAG_TMO = mag_timeout_us: 30.0, MIN_US, DUR, "s", Estimator, disarmed,
            "Heading stays valid this long after the last accepted magnetometer reading.";
        f32 EKF_OUT_TAU = output_tau_s: 0.25, TINY, 10.0, "s", Estimator, disarmed,
            "Time constant with which the output predictor follows the filter.";
    }
    EstimatorConfig {
        source EKF_SOURCE = source: 0, 0, 1, "", Estimator, disarmed,
            "Where the state estimate comes from: 0 external (simulator truth or motion capture), 1 the EKF.";
    }
    FailsafeParams {
        secs FS_RC_TIMEOUT = rc_timeout_us: 0.5, MIN_TIMEOUT, MAX_TIMEOUT, "s", Failsafe, live,
            "RC counts as lost after this long without a valid frame.";
        f32 FS_RC_VALID = rc_valid_min_s: 1.0, 0.0, DUR, "s", Failsafe, live,
            "RC must be valid this long without a gap before pilot input counts (arming, override).";
        f32 FS_RC_HOLD = rc_loss_hold_s: 3.0, 0.0, DUR, "s", Failsafe, live,
            "After RC loss in a pilot mode: hold this long, then return.";
        secs FS_GCS_TIMEOUT = gcs_timeout_us: 3.0, MIN_TIMEOUT, MAX_TIMEOUT, "s", Failsafe, live,
            "The ground station counts as lost after this long without a heartbeat.";
        f32 FS_NOLINK_RTL = no_link_rtl_s: 60.0, 0.0, DUR, "s", Failsafe, live,
            "Offboard with no human link: continue this long, then return.";
        secs FS_BRAIN_TMO = brain_timeout_us: 2.0, MIN_TIMEOUT, MAX_TIMEOUT, "s", Failsafe, live,
            "The brain counts as lost when its progress counter has not advanced for this long.";
        f32 FS_BRAIN_RTL = brain_loss_rtl_s: 10.0, 0.0, DUR, "s", Failsafe, live,
            "After brain loss: hold this long, then return.";
        secs FS_SETPT_TMO = setpoint_timeout_us: 0.5, MIN_TIMEOUT, MAX_TIMEOUT, "s", Failsafe, live,
            "Tier-2 setpoints count as stale after this long.";
        f32 FS_GPS_LAND = gps_loss_land_s: 30.0, 0.0, DUR, "s", Failsafe, live,
            "After losing position with nobody flying: hold this long, then land.";
        f32 FS_CLEAR_HYST = clear_hysteresis_s: 1.0, 0.0, DUR, "s", Failsafe, live,
            "A hold ends when its reasons have been clear this long.";
        f32 BAT_LOW_V = battery_low_v: 3.5, BATTERY_MIN_V, BATTERY_MAX_V, "V", Battery, live,
            "Low battery threshold, volts per cell, on the filtered, load-compensated voltage. Must not be below BAT_CRIT_V.";
        f32 BAT_CRIT_V = battery_critical_v: 3.3, BATTERY_MIN_V, BATTERY_MAX_V, "V", Battery, live,
            "Critical battery threshold, volts per cell. Must not be above BAT_LOW_V.";
        f32 BAT_DEBOUNCE = battery_debounce_s: 2.0, 0.0, DUR, "s", Battery, live,
            "The voltage must stay below a threshold this long to latch.";
        f32 BAT_FILT_TAU = battery_filter_tau_s: 2.0, TINY, DUR, "s", Battery, live,
            "Time constant of the battery voltage low-pass.";
        f32 FS_FENCE_RAD = fence_radius_m: 0.0, 0.0, 1000000.0, "m", Failsafe, live,
            "Geofence radius around home. Zero disables it.";
        f32 FS_FENCE_ALT = fence_altitude_m: 0.0, 0.0, 1000000.0, "m", Failsafe, live,
            "Geofence height above home. Zero disables it.";
        f32 FS_LOC_ERR = loc_error_rad: 45.0_f32.to_radians(), TINY, PI, "rad", Failsafe, live,
            "Loss of control: attitude error above this for FS_LOC_ERR_T.";
        f32 FS_LOC_ERR_T = loc_error_s: 0.5, 0.0, DUR, "s", Failsafe, live,
            "Loss of control: how long the attitude error must stay above FS_LOC_ERR.";
        f32 FS_LOC_TILT = loc_tilt_rad: 75.0_f32.to_radians(), TINY, PI, "rad", Failsafe, live,
            "Loss of control: tilt above this for FS_LOC_TILT_T.";
        f32 FS_LOC_TILT_T = loc_tilt_s: 0.25, 0.0, DUR, "s", Failsafe, live,
            "Loss of control: how long the tilt must stay above FS_LOC_TILT.";
        f32 RTL_ALT = rtl_altitude_m: 10.0, 0.0, 10000.0, "m", Return, live,
            "Return altitude above home.";
        f32 RTL_SPEED = rtl_speed_mps: 3.0, 0.1, 100.0, "m/s", Return, live,
            "Return ground speed.";
        f32 LAND_SPEED = land_speed_mps: 0.7, 0.0, 10.0, "m/s", Land, live,
            "Landing descent rate. Must be at least twice LAND_TD_SPEED.";
        f32 LAND_BLIND_THR = land_blind_thrust: 0.9, 0.0, 1.0, "", Land, live,
            "Landing without a valid height: thrust as a fraction of hover thrust. Must be above LAND_TD_THRUST.";
        f32 LAND_TD_THRUST = touchdown_thrust: 0.3, TINY, BELOW_ONE, "", Land, live,
            "Touchdown: thrust below this fraction of hover thrust. Must be above LIM_MIN_THRUST and below LAND_BLIND_THR.";
        f32 LAND_TD_SPEED = touchdown_speed_mps: 0.25, TINY, 5.0, "m/s", Land, live,
            "Touchdown: vertical speed below this.";
        f32 LAND_TD_TIME = touchdown_s: 1.0, TINY, DUR, "s", Land, live,
            "Touchdown: for this long.";
    }
}

// ---- access ----

fn load_cfg<const N: usize>(cfg: &VehicleConfig<N>, id: ParamId) -> Option<Stored> {
    Section::load(&cfg.gains, id)
        .or_else(|| Section::load(&cfg.limits, id))
        .or_else(|| Section::load(&cfg.estimator.ekf, id))
        .or_else(|| Section::load(&cfg.estimator, id))
        .or_else(|| Section::load(&cfg.failsafe, id))
}

fn store_cfg<const N: usize>(
    cfg: &mut VehicleConfig<N>,
    id: ParamId,
    v: ParamValue,
) -> Result<(), ParamError> {
    if Section::store(&mut cfg.gains, id, v)?
        || Section::store(&mut cfg.limits, id, v)?
        || Section::store(&mut cfg.estimator.ekf, id, v)?
        || Section::store(&mut cfg.estimator, id, v)?
        || Section::store(&mut cfg.failsafe, id, v)?
    {
        Ok(())
    } else {
        Err(ParamError::Unknown)
    }
}

/// The current value of a parameter; `None` for an unknown id. A parameter
/// held as whole microseconds reads in seconds.
pub fn get<const N: usize>(cfg: &VehicleConfig<N>, id: ParamId) -> Option<ParamValue> {
    load_cfg(cfg, id).map(to_value)
}

/// Changes one parameter. The value must be of the parameter's type, finite
/// and within `[min, max]`, and the configuration with the change must pass
/// [`VehicleConfig::validate`] and the estimator's checks against `profile`
/// (rules that involve more than one field). A refused change leaves `cfg`
/// untouched. Whether the vehicle is armed is the caller's business: see
/// [`ParamDef::live`].
pub fn set<const N: usize>(
    cfg: &mut VehicleConfig<N>,
    profile: &Profile,
    id: ParamId,
    value: ParamValue,
) -> Result<(), ParamError> {
    let def = PARAMS.get(id.index()).ok_or(ParamError::Unknown)?;
    if value.kind() != def.kind {
        return Err(ParamError::WrongType);
    }
    if let ParamValue::F32(v) = value
        && !v.is_finite()
    {
        return Err(ParamError::NotFinite);
    }
    let mut next = *cfg;
    store_cfg(&mut next, id, value)?;
    let stored = load_cfg(&next, id).ok_or(ParamError::Unknown)?;
    if !stored_in_range(def, stored) {
        return Err(ParamError::OutOfRange);
    }
    next.validate().map_err(ParamError::Invalid)?;
    next.estimator
        .ekf
        .validate(profile)
        .map_err(ParamError::Invalid)?;
    *cfg = next;
    Ok(())
}

/// The parameter called `name`.
pub fn find(name: &str) -> Option<ParamId> {
    PARAMS
        .iter()
        .position(|d| d.name == name)
        .and_then(|i| u16::try_from(i).ok())
        .map(ParamId)
}

/// The stable storage key of a parameter name: 32-bit FNV-1a. Reordering the
/// table never changes what a stored key means.
pub const fn key(name: &str) -> u32 {
    let mut h: u32 = 0x811c_9dc5;
    let mut rest = name.as_bytes();
    while let [first, tail @ ..] = rest {
        h ^= *first as u32;
        h = h.wrapping_mul(0x0100_0193);
        rest = tail;
    }
    h
}
