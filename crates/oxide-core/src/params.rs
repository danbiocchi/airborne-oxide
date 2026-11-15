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

        /// The storage key of every parameter, in table order: `KEYS[id]` is
        /// `key(PARAMS[id].name)`, computed at compile time.
        pub const KEYS: [u32; COUNT] = [$( $( key(stringify!($name)), )* )*];

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
            "Cut-off of the second-order Butterworth low-pass on the gyro for the rate loop. Zero turns it off. Must be below 0.45 times the rate-loop frequency of the profile (450 Hz at 1 kHz, 1800 Hz at 4 kHz); at or above that the filter used to be switched off silently, and now the value is refused. The filter is rebuilt when this changes.";
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
            "Maximum tilt in the position and offboard modes. FS_LOC_TILT must be at least 0.175 rad above it.";
        f32 LIM_TILT_STAB = max_tilt_stabilized_rad: 45.0_f32.to_radians(), 0.01, 1.39, "rad", Limits, live,
            "Maximum tilt in the stabilized mode, at full stick. FS_LOC_TILT must be at least 0.175 rad above it.";
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
            "Maximum descent speed. It clamps the descent LAND_SPEED commands, so it must be at least twice LAND_TD_SPEED.";
        f32 LIM_YAW_RATE = max_yaw_rate: 2.0, 0.01, 35.0, "rad/s", Limits, live,
            "Yaw rate at full yaw stick.";
        f32 LIM_RATE_I = rate_integral_limit: 30.0, 0.0, 1000.0, "rad/s^2", Limits, live,
            "Limit on the rate loop integral term.";
        f32 LIM_VEL_I = velocity_integral_limit: 3.0, 0.0, 30.0, "m/s^2", Limits, live,
            "Limit on the velocity loop integral term.";
        f32 LIM_MIN_THRUST = min_thrust_fraction_of_hover: 0.1, 0.0, BELOW_ONE, "", Limits, live,
            "Minimum collective thrust while flying, as a fraction of hover thrust. Must stay below LAND_TD_THRUST.";
        f32 LIM_IDLE = idle_command: 0.05, 0.0, 0.5, "", Limits, live,
            "Lowest motor command on every motor at all times while armed, 0 to 0.5: a floor, not only a command for no demand. The thrust it gives (through the thrust curve) must be at most half of the hover thrust of a rotor, or the vehicle could not be held down or steered.";
        f32 LIM_STICK_DB = stick_deadband: 0.05, 0.0, 0.5, "", Limits, live,
            "Stick deadband: a stick within this of centre reads as centred, and the rest of the travel is rescaled to the full range.";
    }
    EkfParams {
        secs EKF_HORIZON = delay_horizon_us: 0.2, MIN_US, 0.248, "s", Estimator, disarmed,
            "How far the filter runs behind real time. Must exceed the largest GPS latency, fit the profile's IMU buffer, and not exceed EKF_OUT_TAU.";
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
        f32 EKF_OUT_TAU = output_tau_s: 0.25, 0.15, 10.0, "s", Estimator, disarmed,
            "Time constant with which the output predictor follows the filter. At least 0.15 s and at least EKF_HORIZON: flown on gust-hold, the estimate oscillates and the flight is lost below about 0.7 of the horizon (0.14 s at the default horizon).";
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
            "The ground station counts as lost after this long without a heartbeat. Must be longer than the period at which the ground station sends it (the simulator's is 1 s), or a live link flaps between lost and found.";
        f32 FS_NOLINK_RTL = no_link_rtl_s: 60.0, 0.0, DUR, "s", Failsafe, live,
            "Offboard with no human link: continue this long, then return.";
        secs FS_BRAIN_TMO = brain_timeout_us: 2.0, MIN_TIMEOUT, MAX_TIMEOUT, "s", Failsafe, live,
            "The brain counts as lost when its progress counter has not advanced for this long. Must be longer than the period at which the brain sends its heartbeat (the simulator's is 0.2 s), or a live brain counts as lost.";
        f32 FS_BRAIN_RTL = brain_loss_rtl_s: 10.0, 0.0, DUR, "s", Failsafe, live,
            "After brain loss: hold this long, then return.";
        secs FS_SETPT_TMO = setpoint_timeout_us: 0.5, MIN_TIMEOUT, MAX_TIMEOUT, "s", Failsafe, live,
            "Tier-2 setpoints count as stale after this long. Must be longer than the period at which the brain sends setpoints (the simulator's is 0.02 s), or live setpoints count as stale.";
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
        f32 FS_LOC_ERR = loc_error_rad: 45.0_f32.to_radians(), 0.35, PI, "rad", Failsafe, live,
            "Loss of control: attitude error above this for FS_LOC_ERR_T. At least 0.35 rad (20 degrees): below it normal flight switches the motors off.";
        f32 FS_LOC_ERR_T = loc_error_s: 0.5, 0.1, DUR, "s", Failsafe, live,
            "Loss of control: how long the attitude error must stay above FS_LOC_ERR. At least 0.1 s.";
        f32 FS_LOC_TILT = loc_tilt_rad: 75.0_f32.to_radians(), 0.185, PI, "rad", Failsafe, live,
            "Loss of control: tilt above this for FS_LOC_TILT_T. Must be at least 0.175 rad (10 degrees) above both LIM_TILT and LIM_TILT_STAB, or the tilt the controller commands switches the motors off.";
        f32 FS_LOC_TILT_T = loc_tilt_s: 0.25, 0.05, DUR, "s", Failsafe, live,
            "Loss of control: how long the tilt must stay above FS_LOC_TILT. At least 0.05 s.";
        f32 RTL_ALT = rtl_altitude_m: 10.0, 0.0, 10000.0, "m", Return, live,
            "Return altitude above home. A change applies from the next return: one already under way keeps the height it started with.";
        f32 RTL_SPEED = rtl_speed_mps: 3.0, 0.1, 100.0, "m/s", Return, live,
            "Return ground speed.";
        f32 LAND_SPEED = land_speed_mps: 0.7, 0.0, 10.0, "m/s", Land, live,
            "Landing descent rate. Must be at least twice LAND_TD_SPEED.";
        f32 LAND_BLIND_THR = land_blind_thrust: 0.9, 0.0, 1.0, "", Land, live,
            "Landing without a valid height: thrust as a fraction of hover thrust. Must be above LAND_TD_THRUST.";
        f32 LAND_TD_THRUST = touchdown_thrust: 0.3, TINY, BELOW_ONE, "", Land, live,
            "Touchdown: thrust below this fraction of hover thrust. Must be above LIM_MIN_THRUST and below LAND_BLIND_THR.";
        f32 LAND_TD_SPEED = touchdown_speed_mps: 0.25, 0.1, 5.0, "m/s", Land, live,
            "Touchdown: vertical speed below this. At least 0.1 m/s: the estimated vertical speed of a landed vehicle on realistic sensors reaches 0.09 m/s, and below its noise the touchdown is never detected. LAND_SPEED and LIM_SPEED_DOWN must each be at least twice this.";
        f32 LAND_TD_TIME = touchdown_s: 1.0, 0.1, 10.0, "s", Land, live,
            "Touchdown: for this long, 0.1 to 10 s (a longer test is a vehicle that never disarms).";
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

/// Checks one value on its own (type, finite, range) and writes it to `next`.
fn put<const N: usize>(
    next: &mut VehicleConfig<N>,
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
    store_cfg(next, id, value)?;
    let stored = load_cfg(next, id).ok_or(ParamError::Unknown)?;
    if !stored_in_range(def, stored) {
        return Err(ParamError::OutOfRange);
    }
    Ok(())
}

/// The rules between fields: the configuration's own and the estimator's
/// against `profile`.
fn check_together<const N: usize>(
    next: &VehicleConfig<N>,
    profile: &Profile,
) -> Result<(), ParamError> {
    next.validate().map_err(ParamError::Invalid)?;
    next.gains
        .check_against(profile)
        .map_err(ParamError::Invalid)?;
    next.estimator
        .ekf
        .validate(profile)
        .map_err(ParamError::Invalid)
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
    let mut next = *cfg;
    put(&mut next, id, value)?;
    check_together(&next, profile)?;
    *cfg = next;
    Ok(())
}

/// Applies several changes as one. Each value is checked on its own (type,
/// finite, range); a later entry for the same parameter overrides an earlier
/// one; the rules between fields are checked once, on the result. All or
/// nothing: on an error `cfg` is untouched.
///
/// The error carries the parameter when one value failed, and `None` (with
/// [`ParamError::Invalid`]) when the values are each fine but together break
/// a rule.
pub fn set_all<const N: usize>(
    cfg: &mut VehicleConfig<N>,
    profile: &Profile,
    changes: impl IntoIterator<Item = (ParamId, ParamValue)>,
) -> Result<(), (Option<ParamId>, ParamError)> {
    let mut next = *cfg;
    for (id, value) in changes {
        put(&mut next, id, value).map_err(|e| (Some(id), e))?;
    }
    check_together(&next, profile).map_err(|e| (None, e))?;
    *cfg = next;
    Ok(())
}

// ---- stored values ----

/// What loading stored values did (ADR-0004 §6).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct LoadReport {
    /// Values taken.
    pub applied: u16,
    /// Keys this firmware does not know.
    pub unknown: u16,
    /// Values not finite or out of this firmware's range; the default stays.
    pub dropped: u16,
    /// The values together broke a rule; none was taken.
    pub invalid: bool,
    /// The store itself could not be read. Set by the caller, never by
    /// [`load`].
    pub unreadable: bool,
    /// Records the store skipped as damaged or half-written, so the values
    /// taken may be older than the last save. Set by the caller, never by
    /// [`load`]. It does not make the report unclean: the core reports it
    /// (`EventKind::ParamLoadRecovered`) and holds no fault.
    pub torn: u16,
}

impl LoadReport {
    /// Everything stored was understood and taken: nothing unknown, dropped,
    /// invalid or unreadable (`applied` and `torn` do not matter).
    pub const fn clean(&self) -> bool {
        self.unknown == 0 && self.dropped == 0 && !self.invalid && !self.unreadable
    }
}

/// The 32 bits a stored value is kept as: `to_bits` of an `f32` (seconds for
/// the fields held as microseconds), an `i32` as `u32`.
fn stored_bits(s: Stored) -> u32 {
    match s {
        Stored::F(v) => v.to_bits(),
        Stored::Us(us) => us_to_secs(us).to_bits(),
        Stored::I(v) => v as u32,
    }
}

/// Decodes stored bits by the parameter's own kind, so there is no wrong type.
fn value_from_bits(kind: ParamKind, bits: u32) -> ParamValue {
    match kind {
        ParamKind::F32 => ParamValue::F32(f32::from_bits(bits)),
        ParamKind::I32 => ParamValue::I32(bits as i32),
    }
}

/// Applies stored `(key, bits)` pairs onto `cfg`, which holds the firmware's
/// own values (the baseline). Never fails and never leaves `cfg` invalid.
///
/// A key no parameter has is counted as unknown. A value that is not finite or
/// is out of range on its own is dropped and counted: the baseline value
/// stays. The rest are applied; then the rules between fields are checked once
/// on the result, and if they fail `cfg` is left exactly as it was given,
/// nothing counts as applied and `invalid` is set. A key that appears twice
/// takes its later value.
pub fn load<const N: usize>(
    cfg: &mut VehicleConfig<N>,
    profile: &Profile,
    records: impl IntoIterator<Item = (u32, u32)>,
) -> LoadReport {
    let mut report = LoadReport::default();
    let mut next = *cfg;
    for (key, bits) in records {
        let Some(id) = find_key(key) else {
            report.unknown = report.unknown.saturating_add(1);
            continue;
        };
        let Some(def) = PARAMS.get(id.index()) else {
            report.unknown = report.unknown.saturating_add(1);
            continue;
        };
        // `put` writes before it checks the range: try on a copy.
        let mut trial = next;
        if put(&mut trial, id, value_from_bits(def.kind, bits)).is_ok() {
            next = trial;
            report.applied = report.applied.saturating_add(1);
        } else {
            report.dropped = report.dropped.saturating_add(1);
        }
    }
    if report.applied > 0 {
        if check_together(&next, profile).is_ok() {
            *cfg = next;
        } else {
            report.applied = 0;
            report.invalid = true;
        }
    }
    report
}

/// The parameters whose value differs between `cfg` and `baseline`, as
/// `(key, bits)` in table order: what a store must hold so that [`load`] onto
/// `baseline` gives `cfg` again. Values are compared bit for bit.
pub fn changed<'a, const N: usize>(
    cfg: &'a VehicleConfig<N>,
    baseline: &'a VehicleConfig<N>,
) -> impl Iterator<Item = (u32, u32)> + Clone + 'a {
    KEYS.iter().enumerate().filter_map(move |(i, key)| {
        let id = ParamId(u16::try_from(i).ok()?);
        let now = stored_bits(load_cfg(cfg, id)?);
        let before = stored_bits(load_cfg(baseline, id)?);
        (now != before).then_some((*key, now))
    })
}

// Two names with one key would make a stored value ambiguous: refuse to build.
const _: () = {
    let mut i = 0;
    while i < COUNT {
        let mut j = i + 1;
        while j < COUNT {
            #[allow(clippy::indexing_slicing)]
            let same = KEYS[i] == KEYS[j];
            assert!(!same, "two parameters share a storage key");
            j += 1;
        }
        i += 1;
    }
};

/// The storage key of a parameter; `None` for an unknown id.
pub fn key_of(id: ParamId) -> Option<u32> {
    KEYS.get(id.index()).copied()
}

/// The parameter stored under `key`; `None` if this firmware has none.
pub fn find_key(key: u32) -> Option<ParamId> {
    KEYS.iter()
        .position(|k| *k == key)
        .and_then(|i| u16::try_from(i).ok())
        .map(ParamId)
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::{Rotor, Spin};
    use crate::math::Vec3;
    use crate::schedule::{F405_LEAN, H743_FULL};

    fn cfg() -> VehicleConfig<4> {
        let a = 0.08;
        let r = |x: f32, y: f32, spin| Rotor {
            x,
            y,
            spin,
            max_thrust_n: 8.0,
            torque_coeff_m: 0.016,
        };
        VehicleConfig {
            mass_kg: 0.65,
            inertia: Vec3::new(1.6e-3, 1.6e-3, 2.8e-3),
            rotors: [
                r(a, a, Spin::Ccw),
                r(-a, -a, Spin::Ccw),
                r(a, -a, Spin::Cw),
                r(-a, a, Spin::Cw),
            ],
            thrust_expo: 1.0,
            nominal_voltage_v: 22.2,
            gains: Gains::default(),
            limits: Limits::default(),
            estimator: EstimatorConfig::default(),
            failsafe: FailsafeParams::default(),
            battery_cells: 6,
            battery_resistance_ohm: 0.025,
        }
    }

    fn ids() -> impl Iterator<Item = (ParamId, &'static ParamDef)> {
        PARAMS
            .iter()
            .enumerate()
            .map(|(i, d)| (ParamId(i as u16), d))
    }

    fn f(v: ParamValue) -> f32 {
        match v {
            ParamValue::F32(x) => x,
            ParamValue::I32(_) => panic!("not a float"),
        }
    }

    fn next_up(x: f32) -> f32 {
        if x == 0.0 {
            f32::from_bits(1)
        } else if x > 0.0 {
            f32::from_bits(x.to_bits() + 1)
        } else {
            f32::from_bits(x.to_bits() - 1)
        }
    }

    fn next_down(x: f32) -> f32 {
        -next_up(-x)
    }

    /// True if the parameter is held as whole microseconds.
    fn is_us(id: ParamId) -> bool {
        matches!(
            Section::load(&FailsafeParams::default(), id)
                .or_else(|| Section::load(&EkfParams::default(), id)),
            Some(Stored::Us(_))
        )
    }

    /// The value just below `min` that must be refused.
    fn below(id: ParamId, def: &ParamDef) -> ParamValue {
        match def.min {
            ParamValue::I32(v) => ParamValue::I32(v - 1),
            ParamValue::F32(lo) if is_us(id) => ParamValue::F32(lo - 2.0e-6),
            ParamValue::F32(lo) => ParamValue::F32(next_down(lo)),
        }
    }

    /// The value just above `max` that must be refused.
    fn above(id: ParamId, def: &ParamDef) -> ParamValue {
        match def.max {
            ParamValue::I32(v) => ParamValue::I32(v + 1),
            ParamValue::F32(hi) if is_us(id) => {
                let hi_us = secs_to_us(hi).unwrap();
                let mut v = hi + 2.0e-6;
                while secs_to_us(v).is_none_or(|u| u <= hi_us) {
                    v = next_up(v);
                }
                ParamValue::F32(v)
            }
            ParamValue::F32(hi) => ParamValue::F32(next_up(hi)),
        }
    }

    #[test]
    fn names_are_unique_legal_and_at_most_16_characters() {
        for (i, d) in PARAMS.iter().enumerate() {
            assert!(!d.name.is_empty() && d.name.len() <= 16, "{}", d.name);
            assert!(
                d.name
                    .bytes()
                    .all(|b| b.is_ascii_uppercase() || b.is_ascii_digit() || b == b'_'),
                "{}",
                d.name
            );
            assert!(
                d.name.starts_with(d.group.prefix()),
                "{} is in group {:?}",
                d.name,
                d.group
            );
            for other in PARAMS.iter().skip(i + 1) {
                assert_ne!(d.name, other.name);
            }
            assert!(!d.description.is_empty(), "{}", d.name);
        }
        assert_eq!(PARAMS.len(), COUNT);
        assert!(COUNT <= usize::from(u16::MAX));
    }

    #[test]
    fn keys_are_unique() {
        for (i, d) in PARAMS.iter().enumerate() {
            for other in PARAMS.iter().skip(i + 1) {
                assert_ne!(key(d.name), key(other.name), "{} {}", d.name, other.name);
            }
        }
    }

    #[test]
    fn the_key_table_has_one_distinct_key_per_parameter_and_find_key_inverts_it() {
        assert_eq!(KEYS.len(), COUNT);
        for (id, d) in ids() {
            let k = KEYS[id.index()];
            assert_eq!(k, key(d.name), "{}", d.name);
            assert_eq!(key_of(id), Some(k));
            assert_eq!(find_key(k), Some(id), "{}", d.name);
        }
        for (i, a) in KEYS.iter().enumerate() {
            for b in KEYS.iter().skip(i + 1) {
                assert_ne!(a, b);
            }
        }
        assert_eq!(key_of(ParamId(COUNT as u16)), None);
        // A key no parameter has.
        let unused = (0u32..).find(|k| !KEYS.contains(k)).unwrap();
        assert_eq!(find_key(unused), None);
        assert_eq!(find_key(u32::MAX).is_some(), KEYS.contains(&u32::MAX));
    }

    #[test]
    fn the_key_is_fnv1a_32() {
        // Published test vectors of 32-bit FNV-1a.
        assert_eq!(key(""), 0x811c_9dc5);
        assert_eq!(key("a"), 0xe40c_292c);
        assert_eq!(key("foobar"), 0xbf9c_f968);
    }

    #[test]
    fn every_default_is_within_its_range_and_of_its_type() {
        for (id, d) in ids() {
            assert_eq!(d.default.kind(), d.kind, "{}", d.name);
            assert_eq!(d.min.kind(), d.kind, "{}", d.name);
            assert_eq!(d.max.kind(), d.kind, "{}", d.name);
            let s = match d.default {
                ParamValue::F32(v) if is_us(id) => Stored::Us(secs_to_us(v).unwrap()),
                ParamValue::F32(v) => Stored::F(v),
                ParamValue::I32(v) => Stored::I(v),
            };
            assert!(stored_in_range(d, s), "{} default {:?}", d.name, d.default);
        }
    }

    #[test]
    fn the_default_configuration_reads_back_the_table_defaults() {
        let c = cfg();
        for (id, d) in ids() {
            assert_eq!(get(&c, id), Some(d.default), "{}", d.name);
        }
    }

    #[test]
    fn set_to_default_minimum_and_maximum_reads_back() {
        for profile in [&F405_LEAN, &H743_FULL] {
            for (id, d) in ids() {
                for v in [d.default, d.min, d.max] {
                    let mut c = cfg();
                    match set(&mut c, profile, id, v) {
                        Ok(()) => {
                            let got = get(&c, id).unwrap();
                            if is_us(id) {
                                assert_eq!(
                                    secs_to_us(f(got)),
                                    secs_to_us(f(v)),
                                    "{} = {v:?}",
                                    d.name
                                );
                            } else {
                                assert_eq!(got, v, "{} = {v:?}", d.name);
                            }
                        }
                        // A rule between two fields; nothing else may refuse
                        // an in-range value.
                        Err(ParamError::Invalid(_)) => {
                            assert_eq!(c, cfg(), "{} = {v:?} left a change", d.name);
                        }
                        Err(e) => panic!("{} = {v:?}: {e:?}", d.name),
                    }
                }
            }
        }
    }

    #[test]
    fn the_table_minimum_and_maximum_are_each_accepted_on_a_profile_or_refused_by_a_rule_between_fields()
     {
        // Bounds that only a rule between two fields refuses, by name. Any
        // other refusal of a bound means the table lies about the range.
        let between_fields = [
            "LAND_TD_THRUST",
            "LAND_TD_SPEED",
            "LIM_MIN_THRUST",
            "LAND_BLIND_THR",
            "LAND_SPEED",
            "LIM_IDLE",
            "LIM_SPEED_DOWN",
            "LIM_TILT",
            "LIM_TILT_STAB",
            "FS_LOC_TILT",
            "RATE_GYRO_CUTOFF",
            "EKF_OUT_TAU",
            "BAT_CRIT_V",
            "BAT_LOW_V",
        ];
        let mut refused: Vec<&str> = Vec::new();
        for (id, d) in ids() {
            for (what, v) in [("minimum", d.min), ("maximum", d.max)] {
                let mut accepted = false;
                for profile in [&F405_LEAN, &H743_FULL] {
                    let mut c = cfg();
                    match set(&mut c, profile, id, v) {
                        Ok(()) => accepted = true,
                        Err(ParamError::Invalid(_)) => {
                            assert!(
                                between_fields.contains(&d.name),
                                "{} {what} {v:?} is refused by a rule on that field alone",
                                d.name
                            );
                            if !refused.contains(&d.name) {
                                refused.push(d.name);
                            }
                            assert_eq!(c, cfg(), "{}", d.name);
                        }
                        Err(e) => panic!("{} {what} {v:?}: {e:?}", d.name),
                    }
                }
                assert!(
                    accepted || between_fields.contains(&d.name),
                    "{} {what} {v:?} is accepted on neither profile",
                    d.name
                );
            }
        }
        // The list holds exactly the bounds that are refused, no more.
        for name in between_fields {
            assert!(
                refused.contains(&name),
                "{name} is listed but never refused"
            );
        }
    }

    #[test]
    fn the_defaults_themselves_are_always_accepted() {
        for (id, d) in ids() {
            let mut c = cfg();
            assert_eq!(set(&mut c, &F405_LEAN, id, d.default), Ok(()), "{}", d.name);
            assert_eq!(c, cfg(), "{}", d.name);
        }
    }

    #[test]
    fn each_parameter_is_its_own_field() {
        // Changing one parameter changes that one and no other.
        let mut moved = 0;
        for (id, d) in ids() {
            let before = moved;
            // The extremes may break a rule between fields (the touchdown
            // thrust must stay between two other values), so try a value
            // a little off the default too.
            let near = match d.default {
                ParamValue::F32(x) => ParamValue::F32(x * 0.9),
                ParamValue::I32(x) => ParamValue::I32(x),
            };
            for v in [d.max, d.min, near] {
                let mut c = cfg();
                if set(&mut c, &F405_LEAN, id, v).is_err() || c == cfg() {
                    continue;
                }
                moved += 1;
                for (other, od) in ids() {
                    let want = if other == id {
                        get(&c, id)
                    } else {
                        Some(od.default)
                    };
                    assert_eq!(get(&c, other), want, "{} moved {}", d.name, od.name);
                }
                break;
            }
            assert_eq!(moved, before + 1, "{} never changes a field", d.name);
        }
        assert_eq!(moved, COUNT, "every parameter must reach a changed field");
    }

    #[test]
    fn values_just_outside_the_range_are_refused_and_change_nothing() {
        for (id, d) in ids() {
            for v in [below(id, d), above(id, d)] {
                let mut c = cfg();
                assert_eq!(
                    set(&mut c, &F405_LEAN, id, v),
                    Err(ParamError::OutOfRange),
                    "{} = {v:?}",
                    d.name
                );
                assert_eq!(c, cfg(), "{}", d.name);
                assert_eq!(format!("{c:?}"), format!("{:?}", cfg()), "{}", d.name);
            }
        }
    }

    #[test]
    fn nan_infinity_and_the_wrong_type_are_refused_and_change_nothing() {
        for (id, d) in ids() {
            let mut c = cfg();
            let wrong = match d.kind {
                ParamKind::F32 => ParamValue::I32(0),
                ParamKind::I32 => ParamValue::F32(0.0),
            };
            assert_eq!(
                set(&mut c, &F405_LEAN, id, wrong),
                Err(ParamError::WrongType),
                "{}",
                d.name
            );
            if d.kind == ParamKind::F32 {
                for bad in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
                    assert_eq!(
                        set(&mut c, &F405_LEAN, id, ParamValue::F32(bad)),
                        Err(ParamError::NotFinite),
                        "{} = {bad}",
                        d.name
                    );
                }
            }
            assert_eq!(c, cfg(), "{}", d.name);
            assert_eq!(format!("{c:?}"), format!("{:?}", cfg()), "{}", d.name);
        }
    }

    #[test]
    fn an_unknown_id_is_refused() {
        let mut c = cfg();
        let bad = ParamId(COUNT as u16);
        assert_eq!(
            set(&mut c, &F405_LEAN, bad, ParamValue::F32(1.0)),
            Err(ParamError::Unknown)
        );
        assert_eq!(
            set(&mut c, &F405_LEAN, ParamId(u16::MAX), ParamValue::I32(1)),
            Err(ParamError::Unknown)
        );
        assert_eq!(get(&c, bad), None);
        assert_eq!(c, cfg());
    }

    #[test]
    fn a_change_that_breaks_a_rule_between_fields_is_refused_and_changes_nothing() {
        let touchdown_speed = FailsafeParams::default().touchdown_speed_mps;
        let cases: [(&str, f32); 4] = [
            // Landing slower than twice the touchdown speed.
            ("LAND_SPEED", 2.0 * touchdown_speed * 0.99),
            // Critical battery above the low threshold.
            ("BAT_CRIT_V", 3.6),
            // Minimum thrust not below the touchdown thrust.
            ("LIM_MIN_THRUST", 0.3),
            // Blind landing thrust not above the touchdown thrust.
            ("LAND_BLIND_THR", 0.3),
        ];
        for (name, v) in cases {
            let mut c = cfg();
            let id = find(name).unwrap();
            assert_eq!(
                set(&mut c, &F405_LEAN, id, ParamValue::F32(v)),
                Err(ParamError::Invalid(ConfigError::BadFailsafe)),
                "{name} = {v}"
            );
            assert_eq!(c, cfg(), "{name}");
            assert_eq!(format!("{c:?}"), format!("{:?}", cfg()), "{name}");
        }
        // The low threshold below the critical one.
        let mut c = cfg();
        assert_eq!(
            set(&mut c, &F405_LEAN, ParamId::BAT_LOW_V, ParamValue::F32(3.2)),
            Err(ParamError::Invalid(ConfigError::BadFailsafe))
        );
        assert_eq!(c, cfg());
    }

    fn pair(name: &str, v: f32) -> (ParamId, ParamValue) {
        (find(name).unwrap(), ParamValue::F32(v))
    }

    #[test]
    fn set_all_accepts_a_pair_that_is_valid_only_together_in_either_order() {
        let crit = pair("BAT_CRIT_V", 3.6);
        let low = pair("BAT_LOW_V", 3.8);
        // One at a time, the critical threshold first is refused today.
        let mut one_by_one = cfg();
        assert_eq!(
            set(&mut one_by_one, &F405_LEAN, crit.0, crit.1),
            Err(ParamError::Invalid(ConfigError::BadFailsafe))
        );
        for order in [[crit, low], [low, crit]] {
            for profile in [&F405_LEAN, &H743_FULL] {
                let mut c = cfg();
                assert_eq!(set_all(&mut c, profile, order), Ok(()));
                assert_eq!(c.failsafe.battery_critical_v, 3.6);
                assert_eq!(c.failsafe.battery_low_v, 3.8);
            }
        }
    }

    #[test]
    fn set_all_is_all_or_nothing_and_names_the_bad_value() {
        let good = [pair("LIM_TILT", 0.5), pair("RATE_ROLL_P", 30.0)];
        let bad: [(ParamId, ParamValue, ParamError); 4] = [
            (
                find("LIM_TILT").unwrap(),
                ParamValue::F32(5.0),
                ParamError::OutOfRange,
            ),
            (
                find("LIM_TILT").unwrap(),
                ParamValue::F32(f32::NAN),
                ParamError::NotFinite,
            ),
            (
                find("LIM_TILT").unwrap(),
                ParamValue::I32(1),
                ParamError::WrongType,
            ),
            (ParamId(u16::MAX), ParamValue::F32(1.0), ParamError::Unknown),
        ];
        for (id, v, why) in bad {
            // Bad value first, in the middle and last.
            for at in 0..3 {
                let mut list = good.to_vec();
                list.insert(at, (id, v));
                let mut c = cfg();
                assert_eq!(
                    set_all(&mut c, &F405_LEAN, list),
                    Err((Some(id), why)),
                    "{why:?} at {at}"
                );
                assert_eq!(c, cfg());
                assert_eq!(format!("{c:?}"), format!("{:?}", cfg()));
            }
        }
    }

    #[test]
    fn set_all_with_nothing_changes_nothing_and_a_later_entry_overrides() {
        let mut c = cfg();
        assert_eq!(set_all(&mut c, &F405_LEAN, []), Ok(()));
        assert_eq!(c, cfg());
        // The first entry alone would break a rule; the second replaces it.
        let mut c = cfg();
        let list = [
            pair("BAT_LOW_V", 3.0),
            pair("BAT_LOW_V", 3.8),
            pair("BAT_CRIT_V", 3.6),
        ];
        assert_eq!(set_all(&mut c, &F405_LEAN, list), Ok(()));
        assert_eq!(c.failsafe.battery_low_v, 3.8);
        assert_eq!(c.failsafe.battery_critical_v, 3.6);
        // And the other way: the last entry is the one that must be valid.
        let mut c = cfg();
        let list = [pair("BAT_LOW_V", 3.8), pair("BAT_LOW_V", 3.0)];
        assert_eq!(
            set_all(&mut c, &F405_LEAN, list),
            Err((None, ParamError::Invalid(ConfigError::BadFailsafe)))
        );
        assert_eq!(c, cfg());
    }

    #[test]
    fn set_all_refuses_a_list_that_breaks_a_rule_together_with_none() {
        let mut c = cfg();
        let list = [pair("BAT_CRIT_V", 3.9), pair("BAT_LOW_V", 3.8)];
        assert_eq!(
            set_all(&mut c, &F405_LEAN, list),
            Err((None, ParamError::Invalid(ConfigError::BadFailsafe)))
        );
        assert_eq!(c, cfg());
        // The estimator's rule against the profile is part of "together".
        let mut c = cfg();
        let list = [pair("EKF_HORIZON", 0.248), pair("LIM_TILT", 0.5)];
        assert_eq!(set_all(&mut c, &F405_LEAN, list), Ok(()));
    }

    /// The bits a value is stored as: `to_bits` of an `f32`, an `i32` as `u32`.
    fn bits_of(v: ParamValue) -> u32 {
        match v {
            ParamValue::F32(x) => x.to_bits(),
            ParamValue::I32(x) => x as u32,
        }
    }

    fn rec(name: &str, v: ParamValue) -> (u32, u32) {
        (key(name), bits_of(v))
    }

    /// Every parameter's value, as bits, in table order.
    fn all_bits(c: &VehicleConfig<4>) -> Vec<u32> {
        ids().map(|(id, _)| bits_of(get(c, id).unwrap())).collect()
    }

    #[test]
    fn changed_then_load_reproduces_every_parameter_at_its_bounds_default_and_between() {
        for profile in [&F405_LEAN, &H743_FULL] {
            let base = cfg();
            let mut tried = 0;
            for (id, d) in ids() {
                let mid = match (d.min, d.max) {
                    (ParamValue::F32(lo), ParamValue::F32(hi)) => {
                        ParamValue::F32(lo + (hi - lo) * 0.5)
                    }
                    (ParamValue::I32(lo), ParamValue::I32(hi)) => {
                        ParamValue::I32(lo + (hi - lo) / 2)
                    }
                    _ => unreachable!(),
                };
                for v in [d.min, d.max, d.default, mid] {
                    let mut c = base;
                    match set(&mut c, profile, id, v) {
                        Ok(()) => {}
                        // A bound a rule between fields refuses, as in the
                        // bound test above.
                        Err(ParamError::Invalid(_)) => continue,
                        Err(e) => panic!("{} = {v:?}: {e:?}", d.name),
                    }
                    tried += 1;
                    let records: Vec<(u32, u32)> = changed(&c, &base).collect();
                    assert!(records.len() <= 1, "{} = {v:?}: {records:?}", d.name);
                    let mut loaded = base;
                    let r = load(&mut loaded, profile, records.iter().copied());
                    assert!(r.clean(), "{} = {v:?}: {r:?}", d.name);
                    assert_eq!(usize::from(r.applied), records.len());
                    assert_eq!(loaded, c, "{} = {v:?}", d.name);
                    assert_eq!(all_bits(&loaded), all_bits(&c), "{} = {v:?}", d.name);
                    // Loading what was just loaded changes nothing.
                    let mut twice = loaded;
                    let r2 = load(&mut twice, profile, records.iter().copied());
                    assert_eq!(r2, r);
                    assert_eq!(twice, loaded);
                }
            }
            assert!(tried > 4 * COUNT - 30, "only {tried} cases ran");
        }
    }

    #[test]
    fn changed_of_a_configuration_against_itself_is_empty_and_lists_only_differences() {
        let c = cfg();
        assert_eq!(changed(&c, &c).count(), 0);
        let mut d = c;
        set(&mut d, &F405_LEAN, ParamId::LIM_TILT, ParamValue::F32(0.5)).unwrap();
        set(&mut d, &F405_LEAN, ParamId::EKF_SOURCE, ParamValue::I32(1)).unwrap();
        set(
            &mut d,
            &F405_LEAN,
            ParamId::FS_RC_TIMEOUT,
            ParamValue::F32(0.75),
        )
        .unwrap();
        let got: Vec<(u32, u32)> = changed(&d, &c).collect();
        // In table order.
        assert_eq!(
            got,
            vec![
                rec("LIM_TILT", ParamValue::F32(0.5)),
                rec("EKF_SOURCE", ParamValue::I32(1)),
                // A parameter held as microseconds is stored in seconds.
                rec("FS_RC_TIMEOUT", ParamValue::F32(0.75)),
            ]
        );
        // The iterator can be walked more than once.
        let it = changed(&d, &c);
        assert_eq!(it.clone().count(), it.count());
        // The other way round, the values are the baseline's.
        let back: Vec<(u32, u32)> = changed(&c, &d).collect();
        assert_eq!(
            back,
            vec![
                rec("LIM_TILT", ParamValue::F32(Limits::default().max_tilt_rad)),
                rec("EKF_SOURCE", ParamValue::I32(0)),
                rec("FS_RC_TIMEOUT", ParamValue::F32(0.5)),
            ]
        );
    }

    #[test]
    fn load_counts_unknown_keys_and_changes_nothing_else() {
        let mut unknown = 0u32;
        while KEYS.contains(&unknown) {
            unknown += 1;
        }
        let mut c = cfg();
        let r = load(
            &mut c,
            &F405_LEAN,
            [(unknown, 0x3f80_0000), (unknown + 7, 1)],
        );
        assert_eq!(
            r,
            LoadReport {
                unknown: 2,
                ..LoadReport::default()
            }
        );
        assert!(!r.clean());
        assert_eq!(c, cfg());
        // A clean load is clean whatever it applied.
        let mut c = cfg();
        let r = load(&mut c, &F405_LEAN, [rec("LIM_TILT", ParamValue::F32(0.5))]);
        assert!(r.clean() && r.applied == 1, "{r:?}");
        assert_eq!(c.limits.max_tilt_rad, 0.5);
        let mut c = cfg();
        let r = load(&mut c, &F405_LEAN, []);
        assert_eq!(r, LoadReport::default());
        assert!(r.clean());
        assert_eq!(c, cfg());
    }

    #[test]
    fn load_drops_what_is_not_finite_or_out_of_range_and_applies_the_rest() {
        let mut c = cfg();
        let records = [
            rec("LIM_TILT", ParamValue::F32(f32::NAN)),
            rec("LIM_SPEED_XY", ParamValue::F32(f32::INFINITY)),
            rec("LIM_SPEED_UP", ParamValue::F32(f32::NEG_INFINITY)),
            rec("POS_X_P", ParamValue::F32(1.0e9)),
            rec("POS_Y_P", ParamValue::F32(-1.0)),
            rec("EKF_SOURCE", ParamValue::I32(2)),
            rec("EKF_HORIZON", ParamValue::F32(0.3)),
            rec("RTL_ALT", ParamValue::F32(12.0)),
        ];
        let r = load(&mut c, &F405_LEAN, records);
        assert_eq!(
            r,
            LoadReport {
                applied: 1,
                dropped: 7,
                ..LoadReport::default()
            }
        );
        assert!(!r.clean());
        let mut want = cfg();
        want.failsafe.rtl_altitude_m = 12.0;
        assert_eq!(c, want);
        // A dropped value leaves the default, not the last stored one.
        assert_eq!(c.limits.max_tilt_rad, Limits::default().max_tilt_rad);
    }

    #[test]
    fn load_leaves_the_baseline_alone_when_the_values_break_a_rule_together() {
        // Critical above low, with something valid in the same load.
        let mut c = cfg();
        let r = load(
            &mut c,
            &F405_LEAN,
            [
                rec("RTL_ALT", ParamValue::F32(12.0)),
                rec("BAT_CRIT_V", ParamValue::F32(3.9)),
            ],
        );
        assert_eq!(
            r,
            LoadReport {
                invalid: true,
                ..LoadReport::default()
            }
        );
        assert!(!r.clean());
        assert_eq!(c, cfg());
        assert_eq!(format!("{c:?}"), format!("{:?}", cfg()));
        // The same pair that is valid only together loads in either order.
        for pair in [
            [
                rec("BAT_CRIT_V", ParamValue::F32(3.6)),
                rec("BAT_LOW_V", ParamValue::F32(3.8)),
            ],
            [
                rec("BAT_LOW_V", ParamValue::F32(3.8)),
                rec("BAT_CRIT_V", ParamValue::F32(3.6)),
            ],
        ] {
            let mut c = cfg();
            let r = load(&mut c, &F405_LEAN, pair);
            assert!(r.clean() && r.applied == 2, "{r:?}");
            assert_eq!(c.failsafe.battery_critical_v, 3.6);
            assert_eq!(c.failsafe.battery_low_v, 3.8);
        }
        // Unknown and dropped records are still counted next to a broken rule.
        let mut c = cfg();
        let r = load(
            &mut c,
            &F405_LEAN,
            [
                rec("BAT_CRIT_V", ParamValue::F32(3.9)),
                rec("LIM_TILT", ParamValue::F32(f32::NAN)),
            ],
        );
        assert_eq!((r.invalid, r.dropped, r.applied), (true, 1, 0));
        assert_eq!(c, cfg());
    }

    #[test]
    fn a_microsecond_value_an_f32_cannot_hold_does_not_round_trip_exactly() {
        // Parameters in seconds are `f32`, which holds whole microseconds
        // exactly only up to 2^24 (16.7 s); `set` can only produce values
        // that survive, a field written directly may not.
        let base = cfg();
        let mut c = base;
        c.failsafe.rc_timeout_us = 3_600_000_001;
        let mut loaded = base;
        let r = load(&mut loaded, &F405_LEAN, changed(&c, &base));
        assert!(r.clean());
        assert_eq!(loaded.failsafe.rc_timeout_us, 3_600_000_000);
        // Every value `set` can make survives, up to the table's maximum.
        for x in [
            0.05_f32,
            0.123_456_7,
            16.777_217,
            100.000_01,
            3599.999,
            3600.0,
        ] {
            let mut c = base;
            set(
                &mut c,
                &F405_LEAN,
                ParamId::FS_RC_TIMEOUT,
                ParamValue::F32(x),
            )
            .unwrap();
            let mut loaded = base;
            load(&mut loaded, &F405_LEAN, changed(&c, &base));
            assert_eq!(loaded, c, "{x}");
        }
    }

    #[test]
    fn the_delay_horizon_is_checked_against_the_profile() {
        // 250 Hz filter on both profiles: 62 deltas of 4 ms fit, 63 do not. The
        // table maximum is the largest horizon that fits, so the range stops it.
        for profile in [&F405_LEAN, &H743_FULL] {
            let mut c = cfg();
            assert_eq!(
                set(
                    &mut c,
                    profile,
                    ParamId::EKF_HORIZON,
                    ParamValue::F32(0.248)
                ),
                Ok(())
            );
            assert_eq!(c.estimator.ekf.delay_horizon_us, 248_000);
            let before = c;
            assert_eq!(
                set(
                    &mut c,
                    profile,
                    ParamId::EKF_HORIZON,
                    ParamValue::F32(0.249)
                ),
                Err(ParamError::OutOfRange)
            );
            assert_eq!(c, before);
        }
    }

    #[test]
    fn microsecond_parameters_round_to_the_microsecond() {
        let mut c = cfg();
        let id = ParamId::FS_RC_TIMEOUT;
        assert_eq!(
            set(&mut c, &F405_LEAN, id, ParamValue::F32(0.1234567)),
            Ok(())
        );
        assert_eq!(c.failsafe.rc_timeout_us, 123_457);
        // The bound is exact in microseconds: 49 999 us is refused, 50 000
        // accepted, whatever the rounding of the f32 seconds.
        assert_eq!(
            set(&mut c, &F405_LEAN, id, ParamValue::F32(0.049_999)),
            Err(ParamError::OutOfRange)
        );
        assert_eq!(set(&mut c, &F405_LEAN, id, ParamValue::F32(0.05)), Ok(()));
        assert_eq!(c.failsafe.rc_timeout_us, 50_000);
        assert_eq!(set(&mut c, &F405_LEAN, id, ParamValue::F32(3600.0)), Ok(()));
        assert_eq!(c.failsafe.rc_timeout_us, 3_600_000_000);
    }

    #[test]
    fn the_estimator_source_is_an_enum_as_an_integer() {
        let mut c = cfg();
        let id = ParamId::EKF_SOURCE;
        assert_eq!(get(&c, id), Some(ParamValue::I32(0)));
        assert_eq!(set(&mut c, &F405_LEAN, id, ParamValue::I32(1)), Ok(()));
        assert_eq!(c.estimator.source, EstimatorSource::Ekf);
        assert_eq!(get(&c, id), Some(ParamValue::I32(1)));
        assert_eq!(set(&mut c, &F405_LEAN, id, ParamValue::I32(0)), Ok(()));
        assert_eq!(c.estimator.source, EstimatorSource::External);
        assert_eq!(
            set(&mut c, &F405_LEAN, id, ParamValue::I32(2)),
            Err(ParamError::OutOfRange)
        );
        assert_eq!(
            set(&mut c, &F405_LEAN, id, ParamValue::I32(-1)),
            Err(ParamError::OutOfRange)
        );
    }

    #[test]
    fn find_finds_every_name_and_nothing_else() {
        for (id, d) in ids() {
            assert_eq!(find(d.name), Some(id));
            assert_eq!(PARAMS.get(id.index()).map(|d| d.name), Some(d.name));
            // Not found with a change of case, a trailing character or a
            // dropped one.
            let lower = d.name.to_ascii_lowercase();
            assert_eq!(find(&lower), None, "{lower}");
            let longer = format!("{}X", d.name);
            assert_eq!(find(&longer), None, "{longer}");
            let shorter = &d.name[..d.name.len() - 1];
            if PARAMS.iter().all(|o| o.name != shorter) {
                assert_eq!(find(shorter), None, "{shorter}");
            }
        }
        assert_eq!(find(""), None);
        assert_eq!(find("NOT_A_PARAM"), None);
        assert_eq!(find("RATE_ROLL_P"), Some(ParamId::RATE_ROLL_P));
        assert_eq!(find("LIM_STICK_DB"), Some(ParamId::LIM_STICK_DB));
    }

    #[test]
    fn the_whole_configuration_validates_against_the_table() {
        assert_eq!(cfg().validate(), Ok(()));
        let mut c = cfg();
        c.gains.rate_p.x = -1.0;
        assert_eq!(c.validate(), Err(ConfigError::BadGains));
        let mut c = cfg();
        c.gains.position_p.z = f32::NAN;
        assert_eq!(c.validate(), Err(ConfigError::BadGains));
        let mut c = cfg();
        c.limits.max_tilt_rad = 0.0;
        assert_eq!(c.validate(), Err(ConfigError::BadLimits));
        let mut c = cfg();
        c.limits.idle_command = 1.5;
        assert_eq!(c.validate(), Err(ConfigError::BadLimits));
        let mut c = cfg();
        c.limits.stick_deadband = 0.6;
        assert_eq!(c.validate(), Err(ConfigError::BadLimits));
        let mut c = cfg();
        c.estimator.ekf.gps_gate = 0.0;
        assert_eq!(c.validate(), Err(ConfigError::BadEstimator));
        let mut c = cfg();
        c.failsafe.rtl_speed_mps = 0.0;
        assert_eq!(c.validate(), Err(ConfigError::BadFailsafe));
    }

    #[test]
    fn torn_records_are_reported_but_never_make_a_report_unclean_or_come_from_load() {
        let r = LoadReport {
            torn: 3,
            applied: 2,
            ..LoadReport::default()
        };
        assert!(r.clean());
        assert!(!LoadReport { unknown: 1, ..r }.clean());
        // `load` leaves `torn` to the caller.
        let mut c = cfg();
        let rep = load(&mut c, &F405_LEAN, [(KEYS[0], 0u32)]);
        assert_eq!(rep.torn, 0);
        let rep = load(&mut c, &F405_LEAN, []);
        assert_eq!(rep, LoadReport::default());
    }
}
