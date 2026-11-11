// SPDX-License-Identifier: AGPL-3.0-or-later
//! The parameter table (ADR-0004): every tunable value declared once, with
//! its name, type, default, range, unit, group and description.
//!
//! The configuration structs stay the storage; the table describes their
//! fields and [`get`] and [`set`] read and write them.

use crate::config::ConfigError;

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
