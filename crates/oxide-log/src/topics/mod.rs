// SPDX-License-Identifier: AGPL-3.0-or-later
//! The normal level's topics and their encoders (ADR-0005 §3).
//!
//! Each topic is a [`Topic`] constant (name, format string in PX4's field
//! order and padding, record length) and one function that fills a record
//! through a [`RecordWriter`]. Encoders do not decide when to write: the
//! recorder does. Each is given only what it needs, plus a [`Scales`] value
//! computed once from the configuration.
//!
//! Conventions are PX4's: quaternions are `w` first and rotate body to world,
//! thrust is normalised by the rotors' total maximum thrust and is negative
//! for up on the body z axis, torque is normalised by the largest torque the
//! airframe can produce about each axis, the magnetic field is in gauss.
//! `timestamp` is the tick time; `timestamp_sample`, where PX4's topic has it,
//! is the time of the data the record is based on (a sample's own
//! `t_sample`). Enumerations go through `oxide_core::codes`.
//!
//! A field PX4 has and the core has no value for is written as PX4's
//! "unknown": NaN where PX4 says NaN means "not controlled" or "invalid",
//! the `@invalid` value its message declares (battery), and zero otherwise.
//! Each encoder lists its own.

mod px4;

pub use px4::*;

use crate::Sink;
use crate::header::NestedFormat;
use crate::record::{RecordWriter, Topic};
use oxide_core::config::{Rotor, VehicleConfig};
use oxide_core::math::{Quat, Vec3};

/// The topics of the normal level, in the order of their message ids: a
/// topic's id is its position in this list.
pub const NORMAL: &[Topic] = &[
    px4::SENSOR_COMBINED,
    px4::VEHICLE_ANGULAR_VELOCITY,
    px4::VEHICLE_RATES_SETPOINT,
    px4::ACTUATOR_MOTORS,
    px4::VEHICLE_THRUST_SETPOINT,
    px4::VEHICLE_TORQUE_SETPOINT,
    px4::VEHICLE_ATTITUDE,
    px4::VEHICLE_ATTITUDE_SETPOINT,
    px4::VEHICLE_LOCAL_POSITION,
    px4::VEHICLE_LOCAL_POSITION_SETPOINT,
    px4::MANUAL_CONTROL_SETPOINT,
    px4::VEHICLE_MAGNETOMETER,
    px4::VEHICLE_AIR_DATA,
    px4::BATTERY_STATUS,
    px4::VEHICLE_STATUS,
    px4::FAILSAFE_FLAGS,
];

/// Message ids of [`NORMAL`]: `(topic name, id)`.
pub const IDS: &[(&str, u16)] = &[
    ("sensor_combined", id::SENSOR_COMBINED),
    ("vehicle_angular_velocity", id::VEHICLE_ANGULAR_VELOCITY),
    ("vehicle_rates_setpoint", id::VEHICLE_RATES_SETPOINT),
    ("actuator_motors", id::ACTUATOR_MOTORS),
    ("vehicle_thrust_setpoint", id::VEHICLE_THRUST_SETPOINT),
    ("vehicle_torque_setpoint", id::VEHICLE_TORQUE_SETPOINT),
    ("vehicle_attitude", id::VEHICLE_ATTITUDE),
    ("vehicle_attitude_setpoint", id::VEHICLE_ATTITUDE_SETPOINT),
    ("vehicle_local_position", id::VEHICLE_LOCAL_POSITION),
    (
        "vehicle_local_position_setpoint",
        id::VEHICLE_LOCAL_POSITION_SETPOINT,
    ),
    ("manual_control_setpoint", id::MANUAL_CONTROL_SETPOINT),
    ("vehicle_magnetometer", id::VEHICLE_MAGNETOMETER),
    ("vehicle_air_data", id::VEHICLE_AIR_DATA),
    ("battery_status", id::BATTERY_STATUS),
    ("vehicle_status", id::VEHICLE_STATUS),
    ("failsafe_flags", id::FAILSAFE_FLAGS),
];

/// The message id of each topic of [`NORMAL`] (its position in the list).
pub mod id {
    pub const SENSOR_COMBINED: u16 = 0;
    pub const VEHICLE_ANGULAR_VELOCITY: u16 = 1;
    pub const VEHICLE_RATES_SETPOINT: u16 = 2;
    pub const ACTUATOR_MOTORS: u16 = 3;
    pub const VEHICLE_THRUST_SETPOINT: u16 = 4;
    pub const VEHICLE_TORQUE_SETPOINT: u16 = 5;
    pub const VEHICLE_ATTITUDE: u16 = 6;
    pub const VEHICLE_ATTITUDE_SETPOINT: u16 = 7;
    pub const VEHICLE_LOCAL_POSITION: u16 = 8;
    pub const VEHICLE_LOCAL_POSITION_SETPOINT: u16 = 9;
    pub const MANUAL_CONTROL_SETPOINT: u16 = 10;
    pub const VEHICLE_MAGNETOMETER: u16 = 11;
    pub const VEHICLE_AIR_DATA: u16 = 12;
    pub const BATTERY_STATUS: u16 = 13;
    pub const VEHICLE_STATUS: u16 = 14;
    pub const FAILSAFE_FLAGS: u16 = 15;
}

/// Formats that topics of [`NORMAL`] nest, written before the topics. None of
/// the normal level's topics nests another type (`vehicle_gnss` will).
pub const NESTED: &[NestedFormat] = &[];

/// Constants computed once from the configuration that the encoders divide
/// by or copy.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Scales {
    /// The rotors' total maximum thrust, N (what thrust is normalised by).
    pub thrust_max_n: f32,
    /// The largest torque the rotors can produce about the body x, y and z
    /// axes, N·m (what torque is normalised by); see [`torque_limits`].
    pub torque_max_nm: Vec3<f32>,
    /// Battery cells in series.
    pub battery_cells: u8,
    /// Battery voltage at which the rotors' maximum thrust applies, V.
    pub nominal_voltage_v: f32,
    /// Internal resistance per cell, Ω: the configuration's pack resistance
    /// over the cell count (zero without cells).
    pub cell_resistance_ohm: f32,
}

impl Scales {
    /// The scales of a configuration.
    pub fn new<const N: usize>(cfg: &VehicleConfig<N>) -> Self {
        let cells = f32::from(cfg.battery_cells);
        Self {
            thrust_max_n: finite_or_zero(cfg.total_max_thrust_n()),
            torque_max_nm: torque_limits(&cfg.rotors),
            battery_cells: cfg.battery_cells,
            nominal_voltage_v: cfg.nominal_voltage_v,
            cell_resistance_ohm: if cells > 0.0 {
                cfg.battery_resistance_ohm / cells
            } else {
                0.0
            },
        }
    }
}

fn finite_or_zero(v: f32) -> f32 {
    if v.is_finite() { v } else { 0.0 }
}

/// The largest torque the rotors can produce about each body axis, N·m.
///
/// A rotor at `(x, y)` thrusting `T` along body −z gives a roll torque
/// `−y·T`, a pitch torque `x·T` and a yaw reaction torque `±k·T` (`k` its
/// torque coefficient, the sign its spin). Each torque is a sum of
/// coefficient times thrust with every thrust free in `[0, T_max]`, so about
/// one axis the largest positive torque has every rotor with a positive
/// coefficient at its maximum and the rest off, and the largest negative one
/// is the mirror image. The limit of an axis is the larger of the two
/// magnitudes (equal for a symmetric airframe). The other axes and the
/// collective thrust are left free: the limit is what that axis alone could
/// get, not a point the airframe can hold with the others at hover. A value
/// that is not finite counts as zero.
pub fn torque_limits<const N: usize>(rotors: &[Rotor; N]) -> Vec3<f32> {
    let axis = |coeff: fn(&Rotor) -> f32| -> f32 {
        let (mut up, mut down) = (0.0_f32, 0.0_f32);
        for r in rotors {
            let c = coeff(r) * r.max_thrust_n;
            if c > 0.0 {
                up += c;
            } else if c < 0.0 {
                down -= c;
            }
        }
        finite_or_zero(up.max(down))
    };
    Vec3::new(
        axis(|r| -r.y),
        axis(|r| r.x),
        axis(|r| r.spin.sign() * r.torque_coeff_m),
    )
}

// ---- helpers shared by the encoders ----

/// `v / scale`, or zero when the scale is not a positive finite number.
pub(crate) fn ratio(v: f32, scale: f32) -> f32 {
    if scale.is_finite() && scale > 0.0 {
        v / scale
    } else {
        0.0
    }
}

/// `-ratio`, with a zero (the usual one) written as plus zero.
pub(crate) fn neg_ratio(v: f32, scale: f32) -> f32 {
    let r = ratio(v, scale);
    if r == 0.0 { 0.0 } else { -r }
}

pub(crate) fn put_vec3<S: Sink + ?Sized>(w: &mut RecordWriter<'_, S>, v: Vec3<f32>) {
    w.f32(v.x).f32(v.y).f32(v.z);
}

pub(crate) fn put_quat<S: Sink + ?Sized>(w: &mut RecordWriter<'_, S>, q: Quat<f32>) {
    w.f32(q.w).f32(q.x).f32(q.y).f32(q.z);
}
