// SPDX-License-Identifier: AGPL-3.0-or-later
//! The replay level: exact records of the core's boundary (ADR-0005 §5).
//!
//! A replay log is a normal log plus everything the core saw and said. Its
//! records, written in this order by the replay recorder:
//!
//! - once, at the start: [`VEHICLE`] (one record per number of the vehicle
//!   description), [`MICROS`] (the exact value of each parameter the core
//!   holds as whole microseconds, which the `f32` seconds of the `P`
//!   messages cannot always name) and [`BOOT`] (the stored-parameter load report, the rotor
//!   count, the profile's rates);
//! - every tick: [`IN_TICK`] (which inputs are present, the sequence number,
//!   the parameter acknowledgement, whether the store's "saved" call
//!   happened), then one `oxide_in_*` record for each present input, then
//!   [`OUT`] (the motor commands, every status field, the number of events).
//!
//! Exact means bits. Floats are written as their bit patterns, enumerations
//! through [`oxide_core::codes`] (never `as`), an `Option` inside a sample is
//! a presence flag and a value, a `bool` is one byte that is 0 or 1. Every
//! topic's first field is `timestamp`, the tick time. Padding follows PX4's
//! rules: fields aligned to their size, the format a multiple of 8 bytes,
//! trailing padding in the format and never in a record.
//!
//! The encoders (`fill_*`) write into a [`RecordWriter`] and decide nothing
//! about when; `record::data` or the logger turns the record into a message.
//! With the `std` feature, [`decode`] reads the records back into the core's
//! own types.

use crate::msg::{Error, Sink};
use crate::record::{RecordWriter, Topic};
use oxide_core::config::{NumbersError, VehicleConfig};
use oxide_core::io::{
    BaroSample, BatterySample, BrainHeartbeat, GcsHeartbeat, GpsSample, ImuSample, Inputs,
    MagSample, Outputs, ParamSet, RcSample, Stamped, StateSample, TrajectorySetpoint,
};
use oxide_core::math::Vec3;
use oxide_core::params::{self, LoadReport, ParamError, ParamId, ParamValue};
use oxide_core::schedule::Profile;
use oxide_core::time::Micros;

#[cfg(any(test, feature = "std"))]
pub mod decode;

/// Most motor commands one `oxide_out` record carries.
pub const MAX_MOTORS: usize = 8;
/// Bytes of the name in an `oxide_vehicle` record (the longest name has 22).
pub const VEHICLE_NAME_LEN: usize = 24;

/// Bit numbers of `oxide_in_tick.present`: one per input, in `Inputs` order.
pub mod bit {
    /// `oxide_in_imu`.
    pub const IMU: u8 = 0;
    /// `oxide_in_state`.
    pub const STATE: u8 = 1;
    /// `oxide_in_rc`.
    pub const RC: u8 = 2;
    /// `oxide_in_battery`.
    pub const BATTERY: u8 = 3;
    /// `oxide_in_setpoint`.
    pub const SETPOINT: u8 = 4;
    /// `oxide_in_gps`.
    pub const GPS: u8 = 5;
    /// `oxide_in_baro`.
    pub const BARO: u8 = 6;
    /// `oxide_in_mag`.
    pub const MAG: u8 = 7;
    /// `oxide_in_gcs`.
    pub const GCS: u8 = 8;
    /// `oxide_in_brain`.
    pub const BRAIN: u8 = 9;
    /// `oxide_in_param_set`.
    pub const PARAM_SET: u8 = 10;
    /// Every defined bit.
    pub const ALL: u16 = 0x07FF;
}

/// `oxide_in_param_set.kind` of an `f32` value.
pub const KIND_F32: u8 = 0;
/// `oxide_in_param_set.kind` of an `i32` value.
pub const KIND_I32: u8 = 1;

/// The vehicle description: one record per number of
/// [`VehicleConfig::vehicle_numbers`].
pub const VEHICLE: Topic = Topic {
    name: "oxide_vehicle",
    format: "uint64_t timestamp;float value;uint16_t index;char[24] name;uint8_t[2] _padding0;",
    record_len: 38,
};

/// The exact microseconds of one parameter the core holds as whole
/// microseconds ([`params::micro_ids`], in table order), by the parameter's
/// stable key. The log's `P` messages carry them as `f32` seconds, which
/// names the microsecond exactly only up to 16 s; this carries the integer.
pub const MICROS: Topic = Topic {
    name: "oxide_param_us",
    format: "uint64_t timestamp;uint32_t key;uint32_t micros;",
    record_len: 16,
};

/// The stored-parameter load report, the rotor count and the profile's rates.
pub const BOOT: Topic = Topic {
    name: "oxide_boot",
    format: "uint64_t timestamp;uint16_t applied;uint16_t unknown;uint16_t dropped;\
             uint16_t torn;uint32_t base_period_us;uint32_t filter_period_ticks;\
             uint32_t[5] task_period_ticks;bool invalid;bool unreadable;uint8_t rotors;\
             uint8_t[1] _padding0;",
    record_len: 47,
};

/// The start of a tick's group of records.
pub const IN_TICK: Topic = Topic {
    name: "oxide_in_tick",
    format: "uint64_t timestamp;uint64_t seq;uint16_t present;bool param_ack;bool saved;\
             uint8_t[4] _padding0;",
    record_len: 20,
};

/// An IMU sample.
pub const IN_IMU: Topic = Topic {
    name: "oxide_in_imu",
    format: "uint64_t timestamp;uint64_t t_sample;uint64_t t_arrival;float[3] gyro;\
             float[3] accel;",
    record_len: 48,
};

/// A full-state sample.
pub const IN_STATE: Topic = Topic {
    name: "oxide_in_state",
    format: "uint64_t timestamp;uint64_t t_sample;uint64_t t_arrival;float[3] position;\
             float[3] velocity;float[4] attitude;float[3] rate;uint8_t[4] _padding0;",
    record_len: 76,
};

/// An RC sample.
pub const IN_RC: Topic = Topic {
    name: "oxide_in_rc",
    format: "uint64_t timestamp;uint64_t t_sample;uint64_t t_arrival;float roll;float pitch;\
             float yaw;float throttle;bool arm_switch;bool kill_switch;uint8_t mode_switch;\
             bool failsafe;bool frame_lost;uint8_t[3] _padding0;",
    record_len: 45,
};

/// A battery sample.
pub const IN_BATTERY: Topic = Topic {
    name: "oxide_in_battery",
    format: "uint64_t timestamp;uint64_t t_sample;uint64_t t_arrival;float voltage_v;\
             float current_a;",
    record_len: 32,
};

/// A trajectory setpoint.
pub const IN_SETPOINT: Topic = Topic {
    name: "oxide_in_setpoint",
    format: "uint64_t timestamp;uint64_t t_sample;uint64_t t_arrival;float[3] position;\
             float[3] velocity;float yaw;uint8_t[4] _padding0;",
    record_len: 52,
};

/// A GNSS sample.
pub const IN_GPS: Topic = Topic {
    name: "oxide_in_gps",
    format: "uint64_t timestamp;uint64_t t_sample;uint64_t t_arrival;float[3] position;\
             float[3] velocity;float h_acc_m;float v_acc_m;float s_acc_mps;bool fix_3d;\
             uint8_t[3] _padding0;",
    record_len: 61,
};

/// A barometer sample.
pub const IN_BARO: Topic = Topic {
    name: "oxide_in_baro",
    format: "uint64_t timestamp;uint64_t t_sample;uint64_t t_arrival;float altitude_m;\
             uint8_t[4] _padding0;",
    record_len: 28,
};

/// A magnetometer sample.
pub const IN_MAG: Topic = Topic {
    name: "oxide_in_mag",
    format: "uint64_t timestamp;uint64_t t_sample;uint64_t t_arrival;float[3] field_ut;\
             uint8_t[4] _padding0;",
    record_len: 36,
};

/// A ground-station heartbeat.
pub const IN_GCS: Topic = Topic {
    name: "oxide_in_gcs",
    format: "uint64_t timestamp;uint64_t t_sample;uint64_t t_arrival;",
    record_len: 24,
};

/// A brain heartbeat.
pub const IN_BRAIN: Topic = Topic {
    name: "oxide_in_brain",
    format: "uint64_t timestamp;uint64_t t_sample;uint64_t t_arrival;uint32_t progress;\
             uint8_t[4] _padding0;",
    record_len: 28,
};

/// A parameter change request, by the parameter's stable key.
pub const IN_PARAM_SET: Topic = Topic {
    name: "oxide_in_param_set",
    format: "uint64_t timestamp;uint32_t key;uint32_t bits;uint16_t id;uint8_t kind;bool known;\
             uint8_t[4] _padding0;",
    record_len: 20,
};

/// The core's outputs of a tick, minus the events (`oxide_event` has them).
pub const OUT: Topic = Topic {
    name: "oxide_out",
    format: "uint64_t timestamp;uint64_t t_valid;float[8] motors;uint32_t timing_faults;\
             uint32_t rejected;uint32_t resets;uint16_t reasons;uint16_t prearm;\
             uint8_t motor_count;uint8_t events;uint8_t mode;uint8_t nav;uint8_t action;\
             bool armed;bool killed;bool saturated;bool no_estimate;bool aligned;\
             bool gps_aiding;bool valid_attitude;bool valid_heading;bool valid_altitude;\
             bool valid_velocity;bool valid_position;bool home_set;bool params_unsaved;\
             uint8_t[6] _padding0;",
    record_len: 82,
};

/// The replay topics in their fixed order, for the header.
pub const TOPICS: [Topic; 16] = [
    VEHICLE,
    MICROS,
    BOOT,
    IN_TICK,
    IN_IMU,
    IN_STATE,
    IN_RC,
    IN_BATTERY,
    IN_SETPOINT,
    IN_GPS,
    IN_BARO,
    IN_MAG,
    IN_GCS,
    IN_BRAIN,
    IN_PARAM_SET,
    OUT,
];

/// The presence bit of the input topic called `name`; `None` for any other.
pub fn input_bit(name: &str) -> Option<u8> {
    Some(match name {
        "oxide_in_imu" => bit::IMU,
        "oxide_in_state" => bit::STATE,
        "oxide_in_rc" => bit::RC,
        "oxide_in_battery" => bit::BATTERY,
        "oxide_in_setpoint" => bit::SETPOINT,
        "oxide_in_gps" => bit::GPS,
        "oxide_in_baro" => bit::BARO,
        "oxide_in_mag" => bit::MAG,
        "oxide_in_gcs" => bit::GCS,
        "oxide_in_brain" => bit::BRAIN,
        "oxide_in_param_set" => bit::PARAM_SET,
        _ => return None,
    })
}

/// The inputs present, one bit each ([`bit`]).
pub fn present_mask(inputs: &Inputs) -> u16 {
    let flag = |present: bool, b: u8| u16::from(present) << b;
    flag(inputs.imu.is_some(), bit::IMU)
        | flag(inputs.state.is_some(), bit::STATE)
        | flag(inputs.rc.is_some(), bit::RC)
        | flag(inputs.battery.is_some(), bit::BATTERY)
        | flag(inputs.setpoint.is_some(), bit::SETPOINT)
        | flag(inputs.gps.is_some(), bit::GPS)
        | flag(inputs.baro.is_some(), bit::BARO)
        | flag(inputs.mag.is_some(), bit::MAG)
        | flag(inputs.gcs_heartbeat.is_some(), bit::GCS)
        | flag(inputs.brain_heartbeat.is_some(), bit::BRAIN)
        | flag(inputs.param_set.is_some(), bit::PARAM_SET)
}

/// More rotors than a record can carry.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TooManyRotors {
    /// The rotor count asked for.
    pub got: usize,
    /// The most a record carries.
    pub max: usize,
}

// ---- encoders ----

fn vec3<S: Sink + ?Sized>(w: &mut RecordWriter<'_, S>, v: &Vec3<f32>) {
    w.f32_array(&[v.x, v.y, v.z]);
}

/// The three time stamps every stamped input starts with.
fn stamps<S: Sink + ?Sized, T>(w: &mut RecordWriter<'_, S>, t: Micros, s: &Stamped<T>) {
    w.u64(t.0).u64(s.t_sample.0).u64(s.t_arrival.0);
}

/// One number of the vehicle description. A `name` longer than
/// [`VEHICLE_NAME_LEN`] writes nothing (the record then fails its length
/// check) rather than being cut.
pub fn fill_vehicle_number<S: Sink + ?Sized>(
    w: &mut RecordWriter<'_, S>,
    t: Micros,
    index: u16,
    name: &str,
    value: f32,
) {
    if name.len() > VEHICLE_NAME_LEN {
        return;
    }
    w.u64(t.0)
        .f32(value)
        .u16(index)
        .char_array(VEHICLE_NAME_LEN, name);
}

/// Writes the whole vehicle description, one message per number, straight
/// into `sink` (the header's sink, not the ring: the logger writes each
/// number with its own call). `msg_id` is the topic's subscription id.
pub fn write_vehicle<const N: usize, S: Sink + ?Sized>(
    sink: &mut S,
    msg_id: u16,
    t: Micros,
    cfg: &VehicleConfig<N>,
) -> Result<(), Error> {
    let mut result = Ok(());
    let mut index: u16 = 0;
    cfg.vehicle_numbers(|name, value| {
        if result.is_ok() {
            result = crate::record::data(sink, msg_id, &VEHICLE, |w| {
                fill_vehicle_number(w, t, index, name, value);
            });
            index = index.saturating_add(1);
        }
    });
    result
}

/// One `oxide_param_us` record: the parameter's stable key and its exact
/// microseconds. Writes nothing for a name that is not a parameter held as
/// microseconds ([`params::is_micros`]).
pub fn fill_micros<S: Sink + ?Sized>(
    w: &mut RecordWriter<'_, S>,
    t: Micros,
    name: &str,
    micros: u32,
) {
    if let Some(id) = params::find(name).filter(|id| params::is_micros(*id))
        && let Some(key) = params::key_of(id)
    {
        w.u64(t.0).u32(key).u32(micros);
    }
}

/// Writes the exact microseconds of every parameter of [`params::micro_ids`],
/// straight into `sink` like [`write_vehicle`]. `msg_id` is the topic's
/// subscription id.
pub fn write_micros<const N: usize, S: Sink + ?Sized>(
    sink: &mut S,
    msg_id: u16,
    t: Micros,
    cfg: &VehicleConfig<N>,
) -> Result<(), Error> {
    for id in params::micro_ids() {
        let name = params::PARAMS.get(id.index()).map_or("", |d| d.name);
        let us = params::micros(cfg, id).unwrap_or(0);
        crate::record::data(sink, msg_id, &MICROS, |w| fill_micros(w, t, name, us))?;
    }
    Ok(())
}

/// The boot record: the report of the stored-parameter load, the number of
/// rotors and the profile's rates. Writes nothing for a rotor count over 255.
pub fn fill_boot<S: Sink + ?Sized>(
    w: &mut RecordWriter<'_, S>,
    t: Micros,
    report: &LoadReport,
    rotors: usize,
    profile: &Profile,
) -> Result<(), TooManyRotors> {
    let rotors = u8::try_from(rotors).map_err(|_| TooManyRotors {
        got: rotors,
        max: usize::from(u8::MAX),
    })?;
    w.u64(t.0)
        .u16(report.applied)
        .u16(report.unknown)
        .u16(report.dropped)
        .u16(report.torn)
        .u32(profile.base_period_us)
        .u32(profile.filter_period_ticks);
    for task in &profile.tasks {
        w.u32(task.period_ticks);
    }
    w.bool(report.invalid).bool(report.unreadable).u8(rotors);
    Ok(())
}

/// The tick record: `seq` is the tick's number from 0, `saved` is "the
/// store's saved call happened since the previous tick".
pub fn fill_tick<S: Sink + ?Sized>(
    w: &mut RecordWriter<'_, S>,
    t: Micros,
    seq: u64,
    inputs: &Inputs,
    saved: bool,
) {
    w.u64(t.0)
        .u64(seq)
        .u16(present_mask(inputs))
        .bool(inputs.param_ack)
        .bool(saved);
}

/// An IMU sample.
pub fn fill_imu<S: Sink + ?Sized>(w: &mut RecordWriter<'_, S>, t: Micros, s: &Stamped<ImuSample>) {
    stamps(w, t, s);
    vec3(w, &s.value.gyro);
    vec3(w, &s.value.accel);
}

/// A full-state sample; the attitude is `w, x, y, z`.
pub fn fill_state<S: Sink + ?Sized>(
    w: &mut RecordWriter<'_, S>,
    t: Micros,
    s: &Stamped<StateSample>,
) {
    stamps(w, t, s);
    vec3(w, &s.value.position);
    vec3(w, &s.value.velocity);
    let q = &s.value.attitude;
    w.f32_array(&[q.w, q.x, q.y, q.z]);
    vec3(w, &s.value.rate);
}

/// An RC sample.
pub fn fill_rc<S: Sink + ?Sized>(w: &mut RecordWriter<'_, S>, t: Micros, s: &Stamped<RcSample>) {
    stamps(w, t, s);
    let v = &s.value;
    w.f32(v.roll)
        .f32(v.pitch)
        .f32(v.yaw)
        .f32(v.throttle)
        .bool(v.arm_switch)
        .bool(v.kill_switch)
        .u8(v.mode_switch)
        .bool(v.failsafe)
        .bool(v.frame_lost);
}

/// A battery sample.
pub fn fill_battery<S: Sink + ?Sized>(
    w: &mut RecordWriter<'_, S>,
    t: Micros,
    s: &Stamped<BatterySample>,
) {
    stamps(w, t, s);
    w.f32(s.value.voltage_v).f32(s.value.current_a);
}

/// A trajectory setpoint.
pub fn fill_setpoint<S: Sink + ?Sized>(
    w: &mut RecordWriter<'_, S>,
    t: Micros,
    s: &Stamped<TrajectorySetpoint>,
) {
    stamps(w, t, s);
    vec3(w, &s.value.position);
    vec3(w, &s.value.velocity);
    w.f32(s.value.yaw);
}

/// A GNSS sample.
pub fn fill_gps<S: Sink + ?Sized>(w: &mut RecordWriter<'_, S>, t: Micros, s: &Stamped<GpsSample>) {
    stamps(w, t, s);
    vec3(w, &s.value.position);
    vec3(w, &s.value.velocity);
    w.f32(s.value.h_acc_m)
        .f32(s.value.v_acc_m)
        .f32(s.value.s_acc_mps)
        .bool(s.value.fix_3d);
}

/// A barometer sample.
pub fn fill_baro<S: Sink + ?Sized>(
    w: &mut RecordWriter<'_, S>,
    t: Micros,
    s: &Stamped<BaroSample>,
) {
    stamps(w, t, s);
    w.f32(s.value.altitude_m);
}

/// A magnetometer sample.
pub fn fill_mag<S: Sink + ?Sized>(w: &mut RecordWriter<'_, S>, t: Micros, s: &Stamped<MagSample>) {
    stamps(w, t, s);
    vec3(w, &s.value.field_ut);
}

/// A ground-station heartbeat.
pub fn fill_gcs<S: Sink + ?Sized>(
    w: &mut RecordWriter<'_, S>,
    t: Micros,
    s: &Stamped<GcsHeartbeat>,
) {
    stamps(w, t, s);
}

/// A brain heartbeat.
pub fn fill_brain<S: Sink + ?Sized>(
    w: &mut RecordWriter<'_, S>,
    t: Micros,
    s: &Stamped<BrainHeartbeat>,
) {
    stamps(w, t, s);
    w.u32(s.value.progress);
}

/// A parameter change request. The parameter is named by its stable key
/// (`params::key_of`). An id outside the table has no key: it is written as
/// `known = false` with the raw id, so that the core's refusal of it can be
/// replayed.
pub fn fill_param_set<S: Sink + ?Sized>(w: &mut RecordWriter<'_, S>, t: Micros, p: &ParamSet) {
    let (key, known) = match params::key_of(p.id) {
        Some(k) => (k, true),
        None => (0, false),
    };
    let (kind, bits) = match p.value {
        ParamValue::F32(v) => (KIND_F32, v.to_bits()),
        ParamValue::I32(v) => (KIND_I32, v as u32),
    };
    w.u64(t.0)
        .u32(key)
        .u32(bits)
        .u16(p.id.0)
        .u8(kind)
        .bool(known);
}

/// The outputs of a tick: `t_valid`, the motor commands (unused entries of
/// the eight are zero), every `Status` field, and the number of events.
/// Writes nothing if the vehicle has more than [`MAX_MOTORS`] rotors.
pub fn fill_out<const N: usize, S: Sink + ?Sized>(
    w: &mut RecordWriter<'_, S>,
    t: Micros,
    o: &Outputs<N>,
) -> Result<(), TooManyRotors> {
    if N > MAX_MOTORS {
        return Err(TooManyRotors {
            got: N,
            max: MAX_MOTORS,
        });
    }
    let s = &o.status;
    let count = u8::try_from(N).unwrap_or(u8::MAX);
    let events = u8::try_from(o.events.len()).unwrap_or(u8::MAX);
    w.u64(t.0).u64(o.t_valid.0);
    for m in o
        .motors
        .iter()
        .chain(core::iter::repeat(&0.0_f32))
        .take(MAX_MOTORS)
    {
        w.f32(*m);
    }
    w.u32(s.timing_faults)
        .u32(s.estimator.rejected)
        .u32(s.estimator.resets)
        .u16(s.reasons.0)
        .u16(s.prearm.0)
        .u8(count)
        .u8(events)
        .u8(s.mode.code())
        .u8(s.nav.code())
        .u8(s.action.code())
        .bool(s.armed)
        .bool(s.killed)
        .bool(s.saturated)
        .bool(s.no_estimate)
        .bool(s.estimator.aligned)
        .bool(s.estimator.gps_aiding)
        .bool(s.valid.attitude)
        .bool(s.valid.heading)
        .bool(s.valid.altitude)
        .bool(s.valid.velocity)
        .bool(s.valid.position)
        .bool(s.home_set)
        .bool(s.params_unsaved);
    Ok(())
}

// ---- rebuilding the configuration ----

/// Why a configuration could not be rebuilt from a log, or does not survive
/// being logged.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RebuildError {
    /// The vehicle numbers are not this vehicle's.
    Numbers(NumbersError),
    /// The parameter at this position in the list has a name this build does
    /// not have; `key` is its stable key ([`params::key`] of the name).
    UnknownParam {
        /// Position in the list given.
        position: usize,
        /// The key of the name.
        key: u32,
    },
    /// A parameter appears twice.
    DuplicateParam(ParamId),
    /// A parameter of this build is not in the list.
    MissingParam(ParamId),
    /// The core refused this parameter's value on its own (type, finite,
    /// range).
    Refused {
        /// The parameter.
        id: ParamId,
        /// Why.
        why: ParamError,
    },
    /// Every value is fine alone but together they break a rule.
    Invalid(ParamError),
    /// Setting the parameter to the value it reads back as does not give the
    /// same configuration: it is held as whole microseconds and its seconds
    /// as `f32` do not name that microsecond.
    NotExact(ParamId),
}

fn same_param_bits(a: ParamValue, b: ParamValue) -> bool {
    match (a, b) {
        (ParamValue::F32(x), ParamValue::F32(y)) => x.to_bits() == y.to_bits(),
        (ParamValue::I32(x), ParamValue::I32(y)) => x == y,
        _ => false,
    }
}

/// Rebuilds a configuration from the vehicle numbers (any order; names as in
/// `VehicleConfig::vehicle_numbers`), **all** the parameters by name (any
/// order), and the exact microseconds of every parameter of [`params::micro_ids`]
/// as `(key, microseconds)` (any order), checked against `profile` as
/// `params::set_all` does. Every parameter of this build must appear exactly
/// once. Each must read back from the result with the bits it was given, and
/// each microsecond parameter must be the exact value logged for it, else
/// [`RebuildError::NotExact`] names it: the result is the configuration that
/// was logged or an error, never another configuration with the same hash.
pub fn rebuild_config<'a, const N: usize>(
    numbers: impl IntoIterator<Item = (&'a str, f32)>,
    parameters: impl IntoIterator<Item = (&'a str, ParamValue)>,
    micros: impl IntoIterator<Item = (u32, u32)>,
    profile: &Profile,
) -> Result<VehicleConfig<N>, RebuildError> {
    let mut cfg =
        VehicleConfig::<N>::from_vehicle_numbers(numbers).map_err(RebuildError::Numbers)?;
    let mut given: [Option<ParamValue>; params::COUNT] = [None; params::COUNT];
    for (position, (name, value)) in parameters.into_iter().enumerate() {
        let id = params::find(name).ok_or(RebuildError::UnknownParam {
            position,
            key: params::key(name),
        })?;
        let slot = given
            .get_mut(id.index())
            .ok_or(RebuildError::UnknownParam {
                position,
                key: params::key(name),
            })?;
        if slot.is_some() {
            return Err(RebuildError::DuplicateParam(id));
        }
        *slot = Some(value);
    }
    let mut exact: [Option<u32>; params::COUNT] = [None; params::COUNT];
    for (position, (key, us)) in micros.into_iter().enumerate() {
        let unknown = RebuildError::UnknownParam { position, key };
        let id = params::find_key(key).ok_or(unknown)?;
        if !params::is_micros(id) {
            return Err(unknown);
        }
        let slot = exact.get_mut(id.index()).ok_or(unknown)?;
        if slot.is_some() {
            return Err(RebuildError::DuplicateParam(id));
        }
        *slot = Some(us);
    }
    for id in params::micro_ids() {
        if exact.get(id.index()).is_none_or(Option::is_none) {
            return Err(RebuildError::MissingParam(id));
        }
    }
    for (i, slot) in given.iter().enumerate() {
        if slot.is_none() {
            return Err(RebuildError::MissingParam(ParamId(
                u16::try_from(i).unwrap_or(u16::MAX),
            )));
        }
    }
    let changes = given
        .iter()
        .enumerate()
        .filter_map(|(i, v)| Some((ParamId(u16::try_from(i).ok()?), (*v)?)));
    params::set_all(&mut cfg, profile, changes).map_err(|(id, why)| match id {
        Some(id) => RebuildError::Refused { id, why },
        None => RebuildError::Invalid(why),
    })?;
    for (i, slot) in given.iter().enumerate() {
        let id = ParamId(u16::try_from(i).unwrap_or(u16::MAX));
        match (slot, params::get(&cfg, id)) {
            (Some(want), Some(got)) if same_param_bits(*want, got) => {}
            _ => return Err(RebuildError::NotExact(id)),
        }
    }
    // The seconds name one microsecond; it must be the one that was logged.
    for id in params::micro_ids() {
        let us = exact.get(id.index()).copied().flatten();
        let secs = match given.get(id.index()) {
            Some(Some(ParamValue::F32(v))) => params::secs_to_micros(*v),
            _ => None,
        };
        if secs.is_none() || secs != us || params::micros(&cfg, id) != us {
            return Err(RebuildError::NotExact(id));
        }
    }
    Ok(cfg)
}

/// Whether parameter `id` of `cfg` survives being read as `f32` seconds and
/// set again. The log carries the exact microseconds too ([`MICROS`]), so a
/// parameter that does not survive is not replayable: the rebuild names it
/// ([`RebuildError::NotExact`]). A recorder may call this (or
/// [`check_exact`]) to refuse such a configuration up front. Setting it on a
/// copy must give `cfg` back. A parameter held as whole microseconds reads as `f32` seconds, which
/// names the microsecond exactly for every value up to 16 s (16,000,000 µs)
/// and for round values above, but not for every value above (the first
/// that fails is 16,000,001 µs).
pub fn exact_param<const N: usize>(
    cfg: &VehicleConfig<N>,
    profile: &Profile,
    id: ParamId,
) -> Result<(), RebuildError> {
    let value = params::get(cfg, id).ok_or(RebuildError::Refused {
        id,
        why: ParamError::Unknown,
    })?;
    let mut copy = *cfg;
    params::set(&mut copy, profile, id, value).map_err(|why| RebuildError::Refused { id, why })?;
    if copy == *cfg {
        Ok(())
    } else {
        Err(RebuildError::NotExact(id))
    }
}

/// [`exact_param`] for every parameter: the first that does not survive, or
/// that the core refuses to set to its own value (an invalid configuration).
pub fn check_exact<const N: usize>(
    cfg: &VehicleConfig<N>,
    profile: &Profile,
) -> Result<(), RebuildError> {
    for i in 0..params::COUNT {
        let id = ParamId(u16::try_from(i).unwrap_or(u16::MAX));
        exact_param(cfg, profile, id)?;
    }
    Ok(())
}
