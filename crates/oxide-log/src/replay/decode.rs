// SPDX-License-Identifier: AGPL-3.0-or-later
//! Reading the replay records back into the core's own types (`std` only).
//!
//! Every decoder takes a [`RecordView`] and returns the core's type, or a
//! [`DecodeError`] that names the topic and the field: an unknown code, a flag
//! byte that is not 0 or 1, bits no flag has, a motor count above 8 or not
//! `N`. The decoders do not look at the timestamp (the tick time, first field
//! of every topic); [`record_time`] reads it, and [`inputs_from_records`]
//! checks it.

use super::{
    KIND_F32, KIND_I32, MAX_MOTORS, RebuildError, TOPICS, VEHICLE_NAME_LEN, bit, input_bit,
    rebuild_config,
};
use crate::msg::ParamValue as MsgParam;
use crate::reader::{RecordView, UlogFile};
use crate::record::Topic;
use core::fmt;
use oxide_core::config::VehicleConfig;
use oxide_core::io::{
    Action, BaroSample, BatterySample, BrainHeartbeat, EVENTS_PER_TICK, EstimatorStatus, EventList,
    FlightMode, GcsHeartbeat, GpsSample, ImuSample, Inputs, MagSample, NavState, Outputs, ParamSet,
    PrearmFlags, RcSample, Reasons, Stamped, StateSample, Status, TrajectorySetpoint, Validity,
};
use oxide_core::math::{Quat, Vec3};
use oxide_core::params::{self, LoadReport, ParamId, ParamValue};
use oxide_core::schedule::Profile;
use oxide_core::time::Micros;
use std::string::String;
use std::vec::Vec;

/// What is wrong with a field.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DecodeErrorKind {
    /// The record is of another topic.
    WrongTopic,
    /// The record has no such field (or not of the declared type, or it lies
    /// in the trailing padding).
    Missing,
    /// A `bool` byte that is neither 0 nor 1.
    BadFlag(u8),
    /// A code that is not one of `oxide_core::codes`' (or a kind code).
    UnknownCode(u32),
    /// Bits set that no flag of the set has (the bits that are not defined).
    UnknownBits(u32),
    /// A motor count above [`MAX_MOTORS`].
    TooManyMotors(u8),
    /// A motor count that is not the `N` being decoded into.
    MotorCount {
        /// In the record.
        recorded: u8,
        /// Asked for.
        expected: usize,
    },
    /// An event count above what one tick reports.
    TooManyEvents(u8),
    /// A parameter key this build has no parameter for.
    UnknownKey(u32),
    /// A parameter marked unknown whose id is in this build's table.
    UnknownId(u16),
    /// A name that is not UTF-8, or has text after its terminator.
    BadName,
}

/// A decoding failure: where, and what.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DecodeError {
    /// The topic.
    pub topic: &'static str,
    /// The field (empty for a wrong topic).
    pub field: &'static str,
    /// What is wrong.
    pub kind: DecodeErrorKind,
}

impl fmt::Display for DecodeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.topic)?;
        if !self.field.is_empty() {
            write!(f, ".{}", self.field)?;
        }
        match self.kind {
            DecodeErrorKind::WrongTopic => write!(f, ": the record is of another topic"),
            DecodeErrorKind::Missing => write!(f, ": no such field in the record"),
            DecodeErrorKind::BadFlag(b) => write!(f, ": flag byte {b} is not 0 or 1"),
            DecodeErrorKind::UnknownCode(c) => write!(f, ": unknown code {c}"),
            DecodeErrorKind::UnknownBits(b) => write!(f, ": undefined bits {b:#x}"),
            DecodeErrorKind::TooManyMotors(n) => write!(f, ": {n} motors, at most {MAX_MOTORS}"),
            DecodeErrorKind::MotorCount { recorded, expected } => {
                write!(
                    f,
                    ": {recorded} motors recorded, the vehicle has {expected}"
                )
            }
            DecodeErrorKind::TooManyEvents(n) => {
                write!(f, ": {n} events, at most {EVENTS_PER_TICK} per tick")
            }
            DecodeErrorKind::UnknownKey(k) => write!(f, ": no parameter has key {k:#010x}"),
            DecodeErrorKind::UnknownId(i) => {
                write!(f, ": id {i} is marked outside the table but is in it")
            }
            DecodeErrorKind::BadName => write!(f, ": the name is damaged"),
        }
    }
}

impl std::error::Error for DecodeError {}

type Res<T> = Result<T, DecodeError>;

/// Field access with the topic's name in every error.
struct Rd<'r, 'a> {
    r: &'r RecordView<'a>,
    topic: &'static str,
}

impl<'r, 'a> Rd<'r, 'a> {
    fn new(r: &'r RecordView<'a>, topic: &Topic) -> Res<Self> {
        if r.topic() == topic.name {
            Ok(Self {
                r,
                topic: topic.name,
            })
        } else {
            Err(DecodeError {
                topic: topic.name,
                field: "",
                kind: DecodeErrorKind::WrongTopic,
            })
        }
    }

    fn err(&self, field: &'static str, kind: DecodeErrorKind) -> DecodeError {
        DecodeError {
            topic: self.topic,
            field,
            kind,
        }
    }

    fn get<T>(&self, field: &'static str, v: Option<T>) -> Res<T> {
        v.ok_or_else(|| self.err(field, DecodeErrorKind::Missing))
    }

    fn u64(&self, f: &'static str) -> Res<u64> {
        self.get(f, self.r.u64(f))
    }
    fn u32(&self, f: &'static str) -> Res<u32> {
        self.get(f, self.r.u32(f))
    }
    fn u16(&self, f: &'static str) -> Res<u16> {
        self.get(f, self.r.u16(f))
    }
    fn u8(&self, f: &'static str) -> Res<u8> {
        self.get(f, self.r.u8(f))
    }
    fn f32(&self, f: &'static str) -> Res<f32> {
        self.get(f, self.r.f32(f))
    }
    fn micros(&self, f: &'static str) -> Res<Micros> {
        self.u64(f).map(Micros)
    }

    /// A `bool`: the byte must be 0 or 1 (the reader's own `bool` would read
    /// any other value as true).
    fn flag(&self, f: &'static str) -> Res<bool> {
        let b = self.get(f, self.r.field_bytes(f).and_then(|b| b.first().copied()))?;
        match b {
            0 => Ok(false),
            1 => Ok(true),
            other => Err(self.err(f, DecodeErrorKind::BadFlag(other))),
        }
    }

    fn f32s<const K: usize>(&self, f: &'static str) -> Res<[f32; K]> {
        let bytes = self.get(f, self.r.field_bytes(f))?;
        if bytes.len() != 4 * K {
            return Err(self.err(f, DecodeErrorKind::Missing));
        }
        let mut out = [0.0_f32; K];
        for (o, c) in out.iter_mut().zip(bytes.chunks_exact(4)) {
            let mut b = [0u8; 4];
            b.copy_from_slice(c);
            *o = f32::from_bits(u32::from_le_bytes(b));
        }
        Ok(out)
    }

    fn vec3(&self, f: &'static str) -> Res<Vec3<f32>> {
        let [x, y, z] = self.f32s::<3>(f)?;
        Ok(Vec3 { x, y, z })
    }

    /// A code decoded by `from_code`.
    fn code<T>(&self, f: &'static str, from: impl Fn(u8) -> Option<T>) -> Res<T> {
        let c = self.u8(f)?;
        from(c).ok_or_else(|| self.err(f, DecodeErrorKind::UnknownCode(u32::from(c))))
    }

    /// A 16-bit set that may only have the bits of `all`.
    fn bits(&self, f: &'static str, all: u16) -> Res<u16> {
        let v = self.u16(f)?;
        if v & !all != 0 {
            return Err(self.err(f, DecodeErrorKind::UnknownBits(u32::from(v & !all))));
        }
        Ok(v)
    }

    fn stamped<T>(&self, value: impl FnOnce(&Self) -> Res<T>) -> Res<Stamped<T>> {
        Ok(Stamped {
            t_sample: self.micros("t_sample")?,
            t_arrival: self.micros("t_arrival")?,
            value: value(self)?,
        })
    }
}

/// The `timestamp` of any record: the tick time.
pub fn record_time(r: &RecordView<'_>) -> Result<Micros, DecodeError> {
    let name = TOPICS
        .iter()
        .find(|t| t.name == r.topic())
        .map_or("", |t| t.name);
    r.u64("timestamp").map(Micros).ok_or(DecodeError {
        topic: name,
        field: "timestamp",
        kind: DecodeErrorKind::Missing,
    })
}

// ---- the records ----

/// One number of the vehicle description.
#[derive(Debug, Clone, PartialEq)]
pub struct VehicleNumber {
    /// The record's tick time.
    pub t: Micros,
    /// Position in `vehicle_numbers`' order.
    pub index: u16,
    /// Its name.
    pub name: String,
    /// Its exact value.
    pub value: f32,
}

/// `oxide_vehicle`.
pub fn decode_vehicle(r: &RecordView<'_>) -> Res<VehicleNumber> {
    let d = Rd::new(r, &super::VEHICLE)?;
    let raw = d.get("name", r.field_bytes("name"))?;
    let end = raw.iter().position(|b| *b == 0).unwrap_or(raw.len());
    let (text, rest) = raw.split_at(end.min(VEHICLE_NAME_LEN));
    if rest.iter().any(|b| *b != 0) {
        return Err(d.err("name", DecodeErrorKind::BadName));
    }
    let name = core::str::from_utf8(text)
        .map_err(|_| d.err("name", DecodeErrorKind::BadName))?
        .into();
    Ok(VehicleNumber {
        t: d.micros("timestamp")?,
        index: d.u16("index")?,
        name,
        value: d.f32("value")?,
    })
}

/// `oxide_boot`: what the core was booted with.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Boot {
    /// The record's tick time.
    pub t: Micros,
    /// The report given to `Core::report_param_load`.
    pub report: LoadReport,
    /// The vehicle's rotor count.
    pub rotors: u8,
    /// The profile's base period, µs.
    pub base_period_us: u32,
    /// The estimator filter's period, in base ticks.
    pub filter_period_ticks: u32,
    /// The profile's task periods in base ticks, in the profile's table order
    /// (estimator, supervisor, guidance, attitude, rate).
    pub task_period_ticks: [u32; 5],
}

/// `oxide_boot`.
pub fn decode_boot(r: &RecordView<'_>) -> Res<Boot> {
    let d = Rd::new(r, &super::BOOT)?;
    let t = d.micros("timestamp")?;
    let tasks = d.get("task_period_ticks", r.field_bytes("task_period_ticks"))?;
    let mut task_period_ticks = [0u32; 5];
    if tasks.len() != 20 {
        return Err(d.err("task_period_ticks", DecodeErrorKind::Missing));
    }
    for (o, c) in task_period_ticks.iter_mut().zip(tasks.chunks_exact(4)) {
        let mut b = [0u8; 4];
        b.copy_from_slice(c);
        *o = u32::from_le_bytes(b);
    }
    Ok(Boot {
        t,
        report: LoadReport {
            applied: d.u16("applied")?,
            unknown: d.u16("unknown")?,
            dropped: d.u16("dropped")?,
            invalid: d.flag("invalid")?,
            unreadable: d.flag("unreadable")?,
            torn: d.u16("torn")?,
        },
        rotors: d.u8("rotors")?,
        base_period_us: d.u32("base_period_us")?,
        filter_period_ticks: d.u32("filter_period_ticks")?,
        task_period_ticks,
    })
}

/// `oxide_in_tick`: the start of a tick's records.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TickRecord {
    /// The tick time.
    pub t: Micros,
    /// The tick's number, from 0.
    pub seq: u64,
    /// The inputs present, one bit each (`bit`).
    pub present: u16,
    /// `Inputs::param_ack`.
    pub param_ack: bool,
    /// The store's saved call happened since the previous tick.
    pub saved: bool,
}

impl TickRecord {
    /// Whether the input with presence bit `b` is present.
    pub fn has(&self, b: u8) -> bool {
        self.present & (1u16 << b) != 0
    }
}

/// `oxide_in_tick`.
pub fn decode_tick(r: &RecordView<'_>) -> Res<TickRecord> {
    let d = Rd::new(r, &super::IN_TICK)?;
    Ok(TickRecord {
        t: d.micros("timestamp")?,
        seq: d.u64("seq")?,
        present: d.bits("present", bit::ALL)?,
        param_ack: d.flag("param_ack")?,
        saved: d.flag("saved")?,
    })
}

/// `oxide_in_imu`.
pub fn decode_imu(r: &RecordView<'_>) -> Res<Stamped<ImuSample>> {
    let d = Rd::new(r, &super::IN_IMU)?;
    d.stamped(|d| {
        Ok(ImuSample {
            gyro: d.vec3("gyro")?,
            accel: d.vec3("accel")?,
        })
    })
}

/// `oxide_in_state`.
pub fn decode_state(r: &RecordView<'_>) -> Res<Stamped<StateSample>> {
    let d = Rd::new(r, &super::IN_STATE)?;
    d.stamped(|d| {
        let [w, x, y, z] = d.f32s::<4>("attitude")?;
        Ok(StateSample {
            position: d.vec3("position")?,
            velocity: d.vec3("velocity")?,
            attitude: Quat { w, x, y, z },
            rate: d.vec3("rate")?,
        })
    })
}

/// `oxide_in_rc`.
pub fn decode_rc(r: &RecordView<'_>) -> Res<Stamped<RcSample>> {
    let d = Rd::new(r, &super::IN_RC)?;
    d.stamped(|d| {
        Ok(RcSample {
            roll: d.f32("roll")?,
            pitch: d.f32("pitch")?,
            yaw: d.f32("yaw")?,
            throttle: d.f32("throttle")?,
            arm_switch: d.flag("arm_switch")?,
            kill_switch: d.flag("kill_switch")?,
            mode_switch: d.u8("mode_switch")?,
            failsafe: d.flag("failsafe")?,
            frame_lost: d.flag("frame_lost")?,
        })
    })
}

/// `oxide_in_battery`.
pub fn decode_battery(r: &RecordView<'_>) -> Res<Stamped<BatterySample>> {
    let d = Rd::new(r, &super::IN_BATTERY)?;
    d.stamped(|d| {
        Ok(BatterySample {
            voltage_v: d.f32("voltage_v")?,
            current_a: d.f32("current_a")?,
        })
    })
}

/// `oxide_in_setpoint`.
pub fn decode_setpoint(r: &RecordView<'_>) -> Res<Stamped<TrajectorySetpoint>> {
    let d = Rd::new(r, &super::IN_SETPOINT)?;
    d.stamped(|d| {
        Ok(TrajectorySetpoint {
            position: d.vec3("position")?,
            velocity: d.vec3("velocity")?,
            yaw: d.f32("yaw")?,
        })
    })
}

/// `oxide_in_gps`.
pub fn decode_gps(r: &RecordView<'_>) -> Res<Stamped<GpsSample>> {
    let d = Rd::new(r, &super::IN_GPS)?;
    d.stamped(|d| {
        Ok(GpsSample {
            position: d.vec3("position")?,
            velocity: d.vec3("velocity")?,
            h_acc_m: d.f32("h_acc_m")?,
            v_acc_m: d.f32("v_acc_m")?,
            s_acc_mps: d.f32("s_acc_mps")?,
            fix_3d: d.flag("fix_3d")?,
        })
    })
}

/// `oxide_in_baro`.
pub fn decode_baro(r: &RecordView<'_>) -> Res<Stamped<BaroSample>> {
    let d = Rd::new(r, &super::IN_BARO)?;
    d.stamped(|d| {
        Ok(BaroSample {
            altitude_m: d.f32("altitude_m")?,
        })
    })
}

/// `oxide_in_mag`.
pub fn decode_mag(r: &RecordView<'_>) -> Res<Stamped<MagSample>> {
    let d = Rd::new(r, &super::IN_MAG)?;
    d.stamped(|d| {
        Ok(MagSample {
            field_ut: d.vec3("field_ut")?,
        })
    })
}

/// `oxide_in_gcs`.
pub fn decode_gcs(r: &RecordView<'_>) -> Res<Stamped<GcsHeartbeat>> {
    let d = Rd::new(r, &super::IN_GCS)?;
    d.stamped(|_| Ok(GcsHeartbeat))
}

/// `oxide_in_brain`.
pub fn decode_brain(r: &RecordView<'_>) -> Res<Stamped<BrainHeartbeat>> {
    let d = Rd::new(r, &super::IN_BRAIN)?;
    d.stamped(|d| {
        Ok(BrainHeartbeat {
            progress: d.u32("progress")?,
        })
    })
}

/// `oxide_in_param_set`. A parameter is found by its key; one marked unknown
/// keeps the raw id it had (the core refuses it as `ParamError::Unknown`).
/// The value's kind is the record's own and need not be the parameter's.
pub fn decode_param_set(r: &RecordView<'_>) -> Res<ParamSet> {
    let d = Rd::new(r, &super::IN_PARAM_SET)?;
    let key = d.u32("key")?;
    let bits = d.u32("bits")?;
    let raw_id = d.u16("id")?;
    let kind = d.u8("kind")?;
    let known = d.flag("known")?;
    let id = if known {
        params::find_key(key).ok_or_else(|| d.err("key", DecodeErrorKind::UnknownKey(key)))?
    } else if usize::from(raw_id) < params::COUNT {
        return Err(d.err("id", DecodeErrorKind::UnknownId(raw_id)));
    } else {
        ParamId(raw_id)
    };
    let value = match kind {
        KIND_F32 => ParamValue::F32(f32::from_bits(bits)),
        KIND_I32 => ParamValue::I32(bits as i32),
        other => return Err(d.err("kind", DecodeErrorKind::UnknownCode(u32::from(other)))),
    };
    Ok(ParamSet { id, value })
}

/// `oxide_out`, without the events (the `oxide_event` records have them).
#[derive(Debug, Clone, PartialEq)]
pub struct OutRecord<const N: usize> {
    /// The record's tick time.
    pub t: Micros,
    /// `Outputs::t_valid`.
    pub t_valid: Micros,
    /// The motor commands.
    pub motors: [f32; N],
    /// Every status field.
    pub status: Status,
    /// How many events the tick reported.
    pub events: u8,
}

impl<const N: usize> OutRecord<N> {
    /// The core's `Outputs`, with the events read from the `oxide_event`
    /// records of the tick.
    pub fn into_outputs(self, events: EventList) -> Outputs<N> {
        Outputs {
            t_valid: self.t_valid,
            motors: self.motors,
            status: self.status,
            events,
        }
    }
}

/// `oxide_out` for a vehicle with `N` motors (the record's count must be `N`,
/// and at most 8).
pub fn decode_out<const N: usize>(r: &RecordView<'_>) -> Res<OutRecord<N>> {
    let d = Rd::new(r, &super::OUT)?;
    let t = d.micros("timestamp")?;
    let t_valid = d.micros("t_valid")?;
    let count = d.u8("motor_count")?;
    if usize::from(count) > MAX_MOTORS {
        return Err(d.err("motor_count", DecodeErrorKind::TooManyMotors(count)));
    }
    if usize::from(count) != N {
        return Err(d.err(
            "motor_count",
            DecodeErrorKind::MotorCount {
                recorded: count,
                expected: N,
            },
        ));
    }
    let all = d.f32s::<MAX_MOTORS>("motors")?;
    let mut motors = [0.0_f32; N];
    for (m, v) in motors.iter_mut().zip(all) {
        *m = v;
    }
    let events = d.u8("events")?;
    if usize::from(events) > EVENTS_PER_TICK {
        return Err(d.err("events", DecodeErrorKind::TooManyEvents(events)));
    }
    let status = Status {
        armed: d.flag("armed")?,
        killed: d.flag("killed")?,
        mode: d.code("mode", FlightMode::from_code)?,
        saturated: d.flag("saturated")?,
        timing_faults: d.u32("timing_faults")?,
        no_estimate: d.flag("no_estimate")?,
        estimator: EstimatorStatus {
            aligned: d.flag("aligned")?,
            gps_aiding: d.flag("gps_aiding")?,
            rejected: d.u32("rejected")?,
            resets: d.u32("resets")?,
        },
        nav: d.code("nav", NavState::from_code)?,
        valid: Validity {
            attitude: d.flag("valid_attitude")?,
            heading: d.flag("valid_heading")?,
            altitude: d.flag("valid_altitude")?,
            velocity: d.flag("valid_velocity")?,
            position: d.flag("valid_position")?,
        },
        reasons: Reasons(d.bits("reasons", Reasons::ALL.0)?),
        action: d.code("action", Action::from_code)?,
        prearm: PrearmFlags(d.bits("prearm", PrearmFlags::ALL.0)?),
        home_set: d.flag("home_set")?,
        params_unsaved: d.flag("params_unsaved")?,
    };
    Ok(OutRecord {
        t,
        t_valid,
        motors,
        status,
        events,
    })
}

// ---- a tick's inputs ----

/// Why a tick's records did not make an `Inputs`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InputsError {
    /// A record is damaged.
    Decode(DecodeError),
    /// An input record whose presence bit the tick record does not set.
    NotPresent(&'static str),
    /// A presence bit with no record.
    Missing(&'static str),
    /// Two records of one input.
    Duplicate(&'static str),
    /// An input record with another tick's time.
    TimeMismatch {
        /// The input.
        topic: &'static str,
        /// The tick record's time.
        tick: u64,
        /// The input record's time.
        record: u64,
    },
}

impl From<DecodeError> for InputsError {
    fn from(e: DecodeError) -> Self {
        Self::Decode(e)
    }
}

impl fmt::Display for InputsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Decode(e) => write!(f, "{e}"),
            Self::NotPresent(t) => write!(f, "{t}: a record the tick record says is absent"),
            Self::Missing(t) => write!(f, "{t}: the tick record says present, no record"),
            Self::Duplicate(t) => write!(f, "{t}: two records in one tick"),
            Self::TimeMismatch {
                topic,
                tick,
                record,
            } => {
                write!(f, "{topic}: time {record} in the tick at {tick}")
            }
        }
    }
}

impl std::error::Error for InputsError {}

fn topic_name(b: u8) -> &'static str {
    TOPICS
        .iter()
        .find(|t| input_bit(t.name) == Some(b))
        .map_or("", |t| t.name)
}

/// Builds the `Inputs` of one tick from its tick record and the records that
/// follow it (up to the next tick record). Records of topics that are not
/// inputs (`oxide_out`, events, anything else) are left alone. Refuses an
/// input record whose presence bit is not set, a set bit with no record, two
/// records of one input, and a record whose time is not the tick's.
pub fn inputs_from_records<'a>(
    tick: &RecordView<'a>,
    records: impl IntoIterator<Item = RecordView<'a>>,
) -> Result<(TickRecord, Inputs), InputsError> {
    let tk = decode_tick(tick)?;
    let mut inputs = Inputs {
        param_ack: tk.param_ack,
        ..Inputs::default()
    };
    let mut seen: u16 = 0;
    for rec in records {
        let Some(b) = input_bit(rec.topic()) else {
            continue;
        };
        let name = topic_name(b);
        if !tk.has(b) {
            return Err(InputsError::NotPresent(name));
        }
        if seen & (1u16 << b) != 0 {
            return Err(InputsError::Duplicate(name));
        }
        seen |= 1u16 << b;
        let at = record_time(&rec)?;
        if at != tk.t {
            return Err(InputsError::TimeMismatch {
                topic: name,
                tick: tk.t.0,
                record: at.0,
            });
        }
        match b {
            bit::IMU => inputs.imu = Some(decode_imu(&rec)?),
            bit::STATE => inputs.state = Some(decode_state(&rec)?),
            bit::RC => inputs.rc = Some(decode_rc(&rec)?),
            bit::BATTERY => inputs.battery = Some(decode_battery(&rec)?),
            bit::SETPOINT => inputs.setpoint = Some(decode_setpoint(&rec)?),
            bit::GPS => inputs.gps = Some(decode_gps(&rec)?),
            bit::BARO => inputs.baro = Some(decode_baro(&rec)?),
            bit::MAG => inputs.mag = Some(decode_mag(&rec)?),
            bit::GCS => inputs.gcs_heartbeat = Some(decode_gcs(&rec)?),
            bit::BRAIN => inputs.brain_heartbeat = Some(decode_brain(&rec)?),
            _ => inputs.param_set = Some(decode_param_set(&rec)?),
        }
    }
    for b in 0..=bit::PARAM_SET {
        if tk.has(b) && seen & (1u16 << b) == 0 {
            return Err(InputsError::Missing(topic_name(b)));
        }
    }
    Ok((tk, inputs))
}

// ---- the configuration of a file ----

/// Why a file's configuration could not be rebuilt.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FileConfigError {
    /// A vehicle record is damaged.
    Decode(DecodeError),
    /// The `oxide_vehicle` records do not number 0, 1, 2, ... in order.
    IndexOutOfOrder {
        /// Position among the records.
        position: usize,
        /// The index it carries.
        index: u16,
    },
    /// The numbers and parameters do not make a configuration.
    Rebuild(RebuildError),
    /// The file has no `oxide_config_hash` information.
    NoHash,
    /// The rebuilt configuration's hash is not the one the file states.
    HashMismatch {
        /// In the file.
        logged: u64,
        /// Of the rebuilt configuration.
        rebuilt: u64,
    },
}

impl From<DecodeError> for FileConfigError {
    fn from(e: DecodeError) -> Self {
        Self::Decode(e)
    }
}

impl From<RebuildError> for FileConfigError {
    fn from(e: RebuildError) -> Self {
        Self::Rebuild(e)
    }
}

impl fmt::Display for FileConfigError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Decode(e) => write!(f, "{e}"),
            Self::IndexOutOfOrder { position, index } => {
                write!(f, "oxide_vehicle record {position} has index {index}")
            }
            Self::Rebuild(e) => write!(f, "cannot rebuild the configuration: {e:?}"),
            Self::NoHash => write!(f, "the file has no oxide_config_hash"),
            Self::HashMismatch { logged, rebuilt } => write!(
                f,
                "oxide_config_hash is {logged:#018x}, the rebuilt configuration's is {rebuilt:#018x}"
            ),
        }
    }
}

impl std::error::Error for FileConfigError {}

/// The `oxide_vehicle` records of a file in file order. Their indices must run
/// 0, 1, 2, ...
pub fn vehicle_numbers_of(file: &UlogFile) -> Result<Vec<VehicleNumber>, FileConfigError> {
    let mut out = Vec::new();
    for (position, r) in file.records("oxide_vehicle").enumerate() {
        let n = decode_vehicle(&r)?;
        if usize::from(n.index) != position {
            return Err(FileConfigError::IndexOutOfOrder {
                position,
                index: n.index,
            });
        }
        out.push(n);
    }
    Ok(out)
}

/// A log parameter as the core's value.
pub fn core_param(v: MsgParam) -> ParamValue {
    match v {
        MsgParam::F32(x) => ParamValue::F32(x),
        MsgParam::I32(x) => ParamValue::I32(x),
    }
}

/// Rebuilds the configuration a file was flown with: the `oxide_vehicle`
/// records, the initial parameters of the definitions (all of them), `profile`
/// for the rules between fields, and the `oxide_config_hash` information as a
/// check on the result.
pub fn config_of<const N: usize>(
    file: &UlogFile,
    profile: &Profile,
) -> Result<VehicleConfig<N>, FileConfigError> {
    let numbers = vehicle_numbers_of(file)?;
    let cfg = rebuild_config::<N>(
        numbers.iter().map(|n| (n.name.as_str(), n.value)),
        file.initial_params
            .iter()
            .map(|p| (p.name.as_str(), core_param(p.value))),
        profile,
    )?;
    let logged = file
        .info("oxide_config_hash")
        .and_then(crate::reader::InfoEntry::as_u64)
        .ok_or(FileConfigError::NoHash)?;
    let rebuilt = cfg.config_hash();
    if logged != rebuilt {
        return Err(FileConfigError::HashMismatch { logged, rebuilt });
    }
    Ok(cfg)
}
