// SPDX-License-Identifier: AGPL-3.0-or-later
//! Our own topics, for what PX4 has no counterpart of (ADR-0005 §3): the
//! core's own state in `oxide_status`, the estimator's health counters in
//! `oxide_estimator`, every event exactly in `oxide_event`, and the logger's
//! own statistics in `oxide_logger`.
//!
//! Field order and padding follow PX4's generator, like the PX4 topics.

use crate::Sink;
use crate::logger::Stats;
use crate::record::{RecordWriter, Topic};
use oxide_core::config::EstimatorSource;
use oxide_core::estimator::heading_check::{HeadingFit, HeadingGuard};
use oxide_core::io::{
    Action, DisarmReason, EstimatorStatus, Event, EventKind, FlightMode, NavState, PrearmFlags,
    Reasons, Status, Validity,
};
use oxide_core::math::Vec3;
use oxide_core::params::{self, LoadReport, ParamError, ParamId};
use oxide_core::time::Micros;

/// `oxide_status`: the core's own state.
pub const OXIDE_STATUS: Topic = Topic {
    name: "oxide_status",
    format: "uint64_t timestamp;uint32_t timing_faults;uint32_t events_dropped;\
             uint16_t reasons;uint16_t prearm;uint8_t mode;uint8_t nav;uint8_t action;\
             uint8_t estimator_source;uint8_t valid;bool armed;bool killed;bool saturated;\
             bool no_estimate;bool home_set;bool params_unsaved;bool landed;\
             bool ground_contact;uint8_t[7] _padding0;",
    record_len: 33,
};

/// `oxide_estimator`: the estimator's health, and the heading check's fit
/// after the latest GPS fix it judged while airborne (`fit_*`; all zero on
/// the ground: `fit_seq` counts the fixes, `fit_t` is the fix's sample time,
/// `fit_vel_ratio` is the GPS velocity test ratio of the filter's fusion of
/// that fix, -1 when it was not fused), and the heading guard's state:
/// `hdg_corrections` made in this flight, `hdg_contained` (the latch) and
/// `hdg_shift`, the run-time shift of the declination, rad.
pub const OXIDE_ESTIMATOR: Topic = Topic {
    name: "oxide_estimator",
    format: "uint64_t timestamp;uint32_t rejected;uint32_t resets;float[3] gyro_filtered;\
             bool aligned;bool gps_aiding;bool has_estimate;uint8_t fit_verdict;\
             uint32_t fit_seq;float fit_eps;float fit_sigma;float fit_sa;float fit_kappa;\
             float fit_rho;float fit_stat;float fit_vel_ratio;uint64_t fit_t;\
             uint8_t fit_streak;bool fit_used;uint8_t hdg_corrections;bool hdg_contained;\
             float hdg_shift;",
    record_len: 80,
};

/// `oxide_event`: one event, exactly. The fields are numbers; which of them
/// an event uses depends on its kind ([`EventFields`]).
pub const OXIDE_EVENT: Topic = Topic {
    name: "oxide_event",
    format: "uint64_t timestamp;uint64_t event_time;uint32_t param_key;uint16_t param_index;\
             uint16_t mask;uint16_t applied;uint16_t unknown;uint16_t dropped;uint16_t torn;\
             uint8_t kind;uint8_t code_a;uint8_t code_b;uint8_t why;uint8_t why_detail;\
             bool invalid;bool unreadable;uint8_t[1] _padding0;",
    record_len: 39,
};

/// `oxide_logger`: the logger's own counters, from [`Stats`].
pub const OXIDE_LOGGER: Topic = Topic {
    name: "oxide_logger",
    format: "uint64_t timestamp;uint64_t bytes;uint32_t written;uint32_t dropped_stream;\
             uint32_t dropped_must;uint32_t high_water;uint32_t dropouts;uint32_t syncs;",
    record_len: 40,
};

/// Bit numbers of the bit fields of `oxide_status`, written down. A bit
/// number is part of the log format and never changes; a new flag gets the
/// next unused one.
pub mod bits {
    /// `oxide_status.valid`: [`Validity`](oxide_core::io::Validity), one bit
    /// per field.
    pub mod valid {
        pub const ATTITUDE: u8 = 0;
        pub const HEADING: u8 = 1;
        pub const ALTITUDE: u8 = 2;
        pub const VELOCITY: u8 = 3;
        pub const POSITION: u8 = 4;
    }

    /// `oxide_status.reasons`: [`Reasons`](oxide_core::io::Reasons) as the
    /// core numbers them.
    pub mod reasons {
        pub const RC_LOST: u8 = 0;
        pub const NO_HUMAN_LINK: u8 = 1;
        pub const BRAIN_LOST: u8 = 2;
        pub const SETPOINT_STALE: u8 = 3;
        pub const BATTERY_LOW: u8 = 4;
        pub const BATTERY_CRITICAL: u8 = 5;
        pub const GPS_LOST: u8 = 6;
        pub const ESTIMATOR_UNHEALTHY: u8 = 7;
        pub const IMU_FAILURE: u8 = 8;
        pub const GEOFENCE: u8 = 9;
        pub const LOSS_OF_CONTROL: u8 = 10;
    }

    /// `oxide_status.prearm`: [`PrearmFlags`](oxide_core::io::PrearmFlags) as
    /// the core numbers them.
    pub mod prearm {
        pub const KILL: u8 = 0;
        pub const THROTTLE: u8 = 1;
        pub const RC: u8 = 2;
        pub const NOT_STILL: u8 = 3;
        pub const ESTIMATOR: u8 = 4;
        pub const POSITION: u8 = 5;
        pub const BATTERY: u8 = 6;
        pub const CONFIG: u8 = 7;
        pub const LINK: u8 = 8;
    }
}

/// The validity flags as the bits of [`bits::valid`].
pub const fn validity_bits(v: &Validity) -> u8 {
    (v.attitude as u8) << bits::valid::ATTITUDE
        | (v.heading as u8) << bits::valid::HEADING
        | (v.altitude as u8) << bits::valid::ALTITUDE
        | (v.velocity as u8) << bits::valid::VELOCITY
        | (v.position as u8) << bits::valid::POSITION
}

/// `oxide_status` from the status and what the telemetry adds. `mode`, `nav`,
/// `action` and `estimator_source` are the core's own codes
/// (`oxide_core::codes`), `valid`, `reasons` and `prearm` bit fields
/// ([`bits`]); `timing_faults` and the flags are the status's; `landed` and
/// `ground_contact` are the landing detector's and guidance's flags.
pub fn oxide_status<S: Sink + ?Sized>(
    w: &mut RecordWriter<'_, S>,
    now: Micros,
    s: &Status,
    source: EstimatorSource,
    events_dropped: u32,
    landed: bool,
    ground_contact: bool,
) {
    w.u64(now.0).u32(s.timing_faults).u32(events_dropped);
    w.u16(s.reasons.0).u16(s.prearm.0);
    w.u8(s.mode.code())
        .u8(s.nav.code())
        .u8(s.action.code())
        .u8(source.code())
        .u8(validity_bits(&s.valid));
    w.bool(s.armed)
        .bool(s.killed)
        .bool(s.saturated)
        .bool(s.no_estimate)
        .bool(s.home_set)
        .bool(s.params_unsaved)
        .bool(landed)
        .bool(ground_contact);
}

/// `oxide_estimator`: the rejected-measurement and reset counters, the
/// rate loop's filtered gyro (rad/s, body), the alignment and GPS-aiding
/// flags, whether the core had published an estimate at all, and the
/// heading check's fit after the latest GPS fix it judged while airborne,
/// and the heading guard's state (corrections, latch, declination shift).
pub fn oxide_estimator<S: Sink + ?Sized>(
    w: &mut RecordWriter<'_, S>,
    now: Micros,
    e: &EstimatorStatus,
    has_estimate: bool,
    gyro_filtered: Vec3<f32>,
    fit: &HeadingFit,
    guard: &HeadingGuard,
) {
    w.u64(now.0).u32(e.rejected).u32(e.resets);
    w.f32(gyro_filtered.x)
        .f32(gyro_filtered.y)
        .f32(gyro_filtered.z);
    w.bool(e.aligned).bool(e.gps_aiding).bool(has_estimate);
    w.u8(fit.verdict).u32(fit.seq);
    w.f32(fit.eps)
        .f32(fit.sigma)
        .f32(fit.sa)
        .f32(fit.kappa)
        .f32(fit.rho)
        .f32(fit.stat)
        .f32(fit.vel_ratio);
    w.u64(fit.t_us);
    w.u8(fit.streak).bool(fit.used);
    w.u8(guard.corrections)
        .bool(guard.contained)
        .f32(guard.shift_rad);
}

/// `oxide_logger` from the logger's statistics. `high_water` is the ring's
/// high-water mark in bytes, saturating at `u32::MAX`.
pub fn oxide_logger<S: Sink + ?Sized>(w: &mut RecordWriter<'_, S>, now: Micros, st: &Stats) {
    w.u64(now.0).u64(st.bytes);
    w.u32(st.written)
        .u32(st.dropped_stream)
        .u32(st.dropped_must)
        .u32(u32::try_from(st.high_water).unwrap_or(u32::MAX))
        .u32(st.dropouts)
        .u32(st.syncs);
}

// ---- oxide_event ----

/// An event as the numbers of an `oxide_event` record (all but `timestamp`).
/// Which fields an event uses depends on its kind; the others are zero.
///
/// | kind | fields |
/// |---|---|
/// | `Armed`, `Touchdown`, `HomeSet`, `ParamFaultAcked` | none |
/// | `Disarmed` | `code_a` the [`DisarmReason`] code |
/// | `ArmRefused` | `mask` the [`PrearmFlags`] bits |
/// | `ReasonRaised`, `ReasonCleared`, `PilotOverride` | `mask` the [`Reasons`] bits |
/// | `NavChanged` | `code_a` from, `code_b` to ([`NavState`] codes) |
/// | `ModeDegraded` | `code_a` requested ([`FlightMode`] code), `code_b` flown ([`NavState`] code) |
/// | `ActionChanged` | `code_a` from, `code_b` to ([`Action`] codes) |
/// | `ParamChanged` | `param_key`, `param_index` |
/// | `ParamRefused` | `param_key`, `param_index`, `why` the [`ParamError`] code, `why_detail` its detail code |
/// | `ParamLoadFault`, `ParamLoadRecovered` | `applied`, `unknown`, `dropped`, `torn`, `invalid`, `unreadable` |
/// | `HeadingCorrected` | `mask` the yaw change in milliradians (two's complement), `code_a` the corrections made in this flight |
/// | `HeadingContained`, `GpsSnapRepeated` | none |
///
/// A parameter is identified by its stable key (`params::key_of`);
/// `param_index` is where the table had it when the log was written, a note
/// that the decoder does not rely on. A parameter outside the table (a refused
/// change to an unknown id) has key 0, which no parameter has, and its id in
/// `param_index`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct EventFields {
    /// The event's own time, µs (`Event::t`).
    pub event_time: u64,
    /// Stable key of the parameter, 0 if it is not in the table.
    pub param_key: u32,
    /// `ParamId.0`.
    pub param_index: u16,
    /// Prearm flags or reasons.
    pub mask: u16,
    /// `LoadReport::applied`.
    pub applied: u16,
    /// `LoadReport::unknown`.
    pub unknown: u16,
    /// `LoadReport::dropped`.
    pub dropped: u16,
    /// `LoadReport::torn`.
    pub torn: u16,
    /// `EventKind::code()`.
    pub kind: u8,
    /// First code of the payload.
    pub code_a: u8,
    /// Second code of the payload.
    pub code_b: u8,
    /// `ParamError::code()`.
    pub why: u8,
    /// `ParamError::detail_code()`.
    pub why_detail: u8,
    /// `LoadReport::invalid`.
    pub invalid: bool,
    /// `LoadReport::unreadable`.
    pub unreadable: bool,
}

/// Why a record is not an event we wrote.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EventError {
    /// The record has no field of this name (it is not an `oxide_event`).
    Missing(&'static str),
    /// No event kind has this code.
    BadKind(u8),
    /// A code field holds a value no enumeration has.
    BadCode {
        /// The field.
        field: &'static str,
        /// Its value.
        code: u8,
    },
    /// A key no parameter has (or key 0 with an index that is in the table).
    BadParam {
        /// `param_key`.
        key: u32,
        /// `param_index`.
        index: u16,
    },
    /// Fields the kind does not use are not zero: not what the encoder writes.
    NotCanonical,
}

impl EventFields {
    /// The numbers of an event.
    pub fn from_event(e: &Event) -> Self {
        let mut f = Self {
            event_time: e.t.0,
            kind: e.kind.code(),
            ..Self::default()
        };
        let param = |f: &mut Self, id: ParamId| {
            f.param_key = params::key_of(id).unwrap_or(0);
            f.param_index = id.0;
        };
        match e.kind {
            EventKind::Armed
            | EventKind::Touchdown
            | EventKind::HomeSet
            | EventKind::ParamFaultAcked
            | EventKind::HeadingContained
            | EventKind::GpsSnapRepeated => {}
            EventKind::HeadingCorrected { angle_mrad, count } => {
                f.mask = u16::from_ne_bytes(angle_mrad.to_ne_bytes());
                f.code_a = count;
            }
            EventKind::Disarmed(r) => f.code_a = r.code(),
            EventKind::ArmRefused(p) => f.mask = p.0,
            EventKind::NavChanged { from, to } => {
                f.code_a = from.code();
                f.code_b = to.code();
            }
            EventKind::ModeDegraded { requested, flown } => {
                f.code_a = requested.code();
                f.code_b = flown.code();
            }
            EventKind::ReasonRaised(r)
            | EventKind::ReasonCleared(r)
            | EventKind::PilotOverride(r) => f.mask = r.0,
            EventKind::ActionChanged { from, to } => {
                f.code_a = from.code();
                f.code_b = to.code();
            }
            EventKind::ParamChanged(id) => param(&mut f, id),
            EventKind::ParamRefused { id, why } => {
                param(&mut f, id);
                f.why = why.code();
                f.why_detail = why.detail_code();
            }
            EventKind::ParamLoadFault(r) | EventKind::ParamLoadRecovered(r) => {
                f.applied = r.applied;
                f.unknown = r.unknown;
                f.dropped = r.dropped;
                f.torn = r.torn;
                f.invalid = r.invalid;
                f.unreadable = r.unreadable;
            }
        }
        f
    }

    /// The event these numbers describe. Refuses a kind or code that no
    /// variant has, a parameter the table does not know, and fields the kind
    /// does not use being non-zero (so that `from_event(to_event(x)) == x`).
    pub fn to_event(&self) -> Result<Event, EventError> {
        let a = self.code_a;
        let b = self.code_b;
        let bad = |field: &'static str, code: u8| EventError::BadCode { field, code };
        let disarm = |c: u8| DisarmReason::from_code(c).ok_or(bad("code_a", c));
        let nav = |field: &'static str, c: u8| NavState::from_code(c).ok_or(bad(field, c));
        let action = |field: &'static str, c: u8| Action::from_code(c).ok_or(bad(field, c));
        let report = || LoadReport {
            applied: self.applied,
            unknown: self.unknown,
            dropped: self.dropped,
            invalid: self.invalid,
            unreadable: self.unreadable,
            torn: self.torn,
        };
        let kind = match self.kind {
            0 => EventKind::Armed,
            1 => EventKind::Disarmed(disarm(a)?),
            2 => EventKind::ArmRefused(PrearmFlags(self.mask)),
            3 => EventKind::NavChanged {
                from: nav("code_a", a)?,
                to: nav("code_b", b)?,
            },
            4 => EventKind::ModeDegraded {
                requested: FlightMode::from_code(a).ok_or(bad("code_a", a))?,
                flown: nav("code_b", b)?,
            },
            5 => EventKind::ReasonRaised(Reasons(self.mask)),
            6 => EventKind::ReasonCleared(Reasons(self.mask)),
            7 => EventKind::ActionChanged {
                from: action("code_a", a)?,
                to: action("code_b", b)?,
            },
            8 => EventKind::PilotOverride(Reasons(self.mask)),
            9 => EventKind::Touchdown,
            10 => EventKind::HomeSet,
            11 => EventKind::ParamChanged(self.param_id()?),
            12 => {
                let id = self.param_id()?;
                let why = ParamError::from_code(self.why, self.why_detail).ok_or_else(|| {
                    if ParamError::from_code(self.why, 0).is_some() {
                        bad("why_detail", self.why_detail)
                    } else {
                        bad("why", self.why)
                    }
                })?;
                EventKind::ParamRefused { id, why }
            }
            13 => EventKind::ParamLoadFault(report()),
            14 => EventKind::ParamLoadRecovered(report()),
            15 => EventKind::ParamFaultAcked,
            16 => EventKind::HeadingCorrected {
                angle_mrad: i16::from_ne_bytes(self.mask.to_ne_bytes()),
                count: a,
            },
            17 => EventKind::HeadingContained,
            18 => EventKind::GpsSnapRepeated,
            k => return Err(EventError::BadKind(k)),
        };
        let event = Event {
            t: Micros(self.event_time),
            kind,
        };
        // The index of a parameter found by its key is a note, not part of
        // what must match.
        let mut again = Self::from_event(&event);
        if self.param_key != 0 {
            again.param_index = self.param_index;
        }
        if again != *self {
            return Err(EventError::NotCanonical);
        }
        Ok(event)
    }

    fn param_id(&self) -> Result<ParamId, EventError> {
        let bad = EventError::BadParam {
            key: self.param_key,
            index: self.param_index,
        };
        if self.param_key != 0 {
            params::find_key(self.param_key).ok_or(bad)
        } else {
            let id = ParamId(self.param_index);
            if params::key_of(id).is_none() {
                Ok(id)
            } else {
                Err(bad)
            }
        }
    }

    /// Writes the record: `now` is the tick time the record is written at.
    pub fn write<S: Sink + ?Sized>(&self, w: &mut RecordWriter<'_, S>, now: Micros) {
        w.u64(now.0).u64(self.event_time).u32(self.param_key);
        w.u16(self.param_index)
            .u16(self.mask)
            .u16(self.applied)
            .u16(self.unknown)
            .u16(self.dropped)
            .u16(self.torn);
        w.u8(self.kind)
            .u8(self.code_a)
            .u8(self.code_b)
            .u8(self.why)
            .u8(self.why_detail);
        w.bool(self.invalid).bool(self.unreadable);
    }
}

/// `oxide_event`: one event, exactly (see [`EventFields`]); `now` is the tick
/// time the record is written at, `e.t` the event's own time.
pub fn oxide_event<S: Sink + ?Sized>(w: &mut RecordWriter<'_, S>, now: Micros, e: &Event) {
    EventFields::from_event(e).write(w, now);
}

#[cfg(any(test, feature = "std"))]
mod decode {
    use super::{EventError, EventFields};
    use crate::reader::RecordView;
    use core::fmt;
    use oxide_core::io::Event;

    impl fmt::Display for EventError {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            match self {
                Self::Missing(field) => write!(f, "oxide_event: field {field} is missing"),
                Self::BadKind(k) => write!(f, "oxide_event: no event kind has code {k}"),
                Self::BadCode { field, code } => {
                    write!(f, "oxide_event: {field} has no meaning for code {code}")
                }
                Self::BadParam { key, index } => write!(
                    f,
                    "oxide_event: no parameter has key {key:#010x} (index {index})"
                ),
                Self::NotCanonical => {
                    write!(f, "oxide_event: fields this kind does not use are not zero")
                }
            }
        }
    }

    impl std::error::Error for EventError {}

    /// The numbers of an `oxide_event` record, as written; refuses a record
    /// that lacks a field.
    pub fn event_fields(r: &RecordView<'_>) -> Result<EventFields, EventError> {
        fn need<T>(v: Option<T>, name: &'static str) -> Result<T, EventError> {
            v.ok_or(EventError::Missing(name))
        }
        // A flag is one byte, 0 or 1: any other value is not what we write.
        let flag = |name: &'static str| match r.field_bytes(name).and_then(|b| b.first()) {
            Some(0) => Ok(false),
            Some(1) => Ok(true),
            Some(_) => Err(EventError::NotCanonical),
            None => Err(EventError::Missing(name)),
        };
        Ok(EventFields {
            event_time: need(r.u64("event_time"), "event_time")?,
            param_key: need(r.u32("param_key"), "param_key")?,
            param_index: need(r.u16("param_index"), "param_index")?,
            mask: need(r.u16("mask"), "mask")?,
            applied: need(r.u16("applied"), "applied")?,
            unknown: need(r.u16("unknown"), "unknown")?,
            dropped: need(r.u16("dropped"), "dropped")?,
            torn: need(r.u16("torn"), "torn")?,
            kind: need(r.u8("kind"), "kind")?,
            code_a: need(r.u8("code_a"), "code_a")?,
            code_b: need(r.u8("code_b"), "code_b")?,
            why: need(r.u8("why"), "why")?,
            why_detail: need(r.u8("why_detail"), "why_detail")?,
            invalid: flag("invalid")?,
            unreadable: flag("unreadable")?,
        })
    }

    /// The event an `oxide_event` record holds, exactly. The record's own
    /// `timestamp` is the tick it was written at: read it from the record.
    pub fn decode_event(r: &RecordView<'_>) -> Result<Event, EventError> {
        event_fields(r)?.to_event()
    }
}

#[cfg(any(test, feature = "std"))]
pub use decode::{decode_event, event_fields};
