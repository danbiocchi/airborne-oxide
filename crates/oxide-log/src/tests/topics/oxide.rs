// SPDX-License-Identifier: AGPL-3.0-or-later
//! Our own topics: `oxide_status`, `oxide_estimator`, `oxide_event`,
//! `oxide_logger`, and the decoder of `oxide_event`.

use super::{floats, ints, one_record, quad};
use crate::logger::Stats;
use crate::msg;
use crate::reader::UlogFile;
use crate::record::{self, RecordWriter};
use crate::topics::{
    self, EventError, EventFields, NORMAL, OXIDE_ESTIMATOR, OXIDE_EVENT, OXIDE_LOGGER,
    OXIDE_STATUS, bits, decode_event, event_fields,
};
use oxide_core::config::EstimatorSource;
use oxide_core::estimator::heading_check::{HeadingFit, HeadingGuard};
use oxide_core::io::{
    Action, DisarmReason, EstimatorStatus, Event, EventKind, FlightMode, NavState, PrearmFlags,
    Reasons, Status, Validity,
};
use oxide_core::math::Vec3;
use oxide_core::params::{self, LoadReport, ParamError, ParamId};
use oxide_core::time::Micros;

fn topic(name: &str) -> &'static record::Topic {
    NORMAL.iter().find(|t| t.name == name).unwrap()
}

pub(super) fn all_samples() -> Vec<Vec<u8>> {
    let mut out = Vec::new();
    let mut put = |name: &str, fill: &mut dyn FnMut(&mut RecordWriter<'_, Vec<u8>>)| {
        let id = topics::IDS.iter().find(|(n, _)| *n == name).unwrap().1;
        let mut v = Vec::new();
        record::data(&mut v, id, topic(name), |w| fill(w)).unwrap();
        out.push(v);
    };
    let s = Status {
        armed: true,
        valid: Validity::ALL,
        ..Status::default()
    };
    put("oxide_status", &mut |w| {
        topics::oxide_status(
            w,
            Micros(2_000_000),
            &s,
            EstimatorSource::Ekf,
            1,
            true,
            false,
        )
    });
    put("oxide_estimator", &mut |w| {
        topics::oxide_estimator(
            w,
            Micros(2_000_000),
            &EstimatorStatus {
                aligned: true,
                gps_aiding: true,
                rejected: 3,
                resets: 1,
            },
            true,
            Vec3::new(0.1, 0.2, 0.3),
            &HeadingFit::default(),
            &HeadingGuard {
                corrections: 1,
                contained: true,
                shift_rad: -0.75,
                tilt_roll_rad: 0.0,
                tilt_pitch_rad: 0.0,
            },
        )
    });
    let e = Event {
        t: Micros(1_999_000),
        kind: EventKind::Armed,
    };
    put("oxide_event", &mut |w| {
        topics::oxide_event(w, Micros(2_000_000), &e)
    });
    let st = Stats {
        written: 10,
        bytes: 1000,
        high_water: 512,
        ..Stats::default()
    };
    put("oxide_logger", &mut |w| {
        topics::oxide_logger(w, Micros(2_000_000), &st)
    });
    out
}

// ---- oxide_status ----

#[test]
fn oxide_status_carries_the_cores_own_codes_and_bit_fields() {
    let s = Status {
        armed: true,
        killed: false,
        mode: FlightMode::Position,
        saturated: true,
        timing_faults: 7,
        no_estimate: false,
        estimator: EstimatorStatus::default(),
        nav: NavState::Hold,
        valid: Validity {
            attitude: true,
            heading: false,
            altitude: true,
            velocity: false,
            position: true,
        },
        reasons: Reasons::RC_LOST.union(Reasons::GPS_LOST),
        action: Action::Rtl,
        prearm: PrearmFlags::KILL.union(PrearmFlags::LINK),
        home_set: true,
        params_unsaved: true,
    };
    let f = one_record(&OXIDE_STATUS, |w| {
        topics::oxide_status(w, Micros(123), &s, EstimatorSource::Ekf, 3, true, false)
    });
    let r = f.records("oxide_status").next().unwrap();
    ints(
        &r,
        &[
            ("timestamp", 123),
            ("timing_faults", 7),
            ("events_dropped", 3),
            ("reasons", 0b100_0001),
            ("prearm", 0b1_0000_0001),
            ("mode", 2),
            ("nav", 5),
            ("action", 2),
            ("estimator_source", 1),
            ("valid", 0b1_0101),
            ("armed", 1),
            ("killed", 0),
            ("saturated", 1),
            ("no_estimate", 0),
            ("home_set", 1),
            ("params_unsaved", 1),
            ("landed", 1),
            ("ground_contact", 0),
        ],
    );
    assert_eq!(r.raw().len(), 33);

    // The other value of every flag.
    let s = Status {
        killed: true,
        no_estimate: true,
        mode: FlightMode::Offboard,
        nav: NavState::Terminated,
        action: Action::Terminate,
        ..Status::default()
    };
    let f = one_record(&OXIDE_STATUS, |w| {
        topics::oxide_status(w, Micros(1), &s, EstimatorSource::External, 0, false, true)
    });
    let r = f.records("oxide_status").next().unwrap();
    ints(
        &r,
        &[
            ("armed", 0),
            ("killed", 1),
            ("no_estimate", 1),
            ("mode", 3),
            ("nav", 9),
            ("action", 5),
            ("estimator_source", 0),
            ("landed", 0),
            ("ground_contact", 1),
            ("valid", 0),
            ("reasons", 0),
            ("prearm", 0),
        ],
    );
}

#[test]
fn the_bit_numbers_of_the_flag_sets_are_pinned() {
    // Validity: the log's own numbering.
    let valid: &[(&str, u8, Validity)] = &[
        (
            "ATTITUDE",
            bits::valid::ATTITUDE,
            Validity {
                attitude: true,
                ..Validity::NONE
            },
        ),
        (
            "HEADING",
            bits::valid::HEADING,
            Validity {
                heading: true,
                ..Validity::NONE
            },
        ),
        (
            "ALTITUDE",
            bits::valid::ALTITUDE,
            Validity {
                altitude: true,
                ..Validity::NONE
            },
        ),
        (
            "VELOCITY",
            bits::valid::VELOCITY,
            Validity {
                velocity: true,
                ..Validity::NONE
            },
        ),
        (
            "POSITION",
            bits::valid::POSITION,
            Validity {
                position: true,
                ..Validity::NONE
            },
        ),
    ];
    let numbers: Vec<(&str, u8)> = valid.iter().map(|(n, b, _)| (*n, *b)).collect();
    assert_eq!(
        numbers,
        [
            ("ATTITUDE", 0),
            ("HEADING", 1),
            ("ALTITUDE", 2),
            ("VELOCITY", 3),
            ("POSITION", 4)
        ]
    );
    for (name, bit, v) in valid {
        assert_eq!(topics::validity_bits(v), 1 << bit, "{name}");
    }
    assert_eq!(topics::validity_bits(&Validity::ALL), 0b1_1111);
    assert_eq!(topics::validity_bits(&Validity::NONE), 0);

    // Reasons and pre-arm flags: the core's own, which the log restates.
    let reasons: &[(&str, u8, Reasons)] = &[
        ("RC_LOST", bits::reasons::RC_LOST, Reasons::RC_LOST),
        (
            "NO_HUMAN_LINK",
            bits::reasons::NO_HUMAN_LINK,
            Reasons::NO_HUMAN_LINK,
        ),
        ("BRAIN_LOST", bits::reasons::BRAIN_LOST, Reasons::BRAIN_LOST),
        (
            "SETPOINT_STALE",
            bits::reasons::SETPOINT_STALE,
            Reasons::SETPOINT_STALE,
        ),
        (
            "BATTERY_LOW",
            bits::reasons::BATTERY_LOW,
            Reasons::BATTERY_LOW,
        ),
        (
            "BATTERY_CRITICAL",
            bits::reasons::BATTERY_CRITICAL,
            Reasons::BATTERY_CRITICAL,
        ),
        ("GPS_LOST", bits::reasons::GPS_LOST, Reasons::GPS_LOST),
        (
            "ESTIMATOR_UNHEALTHY",
            bits::reasons::ESTIMATOR_UNHEALTHY,
            Reasons::ESTIMATOR_UNHEALTHY,
        ),
        (
            "IMU_FAILURE",
            bits::reasons::IMU_FAILURE,
            Reasons::IMU_FAILURE,
        ),
        ("GEOFENCE", bits::reasons::GEOFENCE, Reasons::GEOFENCE),
        (
            "LOSS_OF_CONTROL",
            bits::reasons::LOSS_OF_CONTROL,
            Reasons::LOSS_OF_CONTROL,
        ),
    ];
    for (i, (name, bit, flag)) in reasons.iter().enumerate() {
        assert_eq!(usize::from(*bit), i, "{name}: pinned to its position");
        assert_eq!(flag.0, 1_u16 << bit, "{name}");
    }
    assert_eq!(Reasons::ALL.0, 0x07ff);
    let prearm: &[(&str, u8, PrearmFlags)] = &[
        ("KILL", bits::prearm::KILL, PrearmFlags::KILL),
        ("THROTTLE", bits::prearm::THROTTLE, PrearmFlags::THROTTLE),
        ("RC", bits::prearm::RC, PrearmFlags::RC),
        ("NOT_STILL", bits::prearm::NOT_STILL, PrearmFlags::NOT_STILL),
        ("ESTIMATOR", bits::prearm::ESTIMATOR, PrearmFlags::ESTIMATOR),
        ("POSITION", bits::prearm::POSITION, PrearmFlags::POSITION),
        ("BATTERY", bits::prearm::BATTERY, PrearmFlags::BATTERY),
        ("CONFIG", bits::prearm::CONFIG, PrearmFlags::CONFIG),
        ("LINK", bits::prearm::LINK, PrearmFlags::LINK),
    ];
    for (i, (name, bit, flag)) in prearm.iter().enumerate() {
        assert_eq!(usize::from(*bit), i, "{name}");
        assert_eq!(flag.0, 1_u16 << bit, "{name}");
    }
    assert_eq!(PrearmFlags::ALL.0, 0x01ff);
}

// ---- oxide_estimator, oxide_logger ----

#[test]
fn oxide_estimator_table() {
    let e = EstimatorStatus {
        aligned: true,
        gps_aiding: false,
        rejected: 70_000,
        resets: 4_000_000_000,
    };
    let guard = HeadingGuard {
        corrections: 2,
        contained: true,
        shift_rad: -1.5,
        tilt_roll_rad: 0.5,
        tilt_pitch_rad: -0.25,
    };
    let fit = HeadingFit {
        seq: 7,
        t_us: 5_000_000_001,
        used: true,
        eps: 1.5,
        sigma: 0.25,
        sa: 4.0,
        kappa: 1.25,
        rho: 0.125,
        stat: 4.75,
        streak: 3,
        verdict: 1,
        vel_ratio: 0.5,
    };
    let f = one_record(&OXIDE_ESTIMATOR, |w| {
        topics::oxide_estimator(
            w,
            Micros(55),
            &e,
            true,
            Vec3::new(0.5, -1.5, 2.0),
            &fit,
            &guard,
        )
    });
    let r = f.records("oxide_estimator").next().unwrap();
    ints(
        &r,
        &[
            ("timestamp", 55),
            ("rejected", 70_000),
            ("resets", 4_000_000_000),
            ("aligned", 1),
            ("gps_aiding", 0),
            ("has_estimate", 1),
            ("fit_verdict", 1),
            ("fit_seq", 7),
            ("fit_t", 5_000_000_001),
            ("fit_streak", 3),
            ("fit_used", 1),
            ("hdg_corrections", 2),
            ("hdg_contained", 1),
        ],
    );
    floats(
        &r,
        &[
            ("gyro_filtered[0]", 0.5),
            ("gyro_filtered[1]", -1.5),
            ("gyro_filtered[2]", 2.0),
            ("fit_eps", 1.5),
            ("fit_sigma", 0.25),
            ("fit_sa", 4.0),
            ("fit_kappa", 1.25),
            ("fit_rho", 0.125),
            ("fit_stat", 4.75),
            ("fit_vel_ratio", 0.5),
            ("hdg_shift", -1.5),
            ("tilt_roll", 0.5),
            ("tilt_pitch", -0.25),
        ],
    );
    // 31 bytes before the heading fit was added, 74 before the guard's
    // state, 80 before the tilt reference's roll and pitch.
    assert_eq!(r.raw().len(), 88);
}

#[test]
fn oxide_logger_table() {
    let st = Stats {
        written: 1,
        dropped_stream: 2,
        dropped_must: 3,
        bytes: 5_000_000_000,
        high_water: 4096,
        dropouts: 6,
        syncs: 7,
    };
    let f = one_record(&OXIDE_LOGGER, |w| topics::oxide_logger(w, Micros(9), &st));
    let r = f.records("oxide_logger").next().unwrap();
    ints(
        &r,
        &[
            ("timestamp", 9),
            ("written", 1),
            ("dropped_stream", 2),
            ("dropped_must", 3),
            ("bytes", 5_000_000_000),
            ("high_water", 4096),
            ("dropouts", 6),
            ("syncs", 7),
        ],
    );
    // A ring larger than u32 saturates rather than wrapping.
    let st = Stats {
        high_water: usize::MAX,
        ..Stats::default()
    };
    let f = one_record(&OXIDE_LOGGER, |w| topics::oxide_logger(w, Micros(9), &st));
    let r = f.records("oxide_logger").next().unwrap();
    assert_eq!(r.u32("high_water"), Some(u32::MAX));
}

// ---- oxide_event ----

/// Exhaustive: a new `EventKind` variant stops this compiling, which is the
/// reminder to give it a code, a payload layout and cases below.
fn variant_index(k: &EventKind) -> usize {
    match k {
        EventKind::Armed => 0,
        EventKind::Disarmed(_) => 1,
        EventKind::ArmRefused(_) => 2,
        EventKind::NavChanged { .. } => 3,
        EventKind::ModeDegraded { .. } => 4,
        EventKind::ReasonRaised(_) => 5,
        EventKind::ReasonCleared(_) => 6,
        EventKind::ActionChanged { .. } => 7,
        EventKind::PilotOverride(_) => 8,
        EventKind::Touchdown => 9,
        EventKind::HomeSet => 10,
        EventKind::ParamChanged(_) => 11,
        EventKind::ParamRefused { .. } => 12,
        EventKind::ParamLoadFault(_) => 13,
        EventKind::ParamLoadRecovered(_) => 14,
        EventKind::ParamFaultAcked => 15,
        EventKind::HeadingCorrected { .. } => 16,
        EventKind::HeadingContained => 17,
        EventKind::GpsSnapRepeated => 18,
    }
}
const VARIANTS: usize = 19;

const NAVS: [NavState; 10] = [
    NavState::Disarmed,
    NavState::Stabilized,
    NavState::Altitude,
    NavState::Position,
    NavState::Offboard,
    NavState::Hold,
    NavState::Rtl,
    NavState::Land,
    NavState::LandNow,
    NavState::Terminated,
];
const MODES: [FlightMode; 4] = [
    FlightMode::Stabilized,
    FlightMode::Altitude,
    FlightMode::Position,
    FlightMode::Offboard,
];
const ACTIONS: [Action; 6] = [
    Action::None,
    Action::Hold,
    Action::Rtl,
    Action::Land,
    Action::LandNow,
    Action::Terminate,
];
const DISARMS: [DisarmReason; 4] = [
    DisarmReason::Switch,
    DisarmReason::Kill,
    DisarmReason::Touchdown,
    DisarmReason::Terminated,
];

fn param_errors() -> Vec<ParamError> {
    use oxide_core::config::ConfigError as C;
    let mut v = vec![
        ParamError::Unknown,
        ParamError::WrongType,
        ParamError::NotFinite,
        ParamError::OutOfRange,
        ParamError::Armed,
    ];
    for c in [
        C::BadMassProperties,
        C::BadRotor,
        C::UncontrollableLayout,
        C::BadThrustExpo,
        C::BadProfile,
        C::BadEstimator,
        C::BadFailsafe,
        C::BadGains,
        C::BadLimits,
        C::CannotLift,
        C::IdleTooHigh,
        C::DescentLimitTooLow,
        C::LossOfControlTooTight,
        C::FilterCutoffTooHigh,
    ] {
        v.push(ParamError::Invalid(c));
    }
    v
}

fn reports() -> Vec<LoadReport> {
    let mut v = vec![
        LoadReport::default(),
        LoadReport {
            applied: 93,
            unknown: 1,
            dropped: 2,
            invalid: true,
            unreadable: false,
            torn: 5,
        },
        LoadReport {
            applied: u16::MAX,
            unknown: u16::MAX,
            dropped: u16::MAX,
            invalid: true,
            unreadable: true,
            torn: u16::MAX,
        },
        LoadReport {
            applied: 0,
            unknown: 0,
            dropped: 0,
            invalid: false,
            unreadable: true,
            torn: 0,
        },
    ];
    // Each field alone.
    v.push(LoadReport {
        applied: 1,
        ..LoadReport::default()
    });
    v.push(LoadReport {
        unknown: 1,
        ..LoadReport::default()
    });
    v.push(LoadReport {
        dropped: 1,
        ..LoadReport::default()
    });
    v.push(LoadReport {
        torn: 1,
        ..LoadReport::default()
    });
    v.push(LoadReport {
        invalid: true,
        ..LoadReport::default()
    });
    v
}

/// Every variant with many payloads, and event times at the extremes.
fn events() -> Vec<Event> {
    // The variants with no payload twice, at different times.
    let mut kinds = vec![
        EventKind::Armed,
        EventKind::Touchdown,
        EventKind::HomeSet,
        EventKind::ParamFaultAcked,
        EventKind::Armed,
        EventKind::Touchdown,
        EventKind::HomeSet,
        EventKind::ParamFaultAcked,
    ];
    kinds.extend(DISARMS.iter().map(|r| EventKind::Disarmed(*r)));
    let masks = [0_u16, 1, 1 << 3, 0x01ff, 0x07ff, 0x8000, 0xffff];
    for m in masks {
        kinds.push(EventKind::ArmRefused(PrearmFlags(m)));
        kinds.push(EventKind::ReasonRaised(Reasons(m)));
        kinds.push(EventKind::ReasonCleared(Reasons(m)));
        kinds.push(EventKind::PilotOverride(Reasons(m)));
    }
    for from in NAVS {
        for to in NAVS {
            kinds.push(EventKind::NavChanged { from, to });
        }
        for requested in MODES {
            kinds.push(EventKind::ModeDegraded {
                requested,
                flown: from,
            });
        }
    }
    for from in ACTIONS {
        for to in ACTIONS {
            kinds.push(EventKind::ActionChanged { from, to });
        }
    }
    for i in 0..params::COUNT {
        kinds.push(EventKind::ParamChanged(ParamId(i as u16)));
    }
    let errors = param_errors();
    for id in [0_u16, 1, 46, params::COUNT as u16 - 1, 60_000, u16::MAX] {
        for why in &errors {
            kinds.push(EventKind::ParamRefused {
                id: ParamId(id),
                why: *why,
            });
        }
    }
    for rep in reports() {
        kinds.push(EventKind::ParamLoadFault(rep));
        kinds.push(EventKind::ParamLoadRecovered(rep));
    }
    for angle_mrad in [i16::MIN, -3142, -1, 0, 1, 1000, 3142, i16::MAX] {
        for count in [0_u8, 1, 2, 5, u8::MAX] {
            kinds.push(EventKind::HeadingCorrected { angle_mrad, count });
        }
    }
    kinds.push(EventKind::HeadingContained);
    kinds.push(EventKind::HeadingContained);
    kinds.push(EventKind::GpsSnapRepeated);
    kinds.push(EventKind::GpsSnapRepeated);
    let times = [0_u64, 1, 123_456_789, 4_294_967_296, u64::MAX];
    kinds
        .into_iter()
        .enumerate()
        .map(|(i, kind)| Event {
            t: Micros(times[i % times.len()]),
            kind,
        })
        .collect()
}

fn log_of(records: impl FnOnce(&mut Vec<u8>)) -> UlogFile {
    let mut f = Vec::new();
    msg::file_header(&mut f, 0).unwrap();
    msg::flag_bits(&mut f, [0; 8], [0; 8]).unwrap();
    OXIDE_EVENT.write_format(&mut f).unwrap();
    OXIDE_EVENT.add_logged(&mut f, 0, 0).unwrap();
    records(&mut f);
    UlogFile::parse(&f).unwrap()
}

#[test]
fn every_event_kind_with_every_payload_round_trips_exactly() {
    let events = events();
    // Every variant, at least twice with different payloads.
    let mut seen = [0usize; VARIANTS];
    for e in &events {
        assert_eq!(variant_index(&e.kind), usize::from(e.kind.code()));
        seen[variant_index(&e.kind)] += 1;
    }
    for (i, n) in seen.iter().enumerate() {
        assert!(*n >= 2, "variant {i} has {n} payloads");
    }
    let f = log_of(|out| {
        for (i, e) in events.iter().enumerate() {
            record::data(out, 0, &OXIDE_EVENT, |w| {
                topics::oxide_event(w, Micros(1_000 * i as u64), e)
            })
            .unwrap();
        }
    });
    let recs: Vec<_> = f.records("oxide_event").collect();
    assert_eq!(recs.len(), events.len());
    for (i, (r, e)) in recs.iter().zip(&events).enumerate() {
        assert_eq!(r.u64("timestamp"), Some(1_000 * i as u64));
        assert_eq!(decode_event(r), Ok(*e), "{e:?}");
    }
}

#[test]
fn the_payload_is_numbers_in_named_fields() {
    let f = one_record(&OXIDE_EVENT, |w| {
        topics::oxide_event(
            w,
            Micros(77),
            &Event {
                t: Micros(70),
                kind: EventKind::NavChanged {
                    from: NavState::Position,
                    to: NavState::Land,
                },
            },
        )
    });
    let r = f.records("oxide_event").next().unwrap();
    ints(
        &r,
        &[
            ("timestamp", 77),
            ("event_time", 70),
            ("kind", 3),
            ("code_a", 3),
            ("code_b", 7),
        ],
    );
    let key = params::key_of(ParamId::RATE_ROLL_P).unwrap();
    let f = one_record(&OXIDE_EVENT, |w| {
        topics::oxide_event(
            w,
            Micros(1),
            &Event {
                t: Micros(1),
                kind: EventKind::ParamRefused {
                    id: ParamId::RATE_ROLL_P,
                    why: ParamError::Invalid(oxide_core::config::ConfigError::BadGains),
                },
            },
        )
    });
    let r = f.records("oxide_event").next().unwrap();
    assert_eq!(r.u32("param_key"), Some(key));
    ints(
        &r,
        &[
            ("kind", 12),
            ("why", 4),
            ("why_detail", 7),
            ("param_index", i64::from(ParamId::RATE_ROLL_P.0)),
        ],
    );
}

fn decode_fields(fields: &EventFields) -> Result<Event, EventError> {
    let f = log_of(|out| {
        record::data(out, 0, &OXIDE_EVENT, |w| fields.write(w, Micros(5))).unwrap();
    });
    let r = f.records("oxide_event").next().unwrap();
    decode_event(&r)
}

fn base(kind: EventKind) -> EventFields {
    EventFields::from_event(&Event { t: Micros(9), kind })
}

#[test]
fn every_other_code_is_refused_by_the_decoder() {
    // Event kinds.
    for k in 19..=255_u8 {
        let fields = EventFields {
            kind: k,
            ..base(EventKind::Armed)
        };
        assert_eq!(decode_event_of(&fields), Err(EventError::BadKind(k)), "{k}");
    }
    // Each code field of each variant that has one.
    let nav = base(EventKind::NavChanged {
        from: NavState::Hold,
        to: NavState::Land,
    });
    let degraded = base(EventKind::ModeDegraded {
        requested: FlightMode::Position,
        flown: NavState::Hold,
    });
    let action = base(EventKind::ActionChanged {
        from: Action::None,
        to: Action::Hold,
    });
    let disarmed = base(EventKind::Disarmed(DisarmReason::Kill));
    let refused = base(EventKind::ParamRefused {
        id: ParamId::RATE_ROLL_P,
        why: ParamError::Armed,
    });
    for bad in 0..=255_u8 {
        let a_ok = |max: u8| bad < max;
        let cases = [
            (EventFields { code_a: bad, ..nav }, "code_a", a_ok(10)),
            (EventFields { code_b: bad, ..nav }, "code_b", a_ok(10)),
            (
                EventFields {
                    code_a: bad,
                    ..degraded
                },
                "code_a",
                a_ok(4),
            ),
            (
                EventFields {
                    code_b: bad,
                    ..degraded
                },
                "code_b",
                a_ok(10),
            ),
            (
                EventFields {
                    code_a: bad,
                    ..action
                },
                "code_a",
                a_ok(6),
            ),
            (
                EventFields {
                    code_b: bad,
                    ..action
                },
                "code_b",
                a_ok(6),
            ),
            (
                EventFields {
                    code_a: bad,
                    ..disarmed
                },
                "code_a",
                a_ok(4),
            ),
        ];
        for (fields, name, ok) in cases {
            match decode_event_of(&fields) {
                Ok(_) => assert!(ok, "{name} = {bad} was accepted"),
                Err(EventError::BadCode { field, code }) => {
                    assert!(!ok, "{name} = {bad} was refused");
                    assert_eq!((field, code), (name, bad));
                }
                Err(e) => panic!("{name} = {bad}: {e:?}"),
            }
        }
        // The refusal kind: 0..=5; and a detail only where it means something.
        let why = EventFields {
            why: bad,
            ..refused
        };
        match decode_event_of(&why) {
            Ok(_) => assert!(bad <= 5, "why = {bad}"),
            Err(EventError::BadCode { field, code }) => {
                assert_eq!((field, code), ("why", bad));
                assert!(bad > 5);
            }
            Err(e) => panic!("why = {bad}: {e:?}"),
        }
    }
    // `Invalid` carries a config error code 0..=13; the others carry none.
    let invalid = EventFields { why: 4, ..refused };
    for d in 0..=255_u8 {
        let r = decode_event_of(&EventFields {
            why_detail: d,
            ..invalid
        });
        match r {
            Ok(_) => assert!(d <= 13),
            Err(EventError::BadCode { field, code }) => {
                assert_eq!((field, code, d > 13), ("why_detail", d, true))
            }
            Err(e) => panic!("{e:?}"),
        }
    }
    let r = decode_event_of(&EventFields {
        why: 3,
        why_detail: 1,
        ..refused
    });
    assert_eq!(
        r,
        Err(EventError::BadCode {
            field: "why_detail",
            code: 1
        })
    );
}

fn decode_event_of(fields: &EventFields) -> Result<Event, EventError> {
    decode_fields(fields)
}

#[test]
fn a_record_that_does_not_say_what_the_encoder_says_is_refused() {
    // A payload field set for a kind that has none: not what we wrote.
    let armed = EventFields {
        code_a: 1,
        ..base(EventKind::Armed)
    };
    assert_eq!(decode_event_of(&armed), Err(EventError::NotCanonical));
    let raised = EventFields {
        applied: 3,
        ..base(EventKind::ReasonRaised(Reasons::RC_LOST))
    };
    assert_eq!(decode_event_of(&raised), Err(EventError::NotCanonical));
}

#[test]
fn parameters_are_identified_by_their_stable_key() {
    let changed = base(EventKind::ParamChanged(ParamId::RATE_ROLL_P));
    assert_eq!(
        changed.param_key,
        params::key_of(ParamId::RATE_ROLL_P).unwrap()
    );
    // The key is the identity; the index is a note of where it was.
    let moved = EventFields {
        param_index: 60_000,
        ..changed
    };
    assert_eq!(
        decode_event_of(&moved),
        Ok(Event {
            t: Micros(9),
            kind: EventKind::ParamChanged(ParamId::RATE_ROLL_P)
        })
    );
    // A key no parameter has.
    let unknown = EventFields {
        param_key: 0x1234_5678,
        ..changed
    };
    assert!(params::find_key(0x1234_5678).is_none());
    assert_eq!(
        decode_event_of(&unknown),
        Err(EventError::BadParam {
            key: 0x1234_5678,
            index: changed.param_index
        })
    );
    // No key means "not in the table": the index must really be outside it.
    let no_key = EventFields {
        param_key: 0,
        ..changed
    };
    assert!(matches!(
        decode_event_of(&no_key),
        Err(EventError::BadParam { key: 0, .. })
    ));
    // No parameter has key 0, so 0 can stand for "no such parameter".
    for i in 0..params::COUNT {
        assert_ne!(params::key_of(ParamId(i as u16)), Some(0));
    }
    let outside = base(EventKind::ParamRefused {
        id: ParamId(60_000),
        why: ParamError::Unknown,
    });
    assert_eq!((outside.param_key, outside.param_index), (0, 60_000));
}

#[test]
fn a_record_of_another_topic_is_missing_the_fields() {
    let f = one_record(&OXIDE_LOGGER, |w| {
        topics::oxide_logger(w, Micros(1), &Stats::default())
    });
    let r = f.records("oxide_logger").next().unwrap();
    assert_eq!(decode_event(&r), Err(EventError::Missing("event_time")));
    assert!(event_fields(&r).is_err());
}

#[test]
fn event_errors_say_what_is_wrong() {
    assert_eq!(
        EventError::BadKind(77).to_string(),
        "oxide_event: no event kind has code 77"
    );
    assert_eq!(
        EventError::BadCode {
            field: "code_b",
            code: 200
        }
        .to_string(),
        "oxide_event: code_b has no meaning for code 200"
    );
}

#[test]
fn the_quad_is_a_valid_configuration() {
    // The configuration the header tests use.
    assert!(quad().validate().is_ok());
}

#[test]
fn a_flag_byte_other_than_zero_or_one_is_refused() {
    let fields = base(EventKind::ParamLoadFault(LoadReport::default()));
    for (invalid, unreadable) in [(2u8, 0u8), (0, 2), (255, 1)] {
        let f = log_of(|out| {
            record::data(out, 0, &OXIDE_EVENT, |w| {
                w.u64(5).u64(fields.event_time).u32(fields.param_key);
                w.u16(fields.param_index)
                    .u16(fields.mask)
                    .u16(fields.applied)
                    .u16(fields.unknown)
                    .u16(fields.dropped)
                    .u16(fields.torn);
                w.u8(fields.kind)
                    .u8(fields.code_a)
                    .u8(fields.code_b)
                    .u8(fields.why)
                    .u8(fields.why_detail);
                w.u8(invalid).u8(unreadable);
            })
            .unwrap();
        });
        let r = f.records("oxide_event").next().unwrap();
        assert_eq!(
            decode_event(&r),
            Err(EventError::NotCanonical),
            "{invalid} {unreadable}"
        );
    }
}
