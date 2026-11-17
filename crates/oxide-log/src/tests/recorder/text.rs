// SPDX-License-Identifier: AGPL-3.0-or-later
//! Step 3: the wording of an event and its level.

use crate::recorder::text::{TextBuf, decimal, event_level, event_text};
use crate::topics::bits;
use oxide_core::config::ConfigError;
use oxide_core::io::{
    Action, DisarmReason, Event, EventKind, FlightMode, NavState, PrearmFlags, Reasons,
};
use oxide_core::params::{self, LoadReport, ParamError, ParamId};
use oxide_core::time::Micros;

const EMERGENCY: u8 = 0;
const WARNING: u8 = 4;
const INFO: u8 = 6;

fn text(k: &EventKind) -> String {
    let mut b = TextBuf::new();
    event_text(k, &mut b);
    assert!(!b.truncated(), "{k:?} does not fit the text buffer");
    b.as_str().to_string()
}

fn param(name: &str) -> ParamId {
    params::find(name).unwrap()
}

fn report(applied: u16, unknown: u16, dropped: u16, torn: u16) -> LoadReport {
    LoadReport {
        applied,
        unknown,
        dropped,
        torn,
        invalid: false,
        unreadable: false,
    }
}

/// Every variant with the payloads worth pinning: text and ULog level.
fn pins() -> Vec<(EventKind, String, u8)> {
    let p = |k: EventKind, t: &str, l: u8| (k, t.to_string(), l);
    let refused = |why| EventKind::ParamRefused {
        id: param("LIM_TILT"),
        why,
    };
    vec![
        p(EventKind::Armed, "Armed", INFO),
        p(
            EventKind::Disarmed(DisarmReason::Switch),
            "Disarmed: switch",
            INFO,
        ),
        p(
            EventKind::Disarmed(DisarmReason::Kill),
            "Disarmed: kill",
            INFO,
        ),
        p(
            EventKind::Disarmed(DisarmReason::Touchdown),
            "Disarmed: touchdown",
            INFO,
        ),
        p(
            EventKind::Disarmed(DisarmReason::Terminated),
            "Disarmed: terminated",
            EMERGENCY,
        ),
        p(
            EventKind::ArmRefused(PrearmFlags::KILL.union(PrearmFlags::THROTTLE)),
            "Arming refused: KILL, THROTTLE",
            WARNING,
        ),
        p(
            EventKind::ArmRefused(PrearmFlags::empty()),
            "Arming refused",
            WARNING,
        ),
        p(
            EventKind::ArmRefused(PrearmFlags::ALL),
            "Arming refused: KILL, THROTTLE, RC, NOT_STILL, ESTIMATOR, POSITION, BATTERY, CONFIG, LINK",
            WARNING,
        ),
        p(
            EventKind::ArmRefused(PrearmFlags(1 << 12)),
            "Arming refused: BIT12",
            WARNING,
        ),
        p(
            EventKind::NavChanged {
                from: NavState::Disarmed,
                to: NavState::Stabilized,
            },
            "Nav state: DISARMED -> STABILIZED",
            INFO,
        ),
        p(
            EventKind::NavChanged {
                from: NavState::Position,
                to: NavState::LandNow,
            },
            "Nav state: POSITION -> LAND_NOW",
            INFO,
        ),
        p(
            EventKind::NavChanged {
                from: NavState::Land,
                to: NavState::Terminated,
            },
            "Nav state: LAND -> TERMINATED",
            EMERGENCY,
        ),
        p(
            EventKind::ModeDegraded {
                requested: FlightMode::Position,
                flown: NavState::Altitude,
            },
            "Mode degraded: POSITION flown as ALTITUDE",
            INFO,
        ),
        p(
            EventKind::ModeDegraded {
                requested: FlightMode::Offboard,
                flown: NavState::Hold,
            },
            "Mode degraded: OFFBOARD flown as HOLD",
            INFO,
        ),
        p(
            EventKind::ReasonRaised(Reasons::RC_LOST),
            "Reason raised: RC_LOST",
            WARNING,
        ),
        p(
            EventKind::ReasonRaised(Reasons::GPS_LOST.union(Reasons::RC_LOST)),
            "Reason raised: RC_LOST, GPS_LOST",
            WARNING,
        ),
        p(
            EventKind::ReasonRaised(Reasons::LOSS_OF_CONTROL),
            "Reason raised: LOSS_OF_CONTROL",
            EMERGENCY,
        ),
        p(
            EventKind::ReasonCleared(Reasons::BATTERY_LOW),
            "Reason cleared: BATTERY_LOW",
            INFO,
        ),
        p(
            EventKind::ReasonCleared(Reasons::LOSS_OF_CONTROL),
            "Reason cleared: LOSS_OF_CONTROL",
            INFO,
        ),
        p(
            EventKind::ActionChanged {
                from: Action::None,
                to: Action::Land,
            },
            "Action: LAND",
            INFO,
        ),
        p(
            EventKind::ActionChanged {
                from: Action::Land,
                to: Action::None,
            },
            "Action: NONE",
            INFO,
        ),
        p(
            EventKind::ActionChanged {
                from: Action::LandNow,
                to: Action::Terminate,
            },
            "Action: TERMINATE",
            EMERGENCY,
        ),
        p(
            EventKind::PilotOverride(Reasons::RC_LOST.union(Reasons::BATTERY_CRITICAL)),
            "Pilot override: RC_LOST, BATTERY_CRITICAL",
            INFO,
        ),
        p(EventKind::Touchdown, "Touchdown", INFO),
        p(EventKind::HomeSet, "Home set", INFO),
        p(
            EventKind::ParamChanged(param("RATE_ROLL_P")),
            "Parameter changed: RATE_ROLL_P",
            INFO,
        ),
        p(
            EventKind::ParamChanged(ParamId(999)),
            "Parameter changed: #999",
            INFO,
        ),
        p(
            refused(ParamError::OutOfRange),
            "Parameter refused: LIM_TILT (out of range)",
            WARNING,
        ),
        p(
            refused(ParamError::Unknown),
            "Parameter refused: LIM_TILT (unknown parameter)",
            WARNING,
        ),
        p(
            refused(ParamError::WrongType),
            "Parameter refused: LIM_TILT (wrong type)",
            WARNING,
        ),
        p(
            refused(ParamError::NotFinite),
            "Parameter refused: LIM_TILT (not finite)",
            WARNING,
        ),
        p(
            refused(ParamError::Armed),
            "Parameter refused: LIM_TILT (armed)",
            WARNING,
        ),
        p(
            refused(ParamError::Invalid(ConfigError::BadGains)),
            "Parameter refused: LIM_TILT (invalid combination, code 7)",
            WARNING,
        ),
        p(
            EventKind::ParamRefused {
                id: ParamId(65535),
                why: ParamError::Unknown,
            },
            "Parameter refused: #65535 (unknown parameter)",
            WARNING,
        ),
        p(
            EventKind::ParamLoadFault(report(90, 2, 1, 3)),
            "Parameter load fault: applied 90, unknown 2, dropped 1, torn 3",
            WARNING,
        ),
        p(
            EventKind::ParamLoadFault(LoadReport {
                invalid: true,
                unreadable: true,
                ..report(0, 0, 0, 0)
            }),
            "Parameter load fault: applied 0, unknown 0, dropped 0, torn 0, invalid, unreadable",
            WARNING,
        ),
        p(
            EventKind::ParamLoadRecovered(report(93, 0, 0, 2)),
            "Parameter load recovered: applied 93, torn 2",
            INFO,
        ),
        p(
            EventKind::ParamFaultAcked,
            "Parameter fault acknowledged",
            INFO,
        ),
    ]
}

/// Compile-time proof that `pins` thought of every variant: a new variant
/// makes this `match` fail to compile. Returns the variant's code.
fn variant_code(k: &EventKind) -> u8 {
    match k {
        EventKind::Armed
        | EventKind::Disarmed(_)
        | EventKind::ArmRefused(_)
        | EventKind::NavChanged { .. }
        | EventKind::ModeDegraded { .. }
        | EventKind::ReasonRaised(_)
        | EventKind::ReasonCleared(_)
        | EventKind::ActionChanged { .. }
        | EventKind::PilotOverride(_)
        | EventKind::Touchdown
        | EventKind::HomeSet
        | EventKind::ParamChanged(_)
        | EventKind::ParamRefused { .. }
        | EventKind::ParamLoadFault(_)
        | EventKind::ParamLoadRecovered(_)
        | EventKind::ParamFaultAcked => k.code(),
    }
}

#[test]
fn every_event_kind_has_its_text_and_its_level_pinned() {
    let all = pins();
    for (k, want, level) in &all {
        assert_eq!(&text(k), want, "{k:?}");
        assert_eq!(event_level(k), *level, "{k:?}");
    }
    // Every variant is in the table (codes 0..=15, each at least once).
    for code in 0..16 {
        assert!(
            all.iter().any(|(k, _, _)| variant_code(k) == code),
            "no pin for the event kind with code {code}"
        );
    }
}

#[test]
fn the_levels_are_ulog_digits() {
    for (k, _, _) in pins() {
        assert!(event_level(&k) <= 7);
    }
}

#[test]
fn flag_names_follow_the_bit_numbers() {
    let reason = |bit: u8| text(&EventKind::ReasonRaised(Reasons(1 << bit)));
    let named = [
        (bits::reasons::RC_LOST, "RC_LOST"),
        (bits::reasons::NO_HUMAN_LINK, "NO_HUMAN_LINK"),
        (bits::reasons::BRAIN_LOST, "BRAIN_LOST"),
        (bits::reasons::SETPOINT_STALE, "SETPOINT_STALE"),
        (bits::reasons::BATTERY_LOW, "BATTERY_LOW"),
        (bits::reasons::BATTERY_CRITICAL, "BATTERY_CRITICAL"),
        (bits::reasons::GPS_LOST, "GPS_LOST"),
        (bits::reasons::ESTIMATOR_UNHEALTHY, "ESTIMATOR_UNHEALTHY"),
        (bits::reasons::IMU_FAILURE, "IMU_FAILURE"),
        (bits::reasons::GEOFENCE, "GEOFENCE"),
        (bits::reasons::LOSS_OF_CONTROL, "LOSS_OF_CONTROL"),
    ];
    for (bit, name) in named {
        assert_eq!(reason(bit), format!("Reason raised: {name}"));
    }
    assert_eq!(named.len(), 11);
    let prearm = |bit: u8| text(&EventKind::ArmRefused(PrearmFlags(1 << bit)));
    let named = [
        (bits::prearm::KILL, "KILL"),
        (bits::prearm::THROTTLE, "THROTTLE"),
        (bits::prearm::RC, "RC"),
        (bits::prearm::NOT_STILL, "NOT_STILL"),
        (bits::prearm::ESTIMATOR, "ESTIMATOR"),
        (bits::prearm::POSITION, "POSITION"),
        (bits::prearm::BATTERY, "BATTERY"),
        (bits::prearm::CONFIG, "CONFIG"),
        (bits::prearm::LINK, "LINK"),
    ];
    for (bit, name) in named {
        assert_eq!(prearm(bit), format!("Arming refused: {name}"));
    }
}

#[test]
fn the_longest_texts_fit_the_buffer() {
    let kinds = [
        EventKind::ArmRefused(PrearmFlags(u16::MAX)),
        EventKind::ReasonRaised(Reasons(u16::MAX)),
        EventKind::PilotOverride(Reasons(u16::MAX)),
        EventKind::ParamLoadFault(LoadReport {
            applied: u16::MAX,
            unknown: u16::MAX,
            dropped: u16::MAX,
            torn: u16::MAX,
            invalid: true,
            unreadable: true,
        }),
        EventKind::ParamRefused {
            id: ParamId(u16::MAX),
            why: ParamError::Invalid(ConfigError::FilterCutoffTooHigh),
        },
    ];
    for k in kinds {
        let t = text(&k); // asserts it was not truncated
        assert!(t.is_ascii(), "{t}");
    }
    let longest = params::PARAMS.iter().map(|d| d.name.len()).max().unwrap();
    assert!(longest <= 16);
}

#[test]
fn every_parameter_name_reads_in_its_event() {
    for (i, def) in params::PARAMS.iter().enumerate() {
        let k = EventKind::ParamChanged(ParamId(i as u16));
        assert_eq!(text(&k), format!("Parameter changed: {}", def.name));
    }
}

#[test]
fn decimal_matches_the_standard_library() {
    let mut b = [0u8; 20];
    for v in [
        0u64,
        1,
        9,
        10,
        11,
        99,
        100,
        101,
        999,
        1000,
        65_535,
        65_536,
        u64::from(u32::MAX),
        u64::from(u32::MAX) + 1,
        9_999_999_999_999_999_999,
        10_000_000_000_000_000_000,
        u64::MAX - 1,
        u64::MAX,
    ] {
        assert_eq!(decimal(v, &mut b), v.to_string(), "{v}");
    }
}

proptest::proptest! {
    #[test]
    fn decimal_matches_the_standard_library_for_any_number(v in proptest::prelude::any::<u64>()) {
        let mut b = [0u8; 20];
        proptest::prop_assert_eq!(decimal(v, &mut b), v.to_string());
    }
}

#[test]
fn an_event_has_no_other_state() {
    // Text depends on the kind only, not on the event's time.
    let k = EventKind::Armed;
    let a = Event {
        t: Micros(1),
        kind: k,
    };
    let b = Event {
        t: Micros(u64::MAX),
        kind: k,
    };
    assert_eq!(text(&a.kind), text(&b.kind));
}
