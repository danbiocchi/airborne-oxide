// SPDX-License-Identifier: AGPL-3.0-or-later
//! Step 3: events and parameter changes in the log.

use super::rates::{Tick, run};
use crate::msg::ParamValue;
use crate::reader::{Item, UlogFile};
use crate::topics::decode_event;
use oxide_core::io::{DisarmReason, Event, EventKind, NavState, Reasons};
use oxide_core::params::{self, LoadReport};
use oxide_core::time::Micros;

fn ev(t: u64, kind: EventKind) -> Event {
    Event { t: Micros(t), kind }
}

/// The log's items of interest, in file order: texts, event records, `P`s.
fn story(f: &UlogFile) -> Vec<String> {
    let mut out = Vec::new();
    for item in &f.items {
        match item {
            Item::Log {
                level,
                time_us,
                text,
                ..
            } => out.push(format!("L{level} {time_us} {text}")),
            Item::Data(d) if f.subscriptions[d.subscription].name == "oxide_event" => {
                let v = f.view(d);
                let e = decode_event(&v).unwrap();
                out.push(format!("D {} {:?}", v.u64("timestamp").unwrap(), e));
            }
            Item::Param { name, value } => out.push(format!("P {name} {value:?}")),
            _ => {}
        }
    }
    out
}

#[test]
fn an_event_is_text_then_data_on_the_tick_it_happened() {
    let (f, rec, _) = run(1000, 50_000, |t| {
        if t.now == 20_000 {
            assert!(t.outputs.events.push(ev(20_000, EventKind::Armed)));
        }
        if t.now == 30_000 {
            assert!(
                t.outputs
                    .events
                    .push(ev(30_000, EventKind::Disarmed(DisarmReason::Touchdown)))
            );
        }
    });
    assert_eq!(
        story(&f),
        [
            "L6 20000 Armed".to_string(),
            format!("D 20000 {:?}", ev(20_000, EventKind::Armed)),
            "L6 30000 Disarmed: touchdown".to_string(),
            format!(
                "D 30000 {:?}",
                ev(30_000, EventKind::Disarmed(DisarmReason::Touchdown))
            ),
        ]
    );
    assert_eq!(f.records("oxide_event").count(), 2);
    assert_eq!(rec.stats().dropped_must, 0);
}

#[test]
fn several_events_in_a_tick_keep_their_order_text_before_data_each() {
    let a = ev(5_000, EventKind::ReasonRaised(Reasons::RC_LOST));
    let b = ev(
        5_000,
        EventKind::ActionChanged {
            from: oxide_core::io::Action::None,
            to: oxide_core::io::Action::Land,
        },
    );
    let c = ev(
        5_000,
        EventKind::NavChanged {
            from: NavState::Position,
            to: NavState::Land,
        },
    );
    let (f, _, _) = run(1000, 10_000, |t| {
        if t.now == 5_000 {
            for e in [a, b, c] {
                assert!(t.outputs.events.push(e));
            }
        }
    });
    assert_eq!(
        story(&f),
        [
            "L4 5000 Reason raised: RC_LOST".to_string(),
            format!("D 5000 {a:?}"),
            "L6 5000 Action: LAND".to_string(),
            format!("D 5000 {b:?}"),
            "L6 5000 Nav state: POSITION -> LAND".to_string(),
            format!("D 5000 {c:?}"),
        ]
    );
}

#[test]
fn the_text_carries_the_event_time_and_the_record_the_tick_time() {
    // An event the core held back from an earlier tick keeps its own time.
    let e = ev(3_000, EventKind::Touchdown);
    let (f, _, _) = run(1000, 10_000, |t| {
        if t.now == 8_000 {
            assert!(t.outputs.events.push(e));
        }
    });
    let log = f
        .items
        .iter()
        .find_map(|i| match i {
            Item::Log { time_us, .. } => Some(*time_us),
            _ => None,
        })
        .unwrap();
    assert_eq!(log, 3_000);
    let r = f.records("oxide_event").next().unwrap();
    assert_eq!(r.u64("timestamp"), Some(8_000));
    assert_eq!(decode_event(&r).unwrap(), e);
}

#[test]
fn a_parameter_change_is_a_p_message_with_the_exact_bits() {
    let mut cfg_after = super::super::topics::quad();
    let profile = oxide_core::schedule::F405_LEAN;
    let rate = params::find("RATE_ROLL_P").unwrap();
    let source = params::find("EKF_SOURCE").unwrap();
    let odd = f32::from_bits(0x4190_0001);
    params::set(&mut cfg_after, &profile, rate, params::ParamValue::F32(odd)).unwrap();
    params::set(&mut cfg_after, &profile, source, params::ParamValue::I32(1)).unwrap();

    let mut file = Vec::new();
    let (mut rec, _cfg) = super::started(&mut file, crate::header::Level::Normal);
    let mut tick = Tick::at(7_000);
    for id in [rate, source] {
        assert!(
            tick.outputs
                .events
                .push(ev(7_000, EventKind::ParamChanged(id)))
        );
        // One tick holds one event of this kind in practice; two here to
        // see both value types.
    }
    rec.record(
        Micros(7_000),
        &tick.inputs,
        &tick.outputs,
        &tick.tel,
        &cfg_after,
    );
    rec.drain(&mut file);
    let f = UlogFile::parse(&file).unwrap();
    let items = story(&f);
    assert_eq!(items.len(), 6, "{items:?}");
    assert!(items[0].starts_with("L6 7000 Parameter changed: RATE_ROLL_P"));
    assert!(items[1].starts_with("D 7000"));
    let ps: Vec<(&String, &ParamValue)> = f
        .items
        .iter()
        .filter_map(|i| match i {
            Item::Param { name, value } => Some((name, value)),
            _ => None,
        })
        .collect();
    assert_eq!(ps.len(), 2);
    assert_eq!(ps[0].0, "RATE_ROLL_P");
    match ps[0].1 {
        ParamValue::F32(v) => assert_eq!(v.to_bits(), odd.to_bits()),
        other => panic!("{other:?}"),
    }
    assert_eq!(ps[1].0, "EKF_SOURCE");
    assert_eq!(*ps[1].1, ParamValue::I32(1));
    // text, data, P for each event.
    assert!(items[2].starts_with("P RATE_ROLL_P"));
}

#[test]
fn a_refused_parameter_writes_no_p() {
    let refused = EventKind::ParamRefused {
        id: params::find("LIM_TILT").unwrap(),
        why: oxide_core::params::ParamError::OutOfRange,
    };
    let (f, _, _) = run(1000, 5_000, |t| {
        if t.now == 2_000 {
            assert!(t.outputs.events.push(ev(2_000, refused)));
        }
    });
    assert_eq!(
        f.items
            .iter()
            .filter(|i| matches!(i, Item::Param { .. }))
            .count(),
        0
    );
    assert_eq!(
        story(&f)[0],
        "L4 2000 Parameter refused: LIM_TILT (out of range)"
    );
}

#[test]
fn every_event_decodes_back_from_the_log_exactly() {
    let load = LoadReport {
        applied: 80,
        unknown: 1,
        dropped: 2,
        torn: 3,
        invalid: true,
        unreadable: false,
    };
    let kinds = [
        EventKind::Armed,
        EventKind::ParamLoadFault(load),
        EventKind::ParamLoadRecovered(load),
        EventKind::ParamFaultAcked,
    ];
    let (f, _, _) = run(1000, 6_000, |t| {
        if t.now == 1_000 {
            for k in kinds {
                t.outputs.events.push(ev(1_000, k));
            }
        }
    });
    let got: Vec<Event> = f
        .records("oxide_event")
        .map(|r| decode_event(&r).unwrap())
        .collect();
    let want: Vec<Event> = kinds.iter().map(|k| ev(1_000, *k)).collect();
    assert_eq!(got, want);
}
