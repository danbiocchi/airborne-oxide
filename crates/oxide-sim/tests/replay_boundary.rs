// SPDX-License-Identifier: AGPL-3.0-or-later
//! The boundary of the core outside its inputs (ADR-0005 §5, task P2): the
//! boot report of the stored-parameter load, and the store's "saved" call
//! between ticks. A flight booted from a flash with a record no parameter
//! owns (an unclean load), with the fault acknowledged, a parameter changed
//! on the ground and in flight, one change refused, and the store's save
//! after the disarm, replays identically. Taking away the saved flag, or the
//! unclean report, from the recorded file shows that both are needed.
//!
//! The setup is that of `param_store.rs`.

use oxide_core::io::{EventKind, ParamSet};
use oxide_core::params::{self, LoadReport, ParamId, ParamValue};
use oxide_core::schedule::{F405_LEAN, Profile};
use oxide_log::reader::UlogFile;
use oxide_sim::flight_log::{Level, LogBuild, SharedBuffer};
use oxide_sim::replay::{ReplayOptions, Stop, replay};
use oxide_sim::scenario::Script;
use oxide_sim::{Airframe, SimOptions, Simulation};
use oxide_store::sim::SimFlash;
use oxide_store::{ParamStore, Serviced, Store};

const PROFILE: Profile = F405_LEAN;
type Flash = SimFlash<1024>;
type Sim = Simulation<Script, 4>;

/// Arm at 2 s, climb, hover, land, disarm at 13 s, run on to 16 s.
fn script() -> Script {
    Script {
        arm_at_s: 2.0,
        disarm_at_s: Some(13.0),
        mode: vec![(0.0, 1)],
        sticks: vec![
            (0.0, [0.0, 0.0, 0.0, 0.0]),
            (2.5, [0.0, 0.0, 0.0, 1.0]),
            (4.0, [0.0, 0.0, 0.0, 0.5]),
            (8.0, [0.0, 0.0, 0.0, 0.0]),
        ],
        duration_s: 16.0,
        ..Script::default()
    }
}

fn live(id: ParamId, v: f32) -> ParamSet {
    ParamSet {
        id,
        value: ParamValue::F32(v),
    }
}

/// A flash holding one record whose key no parameter has: the load counts it
/// as unknown and the core holds a parameter fault until it is acknowledged.
fn flash_with_an_unknown_key() -> Flash {
    let mut unknown = 0x5a5a_0001_u32;
    while params::KEYS.contains(&unknown) {
        unknown += 1;
    }
    let mut store = Store::unmounted(Flash::new());
    store.remount().expect("a fresh flash mounts");
    store.set(unknown, 0x3f80_0000).expect("writes");
    store.into_flash()
}

/// What the flight did that the proofs need to know.
struct Flown {
    bytes: Vec<u8>,
    ticks: u64,
    report: LoadReport,
    /// Ticks (0-based) on which the store's save finished.
    saves: Vec<u64>,
    /// Tick of the first tick on which the core was disarmed after armed.
    disarmed_at: u64,
    changed: usize,
    refused: usize,
    fault_events: usize,
    acked: usize,
}

fn fly(flash: Flash) -> Flown {
    let airframe = Airframe::reference_quad().expect("reference airframe");
    let mut flash = Some(flash);
    let mut store = None;
    let mut report = LoadReport::default();
    let mut sim = Sim::with_boot(&airframe, PROFILE, script(), SimOptions::default(), |cfg| {
        let (s, r) = ParamStore::boot(flash.take().expect("once"), cfg, &PROFILE);
        store = Some(s);
        report = r;
        r
    })
    .expect("simulation");
    let mut store: ParamStore<Flash, 4> = store.expect("booted");
    let buf = SharedBuffer::new();
    sim.start_log(Level::Replay, &LogBuild::sim("test"), Box::new(buf.clone()))
        .expect("the replay level starts");
    // (time in s, change): accepted on the ground, accepted in flight,
    // refused in flight (out of range).
    let changes = [
        (1.5, live(ParamId::LIM_TILT, 0.6)),
        (8.0, live(ParamId::LIM_TILT, 0.5)),
        (9.0, live(ParamId::LIM_TILT, 5.0)),
    ];
    let mut next = 0;
    let mut acked_queued = false;
    let mut out = Flown {
        bytes: Vec::new(),
        ticks: 0,
        report,
        saves: Vec::new(),
        disarmed_at: 0,
        changed: 0,
        refused: 0,
        fault_events: 0,
        acked: 0,
    };
    let mut was_armed = false;
    while sim.time().as_secs_f64() < 16.0 {
        let t = sim.time().as_secs_f64();
        if !acked_queued && t >= 1.0 {
            sim.queue_param_ack();
            acked_queued = true;
        } else if let Some((at, ps)) = changes.get(next)
            && t >= *at
        {
            sim.queue_param_set(*ps);
            next += 1;
        }
        let rec = sim.step();
        for e in rec.events.iter() {
            match e.kind {
                EventKind::ParamChanged(_) => out.changed += 1,
                EventKind::ParamRefused { .. } => out.refused += 1,
                EventKind::ParamLoadFault(_) => out.fault_events += 1,
                EventKind::ParamFaultAcked => out.acked += 1,
                _ => {}
            }
        }
        let armed = sim.core().is_armed();
        if was_armed && !armed && out.disarmed_at == 0 {
            out.disarmed_at = out.ticks;
        }
        was_armed |= armed;
        if let Ok(Serviced::Saved(_)) = store.service(sim.core_mut()) {
            out.saves.push(out.ticks);
        }
        out.ticks += 1;
    }
    sim.finish_log().expect("finishes");
    out.bytes = buf.bytes();
    out
}

/// The flight from the flash with an unknown key, flown once for the tests.
fn unclean() -> &'static Flown {
    static FLOWN: std::sync::OnceLock<Flown> = std::sync::OnceLock::new();
    FLOWN.get_or_init(|| fly(flash_with_an_unknown_key()))
}

/// The offset in the file of field `field` of record `n` of `topic`.
fn field_at(bytes: &[u8], topic: &str, n: usize, field: &str) -> usize {
    let f = UlogFile::parse(bytes).unwrap();
    let r = f.records(topic).nth(n).unwrap();
    let inner = r.field_bytes(field).unwrap().as_ptr() as usize - r.raw().as_ptr() as usize;
    r.offset() + 5 + inner
}

/// The offset in the file of field `field` of every record of `topic`.
fn field_offsets(bytes: &[u8], topic: &str, field: &str) -> Vec<usize> {
    let f = UlogFile::parse(bytes).unwrap();
    f.records(topic)
        .map(|r| {
            let inner = r.field_bytes(field).unwrap().as_ptr() as usize - r.raw().as_ptr() as usize;
            r.offset() + 5 + inner
        })
        .collect()
}

/// The sequence numbers of the tick records that carry the saved flag.
fn saved_ticks(bytes: &[u8]) -> Vec<u64> {
    let f = UlogFile::parse(bytes).unwrap();
    f.records("oxide_in_tick")
        .filter(|r| r.bool("saved") == Some(true))
        .filter_map(|r| r.u64("seq"))
        .collect()
}

#[test]
fn a_flight_with_an_unclean_boot_an_ack_changes_a_refusal_and_saves_replays_identically() {
    let flown = unclean();
    // The flight is the one the proof is about.
    assert_eq!(flown.report.unknown, 1);
    assert!(!flown.report.clean());
    assert_eq!(flown.fault_events, 1, "the load fault is reported once");
    assert_eq!(flown.acked, 1, "and acknowledged");
    assert_eq!(flown.changed, 2, "two changes accepted");
    assert_eq!(flown.refused, 1, "one refused");
    assert!(flown.disarmed_at > 12_000, "{}", flown.disarmed_at);
    let last = *flown.saves.last().expect("a save");
    assert!(
        last >= flown.disarmed_at,
        "saved on the first disarmed tick or later: {last} < {}",
        flown.disarmed_at
    );
    // Every save is on the next tick record, and on no other.
    let marked = saved_ticks(&flown.bytes);
    let want: Vec<u64> = flown.saves.iter().map(|s| s + 1).collect();
    assert_eq!(marked, want);
    assert!(marked.len() >= 2, "{marked:?}");

    let r = replay(&flown.bytes, &ReplayOptions::default()).expect("replays");
    assert!(r.identical(), "{:?}", r.first_difference);
    assert_eq!(r.stop, Stop::End);
    assert_eq!(r.ticks, flown.ticks);
}

#[test]
fn the_same_log_replayed_without_the_saved_flag_differs_at_the_first_save() {
    let flown = unclean();
    let marked = saved_ticks(&flown.bytes);
    let mut bytes = flown.bytes.clone();
    for at in field_offsets(&flown.bytes, "oxide_in_tick", "saved") {
        bytes[at] = 0;
    }
    assert!(saved_ticks(&bytes).is_empty());
    let r = replay(&bytes, &ReplayOptions::default()).expect("replays");
    let d = r.first_difference.as_ref().expect("a difference");
    assert_eq!(
        (d.seq, d.field.as_str()),
        (marked[0], "params_unsaved"),
        "{d}"
    );
    assert_eq!((&d.recorded[..], &d.replayed[..]), ("false", "true"));
    assert_eq!(r.exit_code(), 1);
}

#[test]
fn the_same_log_replayed_with_a_clean_boot_report_differs_at_the_first_tick() {
    let flown = unclean();
    let mut bytes = flown.bytes.clone();
    for (field, len) in [
        ("applied", 2),
        ("unknown", 2),
        ("dropped", 2),
        ("torn", 2),
        ("invalid", 1),
        ("unreadable", 1),
    ] {
        let at = field_at(&flown.bytes, "oxide_boot", 0, field);
        bytes[at..at + len].fill(0);
    }
    let boot = UlogFile::parse(&bytes).unwrap();
    let b = boot.records("oxide_boot").next().unwrap();
    assert_eq!(b.u16("unknown"), Some(0));
    drop(boot);
    let r = replay(&bytes, &ReplayOptions::default()).expect("replays");
    let d = r.first_difference.as_ref().expect("a difference");
    assert_eq!(d.seq, 0, "{d}");
    assert_eq!(r.stop, Stop::Difference);
    // And the recorded report is what made it identical.
    assert!(
        replay(&flown.bytes, &ReplayOptions::default())
            .expect("replays")
            .identical()
    );
}

#[test]
fn a_clean_boot_flight_has_no_fault_and_replays_identically_too() {
    // The baseline of the two proofs above: the same flight from a fresh
    // flash (the ack and the changes still happen).
    let flown = fly(Flash::new());
    assert!(flown.report.clean());
    assert_eq!(flown.fault_events, 0);
    let r = replay(&flown.bytes, &ReplayOptions::default()).expect("replays");
    assert!(r.identical(), "{:?}", r.first_difference);
    assert_eq!(r.ticks, flown.ticks);
}
