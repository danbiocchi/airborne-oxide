// SPDX-License-Identifier: AGPL-3.0-or-later
//! Step 4: a full ring, a whole log, pyulog, a real core, sizes.

use super::rates::{Tick, run, times};
use super::{BUILD, started};
use crate::header::Level;
use crate::reader::{Item, UlogFile};
use crate::recorder::{Boot, Recorder};
use crate::topics::decode_event;
use oxide_core::config::VehicleConfig;
use oxide_core::flight::Core;
use oxide_core::io::{
    DisarmReason, Event, EventKind, Inputs, ParamSet, Reasons, Stamped, StateSample,
};
use oxide_core::math::{Quat, Vec3};
use oxide_core::params::{self, LoadReport};
use oxide_core::schedule::F405_LEAN;
use oxide_core::time::Micros;
use std::process::Command;

fn ev(t: u64, kind: EventKind) -> Event {
    Event { t: Micros(t), kind }
}

#[test]
fn a_full_ring_drops_the_stream_and_keeps_events_and_status() {
    const SMALL: usize = 4096;
    let cfg = super::super::topics::quad();
    let mut file = Vec::new();
    let mut rec = Recorder::<SMALL>::new();
    rec.start(
        &mut file,
        Micros(0),
        BUILD,
        Boot::clean(&F405_LEAN),
        Level::Normal,
        &cfg,
    )
    .unwrap();
    let events = [
        (250_000, EventKind::Armed),
        (260_000, EventKind::ReasonRaised(Reasons::RC_LOST)),
        (270_000, EventKind::Disarmed(DisarmReason::Switch)),
    ];
    // 300 ms at 1 kHz, nothing drains the ring.
    for i in 0..300u64 {
        let t = i * 1000;
        let mut tick = Tick::at(t);
        if t >= 200_000 {
            tick.outputs.status.armed = true; // one status change
        }
        for (at, k) in events {
            if t == at {
                assert!(tick.outputs.events.push(ev(t, k)));
            }
        }
        rec.record(Micros(t), &tick.inputs, &tick.outputs, &tick.tel, &cfg);
    }
    let st = rec.stats();
    assert!(st.dropped_stream > 0, "the ring never filled: {st:?}");
    assert_eq!(st.dropped_must, 0, "{st:?}");
    assert!(st.high_water <= SMALL);

    // Drain it all and look at what is in the log.
    assert!(rec.drain(&mut file) > 0);
    let before = file.len();
    let f = UlogFile::parse(&file).unwrap();
    let got: Vec<Event> = f
        .records("oxide_event")
        .map(|r| decode_event(&r).unwrap())
        .collect();
    let want: Vec<Event> = events.iter().map(|(t, k)| ev(*t, *k)).collect();
    assert_eq!(got, want, "every event survived");
    let armed: Vec<_> = f
        .records("vehicle_status")
        .map(|r| (r.u64("timestamp").unwrap(), r.u8("arming_state").unwrap()))
        .collect();
    assert!(
        armed.contains(&(200_000, 2)),
        "the status change survived: {armed:?}"
    );
    let texts = f
        .items
        .iter()
        .filter(|i| matches!(i, Item::Log { .. }))
        .count();
    assert_eq!(texts, 3);
    let gaps_before = f
        .items
        .iter()
        .filter(|i| matches!(i, Item::Dropout { .. }))
        .count();
    assert!(
        gaps_before >= 1,
        "a Must record after a dropped one carries the dropout"
    );

    // The stream stopped short of the reserve: its records are fewer than a
    // full run's.
    let full = run(1000, 300_000, |_| {}).0;
    assert!(f.records("sensor_combined").count() < full.records("sensor_combined").count());

    // Room is back. The next tick's first message is preceded by exactly one
    // dropout, and then the log goes on without further drops.
    let mut tick = Tick::at(300_000);
    tick.outputs.status.armed = true;
    rec.record(
        Micros(300_000),
        &tick.inputs,
        &tick.outputs,
        &tick.tel,
        &cfg,
    );
    rec.drain(&mut file);
    let f = UlogFile::parse(&file).unwrap();
    let gaps_after = f
        .items
        .iter()
        .filter(|i| matches!(i, Item::Dropout { .. }))
        .count();
    assert_eq!(gaps_after, gaps_before + 1);
    assert!(file.len() > before);
    let last_gap = f
        .items
        .iter()
        .rposition(|i| matches!(i, Item::Dropout { .. }))
        .unwrap();
    match &f.items[last_gap + 1] {
        Item::Data(d) => assert_eq!(f.view(d).u64("timestamp"), Some(300_000)),
        other => panic!("after the dropout: {other:?}"),
    }

    // oxide_logger says what was dropped, at its next second.
    let drops = rec.stats().dropped_stream;
    for i in 301..1100u64 {
        let t = i * 1000;
        let mut tick = Tick::at(t);
        tick.outputs.status.armed = true;
        rec.record(Micros(t), &tick.inputs, &tick.outputs, &tick.tel, &cfg);
        rec.drain(&mut file);
    }
    assert_eq!(rec.stats().dropped_stream, drops, "no drops with draining");
    let f = UlogFile::parse(&file).unwrap();
    let last = f.records("oxide_logger").last().unwrap();
    assert_eq!(last.u64("timestamp"), Some(1_000_000));
    assert_eq!(last.u32("dropped_stream"), Some(drops));
    assert_eq!(last.u32("dropped_must"), Some(0));
    assert!(last.u32("high_water").unwrap() as usize <= SMALL);
    assert!(last.u32("dropouts").unwrap() >= 1);
}

#[test]
fn a_log_that_the_ring_cannot_hold_still_parses() {
    // A ring that holds a few messages, drained a little at a time.
    const TINY: usize = 1024;
    let cfg = super::super::topics::quad();
    let mut file = Vec::new();
    let mut rec = Recorder::<TINY>::new();
    rec.start(
        &mut file,
        Micros(0),
        BUILD,
        Boot::clean(&F405_LEAN),
        Level::Normal,
        &cfg,
    )
    .unwrap();
    let mut slow = Vec::new();
    for i in 0..2000u64 {
        let t = i * 1000;
        let tick = Tick::at(t);
        rec.record(Micros(t), &tick.inputs, &tick.outputs, &tick.tel, &cfg);
        if i % 7 == 0 {
            // A sink that takes at most 100 bytes at a time.
            rec.drain_with(|chunk| {
                let n = chunk.len().min(100);
                slow.extend_from_slice(&chunk[..n]);
                n
            });
        }
    }
    while rec.drain_with(|c| {
        slow.extend_from_slice(c);
        c.len()
    }) > 0
    {}
    file.extend_from_slice(&slow);
    let f = UlogFile::parse(&file).unwrap();
    assert!(f.truncated.is_none());
    let st = rec.stats();
    assert!(st.dropped_stream > 0);
    assert!(st.dropouts > 0);
}

// ---- a scripted run, whole ----

fn script() -> (Vec<u8>, Recorder<{ super::CAP }>) {
    let (_, rec, file) = run(1000, 3_000_000, |t| {
        if t.now == 500_000 {
            t.outputs.events.push(ev(t.now, EventKind::Armed));
            t.outputs.status.armed = true;
        }
        if t.now >= 500_000 {
            t.outputs.status.armed = true;
        }
        if t.now == 1_200_000 {
            t.outputs
                .events
                .push(ev(t.now, EventKind::ReasonRaised(Reasons::BATTERY_LOW)));
        }
        if t.now >= 1_200_000 {
            t.outputs.status.reasons = Reasons::BATTERY_LOW;
            t.tel.battery_low = true;
        }
        if t.now == 2_000_000 {
            t.outputs.events.push(ev(
                t.now,
                EventKind::ParamLoadFault(LoadReport {
                    applied: 90,
                    unknown: 3,
                    ..LoadReport::default()
                }),
            ));
        }
    });
    (file, rec)
}

#[test]
fn the_whole_output_of_a_scripted_run_parses() {
    let (file, rec) = script();
    let f = UlogFile::parse(&file).unwrap();
    assert!(f.truncated.is_none());
    assert_eq!(f.records("oxide_event").count(), 3);
    assert!(f.items.iter().any(|i| matches!(i, Item::Log { .. })));
    // Syncs every 500 ms.
    assert!(rec.stats().syncs >= 5);
    assert_eq!(
        f.items.iter().filter(|i| matches!(i, Item::Sync)).count() as u32,
        rec.stats().syncs
    );
    assert_eq!(times(&f, "oxide_logger"), [0, 1_000_000, 2_000_000]);
}

fn python() -> Option<String> {
    let Ok(py) = std::env::var("OXIDE_PYTHON") else {
        println!("NOTICE: OXIDE_PYTHON is not set: the pyulog cross-check is skipped");
        return None;
    };
    let ok = Command::new(&py)
        .args(["-c", "import pyulog"])
        .output()
        .is_ok_and(|o| o.status.success());
    if !ok {
        println!("NOTICE: {py} cannot import pyulog: the pyulog cross-check is skipped");
        return None;
    }
    Some(py)
}

fn check(py: &str, args: &[&str]) -> (bool, String, String) {
    let script = concat!(env!("CARGO_MANIFEST_DIR"), "/../../tools/ulog_check.py");
    let o = Command::new(py).arg(script).args(args).output().unwrap();
    (
        o.status.success(),
        String::from_utf8(o.stdout).unwrap(),
        String::from_utf8(o.stderr).unwrap(),
    )
}

#[test]
fn pyulog_reads_a_scripted_run_without_corruption() {
    let Some(py) = python() else { return };
    let (file, _) = script();
    let path = std::env::temp_dir().join(format!("oxide-log-recorder-{}.ulg", std::process::id()));
    std::fs::write(&path, &file).unwrap();
    let p = path.to_str().unwrap();
    let (ok, out, err) = check(&py, &[p]);
    assert!(ok, "ulog_check.py failed: {err}\n{out}");
    for (topic, n) in [
        ("sensor_combined", 300),
        ("vehicle_attitude", 150),
        ("vehicle_local_position", 60),
        ("manual_control_setpoint", 30),
        ("battery_status", 15),
        ("oxide_event", 3),
        ("oxide_logger", 3),
    ] {
        assert!(
            out.contains(&format!("topic {topic} 0 {n}\n")),
            "{topic}: {out}"
        );
    }
    let (ok, meta, err) = check(&py, &[p, "--meta"]);
    assert!(ok, "{err}");
    assert!(meta.contains("log 500000 INFO Armed"), "{meta}");
    assert!(
        meta.contains("log 1200000 WARNING Reason raised: BATTERY_LOW"),
        "{meta}"
    );
    assert!(
        meta.contains("WARNING Parameter load fault: applied 90, unknown 3, dropped 0, torn 0"),
        "{meta}"
    );
    assert!(meta.lines().any(|l| l.starts_with("sync ")), "{meta}");
    std::fs::remove_file(&path).ok();
}

// ---- a real core ----

#[test]
fn a_real_core_driven_for_a_few_hundred_ticks_records_a_parseable_log() {
    let cfg: VehicleConfig<4> = super::super::topics::quad();
    let mut core = Core::<4>::new(cfg, F405_LEAN).unwrap();
    let report = LoadReport {
        applied: 90,
        unknown: 3,
        ..LoadReport::default()
    };
    core.report_param_load(report);
    let mut file = Vec::new();
    let mut rec = Recorder::<{ 32 * 1024 }>::new();
    rec.start(
        &mut file,
        Micros(0),
        BUILD,
        Boot {
            profile: &F405_LEAN,
            report,
        },
        Level::Replay,
        core.config(),
    )
    .unwrap();
    let rate = params::find("RATE_ROLL_P").unwrap();
    for i in 0..400u64 {
        let t = i * 1000;
        let mut inputs: Inputs = super::inputs_at(t);
        inputs.state = Some(Stamped::ideal(
            Micros(t),
            StateSample {
                position: Vec3::new(0.0, 0.0, 0.0),
                velocity: Vec3::new(0.0, 0.0, 0.0),
                attitude: Quat::identity(),
                rate: Vec3::new(0.0, 0.0, 0.0),
            },
        ));
        if i == 100 {
            inputs.param_set = Some(ParamSet {
                id: rate,
                value: params::ParamValue::F32(31.5),
            });
        }
        let out = core.tick(Micros(t), &inputs);
        rec.record(Micros(t), &inputs, &out, &core.telemetry(), core.config());
        rec.drain(&mut file);
    }
    let f = UlogFile::parse(&file).unwrap();
    assert!(f.truncated.is_none());
    assert_eq!(f.info_str("oxide_log_level"), Some("replay"));
    assert_eq!(f.records("sensor_combined").count(), 40);
    assert!(f.records("vehicle_attitude").count() > 0);
    let kinds: Vec<u8> = f
        .records("oxide_event")
        .map(|r| r.u8("kind").unwrap())
        .collect();
    assert!(kinds.contains(&13), "the load fault: {kinds:?}");
    assert!(kinds.contains(&11), "the parameter change: {kinds:?}");
    let p = f.items.iter().find_map(|i| match i {
        Item::Param { name, value } => Some((name.clone(), *value)),
        _ => None,
    });
    assert_eq!(
        p,
        Some(("RATE_ROLL_P".to_string(), crate::msg::ParamValue::F32(31.5)))
    );
    assert_eq!(rec.stats().dropped_stream + rec.stats().dropped_must, 0);
}

// ---- sizes and rates of bytes ----

#[test]
fn the_recorder_is_the_ring_plus_a_small_state() {
    let overhead = |size: usize, cap: usize| size - cap;
    let a = overhead(std::mem::size_of::<Recorder<4096>>(), 4096);
    let b = overhead(std::mem::size_of::<Recorder<16384>>(), 16384);
    let c = overhead(std::mem::size_of::<Recorder<65536>>(), 65536);
    println!(
        "size_of Recorder: 4096 -> {}, 16384 -> {}, 65536 -> {}",
        std::mem::size_of::<Recorder<4096>>(),
        std::mem::size_of::<Recorder<16384>>(),
        std::mem::size_of::<Recorder<65536>>()
    );
    assert_eq!(a, b);
    assert_eq!(b, c);
    assert!(a < 2048, "{a}");
}

#[test]
fn bytes_per_second_at_1_khz_and_8_khz() {
    let mut rates = Vec::new();
    for dt in [1000u64, 125] {
        let secs = 4;
        let mut head = Vec::new();
        started(&mut head, Level::Normal);
        let (_, _, file) = run(dt, secs * 1_000_000, |_| {});
        let bps = (file.len() - head.len()) as f64 / secs as f64;
        println!("tick {dt} us: {bps:.0} bytes/s");
        rates.push(bps);
    }
    // The same flight at either tick rate writes the same records.
    assert_eq!(rates[0], rates[1]);
    assert!(rates[0] > 40_000.0 && rates[0] < 41_000.0, "{rates:?}");
}
