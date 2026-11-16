// SPDX-License-Identifier: AGPL-3.0-or-later
//! The chain simulator -> file -> reader -> pyulog (ADR-0005 §9): what a log
//! holds equals what the simulator knew at the same tick, and `pyulog`, the
//! strictest reader, reads the same bits as the crate's own reader.
//!
//! The pyulog tests are skipped, with a printed notice, unless the
//! environment variable `OXIDE_PYTHON` names a Python that has `pyulog`.

use std::collections::BTreeMap;
use std::process::Command;

use oxide_core::codes::{px4_arming_state, px4_nav_state};
use oxide_core::io::{Action, BatterySample, NavState, StateSample, Status};
use oxide_core::schedule::{F405_LEAN, H743_FULL, Profile};
use oxide_log::format::{FieldType, Layout, Scalar};
use oxide_log::reader::{RecordView, UlogFile};
use oxide_log::topics::NORMAL;
use oxide_sim::fault_scenarios;
use oxide_sim::flight_log::{Level, LogBuild, SharedBuffer};

const BOTH: [Profile; 2] = [F405_LEAN, H743_FULL];

/// What the simulator knew at one tick.
struct Tick {
    estimate: Option<StateSample>,
    motors: [f32; 4],
    status: Status,
    /// The newest battery sample up to this tick.
    battery: Option<BatterySample>,
}

/// Flies `name` with a normal-level log; returns the file and what the
/// simulator knew at each tick time.
fn fly(name: &str, profile: Profile) -> (Vec<u8>, BTreeMap<u64, Tick>) {
    let scenario = fault_scenarios::by_name(name).expect("scenario");
    let mut sim = scenario.build(profile, 0).expect("builds");
    let buf = SharedBuffer::new();
    sim.start_log(Level::Normal, &LogBuild::sim("hash"), Box::new(buf.clone()))
        .expect("log starts");
    let mut ticks = BTreeMap::new();
    let mut battery = None;
    while sim.time() < scenario.end() {
        let r = sim.step();
        if let Some(b) = sim.last_inputs().battery {
            battery = Some(b.value);
        }
        ticks.insert(
            r.t.0,
            Tick {
                estimate: sim.core().telemetry().estimate,
                motors: r.motors,
                status: r.status,
                battery,
            },
        );
    }
    sim.finish_log().expect("finishes");
    (buf.bytes(), ticks)
}

fn bits(v: f32) -> u32 {
    v.to_bits()
}

#[test]
fn the_reader_gives_the_values_the_simulator_had_at_the_same_ticks() {
    for profile in BOTH {
        for name in ["rc-loss-position", "gust-hold"] {
            let (bytes, ticks) = fly(name, profile);
            let file = UlogFile::parse(&bytes).expect("parses");
            let at = |r: &RecordView<'_>| {
                let t = r.u64("timestamp").expect("timestamp");
                ticks.get(&t).unwrap_or_else(|| panic!("no tick at {t}"))
            };
            let ctx = format!("{name} on {}", profile.name);

            let mut n = 0;
            for r in file.records("vehicle_attitude") {
                let e = at(&r).estimate.expect("an estimate").attitude;
                let q = [e.w, e.x, e.y, e.z];
                for (i, want) in q.iter().enumerate() {
                    assert_eq!(
                        bits(r.f32(&format!("q[{i}]")).unwrap()),
                        bits(*want),
                        "{ctx}"
                    );
                }
                n += 1;
            }
            assert!(n > 100, "{ctx}: {n} attitude records");

            n = 0;
            for r in file.records("vehicle_local_position") {
                let e = at(&r).estimate.expect("an estimate");
                for (field, want) in [
                    ("x", e.position.x),
                    ("y", e.position.y),
                    ("z", e.position.z),
                    ("vx", e.velocity.x),
                    ("vy", e.velocity.y),
                    ("vz", e.velocity.z),
                    ("z_deriv", e.velocity.z),
                    ("heading", e.attitude.yaw()),
                ] {
                    assert_eq!(bits(r.f32(field).unwrap()), bits(want), "{ctx} {field}");
                }
                n += 1;
            }
            assert!(n > 20, "{ctx}: {n} position records");

            n = 0;
            for r in file.records("actuator_motors") {
                let m = at(&r).motors;
                for (i, want) in m.iter().enumerate() {
                    assert_eq!(
                        bits(r.f32(&format!("control[{i}]")).unwrap()),
                        bits(*want),
                        "{ctx} motor {i}"
                    );
                }
                for i in 4..12 {
                    assert!(r.f32(&format!("control[{i}]")).unwrap().is_nan(), "{ctx}");
                }
                n += 1;
            }
            assert!(n > 1000, "{ctx}: {n} motor records");

            n = 0;
            for r in file.records("battery_status") {
                let b = at(&r).battery.expect("a battery sample");
                assert_eq!(
                    bits(r.f32("voltage_v").unwrap()),
                    bits(b.voltage_v),
                    "{ctx}"
                );
                assert_eq!(
                    bits(r.f32("current_a").unwrap()),
                    bits(b.current_a),
                    "{ctx}"
                );
                n += 1;
            }
            assert!(n > 50, "{ctx}: {n} battery records");

            n = 0;
            for r in file.records("vehicle_status") {
                let s = &at(&r).status;
                assert_eq!(
                    r.u8("arming_state"),
                    Some(px4_arming_state(s.armed)),
                    "{ctx}"
                );
                assert_eq!(
                    r.u8("nav_state"),
                    Some(px4_nav_state(s.nav, s.mode)),
                    "{ctx}"
                );
                assert_eq!(
                    r.u8("nav_state_display"),
                    Some(px4_nav_state(s.nav, s.mode))
                );
                assert_eq!(
                    r.u8("nav_state_user_intention"),
                    Some(px4_nav_state(NavState::Disarmed, s.mode)),
                    "{ctx}"
                );
                assert_eq!(r.bool("failsafe"), Some(s.action != Action::None), "{ctx}");
                n += 1;
            }
            assert!(n >= 3, "{ctx}: {n} status records");
        }
    }
}

// ---- pyulog ----

/// The columns `pyulog` makes of a record: `(name, type)`, flattened, the
/// trailing padding left out.
fn columns(
    file: &UlogFile,
    layout: &Layout,
    prefix: &str,
    top: bool,
    out: &mut Vec<(String, String)>,
) {
    let trailing = if top {
        layout
            .fields
            .iter()
            .rev()
            .take_while(|f| f.is_padding())
            .count()
    } else {
        0
    };
    let last = layout.fields.len() - trailing;
    for f in &layout.fields[..last] {
        let n = f.array.unwrap_or(0);
        let names: Vec<String> = if n > 0 {
            (0..n).map(|i| format!("{prefix}{}[{i}]", f.name)).collect()
        } else {
            vec![format!("{prefix}{}", f.name)]
        };
        for name in names {
            match &f.ty {
                FieldType::Scalar(s) => {
                    let t = match s {
                        Scalar::I8 => "int8_t",
                        Scalar::U8 => "uint8_t",
                        Scalar::I16 => "int16_t",
                        Scalar::U16 => "uint16_t",
                        Scalar::I32 => "int32_t",
                        Scalar::U32 => "uint32_t",
                        Scalar::I64 => "int64_t",
                        Scalar::U64 => "uint64_t",
                        Scalar::F32 => "float",
                        Scalar::F64 => "double",
                        Scalar::Bool => "bool",
                        Scalar::Char => "char",
                    };
                    out.push((name, t.to_string()));
                }
                FieldType::Nested(inner) => columns(
                    file,
                    file.layout(inner).unwrap(),
                    &format!("{name}."),
                    false,
                    out,
                ),
            }
        }
    }
}

fn cell(r: &RecordView<'_>, name: &str, ty: &str) -> String {
    match ty {
        "float" => format!("0x{:08x}", r.f32(name).unwrap().to_bits()),
        "double" => format!("0x{:016x}", r.f64(name).unwrap().to_bits()),
        "uint8_t" => r.u8(name).unwrap().to_string(),
        "int8_t" => r.i8(name).unwrap().to_string(),
        "uint16_t" => r.u16(name).unwrap().to_string(),
        "int16_t" => r.i16(name).unwrap().to_string(),
        "uint32_t" => r.u32(name).unwrap().to_string(),
        "int32_t" => r.i32(name).unwrap().to_string(),
        "uint64_t" => r.u64(name).unwrap().to_string(),
        "int64_t" => r.i64(name).unwrap().to_string(),
        "bool" => i8::from(r.bool(name).unwrap()).to_string(),
        "char" => (r.char(name).unwrap() as i8).to_string(),
        other => panic!("{other}"),
    }
}

fn python() -> Option<String> {
    let Ok(py) = std::env::var("OXIDE_PYTHON") else {
        println!("NOTICE: OXIDE_PYTHON is not set: the pyulog checks are skipped");
        return None;
    };
    let ok = Command::new(&py)
        .args(["-c", "import pyulog"])
        .output()
        .is_ok_and(|o| o.status.success());
    if !ok {
        println!("NOTICE: {py} cannot import pyulog: the pyulog checks are skipped");
        return None;
    }
    Some(py)
}

fn run(py: &str, args: &[&str]) -> (bool, String, String) {
    let script = concat!(env!("CARGO_MANIFEST_DIR"), "/../../tools/ulog_check.py");
    let o = Command::new(py).arg(script).args(args).output().unwrap();
    (
        o.status.success(),
        String::from_utf8(o.stdout).unwrap(),
        String::from_utf8(o.stderr).unwrap(),
    )
}

#[test]
fn pyulog_reads_every_topic_of_a_flight_and_the_same_values() {
    let Some(py) = python() else { return };
    let expected: Vec<&str> = NORMAL.iter().map(|t| t.name).collect();
    let expected = expected.join(",");
    for profile in BOTH {
        for name in ["rc-loss-position", "gust-hold"] {
            let (bytes, _) = fly(name, profile);
            let path = std::env::temp_dir().join(format!(
                "oxide-sim-pyulog-{}-{name}-{}.ulg",
                std::process::id(),
                profile.name
            ));
            std::fs::write(&path, &bytes).unwrap();
            let p = path.to_str().unwrap();
            let ctx = format!("{name} on {}", profile.name);
            let file = UlogFile::parse(&bytes).unwrap();

            // No corruption, every topic present, add_roll_pitch_yaw works.
            let (ok, out, err) = run(&py, &[p, "--expect-topics", &expected]);
            assert!(ok, "{ctx}: ulog_check.py failed: {err}\n{out}");
            for topic in NORMAL.iter().map(|t| t.name) {
                assert!(
                    out.lines()
                        .any(|l| l.starts_with(&format!("topic {topic} 0 "))),
                    "{ctx}: no {topic} in\n{out}"
                );
            }

            for topic in [
                "vehicle_attitude",
                "vehicle_local_position",
                "actuator_motors",
                "battery_status",
                "vehicle_status",
            ] {
                let (ok, csv, err) = run(&py, &[p, "--dump", topic]);
                assert!(ok, "{ctx}: {err}");
                let sub = file.subscriptions.iter().find(|s| s.name == topic).unwrap();
                let mut cols = Vec::new();
                columns(&file, &file.formats[sub.format].layout, "", true, &mut cols);
                let mut want = vec![
                    cols.iter()
                        .map(|(n, t)| format!("{n}:{t}"))
                        .collect::<Vec<_>>()
                        .join(","),
                ];
                for r in file.records(topic) {
                    want.push(
                        cols.iter()
                            .map(|(n, t)| cell(&r, n, t))
                            .collect::<Vec<_>>()
                            .join(","),
                    );
                }
                assert!(want.len() > 3, "{ctx}: {topic} has {} lines", want.len());
                let got: Vec<&str> = csv.lines().collect();
                assert_eq!(got.len(), want.len(), "{ctx} {topic}: record count");
                for (i, (g, w)) in got.iter().zip(&want).enumerate() {
                    assert_eq!(g, w, "{ctx} {topic}: line {i}");
                }
            }
            let _ = std::fs::remove_file(path);
        }
    }
}

#[test]
fn ulog_check_without_expect_topics_accepts_a_topic_with_no_record() {
    let Some(py) = python() else { return };
    // A short flight: no parameter changes, and the EKF has not published yet.
    let scenario = fault_scenarios::by_name("rc-loss-position").expect("scenario");
    let mut sim = scenario.build(F405_LEAN, 0).expect("builds");
    let buf = SharedBuffer::new();
    sim.start_log(Level::Normal, &LogBuild::sim("hash"), Box::new(buf.clone()))
        .unwrap();
    for _ in 0..300 {
        sim.step();
    }
    sim.finish_log().unwrap();
    let path = std::env::temp_dir().join(format!("oxide-sim-short-{}.ulg", std::process::id()));
    std::fs::write(&path, buf.bytes()).unwrap();
    let p = path.to_str().unwrap();
    let (ok, out, err) = run(&py, &[p]);
    assert!(ok, "{err}\n{out}");
    let (ok, _, err) = run(&py, &[p, "--expect-topics", "no_such_topic"]);
    assert!(!ok);
    assert!(err.contains("no_such_topic"), "{err}");
    // 300 ticks: vehicle_attitude waits for the estimate (1 s).
    let (ok, _, err) = run(&py, &[p, "--expect-topics", "vehicle_attitude"]);
    assert!(!ok, "{err}");
    let _ = std::fs::remove_file(path);
}
