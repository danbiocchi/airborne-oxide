// SPDX-License-Identifier: AGPL-3.0-or-later
//! Step 2: rates on the tick clock, and status on change.

use super::{CAP, inputs_at, started, telemetry};
use crate::header::Level;
use crate::reader::UlogFile;
use crate::recorder::{self, Recorder};
use crate::topics::{self, NORMAL};
use oxide_core::config::EstimatorSource;
use oxide_core::flight::Telemetry;
use oxide_core::io::{
    Action, FlightMode, Inputs, NavState, Outputs, PrearmFlags, Reasons, Status, Validity,
};
use oxide_core::time::Micros;

/// What a scripted tick is given and may change.
pub(crate) struct Tick {
    pub now: u64,
    pub inputs: Inputs,
    pub outputs: Outputs<4>,
    pub tel: Telemetry,
}

impl Tick {
    pub fn at(now: u64) -> Self {
        Self {
            now,
            inputs: inputs_at(now),
            outputs: Outputs::off(Micros(now + 1000)),
            tel: telemetry(),
        }
    }
}

/// Runs ticks `0, dt, 2dt, ...` below `end_us`, calling `each` before each
/// record, and drains after each. Returns the parsed file and the recorder.
pub(crate) fn run(
    dt: u64,
    end_us: u64,
    mut each: impl FnMut(&mut Tick),
) -> (UlogFile, Recorder<CAP>, Vec<u8>) {
    let mut file = Vec::new();
    let (mut rec, cfg) = started(&mut file, Level::Normal);
    let mut t = 0;
    while t < end_us {
        let mut tick = Tick::at(t);
        each(&mut tick);
        rec.record(
            Micros(tick.now),
            &tick.inputs,
            &tick.outputs,
            &tick.tel,
            &cfg,
        );
        rec.drain(&mut file);
        t += dt;
    }
    let f = UlogFile::parse(&file).unwrap();
    (f, rec, file)
}

pub(crate) fn times(f: &UlogFile, topic: &str) -> Vec<u64> {
    f.records(topic)
        .map(|r| r.u64("timestamp").unwrap())
        .collect()
}

/// Records a topic writes in `secs` seconds at `hz`, whatever the tick rate.
fn expected(name: &str, secs: u64) -> u64 {
    let hz = match name {
        "sensor_combined"
        | "vehicle_angular_velocity"
        | "vehicle_rates_setpoint"
        | "actuator_motors"
        | "vehicle_thrust_setpoint"
        | "vehicle_torque_setpoint" => 100,
        "vehicle_attitude" | "vehicle_attitude_setpoint" => 50,
        "vehicle_local_position" | "vehicle_local_position_setpoint" => 20,
        "manual_control_setpoint" | "vehicle_magnetometer" | "vehicle_air_data" => 10,
        "battery_status" | "oxide_estimator" => 5,
        // Status: on change, and once a second; logger: once a second.
        "vehicle_status" | "failsafe_flags" | "oxide_status" | "oxide_logger" => 1,
        "oxide_event" => 0,
        other => panic!("{other}"),
    };
    hz * secs
}

#[test]
fn every_topic_is_written_at_its_rate_at_1_khz_and_at_8_khz() {
    for dt in [1000, 125] {
        let secs = 4;
        let (f, rec, _) = run(dt, secs * 1_000_000, |_| {});
        for t in NORMAL {
            let want = expected(t.name, secs);
            let got = f.records(t.name).count() as u64;
            assert!(
                got.abs_diff(want) <= 1,
                "dt {dt}: {} wrote {got}, wanted {want} +- 1",
                t.name
            );
            let ts = times(&f, t.name);
            assert!(
                ts.windows(2).all(|w| w[0] <= w[1]),
                "dt {dt}: {} went back in time",
                t.name
            );
        }
        assert_eq!(rec.stats().dropped_stream + rec.stats().dropped_must, 0);
    }
}

#[test]
fn records_are_spaced_by_their_period() {
    let (f, _, _) = run(125, 1_000_000, |_| {});
    for (name, period) in [
        ("sensor_combined", 10_000),
        ("vehicle_attitude", 20_000),
        ("vehicle_local_position", 50_000),
        ("manual_control_setpoint", 100_000),
        ("battery_status", 200_000),
    ] {
        let ts = times(&f, name);
        assert_eq!(ts[0], 0, "{name}");
        assert!(
            ts.windows(2).all(|w| w[1] - w[0] == period),
            "{name}: {ts:?}"
        );
    }
}

#[test]
fn a_pause_gives_one_record_per_topic_not_a_burst() {
    let mut file = Vec::new();
    let (mut rec, cfg) = started(&mut file, Level::Normal);
    // One tick at 0, then a pause of two seconds, then five ticks 1 ms apart.
    for t in [0, 2_000_000, 2_001_000, 2_002_000, 2_003_000, 2_004_000] {
        let tick = Tick::at(t);
        rec.record(Micros(t), &tick.inputs, &tick.outputs, &tick.tel, &cfg);
        rec.drain(&mut file);
    }
    let f = UlogFile::parse(&file).unwrap();
    for t in NORMAL {
        if t.name == "oxide_event" {
            continue;
        }
        assert_eq!(
            times(&f, t.name),
            [0, 2_000_000],
            "{} wrote a burst or skipped a record",
            t.name
        );
    }
}

#[test]
fn after_a_pause_the_clock_keeps_its_phase() {
    // Records at 0 and 2.003 s; the next 100 Hz time on the grid of the
    // second record is 2.013 s, so 2.009 s writes nothing and 2.013 s does.
    let mut file = Vec::new();
    let (mut rec, cfg) = started(&mut file, Level::Normal);
    for t in [0, 2_003_000, 2_009_000, 2_013_000, 2_014_000, 2_023_000] {
        let tick = Tick::at(t);
        rec.record(Micros(t), &tick.inputs, &tick.outputs, &tick.tel, &cfg);
        rec.drain(&mut file);
    }
    let f = UlogFile::parse(&file).unwrap();
    assert_eq!(
        times(&f, "sensor_combined"),
        [0, 2_003_000, 2_013_000, 2_023_000]
    );
}

#[test]
fn a_status_change_is_written_when_it_happens_and_not_again_for_a_second() {
    let (f, _, _) = run(1000, 3_500_000, |t| {
        if t.now >= 500_000 {
            t.outputs.status.armed = true;
            t.outputs.status.nav = NavState::Stabilized;
        }
    });
    assert_eq!(
        times(&f, "vehicle_status"),
        [0, 500_000, 1_500_000, 2_500_000]
    );
    let rec: Vec<_> = f.records("vehicle_status").collect();
    assert_eq!(rec[0].u8("arming_state"), Some(1));
    assert_eq!(rec[1].u8("arming_state"), Some(2));
    assert_eq!(rec[2].u8("arming_state"), Some(2));
    // oxide_status carries those too; failsafe_flags does not change.
    assert_eq!(
        times(&f, "oxide_status"),
        [0, 500_000, 1_500_000, 2_500_000]
    );
    assert_eq!(
        times(&f, "failsafe_flags"),
        [0, 1_000_000, 2_000_000, 3_000_000]
    );
}

#[test]
fn a_change_after_a_write_waits_out_the_second_and_a_second_change_is_not_lost() {
    // Changes at 1.2 s and 1.4 s: each is written at once ("not again until
    // a second later" is about an unchanged status); then a second after the
    // last write.
    let (f, _, _) = run(1000, 2_500_000, |t| {
        t.outputs.status.reasons = if t.now >= 1_400_000 {
            Reasons::RC_LOST.union(Reasons::BATTERY_LOW)
        } else if t.now >= 1_200_000 {
            Reasons::RC_LOST
        } else {
            Reasons::empty()
        };
    });
    assert_eq!(
        times(&f, "failsafe_flags"),
        [0, 1_000_000, 1_200_000, 1_400_000, 2_400_000]
    );
    // vehicle_status does not carry the reasons.
    assert_eq!(times(&f, "vehicle_status"), [0, 1_000_000, 2_000_000]);
}

#[test]
fn no_change_gives_one_record_a_second() {
    let (f, _, _) = run(1000, 5_000_000, |_| {});
    for name in [
        "vehicle_status",
        "failsafe_flags",
        "oxide_status",
        "oxide_logger",
    ] {
        assert_eq!(
            times(&f, name),
            [0, 1_000_000, 2_000_000, 3_000_000, 4_000_000],
            "{name}"
        );
    }
}

#[test]
fn a_topic_that_comes_from_an_input_is_absent_until_its_first_sample() {
    let (f, _, _) = run(1000, 400_000, |t| {
        let keep = |at: u64| t.now >= at;
        let i = &mut t.inputs;
        if !keep(100_000) {
            i.imu = None;
        }
        if !keep(150_000) {
            i.rc = None;
        }
        if !keep(220_000) {
            i.mag = None;
        }
        if !keep(300_000) {
            i.baro = None;
        }
        if !keep(350_000) {
            i.battery = None;
        }
    });
    let first = |n: &str| times(&f, n).first().copied();
    assert_eq!(first("sensor_combined"), Some(100_000));
    assert_eq!(first("manual_control_setpoint"), Some(150_000));
    assert_eq!(first("vehicle_magnetometer"), Some(220_000));
    assert_eq!(first("vehicle_air_data"), Some(300_000));
    assert_eq!(first("battery_status"), Some(350_000));
    // A topic made from an input sample waits for a sample (ADR-0005 §3):
    // with samples only on the first tick, each is written once and then
    // nothing is repeated.
    let held = run(1000, 400_000, |t| {
        if t.now > 0 {
            t.inputs = Inputs::default();
        }
    })
    .0;
    assert_eq!(times(&held, "sensor_combined"), [0]);
    assert_eq!(times(&held, "battery_status"), [0]);
}

#[test]
fn estimate_topics_wait_for_an_estimate() {
    let (f, _, _) = run(1000, 200_000, |t| {
        if t.now < 60_000 {
            t.tel.estimate = None;
        }
    });
    assert_eq!(times(&f, "vehicle_attitude").first(), Some(&60_000));
    assert_eq!(times(&f, "vehicle_angular_velocity").first(), Some(&60_000));
    assert_eq!(times(&f, "vehicle_local_position").first(), Some(&60_000));
    // The setpoint and the estimator's health are written from the start.
    assert_eq!(
        times(&f, "vehicle_local_position_setpoint").first(),
        Some(&0)
    );
    assert_eq!(times(&f, "oxide_estimator").first(), Some(&0));
    assert_eq!(
        f.records("oxide_estimator")
            .next()
            .unwrap()
            .bool("has_estimate"),
        Some(false)
    );
}

#[test]
fn the_imu_interval_comes_from_consecutive_samples() {
    let (f, _, _) = run(1000, 40_000, |_| {});
    let dts: Vec<u32> = f
        .records("sensor_combined")
        .map(|r| r.u32("gyro_integral_dt").unwrap())
        .collect();
    assert_eq!(dts, [0, 1000, 1000, 1000]);
}

// ---- the keys that decide "something changed" ----

fn statuses() -> Vec<(&'static str, Status)> {
    let b = Status::default();
    let mut v = vec![("base", b)];
    let mut add = |name: &'static str, f: &dyn Fn(&mut Status)| {
        let mut s = b;
        f(&mut s);
        v.push((name, s));
    };
    add("armed", &|s| s.armed = true);
    add("killed", &|s| s.killed = true);
    for m in [
        FlightMode::Altitude,
        FlightMode::Position,
        FlightMode::Offboard,
    ] {
        add("mode", &move |s| s.mode = m);
    }
    add("saturated", &|s| s.saturated = true);
    add("timing_faults", &|s| s.timing_faults = 3);
    add("no_estimate", &|s| s.no_estimate = true);
    add("aligned", &|s| s.estimator.aligned = true);
    add("gps_aiding", &|s| s.estimator.gps_aiding = true);
    add("rejected", &|s| s.estimator.rejected = 9);
    add("resets", &|s| s.estimator.resets = 9);
    for n in 1..10u8 {
        add("nav", &move |s| s.nav = NavState::from_code(n).unwrap());
    }
    for bit in 0..5 {
        add("valid", &move |s| {
            let mut v = Validity::NONE;
            match bit {
                0 => v.attitude = true,
                1 => v.heading = true,
                2 => v.altitude = true,
                3 => v.velocity = true,
                _ => v.position = true,
            }
            s.valid = v;
        });
    }
    for bit in 0..11 {
        add("reasons", &move |s| s.reasons = Reasons(1 << bit));
    }
    for n in 1..6u8 {
        add("action", &move |s| s.action = Action::from_code(n).unwrap());
    }
    for bit in 0..9 {
        add("prearm", &move |s| s.prearm = PrearmFlags(1 << bit));
    }
    add("prearm_two", &|s| s.prearm = PrearmFlags(3));
    add("home_set", &|s| s.home_set = true);
    add("params_unsaved", &|s| s.params_unsaved = true);
    v
}

fn bytes(
    topic: &crate::record::Topic,
    f: impl FnOnce(&mut crate::record::RecordWriter<'_, Vec<u8>>),
) -> Vec<u8> {
    let mut v = Vec::new();
    crate::record::data(&mut v, 0, topic, f).unwrap();
    // Without the header, the id and the timestamp.
    v[13..].to_vec()
}

#[test]
fn the_status_keys_change_exactly_when_the_record_changes() {
    let all = statuses();
    let enc = |s: &Status| {
        (
            bytes(&topics::VEHICLE_STATUS, |w| {
                topics::vehicle_status(w, Micros(0), s)
            }),
            bytes(&topics::FAILSAFE_FLAGS, |w| {
                topics::failsafe_flags(w, Micros(0), s)
            }),
            bytes(&topics::OXIDE_STATUS, |w| {
                topics::oxide_status(w, Micros(0), s, EstimatorSource::Ekf, 0, false, false)
            }),
        )
    };
    let key = |s: &Status| {
        (
            recorder::vehicle_status_key(s),
            recorder::failsafe_flags_key(s),
            recorder::oxide_status_key(s, EstimatorSource::Ekf, 0, false, false),
        )
    };
    for (na, a) in &all {
        for (nb, b) in &all {
            let (ea, eb) = (enc(a), enc(b));
            let (ka, kb) = (key(a), key(b));
            assert_eq!(ea.0 == eb.0, ka.0 == kb.0, "vehicle_status {na} vs {nb}");
            assert_eq!(ea.1 == eb.1, ka.1 == kb.1, "failsafe_flags {na} vs {nb}");
            assert_eq!(ea.2 == eb.2, ka.2 == kb.2, "oxide_status {na} vs {nb}");
        }
    }
    // The non-status inputs of oxide_status.
    let s = Status::default();
    let k = |src, dropped, landed, contact| {
        recorder::oxide_status_key(&s, src, dropped, landed, contact)
    };
    let base = k(EstimatorSource::External, 0, false, false);
    assert_ne!(base, k(EstimatorSource::Ekf, 0, false, false));
    assert_ne!(base, k(EstimatorSource::External, 1, false, false));
    assert_ne!(base, k(EstimatorSource::External, 0, true, false));
    assert_ne!(base, k(EstimatorSource::External, 0, false, true));
}
