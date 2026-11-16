// SPDX-License-Identifier: AGPL-3.0-or-later
//! P1 step 1: the recorder at the replay level (ADR-0005 §5).

use super::rates::Tick;
use super::{BUILD, CAP};
use crate::header::Level;
use crate::reader::{Item, UlogFile};
use crate::recorder::{Boot, Recorder, StartError};
use crate::replay::decode::{self, config_of};
use crate::replay::{self, MICRO_PARAMS, RebuildError, TOPICS};
use crate::topics::NORMAL;
use oxide_core::config::{Rotor, Spin, VehicleConfig};
use oxide_core::io::{Event, EventKind, Inputs, Outputs, Reasons};
use oxide_core::params::{self, LoadReport, ParamId, ParamValue};
use oxide_core::schedule::{F405_LEAN, H743_FULL, Profile};
use oxide_core::time::Micros;

// The replay tests' own hand-made values, compiled here too so that this file
// does not depend on the visibility of that module.
#[allow(dead_code, clippy::duplicate_mod)] // only a few of its values are used here
#[path = "../replay/samples.rs"]
mod samples;
use samples::{busy_status, full_inputs};

const REPORT: LoadReport = LoadReport {
    applied: 88,
    unknown: 2,
    dropped: 3,
    invalid: true,
    unreadable: false,
    torn: 4,
};

fn start_with<const N: usize, const C: usize>(
    file: &mut Vec<u8>,
    cfg: &VehicleConfig<N>,
    profile: &Profile,
    level: Level,
) -> Result<Recorder<C>, StartError> {
    let mut rec = Recorder::<C>::new();
    rec.start(
        file,
        Micros(0),
        BUILD,
        Boot {
            profile,
            report: REPORT,
        },
        level,
        cfg,
    )?;
    Ok(rec)
}

fn names_of(f: &UlogFile) -> Vec<&str> {
    f.items
        .iter()
        .filter_map(|i| match i {
            Item::Data(r) => Some(f.view(r).topic()),
            _ => None,
        })
        .collect()
}

fn ev(t: u64, kind: EventKind) -> Event {
    Event { t: Micros(t), kind }
}

/// A tick with every input present and two events.
fn busy_tick(t: u64) -> (Inputs, Outputs<4>) {
    let inputs = full_inputs();
    let mut o = Outputs::<4>::off(Micros(t + 1000));
    o.status = busy_status();
    o.motors = [0.1, 0.2, 0.3, 0.4];
    assert!(o.events.push(ev(t, EventKind::Armed)));
    assert!(
        o.events
            .push(ev(t, EventKind::ReasonRaised(Reasons::RC_LOST)))
    );
    (inputs, o)
}

#[test]
fn the_replay_level_declares_the_normal_topics_then_the_replay_topics() {
    let cfg = super::super::topics::quad();
    let mut file = Vec::new();
    start_with::<4, CAP>(&mut file, &cfg, &F405_LEAN, Level::Replay).unwrap();
    let f = UlogFile::parse(&file).unwrap();
    let formats: Vec<&str> = f.formats.iter().map(|d| d.name.as_str()).collect();
    let want: Vec<&str> = NORMAL.iter().chain(TOPICS.iter()).map(|t| t.name).collect();
    assert_eq!(formats, want);
    for (i, s) in f.subscriptions.iter().enumerate() {
        assert_eq!(s.msg_id as usize, i);
        assert_eq!(s.name, want[i]);
    }
    assert_eq!(f.info_str("oxide_log_level"), Some("replay"));
}

#[test]
fn the_start_writes_the_vehicle_the_exact_microseconds_and_the_boot_record() {
    let cfg = super::super::topics::quad();
    let mut file = Vec::new();
    start_with::<4, CAP>(&mut file, &cfg, &H743_FULL, Level::Replay).unwrap();
    let f = UlogFile::parse(&file).unwrap();
    let mut count = 0;
    cfg.vehicle_numbers(|_, _| count += 1);
    let names = names_of(&f);
    let mut want = vec!["oxide_vehicle"; count];
    want.extend(vec!["oxide_param_us"; MICRO_PARAMS.len()]);
    want.push("oxide_boot");
    assert_eq!(names, want);
    let boot = f.records("oxide_boot").next().unwrap();
    let b = decode::decode_boot(&boot).unwrap();
    assert_eq!(b.report, REPORT);
    assert_eq!(b.rotors, 4);
    assert_eq!(b.base_period_us, H743_FULL.base_period_us);
    assert_eq!(b.filter_period_ticks, H743_FULL.filter_period_ticks);
    let rebuilt = config_of::<4>(&f, &H743_FULL).unwrap();
    assert_eq!(rebuilt, cfg);
    assert_eq!(f.info_str("oxide_profile"), Some("h743-full"));
}

#[test]
fn the_records_of_a_tick_are_in_the_order_of_the_adr_and_decode_to_the_values_given() {
    let cfg = super::super::topics::quad();
    let mut file = Vec::new();
    let mut rec: Recorder<CAP> = start_with(&mut file, &cfg, &F405_LEAN, Level::Replay).unwrap();
    let before = UlogFile::parse(&file).unwrap().items.len();
    let (inputs, outputs) = busy_tick(5_000);
    let tel = super::telemetry();
    rec.record(Micros(5_000), &inputs, &outputs, &tel, &cfg);
    rec.drain(&mut file);
    let f = UlogFile::parse(&file).unwrap();
    assert!(f.items.len() > before);
    let first: Vec<&str> = names_of(&f)
        .into_iter()
        .skip_while(|n| !n.starts_with("oxide_in_tick"))
        .collect();
    // The replay records come first, then the events, then the normal level.
    assert_eq!(
        &first[..14],
        [
            "oxide_in_tick",
            "oxide_in_imu",
            "oxide_in_state",
            "oxide_in_rc",
            "oxide_in_battery",
            "oxide_in_setpoint",
            "oxide_in_gps",
            "oxide_in_baro",
            "oxide_in_mag",
            "oxide_in_gcs",
            "oxide_in_brain",
            "oxide_in_param_set",
            "oxide_out",
            "oxide_event",
        ]
    );
    assert_eq!(first[14], "oxide_event");
    // Decode them back.
    let data: Vec<_> = f
        .items
        .iter()
        .filter_map(|i| match i {
            Item::Data(r) => Some(f.view(r)),
            _ => None,
        })
        .skip_while(|r| r.topic() != "oxide_in_tick")
        .collect();
    let tick = data[0];
    let (tk, back) = decode::inputs_from_records(&tick, data[1..].iter().copied()).unwrap();
    assert_eq!((tk.seq, tk.t, tk.saved), (0, Micros(5_000), false));
    assert!(back.same_bits(&inputs), "{back:?}");
    let out = decode::decode_out::<4>(&data[12]).unwrap();
    assert_eq!(out.t, Micros(5_000));
    assert_eq!(out.events, 2);
    let got = out.into_outputs(outputs.events);
    assert!(got.same_bits(&outputs));
    for (r, want) in data[13..15].iter().zip(outputs.events.iter()) {
        assert_eq!(&crate::topics::decode_event(r).unwrap(), want);
    }
}

#[test]
fn the_sequence_number_counts_every_call_and_saved_marks_the_next_tick_only() {
    let cfg = super::super::topics::quad();
    let mut file = Vec::new();
    let mut rec: Recorder<CAP> = start_with(&mut file, &cfg, &F405_LEAN, Level::Replay).unwrap();
    for i in 0..4u64 {
        let t = i * 1000;
        if i == 2 {
            rec.note_saved();
        }
        let tick = Tick::at(t);
        rec.record(Micros(t), &tick.inputs, &tick.outputs, &tick.tel, &cfg);
        rec.drain(&mut file);
    }
    let f = UlogFile::parse(&file).unwrap();
    let ticks: Vec<_> = f
        .records("oxide_in_tick")
        .map(|r| decode::decode_tick(&r).unwrap())
        .collect();
    let seqs: Vec<u64> = ticks.iter().map(|t| t.seq).collect();
    let saved: Vec<bool> = ticks.iter().map(|t| t.saved).collect();
    assert_eq!(seqs, [0, 1, 2, 3]);
    assert_eq!(saved, [false, false, true, false]);
}

#[test]
fn a_normal_level_log_has_none_of_the_replay_records() {
    let cfg = super::super::topics::quad();
    let mut file = Vec::new();
    let mut rec: Recorder<CAP> = start_with(&mut file, &cfg, &F405_LEAN, Level::Normal).unwrap();
    let (inputs, outputs) = busy_tick(0);
    rec.record(Micros(0), &inputs, &outputs, &super::telemetry(), &cfg);
    rec.drain(&mut file);
    let f = UlogFile::parse(&file).unwrap();
    assert_eq!(f.formats.len(), NORMAL.len());
    for t in &TOPICS {
        assert_eq!(f.records(t.name).count(), 0, "{}", t.name);
    }
}

#[test]
fn a_ring_too_small_for_a_ticks_records_drops_them_and_writes_a_dropout() {
    const TINY: usize = 1024;
    let cfg = super::super::topics::quad();
    let mut file = Vec::new();
    let mut rec: Recorder<TINY> = start_with(&mut file, &cfg, &F405_LEAN, Level::Replay).unwrap();
    let (inputs, outputs) = busy_tick(0);
    let tel = super::telemetry();
    // A tick's replay records alone are about 500 bytes: two ticks without a
    // drain do not fit.
    rec.record(Micros(0), &inputs, &outputs, &tel, &cfg);
    rec.record(Micros(1_000), &inputs, &outputs, &tel, &cfg);
    let st = rec.stats();
    assert!(st.dropped_stream > 0, "{st:?}");
    rec.drain(&mut file);
    rec.record(Micros(2_000), &inputs, &outputs, &tel, &cfg);
    rec.drain(&mut file);
    let f = UlogFile::parse(&file).unwrap();
    assert!(f.truncated.is_none());
    assert!(
        f.items.iter().any(|i| matches!(i, Item::Dropout { .. })),
        "no dropout written"
    );
    assert!(rec.stats().dropouts > 0);
}

#[test]
fn a_configuration_that_cannot_replay_is_refused_at_start_naming_the_parameter() {
    let mut cfg = super::super::topics::quad();
    // 16,000,001 us: not the `f32` number of seconds it reads back as.
    cfg.failsafe.rc_timeout_us = 16_000_001;
    let mut file = Vec::new();
    let err = start_with::<4, CAP>(&mut file, &cfg, &F405_LEAN, Level::Replay).unwrap_err();
    let id = params::find("FS_RC_TIMEOUT").unwrap();
    assert_eq!(err, StartError::NotReplayable(RebuildError::NotExact(id)));
    // The normal level does not care.
    let mut file = Vec::new();
    start_with::<4, CAP>(&mut file, &cfg, &F405_LEAN, Level::Normal).unwrap();
}

#[test]
fn more_than_eight_rotors_are_refused_at_the_replay_level() {
    let q = super::super::topics::quad();
    let r = Rotor {
        x: 0.1,
        y: 0.1,
        spin: Spin::Ccw,
        max_thrust_n: 8.0,
        torque_coeff_m: 0.016,
    };
    let cfg = VehicleConfig::<9> {
        rotors: [r; 9],
        mass_kg: 1.0,
        inertia: q.inertia,
        thrust_expo: q.thrust_expo,
        nominal_voltage_v: q.nominal_voltage_v,
        gains: q.gains,
        limits: q.limits,
        estimator: q.estimator,
        failsafe: q.failsafe,
        battery_cells: q.battery_cells,
        battery_resistance_ohm: q.battery_resistance_ohm,
    };
    let mut file = Vec::new();
    let err = start_with::<9, CAP>(&mut file, &cfg, &F405_LEAN, Level::Replay).unwrap_err();
    assert_eq!(
        err,
        StartError::TooManyRotors(replay::TooManyRotors { got: 9, max: 8 })
    );
}

/// The configuration with every parameter moved to a value other than the one
/// it has, that the core accepts and that survives being logged.
fn every_parameter_changed() -> VehicleConfig<4> {
    let mut cfg = super::super::topics::quad();
    let mut changed = 0;
    for (i, def) in params::PARAMS.iter().enumerate() {
        let id = ParamId(i as u16);
        let now = params::get(&cfg, id).unwrap();
        let candidates: Vec<ParamValue> = match (def.min, def.max, now) {
            (ParamValue::F32(lo), ParamValue::F32(hi), _) => [0.37, 0.61, 0.2, 0.8, 0.05]
                .iter()
                .map(|k| ParamValue::F32(lo + (hi - lo) * k))
                .collect(),
            (ParamValue::I32(lo), ParamValue::I32(hi), _) => {
                (lo..=hi).take(6).map(ParamValue::I32).collect()
            }
            _ => Vec::new(),
        };
        for c in candidates {
            let mut trial = cfg;
            if params::set(&mut trial, &F405_LEAN, id, c).is_ok()
                && params::get(&trial, id) != Some(now)
                && replay::check_exact(&trial, &F405_LEAN).is_ok()
            {
                cfg = trial;
                changed += 1;
                break;
            }
        }
    }
    assert_eq!(changed, params::COUNT, "a parameter could not be moved");
    cfg
}

#[test]
fn the_header_of_a_replay_log_rebuilds_the_reference_and_a_fully_changed_configuration() {
    for cfg in [super::super::topics::quad(), every_parameter_changed()] {
        for profile in [F405_LEAN, H743_FULL] {
            let mut file = Vec::new();
            start_with::<4, CAP>(&mut file, &cfg, &profile, Level::Replay).unwrap();
            let f = UlogFile::parse(&file).unwrap();
            let back = config_of::<4>(&f, &profile).unwrap();
            assert_eq!(back, cfg);
            assert_eq!(back.config_hash(), cfg.config_hash());
        }
    }
    assert_ne!(
        every_parameter_changed(),
        super::super::topics::quad(),
        "the changed configuration is not changed"
    );
}
