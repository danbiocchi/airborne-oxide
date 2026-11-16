// SPDX-License-Identifier: AGPL-3.0-or-later
//! The recorder: start, record, drain, rates, events (ADR-0005 §3, §7).

use super::topics::quad;
use crate::header::{BuildInfo, Level};
use crate::reader::UlogFile;
use crate::recorder::{Boot, Recorder};
use crate::topics::NORMAL;
use oxide_core::config::VehicleConfig;
use oxide_core::flight::{RateCommand, Telemetry};
use oxide_core::io::{
    BaroSample, BatterySample, ImuSample, Inputs, MagSample, Outputs, RcSample, Stamped,
    StateSample, Validity,
};
use oxide_core::math::{Quat, Vec3};
use oxide_core::schedule::F405_LEAN;
use oxide_core::time::Micros;

pub(crate) const CAP: usize = 16 * 1024;

pub(crate) const BUILD: BuildInfo<'static> = BuildInfo {
    git_hash: "3f2a9c1d0e5b7a8c9d0e1f2a3b4c5d6e7f8a9b0c",
    board: "SIM",
};

/// A telemetry snapshot with an estimate and every value in a plausible state.
pub(crate) fn telemetry() -> Telemetry {
    Telemetry {
        estimate: Some(StateSample {
            position: Vec3::new(1.0, 2.0, -3.0),
            velocity: Vec3::new(0.5, 0.0, 0.0),
            attitude: Quat::identity(),
            rate: Vec3::new(0.1, 0.2, 0.3),
        }),
        valid: Validity::ALL,
        attitude_target: Quat::identity(),
        thrust_target_n: 6.0,
        rate_target: Vec3::new(0.0, 0.0, 0.0),
        rate_command: RateCommand::default(),
        setpoints: None,
        landed: false,
        ground_contact: false,
        gyro_filtered: Vec3::new(0.0, 0.0, 0.0),
        battery_cell_v: Some(3.9),
        battery_low: false,
        battery_critical: false,
        battery_voltage_v: Some(23.4),
        battery_current_a: Some(10.0),
        events_dropped: 0,
    }
}

/// Inputs with every sample present, stamped at `t`.
pub(crate) fn inputs_at(t: u64) -> Inputs {
    let m = Micros(t);
    Inputs {
        imu: Some(Stamped::ideal(
            m,
            ImuSample {
                gyro: Vec3::new(0.01, 0.02, 0.03),
                accel: Vec3::new(0.0, 0.0, -9.81),
            },
        )),
        rc: Some(Stamped::ideal(
            m,
            RcSample {
                throttle: 0.5,
                ..RcSample::default()
            },
        )),
        mag: Some(Stamped::ideal(
            m,
            MagSample {
                field_ut: Vec3::new(20.0, 0.0, 40.0),
            },
        )),
        baro: Some(Stamped::ideal(m, BaroSample { altitude_m: 3.0 })),
        battery: Some(Stamped::ideal(
            m,
            BatterySample {
                voltage_v: 23.4,
                current_a: 10.0,
            },
        )),
        ..Inputs::default()
    }
}

/// A recorder started into `file` with the reference quad.
pub(crate) fn started(file: &mut Vec<u8>, level: Level) -> (Recorder<CAP>, VehicleConfig<4>) {
    let cfg = quad();
    let mut rec = Recorder::<CAP>::new();
    rec.start(file, Micros(0), BUILD, Boot::clean(&F405_LEAN), level, &cfg)
        .unwrap();
    (rec, cfg)
}

pub(crate) fn names() -> Vec<&'static str> {
    NORMAL.iter().map(|t| t.name).collect()
}

// ---- step 1: start, record, drain ----

#[test]
fn start_writes_the_header_for_the_normal_topics() {
    let mut file = Vec::new();
    let (_rec, cfg) = started(&mut file, Level::Normal);
    let f = UlogFile::parse(&file).unwrap();
    let formats: Vec<&str> = f.formats.iter().map(|d| d.name.as_str()).collect();
    assert_eq!(formats, names());
    assert_eq!(f.subscriptions.len(), NORMAL.len());
    assert_eq!(f.info_str("oxide_log_level"), Some("normal"));
    assert_eq!(f.info_str("oxide_profile"), Some("f405-lean"));
    assert_eq!(f.info_str("ver_sw"), Some(BUILD.git_hash));
    assert_eq!(
        f.info("oxide_config_hash").unwrap().as_u64(),
        Some(cfg.config_hash())
    );
    assert_eq!(f.initial_params.len(), 93);
    assert!(f.items.is_empty());
}

#[test]
fn the_replay_level_writes_what_the_normal_level_writes_with_its_own_level() {
    let (mut a, mut b) = (Vec::new(), Vec::new());
    started(&mut a, Level::Normal);
    started(&mut b, Level::Replay);
    let fa = UlogFile::parse(&a).unwrap();
    let fb = UlogFile::parse(&b).unwrap();
    assert_eq!(fb.info_str("oxide_log_level"), Some("replay"));
    assert_eq!(fa.formats, fb.formats);
    assert_eq!(fa.subscriptions, fb.subscriptions);
    assert_eq!(a.len(), b.len());
}

#[test]
fn nothing_is_written_before_start() {
    let cfg = quad();
    let mut rec = Recorder::<CAP>::new();
    rec.record(
        Micros(0),
        &inputs_at(0),
        &Outputs::<4>::off(Micros(0)),
        &telemetry(),
        &cfg,
    );
    let mut out = Vec::new();
    assert_eq!(rec.drain(&mut out), 0);
    assert!(out.is_empty());
    assert_eq!(rec.stats().written, 0);
}

#[test]
fn the_first_record_call_writes_every_streamed_topic_once() {
    let mut file = Vec::new();
    let (mut rec, cfg) = started(&mut file, Level::Normal);
    rec.record(
        Micros(0),
        &inputs_at(0),
        &Outputs::<4>::off(Micros(10_000)),
        &telemetry(),
        &cfg,
    );
    let n = rec.drain(&mut file);
    assert!(n > 0);
    assert_eq!(rec.stats().bytes as usize, n);
    let f = UlogFile::parse(&file).unwrap();
    for t in NORMAL {
        let c = f.records(t.name).count();
        // oxide_event has no event to write.
        let want = usize::from(t.name != "oxide_event");
        assert_eq!(c, want, "{}", t.name);
    }
}

#[test]
fn drain_passes_through_to_a_sink_that_takes_bytes_whole_or_not_at_all() {
    struct Refuse;
    impl crate::Sink for Refuse {
        fn put(&mut self, _: &[u8]) -> Result<(), crate::Error> {
            Err(crate::Error::Full)
        }
    }
    let mut file = Vec::new();
    let (mut rec, cfg) = started(&mut file, Level::Normal);
    rec.record(
        Micros(0),
        &inputs_at(0),
        &Outputs::<4>::off(Micros(10_000)),
        &telemetry(),
        &cfg,
    );
    assert_eq!(rec.drain(&mut Refuse), 0);
    let held = rec.drain(&mut file);
    assert!(held > 0);
}

mod events;
mod newest;
mod rates;
mod text;
mod whole;
