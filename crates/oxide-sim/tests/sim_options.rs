// SPDX-License-Identifier: AGPL-3.0-or-later
//! Simulator options: sensors, wind and estimator source (M1b).

use oxide_core::config::EstimatorSource;
use oxide_core::math::Vec3;
use oxide_core::schedule::F405_LEAN;
use oxide_core::time::Micros;
use oxide_sim::scenario::{HoverStep, gust_hold};
use oxide_sim::sensors::SensorConfig;
use oxide_sim::wind::WindConfig;
use oxide_sim::{Airframe, SimOptions, Simulation, StepRecord};

fn fly(options: Option<SimOptions>, seconds: u64) -> (Vec<StepRecord<4>>, u64) {
    let a = Airframe::reference_quad().expect("reference airframe");
    let mut sim = match options {
        Some(o) => Simulation::<_, 4>::with_options(&a, F405_LEAN, HoverStep::default(), o),
        None => Simulation::<_, 4>::new(&a, F405_LEAN, HoverStep::default()),
    }
    .expect("sim");
    let mut log = Vec::new();
    sim.run_until(Micros::from_secs(seconds), |r| log.push(*r));
    (log, sim.trace_hash())
}

fn at(log: &[StepRecord<4>], t: f64) -> &StepRecord<4> {
    let i = log.partition_point(|r| r.t.as_secs_f64() < t);
    &log[i.min(log.len() - 1)]
}

#[test]
fn default_options_match_new() {
    let (_, a) = fly(None, 20);
    let (_, b) = fly(Some(SimOptions::default()), 20);
    assert_eq!(a, b);
}

#[test]
fn realistic_sensors_with_external_estimator_still_fly() {
    let options = SimOptions {
        seed: 7,
        sensors: SensorConfig::realistic(),
        ..SimOptions::default()
    };
    let (log, _) = fly(Some(options), 20);
    let err = (at(&log, 19.9).position - Vec3::new(1.0, 1.0, -3.0)).norm();
    assert!(err < 0.1, "final position error {err}");
    assert!(
        log.iter().any(|r| r.estimate.is_some()),
        "truth state should be published as the estimate"
    );
}

#[test]
fn steady_wind_is_applied_and_rejected() {
    let wind = Vec3::new(3.0, 0.0, 0.0);
    let options = SimOptions {
        wind: WindConfig {
            steady_ned: wind,
            ..WindConfig::calm()
        },
        ..SimOptions::default()
    };
    let (log, _) = fly(Some(options), 12);
    assert!(log.iter().all(|r| r.wind == wind), "StepRecord.wind");
    let err = (at(&log, 10.9).position - Vec3::new(1.0, 0.0, -2.0)).norm();
    assert!(err < 0.1, "position error at 10.9 s: {err}");

    // Mean pitch during the hold: thrust leans into the wind (nose up).
    let pitches: Vec<f64> = log
        .iter()
        .filter(|r| (8.0..10.9).contains(&r.t.as_secs_f64()))
        .map(|r| r.attitude.to_euler().y)
        .collect();
    let mean = pitches.iter().sum::<f64>() / pitches.len() as f64;
    assert!(mean > 0.0, "mean pitch {mean} rad should be positive");
}

#[test]
fn gust_hold_is_deterministic_per_index() {
    let (p0, a) = gust_hold(0);
    let (p0b, b) = gust_hold(0);
    let (_, c) = gust_hold(1);
    assert_eq!(a, b);
    assert_eq!(p0, p0b);
    assert_ne!(a.seed, c.seed);
    assert_ne!(a.wind, c.wind);
    assert_eq!(a.estimator, EstimatorSource::Ekf);
    assert_eq!(a.wind.gusts.len(), 3);
}
