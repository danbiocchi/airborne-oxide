// SPDX-License-Identifier: AGPL-3.0-or-later
//! A heading step is not a loss of control (ADR-0003 §3, amendment). The
//! flight `scenarios/yaw-step-large.toml` hovers at 5 m and steps the heading
//! setpoint by 2.5 rad and by 3.14 rad and back. The file judges the failsafe
//! reasons, the ending, the position hold and the tilt; this test judges the
//! heading, which the file format cannot express: the truth heading must
//! reach each target within `SETTLE_S` seconds of the step and stay there
//! until the next one.

use std::path::Path;

use oxide_core::schedule::{F405_LEAN, H743_FULL};
use oxide_sim::scenario_file;

/// (time of the step, s; heading setpoint, rad), from the file.
const STEPS: [(f64, f64); 4] = [(10.0, 2.5), (17.0, 0.0), (24.0, 3.14), (31.0, 0.0)];
/// The heading must be within `TOL_RAD` of the setpoint this long after a step.
const SETTLE_S: f64 = 4.0;
const TOL_RAD: f64 = 0.1;

fn wrap(a: f64) -> f64 {
    let two_pi = 2.0 * std::f64::consts::PI;
    let mut x = a % two_pi;
    if x > std::f64::consts::PI {
        x -= two_pi;
    } else if x < -std::f64::consts::PI {
        x += two_pi;
    }
    x
}

#[test]
fn heading_steps_reach_their_target_without_a_failsafe() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scenarios/yaw-step-large.toml");
    let file = scenario_file::load_path(&path).expect("the file loads");
    for profile in [F405_LEAN, H743_FULL] {
        for index in 0..3_u64 {
            let mut sim = file.build(profile, index).expect("builds");
            // Worst heading error in the last second before each next step
            // (or the end), and the first time the error is within tolerance.
            let mut worst_late = [0.0_f64; 4];
            let mut reached_at = [f64::NAN; 4];
            sim.run_until(file.end(), |r| {
                let t = r.t.as_secs_f64();
                let yaw = r.attitude.yaw();
                for (k, (t0, target)) in STEPS.iter().enumerate() {
                    let t_next = STEPS.get(k + 1).map_or(file.duration_s, |s| s.0);
                    let err = wrap(yaw - target).abs();
                    if t >= *t0 && t < t_next && err < TOL_RAD && reached_at[k].is_nan() {
                        reached_at[k] = t - t0;
                    }
                    if t >= t0 + SETTLE_S && t < t_next {
                        worst_late[k] = worst_late[k].max(err);
                    }
                }
            });
            for k in 0..4 {
                assert!(
                    reached_at[k] <= SETTLE_S,
                    "{} run {index} step {k}: heading reached after {} s",
                    profile.name,
                    reached_at[k]
                );
                assert!(
                    worst_late[k] < TOL_RAD,
                    "{} run {index} step {k}: heading error {} rad after settling",
                    profile.name,
                    worst_late[k]
                );
            }
        }
    }
}

#[test]
fn the_file_flies_clean_on_every_run_of_its_own() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scenarios/yaw-step-large.toml");
    let file = scenario_file::load_path(&path).expect("the file loads");
    for profile in [F405_LEAN, H743_FULL] {
        for index in 0..file.run.seeds {
            let v = file.fly_checked(profile, index, &[]).expect("flies");
            assert!(
                v.passed(),
                "{} run {index}: {:?} {:?}",
                profile.name,
                v.violations,
                v.not_judged
            );
        }
    }
}
