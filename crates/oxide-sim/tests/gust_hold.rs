// SPDX-License-Identifier: AGPL-3.0-or-later
//! M1b acceptance: position hold in gusts on noisy sensors (DESIGN §13).

use oxide_core::schedule::{F405_LEAN, H743_FULL, Profile};
use oxide_core::time::Micros;
use oxide_sim::metrics::{HoldCriteria, HoldMetrics, fly_gust_hold};
use oxide_sim::montecarlo::run_indexed;
use oxide_sim::scenario::gust_hold;
use oxide_sim::{Airframe, Simulation};

const PROFILES: [Profile; 2] = [F405_LEAN, H743_FULL];

fn run_all(n: usize) -> Vec<(Profile, Vec<HoldMetrics>)> {
    let c = HoldCriteria::default();
    PROFILES
        .iter()
        .map(|&p| {
            let runs = run_indexed(n, 0, |i| {
                fly_gust_hold(p, i as u64, &c).expect("simulation setup")
            });
            (p, runs)
        })
        .collect()
}

fn report_failures(results: &[(Profile, Vec<HoldMetrics>)]) -> usize {
    let c = HoldCriteria::default();
    let mut failed = 0;
    for (p, runs) in results {
        for (i, m) in runs.iter().enumerate() {
            let f = m.failures(&c);
            if !f.is_empty() {
                failed += 1;
                eprintln!("FAIL index {i} profile {}: {}", p.name, f.join("; "));
            }
        }
    }
    failed
}

#[test]
fn gust_hold_quick() {
    let results = run_all(8);
    assert_eq!(report_failures(&results), 0, "see FAIL lines above");
}

#[test]
#[ignore = "200 seeds x 2 profiles; run with --release"]
fn gust_hold_200_seeds() {
    let results = run_all(200);
    for (p, runs) in &results {
        println!("profile {} ({} runs)", p.name, runs.len());
        let worst = |name: &str, f: fn(&HoldMetrics) -> f64| {
            let (i, v) = runs
                .iter()
                .enumerate()
                .map(|(i, m)| (i, f(m)))
                .fold((0, f64::MIN), |a, b| if b.1 > a.1 { b } else { a });
            println!("  worst {name:<34} {v:9.4} at index {i}");
        };
        worst("max horizontal error (m)", |m| m.max_h_error_m);
        worst("rms horizontal error (m)", |m| m.rms_h_error_m);
        worst("max vertical error (m)", |m| m.max_v_error_m);
        worst("max tilt (deg)", |m| m.max_tilt_deg);
        worst("estimate max horizontal (m)", |m| m.est_max_h_error_m);
        worst("estimate max vertical (m)", |m| m.est_max_v_error_m);
        worst("estimate max velocity (m/s)", |m| m.est_max_vel_error_mps);
        worst("estimate max attitude (deg)", |m| m.est_max_att_error_deg);
        worst("rejected measurements", |m| f64::from(m.rejected));
        worst("estimator resets", |m| f64::from(m.resets));
        let c = HoldCriteria::default();
        let passed = runs.iter().filter(|m| m.passed(&c)).count();
        println!("  passed {passed}/{}", runs.len());
    }
    assert_eq!(report_failures(&results), 0, "see FAIL lines above");
}

fn fly_with_hash(index: u64) -> (HoldMetrics, u64) {
    let c = HoldCriteria::default();
    let airframe = Airframe::reference_quad().expect("airframe");
    let (pilot, options) = gust_hold(index);
    let duration = Micros((pilot.duration_s * 1e6) as u64);
    let target = pilot.hold.cast::<f64>();
    let mut sim =
        Simulation::<_, 4>::with_options(&airframe, F405_LEAN, pilot, options).expect("sim");
    let mut m = HoldMetrics::new();
    sim.run_until(duration, |r| m.add(r, target, &c));
    (m, sim.trace_hash())
}

#[test]
fn gust_hold_is_deterministic() {
    let (a, ha) = fly_with_hash(3);
    let (b, hb) = fly_with_hash(3);
    assert_eq!(a, b);
    assert_eq!(ha, hb);
    // The shared helper agrees with the hand-rolled loop.
    let c = HoldCriteria::default();
    assert_eq!(fly_gust_hold(F405_LEAN, 3, &c).expect("run"), a);
}
