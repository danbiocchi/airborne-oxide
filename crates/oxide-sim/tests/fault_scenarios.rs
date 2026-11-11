// SPDX-License-Identifier: AGPL-3.0-or-later
//! M1c acceptance: the fault scenarios (ADR-0003). Each injects a failure in
//! flight and checks that the vehicle does what the ADR says.

use oxide_core::io::NavState;
use oxide_core::math::Vec3;
use oxide_core::schedule::{F405_LEAN, H743_FULL, Profile};
use oxide_core::time::Micros;
use oxide_sim::fault_scenarios::{FaultScenario, all, build_with_wind, by_name, fly, pairs};
use oxide_sim::montecarlo::run_indexed;
use oxide_sim::outcome::{FlightOutcome, event_line};
use oxide_sim::wind::Gust;

const PROFILES: [Profile; 2] = [F405_LEAN, H743_FULL];

/// The scenario's violations on one run, or a description of the setup error.
fn violations(s: &FaultScenario, profile: Profile, index: u64) -> (Vec<String>, FlightOutcome) {
    let o = fly(s, profile, index).expect("simulation setup");
    (s.expect.check(&o), o)
}

/// What to print for a failed run.
fn report(s: &FaultScenario, profile: Profile, index: u64, v: &[String], o: &FlightOutcome) {
    eprintln!(
        "FAIL {} (index {index}, profile {})\n  about: {}",
        s.name, profile.name, s.about
    );
    for line in v {
        eprintln!("  violated: {line}");
    }
    eprintln!("  navigation: {:?}", o.nav_sequence);
    for e in &o.events {
        eprintln!("  {}", event_line(e));
    }
}

/// Runs index 0 of each scenario on the lean profile; collects every
/// failure before asserting, so one failing scenario does not hide others.
fn run_quick(scenarios: &[FaultScenario]) {
    let results = run_indexed(scenarios.len(), 0, |i| {
        let s = &scenarios[i];
        let (v, o) = violations(s, F405_LEAN, 0);
        (v, o)
    });
    let mut failed = Vec::new();
    for (s, (v, o)) in scenarios.iter().zip(&results) {
        if !v.is_empty() {
            report(s, F405_LEAN, 0, v, o);
            failed.push(s.name);
        }
    }
    assert!(
        failed.is_empty(),
        "{} of {} scenarios failed: {}",
        failed.len(),
        scenarios.len(),
        failed.join(", ")
    );
}

#[test]
fn fault_scenarios_quick() {
    run_quick(&all());
}

#[test]
fn fault_pairs_quick() {
    run_quick(&pairs());
}

#[test]
#[ignore = "10 runs x 2 profiles of every scenario and pairing; run with --release"]
fn fault_scenarios_full() {
    const RUNS: usize = 10;
    let scenarios: Vec<FaultScenario> = all().into_iter().chain(pairs()).collect();
    let mut failed = 0_usize;
    let mut failed_scenarios = Vec::new();
    for s in &scenarios {
        let mut passed = 0_usize;
        let mut total = 0_usize;
        for &profile in &PROFILES {
            let runs = run_indexed(RUNS, 0, |i| violations(s, profile, i as u64));
            for (i, (v, o)) in runs.iter().enumerate() {
                total += 1;
                if v.is_empty() {
                    passed += 1;
                } else {
                    report(s, profile, i as u64, v, o);
                }
            }
        }
        println!("{:<40} passed {passed}/{total}", s.name);
        if passed != total {
            failed += total - passed;
            failed_scenarios.push(s.name);
        }
    }
    assert_eq!(
        failed,
        0,
        "{failed} runs failed in: {}",
        failed_scenarios.join(", ")
    );
}

#[test]
fn fault_scenario_is_deterministic() {
    let s = by_name("brain-crash").expect("scenario");
    let a = fly(&s, F405_LEAN, 2).expect("run");
    let b = fly(&s, F405_LEAN, 2).expect("run");
    assert_eq!(a, b);
    // A different run index is a different flight (different wind).
    let c = fly(&s, F405_LEAN, 3).expect("run");
    assert_ne!(a.position_at_end, c.position_at_end);
}

/// Landings in wind: in an Rtl or Land descent the thrust is never cut to the
/// minimum while the vehicle is clearly in the air, over 10 seeds, in still
/// air, in a steady 3 m/s upward wind and with a 6 m/s upward gust.
///
/// This is NOT a test of false ground contact against a real upward
/// disturbance: wind acts on the sim vehicle only through drag, so it never
/// lifts the vehicle against its command, and `SimOptions` has no honest way
/// to push it up. That the contact test cannot fire on a rising vehicle is
/// covered at unit level in the core (`contact_is_not_declared_on_a_vehicle_rising`).
#[test]
fn landing_thrust_is_not_cut_in_the_air_in_wind() {
    // geofence: Rtl from 5 m, lands at home. gps-loss: Land.
    for name in ["geofence", "gps-loss", "battery-low"] {
        for air in ["still", "up-wind", "up-gust-wind"] {
            let s = by_name(name).expect("scenario");
            for index in 0..10_u64 {
                let mut sim = build_with_wind(&s, F405_LEAN, index, |mut w| {
                    match air {
                        "up-wind" => w.steady_ned.z = -3.0,
                        "up-gust-wind" => w.gusts.push(Gust {
                            start_s: 20.0,
                            duration_s: 15.0,
                            peak_ned: Vec3::new(0.0, 0.0, -6.0),
                        }),
                        _ => {}
                    }
                    w
                })
                .expect("simulation setup");
                let end = Micros((s.script.duration_s * 1e6).round() as u64);
                let mut landing_seen = false;
                sim.run_until(end, |r| {
                    let landing = matches!(
                        r.status.nav,
                        NavState::Rtl | NavState::Land | NavState::LandNow
                    );
                    landing_seen |= landing;
                    let mean = r.motors.iter().sum::<f32>() / r.motors.len() as f32;
                    // The minimum thrust is a motor command of about 0.14.
                    if r.status.armed && landing && -r.position.z > 0.2 {
                        assert!(
                            mean > 0.2,
                            "{name} ({air}) index {index}: thrust cut {:.2} m up at {:.2} s (nav {:?})",
                            -r.position.z,
                            r.t.as_secs_f64(),
                            r.status.nav
                        );
                    }
                });
                assert!(landing_seen, "{name} ({air}) index {index} never landed");
            }
        }
    }
}
