// SPDX-License-Identifier: AGPL-3.0-or-later
//! M1c acceptance: the fault scenarios (ADR-0003). Each injects a failure in
//! flight and checks that the vehicle does what the ADR says.

use oxide_core::io::NavState;
use oxide_core::math::Vec3;
use oxide_core::schedule::{F405_LEAN, Profile};
use oxide_core::time::Micros;
use oxide_sim::fault_scenarios::by_name;
use oxide_sim::montecarlo::run_indexed;
use oxide_sim::outcome::{FlightOutcome, event_line};
use oxide_sim::scenario_file::{ScenarioFile, load_path};
use oxide_sim::wind::Gust;

/// The fault scenarios: every file in `scenarios/` that states `[expect]`,
/// sorted by name. The tests iterate over the directory, so a new file is
/// flown without a change here.
fn files() -> Vec<ScenarioFile> {
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scenarios");
    let mut paths: Vec<_> = std::fs::read_dir(&dir)
        .expect("scenarios/")
        .map(|e| e.expect("entry").path())
        .filter(|p| p.extension().is_some_and(|x| x == "toml"))
        .collect();
    paths.sort();
    paths
        .iter()
        .map(|p| load_path(p).unwrap_or_else(|e| panic!("{e}")))
        .filter(|f| f.expect.is_some())
        .collect()
}

/// The named failures, then the pairings.
fn named() -> Vec<ScenarioFile> {
    files()
        .into_iter()
        .filter(|f| !f.name.starts_with("pair-"))
        .collect()
}

fn pairings() -> Vec<ScenarioFile> {
    files()
        .into_iter()
        .filter(|f| f.name.starts_with("pair-"))
        .collect()
}

fn profile_of(p: oxide_sim::scenario_file::RunProfile) -> Profile {
    p.profile()
}

/// The scenario's violations on one run, or a description of the setup error.
fn violations(s: &ScenarioFile, profile: Profile, index: u64) -> (Vec<String>, FlightOutcome) {
    let o = s.fly(profile, index).expect("simulation setup");
    let expect = s.expect.as_ref().expect("a fault scenario has [expect]");
    (expect.check(&o).violations, o)
}

/// What to print for a failed run.
fn report(s: &ScenarioFile, profile: Profile, index: u64, v: &[String], o: &FlightOutcome) {
    eprintln!(
        "FAIL {} (index {index}, profile {})\n  about: {}",
        s.name, profile.name, s.about
    );
    for line in v {
        eprintln!("  violated: {line}");
    }
    eprintln!(
        "  navigation: {:?}",
        o.core.as_ref().expect("core part").nav_sequence
    );
    for e in &o.core.as_ref().expect("core part").events {
        eprintln!("  {}", event_line(e));
    }
}

/// Runs index 0 of each scenario on the lean profile; collects every
/// failure before asserting, so one failing scenario does not hide others.
fn run_quick(scenarios: &[ScenarioFile]) {
    let results = run_indexed(scenarios.len(), 0, |i| {
        let s = &scenarios[i];
        let (v, o) = violations(s, F405_LEAN, 0);
        (v, o)
    });
    let mut failed = Vec::new();
    for (s, (v, o)) in scenarios.iter().zip(&results) {
        if !v.is_empty() {
            report(s, F405_LEAN, 0, v, o);
            failed.push(s.name.as_str());
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
    run_quick(&named());
}

#[test]
fn fault_pairs_quick() {
    run_quick(&pairings());
}

#[test]
#[ignore = "every file's [run] seeds x profiles; run with --release"]
fn fault_scenarios_full() {
    let scenarios = files();
    assert_eq!(scenarios.len(), 41);
    let mut failed = 0_usize;
    let mut failed_scenarios = Vec::new();
    for s in &scenarios {
        let mut passed = 0_usize;
        let mut total = 0_usize;
        for &p in &s.run.profiles {
            let profile = profile_of(p);
            let runs = run_indexed(s.run.seeds as usize, 0, |i| {
                violations(s, profile, i as u64)
            });
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
            failed_scenarios.push(s.name.as_str());
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
    let a = s.fly(F405_LEAN, 2).expect("run");
    let b = s.fly(F405_LEAN, 2).expect("run");
    assert_eq!(a, b);
    // A different run index is a different flight (different wind).
    let c = s.fly(F405_LEAN, 3).expect("run");
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
                let mut sim = s
                    .build_with_wind(F405_LEAN, index, |mut w| {
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
                let end = Micros((s.duration_s * 1e6).round() as u64);
                let mut landing_seen = false;
                sim.run_until(end, |r| {
                    let landing = matches!(
                        r.status().expect("core part").nav,
                        NavState::Rtl | NavState::Land | NavState::LandNow
                    );
                    landing_seen |= landing;
                    let mean = r.motors.iter().sum::<f32>() / r.motors.len() as f32;
                    // The minimum thrust is a motor command of about 0.14.
                    if r.armed && landing && -r.position.z > 0.2 {
                        assert!(
                            mean > 0.2,
                            "{name} ({air}) index {index}: thrust cut {:.2} m up at {:.2} s (nav {:?})",
                            -r.position.z,
                            r.t.as_secs_f64(),
                            r.status().expect("core part").nav
                        );
                    }
                });
                assert!(landing_seen, "{name} ({air}) index {index} never landed");
            }
        }
    }
}
