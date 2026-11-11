// SPDX-License-Identifier: AGPL-3.0-or-later
//! The scenario files in `scenarios/` against the Rust definitions they were
//! exported from (ADR-0004 §7): same flight, bit for bit.

use std::path::Path;

use oxide_core::schedule::{F405_LEAN, H743_FULL, Profile};
use oxide_core::time::Micros;
use oxide_sim::fault_scenarios::{self, FaultScenario};
use oxide_sim::metrics::HoldCriteria;
use oxide_sim::montecarlo::run_indexed;
use oxide_sim::scenario::{GUST_HOLD_ID, HoverStep, gust_hold};
use oxide_sim::scenario_file::{self, ScenarioFile};
use oxide_sim::{Airframe, Simulation};

fn load(name: &str) -> ScenarioFile {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../scenarios")
        .join(format!("{name}.toml"));
    let f = scenario_file::load_path(&path).unwrap_or_else(|e| panic!("{e}"));
    assert_eq!(f.name, name, "the file name and the name key differ");
    f
}

/// What the Rust definition of a scenario says.
enum Definition {
    Fault(Box<FaultScenario>),
    HoverStep,
    GustHold,
}

fn definitions() -> Vec<(String, Definition)> {
    let mut out = vec![
        ("hover-step".to_string(), Definition::HoverStep),
        (GUST_HOLD_ID.to_string(), Definition::GustHold),
    ];
    for s in fault_scenarios::all()
        .into_iter()
        .chain(fault_scenarios::pairs())
    {
        out.push((s.name.to_string(), Definition::Fault(Box::new(s))));
    }
    out
}

/// The trace hash of the flight built from the Rust definition.
fn definition_hash(d: &Definition, profile: Profile, index: u64) -> u64 {
    let airframe = Airframe::reference_quad().expect("airframe");
    match d {
        Definition::Fault(s) => {
            let mut sim = fault_scenarios::build(s, profile, index).expect("builds");
            sim.run_until(Micros((s.script.duration_s * 1e6).round() as u64), |_| {});
            sim.trace_hash()
        }
        Definition::HoverStep => {
            let pilot = HoverStep::default();
            let end = Micros((pilot.duration_s * 1e6).round() as u64);
            let mut sim = Simulation::<_, 4>::new(&airframe, profile, pilot).expect("builds");
            sim.run_until(end, |_| {});
            sim.trace_hash()
        }
        Definition::GustHold => {
            let (pilot, options) = gust_hold(index);
            let end = Micros((pilot.duration_s * 1e6).round() as u64);
            let mut sim = Simulation::<_, 4>::with_options(&airframe, profile, pilot, options)
                .expect("builds");
            sim.run_until(end, |_| {});
            sim.trace_hash()
        }
    }
}

fn file_hash(f: &ScenarioFile, profile: Profile, index: u64) -> u64 {
    let mut sim = f.build(profile, index).expect("builds");
    sim.run_until(Micros((f.duration_s * 1e6).round() as u64), |_| {});
    sim.trace_hash()
}

/// What the file says must equal what the definition says.
fn check_data(name: &str, f: &ScenarioFile, d: &Definition) {
    match d {
        Definition::Fault(s) => {
            assert_eq!(f.expect.as_ref(), Some(&s.expect), "{name}: expect");
            assert_eq!(f.hold, None, "{name}: hold");
            assert_eq!(f.pilot, s.script, "{name}: pilot");
            assert_eq!(f.faults, s.faults, "{name}: faults");
            assert_eq!(f.links, s.links, "{name}: links");
            assert_eq!(f.about, s.about, "{name}: about");
        }
        Definition::HoverStep => {
            assert_eq!(f.expect, None, "{name}: expect");
            assert_eq!(f.hold, None, "{name}: hold");
        }
        Definition::GustHold => {
            assert_eq!(f.expect, None, "{name}: expect");
            assert_eq!(f.hold, Some(HoldCriteria::default()), "{name}: hold");
        }
    }
}

/// Flies every scenario on each `(profile, index)` of `cases` from the file
/// and from the definition and collects the differences.
fn compare(cases: &[(Profile, u64)]) {
    let defs = definitions();
    assert_eq!(defs.len(), 43);
    let jobs: Vec<(usize, Profile, u64)> = (0..defs.len())
        .flat_map(|i| cases.iter().map(move |&(p, k)| (i, p, k)))
        .collect();
    let results = run_indexed(jobs.len(), 0, |j| {
        let (i, profile, index) = jobs[j];
        let (name, def) = &defs[i];
        let f = load(name);
        check_data(name, &f, def);
        (
            file_hash(&f, profile, index),
            definition_hash(def, profile, index),
        )
    });
    let mut bad = Vec::new();
    for ((i, profile, index), (from_file, from_code)) in jobs.iter().zip(results) {
        if from_file != from_code {
            bad.push(format!(
                "{} on {} index {index}: file {from_file:016x}, definition {from_code:016x}",
                defs[*i].0, profile.name
            ));
        }
    }
    assert!(
        bad.is_empty(),
        "{} flights differ:\n{}",
        bad.len(),
        bad.join("\n")
    );
}

/// The quick form of the proof: every file on the lean profile, index 0. The
/// full form below adds index 1 and the H743 profile.
#[test]
fn every_scenario_file_flies_the_same_flight_as_its_definition() {
    compare(&[(F405_LEAN, 0)]);
}

/// The proof: f405 index 0 and 1, h743 index 0, all 43 scenarios. About a
/// minute in the test profile: `cargo test --release -p oxide-sim --test
/// scenario_files -- --ignored`.
#[test]
#[ignore = "the full sweep takes about a minute"]
fn every_scenario_file_flies_the_same_flight_as_its_definition_full() {
    compare(&[(F405_LEAN, 0), (F405_LEAN, 1), (H743_FULL, 0)]);
}
