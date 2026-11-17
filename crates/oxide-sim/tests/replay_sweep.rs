// SPDX-License-Identifier: AGPL-3.0-or-later
//! Every scenario file replays (ADR-0005 §9, task P2): each file in
//! `scenarios/`, on both chip profiles, seed 0, is flown with a replay-level
//! log in memory and replayed; every tick must be identical. Release only:
//! `cargo test --release -p oxide-sim --test replay_sweep -- --ignored
//! every_scenario_file_replays_identically_on_both_profiles --nocapture`.

use std::path::{Path, PathBuf};
use std::time::Instant;

use oxide_core::schedule::{F405_LEAN, H743_FULL, Profile};
use oxide_sim::flight_log::{Level, LogBuild, SharedBuffer};
use oxide_sim::montecarlo::run_indexed;
use oxide_sim::replay::{ReplayOptions, Stop, replay};
use oxide_sim::scenario_file::{self, ScenarioFile};

fn files() -> Vec<ScenarioFile> {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scenarios");
    let mut paths: Vec<PathBuf> = std::fs::read_dir(dir)
        .expect("scenarios/")
        .map(|e| e.expect("entry").path())
        .filter(|p| p.extension().is_some_and(|x| x == "toml"))
        .collect();
    paths.sort();
    paths
        .iter()
        .map(|p| scenario_file::load_path(p).unwrap_or_else(|e| panic!("{e}")))
        .collect()
}

/// One flight's result: its label, ticks flown, bytes of the log, and what
/// the replay said (`None` when it was identical over every tick).
struct Proved {
    label: String,
    flown: u64,
    bytes: usize,
    problem: Option<String>,
}

fn prove(f: &ScenarioFile, profile: Profile) -> Proved {
    let label = format!("{} on {}", f.name, profile.name);
    let mut sim = f.build(profile, 0).expect("builds");
    let buf = SharedBuffer::new();
    sim.start_log(
        Level::Replay,
        &LogBuild::sim("sweep"),
        Box::new(buf.clone()),
    )
    .expect("the replay level starts");
    let mut flown = 0_u64;
    sim.run_until(f.end(), |_| flown += 1);
    sim.finish_log().expect("finishes");
    let bytes = buf.bytes();
    let problem = match replay(&bytes, &ReplayOptions::default()) {
        Err(e) => Some(format!("cannot replay: {e}")),
        Ok(r) if r.ticks != flown => Some(format!("replayed {} of {flown} ticks", r.ticks)),
        Ok(r) if !r.identical() || r.stop != Stop::End => Some(r.lines().join("; ")),
        Ok(_) => None,
    };
    Proved {
        label,
        flown,
        bytes: bytes.len(),
        problem,
    }
}

#[test]
#[ignore = "flies every scenario file on both profiles with a replay-level log: release only"]
fn every_scenario_file_replays_identically_on_both_profiles() {
    let started = Instant::now();
    let files = files();
    assert_eq!(files.len(), 43);
    let jobs: Vec<(usize, Profile)> = (0..files.len())
        .flat_map(|i| [(i, F405_LEAN), (i, H743_FULL)])
        .collect();
    let results = run_indexed(jobs.len(), 0, |j| {
        let (i, profile) = jobs[j];
        prove(&files[i], profile)
    });
    let bad: Vec<String> = results
        .iter()
        .filter_map(|r| r.problem.as_ref().map(|p| format!("{}: {p}", r.label)))
        .collect();
    let ticks: u64 = results.iter().map(|r| r.flown).sum();
    let bytes: usize = results.iter().map(|r| r.bytes).sum();
    println!(
        "replay sweep: {} flights, {ticks} ticks, {bytes} bytes, {:.1} s",
        results.len(),
        started.elapsed().as_secs_f64()
    );
    assert!(
        bad.is_empty(),
        "{} flights differ:\n{}",
        bad.len(),
        bad.join("\n")
    );
}
