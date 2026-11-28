// SPDX-License-Identifier: AGPL-3.0-or-later
//! The scenario files in `scenarios/` are the only definition of the 44
//! scenarios (ADR-0004 §7). Task S1 proved that each file flies the same
//! flight as the Rust definition it was exported from; the definitions are
//! gone, so the trace hashes of those flights are pinned in
//! `tests/data/scenario_hashes.txt` and every file must still produce them.
//! A change that moves a hash moves a flight: it is a decision, not a
//! side effect. Regenerate the table only for such a decision:
//! `OXIDE_REGENERATE=1 cargo test --release -p oxide-sim --test scenario_files
//! -- --ignored regenerate_scenario_hashes`.

use std::path::{Path, PathBuf};

use oxide_core::schedule::{F405_LEAN, H743_FULL, Profile};
use oxide_core::time::Micros;
use oxide_sim::montecarlo::run_indexed;
use oxide_sim::scenario_file::{self, ScenarioFile};

const TABLE: &str = "tests/data/scenario_hashes.txt";

/// The flights pinned for every file: profile and run index.
fn cases() -> [(Profile, u64); 3] {
    [(F405_LEAN, 0), (F405_LEAN, 1), (H743_FULL, 0)]
}

fn dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scenarios")
}

/// Every file in `scenarios/`, sorted by name.
fn files() -> Vec<ScenarioFile> {
    let mut paths: Vec<PathBuf> = std::fs::read_dir(dir())
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

fn flight_hash(f: &ScenarioFile, profile: Profile, index: u64) -> u64 {
    let mut sim = f.build(profile, index).expect("builds");
    sim.run_until(Micros((f.duration_s * 1e6).round() as u64), |_| {});
    sim.trace_hash()
}

/// `name profile index hash` for each file and case, in file order.
fn pinned() -> Vec<(String, String, u64, u64)> {
    let text = std::fs::read_to_string(Path::new(env!("CARGO_MANIFEST_DIR")).join(TABLE))
        .expect("the pinned hashes");
    text.lines()
        .map(|l| {
            let w: Vec<&str> = l.split(' ').collect();
            assert_eq!(w.len(), 4, "bad line {l:?}");
            (
                w[0].to_string(),
                w[1].to_string(),
                w[2].parse().expect("index"),
                u64::from_str_radix(w[3], 16).expect("hash"),
            )
        })
        .collect()
}

/// Flies `pinned` entries selected by `keep` and compares.
fn check(keep: impl Fn(&str, u64) -> bool + Sync) {
    let files = files();
    assert_eq!(files.len(), 44);
    let jobs: Vec<_> = pinned()
        .into_iter()
        .filter(|(_, p, i, _)| keep(p, *i))
        .collect();
    let results = run_indexed(jobs.len(), 0, |j| {
        let (name, profile, index, _) = &jobs[j];
        let f = files.iter().find(|f| &f.name == name).expect("file");
        let profile = cases()
            .iter()
            .map(|c| c.0)
            .find(|p| p.name == profile)
            .expect("profile");
        flight_hash(f, profile, *index)
    });
    let bad: Vec<String> = jobs
        .iter()
        .zip(results)
        .filter(|((_, _, _, want), got)| want != got)
        .map(|((n, p, i, want), got)| {
            format!("{n} on {p} index {i}: pinned {want:016x}, flown {got:016x}")
        })
        .collect();
    assert!(
        bad.is_empty(),
        "{} flights moved:\n{}",
        bad.len(),
        bad.join("\n")
    );
}

#[test]
fn the_pinned_table_covers_every_file_and_case() {
    let files = files();
    let table = pinned();
    assert_eq!(table.len(), files.len() * cases().len());
    for f in &files {
        for (p, i) in cases() {
            assert!(
                table
                    .iter()
                    .any(|(n, tp, ti, _)| *n == f.name && tp == p.name && *ti == i),
                "{} {} {i} is not pinned",
                f.name,
                p.name
            );
        }
    }
}

/// Quick form: every file on the lean profile, index 0.
#[test]
fn every_scenario_file_still_flies_its_pinned_flight() {
    check(|p, i| p == F405_LEAN.name && i == 0);
}

/// Full form: f405 index 0 and 1, h743 index 0.
#[test]
#[ignore = "the full sweep takes about a minute in release"]
fn every_scenario_file_still_flies_its_pinned_flight_full() {
    check(|_, _| true);
}

/// Rewrites the table, and only when `OXIDE_REGENERATE=1` is set: running
/// every ignored test of this file must never move a pin by accident.
#[test]
#[ignore = "rewrites the pinned table: only for a decided change of a flight"]
fn regenerate_scenario_hashes() {
    if std::env::var("OXIDE_REGENERATE").as_deref() != Ok("1") {
        eprintln!("regenerate_scenario_hashes: skipped, set OXIDE_REGENERATE=1 to rewrite");
        return;
    }
    let files = files();
    let jobs: Vec<(usize, Profile, u64)> = (0..files.len())
        .flat_map(|i| cases().into_iter().map(move |(p, k)| (i, p, k)))
        .collect();
    let hashes = run_indexed(jobs.len(), 0, |j| {
        let (i, p, k) = jobs[j];
        flight_hash(&files[i], p, k)
    });
    let mut out = String::new();
    for ((i, p, k), h) in jobs.iter().zip(hashes) {
        out.push_str(&format!("{} {} {k} {h:016x}\n", files[*i].name, p.name));
    }
    std::fs::write(Path::new(env!("CARGO_MANIFEST_DIR")).join(TABLE), out).expect("write");
}
