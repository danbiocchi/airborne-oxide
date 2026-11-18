// SPDX-License-Identifier: AGPL-3.0-or-later
//! `oxide-bridge gate fly` and `gate verdict` on the real ArduCopter
//! simulator program, on a small plan: a directory with `gust-hold` cut to 3
//! indices and `gps-loss` cut to 2. The real gate is flown by the planner on a
//! clean commit, not here.
//!
//! Every test here needs the program and is ignored by default: set
//! `OXIDE_ARDUCOPTER` to its path and run with `--ignored`, `--release` and
//! `--test-threads 1`. The instance numbers are the range of
//! `OXIDE_ARDUCOPTER_INSTANCES` (see `tests/support/pool.rs`).
//!
//! When the working tree has uncommitted changes (or the program was built at
//! another commit), the test flies with `--allow-dirty` and the verdict must
//! refuse the tables for that reason and no other; on a clean commit the
//! verdict is a real one, never a pass because the plan is a trial.

use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

use oxide_bridge::gate::fly::{schedule, table_complete, tree_state};
use oxide_bridge::gate::plan::Plan;
use oxide_bridge::sitl::PINNED;

#[allow(dead_code)]
#[path = "support/pool.rs"]
mod pool;

fn binary() -> PathBuf {
    match std::env::var_os("OXIDE_ARDUCOPTER") {
        Some(p) if !p.is_empty() => PathBuf::from(p),
        _ => panic!(
            "OXIDE_ARDUCOPTER is not set: set it to the path of the ArduCopter SITL V4.7.2 \
             binary (the one with SHA-256 {}) to run these tests",
            PINNED.sha256
        ),
    }
}

fn repo_scenarios() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../scenarios")
}

/// A scenario directory with `gust-hold` on 3 indices and `gps-loss` on 2.
fn small_directory(tag: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("oxide-gate-real-{tag}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).expect("dir");
    for (name, was, now) in [
        ("gust-hold", "seeds = 200", "seeds = 3"),
        ("gps-loss", "seeds = 10", "seeds = 2"),
    ] {
        let text = std::fs::read_to_string(repo_scenarios().join(format!("{name}.toml")))
            .expect("scenario");
        assert!(text.contains(was), "{name} has no {was:?}");
        std::fs::write(dir.join(format!("{name}.toml")), text.replace(was, now)).expect("write");
    }
    dir
}

/// Whether the gate has to be flown with `--allow-dirty`: the tree has
/// uncommitted changes, or this build is not of its commit.
fn needs_allow_dirty() -> bool {
    match tree_state(Path::new(".")) {
        Ok(t) => t.dirty || t.commit != env!("OXIDE_GIT_HASH"),
        Err(_) => true,
    }
}

fn gate_fly_command(scenarios: &Path, out: &Path) -> Command {
    let (lo, hi) = pool::range();
    let jobs = usize::from(hi - lo).min(1) + 1;
    let mut c = Command::new(env!("CARGO_BIN_EXE_oxide-bridge"));
    c.arg("gate")
        .arg("fly")
        .arg("--scenarios")
        .arg(scenarios)
        .arg("--out")
        .arg(out)
        .arg("--binary")
        .arg(binary())
        .arg("--reduced")
        .arg("--jobs")
        .arg(jobs.to_string())
        .arg("--instances")
        .arg(format!("{lo}-{hi}"));
    if needs_allow_dirty() {
        c.arg("--allow-dirty");
    }
    c
}

fn verdict(out: &Path) -> (i32, String) {
    let o = Command::new(env!("CARGO_BIN_EXE_oxide-bridge"))
        .args(["gate", "verdict", "--in"])
        .arg(out)
        .arg("--report")
        .arg(out.join("report.md"))
        .output()
        .expect("runs");
    (
        o.status.code().unwrap_or(-1),
        String::from_utf8_lossy(&o.stdout).into_owned(),
    )
}

/// The verdict of a trial: 0 or 4 on a clean commit, 2 for the dirty tree
/// and nothing else when it is dirty.
fn check_verdict(out: &Path) {
    let (code, text) = verdict(out);
    println!("{text}");
    if needs_allow_dirty() {
        assert_eq!(code, 2, "{text}");
        for line in text.lines().filter(|l| l.starts_with("  - ")) {
            assert!(
                line.contains("dirty tree") || line.contains("commit"),
                "a problem other than the dirty tree: {line}"
            );
        }
    } else {
        assert!(code == 0 || code == 4, "exit code {code}\n{text}");
        // The plan is a trial: never a pass.
        assert_eq!(code, 4, "{text}");
        assert!(out.join("report.md").is_file());
        assert!(out.join("report-cumulative.svg").is_file());
        assert!(out.join("report-scatter.svg").is_file());
    }
}

fn all_tables_complete(plan: &Plan, out: &Path) {
    for b in schedule(plan) {
        assert!(
            table_complete(&b.dir(out), &b.scenario, &b.autopilot.name(), b.count),
            "the table {} is not complete",
            b.label()
        );
    }
}

#[test]
#[ignore = "needs the ArduCopter program: OXIDE_ARDUCOPTER"]
fn a_small_gate_is_flown_complete_and_its_verdict_is_read_from_the_tables() {
    let scenarios = small_directory("small");
    let out = std::env::temp_dir().join(format!("oxide-gate-real-out-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&out);
    let plan = Plan::build(&scenarios, true).expect("plan");
    let start = Instant::now();
    let st = gate_fly_command(&scenarios, &out)
        .stdout(Stdio::inherit())
        .status()
        .expect("runs");
    println!(
        "MEASURED wall time of gate fly: {:.1} s",
        start.elapsed().as_secs_f64()
    );
    assert_eq!(st.code(), Some(0));
    all_tables_complete(&plan, &out);
    let info = std::fs::read_to_string(out.join("gate.txt")).expect("gate.txt");
    for needle in [
        "commit ",
        "binary sha256 ",
        "param file base.parm sha256 ",
        "scenario file gust-hold sha256 ",
        "cores ",
        "ended 2",
    ] {
        assert!(
            info.contains(needle),
            "{needle:?} is not in gate.txt:\n{info}"
        );
    }
    assert!(
        info.contains(PINNED.sha256),
        "the binary's checksum: {info}"
    );
    check_verdict(&out);
    let _ = std::fs::remove_dir_all(&out);
    let _ = std::fs::remove_dir_all(&scenarios);
}

#[test]
#[ignore = "needs the ArduCopter program: OXIDE_ARDUCOPTER"]
fn an_interrupted_gate_is_continued_and_reuses_the_tables_of_our_core() {
    let scenarios = small_directory("interrupted");
    let out = std::env::temp_dir().join(format!("oxide-gate-real-cont-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&out);
    let plan = Plan::build(&scenarios, true).expect("plan");
    let core: Vec<_> = schedule(&plan)
        .into_iter()
        .filter(|b| b.autopilot != oxide_bridge::runner::Autopilot::ArduCopter)
        .collect();
    assert!(!core.is_empty());
    // Start it and end it as soon as our core's tables are there.
    let mut child = gate_fly_command(&scenarios, &out)
        .stdout(Stdio::null())
        .spawn()
        .expect("starts");
    let begun = Instant::now();
    loop {
        let done = core
            .iter()
            .all(|b| table_complete(&b.dir(&out), &b.scenario, &b.autopilot.name(), b.count));
        if done {
            break;
        }
        assert!(
            begun.elapsed() < Duration::from_secs(600),
            "our core's tables never came"
        );
        std::thread::sleep(Duration::from_millis(200));
    }
    let _ = child.kill();
    let _ = child.wait();
    // The children of the batch that was running end by themselves: wait until
    // every run directory has its row.
    let waited = Instant::now();
    loop {
        let mut pending = false;
        for e in std::fs::read_dir(&out).into_iter().flatten().flatten() {
            for t in std::fs::read_dir(e.path()).into_iter().flatten().flatten() {
                for r in std::fs::read_dir(t.path()).into_iter().flatten().flatten() {
                    let name = r.file_name().to_string_lossy().into_owned();
                    if name.starts_with("run-")
                        && !name.ends_with(".failed")
                        && !r.path().join("row.csv").is_file()
                    {
                        pending = true;
                    }
                }
            }
        }
        if !pending {
            break;
        }
        assert!(
            waited.elapsed() < Duration::from_secs(600),
            "a child never ended"
        );
        std::thread::sleep(Duration::from_millis(500));
    }
    std::thread::sleep(Duration::from_secs(3));
    // Continue: the same command.
    let st = gate_fly_command(&scenarios, &out)
        .stdout(Stdio::inherit())
        .status()
        .expect("runs");
    assert_eq!(st.code(), Some(0));
    all_tables_complete(&plan, &out);
    let info = std::fs::read_to_string(out.join("gate.txt")).expect("gate.txt");
    for b in &core {
        assert!(
            info.lines().any(|l| l == format!("reused {}", b.label())),
            "{} was flown again: {info}",
            b.label()
        );
    }
    check_verdict(&out);
    let _ = std::fs::remove_dir_all(&out);
    let _ = std::fs::remove_dir_all(&scenarios);
}
