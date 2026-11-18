// SPDX-License-Identifier: AGPL-3.0-or-later
//! `gate fly` without the simulator program: its command line, the check of
//! the working tree, the order of the batches, and what makes a table
//! complete. No binary needed.

use std::path::{Path, PathBuf};
use std::process::Command;

use oxide_bridge::cli::{Cmd, parse};
use oxide_bridge::gate::cmd::GateCmd;
use oxide_bridge::gate::fly::{
    Batch, mark_dirty, refuse_other_files, schedule, status_is_dirty, table_complete, tree_state,
};
use oxide_bridge::gate::plan::Plan;
use oxide_bridge::record::{Row, header};
use oxide_bridge::runner::Autopilot;

fn scenarios_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../scenarios")
}

fn words(line: &str) -> Vec<String> {
    line.split_whitespace().map(String::from).collect()
}

fn temp(tag: &str) -> PathBuf {
    let d = std::env::temp_dir().join(format!("oxide-gate-fly-{tag}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&d);
    std::fs::create_dir_all(&d).expect("dir");
    d
}

fn git(dir: &Path, args: &[&str]) {
    let st = Command::new("git")
        .current_dir(dir)
        .args(["-c", "user.name=t", "-c", "user.email=t@example.com"])
        .args(["-c", "commit.gpgsign=false"])
        .args(args)
        .status()
        .expect("git");
    assert!(st.success(), "git {args:?}");
}

/// A repository with one committed file.
fn repo(tag: &str) -> PathBuf {
    let d = temp(tag);
    git(&d, &["init", "-q"]);
    std::fs::write(d.join("a.txt"), "one\n").expect("write");
    git(&d, &["add", "a.txt"]);
    git(&d, &["commit", "-q", "-m", "first"]);
    d
}

// ---- the command line ------------------------------------------------------

#[test]
fn gate_fly_takes_its_options_and_defaults_the_rest() {
    let Ok(Cmd::Gate(GateCmd::Fly(s))) = parse(&words("gate fly --scenarios sc --out o")) else {
        panic!("gate fly");
    };
    assert_eq!(
        (s.scenarios, s.out),
        (PathBuf::from("sc"), PathBuf::from("o"))
    );
    assert_eq!((s.binary, s.jobs, s.instances), (None, None, (0, 9)));
    assert!(!s.allow_dirty && !s.reduced);
    assert_eq!(
        s.command_line,
        "oxide-bridge gate fly --scenarios sc --out o"
    );
    let Ok(Cmd::Gate(GateCmd::Fly(s))) = parse(&words(
        "gate fly --scenarios sc --out o --binary /x/ac --jobs 3 --instances 5-9 --allow-dirty --reduced",
    )) else {
        panic!("gate fly");
    };
    assert_eq!(s.binary, Some(PathBuf::from("/x/ac")));
    assert_eq!((s.jobs, s.instances), (Some(3), (5, 9)));
    assert!(s.allow_dirty && s.reduced);
}

#[test]
fn gate_verdict_takes_its_options() {
    let Ok(Cmd::Gate(GateCmd::Verdict { input, report })) =
        parse(&words("gate verdict --in o --report r.md"))
    else {
        panic!("gate verdict");
    };
    assert_eq!(
        (input, report),
        (PathBuf::from("o"), Some(PathBuf::from("r.md")))
    );
    let Ok(Cmd::Gate(GateCmd::Verdict { report, .. })) = parse(&words("gate verdict --in o"))
    else {
        panic!("gate verdict");
    };
    assert_eq!(report, None);
}

#[test]
fn every_wrong_use_of_gate_is_one_sentence() {
    let cases = [
        ("gate", "no gate command"),
        ("gate run", "not a gate command"),
        ("gate fly --out o", "--scenarios DIR is required"),
        ("gate fly --scenarios s", "--out OUT is required"),
        ("gate fly --scenarios s --out o --jobs 0", "from 1"),
        ("gate fly --scenarios s --out o --jobs x", "--jobs"),
        (
            "gate fly --scenarios s --out o --jobs 3 --instances 0-1",
            "instance numbers",
        ),
        (
            "gate fly --scenarios s --out o --instances 3",
            "--instances",
        ),
        (
            "gate fly --scenarios s --out o --nonsense",
            "not an option of gate fly",
        ),
        ("gate fly --scenarios", "needs a value"),
        ("gate verdict", "--in OUT is required"),
        (
            "gate verdict --in o --jobs 2",
            "not an option of gate verdict",
        ),
        ("gate verdict --in", "needs a value"),
    ];
    for (line, needle) in cases {
        let Err(e) = parse(&words(line)) else {
            panic!("{line:?} should be a wrong use");
        };
        assert!(e.0.contains(needle), "{line:?}: {}", e.0);
        assert!(!e.0.contains('\n') && !e.0.is_empty());
    }
}

// ---- the working tree ------------------------------------------------------

#[test]
fn git_status_lists_a_change_or_nothing() {
    assert!(!status_is_dirty(""));
    assert!(!status_is_dirty("\n  \n"));
    assert!(status_is_dirty(" M crates/x.rs\n"));
    assert!(status_is_dirty("A  new.rs"));
}

#[test]
fn a_clean_tree_a_modified_tracked_file_and_an_untracked_file() {
    let d = repo("tree");
    let clean = tree_state(&d).expect("state");
    assert!(!clean.dirty);
    assert_eq!(clean.commit.len(), 40);
    // An untracked file is not a change to a tracked file.
    std::fs::write(d.join("new.txt"), "x").expect("write");
    assert!(!tree_state(&d).expect("state").dirty);
    // A change to a tracked file is.
    std::fs::write(d.join("a.txt"), "two\n").expect("write");
    let dirty = tree_state(&d).expect("state");
    assert!(dirty.dirty);
    assert_eq!(dirty.commit, clean.commit);
    // So is a staged one.
    git(&d, &["add", "a.txt"]);
    assert!(tree_state(&d).expect("state").dirty);
    git(&d, &["commit", "-q", "-m", "second"]);
    let now = tree_state(&d).expect("state");
    assert!(!now.dirty);
    assert_ne!(now.commit, clean.commit);
    let _ = std::fs::remove_dir_all(&d);
}

#[test]
fn a_directory_that_is_no_repository_is_a_sentence() {
    let d = temp("norepo");
    let e = tree_state(&d).expect_err("not a repository");
    assert!(e.contains("git"), "{e}");
    let _ = std::fs::remove_dir_all(&d);
}

fn gate_fly(cwd: &Path, extra: &[&str], path: Option<&str>) -> (i32, String) {
    let out = cwd.join("gate-out");
    let mut c = Command::new(env!("CARGO_BIN_EXE_oxide-bridge"));
    c.current_dir(cwd)
        .arg("gate")
        .arg("fly")
        .arg("--scenarios")
        .arg(scenarios_dir())
        .arg("--out")
        .arg(&out)
        .args(extra)
        .env_remove("OXIDE_ARDUCOPTER");
    if let Some(p) = path {
        c.env("PATH", p);
    }
    let o = c.output().expect("runs");
    (
        o.status.code().unwrap_or(-1),
        String::from_utf8_lossy(&o.stderr).into_owned(),
    )
}

#[test]
fn a_dirty_tree_is_refused_before_anything_is_flown_unless_allowed() {
    let d = repo("dirty");
    std::fs::write(d.join("a.txt"), "two\n").expect("write");
    let (code, err) = gate_fly(&d, &["--binary", "/no/such/arducopter"], None);
    assert_eq!(code, 1, "{err}");
    assert!(err.contains("uncommitted changes"), "{err}");
    assert!(!d.join("gate-out").exists(), "nothing was written");
    // With --allow-dirty the check is passed (it stops later: no such binary).
    let (code, err) = gate_fly(
        &d,
        &["--binary", "/no/such/arducopter", "--allow-dirty"],
        None,
    );
    assert_eq!(code, 1, "{err}");
    assert!(!err.contains("uncommitted changes"), "{err}");
    assert!(err.contains("/no/such/arducopter"), "{err}");
    let _ = std::fs::remove_dir_all(&d);
}

#[test]
fn a_clean_tree_that_is_not_the_one_the_program_was_built_at_is_refused() {
    // The temporary repository's commit is not this program's.
    let d = repo("builtat");
    let (code, err) = gate_fly(&d, &["--binary", "/no/such/arducopter"], None);
    assert_eq!(code, 1, "{err}");
    assert!(err.contains("was built at"), "{err}");
    let _ = std::fs::remove_dir_all(&d);
}

#[test]
fn where_git_is_not_found_it_says_so_in_a_sentence() {
    let d = temp("nogit");
    let empty = temp("emptypath");
    let (code, err) = gate_fly(
        &d,
        &["--binary", "/no/such/arducopter"],
        Some(empty.to_str().expect("path")),
    );
    assert_eq!(code, 1, "{err}");
    assert!(err.contains("git was not found"), "{err}");
    // A trial may go on without it, and stops at the binary.
    let (code, err) = gate_fly(
        &d,
        &["--binary", "/no/such/arducopter", "--allow-dirty"],
        Some(empty.to_str().expect("path")),
    );
    assert_eq!(code, 1, "{err}");
    assert!(err.contains("/no/such/arducopter"), "{err}");
    let _ = std::fs::remove_dir_all(&d);
    let _ = std::fs::remove_dir_all(&empty);
}

// ---- the order of the batches ----------------------------------------------

#[test]
fn our_core_is_flown_first_then_arducopter_then_the_flights_flown_twice() {
    let plan = Plan::build(&scenarios_dir(), false).expect("plan");
    let all = schedule(&plan);
    let first_ac = all
        .iter()
        .position(|b| b.autopilot == Autopilot::ArduCopter)
        .expect("an ArduCopter batch");
    let first_twice = all.iter().position(|b| b.twice).expect("a twice batch");
    assert!(
        all[..first_ac]
            .iter()
            .all(|b| b.autopilot != Autopilot::ArduCopter)
    );
    assert!(
        all[first_ac..first_twice]
            .iter()
            .all(|b| b.autopilot == Autopilot::ArduCopter && !b.twice)
    );
    assert!(all[first_twice..].iter().all(|b| b.twice));
    // Counts: 4 + 3 * (every other scenario) tables, six twice tables.
    let others = plan.entries.len() - 1;
    assert_eq!(all.len(), 4 + 3 * others + 6);
    assert_eq!(all.iter().filter(|b| b.twice).count(), 6);
    // Every table once.
    let mut labels: Vec<String> = all.iter().map(Batch::label).collect();
    let n = labels.len();
    labels.sort();
    labels.dedup();
    assert_eq!(labels.len(), n);
    // The truth flights are gust-hold's only.
    assert_eq!(
        all.iter()
            .filter(|b| b.autopilot == Autopilot::CoreTruth)
            .count(),
        1
    );
    // Twice flights are ArduCopter's.
    assert!(
        all.iter()
            .filter(|b| b.twice)
            .all(|b| b.autopilot == Autopilot::ArduCopter)
    );
}

// ---- a table that need not be flown again ----------------------------------

fn write_table(dir: &Path, rows: &[Row]) {
    std::fs::create_dir_all(dir).expect("dir");
    let mut text = format!("{}\n", header());
    for r in rows {
        text.push_str(&r.line());
        text.push('\n');
    }
    std::fs::write(dir.join("flights.csv"), text).expect("write");
}

fn row(scenario: &str, autopilot: &str, index: u64, started: bool) -> Row {
    let mut r = Row::empty();
    r.set("scenario", scenario);
    r.set("autopilot", autopilot);
    r.set("index", &index.to_string());
    r.set("started", if started { "yes" } else { "no" });
    r
}

#[test]
fn a_table_is_complete_when_every_index_has_one_started_row() {
    let d = temp("complete");
    let ok: Vec<Row> = (0..3).map(|i| row("s", "core-f405", i, true)).collect();
    write_table(&d, &ok);
    assert!(table_complete(&d, "s", "core-f405", 3));
    // Fewer rows than planned, or more indices than the table has.
    assert!(!table_complete(&d, "s", "core-f405", 4));
    assert!(
        !table_complete(&d, "s", "core-f405", 2),
        "a row outside the plan"
    );
    // Another scenario or autopilot.
    assert!(!table_complete(&d, "t", "core-f405", 3));
    assert!(!table_complete(&d, "s", "arducopter", 3));
    // A run that did not start (the gate stopped there).
    let mut bad = ok.clone();
    bad[1] = row("s", "core-f405", 1, false);
    write_table(&d, &bad);
    assert!(!table_complete(&d, "s", "core-f405", 3));
    // A doubled row.
    let mut twice = ok.clone();
    twice[2] = row("s", "core-f405", 1, true);
    write_table(&d, &twice);
    assert!(!table_complete(&d, "s", "core-f405", 3));
    // No table, or one that is not a table.
    assert!(!table_complete(&d.join("nowhere"), "s", "core-f405", 3));
    std::fs::write(d.join("flights.csv"), "nonsense\n").expect("write");
    assert!(!table_complete(&d, "s", "core-f405", 3));
    let _ = std::fs::remove_dir_all(&d);
}

#[test]
fn the_dirty_note_is_added_to_every_row_once() {
    let d = temp("mark");
    let mut a = row("s", "core-f405", 0, true);
    a.set("note", "first problem: x");
    let b = row("s", "core-f405", 1, true);
    write_table(&d, &[a, b]);
    mark_dirty(&d).expect("marks");
    mark_dirty(&d).expect("marks again");
    let text = std::fs::read_to_string(d.join("flights.csv")).expect("read");
    let rows: Vec<Row> = text
        .lines()
        .skip(1)
        .map(|l| Row::parse(l).expect("row"))
        .collect();
    assert_eq!(rows[0].get("note"), "first problem: x; dirty tree");
    assert_eq!(rows[1].get("note"), "dirty tree");
    let _ = std::fs::remove_dir_all(&d);
}

// ---- the files the gate flies ----------------------------------------------

fn settings(line: &str) -> oxide_bridge::gate::fly::Settings {
    match parse(&words(line)).expect("parses") {
        Cmd::Gate(GateCmd::Fly(s)) => s,
        other => panic!("not gate fly: {other:?}"),
    }
}

#[test]
fn only_a_trial_may_fly_files_that_are_not_the_commits() {
    let others =
        vec!["the scenario file gust-hold has SHA-256 aa and the commit's is bb".to_string()];
    // The gate itself: refused, with the differences in the sentence.
    let e = refuse_other_files(&settings("gate fly --scenarios s --out o"), &others)
        .expect_err("refused");
    assert!(
        e.contains("the commit's own files only") && e.contains("gust-hold"),
        "{e}"
    );
    // A trial is let through; so is a gate with nothing different.
    for line in [
        "gate fly --scenarios s --out o --reduced",
        "gate fly --scenarios s --out o --allow-dirty",
    ] {
        assert_eq!(
            refuse_other_files(&settings(line), &others),
            Ok(()),
            "{line}"
        );
    }
    assert_eq!(
        refuse_other_files(&settings("gate fly --scenarios s --out o"), &[]),
        Ok(())
    );
}

/// A scenario directory that is the repository's with one file edited.
fn edited_scenarios(tag: &str) -> PathBuf {
    let d = temp(tag);
    for e in std::fs::read_dir(scenarios_dir()).expect("read").flatten() {
        std::fs::copy(e.path(), d.join(e.file_name())).expect("copy");
    }
    let hover = d.join("hover-step.toml");
    let mut text = std::fs::read_to_string(&hover).expect("hover-step");
    text.push_str("\n# an edit\n");
    std::fs::write(&hover, text).expect("write");
    d
}

#[test]
fn the_gate_is_not_flown_from_scenario_files_that_are_not_the_commits() {
    // The refusal comes before the simulator program is started, so a file
    // that is not one is enough. It needs a program built at a commit: on a
    // tree with changes the working-tree check refuses first, as it should.
    if oxide_bridge::gate::verdict::own_commit().is_none() {
        return;
    }
    let scen = edited_scenarios("edited-scen");
    let out = temp("edited-out").join("gate-out");
    let dummy = temp("edited-bin").join("arducopter");
    std::fs::write(&dummy, "not a simulator\n").expect("dummy");
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let o = Command::new(env!("CARGO_BIN_EXE_oxide-bridge"))
        .current_dir(root)
        .args(["gate", "fly", "--scenarios"])
        .arg(&scen)
        .arg("--out")
        .arg(&out)
        .arg("--binary")
        .arg(&dummy)
        .env_remove("OXIDE_ARDUCOPTER")
        .output()
        .expect("runs");
    let err = String::from_utf8_lossy(&o.stderr);
    assert_eq!(o.status.code(), Some(1), "{err}");
    assert!(err.contains("the commit's own files only"), "{err}");
    assert!(err.contains("the scenario file hover-step"), "{err}");
    assert!(!out.exists(), "nothing was written before the refusal");
    // The same files as a trial are let through the refusal (and stop later).
    let o = Command::new(env!("CARGO_BIN_EXE_oxide-bridge"))
        .current_dir(PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../.."))
        .args(["gate", "fly", "--reduced", "--scenarios"])
        .arg(&scen)
        .arg("--out")
        .arg(&out)
        .arg("--binary")
        .arg("/no/such/arducopter")
        .env_remove("OXIDE_ARDUCOPTER")
        .output()
        .expect("runs");
    let err = String::from_utf8_lossy(&o.stderr);
    assert!(!err.contains("the commit's own files only"), "{err}");
    let _ = std::fs::remove_dir_all(&scen);
}
