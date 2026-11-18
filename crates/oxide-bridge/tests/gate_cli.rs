// SPDX-License-Identifier: AGPL-3.0-or-later
//! `oxide-bridge gate verdict` as a program: its exit codes (0 passed, 4 not
//! passed, 2 no verdict) and its report. No binary needed.

#[path = "support/gate_fixture.rs"]
mod fx;

use std::process::Command;

use fx::{F405, Fixture};

fn verdict(f: &Fixture, report: Option<&std::path::Path>) -> (i32, String, String) {
    let mut c = Command::new(env!("CARGO_BIN_EXE_oxide-bridge"));
    c.arg("gate").arg("verdict").arg("--in").arg(&f.dir);
    if let Some(r) = report {
        c.arg("--report").arg(r);
    }
    let o = c.output().expect("runs");
    (
        o.status.code().unwrap_or(-1),
        String::from_utf8_lossy(&o.stdout).into_owned(),
        String::from_utf8_lossy(&o.stderr).into_owned(),
    )
}

#[test]
fn the_command_exits_0_when_the_gate_is_passed_and_writes_the_report() {
    let f = Fixture::full("cli-pass");
    f.write();
    let report = f.dir.join("out").join("gate-report.md");
    let (code, out, err) = verdict(&f, Some(&report));
    assert_eq!(code, 0, "{out}{err}");
    assert!(out.contains("gate 1: PASSED"), "{out}");
    let md = std::fs::read_to_string(&report).expect("report");
    assert!(md.contains("**Verdict: PASSED**"));
    assert!(f.dir.join("out/gate-report-cumulative.svg").is_file());
    assert!(f.dir.join("out/gate-report-scatter.svg").is_file());
    f.remove();
}

#[test]
fn the_command_exits_4_when_the_gate_is_not_passed() {
    let mut f = Fixture::full("cli-fail");
    f.row("gust-hold", F405, 3).set("class", "crash");
    f.write();
    let (code, out, _) = verdict(&f, None);
    assert_eq!(code, 4, "{out}");
    assert!(out.contains("gate 1: NOT PASSED"), "{out}");
    assert!(out.contains("gust-hold core-f405 index 3"), "{out}");
    f.remove();
}

#[test]
fn the_command_exits_2_with_no_verdict_and_no_report_when_a_row_is_missing() {
    let mut f = Fixture::small("cli-missing");
    f.tables
        .get_mut(&("gust-hold".to_string(), F405.to_string()))
        .expect("table")
        .pop();
    f.write();
    let report = f.dir.join("r.md");
    let (code, out, err) = verdict(&f, Some(&report));
    assert_eq!(code, 2, "{out}{err}");
    assert!(out.contains("gust-hold core-f405 index 19"), "{out}");
    assert!(out.contains("no verdict"), "{out}");
    assert!(!out.contains("PASSED"), "{out}");
    assert!(!report.exists(), "a report is written for a verdict only");
    f.remove();
}

#[test]
fn the_command_exits_2_for_a_directory_that_is_not_a_gate() {
    let dir = std::env::temp_dir().join(format!("oxide-gate-cli-empty-{}", std::process::id()));
    std::fs::create_dir_all(&dir).expect("dir");
    let o = Command::new(env!("CARGO_BIN_EXE_oxide-bridge"))
        .args(["gate", "verdict", "--in"])
        .arg(&dir)
        .output()
        .expect("runs");
    assert_eq!(o.status.code(), Some(2));
    let _ = std::fs::remove_dir_all(&dir);
}
