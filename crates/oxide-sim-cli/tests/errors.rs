// SPDX-License-Identifier: AGPL-3.0-or-later
//! What the commands do when something goes wrong, from the real binary: the
//! exit status (0 passed, 1 a failing run, 2 an error, for every command), the
//! file named in a write error, and no log file left behind by a command that
//! failed before its flight.

use std::path::PathBuf;
use std::process::{Command, Output};

/// A path in a directory of this test file's own (not shared with
/// `exit_codes.rs`, which removes its directory when it ends).
fn temp(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("oxide-sim-errors-{}", std::process::id()));
    std::fs::create_dir_all(&dir).expect("temp dir");
    dir.join(name)
}

fn text(p: &std::path::Path) -> String {
    p.to_string_lossy().into_owned()
}

fn run(args: &[&str]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_oxide-sim"))
        .args(args)
        .output()
        .expect("runs")
}

fn code(o: &Output) -> i32 {
    o.status.code().expect("exited")
}

fn stderr(o: &Output) -> String {
    String::from_utf8_lossy(&o.stderr).into_owned()
}

fn stdout(o: &Output) -> String {
    String::from_utf8_lossy(&o.stdout).into_owned()
}

/// A CSV path that cannot be created.
fn bad_csv() -> String {
    text(&temp("no-such-dir").join("x.csv"))
}

/// What this system says when a file is created in a directory that does not
/// exist (the wording and the number differ between Linux and Windows).
fn create_error(path: &str) -> String {
    std::fs::File::create(path)
        .expect_err("the directory does not exist")
        .to_string()
}

#[test]
fn a_command_that_fails_before_the_flight_leaves_no_log_file() {
    for args in [
        vec!["scenario", "gcs-loss"],
        vec!["scenario", "gcs-loss", "--log-level", "replay"],
        vec!["run"],
    ] {
        let log = temp(&format!("left-{}.ulg", args.len()));
        let _ = std::fs::remove_file(&log);
        let csv = bad_csv();
        let mut a = args.clone();
        let log_text = text(&log);
        // The level goes after the file, the file after the command.
        if a.contains(&"--log-level") {
            a.insert(2, "--log");
            a.insert(3, &log_text);
        } else {
            a.extend(["--log", &log_text]);
        }
        a.extend(["--csv", &csv]);
        let o = run(&a);
        assert_eq!(code(&o), 2, "{a:?}: {}", stderr(&o));
        assert!(
            stderr(&o).starts_with(&format!("error: {csv}: {}", create_error(&csv))),
            "{a:?}: {}",
            stderr(&o)
        );
        assert!(!log.exists(), "{a:?} left {log:?}");
    }
}

#[test]
fn an_unwritable_log_path_is_an_error_and_names_the_path() {
    let log = text(&temp("no-such-dir").join("f.ulg"));
    for args in [
        vec!["scenario", "gcs-loss", "--log", &log],
        vec!["run", "--log", &log],
    ] {
        let o = run(&args);
        assert_eq!(code(&o), 2, "{args:?}: {}", stderr(&o));
        assert!(
            stderr(&o).starts_with(&format!("error: {log}: {}", create_error(&log))),
            "{}",
            stderr(&o)
        );
    }
}

#[cfg(target_os = "linux")]
#[test]
fn a_log_write_error_names_the_file_and_exits_2() {
    for args in [
        vec!["scenario", "gcs-loss", "--log", "/dev/full"],
        vec![
            "scenario",
            "gcs-loss",
            "--log",
            "/dev/full",
            "--log-level",
            "replay",
        ],
        vec!["run", "--log", "/dev/full"],
    ] {
        let o = run(&args);
        assert_eq!(code(&o), 2, "{args:?}: {}", stderr(&o));
        assert_eq!(
            stderr(&o),
            "error: /dev/full: No space left on device (os error 28)\n",
            "{args:?}"
        );
    }
    // Nothing removed the device.
    assert!(std::path::Path::new("/dev/full").exists());
}

/// A CSV that fails after the flight (the header fits in the buffer): the
/// summary and the verdict are still printed, then the error, exit 2.
#[cfg(target_os = "linux")]
#[test]
fn a_csv_write_error_after_the_flight_still_prints_the_summary() {
    let o = run(&["scenario", "gcs-loss", "--csv", "/dev/full"]);
    assert_eq!(code(&o), 2, "{}", stderr(&o));
    let out = stdout(&o);
    assert!(out.contains("trace hash "), "{out}");
    assert!(out.contains("result     PASS"), "{out}");
    assert_eq!(
        stderr(&o),
        "error: /dev/full: No space left on device (os error 28)\n"
    );
    assert!(std::path::Path::new("/dev/full").exists());
}

#[test]
fn an_error_exits_2_for_every_command_and_a_failing_run_exits_1() {
    let csv = bad_csv();
    for args in [
        vec!["run", "--csv", &csv],
        vec!["run", "--airframe", "/no/such/airframe.toml"],
        vec!["hash", "--scenario", "hover-step", "--csv", &csv],
        vec!["montecarlo", "--seeds", "1", "--param", "RATE_ROLL_P=1e9"],
        vec!["scenario", "gcs-loss", "--csv", &csv],
        vec!["scenario", "no-such-scenario"],
        vec!["scenarios", "/no/such/dir"],
        vec!["params", "bogus"],
        vec!["replay", "/no/such/file.ulg"],
        vec!["run", "--bogus"],
    ] {
        let o = run(&args);
        assert_eq!(code(&o), 2, "{args:?}: {}", stderr(&o));
        assert!(
            stderr(&o).starts_with("error: "),
            "{args:?}: {}",
            stderr(&o)
        );
    }
    // A failing run is not an error: exit 1, and the report is printed.
    let o = run(&["montecarlo", "--seeds", "1", "--param", "ATT_ROLL_P=1"]);
    assert_eq!(code(&o), 1, "{}", stderr(&o));
    assert!(stdout(&o).contains("passed 0/1"), "{}", stdout(&o));
    assert_eq!(code(&run(&["montecarlo", "--seeds", "1"])), 0);
}

#[test]
fn a_csv_that_was_created_is_removed_when_the_log_cannot_be_opened() {
    let csv = temp("kept-out.csv");
    let _ = std::fs::remove_file(&csv);
    let log = text(&temp("no-such-dir").join("f.ulg"));
    for cmd in ["run", "scenario"] {
        let mut a = vec![cmd];
        if cmd == "scenario" {
            a.push("gcs-loss");
        }
        let csv_text = text(&csv);
        a.extend(["--csv", &csv_text, "--log", &log]);
        let o = run(&a);
        assert_eq!(code(&o), 2, "{a:?}: {}", stderr(&o));
        assert!(!csv.exists(), "{a:?} left the csv");
    }
}

#[cfg(target_os = "linux")]
#[test]
fn a_csv_is_removed_when_the_log_cannot_be_started() {
    let csv = temp("kept-out-2.csv");
    let _ = std::fs::remove_file(&csv);
    let o = run(&[
        "scenario",
        "gcs-loss",
        "--csv",
        &text(&csv),
        "--log",
        "/dev/full",
    ]);
    assert_eq!(code(&o), 2, "{}", stderr(&o));
    assert!(!csv.exists(), "the csv was left behind");
    assert!(std::path::Path::new("/dev/full").exists());
}

#[test]
fn a_flight_that_ran_keeps_its_files() {
    let (csv, log) = (temp("kept.csv"), temp("kept.ulg"));
    let o = run(&[
        "scenario",
        "gcs-loss",
        "--seed",
        "1",
        "--csv",
        &text(&csv),
        "--log",
        &text(&log),
    ]);
    assert_eq!(code(&o), 0, "{}", stderr(&o));
    assert!(csv.exists() && log.exists());
    assert!(stdout(&o).contains("result     PASS"), "{}", stdout(&o));
}
