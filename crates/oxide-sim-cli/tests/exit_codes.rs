// SPDX-License-Identifier: AGPL-3.0-or-later
//! The exit status of `oxide-sim replay` (ADR-0005 §5), from the real binary:
//! 0 when every tick replayed is identical (also when the file ends in a
//! cut), 1 on a difference or a gap, 2 on a file it cannot replay.

use std::path::PathBuf;
use std::process::{Command, Output};

/// A path in a directory of this test run's own.
fn temp(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("oxide-sim-exit-codes-{}", std::process::id()));
    std::fs::create_dir_all(&dir).expect("temp dir");
    dir.join(name)
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

fn stdout(o: &Output) -> String {
    String::from_utf8_lossy(&o.stdout).into_owned()
}

fn stderr(o: &Output) -> String {
    String::from_utf8_lossy(&o.stderr).into_owned()
}

fn text(p: &std::path::Path) -> String {
    p.to_string_lossy().into_owned()
}

/// A log of `gcs-loss` (seed 1) at `level`, written by the binary.
fn log_at(name: &str, level: &str) -> (PathBuf, Vec<u8>) {
    let path = temp(name);
    let o = run(&[
        "scenario",
        "gcs-loss",
        "--seed",
        "1",
        "--log",
        &text(&path),
        "--log-level",
        level,
    ]);
    assert_eq!(code(&o), 0, "{}", stderr(&o));
    let bytes = std::fs::read(&path).expect("the log");
    (path, bytes)
}

fn write(name: &str, bytes: &[u8]) -> String {
    let p = temp(name);
    std::fs::write(&p, bytes).expect("writes");
    text(&p)
}

#[test]
fn replay_exits_0_1_and_2_as_documented() {
    let (path, bytes) = log_at("replay.ulg", "replay");
    let path = text(&path);

    // 0: identical.
    let o = run(&["replay", &path]);
    assert_eq!(code(&o), 0);
    let out = stdout(&o);
    assert!(
        out.starts_with("replayed ") && out.contains(" ticks (f405-lean): identical"),
        "{out}"
    );

    // 0: a file cut short replays up to the cut and says so.
    let cut = write("cut.ulg", &bytes[..bytes.len() - 7]);
    let o = run(&["replay", &cut]);
    assert_eq!(code(&o), 0, "{}", stdout(&o));
    assert!(
        stdout(&o).contains("the file ends in a cut after tick "),
        "{}",
        stdout(&o)
    );

    // 1: a difference (a changed gain), and with --continue too.
    let o = run(&["replay", &path, "--param", "RATE_ROLL_P=9"]);
    assert_eq!(code(&o), 1);
    assert!(
        stdout(&o).contains("first difference at tick "),
        "{}",
        stdout(&o)
    );
    assert!(stdout(&o).contains("stopped at the first difference"));
    let o = run(&["replay", &path, "--param", "RATE_ROLL_P=9", "--continue"]);
    assert_eq!(code(&o), 1);
    assert!(
        stdout(&o).contains(" differ, the first at tick "),
        "{}",
        stdout(&o)
    );

    // 1: a gap in the middle (a tick and its records removed).
    let file = oxide_log::reader::UlogFile::parse(&bytes).expect("parses");
    let a = file
        .records("oxide_in_tick")
        .nth(300)
        .expect("tick")
        .offset();
    let b = file
        .records("oxide_in_tick")
        .nth(301)
        .expect("tick")
        .offset();
    let mut gapped = bytes[..a].to_vec();
    gapped.extend_from_slice(&bytes[b..]);
    let gapped = write("gapped.ulg", &gapped);
    let o = run(&["replay", &gapped]);
    assert_eq!(code(&o), 1);
    assert!(
        stdout(&o).contains("a tick record is missing after tick 299"),
        "{}",
        stdout(&o)
    );

    // The first line does not say identical for a replay that stopped early.
    assert!(
        stdout(&o).starts_with(
            "replayed 300 ticks (f405-lean): no difference before the replay stopped\n"
        ),
        "{}",
        stdout(&o)
    );

    // 1: a message at the end that declares more than any message of the file
    // is damage, not a cut (a flipped bit in a size looks like this).
    let mut damaged = bytes.clone();
    damaged.extend_from_slice(&[0xF0, 0xF0, b'L', b'6', 0, 0]);
    let damaged = write("damaged.ulg", &damaged);
    let o = run(&["replay", &damaged]);
    assert_eq!(code(&o), 1, "{}", stdout(&o));
    assert!(stdout(&o).contains("damage, not a cut"), "{}", stdout(&o));

    // 2: a file it cannot replay, and arguments it cannot use.
    let junk = write("junk.ulg", b"not a log at all");
    let o = run(&["replay", &junk]);
    assert_eq!(code(&o), 2);
    assert!(stderr(&o).starts_with("error: "), "{}", stderr(&o));
    assert_eq!(code(&run(&["replay", &text(&temp("no-such-file.ulg"))])), 2);
    let (normal, _) = log_at("normal.ulg", "normal");
    let o = run(&["replay", &text(&normal)]);
    assert_eq!(code(&o), 2);
    assert!(
        stderr(&o).contains("not a replay-level log"),
        "{}",
        stderr(&o)
    );
    assert_eq!(code(&run(&["replay"])), 2);
    assert_eq!(code(&run(&["replay", &path, "--param", "NO_SUCH=1"])), 2);
    assert_eq!(code(&run(&["replay", &path, "--frobnicate"])), 2);

    let _ = std::fs::remove_dir_all(temp("x").parent().expect("dir"));
}
