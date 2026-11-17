// SPDX-License-Identifier: AGPL-3.0-or-later
//! The binary `oxide-bridge`: its exit codes, and `fly` and `batch` on our own
//! core (no simulator program needed). The runs that need the program are in
//! `arducopter_scenario.rs`.

use std::path::{Path, PathBuf};
use std::process::{Command, Output};

use oxide_bridge::record::{Row, header, read_row};

fn scenario(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../scenarios")
        .join(format!("{name}.toml"))
}

fn scratch(tag: &str) -> PathBuf {
    let d = std::env::temp_dir().join(format!("oxide-cli-test-{}-{tag}", std::process::id()));
    let _ = std::fs::remove_dir_all(&d);
    d
}

fn bridge(args: &[&str]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_oxide-bridge"))
        .args(args)
        .env_remove("OXIDE_ARDUCOPTER")
        .output()
        .expect("runs")
}

fn text(b: &[u8]) -> String {
    String::from_utf8_lossy(b).into_owned()
}

fn path(p: &Path) -> &str {
    p.to_str().expect("utf-8 path")
}

#[test]
fn a_wrong_use_ends_with_a_sentence_and_exit_code_1() {
    let s = scenario("hover-step");
    let s = path(&s);
    let cases: Vec<Vec<&str>> = vec![
        vec![],
        vec!["nonsense"],
        vec!["fly"],
        vec!["fly", "--scenario", s, "--nonsense"],
        vec!["fly", "--scenario", s, "--index", "x"],
        vec!["fly", "--scenario", "/no/such/file.toml"],
        // ArduCopter with no program named.
        vec!["fly", "--scenario", s],
        vec!["batch", "--scenario", s],
    ];
    for args in cases {
        let o = bridge(&args);
        assert_eq!(o.status.code(), Some(1), "{args:?}: {}", text(&o.stderr));
        let e = text(&o.stderr);
        assert!(e.starts_with("error: "), "{args:?}: {e}");
        assert_eq!(e.lines().count(), 1, "{args:?}: {e}");
    }
}

#[test]
fn a_scenario_the_bridge_cannot_translate_is_a_wrong_use_that_says_why() {
    let sticks = scenario("rc-loss-position");
    let o = bridge(&[
        "fly",
        "--scenario",
        path(&sticks),
        "--binary",
        "/no/such/arducopter",
    ]);
    assert_eq!(o.status.code(), Some(1), "{}", text(&o.stderr));
    assert!(
        text(&o.stderr).contains("stick flying"),
        "{}",
        text(&o.stderr)
    );
}

#[test]
fn fly_on_our_core_judges_writes_the_record_and_exits_0() {
    let out = scratch("fly-core");
    let o = bridge(&[
        "fly",
        "--scenario",
        path(&scenario("hover-step")),
        "--autopilot",
        "core-f405",
        "--out",
        path(&out),
    ]);
    assert_eq!(o.status.code(), Some(0), "{}", text(&o.stderr));
    let out_text = text(&o.stdout);
    assert!(out_text.contains("result     PASS"), "{out_text}");
    assert!(out_text.contains("class      safe"), "{out_text}");
    let row = read_row(&out.join("row.csv")).expect("row");
    assert_eq!(row.get("autopilot"), "core-f405");
    assert_eq!(row.get("started"), "yes");
    let audit = std::fs::read_to_string(out.join("audit.txt")).expect("audit");
    assert!(audit.contains("file sha256"), "{audit}");
    assert!(audit.contains("our commit"), "{audit}");
    let _ = std::fs::remove_dir_all(&out);
}

#[test]
fn fly_twice_on_our_core_says_that_it_repeats() {
    let out = scratch("twice-core");
    let o = bridge(&[
        "fly",
        "--scenario",
        path(&scenario("gust-hold")),
        "--autopilot",
        "core-h743",
        "--twice",
        "--out",
        path(&out),
    ]);
    assert_eq!(o.status.code(), Some(0), "{}", text(&o.stderr));
    assert_eq!(
        read_row(&out.join("row.csv")).expect("row").get("repeats"),
        "yes"
    );
    assert!(out.join("second").join("row.csv").exists());
    let _ = std::fs::remove_dir_all(&out);
}

#[test]
fn a_flight_that_does_not_start_exits_2_and_leaves_a_row_that_says_so() {
    let out = scratch("nostart");
    let o = bridge(&[
        "fly",
        "--scenario",
        path(&scenario("hover-step")),
        "--binary",
        "/no/such/arducopter",
        "--out",
        path(&out),
    ]);
    assert_eq!(o.status.code(), Some(2), "{}", text(&o.stderr));
    let row = read_row(&out.join("row.csv")).expect("row");
    assert_eq!(row.get("started"), "no");
    assert!(row.get("note").contains("did not start"), "{}", row.line());
    assert!(out.join("audit.txt").exists());
    let _ = std::fs::remove_dir_all(&out);
}

#[test]
fn a_batch_on_our_core_has_the_rows_of_single_flights_in_index_order() {
    let out = scratch("batch-core");
    let o = bridge(&[
        "batch",
        "--scenario",
        path(&scenario("gust-hold")),
        "--autopilot",
        "core-f405",
        "--indices",
        "0..4",
        "--jobs",
        "2",
        "--instances",
        "0-1",
        "--out",
        path(&out.join("batch")),
    ]);
    assert_eq!(
        o.status.code(),
        Some(0),
        "{}{}",
        text(&o.stdout),
        text(&o.stderr)
    );
    let table = std::fs::read_to_string(out.join("batch").join("flights.csv")).expect("table");
    let lines: Vec<&str> = table.lines().collect();
    assert_eq!(lines[0], header());
    assert_eq!(lines.len(), 5, "{table}");
    for (i, line) in lines[1..].iter().enumerate() {
        let row = Row::parse(line).expect("row");
        assert_eq!(row.get("index"), i.to_string());
        let single = out.join(format!("single-{i}"));
        let o = bridge(&[
            "fly",
            "--scenario",
            path(&scenario("gust-hold")),
            "--autopilot",
            "core-f405",
            "--index",
            &i.to_string(),
            "--out",
            path(&single),
        ]);
        assert_eq!(o.status.code(), Some(0));
        let one = read_row(&single.join("row.csv")).expect("single row");
        for c in oxide_bridge::record::COLUMNS {
            if c != "wall_s" {
                assert_eq!(row.get(c), one.get(c), "run {i}, column {c}");
            }
        }
    }
    let _ = std::fs::remove_dir_all(&out);
}

#[test]
fn a_batch_whose_run_fails_twice_writes_what_it_has_and_exits_3() {
    let out = scratch("batch-fail");
    let o = bridge(&[
        "batch",
        "--scenario",
        path(&scenario("hover-step")),
        "--binary",
        "/no/such/arducopter",
        "--indices",
        "0..3",
        "--out",
        path(&out),
        "--no-pin",
    ]);
    assert_eq!(
        o.status.code(),
        Some(3),
        "{}{}",
        text(&o.stdout),
        text(&o.stderr)
    );
    let table = std::fs::read_to_string(out.join("flights.csv")).expect("table");
    let lines: Vec<&str> = table.lines().collect();
    assert_eq!(lines.len(), 2, "{table}");
    let row = Row::parse(lines[1]).expect("row");
    assert_eq!(row.get("started"), "no");
    assert_eq!(row.get("retried"), "yes");
    assert!(
        text(&o.stdout).contains("attempt 2: failed"),
        "{}",
        text(&o.stdout)
    );
    let _ = std::fs::remove_dir_all(&out);
}

#[test]
fn the_core_routes_of_the_binary_fly_the_profile_they_name() {
    for (name, hash) in [
        ("core-f405", "9712d37590a317dc"),
        ("core-h743", "476abddf98142653"),
    ] {
        let out = scratch(name);
        let o = bridge(&[
            "fly",
            "--scenario",
            path(&scenario("gust-hold")),
            "--autopilot",
            name,
            "--out",
            path(&out),
        ]);
        assert_eq!(o.status.code(), Some(0), "{}", text(&o.stderr));
        let row = read_row(&out.join("row.csv")).expect("row");
        assert_eq!(row.get("autopilot"), name);
        assert_eq!(row.get("trace_hash"), hash, "{name}");
        let _ = std::fs::remove_dir_all(&out);
    }
}
