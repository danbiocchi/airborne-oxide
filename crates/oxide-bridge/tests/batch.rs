// SPDX-License-Identifier: AGPL-3.0-or-later
//! The batch's logic against a stand-in child: this test program itself,
//! re-run on the test `standin_child`, which behaves as a plan in its
//! environment says. No simulator program is needed, and nothing here is
//! specific to a platform except the pinning test.
//!
//! The plan is `index:behaviour;...`, behaviours: `sleep=MS` (finish after),
//! `fail-first` (attempt 1 exits 2 with a row that says why), `fail-always`,
//! `die-first` (attempt 1 exits 7 and leaves no row), `hang` (never ends).

use std::path::{Path, PathBuf};
use std::process::Command;
use std::sync::Mutex;
use std::time::{Duration, Instant};

use oxide_bridge::batch::{BatchConfig, BatchResult, Event, Job, Pin, run_batch, run_dir};
use oxide_bridge::record::{Row, header, read_row, write_files};
use oxide_bridge::runner::Autopilot;

const OUT: &str = "OXIDE_STANDIN_OUT";

fn env(name: &str) -> String {
    std::env::var(name).unwrap_or_default()
}

fn now_ms() -> u128 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| d.as_millis())
}

/// The stand-in child. With no plan in the environment it is an ordinary
/// test that passes.
#[test]
fn standin_child() {
    let out = env(OUT);
    if out.is_empty() {
        return;
    }
    let index: u64 = env("OXIDE_STANDIN_INDEX").parse().expect("index");
    let attempt: u32 = env("OXIDE_STANDIN_ATTEMPT").parse().expect("attempt");
    let instance = env("OXIDE_STANDIN_INSTANCE");
    let plan = env("OXIDE_STANDIN_PLAN");
    let mine: Vec<&str> = plan
        .split(';')
        .filter_map(|e| e.split_once(':'))
        .filter(|(i, _)| i.parse::<u64>() == Ok(index))
        .map(|(_, b)| b)
        .collect();
    let started = now_ms();
    let cpus = std::fs::read_to_string("/proc/self/status")
        .ok()
        .and_then(|t| {
            t.lines().find_map(|l| {
                l.strip_prefix("Cpus_allowed_list:")
                    .map(|v| v.trim().to_string())
            })
        })
        .unwrap_or_default();
    let mut row = Row::empty();
    row.set("scenario", "stand-in");
    row.set("index", &index.to_string());
    row.set("seed", &(index * 10).to_string());
    row.set("autopilot", "arducopter");
    let write = |row: &Row| write_files(Path::new(&out), "stand-in\n", row).expect("row");
    for b in mine {
        if let Some(ms) = b.strip_prefix("sleep=") {
            std::thread::sleep(Duration::from_millis(ms.parse().expect("ms")));
        } else if b == "hang" {
            std::thread::sleep(Duration::from_secs(120));
        } else if b == "die-first" && attempt == 1 {
            std::process::exit(7);
        } else if b == "fail-always" || (b == "fail-first" && attempt == 1) {
            row.set("started", "no");
            row.set(
                "note",
                &format!("the stand-in could not start (attempt {attempt})"),
            );
            write(&row);
            std::process::exit(2);
        }
    }
    row.set("started", "yes");
    row.set("class", "safe");
    row.set("verdict", "pass");
    row.set("retried", "no");
    row.set(
        "note",
        &format!("instance={instance} attempt={attempt} cpus={cpus}"),
    );
    write(&row);
    // One line for the test: instance, start, end (ms since the epoch).
    let log = env("OXIDE_STANDIN_LOG");
    if !log.is_empty() {
        use std::io::Write;
        let mut f = std::fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open(log)
            .expect("log");
        f.write_all(format!("{instance} {started} {}\n", now_ms()).as_bytes())
            .expect("log line");
    }
}

fn scratch(tag: &str) -> PathBuf {
    let d = std::env::temp_dir().join(format!("oxide-batch-test-{}-{tag}", std::process::id()));
    let _ = std::fs::remove_dir_all(&d);
    std::fs::create_dir_all(&d).expect("scratch");
    d
}

fn launcher(plan: &'static str, log: PathBuf) -> impl Fn(&Job) -> Command + Sync {
    move |job: &Job| {
        let mut c = Command::new(std::env::current_exe().expect("exe"));
        c.args([
            "--exact",
            "standin_child",
            "--nocapture",
            "--test-threads=1",
        ])
        .env(OUT, &job.out)
        .env("OXIDE_STANDIN_INDEX", job.index.to_string())
        .env("OXIDE_STANDIN_ATTEMPT", job.attempt.to_string())
        .env("OXIDE_STANDIN_INSTANCE", job.instance.to_string())
        .env("OXIDE_STANDIN_PLAN", plan)
        .env("OXIDE_STANDIN_LOG", &log);
        c
    }
}

fn config(out: &Path, runs: std::ops::Range<u64>, jobs: usize) -> BatchConfig {
    BatchConfig {
        scenario: "stand-in".into(),
        autopilot: Autopilot::ArduCopter,
        runs: runs.map(|i| (i, i * 10)).collect(),
        jobs,
        instances: (3, 9),
        limit: Duration::from_secs(60),
        out: out.to_path_buf(),
        pin: Pin::Off,
    }
}

struct Run {
    result: BatchResult,
    events: Vec<Event>,
    dir: PathBuf,
}

fn run(tag: &str, plan: &'static str, cfg: impl FnOnce(&Path) -> BatchConfig) -> Run {
    let dir = scratch(tag);
    let cfg = cfg(&dir.join("batch"));
    let log = dir.join("child.log");
    let events = Mutex::new(Vec::new());
    let result = run_batch(&cfg, &launcher(plan, log), &|e| {
        events.lock().expect("events").push(e.clone());
    })
    .unwrap_or_else(|e| panic!("{e}"));
    Run {
        result,
        events: events.into_inner().expect("events"),
        dir,
    }
}

fn indices(rows: &[Row]) -> Vec<u64> {
    rows.iter()
        .map(|r| r.get("index").parse().expect("index"))
        .collect()
}

#[test]
fn rows_come_out_in_index_order_when_children_finish_out_of_order() {
    let r = run(
        "order",
        "0:sleep=900;1:sleep=600;2:sleep=300;3:sleep=10",
        |out| config(out, 0..4, 4),
    );
    let finished: Vec<u64> = r
        .events
        .iter()
        .filter_map(|e| match e {
            Event::Done { index, .. } => Some(*index),
            _ => None,
        })
        .collect();
    assert_ne!(
        finished,
        vec![0, 1, 2, 3],
        "the children finished in order: the test proves nothing"
    );
    assert_eq!(finished.first(), Some(&3), "{finished:?}");
    assert_eq!(indices(&r.result.rows), vec![0, 1, 2, 3]);
    let table = std::fs::read_to_string(r.dir.join("batch").join("flights.csv")).expect("table");
    let lines: Vec<&str> = table.lines().collect();
    assert_eq!(lines[0], header());
    let from_table: Vec<u64> = lines[1..]
        .iter()
        .map(|l| {
            Row::parse(l)
                .expect("row")
                .get("index")
                .parse()
                .expect("index")
        })
        .collect();
    assert_eq!(from_table, vec![0, 1, 2, 3], "{table}");
    assert_eq!(r.result.stopped_at, None);
    let _ = std::fs::remove_dir_all(&r.dir);
}

#[test]
fn a_run_that_did_not_start_is_flown_once_more_and_the_second_row_is_the_complete_one() {
    let r = run("retry", "1:fail-first;2:die-first", |out| {
        config(out, 0..4, 2)
    });
    assert_eq!(indices(&r.result.rows), vec![0, 1, 2, 3]);
    for i in [1usize, 2] {
        let row = &r.result.rows[i];
        assert_eq!(row.get("started"), "yes", "{}", row.line());
        assert_eq!(row.get("retried"), "yes");
        assert!(
            row.get("note").contains("first attempt failed"),
            "{}",
            row.line()
        );
        assert!(row.get("note").contains("attempt=2"), "{}", row.line());
    }
    assert!(r.result.rows[1].get("note").contains("could not start"));
    assert!(r.result.rows[2].get("note").contains("exited with code 7"));
    assert_eq!(r.result.rows[0].get("retried"), "no");
    // The first attempt's directory is kept beside the second's.
    assert!(
        run_dir(&r.dir.join("batch"), 1)
            .with_extension("failed")
            .exists()
            || {
                let mut p = run_dir(&r.dir.join("batch"), 1).into_os_string();
                p.push(".failed");
                PathBuf::from(p).exists()
            }
    );
    let attempts: Vec<(u64, u32)> = r
        .events
        .iter()
        .filter_map(|e| match e {
            Event::Spawned { index, attempt, .. } => Some((*index, *attempt)),
            _ => None,
        })
        .collect();
    assert_eq!(
        attempts.iter().filter(|(i, _)| *i == 1).count(),
        2,
        "{attempts:?}"
    );
    assert_eq!(
        attempts.iter().filter(|(i, _)| *i == 0).count(),
        1,
        "{attempts:?}"
    );
    assert_eq!(r.result.stopped_at, None);
    let _ = std::fs::remove_dir_all(&r.dir);
}

#[test]
fn a_second_failure_stops_the_batch_after_writing_what_it_has() {
    let r = run("stop", "1:fail-always", |out| config(out, 0..6, 1));
    // Run 0 flew; run 1 failed twice; nothing after it was flown.
    assert_eq!(indices(&r.result.rows), vec![0, 1]);
    assert_eq!(r.result.stopped_at, Some(1));
    let last = &r.result.rows[1];
    assert_eq!(last.get("started"), "no");
    assert_eq!(last.get("retried"), "yes");
    assert!(
        last.get("note").contains("could not start"),
        "{}",
        last.line()
    );
    assert!(
        last.get("note").contains("first attempt"),
        "{}",
        last.line()
    );
    let spawned = r
        .events
        .iter()
        .filter(|e| matches!(e, Event::Spawned { .. }))
        .count();
    assert_eq!(spawned, 3, "run 0 once, run 1 twice, no run after");
    let table = std::fs::read_to_string(r.dir.join("batch").join("flights.csv")).expect("table");
    assert_eq!(table.lines().count(), 3, "{table}");
    let _ = std::fs::remove_dir_all(&r.dir);
}

#[test]
fn a_child_past_the_wall_limit_is_ended_and_its_row_says_so() {
    let began = Instant::now();
    let r = run("limit", "0:hang", |out| {
        let mut c = config(out, 0..1, 1);
        c.limit = Duration::from_millis(700);
        c
    });
    assert!(
        began.elapsed() < Duration::from_secs(30),
        "{:?}",
        began.elapsed()
    );
    let row = &r.result.rows[0];
    assert_eq!(row.get("started"), "no");
    assert!(row.get("note").contains("wall limit"), "{}", row.line());
    assert_eq!(r.result.stopped_at, Some(0));
    let _ = std::fs::remove_dir_all(&r.dir);
}

/// Intervals (ms) of every child, with its instance, from the children's log.
fn intervals(dir: &Path) -> Vec<(u8, u128, u128)> {
    std::fs::read_to_string(dir.join("child.log"))
        .expect("log")
        .lines()
        .map(|l| {
            let mut p = l.split(' ');
            (
                p.next().expect("i").parse().expect("instance"),
                p.next().expect("s").parse().expect("start"),
                p.next().expect("e").parse().expect("end"),
            )
        })
        .collect()
}

#[test]
fn no_two_children_running_together_have_the_same_instance_number() {
    let plan = "0:sleep=300;1:sleep=300;2:sleep=300;3:sleep=300;4:sleep=300;5:sleep=300;6:sleep=300;7:sleep=300";
    let r = run("instances", plan, |out| config(out, 0..8, 3));
    let spans = intervals(&r.dir);
    assert_eq!(spans.len(), 8);
    let mut overlapping = 0;
    for (a, x) in spans.iter().enumerate() {
        assert!((3..=9).contains(&x.0), "instance {} is outside 3-9", x.0);
        for y in &spans[a + 1..] {
            if x.1 < y.2 && y.1 < x.2 {
                overlapping += 1;
                assert_ne!(x.0, y.0, "two children at once on instance {}", x.0);
            }
        }
    }
    assert!(
        overlapping >= 3,
        "the children did not overlap ({overlapping}): nothing was tested"
    );
    let used: std::collections::BTreeSet<u8> = spans.iter().map(|s| s.0).collect();
    assert_eq!(used.len(), 3, "{used:?}");
    let _ = std::fs::remove_dir_all(&r.dir);
}

#[test]
fn more_jobs_than_instance_numbers_and_a_directory_in_use_are_refused() {
    let dir = scratch("refuse");
    let mut cfg = config(&dir.join("b"), 0..2, 4);
    cfg.instances = (0, 2);
    let e = run_batch(&cfg, &launcher("", dir.join("l")), &|_| {}).unwrap_err();
    assert!(e.to_string().contains("instance numbers"), "{e}");
    let cfg = config(&dir.join("b"), 0..2, 1);
    std::fs::create_dir_all(run_dir(&dir.join("b"), 1)).expect("dir");
    let e = run_batch(&cfg, &launcher("", dir.join("l")), &|_| {}).unwrap_err();
    assert!(e.to_string().contains("exists already"), "{e}");
    let _ = std::fs::remove_dir_all(&dir);
}

#[cfg(unix)]
#[test]
fn each_child_is_pinned_to_a_core_of_its_own_where_taskset_exists() {
    if !oxide_bridge::batch::taskset_available() {
        eprintln!("taskset is not installed: the pinning is not tested here");
        return;
    }
    let ncpu = std::thread::available_parallelism().map_or(1, usize::from);
    if ncpu < 2 {
        return;
    }
    let r = run("pin", "0:sleep=300;1:sleep=300", |out| {
        let mut c = config(out, 0..2, 2);
        c.pin = Pin::Taskset { first_core: 0 };
        c
    });
    let mut cores: Vec<usize> = r
        .events
        .iter()
        .filter_map(|e| match e {
            Event::Spawned { core, .. } => *core,
            _ => None,
        })
        .collect();
    cores.sort_unstable();
    assert_eq!(cores, vec![0, 1]);
    for row in &r.result.rows {
        let index: usize = row.get("index").parse().expect("index");
        assert!(
            row.get("note").ends_with(&format!("cpus={index}")),
            "child {index} was not on core {index}: {}",
            row.get("note")
        );
    }
    let _ = std::fs::remove_dir_all(&r.dir);
}

#[test]
fn a_row_file_is_what_the_batch_reads() {
    let dir = scratch("rowfile");
    let mut row = Row::empty();
    row.set("scenario", "x");
    write_files(&dir, "a\n", &row).expect("write");
    assert_eq!(read_row(&dir.join("row.csv")).expect("read"), row);
    let _ = std::fs::remove_dir_all(&dir);
}
