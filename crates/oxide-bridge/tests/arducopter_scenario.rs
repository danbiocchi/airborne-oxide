// SPDX-License-Identifier: AGPL-3.0-or-later
//! Scenario files flown through the bridge on the real ArduCopter simulator
//! program, and the binary `oxide-bridge` that does it (ADR-0006, ADR-0007).
//!
//! Every test here needs the program and is ignored by default: set
//! `OXIDE_ARDUCOPTER` to its path and run with `--ignored` and `--release`.
//! Instance numbers come from the pool of `tests/support/pool.rs`
//! (`OXIDE_ARDUCOPTER_INSTANCES`). A test that runs a batch holds the whole
//! range for its time.

use std::io::{BufRead, BufReader};
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::sync::{Mutex, MutexGuard};

use oxide_bridge::arducopter::{
    ArduCopter, ArduCopterConfig, BrainLink, PREROLL_STEPS, bridged_options, fly,
};
use oxide_bridge::mav::MavEvent;
use oxide_bridge::record::{COLUMNS, Row, header, read_row};
use oxide_bridge::runner::{Setup, arducopter_params, fly_scenario};
use oxide_bridge::scenario::{LoadedScenario, translate};
use oxide_bridge::sitl::PINNED;
use oxide_core::time::Micros;
use oxide_sim::outcome::FlightClass;
use oxide_sim::scenario::gust_hold;
use oxide_sim::{Airframe, Simulation};

#[path = "support/pool.rs"]
mod pool;
use pool::Instance;

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

fn scenario(name: &str) -> LoadedScenario {
    LoadedScenario::load(scenario_path(name)).expect("the scenario file")
}

fn scenario_path(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../scenarios")
        .join(format!("{name}.toml"))
}

struct TempDir(PathBuf);

impl TempDir {
    fn new(tag: &str) -> Self {
        let p = std::env::temp_dir().join(format!(
            "oxide-arducopter-scenario-{}-{tag}",
            std::process::id()
        ));
        let _ = std::fs::remove_dir_all(&p);
        std::fs::create_dir_all(&p).expect("temporary directory");
        Self(p)
    }
    fn path(&self) -> &Path {
        &self.0
    }
}

impl Drop for TempDir {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

/// Holds every instance of the range for a test that starts children of its
/// own: one test at a time takes them (so that two such tests cannot each
/// hold some and wait for the rest), and each lease waits for a free number.
static ALL: Mutex<()> = Mutex::new(());

struct AllInstances {
    _guard: MutexGuard<'static, ()>,
    _leases: Vec<Instance>,
    range: (u8, u8),
}

fn lease_all() -> AllInstances {
    let guard = ALL.lock().unwrap_or_else(|e| e.into_inner());
    let range = pool::range();
    let leases = (range.0..=range.1).map(|_| Instance::lease()).collect();
    AllInstances {
        _guard: guard,
        _leases: leases,
        range,
    }
}

fn bridge_command() -> Command {
    let mut c = Command::new(env!("CARGO_BIN_EXE_oxide-bridge"));
    c.env("OXIDE_ARDUCOPTER", binary());
    c
}

fn text(b: &[u8]) -> String {
    String::from_utf8_lossy(b).into_owned()
}

/// Test 1. The file route is the code route.
#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn gust_hold_from_the_file_flies_the_same_flight_as_the_one_built_in_code() {
    let lease = Instance::lease();
    let root = TempDir::new("file-vs-code");
    let src = scenario("gust-hold");
    let mut setup = Setup::new(binary(), root.path().join("file"), lease.0);
    setup.speedup = 100;
    let from_file = fly_scenario(&src, 0, &setup).unwrap_or_else(|e| panic!("{e}"));
    let (pilot, options) = gust_hold(0);
    let secs = pilot.duration_s;
    let config = ArduCopterConfig::new(binary(), root.path().join("code"), lease.0);
    let from_code = fly(
        &Airframe::reference_quad().expect("airframe"),
        pilot,
        options,
        config,
        Micros((secs * 1e6) as u64),
        |_| {},
    )
    .unwrap_or_else(|e| panic!("{e}"));
    let a = from_file.audit.as_ref().expect("audit");
    println!(
        "MEASURED file route: trace {:016x} packet {:016x} reply {:016x}; code route: trace {:016x} \
         packet {:016x} reply {:016x}",
        from_file.trace_hash,
        a.packet_hash,
        a.reply_hash,
        from_code.trace_hash,
        from_code.audit.packet_hash,
        from_code.audit.reply_hash
    );
    assert_eq!(from_file.trace_hash, from_code.trace_hash);
    assert_eq!(a.packet_hash, from_code.audit.packet_hash);
    assert_eq!(a.reply_hash, from_code.audit.reply_hash);
    assert_eq!(a.params_sha256, from_code.audit.params_sha256);
}

/// Test 2. Hover-step: flown, safe; what the verdict could not judge.
#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn hover_step_is_flown_and_safe_and_the_verdict_lists_what_it_could_not_judge() {
    let lease = Instance::lease();
    let root = TempDir::new("hover-step");
    let src = scenario("hover-step");
    let setup = Setup::new(binary(), root.path().join("flight"), lease.0);
    let f = fly_scenario(&src, 0, &setup).unwrap_or_else(|e| panic!("{e}"));
    println!(
        "MEASURED hover-step 0: class {}, {} violations, {} not judged, wall {:.1} s, last motor on {:?}",
        f.class,
        f.verdict.violations.len(),
        f.verdict.not_judged.len(),
        f.wall_s,
        f.last_motor_on_s
    );
    for v in &f.verdict.violations {
        println!("MEASURED hover-step violated: {v}");
    }
    for n in &f.verdict.not_judged {
        println!("MEASURED hover-step not judged: {n}");
    }
    assert_eq!(f.class, FlightClass::Safe);
    assert!(f.verdict.outcome.physics_faults == 0 && !f.verdict.outcome.non_finite);
}

/// Test 3. Geofence: the fence is in the program and acts.
#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn the_fence_parameters_are_in_the_program_and_the_flight_shows_the_fence_acting() {
    let lease = Instance::lease();
    let root = TempDir::new("geofence");
    let src = scenario("geofence");
    let translation = translate(&src.file).expect("translatable");
    // 1. The program has the fence parameters (a parameter read).
    let setup = Setup::new(binary(), root.path().join("probe"), lease.0);
    let (files, extra) = arducopter_params(&translation, &setup);
    let mut config = ArduCopterConfig::new(binary(), root.path().join("probe"), lease.0);
    config.flight_name = Some("sitl".into());
    config.param_files = files;
    config.extra_params = extra;
    config.brain = BrainLink::Serial1;
    let airframe = Airframe::reference_quad().expect("airframe");
    let mut options = src.file.options(&airframe, 0);
    options.params.clear();
    let options = bridged_options(options, &mut config);
    let ap = ArduCopter::start(config).unwrap_or_else(|e| panic!("{e}"));
    let mut sim = Simulation::with_autopilot(&airframe, ap, src.file.pilot.clone(), options)
        .expect("the simulation");
    sim.preroll(PREROLL_STEPS).unwrap_or_else(|e| panic!("{e}"));
    let names = ["FENCE_ENABLE", "FENCE_TYPE", "FENCE_RADIUS", "FENCE_ACTION"];
    for name in names {
        sim.autopilot()
            .instrument()
            .param_request(name)
            .expect("param_request");
    }
    for _ in 0..400 {
        sim.try_step().unwrap_or_else(|e| panic!("{e}"));
    }
    let audit = sim
        .into_autopilot()
        .finish()
        .unwrap_or_else(|e| panic!("{e}"));
    let read = |name: &str| -> f32 {
        audit
            .heard
            .iter()
            .find_map(|h| match &h.event {
                MavEvent::ParamValue { name: n, value, .. } if n == name => Some(*value),
                _ => None,
            })
            .unwrap_or_else(|| panic!("no value for {name} was read"))
    };
    println!(
        "MEASURED the program has FENCE_ENABLE {} FENCE_TYPE {} FENCE_RADIUS {} FENCE_ACTION {}",
        read("FENCE_ENABLE"),
        read("FENCE_TYPE"),
        read("FENCE_RADIUS"),
        read("FENCE_ACTION")
    );
    assert_eq!(read("FENCE_ENABLE"), 1.0);
    assert_eq!(read("FENCE_TYPE"), 2.0);
    assert_eq!(read("FENCE_RADIUS"), 10.0);
    assert_eq!(read("FENCE_ACTION"), 1.0);
    // 2. The flight: the setpoint steps 20 m out past the 10 m fence.
    let setup = Setup::new(binary(), root.path().join("flight"), lease.0);
    let f = fly_scenario(&src, 0, &setup).unwrap_or_else(|e| panic!("{e}"));
    let a = f.audit.as_ref().expect("audit");
    let o = &f.verdict.outcome;
    println!(
        "MEASURED geofence 0: class {}, max distance from home {:.2} m, max height {:.2} m, \
         touchdown {:?} m/s, disarmed at {:?} s, end {:?}; {} violations {:?}; {} not judged",
        f.class,
        o.max_distance_from_home_m,
        o.max_height_m,
        o.touchdown_speed_mps,
        o.disarmed_at_s,
        o.position_at_end,
        f.verdict.violations.len(),
        f.verdict.violations,
        f.verdict.not_judged.len()
    );
    for (t, sev, x) in &a.status_texts {
        println!(
            "MEASURED geofence text [{:.3}] ({sev}) {x}",
            (*t as f64 - (a.preroll_steps * a.step_us) as f64) * 1e-6
        );
    }
    for (t, m) in &a.mode_changes {
        println!(
            "MEASURED geofence mode {m} at {:.3}",
            (*t as f64 - (a.preroll_steps * a.step_us) as f64) * 1e-6
        );
    }
    // The same flight without the fence parameters: the control.
    let mut config = ArduCopterConfig::new(binary(), root.path().join("control"), lease.0);
    config.flight_name = Some("sitl".into());
    let mut control = oxide_sim::outcome::FlightOutcome::new();
    let mut options = src.file.options(&airframe, 0);
    options.params.clear();
    let flown = fly(
        &airframe,
        src.file.pilot.clone(),
        options,
        config,
        src.file.end(),
        |r| control.add(r),
    )
    .unwrap_or_else(|e| panic!("{e}"));
    println!(
        "MEASURED geofence 0 without the fence: max distance from home {:.2} m, max height {:.2} m,          class {}, modes {:?}, texts naming the fence {}",
        control.max_distance_from_home_m,
        control.max_height_m,
        control.class(),
        flown.audit.mode_changes,
        flown.audit.texts_containing("ence").count()
    );
    for (t, sev, x) in flown
        .audit
        .status_texts
        .iter()
        .filter(|(t, _, _)| *t > 45_000_000)
    {
        println!(
            "MEASURED geofence control text [{:.3}] ({sev}) {x}",
            (*t as f64 - (a.preroll_steps * a.step_us) as f64) * 1e-6
        );
    }
    // The fence acts: with it the vehicle does not go to the setpoint 20 m
    // out; without it, it does.
    assert!(
        control.max_distance_from_home_m > 15.0,
        "without the fence the vehicle should reach the setpoint: {}",
        control.max_distance_from_home_m
    );
    assert!(
        o.max_distance_from_home_m < 2.0,
        "with the fence the vehicle went {} m out",
        o.max_distance_from_home_m
    );
}

/// Flies one run with the binary's own `fly` and returns its row.
fn single_row(root: &Path, tag: &str, index: u64, instance: u8, extra: &[&str]) -> Row {
    let out = root.join(tag);
    let o = bridge_command()
        .args(["fly", "--scenario"])
        .arg(scenario_path("gust-hold"))
        .args([
            "--index",
            &index.to_string(),
            "--instance",
            &instance.to_string(),
        ])
        .arg("--out")
        .arg(&out)
        .args(extra)
        .output()
        .expect("runs");
    assert_eq!(
        o.status.code(),
        Some(0),
        "{}{}",
        text(&o.stdout),
        text(&o.stderr)
    );
    read_row(&out.join("row.csv")).expect("row")
}

/// Test 4. A batch of four with two jobs: four rows, in order, each equal in
/// every column but `wall_s` to the row of a single `fly`.
#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn a_batch_of_four_has_the_rows_of_four_single_flights_in_order() {
    let all = lease_all();
    let root = TempDir::new("batch4");
    let out = root.path().join("batch");
    let o = bridge_command()
        .args(["batch", "--scenario"])
        .arg(scenario_path("gust-hold"))
        .args(["--indices", "0..4", "--jobs", "2", "--instances"])
        .arg(format!("{}-{}", all.range.0, all.range.1))
        .arg("--out")
        .arg(&out)
        .output()
        .expect("runs");
    println!("{}", text(&o.stdout));
    assert_eq!(o.status.code(), Some(0), "{}", text(&o.stderr));
    let table = std::fs::read_to_string(out.join("flights.csv")).expect("table");
    let lines: Vec<&str> = table.lines().collect();
    assert_eq!(lines[0], header());
    assert_eq!(lines.len(), 5, "{table}");
    for (i, line) in lines[1..].iter().enumerate() {
        let row = Row::parse(line).expect("row");
        assert_eq!(row.get("index"), i.to_string(), "{table}");
        let single = single_row(
            root.path(),
            &format!("single-{i}"),
            i as u64,
            all.range.0,
            &[],
        );
        println!(
            "MEASURED run {i}: batch {} | single {}",
            row.line(),
            single.line()
        );
        for c in COLUMNS {
            if c != "wall_s" {
                assert_eq!(row.get(c), single.get(c), "run {i}, column {c}");
            }
        }
        assert_eq!(row.get("started"), "yes");
        assert_eq!(row.get("retried"), "no");
    }
}

/// Test 5. `--twice` on indices 0..2: the hashes repeat.
#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn twice_on_two_indices_says_that_each_repeats() {
    let all = lease_all();
    let root = TempDir::new("twice");
    let out = root.path().join("batch");
    let o = bridge_command()
        .args(["batch", "--scenario"])
        .arg(scenario_path("gust-hold"))
        .args(["--indices", "0..2", "--jobs", "2", "--twice", "--instances"])
        .arg(format!("{}-{}", all.range.0, all.range.1))
        .arg("--out")
        .arg(&out)
        .output()
        .expect("runs");
    println!("{}", text(&o.stdout));
    assert_eq!(o.status.code(), Some(0), "{}", text(&o.stderr));
    let table = std::fs::read_to_string(out.join("flights.csv")).expect("table");
    let rows: Vec<Row> = table
        .lines()
        .skip(1)
        .map(|l| Row::parse(l).expect("row"))
        .collect();
    assert_eq!(rows.len(), 2);
    for r in &rows {
        println!("MEASURED twice: {}", r.line());
        assert_eq!(r.get("repeats"), "yes", "{}", r.line());
    }
}

/// Test 6. A child killed in mid-flight: the batch reports it, flies the run
/// again, and the second row is the complete one; the program the child left
/// is ended.
#[cfg(unix)]
#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn a_child_killed_in_mid_flight_is_reported_and_the_run_flown_again() {
    let all = lease_all();
    let root = TempDir::new("killed");
    let out = root.path().join("batch");
    let mut child = bridge_command()
        .args(["batch", "--scenario"])
        .arg(scenario_path("gust-hold"))
        .args(["--indices", "0..2", "--jobs", "2", "--instances"])
        .arg(format!("{}-{}", all.range.0, all.range.1))
        .arg("--out")
        .arg(&out)
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("starts");
    let stdout = child.stdout.take().expect("stdout");
    let mut seen = Vec::new();
    let mut killed = false;
    for line in BufReader::new(stdout).lines() {
        let line = line.expect("line");
        println!("batch says: {line}");
        if !killed && line.starts_with("run 1 attempt 1: started, pid ") {
            let pid = line
                .trim_start_matches("run 1 attempt 1: started, pid ")
                .split(',')
                .next()
                .expect("pid")
                .to_string();
            // Mid-flight: the program is running (it has written its pid)
            // and has flown for a second of the few a flight takes.
            let pid_file = out.join("run-0001").join("sitl").join("sitl.pid");
            let waited = std::time::Instant::now();
            while !pid_file.exists() && waited.elapsed().as_secs() < 30 {
                std::thread::sleep(std::time::Duration::from_millis(20));
            }
            std::thread::sleep(std::time::Duration::from_millis(1200));
            let ok = Command::new("kill")
                .args(["-KILL", &pid])
                .status()
                .expect("kill")
                .success();
            assert!(ok, "kill {pid}");
            killed = true;
        }
        seen.push(line);
    }
    let status = child.wait().expect("waits");
    assert!(killed, "run 1 was never started: {seen:?}");
    assert_eq!(status.code(), Some(0), "{seen:?}");
    assert!(
        seen.iter().any(|l| l
            .starts_with("run 1 attempt 1: failed: the child was ended by signal 9")
            && l.ends_with("(flown once more)")),
        "{seen:?}"
    );
    let table = std::fs::read_to_string(out.join("flights.csv")).expect("table");
    let rows: Vec<Row> = table
        .lines()
        .skip(1)
        .map(|l| Row::parse(l).expect("row"))
        .collect();
    assert_eq!(rows.len(), 2, "{table}");
    println!("MEASURED killed child row: {}", rows[1].line());
    assert_eq!(rows[0].get("retried"), "no");
    assert_eq!(rows[1].get("started"), "yes");
    assert_eq!(rows[1].get("retried"), "yes");
    assert!(
        rows[1].get("note").contains("signal 9"),
        "{}",
        rows[1].line()
    );
    assert!(out.join("run-0001.failed").exists());
    // Nothing of this batch is left running.
    let left = Command::new("pgrep")
        .args(["-f", out.to_str().expect("path")])
        .output()
        .expect("pgrep");
    assert!(
        text(&left.stdout).trim().is_empty(),
        "processes of the batch are left: {}",
        text(&left.stdout)
    );
}

/// Test 7. The binary's exit codes with the real program: 0 for a flight
/// flown and judged, 1 for a wrong use, 2 for a flight that did not start.
#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn the_binary_exits_0_when_flown_1_when_wrongly_used_and_2_when_it_did_not_start() {
    let lease = Instance::lease();
    let root = TempDir::new("exit-codes");
    let hover = scenario_path("hover-step");
    // 0: flown and judged, whatever the verdict.
    let out = root.path().join("ok");
    let o = bridge_command()
        .args(["fly", "--scenario"])
        .arg(&hover)
        .args(["--instance", &lease.0.to_string(), "--out"])
        .arg(&out)
        .output()
        .expect("runs");
    println!("{}", text(&o.stdout));
    assert_eq!(o.status.code(), Some(0), "{}", text(&o.stderr));
    let row = read_row(&out.join("row.csv")).expect("row");
    println!("MEASURED hover-step row: {}", row.line());
    assert_eq!(row.get("started"), "yes");
    let audit = std::fs::read_to_string(out.join("audit.txt")).expect("audit");
    assert!(
        audit.contains("file sha256") && audit.contains("flight audit"),
        "{audit}"
    );
    assert!(
        text(&o.stdout).contains("result     PASS")
            || text(&o.stdout).contains("result     NOT JUDGED")
            || text(&o.stdout).contains("result     FAIL"),
        "{}",
        text(&o.stdout)
    );
    // 1: a wrong use.
    let o = bridge_command()
        .args(["fly", "--scenario"])
        .arg(&hover)
        .args(["--instance", "12"])
        .output()
        .expect("runs");
    assert_eq!(o.status.code(), Some(1), "{}", text(&o.stderr));
    // 2: a program that is not the pinned one does not start.
    let fake = root.path().join("not-arducopter");
    std::fs::write(&fake, b"#!/bin/sh\nexit 0\n").expect("fake");
    let o = bridge_command()
        .args(["fly", "--scenario"])
        .arg(&hover)
        .args(["--instance", &lease.0.to_string(), "--binary"])
        .arg(&fake)
        .arg("--out")
        .arg(root.path().join("nostart"))
        .output()
        .expect("runs");
    assert_eq!(o.status.code(), Some(2), "{}", text(&o.stderr));
    assert!(
        text(&o.stderr).contains("did not start"),
        "{}",
        text(&o.stderr)
    );
    let row = read_row(&root.path().join("nostart").join("row.csv")).expect("row");
    assert_eq!(row.get("started"), "no");
}

/// Not a check: the wall times, printed. A batch of six runs at one, two and
/// three jobs, pinned and not, and one flight alone. It does nothing unless
/// `OXIDE_MEASURE_WALL` is set (it takes minutes and shares the machine).
#[test]
#[ignore = "a measurement: needs the program and OXIDE_MEASURE_WALL"]
fn measure_the_wall_time_of_a_flight_and_of_batches() {
    if std::env::var_os("OXIDE_MEASURE_WALL").is_none() {
        println!("OXIDE_MEASURE_WALL is not set: nothing measured");
        return;
    }
    let all = lease_all();
    let root = TempDir::new("wall");
    let range = format!("{}-{}", all.range.0, all.range.1);
    let alone = std::time::Instant::now();
    let _ = single_row(root.path(), "alone", 0, all.range.0, &[]);
    println!(
        "MEASURED one flight alone: {:.2} s",
        alone.elapsed().as_secs_f64()
    );
    for (jobs, pin) in [
        (1, true),
        (2, true),
        (3, true),
        (1, false),
        (2, false),
        (3, false),
    ] {
        let out = root.path().join(format!("b-{jobs}-{pin}"));
        let began = std::time::Instant::now();
        let mut c = bridge_command();
        c.args(["batch", "--scenario"])
            .arg(scenario_path("gust-hold"))
            .args([
                "--indices",
                "0..6",
                "--jobs",
                &jobs.to_string(),
                "--instances",
                &range,
            ])
            .arg("--out")
            .arg(&out);
        if !pin {
            c.arg("--no-pin");
        }
        let o = c.output().expect("runs");
        assert_eq!(o.status.code(), Some(0), "{}", text(&o.stderr));
        let wall = began.elapsed().as_secs_f64();
        println!(
            "MEASURED batch of 6 at --jobs {jobs}, pinned {pin}: {wall:.2} s ({:.2} s per flight)",
            wall / 6.0
        );
    }
}
