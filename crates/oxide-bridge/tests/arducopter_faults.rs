// SPDX-License-Identifier: AGPL-3.0-or-later
//! Failures reach ArduCopter (ADR-0006 section 6, ADR-0007 section 3).
//!
//! Every test here needs the binary and is ignored by default: set
//! `OXIDE_ARDUCOPTER` to its path and run with `--ignored` and `--release`.
//! Each scenario file of the failures is flown at run 0 through the bridge
//! with `base.parm`, `setup.parm`, `tuned-gains.parm`, `failsafes.parm` and the
//! battery
//! thresholds of our core ([`battery_failsafe_params`]), and each test pins
//! what ArduCopter did. The glue that turns a scenario file into a bridged
//! flight is in this file; the runner for many flights is another task's.

use std::path::{Path, PathBuf};
use std::time::Instant;

use oxide_bridge::arducopter::{
    ArduCopter, ArduCopterConfig, FlightAudit, Flown, PREROLL_STEPS, STEP_US, SentKind,
    bridged_options, fly,
};
use oxide_bridge::bridge::RcLatch;
use oxide_bridge::faults::{HOVER_POWER_W, battery_failsafe_params};
use oxide_bridge::mav::MavEvent;
use oxide_bridge::parm;
use oxide_bridge::sitl::PINNED;
use oxide_core::config::FailsafeParams;
use oxide_core::io::TrajectorySetpoint;
use oxide_core::math::{Real, Vec3};
use oxide_core::time::Micros;
use oxide_sim::autopilot::{Autopilot, AutopilotInput, AutopilotOutput};
use oxide_sim::faults::FaultKind;
use oxide_sim::outcome::FlightOutcome;
use oxide_sim::scenario::NoPilot;
use oxide_sim::scenario_file::{Checker, ScenarioFile, Verdict, load_path};
use oxide_sim::{Airframe, SimOptions, Simulation, StepRecord};

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

struct TempDir(PathBuf);

impl TempDir {
    fn new(tag: &str) -> Self {
        let p = std::env::temp_dir().join(format!(
            "oxide-arducopter-faults-{}-{tag}",
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

fn airframe() -> Airframe {
    Airframe::reference_quad().expect("the reference airframe")
}

// ---- The glue: a scenario file as a bridged flight --------------------------

fn scenario_path(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../scenarios")
        .join(format!("{name}.toml"))
}

/// The files and extra parameters of a bridged flight: the runner's own.
fn configure(c: &mut ArduCopterConfig, airframe: &Airframe) {
    let (files, extras) = oxide_bridge::runner::committed_params(airframe);
    c.param_files = files;
    c.extra_params = extras;
}

/// What a recorded flight keeps of each tick.
#[derive(Clone, Copy, Debug)]
struct Tick {
    t_s: f64,
    motors: [f32; 4],
    position: [f64; 3],
    velocity: [f64; 3],
    tilt_deg: f64,
    battery_v: f64,
}

struct Run {
    name: String,
    file: ScenarioFile,
    verdict: Verdict,
    flown: Flown,
    ticks: Vec<Tick>,
    wall_s: f64,
}

impl Run {
    fn audit(&self) -> &FlightAudit {
        &self.flown.audit
    }

    fn outcome(&self) -> &FlightOutcome {
        &self.verdict.outcome
    }

    /// Flight time of a program time, s.
    fn flight_s(&self, program_us: u64) -> f64 {
        let a = self.audit();
        (program_us as f64 - (a.preroll_steps * a.step_us) as f64) * 1e-6
    }

    /// The time of the last motor command above zero, s.
    fn last_motor_on_s(&self) -> Option<f64> {
        self.ticks
            .iter()
            .rev()
            .find(|k| k.motors.iter().any(|m| *m > 0.0))
            .map(|k| k.t_s)
    }

    /// The vehicle's modes with the flight time each was read at, names.
    fn modes(&self) -> Vec<(f64, &'static str)> {
        self.audit()
            .mode_changes
            .iter()
            .map(|(t, m)| (self.flight_s(*t), mode_name(*m)))
            .collect()
    }

    fn mode_names(&self) -> Vec<&'static str> {
        self.modes().into_iter().map(|(_, m)| m).collect()
    }

    /// Status texts after the pre-roll, with their flight times. Every link
    /// hears each text, so an identical text read at the same step is kept
    /// once.
    fn flight_texts(&self) -> Vec<(f64, String)> {
        let mut out: Vec<(f64, String)> = Vec::new();
        for (t, _, x) in &self.audit().status_texts {
            let item = (self.flight_s(*t), x.clone());
            if item.0 >= 0.0 && !out.contains(&item) {
                out.push(item);
            }
        }
        out
    }

    /// The flight time of the last telemetry message (`SYSTEM_TIME` or
    /// `LOCAL_POSITION_NED`, which the instrument asks for at 10 Hz) read
    /// from the program: when it last spoke.
    fn last_telemetry_s(&self) -> Option<f64> {
        self.audit()
            .heard
            .iter()
            .filter(|h| {
                matches!(
                    h.event,
                    MavEvent::SystemTime { .. } | MavEvent::LocalPosition { .. }
                )
            })
            .map(|h| self.flight_s(h.at_us))
            .fold(None, |m: Option<f64>, t| Some(m.map_or(t, |m| m.max(t))))
    }

    /// The first flight text that contains `needle`, with its time.
    fn text(&self, needle: &str) -> Option<(f64, String)> {
        self.flight_texts()
            .into_iter()
            .find(|(_, x)| x.contains(needle))
    }

    /// The first time the vehicle read as in `mode` at or after `from_s`.
    fn mode_at_or_after(&self, mode: &str, from_s: f64) -> Option<f64> {
        self.modes()
            .into_iter()
            .find(|(t, m)| *m == mode && *t >= from_s)
            .map(|(t, _)| t)
    }

    /// The parameter writes sent, with the flight time of the exchange each
    /// went before.
    fn writes(&self) -> Vec<(f64, String, f32)> {
        self.audit()
            .param_writes()
            .into_iter()
            .map(|(t, n, v)| (self.flight_s(t), n.to_string(), v))
            .collect()
    }

    /// The ticks in `[from_s, to_s)`.
    fn between(&self, from_s: f64, to_s: f64) -> impl Iterator<Item = &Tick> {
        self.ticks
            .iter()
            .filter(move |k| k.t_s >= from_s && k.t_s < to_s)
    }
}

/// ArduCopter's vehicle mode numbers, as the program's heartbeat reports them.
fn mode_name(m: u32) -> &'static str {
    match m {
        0 => "STABILIZE",
        1 => "ACRO",
        2 => "ALT_HOLD",
        3 => "AUTO",
        4 => "GUIDED",
        5 => "LOITER",
        6 => "RTL",
        7 => "CIRCLE",
        9 => "LAND",
        11 => "DRIFT",
        13 => "SPORT",
        16 => "POSHOLD",
        17 => "BRAKE",
        21 => "SMART_RTL",
        _ => "OTHER",
    }
}

/// Flies run `index` of the scenario file `name` through the bridge.
fn fly_run(name: &str, index: u64, instance: u8, tag: &str) -> Run {
    fly_run_with(name, index, instance, tag, |_| {})
}

/// Like [`fly_run`], with the scenario file edited first (a test's own
/// variation of a committed scenario).
fn fly_run_with(
    name: &str,
    index: u64,
    instance: u8,
    tag: &str,
    edit: impl FnOnce(&mut ScenarioFile),
) -> Run {
    let mut file = load_path(scenario_path(name)).unwrap_or_else(|e| panic!("{e}"));
    edit(&mut file);
    let af = airframe();
    // The file's [params] are our core's: they do not reach ArduCopter here.
    let mut options = file.options(&af, index);
    options.params.clear();
    let root = TempDir::new(tag);
    let mut c = ArduCopterConfig::new(binary(), root.path().join("flights"), instance);
    configure(&mut c, &af);
    let mut checker = Checker::new(&file);
    let mut ticks = Vec::new();
    let wall = Instant::now();
    let flown = fly(&af, file.pilot.clone(), options, c, file.end(), |r| {
        checker.add(r);
        ticks.push(tick_of(r));
    })
    .unwrap_or_else(|e| panic!("{name}: {e}"));
    let wall_s = wall.elapsed().as_secs_f64();
    Run {
        name: name.to_string(),
        file,
        verdict: checker.finish(),
        flown,
        ticks,
        wall_s,
    }
}

/// The angle between the body's down axis and the vertical, deg.
fn tilt_deg(r: &StepRecord<4>) -> f64 {
    let q = r.attitude;
    // The third row of the body-to-world rotation: where the body axes point
    // in the vertical.
    let r33 = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    let r31 = 2.0 * (q.x * q.z - q.w * q.y);
    let r32 = 2.0 * (q.y * q.z + q.w * q.x);
    Real::atan2(Real::sqrt(r31 * r31 + r32 * r32), r33) * (180.0 / core::f64::consts::PI)
}

fn tick_of(r: &StepRecord<4>) -> Tick {
    Tick {
        t_s: r.t.as_secs_f64(),
        motors: r.motors,
        position: [r.position.x, r.position.y, r.position.z],
        velocity: [r.velocity.x, r.velocity.y, r.velocity.z],
        tilt_deg: tilt_deg(r),
        battery_v: r.battery_v,
    }
}

/// Prints what a flight did: everything the report of this task is made of.
fn print_run(run: &Run) {
    let a = run.audit();
    let o = run.outcome();
    println!(
        "=== {} run flown in {:.1} s of wall time",
        run.name, run.wall_s
    );
    println!(
        "FAULTS {:?}",
        run.file
            .faults
            .iter()
            .map(|f| (f.kind, f.start_s, f.end_s))
            .collect::<Vec<_>>()
    );
    println!("WRITES {:?}", run.writes());
    println!(
        "ECHOES {:?}",
        a.param_echoes()
            .iter()
            .filter(|(_, n, _)| a.param_writes().iter().any(|(_, w, _)| w == n))
            .map(|(t, n, v)| (run.flight_s(*t), *n, *v))
            .collect::<Vec<_>>()
    );
    println!("MODES {:?}", run.modes());
    for (t, x) in run.flight_texts() {
        println!("TEXT {t:8.3} {x}");
    }
    println!(
        "ARMED {:?}",
        a.armed_changes
            .iter()
            .map(|(t, v)| (run.flight_s(*t), *v))
            .collect::<Vec<_>>()
    );
    println!("OUTCOME {o:?}");
    println!(
        "VERDICT violations {:?}\n        not judged {:?}",
        run.verdict.violations,
        run.verdict
            .not_judged
            .iter()
            .map(ToString::to_string)
            .collect::<Vec<_>>()
    );
    println!("LASTTELEMETRY {:?}", run.last_telemetry_s());
    println!(
        "LAST MOTOR ON {:?} s; flight ended at {:.3} s; hashes trace {:016x} packet {:016x} reply {:016x}",
        run.last_motor_on_s(),
        o.end_s,
        run.flown.trace_hash,
        a.packet_hash,
        a.reply_hash
    );
    if let Some(m) = &run.verdict.hold {
        println!("HOLD {m:?}");
    }
}

// ---- What a run is held against ------------------------------------------

/// How a flight ended, from the physics alone (ADR-0007 section 4).
#[derive(Debug, PartialEq)]
enum Class {
    Safe,
    Crash,
    FlyAway,
}

fn class_of(o: &FlightOutcome) -> Class {
    if o.touchdown_speed_mps.is_some_and(|v| v > 2.0) || o.max_tilt_deg > 60.0 {
        Class::Crash
    } else if o.max_distance_from_home_m > 100.0 || o.max_height_m > 60.0 {
        Class::FlyAway
    } else {
        Class::Safe
    }
}

/// Asserts `actual` is within `tol` of `expect`, and prints it.
fn near(what: &str, actual: f64, expect: f64, tol: f64) {
    println!("MEASURED {what}: {actual:.4} (pinned {expect} +- {tol})");
    assert!(
        (actual - expect).abs() <= tol,
        "{what}: {actual} is not within {tol} of {expect}"
    );
}

/// The parameter writes of a flight, as (start in whole microseconds of
/// flight time, name, value) and asserted equal to `want`: each at the
/// fault's start to the tick.
fn assert_writes(run: &Run, want: &[(u64, &str, f32)]) {
    let a = run.audit();
    let start = a.preroll_steps * a.step_us;
    let got: Vec<(u64, String, f32)> = a
        .param_writes()
        .into_iter()
        .map(|(t, n, v)| (t - start, n.to_string(), v))
        .collect();
    let want: Vec<(u64, String, f32)> = want
        .iter()
        .map(|(t, n, v)| (*t, n.to_string(), *v))
        .collect();
    println!("MEASURED writes (flight time us, name, value): {got:?}");
    assert_eq!(got, want, "the parameter writes and their times");
}

/// Whether the estimator's failsafe text was read after flight time `after_s`.
fn estimator_failsafe_read_after(run: &Run, after_s: f64) -> bool {
    run.text("EKF Failsafe: changed to Land Mode")
        .is_some_and(|(t, _)| t > after_s)
}

/// Every write was acknowledged by a `PARAM_VALUE` with the value written,
/// read within 20 ms of the exchange it went before.
fn assert_echoes(run: &Run) {
    let a = run.audit();
    let echoes = a.param_echoes();
    for (t, name, value) in a.param_writes() {
        let first = echoes
            .iter()
            .find(|(et, en, ev)| *et >= t && *en == name && *ev == value)
            .unwrap_or_else(|| panic!("{name}: no echo with the value {value} was read"));
        let delay_ms = (first.0 - t) as f64 * 1e-3;
        println!("MEASURED echo of {name} {value}: read {delay_ms:.1} ms after the write");
        assert!(delay_ms <= 20.0, "{name}: echo after {delay_ms} ms");
    }
}

/// No status text read in the flight from `from_s` on, but the ones listed.
fn assert_texts_after(run: &Run, from_s: f64, allowed: &[(&str, f64, f64)]) {
    let texts: Vec<(f64, String)> = run
        .flight_texts()
        .into_iter()
        .filter(|(t, _)| *t >= from_s)
        .collect();
    println!("MEASURED status texts from {from_s} s: {texts:?}");
    assert_eq!(texts.len(), allowed.len(), "{texts:?}");
    for ((t, x), (want, at, tol)) in texts.iter().zip(allowed) {
        assert!(x.contains(want), "{x:?} does not contain {want:?}");
        near(&format!("time of {want:?}"), *t, *at, *tol);
    }
}

fn assert_modes(run: &Run, want: &[&str]) {
    println!("MEASURED modes {:?}", run.modes());
    assert_eq!(run.mode_names(), want);
}

fn assert_violations(run: &Run, contain: &[&str]) {
    let v = &run.verdict.violations;
    assert_eq!(v.len(), contain.len(), "violations {v:?}");
    for (v, c) in v.iter().zip(contain) {
        assert!(v.contains(c), "{v:?} does not contain {c:?}");
    }
}

/// The ending of the flight from the truth: the horizontal distance of its
/// last position from `(n, e)` and the height above the ground.
fn end_position(run: &Run) -> ([f64; 3], f64) {
    let p = run.outcome().position_at_end;
    ([p.x, p.y, -p.z], Real::sqrt(p.x * p.x + p.y * p.y))
}

/// The heartbeats the bridge sent as `system_id`, flight times.
fn heartbeats_as(run: &Run, system_id: u8) -> Vec<f64> {
    run.audit()
        .sent
        .iter()
        .filter(|s| s.system_id == system_id && s.what == SentKind::Heartbeat)
        .map(|s| run.flight_s(s.at_us))
        .collect()
}

/// The position targets the bridge sent, flight times.
fn targets_sent(run: &Run) -> Vec<f64> {
    run.audit()
        .sent
        .iter()
        .filter(|s| matches!(s.what, SentKind::PositionTarget { .. }))
        .map(|s| run.flight_s(s.at_us))
        .collect()
}

/// The first flight time the pack's voltage was below `volts`.
fn first_below(run: &Run, volts: f64) -> Option<f64> {
    run.ticks
        .iter()
        .find(|k| k.battery_v < volts)
        .map(|k| k.t_s)
}

/// The start of the last stretch, up to flight time `until`, in which the
/// pack's voltage was below `volts` all the time: the crossing that
/// ArduCopter's low-battery timer (BATT_LOW_TIMER) counts from. A dip of the
/// loaded voltage that recovers before it is not one.
fn below_since(run: &Run, volts: f64, until: f64) -> Option<f64> {
    let mut since = None;
    for k in run.ticks.iter().take_while(|k| k.t_s <= until) {
        if k.battery_v < volts {
            since = since.or(Some(k.t_s));
        } else {
            since = None;
        }
    }
    since
}

fn s_us(s: f64) -> u64 {
    (s * 1e6).round() as u64
}

fn run_of(name: &str) -> (Instance, Run) {
    let lease = Instance::lease();
    let run = fly_run(name, 0, lease.0, name);
    print_run(&run);
    (lease, run)
}

// ---- The names exist on the binary ------------------------------------------

/// Every parameter this crate writes or sets exists in the program (a
/// misspelt name in a defaults file is ignored without a word, and a write to
/// a name that does not exist is not answered), with the value the files and
/// the battery function give.
#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn the_failsafe_names_the_fault_names_and_the_battery_names_exist_in_the_program() {
    let lease = Instance::lease();
    let root = TempDir::new("names");
    let af = airframe();
    let mut c = ArduCopterConfig::new(binary(), root.path().join("flights"), lease.0);
    configure(&mut c, &af);
    let options = bridged_options(SimOptions::realistic(1), &mut c);
    let ap = ArduCopter::start(c).unwrap_or_else(|e| panic!("{e}"));
    let mut sim = Simulation::with_autopilot(&af, ap, NoPilot, options).expect("the simulation");
    sim.preroll(PREROLL_STEPS).unwrap_or_else(|e| panic!("{e}"));
    let failsafes = parm::parse(parm::FAILSAFES).expect("failsafes.parm");
    let battery = battery_failsafe_params(
        &FailsafeParams::default(),
        u8::try_from(af.battery.cells).expect("cells"),
        af.battery.internal_resistance_ohm,
        HOVER_POWER_W,
    );
    let mut names: Vec<String> = [
        "SIM_GPS1_ENABLE",
        "SIM_BARO_DISABLE",
        "SIM_BAR2_DISABLE",
        "SIM_BAR3_DISABLE",
        "SIM_BAR4_DISABLE",
        "SIM_BAR_DISABLE",
        "SIM_MAG1_FAIL",
        "SIM_MAG2_FAIL",
        "SIM_MAG3_FAIL",
        "SIM_MAG4_FAIL",
        "SIM_ACC_FAIL_MSK",
        "SIM_GYR_FAIL_MSK",
        "SIM_IMU_COUNT",
        "SIM_BARO_COUNT",
        "BATT_FS_VOLTSRC",
        "BATT_LOW_TIMER",
        "NO_SUCH_PARAM",
    ]
    .iter()
    .map(ToString::to_string)
    .collect();
    names.extend(failsafes.iter().map(|(n, _)| n.clone()));
    names.extend(battery.iter().map(|(n, _)| n.clone()));
    // The program answers only a few requests at a time: ask in small groups
    // and step on between them.
    for group in names.chunks(6) {
        for n in group {
            sim.autopilot()
                .instrument()
                .param_request(n)
                .expect("request");
        }
        for _ in 0..2_000 {
            sim.try_step().unwrap_or_else(|e| panic!("{e}"));
        }
    }
    let audit = sim
        .into_autopilot()
        .finish()
        .unwrap_or_else(|e| panic!("{e}"));
    let read = |n: &str| -> Option<f32> {
        audit
            .param_echoes()
            .iter()
            .find(|(_, en, _)| *en == n)
            .map(|e| e.2)
    };
    for n in &names {
        println!("MEASURED {n} = {:?}", read(n));
    }
    // The ones that exist, with the value the program starts with.
    for n in [
        "SIM_GPS1_ENABLE",
        "SIM_BARO_DISABLE",
        "SIM_BAR2_DISABLE",
        "SIM_BAR3_DISABLE",
        "SIM_MAG1_FAIL",
        "SIM_MAG2_FAIL",
        "SIM_MAG3_FAIL",
        "SIM_ACC_FAIL_MSK",
        "SIM_GYR_FAIL_MSK",
    ] {
        let want = if n == "SIM_GPS1_ENABLE" { 1.0 } else { 0.0 };
        assert_eq!(read(n), Some(want), "{n}");
    }
    // The ones that do not: the translator's lists are every name there is.
    for n in [
        "SIM_BAR4_DISABLE",
        "SIM_BAR_DISABLE",
        "SIM_MAG4_FAIL",
        "NO_SUCH_PARAM",
    ] {
        assert_eq!(read(n), None, "{n} must not exist");
    }
    // The program simulates the number of IMUs the masks cover.
    assert_eq!(
        read("SIM_IMU_COUNT"),
        Some(oxide_bridge::faults::IMU_COUNT as f32)
    );
    // The failsafes file and the battery thresholds are in the program as set
    // (a name that does not exist would read as nothing).
    for (n, v) in failsafes.iter().chain(&battery) {
        assert_eq!(
            read(n),
            Some(*v as f32),
            "{n} as the files and the battery function set it"
        );
    }
    // What the thresholds are compared with and how long they must hold.
    assert_eq!(read("BATT_FS_VOLTSRC"), Some(0.0), "the raw pack voltage");
    assert_eq!(
        read("BATT_LOW_TIMER"),
        Some(10.0),
        "the program's own timer"
    );
}

// ---- GPS --------------------------------------------------------------------

#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn gps_loss_ends_in_the_estimator_failsafe_8_s_later_and_a_landing_14_m_away() {
    let (_lease, run) = run_of("gps-loss");
    assert_writes(&run, &[(s_us(15.0), "SIM_GPS1_ENABLE", 0.0)]);
    assert_echoes(&run);
    assert_modes(&run, &["STABILIZE", "GUIDED", "LAND"]);
    assert_texts_after(
        &run,
        9.0,
        &[
            ("EKF variance", 23.024, 0.15),
            ("EKF Failsafe: changed to Land Mode", 23.024, 0.15),
            ("EKF3 IMU0 stopped aiding", 30.092, 0.15),
            ("EKF3 IMU1 stopped aiding", 30.097, 0.15),
            ("SmartRTL deactivated: bad position", 37.389, 0.3),
            ("Disarming motors", 40.654, 0.3),
        ],
    );
    let (t, _) = run
        .text("EKF Failsafe: changed to Land Mode")
        .expect("the failsafe text");
    near(
        "time from the GPS loss to the EKF failsafe",
        t - 15.0,
        8.02,
        0.1,
    );
    assert!(run.text("EKF variance").is_some());
    let land = run.mode_at_or_after("LAND", 0.0).expect("land");
    near("LAND read", land, t, 1.1);
    let o = run.outcome();
    assert!(!o.armed_at_end);
    near("disarmed", o.disarmed_at_s.expect("disarmed"), 40.65, 0.3);
    near(
        "last motor command above zero",
        run.last_motor_on_s().expect("on"),
        40.65,
        0.3,
    );
    near(
        "touchdown speed",
        o.touchdown_speed_mps.expect("touchdown"),
        0.517,
        0.1,
    );
    let (_, away) = end_position(&run);
    near("landing distance from home", away, 13.6, 2.0);
    assert!(o.max_height_m < 6.0, "no climb");
    assert_eq!(class_of(o), Class::Safe);
    assert!(
        run.verdict.violations.is_empty(),
        "{:?}",
        run.verdict.violations
    );
    assert_eq!(run.verdict.not_judged.len(), 5);
}

#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn a_gps_loss_of_4_s_that_ends_is_written_off_and_nothing_happens() {
    let (_lease, run) = run_of("gps-loss-recovers");
    assert_writes(
        &run,
        &[
            (s_us(15.0), "SIM_GPS1_ENABLE", 0.0),
            (s_us(19.0), "SIM_GPS1_ENABLE", 1.0),
        ],
    );
    assert_echoes(&run);
    assert_modes(&run, &["STABILIZE", "GUIDED"]);
    assert_texts_after(&run, 9.0, &[]);
    let o = run.outcome();
    assert!(o.armed_at_end);
    let ([n, e, h], _) = end_position(&run);
    near("end north", n, 0.0, 0.5);
    near("end east", e, 0.0, 0.5);
    near("end height", h, 5.0, 0.5);
    assert!(o.max_tilt_deg < 10.0, "{}", o.max_tilt_deg);
    assert_eq!(class_of(o), Class::Safe);
    assert!(run.verdict.violations.is_empty());
}

// ---- Barometer, compass -----------------------------------------------------

#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn baro_loss_is_not_noticed_and_the_vehicle_climbs_without_limit() {
    let (_lease, run) = run_of("baro-loss");
    assert_writes(
        &run,
        &[
            (s_us(15.0), "SIM_BARO_DISABLE", 1.0),
            (s_us(15.0), "SIM_BAR2_DISABLE", 1.0),
            (s_us(15.0), "SIM_BAR3_DISABLE", 1.0),
        ],
    );
    assert_echoes(&run);
    assert_modes(&run, &["STABILIZE", "GUIDED"]);
    assert_texts_after(
        &run,
        9.0,
        &[("SmartRTL deactivated: bad position", 40.05, 0.5)],
    );
    let o = run.outcome();
    assert!(o.armed_at_end);
    assert!(o.touchdown_speed_mps.is_none());
    near("greatest height", o.max_height_m, 137.7, 3.0);
    let above_60 = run
        .ticks
        .iter()
        .find(|k| -k.position[2] > 60.0)
        .map(|k| k.t_s)
        .expect("above 60 m");
    near("time to 60 m after the fault", above_60 - 15.0, 25.0, 4.0);
    let last = run.ticks.last().expect("ticks");
    assert!(
        last.velocity[2] < -1.0,
        "still climbing: {}",
        last.velocity[2]
    );
    assert!(last.motors.iter().all(|m| *m > 0.0));
    assert_eq!(class_of(o), Class::FlyAway);
}

#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn mag_loss_is_not_noticed_in_a_hover_for_75_s() {
    let (_lease, run) = run_of("mag-loss");
    assert_writes(
        &run,
        &[
            (s_us(15.0), "SIM_MAG1_FAIL", 1.0),
            (s_us(15.0), "SIM_MAG2_FAIL", 1.0),
            (s_us(15.0), "SIM_MAG3_FAIL", 1.0),
        ],
    );
    assert_echoes(&run);
    assert_modes(&run, &["STABILIZE", "GUIDED"]);
    assert_texts_after(&run, 9.0, &[]);
    let o = run.outcome();
    assert!(o.armed_at_end);
    let ([n, e, h], _) = end_position(&run);
    near("end north", n, 0.0, 0.5);
    near("end east", e, 0.0, 0.5);
    near("end height", h, 5.0, 0.5);
    assert!(o.max_distance_from_home_m < 1.0);
    assert_eq!(class_of(o), Class::Safe);
    assert_violations(&run, &["still armed", "never came down"]);
}

// ---- IMU and motor ----------------------------------------------------------

#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn imu_loss_stops_the_program_and_its_motors_go_to_zero_2_s_later_and_the_vehicle_falls() {
    let (_lease, run) = run_of("imu-loss");
    assert_writes(
        &run,
        &[
            (s_us(15.0), "SIM_ACC_FAIL_MSK", 3.0),
            (s_us(15.0), "SIM_GYR_FAIL_MSK", 3.0),
        ],
    );
    assert_echoes(&run);
    assert_modes(&run, &["STABILIZE", "GUIDED"]);
    assert_texts_after(&run, 9.0, &[]);
    // The program says nothing after the write's own echo.
    let last = run.last_telemetry_s().expect("telemetry");
    near("last telemetry read", last, 15.0, 0.02);
    let off = run.last_motor_on_s().expect("on");
    near(
        "time from the fault to the last motor command above zero",
        off - 15.0,
        2.0,
        0.01,
    );
    let o = run.outcome();
    near(
        "disarmed (the pulses at idle)",
        o.disarmed_at_s.expect("disarmed"),
        17.0,
        0.01,
    );
    near(
        "touchdown speed",
        o.touchdown_speed_mps.expect("touchdown"),
        10.16,
        0.5,
    );
    assert_eq!(class_of(o), Class::Crash);
    // After that the motors never come on again.
    assert!(
        run.between(off + 0.001, 1e9)
            .all(|k| k.motors.iter().all(|m| *m == 0.0))
    );
    assert_violations(&run, &["disarmed at 17.00 s, later than 15.3 s"]);
}

#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn motor_loss_is_not_answered_the_vehicle_falls_in_2_s_and_its_motors_are_still_on_when_it_is_down()
{
    let (_lease, run) = run_of("motor-loss");
    assert_writes(&run, &[]);
    // In run 0, with either tuned file, the vehicle is on the ground 2.2 s
    // after the fault. What follows depends on the draw of the IMU's noise.
    // (a) The IMU before ADR-0008 with gate 1's tuned file: it stayed within 24 m, the
    // program's own estimator failsafe (EKF variance, LAND) came 1.2 s after
    // it was down, and it was still armed at the end. (b) The IMU of ADR-0008
    // with gate 1's file: the motors, still on, dragged it along the ground
    // to 237 m from home, never more than 0.1 m up, with no mode change,
    // armed to the end. (c) The IMU of ADR-0008 with the tuned file of
    // ADR-0009 (this pin): it stays within 25 m, the estimator failsafe and
    // LAND come 1.2 s after it is down, and the program, in LAND, judges it
    // landed and disarms 11.3 s after the fault. On the starting gains it
    // tumbled away in the air for 8.9 s, over 100 m, with no mode change. In
    // none of the four is the lost motor noticed or answered before the
    // vehicle is down, and with either tuned file the motors are still on
    // when it touches the ground. How long they stay on is the draw: on runs
    // 500 to 519 the program's crash check or a completed LAND disarms 1
    // flight of 20 on this file and 4 of 20 on gate 1's, 5.2 s after the
    // fault at the earliest (ADR-0009). Every number is as measured on the
    // flown set.
    assert_modes(&run, &["STABILIZE", "GUIDED", "LAND"]);
    assert_texts_after(
        &run,
        9.0,
        &[
            ("EKF3 lane switch 1", 17.6895, 0.3),
            ("EKF primary changed:1", 17.6895, 0.3),
            ("EKF variance", 18.4245, 0.3),
            ("EKF Failsafe: changed to Land Mode", 18.4245, 0.3),
            ("GPS Glitch or Compass error", 18.5245, 0.3),
            ("EKF3 IMU0 emergency yaw reset", 18.8995, 0.3),
            ("EKF3 lane switch 0", 18.8995, 0.3),
            ("EKF primary changed:0", 18.8995, 0.3),
            ("EKF3 IMU1 emergency yaw reset", 19.107, 0.3),
            ("EKF Failsafe Cleared", 19.8245, 0.3),
            ("Disarming motors", 26.3195, 0.3),
            ("Glitch cleared", 29.1245, 0.3),
        ],
    );
    let (failsafe, _) = run
        .text("EKF Failsafe: changed to Land Mode")
        .expect("the estimator failsafe");
    near(
        "the fault to the estimator failsafe",
        failsafe - 15.0,
        3.4245,
        0.3,
    );
    near(
        "LAND read",
        run.mode_at_or_after("LAND", 0.0).expect("land"),
        failsafe,
        1.1,
    );
    let o = run.outcome();
    assert!(!o.armed_at_end, "the program disarmed in LAND");
    near("greatest tilt", o.max_tilt_deg, 112.7, 5.0);
    let last = run.ticks.last().expect("ticks");
    println!(
        "MEASURED tilt at the end: {:.1} deg, height {:.3} m",
        last.tilt_deg, -last.position[2]
    );
    let beyond_60 = run
        .between(15.0, 1e9)
        .find(|k| k.tilt_deg > 60.0)
        .map(|k| k.t_s)
        .expect("beyond 60 degrees");
    let ground = run
        .between(15.5, 1e9)
        .find(|k| -k.position[2] < 0.05)
        .map(|k| k.t_s)
        .expect("ground contact");
    near(
        "time from the fault to a tilt beyond 60 degrees",
        beyond_60 - 15.0,
        0.3045,
        0.1,
    );
    near(
        "time from the fault to the ground",
        ground - 15.0,
        2.2375,
        0.3,
    );
    near(
        "touchdown speed",
        o.touchdown_speed_mps.expect("touchdown"),
        7.41,
        0.5,
    );
    near("furthest from home", o.max_distance_from_home_m, 24.56, 2.0);
    let after = || run.between(ground + 1.0, 1e9);
    let highest = after().map(|k| -k.position[2]).fold(0.0, f64::max);
    assert!(highest < 0.2, "it stays on the ground: {highest}");
    // The program does nothing before the vehicle is down: every mode change
    // after the arming, and the estimator failsafe, come after the contact.
    let armed_at = run
        .audit()
        .armed_changes
        .iter()
        .find(|(_, armed)| *armed)
        .map(|(t, _)| run.flight_s(*t))
        .expect("armed");
    println!("MEASURED armed read at {armed_at:.3} s, ground contact at {ground:.3} s");
    for (t, m) in run.modes().into_iter().filter(|(t, _)| *t > armed_at) {
        assert!(
            t > ground,
            "the mode {m} read at {t} is before the ground {ground}"
        );
    }
    assert!(
        estimator_failsafe_read_after(&run, ground),
        "the estimator failsafe comes after the vehicle is down"
    );
    // The motors are still on when it is down. How long after is this
    // flight's draw (the pin below), not a rule.
    let off = run.last_motor_on_s().expect("on");
    assert!(
        off > ground,
        "the last motor command above zero, at {off} s, is before the ground contact at {ground} s"
    );
    near("last motor command above zero", off, 26.318, 0.3);
    near("disarmed", o.disarmed_at_s.expect("disarmed"), 26.321, 0.3);
    assert_eq!(class_of(o), Class::Crash);
    assert_violations(&run, &["disarmed at 26.32 s, later than 17 s"]);
}

// ---- Radio, ground station --------------------------------------------------

#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn a_lost_radio_in_guided_with_the_brain_alive_returns_1_s_later_and_lands() {
    let (_lease, run) = run_of("rc-loss-offboard-gcs");
    assert_writes(&run, &[]);
    assert_modes(&run, &["STABILIZE", "GUIDED", "RTL"]);
    assert_texts_after(
        &run,
        9.0,
        &[
            ("Radio Failsafe", 15.98, 0.05),
            ("Disarming motors", 36.04, 0.3),
            ("PreArm: Radio failsafe on", 37.93, 0.3),
        ],
    );
    let (t, _) = run.text("Radio Failsafe").expect("text");
    near(
        "time from the radio loss to the failsafe",
        t - 15.0,
        0.98,
        0.05,
    );
    near(
        "RTL read",
        run.mode_at_or_after("RTL", 0.0).expect("rtl"),
        t,
        1.1,
    );
    // The brain and the ground station stayed alive all the time.
    assert!(heartbeats_as(&run, 251).last().copied().expect("brain") > 38.0);
    assert!(heartbeats_as(&run, 255).last().copied().expect("gcs") > 38.0);
    let o = run.outcome();
    assert!(!o.armed_at_end);
    near(
        "touchdown speed",
        o.touchdown_speed_mps.expect("touchdown"),
        0.51,
        0.1,
    );
    near("disarmed", o.disarmed_at_s.expect("disarmed"), 36.05, 0.3);
    assert!(o.max_height_m < 6.0, "it was at home and did not climb");
    let (_, away) = end_position(&run);
    assert!(away < 1.0, "{away}");
    assert_eq!(class_of(o), Class::Safe);
    assert_violations(
        &run,
        &["not armed at the end", "ended 5.00 m from the point"],
    );
}

#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn a_lost_ground_station_returns_5_1_s_after_its_last_heartbeat_and_lands() {
    let (_lease, run) = run_of("gcs-loss");
    assert_writes(&run, &[]);
    assert_modes(&run, &["STABILIZE", "GUIDED", "RTL"]);
    assert_texts_after(
        &run,
        9.0,
        &[
            ("GCS Failsafe", 19.1, 0.05),
            ("Disarming motors", 39.34, 0.3),
        ],
    );
    let hb = heartbeats_as(&run, 255);
    let last = hb.last().copied().expect("heartbeats");
    near(
        "the last ground-station heartbeat the bridge sent",
        last,
        14.0,
        1e-9,
    );
    assert!(hb.iter().all(|t| *t <= 14.0 + 1e-9), "none after the fault");
    let (t, _) = run.text("GCS Failsafe").expect("text");
    near(
        "time from the last heartbeat to the failsafe",
        t - last,
        5.1,
        0.05,
    );
    near("time from the fault to the failsafe", t - 15.0, 4.1, 0.05);
    near(
        "RTL read",
        run.mode_at_or_after("RTL", 0.0).expect("rtl"),
        t,
        1.1,
    );
    let o = run.outcome();
    assert!(!o.armed_at_end);
    near(
        "touchdown speed",
        o.touchdown_speed_mps.expect("touchdown"),
        0.51,
        0.1,
    );
    near("disarmed", o.disarmed_at_s.expect("disarmed"), 39.34, 0.3);
    assert_eq!(class_of(o), Class::Safe);
    assert_violations(
        &run,
        &["not armed at the end", "ended 5.00 m from the point"],
    );
}

#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn no_human_link_is_the_radio_failsafe_first_and_the_ground_station_failsafe_changes_nothing() {
    let (_lease, run) = run_of("no-human-link");
    assert_writes(&run, &[]);
    assert_modes(&run, &["STABILIZE", "GUIDED", "RTL"]);
    assert_texts_after(
        &run,
        9.0,
        &[
            ("Radio Failsafe", 15.98, 0.05),
            ("GCS Failsafe", 24.09, 0.05),
            ("Disarming motors", 36.24, 0.3),
            ("PreArm: GCS failsafe on", 37.93, 0.3),
            ("PreArm: Radio failsafe on", 37.93, 0.3),
            ("PreArm: GCS failsafe on", 68.93, 0.3),
            ("PreArm: Radio failsafe on", 68.93, 0.3),
        ],
    );
    let (t, _) = run.text("GCS Failsafe").expect("text");
    near(
        "time from the ground station's loss to its failsafe",
        t - 20.0,
        4.09,
        0.05,
    );
    let o = run.outcome();
    assert!(!o.armed_at_end);
    near("disarmed", o.disarmed_at_s.expect("disarmed"), 36.24, 0.3);
    near(
        "touchdown speed",
        o.touchdown_speed_mps.expect("touchdown"),
        0.53,
        0.15,
    );
    let (_, away) = end_position(&run);
    assert!(away < 1.0, "{away}");
    assert_eq!(class_of(o), Class::Safe);
    assert!(run.verdict.violations.is_empty());
}

// ---- Battery ----------------------------------------------------------------

fn battery_names() -> (f64, f64) {
    let a = airframe();
    let p = battery_failsafe_params(
        &FailsafeParams::default(),
        u8::try_from(a.battery.cells).expect("cells"),
        a.battery.internal_resistance_ohm,
        HOVER_POWER_W,
    );
    (p[0].1, p[1].1)
}

/// The volts in "Battery 1 is low 20.51V used 72 mAh".
fn volts_of(text: &str) -> f64 {
    let at = text.find("V used").expect("a voltage");
    let start = text[..at].rfind(' ').map_or(0, |i| i + 1);
    text[start..at].parse().expect("volts")
}

#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn battery_low_arms_and_returns_10_s_after_the_threshold_and_is_still_coming_down_at_70_s() {
    let (_lease, run) = run_of("battery-low");
    assert_writes(&run, &[]);
    // It armed on the nearly empty pack: no refusal.
    assert_eq!(
        run.audit().armed_changes.iter().filter(|(_, v)| *v).count(),
        1
    );
    assert_modes(&run, &["STABILIZE", "GUIDED", "RTL"]);
    assert_texts_after(
        &run,
        9.0,
        &[
            ("Battery 1 is low", 33.82, 0.05),
            ("Battery Failsafe", 33.82, 0.05),
        ],
    );
    let (t, text) = run.text("Battery 1 is low").expect("text");
    let read = volts_of(&text);
    let truth = run
        .ticks
        .iter()
        .find(|k| k.t_s >= t)
        .expect("tick")
        .battery_v;
    near(
        "the pack voltage in the text against the pack's",
        read,
        truth,
        0.1,
    );
    let (low, _) = battery_names();
    // From the pack's last crossing before the failsafe, the one that stayed
    // below the threshold: the loaded voltage dips under it earlier and
    // recovers (`first_below` is 1.18 s before this one), which the timer
    // does not count.
    let crossed = below_since(&run, low, t).expect("the pack crosses the threshold");
    println!(
        "MEASURED the first dip below BATT_LOW_VOLT {:?}, the sustained crossing {crossed}",
        first_below(&run, low)
    );
    near(
        "time from the pack's sustained crossing of BATT_LOW_VOLT to the failsafe",
        t - crossed,
        10.0,
        0.3,
    );
    near("BATT_LOW_VOLT", low, 20.767, 0.005);
    assert!(run.text("critical").is_none());
    near(
        "RTL read",
        run.mode_at_or_after("RTL", 0.0).expect("rtl"),
        t,
        1.1,
    );
    let o = run.outcome();
    near(
        "greatest height (it climbs to its return height)",
        o.max_height_m,
        14.88,
        0.3,
    );
    // It ends in the final descent, not yet down.
    assert!(o.armed_at_end && o.touchdown_speed_mps.is_none());
    let (_, away) = end_position(&run);
    assert!(away < 1.0, "back above home: {away}");
    let last = run.ticks.last().expect("ticks");
    near("height at the end", -last.position[2], 3.79, 0.3);
    assert!(last.velocity[2] > 0.1, "descending");
    assert_eq!(class_of(o), Class::Safe);
    assert_violations(&run, &["still armed", "never came down"]);
}

#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn battery_critical_arms_returns_at_low_lands_at_critical_and_is_down_at_43_s() {
    let (_lease, run) = run_of("battery-critical");
    assert_writes(&run, &[]);
    assert_eq!(
        run.audit().armed_changes.iter().filter(|(_, v)| *v).count(),
        1
    );
    assert_modes(&run, &["STABILIZE", "GUIDED", "RTL", "LAND"]);
    assert_texts_after(
        &run,
        9.0,
        &[
            ("Battery 1 is low", 21.62, 0.05),
            ("Battery Failsafe", 21.62, 0.05),
            ("Battery 1 is critical", 27.32, 0.05),
            ("Battery Failsafe", 27.32, 0.05),
            ("Disarming motors", 43.40, 0.3),
            ("PreArm: Battery 1 low voltage failsafe", 44.93, 0.3),
            ("PreArm: Battery failsafe", 44.93, 0.3),
        ],
    );
    let (low, crit) = battery_names();
    near("BATT_LOW_VOLT", low, 20.767, 0.005);
    near("BATT_CRT_VOLT", crit, 19.55, 0.01);
    let (tl, _) = run.text("Battery 1 is low").expect("low");
    let (tc, _) = run.text("Battery 1 is critical").expect("critical");
    near(
        "time from the pack crossing BATT_LOW_VOLT to the low failsafe",
        tl - first_below(&run, low).expect("low"),
        10.0,
        0.3,
    );
    near(
        "time from the pack crossing BATT_CRT_VOLT to the critical failsafe",
        tc - first_below(&run, crit).expect("crit"),
        10.0,
        0.3,
    );
    near(
        "RTL read",
        run.mode_at_or_after("RTL", 0.0).expect("rtl"),
        tl,
        1.1,
    );
    near(
        "LAND read",
        run.mode_at_or_after("LAND", 0.0).expect("land"),
        tc,
        1.1,
    );
    let o = run.outcome();
    assert!(!o.armed_at_end);
    near("disarmed", o.disarmed_at_s.expect("disarmed"), 43.40, 0.3);
    near(
        "touchdown speed",
        o.touchdown_speed_mps.expect("touchdown"),
        0.53,
        0.15,
    );
    assert_eq!(class_of(o), Class::Safe);
    assert_violations(&run, &["disarmed at 43.40 s, later than 40 s"]);
}

// ---- The brain --------------------------------------------------------------

/// What every brain failure has in common: ArduCopter does nothing, the
/// vehicle holds the last target it was sent and stays armed.
fn assert_holds_the_last_target(run: &Run, n: f64) {
    assert_modes(run, &["STABILIZE", "GUIDED"]);
    assert_texts_after(run, 9.0, &[]);
    let o = run.outcome();
    assert!(o.armed_at_end && o.touchdown_speed_mps.is_none());
    let ([x, y, h], _) = end_position(run);
    near("end north", x, n, 0.5);
    near("end east", y, 0.0, 0.5);
    near("end height", h, 5.0, 0.5);
    assert_eq!(class_of(o), Class::Safe);
}

#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn a_brain_crash_is_not_noticed_and_the_vehicle_holds_the_last_target() {
    let (_lease, run) = run_of("brain-crash");
    assert_writes(&run, &[]);
    assert_holds_the_last_target(&run, 15.0);
    // The brain's heartbeats and targets stop with the fault.
    let hb = heartbeats_as(&run, 251);
    near(
        "the last brain heartbeat sent",
        hb.last().copied().expect("hb"),
        19.0,
        1e-9,
    );
    assert!(targets_sent(&run).iter().all(|t| *t < 20.0));
    assert_violations(&run, &["still armed", "never came down", "landed 15."]);
}

#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn a_brain_hang_is_not_noticed_either_its_heartbeats_go_on_and_its_targets_stop() {
    let (_lease, run) = run_of("brain-hang");
    assert_writes(&run, &[]);
    assert_holds_the_last_target(&run, 15.0);
    let hb = heartbeats_as(&run, 251);
    near(
        "the last brain heartbeat sent",
        hb.last().copied().expect("hb"),
        89.0,
        1e-9,
    );
    assert!(targets_sent(&run).iter().all(|t| *t < 20.0));
    // 14.99 m from home on this run: the distance itself is held by
    // `end north` above, to 0.5 m.
    assert_violations(&run, &["still armed", "never came down", "landed 14.99 m"]);
}

#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn a_setpoint_gap_is_not_noticed_and_the_target_sent_after_it_is_the_same() {
    let (_lease, run) = run_of("setpoint-gap");
    assert_writes(&run, &[]);
    assert_holds_the_last_target(&run, 0.0);
    // No target while the brain is silent (15 to 18 s), the first at 18 s.
    let sent = targets_sent(&run);
    let during: Vec<&f64> = sent.iter().filter(|t| **t > 15.0 && **t < 18.0).collect();
    assert!(during.is_empty(), "{during:?}");
    assert!(sent.iter().any(|t| (*t - 18.0).abs() < 1e-9), "{sent:?}");
    // Its heartbeats went on.
    assert!(heartbeats_as(&run, 251).last().copied().expect("hb") > 38.0);
    assert!(run.verdict.violations.is_empty());
}

/// A target that is far away is followed to its end after the brain goes
/// quiet: ArduCopter does not stop following a position target after a few
/// seconds (the brain's last target was sent 0.5 s before the failure).
fn far_target_run(name: &str) -> Run {
    let lease = Instance::lease();
    let run = fly_run_with(name, 0, lease.0, "far", |f| {
        f.duration_s = 60.0;
        f.pilot.duration_s = 60.0;
        f.pilot.setpoints.push((
            19.5,
            Some(TrajectorySetpoint {
                position: Vec3::new(45.0, 0.0, -5.0),
                velocity: Vec3::zero(),
                yaw: 0.0,
            }),
        ));
    });
    print_run(&run);
    run
}

#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn a_target_the_brain_sent_just_before_it_failed_is_followed_to_its_end_and_held() {
    for name in ["brain-crash", "brain-hang"] {
        let run = far_target_run(name);
        // The leg is 30 m at about 5 m/s: far longer than 3 s.
        let reached = run
            .ticks
            .iter()
            .find(|k| k.t_s > 19.5 && (k.position[0] - 45.0).abs() < 1.0)
            .map(|k| k.t_s)
            .expect("the target is reached");
        near(&format!("{name}: time to reach 45 m"), reached, 25.5, 1.5);
        let ([x, y, h], _) = end_position(&run);
        near(&format!("{name}: end north"), x, 45.0, 0.5);
        near(&format!("{name}: end east"), y, 0.0, 0.5);
        near(&format!("{name}: end height"), h, 5.0, 0.5);
        assert!(
            run.between(reached, 1e9)
                .all(|k| (k.position[0] - 45.0).abs() < 1.0),
            "{name}: it holds there"
        );
        assert_modes(&run, &["STABILIZE", "GUIDED"]);
        assert_texts_after(&run, 9.0, &[]);
        assert_eq!(targets_sent(&run).last().copied(), Some(19.5), "{name}");
    }
}

// ---- The IMU's last sample --------------------------------------------------

/// With an IMU fault our simulation's frame has no IMU sample, so the bridge's
/// reply carries the last one it latched, and the program is told the two IMUs
/// are lost by two parameter writes sent at the tick of the fault.
///
/// This test first held that the program then does not use the sample at all:
/// the same flight with fresh samples in every reply, and the same two writes
/// sent by hand at the same tick, had the same motor packets and the same
/// physics, bit for bit. That held on the starting gains and does not hold on
/// the flown set (base, setup, tuned gains, failsafes): the two flights are
/// the same up to 15.001 s and differ from 15.0015 s, three steps after the
/// fault and before the program's echo of the masks was read (15.002 s). It
/// holds with `INS_GYRO_FILTER` 20, 21 and 40 and with `ATC_RAT_YAW_FLTE` 2.5
/// and 3 on the starting gains and fails with `INS_GYRO_FILTER` 80 or
/// `ATC_RAT_YAW_FLTE` 2 added to them, which is not a monotone dependence: a
/// flip of one microsecond in a motor pulse in the steps before the masks are
/// applied would be the same, and is a property of the quantised output, not
/// of the bridge. Whether the sample still matters after the masks are applied
/// is not isolated by a whole-flight comparison. So the test pins what was
/// measured, deterministically (the flights repeat bit for bit): equal up to
/// 15.001 s, different at 15.0015 s and at the end.
fn imu_loss_flight_to(lease: &Instance, end_s: Option<f64>, fresh_samples: bool) -> u64 {
    let file = load_path(scenario_path("imu-loss")).expect("file");
    let af = airframe();
    let mut options = file.options(&af, 0);
    if fresh_samples {
        options.params.clear();
        options.faults.clear();
    }
    let root = TempDir::new("imu-window");
    let mut c = ArduCopterConfig::new(binary(), root.path().join("flights"), lease.0);
    configure(&mut c, &af);
    let options = bridged_options(options, &mut c);
    let ap = ArduCopter::start(c).unwrap_or_else(|e| panic!("{e}"));
    let mut sim = Simulation::with_autopilot(&af, ap, file.pilot.clone(), options).expect("sim");
    sim.preroll(PREROLL_STEPS).unwrap_or_else(|e| panic!("{e}"));
    let mut written = false;
    let end_us = end_s.map_or(file.end().0, s_us);
    while sim.time().0 < end_us {
        if fresh_samples && !written && sim.time().0 >= 15_000_000 {
            for n in ["SIM_ACC_FAIL_MSK", "SIM_GYR_FAIL_MSK"] {
                sim.autopilot().instrument().param_set(n, 3.0).expect("set");
            }
            written = true;
        }
        sim.try_step().unwrap_or_else(|e| panic!("{e}"));
    }
    let trace = sim.trace_hash();
    sim.into_autopilot()
        .finish()
        .unwrap_or_else(|e| panic!("{e}"));
    trace
}

#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn the_latched_imu_sample_reaches_the_flight_within_three_steps_of_the_fault_on_the_flown_set() {
    let lease = Instance::lease();
    for (end_s, same) in [(Some(15.001), true), (Some(15.0015), false), (None, false)] {
        let latched = imu_loss_flight_to(&lease, end_s, false);
        let fresh = imu_loss_flight_to(&lease, end_s, true);
        println!(
            "MEASURED latched IMU sample against fresh samples, to {end_s:?} s: trace \
             {latched:016x} against {fresh:016x}"
        );
        assert_eq!(latched == fresh, same, "to {end_s:?} s");
    }
    // The program's echo of the masks was read 2 ms after the writes.
    let a = fly_run("imu-loss", 0, lease.0, "imu-echo");
    let echoes = a.audit().param_echoes();
    let first = echoes
        .iter()
        .find(|(_, n, v)| *n == "SIM_GYR_FAIL_MSK" && *v == 3.0)
        .expect("echo of the mask");
    near(
        "the echo of the masks, read at",
        a.flight_s(first.0),
        15.002,
        0.0006,
    );
}

// ---- A faulted flight repeats -----------------------------------------------

fn repeat_key(r: &Run) -> (u64, u64, u64, String) {
    (
        r.flown.trace_hash,
        r.audit().packet_hash,
        r.audit().reply_hash,
        r.audit().params_sha256.clone(),
    )
}

fn assert_repeats(name: &str) {
    let (la, lb) = (Instance::lease(), Instance::lease());
    let (ia, ib) = (la.0, lb.0);
    let (a, b) = std::thread::scope(|s| {
        let a = s.spawn(|| fly_run(name, 0, ia, "repeat-a"));
        let b = s.spawn(|| fly_run(name, 0, ib, "repeat-b"));
        (a.join().expect("flight a"), b.join().expect("flight b"))
    });
    println!(
        "MEASURED {name} on instances {ia} and {ib}:\n a {:?}\n b {:?}",
        repeat_key(&a),
        repeat_key(&b)
    );
    assert_eq!(repeat_key(&a), repeat_key(&b));
    let writes = |r: &Run| -> Vec<(u64, String, f32)> {
        r.audit()
            .param_writes()
            .into_iter()
            .map(|(t, n, v)| (t, n.to_string(), v))
            .collect()
    };
    assert_eq!(writes(&a), writes(&b));
    let sent = |r: &Run| r.audit().sent.clone();
    assert_eq!(
        sent(&a),
        sent(&b),
        "everything sent, in order and by the step"
    );
}

#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn the_gps_loss_flight_repeats_bit_for_bit_on_two_instances() {
    assert_repeats("gps-loss");
}

#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn the_battery_low_flight_repeats_bit_for_bit_on_two_instances() {
    assert_repeats("battery-low");
}

/// The runner flies a faulted file as this file's glue does: the same trace,
/// packet and reply hashes, the same parameter hash, the same writes.
fn assert_runner_agrees(name: &str) {
    let (la, lb) = (Instance::lease(), Instance::lease());
    let (ia, ib) = (la.0, lb.0);
    let (glue, runner) = std::thread::scope(|s| {
        let a = s.spawn(|| fly_run(name, 0, ia, "glue"));
        let b = s.spawn(|| {
            let root = TempDir::new("runner");
            let src = oxide_bridge::scenario::LoadedScenario::load(scenario_path(name))
                .unwrap_or_else(|e| panic!("{e}"));
            let setup = oxide_bridge::runner::Setup::new(binary(), root.path().join("f"), ib);
            oxide_bridge::runner::fly_scenario(&src, 0, &setup).unwrap_or_else(|e| panic!("{e}"))
        });
        (a.join().expect("glue"), b.join().expect("runner"))
    });
    let audit = runner
        .audit
        .as_ref()
        .expect("a bridged flight has an audit");
    println!(
        "MEASURED {name}: glue trace {:016x} packet {:016x} reply {:016x}; runner trace {:016x} packet {:016x} reply {:016x}",
        glue.flown.trace_hash,
        glue.audit().packet_hash,
        glue.audit().reply_hash,
        runner.trace_hash,
        audit.packet_hash,
        audit.reply_hash
    );
    assert_eq!(runner.trace_hash, glue.flown.trace_hash);
    assert_eq!(audit.packet_hash, glue.audit().packet_hash);
    assert_eq!(audit.reply_hash, glue.audit().reply_hash);
    assert_eq!(audit.params_sha256, glue.audit().params_sha256);
    assert_eq!(audit.param_writes(), glue.audit().param_writes());
    assert!(!audit.param_writes().is_empty(), "the faults were written");
}

#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn the_runner_flies_gps_loss_as_the_glue_of_this_file_does() {
    assert_runner_agrees("gps-loss");
}

// ---- What needs no write, without the binary --------------------------------

/// An autopilot that flies nothing and records what the bridge would be shown
/// at every tick: the radio channels as the bridge's latch would send them
/// and which heartbeats, setpoints and rotors the faults leave.
struct Probe {
    latch: RcLatch,
    ticks: Vec<ProbeTick>,
}

#[derive(Clone, Copy, Debug)]
struct ProbeTick {
    t_us: u64,
    rc: Option<[u16; 8]>,
    gcs_heartbeat: bool,
    brain_heartbeat: bool,
    setpoint: bool,
    rotor_scale: [f64; 4],
}

#[derive(Debug)]
struct Never;

impl core::fmt::Display for Never {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str("never")
    }
}

impl std::error::Error for Never {}

impl Autopilot<4> for Probe {
    type Error = Never;

    fn period_us(&self) -> u64 {
        STEP_US
    }

    fn tick(
        &mut self,
        t: Micros,
        input: &AutopilotInput<'_, 4>,
    ) -> Result<AutopilotOutput<4>, Never> {
        self.ticks.push(ProbeTick {
            t_us: t.0,
            rc: self.latch.at(t, &input.rc),
            gcs_heartbeat: input.gcs_heartbeat.is_some(),
            brain_heartbeat: input.brain_heartbeat.is_some(),
            setpoint: input.setpoint.is_some(),
            rotor_scale: input.rotor_scale,
        });
        Ok(AutopilotOutput {
            t_valid: t,
            motors: [0.0; 4],
            armed: false,
            core: None,
        })
    }
}

fn probe(name: &str, edit: impl FnOnce(&mut ScenarioFile)) -> Vec<ProbeTick> {
    let mut file = load_path(scenario_path(name)).unwrap_or_else(|e| panic!("{e}"));
    edit(&mut file);
    let af = airframe();
    let mut options = file.options(&af, 0);
    options.params.clear();
    let p = Probe {
        latch: RcLatch::new(),
        ticks: Vec::new(),
    };
    let mut sim = Simulation::with_autopilot(&af, p, file.pilot.clone(), options).expect("sim");
    sim.try_run_until(Micros(30_000_000), |_| {})
        .expect("the probe never fails");
    sim.into_autopilot().ticks
}

fn at(ticks: &[ProbeTick], t_us: u64) -> &ProbeTick {
    ticks.iter().find(|k| k.t_us == t_us).expect("the tick")
}

fn first_where(ticks: &[ProbeTick], from_us: u64, f: impl Fn(&ProbeTick) -> bool) -> Option<u64> {
    ticks
        .iter()
        .find(|k| k.t_us >= from_us && f(k))
        .map(|k| k.t_us)
}

#[test]
fn a_lost_radio_takes_the_channels_out_of_the_reply_whether_the_receiver_reports_failsafe_or_falls_silent()
 {
    // A receiver that reports the loss (frame lost at once, failsafe 1 s
    // later): the channels are gone from the fault's own tick.
    let reports = probe("rc-loss-offboard-gcs", |_| {});
    assert!(at(&reports, 14_990_000).rc.is_some());
    assert!(at(&reports, 14_999_500).rc.is_some());
    let gone = first_where(&reports, 14_000_000, |k| k.rc.is_none()).expect("gone");
    println!("MEASURED receiver that reports the loss: channels gone from {gone} us");
    assert_eq!(gone, 15_000_000);
    assert!(
        reports
            .iter()
            .filter(|k| k.t_us >= 15_000_000)
            .all(|k| k.rc.is_none())
    );

    // A receiver that falls silent: the last frame stands for one radio
    // period (20 ms), then the channels are gone.
    let silent = probe("rc-loss-offboard-gcs", |f| {
        for fault in &mut f.faults {
            if fault.kind == FaultKind::RcLoss {
                fault.kind = FaultKind::RcSilent;
            }
        }
    });
    assert!(at(&silent, 14_990_000).rc.is_some());
    let gone = first_where(&silent, 14_000_000, |k| k.rc.is_none()).expect("gone");
    println!("MEASURED receiver that falls silent: channels gone from {gone} us");
    assert!(
        (15_000_000..=15_020_500).contains(&gone),
        "gone from {gone}"
    );
    assert!(
        silent
            .iter()
            .filter(|k| k.t_us >= 15_021_000)
            .all(|k| k.rc.is_none())
    );
}

#[test]
fn the_other_faults_leave_the_reply_and_the_links_as_the_text_says() {
    // Ground station lost: its heartbeats stop at the fault (the last at 14 s).
    let gcs = probe("gcs-loss", |_| {});
    assert!(at(&gcs, 14_000_000).gcs_heartbeat);
    assert!(
        gcs.iter()
            .filter(|k| k.t_us >= 15_000_000)
            .all(|k| !k.gcs_heartbeat)
    );
    // The brain stays: heartbeats and setpoints.
    assert!(
        gcs.iter()
            .filter(|k| k.t_us >= 15_000_000)
            .any(|k| k.brain_heartbeat)
    );
    assert!(at(&gcs, 20_000_000).setpoint || at(&gcs, 20_020_000).setpoint);

    // Brain crash: heartbeats and setpoints stop at the fault.
    let crash = probe("brain-crash", |_| {});
    assert!(
        crash
            .iter()
            .filter(|k| k.t_us < 20_000_000 && k.t_us > 10_000_000)
            .any(|k| k.brain_heartbeat)
    );
    assert!(
        crash
            .iter()
            .filter(|k| k.t_us >= 20_000_000)
            .all(|k| !k.brain_heartbeat && !k.setpoint)
    );

    // Brain hang: heartbeats go on, setpoints stop.
    let hang = probe("brain-hang", |_| {});
    assert!(
        hang.iter()
            .filter(|k| k.t_us >= 20_000_000)
            .any(|k| k.brain_heartbeat)
    );
    assert!(
        hang.iter()
            .filter(|k| k.t_us >= 20_000_000)
            .all(|k| !k.setpoint)
    );
    assert!(
        hang.iter()
            .filter(|k| k.t_us > 10_000_000 && k.t_us < 20_000_000)
            .any(|k| k.setpoint)
    );

    // Setpoint gap: setpoints stop from 15 s to 18 s, heartbeats go on.
    let gap = probe("setpoint-gap", |_| {});
    assert!(
        gap.iter()
            .filter(|k| k.t_us >= 15_000_000 && k.t_us < 18_000_000)
            .all(|k| !k.setpoint)
    );
    assert!(
        gap.iter()
            .filter(|k| k.t_us >= 15_000_000 && k.t_us < 18_000_000)
            .any(|k| k.brain_heartbeat)
    );
    assert!(
        gap.iter()
            .filter(|k| k.t_us >= 18_000_000)
            .any(|k| k.setpoint)
    );

    // Motor lost: the physics gives rotor 0 no thrust from the fault, and the
    // other three all of it.
    let motor = probe("motor-loss", |_| {});
    assert_eq!(at(&motor, 14_999_500).rotor_scale, [1.0; 4]);
    assert_eq!(at(&motor, 15_000_000).rotor_scale, [0.0, 1.0, 1.0, 1.0]);
    assert!(
        motor
            .iter()
            .filter(|k| k.t_us >= 15_000_000)
            .all(|k| k.rotor_scale == [0.0, 1.0, 1.0, 1.0])
    );
}
