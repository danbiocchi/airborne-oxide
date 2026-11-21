// SPDX-License-Identifier: AGPL-3.0-or-later
//! Stick flying under ArduCopter (ADR-0006, ADR-0007): the four sticks proven
//! against our own core, the pilot's script delayed for ArduCopter's arming
//! delay, and the five scenario files whose pilot flies a mode other than
//! offboard, each flown on the real program and pinned.
//!
//! Every test here needs the program and is ignored by default: set
//! `OXIDE_ARDUCOPTER` to its path and run with `--ignored` and `--release`.
//! Instance numbers come from the pool of `tests/support/pool.rs`.

use std::path::{Path, PathBuf};
use std::process::Command;

use oxide_bridge::arducopter::{
    ArduCopter, ArduCopterConfig, FlightAudit, PREROLL_STEPS, SentKind, bridged_options, fly,
};
use oxide_bridge::mav::MavEvent;
use oxide_bridge::record::read_row;
use oxide_bridge::runner::{ScenarioFlight, Setup, arducopter_params, fly_scenario};
use oxide_bridge::scenario::{ARM_DELAY_S, LoadedScenario, translate};
use oxide_bridge::sitl::PINNED;
use oxide_core::math::Real;
use oxide_core::time::Micros;
use oxide_sim::outcome::FlightClass;
use oxide_sim::scenario::NoPilot;
use oxide_sim::scenario_file::RunProfile;
use oxide_sim::{Airframe, Simulation, StepRecord};

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
            "oxide-arducopter-sticks-{}-{tag}",
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

/// A calm Position flight in the style of the stick files: arm at 3 s, the
/// throttle full at 3.5 s and half at 5.2 s, then `stick` (roll, pitch, yaw,
/// throttle) from 8 s to 13 s, centred after.
fn proof_text(stick: [f32; 4], end_s: f64) -> String {
    let [roll, pitch, yaw, throttle] = stick;
    format!(
        "name = \"stick-proof\"\nduration_s = {end_s}\nairframe = \"quad-x-5in\"\n\
         [sim]\nsensors = \"realistic\"\nestimator = \"ekf\"\n\
         [sim.links]\ngcs = false\nbrain = true\n\
         [pilot]\narm_at_s = 3.0\n\
         [[pilot.mode]]\nt = 0.0\nswitch = \"position\"\n\
         [[pilot.sticks]]\nt = 0.0\nroll = 0.0\npitch = 0.0\nyaw = 0.0\nthrottle = 0.0\n\
         [[pilot.sticks]]\nt = 3.5\nroll = 0.0\npitch = 0.0\nyaw = 0.0\nthrottle = 1.0\n\
         [[pilot.sticks]]\nt = 5.2\nroll = 0.0\npitch = 0.0\nyaw = 0.0\nthrottle = 0.5\n\
         [[pilot.sticks]]\nt = 8.0\nroll = {roll}\npitch = {pitch}\nyaw = {yaw}\nthrottle = {throttle}\n\
         [[pilot.sticks]]\nt = 13.0\nroll = 0.0\npitch = 0.0\nyaw = 0.0\nthrottle = 0.5\n\
         [[pilot.setpoint]]\nt = 0.0\nposition = [0.0, 0.0, 0.0]\nyaw = 0.0\n"
    )
}

fn yaw_of(r: &StepRecord<4>) -> f64 {
    let q = r.attitude;
    Real::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )
}

/// Flies the text on our core and returns the records.
fn fly_ours(text: &str) -> Vec<StepRecord<4>> {
    let src = LoadedScenario::from_text(text).expect("the scenario");
    let mut sim = src
        .file
        .build_with_params(RunProfile::F405.profile(), 0, &[])
        .expect("the simulation");
    let mut log = Vec::new();
    sim.run_until(src.file.end(), |r| log.push(*r));
    log
}

/// Flies the text on ArduCopter with the translated script and the
/// committed parameters, then `extra` (parameters written after them).
fn fly_theirs(
    text: &str,
    instance: u8,
    tag: &str,
    extra: &[(&str, f64)],
) -> (Vec<StepRecord<4>>, FlightAudit) {
    fly_theirs_with(text, instance, tag, extra, false)
}

/// [`fly_theirs`], optionally with the file's own pilot, not delayed.
fn fly_theirs_with(
    text: &str,
    instance: u8,
    tag: &str,
    extra: &[(&str, f64)],
    undelayed: bool,
) -> (Vec<StepRecord<4>>, FlightAudit) {
    let root = TempDir::new(tag);
    let src = LoadedScenario::from_text(text).expect("the scenario");
    let t = translate(&src.file).expect("translatable");
    let af = Airframe::reference_quad().expect("airframe");
    let setup = Setup::new(binary(), root.path().join("flights"), instance);
    let (files, mut extras) = arducopter_params(&t, &setup, &af);
    extras.extend(extra.iter().map(|(n, v)| ((*n).to_string(), *v)));
    let mut c = ArduCopterConfig::new(binary(), root.path().join("flights"), instance);
    c.param_files = files;
    c.extra_params = extras;
    c.brain = t.brain;
    let mut options = src.file.options(&af, 0);
    options.params.clear();
    let mut log = Vec::new();
    let pilot = if undelayed {
        src.file.pilot.clone()
    } else {
        t.pilot
    };
    let flown = fly(&af, pilot, options, c, src.file.end(), |r| log.push(*r))
        .unwrap_or_else(|e| panic!("{e}"));
    (log, flown.audit)
}

/// The record at or after flight time `t_s`.
fn at(log: &[StepRecord<4>], t_s: f64) -> &StepRecord<4> {
    log.iter()
        .find(|r| r.t >= Micros((t_s * 1e6).round() as u64))
        .expect("a record")
}

/// The heading change between two flight times, degrees, summed over the
/// records in between so that it does not wrap.
fn turned_deg(log: &[StepRecord<4>], from_s: f64, to_s: f64) -> f64 {
    let mut total = 0.0;
    let mut last: Option<f64> = None;
    for r in log
        .iter()
        .filter(|r| r.t.as_secs_f64() >= from_s && r.t.as_secs_f64() <= to_s)
        .step_by(20)
    {
        let y = yaw_of(r);
        if let Some(l) = last {
            let mut d = y - l;
            while d > core::f64::consts::PI {
                d -= 2.0 * core::f64::consts::PI;
            }
            while d < -core::f64::consts::PI {
                d += 2.0 * core::f64::consts::PI;
            }
            total += d;
        }
        last = Some(y);
    }
    total.to_degrees()
}

/// What a stick did in the nine seconds from its start: north, east and up
/// in metres, and the heading change in degrees.
#[derive(Clone, Copy, Debug)]
struct Moved {
    north: f64,
    east: f64,
    up: f64,
    turned: f64,
}

/// The stick of the proof flights starts at 8 s for ours and 2.24 s later for
/// ArduCopter (the pilot waits for its motors).
const STICK_AT_S: f64 = 8.0;
const WINDOW_S: f64 = 9.0;

fn moved(log: &[StepRecord<4>], start_s: f64) -> Moved {
    let (a, b) = (at(log, start_s), at(log, start_s + WINDOW_S));
    Moved {
        north: b.position.x - a.position.x,
        east: b.position.y - a.position.y,
        up: -(b.position.z - a.position.z),
        turned: turned_deg(log, start_s, start_s + WINDOW_S),
    }
}

const PROOF_END_S: f64 = 22.0;

/// The four sticks, one at a time (and the throttle both ways).
const PROOFS: [(&str, [f32; 4]); 5] = [
    ("roll", [0.5, 0.0, 0.0, 0.5]),
    ("pitch", [0.0, 0.5, 0.0, 0.5]),
    ("yaw", [0.0, 0.0, 0.5, 0.5]),
    ("throttle-up", [0.0, 0.0, 0.0, 0.8]),
    ("throttle-down", [0.0, 0.0, 0.0, 0.2]),
];

fn stick_of(name: &str) -> [f32; 4] {
    PROOFS
        .iter()
        .find(|(n, _)| *n == name)
        .map(|(_, s)| *s)
        .expect("a proof")
}

/// Flies one proof stick on both and returns what each moved.
fn proof(name: &str, instance: u8, extra: &[(&str, f64)]) -> (Moved, Moved) {
    let text = proof_text(stick_of(name), PROOF_END_S);
    let ours = moved(&fly_ours(&text), STICK_AT_S);
    let (log, _) = fly_theirs(&text, instance, name, extra);
    let theirs = moved(&log, STICK_AT_S + ARM_DELAY_S);
    println!("MEASURED stick {name}: ours {ours:?}");
    println!("MEASURED stick {name}: ArduCopter {theirs:?} (extra {extra:?})");
    (ours, theirs)
}

/// Sign and size of a movement, with a tolerance: the flights repeat bit
/// for bit, the bands only say what a change would have to be to be noticed.
fn near(what: &str, got: f64, want: f64, tol: f64) {
    assert!(
        (got - want).abs() <= tol,
        "{what}: {got:.3} against {want} +- {tol}"
    );
}

/// Test 1. Each stick moves ArduCopter and our core the same way.
///
/// Roll +0.5 (right): east. Pitch +0.5 (forward): north. Yaw +0.5 (right):
/// the heading up. Throttle 0.8 above the centre: up; 0.2 below it: down.
/// ArduCopter's pitch channel reads a forward stick as a low pulse, ours is
/// high, so `RC2_REVERSED 1` of `base.parm` makes them agree: the flight
/// without it goes the other way (the second half of the test). The open-loop
/// responses differ in size and are reported, not compared.
#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn each_stick_moves_arducopter_and_our_core_the_same_way() {
    let lease = Instance::lease();
    let (ours, theirs) = proof("roll", lease.0, &[]);
    assert!(ours.east > 5.0 && theirs.east > 5.0, "{ours:?} {theirs:?}");
    near("roll ours east", ours.east, 11.98, 0.5);
    near("roll ArduCopter east", theirs.east, 19.78, 0.5);
    assert!(ours.north.abs() < 1.0 && theirs.north.abs() < 1.0);
    let (ours, theirs) = proof("pitch", lease.0, &[]);
    assert!(
        ours.north > 5.0 && theirs.north > 5.0,
        "{ours:?} {theirs:?}"
    );
    near("pitch ours north", ours.north, 11.86, 0.5);
    near("pitch ArduCopter north", theirs.north, 19.88, 0.5);
    assert!(ours.east.abs() < 1.0 && theirs.east.abs() < 1.0);
    let (ours, theirs) = proof("yaw", lease.0, &[]);
    assert!(
        ours.turned > 90.0 && theirs.turned > 90.0,
        "{ours:?} {theirs:?}"
    );
    near("yaw ours turned", ours.turned, 271.6, 5.0);
    near("yaw ArduCopter turned", theirs.turned, 486.6, 5.0);
    let (ours, theirs) = proof("throttle-up", lease.0, &[]);
    assert!(ours.up > 3.0 && theirs.up > 3.0, "{ours:?} {theirs:?}");
    near("throttle up ours", ours.up, 8.49, 0.5);
    near("throttle up ArduCopter", theirs.up, 6.20, 0.5);
    let (ours, theirs) = proof("throttle-down", lease.0, &[]);
    assert!(ours.up < -2.0 && theirs.up < -2.0, "{ours:?} {theirs:?}");
    near("throttle down ours", ours.up, -4.49, 0.5);
    near("throttle down ArduCopter", theirs.up, -3.33, 0.5);
    // The pitch channel as the program has it by default flies south.
    let (_, unreversed) = proof("pitch", lease.0, &[("RC2_REVERSED", 0.0)]);
    assert!(unreversed.north < -5.0, "{unreversed:?}");
    near("pitch unreversed north", unreversed.north, -19.59, 0.5);
}

/// Reads parameters from a program started with the committed files: asked
/// in groups of six with a second between.
fn read_params(instance: u8, tag: &str, names: &[&str]) -> Vec<(String, f32)> {
    let root = TempDir::new(tag);
    let af = Airframe::reference_quad().expect("airframe");
    let src = LoadedScenario::load(
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../scenarios/hover-step.toml"),
    )
    .expect("hover-step");
    let t = translate(&src.file).expect("translatable");
    let setup = Setup::new(binary(), root.path().join("flights"), instance);
    let (files, extras) = arducopter_params(&t, &setup, &af);
    let mut c = ArduCopterConfig::new(binary(), root.path().join("flights"), instance);
    c.param_files = files;
    c.extra_params = extras;
    c.flight_name = Some("sitl".into());
    let options = bridged_options(src.file.options(&af, 0), &mut c);
    let ap = ArduCopter::start(c).unwrap_or_else(|e| panic!("{e}"));
    let mut sim = Simulation::with_autopilot(&af, ap, NoPilot, options).expect("the simulation");
    sim.preroll(PREROLL_STEPS).unwrap_or_else(|e| panic!("{e}"));
    for group in names.chunks(6) {
        for name in group {
            sim.autopilot()
                .instrument()
                .param_request(name)
                .expect("param_request");
        }
        for _ in 0..2_000 {
            sim.try_step().unwrap_or_else(|e| panic!("{e}"));
        }
    }
    let audit = sim
        .into_autopilot()
        .finish()
        .unwrap_or_else(|e| panic!("{e}"));
    names
        .iter()
        .map(|n| {
            let v = audit
                .heard
                .iter()
                .find_map(|h| match &h.event {
                    MavEvent::ParamValue { name, value, .. } if name == n => Some(*value),
                    _ => None,
                })
                .unwrap_or_else(|| panic!("no value for {n} was read"));
            ((*n).to_string(), v)
        })
        .collect()
}

/// Test 2. The radio's set-up is in the program: the sticks' range is ours
/// (1000 to 2000 us, trim 1500), the dead zones are the defaults and the
/// pitch channel is reversed.
#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn the_radios_set_up_is_in_the_program_and_a_centred_stick_is_in_the_dead_zone() {
    let lease = Instance::lease();
    let names: Vec<String> = (1..=4)
        .flat_map(|c| {
            ["MIN", "MAX", "TRIM", "DZ", "REVERSED"]
                .iter()
                .map(move |p| format!("RC{c}_{p}"))
        })
        .collect();
    let refs: Vec<&str> = names.iter().map(String::as_str).collect();
    let got = read_params(lease.0, "radio", &refs);
    println!("MEASURED the radio set-up: {got:?}");
    let value = |n: &str| got.iter().find(|(m, _)| m == n).expect("read").1;
    for c in 1..=4 {
        assert_eq!(value(&format!("RC{c}_MIN")), 1000.0, "RC{c}_MIN");
        assert_eq!(value(&format!("RC{c}_MAX")), 2000.0, "RC{c}_MAX");
        assert_eq!(value(&format!("RC{c}_TRIM")), 1500.0, "RC{c}_TRIM");
        // A centred stick of ours is 1500 us: inside the dead zone, which is
        // symmetric about the trim.
        let dz = value(&format!("RC{c}_DZ"));
        assert!(dz >= 20.0, "RC{c}_DZ {dz}");
    }
    assert_eq!(value("RC3_DZ"), 30.0);
    for (c, rev) in [(1, 0.0), (2, 1.0), (3, 0.0), (4, 0.0)] {
        assert_eq!(value(&format!("RC{c}_REVERSED")), rev, "RC{c}_REVERSED");
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

/// The flight time of a program time, s.
fn flight_s(a: &FlightAudit, program_us: u64) -> f64 {
    (program_us as f64 - (a.preroll_steps * a.step_us) as f64) * 1e-6
}

/// The vehicle's mode changes from the flight's start on: flight time and
/// ArduCopter's mode number (0 STABILIZE, 4 GUIDED, 5 LOITER, 6 RTL, 9 LAND).
/// Read from the heartbeat, so the time is that of the step that read it, not
/// repeatable to the step.
fn modes_of(a: &FlightAudit) -> Vec<(f64, u32)> {
    a.mode_changes
        .iter()
        .map(|(t, m)| (flight_s(a, *t), *m))
        .filter(|(t, _)| *t >= 0.0)
        .collect()
}

/// The status texts from the flight's start on, each once (the program tells
/// two links, and the two estimator cores each tell theirs: a text equal to
/// the one kept within 50 ms is dropped).
fn texts_of(a: &FlightAudit) -> Vec<(f64, String)> {
    let mut out: Vec<(f64, String)> = Vec::new();
    for (t, _, x) in &a.status_texts {
        let t = flight_s(a, *t);
        if t < 0.0 || out.iter().any(|(u, y)| y == x && (t - u).abs() < 0.05) {
            continue;
        }
        out.push((t, x.clone()));
    }
    out
}

/// What the program does and when is read from its heartbeat and status
/// texts, whose times depend on the step that read them: 0.25 s.
const READ_TOL_S: f64 = 0.25;

/// What a stick file's run 0 must show.
struct Pin<'a> {
    class: FlightClass,
    violations: &'a [&'a str],
    not_judged: &'a [&'a str],
    /// The last motor command above zero, flight time, s; `None` for a
    /// flight whose motors never left zero.
    last_motor_on_s: Option<f64>,
    modes: &'a [(f64, u32)],
    texts: &'a [(f64, &'a str)],
    armed_at_end: bool,
    disarmed_at_s: Option<f64>,
    touchdown_mps: Option<f64>,
    max_height_m: f64,
    max_distance_m: f64,
    max_tilt_deg: f64,
    /// North, east, up at the end, m.
    end_m: [f64; 3],
}

fn near_opt(what: &str, got: Option<f64>, want: Option<f64>, tol: f64) {
    match (got, want) {
        (Some(g), Some(w)) => near(what, g, w, tol),
        (None, None) => {}
        _ => panic!("{what}: {got:?} against {want:?}"),
    }
}

fn check(f: &ScenarioFlight, pin: &Pin<'_>) {
    let a = f.audit.as_ref().expect("audit");
    let o = &f.verdict.outcome;
    let name = &f.scenario;
    assert_eq!(f.class, pin.class, "{name}: class");
    let violations: Vec<String> = f.verdict.violations.clone();
    assert_eq!(violations, pin.violations, "{name}: violations");
    let not_judged: Vec<String> = f.verdict.not_judged.iter().map(|n| n.to_string()).collect();
    assert_eq!(not_judged, pin.not_judged, "{name}: not judged");
    near_opt(
        &format!("{name}: last motor on"),
        f.last_motor_on_s,
        pin.last_motor_on_s,
        0.003,
    );
    let modes = modes_of(a);
    assert_eq!(
        modes.iter().map(|m| m.1).collect::<Vec<_>>(),
        pin.modes.iter().map(|m| m.1).collect::<Vec<_>>(),
        "{name}: mode sequence {modes:?}"
    );
    for (got, want) in modes.iter().zip(pin.modes) {
        near(
            &format!("{name}: mode {} at", want.1),
            got.0,
            want.0,
            READ_TOL_S,
        );
    }
    let texts = texts_of(a);
    assert_eq!(
        texts.iter().map(|t| t.1.as_str()).collect::<Vec<_>>(),
        pin.texts.iter().map(|t| t.1).collect::<Vec<_>>(),
        "{name}: status texts {texts:?}"
    );
    for (got, want) in texts.iter().zip(pin.texts) {
        near(
            &format!("{name}: {:?} at", want.1),
            got.0,
            want.0,
            READ_TOL_S,
        );
    }
    assert_eq!(o.armed_at_end, pin.armed_at_end, "{name}: armed at end");
    near_opt(
        &format!("{name}: disarmed at"),
        o.disarmed_at_s,
        pin.disarmed_at_s,
        0.01,
    );
    near_opt(
        &format!("{name}: touchdown speed"),
        o.touchdown_speed_mps,
        pin.touchdown_mps,
        0.05,
    );
    near(
        &format!("{name}: max height"),
        o.max_height_m,
        pin.max_height_m,
        0.05,
    );
    near(
        &format!("{name}: max distance"),
        o.max_distance_from_home_m,
        pin.max_distance_m,
        0.05,
    );
    near(
        &format!("{name}: max tilt"),
        o.max_tilt_deg,
        pin.max_tilt_deg,
        0.2,
    );
    let end = [
        o.position_at_end.x,
        o.position_at_end.y,
        -o.position_at_end.z,
    ];
    for (axis, (g, w)) in ["north", "east", "up"]
        .iter()
        .zip(end.iter().zip(pin.end_m))
    {
        near(&format!("{name}: end {axis}"), *g, w, 0.05);
    }
    assert_eq!(a.stamp_breaks, 0, "{name}: the program's clock jumped");
    assert_eq!(o.motor_ticks_while_disarmed, 0, "{name}");
    assert!(o.physics_faults == 0 && !o.non_finite, "{name}");
}

fn fly_file(name: &str, instance: u8, tag: &str) -> ScenarioFlight {
    let root = TempDir::new(tag);
    let setup = Setup::new(binary(), root.path().join("flight"), instance);
    let f = fly_scenario(&scenario(name), 0, &setup).unwrap_or_else(|e| panic!("{e}"));
    describe(&f);
    f
}

fn describe(f: &ScenarioFlight) {
    let a = f.audit.as_ref().expect("audit");
    let o = &f.verdict.outcome;
    println!(
        "MEASURED {} {}: class {}, {} violations, {} not judged, last motor on {:?}, wall {:.1} s",
        f.scenario,
        f.index,
        f.class,
        f.verdict.violations.len(),
        f.verdict.not_judged.len(),
        f.last_motor_on_s,
        f.wall_s
    );
    println!(
        "MEASURED {} outcome: armed_at_end {} end {:.3?} at {:.4} s, max height {:.3}, max dist {:.3}, \
         max tilt {:.2}, touchdown {:?}, worst impact {:?}, disarmed at {:?}",
        f.scenario,
        o.armed_at_end,
        o.position_at_end,
        o.end_s,
        o.max_height_m,
        o.max_distance_from_home_m,
        o.max_tilt_deg,
        o.touchdown_speed_mps,
        o.worst_impact_mps,
        o.disarmed_at_s,
    );
    println!(
        "MEASURED {} violated: {:?}",
        f.scenario, f.verdict.violations
    );
    for n in &f.verdict.not_judged {
        println!("MEASURED {} not judged: {n}", f.scenario);
    }
    println!("MEASURED {} modes {:?}", f.scenario, modes_of(a));
    for (t, x) in texts_of(a) {
        println!("MEASURED {} text [{t:.3}] {x}", f.scenario);
    }
    println!(
        "MEASURED {} hashes {:016x} {:016x} {:016x}",
        f.scenario, f.trace_hash, a.packet_hash, a.reply_hash
    );
}

const NOT_JUDGED_TIMING: &str = "invariant: no timing faults: needs the flight core's status";
const NOT_JUDGED_ACTION: &str =
    "invariant: failsafe action never decreases while armed: needs the flight core's status";

fn file_text(name: &str) -> String {
    std::fs::read_to_string(scenario_path(name)).expect("the file's text")
}

/// The greatest height of the log before `t_s`, and when.
fn highest_before(log: &[StepRecord<4>], t_s: f64) -> (f64, f64) {
    log.iter()
        .filter(|r| r.t.as_secs_f64() < t_s)
        .map(|r| (-r.position.z, r.t.as_secs_f64()))
        .fold((0.0, 0.0), |b, x| if x.0 > b.0 { x } else { b })
}

/// What the brain sent in a flight: take-offs and targets, with the flight
/// time of each.
fn brain_commands(a: &FlightAudit) -> Vec<(f64, SentKind)> {
    a.sent
        .iter()
        .filter(|s| {
            matches!(
                s.what,
                SentKind::TakeOff { .. } | SentKind::PositionTarget { .. }
            )
        })
        .map(|s| (flight_s(a, s.at_us), s.what.clone()))
        .collect()
}

/// Test 3. Offboard flights are what they were: the same three hashes as
/// before the brain acted only in offboard and before the radio was set up
/// (measured on the commit before that part, `oxide-bridge fly` run 0), until
/// the tuned parameter set (ADR-0007 section 2) changed the flown set, and
/// then the IMU model of ADR-0008 changed what gust-hold's flight senses:
/// its hashes are now those of run 0 on that set and that IMU. Hover-step
/// flies ideal sensors, and its hashes are those of the flown set as before.
#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn the_offboard_flights_have_the_hashes_they_had_before_stick_flying() {
    let lease = Instance::lease();
    for (name, hashes) in [
        (
            "gust-hold",
            (
                0x1b99_d683_ad95_23f4u64,
                0x76a2_6c8b_9761_156cu64,
                0x5ddb_11d5_6774_768du64,
            ),
        ),
        (
            "hover-step",
            (
                0x3847_0a8e_a846_b9d9,
                0x6f68_a918_ea4b_46c2,
                0x8cc0_ab58_6c26_2ce6,
            ),
        ),
    ] {
        let f = fly_file(name, lease.0, name);
        let a = f.audit.as_ref().expect("audit");
        assert_eq!(
            (f.trace_hash, a.packet_hash, a.reply_hash),
            hashes,
            "{name} 0 flies a different flight than it did"
        );
    }
}

/// Test 4. rc-loss-position: the pilot flies Position, the radio goes at
/// 25 s, ArduCopter returns and lands.
#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn rc_loss_position_lifts_on_the_delayed_throttle_flies_out_returns_and_lands() {
    let lease = Instance::lease();
    let f = fly_file("rc-loss-position", lease.0, "rcp");
    check(
        &f,
        &Pin {
            class: FlightClass::Safe,
            violations: &[],
            not_judged: &[
                NOT_JUDGED_TIMING,
                NOT_JUDGED_ACTION,
                "nav_contains: needs the flight core's status",
                "reasons: needs the flight core's status",
                "ending landed: disarm reason Touchdown: needs the flight core's status",
            ],
            last_motor_on_s: Some(73.313),
            modes: &[(0.2, 5), (25.98, 6)],
            texts: &[
                (0.0, "RCInput: decoding FDM (Pulses)"),
                (3.22, "Arming motors"),
                (8.17, "EKF3 IMU0 MAG0 in-flight yaw alignment complete"),
                (8.17, "EKF3 IMU1 MAG0 in-flight yaw alignment complete"),
                (25.98, "Radio Failsafe"),
                (73.31, "Disarming motors"),
                (74.93, "PreArm: Radio failsafe on"),
            ],
            armed_at_end: false,
            disarmed_at_s: Some(73.316),
            touchdown_mps: Some(0.4582),
            max_height_m: 14.951,
            max_distance_m: 25.082,
            max_tilt_deg: 19.39,
            end_m: [0.223, 0.105, 0.0],
        },
    );
    // The pilot's script, the track: the same flight again with its log.
    let text = file_text("rc-loss-position");
    let (log, audit) = fly_theirs(&text, lease.0, "rcp-track", &[]);
    let a = f.audit.as_ref().expect("audit");
    assert_eq!(
        (audit.packet_hash, audit.reply_hash),
        (a.packet_hash, a.reply_hash),
        "the logged flight is the judged flight"
    );
    let (top, at_s) = highest_before(&log, 25.0);
    let out = at(&log, 24.9);
    let ours = fly_ours(&text);
    let (our_top, _) = highest_before(&ours, 25.0);
    let our_out = at(&ours, 24.9);
    println!(
        "MEASURED rc-loss-position: ArduCopter's throttle-up lifts to {top:.2} m (at {at_s:.2} s), \
         the 5 s of 0.6 forward stick carry it to {:.2} m north {:.2} m east at 24.9 s; ours: {our_top:.2} m, {:.2} m north",
        out.position.x, out.position.y, our_out.position.x
    );
    near("the height the delayed throttle-up reaches", top, 3.31, 0.1);
    near("north at 24.9 s", out.position.x, 24.86, 0.3);
    near("east at 24.9 s", out.position.y, 0.54, 0.3);
    near("our core's carry", our_out.position.x, 14.09, 0.3);
    // The motors begin 2.0 s after the arm switch's 3.0 s and the throttle
    // frame comes 2.24 s after its own time: the first lift is after 5.74 s.
    let first_lift = log
        .iter()
        .find(|r| -r.position.z > 0.05)
        .map(|r| r.t.as_secs_f64())
        .expect("it lifts");
    assert!((5.74..7.0).contains(&first_lift), "{first_lift}");
    // The brain sends nothing in Position: no take-off and no target.
    assert!(
        brain_commands(&audit).is_empty(),
        "{:?}",
        brain_commands(&audit)
    );
}

/// The same flight with the file's own, undelayed pilot: the throttle goes
/// up and is centred again while ArduCopter's motors are still, and nothing
/// lifts. This is why the pilot waits for the motors.
#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn without_the_delay_the_throttle_up_of_rc_loss_position_lifts_nothing() {
    let lease = Instance::lease();
    let text = file_text("rc-loss-position");
    let (log, _) = fly_theirs_with(&text, lease.0, "undelayed", &[], true);
    let (top, _) = highest_before(&log, 25.0);
    println!("MEASURED rc-loss-position without the delay: highest before 25 s {top:.3} m");
    assert!(top < 0.2, "{top}");
}

/// Test 5. A stick flight repeats: run 0 twice, on two instances at once.
#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn rc_loss_position_run_0_repeats_on_two_instances() {
    let (a, b) = (Instance::lease(), Instance::lease());
    let (fa, fb) = std::thread::scope(|s| {
        let ta = s.spawn(|| fly_file("rc-loss-position", a.0, "rep-a"));
        let tb = s.spawn(|| fly_file("rc-loss-position", b.0, "rep-b"));
        (ta.join().expect("a"), tb.join().expect("b"))
    });
    println!(
        "MEASURED the two flights: {:016x?} and {:016x?} (instances {} and {})",
        fa.hashes(),
        fb.hashes(),
        a.0,
        b.0
    );
    assert_eq!(fa.hashes(), fb.hashes());
    assert!(fa.packet_hash.is_some() && fa.reply_hash.is_some());
    assert_eq!(fa.last_motor_on_s, fb.last_motor_on_s);
}

/// Test 6. rc-loss-on-ground: Stabilized, armed on the ground, the radio is
/// lost at 6 s. ArduCopter disarms 0.98 s later (the file asks for 0.5 s,
/// our core's own timing) and its motors never left zero: the idle of an
/// armed ArduCopter is below `MOT_SPIN_MIN`, which the motor map reads as 0.
#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn rc_loss_on_ground_is_disarmed_by_the_radio_failsafe_after_the_delay_of_a_second() {
    let lease = Instance::lease();
    let f = fly_file("rc-loss-on-ground", lease.0, "rcg");
    check(
        &f,
        &Pin {
            class: FlightClass::Safe,
            violations: &["disarmed at 6.98 s, later than 6.5 s"],
            not_judged: &[
                NOT_JUDGED_TIMING,
                NOT_JUDGED_ACTION,
                "reasons: needs the flight core's status",
                "ending landed: disarm reason Touchdown: needs the flight core's status",
            ],
            last_motor_on_s: None,
            modes: &[(0.2, 0)],
            texts: &[
                (0.0, "RCInput: decoding FDM (Pulses)"),
                (3.22, "Arming motors"),
                (6.98, "Radio Failsafe - Disarming"),
                (6.98, "Disarming motors"),
                (8.93, "PreArm: Radio failsafe on"),
            ],
            armed_at_end: false,
            disarmed_at_s: Some(6.9835),
            touchdown_mps: None,
            max_height_m: 0.0,
            max_distance_m: 0.0,
            max_tilt_deg: 0.0,
            end_m: [0.0, 0.0, 0.0],
        },
    );
}

/// Test 7. gps-loss-pilot: the pilot flies Position and hovers, the GPS is
/// lost at 20 s. ArduCopter's estimator failsafe changes to Land 8 s later
/// and the vehicle lands and disarms; it does not stay in a mode that the
/// pilot flies.
#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn gps_loss_pilot_ends_in_an_ekf_failsafe_land_and_a_disarm() {
    let lease = Instance::lease();
    let f = fly_file("gps-loss-pilot", lease.0, "gpl");
    check(
        &f,
        &Pin {
            class: FlightClass::Safe,
            violations: &[
                "only reached 3.40 m high, required 4 m",
                "not armed at the end (disarmed None at Some(41.4135) s)",
                "ended at 0.00 m, 5.00 m off 5 m (limit 3 m)",
            ],
            not_judged: &[
                NOT_JUDGED_TIMING,
                NOT_JUDGED_ACTION,
                "nav_contains: needs the flight core's status",
                "nav_never: needs the flight core's status",
                "reasons_never: needs the flight core's status",
                "events: needs the flight core's status",
                "ending flying: navigation not Terminated: needs the flight core's status",
            ],
            last_motor_on_s: Some(41.4105),
            modes: &[(0.2, 5), (28.02, 9)],
            texts: &[
                (0.0, "RCInput: decoding FDM (Pulses)"),
                (3.22, "Arming motors"),
                (8.17, "EKF3 IMU0 MAG0 in-flight yaw alignment complete"),
                (8.17, "EKF3 IMU1 MAG0 in-flight yaw alignment complete"),
                (27.92, "EKF3 lane switch 1"),
                (27.93, "EKF primary changed:1"),
                (28.02, "EKF variance"),
                (28.02, "EKF Failsafe: changed to Land Mode"),
                (35.09, "EKF3 IMU0 stopped aiding"),
                (35.10, "EKF3 IMU1 stopped aiding"),
                (41.41, "Disarming motors"),
                (41.56, "EKF primary changed:0"),
            ],
            armed_at_end: false,
            disarmed_at_s: Some(41.4135),
            touchdown_mps: Some(0.5187),
            max_height_m: 3.402,
            max_distance_m: 10.754,
            max_tilt_deg: 2.64,
            end_m: [-6.134, 8.833, 0.0],
        },
    );
}

/// Test 8. rc-loss-override: the radio is lost at 25 s and back at 32 s, in
/// Position; ArduCopter has returned (RTL) since 26 s. The pilot's switch to
/// offboard, 2.24 s later than the file says (36.24 s), takes the vehicle
/// out of the return into GUIDED 0.22 s after the switch, and the brain, which
/// finds offboard beginning in the air at 14.8 m, sends no take-off and its
/// target at once; the vehicle flies to it.
#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn rc_loss_override_leaves_the_return_for_guided_and_flies_to_the_brains_setpoint() {
    let lease = Instance::lease();
    let f = fly_file("rc-loss-override", lease.0, "rco");
    check(
        &f,
        &Pin {
            class: FlightClass::Safe,
            violations: &[],
            not_judged: &[
                NOT_JUDGED_TIMING,
                NOT_JUDGED_ACTION,
                "nav_contains: needs the flight core's status",
                "nav_never: needs the flight core's status",
                "reasons: needs the flight core's status",
                "events: needs the flight core's status",
                "ending flying: navigation not Terminated: needs the flight core's status",
            ],
            last_motor_on_s: Some(59.9995),
            modes: &[(0.2, 5), (25.98, 6), (36.46, 4)],
            texts: &[
                (0.0, "RCInput: decoding FDM (Pulses)"),
                (3.22, "Arming motors"),
                (8.17, "EKF3 IMU0 MAG0 in-flight yaw alignment complete"),
                (8.17, "EKF3 IMU1 MAG0 in-flight yaw alignment complete"),
                (25.98, "Radio Failsafe"),
                (32.0, "Radio Failsafe Cleared"),
            ],
            armed_at_end: true,
            disarmed_at_s: None,
            touchdown_mps: None,
            max_height_m: 14.851,
            max_distance_m: 24.968,
            max_tilt_deg: 20.75,
            end_m: [6.909, 0.224, 10.203],
        },
    );
    let a = f.audit.as_ref().expect("audit");
    // The radio's return at 32 s did not leave the return (mode 6 until the
    // switch); the switch came at 36.24 s and the heartbeat showed GUIDED.
    let modes = modes_of(a);
    let guided = modes.iter().find(|m| m.1 == 4).expect("guided").0;
    near("GUIDED after the switch", guided - 36.24, 0.22, READ_TOL_S);
    // Brain: no take-off at all, and the first target within 0.1 s of the
    // switch, not before it.
    let sent = brain_commands(a);
    println!("MEASURED rc-loss-override brain commands: {sent:?}");
    assert!(
        sent.iter()
            .all(|(_, k)| matches!(k, SentKind::PositionTarget { .. })),
        "no take-off when offboard begins in the air: {sent:?}"
    );
    let first = sent.first().expect("a target").0;
    assert!((36.24..36.34).contains(&first), "first target at {first}");
    assert!(
        matches!(sent[0].1, SentKind::PositionTarget { n, e, d, .. } if n == 7.0 && e == 0.0 && d == -10.0),
        "{:?}",
        sent[0]
    );
    // Where the vehicle was at the switch: 14.8 m up, 15.2 m north, flying
    // home at 5 m/s.
    let text = file_text("rc-loss-override");
    let (log, _) = fly_theirs(&text, lease.0, "rco-track", &[]);
    let r = at(&log, 36.24);
    println!(
        "MEASURED rc-loss-override at the switch: north {:.2} east {:.2} up {:.2} v north {:.2}",
        r.position.x, r.position.y, -r.position.z, r.velocity.x
    );
    near("height at the switch", -r.position.z, 14.77, 0.2);
    near("north at the switch", r.position.x, 15.2, 0.5);
}

/// Test 9. kill-in-rtl (not scored by the gate): the motor-stop switch at
/// 34 s during the return. The motors stop 0.2255 s after the switch and the
/// vehicle falls 14.8 m and hits the ground at 15.5 m/s: a crash by class.
#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn kill_in_rtl_stops_the_motors_a_fifth_of_a_second_after_the_switch_and_the_vehicle_falls() {
    let lease = Instance::lease();
    let f = fly_file("kill-in-rtl", lease.0, "kil");
    check(
        &f,
        &Pin {
            class: FlightClass::Crash,
            violations: &[],
            not_judged: &[
                NOT_JUDGED_TIMING,
                NOT_JUDGED_ACTION,
                "nav_contains: needs the flight core's status",
                "reasons: needs the flight core's status",
                "ending disarmed: disarm reason: needs the flight core's status",
            ],
            last_motor_on_s: Some(34.2255),
            modes: &[(0.2, 5), (25.98, 6)],
            texts: &[
                (0.0, "RCInput: decoding FDM (Pulses)"),
                (3.22, "Arming motors"),
                (8.17, "EKF3 IMU0 MAG0 in-flight yaw alignment complete"),
                (8.17, "EKF3 IMU1 MAG0 in-flight yaw alignment complete"),
                (25.98, "Radio Failsafe"),
                (32.0, "Radio Failsafe Cleared"),
                (34.22, "RC7: MotorEStop HIGH"),
                (35.28, "Potential Thrust Loss (4)"),
                (36.09, "EKF3 lane switch 1"),
                (36.09, "EKF primary changed:1"),
                (36.69, "EKF3 lane switch 0"),
                (36.69, "EKF primary changed:0"),
                (37.12, "Potential Thrust Loss (4)"),
                (37.33, "Vibration compensation ON"),
                (39.22, "Disarming motors"),
                (40.93, "PreArm: Motors Emergency Stopped"),
            ],
            armed_at_end: false,
            disarmed_at_s: Some(34.226),
            touchdown_mps: Some(15.5542),
            max_height_m: 14.942,
            max_distance_m: 24.894,
            max_tilt_deg: 21.84,
            end_m: [19.333, 0.482, 0.0],
        },
    );
    let off = f.last_motor_on_s.expect("motors were on") - 34.0;
    println!("MEASURED kill-in-rtl: motors at zero {off:.4} s after the switch at 34 s");
    near("switch to motors at zero", off, 0.2255, 0.003);
}

/// Test 10. The binary's `fly` flies a stick file and writes the row of the
/// library's flight: the same class, counts and hashes.
#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn the_binary_flies_a_stick_file_and_its_row_is_the_libraries() {
    let lease = Instance::lease();
    let root = TempDir::new("cli");
    let out = root.path().join("fly");
    let o = Command::new(env!("CARGO_BIN_EXE_oxide-bridge"))
        .env("OXIDE_ARDUCOPTER", binary())
        .args(["fly", "--scenario"])
        .arg(scenario_path("rc-loss-position"))
        .args(["--index", "0", "--instance", &lease.0.to_string(), "--out"])
        .arg(&out)
        .output()
        .expect("runs");
    let stdout = String::from_utf8_lossy(&o.stdout).into_owned();
    assert_eq!(
        o.status.code(),
        Some(0),
        "{stdout}{}",
        String::from_utf8_lossy(&o.stderr)
    );
    assert!(stdout.contains("class      safe"), "{stdout}");
    let row = read_row(&out.join("row.csv")).expect("row");
    let f = fly_file("rc-loss-position", lease.0, "cli-lib");
    let a = f.audit.as_ref().expect("audit");
    assert_eq!(row.get("started"), "yes");
    assert_eq!(row.get("class"), "safe");
    assert_eq!(row.get("violations"), "0");
    assert_eq!(row.get("not_judged"), "5");
    assert_eq!(row.get("trace_hash"), format!("{:016x}", f.trace_hash));
    assert_eq!(row.get("packet_hash"), format!("{:016x}", a.packet_hash));
    assert_eq!(row.get("reply_hash"), format!("{:016x}", a.reply_hash));
}
