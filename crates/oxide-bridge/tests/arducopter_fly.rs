// SPDX-License-Identifier: AGPL-3.0-or-later
//! The real ArduCopter simulator program flies our quad (ADR-0006, ADR-0007).
//!
//! Every test here needs the binary and is ignored by default: set
//! `OXIDE_ARDUCOPTER` to its path and run with `--ignored` (and `--release`:
//! a bridged flight is 85 simulated seconds). With the variable unset an
//! ignored test that is run fails and says so. Each test has its own
//! instance number (so that they can run side by side), its own directory
//! under the system's temporary directory, and leaves no process behind.

use std::path::{Path, PathBuf};
use std::sync::{Condvar, Mutex};
use std::time::Instant;

use oxide_bridge::arducopter::{
    ArduCopter, ArduCopterConfig, ArduCopterError, BrainLink, FlightAudit, Flown, FlyError,
    GpsError, GpsErrorState, LinkName, PREROLL_STEPS, Sent, SentKind, TelemetryRequest,
    bridged_options, fly,
};
use oxide_bridge::mav::{MavEvent, id};
use oxide_bridge::sitl::PINNED;
use oxide_core::math::{Real, Vec3};
use oxide_core::schedule::F405_LEAN;
use oxide_core::time::Micros;
use oxide_sim::metrics::{HoldCriteria, HoldMetrics};
use oxide_sim::outcome::FlightOutcome;
use oxide_sim::scenario::{GustHold, HoverStep, NoPilot, Pilot, gust_hold};
use oxide_sim::{Airframe, SimOptions, Simulation, StepRecord};

/// The binary's path, or a failure that says what to set.
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

/// A directory that is removed when it goes out of scope.
struct TempDir(PathBuf);

impl TempDir {
    fn new(tag: &str) -> Self {
        let p =
            std::env::temp_dir().join(format!("oxide-arducopter-fly-{}-{tag}", std::process::id()));
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

/// The program's instance numbers, 0 to 9, leased one to a flight at a time so
/// that tests that run side by side never share a port. A test holds its lease
/// for all the flights it flies one after the other.
static POOL: (Mutex<u16>, Condvar) = (Mutex::new(0), Condvar::new());

struct Instance(u8);

impl Instance {
    /// The lowest free instance, waiting for one when all ten are taken.
    fn lease() -> Self {
        let mut taken = POOL.0.lock().expect("the pool");
        loop {
            if let Some(i) = (0..10u8).find(|i| *taken & (1 << i) == 0) {
                *taken |= 1 << i;
                return Self(i);
            }
            taken = POOL.1.wait(taken).expect("the pool");
        }
    }
}

impl Drop for Instance {
    fn drop(&mut self) {
        if let Ok(mut taken) = POOL.0.lock() {
            *taken &= !(1 << self.0);
        }
        POOL.1.notify_all();
    }
}

fn airframe() -> Airframe {
    Airframe::reference_quad().expect("the reference airframe")
}

/// The configuration of a test's flight on its own instance.
fn config(instance: u8, root: &TempDir) -> ArduCopterConfig {
    ArduCopterConfig::new(binary(), root.path().join("flights"), instance)
}

/// A simulation around a started ArduCopter (nothing run yet).
fn simulation<P: Pilot>(
    instance: u8,
    root: &TempDir,
    pilot: P,
    options: SimOptions,
    edit: impl FnOnce(&mut ArduCopterConfig),
) -> Simulation<P, 4, ArduCopter> {
    let mut c = config(instance, root);
    edit(&mut c);
    let options = bridged_options(options, &mut c);
    let ap = ArduCopter::start(c).unwrap_or_else(|e| panic!("{e}"));
    Simulation::with_autopilot(&airframe(), ap, pilot, options).expect("the simulation")
}

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

fn outcome_of(log: &[StepRecord<4>]) -> FlightOutcome {
    let mut o = FlightOutcome::new();
    for r in log {
        o.add(r);
    }
    o
}

/// Flies `pilot` for `seconds` with every record kept.
fn fly_logged<P: Pilot>(
    instance: u8,
    tag: &str,
    pilot: P,
    options: SimOptions,
    seconds: f64,
    edit: impl FnOnce(&mut ArduCopterConfig),
) -> (Vec<StepRecord<4>>, Flown) {
    let root = TempDir::new(tag);
    let mut c = config(instance, &root);
    edit(&mut c);
    let mut log = Vec::new();
    let wall = Instant::now();
    let flown = fly(
        &airframe(),
        pilot,
        options,
        c,
        Micros((seconds * 1e6) as u64),
        |r| log.push(*r),
    )
    .unwrap_or_else(|e| panic!("{e}"));
    println!("WALL {tag}: {:.2} s", wall.elapsed().as_secs_f64());
    (log, flown)
}

fn texts(audit: &FlightAudit) -> String {
    audit
        .status_texts
        .iter()
        .map(|(t, _, x)| format!("[{:.3}] {x}", *t as f64 * 1e-6))
        .collect::<Vec<_>>()
        .join("\n")
}

/// Test 1. The pre-roll alone.
#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn the_pre_roll_leaves_a_vehicle_that_is_ready_and_has_never_armed() {
    let root = TempDir::new("preroll");
    let lease = Instance::lease();
    let mut sim = simulation(
        lease.0,
        &root,
        HoverStep::default(),
        SimOptions::realistic(1),
        |c| {
            // The program's clock at 50 Hz, so that the last message read is
            // at most 20 ms old at the end of the pre-roll.
            c.telemetry = vec![TelemetryRequest {
                message_id: id::SYSTEM_TIME,
                hz: 50.0,
                at_step: 4_000,
            }];
        },
    );
    sim.preroll(PREROLL_STEPS).unwrap_or_else(|e| panic!("{e}"));
    let ap = sim.autopilot();
    let a = ap.audit();
    assert_eq!(a.preroll_steps, PREROLL_STEPS);
    assert_eq!(a.exchanges, PREROLL_STEPS);
    // No pulse above 1000 us on outputs 1 to 4 through the whole pre-roll
    // (a pulse above is an error that stops the pre-roll).
    println!(
        "MEASURED pre-roll: highest pulse {} us",
        a.preroll_max_pulse_us
    );
    assert!(a.preroll_max_pulse_us <= 1000, "{}", a.preroll_max_pulse_us);
    assert!(!ap.armed());
    assert!(
        a.armed_changes.iter().all(|(_, armed)| !armed),
        "{:?}",
        a.armed_changes
    );
    // ArduCopter says it is ready.
    let ready = a.texts_containing("ArduPilot Ready").next();
    let ready = ready.unwrap_or_else(|| panic!("no ready text:\n{}", texts(a)));
    println!(
        "MEASURED pre-roll: ready text read at {:.4} s",
        ready.0 as f64 * 1e-6
    );
    // Its estimator uses GPS before 45 s.
    let gps_at: Vec<f64> = a
        .texts_containing("is using GPS")
        .map(|(t, _, _)| *t as f64 * 1e-6)
        .collect();
    println!("MEASURED pre-roll: 'is using GPS' texts at {gps_at:?} s");
    assert!(!gps_at.is_empty(), "{}", texts(a));
    assert!(gps_at.iter().all(|&t| t < 45.0), "{gps_at:?}");
    // Its own clock is ours: SYSTEM_TIME's boot time against the step that
    // read it, and the last one at the end of the pre-roll.
    let boots: Vec<(u64, u32)> = a
        .heard
        .iter()
        .filter_map(|h| match h.event {
            MavEvent::SystemTime { time_boot_ms, .. } => Some((h.at_us, time_boot_ms)),
            _ => None,
        })
        .collect();
    assert!(boots.len() > 100, "{}", boots.len());
    let worst = boots
        .iter()
        .map(|&(at, boot)| (f64::from(boot) - at as f64 * 1e-3).abs())
        .fold(0.0, f64::max);
    let (last_at, last_boot) = *boots.last().expect("a SYSTEM_TIME");
    println!(
        "MEASURED pre-roll: {} SYSTEM_TIME messages; the last read at {:.4} s says {:.3} s; \
         the largest gap between its clock and ours {worst:.1} ms",
        boots.len(),
        last_at as f64 * 1e-6,
        f64::from(last_boot) * 1e-3
    );
    assert!(
        (f64::from(last_boot) * 1e-3 - 45.0).abs() <= 0.05,
        "{last_boot}"
    );
    assert!((last_at as f64 * 1e-6 - 45.0).abs() <= 0.1, "{last_at}");
    assert!(worst <= 50.0, "{worst}");
    // The last PreArm text and when it was read: after 45 s none is
    // outstanding, which test 2 shows by arming.
    let prearm: Vec<&(u64, u8, String)> = a.texts_containing("PreArm").collect();
    println!("MEASURED pre-roll: PreArm texts {prearm:?}");
    if let Some(last) = prearm.last() {
        assert!(last.0 < PREROLL_STEPS * 500, "{last:?}");
    }
    // The two links of the brain and the instrument: heartbeats of the vehicle
    // were read (statuses need a heartbeat from us).
    assert!(a.vehicle_heartbeats >= 40, "{}", a.vehicle_heartbeats);
    // The instrument is not the ground station.
    assert_eq!(a.sent_as(255), 0, "nothing sent as system 255");
    assert!(
        a.heartbeats_sent_as(250) >= 44,
        "{}",
        a.heartbeats_sent_as(250)
    );
}

/// What test 2 measured for one hover-step flight.
struct HoverNumbers {
    errors: Vec<(f64, f64, f64)>,
    max_tilt: f64,
}

fn check_hover_step(log: &[StepRecord<4>], pilot: &HoverStep, label: &str) -> HoverNumbers {
    let at = |t: f64| {
        log.iter()
            .find(|r| r.t.as_secs_f64() >= t)
            .unwrap_or_else(|| panic!("no record at {t} s"))
    };
    let t_arm = log
        .iter()
        .find(|r| r.armed)
        .map(|r| r.t.as_secs_f64())
        .expect("armed by its pulses");
    let first_airborne = pilot
        .waypoints
        .iter()
        .find(|w| -w.position.z > 0.2)
        .map(|w| w.from_s)
        .expect("an airborne setpoint");
    let t_height = log
        .iter()
        .find(|r| (-r.position.z - 2.0).abs() <= 0.3)
        .map(|r| r.t.as_secs_f64())
        .expect("reached 2 m");
    // Each waypoint 6 s later, or just before the next change when that comes
    // sooner, or at the end. The first airborne waypoint is the take-off: its
    // 6 s fall at the time of the next change, when ArduCopter, which arms
    // 2 s after the switch and then climbs, has not reached it (measured);
    // it is reported and judged by the height check instead.
    let mut errors = Vec::new();
    for (i, w) in pilot.waypoints.iter().enumerate().skip(1) {
        let next = pilot
            .waypoints
            .get(i + 1)
            .map_or(pilot.duration_s - 0.001, |n| n.from_s - 0.001);
        let t = (w.from_s + 6.0).min(next);
        let p = at(t).position;
        let want = w.position.cast::<f64>();
        let d = p - want;
        let e = d.norm();
        errors.push((w.from_s, t, e));
    }
    let o = outcome_of(log);
    let numbers = HoverNumbers {
        errors,
        max_tilt: o.max_tilt_deg,
    };
    println!(
        "MEASURED {label}: arm switch at {:.2} s, first pulse above 1000 us at {:.3} s \
         ({:.3} s after the switch); height 2 m +- 0.3 m first at {:.3} s ({:.3} s after the \
         first airborne setpoint at {first_airborne:.1} s); position error to the waypoint \
         (waypoint from, evaluated at, metres): {:?}; largest tilt {:.2} deg; max height {:.3} m; \
         touchdown {:?}; class {:?}",
        pilot.arm_at_s,
        t_arm,
        t_arm - pilot.arm_at_s,
        t_height,
        t_height - first_airborne,
        numbers.errors,
        numbers.max_tilt,
        o.max_height_m,
        o.touchdown_speed_mps,
        class_of(&o)
    );
    assert!(
        t_arm - pilot.arm_at_s <= 3.0,
        "armed {} s after the switch",
        t_arm - pilot.arm_at_s
    );
    assert!(
        t_height - first_airborne <= 10.0,
        "height reached {} s after",
        t_height - first_airborne
    );
    for (from, t, e) in numbers
        .errors
        .iter()
        .filter(|(from, _, _)| *from > first_airborne)
    {
        assert!(*e <= 0.3, "waypoint {from}: {e} m at {t} s");
    }
    assert!(numbers.max_tilt <= 35.0, "tilt {}", numbers.max_tilt);
    assert_eq!(class_of(&o), Class::Safe);
    assert_eq!(o.physics_faults, 0);
    assert!(!o.non_finite);
    numbers
}

/// Test 2. The hover-step flies.
#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn the_hover_step_flies() {
    let pilot = HoverStep::default();
    let secs = pilot.duration_s;
    let lease = Instance::lease();
    let (log, flown) = fly_logged(
        lease.0,
        "hover",
        pilot.clone(),
        SimOptions::realistic(1),
        secs,
        |_| {},
    );
    check_hover_step(&log, &pilot, "hover-step");
    // After 45 s no PreArm text is outstanding: it armed.
    let a = &flown.audit;
    assert!(a.armed_changes.iter().any(|(_, armed)| *armed), "{a}");
    println!("{a}");

    // What was sent, to whom, and when (all decided from lockstep data).
    let program_us = |flight_s: f64| a.program_us_of_flight_s(flight_s);
    let t_arm = log.iter().find(|r| r.armed).expect("armed").t;
    let armed_at = program_us(t_arm.as_secs_f64());
    let takeoffs: Vec<&Sent> = a
        .sent
        .iter()
        .filter(|s| matches!(s.what, SentKind::TakeOff { .. }))
        .collect();
    let targets: Vec<&Sent> = a
        .sent
        .iter()
        .filter(|s| matches!(s.what, SentKind::PositionTarget { .. }))
        .collect();
    assert!(!takeoffs.is_empty() && !targets.is_empty(), "{a}");
    // Nothing for the brain before the pulses say armed: the take-off goes at
    // the first armed tick (the pulses of the exchange before it), and the
    // targets after the height is within 0.3 m of the take-off height.
    assert!(
        takeoffs.iter().all(|s| s.at_us >= armed_at),
        "a take-off before arming: {takeoffs:?} against {armed_at}"
    );
    println!(
        "MEASURED brain: first take-off at {:.4} s, the pulses armed at {:.4} s; {} take-offs, {} position targets; first target at {:.4} s",
        takeoffs[0].at_us as f64 * 1e-6,
        armed_at as f64 * 1e-6,
        takeoffs.len(),
        targets.len(),
        targets[0].at_us as f64 * 1e-6
    );
    assert!(
        takeoffs[0].at_us - armed_at <= 40_000,
        "the first take-off is at the first armed setpoint tick"
    );
    for s in &targets {
        let t = (s.at_us - a.preroll_steps * a.step_us) as f64 * 1e-6;
        let h = log
            .iter()
            .find(|r| r.t.as_secs_f64() >= t)
            .map(|r| -r.position.z)
            .expect("a record");
        assert!(h >= 1.7 - 1e-9, "a target at {t} s with the height {h} m");
    }
    // The instrument is not the ground station; the brain has an id of its own.
    assert_eq!(a.sent_as(255), 0, "nothing was sent as system 255");
    assert!(a.sent.iter().all(|s| match s.link {
        LinkName::Instrument => s.system_id == 250,
        LinkName::Brain => s.system_id == 251,
        LinkName::Gcs => false,
    }));
    assert!(
        a.sent
            .iter()
            .filter(|s| matches!(
                s.what,
                SentKind::TakeOff { .. } | SentKind::PositionTarget { .. }
            ))
            .all(|s| s.link == LinkName::Brain)
    );
    // Heartbeats at 1 Hz: the instrument's from the first exchange, the
    // brain's from the first flight tick.
    let instr = a.heartbeats_sent_as(250);
    println!(
        "MEASURED heartbeats as 250: {instr}, as 251: {}",
        a.heartbeats_sent_as(251)
    );
    assert_eq!(
        instr, 65,
        "45 s of pre-roll and 20 s of flight, one per second from the first"
    );
    assert_eq!(a.heartbeats_sent_as(251), 20);
    // The vehicle acknowledged the take-off; every acknowledgement it sent to
    // the brain is audit data.
    assert!(a.heard.iter().any(|h| h.link == LinkName::Brain
        && matches!(
            h.event,
            MavEvent::CommandAck {
                command: 22,
                result: 0
            }
        )));
}

/// The same hover-step three times, on two instances and at two speed-ups of
/// the program's own clock: the physics, the reply lines and the motor packets
/// repeat bit for bit.
#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn the_hover_step_repeats_bit_for_bit() {
    let mut results = Vec::new();
    let lease = Instance::lease();
    for (run, speedup) in [(0, 100), (1, 100), (2, 10)] {
        let pilot = HoverStep::default();
        let secs = pilot.duration_s;
        let (_, flown) = fly_logged(
            lease.0,
            &format!("repeat-{run}"),
            pilot,
            SimOptions::realistic(1),
            secs,
            move |c| c.speedup = speedup,
        );
        results.push((
            flown.trace_hash,
            flown.audit.packet_hash,
            flown.audit.reply_hash,
            flown.audit.params_sha256,
        ));
    }
    println!("MEASURED repeat: {results:?}");
    assert_eq!(results[0], results[1], "two instances");
    assert_eq!(results[0], results[2], "another speed-up");
}

fn alive(pid: u32) -> bool {
    #[cfg(unix)]
    {
        std::process::Command::new("kill")
            .args(["-0", &pid.to_string()])
            .stderr(std::process::Stdio::null())
            .status()
            .is_ok_and(|s| s.success())
    }
    #[cfg(not(unix))]
    {
        let _ = pid;
        false
    }
}

/// Test 9. A vehicle that cannot arm is an error, not a hang.
#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn a_vehicle_that_cannot_arm_ends_the_flight_with_an_error_within_5_s() {
    let root = TempDir::new("noarm");
    let pilot = HoverStep::default();
    let arm_at = pilot.arm_at_s;
    let lease = Instance::lease();
    let mut c = config(lease.0, &root);
    c.extra_params.push(("FRAME_CLASS".into(), 0.0));
    let wall = Instant::now();
    let r = fly(
        &airframe(),
        pilot,
        SimOptions::realistic(1),
        c,
        Micros(20_000_000),
        |_| {},
    );
    println!("WALL did-not-arm: {:.2} s", wall.elapsed().as_secs_f64());
    let Err(FlyError::Flight { source, audit }) = r else {
        panic!("the flight should end with an error, not {r:?}");
    };
    let text = source.to_string();
    println!("MEASURED did-not-arm: {text}");
    let ArduCopterError::DidNotArm { at, texts } = &source else {
        panic!("{text}");
    };
    let flight_s = at.flight_s().expect("in flight");
    println!(
        "MEASURED did-not-arm: the error at flight time {flight_s:.4} s, {:.4} s after the arm \
         switch at {arm_at} s",
        flight_s - arm_at
    );
    assert!(text.starts_with("ArduCopter did not arm"), "{text}");
    assert!(
        flight_s - arm_at >= 5.0 && flight_s - arm_at <= 5.1,
        "{flight_s}"
    );
    assert!(texts.iter().any(|(_, t)| t.contains("PreArm")), "{texts:?}");
    assert!(text.contains("PreArm"), "{text}");
    // No process is left: the one the flight started has gone.
    let audit = audit.expect("the failed flight's audit");
    assert_eq!(audit.sitl_ended_before_stop, Some(false));
    let pids: Vec<u32> = std::fs::read_dir(root.path().join("flights"))
        .expect("flights")
        .filter_map(|d| std::fs::read_to_string(d.ok()?.path().join("sitl.pid")).ok())
        .filter_map(|t| t.lines().next()?.parse().ok())
        .collect();
    assert_eq!(pids.len(), 1, "{pids:?}");
    assert!(pids.iter().all(|&p| !alive(p)), "{pids:?}");
}

// ---------------------------------------------------------------------------
// Statistics (no float method that the determinism rule forbids).

fn mean(v: &[f64]) -> f64 {
    v.iter().sum::<f64>() / v.len() as f64
}

fn std_dev(v: &[f64]) -> f64 {
    let m = mean(v);
    Real::sqrt(v.iter().map(|x| (x - m) * (x - m)).sum::<f64>() / v.len() as f64)
}

/// The lag-1 autocorrelation of a series.
fn lag1(v: &[f64]) -> f64 {
    let m = mean(v);
    let var = v.iter().map(|x| (x - m) * (x - m)).sum::<f64>() / v.len() as f64;
    let cov = v.windows(2).map(|w| (w[0] - m) * (w[1] - m)).sum::<f64>() / (v.len() - 1) as f64;
    cov / var
}

/// A vehicle at rest on the ground after the pre-roll: no pilot.
struct RestRun {
    audit: FlightAudit,
    /// Our sensor suite's gyro and accelerometer bias after every flight
    /// tick: `(program time of the tick's end, gyro, accel)`.
    biases: Vec<(u64, Vec3<f64>, Vec3<f64>)>,
    flight_start_us: u64,
}

impl RestRun {
    /// Our bias (gyro, accel) at program time `at_us`.
    fn bias_at(&self, at_us: u64) -> (Vec3<f64>, Vec3<f64>) {
        let i = self.biases.partition_point(|b| b.0 < at_us);
        let b = self.biases.get(i).or(self.biases.last()).expect("biases");
        (b.1, b.2)
    }
}

fn rest_run(
    instance: u8,
    tag: &str,
    seed: u64,
    gps_error: bool,
    seconds: f64,
    telemetry: Vec<TelemetryRequest>,
) -> RestRun {
    let root = TempDir::new(tag);
    let mut sim = simulation(instance, &root, NoPilot, SimOptions::realistic(seed), |c| {
        c.gps_error.enabled = gps_error;
        c.telemetry = telemetry;
    });
    sim.preroll(PREROLL_STEPS).unwrap_or_else(|e| panic!("{e}"));
    let end = Micros((seconds * 1e6) as u64);
    let mut biases = Vec::new();
    while sim.time() < end {
        sim.try_step().unwrap_or_else(|e| panic!("{e}"));
        biases.push((
            PREROLL_STEPS * 500 + sim.time().0,
            sim.sensors().gyro_bias(),
            sim.sensors().accel_bias(),
        ));
    }
    let audit = sim
        .into_autopilot()
        .finish()
        .unwrap_or_else(|e| panic!("{e}"));
    RestRun {
        audit,
        biases,
        flight_start_us: PREROLL_STEPS * 500,
    }
}

fn telemetry(message_id: u32, hz: f32, at_step: u64) -> TelemetryRequest {
    TelemetryRequest {
        message_id,
        hz,
        at_step,
    }
}

/// One GPS fix as read.
#[derive(Clone, Copy, Debug)]
struct Fix {
    at_us: u64,
    /// The fix's own time stamp (the program's clock), µs.
    time_usec: u64,
    lat: i32,
    lon: i32,
    vel_cms: u16,
}

/// The distinct GPS fixes read (by their own time stamp).
fn fixes(a: &FlightAudit) -> Vec<Fix> {
    let mut seen = None;
    let mut out = Vec::new();
    for h in &a.heard {
        if let MavEvent::GpsRaw {
            time_usec,
            lat,
            lon,
            vel_cms,
            ..
        } = h.event
            && seen != Some(time_usec)
        {
            seen = Some(time_usec);
            out.push(Fix {
                at_us: h.at_us,
                time_usec,
                lat,
                lon,
                vel_cms,
            });
        }
    }
    out
}

/// Metres per degree of latitude as ArduPilot's own conversion has it
/// (1.113195e-2 m per 1e-7 degree), and the east scale at latitude `lat`.
const METRES_PER_DEGREE: f64 = 111_319.5;

/// North and east in metres of a fix from a reference fix: the difference of
/// latitude times the metres per degree, and of longitude times that and the
/// cosine of the reference latitude.
fn north_east(f: &Fix, from: &Fix) -> (f64, f64) {
    let north = f64::from(f.lat - from.lat) * 1e-7 * METRES_PER_DEGREE;
    let lat0 = f64::from(from.lat) * 1e-7 * core::f64::consts::PI / 180.0;
    let east = f64::from(f.lon - from.lon) * 1e-7 * METRES_PER_DEGREE * Real::cos(lat0);
    (north, east)
}

fn per_axis_raw_imu(a: &FlightAudit, from_us: u64) -> ([Vec<f64>; 3], [Vec<f64>; 3]) {
    let mut acc = [Vec::new(), Vec::new(), Vec::new()];
    let mut gyro = [Vec::new(), Vec::new(), Vec::new()];
    for h in &a.heard {
        if h.at_us < from_us {
            continue;
        }
        if let MavEvent::RawImu {
            acc: ac, gyro: gy, ..
        } = h.event
        {
            for k in 0..3 {
                acc[k].push(f64::from(ac[k]));
                gyro[k].push(f64::from(gy[k]));
            }
        }
    }
    (acc, gyro)
}

fn pressures(a: &FlightAudit, from_us: u64) -> Vec<f64> {
    a.heard
        .iter()
        .filter(|h| h.at_us >= from_us)
        .filter_map(|h| match h.event {
            MavEvent::ScaledPressure {
                id: 1,
                press_abs_hpa,
                ..
            } => Some(f64::from(press_abs_hpa)),
            _ => None,
        })
        .collect()
}

/// Mean and standard deviation of a message's field over the flight window.
fn summary(v: &[f64]) -> (f64, f64) {
    (mean(v), std_dev(v))
}

/// Test 6. The GPS error reaches ArduCopter's GPS, and nothing else.
#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn the_gps_error_reaches_arducopters_gps_and_nothing_else() {
    let tel = || {
        vec![
            telemetry(id::GPS_RAW_INT, 10.0, 4_000),
            telemetry(id::RAW_IMU, 50.0, 4_001),
            telemetry(id::SCALED_PRESSURE, 20.0, 4_002),
        ]
    };
    let lease = Instance::lease();
    let with = rest_run(lease.0, "rest-with", 1, true, 30.0, tel());
    let without = rest_run(lease.0, "rest-without", 1, false, 30.0, tel());
    let sigma = SimOptions::realistic(1).sensors.gps.pos_sigma_h_m;
    let window = with.flight_start_us + 1_000_000;

    // ArduCopter's reported GPS position, converted to metres north and east
    // of the last fix of the pre-roll (which has no error; the truth is the
    // start point, so the reported offset is the error): the latitude and
    // longitude differences times 111 319.5 m per degree, the longitude's
    // also times the cosine of the reference latitude. Each distinct fix
    // (by its own time stamp) once.
    let errors = |r: &RestRun| {
        let all = fixes(&r.audit);
        let reference = *all
            .iter()
            .rev()
            .find(|f| f.at_us < r.flight_start_us)
            .expect("a fix in the pre-roll");
        let flight: Vec<&Fix> = all.iter().filter(|f| f.at_us >= window).collect();
        let n: Vec<f64> = flight.iter().map(|f| north_east(f, &reference).0).collect();
        let e: Vec<f64> = flight.iter().map(|f| north_east(f, &reference).1).collect();
        let v: Vec<f64> = flight.iter().map(|f| f64::from(f.vel_cms)).collect();
        (n, e, v)
    };
    let (n1, e1, v1) = errors(&with);
    let (n0, e0, v0) = errors(&without);
    println!(
        "MEASURED GPS error: with it {} fixes: north mean {:.3} sd {:.3} lag-1 {:.3}; \
         east mean {:.3} sd {:.3} lag-1 {:.3} (our model's sigma {sigma}); \
         without it {} fixes: north sd {:.4} east sd {:.4} (largest {:.3} m)",
        n1.len(),
        mean(&n1),
        std_dev(&n1),
        lag1(&n1),
        mean(&e1),
        std_dev(&e1),
        lag1(&e1),
        n0.len(),
        std_dev(&n0),
        std_dev(&e0),
        n0.iter().chain(&e0).fold(0.0_f64, |m, x| m.max(x.abs()))
    );
    assert!(
        n1.len() >= 140 && n0.len() >= 140,
        "{} {}",
        n1.len(),
        n0.len()
    );
    // What the real program does with the error (measured; the reason the
    // specification's two numbers are not met): its GPS lag reads the position
    // 150 ms back between two stored samples 200 ms apart, so each fix is
    // 0.75 of the error held before and 0.25 of the error held after. Fix j
    // (j = 1 is the first of the flight) is 0.75 d(j-2) + 0.25 d(j-1) of our
    // draws d(0), d(1), ...
    let predicted = |j: usize, draws: &[[f64; 2]]| -> (f64, f64) {
        let d = |k: usize| -> [f64; 2] { draws.get(k).copied().unwrap_or([0.0; 2]) };
        let (a, b) = (
            j.checked_sub(2).map_or([0.0; 2], d),
            j.checked_sub(1).map_or([0.0; 2], d),
        );
        (0.75 * a[0] + 0.25 * b[0], 0.75 * a[1] + 0.25 * b[1])
    };
    let mut state = GpsErrorState::new(
        GpsError {
            enabled: true,
            sigma_m: sigma,
            period_us: 200_000,
            first_fix_us: 0,
        },
        1,
    );
    let draws: Vec<[f64; 2]> = (0..200).map(|k| state.at(k * 200_000)).collect();
    let all = fixes(&with.audit);
    let first_flight = all
        .iter()
        .position(|f| f.at_us >= with.flight_start_us)
        .expect("a fix in the flight");
    let reference = *all
        .iter()
        .rev()
        .find(|f| f.at_us < with.flight_start_us)
        .expect("a fix in the pre-roll");
    let (mut sum_sq, mut count) = (0.0, 0);
    // The fixes are 200 ms apart on the program's own clock. The last fix of
    // the pre-roll is fix 0 and the flight's own are 1, 2, ...
    for f in all.iter().skip(first_flight).take(140) {
        let j = ((f.time_usec - reference.time_usec) as f64 / 200_000.0).round() as usize;
        let (n, e) = north_east(f, &reference);
        let (pn, pe) = predicted(j, &draws);
        sum_sq += (n - pn) * (n - pn) + (e - pe) * (e - pe);
        count += 2;
    }
    let rms = Real::sqrt(sum_sq / f64::from(count));
    let expect_sd = sigma * Real::sqrt(0.75 * 0.75 + 0.25 * 0.25);
    println!(
        "MEASURED GPS error: the reported error against 0.75 d(j-2) + 0.25 d(j-1): rms residual \
         {rms:.4} m over {count} values; expected sd {expect_sd:.4} m; specification's criteria: \
         sd within 20 % of {sigma}: north {:.1} %, east {:.1} % (NOT MET); lag-1 under 0.3: \
         {:.3}, {:.3} (NOT MET)",
        100.0 * (std_dev(&n1) / sigma - 1.0),
        100.0 * (std_dev(&e1) / sigma - 1.0),
        lag1(&n1),
        lag1(&e1)
    );
    assert!(
        rms < 0.03,
        "the reported error is our error through the blend: rms {rms}"
    );
    // The error starts after the pre-roll: the fixes of its last 20 s, with
    // the error on, show none (ArduCopter's origin is set without it).
    for (name, run) in [("with", &with), ("without", &without)] {
        let all = fixes(&run.audit);
        let quiet: Vec<&Fix> = all
            .iter()
            .filter(|f| f.at_us >= 25_000_000 && f.at_us < run.flight_start_us)
            .collect();
        let n: Vec<f64> = quiet.iter().map(|f| north_east(f, &reference).0).collect();
        let e: Vec<f64> = quiet.iter().map(|f| north_east(f, &reference).1).collect();
        println!(
            "MEASURED pre-roll fixes {name} the error (25 s to 45 s): {} fixes, north sd {:.4} east sd {:.4}",
            quiet.len(),
            std_dev(&n),
            std_dev(&e)
        );
        assert!(quiet.len() >= 90, "{}", quiet.len());
        assert!(
            std_dev(&n) < 0.03 && std_dev(&e) < 0.03,
            "{name}: the pre-roll has an error"
        );
    }
    for (name, v) in [("north", &n1), ("east", &e1)] {
        let sd = std_dev(v);
        assert!(
            (sd - expect_sd).abs() <= 0.12 * expect_sd,
            "{name}: sd {sd} against {expect_sd}"
        );
        // The blend makes successive fixes share a draw: a lag-1 correlation of
        // 0.1875 / 0.625 = 0.30 for white draws.
        assert!(lag1(v).abs() < 0.5, "{name}: lag-1 {}", lag1(v));
    }
    for (name, v) in [("north", &n0), ("east", &e0)] {
        assert!(
            std_dev(v) < 0.03,
            "{name} without the error: sd {}",
            std_dev(v)
        );
    }

    // Nothing else changed. The velocity ArduCopter reports (ground speed).
    let (vm1, vs1) = summary(&v1);
    let (vm0, vs0) = summary(&v0);
    println!(
        "MEASURED GPS velocity (cm/s, ground speed): with mean {vm1:.3} sd {vs1:.3}; without mean {vm0:.3} sd {vs0:.3}"
    );
    assert!(
        (vm1 - vm0).abs() <= 0.5 && (vs1 - vs0).abs() <= 0.5,
        "velocity changed"
    );
    // The pressure: SCALED_PRESSURE in hPa.
    let (p1, p0) = (
        pressures(&with.audit, window),
        pressures(&without.audit, window),
    );
    let (pm1, ps1) = summary(&p1);
    let (pm0, ps0) = summary(&p0);
    println!(
        "MEASURED pressure (hPa): with mean {pm1:.4} sd {ps1:.5} n {}; without mean {pm0:.4} sd {ps0:.5} n {}",
        p1.len(),
        p0.len()
    );
    // Equal within noise: the means within 3 standard errors of a difference,
    // the standard deviations within 10 %.
    let se = |sd: f64, n: usize| sd * Real::sqrt(2.0 / n as f64);
    assert!(
        (pm1 - pm0).abs() <= 3.0 * se(ps0, p0.len().min(p1.len())),
        "pressure mean"
    );
    assert!((ps1 / ps0 - 1.0).abs() <= 0.1, "pressure sd");
    // The IMU, every axis, accelerometer in milli-g and gyro in milli-rad/s.
    let (a1, g1) = per_axis_raw_imu(&with.audit, window);
    let (a0, g0) = per_axis_raw_imu(&without.audit, window);
    for axis in 0..3 {
        for (name, x1, x0) in [("accel", &a1, &a0), ("gyro", &g1, &g0)] {
            let (m1, s1) = summary(&x1[axis]);
            let (m0, s0) = summary(&x0[axis]);
            println!(
                "MEASURED raw IMU {name} axis {axis}: with mean {m1:.3} sd {s1:.3}; without mean {m0:.3} sd {s0:.3}; n {} {}",
                x1[axis].len(),
                x0[axis].len()
            );
            let n = x1[axis].len().min(x0[axis].len());
            assert!(
                (m1 - m0).abs() <= 3.0 * se(s0, n),
                "{name} {axis} mean: {m1} {m0}"
            );
            assert!((s1 / s0 - 1.0).abs() <= 0.1, "{name} {axis} sd: {s1} {s0}");
        }
    }
}

/// Test 7. The IMU is our model's.
#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn the_imu_arducopter_reads_is_our_models() {
    let run = |instance: u8, seed: u64| {
        rest_run(
            instance,
            &format!("imu-{seed}"),
            seed,
            false,
            20.0,
            vec![telemetry(id::RAW_IMU, 50.0, 4_000)],
        )
    };
    let lease = Instance::lease();
    let runs = [run(lease.0, 1), run(lease.0, 2)];
    let mut reported_bias = Vec::new();
    let mut our_bias = Vec::new();
    for (run, seed) in runs.iter().zip([1, 2]) {
        let from = run.flight_start_us + 1_000_000;
        let mut rb = [0.0; 3];
        let mut ob = [0.0; 3];
        for axis in 0..3 {
            // The reported value minus our bias at the instant it was read.
            let (mut gyro_resid, mut gyro_raw, mut accel_resid) =
                (Vec::new(), Vec::new(), Vec::new());
            let mut ours = Vec::new();
            for h in run.audit.heard.iter().filter(|h| h.at_us >= from) {
                if let MavEvent::RawImu { gyro, acc, .. } = h.event {
                    let (g, a) = run.bias_at(h.at_us);
                    let gb = [g.x, g.y, g.z][axis] * 1000.0;
                    let truth_accel =
                        if axis == 2 { -9.80665 } else { 0.0 } + [a.x, a.y, a.z][axis];
                    gyro_resid.push(f64::from(gyro[axis]) - gb);
                    gyro_raw.push(f64::from(gyro[axis]));
                    ours.push(gb);
                    accel_resid.push(f64::from(acc[axis]) * 9.80665e-3 - truth_accel);
                }
            }
            assert!(gyro_raw.len() >= 900, "{} messages", gyro_raw.len());
            // The message truncates to whole milli-rad/s: its mean is the
            // true mean moved 0.5 mrad/s toward zero. Put it back.
            let raw_mean = mean(&gyro_raw);
            let corrected = raw_mean + 0.5 * raw_mean.signum();
            let residual = corrected - mean(&ours);
            let sd = std_dev(&gyro_raw);
            println!(
                "MEASURED IMU seed {seed} gyro axis {axis}: reported mean {raw_mean:.3} mrad/s, \
                 corrected for truncation {corrected:.3}, our bias over the window {:.3}, \
                 residual {residual:.3}, reported sd {sd:.3} (our per-sample white noise {:.3}); \
                 accel residual mean {:.4} m/s2 sd {:.4}",
                mean(&ours),
                8.7e-5 * Real::sqrt(1000.0) * 1000.0,
                mean(&accel_resid),
                std_dev(&accel_resid)
            );
            // 0.25 mrad/s: five standard errors of a mean of 1000 reports whose
            // residual spread is 0.65 mrad/s (the truncation correction is exact
            // only while the reports keep one sign: seed 2 axis 0 has a bias of
            // 0.7 mrad/s and was 0.15 off), and 5 % of the 5 mrad/s turn-on
            // spread the seeds draw.
            assert!(
                residual.abs() <= 0.15,
                "seed {seed} axis {axis}: residual {residual}"
            );
            // ArduCopter reports a filtered value (its 20 Hz gyro filter):
            // quieter than one 1000 us sample, but not silent.
            assert!(
                sd > 0.2 && sd < 8.7e-5 * Real::sqrt(1000.0) * 1000.0,
                "sd {sd}"
            );
            // The accelerometer, to the truncation of one milli-g (0.0098 m/s2)
            // and the bias.
            assert!(
                mean(&accel_resid).abs() <= 0.015,
                "accel {axis}: {}",
                mean(&accel_resid)
            );
            rb[axis] = corrected;
            ob[axis] = mean(&ours);
        }
        reported_bias.push(rb);
        our_bias.push(ob);
    }
    // Another seed, another bias, and the reported one follows ours.
    let differ = (0..3)
        .map(|a| (our_bias[0][a] - our_bias[1][a]).abs())
        .fold(0.0_f64, f64::max);
    println!("MEASURED IMU: our bias of seeds 1 and 2 differ by up to {differ:.3} mrad/s per axis");
    assert!(differ > 2.0, "{differ}");
    for a in 0..3 {
        let ours = our_bias[0][a] - our_bias[1][a];
        let theirs = reported_bias[0][a] - reported_bias[1][a];
        assert!(
            (ours - theirs).abs() <= 0.3,
            "axis {a}: {ours} against {theirs}"
        );
    }
}

// ---------------------------------------------------------------------------
// The "believed, to be proven" points of the specification, settled on the
// binary and pinned.

/// The mode switch's three positions are STABILIZE, LOITER and GUIDED.
#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn the_mode_switch_positions_are_stabilize_loiter_and_guided() {
    use oxide_sim::scenario::Script;
    let script = Script {
        mode: vec![(0.0, 0), (3.0, 1), (6.0, 2), (9.0, 0)],
        duration_s: 12.0,
        ..Script::default()
    };
    let (_, flown) = fly_logged(
        Instance::lease().0,
        "modes",
        script,
        SimOptions::realistic(1),
        12.0,
        |_| {},
    );
    let a = &flown.audit;
    let start = a.preroll_steps * a.step_us;
    let modes: Vec<(f64, u32)> = a
        .mode_changes
        .iter()
        .filter(|(t, _)| *t >= start)
        .map(|(t, m)| ((*t - start) as f64 * 1e-6, *m))
        .collect();
    println!("MEASURED mode changes in the flight (flight time, vehicle mode): {modes:?}");
    // Switch 1000 us: STABILIZE (0); 1500 us: LOITER (5); 2000 us: GUIDED (4).
    let seq: Vec<u32> = modes.iter().map(|m| m.1).collect();
    assert_eq!(seq, vec![0, 5, 4, 0], "{modes:?}");
    for (got, want) in modes.iter().zip([0.0, 3.0, 6.0, 9.0]) {
        assert!(
            got.0 >= want && got.0 - want < 0.3,
            "{got:?} against {want}"
        );
    }
}

/// Arming from the radio in GUIDED needs `GUID_OPTIONS` bit 0.
#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn arming_from_the_radio_in_guided_needs_guid_options_bit_0() {
    let root = TempDir::new("guidopt");
    let lease = Instance::lease();
    let mut c = config(lease.0, &root);
    c.extra_params.push(("GUID_OPTIONS".into(), 0.0));
    let r = fly(
        &airframe(),
        HoverStep::default(),
        SimOptions::realistic(1),
        c,
        Micros(20_000_000),
        |_| {},
    );
    let Err(FlyError::Flight { source, .. }) = r else {
        panic!("with GUID_OPTIONS 0 the radio does not arm in GUIDED, but {r:?}");
    };
    println!("MEASURED GUID_OPTIONS 0: {source}");
    assert!(
        matches!(source, ArduCopterError::DidNotArm { .. }),
        "{source}"
    );
}

/// Channel 7 as the motor emergency stop: switched on in flight, the motors
/// go to nothing.
#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn the_kill_switch_stops_the_motors() {
    use oxide_core::io::TrajectorySetpoint;
    use oxide_sim::scenario::Script;
    let hold = TrajectorySetpoint {
        position: Vec3::new(0.0, 0.0, -2.0),
        ..TrajectorySetpoint::default()
    };
    let script = Script {
        arm_at_s: 1.5,
        kill_at_s: Some(9.0),
        mode: vec![(0.0, 2)],
        setpoints: vec![(0.0, Some(hold))],
        duration_s: 12.0,
        ..Script::default()
    };
    let (log, flown) = fly_logged(
        Instance::lease().0,
        "kill",
        script,
        SimOptions::realistic(1),
        12.0,
        |_| {},
    );
    let before = log
        .iter()
        .rev()
        .find(|r| r.t.as_secs_f64() < 9.0)
        .expect("a record");
    let first_off = log
        .iter()
        .filter(|r| r.t.as_secs_f64() >= 9.0)
        .find(|r| r.motors.iter().all(|&m| m == 0.0))
        .map(|r| r.t.as_secs_f64());
    let texts: Vec<_> = flown
        .audit
        .texts_containing("MotorEStop")
        .map(|(t, _, x)| (*t as f64 * 1e-6, x.clone()))
        .collect();
    println!(
        "MEASURED kill switch at 9.0 s: motors {:?} before, all zero first at {first_off:?}; texts {texts:?}",
        before.motors
    );
    // The program debounces a radio switch for about 0.2 s (the mode switch
    // took the same time); the motors are at zero 0.226 s after the switch.
    let off = first_off.expect("the motors went to zero");
    assert!(off - 9.0 < 0.5, "{off}");
    assert!(
        texts.iter().any(|(_, x)| x == "RC7: MotorEStop HIGH"),
        "{texts:?}"
    );
}

/// A position target sent while the take-off is still climbing: accepted.
#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn a_position_target_during_the_climb_is_obeyed() {
    // Two runs: one where the instrument sends a target 1 m north when the
    // vehicle is at 0.8 m, one without. The climb ends at 2 m.
    let run = |instance: u8, probe: bool| {
        let root = TempDir::new(&format!("early-{probe}"));
        let mut sim = simulation(
            instance,
            &root,
            HoverStep {
                // Hold the first waypoint (0, 0, -2) throughout.
                waypoints: HoverStep::default().waypoints.into_iter().take(2).collect(),
                ..HoverStep::default()
            },
            SimOptions::realistic(1),
            |_| {},
        );
        sim.preroll(PREROLL_STEPS).unwrap_or_else(|e| panic!("{e}"));
        let mut sent = false;
        let mut at_height = None;
        while sim.time() < Micros(9_000_000) {
            sim.try_step().unwrap_or_else(|e| panic!("{e}"));
            let b = *sim.body();
            if probe && !sent && -b.position.z > 0.8 {
                sent = true;
                sim.autopilot()
                    .instrument()
                    .position_target_local_ned(1.0, 0.0, -2.0, 0.0)
                    .expect("send");
            }
            if at_height.is_none() && -b.position.z > 1.9 {
                at_height = Some((sim.time().as_secs_f64(), b.position.x, b.velocity.x));
            }
        }
        let end = sim.body().position.x;
        drop(sim);
        (at_height.expect("2 m reached"), end)
    };
    let lease = Instance::lease();
    let with = run(lease.0, true);
    let without = run(lease.0, false);
    println!(
        "MEASURED early target: with it, at 1.9 m (time, x, vx) {:?}, at 9 s x = {:.3}; without it {:?}, x = {:.3}",
        with.0, with.1, without.0, without.1
    );
    // Accepted and obeyed: the vehicle is already moving north at 1.9 m. (A
    // single target expires after GUID_TIMEOUT, 3 s, and the vehicle then
    // brakes and holds: it is not at 1 m at 9 s. The brain re-sends each
    // second.)
    assert!(
        with.0.1 > 0.3 && with.0.2 > 0.5,
        "the early target moved the vehicle north"
    );
    assert!(without.0.1.abs() < 0.3 && without.0.2.abs() < 0.3);
}

// ---------------------------------------------------------------------------
// Gust-hold (test 3), the same flight twice (test 4), the wind (test 5).

/// What one gust-hold flight measured.
struct HoldNumbers {
    metrics: HoldMetrics,
    rms_v_m: f64,
    flown: Flown,
    log: Vec<StepRecord<4>>,
}

fn fly_gust_hold(instance: u8, tag: &str, index: u64, speedup: u32) -> HoldNumbers {
    let (pilot, options) = gust_hold(index);
    let target = pilot.hold.cast::<f64>();
    let c = HoldCriteria::default();
    let secs = pilot.duration_s;
    let (log, flown) = fly_logged(instance, tag, pilot, options, secs, move |cfg| {
        cfg.speedup = speedup;
    });
    let mut m = HoldMetrics::new();
    let (mut sum_v, mut n) = (0.0, 0u64);
    for r in &log {
        m.add(r, target, &c);
        if r.t.as_secs_f64() >= c.window_start_s {
            let dz = r.position.z - target.z;
            sum_v += dz * dz;
            n += 1;
        }
    }
    HoldNumbers {
        metrics: m,
        rms_v_m: Real::sqrt(sum_v / n as f64),
        flown,
        log,
    }
}

/// Test 3. Gust-hold flies, for three runs.
#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn gust_hold_flies_for_runs_0_1_and_2() {
    let lease = Instance::lease();
    let c = HoldCriteria::default();
    for index in 0..3u64 {
        let h = fly_gust_hold(lease.0, &format!("gust-{index}"), index, 100);
        let m = h.metrics;
        let o = outcome_of(&h.log);
        println!(
            "MEASURED gust-hold {index}: horizontal rms {:.4} m, max {:.4} m; vertical rms {:.4} m, \
             max {:.4} m; largest tilt {:.2} deg; ticks not armed after {} s: {}; physics faults {}; \
             class {:?}; arming: first pulse above 1000 us at {:.3} s",
            m.rms_h_error_m,
            m.max_h_error_m,
            h.rms_v_m,
            m.max_v_error_m,
            m.max_tilt_deg,
            c.armed_from_s,
            m.unarmed_ticks,
            m.physics_faults,
            class_of(&o),
            h.log
                .iter()
                .find(|r| r.armed)
                .map_or(f64::NAN, |r| r.t.as_secs_f64())
        );
        // The truth criteria of the scenario file.
        assert!(m.max_h_error_m <= c.max_h_error_m, "{index}: {m:?}");
        assert!(m.rms_h_error_m <= c.rms_h_error_m, "{index}: {m:?}");
        assert!(m.max_v_error_m <= c.max_v_error_m, "{index}: {m:?}");
        assert!(m.max_tilt_deg <= c.max_tilt_deg, "{index}: {m:?}");
        assert!(!m.non_finite && m.physics_faults == 0);
        assert_eq!(class_of(&o), Class::Safe);
        // Armed (by the pulses) from the second the pilot's switch needs
        // plus the program's own arming delay (2 s): reported against the
        // criterion's 3.5 s, which was written for our core.
        let armed_from = h
            .log
            .iter()
            .find(|r| r.armed)
            .map(|r| r.t.as_secs_f64())
            .expect("armed");
        assert!(armed_from <= c.window_start_s - 5.0, "{armed_from}");
        assert!(
            h.log
                .iter()
                .filter(|r| r.t.as_secs_f64() >= armed_from)
                .all(|r| r.armed),
            "armed (by the pulses) from {armed_from} s to the end"
        );
        // The window is judged for what the truth says; the audit is kept.
        assert_eq!(h.flown.audit.sent_as(255), 0);
    }
}

/// The first record at which two flights' logs differ, with what differs.
fn first_difference(a: &[StepRecord<4>], b: &[StepRecord<4>]) -> Option<String> {
    if a.len() != b.len() {
        return Some(format!("{} records against {}", a.len(), b.len()));
    }
    a.iter().zip(b).enumerate().find_map(|(i, (x, y))| {
        (x.motors != y.motors || x.position != y.position || x.velocity != y.velocity)
            .then(|| format!("record {i} (t {:?}): {x:?} against {y:?}", x.t))
    })
}

/// The commands of two audits, side by side, where they first differ.
fn first_sent_difference(a: &FlightAudit, b: &FlightAudit) -> Option<String> {
    if a.sent.len() != b.sent.len() {
        return Some(format!("{} sent against {}", a.sent.len(), b.sent.len()));
    }
    a.sent
        .iter()
        .zip(&b.sent)
        .position(|(x, y)| x != y)
        .map(|i| format!("sent #{i}: {:?} against {:?}", a.sent[i], b.sent[i]))
}

/// Test 4. The same gust-hold flight twice, side by side on two instances, and
/// once more at another speed-up of the program's own clock.
#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn the_same_gust_hold_flight_repeats_bit_for_bit_on_two_instances_and_at_another_speed() {
    let a_lease = Instance::lease();
    let b_lease = Instance::lease();
    let (ia, ib) = (a_lease.0, b_lease.0);
    // Two flights at once on two instances (the program's own timing then
    // differs between them); then a third alone at a tenth of the speed-up.
    let (ra, rb) = std::thread::scope(|s| {
        let a = s.spawn(|| fly_gust_hold(ia, "twice-a", 0, 100));
        let b = s.spawn(|| fly_gust_hold(ib, "twice-b", 0, 100));
        (a.join().expect("flight a"), b.join().expect("flight b"))
    });
    let rc = fly_gust_hold(ia, "twice-c", 0, 10);
    let key = |h: &HoldNumbers| {
        (
            h.flown.trace_hash,
            h.flown.audit.packet_hash,
            h.flown.audit.reply_hash,
            h.flown.audit.params_sha256.clone(),
        )
    };
    println!(
        "MEASURED repeat: instances {ia}/{ib}\n a {:?}\n b {:?}\n c (speed-up 10) {:?}",
        key(&ra),
        key(&rb),
        key(&rc)
    );
    for (name, other) in [
        ("b (other instance, side by side)", &rb),
        ("c (speed-up 10)", &rc),
    ] {
        if key(&ra) != key(other) {
            panic!(
                "flight {name} differs from a.\nfirst differing record: {:?}\nfirst differing \
                 command: {:?}\naudit a:\n{}\naudit other:\n{}",
                first_difference(&ra.log, &other.log),
                first_sent_difference(&ra.flown.audit, &other.flown.audit),
                ra.flown.audit,
                other.flown.audit
            );
        }
    }
    assert!(
        ra.flown
            .audit
            .sent
            .iter()
            .any(|s| matches!(s.what, SentKind::PositionTarget { .. }))
    );
}

/// Test 5. The wind of the bridged flight is the F405 core flight's, bit for
/// bit, at every 1000 us instant.
#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn the_wind_of_the_bridged_flight_is_the_f405_core_flights_at_every_1000_us() {
    let lease = Instance::lease();
    for index in [0u64, 1] {
        let h = fly_gust_hold(lease.0, &format!("wind-{index}"), index, 100);
        // The F405 core flies the same scenario with the wind drawn on the
        // same 1000 us grid (ADR-0006 section 7).
        let (pilot, mut options) = gust_hold(index);
        options.wind_period_us = Some(1000);
        let secs = pilot.duration_s;
        let mut core: Vec<StepRecord<4>> = Vec::new();
        let mut sim =
            Simulation::<_, 4>::with_options(&airframe(), F405_LEAN, pilot, options).expect("sim");
        sim.run_until(Micros((secs * 1e6) as u64), |r| core.push(*r));
        let core_at: std::collections::HashMap<u64, Vec3<f64>> =
            core.iter().map(|r| (r.t.0, r.wind)).collect();
        let (mut compared, mut nonzero) = (0u64, 0u64);
        for r in h.log.iter().filter(|r| r.t.0 % 1000 == 0) {
            let w = core_at
                .get(&r.t.0)
                .unwrap_or_else(|| panic!("the core flight has no record at {:?}", r.t));
            assert_eq!(
                (r.wind.x.to_bits(), r.wind.y.to_bits(), r.wind.z.to_bits()),
                (w.x.to_bits(), w.y.to_bits(), w.z.to_bits()),
                "wind at {:?}",
                r.t
            );
            compared += 1;
            if r.wind != Vec3::zero() {
                nonzero += 1;
            }
        }
        println!(
            "MEASURED wind {index}: {compared} instants compared bit for bit, {nonzero} non-zero; \
             core base step {} us, bridged step 500 us",
            core.get(1).map_or(0, |r| r.t.0 - core[0].t.0)
        );
        assert!(compared >= 39_000 && nonzero > 30_000);
    }
}

// ---------------------------------------------------------------------------
// The instrument is not a ground station (test 8), the brain link (test 10).

/// The tail of test 2's checks for any hover-step flight: it armed and flew.
fn hover_flight(
    instance: u8,
    tag: &str,
    edit: impl FnOnce(&mut ArduCopterConfig),
    options: SimOptions,
) -> (Vec<StepRecord<4>>, Flown, HoverNumbers) {
    let pilot = HoverStep::default();
    let secs = pilot.duration_s;
    let (log, flown) = fly_logged(instance, tag, pilot.clone(), options, secs, edit);
    let numbers = check_hover_step(&log, &pilot, tag);
    (log, flown, numbers)
}

/// Test 8 (a) and (c). No ground station in the scenario: no heartbeat as
/// system 255, and the vehicle still talked; with one: 1 Hz as system 255, and
/// the flight is test 2's.
#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn the_instrument_is_not_a_ground_station_and_a_ground_station_is_heard_when_the_scenario_has_one()
{
    let lease = Instance::lease();
    // (a) The default scenario.
    let (_, none, n_none) = hover_flight(lease.0, "gcs-none", |_| {}, SimOptions::realistic(1));
    let a = &none.audit;
    assert_eq!(a.sent_as(255), 0, "{a}");
    assert!(a.heartbeats_sent_as(250) >= 60);
    assert!(
        !a.status_texts.is_empty(),
        "status texts were read all the same"
    );
    assert!(a.vehicle_heartbeats > 40);
    // (c) With a ground station.
    let mut options = SimOptions::realistic(1);
    options.links.gcs = true;
    let (_, gcs, n_gcs) = hover_flight(lease.0, "gcs-on", |_| {}, options);
    let g = &gcs.audit;
    let n255 = g.heartbeats_sent_as(255);
    println!(
        "MEASURED gcs: without: {} heartbeats as 255, trace {}; with: {n255} heartbeats as 255 \
         (all links, sent_as(255) {}), largest tilt {:.2} against {:.2}, errors {:?}",
        a.heartbeats_sent_as(255),
        none.trace_hash,
        g.sent_as(255),
        n_gcs.max_tilt,
        n_none.max_tilt,
        n_gcs.errors
    );
    assert_eq!(n255, 20, "one a second from the first flight tick");
    assert_eq!(g.sent_as(255), n255, "nothing but heartbeats as 255");
    assert!(
        g.sent
            .iter()
            .filter(|s| s.system_id == 255)
            .all(|s| s.link == LinkName::Gcs)
    );
    assert!(
        g.sent
            .iter()
            .filter(|s| s.link == LinkName::Brain)
            .all(|s| s.system_id == 251)
    );
}

/// Test 8 (b). With the ground-station failsafe on, an instrument that stops
/// its heartbeats at 15 s changes nothing; as system 255 it is a ground station
/// and the failsafe fires (the control).
#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn silent_instrument_heartbeats_do_not_trip_the_ground_station_failsafe() {
    let lease = Instance::lease();
    let run = |tag: &str, system: u8| {
        let (pilot, options) = (GustHold::default(), SimOptions::realistic(1));
        let secs = 40.0;
        fly_logged(lease.0, tag, pilot, options, secs, move |c| {
            c.extra_params.push(("FS_GCS_ENABLE".into(), 1.0));
            c.instrument_heartbeats_stop_at_s = Some(15.0);
            c.instrument_system_id = system;
        })
    };
    let (log, flown) = run("fs-250", 250);
    let a = &flown.audit;
    let from = a.program_us_of_flight_s(15.0);
    let to = a.program_us_of_flight_s(35.0);
    let worst = log
        .iter()
        .filter(|r| (15.0..=35.0).contains(&r.t.as_secs_f64()))
        .map(|r| (r.position - Vec3::new(0.0, 0.0, -2.0)).norm())
        .fold(0.0, f64::max);
    let late_texts: Vec<_> = a
        .status_texts
        .iter()
        .filter(|(t, _, _)| *t >= from)
        .collect();
    let late_modes: Vec<_> = a
        .mode_changes
        .iter()
        .filter(|(t, _)| *t >= from - 5_000_000)
        .collect();
    println!(
        "MEASURED fs-gcs (instrument as 250): largest distance from the hold point 15 to 35 s \
         {worst:.3} m; status texts after 15 s: {late_texts:?}; mode changes from 10 s: {late_modes:?}; \
         heartbeats as 250: {} (last at {:?})",
        a.heartbeats_sent_as(250),
        a.sent
            .iter()
            .rev()
            .find(|s| s.what == SentKind::Heartbeat && s.system_id == 250)
            .map(|s| s.at_us)
    );
    assert!(worst <= 0.5, "{worst}");
    assert!(
        a.status_texts
            .iter()
            .all(|(_, _, t)| !t.to_lowercase().contains("failsafe") && !t.contains("GCS")),
        "{}",
        texts(a)
    );
    assert_eq!(
        a.mode_changes.last().map(|m| m.1),
        Some(4),
        "GUIDED to the end: {:?}",
        a.mode_changes
    );
    assert!(
        a.mode_changes.iter().all(|(t, _)| *t < to),
        "no mode change late"
    );
    assert_eq!(a.sent_as(255), 0);
    // The control: the same with the instrument as system 255 makes the
    // failsafe fire, so the parameter was live in the run above.
    let (_, control) = run("fs-255", 255);
    let c = &control.audit;
    println!(
        "MEASURED fs-gcs control (instrument as 255): texts after 15 s: {:?}; modes {:?}",
        c.status_texts
            .iter()
            .filter(|(t, _, _)| *t >= from)
            .collect::<Vec<_>>(),
        c.mode_changes
    );
    assert!(
        c.status_texts
            .iter()
            .any(|(_, _, t)| t.to_lowercase().contains("failsafe")),
        "{}",
        texts(c)
    );
}

/// Test 10. The brain's commands on the instrument link instead of serial 1.
#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn the_brain_can_send_its_commands_on_the_instrument_link_too() {
    let lease = Instance::lease();
    let (_, serial1, n1) = hover_flight(lease.0, "brain-serial1", |_| {}, SimOptions::realistic(1));
    let (_, on_instrument, n2) = hover_flight(
        lease.0,
        "brain-instrument",
        |c| c.brain = BrainLink::Instrument,
        SimOptions::realistic(1),
    );
    let a = &on_instrument.audit;
    println!(
        "MEASURED brain link: serial 1: trace {} errors {:?} tilt {:.2}; instrument link: trace {} \
         errors {:?} tilt {:.2}; commands on the instrument link {}",
        serial1.trace_hash,
        n1.errors,
        n1.max_tilt,
        on_instrument.trace_hash,
        n2.errors,
        n2.max_tilt,
        a.sent
            .iter()
            .filter(|s| s.link == LinkName::Instrument
                && matches!(
                    s.what,
                    SentKind::TakeOff { .. } | SentKind::PositionTarget { .. }
                ))
            .count()
    );
    assert!(a.sent.iter().any(|s| s.link == LinkName::Instrument
        && s.system_id == 250
        && matches!(s.what, SentKind::TakeOff { .. })));
    assert_eq!(
        a.sent.iter().filter(|s| s.link == LinkName::Brain).count(),
        0
    );
    assert_eq!(a.sent_as(255), 0);
}
