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
use std::time::Instant;

use oxide_bridge::arducopter::{
    ArduCopter, ArduCopterConfig, ArduCopterError, FlightAudit, Flown, FlyError, PREROLL_STEPS,
    TelemetryRequest, bridged_options, fly,
};
use oxide_bridge::mav::{MavEvent, id};
use oxide_bridge::sitl::PINNED;
use oxide_core::time::Micros;
use oxide_sim::outcome::FlightOutcome;
use oxide_sim::scenario::{HoverStep, Pilot};
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
    let mut sim = simulation(
        0,
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
    let (log, flown) = fly_logged(
        1,
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
    let mut c = config(2, &root);
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
