// SPDX-License-Identifier: AGPL-3.0-or-later
//! The JSON bridge against a stand-in for the simulator build: a real UDP
//! exchange on the local machine, in lockstep, through the simulator's
//! autopilot seam.
//!
//! Every test binds port 0 and hands the real port to the stand-in, so the
//! tests run in parallel and on any machine.

#[path = "support/fake_sitl.rs"]
mod fake_sitl;

use std::time::{Duration, Instant};

use fake_sitl::{FakeSitl, Options, Report};
use oxide_bridge::bridge::{BridgeConfig, BridgeError, JsonBridge};
use oxide_bridge::link::{Link, LinkError};
use oxide_bridge::reply::ReplyError;
use oxide_core::io::RcSample;
use oxide_core::math::Vec3;
use oxide_core::time::Micros;
use oxide_sim::scenario::{NoPilot, Pilot};
use oxide_sim::vehicle::BodyState;
use oxide_sim::{Airframe, SimOptions, Simulation, StepRecord};

const STEP_US: u64 = 500;

fn airframe() -> Airframe {
    Airframe::reference_quad().expect("reference airframe")
}

fn config(timeout: Duration) -> BridgeConfig {
    let mut c = BridgeConfig::new(STEP_US);
    c.reply_timeout = timeout;
    c.first_packet_timeout = Duration::from_secs(20);
    c
}

/// A bridge on a fresh socket and a stand-in aimed at it.
fn rig(sitl: Options, timeout: Duration) -> (JsonBridge, FakeSitl) {
    rig_with(sitl, config(timeout))
}

/// The same with a given bridge configuration.
fn rig_with(mut sitl: Options, config: BridgeConfig) -> (JsonBridge, FakeSitl) {
    let link = Link::bind(0).expect("bind");
    sitl.bridge_port = link.local_port();
    let bridge = JsonBridge::new(link, config).expect("config");
    (bridge, FakeSitl::spawn(sitl))
}

/// No reply broke the wire format, as the strict stand-in reads it.
fn assert_well_formed(report: &Report) {
    assert_eq!(
        report.rejected, 0,
        "{} replies broke the wire format; the first: {:?}",
        report.rejected, report.first_rejection
    );
}

fn simulation<P: Pilot>(bridge: JsonBridge, pilot: P) -> Simulation<P, 4, JsonBridge> {
    Simulation::with_autopilot(&airframe(), bridge, pilot, SimOptions::default()).expect("builds")
}

struct Flight {
    records: Vec<StepRecord<4>>,
    hash: u64,
    report: Report,
}

/// Flies `seconds` against a stand-in with these options.
fn fly(sitl: Options, seconds: u64) -> Flight {
    let (bridge, sitl) = rig(sitl, Duration::from_secs(10));
    let mut sim = simulation(bridge, NoPilot);
    let mut records = Vec::new();
    sim.try_run_until(Micros::from_secs(seconds), |r| records.push(*r))
        .expect("the flight does not fail");
    let hash = sim.trace_hash();
    drop(sim);
    Flight {
        records,
        hash,
        report: sitl.finish(),
    }
}

fn height(r: &StepRecord<4>) -> f64 {
    -r.position.z
}

/// The text between `key` and the next `,` in a reply line.
fn timestamp_of(line: &str) -> &str {
    let key = "\"timestamp\":";
    let start = line.find(key).expect("a timestamp") + key.len();
    let rest = &line[start..];
    &rest[..rest.find(',').expect("a comma")]
}

#[test]
fn a_flight_of_5_s_against_the_stand_in_leaves_the_ground_and_holds_the_height() {
    let o = Options::new(0);
    let target = o.target_height_m;
    let f = fly(o, 5);
    assert_eq!(f.records.len(), 10_000, "5 s at 500 us");
    let max = f.records.iter().map(height).fold(0.0, f64::max);
    assert!(max > 1.0, "the vehicle left the ground: max height {max}");
    for r in f.records.iter().filter(|r| r.t.as_secs_f64() >= 4.0) {
        let h = height(r);
        assert!(
            (h - target).abs() < 0.5,
            "{:.2} s: height {h:.3} m is not within 0.5 m of {target} m",
            r.t.as_secs_f64()
        );
    }
    // The armed flag is "a pulse above 1000": from the moment the stand-in arms.
    assert!(f.records.iter().any(|r| r.armed));
    assert!(
        f.records.iter().take(900).all(|r| !r.armed),
        "disarmed before 0.5 s"
    );
    // The command acts at once: the tick that first commands thrust shows the
    // rotors turning in the very next record.
    let first = f
        .records
        .iter()
        .position(|r| r.motors[0] > 0.0)
        .expect("a motor was commanded");
    assert_eq!(f.records[first].rotor_speed[0], 0.0);
    assert!(
        f.records[first + 1].rotor_speed[0] > 0.0,
        "the command of tick {first} acted over that step"
    );
    // One reply per frame, each framed by newlines.
    assert_eq!(f.report.replies.len(), f.records.len());
    assert_eq!(f.report.resends, 0);
    assert_well_formed(&f.report);
    for (i, line) in f.report.replies.iter().enumerate() {
        assert!(
            line.starts_with('\n') && line.ends_with("}\n"),
            "reply {i}: {line:?}"
        );
        assert!(
            line.ends_with("\n") && line.matches('\n').count() == 2,
            "reply {i}"
        );
    }
}

#[test]
fn the_same_flight_twice_hashes_the_same_and_another_target_does_not() {
    let a = fly(Options::new(0), 5);
    let b = fly(Options::new(0), 5);
    assert_eq!(a.hash, b.hash, "the same flight twice");
    assert_eq!(a.report.replies, b.report.replies, "and the same replies");
    let mut other = Options::new(0);
    other.target_height_m = 2.5;
    let c = fly(other, 5);
    assert_ne!(a.hash, c.hash, "another target flies another flight");
}

#[test]
fn the_time_stamps_the_stand_in_received_are_k_plus_1_times_500_001_ns_as_text() {
    let f = fly(Options::new(0), 3);
    assert_eq!(f.report.replies.len(), 6_000);
    for (k, line) in f.report.replies.iter().enumerate() {
        let ns = (k as u64 + 1) * 500_001;
        let want = format!("{}.{:09}", ns / 1_000_000_000, ns % 1_000_000_000);
        assert_eq!(timestamp_of(line), want, "frame {k}");
    }
    // The first reply carries one step, not zero (measured on the real
    // program: a first stamp of zero does not repeat).
    assert_eq!(timestamp_of(&f.report.replies[0]), "0.000500001");
    assert_eq!(timestamp_of(&f.report.replies[1]), "0.001000002");
    assert_eq!(timestamp_of(&f.report.replies[2]), "0.001500003");
    assert_eq!(timestamp_of(&f.report.replies[1_999]), "1.000002000");
}

#[test]
fn a_lost_reply_is_answered_again_and_costs_no_physics_step() {
    let base = fly(Options::new(0), 5);
    let mut o = Options::new(0);
    o.lose_reply_at = Some(1_234);
    let f = fly(o, 5);
    assert_eq!(
        f.hash, base.hash,
        "the same flight: no extra step was taken"
    );
    assert_eq!(f.records.len(), base.records.len());
    // The stand-in saw one reply more, and it is the same line twice.
    assert_eq!(f.report.replies.len(), base.report.replies.len() + 1);
    let pairs: Vec<usize> = (1..f.report.replies.len())
        .filter(|&i| f.report.replies[i] == f.report.replies[i - 1])
        .collect();
    assert_eq!(pairs.len(), 1, "exactly one repeated reply");
    assert_eq!(
        pairs[0], 1_234,
        "the reply to frame 1234 (index 1233) came twice"
    );
    // Without the repeat the replies are the base flight's.
    let mut without = f.report.replies.clone();
    without.remove(pairs[0]);
    assert_eq!(without, base.report.replies);
}

#[test]
fn a_skipped_frame_number_ends_the_flight_and_the_step_did_not_happen() {
    let mut o = Options::new(0);
    o.skip_after = Some(50);
    let (bridge, sitl) = rig(o, Duration::from_secs(10));
    let mut sim = simulation(bridge, NoPilot);
    // The reply to frame 50 is the 50th tick (index 49); its answer is frame 52.
    for _ in 0..49 {
        sim.try_step().expect("the first 49 ticks succeed");
    }
    let (time, hash, body) = (sim.time(), sim.trace_hash(), *sim.body());
    let err = sim.try_step().expect_err("the skip is an error");
    match &err {
        BridgeError::Link {
            step,
            source: LinkError::FrameCount { expected, found },
        } => assert_eq!((*step, *expected, *found), (49, 51, 52)),
        other => panic!("{other:?}"),
    }
    let text = err.to_string();
    assert!(text.contains("51") && text.contains("52"), "{text}");
    assert_eq!(sim.time(), time, "the tick counter did not advance");
    assert_eq!(sim.time(), Micros(49 * STEP_US));
    assert_eq!(sim.trace_hash(), hash);
    assert_eq!(*sim.body(), body);
    drop(sim);
    let report = sitl.finish();
    assert_eq!(report.replies.len(), 50, "the reply for frame 50 was sent");
}

#[test]
fn a_stand_in_that_stops_answering_is_a_timeout_not_a_hang() {
    let mut o = Options::new(0);
    o.stop_after = Some(100);
    let (bridge, sitl) = rig(o, Duration::from_millis(200));
    let mut sim = simulation(bridge, NoPilot);
    for _ in 0..99 {
        sim.try_step().expect("the first 99 ticks succeed");
    }
    let t0 = Instant::now();
    let err = sim.try_step().expect_err("no packet 101");
    let took = t0.elapsed();
    match &err {
        BridgeError::Link {
            step: 99,
            source: LinkError::Timeout { frame: 101, .. },
        } => {}
        other => panic!("{other:?}"),
    }
    assert!(err.to_string().contains("101"), "{err}");
    assert!(
        took >= Duration::from_millis(150) && took < Duration::from_secs(3),
        "{took:?}"
    );
    assert_eq!(sim.time(), Micros(99 * STEP_US), "the tick did not happen");
    drop(sim);
    assert_eq!(sitl.finish().replies.len(), 100);
}

#[test]
fn the_stand_in_that_needs_the_leading_newline_accepts_every_reply() {
    let mut o = Options::new(0);
    o.require_leading_newline = true;
    let f = fly(o, 1);
    assert_eq!(f.report.ignored, 0, "none was ignored");
    assert_well_formed(&f.report);
    assert_eq!(f.report.replies.len(), 2_000);
    assert_eq!(f.report.resends, 0, "and so none had to be sent again");
    assert!(f.records.iter().any(|r| r.armed), "the flight ran");
}

#[test]
fn the_stand_in_ignores_a_reply_without_the_leading_newline() {
    // The check that the stand-in is as strict as the real thing: without the
    // newline it answers nothing and sends its packet again.
    let link = Link::bind(0).expect("bind");
    let mut o = Options::new(link.local_port());
    o.resend_after = Duration::from_millis(50);
    let mut link = link;
    let sitl = FakeSitl::spawn(o);
    let bad = "{\"timestamp\":0.000000000}\n";
    let e = link
        .exchange(bad, Duration::from_millis(400))
        .expect_err("it never answers");
    assert!(matches!(e, LinkError::Timeout { frame: 2, .. }), "{e}");
    let report = sitl.finish();
    assert!(
        report.ignored >= 2,
        "it ignored the reply and each resend of it: {report:?}"
    );
    assert!(report.resends >= 2, "{report:?}");
    assert!(report.replies.is_empty());
}

/// Sticks for the first 2 s, then a lost radio: no frames at all.
struct RadioFor2s;

impl Pilot for RadioFor2s {
    fn rc(&mut self, t: Micros) -> Option<RcSample> {
        (t < Micros::from_secs(2) && t.0.is_multiple_of(20_000)).then_some(RcSample {
            roll: 0.5,
            pitch: -0.25,
            yaw: 0.0,
            throttle: 0.6,
            arm_switch: true,
            kill_switch: false,
            mode_switch: 1,
            failsafe: false,
            frame_lost: false,
        })
    }
    fn setpoint(&mut self, _t: Micros) -> Option<oxide_core::io::TrajectorySetpoint> {
        None
    }
}

#[test]
fn the_radio_is_sent_as_pulses_until_it_is_lost_and_then_not_at_all() {
    let (bridge, sitl) = rig(Options::new(0), Duration::from_secs(10));
    let mut sim = simulation(bridge, RadioFor2s);
    sim.try_run_until(Micros::from_secs(3), |_| {})
        .expect("the flight does not fail");
    drop(sim);
    let report = sitl.finish();
    assert_well_formed(&report);
    assert_eq!(report.replies.len(), 6_000);
    let rc = "\"rc\":{\"rc_1\":1750,\"rc_2\":1375,\"rc_3\":1600,\"rc_4\":1500,\
              \"rc_5\":1500,\"rc_6\":2000,\"rc_7\":1000,\"rc_8\":1000}}";
    for (k, line) in report.replies.iter().enumerate() {
        let t_us = k as u64 * STEP_US;
        if t_us <= 2_000_000 {
            assert!(line.contains(rc), "t = {t_us} us: {line}");
            assert!(line.ends_with(&format!("{rc}\n")), "rc is the last section");
        } else {
            assert!(!line.contains("\"rc\""), "t = {t_us} us: {line}");
        }
        assert!(
            line.contains("\"battery\":{\"voltage\":"),
            "the battery is always sent"
        );
    }
}

#[test]
fn a_non_finite_truth_ends_the_flight_with_the_field_named_and_sends_nothing() {
    let (bridge, sitl) = rig(Options::new(0), Duration::from_secs(10));
    let mut sim = simulation(bridge, NoPilot);
    for _ in 0..10 {
        sim.try_step().expect("the first ticks succeed");
    }
    let (time, hash) = (sim.time(), sim.trace_hash());
    let body = BodyState::<4> {
        velocity: Vec3::new(0.0, f64::NAN, 0.0),
        ..BodyState::default()
    };
    sim.set_body(body);
    let err = sim.try_step().expect_err("a NaN is refused");
    match &err {
        BridgeError::Reply {
            step: 10,
            source: ReplyError::NonFinite { field: "velocity" },
        } => {}
        other => panic!("{other:?}"),
    }
    assert!(err.to_string().contains("velocity"), "{err}");
    assert_eq!(sim.time(), time);
    assert_eq!(sim.trace_hash(), hash);
    drop(sim);
    let report = sitl.finish();
    assert_eq!(report.replies.len(), 10, "nothing was sent for that tick");
}

#[test]
fn without_the_radio_and_the_battery_options_no_reply_has_either_section() {
    // The pilot sends frames for 2 s; with `send_rc` off none of them reaches
    // the reply, and with `send_battery` off the battery is left out too.
    let mut c = config(Duration::from_secs(10));
    c.send_rc = false;
    c.send_battery = false;
    let (bridge, sitl) = rig_with(Options::new(0), c);
    let mut sim = simulation(bridge, RadioFor2s);
    sim.try_run_until(Micros::from_secs(3), |_| {})
        .expect("the flight does not fail");
    drop(sim);
    let report = sitl.finish();
    assert_well_formed(&report);
    assert_eq!(report.replies.len(), 6_000);
    for (k, line) in report.replies.iter().enumerate() {
        assert!(!line.contains("\"rc\""), "frame {k}: {line}");
        assert!(!line.contains("\"battery\""), "frame {k}: {line}");
        assert!(line.ends_with("]}\n"), "the line ends after the velocity");
    }
    // And the options are independent: the radio alone, the battery alone.
    for (rc, battery) in [(true, false), (false, true)] {
        let mut c = config(Duration::from_secs(10));
        c.send_rc = rc;
        c.send_battery = battery;
        let (bridge, sitl) = rig_with(Options::new(0), c);
        let mut sim = simulation(bridge, RadioFor2s);
        sim.try_run_until(Micros::from_secs(1), |_| {})
            .expect("the flight does not fail");
        drop(sim);
        let report = sitl.finish();
        assert_well_formed(&report);
        for line in &report.replies {
            assert_eq!(line.contains("\"rc\""), rc, "{line}");
            assert_eq!(line.contains("\"battery\""), battery, "{line}");
        }
    }
}

#[test]
fn a_slow_start_up_is_waited_for_by_the_first_packet_time_out_not_the_reply_one() {
    // The stand-in sends its first packet after 300 ms; the reply time-out is
    // 100 ms and the first-packet time-out 5 s. The flight must succeed.
    let mut o = Options::new(0);
    o.start_delay = Duration::from_millis(300);
    let mut c = config(Duration::from_millis(100));
    c.first_packet_timeout = Duration::from_secs(5);
    let (bridge, sitl) = rig_with(o, c);
    let mut sim = simulation(bridge, NoPilot);
    let t0 = Instant::now();
    sim.try_run_until(Micros::from_secs(1), |_| {})
        .expect("the start-up is not a time-out");
    assert!(t0.elapsed() >= Duration::from_millis(250), "it did wait");
    drop(sim);
    let report = sitl.finish();
    assert_well_formed(&report);
    assert_eq!(report.replies.len(), 2_000);
}

#[test]
fn a_simulator_that_never_starts_is_the_first_packet_time_out_within_that_time() {
    // Nothing ever comes: the error is the first-packet one, after about the
    // first-packet time-out (400 ms here), not the reply time-out (50 ms).
    let link = Link::bind(0).expect("bind");
    let mut c = config(Duration::from_millis(50));
    c.first_packet_timeout = Duration::from_millis(400);
    let bridge = JsonBridge::new(link, c).expect("config");
    let mut sim = simulation(bridge, NoPilot);
    let t0 = Instant::now();
    let err = sim.try_step().expect_err("nobody is there");
    let took = t0.elapsed();
    match &err {
        BridgeError::Link {
            step: 0,
            source: LinkError::FirstPacketTimeout { waited },
        } => assert_eq!(*waited, Duration::from_millis(400)),
        other => panic!("{other:?}"),
    }
    assert!(
        took >= Duration::from_millis(300) && took < Duration::from_secs(5),
        "{took:?}"
    );
    assert_eq!(sim.time(), Micros(0), "the tick did not happen");
}
