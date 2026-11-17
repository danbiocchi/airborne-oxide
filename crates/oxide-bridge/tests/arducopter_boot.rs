// SPDX-License-Identifier: AGPL-3.0-or-later
//! The real ArduCopter simulator program under our exchange (ADR-0006).
//!
//! Every test here needs the binary and is ignored by default: set
//! `OXIDE_ARDUCOPTER` to its path and run with `--ignored`. With the
//! variable unset an ignored test that is run fails and says so; it never
//! passes silently. Each test has its own instance number (so they can run
//! side by side), its own directory under the system's temporary directory,
//! and leaves no process behind: the child is ended by `Drop` on every way
//! out, a panic included.

#[path = "support/rest.rs"]
mod rest;

use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use oxide_bridge::link::{Link, LinkError};
use oxide_bridge::mav::{MavClient, MavEvent, id};
use oxide_bridge::sitl::{PINNED, Sitl, SitlConfig, SitlError, json_port_of};
use rest::{Rest, RestError};

const BASE_PARAMS: &str = "FRAME_CLASS 1\nFRAME_TYPE 1\nLOG_BACKEND_TYPE 0\n";
const REPLY_TIMEOUT: Duration = Duration::from_secs(5);
const LONG: Duration = Duration::from_secs(20);

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
        let p = std::env::temp_dir().join(format!(
            "oxide-arducopter-boot-{}-{tag}",
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

/// One flight: the exchange and client, the process, the directory. The
/// fields drop in this order: sockets, then the child, then the directory.
struct Flight {
    rest: Rest,
    sitl: Sitl,
    _root: TempDir,
}

impl Flight {
    fn start(instance: u8, tag: &str) -> Self {
        Self::start_with(instance, tag, 1, Vec::new())
    }

    fn start_with(instance: u8, tag: &str, first_k: u64, extra: Vec<(String, f64)>) -> Self {
        let bin = binary();
        let root = TempDir::new(&format!("{tag}-i{instance}"));
        let params = root.path().join("base.parm");
        std::fs::write(&params, BASE_PARAMS).expect("parameter file");
        let mut config = SitlConfig::new(bin, root.path().join("flights"));
        config.instance = instance;
        config.param_files = vec![params];
        config.extra_params = extra;
        let port = json_port_of(instance).expect("instance");
        // The socket is bound before the program starts, as it sends to it.
        let link = Link::bind(port).unwrap_or_else(|e| panic!("cannot bind UDP port {port}: {e}"));
        let sitl = Sitl::start(config).expect("start the simulator");
        let mav_port = sitl.mavlink_port(0).expect("serial 0");
        let client = MavClient::connect(mav_port, 255, 190, config_limit())
            .unwrap_or_else(|e| panic!("{e}"));
        let rest = Rest::new(link, client, first_k, REPLY_TIMEOUT, LONG)
            .unwrap_or_else(|e| panic!("first packet: {e}"));
        Self {
            rest,
            sitl,
            _root: root,
        }
    }
}

fn config_limit() -> Duration {
    LONG
}

fn is_heartbeat(e: &MavEvent) -> bool {
    matches!(e, MavEvent::Heartbeat { .. })
}

fn is_ready(e: &MavEvent) -> bool {
    matches!(e, MavEvent::StatusText { text, .. } if text.contains("ArduPilot Ready"))
}

fn is_gps(e: &MavEvent) -> bool {
    matches!(e, MavEvent::GpsRaw { .. })
}

fn boot_ms(e: &MavEvent) -> Option<u32> {
    match e {
        MavEvent::SystemTime { time_boot_ms, .. } => Some(*time_boot_ms),
        _ => None,
    }
}

/// The motor pulses of every packet, all channels.
fn pulses(r: &Rest) -> Vec<(u32, [u16; 32])> {
    r.packets.iter().map(|p| (p.frame_count, p.pwm)).collect()
}

#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn boot_at_rest() {
    let mut f = Flight::start(0, "boot");
    let wall = Instant::now();
    f.rest.run_s(5.0).expect("5 simulated seconds");
    let wall = wall.elapsed();
    let r = &f.rest;
    let hb = r.first_at(0, is_heartbeat).expect("a heartbeat");
    let ready = r.first_at(0, is_ready).expect("the ready text");
    println!(
        "MEASURED boot: first frame count {}; first heartbeat read at exchange {hb} ({:.4} s); \
         ready text at exchange {ready} ({:.4} s); wall time of 5 simulated seconds {:.3} s; \
         exchanges {}; MAVLink messages read {}, other counts {:?}",
        r.first_frame,
        Rest::seconds(hb),
        Rest::seconds(ready),
        wall.as_secs_f64(),
        r.exchanges,
        r.client.received(),
        r.client.other_counts()
    );
    // Not armed, in every heartbeat.
    let beats: Vec<_> = r
        .events
        .iter()
        .filter_map(|t| match &t.event {
            MavEvent::Heartbeat { armed, .. } => Some(*armed),
            _ => None,
        })
        .collect();
    assert!(!beats.is_empty() && beats.iter().all(|a| !a), "{beats:?}");
    // Disarmed pulses at the end.
    for p in r.packets.iter().rev().take(200) {
        assert_eq!(&p.pwm[..4], &[1000; 4], "frame {}", p.frame_count);
    }
    // Consecutive frame counts.
    for w in r.packets.windows(2) {
        assert_eq!(w[1].frame_count, w[0].frame_count + 1);
    }
    assert_eq!(
        r.packets.first().map(|p| p.frame_count),
        Some(r.first_frame + 1)
    );
    // Files: eeprom.bin yes, logs no.
    let dir = f.sitl.dir();
    assert!(dir.join("eeprom.bin").exists(), "{}", dir.display());
    assert!(!dir.join("logs").exists(), "{}", dir.display());
    f.sitl.check().expect("still running");
}

#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn the_pinned_checksum_is_accepted_and_reported() {
    let bin = binary();
    let root = TempDir::new("checksum");
    let size = std::fs::metadata(&bin).expect("binary").len();
    let config = SitlConfig::new(&bin, root.path().join("flights"));
    let mut config = config;
    config.instance = 1;
    let sitl = Sitl::start(config).expect("the pinned binary starts");
    assert_eq!(sitl.binary_sha256(), PINNED.sha256);
    assert_eq!(size, PINNED.size_bytes);
    // A binary with another checksum is refused unless asked for.
    let other = root.path().join("other");
    std::fs::write(&other, b"not it").expect("write");
    let mut config = SitlConfig::new(&other, root.path().join("flights"));
    config.instance = 1;
    assert!(matches!(
        Sitl::start(config),
        Err(SitlError::ChecksumMismatch { .. })
    ));
    sitl.stop().expect("stop");
}

#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn a_parameter_round_trip() {
    let mut f = Flight::start(2, "param");
    f.rest.run_s(3.0).expect("boot");
    let sent = f.rest.exchanges;
    f.rest
        .client
        .param_set("SIM_BARO_RND", 0.173)
        .expect("param_set");
    f.rest.run_s(1.0).expect("1 s");
    let is_value = |e: &MavEvent| {
        matches!(e, MavEvent::ParamValue { name, value, .. }
            if name == "SIM_BARO_RND" && (f64::from(*value) - 0.173).abs() < 1e-6)
    };
    let got = f
        .rest
        .first_at(sent, is_value)
        .expect("a PARAM_VALUE with the new value within 1 simulated second");
    println!(
        "MEASURED param_set echoed after {} exchanges ({:.4} s)",
        got - sent,
        Rest::seconds(got - sent)
    );
    // Read it back.
    let asked = f.rest.exchanges;
    f.rest
        .client
        .param_request("SIM_BARO_RND")
        .expect("param_request");
    f.rest.run_s(1.0).expect("1 s");
    let again = f
        .rest
        .first_at(asked, is_value)
        .expect("the value read back");
    println!(
        "MEASURED param_request answered after {} exchanges",
        again - asked
    );
}

/// Runs 8 simulated seconds, asking for GPS_RAW_INT at 5 Hz at second 2 when
/// `ask`; the GPS events read between seconds 3 and 8.
fn gps_events(instance: u8, tag: &str, ask: bool) -> usize {
    let mut f = Flight::start(instance, tag);
    f.rest.run_s(2.0).expect("2 s");
    if ask {
        f.rest
            .client
            .set_message_interval(id::GPS_RAW_INT, 5.0)
            .expect("interval");
    }
    f.rest.run_s(6.0).expect("to 8 s");
    f.rest.count(6000, 16000, is_gps)
}

#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn a_message_interval_sets_the_rate() {
    let with = gps_events(5, "interval", true);
    println!("MEASURED GPS_RAW_INT events between 3 s and 8 s at 5 Hz: {with}");
    assert!((23..=27).contains(&with), "{with}");
    let without = gps_events(5, "nointerval", false);
    assert_eq!(without, 0, "without the request");
}

/// The SYSTEM_TIME offset series of one run: for each event, the program's
/// boot time in half-milliseconds minus our exchange count, over 10,000
/// exchanges.
fn system_time_run(instance: u8, tag: &str) -> (u64, u64, u32, u32) {
    let mut f = Flight::start(instance, tag);
    f.rest.run_s(2.0).expect("boot");
    f.rest
        .client
        .set_message_interval(id::SYSTEM_TIME, 50.0)
        .expect("interval");
    f.rest.steps(500).expect("settle");
    f.rest.steps(10_000 + 200).expect("10,000 exchanges");
    let times: Vec<(u64, u32)> = f
        .rest
        .events
        .iter()
        .filter_map(|t| boot_ms(&t.event).map(|b| (t.exchange, b)))
        .collect();
    assert!(times.len() > 100, "{}", times.len());
    let (e0, b0) = *times.iter().find(|(e, _)| *e >= 3000).expect("first");
    let (e1, b1) = *times.iter().find(|(e, _)| *e >= e0 + 10_000).expect("last");
    // Every event's offset from our clock stays within a few milliseconds.
    let offsets: Vec<f64> = times
        .iter()
        .map(|&(e, b)| f64::from(b) - Rest::seconds(e) * 1000.0)
        .collect();
    let lo = offsets.iter().cloned().fold(f64::MAX, f64::min);
    let hi = offsets.iter().cloned().fold(f64::MIN, f64::max);
    println!(
        "MEASURED system time: {tag}: boot_ms {b0} -> {b1} over exchanges {e0} -> {e1} \
         ({} exchanges, {:.1} ms of ours); offset range {lo:.1}..{hi:.1} ms",
        e1 - e0,
        Rest::seconds(e1 - e0) * 1000.0
    );
    (e0, e1, b0, b1)
}

#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn the_simulators_clock_is_ours() {
    let mut deltas = Vec::new();
    for run in 0..2 {
        let (e0, e1, b0, b1) = system_time_run(6, &format!("clock{run}"));
        let ours = (e1 - e0) as f64 * 0.5;
        let theirs = f64::from(b1 - b0);
        assert!(
            (theirs - ours).abs() <= 3.0,
            "run {run}: {theirs} ms vs {ours} ms"
        );
        assert!(
            (e1 - e0).abs_diff(10_000) <= 40,
            "run {run}: {} exchanges between the events",
            e1 - e0
        );
        deltas.push(theirs);
        assert!((ours - 5000.0).abs() <= 20.0, "{ours}");
    }
    assert!((deltas[0] - deltas[1]).abs() <= 3.0, "{deltas:?}");
}

#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn two_at_once() {
    let run = |instance: u8| {
        std::thread::spawn(move || {
            let mut f = Flight::start(instance, "two");
            f.rest.run_s(3.0).expect("3 s");
            let r = &f.rest;
            assert!(r.first_at(0, is_heartbeat).is_some(), "instance {instance}");
            // Frame counts consecutive, and as many as exchanges.
            for w in r.packets.windows(2) {
                assert_eq!(
                    w[1].frame_count,
                    w[0].frame_count + 1,
                    "instance {instance}"
                );
            }
            assert_eq!(r.packets.len() as u64, r.exchanges);
            let last = r.packets.last().map(|p| p.frame_count);
            assert_eq!(
                last,
                Some(r.first_frame + r.exchanges as u32),
                "instance {instance}"
            );
            f.sitl.check().expect("running");
            (r.first_frame, r.exchanges)
        })
    };
    let (a, b) = (run(3), run(4));
    let a = a.join().expect("instance 3");
    let b = b.join().expect("instance 4");
    assert_eq!(a.1, b.1);
}

#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn a_dead_child_ends_the_exchange_within_the_time_out() {
    let started = Instant::now();
    let mut f = Flight::start(7, "dead");
    f.rest.reply_timeout = Duration::from_secs(2);
    f.rest.run_s(1.0).expect("1 s");
    f.sitl.check().expect("running");
    let pid = f.sitl.pid().expect("pid");
    let status = std::process::Command::new("kill")
        .args(["-9", &pid.to_string()])
        .status()
        .expect("run kill");
    assert!(status.success(), "{status}");
    let before = Instant::now();
    let e = f
        .rest
        .steps(10_000)
        .expect_err("the exchange must end with an error");
    let took = before.elapsed();
    println!("MEASURED dead child: error after {took:?}: {e}");
    assert!(
        matches!(e, RestError::Link(LinkError::Timeout { .. })) || matches!(e, RestError::Mav(_)),
        "{e}"
    );
    assert!(took < Duration::from_millis(2500), "{took:?}");
    let c = f.sitl.check().expect_err("the child has ended");
    assert!(matches!(c, SitlError::Exited { .. }), "{c}");
    println!("MEASURED dead child: check says {c}");
    drop(f);
    assert!(
        started.elapsed() < Duration::from_secs(10),
        "{:?}",
        started.elapsed()
    );
}

#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn rest_runs_repeat_exactly() {
    let run = |tag: &str| {
        let mut f = Flight::start(8, tag);
        f.rest.run_s(5.0).expect("5 s");
        (
            f.rest.first_frame,
            pulses(&f.rest),
            f.sitl.params_sha256().to_string(),
        )
    };
    let a = run("repeat-a");
    let b = run("repeat-b");
    assert_eq!(a.0, b.0, "first frame counts");
    assert_eq!(a.2, b.2, "parameter files");
    assert_eq!(a.1.len(), b.1.len());
    let first_diff = a.1.iter().zip(&b.1).position(|(x, y)| x != y);
    if let Some(i) = first_diff {
        let lo = i.saturating_sub(2);
        for k in lo..(i + 6).min(a.1.len()) {
            println!("DIFF {k}: a {:?} b {:?}", &a.1[k].1[..8], &b.1[k].1[..8]);
        }
    }
    assert_eq!(first_diff, None, "first differing packet");
}

/// A rest run of 5 simulated seconds that starts with step `first_k`, asking
/// for SYSTEM_TIME at 50 Hz at second 2: the motor pulses, and the offset of
/// the program's clock from ours (ms) at the last SYSTEM_TIME.
fn stamp_run(instance: u8, tag: &str, first_k: u64) -> (Vec<(u32, [u16; 32])>, f64, u64, u64) {
    let mut f = Flight::start_with(instance, tag, first_k, Vec::new());
    f.rest.run_s(2.0).expect("2 s");
    f.rest
        .client
        .set_message_interval(id::SYSTEM_TIME, 50.0)
        .expect("interval");
    f.rest.run_s(3.0).expect("5 s");
    let r = &f.rest;
    let hb = r
        .first_at(0, is_heartbeat)
        .expect("a heartbeat: the program booted");
    let ready = r.first_at(0, is_ready).expect("the ready text");
    let offsets: Vec<f64> = r
        .events
        .iter()
        .filter_map(|t| {
            boot_ms(&t.event).map(|b| f64::from(b) - Rest::seconds(t.exchange) * 1000.0)
        })
        .collect();
    assert!(offsets.len() > 100, "{}", offsets.len());
    let lo = offsets.iter().cloned().fold(f64::MAX, f64::min);
    let hi = offsets.iter().cloned().fold(f64::MIN, f64::max);
    assert!(
        hi - lo <= 3.0,
        "the program's clock drifts from ours: {lo}..{hi}"
    );
    println!(
        "MEASURED first stamp step {first_k}: heartbeat at exchange {hb}, ready at {ready}, \
         SYSTEM_TIME minus our clock {lo:.2}..{hi:.2} ms"
    );
    (pulses(r), offsets[offsets.len() - 1], hb, ready)
}

/// The bridge's first reply is for step 0 and carries the stamp
/// `0.000000000`. The program accepts it and its clock is ours, but a run
/// that starts so does not repeat bit for bit (about one run in two moves the
/// moment the motors come on by a few steps, measured), while a run that
/// starts with step 1 does. This test pins both the facts the bridge relies
/// on: zero is accepted and tracked, and starting at step 1 repeats.
#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn a_first_time_stamp_of_zero_works_but_only_a_first_step_repeats() {
    let (zero, offset_zero, _, _) = stamp_run(9, "stamp-zero", 0);
    let (one_a, offset_one, _, _) = stamp_run(9, "stamp-one-a", 1);
    let (one_b, offset_one_b, _, _) = stamp_run(9, "stamp-one-b", 1);
    println!(
        "MEASURED offsets (ms): stamp 0 first {offset_zero:.2}, stamp 1 first {offset_one:.2} / \
         {offset_one_b:.2}; pulses of the zero-first run equal to the one-first run: {}",
        zero == one_a
    );
    // Starting with step 1 repeats exactly.
    assert_eq!(one_a, one_b, "two runs that start with step 1");
    assert!((offset_one - offset_one_b).abs() < 1e-9);
    // Starting with zero is accepted: it boots (checked in `stamp_run`) and
    // its clock follows ours (the range check there); how far the program's
    // clock is from ours differs from the other start by one step at most.
    assert!(
        (offset_zero - offset_one).abs() <= 2.0,
        "{offset_zero} vs {offset_one}"
    );
}
