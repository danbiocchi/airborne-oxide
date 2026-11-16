// SPDX-License-Identifier: AGPL-3.0-or-later
//! Replay of whole flights is seen to fail when it should (ADR-0005 §5 and §9,
//! task P2): a changed core is a difference at the tick where it matters, a
//! flipped bit in a recorded input or output is found, and damaged files
//! (cut, gapped, refused) are never a panic and say what they are.
//!
//! The flight is `rc-loss-position` on the F405 profile, seed 0, logged at
//! the replay level in memory and flown once for all the tests.

use std::sync::OnceLock;

use oxide_core::Core;
use oxide_core::io::Outputs;
use oxide_core::params::{self, LoadReport, ParamValue};
use oxide_core::schedule::F405_LEAN;
use oxide_core::time::Micros;
use oxide_log::header::{BuildInfo, Level as LogLevel};
use oxide_log::reader::{Item, UlogFile};
use oxide_log::recorder::{Boot, Recorder};
use oxide_sim::fault_scenarios;
use oxide_sim::flight_log::{Level, LogBuild, SharedBuffer};
use oxide_sim::replay::{ReplayError, ReplayOptions, ReplayReport, Stop, replay};

/// The file of the flight and the number of ticks it flew.
struct Flight {
    bytes: Vec<u8>,
    ticks: u64,
}

fn fly_at(level: Level) -> Flight {
    let scenario = fault_scenarios::by_name("rc-loss-position").expect("scenario");
    let mut sim = scenario.build(F405_LEAN, 0).expect("builds");
    let buf = SharedBuffer::new();
    sim.start_log(level, &LogBuild::sim("test"), Box::new(buf.clone()))
        .expect("starts");
    let mut ticks = 0;
    sim.run_until(scenario.end(), |_| ticks += 1);
    sim.finish_log().expect("finishes");
    Flight {
        bytes: buf.bytes(),
        ticks,
    }
}

fn flight() -> &'static Flight {
    static F: OnceLock<Flight> = OnceLock::new();
    F.get_or_init(|| fly_at(Level::Replay))
}

fn plain(bytes: &[u8]) -> ReplayReport {
    replay(bytes, &ReplayOptions::default()).expect("replays")
}

/// The offset in the file of `path` of the `n`th record of `topic`.
fn field_at(bytes: &[u8], topic: &str, n: usize, path: &str) -> usize {
    let f = UlogFile::parse(bytes).unwrap();
    let r = f.records(topic).nth(n).unwrap();
    let inner = r.field_bytes(path).unwrap().as_ptr() as usize - r.raw().as_ptr() as usize;
    r.offset() + 5 + inner
}

/// The sequence number of the first tick whose outputs say armed.
fn first_armed_tick(bytes: &[u8]) -> u64 {
    let f = UlogFile::parse(bytes).unwrap();
    let n = f
        .records("oxide_out")
        .position(|r| r.bool("armed") == Some(true))
        .expect("the flight arms");
    n as u64
}

#[test]
fn the_flight_is_worth_replaying() {
    let f = flight();
    let r = plain(&f.bytes);
    assert!(r.identical(), "{:?}", r.first_difference);
    assert_eq!((r.ticks, &r.stop), (f.ticks, &Stop::End));
    // It arms, and has events and a saved-free boot.
    assert!(first_armed_tick(&f.bytes) > 100);
    let file = UlogFile::parse(&f.bytes).unwrap();
    assert!(file.records("oxide_event").count() > 3);
    assert_eq!(file.records("oxide_in_tick").count() as u64, f.ticks);
}

// ---- a changed core is seen ----

fn default_of(name: &str) -> f32 {
    let id = params::find(name).expect("parameter");
    match params::PARAMS[id.index()].default {
        ParamValue::F32(v) => v,
        other => panic!("{name} is {other:?}"),
    }
}

#[test]
fn rc_loss_position_replayed_with_one_gain_changed_differs_in_a_motor_command_after_arming() {
    let f = flight();
    let armed = first_armed_tick(&f.bytes);
    for (name, factor) in [("RATE_PITCH_P", 1.5), ("RATE_ROLL_P", 0.6)] {
        let value = f64::from(default_of(name)) * factor;
        let r = replay(
            &f.bytes,
            &ReplayOptions {
                params: vec![(name.to_string(), value)],
                keep_going: false,
            },
        )
        .expect("replays");
        let d = r.first_difference.as_ref().expect("a difference");
        assert!(d.field.starts_with("motors["), "{name}: {d}");
        assert!(d.seq >= armed, "{name}: {d}, armed at {armed}");
        assert!(d.recorded_bits.is_some() && d.replayed_bits.is_some());
        assert_eq!((r.stop.clone(), r.exit_code()), (Stop::Difference, 1));
        assert_eq!(r.ticks, d.seq + 1);
        println!("{name} x{factor}: {d} (armed at tick {armed})");
    }
}

#[test]
fn keep_going_counts_every_tick_a_changed_gain_changes() {
    let f = flight();
    let value = f64::from(default_of("RATE_PITCH_P")) * 1.5;
    let r = replay(
        &f.bytes,
        &ReplayOptions {
            params: vec![("RATE_PITCH_P".to_string(), value)],
            keep_going: true,
        },
    )
    .expect("replays");
    assert_eq!(r.ticks, f.ticks);
    assert!(r.differing > 100, "{}", r.differing);
    assert_eq!(r.stop, Stop::End);
    assert_eq!(r.exit_code(), 1);
}

/// A flipped bit in the first gyro value of the IMU record at or after
/// `after_us`: the tick it is in, and the damaged file.
fn flip_gyro(bytes: &[u8], after_us: u64, byte: usize, mask: u8) -> (u64, Vec<u8>) {
    let f = UlogFile::parse(bytes).unwrap();
    let n = f
        .records("oxide_in_imu")
        .position(|r| r.u64("timestamp").unwrap() >= after_us)
        .expect("an imu record");
    let t = f
        .records("oxide_in_imu")
        .nth(n)
        .and_then(|r| r.u64("timestamp"))
        .unwrap();
    let seq = f
        .records("oxide_in_tick")
        .position(|r| r.u64("timestamp") == Some(t))
        .expect("its tick") as u64;
    drop(f);
    let mut out = bytes.to_vec();
    out[field_at(bytes, "oxide_in_imu", n, "gyro") + byte] ^= mask;
    (seq, out)
}

#[test]
fn one_flipped_bit_in_a_recorded_gyro_value_is_a_difference_at_or_after_that_tick() {
    let f = flight();
    // Middle of the flight, a high mantissa bit of the first float of the
    // gyro (byte 2 of the little-endian float), and the same for the second
    // and third axis.
    for (after_us, byte) in [(6_000_000, 2), (6_000_000, 6), (9_000_000, 10)] {
        let (seq, bytes) = flip_gyro(&f.bytes, after_us, byte, 0x10);
        assert_ne!(bytes, f.bytes);
        let r = plain(&bytes);
        let d = r
            .first_difference
            .as_ref()
            .unwrap_or_else(|| panic!("flip at tick {seq}: no difference"));
        assert!(d.seq >= seq, "flipped in tick {seq}, found at {d}");
        assert_eq!(r.stop, Stop::Difference);
        assert_eq!(r.exit_code(), 1);
        println!("gyro byte {byte} flipped in tick {seq}: {d}");
    }
}

#[test]
fn one_flipped_bit_in_a_recorded_output_is_that_tick_and_that_field() {
    let f = flight();
    let tick = 7_000;
    for (path, field, mask) in [
        ("motors[0]", "motors[0]", 0x01),
        ("motors[3]", "motors[3]", 0x80),
        ("t_valid", "t_valid", 0x01),
    ] {
        let mut bytes = f.bytes.clone();
        bytes[field_at(&f.bytes, "oxide_out", tick, path)] ^= mask;
        let r = plain(&bytes);
        let d = r.first_difference.as_ref().expect("a difference");
        assert_eq!((d.seq, d.field.as_str()), (tick as u64, field), "{d}");
        assert_eq!(r.ticks, tick as u64 + 1);
        assert_eq!(r.stop, Stop::Difference);
    }
    // A flipped bit in the last bit of a float is one bit of difference.
    let mut bytes = f.bytes.clone();
    bytes[field_at(&f.bytes, "oxide_out", tick, "motors[1]")] ^= 1;
    let d = plain(&bytes).first_difference.unwrap();
    assert_eq!(d.recorded_bits.unwrap() ^ d.replayed_bits.unwrap(), 1);
}

// ---- damage and gaps ----

/// A deterministic sequence of cut positions below `len`.
fn cuts(len: usize, n: usize) -> Vec<usize> {
    let mut x: u64 = 0x9e37_79b9_7f4a_7c15;
    (0..n)
        .map(|_| {
            x = x.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1);
            (x >> 33) as usize % len
        })
        .collect()
}

/// What a cut file may say: nothing (a cut in the definitions), or the
/// replay of a prefix of the flight, identical, ended by the cut.
fn check_cut(bytes: &[u8], cut: usize, full_ticks: u64) -> Option<ReplayReport> {
    match replay(&bytes[..cut], &ReplayOptions::default()) {
        Err(_) => None,
        Ok(r) => {
            assert!(r.identical(), "cut at {cut}: {:?}", r.first_difference);
            assert!(r.ticks <= full_ticks, "cut at {cut}");
            if r.ticks > 0 {
                assert!(
                    matches!(r.stop, Stop::Cut | Stop::End),
                    "cut at {cut}: {:?}",
                    r.stop
                );
                assert_eq!(r.exit_code(), 0, "cut at {cut}");
            }
            if r.stop == Stop::Cut {
                assert!(
                    r.lines()
                        .iter()
                        .any(|l| l.starts_with("the file ends in a cut ")),
                    "cut at {cut}: {:?}",
                    r.lines()
                );
            }
            Some(r)
        }
    }
}

#[test]
fn a_file_cut_at_every_byte_of_its_first_ticks_never_panics_and_replays_up_to_the_cut() {
    let f = flight();
    let file = UlogFile::parse(&f.bytes).unwrap();
    let first = file.records("oxide_in_tick").next().unwrap().offset();
    let end_of_third = file.records("oxide_in_tick").nth(3).unwrap().offset();
    drop(file);
    let mut ticks_before = 0;
    let mut cut_reports = 0;
    for cut in first..=end_of_third + 8 {
        if let Some(r) = check_cut(&f.bytes, cut, f.ticks) {
            assert!(
                r.ticks >= ticks_before,
                "cut at {cut}: fewer ticks than before"
            );
            ticks_before = r.ticks;
            cut_reports += usize::from(r.stop == Stop::Cut);
        }
    }
    assert!(ticks_before >= 3, "{ticks_before}");
    assert!(cut_reports > 100, "{cut_reports} cuts said so");
    // Inside the definitions and the start records: an error or a replay of
    // nothing, never a panic.
    for cut in (0..first).step_by(7) {
        if let Ok(r) = replay(&f.bytes[..cut], &ReplayOptions::default()) {
            assert_eq!(r.ticks, 0, "cut at {cut}");
            assert_ne!(r.exit_code(), 0, "cut at {cut}: nothing was proven");
        }
    }
}

#[test]
fn a_file_cut_at_arbitrary_bytes_replays_up_to_the_cut_and_says_so() {
    let f = flight();
    let mut said = 0;
    for cut in cuts(f.bytes.len(), 40) {
        let Some(r) = check_cut(&f.bytes, cut, f.ticks) else {
            continue;
        };
        said += usize::from(r.stop == Stop::Cut);
    }
    assert!(said > 20, "{said} of 40 cuts said so");
}

#[test]
fn a_flight_logged_through_a_ring_too_small_stops_at_the_gap_and_says_so() {
    // The flight's own inputs go to a second core, which a recorder with a
    // 4 kB ring logs, drained every 25th tick: records are dropped and a
    // dropout is written.
    let scenario = fault_scenarios::by_name("rc-loss-position").expect("scenario");
    let mut sim = scenario.build(F405_LEAN, 0).expect("builds");
    let cfg = *sim.core().config();
    let report = sim.load_report();
    let mut core = Core::<4>::new(cfg, F405_LEAN).expect("core");
    core.report_param_load(report);
    let mut bytes = Vec::new();
    let mut rec = Box::new(Recorder::<4096>::new());
    rec.start(
        &mut bytes,
        Micros::ZERO,
        BuildInfo {
            git_hash: "test",
            board: "SIM",
        },
        Boot {
            profile: &F405_LEAN,
            report: LoadReport::default(),
        },
        LogLevel::Replay,
        core.config(),
    )
    .expect("starts");
    let mut k = 0_u64;
    while sim.time() < scenario.end() {
        let step = sim.step();
        let out: Outputs<4> = core.tick(step.t, sim.last_inputs());
        assert_eq!(out.motors, step.motors, "the second core flies the same");
        rec.record(
            step.t,
            sim.last_inputs(),
            &out,
            &core.telemetry(),
            core.config(),
        );
        if k.is_multiple_of(25) {
            rec.drain(&mut bytes);
        }
        k += 1;
    }
    rec.drain(&mut bytes);
    let file = UlogFile::parse(&bytes).unwrap();
    let dropouts = file
        .items
        .iter()
        .filter(|i| matches!(i, Item::Dropout { .. }))
        .count();
    assert!(dropouts > 0, "the ring was meant to be too small");
    drop(file);
    let r = plain(&bytes);
    assert!(r.identical(), "{:?}", r.first_difference);
    assert!(matches!(r.stop, Stop::Dropout { .. }), "{:?}", r.stop);
    assert!(r.ticks < k);
    assert_eq!(r.exit_code(), 1);
    assert!(
        r.lines()
            .iter()
            .any(|l| l.starts_with("the log has a gap (") && l.contains("nothing after it")),
        "{:?}",
        r.lines()
    );
}

#[test]
fn a_normal_level_log_of_the_flight_is_refused() {
    let normal = fly_at(Level::Normal);
    let e = replay(&normal.bytes, &ReplayOptions::default()).unwrap_err();
    assert!(matches!(e, ReplayError::NotReplayLevel(Some(ref l)) if l == "normal"));
    assert_eq!(
        e.to_string(),
        "not a replay-level log: oxide_log_level is \"normal\" (record with --log-level replay)"
    );
}

#[test]
fn a_header_that_names_another_configuration_hash_is_refused() {
    let f = flight();
    let scenario = fault_scenarios::by_name("rc-loss-position").expect("scenario");
    let sim = scenario.build(F405_LEAN, 0).expect("builds");
    let hash = sim.core().config().config_hash();
    let at = f
        .bytes
        .windows(8)
        .position(|w| w == hash.to_le_bytes())
        .expect("the hash is in the header");
    let mut bytes = f.bytes.clone();
    bytes[at + 3] ^= 0x40;
    let e = replay(&bytes, &ReplayOptions::default()).unwrap_err();
    assert!(
        matches!(
            e,
            ReplayError::Config(oxide_log::replay::decode::FileConfigError::HashMismatch { .. })
        ),
        "{e}"
    );
    assert!(e.to_string().contains("oxide_config_hash is 0x"), "{e}");
}

#[test]
fn a_tick_record_removed_from_the_file_stops_the_replay_there() {
    let f = flight();
    let file = UlogFile::parse(&f.bytes).unwrap();
    let at = |n: usize| file.records("oxide_in_tick").nth(n).unwrap().offset();
    let (a, b) = (at(500), at(501));
    drop(file);
    // The whole tick (its record and everything it owns) is gone: the next
    // tick record is number 501 where 500 should be.
    let mut gone = f.bytes[..a].to_vec();
    gone.extend_from_slice(&f.bytes[b..]);
    let r = plain(&gone);
    assert_eq!(
        r.stop,
        Stop::SequenceGap {
            expected: 500,
            found: 501
        }
    );
    assert_eq!((r.ticks, r.identical(), r.exit_code()), (500, true, 1));
    // Only the tick record is gone (25 bytes: 5 of header and the 20 of the
    // record): the records after it join the previous tick's, and the tick
    // is not replayed.
    let mut bare = f.bytes[..a].to_vec();
    bare.extend_from_slice(&f.bytes[a + 25..]);
    let r = plain(&bare);
    assert_eq!(r.ticks, 499);
    assert!(r.identical());
    assert!(
        matches!(r.stop, Stop::Malformed { seq: Some(499), .. }),
        "{:?}",
        r.stop
    );
    assert_eq!(r.exit_code(), 1);
    assert_eq!(
        r.lines()[1],
        "tick 499 is malformed: oxide_in_imu: two records in one tick"
    );
}
