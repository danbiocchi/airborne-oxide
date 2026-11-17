// SPDX-License-Identifier: AGPL-3.0-or-later
//! A damaged replay-level file is never called a cut (ADR-0005 §5, task R).
//!
//! One flipped bit in a message's size used to make the reader lose its place,
//! land on zeros, and report a zero-filled tail: the replay then said
//! "identical, the file ends in a cut" after a fraction of the flight and
//! exited 0. The rule these tests hold: after any single bit flip in the data
//! section, a replay is refused, stops with exit 1, reports a difference, or
//! exits 0 having replayed at least T-2 ticks of the T the undamaged file
//! holds. The T-1 and T-2 are honest: a flip in the last tick's own records
//! makes the end of the file look like a cut, which is what a power loss at
//! that moment looks like.

use oxide_core::Core;
use oxide_core::io::{
    BaroSample, BatterySample, BrainHeartbeat, GcsHeartbeat, GpsSample, ImuSample, Inputs,
    MagSample, ParamSet, RcSample, Stamped, StateSample, TrajectorySetpoint,
};
use oxide_core::math::{Quat, Vec3};
use oxide_core::params::{self, LoadReport, ParamId, ParamValue};
use oxide_core::schedule::F405_LEAN;
use oxide_core::time::Micros;
use oxide_log::header::{BuildInfo, Level};
use oxide_log::reader::UlogFile;
use oxide_log::recorder::{Boot, Recorder};
use oxide_sim::Airframe;
use oxide_sim::replay::{ReplayOptions, Stop, replay};
use std::collections::BTreeMap;

const BUILD: BuildInfo<'static> = BuildInfo {
    git_hash: "test",
    board: "SIM",
};

/// The scripted inputs of tick `k` (1 kHz). Every kind of input record is in
/// the first 60 ticks: IMU, state, radio, battery, both heartbeats, barometer,
/// magnetometer, GPS, setpoint, and a parameter change that is applied, one for
/// a parameter this build does not have, one the core refuses, a duration, and
/// the acknowledgement of the load.
fn inputs_at(k: u64) -> Inputs {
    let t = Micros(k * 1000);
    let noise = |s: u64| ((k.wrapping_mul(s) % 101) as f32 - 50.0) * 1e-3;
    let mut i = Inputs {
        imu: Some(Stamped::ideal(
            t,
            ImuSample {
                gyro: Vec3::new(noise(37), noise(53), noise(71)),
                accel: Vec3::new(noise(11), noise(13), -9.81 + noise(17)),
            },
        )),
        state: Some(Stamped::ideal(
            t,
            StateSample {
                position: Vec3::new(0.0, 0.0, 0.0),
                velocity: Vec3::new(0.0, 0.0, 0.0),
                attitude: Quat::identity(),
                rate: Vec3::new(0.0, 0.0, 0.0),
            },
        )),
        ..Inputs::default()
    };
    if k.is_multiple_of(20) {
        i.rc = Some(Stamped::ideal(t, RcSample::default()));
    }
    if k.is_multiple_of(10) {
        i.battery = Some(Stamped::ideal(
            t,
            BatterySample {
                voltage_v: 22.4,
                current_a: 1.0,
            },
        ));
        i.gcs_heartbeat = Some(Stamped::ideal(t, GcsHeartbeat));
        i.brain_heartbeat = Some(Stamped::ideal(t, BrainHeartbeat { progress: k as u32 }));
        i.baro = Some(Stamped::ideal(t, BaroSample { altitude_m: 0.5 }));
        i.mag = Some(Stamped::ideal(
            t,
            MagSample {
                field_ut: Vec3::new(20.0, 1.0, 40.0),
            },
        ));
    }
    if k % 20 == 10 {
        i.gps = Some(Stamped::ideal(
            t,
            GpsSample {
                position: Vec3::new(1.0, 2.0, -3.0),
                velocity: Vec3::new(0.1, 0.2, 0.3),
                h_acc_m: 1.5,
                v_acc_m: 2.5,
                s_acc_mps: 0.5,
                fix_3d: true,
            },
        ));
        i.setpoint = Some(Stamped::ideal(
            t,
            TrajectorySetpoint {
                position: Vec3::new(1.0, 2.0, -3.0),
                velocity: Vec3::new(0.0, 0.0, 0.0),
                yaw: 0.3,
            },
        ));
    }
    i.param_ack = k == 50;
    let roll = params::find("RATE_ROLL_P").unwrap();
    let ParamValue::F32(p) = params::PARAMS[roll.index()].default else {
        panic!("RATE_ROLL_P is an f32")
    };
    let change = |id, value| Some(ParamSet { id, value });
    i.param_set = match k {
        10 => change(roll, ParamValue::F32(p * 1.1)),
        20 => change(ParamId(9999), ParamValue::F32(1.0)),
        25 => change(roll, ParamValue::F32(1.0e9)),
        30 => change(roll, ParamValue::F32(p * 1.2)),
        35 => {
            let us = params::micro_ids().next().expect("a duration parameter");
            let ParamValue::F32(d) = params::PARAMS[us.index()].default else {
                panic!("durations are f32 seconds")
            };
            change(us, ParamValue::F32(d))
        }
        _ => None,
    };
    i
}

/// A replay-level log of `ticks` ticks of the script.
fn fly(ticks: u64) -> Vec<u8> {
    let airframe = Airframe::reference_quad().unwrap();
    let cfg = airframe.core_config::<4>().unwrap();
    let mut core = Core::<4>::new(cfg, F405_LEAN).unwrap();
    let report = LoadReport::default();
    core.report_param_load(report);
    let mut bytes = Vec::new();
    let mut rec = Box::new(Recorder::<65_536>::new());
    rec.start(
        &mut bytes,
        Micros::ZERO,
        BUILD,
        Boot {
            profile: &F405_LEAN,
            report,
        },
        Level::Replay,
        core.config(),
    )
    .unwrap();
    for k in 0..ticks {
        let t = Micros(k * 1000);
        let inputs = inputs_at(k);
        let out = core.tick(t, &inputs);
        rec.record(t, &inputs, &out, &core.telemetry(), core.config());
        rec.drain(&mut bytes);
    }
    rec.drain(&mut bytes);
    bytes
}

/// (offset, length, type) of every message of the data section, from the first
/// subscription to the end.
fn messages(bytes: &[u8]) -> Vec<(usize, usize, u8)> {
    let file = UlogFile::parse(bytes).unwrap();
    let mut p = file.subscriptions.first().unwrap().offset;
    let mut out = Vec::new();
    while p + 3 <= bytes.len() {
        let size = usize::from(u16::from_le_bytes([bytes[p], bytes[p + 1]]));
        out.push((p, 3 + size, bytes[p + 2]));
        p += 3 + size;
    }
    assert_eq!(p, bytes.len(), "the undamaged file is whole messages");
    out
}

/// What a replay of a damaged file came to, against the `t` ticks of the
/// undamaged one.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum Outcome {
    /// Refused: an error, exit 2.
    Refused,
    /// A gap, a malformed tick or damage: exit 1, no difference.
    Stopped,
    /// A difference: exit 1.
    Difference,
    /// Exit 0, every tick.
    Identical,
    /// Exit 0, one tick short.
    ShortBy1,
    /// Exit 0, two ticks short.
    ShortBy2,
    /// Exit 0 with more than two ticks missing: the failure this file exists
    /// to rule out.
    Wrong,
}

fn outcome(bytes: &[u8], t: u64) -> Outcome {
    match replay(bytes, &ReplayOptions::default()) {
        Err(_) => Outcome::Refused,
        Ok(r) if r.exit_code() != 0 => {
            if r.differing > 0 {
                Outcome::Difference
            } else {
                Outcome::Stopped
            }
        }
        Ok(r) => match t.checked_sub(r.ticks) {
            Some(0) => Outcome::Identical,
            Some(1) => Outcome::ShortBy1,
            Some(2) => Outcome::ShortBy2,
            _ => Outcome::Wrong,
        },
    }
}

fn flip(bytes: &[u8], byte: usize, bit: u32) -> Vec<u8> {
    let mut m = bytes.to_vec();
    m[byte] ^= 1 << bit;
    m
}

fn undamaged(ticks: u64) -> (Vec<u8>, u64) {
    let bytes = fly(ticks);
    let r = replay(&bytes, &ReplayOptions::default()).unwrap();
    assert_eq!((r.ticks, &r.stop, r.exit_code()), (ticks, &Stop::End, 0));
    (bytes, ticks)
}

#[test]
fn a_flipped_bit_in_a_message_header_or_a_logged_string_never_replays_clean_but_short() {
    let (bytes, t) = undamaged(60);
    let file = UlogFile::parse(&bytes).unwrap();
    // Every kind of record the script can make is in the file.
    for topic in [
        "oxide_in_tick",
        "oxide_in_imu",
        "oxide_in_state",
        "oxide_in_rc",
        "oxide_in_battery",
        "oxide_in_gcs",
        "oxide_in_brain",
        "oxide_in_baro",
        "oxide_in_mag",
        "oxide_in_gps",
        "oxide_in_setpoint",
        "oxide_in_param_set",
        "oxide_out",
        "oxide_event",
    ] {
        assert!(file.records(topic).count() > 0, "{topic}");
    }
    let mut counts: BTreeMap<Outcome, usize> = BTreeMap::new();
    for (at, _, ty) in messages(&bytes) {
        // The 3-byte header; and for a logged string the first two payload
        // bytes (its level and the first byte of its time).
        let span = if ty == b'L' { 5 } else { 3 };
        for byte in at..at + span {
            for bit in 0..8 {
                let o = outcome(&flip(&bytes, byte, bit), t);
                assert_ne!(
                    o,
                    Outcome::Wrong,
                    "bit {bit} of byte {byte} (message of type {:?} at {at})",
                    char::from(ty)
                );
                *counts.entry(o).or_default() += 1;
            }
        }
    }
    println!("header flips of {t} ticks: {counts:?}");
}

#[test]
fn the_reviewers_flip_of_a_logged_strings_size_is_no_longer_a_cut_at_a_fraction_of_the_flight() {
    let (bytes, t) = undamaged(100);
    let (at, _, _) = messages(&bytes)
        .into_iter()
        .find(|m| m.2 == b'L')
        .expect("a logged string");
    // The size of the string message, bit 0: 14 becomes 15 (or the reverse).
    let r = replay(&flip(&bytes, at, 0), &ReplayOptions::default());
    match r {
        Err(_) => {}
        Ok(r) => assert!(
            r.exit_code() == 1 || r.ticks + 2 >= t,
            "{} ticks of {t}, exit {}: {:?}",
            r.ticks,
            r.exit_code(),
            r.lines()
        ),
    }
}

#[test]
fn a_cut_whose_message_declares_more_than_any_message_of_the_file_is_damage_not_a_cut() {
    let (bytes, t) = undamaged(60);
    let largest = UlogFile::parse(&bytes).unwrap().largest_data_message;
    // A logged-string header declaring `size`, and ten bytes of it.
    let with_tail = |size: usize| {
        let mut b = bytes.clone();
        b.extend_from_slice(&u16::try_from(size).unwrap().to_le_bytes());
        b.push(b'L');
        b.extend_from_slice(b"6\0\0\0\0\0\0\0\0x");
        b
    };
    // As large as the largest message: a plausible cut.
    let r = replay(&with_tail(largest), &ReplayOptions::default()).unwrap();
    assert_eq!((r.ticks, &r.stop, r.exit_code()), (t, &Stop::Cut, 0));
    // One more byte than any message has: not one this file holds.
    let r = replay(&with_tail(largest + 1), &ReplayOptions::default()).unwrap();
    assert_eq!((r.ticks, r.exit_code()), (t, 1));
    assert!(r.identical());
    let Stop::Damaged { why } = &r.stop else {
        panic!("{:?}", r.stop)
    };
    assert!(why.contains("damage, not a cut"), "{why}");
    let lines = r.lines();
    assert!(
        lines[1].starts_with("the file is damaged after tick 59: the last message, at byte "),
        "{lines:?}"
    );
}

#[test]
fn a_real_cut_at_any_byte_of_the_last_three_ticks_replays_what_is_there_and_says_so() {
    let (bytes, t) = undamaged(60);
    let file = UlogFile::parse(&bytes).unwrap();
    let from = file
        .records("oxide_in_tick")
        .nth(t as usize - 3)
        .unwrap()
        .offset();
    let mut last_ticks = 0;
    for cut in from..bytes.len() {
        let r = replay(&bytes[..cut], &ReplayOptions::default()).unwrap();
        assert!(r.identical(), "cut at {cut}: {:?}", r.first_difference);
        assert_eq!(r.exit_code(), 0, "cut at {cut}: {:?}", r.lines());
        assert!(
            (t - 3..t).contains(&r.ticks) && r.ticks >= last_ticks,
            "cut at {cut}: {} ticks after {last_ticks}",
            r.ticks
        );
        last_ticks = r.ticks;
        match r.stop {
            // Between two ticks the file looks whole.
            Stop::End => assert_eq!(r.lines().len(), 1),
            Stop::Cut => assert_eq!(
                r.lines()[1],
                format!(
                    "the file ends in a cut after tick {}: the ticks before it were replayed",
                    r.ticks - 1
                ),
                "cut at {cut}"
            ),
            ref other => panic!("cut at {cut}: {other:?}"),
        }
    }
    assert_eq!(last_ticks, t - 1);
    // The whole file.
    assert_eq!(replay(&bytes, &ReplayOptions::default()).unwrap().ticks, t);
}

#[test]
fn a_zero_filled_tail_replays_every_tick_and_is_called_a_cut() {
    let (mut bytes, t) = undamaged(60);
    bytes.extend_from_slice(&[0; 4096]);
    let r = replay(&bytes, &ReplayOptions::default()).unwrap();
    assert_eq!((r.ticks, &r.stop, r.exit_code()), (t, &Stop::Cut, 0));
    assert_eq!(
        r.lines(),
        [
            "replayed 60 ticks (f405-lean): identical".to_string(),
            "the file ends in a cut after tick 59: the ticks before it were replayed".to_string()
        ]
    );
    // Zeros and then anything is damage, and the file is refused.
    *bytes.last_mut().unwrap() = 1;
    assert!(replay(&bytes, &ReplayOptions::default()).is_err());
}

/// Every bit of every byte of the data section of a 100-tick log (release
/// only, about half a minute on 4 threads):
/// `cargo test --release -p oxide-sim --test replay_damage -- --ignored --nocapture
/// every_bit_of_the_data_section`.
#[test]
#[ignore = "release only: a few hundred thousand replays"]
fn every_bit_of_the_data_section_is_refused_stopped_different_or_within_two_ticks() {
    if cfg!(debug_assertions) {
        panic!("run with --release (about 300,000 replays)");
    }
    let started = std::time::Instant::now();
    let (bytes, t) = undamaged(100);
    let first = UlogFile::parse(&bytes)
        .unwrap()
        .subscriptions
        .first()
        .unwrap()
        .offset;
    let threads = std::thread::available_parallelism().map_or(2, |n| n.get().min(4));
    let total = bytes.len() - first;
    let mut counts: BTreeMap<Outcome, usize> = BTreeMap::new();
    let mut wrong: Vec<(usize, u32)> = Vec::new();
    std::thread::scope(|s| {
        let workers: Vec<_> = (0..threads)
            .map(|w| {
                let bytes = &bytes;
                s.spawn(move || {
                    let mut counts: BTreeMap<Outcome, usize> = BTreeMap::new();
                    let mut wrong = Vec::new();
                    for byte in (first + w..bytes.len()).step_by(threads) {
                        for bit in 0..8 {
                            let o = outcome(&flip(bytes, byte, bit), t);
                            if o == Outcome::Wrong {
                                wrong.push((byte, bit));
                            }
                            *counts.entry(o).or_default() += 1;
                        }
                    }
                    (counts, wrong)
                })
            })
            .collect();
        for w in workers {
            let (c, bad) = w.join().unwrap();
            for (o, n) in c {
                *counts.entry(o).or_default() += n;
            }
            wrong.extend(bad);
        }
    });
    println!(
        "every bit of the {total} data bytes ({} flips) of a {t}-tick log, {} s: {counts:?}",
        total * 8,
        started.elapsed().as_secs()
    );
    wrong.sort_unstable();
    assert!(
        wrong.is_empty(),
        "{} flips exit 0 with more than 2 ticks missing, the first (byte, bit): {:?}",
        wrong.len(),
        &wrong[..wrong.len().min(10)]
    );
}
