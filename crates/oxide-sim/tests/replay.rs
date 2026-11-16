// SPDX-License-Identifier: AGPL-3.0-or-later
//! Replay of a replay-level log (ADR-0005 §5, task P1).
//!
//! The first group drives a bare `Core` and a `Recorder` with a scripted
//! sequence of inputs, so that the replay is tested without the simulator:
//! identical replays, a changed gain, damaged files, gaps. The second group
//! (below) flies whole scenarios.

use oxide_core::Core;
use oxide_core::io::{
    BaroSample, BatterySample, BrainHeartbeat, GcsHeartbeat, ImuSample, Inputs, MagSample, Outputs,
    ParamSet, RcSample, Stamped, StateSample,
};
use oxide_core::math::{Quat, Vec3};
use oxide_core::params::{self, LoadReport, ParamValue};
use oxide_core::schedule::F405_LEAN;
use oxide_core::time::Micros;
use oxide_log::header::{BuildInfo, Level};
use oxide_log::reader::UlogFile;
use oxide_log::recorder::{Boot, Recorder};
use oxide_sim::Airframe;
use oxide_sim::replay::{ReplayError, ReplayOptions, Stop, replay};

const BUILD: BuildInfo<'static> = BuildInfo {
    git_hash: "test",
    board: "SIM",
};

/// What a scripted flight of a bare core gave.
struct Flown {
    bytes: Vec<u8>,
    outputs: Vec<Outputs<4>>,
}

/// The scripted inputs of tick `k` (1 kHz): noise on the gyro, a state sample,
/// the radio at 50 Hz (armed from tick 1200 to 2000, the one second of valid radio the core wants first), a parameter acknowledgement
/// at 50, one parameter change while disarmed (100) and one while armed
/// (1600), heartbeats, the barometer, the magnetometer and the battery.
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
        i.rc = Some(Stamped::ideal(
            t,
            RcSample {
                throttle: if (1300..2000).contains(&k) { 0.55 } else { 0.0 },
                arm_switch: (1200..2000).contains(&k),
                mode_switch: 0,
                ..RcSample::default()
            },
        ));
    }
    if k.is_multiple_of(100) {
        i.battery = Some(Stamped::ideal(
            t,
            BatterySample {
                voltage_v: 22.4,
                current_a: 1.0,
            },
        ));
        i.gcs_heartbeat = Some(Stamped::ideal(t, GcsHeartbeat));
        i.brain_heartbeat = Some(Stamped::ideal(t, BrainHeartbeat { progress: k as u32 }));
    }
    if k.is_multiple_of(10) {
        i.baro = Some(Stamped::ideal(t, BaroSample { altitude_m: 0.0 }));
        i.mag = Some(Stamped::ideal(
            t,
            MagSample {
                field_ut: Vec3::new(20.0, 0.0, 40.0),
            },
        ));
    }
    i.param_ack = k == 50;
    if k == 100 || k == 1600 {
        let id = params::find("RATE_ROLL_P").unwrap();
        let ParamValue::F32(p) = params::PARAMS[id.index()].default else {
            panic!("RATE_ROLL_P is an f32")
        };
        i.param_set = Some(ParamSet {
            id,
            value: ParamValue::F32(p * if k == 100 { 1.1 } else { 1.2 }),
        });
    }
    i
}

/// Flies a bare core for `ticks` ticks of the script and logs it at the
/// replay level into a ring of `C` bytes drained every `drain_every` ticks.
/// The store's saved call is made at tick 2200 (disarmed, with a parameter
/// unsaved) and noted the way the simulator notes it.
fn fly<const C: usize>(ticks: u64, drain_every: u64, report: LoadReport) -> Flown {
    let airframe = Airframe::reference_quad().unwrap();
    let cfg = airframe.core_config::<4>().unwrap();
    let mut core = Core::<4>::new(cfg, F405_LEAN).unwrap();
    core.report_param_load(report);
    let mut bytes = Vec::new();
    let mut rec = Box::new(Recorder::<C>::new());
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
    let mut outputs = Vec::new();
    let mut was_unsaved = false;
    for k in 0..ticks {
        let t = Micros(k * 1000);
        if k == 2200 {
            core.clear_unsaved();
        }
        let unsaved_before = core.params_unsaved();
        if was_unsaved && !unsaved_before {
            rec.note_saved();
        }
        let inputs = inputs_at(k);
        let out = core.tick(t, &inputs);
        was_unsaved = out.status.params_unsaved;
        rec.record(t, &inputs, &out, &core.telemetry(), core.config());
        if k % drain_every == 0 {
            rec.drain(&mut bytes);
        }
        outputs.push(out);
    }
    rec.drain(&mut bytes);
    Flown { bytes, outputs }
}

fn default_flight() -> Flown {
    fly::<65_536>(2500, 1, LoadReport::default())
}

#[test]
fn a_scripted_flight_replays_identically_tick_for_tick() {
    let flown = default_flight();
    // The script is worth replaying: the vehicle armed and flew its motors,
    // a parameter changed, the saved call happened.
    assert!(
        flown
            .outputs
            .iter()
            .any(|o| o.motors.iter().any(|m| *m > 0.0))
    );
    let file = UlogFile::parse(&flown.bytes).unwrap();
    assert!(
        file.records("oxide_in_tick")
            .any(|r| r.bool("saved") == Some(true))
    );
    let r = replay(&flown.bytes, &ReplayOptions::default()).unwrap();
    assert_eq!(r.ticks, 2500);
    assert!(r.identical(), "{:?}", r.first_difference);
    assert_eq!(r.stop, Stop::End);
    assert_eq!(r.last_tick, Some((2499, 2_499_000)));
    assert_eq!(r.profile, "f405-lean");
    assert_eq!(r.exit_code(), 0);
    assert_eq!(
        r.lines(),
        ["replayed 2500 ticks (f405-lean): identical".to_string()]
    );
}

#[test]
fn an_unclean_boot_report_is_part_of_what_replays() {
    let report = LoadReport {
        applied: 80,
        unknown: 2,
        dropped: 1,
        invalid: false,
        unreadable: false,
        torn: 0,
    };
    let flown = fly::<65_536>(400, 1, report);
    let r = replay(&flown.bytes, &ReplayOptions::default()).unwrap();
    assert_eq!(r.ticks, 400);
    assert!(r.identical(), "{:?}", r.first_difference);
}

#[test]
fn a_normal_level_log_is_refused() {
    let airframe = Airframe::reference_quad().unwrap();
    let cfg = airframe.core_config::<4>().unwrap();
    let mut bytes = Vec::new();
    let mut rec = Box::new(Recorder::<65_536>::new());
    rec.start(
        &mut bytes,
        Micros::ZERO,
        BUILD,
        Boot::clean(&F405_LEAN),
        Level::Normal,
        &cfg,
    )
    .unwrap();
    let e = replay(&bytes, &ReplayOptions::default()).unwrap_err();
    assert!(matches!(e, ReplayError::NotReplayLevel(Some(ref l)) if l == "normal"));
    assert_eq!(
        e.to_string(),
        "not a replay-level log: oxide_log_level is \"normal\" (record with --log-level replay)"
    );
}

#[test]
fn bytes_that_are_not_a_log_are_refused_without_panic() {
    for bytes in [&b""[..], b"not a ulog file at all", &[0u8; 64]] {
        let e = replay(bytes, &ReplayOptions::default()).unwrap_err();
        assert!(matches!(e, ReplayError::Read(_)), "{e}");
    }
}

#[test]
fn a_header_with_another_configuration_hash_is_refused() {
    let flown = default_flight();
    let airframe = Airframe::reference_quad().unwrap();
    let hash = airframe.core_config::<4>().unwrap().config_hash();
    let mut bytes = flown.bytes.clone();
    let at = bytes
        .windows(8)
        .position(|w| w == hash.to_le_bytes())
        .expect("the hash is in the header");
    bytes[at] ^= 1;
    let e = replay(&bytes, &ReplayOptions::default()).unwrap_err();
    assert!(
        matches!(
            e,
            ReplayError::Config(oxide_log::replay::decode::FileConfigError::HashMismatch { .. })
        ),
        "{e}"
    );
}

/// The offset in `bytes` of record `n` of `topic`'s payload.
fn payload_of(bytes: &[u8], topic: &str, n: usize) -> usize {
    let f = UlogFile::parse(bytes).unwrap();
    // 2 bytes of size, 1 of type, 2 of message id.
    f.records(topic).nth(n).unwrap().offset() + 5
}

#[test]
fn a_boot_record_that_is_not_this_builds_profile_or_vehicle_is_refused() {
    let flown = default_flight();
    // oxide_boot: timestamp 8, four u16 8, base_period_us 4 at 16, ...,
    // rotors the last byte (index 46).
    let at = payload_of(&flown.bytes, "oxide_boot", 0);
    let mut six = flown.bytes.clone();
    six[at + 46] = 6;
    let e = replay(&six, &ReplayOptions::default()).unwrap_err();
    assert!(matches!(e, ReplayError::Rotors(6)), "{e}");
    assert_eq!(
        e.to_string(),
        "the vehicle has 6 rotors; this build replays vehicles with 4"
    );
    let mut rate = flown.bytes.clone();
    rate[at + 16] ^= 1;
    let e = replay(&rate, &ReplayOptions::default()).unwrap_err();
    assert!(
        matches!(e, ReplayError::ProfileMismatch("f405-lean")),
        "{e}"
    );
}

#[test]
fn a_changed_gain_is_a_difference_in_a_motor_command_after_arming() {
    let flown = default_flight();
    let opts = ReplayOptions {
        params: vec![("RATE_PITCH_P".to_string(), 14.0)],
        keep_going: false,
    };
    let r = replay(&flown.bytes, &opts).unwrap();
    let d = r.first_difference.as_ref().expect("a difference");
    assert!(d.field.starts_with("motors["), "{d}");
    assert!(d.seq >= 1200, "{d}");
    assert!(d.recorded_bits.is_some() && d.replayed_bits.is_some());
    assert_eq!(r.differing, 1);
    assert_eq!(r.stop, Stop::Difference);
    assert_eq!(r.exit_code(), 1);
    assert_eq!(r.ticks, d.seq + 1);
    // Going on counts every tick that differs.
    let all = replay(
        &flown.bytes,
        &ReplayOptions {
            keep_going: true,
            ..opts
        },
    )
    .unwrap();
    assert!(all.differing > 1, "{all:?}");
    assert_eq!(all.ticks, 2500);
    assert_eq!(all.first_difference, r.first_difference);
    assert_eq!(all.stop, Stop::End);
    assert!(all.lines()[0].contains(" differ, the first at tick "));
}

#[test]
fn an_override_the_core_refuses_is_an_error_not_a_difference() {
    let flown = default_flight();
    let opts = ReplayOptions {
        params: vec![("NO_SUCH_PARAM".to_string(), 1.0)],
        keep_going: false,
    };
    let e = replay(&flown.bytes, &opts).unwrap_err();
    assert!(matches!(e, ReplayError::Override(_)), "{e}");
}

#[test]
fn one_flipped_bit_in_a_recorded_output_is_that_tick_and_that_field() {
    let flown = default_flight();
    let mut bytes = flown.bytes.clone();
    // oxide_out: timestamp 0..8, t_valid 8..16, motors 16..48.
    let at = payload_of(&bytes, "oxide_out", 1700) + 16 + 4 * 2;
    bytes[at] ^= 1;
    let r = replay(&bytes, &ReplayOptions::default()).unwrap();
    let d = r.first_difference.unwrap();
    assert_eq!((d.seq, d.field.as_str()), (1700, "motors[2]"));
    assert_eq!(
        d.recorded_bits.unwrap() ^ d.replayed_bits.unwrap(),
        1,
        "{d}"
    );
    assert_eq!(r.ticks, 1701);
}

#[test]
fn a_cut_file_replays_up_to_the_cut_and_says_so() {
    let flown = default_flight();
    let f = UlogFile::parse(&flown.bytes).unwrap();
    let start = f.records("oxide_in_tick").nth(100).unwrap().offset();
    let mut cuts = vec![start + 1, start + 2, start + 7, start + 100, start + 12_345];
    cuts.extend((0..40).map(|i| start + 3_000 + i * 997));
    for cut in cuts {
        let r = replay(&flown.bytes[..cut], &ReplayOptions::default()).unwrap();
        assert!(r.identical(), "cut at {cut}: {:?}", r.first_difference);
        assert!(
            r.ticks >= 100 && r.ticks < 2500,
            "cut at {cut}: {}",
            r.ticks
        );
        assert!(matches!(r.stop, Stop::Cut | Stop::End), "{:?}", r.stop);
        assert_eq!(r.exit_code(), 0);
    }
    // In the middle of a message the cut is seen by the reader.
    let r = replay(&flown.bytes[..start + 2], &ReplayOptions::default()).unwrap();
    assert_eq!((r.ticks, &r.stop), (100, &Stop::Cut));
    assert_eq!(
        r.lines()[1],
        "the file ends in a cut after tick 99: the ticks before it were replayed"
    );
    // Cuts inside the definitions are not logs at all.
    for cut in [0, 10, 100, 5_000] {
        assert!(replay(&flown.bytes[..cut], &ReplayOptions::default()).is_err());
    }
}

#[test]
fn a_tick_whose_records_are_cut_off_at_the_end_is_not_replayed() {
    let flown = default_flight();
    let f = UlogFile::parse(&flown.bytes).unwrap();
    // Keep tick 200's tick record and nothing after it.
    let end = f.records("oxide_in_tick").nth(200).unwrap().offset();
    let tick = f.records("oxide_in_tick").nth(200).unwrap();
    let len = 5 + 20; // header and id, then the record (no trailing padding)
    assert_eq!(tick.raw().len(), 20);
    let r = replay(&flown.bytes[..end + len], &ReplayOptions::default()).unwrap();
    assert_eq!((r.ticks, &r.stop), (200, &Stop::Cut));
    assert!(r.identical());
}

#[test]
fn a_missing_tick_stops_the_replay_at_the_gap() {
    let flown = default_flight();
    let f = UlogFile::parse(&flown.bytes).unwrap();
    let a = f.records("oxide_in_tick").nth(300).unwrap().offset();
    let b = f.records("oxide_in_tick").nth(301).unwrap().offset();
    let mut bytes = flown.bytes[..a].to_vec();
    bytes.extend_from_slice(&flown.bytes[b..]);
    let r = replay(&bytes, &ReplayOptions::default()).unwrap();
    assert_eq!(
        r.stop,
        Stop::SequenceGap {
            expected: 300,
            found: 301
        }
    );
    assert_eq!(r.ticks, 300);
    assert!(r.identical());
    assert_eq!(r.exit_code(), 1);
    assert!(
        r.lines()[1]
            .starts_with("a tick record is missing after tick 299: expected number 300, found 301"),
        "{:?}",
        r.lines()
    );
}

#[test]
fn a_ring_too_small_makes_a_gap_and_the_replay_stops_there() {
    // 2 kB, drained every 25th tick: records are lost and a dropout written.
    let flown = fly::<2048>(2500, 25, LoadReport::default());
    let file = UlogFile::parse(&flown.bytes).unwrap();
    assert!(
        file.items
            .iter()
            .any(|i| matches!(i, oxide_log::reader::Item::Dropout { .. }))
    );
    let r = replay(&flown.bytes, &ReplayOptions::default()).unwrap();
    assert!(matches!(r.stop, Stop::Dropout { .. }), "{:?}", r.stop);
    assert!(r.identical(), "{:?}", r.first_difference);
    assert!(r.ticks < 2500);
    assert_eq!(r.exit_code(), 1);
    assert!(
        r.lines()[1].starts_with("the log has a gap ("),
        "{:?}",
        r.lines()
    );
}
