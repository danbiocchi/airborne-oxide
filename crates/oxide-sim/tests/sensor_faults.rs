// SPDX-License-Identifier: AGPL-3.0-or-later
//! The sensor-value faults (`mag-rotate`, `gps-vel-offset`, `gps-vel-noise`)
//! and the GPS delay from a scenario file, in whole flights: a fault that has
//! not begun changes nothing, one that has changes the flight, and the flight
//! replays bit for bit (the replay log holds the inputs the core saw).

use std::path::Path;

use oxide_core::math::Real;
use oxide_core::schedule::{F405_LEAN, H743_FULL, Profile};
use oxide_sim::faults::{Fault, FaultKind};
use oxide_sim::flight_log::{Level, LogBuild, SharedBuffer};
use oxide_sim::replay::{ReplayOptions, Stop, replay};
use oxide_sim::scenario_file::{self, ScenarioFile};

fn gust_hold() -> ScenarioFile {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scenarios/gust-hold.toml");
    scenario_file::load_path(&path).expect("the file loads")
}

fn fault(kind: FaultKind, start_s: f64, end_s: Option<f64>) -> Fault {
    Fault {
        kind,
        start_s,
        end_s,
    }
}

/// The faults of the four check flights (the brief's item 7).
fn rotate_1_5() -> Fault {
    fault(
        FaultKind::MagRotate {
            angle_rad: 1.5,
            ramp_s: 0.0,
        },
        10.0,
        Some(25.0),
    )
}

fn jump_5_north() -> Fault {
    fault(
        FaultKind::GpsVelOffset {
            north_mps: 5.0,
            east_mps: 0.0,
        },
        10.0,
        Some(11.0),
    )
}

fn noise_2() -> Fault {
    fault(
        FaultKind::GpsVelNoise {
            sigma_mps: 2.0,
            s_acc_mps: None,
        },
        10.0,
        Some(15.0),
    )
}

/// What a flight leaves.
struct Flight {
    hash: u64,
    ticks: u64,
    /// The events, as (time in s, text).
    events: Vec<(f64, String)>,
    /// The largest distance from the set-point (0, 0, 2 m up) from 6 s on,
    /// m: horizontal, and in three dimensions.
    max_h_m: f64,
    max_3d_m: f64,
    /// The replay of the flight's replay-level log.
    replay_identical: bool,
    replayed_ticks: u64,
    /// The estimate's heading error (estimate minus truth, wrapped), rad, at
    /// whole seconds.
    yaw_error: Vec<(u32, f64)>,
}

fn fly(file: &ScenarioFile, profile: Profile, index: u64) -> Flight {
    let mut sim = file.build(profile, index).expect("builds");
    let buf = SharedBuffer::new();
    sim.start_log(
        Level::Replay,
        &LogBuild::sim("sensor-faults"),
        Box::new(buf.clone()),
    )
    .expect("the replay level starts");
    let mut f = Flight {
        hash: 0,
        ticks: 0,
        events: Vec::new(),
        max_h_m: 0.0,
        max_3d_m: 0.0,
        replay_identical: false,
        replayed_ticks: 0,
        yaw_error: Vec::new(),
    };
    loop {
        let r = sim.step();
        f.ticks += 1;
        if r.t >= file.end() {
            break;
        }
        if let Some(c) = &r.core {
            for e in c.events.iter() {
                f.events.push((e.t.as_secs_f64(), format!("{:?}", e.kind)));
            }
        }
        if r.t.0.is_multiple_of(1_000_000)
            && let Some(est) = r.estimate()
        {
            let d = f64::from(est.attitude.yaw()) - r.attitude.yaw();
            let wrapped = (d + std::f64::consts::PI).rem_euclid(2.0 * std::f64::consts::PI)
                - std::f64::consts::PI;
            f.yaw_error.push(((r.t.0 / 1_000_000) as u32, wrapped));
        }
        if r.t.as_secs_f64() >= 6.0 {
            let h = Real::sqrt(r.position.x * r.position.x + r.position.y * r.position.y);
            let dz = r.position.z + 2.0;
            f.max_h_m = f.max_h_m.max(h);
            f.max_3d_m = f.max_3d_m.max(Real::sqrt(h * h + dz * dz));
        }
    }
    f.hash = sim.trace_hash();
    sim.finish_log().expect("finishes");
    let r = replay(&buf.bytes(), &ReplayOptions::default()).expect("replays");
    f.replay_identical = r.identical() && r.stop == Stop::End;
    f.replayed_ticks = r.ticks;
    f
}

#[test]
fn faults_that_have_not_begun_leave_the_flight_bit_for_bit_as_it_was() {
    let base = gust_hold();
    let mut late = base.clone();
    // Every fault begins after the 40 s flight ends; the delay is the
    // preset's own.
    late.faults = vec![
        fault(
            FaultKind::MagRotate {
                angle_rad: 1.5,
                ramp_s: 2.0,
            },
            100.0,
            Some(130.0),
        ),
        fault(
            FaultKind::GpsVelOffset {
                north_mps: 5.0,
                east_mps: 5.0,
            },
            100.0,
            Some(101.0),
        ),
        fault(
            FaultKind::GpsVelNoise {
                sigma_mps: 2.0,
                s_acc_mps: Some(1.0),
            },
            100.0,
            Some(105.0),
        ),
    ];
    late.gps_latency_us = Some(150_000);
    for (profile, index) in [(F405_LEAN, 500), (H743_FULL, 501)] {
        let a = fly(&base, profile, index);
        let b = fly(&late, profile, index);
        assert_eq!(a.hash, b.hash, "{}", profile.name);
        assert_eq!(a.ticks, b.ticks);
    }
}

#[test]
fn a_window_that_has_ended_leaves_what_came_before_it_as_it_was() {
    // The flight up to the fault's start is the flight without it: compare
    // the position hash of the first 9 s by flying a copy that ends there.
    let mut base = gust_hold();
    base.duration_s = 9.0;
    base.pilot.duration_s = 9.0;
    let mut hit = base.clone();
    hit.faults = vec![rotate_1_5(), jump_5_north(), noise_2()];
    let a = fly(&base, F405_LEAN, 500);
    let b = fly(&hit, F405_LEAN, 500);
    assert_eq!(a.hash, b.hash);
}

#[test]
fn each_fault_changes_the_flight_and_the_same_flight_twice_is_the_same_flight() {
    let base = gust_hold();
    let plain = fly(&base, F405_LEAN, 500);
    let mut seen = vec![plain.hash];
    for (name, fault) in [
        ("mag-rotate", Some(rotate_1_5())),
        ("gps-vel-offset", Some(jump_5_north())),
        ("gps-vel-noise", Some(noise_2())),
        ("gps latency", None),
    ] {
        let mut file = base.clone();
        match fault {
            Some(f) => file.faults = vec![f],
            None => file.gps_latency_us = Some(250_000),
        }
        let a = fly(&file, F405_LEAN, 500);
        let b = fly(&file, F405_LEAN, 500);
        assert_eq!(a.hash, b.hash, "{name} is not deterministic");
        assert!(!seen.contains(&a.hash), "{name} changed nothing");
        seen.push(a.hash);
    }
}

#[test]
fn flights_with_each_fault_replay_bit_for_bit() {
    let base = gust_hold();
    for (name, profile, index, fault) in [
        ("mag-rotate", F405_LEAN, 500, Some(rotate_1_5())),
        ("mag-rotate", H743_FULL, 501, Some(rotate_1_5())),
        ("gps-vel-offset", F405_LEAN, 500, Some(jump_5_north())),
        ("gps-vel-noise", F405_LEAN, 500, Some(noise_2())),
        ("gps-vel-noise", H743_FULL, 502, Some(noise_2())),
        ("gps latency", F405_LEAN, 500, None),
        ("gps latency", H743_FULL, 500, None),
    ] {
        let mut file = base.clone();
        match fault {
            Some(f) => file.faults = vec![f],
            None => file.gps_latency_us = Some(250_000),
        }
        let f = fly(&file, profile, index);
        assert!(
            f.replay_identical && f.replayed_ticks == f.ticks,
            "{name} on {}: replayed {} of {}",
            profile.name,
            f.replayed_ticks,
            f.ticks
        );
    }
}

/// The rotated field reaches the filter: its heading error during the
/// rotation differs from the same flight's without it. (What the filter does
/// with a field that no longer matches its own, and what the guard does, are
/// for the experiments; this only shows the fault is seen.)
#[test]
fn a_rotated_field_reaches_the_filter() {
    let base = gust_hold();
    let plain = fly(&base, F405_LEAN, 500);
    let mut file = base.clone();
    file.faults = vec![rotate_1_5()];
    let f = fly(&file, F405_LEAN, 500);
    let at = |v: &[(u32, f64)], s: u32| v.iter().find(|(t, _)| *t == s).expect("a second").1;
    // Before the window the two flights agree; inside they do not.
    assert_eq!(at(&plain.yaw_error, 9), at(&f.yaw_error, 9));
    assert!(
        (10..25).any(|s| at(&plain.yaw_error, s) != at(&f.yaw_error, s)),
        "{:?}",
        f.yaw_error
    );
}

/// The four flights of the report, run with `--ignored --nocapture`.
#[test]
#[ignore = "a report of four flights, not a condition"]
fn the_four_check_flights() {
    let base = gust_hold();
    let plain = fly(&base, F405_LEAN, 500);
    println!(
        "plain: max {:.2} m horizontal, {:.2} m in 3-D, events {:?}",
        plain.max_h_m, plain.max_3d_m, plain.events
    );
    for (name, fault) in [
        ("mag-rotate 1.5 rad 10 s to 25 s", Some(rotate_1_5())),
        (
            "gps-vel-offset 5 m/s north 10 s to 11 s",
            Some(jump_5_north()),
        ),
        ("gps-vel-noise 2 m/s 10 s to 15 s", Some(noise_2())),
        ("gps latency 0.25 s", None),
    ] {
        let mut file = base.clone();
        match fault {
            Some(f) => file.faults = vec![f],
            None => file.gps_latency_us = Some(250_000),
        }
        let f = fly(&file, F405_LEAN, 500);
        let yaw: Vec<String> = f
            .yaw_error
            .iter()
            .filter(|(t, _)| t % 5 == 0 || (9..=11).contains(t))
            .map(|(t, e)| format!("{t}s {e:.3}"))
            .collect();
        println!("  heading error rad: {}", yaw.join(", "));
        println!(
            "{name}: max {:.2} m horizontal, {:.2} m in 3-D, replay {} ({} of {} ticks), events {:?}",
            f.max_h_m, f.max_3d_m, f.replay_identical, f.replayed_ticks, f.ticks, f.events
        );
    }
}

/// How far the filter's heading follows a rotated field, by angle and sign
/// (a step from 10 s to 25 s): the heading error at 12, 15, 20 and 24 s and
/// at 30 s. Run with `--ignored --nocapture`.
#[test]
#[ignore = "a report, not a condition"]
fn how_far_the_heading_follows_the_rotation() {
    let base = gust_hold();
    for angle_rad in [0.2, -0.2, 0.4, -0.4, 0.6, 1.0, -1.0, 1.5, 3.0] {
        let mut file = base.clone();
        file.faults = vec![fault(
            FaultKind::MagRotate {
                angle_rad,
                ramp_s: 0.0,
            },
            10.0,
            Some(25.0),
        )];
        let f = fly(&file, F405_LEAN, 500);
        let e: Vec<String> = [12, 15, 20, 24, 30]
            .iter()
            .map(|s| {
                let v = f.yaw_error.iter().find(|(t, _)| t == s).expect("second").1;
                format!("{s}s {v:+.3}")
            })
            .collect();
        println!(
            "rotation {angle_rad:+.1}: heading error {}; max {:.2} m; events {:?}",
            e.join(", "),
            f.max_3d_m,
            f.events.iter().skip(3).collect::<Vec<_>>()
        );
    }
}
