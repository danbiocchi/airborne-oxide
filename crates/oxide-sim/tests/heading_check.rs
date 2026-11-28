// SPDX-License-Identifier: AGPL-3.0-or-later
//! The heading check (stage 1, piece P1 of the heading guard) in a whole
//! flight: the supervisor's airborne state reaches it, a wrong declination
//! gets a verdict, a right heading never does, and the verdict acts only
//! when it is given (the heading guard's second part): until its first
//! correction a flight is the flight with the check silenced by its
//! parameters, bit for bit.

use std::path::Path;

use oxide_core::schedule::{F405_LEAN, H743_FULL, Profile};
use oxide_sim::flight_log::{Level, LogBuild, SharedBuffer};
use oxide_sim::replay::{ReplayOptions, Stop, replay};
use oxide_sim::scenario_file;

/// What a flight leaves of the check.
struct Outcome {
    /// Fixes judged before the vehicle was first armed (none expected).
    judged_before_arming: u32,
    /// Fixes judged by the end.
    judged: u32,
    /// Fixes at which the verdict was `Correct`.
    verdicts: u32,
    /// `t_sample` of the first fix with the verdict, s.
    first_verdict_s: Option<f64>,
    /// Index in `trace` of the first tick with a correction made.
    first_correction_tick: Option<usize>,
    /// Corrections made and the latch, at the end.
    corrections: u8,
    contained: bool,
    /// Truth and estimate of every tick, as bits.
    trace: Vec<[u64; 8]>,
}

fn fly(scenario: &str, profile: Profile, index: u64, params: &[(&str, f64)]) -> Outcome {
    let path =
        Path::new(env!("CARGO_MANIFEST_DIR")).join(format!("../../scenarios/{scenario}.toml"));
    let file = scenario_file::load_path(&path).expect("the file loads");
    let list: Vec<(String, f64)> = params.iter().map(|(n, v)| ((*n).to_string(), *v)).collect();
    let mut sim = file
        .build_with_params(profile, index, &list)
        .expect("builds");
    let end = file.end();
    let mut out = Outcome {
        judged_before_arming: 0,
        judged: 0,
        verdicts: 0,
        first_verdict_s: None,
        first_correction_tick: None,
        corrections: 0,
        contained: false,
        trace: Vec::new(),
    };
    let mut armed_once = false;
    loop {
        let r = sim.step();
        if r.t >= end {
            break;
        }
        armed_once |= r.armed;
        let fit = sim.core().telemetry().heading_fit;
        if !armed_once {
            out.judged_before_arming = fit.seq;
        }
        if fit.verdict > 0 && out.first_verdict_s.is_none() {
            out.first_verdict_s = Some(fit.t_us as f64 * 1e-6);
        }
        let est = r.core.and_then(|c| c.estimate);
        out.trace.push([
            r.position.x.to_bits(),
            r.position.y.to_bits(),
            r.position.z.to_bits(),
            r.velocity.x.to_bits(),
            r.velocity.y.to_bits(),
            r.attitude.z.to_bits(),
            est.map_or(0, |e| u64::from(e.position.x.to_bits())),
            est.map_or(0, |e| u64::from(e.attitude.z.to_bits())),
        ]);
        out.judged = fit.seq;
        if out.first_correction_tick.is_none() && sim.core().estimator().heading_corrections() > 0 {
            out.first_correction_tick = Some(out.trace.len() - 1);
        }
    }
    out.verdicts = sim.core().estimator().heading_verdicts();
    out.corrections = sim.core().estimator().heading_corrections();
    out.contained = sim.core().estimator().heading_contained();
    out
}

/// The check's own parameters set so that it can never say anything: a
/// threshold of 1 rad at 5 sigma held for 5 s.
const SILENT: [(&str, f64); 3] = [("EKF_HDG_THR", 1.0), ("EKF_HDG_K", 5.0), ("EKF_HDG_T", 5.0)];

#[test]
fn the_airborne_state_reaches_the_check_and_a_right_heading_gets_no_verdict() {
    for profile in [F405_LEAN, H743_FULL] {
        let o = fly("gust-hold", profile, 500, &[]);
        assert_eq!(o.judged_before_arming, 0, "{}", profile.name);
        // Take-off at about 3 s of a 40 s flight at 5 fixes a second.
        assert!(
            o.judged > 100,
            "{}: {} fixes judged",
            profile.name,
            o.judged
        );
        assert_eq!(o.verdicts, 0, "{}", profile.name);
    }
}

#[test]
fn a_wrong_declination_is_corrected_and_until_then_the_flight_is_the_silenced_one() {
    for (profile, index) in [(F405_LEAN, 500), (H743_FULL, 501)] {
        let mut wrong = vec![("EKF_MAG_DECL", 1.0)];
        let loud = fly("gust-hold", profile, index, &wrong);
        assert_eq!(loud.judged_before_arming, 0);
        assert!(loud.verdicts > 0, "{}: no verdict", profile.name);
        let t = loud.first_verdict_s.expect("a first verdict");
        // D2 measured 4 to 6 s after take-off for 1 rad.
        assert!(
            (4.0..14.0).contains(&t),
            "{}: verdict at {t} s",
            profile.name
        );
        // One verdict, one correction, no containment.
        assert_eq!(loud.corrections, 1, "{}", profile.name);
        assert!(!loud.contained, "{}", profile.name);
        // The same flight with the check silenced never corrects, and is the
        // same flight bit for bit until the correction (the plumbing adds
        // nothing before it).
        wrong.extend(SILENT);
        let quiet = fly("gust-hold", profile, index, &wrong);
        assert_eq!(quiet.verdicts, 0, "{}", profile.name);
        assert_eq!(quiet.corrections, 0, "{}", profile.name);
        assert_eq!(loud.trace.len(), quiet.trace.len());
        let first = loud.first_correction_tick.expect("a correction tick");
        assert!(
            loud.trace[..first] == quiet.trace[..first],
            "{}: the flights differ before the correction (tick {first})",
            profile.name
        );
        assert!(
            loud.trace[first..] != quiet.trace[first..],
            "{}: the correction changed nothing",
            profile.name
        );
    }
}

#[test]
fn the_check_parameters_change_nothing_in_a_healthy_flight() {
    let a = fly("rc-loss-position", F405_LEAN, 500, &[]);
    let b = fly("rc-loss-position", F405_LEAN, 500, &SILENT);
    assert_eq!(a.verdicts, 0);
    assert!(a.judged > 100);
    assert!(a.trace == b.trace);
}

/// A vehicle armed and idling on the ground with a wrong declination: the
/// supervisor's flag is `armed && !landed`, and the landing detector says
/// "not landed" while armed with thrust up, so the check is live between
/// arming and lift-off. Whether anything acts there: nothing does. The
/// vehicle does not move, so no interval has any acceleration in it
/// (`Sa` stays below its minimum) and the check never says anything; the
/// correction and the latch are never set.
#[test]
fn a_vehicle_idling_armed_on_the_ground_with_a_wrong_declination_is_not_corrected() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data/armed-idle.toml");
    let file = scenario_file::load_path(&path).expect("the file loads");
    for profile in [F405_LEAN, H743_FULL] {
        let params = vec![("EKF_MAG_DECL".to_string(), 1.0)];
        let mut sim = file.build_with_params(profile, 1, &params).expect("builds");
        let end = file.end();
        let (mut armed_s, mut judged_armed, mut max_sa) = (0.0_f64, 0_u32, 0.0_f32);
        let mut armed_at = None;
        let mut flew = false;
        loop {
            let r = sim.step();
            if r.t >= end {
                break;
            }
            if r.armed {
                armed_at.get_or_insert(r.t.as_secs_f64());
                armed_s = r.t.as_secs_f64() - armed_at.unwrap_or(0.0);
                let fit = sim.core().telemetry().heading_fit;
                judged_armed = fit.seq;
                max_sa = max_sa.max(fit.sa);
            }
            flew |= r.position.z < -0.3;
            let est = sim.core().estimator();
            assert_eq!(est.heading_corrections(), 0, "{}", profile.name);
            assert!(!est.heading_contained(), "{}", profile.name);
            assert_eq!(est.declination_shift(), 0.0, "{}", profile.name);
        }
        assert!(!flew, "{}: the vehicle left the ground", profile.name);
        assert!(armed_s > 10.0, "{}: armed for {armed_s} s", profile.name);
        // The check was live for a moment (fixes were judged while armed on
        // the ground, until the landing detector said landed: the idle
        // thrust of this airframe is below its touchdown fraction) and never
        // said anything.
        assert!(judged_armed >= 1, "{}: {judged_armed} fixes", profile.name);
        assert_eq!(sim.core().estimator().heading_verdicts(), 0);
        assert!(max_sa < 0.3, "{}: Sa reached {max_sa}", profile.name);
    }
}

/// The replay log carries what the core was given, not what the check
/// computed: a flight whose heading was corrected (and one that was
/// contained) replays bit for bit, because the correction is in the outputs
/// the replay compares (the attitude, the set-points, the motors, the status).
#[test]
fn corrected_and_contained_flights_replay_bit_for_bit() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scenarios/gust-hold.toml");
    let file = scenario_file::load_path(&path).expect("the file loads");
    for (profile, index, nmax) in [
        (F405_LEAN, 500, 2.0),
        (H743_FULL, 501, 2.0),
        (F405_LEAN, 502, 0.0),
    ] {
        let params = vec![
            ("EKF_MAG_DECL".to_string(), 1.0),
            ("EKF_HDG_NMAX".to_string(), nmax),
        ];
        let mut sim = file
            .build_with_params(profile, index, &params)
            .expect("builds");
        let buf = SharedBuffer::new();
        sim.start_log(
            Level::Replay,
            &LogBuild::sim("heading-guard"),
            Box::new(buf.clone()),
        )
        .expect("the replay level starts");
        let mut flown = 0_u64;
        let (mut corrected, mut contained) = (false, false);
        loop {
            let r = sim.step();
            flown += 1;
            if r.t >= file.end() {
                break;
            }
            corrected |= sim.core().estimator().heading_corrections() > 0;
            contained |= sim.core().estimator().heading_contained();
        }
        sim.finish_log().expect("finishes");
        // Both are cleared at the disarm after the landing (the core tells
        // the estimator the supervisor's `armed`): the flights end landed.
        if nmax <= 0.0 {
            let est = sim.core().estimator();
            assert!(!est.heading_contained() && est.heading_corrections() == 0);
            assert_eq!(est.declination_shift(), 0.0);
        }
        if nmax > 0.0 {
            assert!(corrected && !contained, "{}: corrected", profile.name);
        } else {
            assert!(contained && !corrected, "{}: contained", profile.name);
        }
        let r = replay(&buf.bytes(), &ReplayOptions::default()).expect("replays");
        eprintln!(
            "{}: {} of {flown}: {:?} {:?}",
            profile.name, r.ticks, r.stop, r.first_difference
        );
        assert_eq!(r.ticks, flown, "{}", profile.name);
        assert!(
            r.identical() && r.stop == Stop::End,
            "{}: {}",
            profile.name,
            r.lines().join("; ")
        );
    }
}

/// A flight with a non-zero `EKF_MAG_DECL` that the guard never corrects is
/// the flight the reference simulator (built before any of the guard's code)
/// flies, bit for bit: the alignment takes the declination parameter as it
/// is when there is no shift (no wrap, no recomputed cosine and sine). The
/// pinned hashes are the reference simulator's own for these flights.
#[test]
fn an_uncorrected_flight_with_a_declination_is_the_flight_before_the_guard() {
    for (decl, index, reference) in [
        (0.3_f64, 500_u64, 0x47cd_7094_5b13_926e_u64),
        (0.7, 500, 0xe07b_80e8_7562_2c05),
    ] {
        let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scenarios/gust-hold.toml");
        let file = scenario_file::load_path(&path).expect("the file loads");
        let params = vec![("EKF_MAG_DECL".to_string(), decl)];
        let mut sim = file
            .build_with_params(F405_LEAN, index, &params)
            .expect("builds");
        sim.run_until(oxide_core::time::Micros(file.end().0), |_| {});
        assert_eq!(
            sim.core().estimator().heading_corrections(),
            0,
            "decl {decl}: the flight must not be corrected"
        );
        assert_eq!(
            sim.trace_hash(),
            reference,
            "decl {decl}: not the reference simulator's flight"
        );
    }
}

/// A flight with a fine step replays bit for bit on both profiles: the step
/// is in the outputs the replay compares (the attitude, the reset count, the
/// set-points), and the log carries its count and its event.
#[test]
fn flights_with_a_fine_step_replay_bit_for_bit_and_log_the_step() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scenarios/gust-hold.toml");
    let file = scenario_file::load_path(&path).expect("the file loads");
    for (profile, index) in [(F405_LEAN, 502), (H743_FULL, 501)] {
        let params = vec![("EKF_MAG_DECL".to_string(), 1.0)];
        let mut sim = file
            .build_with_params(profile, index, &params)
            .expect("builds");
        let buf = SharedBuffer::new();
        sim.start_log(
            Level::Replay,
            &LogBuild::sim("heading-guard"),
            Box::new(buf.clone()),
        )
        .expect("the replay level starts");
        let mut flown = 0_u64;
        let (mut fine, mut seen_in_log) = (0_u8, 0_u8);
        loop {
            let r = sim.step();
            flown += 1;
            if r.t >= file.end() {
                break;
            }
            fine = fine.max(sim.core().estimator().heading_fine_steps());
            seen_in_log = seen_in_log.max(sim.core().telemetry().heading_guard.fine_steps);
        }
        sim.finish_log().expect("finishes");
        assert_eq!(fine, 1, "{}: one fine step", profile.name);
        assert_eq!(
            seen_in_log, 1,
            "{}: in the telemetry the log writes",
            profile.name
        );
        let r = replay(&buf.bytes(), &ReplayOptions::default()).expect("replays");
        assert_eq!(r.ticks, flown, "{}", profile.name);
        assert!(
            r.identical() && r.stop == Stop::End,
            "{}: {}",
            profile.name,
            r.lines().join("; ")
        );
    }
}
