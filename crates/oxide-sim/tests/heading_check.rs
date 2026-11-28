// SPDX-License-Identifier: AGPL-3.0-or-later
//! The heading check (stage 1, piece P1 of the heading guard) in a whole
//! flight: the supervisor's airborne state reaches it, a wrong declination
//! gets a verdict, a right heading never does, and **no verdict changes a
//! flight**: the same flight with the check's parameters set so that it can
//! never say anything is the same flight, bit for bit.

use std::path::Path;

use oxide_core::schedule::{F405_LEAN, H743_FULL, Profile};
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
    }
    out.verdicts = sim.core().estimator().heading_verdicts();
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
fn a_wrong_declination_gets_a_verdict_and_the_verdict_changes_nothing() {
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
        // The same flight with the check silenced: bit for bit the same.
        wrong.extend(SILENT);
        let quiet = fly("gust-hold", profile, index, &wrong);
        assert_eq!(quiet.verdicts, 0, "{}", profile.name);
        assert_eq!(loud.trace.len(), quiet.trace.len());
        assert!(
            loud.trace == quiet.trace,
            "{}: a verdict changed the flight",
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
