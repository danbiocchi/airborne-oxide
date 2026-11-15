// SPDX-License-Identifier: AGPL-3.0-or-later
//! Parameters at their bounds (M1c part two, wave 4, task P): a value inside
//! a declared range must not crash, panic or wedge the vehicle. The flights
//! here are the ones the final review flew.

use std::path::Path;

use oxide_core::config::VehicleConfig;
use oxide_core::io::DisarmReason;
use oxide_core::params::{self, ParamValue};
use oxide_core::schedule::{F405_LEAN, H743_FULL, Profile};
use oxide_sim::scenario_file::{self, ScenarioFile, Verdict};

const PROFILES: [(&str, Profile); 2] = [("f405", F405_LEAN), ("h743", H743_FULL)];

fn scenario(name: &str) -> ScenarioFile {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../scenarios")
        .join(format!("{name}.toml"));
    scenario_file::load_path(path).unwrap_or_else(|e| panic!("{e}"))
}

fn fly(file: &ScenarioFile, profile: Profile, seed: u64, p: &[(&str, f64)]) -> Verdict {
    let params: Vec<(String, f64)> = p.iter().map(|(n, v)| ((*n).to_string(), *v)).collect();
    file.fly_checked(profile, seed, &params)
        .unwrap_or_else(|e| panic!("{p:?}: {e}"))
}

/// The configuration of the scenario's vehicle on `profile`, as built.
fn config_of(file: &ScenarioFile, profile: Profile) -> VehicleConfig<4> {
    *file.build(profile, 0).expect("builds").core().config()
}

/// The largest value of `name` the rules accept on `cfg`, by bisection of
/// `params::set` between `lo` (accepted) and `hi` (refused).
fn largest_accepted(
    cfg: &VehicleConfig<4>,
    profile: &Profile,
    name: &str,
    lo: f32,
    hi: f32,
) -> f32 {
    let id = params::find(name).expect("parameter");
    let try_set = |v: f32| {
        let mut c = *cfg;
        params::set(&mut c, profile, id, ParamValue::F32(v)).is_ok()
    };
    assert!(try_set(lo), "{name} = {lo} must be accepted");
    assert!(!try_set(hi), "{name} = {hi} must be refused");
    let (mut ok, mut bad) = (lo, hi);
    for _ in 0..60 {
        let mid = 0.5 * (ok + bad);
        if mid == ok || mid == bad {
            break;
        }
        if try_set(mid) {
            ok = mid;
        } else {
            bad = mid;
        }
    }
    ok
}

#[test]
fn rc_loss_position_with_the_largest_accepted_idle_still_lands_and_disarms() {
    let f = scenario("rc-loss-position");
    for (pname, profile) in PROFILES {
        let idle = largest_accepted(&config_of(&f, profile), &profile, "LIM_IDLE", 0.05, 0.5);
        assert!(idle > 0.3 && idle < 0.35, "{pname}: largest idle {idle}");
        for seed in 0..3 {
            let v = fly(&f, profile, seed, &[("LIM_IDLE", f64::from(idle))]);
            assert!(
                v.violations.is_empty(),
                "{pname} seed {seed} LIM_IDLE {idle}: {:?}",
                v.violations
            );
            assert_eq!(v.outcome.disarm_reason, Some(DisarmReason::Touchdown));
            assert!(!v.outcome.armed_at_end);
        }
    }
}

/// The hold criteria of `gust-hold` pass (the scenario's own `[expect.hold]`).
fn assert_holds(name: &str, v: &Verdict, what: &str) {
    assert!(v.violations.is_empty(), "{name} {what}: {:?}", v.violations);
    assert!(v.hold.is_some(), "{name} {what}: no hold statistics");
}

#[test]
fn gust_hold_holds_at_the_smallest_output_time_constant_the_rules_allow() {
    let f = scenario("gust-hold");
    // The table minimum is 0.15 s and the time constant must be at least the
    // delay horizon: the minimum belongs with the shortest horizon that
    // still covers the GPS latency (0.15 s). Also the horizon's own upper
    // end, where the rule is tightest, and the default horizon.
    let cases: [(&str, [(&str, f64); 2]); 3] = [
        (
            "tau 0.15 horizon 0.15",
            [("EKF_OUT_TAU", 0.15), ("EKF_HORIZON", 0.15)],
        ),
        (
            "tau 0.2 horizon 0.2",
            [("EKF_OUT_TAU", 0.2), ("EKF_HORIZON", 0.2)],
        ),
        (
            "tau 0.248 horizon 0.248",
            [("EKF_OUT_TAU", 0.248), ("EKF_HORIZON", 0.248)],
        ),
    ];
    for (pname, profile) in PROFILES {
        for (what, p) in &cases {
            for seed in 0..3 {
                let v = fly(&f, profile, seed, p);
                assert_holds("gust-hold", &v, &format!("{pname} seed {seed} {what}"));
            }
        }
    }
}
