// SPDX-License-Identifier: AGPL-3.0-or-later
//! Parameters at their bounds (M1c part two, wave 4, task P): a value inside
//! a declared range must not crash, panic or wedge the vehicle. The flights
//! here are the ones the final review flew.

use std::path::Path;

use oxide_core::config::VehicleConfig;
use oxide_core::io::DisarmReason;
use oxide_core::math::Real;
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

#[test]
fn rc_loss_position_at_the_smallest_touchdown_speed_lands_and_disarms_by_touchdown() {
    // LAND_TD_SPEED 0.1 is the new minimum; LAND_SPEED and LIM_SPEED_DOWN at
    // the smallest values the rules then allow (twice that, 0.2 m/s).
    let mut f = scenario("rc-loss-position");
    // A landing at 0.2 m/s from the 10 m return height takes about 50 s: give
    // the flight the time (the file's own 80 s is for 0.7 m/s).
    f.duration_s = 160.0;
    if let Some(e) = f.expect.as_mut() {
        e.by_s = 150.0;
    }
    let p = [
        ("LAND_TD_SPEED", 0.1),
        ("LAND_SPEED", 0.2),
        ("LIM_SPEED_DOWN", 0.2),
    ];
    for (pname, profile) in PROFILES {
        for seed in 0..5 {
            let v = fly(&f, profile, seed, &p);
            assert!(
                v.violations.is_empty(),
                "{pname} seed {seed}: {:?}",
                v.violations
            );
            assert_eq!(
                v.outcome.disarm_reason,
                Some(DisarmReason::Touchdown),
                "{pname} seed {seed}"
            );
            let speed = v.outcome.touchdown_speed_mps.expect("touched down");
            let limit = f.expect.as_ref().map_or(1.5, |e| e.max_touchdown_mps);
            assert!(speed <= limit, "{pname} seed {seed}: touchdown {speed} m/s");
        }
    }
}

#[test]
fn a_touchdown_time_of_a_tenth_of_a_second_does_not_disarm_above_the_ground() {
    let f = scenario("rc-loss-position");
    for (pname, profile) in PROFILES {
        for seed in 0..3 {
            let params = vec![("LAND_TD_TIME".to_string(), 0.1)];
            let mut sim = f.build_with_params(profile, seed, &params).expect("builds");
            let mut was_armed = false;
            let mut disarm_height = None;
            sim.run_until(f.end(), |r| {
                if was_armed && !r.status.armed && disarm_height.is_none() {
                    disarm_height = Some(-r.position.z);
                }
                was_armed |= r.status.armed;
            });
            let h = disarm_height.expect("disarmed");
            assert!(
                h < 0.3,
                "{pname} seed {seed}: disarmed {h} m above the ground"
            );
        }
    }
}

// ---- every bound flies ----

/// The most a bound may do to a flight: not fly away, blow up or panic.
const RUNAWAY_M: f64 = 200.0;

/// What one flight at one bound did wrong; empty = nothing.
fn fly_bound(f: &ScenarioFile, name: &str, value: f64) -> Vec<String> {
    let params = vec![(name.to_string(), value)];
    let run = std::panic::catch_unwind(|| {
        let mut sim = f.build_with_params(F405_LEAN, 0, &params).expect("builds");
        let mut bad = Vec::new();
        let mut start: Option<oxide_core::math::Vec3<f64>> = None;
        let (mut max_h, mut max_d, mut timing) = (0.0_f64, 0.0_f64, 0);
        let mut finite = true;
        sim.run_until(f.end(), |r| {
            let est_ok = r.estimate.is_none_or(|e| {
                e.position.is_finite()
                    && e.velocity.is_finite()
                    && e.attitude.is_finite()
                    && e.rate.is_finite()
            });
            finite &= r.position.is_finite()
                && r.velocity.is_finite()
                && r.attitude.is_finite()
                && r.rate.is_finite()
                && r.motors.iter().all(|m| m.is_finite())
                && r.battery_v.is_finite()
                && est_ok;
            let s = *start.get_or_insert(r.position);
            max_h = max_h.max(-r.position.z);
            if r.position.is_finite() {
                let dn = r.position.x - s.x;
                let de = r.position.y - s.y;
                max_d = max_d.max(Real::sqrt(dn * dn + de * de));
            }
            timing = r.status.timing_faults;
        });
        if !finite {
            bad.push("a truth or estimate value is not finite".to_string());
        }
        if timing != 0 {
            bad.push(format!("{timing} timing faults"));
        }
        if max_d.is_nan() || max_d > RUNAWAY_M || max_h.is_nan() || max_h > RUNAWAY_M {
            bad.push(format!("ran away: {max_d:.0} m out, {max_h:.0} m high"));
        }
        bad
    });
    match run {
        Ok(bad) => bad,
        Err(p) => {
            let msg = p
                .downcast_ref::<String>()
                .cloned()
                .or_else(|| p.downcast_ref::<&str>().map(|s| (*s).to_string()))
                .unwrap_or_default();
            vec![format!("panicked: {msg}")]
        }
    }
}

/// A bound that does not fly well on the reference quad and is known: the
/// parameter, the bound, the scenario, and why.
const KNOWN: &[(&str, &str, &str, &str)] = &[
    (
        "POS_VELX_P",
        "min",
        "rc-loss-position",
        "a zero north velocity gain leaves the horizontal speed uncontrolled: 250 m out \
         (the review's zero-p-gains finding; a floor above zero needs the loops' own tuning)",
    ),
    (
        "EKF_MAG_DECL",
        "min",
        "rc-loss-position",
        "the simulated site has declination 0, so -pi turns the heading estimate round and the \
         position estimate disagrees with it: 1377 m out. Real declinations reach pi only at \
         the geomagnetic poles, so the range is right and the value is wrong for this site",
    ),
    (
        "EKF_MAG_DECL",
        "max",
        "rc-loss-position",
        "as the minimum: +pi against a site with declination 0",
    ),
];

#[test]
#[ignore = "flies every bound of every parameter: run with --release -- --ignored"]
fn every_bound_flies() {
    let files = [scenario("gust-hold"), scenario("rc-loss-position")];
    let cfg = config_of(&files[0], F405_LEAN);
    let mut jobs: Vec<(&str, &str, f64)> = Vec::new();
    let mut refused: Vec<String> = Vec::new();
    for (i, def) in params::PARAMS.iter().enumerate() {
        for (bound, v) in [("min", def.min), ("max", def.max)] {
            let mut c = cfg;
            match params::set(&mut c, &F405_LEAN, params::ParamId(i as u16), v) {
                Ok(()) => {
                    let x = match v {
                        ParamValue::F32(x) => f64::from(x),
                        ParamValue::I32(x) => f64::from(x),
                    };
                    jobs.push((def.name, bound, x));
                }
                Err(params::ParamError::Invalid(why)) => {
                    refused.push(format!("{} {bound} {v:?}: {why:?}", def.name));
                }
                Err(e) => panic!("{} {bound} {v:?}: {e:?}: the table lies", def.name),
            }
        }
    }
    let n = jobs.len() * files.len();
    let results = oxide_sim::montecarlo::run_indexed(n, 2, |k| {
        let (name, bound, value) = jobs[k / files.len()];
        let f = &files[k % files.len()];
        (name, bound, f.name.clone(), fly_bound(f, name, value))
    });
    let mut failing: Vec<(String, String, String, Vec<String>)> = Vec::new();
    for (name, bound, scen, bad) in results {
        if !bad.is_empty() {
            failing.push((name.to_string(), bound.to_string(), scen, bad));
        }
    }
    let is_known =
        |n: &str, b: &str, s: &str| KNOWN.iter().any(|k| k.0 == n && k.1 == b && k.2 == s);
    let (known, unknown): (Vec<_>, Vec<_>) =
        failing.iter().partition(|(n, b, s, _)| is_known(n, b, s));
    println!(
        "bounds flown {} ({} flights), refused by a rule {}, known {}, failing {}",
        jobs.len(),
        n,
        refused.len(),
        known.len(),
        unknown.len()
    );
    // The line the report quotes: bounds, not flights.
    println!(
        "summary: {} bounds flown, {} refused by a rule on the reference quad, {} known",
        jobs.len(),
        refused.len(),
        KNOWN.len()
    );
    for r in &refused {
        println!("  refused by a rule: {r}");
    }
    for (n, b, s, bad) in &failing {
        println!(
            "  {} {n} {b} {s}: {bad:?}",
            if is_known(n, b, s) { "known" } else { "FAIL" }
        );
    }
    assert!(unknown.is_empty(), "bounds that do not fly: {unknown:#?}");
    for (n, b, s, why) in KNOWN {
        assert!(
            failing.iter().any(|f| f.0 == *n && f.1 == *b && f.2 == *s),
            "{n} {b} {s} is listed as known ({why}) but flies now: remove it"
        );
    }
}
