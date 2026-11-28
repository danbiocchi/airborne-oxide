// SPDX-License-Identifier: AGPL-3.0-or-later
//! Tests of the scenario file loader.

use super::*;

const MINIMAL: &str = "name = \"minimal\"\nduration_s = 5.0\n";

#[test]
fn a_file_with_only_the_required_keys_loads_with_the_defaults() {
    let s = load_str(MINIMAL).expect("loads");
    assert_eq!(s.name, "minimal");
    assert_eq!(s.about, "");
    assert_eq!(s.airframe, REFERENCE_AIRFRAME);
    assert_eq!(s.duration_s, 5.0);
    assert_eq!(s.sensors, SensorPreset::Ideal);
    assert_eq!(s.estimator, EstimatorSource::External);
    assert_eq!(s.links, LinkConfig::default());
    assert_eq!(s.wind, WindSpec::Calm);
    assert_eq!(s.battery, None);
    assert!(s.params.is_empty() && s.faults.is_empty());
    assert_eq!(
        s.pilot,
        Script {
            duration_s: 5.0,
            ..Script::default()
        }
    );
    assert_eq!(s.expect, None);
    assert_eq!(s.hold, None);
    assert_eq!(s.run, RunSpec::default());
}

/// The commented example in the module documentation.
fn documented_example() -> String {
    let src = include_str!("../scenario_file.rs");
    let mut out = String::new();
    let mut inside = false;
    for line in src.lines() {
        let Some(doc) = line.strip_prefix("//!") else {
            continue;
        };
        let doc = doc.strip_prefix(' ').unwrap_or(doc);
        if doc.starts_with("```toml") {
            inside = true;
        } else if doc.starts_with("```") {
            inside = false;
        } else if inside {
            out.push_str(doc);
            out.push('\n');
        }
    }
    out
}

#[test]
fn the_documented_example_loads() {
    let text = documented_example();
    assert!(text.contains("[[pilot.setpoint]]"), "example not found");
    let s = load_str(&text).expect("the example in the module documentation loads");
    assert_eq!(s.name, "gps-loss");
    assert_eq!(s.params, vec![("FS_GPS_LAND".to_string(), 10.0)]);
    assert_eq!(s.faults.len(), 1);
    assert_eq!(s.pilot.mode, vec![(0.0, 2)]);
    // Heights are up-positive in files and NED in memory.
    assert_eq!(s.pilot.setpoints.len(), 2);
    assert_eq!(s.pilot.setpoints[1].1, None);
    let e = s.expect.expect("expect");
    assert_eq!(e.nav_contains, vec![NavState::Hold, NavState::Land]);
    assert_eq!(e.reasons, Reasons::GPS_LOST);
    assert_eq!(
        e.ending,
        Ending::Landed {
            within_m_of_home: Some(40.0)
        }
    );
}

#[test]
fn setpoint_heights_are_up_positive() {
    let s = load_str(
        "name = \"x\"\nduration_s = 5.0\n[pilot]\n[[pilot.setpoint]]\nt = 0.0\nposition = [1.0, 2.0, 5.0]\nyaw = 0.5\n",
    )
    .expect("loads");
    let sp = s.pilot.setpoints[0].1.expect("setpoint");
    assert_eq!(sp.position, Vec3::new(1.0, 2.0, -5.0));
    assert_eq!(sp.yaw, 0.5);
    assert_eq!(sp.velocity, Vec3::zero());
}

// ---- The files in scenarios/ -----------------------------------------------------

use crate::fault_scenarios;
use crate::scenario::{GustHold, HoverStep, gust_hold};
use oxide_core::schedule::{F405_LEAN, H743_FULL};

/// The directory of the scenario files in the repository.
fn scenario_dir() -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scenarios")
}

#[test]
fn every_file_in_scenarios_loads_and_is_named_after_its_file() {
    let mut n = 0;
    for entry in std::fs::read_dir(scenario_dir()).expect("scenarios/") {
        let path = entry.expect("entry").path();
        let f = load_path(&path).unwrap_or_else(|e| panic!("{e}"));
        assert_eq!(
            path.file_stem().and_then(|s| s.to_str()),
            Some(f.name.as_str())
        );
        n += 1;
    }
    assert_eq!(n, 44);
}

#[test]
fn failsafe_overrides_are_named_parameters_in_the_file() {
    let f = fault_scenarios::by_name("pair-rc-loss+geofence").expect("pair");
    assert_eq!(
        f.params,
        vec![
            ("FS_FENCE_RAD".to_string(), 10.0),
            ("FS_GPS_LAND".to_string(), 10.0),
            ("FS_NOLINK_RTL".to_string(), 5.0),
        ]
    );
    let none = fault_scenarios::by_name("gcs-loss").expect("scenario");
    assert!(none.params.is_empty());
}

#[test]
fn computed_battery_sizes_are_written_as_literals() {
    let f = fault_scenarios::by_name("pair-battery-low+battery-critical").expect("pair");
    let b = f.battery.expect("a battery override");
    // 500 mAh; 0.05 + 0.0048 * (15 - 3 - 3) in f64.
    assert_eq!(b.capacity_mah, Some(500.0));
    assert_eq!(
        b.initial_state_of_charge,
        Some(0.05 + 0.0048 * (15.0 - 3.0 - 3.0))
    );
}

#[test]
fn a_combination_valid_only_together_loads_from_a_scenario_file() {
    // The critical battery level above the default low level breaks a rule on
    // its own, and the table is read sorted by name, critical first. The
    // values are applied as one change, so the file loads.
    let s =
        load_str("name = \"x\"\nduration_s = 5.0\n[params]\nBAT_LOW_V = 3.8\nBAT_CRIT_V = 3.6\n")
            .expect("loads");
    assert_eq!(s.params.len(), 2);
    // Together they still have to hold: this pair does not, though each is
    // fine on its own (critical 3.45 against the default low 3.5, low 3.4
    // against the default critical 3.3).
    let e = refused("[params]\nBAT_CRIT_V = 3.45\nBAT_LOW_V = 3.4\n");
    assert!(e.reason.contains("together"), "{e}");
    // An unknown name is still named.
    let e = refused("[params]\nFS_NOLINK_RTL = 5.0\nEKF_HGT_TMO = 2\n");
    assert_eq!(e.key, "params.EKF_HGT_TMO");
}

// ---- hover-step and gust-hold ---------------------------------------------------

/// Hash of the Rust `HoverStep` flight (seed 0, ideal sensors, true state).
fn hover_step_hash(profile: Profile) -> u64 {
    let airframe = Airframe::reference_quad().expect("airframe");
    let mut sim =
        Simulation::<_, 4>::new(&airframe, profile, HoverStep::default()).expect("builds");
    sim.run_until(Micros(20_000_000), |_| {});
    sim.trace_hash()
}

fn file_hash(f: &ScenarioFile, profile: Profile, index: u64) -> u64 {
    let mut sim = f.build(profile, index).expect("builds");
    sim.run_until(Micros((f.duration_s * 1e6).round() as u64), |_| {});
    sim.trace_hash()
}

#[test]
fn hover_step_as_a_script_flies_the_same_flight() {
    let f = fault_scenarios::by_name("hover-step").expect("hover-step");
    assert_eq!(f.duration_s, HoverStep::default().duration_s);
    // The hashes `oxide-sim hash` prints.
    assert_eq!(hover_step_hash(F405_LEAN), 0x3de1_6d7a_9847_deaf);
    assert_eq!(hover_step_hash(H743_FULL), 0x9ff0_2684_5c34_2fbd);
    for profile in [F405_LEAN, H743_FULL] {
        // The Rust flight always has seed 0; ideal sensors and a calm wind
        // leave nothing for another seed to change.
        for index in 0..2 {
            assert_eq!(
                file_hash(&f, profile, index),
                hover_step_hash(profile),
                "{} index {index}",
                profile.name
            );
        }
    }
}

#[test]
fn gust_hold_as_a_script_flies_the_same_flight() {
    let f = fault_scenarios::by_name("gust-hold").expect("gust-hold");
    assert_eq!(f.duration_s, GustHold::default().duration_s);
    let airframe = Airframe::reference_quad().expect("airframe");
    for profile in [F405_LEAN, H743_FULL] {
        for index in 0..2 {
            let (pilot, options) = gust_hold(index);
            let mut sim = Simulation::<_, 4>::with_options(&airframe, profile, pilot, options)
                .expect("builds");
            sim.run_until(Micros(40_000_000), |_| {});
            assert_eq!(
                file_hash(&f, profile, index),
                sim.trace_hash(),
                "{} index {index}",
                profile.name
            );
        }
    }
}

// ---- Refusals -----------------------------------------------------------------

/// The error of loading `MINIMAL` followed by `extra`.
fn refused(extra: &str) -> ScenarioError {
    load_str(&format!("{MINIMAL}{extra}")).expect_err("the file must be refused")
}

#[test]
fn an_unknown_key_is_refused_naming_it() {
    let e = refused("colour = \"red\"\n");
    assert_eq!(e.key, "colour", "{e}");
    assert!(e.reason.contains("unknown field"), "{e}");
    // In a nested table too, with its line.
    let e = refused("[sim]\nsensor = \"ideal\"\n");
    assert_eq!(e.key, "sensor", "{e}");
    assert!(e.reason.contains("line 4"), "{e}");
    let e = refused("[expect.ending]\nkind = \"down\"\ntolerance_m = 1.0\n");
    assert_eq!(e.key, "tolerance_m", "{e}");
    let e = refused("[expect.ending]\nkind = \"terminated\"\nby = 1.0\n");
    assert_eq!(e.key, "by", "{e}");
    let e = refused("[wind]\nkind = \"calm\"\nsteady_ned = [1.0, 0.0, 0.0]\n");
    assert_eq!(e.key, "steady_ned", "{e}");
}

#[test]
fn an_unknown_state_name_is_refused_naming_the_key() {
    let e = refused("[expect]\nnav_contains = [\"hold\", \"Lunch\"]\n");
    assert_eq!(e.key, "expect.nav_contains[1]", "{e}");
    assert!(
        e.reason.contains("`Lunch`") && e.reason.contains("land-now"),
        "{e}"
    );
    let e = refused("[expect]\nreasons = [\"RC_LOST\", \"BORED\"]\n");
    assert_eq!(e.key, "expect.reasons[1]", "{e}");
    let e = refused("[expect]\nmin_action = \"panic\"\n");
    assert_eq!(e.key, "expect.min_action", "{e}");
    let e = refused("[expect]\nevents = [\"armed\", \"landed\"]\n");
    assert_eq!(e.key, "expect.events[1]", "{e}");
    let e = refused("[expect.ending]\nkind = \"disarmed\"\nreason = \"boredom\"\n");
    assert_eq!(e.key, "expect.ending.reason", "{e}");
    let e = refused("[expect.ending]\nkind = \"crashed\"\n");
    assert_eq!(e.key, "expect.ending.kind", "{e}");
    let e = refused("[[pilot.mode]]\nt = 0.0\nswitch = \"acro\"\n");
    assert_eq!(e.key, "pilot.mode[0].switch", "{e}");
    let e = refused("[sim]\nsensors = \"perfect\"\n");
    assert_eq!(e.key, "sim.sensors", "{e}");
    let e = refused("[run]\nprofiles = [\"f405\", \"f777\"]\n");
    assert_eq!(e.key, "run.profiles[1]", "{e}");
}

#[test]
fn an_unsorted_timeline_is_refused_naming_the_entry() {
    let e = refused(
        "[[pilot.setpoint]]\nt = 4.0\nposition = [0.0, 0.0, 1.0]\n[[pilot.setpoint]]\nt = 2.0\nsilent = true\n",
    );
    assert_eq!(e.key, "pilot.setpoint[1].t", "{e}");
    assert!(e.reason.contains("increasing"), "{e}");
    let e = refused(
        "[[pilot.mode]]\nt = 1.0\nswitch = \"position\"\n[[pilot.mode]]\nt = 1.0\nswitch = \"offboard\"\n",
    );
    assert_eq!(e.key, "pilot.mode[1].t", "{e}");
    let e = refused("[[pilot.sticks]]\nt = 3.0\n[[pilot.sticks]]\nt = 1.0\n");
    assert_eq!(e.key, "pilot.sticks[1].t", "{e}");
}

#[test]
fn a_missing_required_key_is_refused_naming_it() {
    let e = load_str("duration_s = 5.0\n").expect_err("no name");
    assert_eq!(e.key, "name", "{e}");
    assert!(e.reason.contains("missing"), "{e}");
    let e = load_str("name = \"x\"\n").expect_err("no duration");
    assert_eq!(e.key, "duration_s", "{e}");
    let e = refused("[[fault]]\nkind = \"gps-loss\"\n");
    assert_eq!(e.key, "fault[0].start_s", "{e}");
    let e = refused("[[fault]]\nkind = \"motor-loss\"\nstart_s = 1.0\nrotor = 0\n");
    assert_eq!(e.key, "fault[0].scale", "{e}");
    let e = refused("[[pilot.setpoint]]\nt = 0.0\n");
    assert_eq!(e.key, "pilot.setpoint[0].position", "{e}");
}

#[test]
fn a_bad_parameter_is_refused_naming_it() {
    let e = refused("[params]\nFS_NO_SUCH = 1.0\n");
    assert_eq!(e.key, "params.FS_NO_SUCH", "{e}");
    assert!(e.reason.contains("no such parameter"), "{e}");
    // A value out of range, a wrong type, and one that is too large for i32.
    let e = refused("[params]\nFS_GPS_LAND = -1.0\n");
    assert_eq!(e.key, "params.FS_GPS_LAND", "{e}");
    assert!(e.reason.contains("range"), "{e}");
    let int = oxide_core::params::PARAMS
        .iter()
        .find(|d| d.kind == oxide_core::params::ParamKind::I32)
        .map(|d| d.name);
    if let Some(name) = int {
        let e = refused(&format!("[params]\n{name} = 1.5\n"));
        assert_eq!(e.key, format!("params.{name}"), "{e}");
        assert!(e.reason.contains("whole number"), "{e}");
        let e = refused(&format!("[params]\n{name} = 99999999999\n"));
        assert!(e.reason.contains("range"), "{e}");
    }
}

#[test]
fn faults_the_simulator_would_refuse_are_refused_at_load() {
    // A rotor the airframe does not have.
    let e = refused("[[fault]]\nkind = \"motor-loss\"\nstart_s = 1.0\nrotor = 4\nscale = 0.0\n");
    assert_eq!(e.key, "fault[0]", "{e}");
    // A scale outside 0..=1.
    let e = refused("[[fault]]\nkind = \"motor-loss\"\nstart_s = 1.0\nrotor = 0\nscale = 2.0\n");
    assert_eq!(e.key, "fault[0]", "{e}");
    // A ground-station fault without a ground station.
    let e = refused("[[fault]]\nkind = \"gcs-loss\"\nstart_s = 1.0\n");
    assert_eq!(e.key, "fault[0]", "{e}");
    assert!(
        load_str(&format!(
            "{MINIMAL}[sim.links]\ngcs = true\n[[fault]]\nkind = \"gcs-loss\"\nstart_s = 1.0\n"
        ))
        .is_ok()
    );
    // A window that ends before it starts.
    let e = refused("[[fault]]\nkind = \"gps-loss\"\nstart_s = 5.0\nend_s = 4.0\n");
    assert_eq!(e.key, "fault[0]", "{e}");
    // An unknown kind, and a field of another kind.
    let e = refused("[[fault]]\nkind = \"gremlins\"\nstart_s = 1.0\n");
    assert_eq!(e.key, "fault[0].kind", "{e}");
    let e = refused("[[fault]]\nkind = \"gps-loss\"\nstart_s = 1.0\nrotor = 0\n");
    assert_eq!(e.key, "fault[0].rotor", "{e}");
}

#[test]
fn other_mistakes_are_refused_naming_the_key() {
    assert_eq!(
        load_str("name = \"a b\"\nduration_s = 1.0\n")
            .expect_err("name")
            .key,
        "name"
    );
    assert_eq!(
        load_str("name = \"x\"\nduration_s = 0.0\n")
            .expect_err("duration")
            .key,
        "duration_s"
    );
    assert_eq!(refused("airframe = \"my-quad\"\n").key, "airframe");
    assert_eq!(
        refused("[battery]\ninitial_state_of_charge = 1.5\n").key,
        "battery.initial_state_of_charge"
    );
    assert_eq!(refused("[run]\nseeds = 0\n").key, "run.seeds");
    assert_eq!(refused("[wind]\nkind = \"hurricane\"\n").key, "wind.kind");
    assert_eq!(
        refused("[wind]\nkind = \"random\"\nsteady_speed_mps = [3.0, 1.0]\nsigma_h_mps = [0.1, 0.2]\ntau_s = 2.0\n").key,
        "wind.steady_speed_mps"
    );
    // A silent setpoint with a position says two things at once.
    assert_eq!(
        refused("[[pilot.setpoint]]\nt = 0.0\nsilent = true\nposition = [0.0, 0.0, 1.0]\n").key,
        "pilot.setpoint[0].silent"
    );
    assert_eq!(
        refused("[[pilot.sticks]]\nt = 0.0\nthrottle = 1.5\n").key,
        "pilot.sticks[0].throttle"
    );
    assert_eq!(refused("[pilot]\narm_at_s = -1.0\n").key, "pilot.arm_at_s");
}

#[test]
fn an_error_from_a_file_names_the_file() {
    let dir = std::env::temp_dir().join(format!("oxide-scenario-file-{}", std::process::id()));
    std::fs::create_dir_all(&dir).expect("temp dir");
    let path = dir.join("bad.toml");
    std::fs::write(&path, format!("{MINIMAL}colour = 1\n")).expect("write");
    let e = load_path(&path).expect_err("refused");
    let shown = path.display().to_string();
    assert_eq!(e.file.as_deref(), Some(shown.as_str()));
    assert!(e.to_string().starts_with(&shown), "{e}");
    assert!(e.to_string().contains("`colour`"), "{e}");
    let e = load_path(dir.join("missing.toml")).expect_err("no such file");
    assert!(e.file.is_some() && e.reason.contains("cannot read"), "{e}");
    std::fs::remove_dir_all(&dir).ok();
}

#[test]
fn loading_the_same_file_twice_gives_equal_values() {
    for entry in std::fs::read_dir(scenario_dir()).expect("scenarios/") {
        let path = entry.expect("entry").path();
        let a = load_path(&path).unwrap_or_else(|e| panic!("{e}"));
        let b = load_path(&path).expect("loads again");
        assert_eq!(a, b, "{}", path.display());
    }
    let text = documented_example();
    assert_eq!(
        load_str(&text).expect("loads"),
        load_str(&text).expect("loads")
    );
}

#[test]
fn type_and_value_errors_from_the_toml_layer_name_the_key() {
    let wrong = |text: &str, key: &str| {
        let e = load_str(text).expect_err("must be refused");
        assert_eq!(e.key, key, "{e}");
    };
    wrong("name = \"x\"\nduration_s = \"ten\"\n", "duration_s");
    wrong(
        &format!("{MINIMAL}[[fault]]\nkind = \"gps-loss\"\nstart_s = \"a\"\n"),
        "fault[0].start_s",
    );
    wrong(
        &format!(
            "{MINIMAL}[[fault]]\nkind = \"gps-loss\"\nstart_s = 1.0\n[[fault]]\nkind = \"gps-loss\"\nstart_s = \"a\"\n"
        ),
        "fault[1].start_s",
    );
    wrong(
        &format!("{MINIMAL}[[pilot.sticks]]\nt = 0.0\nroll = \"a\"\n"),
        "pilot.sticks[0].roll",
    );
    wrong(
        &format!("{MINIMAL}[params]\nFS_GPS_LAND = \"a\"\n"),
        "params.FS_GPS_LAND",
    );
    wrong(
        &format!("{MINIMAL}[[pilot.setpoint]]\nt = 0.0\nposition = [1.0, 2.0]\n"),
        "pilot.setpoint[0].position",
    );
    // Out-of-range integers and unknown variants.
    wrong(&format!("{MINIMAL}[run]\nseeds = -1\n"), "run.seeds");
    wrong(&format!("{MINIMAL}[sim]\nsensors = 3\n"), "sim.sensors");
    wrong(
        &format!("{MINIMAL}[sim.links]\nrc_failsafe_delay_us = -5\n"),
        "sim.links.rc_failsafe_delay_us",
    );
    wrong(
        &format!("{MINIMAL}[sim]\nsensors = \"gale\"\n"),
        "sim.sensors",
    );
}

// ---- Judging a flight -----------------------------------------------------------

#[test]
fn the_hold_of_gust_hold_is_judged_as_fly_gust_hold_judges_it() {
    use crate::metrics::{HoldCriteria, fly_gust_hold};
    let f = fault_scenarios::by_name("gust-hold").expect("gust-hold");
    for (profile, index) in [(F405_LEAN, 0), (F405_LEAN, 5), (H743_FULL, 2)] {
        let v = f.fly_checked(profile, index, &[]).expect("flies");
        let want = fly_gust_hold(profile, index, &HoldCriteria::default()).expect("flies");
        assert_eq!(v.hold, Some(want), "{} index {index}", profile.name);
        assert_eq!(
            v.violations,
            want.judge(&HoldCriteria::default()).violations
        );
    }
}

#[test]
fn a_fault_scenario_is_judged_by_its_expect() {
    let f = fault_scenarios::by_name("brain-crash").expect("scenario");
    let v = f.fly_checked(F405_LEAN, 0, &[]).expect("flies");
    assert_eq!(v.hold, None);
    assert!(v.violations.is_empty(), "{:?}", v.violations);
    // A requirement the flight cannot meet is reported.
    let mut strict = f.clone();
    strict.expect.as_mut().expect("expect").max_height_m = Some(1.0);
    let v = strict.fly_checked(F405_LEAN, 0, &[]).expect("flies");
    assert!(!v.violations.is_empty());
    // A file with nothing to check has nothing to break.
    let v = fault_scenarios::by_name("hover-step")
        .expect("hover-step")
        .fly_checked(F405_LEAN, 0, &[])
        .expect("flies");
    assert!(v.violations.is_empty() && v.hold.is_none());
}

#[test]
fn a_hold_without_a_point_to_hold_is_refused() {
    let e = refused("[expect.hold]\nwindow_start_s = 1.0\n");
    assert_eq!(e.key, "expect.hold", "{e}");
    assert!(e.reason.contains("setpoint"), "{e}");
}

// ---- Wave 4 (S) ---------------------------------------------------------------

#[test]
fn a_file_with_no_expect_is_still_judged_by_the_invariants() {
    let f = load_str(MINIMAL).expect("loads");
    assert!(f.expect.is_none() && f.hold.is_none());
    let mut clean = Checker::new(&f);
    assert!(clean.outcome.invariants().violations.is_empty());
    clean.outcome.core_mut().max_action = Action::Hold;
    assert!(
        clean.finish().violations.is_empty(),
        "a quiet flight passes"
    );

    let mut c = Checker::new(&f);
    c.outcome.non_finite = true;
    c.outcome.core_mut().timing_faults = 2;
    c.outcome.motor_ticks_while_disarmed = 3;
    c.outcome.core_mut().action_decreased = true;
    c.outcome.physics_faults = 1;
    let v = c.finish();
    assert_eq!(v.violations.len(), 5, "{:?}", v.violations);
    assert!(v.violations.iter().any(|m| m.contains("timing faults")));
    assert!(v.violations.iter().any(|m| m.contains("while disarmed")));
}

#[test]
fn states_expectations_tells_a_judged_file_from_an_invariants_only_one() {
    assert!(!load_str(MINIMAL).expect("loads").states_expectations());
    let with =
        format!("{MINIMAL}[expect]\nmin_action = \"hold\"\n[expect.ending]\nkind = \"down\"\n");
    assert!(load_str(&with).expect("loads").states_expectations());
}

const RANDOM_WIND: &str = "[wind]\nkind = \"random\"\nsteady_speed_mps = [0.0, 1.0]\nsigma_h_mps = [0.1, 0.2]\ntau_s = 2.0\n";

#[test]
fn turbulence_without_a_time_constant_is_refused() {
    assert_eq!(
        refused("[wind]\nkind = \"fixed\"\nturbulence_sigma = [0.5, 0.5, 0.2]\n").key,
        "wind.turbulence_tau_s"
    );
    assert_eq!(
        refused(&RANDOM_WIND.replace("tau_s = 2.0", "tau_s = 0.0")).key,
        "wind.tau_s"
    );
    assert_eq!(
        refused("[wind]\nkind = \"fixed\"\nturbulence_sigma = [0.5, 0.5, 0.2]\nturbulence_tau_s = -1.0\n").key,
        "wind.turbulence_tau_s"
    );
    // Without turbulence a time constant of zero is fine; with it, a positive one.
    load_str(&format!(
        "{MINIMAL}[wind]\nkind = \"fixed\"\nsteady_ned = [1.0, 0.0, 0.0]\n"
    ))
    .expect("no turbulence, no tau");
    load_str(&format!(
        "{MINIMAL}[wind]\nkind = \"fixed\"\nturbulence_sigma = [0.5, 0.5, 0.2]\nturbulence_tau_s = 1.5\n"
    ))
    .expect("turbulence with tau");
    load_str(&format!(
        "{MINIMAL}[wind]\nkind = \"random\"\nsteady_speed_mps = [0.0, 1.0]\nsigma_h_mps = [0.0, 0.0]\ntau_s = 0.0\n"
    ))
    .expect("no turbulence drawn, no tau needed");
}

#[test]
fn negative_sigmas_ratios_and_durations_are_refused() {
    assert_eq!(
        refused("[wind]\nkind = \"fixed\"\nturbulence_sigma = [0.5, -0.5, 0.2]\nturbulence_tau_s = 1.0\n").key,
        "wind.turbulence_sigma"
    );
    assert_eq!(
        refused(&RANDOM_WIND.replace("[0.1, 0.2]", "[-0.1, 0.2]")).key,
        "wind.sigma_h_mps"
    );
    assert_eq!(
        refused(&format!("{RANDOM_WIND}sigma_v_ratio = -5.0\n")).key,
        "wind.sigma_v_ratio"
    );
    assert_eq!(
        refused("[wind]\nkind = \"fixed\"\n[[wind.gust]]\nstart_s = 1.0\nduration_s = -2.0\npeak_ned = [1.0, 0.0, 0.0]\n").key,
        "wind.gust[0].duration_s"
    );
    let gusts = |dur: &str| {
        format!(
            "{RANDOM_WIND}[wind.gusts]\ncount = 1\nstart_s = [1.0, 2.0]\nduration_s = {dur}\npeak_h_mps = [1.0, 2.0]\npeak_v_mps = [0.0, 0.0]\n"
        )
    };
    assert_eq!(refused(&gusts("[-1.0, 2.0]")).key, "wind.gusts.duration_s");
    load_str(&format!("{MINIMAL}{}", gusts("[0.0, 2.0]"))).expect("zero is a duration");
}

#[test]
fn at_most_64_gusts() {
    let fixed = |n: usize| {
        let mut s = String::from("[wind]\nkind = \"fixed\"\n");
        for i in 0..n {
            s.push_str(&format!(
                "[[wind.gust]]\nstart_s = {i}.0\nduration_s = 1.0\npeak_ned = [1.0, 0.0, 0.0]\n"
            ));
        }
        s
    };
    assert_eq!(refused(&fixed(65)).key, "wind.gust");
    load_str(&format!("{MINIMAL}{}", fixed(64))).expect("64 fit");
    let random = |n: u32| {
        format!(
            "{RANDOM_WIND}[wind.gusts]\ncount = {n}\nstart_s = [1.0, 2.0]\nduration_s = [1.0, 2.0]\npeak_h_mps = [1.0, 2.0]\npeak_v_mps = [0.0, 0.0]\n"
        )
    };
    assert_eq!(refused(&random(3_000_000)).key, "wind.gusts.count");
    assert_eq!(refused(&random(65)).key, "wind.gusts.count");
    load_str(&format!("{MINIMAL}{}", random(64))).expect("64 fit");
}

#[test]
fn seeds_and_duration_have_limits() {
    assert_eq!(refused("[run]\nseeds = 100001\n").key, "run.seeds");
    load_str(&format!("{MINIMAL}[run]\nseeds = 100000\n")).expect("the limit itself");
    let dur = |d: &str| load_str(&format!("name = \"d\"\nduration_s = {d}\n"));
    // One tick of the slower profile (the F405's 1 ms) up to a day.
    assert_eq!(
        dur("1e-7").expect_err("rounds to no tick").key,
        "duration_s"
    );
    assert_eq!(dur("0.0005").expect_err("under a tick").key, "duration_s");
    assert_eq!(dur("86401.0").expect_err("over a day").key, "duration_s");
    dur("0.001").expect("one tick");
    dur("86400.0").expect("a day");
}

#[test]
fn an_expect_must_say_how_the_flight_ends() {
    let e = refused("[expect]\nreasons_never = [\"GPS_LOST\"]\n");
    assert_eq!(e.key, "expect.ending", "{e}");
    for kind in [
        "flying",
        "flying-at-height",
        "landed",
        "terminated",
        "disarmed",
        "down",
    ] {
        assert!(e.reason.contains(kind), "{e}");
    }
    // With an ending it loads; `[expect.hold]` alone needs none.
    load_str(&format!(
        "{MINIMAL}[expect]\nreasons_never = [\"GPS_LOST\"]\n[expect.ending]\nkind = \"terminated\"\n"
    ))
    .expect("an ending is stated");
    load_str(&format!(
        "{MINIMAL}[[pilot.setpoint]]\nt = 0.0\nposition = [0.0, 0.0, 1.0]\n[expect.hold]\nmax_h_error_m = 1.0\n"
    ))
    .expect("a hold alone needs no ending");
}

#[test]
fn a_single_value_that_breaks_a_rule_is_blamed_by_name() {
    // Ten entries; BAT_CRIT_V = 3.9 alone breaks BAT_CRIT_V <= BAT_LOW_V
    // (the default low level is 3.5). The others are fine alone.
    let e = refused(
        "[params]\nPOS_X_P = 1.1\nPOS_Y_P = 1.1\nPOS_Z_P = 1.2\nRTL_ALT = 12.0\nLAND_SPEED = 0.8\nBAT_CRIT_V = 3.9\nFS_GPS_LAND = 12.0\nLIM_TILT = 0.5\nLIM_SPEED_UP = 3.0\nLIM_SPEED_DOWN = 1.5\n",
    );
    assert_eq!(e.key, "params.BAT_CRIT_V", "{e}");
    assert!(
        e.reason.contains("value 3.9") && e.reason.contains("break a rule"),
        "{e}"
    );
    assert!(!e.reason.contains("together"), "{e}");
}

#[test]
fn a_value_that_another_entry_makes_valid_is_not_blamed_for_a_different_pair() {
    // BAT_CRIT_V = 3.9 breaks a rule alone (the default low level is 3.5),
    // but BAT_LOW_V = 4.0 makes it valid. What breaks the list is the other
    // pair: the touchdown thrust must stay below the blind-landing thrust.
    let e = refused(
        "[params]\nBAT_CRIT_V = 3.9\nBAT_LOW_V = 4.0\nLAND_BLIND_THR = 0.35\nLAND_TD_THRUST = 0.4\n",
    );
    assert_eq!(e.key, "params", "{e}");
    assert!(e.reason.contains("together"), "{e}");
}

// ---- The sensor faults and the GPS delay ------------------------------------------

#[test]
fn the_three_sensor_faults_and_the_gps_delay_load_from_a_file() {
    let s = load_str(&format!(
        "{MINIMAL}[sim]\nsensors = \"realistic\"\ngps_latency_us = 250000\n\
         [[fault]]\nkind = \"mag-rotate\"\nstart_s = 10.0\nend_s = 25.0\nangle_rad = -1.5\nramp_s = 2.0\n\
         [[fault]]\nkind = \"mag-rotate\"\nstart_s = 30.0\nangle_rad = 0.5\n\
         [[fault]]\nkind = \"gps-vel-offset\"\nstart_s = 10.0\nend_s = 11.0\nnorth_mps = 5.0\neast_mps = 0.0\n\
         [[fault]]\nkind = \"gps-vel-noise\"\nstart_s = 10.0\nend_s = 15.0\nsigma_mps = 2.0\n\
         [[fault]]\nkind = \"gps-vel-noise\"\nstart_s = 20.0\nend_s = 25.0\nsigma_mps = 1.0\ns_acc_mps = 1.0\n"
    ))
    .expect("loads");
    let kinds: Vec<FaultKind> = s.faults.iter().map(|f| f.kind).collect();
    assert_eq!(
        kinds,
        vec![
            FaultKind::MagRotate {
                angle_rad: -1.5,
                ramp_s: 2.0
            },
            FaultKind::MagRotate {
                angle_rad: 0.5,
                ramp_s: 0.0
            },
            FaultKind::GpsVelOffset {
                north_mps: 5.0,
                east_mps: 0.0
            },
            FaultKind::GpsVelNoise {
                sigma_mps: 2.0,
                s_acc_mps: None
            },
            FaultKind::GpsVelNoise {
                sigma_mps: 1.0,
                s_acc_mps: Some(1.0)
            },
        ]
    );
    assert_eq!(s.faults[1].end_s, None);
    assert_eq!(s.gps_latency_us, Some(250_000));
    // The delay reaches the sensors of the flight; the preset's stays
    // without it.
    let airframe = Airframe::reference_quad().expect("airframe");
    assert_eq!(s.options(&airframe, 0).sensors.gps.latency_us, 250_000);
    let plain = load_str(&format!("{MINIMAL}[sim]\nsensors = \"realistic\"\n")).expect("loads");
    assert_eq!(plain.gps_latency_us, None);
    assert_eq!(
        plain.options(&airframe, 0).sensors.gps.latency_us,
        SensorConfig::realistic().gps.latency_us
    );
    // And on the ideal preset, whose delay is 0, it still applies.
    let ideal = load_str(&format!("{MINIMAL}[sim]\ngps_latency_us = 50000\n")).expect("loads");
    assert_eq!(ideal.options(&airframe, 0).sensors.gps.latency_us, 50_000);
    assert_eq!(
        load_str(&format!("{MINIMAL}[sim]\ngps_latency_us = 0\n"))
            .expect("loads")
            .gps_latency_us,
        Some(0)
    );
}

#[test]
fn the_new_faults_are_refused_naming_the_field() {
    let fault = |body: &str| format!("[[fault]]\nstart_s = 1.0\nend_s = 9.0\n{body}");
    let key = |body: &str| refused(&fault(body)).key;
    // Missing.
    assert_eq!(key("kind = \"mag-rotate\"\n"), "fault[0].angle_rad");
    assert_eq!(
        key("kind = \"gps-vel-offset\"\neast_mps = 1.0\n"),
        "fault[0].north_mps"
    );
    assert_eq!(
        key("kind = \"gps-vel-offset\"\nnorth_mps = 1.0\n"),
        "fault[0].east_mps"
    );
    assert_eq!(key("kind = \"gps-vel-noise\"\n"), "fault[0].sigma_mps");
    // Negative where a size is meant, not finite anywhere.
    assert_eq!(
        key("kind = \"mag-rotate\"\nangle_rad = 1.0\nramp_s = -1.0\n"),
        "fault[0].ramp_s"
    );
    assert_eq!(
        key("kind = \"gps-vel-noise\"\nsigma_mps = -1.0\n"),
        "fault[0].sigma_mps"
    );
    assert_eq!(
        key("kind = \"gps-vel-noise\"\nsigma_mps = 1.0\ns_acc_mps = -0.5\n"),
        "fault[0].s_acc_mps"
    );
    for (kind, name) in [
        ("mag-rotate", "angle_rad"),
        ("mag-rotate", "ramp_s"),
        ("gps-vel-offset", "north_mps"),
        ("gps-vel-offset", "east_mps"),
        ("gps-vel-noise", "sigma_mps"),
        ("gps-vel-noise", "s_acc_mps"),
    ] {
        // Every field of the three, as NaN and as infinity.
        for bad in ["nan", "inf", "-inf"] {
            let others = match kind {
                "mag-rotate" => "angle_rad = 1.0\n",
                "gps-vel-offset" => "north_mps = 1.0\neast_mps = 1.0\n",
                _ => "sigma_mps = 1.0\n",
            };
            let text = fault(&format!("kind = \"{kind}\"\n{others}{name} = {bad}\n"));
            let e = refused(&text);
            assert_eq!(
                e.key,
                format!("fault[0].{name}"),
                "{kind} {name} {bad}: {e}"
            );
        }
    }
    // A field of another kind, and a ramp that does not fit the window.
    assert_eq!(
        key("kind = \"gps-loss\"\nangle_rad = 1.0\n"),
        "fault[0].angle_rad"
    );
    assert_eq!(
        key("kind = \"gps-vel-offset\"\nnorth_mps = 1.0\neast_mps = 1.0\nsigma_mps = 1.0\n"),
        "fault[0].sigma_mps"
    );
    assert_eq!(
        key("kind = \"mag-rotate\"\nangle_rad = 1.0\nramp_s = 5.0\n"),
        "fault[0]"
    );
    let e = refused(&fault(
        "kind = \"mag-rotate\"\nangle_rad = 1.0\nramp_s = 5.0\n",
    ));
    assert!(e.reason.contains("ramp_s"), "{e}");
    // A mistyped field is blamed by the TOML layer by name.
    assert_eq!(
        key("kind = \"mag-rotate\"\nangle_rad = \"big\"\n"),
        "fault[0].angle_rad"
    );
    assert_eq!(key("kind = \"mag-rotate\"\nangle = 1.0\n"), "angle");
    // The kind list names the new ones.
    let e = refused(&fault("kind = \"gremlins\"\n"));
    assert!(
        e.reason.contains("mag-rotate") && e.reason.contains("gps-vel-noise"),
        "{e}"
    );
}

#[test]
fn the_gps_delay_is_validated() {
    let key = |v: &str| refused(&format!("[sim]\ngps_latency_us = {v}\n")).key;
    assert_eq!(key("-1"), "sim.gps_latency_us");
    assert_eq!(key("0.5"), "sim.gps_latency_us");
    assert_eq!(key("\"fast\""), "sim.gps_latency_us");
    // Not a whole number of milliseconds (the slower profile's step).
    assert_eq!(key("150500"), "sim.gps_latency_us");
    // Beyond five seconds.
    assert_eq!(key("5001000"), "sim.gps_latency_us");
    assert!(load_str(&format!("{MINIMAL}[sim]\ngps_latency_us = 5000000\n")).is_ok());
}
