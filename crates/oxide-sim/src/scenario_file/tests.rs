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

// ---- The exporter ------------------------------------------------------------

use super::export;
use crate::fault_scenarios;

/// The directory of the scenario files in the repository.
fn scenario_dir() -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scenarios")
}

#[test]
fn exported_files_load_back_to_the_same_scenario() {
    let files = export::all_files();
    assert_eq!(files.len(), 43);
    for f in files {
        let text = export::to_toml(&f);
        let back = load_str(&text).unwrap_or_else(|e| panic!("{}: {e}\n{text}", f.name));
        assert_eq!(back, f, "{}", f.name);
        assert_eq!(export::to_toml(&back), text, "{}", f.name);
    }
}

#[test]
fn a_file_carries_what_the_rust_definition_carries() {
    for s in fault_scenarios::all()
        .iter()
        .chain(fault_scenarios::pairs().iter())
    {
        let f = load_str(&export::to_toml(&export::from_fault_scenario(s))).expect("loads");
        assert_eq!(f.name, s.name);
        assert_eq!(f.about, s.about);
        assert_eq!(f.pilot, s.script, "{}", s.name);
        assert_eq!(f.faults, s.faults, "{}", s.name);
        assert_eq!(f.links, s.links, "{}", s.name);
        assert_eq!(f.expect.as_ref(), Some(&s.expect), "{}", s.name);
        let battery = Airframe::reference_quad().expect("airframe").battery;
        assert_eq!(
            f.battery.map(|b| b.apply(&battery)),
            s.battery.clone(),
            "{}",
            s.name
        );
    }
}

#[test]
fn failsafe_overrides_become_named_parameters() {
    let s = fault_scenarios::by_name("pair-rc-loss+geofence").expect("pair");
    let f = export::from_fault_scenario(&s);
    assert_eq!(
        f.params,
        vec![
            ("FS_FENCE_RAD".to_string(), 10.0),
            ("FS_GPS_LAND".to_string(), 10.0),
            ("FS_NOLINK_RTL".to_string(), 5.0),
        ]
    );
    let none =
        export::from_fault_scenario(&fault_scenarios::by_name("gcs-loss").expect("scenario"));
    assert!(none.params.is_empty());
}

#[test]
fn computed_battery_sizes_are_written_as_literals() {
    let f = export::from_fault_scenario(
        &fault_scenarios::by_name("pair-battery-low+battery-critical").expect("pair"),
    );
    let text = export::to_toml(&f);
    // 500 mAh; 0.05 + 0.0048 * (15 - 3 - 3) in f64.
    assert!(text.contains("capacity_mah = 500.0"), "{text}");
    let soc = 0.05 + 0.0048 * (15.0 - 3.0 - 3.0);
    assert!(
        text.contains(&format!("initial_state_of_charge = {soc:?}")),
        "{text}"
    );
}

/// Rewrites `scenarios/` from the Rust definitions:
/// `cargo test -p oxide-sim --lib regenerate_scenario_files -- --ignored`.
#[test]
#[ignore = "writes the files in scenarios/"]
fn regenerate_scenario_files() {
    let dir = scenario_dir();
    std::fs::create_dir_all(&dir).expect("create scenarios/");
    for f in export::all_files() {
        std::fs::write(dir.join(format!("{}.toml", f.name)), export::to_toml(&f)).expect("write");
    }
}

#[test]
fn parameters_are_applied_in_name_order_whatever_the_file_order() {
    // The TOML reader hands a table over sorted by key, not in file order, so
    // the order that counts is the name order; the exporter writes it.
    let s = load_str(
        "name = \"x\"\nduration_s = 5.0\n[params]\nFS_NOLINK_RTL = 5.0\nFS_GPS_LAND = 10.0\nEKF_HGT_TMO = 2\n",
    );
    let s = s.expect_err("EKF_HGT_TMO is not a parameter");
    assert_eq!(s.key, "params.EKF_HGT_TMO");
    let s = load_str(
        "name = \"x\"\nduration_s = 5.0\n[params]\nFS_NOLINK_RTL = 5.0\nFS_GPS_LAND = 10.0\n",
    )
    .expect("loads");
    let names: Vec<&str> = s.params.iter().map(|(n, _)| n.as_str()).collect();
    assert_eq!(names, ["FS_GPS_LAND", "FS_NOLINK_RTL"]);
}

#[test]
fn the_files_in_scenarios_are_what_the_exporter_writes() {
    let dir = scenario_dir();
    let mut on_disk: Vec<String> = std::fs::read_dir(&dir)
        .expect("scenarios/ exists")
        .map(|e| e.expect("entry").file_name().to_string_lossy().into_owned())
        .collect();
    on_disk.sort();
    let mut expected: Vec<String> = Vec::new();
    for f in export::all_files() {
        let name = format!("{}.toml", f.name);
        let text = std::fs::read_to_string(dir.join(&name))
            .unwrap_or_else(|e| panic!("{name}: {e} (run the regenerate_scenario_files test)"));
        assert_eq!(text, export::to_toml(&f), "{name} is out of date");
        expected.push(name);
    }
    expected.sort();
    assert_eq!(
        on_disk, expected,
        "files in scenarios/ and scenarios defined in code differ"
    );
}

// ---- hover-step and gust-hold ---------------------------------------------------

use crate::scenario::{GustHold, HoverStep, gust_hold};
use oxide_core::schedule::{F405_LEAN, H743_FULL};

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
    let f = export::from_hover_step(&HoverStep::default());
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
    let f = export::from_gust_hold(&GustHold::default());
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
    assert_eq!(e.key, "crashed", "{e}");
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
    assert_eq!(e.key, "start_s", "{e}");
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
    assert_eq!(refused("[wind]\nkind = \"hurricane\"\n").key, "hurricane");
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
