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
