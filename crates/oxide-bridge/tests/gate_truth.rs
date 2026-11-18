// SPDX-License-Identifier: AGPL-3.0-or-later
//! `core-f405-truth`: our F405 core flown on the true state (ADR-0007 section
//! 5: reported, decides nothing). No binary needed.

use std::path::PathBuf;

use oxide_bridge::record::Row;
use oxide_bridge::runner::{Autopilot, fly_core};
use oxide_bridge::scenario::LoadedScenario;
use oxide_sim::scenario_file::RunProfile;

fn gust_hold_text() -> String {
    let p = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../scenarios/gust-hold.toml");
    std::fs::read_to_string(p).expect("the scenario file")
}

fn gust_hold() -> LoadedScenario {
    LoadedScenario::from_text(&gust_hold_text()).expect("loads")
}

fn run(src: &LoadedScenario, autopilot: Autopilot) -> Row {
    let f = match autopilot {
        Autopilot::Core(p) => fly_core(src, 0, p),
        Autopilot::CoreTruth => oxide_bridge::runner::fly_core_truth(src, 0),
        Autopilot::ArduCopter => panic!("no binary here"),
    }
    .expect("flies");
    Row::of_flight(&f)
}

#[test]
fn the_name_is_known_to_the_command_line() {
    assert_eq!(
        Autopilot::parse("core-f405-truth"),
        Some(Autopilot::CoreTruth)
    );
    assert_eq!(Autopilot::CoreTruth.name(), "core-f405-truth");
}

#[test]
fn truth_differs_from_the_ekf_flight_in_the_error_and_has_the_same_seed() {
    let src = gust_hold();
    let ekf = run(&src, Autopilot::Core(RunProfile::F405));
    let truth = run(&src, Autopilot::CoreTruth);
    assert_eq!(truth.get("autopilot"), "core-f405-truth");
    assert_eq!(truth.get("seed"), ekf.get("seed"));
    assert_eq!(truth.get("scenario"), "gust-hold");
    assert_eq!(truth.get("index"), "0");
    assert_ne!(truth.get("rms_h_m"), ekf.get("rms_h_m"));
    assert_ne!(truth.get("trace_hash"), ekf.get("trace_hash"));
    assert!(!truth.get("rms_h_m").is_empty());
}

#[test]
fn truth_is_the_file_loaded_with_the_external_estimator_by_hand() {
    let text = gust_hold_text();
    assert!(text.contains("estimator = \"ekf\""));
    let by_hand =
        LoadedScenario::from_text(&text.replace("estimator = \"ekf\"", "estimator = \"external\""))
            .expect("loads");
    let hand = run(&by_hand, Autopilot::Core(RunProfile::F405));
    let truth = run(&gust_hold(), Autopilot::CoreTruth);
    assert_eq!(hand.get("trace_hash"), truth.get("trace_hash"));
    assert_eq!(hand.get("rms_h_m"), truth.get("rms_h_m"));
}
