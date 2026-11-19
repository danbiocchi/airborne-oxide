// SPDX-License-Identifier: AGPL-3.0-or-later
//! Our own core flown through the runner of the bridge crate: the rows it
//! writes carry the simulator's own pinned trace hashes, so the table format
//! of the benchmark is tied to the simulator's flights. No binary needed.

use std::path::PathBuf;

use oxide_bridge::record::Row;
use oxide_bridge::runner::{Autopilot, fly_core};
use oxide_bridge::scenario::LoadedScenario;
use oxide_sim::scenario_file::RunProfile;

fn scenario(name: &str) -> LoadedScenario {
    let p = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../scenarios")
        .join(format!("{name}.toml"));
    LoadedScenario::load(p).expect("the scenario file")
}

/// The trace hashes `oxide-sim hash` pins for gust-hold run 0.
const GUST_HOLD_0_F405: &str = "9a63745df2eb227a";
const GUST_HOLD_0_H743: &str = "d2ecef5541ba9e88";

#[test]
fn the_core_f405_row_of_gust_hold_0_carries_the_pinned_trace_hash() {
    let f = fly_core(&scenario("gust-hold"), 0, RunProfile::F405).expect("flies");
    let row = Row::of_flight(&f);
    assert_eq!(row.get("trace_hash"), GUST_HOLD_0_F405);
    assert_eq!(row.get("autopilot"), "core-f405");
    assert_eq!(row.get("scenario"), "gust-hold");
    assert_eq!(row.get("index"), "0");
    assert_eq!(row.get("started"), "yes");
    assert_eq!(row.get("packet_hash"), "");
    assert_eq!(row.get("class"), "safe");
    assert_eq!(row.get("verdict"), "pass", "{}", row.line());
    assert_eq!(row.get("not_judged"), "0");
    println!("MEASURED core-f405 gust-hold 0: {}", row.line());
}

#[test]
fn the_core_h743_row_of_gust_hold_0_carries_the_pinned_trace_hash() {
    let f = fly_core(&scenario("gust-hold"), 0, RunProfile::H743).expect("flies");
    let row = Row::of_flight(&f);
    assert_eq!(row.get("trace_hash"), GUST_HOLD_0_H743);
    assert_eq!(row.get("autopilot"), "core-h743");
}

#[test]
fn the_two_core_routes_fly_different_profiles() {
    // The mutation "core-f405 flies another profile" cannot hide: the two
    // hashes differ and each is its own.
    assert_ne!(GUST_HOLD_0_F405, GUST_HOLD_0_H743);
    assert_eq!(
        Autopilot::parse("core-f405"),
        Some(Autopilot::Core(RunProfile::F405))
    );
    assert_eq!(
        Autopilot::parse("core-h743"),
        Some(Autopilot::Core(RunProfile::H743))
    );
    assert_eq!(Autopilot::parse("px4"), None);
}

#[test]
fn a_core_flight_is_judged_with_its_status_so_nothing_is_left_unjudged() {
    let f = fly_core(&scenario("hover-step"), 0, RunProfile::F405).expect("flies");
    assert!(
        f.verdict.not_judged.is_empty(),
        "{:?}",
        f.verdict.not_judged
    );
    assert!(f.last_motor_on_s.is_some());
    assert!(f.rms_v_m.is_none(), "hover-step has no hold criteria");
}

#[test]
fn the_core_row_of_gust_hold_0_pins_the_vertical_error_and_the_last_motor_command() {
    // Values of the preview table of the same flight (rms_v_m over the hold
    // window from 15 s; the last tick of a flight that never stops its
    // motors). A window that starts later, or a last-motor test that counts
    // zero, moves them.
    let f = fly_core(&scenario("gust-hold"), 0, RunProfile::F405).expect("flies");
    let row = Row::of_flight(&f);
    assert_eq!(row.get("rms_v_m"), "0.070943102650111", "{}", row.line());
    assert_eq!(row.get("max_v_m"), "0.16323683169359304");
    assert_eq!(row.get("last_motor_on_s"), "39.998999999999995");
}
