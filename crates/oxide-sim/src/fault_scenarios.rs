// SPDX-License-Identifier: AGPL-3.0-or-later
//! The scenario suite, embedded (ADR-0004 §7, ADR-0003, DESIGN §4.10, §5.5).
//!
//! The scenarios are the TOML files in `scenarios/` at the repository root;
//! this module compiles them in, so the simulator, the tests, the viewer and
//! the command line find them by name without a path. [`EMBEDDED`] is a hand
//! kept list of those files and a test fails when it and the directory
//! differ. A scenario that is not embedded is flown from its path with
//! [`scenario_file::load_path`](crate::scenario_file::load_path).
//!
//! Each fault scenario injects one failure in flight (or a pair of them) and
//! states in `[expect]` what the vehicle must do: they are the acceptance
//! test of the failsafes. `hover-step` and `gust-hold` are in the list too;
//! they carry no `[expect]` (the first) or `[expect.hold]` (the second).

use std::sync::OnceLock;

use crate::scenario_file::{self, ScenarioFile};

/// Every scenario file in `scenarios/` as `(name, text)`, sorted by name.
pub const EMBEDDED: [(&str, &str); 43] = [
    (
        "baro-loss",
        include_str!("../../../scenarios/baro-loss.toml"),
    ),
    (
        "battery-critical",
        include_str!("../../../scenarios/battery-critical.toml"),
    ),
    (
        "battery-low",
        include_str!("../../../scenarios/battery-low.toml"),
    ),
    (
        "brain-crash",
        include_str!("../../../scenarios/brain-crash.toml"),
    ),
    (
        "brain-hang",
        include_str!("../../../scenarios/brain-hang.toml"),
    ),
    ("gcs-loss", include_str!("../../../scenarios/gcs-loss.toml")),
    ("geofence", include_str!("../../../scenarios/geofence.toml")),
    ("gps-loss", include_str!("../../../scenarios/gps-loss.toml")),
    (
        "gps-loss-pilot",
        include_str!("../../../scenarios/gps-loss-pilot.toml"),
    ),
    (
        "gps-loss-recovers",
        include_str!("../../../scenarios/gps-loss-recovers.toml"),
    ),
    (
        "gust-hold",
        include_str!("../../../scenarios/gust-hold.toml"),
    ),
    (
        "hover-step",
        include_str!("../../../scenarios/hover-step.toml"),
    ),
    ("imu-loss", include_str!("../../../scenarios/imu-loss.toml")),
    (
        "kill-in-rtl",
        include_str!("../../../scenarios/kill-in-rtl.toml"),
    ),
    ("mag-loss", include_str!("../../../scenarios/mag-loss.toml")),
    (
        "motor-loss",
        include_str!("../../../scenarios/motor-loss.toml"),
    ),
    (
        "no-human-link",
        include_str!("../../../scenarios/no-human-link.toml"),
    ),
    (
        "pair-battery-critical+baro-loss",
        include_str!("../../../scenarios/pair-battery-critical+baro-loss.toml"),
    ),
    (
        "pair-battery-critical+brain-crash",
        include_str!("../../../scenarios/pair-battery-critical+brain-crash.toml"),
    ),
    (
        "pair-battery-critical+geofence",
        include_str!("../../../scenarios/pair-battery-critical+geofence.toml"),
    ),
    (
        "pair-battery-low+baro-loss",
        include_str!("../../../scenarios/pair-battery-low+baro-loss.toml"),
    ),
    (
        "pair-battery-low+battery-critical",
        include_str!("../../../scenarios/pair-battery-low+battery-critical.toml"),
    ),
    (
        "pair-battery-low+brain-crash",
        include_str!("../../../scenarios/pair-battery-low+brain-crash.toml"),
    ),
    (
        "pair-battery-low+geofence",
        include_str!("../../../scenarios/pair-battery-low+geofence.toml"),
    ),
    (
        "pair-brain-crash+baro-loss",
        include_str!("../../../scenarios/pair-brain-crash+baro-loss.toml"),
    ),
    (
        "pair-brain-crash+geofence",
        include_str!("../../../scenarios/pair-brain-crash+geofence.toml"),
    ),
    (
        "pair-geofence+baro-loss",
        include_str!("../../../scenarios/pair-geofence+baro-loss.toml"),
    ),
    (
        "pair-gps-loss+baro-loss",
        include_str!("../../../scenarios/pair-gps-loss+baro-loss.toml"),
    ),
    (
        "pair-gps-loss+battery-critical",
        include_str!("../../../scenarios/pair-gps-loss+battery-critical.toml"),
    ),
    (
        "pair-gps-loss+battery-low",
        include_str!("../../../scenarios/pair-gps-loss+battery-low.toml"),
    ),
    (
        "pair-gps-loss+brain-crash",
        include_str!("../../../scenarios/pair-gps-loss+brain-crash.toml"),
    ),
    (
        "pair-gps-loss+geofence",
        include_str!("../../../scenarios/pair-gps-loss+geofence.toml"),
    ),
    (
        "pair-rc-loss+baro-loss",
        include_str!("../../../scenarios/pair-rc-loss+baro-loss.toml"),
    ),
    (
        "pair-rc-loss+battery-critical",
        include_str!("../../../scenarios/pair-rc-loss+battery-critical.toml"),
    ),
    (
        "pair-rc-loss+battery-low",
        include_str!("../../../scenarios/pair-rc-loss+battery-low.toml"),
    ),
    (
        "pair-rc-loss+brain-crash",
        include_str!("../../../scenarios/pair-rc-loss+brain-crash.toml"),
    ),
    (
        "pair-rc-loss+geofence",
        include_str!("../../../scenarios/pair-rc-loss+geofence.toml"),
    ),
    (
        "pair-rc-loss+gps-loss",
        include_str!("../../../scenarios/pair-rc-loss+gps-loss.toml"),
    ),
    (
        "rc-loss-offboard-gcs",
        include_str!("../../../scenarios/rc-loss-offboard-gcs.toml"),
    ),
    (
        "rc-loss-on-ground",
        include_str!("../../../scenarios/rc-loss-on-ground.toml"),
    ),
    (
        "rc-loss-override",
        include_str!("../../../scenarios/rc-loss-override.toml"),
    ),
    (
        "rc-loss-position",
        include_str!("../../../scenarios/rc-loss-position.toml"),
    ),
    (
        "setpoint-gap",
        include_str!("../../../scenarios/setpoint-gap.toml"),
    ),
];

/// Every embedded scenario, loaded once.
///
/// # Panics
/// If an embedded file does not load: a build defect that the tests catch.
pub fn embedded() -> &'static [ScenarioFile] {
    static LOADED: OnceLock<Vec<ScenarioFile>> = OnceLock::new();
    LOADED.get_or_init(|| {
        EMBEDDED
            .iter()
            .map(|(name, text)| {
                scenario_file::load_str(text)
                    .unwrap_or_else(|e| panic!("embedded scenario {name}: {e}"))
            })
            .collect()
    })
}

/// Whether `s` is a pairing of two failures.
fn is_pair(s: &ScenarioFile) -> bool {
    s.name.starts_with("pair-")
}

/// The 20 named fault scenarios: embedded files that state `[expect]` and
/// are not pairings.
pub fn all() -> Vec<ScenarioFile> {
    embedded()
        .iter()
        .filter(|s| s.expect.is_some() && !is_pair(s))
        .cloned()
        .collect()
}

/// The 21 pairings of two failures.
pub fn pairs() -> Vec<ScenarioFile> {
    embedded().iter().filter(|s| is_pair(s)).cloned().collect()
}

/// The embedded scenario called `name` (fault scenarios, pairings,
/// `hover-step`, `gust-hold`).
pub fn by_name(name: &str) -> Option<ScenarioFile> {
    embedded().iter().find(|s| s.name == name).cloned()
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeSet;

    use oxide_core::io::{Action, Reasons};

    use super::*;

    /// The scenario's expectation; every fault scenario has one.
    fn expect_of(s: &ScenarioFile) -> &crate::outcome::Expect {
        s.expect.as_ref().expect("a fault scenario has [expect]")
    }

    #[test]
    fn the_embedded_list_and_the_scenarios_directory_are_the_same() {
        let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scenarios");
        let on_disk: BTreeSet<String> = std::fs::read_dir(&dir)
            .unwrap_or_else(|e| panic!("{}: {e}", dir.display()))
            .map(|e| {
                e.expect("directory entry")
                    .file_name()
                    .to_string_lossy()
                    .into_owned()
            })
            .collect();
        let listed: BTreeSet<String> = EMBEDDED.iter().map(|(n, _)| format!("{n}.toml")).collect();
        let missing: Vec<_> = on_disk.difference(&listed).collect();
        let stale: Vec<_> = listed.difference(&on_disk).collect();
        assert!(
            missing.is_empty() && stale.is_empty(),
            "add to EMBEDDED: {missing:?}; remove from EMBEDDED: {stale:?}"
        );
        // The text embedded is the text on disk, and each file is named
        // after the name it states.
        for (name, text) in EMBEDDED {
            let on = std::fs::read_to_string(dir.join(format!("{name}.toml"))).expect("read");
            assert_eq!(on, text, "{name}");
        }
        for (s, (name, _)) in embedded().iter().zip(EMBEDDED) {
            assert_eq!(s.name, name, "file name and `name` key differ");
        }
        let names: Vec<&str> = EMBEDDED.iter().map(|(n, _)| *n).collect();
        let mut sorted = names.clone();
        sorted.sort_unstable();
        assert_eq!(names, sorted, "EMBEDDED is not sorted by name");
    }

    #[test]
    fn names_are_unique_and_pairs_are_21() {
        let mut names: Vec<String> = all().iter().map(|s| s.name.clone()).collect();
        assert_eq!(names.len(), 20);
        let p = pairs();
        assert_eq!(p.len(), 21);
        names.extend(p.iter().map(|s| s.name.clone()));
        let mut sorted = names.clone();
        sorted.sort_unstable();
        sorted.dedup();
        assert_eq!(sorted.len(), names.len(), "duplicate scenario name");
        assert_eq!(embedded().len(), 20 + 21 + 2);
    }

    #[test]
    fn by_name_finds_scenarios() {
        assert!(by_name("gps-loss-recovers").is_some());
        assert!(by_name("pair-rc-loss+gps-loss").is_some());
        assert!(by_name("hover-step").is_some());
        assert!(by_name("gust-hold").is_some());
        assert!(by_name("no-such-scenario").is_none());
    }

    #[test]
    fn every_scenario_builds_and_has_a_sensible_script() {
        for s in all().into_iter().chain(pairs()) {
            assert!(
                s.build(oxide_core::schedule::F405_LEAN, 0).is_ok(),
                "{}",
                s.name
            );
            assert!(
                s.duration_s >= expect_of(&s).by_s,
                "{} ends before by_s",
                s.name
            );
            // Arming into Offboard needs a setpoint from the start.
            if s.pilot.mode.first().map(|m| m.1) == Some(2) {
                assert!(
                    matches!(s.pilot.setpoints.first(), Some((t, Some(_))) if *t == 0.0),
                    "{}",
                    s.name
                );
            }
            // The scenario is documented.
            assert!(!s.about.is_empty());
        }
    }

    #[test]
    fn pair_expectation_follows_the_failures() {
        let p = by_name("pair-gps-loss+baro-loss").expect("pair");
        assert_eq!(expect_of(&p).min_action, Action::LandNow);
        let p = by_name("pair-rc-loss+gps-loss").expect("pair");
        assert_eq!(expect_of(&p).min_action, Action::Land);
        let p = by_name("pair-rc-loss+geofence").expect("pair");
        assert_eq!(expect_of(&p).min_action, Action::Rtl);
        let p = by_name("pair-battery-low+battery-critical").expect("pair");
        assert_eq!(expect_of(&p).min_action, Action::LandNow);
        assert!(p.battery.is_some());
    }

    #[test]
    fn pairs_require_both_failures_and_a_flight() {
        for s in pairs() {
            let e = expect_of(&s);
            assert!(
                e.reasons != Reasons::empty() && e.reasons != Reasons::ALL,
                "{} requires fewer than two reasons",
                s.name
            );
            assert_eq!(e.min_height_m, Some(4.0), "{}", s.name);
        }
        let p = by_name("pair-brain-crash+geofence").expect("pair");
        assert_eq!(
            expect_of(&p).reasons,
            Reasons::BRAIN_LOST.union(Reasons::GEOFENCE)
        );
        // The geofence failure always takes the first slot: its setpoint step
        // is at 15 s, and the other failure's fault (if any) at 20 s.
        assert!(p.pilot.setpoints.iter().any(|(t, _)| *t == 15.0));
        assert!(p.faults.iter().all(|f| f.start_s == 20.0));
        // Battery pairings keep the default critical level: the file does not
        // override it, so the parameter is the default.
        let p = by_name("pair-battery-critical+brain-crash").expect("pair");
        assert!(
            p.params.iter().all(|(n, _)| n != "BAT_CRIT_V"),
            "{:?}",
            p.params
        );
    }

    #[test]
    fn wind_is_light_and_repeatable() {
        let s = by_name("rc-loss-position").expect("scenario");
        for seed in 0..50 {
            let w = s.wind.config(seed);
            assert_eq!(w, s.wind.config(seed));
            assert!(w.steady_ned.norm() <= 2.0 && w.steady_ned.z == 0.0);
            assert!((0.1..0.4).contains(&w.turbulence_sigma.x));
            assert!(w.gusts.is_empty());
        }
    }
}
