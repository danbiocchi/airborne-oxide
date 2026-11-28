// SPDX-License-Identifier: AGPL-3.0-or-later
//! The files the gate is made of, as this program was built with them: the
//! scenario files of the repository's `scenarios/` and the parameter files of
//! `arducopter/`. `gate fly` compares what it reads from disk with them and
//! `gate verdict` compares what `gate.txt` and the audits say with them, so
//! that a gate flown from edited files is never taken for the commit's gate
//! (ADR-0007 section 7: no verdict when the tables come from more than one
//! commit, binary or parameter set, and never a pass for a plan that is not
//! the record's).
//!
//! A test holds the list to the directory: a scenario file added to the
//! repository without being listed here (or to the record's lists in
//! [`crate::gate::plan`]) fails it.

use crate::gate::plan::{Plan, Scoring, failure_start, pair_kinds};
use crate::parm;
use crate::runner::committed_params;
use crate::scenario::{LoadedScenario, translate};
use crate::sha256;
use crate::sitl::{PINNED, effective_params};

/// Every scenario file of the repository: its name and its text.
pub const SCENARIO_FILES: &[(&str, &str)] = &[
    (
        "baro-loss",
        include_str!("../../../../scenarios/baro-loss.toml"),
    ),
    (
        "battery-critical",
        include_str!("../../../../scenarios/battery-critical.toml"),
    ),
    (
        "battery-low",
        include_str!("../../../../scenarios/battery-low.toml"),
    ),
    (
        "brain-crash",
        include_str!("../../../../scenarios/brain-crash.toml"),
    ),
    (
        "brain-hang",
        include_str!("../../../../scenarios/brain-hang.toml"),
    ),
    (
        "gcs-loss",
        include_str!("../../../../scenarios/gcs-loss.toml"),
    ),
    (
        "geofence",
        include_str!("../../../../scenarios/geofence.toml"),
    ),
    (
        "gps-loss",
        include_str!("../../../../scenarios/gps-loss.toml"),
    ),
    (
        "gps-loss-pilot",
        include_str!("../../../../scenarios/gps-loss-pilot.toml"),
    ),
    (
        "gps-loss-recovers",
        include_str!("../../../../scenarios/gps-loss-recovers.toml"),
    ),
    (
        "gust-hold",
        include_str!("../../../../scenarios/gust-hold.toml"),
    ),
    (
        "hover-step",
        include_str!("../../../../scenarios/hover-step.toml"),
    ),
    (
        "imu-loss",
        include_str!("../../../../scenarios/imu-loss.toml"),
    ),
    (
        "kill-in-rtl",
        include_str!("../../../../scenarios/kill-in-rtl.toml"),
    ),
    (
        "mag-loss",
        include_str!("../../../../scenarios/mag-loss.toml"),
    ),
    (
        "motor-loss",
        include_str!("../../../../scenarios/motor-loss.toml"),
    ),
    (
        "no-human-link",
        include_str!("../../../../scenarios/no-human-link.toml"),
    ),
    (
        "pair-battery-critical+baro-loss",
        include_str!("../../../../scenarios/pair-battery-critical+baro-loss.toml"),
    ),
    (
        "pair-battery-critical+brain-crash",
        include_str!("../../../../scenarios/pair-battery-critical+brain-crash.toml"),
    ),
    (
        "pair-battery-critical+geofence",
        include_str!("../../../../scenarios/pair-battery-critical+geofence.toml"),
    ),
    (
        "pair-battery-low+baro-loss",
        include_str!("../../../../scenarios/pair-battery-low+baro-loss.toml"),
    ),
    (
        "pair-battery-low+battery-critical",
        include_str!("../../../../scenarios/pair-battery-low+battery-critical.toml"),
    ),
    (
        "pair-battery-low+brain-crash",
        include_str!("../../../../scenarios/pair-battery-low+brain-crash.toml"),
    ),
    (
        "pair-battery-low+geofence",
        include_str!("../../../../scenarios/pair-battery-low+geofence.toml"),
    ),
    (
        "pair-brain-crash+baro-loss",
        include_str!("../../../../scenarios/pair-brain-crash+baro-loss.toml"),
    ),
    (
        "pair-brain-crash+geofence",
        include_str!("../../../../scenarios/pair-brain-crash+geofence.toml"),
    ),
    (
        "pair-geofence+baro-loss",
        include_str!("../../../../scenarios/pair-geofence+baro-loss.toml"),
    ),
    (
        "pair-gps-loss+baro-loss",
        include_str!("../../../../scenarios/pair-gps-loss+baro-loss.toml"),
    ),
    (
        "pair-gps-loss+battery-critical",
        include_str!("../../../../scenarios/pair-gps-loss+battery-critical.toml"),
    ),
    (
        "pair-gps-loss+battery-low",
        include_str!("../../../../scenarios/pair-gps-loss+battery-low.toml"),
    ),
    (
        "pair-gps-loss+brain-crash",
        include_str!("../../../../scenarios/pair-gps-loss+brain-crash.toml"),
    ),
    (
        "pair-gps-loss+geofence",
        include_str!("../../../../scenarios/pair-gps-loss+geofence.toml"),
    ),
    (
        "pair-rc-loss+baro-loss",
        include_str!("../../../../scenarios/pair-rc-loss+baro-loss.toml"),
    ),
    (
        "pair-rc-loss+battery-critical",
        include_str!("../../../../scenarios/pair-rc-loss+battery-critical.toml"),
    ),
    (
        "pair-rc-loss+battery-low",
        include_str!("../../../../scenarios/pair-rc-loss+battery-low.toml"),
    ),
    (
        "pair-rc-loss+brain-crash",
        include_str!("../../../../scenarios/pair-rc-loss+brain-crash.toml"),
    ),
    (
        "pair-rc-loss+geofence",
        include_str!("../../../../scenarios/pair-rc-loss+geofence.toml"),
    ),
    (
        "pair-rc-loss+gps-loss",
        include_str!("../../../../scenarios/pair-rc-loss+gps-loss.toml"),
    ),
    (
        "rc-loss-offboard-gcs",
        include_str!("../../../../scenarios/rc-loss-offboard-gcs.toml"),
    ),
    (
        "rc-loss-on-ground",
        include_str!("../../../../scenarios/rc-loss-on-ground.toml"),
    ),
    (
        "rc-loss-override",
        include_str!("../../../../scenarios/rc-loss-override.toml"),
    ),
    (
        "rc-loss-position",
        include_str!("../../../../scenarios/rc-loss-position.toml"),
    ),
    (
        "setpoint-gap",
        include_str!("../../../../scenarios/setpoint-gap.toml"),
    ),
    (
        "yaw-step-large",
        include_str!("../../../../scenarios/yaw-step-large.toml"),
    ),
];

/// The text of the scenario file `name`, as built.
pub fn scenario_text(name: &str) -> Option<&'static str> {
    SCENARIO_FILES
        .iter()
        .find(|(n, _)| *n == name)
        .map(|(_, t)| *t)
}

/// The SHA-256 of the scenario file `name`, as built.
pub fn scenario_sha256(name: &str) -> Option<String> {
    scenario_text(name).map(|t| sha256::hex(&sha256::digest(t.as_bytes())))
}

/// The text of the parameter file called `name`, as built.
pub fn param_text(name: &str) -> Option<&'static str> {
    parm::embedded(name)
}

/// The SHA-256 of every parameter file a bridged flight is given, in the
/// order it is given, as built.
pub fn param_hashes() -> Result<Vec<(String, String)>, String> {
    let airframe = oxide_sim::Airframe::reference_quad().map_err(|e| e.to_string())?;
    let mut out = Vec::new();
    for path in committed_params(&airframe).0 {
        let name = path
            .file_name()
            .map_or_else(String::new, |n| n.to_string_lossy().into_owned());
        let text = param_text(&name)
            .ok_or_else(|| format!("the parameter file {name} is not built into the program"))?;
        out.push((name, sha256::hex(&sha256::digest(text.as_bytes()))));
    }
    Ok(out)
}

/// The SHA-256 of the effective parameter file a bridged flight of the
/// scenario `name` is given (the audit's `params sha256`): the committed
/// files, then the battery thresholds, then the scenario's own (the fence),
/// all as built.
pub fn expected_params_sha256(name: &str) -> Result<String, String> {
    let text = scenario_text(name).ok_or_else(|| format!("no scenario file {name} is built in"))?;
    let src = LoadedScenario::from_text(text).map_err(|e| e.to_string())?;
    let translation = translate(&src.file).map_err(|e| e.to_string())?;
    let airframe = oxide_sim::Airframe::reference_quad().map_err(|e| e.to_string())?;
    let (files, mut extras) = committed_params(&airframe);
    let mut bytes = Vec::new();
    for path in files {
        let file = path
            .file_name()
            .map_or_else(String::new, |n| n.to_string_lossy().into_owned());
        let t = param_text(&file)
            .ok_or_else(|| format!("the parameter file {file} is not built into the program"))?;
        bytes.push(t.as_bytes().to_vec());
    }
    extras.extend(translation.extra_params.iter().cloned());
    let params = effective_params(&bytes, &extras).map_err(|e| e.to_string())?;
    Ok(sha256::hex(&sha256::digest(&params)))
}

fn files_list(files: &[(String, String)]) -> String {
    files
        .iter()
        .map(|(n, h)| format!("{n} {}", h.get(..12).unwrap_or(h)))
        .collect::<Vec<_>>()
        .join(", ")
}

/// How the files a gate used are not the ones this program was built with,
/// as sentences (empty when they are): the simulator program is not the
/// pinned one, the parameter files or a scenario file of the plan have other
/// hashes. Such a gate is not the commit's gate: a trial, never a pass.
pub fn differences_from_build<'a>(
    binary_sha256: Option<&str>,
    params: &[(String, String)],
    scenario_files: &[(String, String)],
    scenarios: impl IntoIterator<Item = &'a str>,
) -> Vec<String> {
    let mut out = Vec::new();
    if binary_sha256 != Some(PINNED.sha256) {
        out.push(format!(
            "the simulator program is {} and the pinned one is {}",
            binary_sha256.unwrap_or("none"),
            PINNED.sha256
        ));
    }
    match param_hashes() {
        Ok(want) if want == params => {}
        Ok(want) => out.push(format!(
            "the parameter files of the gate ({}) are not the commit's ({})",
            files_list(params),
            files_list(&want)
        )),
        Err(e) => out.push(format!("the commit's parameter files cannot be read: {e}")),
    }
    for name in scenarios {
        let got = scenario_files
            .iter()
            .find(|(n, _)| n == name)
            .map(|(_, h)| h.as_str());
        let want = scenario_sha256(name);
        if got.is_none() || got != want.as_deref() {
            out.push(format!(
                "the scenario file {name} has SHA-256 {} and the commit's is {}",
                got.unwrap_or("none"),
                want.as_deref().unwrap_or("not in the repository")
            ));
        }
    }
    out
}

/// The refusal of `gate fly` to fly the gate with files that are not the
/// commit's (`differences` from [`differences_from_build`]); a trial, which
/// may, passes.
pub fn refuse_others_for_the_gate(differences: &[String], trial: bool) -> Result<(), String> {
    if differences.is_empty() || trial {
        return Ok(());
    }
    Err(format!(
        "the gate is flown with the commit's own files only, and these are not ({}): restore \
         them, or give --reduced for a trial, which is never a pass",
        differences.join("; ")
    ))
}

/// How the facts `gate.txt` states about its plan (what is in each pair, the
/// start of the failure that a time to motors off counts from) are not the
/// facts of the commit's own scenario files, as sentences. They decide
/// criteria B1 to B3, so they are read from the built-in files and not taken
/// on trust.
pub fn plan_fact_differences(plan: &Plan) -> Vec<String> {
    let mut out = Vec::new();
    for e in &plan.entries {
        // A scenario that is not built in is reported by its hash.
        let Some(text) = scenario_text(&e.scenario) else {
            continue;
        };
        let Ok(src) = LoadedScenario::from_text(text) else {
            continue;
        };
        if e.scenario.starts_with("pair-") {
            let want = pair_kinds(&src.file);
            if plan.pair_kinds.get(&e.scenario) != Some(&want) {
                out.push(format!(
                    "gate.txt says {} holds {:?} and the commit's file holds {}",
                    e.scenario,
                    plan.pair_kinds.get(&e.scenario).map(|k| k.join(" and ")),
                    want.join(" and ")
                ));
            }
        }
        if e.scoring == Scoring::MotorsOff {
            let want = failure_start(&src.file, &e.scenario);
            if plan.failure_starts.get(&e.scenario).copied() != want {
                out.push(format!(
                    "gate.txt says the failure of {} starts at {:?} s and the commit's file says \
                     {want:?} s",
                    e.scenario,
                    plan.failure_starts.get(&e.scenario)
                ));
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gate::plan::{
        GUST, MOTORS_OFF_NAMES, NOT_FOR_THE_GATE, NOT_SCORED_NAMES, OUTCOME_NAMES, PAIR_NAMES,
    };
    use std::path::PathBuf;

    fn dir() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../scenarios")
    }

    #[test]
    fn the_hover_step_count_of_the_plan_is_the_count_of_its_built_in_file() {
        let text = scenario_text("hover-step").expect("built in");
        let src = LoadedScenario::from_text(text).expect("parses");
        assert_eq!(src.file.run.seeds, crate::gate::plan::HOVER_STEP_SEEDS);
    }

    #[test]
    fn the_built_in_scenario_files_are_the_repositorys_files_and_the_records_names() {
        let mut on_disk: Vec<String> = std::fs::read_dir(dir())
            .expect("dir")
            .filter_map(Result::ok)
            .filter_map(|e| {
                let p = e.path();
                (p.extension().is_some_and(|x| x == "toml"))
                    .then(|| p.file_stem().map(|s| s.to_string_lossy().into_owned()))
                    .flatten()
            })
            .collect();
        on_disk.sort();
        let mut built: Vec<String> = SCENARIO_FILES
            .iter()
            .map(|(n, _)| (*n).to_string())
            .collect();
        built.sort();
        assert_eq!(
            built, on_disk,
            "a scenario file is not built in (or is built in twice)"
        );
        let mut record: Vec<String> = vec![GUST.to_string()];
        for list in [
            &OUTCOME_NAMES[..],
            &MOTORS_OFF_NAMES[..],
            &NOT_SCORED_NAMES[..],
            &PAIR_NAMES[..],
            &NOT_FOR_THE_GATE[..],
        ] {
            record.extend(list.iter().map(|s| (*s).to_string()));
        }
        record.sort();
        assert_eq!(
            record, on_disk,
            "the record's lists and the directory differ"
        );
        for (name, text) in SCENARIO_FILES {
            let disk = std::fs::read_to_string(dir().join(format!("{name}.toml"))).expect("file");
            assert_eq!(&disk, text, "{name} was built in as another text");
        }
    }

    #[test]
    fn the_built_in_parameter_files_are_every_file_a_flight_is_given_and_the_files_on_disk() {
        let airframe = oxide_sim::Airframe::reference_quad().expect("airframe");
        let files = committed_params(&airframe).0;
        assert!(!files.is_empty());
        for p in files {
            let name = p.file_name().expect("name").to_string_lossy().into_owned();
            let built = param_text(&name).unwrap_or_else(|| panic!("{name} is not built in"));
            assert_eq!(std::fs::read_to_string(&p).expect("file"), built, "{name}");
        }
        let hashes = param_hashes().expect("hashes");
        assert_eq!(hashes.len(), committed_params(&airframe).0.len());
    }

    /// The effective parameter file the real program is given in a bridged
    /// flight (the audits' `params sha256`): the reconstruction here is the
    /// runner's. The values moved with the tuned file of ADR-0009. Both are
    /// the hash of a real flight's audit: the first is compared in
    /// `arducopter_tune` (`a_flight_that_loads_a_tuned_file_with_notch_lines_...`),
    /// the second in `arducopter_scenario`, by a geofence flight:
    /// `the_fence_parameters_are_in_the_program_and_the_flight_shows_the_fence_acting`.
    #[test]
    fn the_expected_parameter_hash_is_the_one_real_flights_recorded() {
        let plain = "71b263adc1cc30e8984eb0a2ac3a782f5bea17c3db6e51a3e54a78a3fe5caf65";
        let fenced = "49be7ab7308148c26a9bb7477378b34db679684b22f1e17453db8f652bc9d75c";
        for s in ["gps-loss", "gust-hold", "battery-low"] {
            assert_eq!(expected_params_sha256(s).as_deref(), Ok(plain), "{s}");
        }
        for s in ["geofence", "pair-rc-loss+geofence"] {
            assert_eq!(expected_params_sha256(s).as_deref(), Ok(fenced), "{s}");
        }
        assert!(expected_params_sha256("no-such-scenario").is_err());
    }

    #[test]
    fn the_gate_is_not_flown_with_other_files_and_a_trial_may_be() {
        let d =
            vec!["the scenario file gust-hold has SHA-256 00 and the commit's is 11".to_string()];
        let e = refuse_others_for_the_gate(&d, false).expect_err("refused");
        assert!(e.contains("gust-hold") && e.contains("--reduced"), "{e}");
        assert_eq!(refuse_others_for_the_gate(&d, true), Ok(()));
        assert_eq!(refuse_others_for_the_gate(&[], false), Ok(()));
        // What it is given for the commit's own files is no difference.
        let params = param_hashes().expect("hashes");
        let one = [(
            "gust-hold".to_string(),
            scenario_sha256("gust-hold").expect("sha"),
        )];
        let own = differences_from_build(Some(PINNED.sha256), &params, &one, ["gust-hold"]);
        assert_eq!(refuse_others_for_the_gate(&own, false), Ok(()));
    }

    #[test]
    fn a_file_other_than_the_builds_is_a_difference_and_the_builds_own_are_none() {
        let params = param_hashes().expect("hashes");
        let names = ["gust-hold", "gps-loss"];
        let scen: Vec<(String, String)> = names
            .iter()
            .map(|n| ((*n).to_string(), scenario_sha256(n).expect("sha")))
            .collect();
        assert!(differences_from_build(Some(PINNED.sha256), &params, &scen, names).is_empty());
        let mut other = scen.clone();
        other[0].1 = "0".repeat(64);
        let d = differences_from_build(Some(PINNED.sha256), &params, &other, names);
        assert!(d.len() == 1 && d[0].contains("gust-hold"), "{d:?}");
        let mut p2 = params.clone();
        p2[0].1 = "0".repeat(64);
        assert_eq!(
            differences_from_build(Some(PINNED.sha256), &p2, &scen, names).len(),
            1
        );
        assert_eq!(
            differences_from_build(Some("00"), &params, &scen, names).len(),
            1
        );
        assert_eq!(differences_from_build(None, &params, &scen, names).len(), 1);
        assert_eq!(
            differences_from_build(Some(PINNED.sha256), &params, &scen, ["mag-loss"]).len(),
            1
        );
    }
}
