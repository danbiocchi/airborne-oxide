// SPDX-License-Identifier: AGPL-3.0-or-later
//! The plan of gate 1, pinned against ADR-0007 line by line, and built from
//! the scenario files of the repository and of hand-made directories. No
//! binary needed.

use std::collections::BTreeSet;
use std::path::{Path, PathBuf};

use oxide_bridge::gate::plan::{Plan, Scoring, pair_kinds, pair_scoring};
use oxide_bridge::runner::Autopilot;
use oxide_bridge::scenario::LoadedScenario;

fn scenarios_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../scenarios")
}

/// A copy of the repository's scenario files in a directory of its own.
fn copy_of_scenarios(tag: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("oxide-gate-plan-{tag}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).expect("dir");
    for e in std::fs::read_dir(scenarios_dir()).expect("scenarios") {
        let p = e.expect("entry").path();
        if p.extension().is_some_and(|x| x == "toml") {
            std::fs::copy(&p, dir.join(p.file_name().expect("name"))).expect("copy");
        }
    }
    dir
}

fn names(plan: &Plan, scoring: Scoring) -> Vec<String> {
    plan.entries
        .iter()
        .filter(|e| e.scoring == scoring)
        .map(|e| e.scenario.clone())
        .collect()
}

fn set(list: &[&str]) -> BTreeSet<String> {
    list.iter().map(|s| (*s).to_string()).collect()
}

/// A pair file with the given extra tables after a common head.
fn write_pair(dir: &Path, name: &str, body: &str) {
    let text = format!(
        "name = \"{name}\"\nduration_s = 60.0\n[sim.links]\nbrain = true\n{body}\n\
         [pilot]\narm_at_s = 3.0\n[[pilot.mode]]\nt = 0.0\nswitch = \"offboard\"\n\
         [run]\nseeds = 10\n"
    );
    std::fs::write(dir.join(format!("{name}.toml")), text).expect("write");
}

#[test]
fn criterion_a_is_gust_hold_0_to_499_for_four_autopilots() {
    let plan = Plan::build(&scenarios_dir(), false).expect("the plan");
    let a: Vec<_> = plan
        .entries
        .iter()
        .filter(|e| e.scoring == Scoring::Gust)
        .collect();
    assert_eq!(a.len(), 1);
    assert_eq!(a[0].scenario, "gust-hold");
    assert_eq!(a[0].count, 500, "indices 0 to 499");
    let who: Vec<String> = Scoring::Gust
        .autopilots()
        .iter()
        .map(Autopilot::name)
        .collect();
    assert_eq!(
        who,
        ["arducopter", "core-f405", "core-h743", "core-f405-truth"]
    );
}

#[test]
fn criterion_b_on_the_outcome_is_the_named_files_and_the_pairs_of_those_kinds() {
    let plan = Plan::build(&scenarios_dir(), false).expect("the plan");
    let got: BTreeSet<String> = names(&plan, Scoring::Outcome).into_iter().collect();
    let mut want = set(&[
        "gps-loss",
        "gps-loss-pilot",
        "gps-loss-recovers",
        "baro-loss",
        "mag-loss",
        "rc-loss-position",
        "rc-loss-override",
        "rc-loss-on-ground",
        "rc-loss-offboard-gcs",
        "gcs-loss",
        "no-human-link",
        "battery-low",
        "battery-critical",
        "geofence",
        "brain-crash",
        "brain-hang",
        "setpoint-gap",
    ]);
    let pairs: Vec<String> = std::fs::read_dir(scenarios_dir())
        .expect("dir")
        .filter_map(|e| e.ok())
        .filter_map(|e| {
            e.path()
                .file_stem()
                .map(|s| s.to_string_lossy().into_owned())
        })
        .filter(|n| n.starts_with("pair-"))
        .collect();
    assert_eq!(pairs.len(), 21, "the repository's pair files");
    want.extend(pairs);
    assert_eq!(got, want);
    // Each on the indices its file names, 10 in every file.
    for e in plan
        .entries
        .iter()
        .filter(|e| e.scoring == Scoring::Outcome)
    {
        let src = LoadedScenario::load(scenarios_dir().join(format!("{}.toml", e.scenario)))
            .expect("loads");
        assert_eq!(e.count, src.file.run.seeds, "{}", e.scenario);
        assert_eq!(e.count, 10, "{}", e.scenario);
    }
    let who: Vec<String> = Scoring::Outcome
        .autopilots()
        .iter()
        .map(Autopilot::name)
        .collect();
    assert_eq!(who, ["arducopter", "core-f405", "core-h743"]);
}

#[test]
fn motor_loss_and_imu_loss_are_scored_on_the_time_to_motors_off() {
    let plan = Plan::build(&scenarios_dir(), false).expect("the plan");
    assert_eq!(
        names(&plan, Scoring::MotorsOff)
            .into_iter()
            .collect::<BTreeSet<_>>(),
        set(&["motor-loss", "imu-loss"])
    );
    assert_eq!(Scoring::MotorsOff.autopilots().len(), 3);
    // The failure's start time comes from the file (15 s in both).
    assert_eq!(plan.failure_starts.get("motor-loss"), Some(&15.0));
    assert_eq!(plan.failure_starts.get("imu-loss"), Some(&15.0));
}

#[test]
fn kill_in_rtl_and_hover_step_are_flown_and_not_scored() {
    let plan = Plan::build(&scenarios_dir(), false).expect("the plan");
    assert_eq!(
        names(&plan, Scoring::NotScored)
            .into_iter()
            .collect::<BTreeSet<_>>(),
        set(&["kill-in-rtl", "hover-step"])
    );
    assert_eq!(Scoring::NotScored.autopilots().len(), 3);
    let count = |n: &str| {
        plan.entries
            .iter()
            .find(|e| e.scenario == n)
            .map(|e| e.count)
    };
    assert_eq!(count("kill-in-rtl"), Some(10));
    assert_eq!(count("hover-step"), Some(1), "its file names one seed");
}

#[test]
fn the_ten_flights_flown_twice_are_the_ones_of_the_record() {
    let plan = Plan::build(&scenarios_dir(), false).expect("the plan");
    let got: Vec<(&str, u64)> = plan.twice.iter().map(|(n, c)| (n.as_str(), *c)).collect();
    assert_eq!(
        got,
        [
            ("gust-hold", 5),
            ("gps-loss", 1),
            ("battery-low", 1),
            ("rc-loss-offboard-gcs", 1),
            ("imu-loss", 1),
            ("pair-gps-loss+baro-loss", 1),
        ]
    );
    assert_eq!(plan.twice.iter().map(|(_, c)| c).sum::<u64>(), 10);
}

#[test]
fn every_file_of_the_directory_is_in_exactly_one_list() {
    let plan = Plan::build(&scenarios_dir(), false).expect("the plan");
    let in_dir = std::fs::read_dir(scenarios_dir())
        .expect("dir")
        .filter_map(|e| e.ok())
        .filter(|e| e.path().extension().is_some_and(|x| x == "toml"))
        .count();
    assert_eq!(plan.entries.len(), in_dir);
    let unique: BTreeSet<&str> = plan.entries.iter().map(|e| e.scenario.as_str()).collect();
    assert_eq!(unique.len(), in_dir, "no scenario twice");
    assert!(plan.deviations_from_the_record().is_empty());
}

#[test]
fn a_new_file_that_is_in_no_list_is_refused_with_its_name() {
    let dir = copy_of_scenarios("new");
    let text = std::fs::read_to_string(dir.join("hover-step.toml"))
        .expect("read")
        .replace("name = \"hover-step\"", "name = \"brand-new\"");
    std::fs::write(dir.join("brand-new.toml"), text).expect("write");
    let e = Plan::build(&dir, false).expect_err("refused");
    assert!(e.to_string().contains("brand-new"), "{e}");
    // Also refused in the reduced plan: nothing is left out by accident.
    let e = Plan::build(&dir, true).expect_err("refused");
    assert!(e.to_string().contains("brand-new"), "{e}");
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn a_listed_file_that_is_missing_is_refused_with_its_name() {
    let dir = copy_of_scenarios("missing");
    std::fs::remove_file(dir.join("mag-loss.toml")).expect("remove");
    let e = Plan::build(&dir, false).expect_err("refused");
    assert!(e.to_string().contains("mag-loss"), "{e}");
    // The reduced plan is allowed to lack it, and says so.
    let plan = Plan::build(&dir, true).expect("reduced");
    assert!(!plan.entries.iter().any(|e| e.scenario == "mag-loss"));
    assert!(
        plan.deviations_from_the_record()
            .iter()
            .any(|d| d.contains("mag-loss"))
    );
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn a_file_whose_name_is_not_its_scenarios_is_refused() {
    let dir = copy_of_scenarios("rename");
    std::fs::rename(dir.join("gcs-loss.toml"), dir.join("gcs-lost.toml")).expect("rename");
    let e = Plan::build(&dir, false).expect_err("refused");
    assert!(e.to_string().contains("gcs-lost"), "{e}");
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn a_pair_is_scored_by_its_failures_and_overrides_not_by_its_name() {
    let dir = copy_of_scenarios("pairs");
    // Named like a safe pair, holds a motor loss: not scored on the outcome,
    // so the plan has no list for it and refuses it.
    write_pair(
        &dir,
        "pair-gps-loss+baro-loss",
        "[[fault]]\nkind = \"gps-loss\"\nstart_s = 10.0\n\
         [[fault]]\nkind = \"motor-loss\"\nrotor = 0\nscale = 0.0\nstart_s = 12.0\n",
    );
    let e = Plan::build(&dir, false).expect_err("refused");
    assert!(
        e.to_string().contains("pair-gps-loss+baro-loss") && e.to_string().contains("motor-loss"),
        "{e}"
    );
    // Named like a motor loss, holds two outcome-scored failures: scored.
    std::fs::remove_file(dir.join("pair-gps-loss+baro-loss.toml")).expect("remove");
    write_pair(
        &dir,
        "pair-motor-loss+imu-loss",
        "[[fault]]\nkind = \"gps-loss\"\nstart_s = 10.0\n\
         [[fault]]\nkind = \"baro-loss\"\nstart_s = 12.0\n",
    );
    let plan = Plan::build(&dir, true).expect("plan");
    let e = plan
        .entries
        .iter()
        .find(|e| e.scenario == "pair-motor-loss+imu-loss")
        .expect("listed");
    assert_eq!(e.scoring, Scoring::Outcome);
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn the_kinds_of_a_pair_are_its_faults_its_battery_override_and_its_fence() {
    let one = |body: &str| {
        let text = format!(
            "name = \"x\"\nduration_s = 10.0\n[sim.links]\ngcs = true\nbrain = true\n{body}\n[pilot]\narm_at_s = 1.0\n\
             [[pilot.mode]]\nt = 0.0\nswitch = \"offboard\"\n"
        );
        LoadedScenario::from_text(&text).expect("loads").file
    };
    let k = |body: &str| pair_kinds(&one(body));
    assert_eq!(
        k("[battery]\ncapacity_mah = 4000.0\n[params]\nFS_FENCE_RAD = 10.0\n"),
        ["battery", "geofence"]
    );
    assert_eq!(
        k(
            "[[fault]]\nkind = \"rc-loss\"\nstart_s = 5.0\n[[fault]]\nkind = \"gcs-loss\"\nstart_s = 6.0\n"
        ),
        ["gcs-loss", "rc-loss"]
    );
    // A fence of radius 0 is no fence, and the same kind twice counts once.
    assert!(k("[params]\nFS_FENCE_RAD = 0.0\n").is_empty());
    assert_eq!(
        k(
            "[[fault]]\nkind = \"gps-loss\"\nstart_s = 5.0\n[[fault]]\nkind = \"gps-loss\"\nstart_s = 9.0\n"
        ),
        ["gps-loss"]
    );
    // The parameters of our core that have no failure in them are not kinds.
    assert!(k("[params]\nFS_GPS_LAND = 10.0\n").is_empty());
}

#[test]
fn only_the_kinds_of_the_outcome_list_are_scored_on_the_outcome() {
    let s = |k: &[&str]| pair_scoring(&k.iter().map(|s| (*s).to_string()).collect::<Vec<_>>());
    assert_eq!(s(&["gps-loss", "baro-loss"]), Some(Scoring::Outcome));
    assert_eq!(s(&["battery", "geofence"]), Some(Scoring::Outcome));
    assert_eq!(s(&["rc-loss", "brain-crash"]), Some(Scoring::Outcome));
    assert_eq!(s(&["mag-loss", "gcs-loss"]), Some(Scoring::Outcome));
    assert_eq!(s(&["brain-hang"]), Some(Scoring::Outcome));
    assert_eq!(s(&["gps-loss", "motor-loss"]), None);
    assert_eq!(s(&["imu-loss", "baro-loss"]), None);
    assert_eq!(s(&["rc-silent", "baro-loss"]), None);
    assert_eq!(s(&[]), None, "a pair with no failure in it is in no list");
}

#[test]
fn every_scenario_planned_for_arducopter_translates_and_the_stick_files_are_flown() {
    let plan = Plan::build(&scenarios_dir(), false).expect("the plan");
    let mut planned = 0;
    let mut sticks = Vec::new();
    for e in &plan.entries {
        if !e.autopilots_include_arducopter() {
            continue;
        }
        let loaded = LoadedScenario::load(scenarios_dir().join(format!("{}.toml", e.scenario)))
            .expect("the file loads");
        if let Err(why) = oxide_bridge::scenario::translate(&loaded.file) {
            panic!(
                "{} is planned for ArduCopter and does not translate: {why}",
                e.scenario
            );
        }
        planned += 1;
        let text = std::fs::read_to_string(&loaded.path).expect("read");
        if text.contains("[[pilot.sticks]]") {
            sticks.push(e.scenario.clone());
        }
    }
    // Every entry of the plan is flown with ArduCopter.
    assert_eq!(planned, plan.entries.len());
    // The files with a stick script are in the plan for it.
    assert!(!sticks.is_empty(), "no stick file was found");
    for s in ["gps-loss-pilot", "rc-loss-override", "rc-loss-position"] {
        assert!(sticks.iter().any(|x| x == s), "{s} not in {sticks:?}");
    }
}

#[test]
fn the_records_pair_list_is_the_repositorys_pair_files() {
    let mut files: Vec<String> = std::fs::read_dir(scenarios_dir())
        .expect("dir")
        .filter_map(|e| e.ok())
        .filter_map(|e| {
            let p = e.path();
            let stem = p.file_stem()?.to_string_lossy().into_owned();
            (p.extension().is_some_and(|x| x == "toml") && stem.starts_with("pair-"))
                .then_some(stem)
        })
        .collect();
    files.sort();
    let mut listed: Vec<String> = oxide_bridge::gate::plan::PAIR_NAMES
        .iter()
        .map(|s| (*s).to_string())
        .collect();
    listed.sort();
    assert_eq!(files, listed);
    assert_eq!(listed.len(), 21);
}

#[test]
fn a_pair_scenario_left_out_is_refused_and_a_reduced_plan_without_it_is_a_trial() {
    let dir = copy_of_scenarios("pair-left-out");
    std::fs::remove_file(dir.join("pair-rc-loss+geofence.toml")).expect("remove");
    let e = Plan::build(&dir, false).expect_err("refused");
    assert!(e.to_string().contains("pair-rc-loss+geofence"), "{e}");
    let plan = Plan::build(&dir, true).expect("reduced");
    assert_eq!(plan.entries.len(), 42);
    let d = plan.deviations_from_the_record();
    assert!(
        d.iter()
            .any(|d| d.contains("pair-rc-loss+geofence") && d.contains("not in the plan")),
        "{d:?}"
    );
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn a_plan_with_a_pair_the_record_does_not_name_is_a_trial() {
    let dir = copy_of_scenarios("pair-extra");
    write_pair(
        &dir,
        "pair-gps-loss+mag-loss",
        "[[fault]]\nkind = \"gps-loss\"\nstart_s = 10.0\n\
         [[fault]]\nkind = \"mag-loss\"\nstart_s = 12.0\n",
    );
    let plan = Plan::build(&dir, false).expect("a valid pair is built");
    let d = plan.deviations_from_the_record();
    assert!(
        d.iter()
            .any(|d| d.contains("pair-gps-loss+mag-loss") && d.contains("not in the record")),
        "{d:?}"
    );
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn every_entry_of_the_plan_is_compared_with_the_record_by_scoring_and_count() {
    let mut plan = Plan::build(&scenarios_dir(), false).expect("the plan");
    assert!(plan.deviations_from_the_record().is_empty());
    // A pair flown on 9 indices, one scored as not scored, one dropped.
    let i = plan
        .entries
        .iter()
        .position(|e| e.scenario == "pair-gps-loss+geofence")
        .expect("pair");
    plan.entries[i].count = 9;
    let j = plan
        .entries
        .iter()
        .position(|e| e.scenario == "pair-rc-loss+gps-loss")
        .expect("pair");
    plan.entries[j].scoring = Scoring::NotScored;
    let k = plan
        .entries
        .iter()
        .position(|e| e.scenario == "pair-battery-low+brain-crash")
        .expect("pair");
    plan.entries.remove(k);
    let d = plan.deviations_from_the_record();
    for word in [
        "pair-gps-loss+geofence is flown on 9",
        "pair-rc-loss+gps-loss is scored not-scored",
        "pair-battery-low+brain-crash is not in the plan",
    ] {
        assert!(d.iter().any(|d| d.contains(word)), "{word}: {d:?}");
    }
}
