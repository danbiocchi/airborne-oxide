// SPDX-License-Identifier: AGPL-3.0-or-later
//! The plan of gate 1 as data (ADR-0007 sections 5 to 7): which scenario is
//! flown by which autopilots on which indices, and how each is scored.
//!
//! [`Plan::build`] reads the scenario directory and fails with a sentence when
//! a file of the directory is in no list or a listed file is missing, so
//! that nothing is scored or left out by accident. A `pair-*` file is scored
//! by what is in it ([`pair_kinds`], [`pair_scoring`]), never by its name.

use std::collections::BTreeMap;
use std::fmt;
use std::path::Path;

use oxide_sim::faults::FaultKind;
use oxide_sim::scenario_file::{RunProfile, ScenarioFile};

use crate::runner::Autopilot;
use crate::scenario::LoadedScenario;

/// The scenario of criterion A.
pub const GUST: &str = "gust-hold";
/// Criterion A is flown on the indices 0 to this number, not included.
pub const GUST_SEEDS: u64 = 500;
/// Indices of every other scenario of the record ("the seeds its file names
/// (10)", ADR-0007 section 6).
pub const B_SEEDS: u64 = 10;

/// Criterion B, scored on the outcome: the named files (every `pair-*` file
/// whose failures are all of the kinds in [`OUTCOME_KINDS`] is added).
pub const OUTCOME_NAMES: [&str; 17] = [
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
];

/// Criterion B, scored on the outcome: the `pair-*` files of ADR-0007
/// section 6, "every `pair-*` scenario whose two failures are both in this
/// list". The record's plan holds exactly these; a test holds the list to the
/// repository's directory, and [`Plan::build`] and
/// [`Plan::deviations_from_the_record`] refuse a plan with one more or one
/// fewer, so that a pair left out cannot go unnoticed.
pub const PAIR_NAMES: [&str; 21] = [
    "pair-battery-critical+baro-loss",
    "pair-battery-critical+brain-crash",
    "pair-battery-critical+geofence",
    "pair-battery-low+baro-loss",
    "pair-battery-low+battery-critical",
    "pair-battery-low+brain-crash",
    "pair-battery-low+geofence",
    "pair-brain-crash+baro-loss",
    "pair-brain-crash+geofence",
    "pair-geofence+baro-loss",
    "pair-gps-loss+baro-loss",
    "pair-gps-loss+battery-critical",
    "pair-gps-loss+battery-low",
    "pair-gps-loss+brain-crash",
    "pair-gps-loss+geofence",
    "pair-rc-loss+baro-loss",
    "pair-rc-loss+battery-critical",
    "pair-rc-loss+battery-low",
    "pair-rc-loss+brain-crash",
    "pair-rc-loss+geofence",
    "pair-rc-loss+gps-loss",
];

/// Criterion B, scored on the time to motors off.
pub const MOTORS_OFF_NAMES: [&str; 2] = ["motor-loss", "imu-loss"];

/// Flown and not scored.
pub const NOT_SCORED_NAMES: [&str; 2] = ["kill-in-rtl", "hover-step"];

/// Flown twice, with ArduCopter only: the scenario and how many indices
/// from 0 (ADR-0007 section 7: ten flights).
pub const TWICE: [(&str, u64); 6] = [
    ("gust-hold", 5),
    ("gps-loss", 1),
    ("battery-low", 1),
    ("rc-loss-offboard-gcs", 1),
    ("imu-loss", 1),
    ("pair-gps-loss+baro-loss", 1),
];

/// The failure kinds a `pair-*` file may hold to be scored on the outcome:
/// the kinds of the named outcome-scored files (`rc-loss` is the kind of the
/// `rc-loss-*` files and `no-human-link`, `battery` a `[battery]` override,
/// `geofence` a fence radius). `motor-loss`, `imu-loss` and `rc-silent` are
/// not among them.
pub const OUTCOME_KINDS: [&str; 8] = [
    "gps-loss",
    "baro-loss",
    "mag-loss",
    "rc-loss",
    "gcs-loss",
    "brain-crash",
    "brain-hang",
    "battery",
];

/// How a scenario is scored.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Scoring {
    /// Criterion A.
    Gust,
    /// Criterion B1 and B2: the outcome.
    Outcome,
    /// Criterion B3: the time to motors off.
    MotorsOff,
    /// Flown, reported, decides nothing.
    NotScored,
}

impl Scoring {
    /// The word in `gate.txt`.
    pub fn name(self) -> &'static str {
        match self {
            Self::Gust => "gust",
            Self::Outcome => "outcome",
            Self::MotorsOff => "motors-off",
            Self::NotScored => "not-scored",
        }
    }

    pub fn parse(text: &str) -> Option<Self> {
        [Self::Gust, Self::Outcome, Self::MotorsOff, Self::NotScored]
            .into_iter()
            .find(|s| s.name() == text)
    }

    /// Who flies it, in the order of the tables.
    pub fn autopilots(self) -> Vec<Autopilot> {
        let mut v = vec![
            Autopilot::ArduCopter,
            Autopilot::Core(RunProfile::F405),
            Autopilot::Core(RunProfile::H743),
        ];
        if self == Self::Gust {
            v.push(Autopilot::CoreTruth);
        }
        v
    }
}

/// One scenario of the plan.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Entry {
    pub scenario: String,
    pub scoring: Scoring,
    /// Indices `0..count`.
    pub count: u64,
}

impl Entry {
    /// Whether ArduCopter flies this scenario (every scenario of the plan).
    pub fn autopilots_include_arducopter(&self) -> bool {
        self.scoring.autopilots().contains(&Autopilot::ArduCopter)
    }
}

/// Why a plan could not be made.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PlanError(pub String);

impl fmt::Display for PlanError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for PlanError {}

fn err<T>(text: impl Into<String>) -> Result<T, PlanError> {
    Err(PlanError(text.into()))
}

/// The gate's plan.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Plan {
    pub entries: Vec<Entry>,
    /// Scenario and the number of indices from 0 flown twice (ArduCopter).
    pub twice: Vec<(String, u64)>,
    /// The failure kinds of each `pair-*` scenario, as read from its file.
    pub pair_kinds: BTreeMap<String, Vec<String>>,
    /// The start of the failure, s, of each scenario scored on the time to
    /// motors off.
    pub failure_starts: BTreeMap<String, f64>,
}

/// The name of a fault kind in the files.
fn kind_name(kind: &FaultKind) -> &'static str {
    match kind {
        FaultKind::MotorLoss { .. } => "motor-loss",
        FaultKind::GpsLoss => "gps-loss",
        FaultKind::ImuLoss => "imu-loss",
        FaultKind::BaroLoss => "baro-loss",
        FaultKind::MagLoss => "mag-loss",
        FaultKind::RcLoss => "rc-loss",
        FaultKind::RcSilent => "rc-silent",
        FaultKind::GcsLoss => "gcs-loss",
        FaultKind::BrainCrash => "brain-crash",
        FaultKind::BrainHang => "brain-hang",
    }
}

/// The failures and overrides a scenario file holds, by kind, sorted and
/// each once: its `[[fault]]` kinds, `battery` for a `[battery]` override and
/// `geofence` for a fence radius above zero (`FS_FENCE_RAD`).
pub fn pair_kinds(file: &ScenarioFile) -> Vec<String> {
    let mut kinds: Vec<String> = file
        .faults
        .iter()
        .map(|f| kind_name(&f.kind).to_string())
        .collect();
    if file.battery.is_some() {
        kinds.push("battery".into());
    }
    if file
        .params
        .iter()
        .any(|(n, v)| n == "FS_FENCE_RAD" && *v > 0.0)
    {
        kinds.push("geofence".into());
    }
    kinds.sort();
    kinds.dedup();
    kinds
}

/// How a `pair-*` file with these kinds is scored: on the outcome when it has
/// at least one and every one is in [`OUTCOME_KINDS`] (or is `geofence`);
/// `None` when the record has no list for it.
pub fn pair_scoring(kinds: &[String]) -> Option<Scoring> {
    let allowed = |k: &String| k == "geofence" || OUTCOME_KINDS.contains(&k.as_str());
    (!kinds.is_empty() && kinds.iter().all(allowed)).then_some(Scoring::Outcome)
}

/// The scoring the record gives a scenario by name (not a pair).
pub fn named_scoring(name: &str) -> Option<Scoring> {
    if name == GUST {
        Some(Scoring::Gust)
    } else if OUTCOME_NAMES.contains(&name) {
        Some(Scoring::Outcome)
    } else if MOTORS_OFF_NAMES.contains(&name) {
        Some(Scoring::MotorsOff)
    } else if NOT_SCORED_NAMES.contains(&name) {
        Some(Scoring::NotScored)
    } else {
        None
    }
}

impl Plan {
    /// The plan of the scenario files in `dir`. With `reduced`, a listed file
    /// may be missing and criterion A is flown on the indices its file names:
    /// the plan of a trial, which [`Plan::deviations_from_the_record`] says
    /// is not the gate.
    pub fn build(dir: &Path, reduced: bool) -> Result<Self, PlanError> {
        let read = std::fs::read_dir(dir).map_err(|e| {
            PlanError(format!(
                "cannot read the scenario directory {}: {e}",
                dir.display()
            ))
        })?;
        let mut paths: Vec<_> = read
            .filter_map(Result::ok)
            .map(|e| e.path())
            .filter(|p| p.extension().is_some_and(|x| x == "toml"))
            .collect();
        paths.sort();
        let mut plan = Self::default();
        let mut found: BTreeMap<String, (Scoring, u64, ScenarioFile)> = BTreeMap::new();
        for path in paths {
            let src = LoadedScenario::load(&path).map_err(|e| PlanError(e.to_string()))?;
            let stem = path
                .file_stem()
                .map(|s| s.to_string_lossy().into_owned())
                .unwrap_or_default();
            let name = src.file.name.clone();
            if stem != name {
                return err(format!(
                    "the file {} holds the scenario {name:?}: a file is named for its scenario",
                    path.display()
                ));
            }
            let seeds = src.file.run.seeds;
            let scoring = if let Some(s) = named_scoring(&name) {
                s
            } else if name.starts_with("pair-") {
                let kinds = pair_kinds(&src.file);
                let Some(s) = pair_scoring(&kinds) else {
                    return err(format!(
                        "the scenario {name:?} is a pair with {} in it, which the record scores \
                         on no list (ADR-0007 section 6): it is not scored by its name, and a \
                         pair with these failures has no place in the gate until the record says",
                        if kinds.is_empty() {
                            "no failure".to_string()
                        } else {
                            kinds.join(" and ")
                        }
                    ));
                };
                plan.pair_kinds.insert(name.clone(), kinds);
                s
            } else {
                return err(format!(
                    "the scenario {name:?} ({}) is in no list of ADR-0007 sections 5 and 6: \
                     nothing is scored or left out by accident, so the record names it first",
                    path.display()
                ));
            };
            let count = if scoring == Scoring::Gust && !reduced {
                GUST_SEEDS
            } else {
                seeds
            };
            found.insert(name, (scoring, count, src.file));
        }
        let mut missing = Vec::new();
        let mut order: Vec<String> = vec![GUST.to_string()];
        order.extend(OUTCOME_NAMES.iter().map(|s| (*s).to_string()));
        order.extend(PAIR_NAMES.iter().map(|s| (*s).to_string()));
        order.extend(
            found
                .keys()
                .filter(|n| n.starts_with("pair-") && !PAIR_NAMES.contains(&n.as_str()))
                .cloned(),
        );
        order.extend(MOTORS_OFF_NAMES.iter().map(|s| (*s).to_string()));
        order.extend(NOT_SCORED_NAMES.iter().map(|s| (*s).to_string()));
        for name in order {
            match found.get(&name) {
                Some((scoring, count, file)) => {
                    if *scoring == Scoring::MotorsOff {
                        let start = file
                            .faults
                            .iter()
                            .filter(|f| kind_name(&f.kind) == name)
                            .map(|f| f.start_s)
                            .fold(None, |a: Option<f64>, s| Some(a.map_or(s, |a| a.min(s))));
                        let Some(start) = start else {
                            return err(format!(
                                "the scenario {name:?} has no {name} failure: the time to motors \
                                 off is counted from its start"
                            ));
                        };
                        plan.failure_starts.insert(name.clone(), start);
                    }
                    plan.entries.push(Entry {
                        scenario: name,
                        scoring: *scoring,
                        count: *count,
                    });
                }
                None => missing.push(name),
            }
        }
        if !missing.is_empty() && !reduced {
            return err(format!(
                "the scenario directory {} lacks {}: the gate's plan lists them (ADR-0007)",
                dir.display(),
                missing
                    .iter()
                    .map(|m| format!("{m}.toml"))
                    .collect::<Vec<_>>()
                    .join(", ")
            ));
        }
        for (name, n) in TWICE {
            match plan.entries.iter().find(|e| e.scenario == name) {
                Some(e) => plan.twice.push((name.to_string(), n.min(e.count))),
                None if reduced => {}
                None => {
                    return err(format!(
                        "the flight flown twice, {name}, is not in the plan"
                    ));
                }
            }
        }
        Ok(plan)
    }

    /// How the plan departs from the record's, as sentences: empty for the
    /// gate itself. A plan with any is a trial and its verdict is never a
    /// pass. The comparison is with the record's whole plan: every scenario
    /// the record names (the 21 pairs included) with its scoring and its
    /// number of indices, and no scenario the record does not name.
    pub fn deviations_from_the_record(&self) -> Vec<String> {
        let mut out = Vec::new();
        let mut fixed: Vec<(String, Scoring, u64)> =
            vec![(GUST.to_string(), Scoring::Gust, GUST_SEEDS)];
        fixed.extend(
            OUTCOME_NAMES
                .iter()
                .map(|n| ((*n).to_string(), Scoring::Outcome, B_SEEDS)),
        );
        fixed.extend(
            PAIR_NAMES
                .iter()
                .map(|n| ((*n).to_string(), Scoring::Outcome, B_SEEDS)),
        );
        fixed.extend(
            MOTORS_OFF_NAMES
                .iter()
                .map(|n| ((*n).to_string(), Scoring::MotorsOff, B_SEEDS)),
        );
        fixed.push(("kill-in-rtl".to_string(), Scoring::NotScored, B_SEEDS));
        for (name, scoring, count) in &fixed {
            match self.entries.iter().find(|e| &e.scenario == name) {
                None => out.push(format!("the scenario {name} is not in the plan")),
                Some(e) => {
                    if e.scoring != *scoring {
                        out.push(format!(
                            "{name} is scored {} and the record says {}",
                            e.scoring.name(),
                            scoring.name()
                        ));
                    }
                    if e.count != *count {
                        out.push(format!(
                            "{name} is flown on {} indices and the record says {count}",
                            e.count
                        ));
                    }
                }
            }
        }
        match self.entries.iter().find(|e| e.scenario == "hover-step") {
            None => out.push("the scenario hover-step is not in the plan".to_string()),
            Some(e) if e.count == 0 => out.push("hover-step is flown on no index".to_string()),
            Some(e) if e.scoring != Scoring::NotScored => out.push(format!(
                "hover-step is scored {} and the record says not-scored",
                e.scoring.name()
            )),
            Some(_) => {}
        }
        for e in &self.entries {
            let known =
                e.scenario == "hover-step" || fixed.iter().any(|(n, _, _)| n == &e.scenario);
            if !known {
                out.push(format!(
                    "the scenario {} is in the plan and not in the record's",
                    e.scenario
                ));
            }
        }
        let record: Vec<(String, u64)> =
            TWICE.iter().map(|(n, c)| ((*n).to_string(), *c)).collect();
        if self.twice != record {
            out.push(format!(
                "the flights flown twice are {:?} and the record says {record:?}",
                self.twice
            ));
        }
        out
    }

    /// What is wrong with the plan itself, as sentences (empty when sound): a
    /// scenario of the record with the wrong scoring, a scenario in no list,
    /// a pair that the record does not score on the outcome, a scenario twice.
    pub fn inconsistencies(&self) -> Vec<String> {
        let mut out = Vec::new();
        let mut seen = std::collections::BTreeSet::new();
        for e in &self.entries {
            if !seen.insert(e.scenario.as_str()) {
                out.push(format!("the scenario {} is in the plan twice", e.scenario));
            }
            if e.count == 0 {
                out.push(format!("the scenario {} is flown on no index", e.scenario));
            }
            if let Some(want) = named_scoring(&e.scenario) {
                if want != e.scoring {
                    out.push(format!(
                        "{} is scored {} in the plan and the record says {}",
                        e.scenario,
                        e.scoring.name(),
                        want.name()
                    ));
                }
            } else if e.scenario.starts_with("pair-") {
                match self.pair_kinds.get(&e.scenario) {
                    Some(k) if pair_scoring(k) == Some(e.scoring) => {}
                    Some(k) => out.push(format!(
                        "{} is scored {} but its failures ({}) are not scored so",
                        e.scenario,
                        e.scoring.name(),
                        k.join(" and ")
                    )),
                    None => out.push(format!("the plan does not say what is in {}", e.scenario)),
                }
            } else {
                out.push(format!("{} is in no list of the record", e.scenario));
            }
            if e.scoring == Scoring::MotorsOff && !self.failure_starts.contains_key(&e.scenario) {
                out.push(format!(
                    "the plan has no failure start time for {}",
                    e.scenario
                ));
            }
        }
        for (name, n) in &self.twice {
            match self.entries.iter().find(|e| &e.scenario == name) {
                Some(e) if *n >= 1 && *n <= e.count => {}
                _ => out.push(format!("{name} cannot be flown twice on {n} indices")),
            }
        }
        out
    }
}
