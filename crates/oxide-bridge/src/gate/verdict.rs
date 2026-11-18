// SPDX-License-Identifier: AGPL-3.0-or-later
//! The verdict of gate 1 (ADR-0007 sections 5 to 7), computed from the tables
//! and `gate.txt` alone. Nothing is flown here.
//!
//! [`read`] checks that the tables are complete and consistent and turns them
//! into typed flights; any failure there is a list of sentences and no
//! verdict ([`EXIT_INCOMPLETE`]). [`evaluate`] then computes each criterion
//! from the flights. A pass is reported only when the plan is the record's
//! plan and every criterion holds on rows that are all there.
//!
//! Readings where the record's words leave a choice, and the one built:
//! the median of an even count is the mean of the two middle values; an
//! ArduCopter with no safe gust flight fails A2; "the time to motors off"
//! of a flight is `last_motor_on_s` less the failure's start, so a flight
//! whose motors never stop counts as the time from the failure to the end of
//! the flight; the win counts are over every index, ArduCopter's unsafe
//! flights included.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write as _;
use std::path::{Path, PathBuf};

use oxide_sim::rng::run_seed;

use crate::batch::run_dir;
use crate::gate::files;
use crate::gate::info::{DIRTY_NOTE, GateInfo};
use crate::gate::plan::{Entry, GUST, Scoring};
use crate::gate::stats::{mean, median, percentile_95};
use crate::record::{COLUMNS, Row, header};

/// The gate is passed.
pub const EXIT_PASSED: u8 = 0;
/// The tables are incomplete or inconsistent: no verdict.
pub const EXIT_INCOMPLETE: u8 = 2;
/// The gate is not passed.
pub const EXIT_NOT_PASSED: u8 = 4;

/// The file name of a table.
pub const TABLE: &str = "flights.csv";
/// The directory of the flights flown twice.
pub const TWICE_DIR: &str = "twice";

/// The directory of the table of `scenario` flown by `autopilot`.
pub fn table_dir(out: &Path, scenario: &str, autopilot: &str) -> PathBuf {
    out.join(scenario).join(autopilot)
}

/// The directory of the table of the twice-flown flights of `scenario`.
pub fn twice_dir(out: &Path, scenario: &str) -> PathBuf {
    out.join(TWICE_DIR).join(scenario)
}

/// How a flight ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Class {
    Safe,
    Crash,
    FlyAway,
}

impl Class {
    fn parse(text: &str) -> Option<Self> {
        match text {
            "safe" => Some(Self::Safe),
            "crash" => Some(Self::Crash),
            "fly-away" => Some(Self::FlyAway),
            _ => None,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Self::Safe => "safe",
            Self::Crash => "crash",
            Self::FlyAway => "fly-away",
        }
    }
}

/// What the row's `verdict` column says.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Judged {
    Pass,
    Fail,
    NotJudged,
}

impl Judged {
    fn parse(text: &str) -> Option<Self> {
        match text {
            "pass" => Some(Self::Pass),
            "fail" => Some(Self::Fail),
            "not-judged" => Some(Self::NotJudged),
            _ => None,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Self::Pass => "pass",
            Self::Fail => "fail",
            Self::NotJudged => "not-judged",
        }
    }
}

/// One flight of a table, as the verdict reads it.
#[derive(Clone, Debug, PartialEq)]
pub struct Flight {
    pub scenario: String,
    pub autopilot: String,
    pub index: u64,
    pub class: Class,
    pub judged: Judged,
    pub rms_h_m: Option<f64>,
    pub rms_v_m: Option<f64>,
    pub max_tilt_deg: Option<f64>,
    pub last_motor_on_s: Option<f64>,
    pub repeats: Option<bool>,
    pub trace_hash: String,
    pub packet_hash: String,
    pub reply_hash: String,
}

impl Flight {
    /// `scenario autopilot index N`, for sentences.
    pub fn name(&self) -> String {
        format!("{} {} index {}", self.scenario, self.autopilot, self.index)
    }

    fn hashes(&self) -> (&str, &str, &str) {
        (&self.trace_hash, &self.packet_hash, &self.reply_hash)
    }
}

/// The tables, read and checked.
#[derive(Clone, Debug)]
pub struct Gate {
    pub info: GateInfo,
    /// By scenario and autopilot, in index order.
    pub tables: BTreeMap<(String, String), Vec<Flight>>,
    /// The flights flown twice, by scenario, in index order.
    pub twice: BTreeMap<String, Vec<Flight>>,
    /// Why the plan is not the record's plan; empty for the gate itself.
    pub trial_reasons: Vec<String>,
}

impl Gate {
    fn flights(&self, scenario: &str, autopilot: &str) -> &[Flight] {
        self.tables
            .get(&(scenario.to_string(), autopilot.to_string()))
            .map_or(&[], Vec::as_slice)
    }
}

/// What a row's audit says, from its `audit.txt`.
#[derive(Debug, Default)]
struct Audit {
    scenario: Option<String>,
    index: Option<String>,
    seed: Option<String>,
    autopilot: Option<String>,
    class: Option<String>,
    commit: Option<String>,
    file_sha: Option<String>,
    binary_sha: Option<String>,
    params_sha: Option<String>,
    /// The packet and the reply hash of the bridge's audit (ArduCopter).
    packet: Option<String>,
    reply: Option<String>,
    /// The first word of the `repeats` line (`yes` or `no`) of a flight flown
    /// twice.
    repeats: Option<String>,
}

/// The value of the line `label  value` of an audit: the label, then at least
/// two spaces (the audit pads its labels), so that `scenario` is not the line
/// of `scenario file` and `seed` not that of `seed 7, step 1000 us`.
fn audit_field(text: &str, label: &str) -> Option<String> {
    text.lines().find_map(|l| {
        l.trim_start()
            .strip_prefix(label)
            .filter(|rest| rest.starts_with("  "))
            .map(|rest| rest.trim().to_string())
    })
}

/// The two hashes of the line `packet hash X, reply hash Y`.
fn audit_hashes(text: &str) -> (Option<String>, Option<String>) {
    for l in text.lines() {
        let l = l.trim();
        if let Some(rest) = l.strip_prefix("packet hash ")
            && let Some((p, r)) = rest.split_once(", reply hash ")
        {
            return (Some(p.trim().to_string()), Some(r.trim().to_string()));
        }
    }
    (None, None)
}

fn read_audit(dir: &Path) -> Result<Audit, String> {
    let path = dir.join("audit.txt");
    let text = std::fs::read_to_string(&path)
        .map_err(|e| format!("cannot read {}: {e}", path.display()))?;
    let (packet, reply) = audit_hashes(&text);
    Ok(Audit {
        scenario: audit_field(&text, "scenario"),
        index: audit_field(&text, "run index"),
        seed: audit_field(&text, "seed"),
        autopilot: audit_field(&text, "autopilot"),
        class: audit_field(&text, "class"),
        commit: audit_field(&text, "our commit"),
        file_sha: audit_field(&text, "file sha256"),
        binary_sha: audit_field(&text, "binary sha256"),
        params_sha: audit_field(&text, "params sha256"),
        packet,
        reply,
        repeats: audit_field(&text, "repeats")
            .and_then(|r| r.split_whitespace().next().map(String::from)),
    })
}

/// The row of `dir/row.csv`, the flight's own record of itself.
fn read_row_file(dir: &Path) -> Result<Row, String> {
    let path = dir.join("row.csv");
    let text = std::fs::read_to_string(&path)
        .map_err(|e| format!("cannot read {}: {e}", path.display()))?;
    let mut lines = text.lines();
    if lines.next() != Some(header().as_str()) {
        return Err(format!(
            "{} does not start with the table's header",
            path.display()
        ));
    }
    let line = lines
        .next()
        .ok_or_else(|| format!("{} holds no row", path.display()))?;
    Row::parse(line).map_err(|e| format!("{}: {e}", path.display()))
}

/// The columns of the table's row that differ from the flight's own
/// `row.csv`: all of them, except that a flight flown again after a failed
/// attempt has its `retried` and `note` set by the batch.
fn differing_columns(table: &Row, own: &Row) -> Vec<&'static str> {
    COLUMNS
        .iter()
        .copied()
        .filter(|c| !(table.get("retried") == "yes" && (*c == "retried" || *c == "note")))
        .filter(|c| table.get(c) != own.get(c))
        .collect()
}

/// A number of a cell: empty is `None`; a word or a non-finite number is an
/// error.
fn number(row: &Row, column: &str) -> Result<Option<f64>, String> {
    let text = row.get(column);
    if text.is_empty() {
        return Ok(None);
    }
    match text.parse::<f64>() {
        Ok(v) if v.is_finite() => Ok(Some(v)),
        _ => Err(format!("{column} is {text:?}, which is not a number")),
    }
}

/// Reads one table and checks its rows; problems are added to `problems`.
struct Reader<'a> {
    info: &'a GateInfo,
    problems: &'a mut Vec<String>,
    /// The parameter hashes seen by ArduCopter's tables.
    params: BTreeMap<(String, bool), BTreeSet<String>>,
    /// Why the flights are not the record's gate although the tables are
    /// consistent (flown with files that are not the commit's).
    trial: BTreeSet<String>,
}

impl Reader<'_> {
    fn table(
        &mut self,
        dir: &Path,
        entry: &Entry,
        count: u64,
        autopilot: &str,
        twice: bool,
    ) -> Vec<Flight> {
        let scenario = entry.scenario.as_str();
        let label = if twice {
            format!("the twice-flown table of {scenario}")
        } else {
            format!("the table of {scenario} flown by {autopilot}")
        };
        let path = dir.join(TABLE);
        let text = match std::fs::read_to_string(&path) {
            Ok(t) => t,
            Err(e) => {
                self.problems.push(format!(
                    "{label} is missing: cannot read {}: {e}",
                    path.display()
                ));
                return Vec::new();
            }
        };
        let mut lines = text.lines();
        if lines.next() != Some(header().as_str()) {
            self.problems.push(format!(
                "{label} ({}) does not start with the table's header",
                path.display()
            ));
            return Vec::new();
        }
        let mut seen: BTreeSet<u64> = BTreeSet::new();
        let mut out = Vec::new();
        for (n, line) in lines.enumerate() {
            let row = match Row::parse(line) {
                Ok(r) => r,
                Err(e) => {
                    self.problems.push(format!("{label}, line {}: {e}", n + 2));
                    continue;
                }
            };
            if let Some(f) = self.row(&row, dir, entry, count, autopilot, twice, &mut seen) {
                out.push(f);
            }
        }
        for index in 0..count {
            if !seen.contains(&index) {
                self.problems.push(format!(
                    "{label} has no row for index {index} ({scenario} {autopilot} index {index} \
                     was not flown or the gate stopped before it)"
                ));
            }
        }
        out.sort_by_key(|f| f.index);
        out
    }

    #[allow(clippy::too_many_arguments)]
    fn row(
        &mut self,
        row: &Row,
        dir: &Path,
        entry: &Entry,
        count: u64,
        autopilot: &str,
        twice: bool,
        seen: &mut BTreeSet<u64>,
    ) -> Option<Flight> {
        let scenario = entry.scenario.as_str();
        let Ok(index) = row.get("index").parse::<u64>() else {
            self.problems.push(format!(
                "a row of {scenario} {autopilot} has the index {:?}, which is not a whole number",
                row.get("index")
            ));
            return None;
        };
        let name = format!("{scenario} {autopilot} index {index}");
        let twice_mark = if twice { " (flown twice)" } else { "" };
        let name = format!("{name}{twice_mark}");
        if row.get("scenario") != scenario || row.get("autopilot") != autopilot {
            self.problems.push(format!(
                "the row {name} says it is {} flown by {}",
                row.get("scenario"),
                row.get("autopilot")
            ));
            return None;
        }
        if index >= count {
            self.problems.push(format!(
                "the row {name} is not in the plan, which has the indices 0 to {}",
                count.saturating_sub(1)
            ));
            return None;
        }
        if !seen.insert(index) {
            self.problems
                .push(format!("the row {name} exists twice in its table"));
            return None;
        }
        if row.get("started") != "yes" {
            self.problems.push(format!(
                "the gate stopped at {name}: it did not start ({})",
                row.get("note")
            ));
            return None;
        }
        let mut whys: Vec<String> = Vec::new();
        if row.get("note").contains(DIRTY_NOTE) {
            whys.push("it was flown on a dirty tree (its note says so)".into());
        }
        // Every seed is the seed of its scenario and index, so the rows of one
        // index have the same seed whoever flew them.
        match row.get("seed").parse::<u64>() {
            Ok(seed) => {
                let want = run_seed(scenario, index);
                if seed != want {
                    whys.push(format!(
                        "its seed is {seed} and the seed of {scenario} index {index} is {want}"
                    ));
                }
            }
            Err(_) => whys.push(format!("its seed is {:?}", row.get("seed"))),
        }
        let class = Class::parse(row.get("class"));
        if class.is_none() {
            whys.push(format!("its class is {:?}", row.get("class")));
        }
        let judged = Judged::parse(row.get("verdict"));
        if judged.is_none() {
            whys.push(format!("its verdict is {:?}", row.get("verdict")));
        }
        let mut num = |column: &str, required: bool| -> Option<f64> {
            match number(row, column) {
                Ok(Some(v)) if v < 0.0 => {
                    whys.push(format!("{column} is {v}, which is negative"));
                    None
                }
                Ok(Some(v)) => Some(v),
                Ok(None) => {
                    if required {
                        whys.push(format!("it has no {column}"));
                    }
                    None
                }
                Err(e) => {
                    whys.push(e);
                    None
                }
            }
        };
        // The statistics read the rms_h_m of every core flight and of
        // ArduCopter's safe flights; an unsafe flight of ArduCopter's is only
        // counted (ADR-0007 section 5), so it need not have one.
        let enters_statistics = autopilot != "arducopter" || class == Some(Class::Safe);
        let rms_h_m = num(
            "rms_h_m",
            entry.scoring == Scoring::Gust && enters_statistics,
        );
        let rms_v_m = num("rms_v_m", false);
        let max_tilt_deg = num("max_tilt_deg", false);
        let last_motor_on_s = num("last_motor_on_s", entry.scoring == Scoring::MotorsOff);
        let audit_dir = run_dir(dir, index);
        match read_row_file(&audit_dir) {
            Err(e) => whys.push(e),
            Ok(own) => {
                let differ = differing_columns(row, &own);
                if !differ.is_empty() {
                    whys.push(format!(
                        "its row of the table is not the row of its own run directory: {} differ",
                        differ.join(", ")
                    ));
                }
            }
        }
        match read_audit(&audit_dir) {
            Err(e) => whys.push(e),
            Ok(a) => {
                audit_problems(
                    self.info,
                    &mut self.params,
                    autopilot,
                    scenario,
                    twice,
                    &a,
                    &mut whys,
                );
                audit_row_problems(row, &a, &mut whys);
                if autopilot == "arducopter" {
                    self.expected_params(scenario, &a);
                }
            }
        }
        if !whys.is_empty() {
            for why in whys {
                self.problems.push(format!("the row {name}: {why}"));
            }
            return None;
        }
        Some(Flight {
            scenario: scenario.to_string(),
            autopilot: autopilot.to_string(),
            index,
            class: class?,
            judged: judged?,
            rms_h_m,
            rms_v_m,
            max_tilt_deg,
            last_motor_on_s,
            repeats: match row.get("repeats") {
                "yes" => Some(true),
                "no" => Some(false),
                _ => None,
            },
            trace_hash: row.get("trace_hash").to_string(),
            packet_hash: row.get("packet_hash").to_string(),
            reply_hash: row.get("reply_hash").to_string(),
        })
    }
}

#[allow(clippy::too_many_arguments)]
fn audit_problems(
    info: &GateInfo,
    params: &mut BTreeMap<(String, bool), BTreeSet<String>>,
    autopilot: &str,
    scenario: &str,
    twice: bool,
    a: &Audit,
    whys: &mut Vec<String>,
) {
    match a.commit.as_deref() {
        None => whys.push("its audit names no commit".into()),
        Some(c) if c.ends_with("-dirty") => {
            whys.push(format!("it was flown on a dirty tree (its audit says {c})"));
        }
        Some(c) if c != info.commit => whys.push(format!(
            "it was flown at commit {c} and the gate's commit is {}",
            info.commit
        )),
        Some(_) => {}
    }
    match (a.file_sha.as_deref(), info.scenario_sha(scenario)) {
        (Some(f), Some(g)) if f == g => {}
        (Some(f), Some(g)) => whys.push(format!(
            "its scenario file has SHA-256 {f} and gate.txt says {g}"
        )),
        (None, _) => whys.push("its audit has no scenario file hash".into()),
        (_, None) => whys.push("gate.txt has no hash of its scenario file".into()),
    }
    if autopilot == "arducopter" {
        match (a.binary_sha.as_deref(), info.binary_sha256.as_deref()) {
            (Some(b), Some(g)) if b == g => {}
            (Some(b), Some(g)) => whys.push(format!(
                "it was flown with the binary {b} and gate.txt says {g}"
            )),
            (None, _) => whys.push("its audit has no binary checksum".into()),
            (_, None) => whys.push("gate.txt has no binary checksum".into()),
        }
        match a.params_sha.as_deref() {
            Some(p) => {
                params
                    .entry((scenario.to_string(), twice))
                    .or_default()
                    .insert(p.to_string());
            }
            None => whys.push("its audit has no parameter hash".into()),
        }
    }
}

/// What the audit says against what the row says: the same flight.
fn audit_row_problems(row: &Row, a: &Audit, whys: &mut Vec<String>) {
    let mut same = |what: &str, audit: &Option<String>, cell: &str| match audit {
        Some(v) if v == cell => {}
        Some(v) => whys.push(format!(
            "its audit says {what} {v:?} and its row says {cell:?}"
        )),
        None => whys.push(format!("its audit has no {what}")),
    };
    same("scenario", &a.scenario, row.get("scenario"));
    same("run index", &a.index, row.get("index"));
    same("seed", &a.seed, row.get("seed"));
    same("autopilot", &a.autopilot, row.get("autopilot"));
    same("class", &a.class, row.get("class"));
    if row.get("autopilot") == "arducopter" {
        same("packet hash", &a.packet, row.get("packet_hash"));
        same("reply hash", &a.reply, row.get("reply_hash"));
    }
}

impl Reader<'_> {
    /// Notes a flight of ArduCopter whose parameters are not the effective
    /// file of the commit's parameter files for its scenario.
    fn expected_params(&mut self, scenario: &str, a: &Audit) {
        let Some(got) = a.params_sha.as_deref() else {
            return;
        };
        match files::expected_params_sha256(scenario) {
            Ok(want) if want == got => {}
            Ok(want) => {
                self.trial.insert(format!(
                    "{scenario} was flown with the parameter hash {got} and the commit's \
                     parameter files give {want}"
                ));
            }
            Err(e) => {
                self.trial.insert(format!(
                    "the parameters of {scenario} cannot be computed from the commit's files: {e}"
                ));
            }
        }
    }
}

/// Why the tables cannot give a verdict: one sentence for each problem.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Incomplete(pub Vec<String>);

/// The directories of `out` that the plan does not name: a scenario that is
/// not in it, an autopilot that does not fly a scenario, a flown-twice
/// scenario that is not in the plan. (A directory a continued gate moved
/// aside, `NAME.interrupted`, is its own leftover and is let be.)
fn stray_directories(out: &Path, info: &GateInfo) -> Vec<String> {
    fn dirs(path: &Path) -> Vec<String> {
        let mut v: Vec<String> = std::fs::read_dir(path)
            .into_iter()
            .flatten()
            .filter_map(Result::ok)
            .filter(|e| e.path().is_dir())
            .map(|e| e.file_name().to_string_lossy().into_owned())
            .filter(|n| !n.ends_with(".interrupted"))
            .collect();
        v.sort();
        v
    }
    let mut stray = Vec::new();
    for name in dirs(out) {
        let dir = out.join(&name);
        if name == TWICE_DIR {
            for s in dirs(&dir) {
                if !info.plan.twice.iter().any(|(n, _)| *n == s) {
                    stray.push(format!(
                        "the directory {TWICE_DIR}/{s} holds flights flown twice that the plan does not name"
                    ));
                }
            }
        } else if let Some(entry) = info.plan.entries.iter().find(|e| e.scenario == name) {
            let known: Vec<String> = entry
                .scoring
                .autopilots()
                .iter()
                .map(|a| a.name())
                .collect();
            for a in dirs(&dir) {
                if !known.contains(&a) {
                    stray.push(format!(
                        "the directory {name}/{a} holds a table of an autopilot that does not fly {name} in the plan"
                    ));
                }
            }
        } else {
            stray.push(format!(
                "the directory {name} holds tables of a scenario that is not in the plan"
            ));
        }
    }
    stray
}

/// The flights flown twice against their evidence and against the main
/// table (ADR-0007 section 7: "both of each pair must agree with each other
/// and with the same flight in the tables"). A flight that does not repeat
/// gives no verdict: "the result waits until the cause is known".
fn twice_problems(
    out: &Path,
    info: &GateInfo,
    tables: &BTreeMap<(String, String), Vec<Flight>>,
    twice: &BTreeMap<String, Vec<Flight>>,
) -> Vec<String> {
    let mut problems = Vec::new();
    for (scenario, _) in &info.plan.twice {
        let main = tables.get(&(scenario.clone(), "arducopter".to_string()));
        for f in twice.get(scenario).map_or(&[][..], Vec::as_slice) {
            let name = format!("the flight {} flown twice", f.name());
            let run = run_dir(&twice_dir(out, scenario), f.index);
            let mut broken = Vec::new();
            match read_audit(&run) {
                Ok(a) if a.repeats.as_deref() == Some("yes") => {}
                Ok(a) => broken.push(format!(
                    "its audit says repeats {:?}",
                    a.repeats.unwrap_or_default()
                )),
                Err(e) => broken.push(e),
            }
            match (
                read_audit(&run.join("second")),
                read_row_file(&run.join("second")),
            ) {
                (Ok(a), Ok(own)) => {
                    if a.packet.as_deref() != Some(f.packet_hash.as_str())
                        || a.reply.as_deref() != Some(f.reply_hash.as_str())
                        || own.get("trace_hash") != f.trace_hash
                    {
                        broken.push(
                            "its second flight's trace, packet and reply hashes are not those of \
                             its first flight"
                                .to_string(),
                        );
                    }
                }
                (Err(e), _) | (_, Err(e)) => {
                    broken.push(format!("the record of its second flight is missing: {e}"));
                }
            }
            let same_as_main = main
                .and_then(|m| m.iter().find(|m| m.index == f.index))
                .is_some_and(|m| m.hashes() == f.hashes())
                && !f.trace_hash.is_empty()
                && !f.packet_hash.is_empty()
                && !f.reply_hash.is_empty();
            if !same_as_main {
                broken.push(
                    "its trace, packet and reply hashes are not those of the same flight in the \
                     main table"
                        .to_string(),
                );
            }
            if f.repeats != Some(true) {
                broken.push(format!(
                    "its repeats cell is {}",
                    match f.repeats {
                        Some(false) => "no",
                        _ => "empty",
                    }
                ));
            }
            if !broken.is_empty() {
                problems.push(format!(
                    "{name} does not repeat bit for bit or cannot be shown to ({}): the result \
                     waits until the cause is known (ADR-0007 section 7)",
                    broken.join("; ")
                ));
            }
        }
    }
    problems
}

/// The commit this program was built at, when that is a commit: not
/// `unknown` (no repository) and not `-dirty` (a build on a tree with
/// uncommitted changes, which is no commit's program).
pub fn own_commit() -> Option<&'static str> {
    let h = env!("OXIDE_GIT_HASH");
    (h != "unknown" && !h.ends_with("-dirty")).then_some(h)
}

/// Reads `out/gate.txt` and every table of its plan and checks them, for the
/// program's own commit ([`own_commit`]).
pub fn read(out: &Path) -> Result<Gate, Incomplete> {
    read_at(out, own_commit())
}

/// [`read`] for a program built at `own` (`None`: not at a commit, which
/// cannot be compared). Tables flown at another commit than the program's own
/// give no verdict (ADR-0007 section 7).
pub fn read_at(out: &Path, own: Option<&str>) -> Result<Gate, Incomplete> {
    let path = out.join("gate.txt");
    let text = std::fs::read_to_string(&path).map_err(|e| {
        Incomplete(vec![format!(
            "cannot read {}: {e}: is {} a gate directory",
            path.display(),
            out.display()
        )])
    })?;
    let info = GateInfo::parse(&text).map_err(|e| Incomplete(vec![e]))?;
    let mut problems = Vec::new();
    if let Some(own) = own
        && info.commit != own
    {
        problems.push(format!(
            "gate.txt says the gate was flown at commit {} and this program was built at {own}: \
             a verdict is given on tables of the program's own commit only (ADR-0007 section 7)",
            info.commit
        ));
    }
    if info.tree_dirty {
        problems.push(format!(
            "gate.txt says {DIRTY_NOTE}: the gate is flown on a commit (ADR-0007 section 1)"
        ));
    }
    if let Some(why) = &info.stopped {
        problems.push(format!("the gate stopped: {why}"));
    }
    problems.extend(info.plan.inconsistencies());
    let mut trial_reasons = info.plan.deviations_from_the_record();
    trial_reasons.extend(files::differences_from_build(
        info.binary_sha256.as_deref(),
        &info.params,
        &info.scenario_files,
        info.plan.entries.iter().map(|e| e.scenario.as_str()),
    ));
    let mut reader = Reader {
        info: &info,
        problems: &mut problems,
        params: BTreeMap::new(),
        trial: BTreeSet::new(),
    };
    let mut tables = BTreeMap::new();
    for entry in &info.plan.entries {
        for autopilot in entry.scoring.autopilots() {
            let name = autopilot.name();
            let dir = table_dir(out, &entry.scenario, &name);
            let flights = reader.table(&dir, entry, entry.count, &name, false);
            tables.insert((entry.scenario.clone(), name), flights);
        }
    }
    let mut twice = BTreeMap::new();
    for (scenario, count) in &info.plan.twice {
        let Some(entry) = info.plan.entries.iter().find(|e| &e.scenario == scenario) else {
            continue;
        };
        let dir = twice_dir(out, scenario);
        let flights = reader.table(&dir, entry, *count, "arducopter", true);
        twice.insert(scenario.clone(), flights);
    }
    let mixed: Vec<String> = reader
        .params
        .iter()
        .filter(|(_, set)| set.len() > 1)
        .map(|((s, twice), set)| {
            format!(
                "the {}table of {s} mixes parameter hashes: {}",
                if *twice { "twice-flown " } else { "" },
                set.iter().cloned().collect::<Vec<_>>().join(" ")
            )
        })
        .collect();
    let scenarios: BTreeSet<&String> = reader.params.keys().map(|(s, _)| s).collect();
    let across: Vec<String> = scenarios
        .into_iter()
        .filter_map(|s| {
            let main = reader.params.get(&(s.clone(), false))?;
            let again = reader.params.get(&(s.clone(), true))?;
            (main.len() == 1 && again.len() == 1 && main != again).then(|| {
                format!("the flights of {s} flown twice have other parameter hashes than its table")
            })
        })
        .collect();
    trial_reasons.extend(std::mem::take(&mut reader.trial));
    problems.extend(mixed);
    problems.extend(across);
    problems.extend(twice_problems(out, &info, &tables, &twice));
    problems.extend(stray_directories(out, &info));
    if info.reduced && trial_reasons.is_empty() {
        trial_reasons.push("gate.txt says the plan is a reduced one".into());
    } else if !info.reduced && !trial_reasons.is_empty() {
        problems.push(format!(
            "gate.txt says the plan is the gate's and it is not: {}",
            trial_reasons.join("; ")
        ));
    }
    if !problems.is_empty() {
        return Err(Incomplete(problems));
    }
    Ok(Gate {
        info,
        tables,
        twice,
        trial_reasons,
    })
}

/// Whether a criterion holds.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Status {
    Holds,
    Fails,
    /// The plan has nothing to judge it on (a trial plan only).
    NotInPlan,
}

impl Status {
    pub fn word(self) -> &'static str {
        match self {
            Self::Holds => "HOLDS",
            Self::Fails => "FAILS",
            Self::NotInPlan => "NOT IN THE PLAN",
        }
    }
}

/// One criterion: its status, its numbers, the rows that fail it.
#[derive(Clone, Debug, PartialEq)]
pub struct Check {
    pub status: Status,
    pub lines: Vec<String>,
    pub failing: Vec<String>,
}

impl Check {
    fn new(status: Status) -> Self {
        Self {
            status,
            lines: Vec::new(),
            failing: Vec::new(),
        }
    }

    fn finish(mut self, any_judged: bool) -> Self {
        self.status = if !any_judged {
            Status::NotInPlan
        } else if self.failing.is_empty() && self.status != Status::Fails {
            Status::Holds
        } else {
            Status::Fails
        };
        self
    }
}

/// The statistics of one autopilot's gust flights.
#[derive(Clone, Debug, PartialEq)]
pub struct GustStats {
    pub autopilot: String,
    /// Flights in the table.
    pub flights: usize,
    /// Flights that are not safe.
    pub unsafe_flights: usize,
    /// Flights in the statistics: ArduCopter's safe ones, all of the others.
    pub counted: usize,
    pub median: Option<f64>,
    pub p95: Option<f64>,
    pub mean: Option<f64>,
    pub worst: Option<f64>,
    pub v_median: Option<f64>,
    pub v_worst: Option<f64>,
    pub tilt_median: Option<f64>,
    pub tilt_worst: Option<f64>,
}

/// In how many indices each had the lower `rms_h_m`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct Wins {
    pub ours: usize,
    pub arducopter: usize,
    pub equal: usize,
}

/// The classes of the flights of one autopilot in one scenario.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct Classes {
    pub safe: usize,
    pub crash: usize,
    pub fly_away: usize,
    /// Flights whose verdict column says pass.
    pub passed: usize,
    pub flights: usize,
}

impl Classes {
    pub fn bad(&self) -> usize {
        self.crash + self.fly_away
    }

    fn of(flights: &[Flight]) -> Self {
        let mut c = Self::default();
        for f in flights {
            c.flights += 1;
            match f.class {
                Class::Safe => c.safe += 1,
                Class::Crash => c.crash += 1,
                Class::FlyAway => c.fly_away += 1,
            }
            if f.judged == Judged::Pass {
                c.passed += 1;
            }
        }
        c
    }
}

/// One scenario of criterion B, for the report.
#[derive(Clone, Debug, PartialEq)]
pub struct ScenarioSummary {
    pub scenario: String,
    pub scoring: Scoring,
    pub arducopter: Classes,
    pub core_f405: Classes,
    pub core_h743: Classes,
}

/// The time to motors off of one scenario (criterion B3), s.
#[derive(Clone, Debug, PartialEq)]
pub struct MotorsSummary {
    pub scenario: String,
    pub failure_start_s: f64,
    pub arducopter: Vec<f64>,
    pub core_f405: Vec<f64>,
    pub core_h743: Vec<f64>,
}

/// One flight flown twice, for the report.
#[derive(Clone, Debug, PartialEq)]
pub struct TwiceSummary {
    pub scenario: String,
    pub index: u64,
    pub repeats: Option<bool>,
    pub hashes_equal: bool,
}

/// What the tables say.
#[derive(Clone, Debug)]
pub struct Evaluation {
    pub info: GateInfo,
    /// Why this is not the gate's plan; the verdict is never a pass then.
    pub trial_reasons: Vec<String>,
    pub a1: Check,
    pub a2: Check,
    pub b1: Check,
    pub b2: Check,
    pub b3: Check,
    pub repeats: Check,
    pub passed: bool,
    pub gust: Vec<GustStats>,
    pub wins: Wins,
    pub scenarios: Vec<ScenarioSummary>,
    pub motors: Vec<MotorsSummary>,
    pub twice: Vec<TwiceSummary>,
    /// ArduCopter's and our F405 core's `rms_h_m` of each gust index where
    /// ArduCopter's flight is safe, for the plots.
    pub paired: Vec<(f64, f64)>,
    /// `rms_h_m` of every gust flight that is in the statistics (all but
    /// ArduCopter's unsafe flights), by autopilot, in index order.
    pub curves: Vec<(String, Vec<f64>)>,
}

impl Evaluation {
    pub fn exit_code(&self) -> u8 {
        if self.passed {
            EXIT_PASSED
        } else {
            EXIT_NOT_PASSED
        }
    }
}

/// Ours is at most theirs: compared as computed, without rounding (ADR-0007
/// section 5). Both are finite: [`read`] refuses a cell that is not a number.
fn at_most(ours: f64, theirs: f64) -> bool {
    ours <= theirs
}

fn rms(flights: &[Flight]) -> Vec<f64> {
    flights.iter().filter_map(|f| f.rms_h_m).collect()
}

fn gust_stats(autopilot: &str, flights: &[Flight]) -> GustStats {
    let counted: Vec<&Flight> = flights
        .iter()
        .filter(|f| autopilot != "arducopter" || f.class == Class::Safe)
        .collect();
    let pick = |g: fn(&Flight) -> Option<f64>| -> Vec<f64> {
        counted.iter().filter_map(|f| g(f)).collect()
    };
    let h = pick(|f| f.rms_h_m);
    let v = pick(|f| f.rms_v_m);
    let t = pick(|f| f.max_tilt_deg);
    let worst = |x: &[f64]| {
        x.iter()
            .copied()
            .fold(None, |a: Option<f64>, b| Some(a.map_or(b, |a| a.max(b))))
    };
    GustStats {
        autopilot: autopilot.to_string(),
        flights: flights.len(),
        unsafe_flights: flights.iter().filter(|f| f.class != Class::Safe).count(),
        counted: counted.len(),
        median: median(&h),
        p95: percentile_95(&h),
        mean: mean(&h),
        worst: worst(&h),
        v_median: median(&v),
        v_worst: worst(&v),
        tilt_median: median(&t),
        tilt_worst: worst(&t),
    }
}

/// Criterion A1 for one core's flights.
fn a1_of(check: &mut Check, who: &str, flights: &[Flight]) {
    let good = flights
        .iter()
        .filter(|f| f.class == Class::Safe && f.judged == Judged::Pass)
        .count();
    check.lines.push(format!(
        "{who}: {good} of {} flights are safe and pass the scenario file's limits",
        flights.len()
    ));
    for f in flights {
        if f.class != Class::Safe || f.judged != Judged::Pass {
            check.failing.push(format!(
                "{}: class {}, verdict {}",
                f.name(),
                f.class.name(),
                f.judged.name()
            ));
        }
    }
}

/// Computes every criterion from the flights of `gate`.
pub fn evaluate(gate: &Gate) -> Evaluation {
    let info = &gate.info;
    let ac = "arducopter";
    let f405 = "core-f405";
    let h743 = "core-h743";
    let truth = "core-f405-truth";
    let gust_entry = info
        .plan
        .entries
        .iter()
        .find(|e| e.scoring == Scoring::Gust);
    let mut a1 = Check::new(Status::Holds);
    let mut a2 = Check::new(Status::Holds);
    let mut gust = Vec::new();
    let mut wins = Wins::default();
    let mut paired = Vec::new();
    let mut curves = Vec::new();
    if gust_entry.is_some() {
        a1_of(&mut a1, f405, gate.flights(GUST, f405));
        a1_of(&mut a1, h743, gate.flights(GUST, h743));
        for who in [ac, f405, h743, truth] {
            let flights = gate.flights(GUST, who);
            gust.push(gust_stats(who, flights));
            // ArduCopter's unsafe flights are left out of its statistics
            // (ADR-0007 section 5) and so out of its curve.
            let drawn: Vec<Flight> = flights
                .iter()
                .filter(|f| who != ac || f.class == Class::Safe)
                .cloned()
                .collect();
            curves.push((who.to_string(), rms(&drawn)));
        }
        let ours = rms(gate.flights(GUST, f405));
        let theirs: Vec<f64> = gate
            .flights(GUST, ac)
            .iter()
            .filter(|f| f.class == Class::Safe)
            .filter_map(|f| f.rms_h_m)
            .collect();
        let unsafe_ac = gate.flights(GUST, ac).len() - theirs.len();
        a2.lines.push(format!(
            "ArduCopter: {unsafe_ac} of {} flights are not safe and are left out of its statistics",
            gate.flights(GUST, ac).len()
        ));
        match (
            median(&ours),
            percentile_95(&ours),
            median(&theirs),
            percentile_95(&theirs),
        ) {
            (Some(om), Some(op), Some(tm), Some(tp)) => {
                a2.lines.push(format!(
                    "median: core-f405 {om} m, ArduCopter {tm} m ({})",
                    if om <= tm {
                        "ours at most theirs"
                    } else {
                        "ours above theirs"
                    }
                ));
                a2.lines.push(format!(
                    "95th percentile: core-f405 {op} m, ArduCopter {tp} m ({})",
                    if op <= tp {
                        "ours at most theirs"
                    } else {
                        "ours above theirs"
                    }
                ));
                if !at_most(om, tm) {
                    a2.failing.push(format!(
                        "the median of core-f405 {om} is above ArduCopter's {tm}"
                    ));
                }
                if !at_most(op, tp) {
                    a2.failing.push(format!(
                        "the 95th percentile of core-f405 {op} is above ArduCopter's {tp}"
                    ));
                }
            }
            _ => a2.failing.push(
                "ArduCopter has no safe gust flight, or ours has none: the statistics cannot be \
                 compared"
                    .into(),
            ),
        }
        for (a, b) in gate.flights(GUST, f405).iter().zip(gate.flights(GUST, ac)) {
            if let (Some(x), Some(y)) = (a.rms_h_m, b.rms_h_m) {
                if b.class == Class::Safe {
                    paired.push((y, x));
                }
                if x < y {
                    wins.ours += 1;
                } else if y < x {
                    wins.arducopter += 1;
                } else {
                    wins.equal += 1;
                }
            }
        }
    }
    let has_gust = gust_entry.is_some();
    let a1 = a1.finish(has_gust);
    let a2 = a2.finish(has_gust);

    // Criterion B.
    let mut b1 = Check::new(Status::Holds);
    let mut b2 = Check::new(Status::Holds);
    let mut b3 = Check::new(Status::Holds);
    let mut scenarios = Vec::new();
    let mut motors = Vec::new();
    let (mut any_outcome, mut any_motors) = (false, false);
    for e in &info.plan.entries {
        let s = e.scenario.as_str();
        match e.scoring {
            Scoring::Gust => continue,
            Scoring::NotScored => {}
            Scoring::Outcome => {
                any_outcome = true;
                let ours = Classes::of(gate.flights(s, f405));
                let theirs = Classes::of(gate.flights(s, ac));
                b1.lines.push(format!(
                    "{s}: core-f405 {} crash or fly-away, ArduCopter {}",
                    ours.bad(),
                    theirs.bad()
                ));
                if ours.bad() > theirs.bad() {
                    b1.failing.push(format!(
                        "{s}: core-f405 has {} flights in a crash or fly-away and ArduCopter {}",
                        ours.bad(),
                        theirs.bad()
                    ));
                }
                for f in gate.flights(s, f405) {
                    if f.class != Class::Safe || f.judged != Judged::Pass {
                        b2.failing.push(format!(
                            "{}: class {}, verdict {}",
                            f.name(),
                            f.class.name(),
                            f.judged.name()
                        ));
                    }
                }
                b2.lines.push(format!(
                    "{s}: core-f405 {} of {} flights safe and pass",
                    gate.flights(s, f405)
                        .iter()
                        .filter(|f| f.class == Class::Safe && f.judged == Judged::Pass)
                        .count(),
                    ours.flights
                ));
            }
            Scoring::MotorsOff => {
                any_motors = true;
                let start = info.plan.failure_starts.get(s).copied().unwrap_or(0.0);
                let delta = |who: &str| -> Vec<f64> {
                    gate.flights(s, who)
                        .iter()
                        .filter_map(|f| f.last_motor_on_s)
                        .map(|t| t - start)
                        .collect()
                };
                let m = MotorsSummary {
                    scenario: s.to_string(),
                    failure_start_s: start,
                    arducopter: delta(ac),
                    core_f405: delta(f405),
                    core_h743: delta(h743),
                };
                match (median(&m.core_f405), median(&m.arducopter)) {
                    (Some(o), Some(t)) => {
                        b3.lines.push(format!(
                            "{s}: median time from the failure to motors off, core-f405 {o} s, \
                             ArduCopter {t} s"
                        ));
                        if !at_most(o, t) {
                            b3.failing.push(format!(
                                "{s}: the median of core-f405 {o} s is above ArduCopter's {t} s"
                            ));
                        }
                    }
                    _ => b3.failing.push(format!("{s}: a median cannot be computed")),
                }
                if let Some(h) = median(&m.core_h743) {
                    b3.lines
                        .push(format!("{s}: core-h743 (reported, decides nothing) {h} s"));
                }
                motors.push(m);
            }
        }
        scenarios.push(ScenarioSummary {
            scenario: s.to_string(),
            scoring: e.scoring,
            arducopter: Classes::of(gate.flights(s, ac)),
            core_f405: Classes::of(gate.flights(s, f405)),
            core_h743: Classes::of(gate.flights(s, h743)),
        });
    }
    let b1 = b1.finish(any_outcome);
    let b2 = b2.finish(any_outcome);
    let b3 = b3.finish(any_motors);

    // Repeats.
    let mut repeats = Check::new(Status::Holds);
    let mut twice = Vec::new();
    let mut any_twice = false;
    for (scenario, _) in &info.plan.twice {
        for f in gate.twice.get(scenario).map_or(&[][..], Vec::as_slice) {
            any_twice = true;
            let main = gate
                .flights(scenario, ac)
                .iter()
                .find(|m| m.index == f.index);
            let equal = main.is_some_and(|m| m.hashes() == f.hashes())
                && !f.trace_hash.is_empty()
                && !f.packet_hash.is_empty()
                && !f.reply_hash.is_empty();
            if f.repeats != Some(true) {
                repeats.failing.push(format!(
                    "{}: repeats is {}",
                    f.name(),
                    match f.repeats {
                        Some(true) => "yes",
                        Some(false) => "no",
                        None => "empty",
                    }
                ));
            }
            if !equal {
                repeats.failing.push(format!(
                    "{}: its trace, packet and reply hashes are not those of the same flight in \
                     the main table",
                    f.name()
                ));
            }
            twice.push(TwiceSummary {
                scenario: scenario.clone(),
                index: f.index,
                repeats: f.repeats,
                hashes_equal: equal,
            });
        }
    }
    repeats.lines.push(format!(
        "{} flights flown twice, {} repeat bit for bit and match the main table",
        twice.len(),
        twice
            .iter()
            .filter(|t| t.repeats == Some(true) && t.hashes_equal)
            .count()
    ));
    let repeats = repeats.finish(any_twice);

    let all_hold = [&a1, &a2, &b1, &b2, &b3, &repeats]
        .iter()
        .all(|c| c.status == Status::Holds);
    Evaluation {
        info: info.clone(),
        trial_reasons: gate.trial_reasons.clone(),
        passed: all_hold && gate.trial_reasons.is_empty(),
        a1,
        a2,
        b1,
        b2,
        b3,
        repeats,
        gust,
        wins,
        scenarios,
        motors,
        twice,
        paired,
        curves,
    }
}

/// What `gate verdict` found.
#[derive(Clone, Debug)]
pub enum Verdict {
    Incomplete(Incomplete),
    Evaluated(Box<Evaluation>),
}

/// Reads the tables of `out` and computes the verdict.
pub fn verdict(out: &Path) -> Verdict {
    verdict_at(out, own_commit())
}

/// [`verdict`] for a program built at `own` (see [`read_at`]).
pub fn verdict_at(out: &Path, own: Option<&str>) -> Verdict {
    match read_at(out, own) {
        Err(e) => Verdict::Incomplete(e),
        Ok(g) => Verdict::Evaluated(Box::new(evaluate(&g))),
    }
}

impl Verdict {
    pub fn exit_code(&self) -> u8 {
        match self {
            Self::Incomplete(_) => EXIT_INCOMPLETE,
            Self::Evaluated(e) => e.exit_code(),
        }
    }

    /// The verdict as printed: criterion by criterion with its numbers, the
    /// failing rows named.
    pub fn text(&self) -> String {
        match self {
            Self::Incomplete(Incomplete(problems)) => {
                let mut s = String::from(
                    "gate 1: the tables are incomplete or inconsistent, so there is no verdict\n",
                );
                for p in problems.iter().take(MAX_LISTED) {
                    let _ = writeln!(s, "  - {p}");
                }
                if problems.len() > MAX_LISTED {
                    let _ = writeln!(s, "  ... and {} more", problems.len() - MAX_LISTED);
                }
                s
            }
            Self::Evaluated(e) => e.text(),
        }
    }
}

/// The most rows or problems a text names before it says how many more.
pub const MAX_LISTED: usize = 20;

impl Evaluation {
    fn check_text(&self, s: &mut String, name: &str, c: &Check) {
        let _ = writeln!(s, "{name}  {}", c.status.word());
        for l in &c.lines {
            let _ = writeln!(s, "    {l}");
        }
        for f in c.failing.iter().take(MAX_LISTED) {
            let _ = writeln!(s, "    fails: {f}");
        }
        if c.failing.len() > MAX_LISTED {
            let _ = writeln!(
                s,
                "    fails: ... and {} more",
                c.failing.len() - MAX_LISTED
            );
        }
    }

    /// The verdict as printed.
    pub fn text(&self) -> String {
        let mut s = String::new();
        let _ = writeln!(
            s,
            "gate 1: {}",
            if self.passed { "PASSED" } else { "NOT PASSED" }
        );
        let _ = writeln!(s, "  commit {}", self.info.commit);
        if !self.trial_reasons.is_empty() {
            let _ = writeln!(
                s,
                "  this is not the gate's plan, so it is not passed whatever the numbers say:"
            );
            for r in &self.trial_reasons {
                let _ = writeln!(s, "    - {r}");
            }
        }
        self.check_text(&mut s, "A1", &self.a1);
        self.check_text(&mut s, "A2", &self.a2);
        for g in &self.gust {
            let f = |x: Option<f64>| x.map_or("-".to_string(), |v| format!("{v:.4}"));
            let _ = writeln!(
                s,
                "      {}: median {} p95 {} mean {} worst {} ({} flights, {} not safe)",
                g.autopilot,
                f(g.median),
                f(g.p95),
                f(g.mean),
                f(g.worst),
                g.flights,
                g.unsafe_flights
            );
        }
        let _ = writeln!(
            s,
            "      lower rms_h_m: core-f405 in {} indices, ArduCopter in {}, equal in {}",
            self.wins.ours, self.wins.arducopter, self.wins.equal
        );
        self.check_text(&mut s, "B1", &self.b1);
        self.check_text(&mut s, "B2", &self.b2);
        self.check_text(&mut s, "B3", &self.b3);
        self.check_text(&mut s, "Repeats", &self.repeats);
        s
    }
}

/// The columns of the table, for the fixtures of the tests.
pub fn column_count() -> usize {
    COLUMNS.len()
}
