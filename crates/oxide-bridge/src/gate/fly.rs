// SPDX-License-Identifier: AGPL-3.0-or-later
//! `oxide-bridge gate fly`: flies the plan of gate 1 with the batch machinery
//! (ADR-0007 section 1 and 7) and writes `gate.txt`.
//!
//! Our core's flights are flown first (they take seconds, so a mistake in the
//! plan shows early), then ArduCopter's, then the flights flown twice. One
//! directory of the output holds a scenario and an autopilot: `OUT/SCENARIO/
//! AUTOPILOT/` has its `flights.csv` and `run-NNNN` directories; the flights
//! flown twice are in `OUT/twice/SCENARIO/`.
//!
//! The gate is flown on a commit: a working tree with uncommitted changes to
//! tracked files is refused unless `--allow-dirty`, which marks every table and
//! `gate.txt` as a dirty tree (the verdict then refuses them).

use std::path::{Path, PathBuf};
use std::process::{Command, ExitCode};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use crate::batch::{self, BatchConfig, Event, Job, Pin};
use crate::cli::{self, BINARY_VARIABLE, BatchArgs, Common, DEFAULT_LIMIT_S};
use crate::gate::files;
use crate::gate::info::{DIRTY_NOTE, GateInfo, utc_text};
use crate::gate::plan::Plan;
use crate::gate::verdict::{TABLE, table_dir, twice_dir};
use crate::record::{Row, header};
use crate::runner::{Autopilot, committed_params};
use crate::scenario::LoadedScenario;
use crate::sha256;

/// Exit code of a gate that stopped: a run failed twice (ADR-0007 section 4).
pub const EXIT_STOPPED: u8 = 3;

/// What `gate fly` is asked.
#[derive(Clone, Debug, PartialEq)]
pub struct Settings {
    pub scenarios: PathBuf,
    pub out: PathBuf,
    pub binary: Option<PathBuf>,
    /// Children at a time; `None` for the machine's cores, at most the
    /// instance numbers.
    pub jobs: Option<usize>,
    pub instances: (u8, u8),
    pub allow_dirty: bool,
    /// Fly the plan of the scenario files that are there and mark it a trial.
    pub reduced: bool,
    /// The command line, for `gate.txt`.
    pub command_line: String,
}

/// The state of the working tree.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Tree {
    pub commit: String,
    /// Tracked files have uncommitted changes.
    pub dirty: bool,
}

/// Whether `git status --porcelain --untracked-files=no` lists a change.
pub fn status_is_dirty(porcelain: &str) -> bool {
    porcelain.lines().any(|l| !l.trim().is_empty())
}

fn git(dir: &Path, args: &[&str]) -> Result<String, String> {
    let out = Command::new("git")
        .current_dir(dir)
        .args(args)
        .output()
        .map_err(|e| {
            if e.kind() == std::io::ErrorKind::NotFound {
                "git was not found, so the working tree cannot be checked for uncommitted \
                 changes: install git, or give --allow-dirty for a trial that no verdict accepts"
                    .to_string()
            } else {
                format!("cannot run git: {e}")
            }
        })?;
    if !out.status.success() {
        return Err(format!(
            "git {} failed in {}: {}: the gate is flown on a commit of a repository",
            args.join(" "),
            dir.display(),
            String::from_utf8_lossy(&out.stderr).trim()
        ));
    }
    Ok(String::from_utf8_lossy(&out.stdout).trim().to_string())
}

/// The commit and the cleanliness of the repository that holds `dir`.
pub fn tree_state(dir: &Path) -> Result<Tree, String> {
    let commit = git(dir, &["rev-parse", "HEAD"])?;
    let status = git(dir, &["status", "--porcelain", "--untracked-files=no"])?;
    Ok(Tree {
        commit,
        dirty: status_is_dirty(&status),
    })
}

fn now_text() -> String {
    let secs = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |d| d.as_secs());
    utc_text(secs)
}

/// Whether `dir` holds a table with a started row for every index of
/// `0..count` and no other row: a table the gate need not fly again.
pub fn table_complete(dir: &Path, scenario: &str, autopilot: &str, count: u64) -> bool {
    let Ok(text) = std::fs::read_to_string(dir.join(TABLE)) else {
        return false;
    };
    let mut lines = text.lines();
    if lines.next() != Some(header().as_str()) {
        return false;
    }
    let mut seen = std::collections::BTreeSet::new();
    for line in lines {
        let Ok(row) = Row::parse(line) else {
            return false;
        };
        let Ok(index) = row.get("index").parse::<u64>() else {
            return false;
        };
        if row.get("scenario") != scenario
            || row.get("autopilot") != autopilot
            || row.get("started") != "yes"
            || index >= count
            || !seen.insert(index)
        {
            return false;
        }
    }
    seen.len() as u64 == count
}

/// One batch of the gate.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Batch {
    pub scenario: String,
    pub autopilot: Autopilot,
    pub count: u64,
    pub twice: bool,
}

impl Batch {
    pub fn dir(&self, out: &Path) -> PathBuf {
        if self.twice {
            twice_dir(out, &self.scenario)
        } else {
            table_dir(out, &self.scenario, &self.autopilot.name())
        }
    }

    /// How it is named in `gate.txt` and on the screen.
    pub fn label(&self) -> String {
        if self.twice {
            format!("twice/{}", self.scenario)
        } else {
            format!("{}/{}", self.scenario, self.autopilot.name())
        }
    }
}

/// The batches of `plan` in the order they are flown: our core's first, then
/// ArduCopter's, then the flights flown twice.
pub fn schedule(plan: &Plan) -> Vec<Batch> {
    let mut v = Vec::new();
    let mut each = |ac: bool| {
        for e in &plan.entries {
            for autopilot in e.scoring.autopilots() {
                if (autopilot == Autopilot::ArduCopter) == ac {
                    v.push(Batch {
                        scenario: e.scenario.clone(),
                        autopilot,
                        count: e.count,
                        twice: false,
                    });
                }
            }
        }
    };
    each(false);
    each(true);
    for (scenario, count) in &plan.twice {
        v.push(Batch {
            scenario: scenario.clone(),
            autopilot: Autopilot::ArduCopter,
            count: *count,
            twice: true,
        });
    }
    v
}

fn binary_path(s: &Settings) -> Result<PathBuf, String> {
    if let Some(b) = &s.binary {
        return Ok(b.clone());
    }
    match std::env::var_os(BINARY_VARIABLE) {
        Some(p) if !p.is_empty() => Ok(PathBuf::from(p)),
        _ => Err(format!(
            "no simulator program: give --binary PATH or set {BINARY_VARIABLE}"
        )),
    }
}

fn sha_of(path: &Path) -> Result<String, String> {
    let bytes = std::fs::read(path).map_err(|e| format!("cannot read {}: {e}", path.display()))?;
    Ok(sha256::hex(&sha256::digest(&bytes)))
}

/// The SHA-256 of every parameter file a bridged flight is given, in the order
/// it is given: exactly the files of [`committed_params`], so that a file
/// added to the runner's list cannot be missing from `gate.txt`.
pub fn param_hashes(airframe: &oxide_sim::Airframe) -> Result<Vec<(String, String)>, String> {
    let mut params = Vec::new();
    for p in committed_params(airframe).0 {
        let name = p
            .file_name()
            .map_or_else(String::new, |n| n.to_string_lossy().into_owned());
        params.push((name, sha_of(&p)?));
    }
    Ok(params)
}

/// Adds the dirty-tree note to every row of a table.
pub fn mark_dirty(table: &Path) -> Result<(), String> {
    let path = table.join(TABLE);
    let text = std::fs::read_to_string(&path)
        .map_err(|e| format!("cannot read {}: {e}", path.display()))?;
    let mut out = format!("{}\n", header());
    for line in text.lines().skip(1) {
        let mut row = Row::parse(line).map_err(|e| format!("{}: {e}", path.display()))?;
        if !row.get("note").contains(DIRTY_NOTE) {
            let note = if row.get("note").is_empty() {
                DIRTY_NOTE.to_string()
            } else {
                format!("{}; {DIRTY_NOTE}", row.get("note"))
            };
            row.set("note", &note);
        }
        out.push_str(&row.line());
        out.push('\n');
    }
    std::fs::write(&path, out).map_err(|e| format!("cannot write {}: {e}", path.display()))
}

fn write_info(out: &Path, info: &GateInfo) -> Result<(), String> {
    let path = out.join("gate.txt");
    std::fs::write(&path, info.to_text())
        .map_err(|e| format!("cannot write {}: {e}", path.display()))
}

/// The refusal to fly the gate with scenario files, parameter files or a
/// simulator program that are not the commit's (`others`, from
/// `files::differences_from_build`). Only a trial (`--reduced`,
/// `--allow-dirty`) may.
pub fn refuse_other_files(s: &Settings, others: &[String]) -> Result<(), String> {
    files::refuse_others_for_the_gate(others, s.reduced || s.allow_dirty)
}

/// Flies the gate. `Ok(code)` is the exit code of a gate that ran (0 done, 3
/// stopped); `Err` is a wrong use or an environment that cannot fly it, as a
/// sentence (exit code 1).
pub fn fly(s: &Settings) -> Result<u8, String> {
    // The working tree first: the gate is flown on a commit.
    let built = env!("OXIDE_GIT_HASH");
    let (commit, dirty) = match tree_state(Path::new(".")) {
        Ok(t) => (t.commit, t.dirty),
        Err(e) if s.allow_dirty => {
            eprintln!("note: {e}");
            (built.to_string(), true)
        }
        Err(e) => return Err(e),
    };
    if dirty && !s.allow_dirty {
        return Err(
            "the working tree has uncommitted changes to tracked files: the gate is flown on a \
             commit (ADR-0007 section 1): commit them, or give --allow-dirty for a trial whose \
             tables say dirty tree and which no verdict accepts"
                .to_string(),
        );
    }
    if !s.allow_dirty && built != commit {
        return Err(format!(
            "this program was built at {built} and the working tree is at {commit}: build it \
             again at the commit that is to be flown"
        ));
    }
    let plan = Plan::build(&s.scenarios, s.reduced).map_err(|e| e.to_string())?;
    let binary = binary_path(s)?;
    let binary_sha = sha_of(&binary).map_err(|e| format!("the simulator program: {e}"))?;
    let airframe = oxide_sim::Airframe::reference_quad().map_err(|e| e.to_string())?;
    let params = param_hashes(&airframe)?;
    let mut scenario_files = Vec::new();
    for e in &plan.entries {
        let path = s.scenarios.join(format!("{}.toml", e.scenario));
        scenario_files.push((e.scenario.clone(), sha_of(&path)?));
    }
    // The children read the parameter files from the source tree and the
    // scenario files from `--scenarios`: for the gate they must be the ones
    // this program was built with, which are the commit's (ADR-0007 section
    // 7). A trial (`--reduced`, `--allow-dirty`) may fly others and is never
    // a pass.
    let others = files::differences_from_build(
        Some(&binary_sha),
        &params,
        &scenario_files,
        plan.entries.iter().map(|e| e.scenario.as_str()),
    );
    refuse_other_files(s, &others)?;
    let mut deviations = plan.deviations_from_the_record();
    deviations.extend(others);
    let reduced = s.reduced || !deviations.is_empty();
    if reduced {
        eprintln!(
            "note: this plan is not the gate's plan ({}): its verdict is never a pass",
            deviations.join("; ")
        );
    }
    let mut info = GateInfo {
        commit,
        tree_dirty: dirty,
        reduced,
        binary_sha256: Some(binary_sha),
        binary_path: binary.display().to_string(),
        command_line: s.command_line.clone(),
        first_started: now_text(),
        started: now_text(),
        ended: "not yet".into(),
        cores: std::thread::available_parallelism().map_or(1, usize::from),
        params,
        scenario_files,
        plan: plan.clone(),
        reused: Vec::new(),
        stopped: None,
    };
    if let Ok(old) = std::fs::read_to_string(s.out.join("gate.txt")) {
        let old = GateInfo::parse(&old).map_err(|e| {
            format!(
                "{} holds a gate.txt that cannot be read ({e}): use a directory of its own",
                s.out.display()
            )
        })?;
        if old.commit != info.commit
            || old.binary_sha256 != info.binary_sha256
            || old.scenario_files != info.scenario_files
            || old.params != info.params
        {
            return Err(format!(
                "{} holds a gate flown with another commit, binary, parameter file or scenario \
                 file: a table never mixes them, so use a directory of its own",
                s.out.display()
            ));
        }
        if !old.first_started.is_empty() {
            info.first_started = old.first_started;
        }
    }
    std::fs::create_dir_all(&s.out).map_err(|e| format!("cannot make {}: {e}", s.out.display()))?;
    write_info(&s.out, &info)?;

    let available = usize::from(s.instances.1 - s.instances.0) + 1;
    let jobs = s
        .jobs
        .unwrap_or_else(|| std::thread::available_parallelism().map_or(1, usize::from))
        .clamp(1, available);
    let exe = std::env::current_exe()
        .map_err(|e| format!("cannot find this program's own path to start the children: {e}"))?;
    let pin = if batch::taskset_available() {
        Pin::Taskset { first_core: 0 }
    } else {
        eprintln!("note: `taskset` was not found, so the children are not pinned to cores");
        Pin::Off
    };
    let all = schedule(&plan);
    println!(
        "gate       {} batches, {} jobs, commit {}",
        all.len(),
        jobs,
        info.commit
    );
    for (n, b) in all.iter().enumerate() {
        let dir = b.dir(&s.out);
        if table_complete(&dir, &b.scenario, &b.autopilot.name(), b.count) {
            println!("batch      {}/{} {} reused", n + 1, all.len(), b.label());
            info.reused.push(b.label());
            if dirty {
                mark_dirty(&dir)?;
            }
            continue;
        }
        if dir.exists() {
            let mut old = dir.clone().into_os_string();
            old.push(".interrupted");
            let old = PathBuf::from(old);
            let _ = std::fs::remove_dir_all(&old);
            std::fs::rename(&dir, &old)
                .map_err(|e| format!("cannot move {} aside: {e}", dir.display()))?;
        }
        println!(
            "batch      {}/{} {} ({} runs)",
            n + 1,
            all.len(),
            b.label(),
            b.count
        );
        let bridged = b.autopilot == Autopilot::ArduCopter;
        let args = BatchArgs {
            common: Common {
                scenario: s.scenarios.join(format!("{}.toml", b.scenario)),
                autopilot: b.autopilot,
                binary: bridged.then(|| binary.clone()),
                speedup: 100,
                param_files: Vec::new(),
                twice: b.twice,
            },
            indices: (0, b.count),
            out: dir.clone(),
            jobs,
            instances: s.instances,
            limit_s: DEFAULT_LIMIT_S,
            pin: true,
            first_core: 0,
        };
        let src = LoadedScenario::load(s.scenarios.join(format!("{}.toml", b.scenario)))
            .map_err(|e| e.to_string())?;
        let cfg = BatchConfig {
            scenario: b.scenario.clone(),
            autopilot: b.autopilot,
            runs: (0..b.count).map(|i| (i, src.file.seed(i))).collect(),
            jobs,
            instances: s.instances,
            limit: Duration::from_secs_f64(DEFAULT_LIMIT_S * if b.twice { 2.0 } else { 1.0 }),
            out: dir.clone(),
            pin,
        };
        let launch =
            |job: &Job| cli::child_command(&exe, &args, args.common.binary.as_deref(), job);
        let say = |e: &Event| {
            if let Event::Failed {
                index,
                attempt,
                reason,
                will_retry,
            } = e
            {
                eprintln!(
                    "run {index} attempt {attempt}: failed: {reason}{}",
                    if *will_retry {
                        " (flown once more)"
                    } else {
                        ""
                    }
                );
            }
        };
        let result = batch::run_batch(&cfg, &launch, &say).map_err(|e| e.to_string())?;
        if let Some(i) = result.stopped_at {
            info.stopped = Some(format!(
                "{} run {i} failed twice: the gate stopped with the tables as they are",
                b.label()
            ));
            info.ended = now_text();
            write_info(&s.out, &info)?;
            eprintln!(
                "error: {} run {i} failed twice: the gate stopped with the tables as they are \
                 (ADR-0007 section 4)",
                b.label()
            );
            return Ok(EXIT_STOPPED);
        }
        if dirty {
            mark_dirty(&dir)?;
        }
    }
    info.ended = now_text();
    write_info(&s.out, &info)?;
    println!(
        "gate       flown{}: oxide-bridge gate verdict --in {}",
        if info.reused.is_empty() {
            String::new()
        } else {
            format!(" ({} tables reused)", info.reused.len())
        },
        s.out.display()
    );
    Ok(0)
}

/// Runs `gate fly` and gives the exit code.
pub fn run(s: &Settings) -> ExitCode {
    match fly(s) {
        Ok(code) => ExitCode::from(code),
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::from(1)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gate_txt_lists_the_sha256_of_exactly_the_runners_files_in_the_runners_order() {
        let airframe = oxide_sim::Airframe::reference_quad().expect("airframe");
        let listed = param_hashes(&airframe).expect("hashes");
        let files = committed_params(&airframe).0;
        let names: Vec<String> = files
            .iter()
            .map(|p| p.file_name().expect("name").to_string_lossy().into_owned())
            .collect();
        assert_eq!(
            listed.iter().map(|(n, _)| n.clone()).collect::<Vec<_>>(),
            names
        );
        assert!(!names.is_empty());
        for ((_, h), p) in listed.iter().zip(&files) {
            assert_eq!(h, &sha_of(p).expect("sha"));
            assert_eq!(h.len(), 64);
        }
    }
}
