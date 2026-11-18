// SPDX-License-Identifier: AGPL-3.0-or-later
//! `gate.txt`: what the gate was, written by `gate fly` and read by `gate
//! verdict` (which reads nothing else but the tables): our commit, the
//! binary's checksum, the SHA-256 of every parameter file and scenario file,
//! the command line, the wall times, the machine's core count, and the plan
//! with the facts of the scenario files that the verdict needs.
//!
//! One fact per line, a key then its value. The plan is written whole, so that
//! the verdict does not need the scenario directory.

use core::fmt::Write as _;

use crate::gate::plan::{Entry, Plan, Scoring};

/// The first line of the file.
pub const MAGIC: &str = "oxide-bridge gate 1";

/// The note of a table flown on a tree with uncommitted changes.
pub const DIRTY_NOTE: &str = "dirty tree";

/// What `gate.txt` says.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct GateInfo {
    /// `git rev-parse HEAD` when the gate was flown.
    pub commit: String,
    /// The tree had uncommitted changes to tracked files (`--allow-dirty`).
    pub tree_dirty: bool,
    /// The plan is a trial: not the gate's plan, its verdict is never a pass.
    pub reduced: bool,
    /// SHA-256 of the simulator program; `None` when nothing was flown with it.
    pub binary_sha256: Option<String>,
    pub binary_path: String,
    pub command_line: String,
    pub first_started: String,
    pub started: String,
    pub ended: String,
    pub cores: usize,
    /// Parameter file name and SHA-256.
    pub params: Vec<(String, String)>,
    /// Scenario name and the SHA-256 of its file.
    pub scenario_files: Vec<(String, String)>,
    pub plan: Plan,
    /// The tables a continued gate did not fly again.
    pub reused: Vec<String>,
    /// Why the gate stopped, when it did.
    pub stopped: Option<String>,
}

impl GateInfo {
    /// The SHA-256 of the file of `scenario`.
    pub fn scenario_sha(&self, scenario: &str) -> Option<&str> {
        self.scenario_files
            .iter()
            .find(|(n, _)| n == scenario)
            .map(|(_, h)| h.as_str())
    }

    /// The text of the file.
    pub fn to_text(&self) -> String {
        let mut s = String::new();
        let _ = writeln!(s, "{MAGIC}");
        let _ = writeln!(s, "commit {}", self.commit);
        let _ = writeln!(
            s,
            "tree {}",
            if self.tree_dirty { DIRTY_NOTE } else { "clean" }
        );
        let _ = writeln!(s, "scope {}", if self.reduced { "reduced" } else { "full" });
        let _ = writeln!(
            s,
            "binary sha256 {}",
            self.binary_sha256.as_deref().unwrap_or("none")
        );
        let _ = writeln!(s, "binary path {}", self.binary_path);
        let _ = writeln!(s, "command line {}", self.command_line);
        let _ = writeln!(s, "first started {}", self.first_started);
        let _ = writeln!(s, "started {}", self.started);
        let _ = writeln!(s, "ended {}", self.ended);
        let _ = writeln!(s, "cores {}", self.cores);
        for (n, h) in &self.params {
            let _ = writeln!(s, "param file {n} sha256 {h}");
        }
        for (n, h) in &self.scenario_files {
            let _ = writeln!(s, "scenario file {n} sha256 {h}");
        }
        for e in &self.plan.entries {
            let _ = writeln!(s, "plan {} {} {}", e.scoring.name(), e.scenario, e.count);
        }
        for (n, c) in &self.plan.twice {
            let _ = writeln!(s, "twice {n} {c}");
        }
        for (n, k) in &self.plan.pair_kinds {
            let _ = writeln!(s, "pair {n} {}", k.join(","));
        }
        for (n, t) in &self.plan.failure_starts {
            let _ = writeln!(s, "failure {n} {t}");
        }
        for r in &self.reused {
            let _ = writeln!(s, "reused {r}");
        }
        if let Some(why) = &self.stopped {
            let _ = writeln!(s, "stopped {why}");
        }
        s
    }

    /// Reads the text of a file; the error is a sentence.
    pub fn parse(text: &str) -> Result<Self, String> {
        let mut lines = text.lines();
        if lines.next() != Some(MAGIC) {
            return Err(format!("gate.txt does not start with {MAGIC:?}"));
        }
        let mut g = Self::default();
        let mut have = std::collections::BTreeSet::new();
        for (i, line) in lines.enumerate() {
            let n = i + 2;
            let bad = |why: &str| format!("gate.txt line {n} ({line:?}): {why}");
            let (key, rest) = line.split_once(' ').unwrap_or((line, ""));
            let words: Vec<&str> = rest.split_whitespace().collect();
            match key {
                "commit" => g.commit = rest.trim().to_string(),
                "tree" => match rest.trim() {
                    "clean" => g.tree_dirty = false,
                    DIRTY_NOTE => g.tree_dirty = true,
                    _ => return Err(bad("the tree is clean or a dirty tree")),
                },
                "scope" => match rest.trim() {
                    "full" => g.reduced = false,
                    "reduced" => g.reduced = true,
                    _ => return Err(bad("the scope is full or reduced")),
                },
                "binary" => match words.as_slice() {
                    ["sha256", "none"] => g.binary_sha256 = None,
                    ["sha256", h] => g.binary_sha256 = Some((*h).to_string()),
                    ["path", ..] => g.binary_path = rest.trim_start_matches("path ").to_string(),
                    _ => return Err(bad("not understood")),
                },
                "command" => g.command_line = rest.trim_start_matches("line ").to_string(),
                "first" => g.first_started = rest.trim_start_matches("started ").to_string(),
                "started" => g.started = rest.trim().to_string(),
                "ended" => g.ended = rest.trim().to_string(),
                "cores" => {
                    g.cores = rest.trim().parse().map_err(|_| bad("a whole number"))?;
                }
                "param" => match words.as_slice() {
                    ["file", name, "sha256", h] => {
                        g.params.push(((*name).to_string(), (*h).to_string()));
                    }
                    _ => return Err(bad("param file NAME sha256 HASH")),
                },
                "scenario" => match words.as_slice() {
                    ["file", name, "sha256", h] => {
                        g.scenario_files
                            .push(((*name).to_string(), (*h).to_string()));
                    }
                    _ => return Err(bad("scenario file NAME sha256 HASH")),
                },
                "plan" => match words.as_slice() {
                    [scoring, name, count] => {
                        let scoring = Scoring::parse(scoring).ok_or_else(|| bad("a scoring"))?;
                        let count = count.parse().map_err(|_| bad("a whole number"))?;
                        g.plan.entries.push(Entry {
                            scenario: (*name).to_string(),
                            scoring,
                            count,
                        });
                    }
                    _ => return Err(bad("plan SCORING SCENARIO COUNT")),
                },
                "twice" => match words.as_slice() {
                    [name, count] => g.plan.twice.push((
                        (*name).to_string(),
                        count.parse().map_err(|_| bad("a whole number"))?,
                    )),
                    _ => return Err(bad("twice SCENARIO COUNT")),
                },
                "pair" => match words.as_slice() {
                    [name, kinds] => {
                        g.plan.pair_kinds.insert(
                            (*name).to_string(),
                            kinds.split(',').map(String::from).collect(),
                        );
                    }
                    [name] => {
                        g.plan.pair_kinds.insert((*name).to_string(), Vec::new());
                    }
                    _ => return Err(bad("pair SCENARIO KINDS")),
                },
                "failure" => match words.as_slice() {
                    [name, start] => {
                        let start: f64 = start.parse().map_err(|_| bad("a number of seconds"))?;
                        if !start.is_finite() {
                            return Err(bad("a number of seconds"));
                        }
                        g.plan.failure_starts.insert((*name).to_string(), start);
                    }
                    _ => return Err(bad("failure SCENARIO START")),
                },
                "reused" => g.reused.push(rest.trim().to_string()),
                "stopped" => g.stopped = Some(rest.trim().to_string()),
                _ => return Err(bad("not understood")),
            }
            have.insert(key.to_string());
        }
        for key in ["commit", "tree", "scope", "binary", "cores", "plan"] {
            if !have.contains(key) {
                return Err(format!("gate.txt has no {key} line"));
            }
        }
        if g.commit.is_empty() {
            return Err("gate.txt names no commit".into());
        }
        Ok(g)
    }
}

/// `YYYY-MM-DDTHH:MM:SSZ` for seconds since the Unix epoch (UTC).
pub fn utc_text(secs: u64) -> String {
    let days = (secs / 86_400) as i64;
    let rem = secs % 86_400;
    // Civil date from days since 1970-01-01 (Howard Hinnant's algorithm).
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1_460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = yoe + era * 400 + i64::from(m <= 2);
    format!(
        "{y:04}-{m:02}-{d:02}T{:02}:{:02}:{:02}Z",
        rem / 3_600,
        rem % 3_600 / 60,
        rem % 60
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample() -> GateInfo {
        let mut g = GateInfo {
            commit: "abc123".into(),
            binary_sha256: Some("ff00".into()),
            binary_path: "/x/arducopter".into(),
            command_line: "oxide-bridge gate fly --out o".into(),
            first_started: "2026-01-01T00:00:00Z".into(),
            started: "2026-01-02T00:00:00Z".into(),
            ended: "2026-01-03T00:00:00Z".into(),
            cores: 4,
            params: vec![("base.parm".into(), "11".into())],
            scenario_files: vec![("gust-hold".into(), "22".into())],
            reused: vec!["gust-hold/core-f405".into()],
            stopped: Some("run 3 failed twice".into()),
            ..GateInfo::default()
        };
        g.plan.entries.push(Entry {
            scenario: "gust-hold".into(),
            scoring: Scoring::Gust,
            count: 500,
        });
        g.plan.twice.push(("gust-hold".into(), 5));
        g.plan
            .pair_kinds
            .insert("pair-a+b".into(), vec!["a".into(), "b".into()]);
        g.plan.failure_starts.insert("motor-loss".into(), 15.5);
        g
    }

    #[test]
    fn the_text_reads_back_as_it_was_written() {
        let g = sample();
        assert_eq!(GateInfo::parse(&g.to_text()), Ok(g.clone()));
        let mut dirty = g;
        dirty.tree_dirty = true;
        dirty.reduced = true;
        dirty.binary_sha256 = None;
        dirty.stopped = None;
        assert_eq!(GateInfo::parse(&dirty.to_text()), Ok(dirty));
    }

    #[test]
    fn a_text_that_is_not_understood_is_a_sentence() {
        assert!(GateInfo::parse("nonsense").is_err());
        let ok = sample().to_text();
        let e = GateInfo::parse(&format!("{ok}wibble 3\n")).unwrap_err();
        assert!(e.contains("line") && e.contains("wibble"), "{e}");
        let e = GateInfo::parse(&ok.replace("commit abc123\n", "")).unwrap_err();
        assert!(e.contains("commit"), "{e}");
        let e = GateInfo::parse(&ok.replace("scope full", "scope big")).unwrap_err();
        assert!(e.contains("scope"), "{e}");
    }

    #[test]
    fn the_utc_text_is_the_calendar_date() {
        assert_eq!(utc_text(0), "1970-01-01T00:00:00Z");
        assert_eq!(utc_text(951_782_400), "2000-02-29T00:00:00Z");
        assert_eq!(utc_text(1_700_000_000), "2023-11-14T22:13:20Z");
        assert_eq!(utc_text(4_102_444_799), "2099-12-31T23:59:59Z");
    }
}
