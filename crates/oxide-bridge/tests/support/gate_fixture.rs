// SPDX-License-Identifier: AGPL-3.0-or-later
//! Gate directories written by hand: `gate.txt`, the tables and the run
//! directories with their `audit.txt`, in the layout `gate fly` leaves. Every
//! row is a good one until a test edits it.

#![allow(dead_code)]

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use oxide_bridge::batch::run_dir;
use oxide_bridge::gate::info::GateInfo;
use oxide_bridge::gate::plan::{Entry, Plan, Scoring};
use oxide_bridge::gate::verdict::{table_dir, twice_dir};
use oxide_bridge::record::{Row, header};
use oxide_sim::rng::run_seed;

pub const COMMIT: &str = "0123456789abcdef0123456789abcdef01234567";
pub const BINARY: &str = "b1b1b1b1b1b1b1b1b1b1b1b1b1b1b1b1b1b1b1b1b1b1b1b1b1b1b1b1b1b1b1b1";

pub const AC: &str = "arducopter";
pub const F405: &str = "core-f405";
pub const H743: &str = "core-h743";
pub const TRUTH: &str = "core-f405-truth";

/// One flight's row and the audit that goes with it.
#[derive(Clone)]
pub struct Flown {
    pub row: Row,
    pub commit: String,
    pub file_sha: String,
    pub binary: String,
    pub params: String,
}

pub struct Fixture {
    pub dir: PathBuf,
    pub info: GateInfo,
    pub tables: BTreeMap<(String, String), Vec<Flown>>,
    pub twice: BTreeMap<String, Vec<Flown>>,
}

fn hash(scenario: &str, autopilot: &str, index: u64, salt: u64) -> String {
    format!(
        "{:016x}",
        run_seed(&format!("{scenario}/{autopilot}/{salt}"), index)
    )
}

/// A good flight: safe, passing (not judged for ArduCopter, as in real
/// tables), with numbers that make everything hold.
fn good(entry: &Entry, autopilot: &str, index: u64, twice: bool) -> Flown {
    let scenario = entry.scenario.as_str();
    let mut r = Row::empty();
    r.set("scenario", scenario);
    r.set("index", &index.to_string());
    r.set("seed", &run_seed(scenario, index).to_string());
    r.set("autopilot", autopilot);
    r.set("started", "yes");
    r.set("class", "safe");
    r.set(
        "verdict",
        if autopilot == AC && entry.scoring != Scoring::Gust {
            "not-judged"
        } else {
            "pass"
        },
    );
    r.set("violations", "0");
    r.set("not_judged", "0");
    if entry.scoring == Scoring::Gust {
        // ArduCopter a little worse than our core.
        let base = if autopilot == AC { 0.2 } else { 0.15 };
        r.set("rms_h_m", &format!("{}", base + (index % 7) as f64 * 0.001));
        r.set("rms_v_m", "0.05");
        r.set("max_tilt_deg", "9.5");
    }
    if entry.scoring == Scoring::MotorsOff {
        // The failure is at 15 s; ArduCopter stops later than our core.
        let t = if autopilot == AC { 17.0 } else { 15.2 };
        r.set("last_motor_on_s", &format!("{t}"));
    }
    r.set("trace_hash", &hash(scenario, autopilot, index, 1));
    if autopilot == AC {
        r.set("packet_hash", &hash(scenario, autopilot, index, 2));
        r.set("reply_hash", &hash(scenario, autopilot, index, 3));
    }
    if twice {
        r.set("repeats", "yes");
    }
    r.set("retried", "no");
    Flown {
        row: r,
        commit: COMMIT.to_string(),
        file_sha: file_sha(scenario),
        binary: BINARY.to_string(),
        params: format!("params-of-{scenario}"),
    }
}

pub fn file_sha(scenario: &str) -> String {
    format!("{:064x}", run_seed(scenario, 99))
}

impl Fixture {
    /// A directory of good tables for `plan`, under a temporary directory
    /// named by `tag`.
    pub fn of(plan: Plan, reduced: bool, tag: &str) -> Self {
        let dir = std::env::temp_dir().join(format!("oxide-gate-fx-{tag}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let mut tables = BTreeMap::new();
        for e in &plan.entries {
            for who in e.scoring.autopilots() {
                let name = who.name();
                let flights = (0..e.count).map(|i| good(e, &name, i, false)).collect();
                tables.insert((e.scenario.clone(), name), flights);
            }
        }
        let mut twice = BTreeMap::new();
        for (s, n) in &plan.twice {
            let e = plan
                .entries
                .iter()
                .find(|e| &e.scenario == s)
                .expect("entry");
            twice.insert(s.clone(), (0..*n).map(|i| good(e, AC, i, true)).collect());
            // The second flight's hashes are the first's.
        }
        let info = GateInfo {
            commit: COMMIT.into(),
            tree_dirty: false,
            reduced,
            binary_sha256: Some(BINARY.into()),
            binary_path: "/x/arducopter".into(),
            command_line: "oxide-bridge gate fly".into(),
            first_started: "2026-01-01T00:00:00Z".into(),
            started: "2026-01-01T00:00:00Z".into(),
            ended: "2026-01-01T09:00:00Z".into(),
            cores: 4,
            params: vec![("base.parm".into(), "aa".into())],
            scenario_files: plan
                .entries
                .iter()
                .map(|e| (e.scenario.clone(), file_sha(&e.scenario)))
                .collect(),
            plan,
            reused: Vec::new(),
            stopped: None,
        };
        Self {
            dir,
            info,
            tables,
            twice,
        }
    }

    /// A small plan of a trial: 20 gust flights, 3 of `gps-loss` (outcome),
    /// 5 of `motor-loss` (motors off, failure at 15 s), 2 of `hover-step`
    /// (not scored); two gust flights and one of `gps-loss` flown twice.
    pub fn small(tag: &str) -> Self {
        let mut plan = Plan::default();
        for (name, scoring, count) in [
            ("gust-hold", Scoring::Gust, 20),
            ("gps-loss", Scoring::Outcome, 3),
            ("motor-loss", Scoring::MotorsOff, 5),
            ("hover-step", Scoring::NotScored, 2),
        ] {
            plan.entries.push(Entry {
                scenario: name.into(),
                scoring,
                count,
            });
        }
        plan.twice = vec![("gust-hold".into(), 2), ("gps-loss".into(), 1)];
        plan.failure_starts.insert("motor-loss".into(), 15.0);
        Self::of(plan, true, tag)
    }

    /// The plan of the repository's scenario directory: the gate itself.
    pub fn full(tag: &str) -> Self {
        let dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../scenarios");
        let plan = Plan::build(&dir, false).expect("the plan");
        Self::of(plan, false, tag)
    }

    pub fn flown(&mut self, scenario: &str, autopilot: &str, index: u64) -> &mut Flown {
        self.tables
            .get_mut(&(scenario.to_string(), autopilot.to_string()))
            .and_then(|v| {
                v.iter_mut()
                    .find(|f| f.row.get("index") == index.to_string())
            })
            .unwrap_or_else(|| panic!("no row {scenario} {autopilot} {index}"))
    }

    pub fn row(&mut self, scenario: &str, autopilot: &str, index: u64) -> &mut Row {
        &mut self.flown(scenario, autopilot, index).row
    }

    pub fn twice_flown(&mut self, scenario: &str, index: u64) -> &mut Flown {
        self.twice
            .get_mut(scenario)
            .and_then(|v| {
                v.iter_mut()
                    .find(|f| f.row.get("index") == index.to_string())
            })
            .expect("a twice row")
    }

    /// Makes a flight of ArduCopter's twice-flown table the same as the main
    /// table's (hashes), which `of` already does by construction of `good`.
    pub fn write(&self) {
        std::fs::create_dir_all(&self.dir).expect("dir");
        std::fs::write(self.dir.join("gate.txt"), self.info.to_text()).expect("gate.txt");
        for ((s, a), flights) in &self.tables {
            write_table(&table_dir(&self.dir, s, a), flights, a == AC);
        }
        for (s, flights) in &self.twice {
            write_table(&twice_dir(&self.dir, s), flights, true);
        }
    }

    pub fn remove(&self) {
        let _ = std::fs::remove_dir_all(&self.dir);
    }
}

fn write_table(dir: &Path, flights: &[Flown], bridged: bool) {
    std::fs::create_dir_all(dir).expect("table dir");
    let mut text = format!("{}\n", header());
    for f in flights {
        text.push_str(&f.row.line());
        text.push('\n');
        let index: u64 = f.row.get("index").parse().unwrap_or(0);
        let run = run_dir(dir, index);
        std::fs::create_dir_all(&run).expect("run dir");
        let mut audit = format!(
            "oxide-bridge flight record\n  scenario        {}\n  file sha256     {}\n  \
             run index       {index}\n  our commit      {}\n",
            f.row.get("scenario"),
            f.file_sha,
            f.commit
        );
        if bridged && f.row.get("autopilot") == AC {
            audit.push_str(&format!(
                "\nflight audit\n  binary sha256   {}\n  params sha256   {}\n",
                f.binary, f.params
            ));
        }
        // A doubled row has the same run directory: it is written once.
        std::fs::write(run.join("audit.txt"), audit).expect("audit");
    }
    std::fs::write(dir.join("flights.csv"), text).expect("flights.csv");
}
