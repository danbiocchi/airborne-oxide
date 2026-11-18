// SPDX-License-Identifier: AGPL-3.0-or-later
//! Gate directories written by hand: `gate.txt`, the tables and the run
//! directories with their `audit.txt`, in the layout `gate fly` leaves. Every
//! row is a good one until a test edits it.

#![allow(dead_code)]

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use oxide_bridge::batch::run_dir;
use oxide_bridge::gate::files;
use oxide_bridge::gate::info::GateInfo;
use oxide_bridge::gate::plan::{Entry, Plan, Scoring};
use oxide_bridge::gate::verdict::{table_dir, twice_dir};
use oxide_bridge::record::{Row, header};
use oxide_sim::rng::run_seed;

const MADE_UP_COMMIT: &str = "0123456789abcdef0123456789abcdef01234567";

/// The commit the fixtures say they were flown at: this build's own when it
/// is a clean commit (the verdict gives none on tables of another commit), a
/// made-up one for a build on a dirty tree.
pub fn commit() -> String {
    oxide_bridge::gate::verdict::own_commit()
        .map_or_else(|| MADE_UP_COMMIT.to_string(), String::from)
}
/// The pinned simulator program's checksum: the only one the verdict accepts.
pub const BINARY: &str = oxide_bridge::sitl::PINNED.sha256;

pub const AC: &str = "arducopter";
pub const F405: &str = "core-f405";
pub const H743: &str = "core-h743";
pub const TRUTH: &str = "core-f405-truth";

/// One flight's row and the audit that goes with it.
#[derive(Clone)]
pub struct Flown {
    /// The row of the table.
    pub row: Row,
    /// What the flight's own run directory holds when it is not the table's
    /// row (`row.csv` and the audit are written from it): a table edited
    /// after the flight.
    pub own: Option<Row>,
    /// The `repeats` word of the audit of a flight flown twice, when it is
    /// not the row's.
    pub audit_repeats: Option<String>,
    /// The trace, packet and reply hash of a second flight, when they are
    /// not the first's.
    pub second: Option<(String, String, String)>,
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
        own: None,
        audit_repeats: None,
        second: None,
        commit: commit(),
        file_sha: file_sha(scenario),
        binary: BINARY.to_string(),
        params: params_sha(scenario),
    }
}

impl Flown {
    /// Edits a cell of the table only: the flight's own run directory keeps
    /// the row it had.
    pub fn edit_table_only(&mut self, column: &str, value: &str) {
        if self.own.is_none() {
            self.own = Some(self.row.clone());
        }
        self.row.set(column, value);
    }
}

/// The SHA-256 of the scenario file of the commit (a made-up one for a name
/// that is not a scenario of the repository).
pub fn file_sha(scenario: &str) -> String {
    files::scenario_sha256(scenario).unwrap_or_else(|| format!("{:064x}", run_seed(scenario, 99)))
}

/// The effective parameter hash of a bridged flight of the scenario.
pub fn params_sha(scenario: &str) -> String {
    files::expected_params_sha256(scenario).unwrap_or_else(|_| format!("params-of-{scenario}"))
}

impl Fixture {
    /// A directory of good tables for `plan`, under a temporary directory
    /// named by `tag`.
    pub fn of(plan: Plan, reduced: bool, tag: &str) -> Self {
        static COUNTER: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
        let n = COUNTER.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
        let dir =
            std::env::temp_dir().join(format!("oxide-gate-fx-{tag}-{}-{n}", std::process::id()));
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
            commit: commit(),
            tree_dirty: false,
            reduced,
            binary_sha256: Some(BINARY.into()),
            binary_path: "/x/arducopter".into(),
            command_line: "oxide-bridge gate fly".into(),
            first_started: "2026-01-01T00:00:00Z".into(),
            started: "2026-01-01T00:00:00Z".into(),
            ended: "2026-01-01T09:00:00Z".into(),
            cores: 4,
            params: files::param_hashes().expect("the commit's parameter hashes"),
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
        // The flight's own record of itself.
        let own = f.own.as_ref().unwrap_or(&f.row);
        let mut audit = format!(
            "oxide-bridge flight record\n  scenario        {}\n  \
             scenario file   /somewhere/{}.toml\n  file sha256     {}\n  \
             run index       {}\n  seed            {}\n  autopilot       {}\n  \
             our commit      {}\n",
            own.get("scenario"),
            own.get("scenario"),
            f.file_sha,
            own.get("index"),
            own.get("seed"),
            own.get("autopilot"),
            f.commit
        );
        if !f.row.get("repeats").is_empty() || f.audit_repeats.is_some() {
            audit.push_str(&format!(
                "  repeats         {} (trace, packet and reply hashes of two flights)\n",
                f.audit_repeats
                    .clone()
                    .unwrap_or_else(|| f.row.get("repeats").to_string())
            ));
        }
        audit.push_str(&format!("  class           {}\n", own.get("class")));
        if bridged && f.row.get("autopilot") == AC {
            audit.push_str(&format!(
                "\nflight audit\n  binary sha256   {}\n  params sha256   {}\n  \
                 seed {}, step 1000 us, 3 pre-roll steps, 9 exchanges\n  \
                 packet hash {}, reply hash {}\n",
                f.binary,
                f.params,
                own.get("seed"),
                own.get("packet_hash"),
                own.get("reply_hash")
            ));
        }
        // A doubled row has the same run directory: it is written once.
        std::fs::write(run.join("audit.txt"), audit).expect("audit");
        std::fs::write(
            run.join("row.csv"),
            format!("{}\n{}\n", header(), own.line()),
        )
        .expect("row.csv");
        if !f.row.get("repeats").is_empty() || f.audit_repeats.is_some() {
            // The second flight of a flight flown twice.
            let second = run.join("second");
            std::fs::create_dir_all(&second).expect("second dir");
            let (trace, packet, reply) = f.second.clone().unwrap_or_else(|| {
                (
                    own.get("trace_hash").to_string(),
                    own.get("packet_hash").to_string(),
                    own.get("reply_hash").to_string(),
                )
            });
            std::fs::write(
                second.join("audit.txt"),
                format!(
                    "oxide-bridge flight record\n  scenario        {}\n\nflight audit\n  \
                     packet hash {packet}, reply hash {reply}\n",
                    own.get("scenario")
                ),
            )
            .expect("second audit");
            let mut again = own.clone();
            again.set("trace_hash", &trace);
            again.set("repeats", "");
            std::fs::write(
                second.join("row.csv"),
                format!("{}\n{}\n", header(), again.line()),
            )
            .expect("second row");
        }
    }
    std::fs::write(dir.join("flights.csv"), text).expect("flights.csv");
}
