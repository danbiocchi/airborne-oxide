// SPDX-License-Identifier: AGPL-3.0-or-later
//! `oxide-sim`: run a scenario headless.
//!
//! `oxide-sim help` (also `--help` and `-h`) prints the usage of every command.
//!
//! ```text
//! oxide-sim run  [--profile f405|h743] [--airframe FILE] [--csv FILE]
//!                [--log FILE.ulg] [--log-level normal|replay]
//!                [--scenario hover-step|gust-hold] [--seed N]
//!                [--estimator truth|ekf] [--sensors ideal|realistic]
//! oxide-sim hash [same options as run, except --log and --log-level]
//! oxide-sim montecarlo [--profile f405|h743] [--seeds N] [--threads T]
//! oxide-sim scenario list
//! oxide-sim scenario NAME|PATH [--profile f405|h743] [--seed N] [--csv FILE]
//!                [--log FILE.ulg] [--log-level normal|replay]
//! oxide-sim scenarios DIR [--seeds N] [--profile f405|h743|both] [--threads T]
//! oxide-sim replay FILE.ulg [--param NAME=VALUE ...] [--continue]
//! oxide-sim params list | json | doc
//! ```
//! `run`, `hash`, `montecarlo` and `scenario` also take `--params FILE.toml`
//! (a `[params]` table) and `--param NAME=VALUE` (repeatable): parameter
//! overrides applied through the checked `set`, the file first, then the flags
//! in order (ADR-0004). `params list` prints the table, `params json` the
//! ground-station metadata and `params doc` the text of `docs/PARAMETERS.md`.
//! `--log FILE.ulg` on `run` and `scenario` writes the flight's log (ADR-0005)
//! in PX4's ULog format; `--log-level` (default `normal`) needs `--log`. The
//! header's `ver_sw` is the git hash the binary was built from (`build.rs`),
//! `-dirty` when the tree had changes the last time that script ran (edits made
//! after it are not seen until it runs again), `ver_hw` is `SIM`.
//! `replay FILE.ulg` rebuilds the flight core from a replay-level log
//! (`--log-level replay`) and runs the recorded ticks through it, comparing
//! every output bit for bit; it exits 0 when every tick replayed is identical
//! (also when the file ends in a cut), 1 on a difference, a gap, a tick that is
//! not whole, damage at the end of the file or a file with no tick, and 2 on a
//! file it cannot replay. `--param` replays through a changed setting,
//! `--continue` counts every tick that differs instead of stopping at the first.
//! Every command exits 0 when it passed, 1 when a run it judged failed
//! (`montecarlo`, `scenario`, `scenarios`, `replay`) and 2 on an error: a bad
//! argument, an unreadable file, an output that cannot be written. An output
//! (`--csv`, `--log`) is opened before the flight, and a command that fails
//! before its flight leaves none behind; a write error after it names the file
//! (`error: FILE: No space left on device (os error 28)`), and `scenario` has
//! printed its summary and verdict by then.
//! `hash` prints trace hashes, which CI compares between Linux and Windows
//! (DESIGN §5.2). Without `--scenario` it prints one line for hover-step and
//! one for gust-hold run 0, so the check covers the RNG, the noise models and
//! the libm paths, then one line per fault scenario in `HASHED_FAULT_SCENARIOS`
//! (run 0, `<profile> <name>#0 <hash>`), so that it also covers ground contact,
//! landings, failsafes and a crash, then one line per logged flight
//! (`<profile> log-normal <name>#0 <FNV-1a 64 of the log's bytes>`, written with
//! the fixed build information `hash`/`SIM`) and one replay-level log of
//! rc-loss-position (`<profile> log-replay rc-loss-position#0 <FNV-1a 64>`).
//! `montecarlo` flies gust-hold indices 0..N (default 200) on
//! all cores (or `--threads T`) against the M1b acceptance criteria and exits
//! with 1 if any run fails. `scenario` flies a scenario (ADR-0003, ADR-0004
//! §7): a name among the embedded ones (`scenario list`), or the path of a
//! scenario file (an argument ending in `.toml` or containing a path
//! separator is a path). It prints its event timeline, a summary and every
//! violated expectation, and exits with 1 if there are any; `--seed` is the
//! run index. `scenarios DIR` flies every `*.toml` of a directory in
//! parallel, on the profiles given (default: each file's `[run] profiles`)
//! and `--seeds` runs (default: each file's `[run] seeds`), prints one line
//! per scenario and profile and exits with 1 if any run fails.

mod params_cmd;
mod scenarios_cmd;

use std::io::Write;
use std::process::ExitCode;
use std::time::Instant;

use oxide_core::config::EstimatorSource;
use oxide_core::math::Vec3;
use oxide_core::schedule::{F405_LEAN, H743_FULL, Profile};
use oxide_core::time::Micros;
use oxide_sim::fault_scenarios;
use oxide_sim::flight_log::{Level, LogBuild, LogError, LogWriter};
use oxide_sim::metrics::{CoreHold, HoldCriteria, HoldMetrics, fly_gust_hold_with_params};
use oxide_sim::montecarlo::run_indexed;
use oxide_sim::scenario::{GustHold, HoverStep, Pilot, gust_hold};
use oxide_sim::scenario_file::{self, Checker, ScenarioFile};
use oxide_sim::sensors::SensorConfig;
use oxide_sim::{Airframe, SimError, SimOptions, Simulation, StepRecord};

/// The git hash this binary was built from, `-dirty` when the tree had
/// changes, `unknown` without git (`build.rs`).
const GIT_HASH: &str = env!("OXIDE_GIT_HASH");

/// The level of a log (`--log-level`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum LogLevel {
    Normal,
    Replay,
}

/// `--log FILE.ulg` and `--log-level`.
#[derive(Clone, Debug, PartialEq, Eq)]
struct LogSpec {
    path: String,
    level: LogLevel,
}

impl LogSpec {
    /// The two options as given: a level needs a file.
    fn from_options(path: Option<String>, level: Option<&str>) -> Result<Option<Self>, String> {
        let Some(path) = path else {
            return match level {
                Some(_) => Err("--log-level needs --log FILE.ulg".into()),
                None => Ok(None),
            };
        };
        let level = match level {
            None | Some("normal") => LogLevel::Normal,
            Some("replay") => LogLevel::Replay,
            Some(other) => return Err(format!("unknown log level {other} (normal | replay)")),
        };
        Ok(Some(Self { path, level }))
    }

    fn level(&self) -> Level {
        match self.level {
            LogLevel::Normal => Level::Normal,
            LogLevel::Replay => Level::Replay,
        }
    }
}

/// The error of a log that could not be written, with the file named: the
/// writer's own message has no path in it.
fn log_error(path: &str, e: LogError) -> Box<dyn std::error::Error> {
    match e {
        LogError::Io(e) => format!("{path}: {e}").into(),
        other => other.into(),
    }
}

/// The files a flight writes besides what the command prints: the CSV and the
/// log. They are opened together, before the first is written to, and a
/// command that fails before its flight removes what it created. A flight
/// that ran keeps its files.
struct Sinks {
    /// The CSV: its path and the writer into it.
    csv: Option<(String, Box<dyn Write + Send>)>,
    /// The log's path, once its file is open.
    log_path: Option<String>,
    /// The log's level and writer until the log is started.
    log_start: Option<(Level, LogWriter)>,
    /// Paths of the files this command created, removed if the command fails
    /// before its flight.
    created: Vec<String>,
    /// The first error of a CSV row, kept until the flight is over.
    csv_error: Option<std::io::Error>,
}

impl Sinks {
    /// No files.
    fn none() -> Self {
        Self {
            csv: None,
            log_path: None,
            log_start: None,
            created: Vec::new(),
            csv_error: None,
        }
    }

    /// Opens the CSV, then the log. When the second cannot be opened the
    /// first is removed again, so a failed command leaves nothing.
    fn open(csv_path: Option<&str>, log: Option<&LogSpec>) -> Result<Self, String> {
        let mut sinks = Self::none();
        if let Some(path) = csv_path {
            let file = std::fs::File::create(path).map_err(|e| format!("{path}: {e}"))?;
            sinks.created.push(path.to_string());
            sinks.csv = Some((
                path.to_string(),
                Box::new(std::io::BufWriter::new(file)) as Box<dyn Write + Send>,
            ));
        }
        if let Some(spec) = log {
            match std::fs::File::create(&spec.path) {
                Ok(file) => {
                    sinks.created.push(spec.path.clone());
                    sinks.log_path = Some(spec.path.clone());
                    sinks.log_start = Some((spec.level(), Box::new(std::io::BufWriter::new(file))));
                }
                Err(e) => {
                    sinks.discard();
                    return Err(format!("{}: {e}", spec.path));
                }
            }
        }
        Ok(sinks)
    }

    /// Removes the regular files this command created. A device such as
    /// `/dev/full` is not one and stays.
    fn discard(&mut self) {
        self.csv = None;
        self.log_start = None;
        for path in self.created.drain(..) {
            if std::fs::metadata(&path).is_ok_and(|m| m.is_file()) {
                let _ = std::fs::remove_file(&path);
            }
        }
    }

    /// Writes the CSV header and starts the log: the last things that can
    /// fail before the flight. On an error the files are removed.
    fn start<P: Pilot>(
        &mut self,
        sim: &mut Simulation<P, 4>,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let result = self.start_inner(sim);
        if result.is_err() {
            self.discard();
        }
        result
    }

    fn start_inner<P: Pilot>(
        &mut self,
        sim: &mut Simulation<P, 4>,
    ) -> Result<(), Box<dyn std::error::Error>> {
        if let Some((path, w)) = self.csv.as_mut() {
            w.write_all(CSV_HEADER)
                .map_err(|e| format!("{path}: {e}"))?;
        }
        if let (Some(path), Some((level, w))) = (self.log_path.as_ref(), self.log_start.take()) {
            sim.start_log(level, &LogBuild::sim(GIT_HASH), w)
                .map_err(|e| log_error(path, e))?;
        }
        Ok(())
    }

    /// One CSV row of the flight. A failure is kept and ends the CSV.
    fn row(&mut self, r: &StepRecord<4>) {
        if self.csv_error.is_some() {
            return;
        }
        if let Some((_, w)) = self.csv.as_mut()
            && let Err(e) = w.write_all(csv_row(r).as_bytes())
        {
            self.csv_error = Some(e);
        }
    }

    /// Ends the CSV and the log after the flight. Both are always finished
    /// (the log keeps what was written); the first error is returned, with the
    /// file's path in it.
    fn finish<P: Pilot>(
        &mut self,
        sim: &mut Simulation<P, 4>,
    ) -> Option<Box<dyn std::error::Error>> {
        let mut first: Option<Box<dyn std::error::Error>> = None;
        if let Some((path, mut w)) = self.csv.take() {
            let err = self.csv_error.take().or_else(|| w.flush().err());
            if let Some(e) = err {
                first = Some(format!("{path}: {e}").into());
            }
        }
        if let Some(path) = self.log_path.take()
            && let Err(e) = sim.finish_log()
        {
            first.get_or_insert_with(|| log_error(&path, e));
        }
        first
    }
}

/// The commands, as `main` dispatches them.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Command {
    Run,
    Hash,
    Montecarlo,
    Scenario,
    Scenarios,
    Replay,
    Params,
}

impl Command {
    /// Every command, in the order the usage text lists them.
    const ALL: [Command; 7] = [
        Command::Run,
        Command::Hash,
        Command::Montecarlo,
        Command::Scenario,
        Command::Scenarios,
        Command::Replay,
        Command::Params,
    ];

    fn name(self) -> &'static str {
        match self {
            Command::Run => "run",
            Command::Hash => "hash",
            Command::Montecarlo => "montecarlo",
            Command::Scenario => "scenario",
            Command::Scenarios => "scenarios",
            Command::Replay => "replay",
            Command::Params => "params",
        }
    }

    fn from_name(name: &str) -> Option<Command> {
        Command::ALL.into_iter().find(|c| c.name() == name)
    }
}

/// The names of all commands, for the error messages: `run | hash | ...`.
fn command_list() -> String {
    let names: Vec<&str> = Command::ALL.iter().map(|c| c.name()).collect();
    names.join(" | ")
}

/// What `oxide-sim help`, `--help` and `-h` print: every command with its
/// options. A line that starts a command begins with `  oxide-sim NAME`.
const USAGE: &str = "usage:
  oxide-sim run [--profile f405|h743] [--airframe FILE] [--csv FILE] [--scenario hover-step|gust-hold]
                [--log FILE.ulg] [--log-level normal|replay] [--seed N] [--estimator truth|ekf] [--sensors ideal|realistic]
                [--params FILE.toml] [--param NAME=VALUE]
  oxide-sim hash [same options as run, except --log and --log-level]
  oxide-sim montecarlo [--profile f405|h743] [--seeds N] [--threads T] [--params FILE.toml] [--param NAME=VALUE]
  oxide-sim scenario list
  oxide-sim scenario NAME|PATH [--profile f405|h743] [--seed N] [--csv FILE] [--log FILE.ulg] [--log-level normal|replay]
                [--params FILE.toml] [--param NAME=VALUE]
  oxide-sim scenarios DIR [--seeds N] [--profile f405|h743|both] [--threads T]
  oxide-sim replay FILE.ulg [--param NAME=VALUE ...] [--continue]
  oxide-sim params list|json|doc
  oxide-sim help | --help | -h
";

#[derive(Clone, Copy, PartialEq, Eq)]
enum ScenarioKind {
    HoverStep,
    GustHold,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum SensorChoice {
    Ideal,
    Realistic,
}

struct Args {
    command: String,
    profile: Profile,
    airframe: Option<String>,
    csv: Option<String>,
    log: Option<LogSpec>,
    scenario: Option<ScenarioKind>,
    seed: u64,
    estimator: Option<EstimatorSource>,
    sensors: Option<SensorChoice>,
    seeds: usize,
    threads: usize,
    /// `--params FILE` then every `--param NAME=VALUE`, in that order.
    params: Vec<(String, f64)>,
}

/// `NAME=VALUE` as given to `--param`.
fn parse_param_flag(text: &str) -> Result<(String, f64), String> {
    let (name, value) = text
        .split_once('=')
        .ok_or_else(|| format!("--param needs NAME=VALUE, got {text}"))?;
    let v: f64 = value
        .trim()
        .parse()
        .map_err(|_| format!("--param {name}: {value} is not a number"))?;
    Ok((name.trim().to_string(), v))
}

/// The overrides of `--params FILE` (if given) followed by those of the
/// `--param` flags, in order.
fn collect_params(
    file: Option<&str>,
    flags: Vec<(String, f64)>,
) -> Result<Vec<(String, f64)>, String> {
    let mut out = Vec::new();
    if let Some(path) = file {
        let text = std::fs::read_to_string(path).map_err(|e| format!("{path}: {e}"))?;
        out = oxide_sim::airframe::parse_params_file(&text).map_err(|e| format!("{path}: {e}"))?;
    }
    out.extend(flags);
    Ok(out)
}

fn parse() -> Result<Args, String> {
    parse_from(std::env::args().skip(1))
}

/// Options `montecarlo` honours; any other would be silently ignored.
const MONTECARLO_OPTIONS: &[&str] = &["--profile", "--seeds", "--threads", "--param", "--params"];

fn parse_from(mut it: impl Iterator<Item = String>) -> Result<Args, String> {
    let command = it
        .next()
        .ok_or_else(|| format!("missing command: {} (try: oxide-sim help)", command_list()))?;
    let mut args = Args {
        command,
        profile: F405_LEAN,
        airframe: None,
        csv: None,
        log: None,
        scenario: None,
        seed: 0,
        estimator: None,
        sensors: None,
        seeds: 200,
        threads: 0,
        params: Vec::new(),
    };
    let mut params_file: Option<String> = None;
    let mut param_flags: Vec<(String, f64)> = Vec::new();
    let mut log_path: Option<String> = None;
    let mut log_level: Option<String> = None;
    let mut given: Vec<String> = Vec::new();
    while let Some(flag) = it.next() {
        given.push(flag.clone());
        let mut value = || it.next().ok_or(format!("{flag} needs a value"));
        match flag.as_str() {
            "--profile" => {
                args.profile = match value()?.as_str() {
                    "f405" => F405_LEAN,
                    "h743" => H743_FULL,
                    other => return Err(format!("unknown profile {other} (f405 | h743)")),
                }
            }
            "--airframe" => args.airframe = Some(value()?),
            "--csv" => args.csv = Some(value()?),
            "--log" => log_path = Some(value()?),
            "--log-level" => log_level = Some(value()?),
            "--scenario" => {
                args.scenario = Some(match value()?.as_str() {
                    "hover-step" => ScenarioKind::HoverStep,
                    "gust-hold" => ScenarioKind::GustHold,
                    other => {
                        return Err(format!("unknown scenario {other} (hover-step | gust-hold)"));
                    }
                });
            }
            "--seed" => {
                let v = value()?;
                args.seed = v
                    .parse()
                    .map_err(|_| format!("--seed needs a non-negative integer, got {v}"))?;
            }
            "--estimator" => {
                args.estimator = Some(match value()?.as_str() {
                    "truth" => EstimatorSource::External,
                    "ekf" => EstimatorSource::Ekf,
                    other => return Err(format!("unknown estimator {other} (truth | ekf)")),
                });
            }
            "--sensors" => {
                args.sensors = Some(match value()?.as_str() {
                    "ideal" => SensorChoice::Ideal,
                    "realistic" => SensorChoice::Realistic,
                    other => return Err(format!("unknown sensors {other} (ideal | realistic)")),
                });
            }
            "--seeds" => {
                let v = value()?;
                args.seeds = v
                    .parse()
                    .map_err(|_| format!("--seeds needs a non-negative integer, got {v}"))?;
                if args.seeds == 0 {
                    return Err("--seeds must be at least 1".into());
                }
            }
            "--threads" => {
                let v = value()?;
                args.threads = v
                    .parse()
                    .map_err(|_| format!("--threads needs a non-negative integer, got {v}"))?;
            }
            "--params" => params_file = Some(value()?),
            "--param" => param_flags.push(parse_param_flag(&value()?)?),
            other => return Err(format!("unknown option {other}")),
        }
    }
    args.params = collect_params(params_file.as_deref(), param_flags)?;
    args.log = LogSpec::from_options(log_path, log_level.as_deref())?;
    if !matches!(
        Command::from_name(&args.command),
        Some(Command::Run | Command::Hash | Command::Montecarlo)
    ) {
        return Err(format!(
            "unknown command {} ({}; try: oxide-sim help)",
            args.command,
            command_list()
        ));
    }
    if args.command == "hash" && args.log.is_some() {
        return Err("--log applies to run and scenario, not to hash".into());
    }
    if args.command == "montecarlo"
        && let Some(bad) = given
            .iter()
            .find(|g| g.starts_with("--") && !MONTECARLO_OPTIONS.contains(&g.as_str()))
    {
        return Err(format!(
            "option {bad} does not apply to montecarlo (only --profile, --seeds, --threads, --param, --params)"
        ));
    }
    Ok(args)
}

fn load_airframe(path: Option<&str>) -> Result<Airframe, SimError> {
    match path {
        Some(p) => {
            let text =
                std::fs::read_to_string(p).map_err(|e| SimError::Airframe(format!("{p}: {e}")))?;
            Airframe::from_toml(&text)
        }
        None => Airframe::reference_quad(),
    }
}

/// Applies the `--estimator` and `--sensors` overrides to a scenario's options.
fn overridden(mut options: SimOptions, args: &Args) -> SimOptions {
    if let Some(e) = args.estimator {
        options.estimator = e;
    }
    options.params.clone_from(&args.params);
    match args.sensors {
        Some(SensorChoice::Ideal) => options.sensors = SensorConfig::ideal(),
        Some(SensorChoice::Realistic) => options.sensors = SensorConfig::realistic(),
        None => {}
    }
    options
}

/// Everything one run needs, and how to name it.
struct Job<P> {
    /// Name used in `hash` output.
    label: String,
    pilot: P,
    options: SimOptions,
    duration_s: f64,
    /// Human-readable target for the run summary.
    target: Option<(Vec3<f64>, f64)>,
}

fn hover_step_job(args: &Args) -> Job<HoverStep> {
    let pilot = HoverStep::default();
    let duration_s = pilot.duration_s;
    Job {
        label: "hover-step".into(),
        pilot,
        options: overridden(SimOptions::default(), args),
        duration_s,
        target: Some((Vec3::new(1.0, 1.0, -3.0), 90.0)),
    }
}

fn gust_hold_job(args: &Args) -> Job<GustHold> {
    let (pilot, options) = gust_hold(args.seed);
    let duration_s = pilot.duration_s;
    let target = Some((
        Vec3::new(
            f64::from(pilot.hold.x),
            f64::from(pilot.hold.y),
            f64::from(pilot.hold.z),
        ),
        f64::from(pilot.yaw).to_degrees(),
    ));
    Job {
        label: format!("gust-hold#{}", args.seed),
        pilot,
        options: overridden(options, args),
        duration_s,
        target,
    }
}

/// Error of the published estimate against truth.
#[derive(Clone, Copy, Default)]
struct EstimateErrors {
    samples: u64,
    /// An estimate or truth value was NaN or infinite at least once.
    non_finite: bool,
    max_horizontal_m: f64,
    max_vertical_m: f64,
    max_attitude_rad: f64,
}

impl EstimateErrors {
    fn update(&mut self, r: &StepRecord<4>) {
        let Some(est) = r.estimate() else { return };
        let p = est.position.cast::<f64>();
        let (dn, de) = (p.x - r.position.x, p.y - r.position.y);
        let dh = oxide_core::math::Real::sqrt(dn * dn + de * de);
        let dv = (p.z - r.position.z).abs();
        let dq = est.attitude.cast::<f64>() * r.attitude.conj();
        let da = dq.to_rotation_vector().norm();
        if dh.is_finite() && dv.is_finite() && da.is_finite() {
            self.samples += 1;
            self.max_horizontal_m = self.max_horizontal_m.max(dh);
            self.max_vertical_m = self.max_vertical_m.max(dv);
            self.max_attitude_rad = self.max_attitude_rad.max(da);
        } else {
            self.non_finite = true;
        }
    }
}

fn csv_row(r: &StepRecord<4>) -> String {
    let e = r.attitude.to_euler();
    let sp = r.setpoint.map(|s| s.position);
    let f = |v: Option<f32>| v.map_or(String::new(), |x| format!("{x:.4}"));
    let est = r.estimate();
    // The core's columns are empty cells for a record without the core part.
    let status = r.status();
    let nav = status.map_or(String::new(), |s| format!("{:?}", s.nav));
    let action = status.map_or(String::new(), |s| format!("{:?}", s.action));
    let reasons = status.map_or(String::new(), |s| s.reasons.0.to_string());
    let est_pos = est.map(|s| s.position);
    let est_euler = est.map(|s| s.attitude.cast::<f64>().to_euler());
    let g = |v: Option<f32>| v.map_or(String::new(), |x| format!("{x:.5}"));
    let h = |v: Option<f64>| v.map_or(String::new(), |x| format!("{x:.5}"));
    format!(
        "{:.4},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.4},{:.4},{:.4},{:.4},{:.3},{:.3},{},{},{},{},{},{},{},{},{},{},{:.4},{:.4},{:.4},{},{},{},{},{},{}\n",
        r.t.as_secs_f64(),
        r.position.x,
        r.position.y,
        r.position.z,
        r.velocity.x,
        r.velocity.y,
        r.velocity.z,
        e.x,
        e.y,
        e.z,
        r.rate.x,
        r.rate.y,
        r.rate.z,
        r.motors[0],
        r.motors[1],
        r.motors[2],
        r.motors[3],
        r.battery_v,
        r.battery_a,
        u8::from(r.armed),
        f(sp.map(|p| p.x)),
        f(sp.map(|p| p.y)),
        f(sp.map(|p| p.z)),
        g(est_pos.map(|p| p.x)),
        g(est_pos.map(|p| p.y)),
        g(est_pos.map(|p| p.z)),
        h(est_euler.map(|v| v.x)),
        h(est_euler.map(|v| v.y)),
        h(est_euler.map(|v| v.z)),
        r.wind.x,
        r.wind.y,
        r.wind.z,
        nav,
        action,
        g(est.map(|s| s.velocity.x)),
        g(est.map(|s| s.velocity.y)),
        g(est.map(|s| s.velocity.z)),
        reasons,
    )
}

const CSV_HEADER: &[u8] = b"t,x,y,z,vx,vy,vz,roll,pitch,yaw,p,q,r,m0,m1,m2,m3,battery_v,battery_a,armed,sp_x,sp_y,sp_z,est_x,est_y,est_z,est_roll,est_pitch,est_yaw,wind_n,wind_e,wind_d,nav,action,est_vx,est_vy,est_vz,reasons\n";

/// What a finished run reports.
struct Outcome {
    label: String,
    hash: u64,
    sim_s: f64,
    wall_s: f64,
    final_position: Vec3<f64>,
    final_yaw_deg: f64,
    max_tilt: f64,
    errors: EstimateErrors,
    target: Option<(Vec3<f64>, f64)>,
}

fn execute<P: Pilot>(
    airframe: &Airframe,
    profile: Profile,
    job: Job<P>,
    csv_path: Option<&str>,
    log: Option<&LogSpec>,
) -> Result<Outcome, Box<dyn std::error::Error>> {
    let duration_us = (job.duration_s * 1e6) as u64;
    let mut sim = Simulation::<_, 4>::with_options(airframe, profile, job.pilot, job.options)?;
    let mut sinks = Sinks::open(csv_path, log)?;
    sinks.start(&mut sim)?;
    let mut max_tilt = 0.0_f64;
    let mut errors = EstimateErrors::default();
    let start = Instant::now();
    sim.run_until(Micros(duration_us), |r| {
        let e = r.attitude.to_euler();
        max_tilt = max_tilt.max(e.x.abs()).max(e.y.abs());
        errors.update(r);
        sinks.row(r);
    });
    let wall_s = start.elapsed().as_secs_f64();
    if let Some(e) = sinks.finish(&mut sim) {
        return Err(e);
    }
    let b = sim.body();
    Ok(Outcome {
        label: job.label,
        hash: sim.trace_hash(),
        sim_s: sim.time().as_secs_f64(),
        wall_s,
        final_position: b.position,
        final_yaw_deg: b.attitude.yaw().to_degrees(),
        max_tilt,
        errors,
        target: job.target,
    })
}

/// Fault scenarios whose trace hash `hash` prints (run 0): an Rtl and a
/// landing on the ground model, a blind landing, and a termination and crash.
const HASHED_FAULT_SCENARIOS: [&str; 3] =
    ["rc-loss-position", "pair-gps-loss+baro-loss", "motor-loss"];

/// `<profile> <name>#0 <hash>` for each of `HASHED_FAULT_SCENARIOS`.
#[cfg(test)]
fn fault_hash_lines(profile: Profile) -> Result<Vec<String>, Box<dyn std::error::Error>> {
    fault_hash_lines_with(profile, &[])
}

/// Like [`fault_hash_lines`], with parameter overrides.
fn fault_hash_lines_with(
    profile: Profile,
    params: &[(String, f64)],
) -> Result<Vec<String>, Box<dyn std::error::Error>> {
    let mut lines = Vec::new();
    for name in HASHED_FAULT_SCENARIOS {
        let scenario = fault_scenarios::by_name(name)
            .ok_or_else(|| format!("hashed fault scenario {name} does not exist"))?;
        let mut sim = scenario.build_with_params(profile, 0, params)?;
        sim.run_until(Micros((scenario.duration_s * 1e6).round() as u64), |_| {});
        lines.push(format!(
            "{} {}#0 {:016x}",
            profile.name,
            name,
            sim.trace_hash()
        ));
    }
    Ok(lines)
}

/// The flights whose normal-level log `hash` hashes (run 0).
const HASHED_LOGGED_FLIGHTS: [&str; 2] = ["rc-loss-position", "gust-hold"];

/// The scenarios whose replay-level log `hash` also prints (`log-replay`).
const HASHED_REPLAY_FLIGHTS: [&str; 1] = ["rc-loss-position"];

/// What `hash` writes into the logs it hashes: the same bytes on every commit.
const HASH_BUILD: &str = "hash";

/// FNV-1a 64 of `bytes`: the hash of the trace hash's family
/// (`oxide_sim::hash::TraceHash`).
fn fnv1a_64(bytes: &[u8]) -> u64 {
    let mut h = oxide_sim::hash::TraceHash::default();
    h.write_bytes(bytes);
    h.value()
}

/// The bytes of the log at `level` of scenario `name`, run `index`, flown to
/// its end with `params`, written with the fixed build information
/// (`ver_sw` `hash`, `ver_hw` `SIM`).
fn logged_flight_bytes_at(
    profile: Profile,
    level: Level,
    name: &str,
    index: u64,
    params: &[(String, f64)],
) -> Result<Vec<u8>, Box<dyn std::error::Error>> {
    let scenario = fault_scenarios::by_name(name)
        .ok_or_else(|| format!("logged scenario {name} does not exist"))?;
    let mut sim = scenario.build_with_params(profile, index, params)?;
    let buffer = oxide_sim::flight_log::SharedBuffer::new();
    sim.start_log(level, &LogBuild::sim(HASH_BUILD), Box::new(buffer.clone()))?;
    sim.run_until(scenario.end(), |_| {});
    sim.finish_log()?;
    Ok(buffer.bytes())
}

/// The bytes of the normal-level log of scenario `name`, run `index`.
fn logged_flight_bytes(
    profile: Profile,
    name: &str,
    index: u64,
    params: &[(String, f64)],
) -> Result<Vec<u8>, Box<dyn std::error::Error>> {
    logged_flight_bytes_at(profile, Level::Normal, name, index, params)
}

/// `<profile> log-normal <name>#0 <fnv-1a 64 of the file>` for each of
/// `HASHED_LOGGED_FLIGHTS`.
fn log_hash_lines(
    profile: Profile,
    params: &[(String, f64)],
) -> Result<Vec<String>, Box<dyn std::error::Error>> {
    let mut lines = Vec::new();
    for name in HASHED_LOGGED_FLIGHTS {
        let bytes = logged_flight_bytes(profile, name, 0, params)?;
        lines.push(format!(
            "{} log-normal {}#0 {:016x}",
            profile.name,
            name,
            fnv1a_64(&bytes)
        ));
    }
    Ok(lines)
}

/// `<profile> log-replay <name>#0 <fnv-1a 64 of the file>` for each of
/// `HASHED_REPLAY_FLIGHTS`: the replay-level log, whose bytes also pin the
/// order and content of every replay record.
fn log_replay_hash_lines(
    profile: Profile,
    params: &[(String, f64)],
) -> Result<Vec<String>, Box<dyn std::error::Error>> {
    let mut lines = Vec::new();
    for name in HASHED_REPLAY_FLIGHTS {
        let bytes = logged_flight_bytes_at(profile, Level::Replay, name, 0, params)?;
        lines.push(format!(
            "{} log-replay {}#0 {:016x}",
            profile.name,
            name,
            fnv1a_64(&bytes)
        ));
    }
    Ok(lines)
}

/// Every line `hash` prints for the profile of `args`.
fn hash_lines(args: &Args, airframe: &Airframe) -> Result<Vec<String>, Box<dyn std::error::Error>> {
    let profile = args.profile;
    let kinds: &[ScenarioKind] = match args.scenario {
        Some(ScenarioKind::HoverStep) => &[ScenarioKind::HoverStep],
        Some(ScenarioKind::GustHold) => &[ScenarioKind::GustHold],
        // Both, with default options: covers the RNG, noise and libm paths.
        None => &[ScenarioKind::HoverStep, ScenarioKind::GustHold],
    };
    let mut lines = Vec::new();
    for &kind in kinds {
        let o = match kind {
            ScenarioKind::HoverStep => execute(
                airframe,
                profile,
                hover_step_job(args),
                args.csv.as_deref(),
                None,
            ),
            ScenarioKind::GustHold => execute(
                airframe,
                profile,
                gust_hold_job(args),
                args.csv.as_deref(),
                None,
            ),
        }?;
        lines.push(format!("{} {} {:016x}", profile.name, o.label, o.hash));
    }
    if args.scenario.is_none() {
        lines.extend(fault_hash_lines_with(profile, &args.params)?);
        lines.extend(log_hash_lines(profile, &args.params)?);
        lines.extend(log_replay_hash_lines(profile, &args.params)?);
    }
    Ok(lines)
}

fn run(args: &Args) -> Result<(), Box<dyn std::error::Error>> {
    let airframe = load_airframe(args.airframe.as_deref())?;
    let profile = args.profile;
    let log = args.log.as_ref();
    let run_one = |kind: ScenarioKind, csv: Option<&str>| match kind {
        ScenarioKind::HoverStep => execute(&airframe, profile, hover_step_job(args), csv, log),
        ScenarioKind::GustHold => execute(&airframe, profile, gust_hold_job(args), csv, log),
    };

    if args.command == "hash" {
        for line in hash_lines(args, &airframe)? {
            println!("{line}");
        }
        return Ok(());
    }

    let o = run_one(
        args.scenario.unwrap_or(ScenarioKind::HoverStep),
        args.csv.as_deref(),
    )?;
    println!("profile          {}", profile.name);
    println!("airframe         {}", airframe.name);
    println!("scenario         {}", o.label);
    println!(
        "simulated        {:.1} s in {:.3} s wall ({:.0}x real time)",
        o.sim_s,
        o.wall_s,
        o.sim_s / o.wall_s.max(1e-9)
    );
    let p = o.final_position;
    if let Some((tp, tyaw)) = o.target {
        println!(
            "final position   N {:.3}  E {:.3}  D {:.3} m (target N {:.3}  E {:.3}  D {:.3})",
            p.x, p.y, p.z, tp.x, tp.y, tp.z
        );
        println!(
            "final yaw        {:.2} deg (target {tyaw:.2})",
            o.final_yaw_deg
        );
    }
    println!("max tilt         {:.1} deg", o.max_tilt.to_degrees());
    if o.errors.non_finite {
        println!("estimate error   NON-FINITE estimate seen");
    }
    if o.errors.samples > 0 {
        println!(
            "estimate error   max horizontal {:.3} m, max vertical {:.3} m, max attitude {:.2} deg",
            o.errors.max_horizontal_m,
            o.errors.max_vertical_m,
            o.errors.max_attitude_rad.to_degrees()
        );
    }
    println!("trace hash       {:016x}", o.hash);
    Ok(())
}

/// A number from the core part of a run's hold metrics (0 without it).
fn core_hold(m: &HoldMetrics, f: fn(&CoreHold) -> f64) -> f64 {
    m.core.as_ref().map_or(0.0, f)
}

/// Flies gust-hold indices `0..seeds`; true when every run passed.
fn montecarlo(args: &Args) -> Result<bool, Box<dyn std::error::Error>> {
    let c = HoldCriteria::default();
    let profile = args.profile;
    let start = Instant::now();
    let results = run_indexed(args.seeds, args.threads, |i| {
        fly_gust_hold_with_params(profile, i as u64, &c, &args.params)
    });
    let wall_s = start.elapsed().as_secs_f64();
    let mut runs: Vec<HoldMetrics> = Vec::with_capacity(results.len());
    for r in results {
        runs.push(r?);
    }
    let mut passed = 0usize;
    for (i, m) in runs.iter().enumerate() {
        let j = m.judge(&c);
        if j.passed() {
            passed += 1;
        } else {
            let mut f = j.violations;
            f.extend(j.not_judged.iter().map(|n| format!("not judged: {n}")));
            println!("FAIL index {i}: {}", f.join("; "));
        }
    }
    println!("profile {}, {} runs", profile.name, runs.len());
    let worst = |name: &str, f: fn(&HoldMetrics) -> f64| {
        let (i, v) = runs
            .iter()
            .enumerate()
            .map(|(i, m)| (i, f(m)))
            .fold((0, f64::MIN), |a, b| if b.1 > a.1 { b } else { a });
        println!("  worst {name:<32} {v:9.4} at index {i}");
    };
    if !runs.is_empty() {
        worst("max horizontal error (m)", |m| m.max_h_error_m);
        worst("rms horizontal error (m)", |m| m.rms_h_error_m);
        worst("max vertical error (m)", |m| m.max_v_error_m);
        worst("max tilt (deg)", |m| m.max_tilt_deg);
        worst("estimate max horizontal (m)", |m| {
            core_hold(m, |k| k.est_max_h_error_m)
        });
        worst("estimate max vertical (m)", |m| {
            core_hold(m, |k| k.est_max_v_error_m)
        });
        worst("estimate max velocity (m/s)", |m| {
            core_hold(m, |k| k.est_max_vel_error_mps)
        });
        worst("estimate max attitude (deg)", |m| {
            core_hold(m, |k| k.est_max_att_error_deg)
        });
        worst("rejected measurements", |m| {
            core_hold(m, |k| f64::from(k.rejected))
        });
        worst("physics faults", |m| f64::from(m.physics_faults));
    }
    println!("passed {passed}/{}", runs.len());
    println!("wall time {wall_s:.1} s");
    Ok(passed == runs.len())
}

/// `oxide-sim params list | json | doc`.
fn params_command(mut it: impl Iterator<Item = String>) -> Result<(), Box<dyn std::error::Error>> {
    let what = it.next().ok_or("params needs list, json or doc")?;
    if let Some(extra) = it.next() {
        return Err(format!("unexpected argument {extra}").into());
    }
    let text = match what.as_str() {
        "list" => params_cmd::list()?,
        "json" => params_cmd::json(),
        "doc" => params_cmd::doc(),
        other => return Err(format!("unknown params command {other} (list | json | doc)").into()),
    };
    std::io::stdout().write_all(text.as_bytes())?;
    Ok(())
}

/// The scenario `arg` names: a path when it ends in `.toml` or has a path
/// separator, else one of the embedded scenarios.
fn resolve_scenario(arg: &str) -> Result<ScenarioFile, Box<dyn std::error::Error>> {
    if arg.ends_with(".toml") || arg.contains('/') || arg.contains('\\') {
        return Ok(scenario_file::load_path(arg)?);
    }
    fault_scenarios::by_name(arg).ok_or_else(|| {
        format!(
            "unknown scenario {arg} (try: oxide-sim scenario list; a file is given by its path, e.g. ./{arg}.toml)"
        )
        .into()
    })
}

/// `oxide-sim scenario list | NAME|PATH [--profile P] [--seed N] [--csv FILE]
/// [--log FILE.ulg] [--log-level normal|replay] [--params FILE] [--param NAME=VALUE]`.
fn scenario_command(
    mut it: impl Iterator<Item = String>,
) -> Result<bool, Box<dyn std::error::Error>> {
    let first = it
        .next()
        .ok_or("scenario needs list, a scenario name or the path of a scenario file")?;
    if first == "list" {
        if let Some(extra) = it.next() {
            return Err(format!("unexpected argument {extra} after list").into());
        }
        for s in fault_scenarios::embedded() {
            println!("{:<34} {}", s.name, s.about);
        }
        return Ok(true);
    }
    let scenario = resolve_scenario(&first)?;
    let (mut profile, mut index, mut csv_path) = (F405_LEAN, 0_u64, None);
    let (mut log_path, mut log_level): (Option<String>, Option<String>) = (None, None);
    let mut params_file: Option<String> = None;
    let mut param_flags: Vec<(String, f64)> = Vec::new();
    while let Some(flag) = it.next() {
        let mut value = || it.next().ok_or(format!("{flag} needs a value"));
        match flag.as_str() {
            "--profile" => {
                profile = match value()?.as_str() {
                    "f405" => F405_LEAN,
                    "h743" => H743_FULL,
                    other => return Err(format!("unknown profile {other} (f405 | h743)").into()),
                }
            }
            "--seed" => {
                let v = value()?;
                index = v
                    .parse()
                    .map_err(|_| format!("--seed needs a non-negative integer, got {v}"))?;
            }
            "--csv" => csv_path = Some(value()?),
            "--log" => log_path = Some(value()?),
            "--log-level" => log_level = Some(value()?),
            "--params" => params_file = Some(value()?),
            "--param" => param_flags.push(parse_param_flag(&value()?)?),
            other => return Err(format!("unknown option {other}").into()),
        }
    }
    let params = collect_params(params_file.as_deref(), param_flags)?;
    let log = LogSpec::from_options(log_path, log_level.as_deref())?;
    let sim = scenario.build_with_params(profile, index, &params)?;
    let sinks = Sinks::open(csv_path.as_deref(), log.as_ref())?;
    let run = fly_scenario(&scenario, profile, index, sim, sinks)?;
    for line in &run.lines {
        println!("{line}");
    }
    match run.write_error {
        Some(e) => Err(e),
        None => Ok(run.passed),
    }
}

/// What a flown scenario reports.
struct ScenarioRun {
    /// The summary, the timeline and the verdict, as the command prints them.
    lines: Vec<String>,
    /// Every expectation held.
    passed: bool,
    /// A CSV or log file that could not be written, found after the flight:
    /// the command prints `lines` and then fails with this (exit 2).
    write_error: Option<Box<dyn std::error::Error>>,
}

/// Flies `scenario` on `sim` into `sinks` and judges it. An error here is one
/// before the flight (the files are removed); a write error after it is in
/// the result, so that the verdict is still shown.
fn fly_scenario<P: Pilot>(
    scenario: &ScenarioFile,
    profile: Profile,
    index: u64,
    mut sim: Simulation<P, 4>,
    mut sinks: Sinks,
) -> Result<ScenarioRun, Box<dyn std::error::Error>> {
    sinks.start(&mut sim)?;
    let mut checker = Checker::new(scenario);
    sim.run_until(scenario.end(), |r| {
        checker.add(r);
        sinks.row(r);
    });
    let write_error = sinks.finish(&mut sim);
    let verdict = checker.finish();
    let outcome = &verdict.outcome;
    let mut lines = Vec::new();
    lines.push(format!(
        "scenario   {} (run {index}, profile {})",
        scenario.name, profile.name
    ));
    lines.push(format!("about      {}", scenario.about));
    lines.push("timeline".to_string());
    let core = outcome.core.as_ref();
    for e in core.iter().flat_map(|c| &c.events) {
        lines.push(format!("  {}", oxide_sim::outcome::event_line(e)));
    }
    let p = outcome.position_at_end;
    // Without the core part there is no status to print: the lines say so.
    let unavailable = || format!("({})", oxide_sim::outcome::NEEDS_CORE);
    lines.push(format!(
        "navigation {}",
        core.map_or_else(unavailable, |c| format!("{:?}", c.nav_sequence))
    ));
    lines.push(match core {
        Some(c) => format!(
            "reasons    {}   highest action {:?}",
            oxide_sim::outcome::reason_names(c.reasons_seen),
            c.max_action
        ),
        None => format!("reasons    {}", unavailable()),
    });
    lines.push(format!(
        "end        t {:.2} s, N {:.2} E {:.2} D {:.2} m, armed {}, {}",
        outcome.end_s,
        p.x,
        p.y,
        p.z,
        outcome.armed_at_end,
        core.map_or_else(unavailable, |c| format!("{:?}", c.nav_at_end))
    ));
    lines.push(format!(
        "limits     max height {:.2} m, max distance from home {:.2} m, max tilt {:.1} deg, touchdown {}",
        outcome.max_height_m,
        outcome.max_distance_from_home_m,
        outcome.max_tilt_deg,
        outcome
            .touchdown_speed_mps
            .map_or("none".into(), |v| format!("{v:.2} m/s"))
    ));
    lines.push(format!(
        "disarmed   {} at {}",
        core.and_then(|c| c.disarm_reason)
            .map_or("never".into(), |r| format!("{r:?}")),
        outcome
            .disarmed_at_s
            .map_or("-".into(), |t| format!("{t:.2} s"))
    ));
    if let Some(h) = &verdict.hold {
        lines.push(hold_line(h));
    }
    lines.push(format!("trace hash {:016x}", sim.trace_hash()));
    lines.extend(result_lines(
        scenario,
        &verdict.violations,
        &verdict.not_judged,
    ));
    Ok(ScenarioRun {
        lines,
        passed: verdict.passed(),
        write_error,
    })
}

/// The `hold` line of a scenario summary. The estimate's error is the core's:
/// without the core part the line says so instead of printing a number.
fn hold_line(h: &HoldMetrics) -> String {
    let estimate = h.core.map_or_else(
        || format!("({})", oxide_sim::outcome::NEEDS_CORE),
        |k| format!("{:.3} m", k.est_max_h_error_m),
    );
    format!(
        "hold       max horizontal {:.3} m, rms {:.3} m, max vertical {:.3} m, estimate max horizontal {estimate}",
        h.max_h_error_m, h.rms_h_error_m, h.max_v_error_m
    )
}

/// Replays the log at `path` with the parameter overrides given, and returns
/// the report with the lines the command prints for it.
fn replay_file_report(
    path: &str,
    params: &[(String, f64)],
    keep_going: bool,
) -> Result<(oxide_sim::replay::ReplayReport, Vec<String>), Box<dyn std::error::Error>> {
    let bytes = std::fs::read(path).map_err(|e| format!("{path}: {e}"))?;
    let options = oxide_sim::replay::ReplayOptions {
        params: params.to_vec(),
        keep_going,
    };
    let report = oxide_sim::replay::replay(&bytes, &options).map_err(|e| format!("{path}: {e}"))?;
    let lines = report.lines();
    Ok((report, lines))
}

/// `oxide-sim replay FILE.ulg [--param NAME=VALUE ...] [--continue]`: rebuilds
/// the flight core from a replay-level log and flies the recorded ticks
/// through it, comparing every output bit for bit (ADR-0005 §5). Prints the
/// number of ticks replayed and `identical`, or the first difference (tick,
/// field, recorded and replayed values); `--continue` goes on and counts the
/// ticks that differ. The status is the command's exit code: `Ok(true)` (0)
/// when every tick replayed is identical, also when the file ends in a cut;
/// `Ok(false)` (1) on a difference, a gap, a missing tick or a malformed one,
/// damage at the end of the file, or a file with no tick; an error (2) for a
/// file that cannot be replayed.
fn replay_command(
    mut it: impl Iterator<Item = String>,
) -> Result<bool, Box<dyn std::error::Error>> {
    let path = it
        .next()
        .ok_or("replay needs the path of a replay-level log")?;
    if path.starts_with("--") {
        return Err(format!("replay needs the path of a log first, got {path}").into());
    }
    let mut params: Vec<(String, f64)> = Vec::new();
    let mut keep_going = false;
    while let Some(flag) = it.next() {
        match flag.as_str() {
            "--param" => {
                let v = it.next().ok_or("--param needs NAME=VALUE")?;
                params.push(parse_param_flag(&v)?);
            }
            "--continue" => keep_going = true,
            other if other.starts_with("--") => {
                return Err(format!("unknown option {other}").into());
            }
            other => return Err(format!("unexpected argument {other}").into()),
        }
    }
    let (report, lines) = replay_file_report(&path, &params, keep_going)?;
    for line in lines {
        println!("{line}");
    }
    Ok(report.exit_code() == 0)
}

/// The `result` line of `scenario` and the violations under it. A file that
/// states no expectations is judged by the invariants, and says so.
fn result_lines(
    scenario: &ScenarioFile,
    violations: &[String],
    not_judged: &[oxide_sim::outcome::NotJudged],
) -> Vec<String> {
    let note = if scenario.states_expectations() {
        ""
    } else {
        " (invariants only: the file states no expectations)"
    };
    let mut out = Vec::new();
    if !violations.is_empty() {
        out.push(format!("result     FAIL{note}"));
        out.extend(violations.iter().map(|v| format!("  violated: {v}")));
    } else if !not_judged.is_empty() {
        // Nothing violated, but not everything was answered: not a pass.
        out.push(format!(
            "result     NOT JUDGED ({} requirement{} could not be judged){note}",
            not_judged.len(),
            if not_judged.len() == 1 { "" } else { "s" }
        ));
    } else {
        out.push(format!("result     PASS{note}"));
    }
    out.extend(not_judged.iter().map(|n| format!("  not judged: {n}")));
    out
}

/// Maps a command's result to the exit status: 0 passed, 1 a failing run, 2 an
/// error.
fn finish(r: Result<bool, Box<dyn std::error::Error>>) -> ExitCode {
    match r {
        Ok(true) => ExitCode::SUCCESS,
        Ok(false) => ExitCode::FAILURE,
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::from(2)
        }
    }
}

fn main() -> ExitCode {
    let first = std::env::args().nth(1);
    if matches!(first.as_deref(), Some("help" | "--help" | "-h")) {
        print!("{USAGE}");
        return ExitCode::SUCCESS;
    }
    match first.as_deref().and_then(Command::from_name) {
        Some(Command::Params) => {
            return finish(params_command(std::env::args().skip(2)).map(|()| true));
        }
        Some(Command::Scenarios) => {
            return finish(scenarios_cmd::run(std::env::args().skip(2)));
        }
        Some(Command::Scenario) => {
            return finish(scenario_command(std::env::args().skip(2)));
        }
        Some(Command::Replay) => {
            return finish(replay_command(std::env::args().skip(2)));
        }
        Some(Command::Run | Command::Hash | Command::Montecarlo) | None => {}
    }
    let args = match parse() {
        Ok(a) => a,
        Err(e) => {
            eprintln!("error: {e}");
            eprint!("{USAGE}");
            return ExitCode::from(2);
        }
    };
    if args.command == "montecarlo" {
        return finish(montecarlo(&args));
    }
    finish(run(&args).map(|()| true))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p(args: &[&str]) -> Result<Args, String> {
        parse_from(args.iter().map(|s| (*s).to_string()))
    }

    #[test]
    fn the_usage_names_exactly_the_commands_main_dispatches() {
        let mut in_usage: Vec<&str> = USAGE
            .lines()
            .filter_map(|l| l.strip_prefix("  oxide-sim "))
            .filter_map(|l| l.split_whitespace().next())
            .collect();
        in_usage.sort_unstable();
        in_usage.dedup();
        // `help` is the usage itself, not a command with options.
        in_usage.retain(|c| *c != "help");
        let mut dispatched: Vec<&str> = Command::ALL.iter().map(|c| c.name()).collect();
        dispatched.sort_unstable();
        assert_eq!(in_usage, dispatched);
        for c in Command::ALL {
            assert_eq!(Command::from_name(c.name()), Some(c));
        }
        assert_eq!(Command::from_name("help"), None);
        assert!(USAGE.contains("--help") && USAGE.contains(" -h"));
    }

    #[test]
    fn missing_and_unknown_command_messages_list_every_command() {
        let missing = p(&[]).err().expect("error");
        let unknown = p(&["frobnicate"]).err().expect("error");
        for c in Command::ALL {
            assert!(missing.contains(c.name()), "{missing}");
            assert!(unknown.contains(c.name()), "{unknown}");
        }
        assert!(unknown.contains("frobnicate"), "{unknown}");
    }

    #[test]
    fn montecarlo_rejects_zero_seeds() {
        let e = p(&["montecarlo", "--seeds", "0"]).err().expect("error");
        assert!(e.contains("--seeds"), "{e}");
    }

    #[test]
    fn montecarlo_rejects_options_it_ignores() {
        let e = p(&["montecarlo", "--estimator", "truth", "--sensors", "ideal"])
            .err()
            .expect("error");
        assert!(e.contains("--estimator"), "{e}");
        let e = p(&["montecarlo", "--seed", "3"]).err().expect("error");
        assert!(e.contains("--seed"), "{e}");
    }

    #[test]
    fn montecarlo_accepts_its_own_options() {
        let a = p(&[
            "montecarlo",
            "--profile",
            "h743",
            "--seeds",
            "5",
            "--threads",
            "2",
        ])
        .expect("valid");
        assert_eq!((a.seeds, a.threads), (5, 2));
        assert_eq!(a.profile.name, H743_FULL.name);
    }

    #[test]
    fn run_still_accepts_estimator_and_sensors() {
        assert!(p(&["run", "--estimator", "truth", "--sensors", "ideal"]).is_ok());
    }

    #[test]
    fn scenario_command_lists_and_rejects_unknown_names() {
        let args = |a: &[&str]| a.iter().map(|s| (*s).to_string()).collect::<Vec<_>>();
        assert!(scenario_command(args(&["list"]).into_iter()).is_ok());
        // Nothing may follow `list`.
        for extra in [
            &["list", "--seed", "3"][..],
            &["list", "x"],
            &["list", "--bogus"],
        ] {
            assert!(
                scenario_command(args(extra).into_iter()).is_err(),
                "{extra:?}"
            );
        }
        assert!(scenario_command(args(&["no-such-scenario"]).into_iter()).is_err());
        assert!(scenario_command(args(&["gcs-loss", "--bogus"]).into_iter()).is_err());
        assert!(scenario_command(args(&["gcs-loss", "--seed", "x"]).into_iter()).is_err());
        assert!(scenario_command(std::iter::empty()).is_err());
    }

    #[test]
    fn hash_covers_landings_and_faults_on_both_profiles() {
        for profile in [F405_LEAN, H743_FULL] {
            let lines = fault_hash_lines(profile).expect("fault hashes");
            assert_eq!(lines.len(), 3);
            for (line, name) in lines.iter().zip(HASHED_FAULT_SCENARIOS) {
                let parts: Vec<&str> = line.split(' ').collect();
                assert_eq!(parts.len(), 3, "{line}");
                assert_eq!(parts[0], profile.name);
                assert_eq!(parts[1], format!("{name}#0"));
                assert_eq!(parts[2].len(), 16, "{line}");
                assert!(u64::from_str_radix(parts[2], 16).is_ok(), "{line}");
            }
            // Deterministic, and the three flights differ.
            assert_eq!(lines, fault_hash_lines(profile).expect("again"));
            assert_ne!(lines[0], lines[1]);
            assert_ne!(lines[1], lines[2]);
        }
    }

    fn temp_file(name: &str, text: &str) -> String {
        let path = std::env::temp_dir().join(format!("oxide-sim-{}-{name}", std::process::id()));
        std::fs::write(&path, text).expect("write temp file");
        path.to_string_lossy().into_owned()
    }

    fn hover_hash(a: &[&str]) -> Result<u64, String> {
        let mut v = vec!["hash"];
        v.extend_from_slice(a);
        let args = p(&v)?;
        let airframe = Airframe::reference_quad().map_err(|e| e.to_string())?;
        execute(&airframe, args.profile, hover_step_job(&args), None, None)
            .map(|o| o.hash)
            .map_err(|e| e.to_string())
    }

    #[test]
    fn param_flags_parse_and_reject_bad_text() {
        let a = p(&["run", "--param", "POS_X_P=1.2", "--param", "RTL_ALT=12"]).expect("valid");
        assert_eq!(
            a.params,
            vec![("POS_X_P".to_string(), 1.2), ("RTL_ALT".to_string(), 12.0)]
        );
        assert!(p(&["run", "--param", "POS_X_P"]).is_err());
        assert!(p(&["run", "--param", "POS_X_P=fast"]).is_err());
        assert!(p(&["run", "--param"]).is_err());
        assert!(p(&["run", "--params", "/no/such/file.toml"]).is_err());
    }

    #[test]
    fn montecarlo_accepts_param_options() {
        assert!(p(&["montecarlo", "--seeds", "2", "--param", "POS_X_P=1.2"]).is_ok());
    }

    #[test]
    fn an_override_changes_the_hash_and_none_does_not() {
        let base = hover_hash(&[]).expect("base");
        let one = hover_hash(&["--param", "POS_X_P=1.2"]).expect("override");
        assert_ne!(base, one);
        assert_eq!(one, hover_hash(&["--param", "POS_X_P=1.2"]).expect("again"));
        // The default value of a parameter is the default flight.
        assert_eq!(
            base,
            hover_hash(&["--param", "POS_X_P=1.5"]).expect("default value")
        );
    }

    #[test]
    fn a_pair_valid_only_together_is_accepted_from_flags_in_either_order() {
        let crit_first = hover_hash(&["--param", "BAT_CRIT_V=3.6", "--param", "BAT_LOW_V=3.8"])
            .expect("critical first");
        let low_first = hover_hash(&["--param", "BAT_LOW_V=3.8", "--param", "BAT_CRIT_V=3.6"])
            .expect("low first");
        // A hover-step never reaches either threshold, so the flight is the
        // default one: what matters is that the run starts.
        assert_eq!(crit_first, low_first);
        assert_eq!(crit_first, hover_hash(&[]).expect("base"));
        // A file and a flag form one change as well.
        let file = temp_file("pair.toml", "[params]\nBAT_CRIT_V = 3.6\n");
        assert!(hover_hash(&["--params", &file, "--param", "BAT_LOW_V=3.8"]).is_ok());
        let _ = std::fs::remove_file(&file);
        // Together they must still hold. (3.45 and 3.4 are each fine against
        // the defaults; a critical level of 3.9 breaks the rule alone and is
        // blamed by name instead, see `a_single_bad_value_is_blamed_alone`.)
        let e = hover_hash(&["--param", "BAT_LOW_V=3.4", "--param", "BAT_CRIT_V=3.45"])
            .expect_err("critical above low");
        assert!(e.contains("BAT_CRIT_V") && e.contains("together"), "{e}");
    }

    #[test]
    fn a_single_bad_value_is_blamed_alone() {
        let e = hover_hash(&["--param", "BAT_LOW_V=3.8", "--param", "BAT_CRIT_V=3.9"])
            .expect_err("critical above the default low level");
        assert!(e.contains("BAT_CRIT_V") && e.contains("value 3.9"), "{e}");
        assert!(!e.contains("BAT_LOW_V") && !e.contains("together"), "{e}");
    }

    #[test]
    fn each_error_case_prints_its_message() {
        for (flag, name, why) in [
            ("NOPE=1", "NOPE", "no such parameter"),
            ("EKF_SOURCE=0.5", "EKF_SOURCE", "whole number"),
            ("POS_X_P=1e9", "POS_X_P", "range"),
            ("BAT_CRIT_V=3.9", "BAT_CRIT_V", "rule"),
        ] {
            let e = hover_hash(&["--param", flag]).expect_err(flag);
            assert!(e.contains(name) && e.contains(why), "{flag}: {e}");
        }
    }

    #[test]
    fn params_file_comes_first_and_the_flag_wins() {
        let file = temp_file("order.toml", "[params]\nPOS_X_P = 1.2\nRTL_ALT = 12\n");
        let a = p(&["run", "--param", "POS_X_P=1.0", "--params", &file]).expect("valid");
        // File entries first (whatever the position of --params), then flags.
        assert_eq!(
            a.params,
            vec![
                ("POS_X_P".to_string(), 1.2),
                ("RTL_ALT".to_string(), 12.0),
                ("POS_X_P".to_string(), 1.0)
            ]
        );
        // The flag wins: the flight equals the one with POS_X_P = 1.0 alone
        // (plus the file's RTL_ALT, which a hover-step never reaches).
        let both = hover_hash(&["--param", "POS_X_P=1.0", "--params", &file]).expect("both");
        let flag_only = hover_hash(&["--param", "POS_X_P=1.0"]).expect("flag");
        let file_only = hover_hash(&["--params", &file]).expect("file");
        assert_eq!(both, flag_only);
        assert_ne!(both, file_only);
        let _ = std::fs::remove_file(&file);
    }

    #[test]
    fn a_params_file_with_other_tables_or_keys_is_refused() {
        let file = temp_file("bad.toml", "[param]\nPOS_X_P = 1.2\n");
        assert!(p(&["run", "--params", &file]).is_err());
        let _ = std::fs::remove_file(&file);
    }

    #[test]
    fn params_command_rejects_unknown_input() {
        let args = |a: &[&str]| a.iter().map(|s| (*s).to_string()).collect::<Vec<_>>();
        assert!(params_command(args(&["bogus"]).into_iter()).is_err());
        assert!(params_command(args(&["list", "extra"]).into_iter()).is_err());
        assert!(params_command(std::iter::empty()).is_err());
    }

    #[test]
    fn scenario_command_takes_overrides() {
        let args = |a: &[&str]| a.iter().map(|s| (*s).to_string()).collect::<Vec<_>>();
        let ok = ["gcs-loss", "--seed", "1", "--param", "RTL_ALT=12"];
        assert!(matches!(scenario_command(args(&ok).into_iter()), Ok(true)));
        let bad = ["gcs-loss", "--param", "NOPE=1"];
        assert!(scenario_command(args(&bad).into_iter()).is_err());
    }

    #[test]
    fn parameters_doc_is_up_to_date() {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../docs/PARAMETERS.md");
        let on_disk = std::fs::read_to_string(path).unwrap_or_else(|e| {
            panic!("{path}: {e}; regenerate with `oxide-sim params doc > docs/PARAMETERS.md`")
        });
        let norm = |t: &str| t.replace("\r\n", "\n");
        assert!(
            norm(&on_disk) == norm(&params_cmd::doc()),
            "docs/PARAMETERS.md is out of date; regenerate with `cargo run -p oxide-sim-cli -- params doc > docs/PARAMETERS.md`"
        );
    }

    #[test]
    fn scenario_command_flies_a_passing_scenario() {
        let a = ["gcs-loss", "--profile", "h743", "--seed", "1"];
        let r = scenario_command(a.iter().map(|s| (*s).to_string()));
        assert!(matches!(r, Ok(true)), "{r:?}");
    }

    #[test]
    fn scenario_command_flies_a_file_by_path_and_names_the_ways_to_name_one() {
        let path = temp_file(
            "mine.toml",
            "name = \"mine\"\nduration_s = 2.0\n[expect]\nmin_height_m = 4.0\n[expect.ending]\nkind = \"down\"\n",
        );
        let r = scenario_command([path.clone()].into_iter());
        assert!(
            matches!(r, Ok(false)),
            "a path flies, and its expectation fails: {r:?}"
        );
        let r = scenario_command(["hover-step".to_string()].into_iter());
        assert!(
            matches!(r, Ok(true)),
            "an embedded scenario without [expect]: {r:?}"
        );
        let e = scenario_command(["nope".to_string()].into_iter()).expect_err("unknown");
        assert!(e.to_string().contains("path"), "{e}");
        let e = scenario_command(["./no-such-file.toml".to_string()].into_iter())
            .expect_err("missing file");
        assert!(e.to_string().contains("no-such-file.toml"), "{e}");
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn the_result_line_says_when_only_the_invariants_were_judged() {
        let bare = scenario_file::load_str("name = \"bare\"\nduration_s = 2.0\n").expect("loads");
        assert_eq!(
            result_lines(&bare, &[], &[]),
            vec!["result     PASS (invariants only: the file states no expectations)"]
        );
        let broken = result_lines(&bare, &["2 timing faults".to_string()], &[]);
        assert!(broken[0].starts_with("result     FAIL"), "{broken:?}");
        assert_eq!(broken[1], "  violated: 2 timing faults");
        let judged = fault_scenarios::by_name("gcs-loss").expect("embedded");
        assert_eq!(result_lines(&judged, &[], &[]), vec!["result     PASS"]);
    }

    #[test]
    fn the_hold_line_without_the_core_part_says_what_it_needs() {
        let line = hold_line(&HoldMetrics::new());
        assert!(
            line.ends_with("estimate max horizontal (needs the flight core's status)"),
            "{line}"
        );
        assert!(!line.contains("NaN"), "{line}");
    }

    #[test]
    fn unjudged_requirements_are_not_a_pass() {
        let file = scenario_file::load_str("name = \"bare\"\nduration_s = 2.0\n").expect("loads");
        let verdict = scenario_file::Verdict {
            outcome: oxide_sim::outcome::FlightOutcome::new(),
            hold: None,
            violations: Vec::new(),
            not_judged: vec![oxide_sim::outcome::NotJudged {
                key: "reasons",
                reason: oxide_sim::outcome::NEEDS_CORE,
            }],
        };
        assert!(!verdict.not_judged.is_empty(), "{:?}", verdict.not_judged);
        assert!(!verdict.passed());
        assert!(
            verdict
                .first_problem()
                .is_some_and(|p| p.starts_with("not judged: ")),
            "{:?}",
            verdict.first_problem()
        );
        let lines = result_lines(&file, &verdict.violations, &verdict.not_judged);
        assert!(
            lines[0].starts_with("result     NOT JUDGED (")
                && !lines.iter().any(|l| l.contains("PASS")),
            "{lines:?}"
        );
        assert_eq!(lines.len(), 1 + verdict.not_judged.len(), "{lines:?}");
    }

    #[test]
    fn scenario_command_judges_a_hold_scenario() {
        let a = ["gust-hold", "--seed", "1"];
        let r = scenario_command(a.iter().map(|s| (*s).to_string()));
        assert!(matches!(r, Ok(true)), "{r:?}");
    }

    fn args(a: &[&str]) -> Vec<String> {
        a.iter().map(|s| (*s).to_string()).collect()
    }

    fn temp_path(name: &str) -> String {
        std::env::temp_dir()
            .join(format!("oxide-sim-{}-{name}", std::process::id()))
            .to_string_lossy()
            .into_owned()
    }

    #[test]
    fn log_options_parse_and_are_checked_together() {
        let a = p(&["run", "--log", "f.ulg"]).expect("valid");
        let spec = a.log.expect("a log");
        assert_eq!(spec.path, "f.ulg");
        assert_eq!(spec.level, LogLevel::Normal);
        let a = p(&["run", "--log", "f.ulg", "--log-level", "normal"]).expect("valid");
        assert_eq!(a.log.expect("a log").level, LogLevel::Normal);
        assert!(p(&["run"]).expect("valid").log.is_none());
        let e = p(&["run", "--log-level", "normal"]).err().expect("error");
        assert_eq!(e, "--log-level needs --log FILE.ulg");
        let e = p(&["run", "--log", "f.ulg", "--log-level", "loud"])
            .err()
            .expect("error");
        assert!(e.contains("normal | replay"), "{e}");
        let a = p(&["run", "--log", "f.ulg", "--log-level", "replay"]).expect("valid");
        assert_eq!(a.log.expect("a log").level, LogLevel::Replay);
        assert!(p(&["run", "--log"]).is_err());
        // hash and montecarlo do not write logs.
        let e = p(&["hash", "--log", "f.ulg"]).err().expect("error");
        assert!(e.contains("run and scenario"), "{e}");
        assert!(p(&["hash", "--log-level", "replay"]).is_err());
        assert!(p(&["montecarlo", "--log", "f.ulg"]).is_err());
        // The usage says so, where it names the options of hash.
        let hash = USAGE
            .lines()
            .find(|l| l.starts_with("  oxide-sim hash"))
            .expect("hash line");
        assert_eq!(
            hash,
            "  oxide-sim hash [same options as run, except --log and --log-level]"
        );
    }

    #[test]
    fn scenario_command_takes_log_options() {
        let path = temp_path("none.ulg");
        let e = scenario_command(args(&["gcs-loss", "--log-level", "normal"]).into_iter())
            .expect_err("level without a file");
        assert_eq!(e.to_string(), "--log-level needs --log FILE.ulg");
        let e = scenario_command(
            args(&["gcs-loss", "--log", &path, "--log-level", "loud"]).into_iter(),
        )
        .expect_err("unknown level");
        assert_eq!(e.to_string(), "unknown log level loud (normal | replay)");
        assert!(!std::path::Path::new(&path).exists());
    }

    /// A replay-level log of `gcs-loss` (seed 1), written by the command.
    fn replay_log(name: &str) -> String {
        let path = temp_path(name);
        let r = scenario_command(
            args(&[
                "gcs-loss",
                "--seed",
                "1",
                "--log",
                &path,
                "--log-level",
                "replay",
            ])
            .into_iter(),
        );
        assert!(matches!(r, Ok(true)), "{r:?}");
        path
    }

    #[test]
    fn scenario_writes_a_replay_level_log_and_replay_proves_it() {
        let path = replay_log("replay-level.ulg");
        let bytes = std::fs::read(&path).expect("the log");
        let file = oxide_log::reader::UlogFile::parse(&bytes).expect("parses");
        assert_eq!(file.info_str("oxide_log_level"), Some("replay"));
        assert!(file.records("oxide_in_tick").count() > 1000);
        assert!(file.records("vehicle_attitude").count() > 100);
        // Identical: exit 0 (true).
        let r = replay_command(args(&[&path]).into_iter());
        assert!(matches!(r, Ok(true)), "{r:?}");
        let (report, lines) = replay_file_report(&path, &[], false).expect("replays");
        assert!(report.identical());
        assert_eq!(
            lines[0],
            format!("replayed {} ticks (f405-lean): identical", report.ticks)
        );
        // A changed gain: a difference, exit 1 (false).
        let r = replay_command(args(&[&path, "--param", "RATE_ROLL_P=9"]).into_iter());
        assert!(matches!(r, Ok(false)), "{r:?}");
        let (report, lines) =
            replay_file_report(&path, &[("RATE_ROLL_P".to_string(), 9.0)], false).expect("replays");
        let d = report.first_difference.as_ref().expect("a difference");
        assert!(d.field.starts_with("motors["), "{d}");
        assert!(
            lines[0].contains(&format!("first difference at tick {}", d.seq)),
            "{lines:?}"
        );
        // --continue counts the ticks that differ.
        let (all, lines) =
            replay_file_report(&path, &[("RATE_ROLL_P".to_string(), 9.0)], true).expect("replays");
        assert!(all.differing > 1);
        assert!(
            lines[0].contains(" differ, the first at tick "),
            "{lines:?}"
        );
        let r =
            replay_command(args(&[&path, "--continue", "--param", "RATE_ROLL_P=9"]).into_iter());
        assert!(matches!(r, Ok(false)), "{r:?}");
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn replay_exits_2_on_a_file_it_cannot_replay_and_on_bad_arguments() {
        // A normal-level log, a missing file, bytes that are not a log.
        let normal = temp_path("normal-for-replay.ulg");
        let r = scenario_command(args(&["gcs-loss", "--log", &normal]).into_iter());
        assert!(matches!(r, Ok(true)), "{r:?}");
        let e = replay_command(args(&[&normal]).into_iter()).expect_err("normal level");
        assert!(e.to_string().contains("not a replay-level log"), "{e}");
        let _ = std::fs::remove_file(&normal);
        let missing = temp_path("no-such-file.ulg");
        assert!(replay_command(args(&[&missing]).into_iter()).is_err());
        let junk = temp_path("junk.ulg");
        std::fs::write(&junk, b"not a log").expect("writes");
        assert!(replay_command(args(&[&junk]).into_iter()).is_err());
        let _ = std::fs::remove_file(&junk);
        // Arguments.
        assert!(replay_command(args(&[]).into_iter()).is_err());
        let path = replay_log("replay-args.ulg");
        for bad in [
            vec![path.as_str(), "--param"],
            vec![path.as_str(), "--param", "RATE_ROLL_P"],
            vec![path.as_str(), "--param", "NO_SUCH=1"],
            vec![path.as_str(), "--frobnicate"],
            vec![path.as_str(), "other.ulg"],
        ] {
            assert!(replay_command(args(&bad).into_iter()).is_err(), "{bad:?}");
        }
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn a_cut_replay_log_still_exits_0_and_says_it_is_cut() {
        let path = replay_log("replay-cut.ulg");
        let bytes = std::fs::read(&path).expect("the log");
        let cut = temp_path("replay-cut-short.ulg");
        std::fs::write(&cut, &bytes[..bytes.len() - 7]).expect("writes");
        let r = replay_command(args(&[&cut]).into_iter());
        assert!(matches!(r, Ok(true)), "{r:?}");
        let (report, lines) = replay_file_report(&cut, &[], false).expect("replays");
        assert!(report.identical());
        assert!(
            lines[1].starts_with("the file ends in a cut after tick "),
            "{lines:?}"
        );
        let _ = std::fs::remove_file(&path);
        let _ = std::fs::remove_file(&cut);
    }

    #[test]
    fn scenario_writes_a_log_the_reader_parses() {
        let path = temp_path("scenario.ulg");
        let r = scenario_command(args(&["gcs-loss", "--seed", "1", "--log", &path]).into_iter());
        assert!(matches!(r, Ok(true)), "{r:?}");
        let bytes = std::fs::read(&path).expect("the log");
        let file = oxide_log::reader::UlogFile::parse(&bytes).expect("parses");
        assert_eq!(file.info_str("ver_hw"), Some("SIM"));
        assert_eq!(file.info_str("ver_sw"), Some(GIT_HASH));
        assert_eq!(file.info_str("oxide_log_level"), Some("normal"));
        assert_eq!(file.info_str("oxide_profile"), Some("f405-lean"));
        assert!(file.records("vehicle_attitude").count() > 100);
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn run_writes_a_log_and_the_hash_is_the_same_without_it() {
        let path = temp_path("run.ulg");
        let airframe = Airframe::reference_quad().expect("airframe");
        let with = p(&["run", "--log", &path]).expect("valid");
        let plain = p(&["run"]).expect("valid");
        let a = execute(
            &airframe,
            with.profile,
            hover_step_job(&with),
            None,
            with.log.as_ref(),
        )
        .expect("flies with a log");
        let b =
            execute(&airframe, plain.profile, hover_step_job(&plain), None, None).expect("flies");
        assert_eq!(a.hash, b.hash);
        let bytes = std::fs::read(&path).expect("the log");
        assert!(oxide_log::reader::UlogFile::parse(&bytes).is_ok());
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn an_unwritable_log_path_is_an_error_before_the_flight() {
        let bad = temp_path("no-such-dir/x.ulg");
        let r = scenario_command(args(&["gcs-loss", "--log", &bad]).into_iter());
        assert!(r.is_err());
    }

    /// A writer that takes `left` bytes and then fails like a full disk.
    struct FailsAfter {
        left: usize,
    }

    impl Write for FailsAfter {
        fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
            if self.left == 0 {
                return Err(std::io::Error::other("disk full"));
            }
            let n = buf.len().min(self.left);
            self.left -= n;
            Ok(n)
        }

        fn flush(&mut self) -> std::io::Result<()> {
            Ok(())
        }
    }

    /// The simulation of `gcs-loss`, seed 1, and its scenario.
    macro_rules! gcs_loss {
        () => {{
            let scenario = fault_scenarios::by_name("gcs-loss").expect("embedded");
            let sim = scenario
                .build_with_params(F405_LEAN, 1, &[])
                .expect("builds");
            (scenario, sim)
        }};
    }

    #[test]
    fn a_log_that_fails_after_the_flight_started_still_prints_the_verdict() {
        let (scenario, sim) = gcs_loss!();
        let mut sinks = Sinks::none();
        sinks.log_path = Some("disk.ulg".to_string());
        // The header goes in; the disk fills up during the flight.
        sinks.log_start = Some((Level::Normal, Box::new(FailsAfter { left: 40_000 })));
        let run = fly_scenario(&scenario, F405_LEAN, 1, sim, sinks).expect("the flight ran");
        assert!(run.passed);
        assert!(
            run.lines.iter().any(|l| l == "result     PASS"),
            "{:?}",
            run.lines
        );
        assert!(run.lines.iter().any(|l| l.starts_with("trace hash ")));
        let e = run.write_error.expect("the write error");
        assert_eq!(e.to_string(), "disk.ulg: disk full");
    }

    #[test]
    fn a_csv_that_fails_during_the_flight_is_named_and_the_log_is_still_finished() {
        let (scenario, sim) = gcs_loss!();
        let buffer = oxide_sim::flight_log::SharedBuffer::new();
        let mut sinks = Sinks::none();
        sinks.csv = Some(("rows.csv".to_string(), Box::new(FailsAfter { left: 1000 })));
        sinks.log_path = Some("ok.ulg".to_string());
        sinks.log_start = Some((Level::Normal, Box::new(buffer.clone())));
        let run = fly_scenario(&scenario, F405_LEAN, 1, sim, sinks).expect("the flight ran");
        assert_eq!(
            run.write_error.expect("the csv error").to_string(),
            "rows.csv: disk full"
        );
        // The log of the flight that ran is complete.
        let bytes = buffer.bytes();
        let file = oxide_log::reader::UlogFile::parse(&bytes).expect("parses");
        assert!(file.records("vehicle_attitude").count() > 100);
    }

    #[test]
    fn a_log_that_cannot_be_started_is_an_error_before_the_flight() {
        let (scenario, sim) = gcs_loss!();
        let mut sinks = Sinks::none();
        sinks.log_path = Some("disk.ulg".to_string());
        sinks.log_start = Some((Level::Normal, Box::new(FailsAfter { left: 100 })));
        let e = fly_scenario(&scenario, F405_LEAN, 1, sim, sinks)
            .err()
            .expect("an error");
        assert_eq!(e.to_string(), "disk.ulg: disk full");
    }

    #[test]
    fn the_usage_names_the_log_options() {
        let run = USAGE.split("  oxide-sim hash").next().expect("run part");
        assert!(run.contains("--log FILE.ulg") && run.contains("--log-level normal|replay"));
        let replay = USAGE
            .lines()
            .find(|l| l.starts_with("  oxide-sim replay"))
            .expect("replay line");
        assert_eq!(
            replay,
            "  oxide-sim replay FILE.ulg [--param NAME=VALUE ...] [--continue]"
        );
        let scenario = USAGE
            .lines()
            .find(|l| l.starts_with("  oxide-sim scenario NAME"))
            .expect("scenario line");
        assert!(scenario.contains("--log FILE.ulg") && scenario.contains("--log-level"));
    }

    /// The five lines `hash` has always printed for a profile.
    const PINNED: [&str; 10] = [
        "f405-lean hover-step 3de16d7a9847deaf",
        "f405-lean gust-hold#0 9712d37590a317dc",
        "f405-lean rc-loss-position#0 927c415bcb82bfc8",
        "f405-lean pair-gps-loss+baro-loss#0 c7ebc02674f19d3a",
        "f405-lean motor-loss#0 86248bd7c7755d84",
        "h743-full hover-step 9ff026845c342fbd",
        "h743-full gust-hold#0 476abddf98142653",
        "h743-full rc-loss-position#0 f28dc4abdee2eae9",
        "h743-full pair-gps-loss+baro-loss#0 212288cff52a1846",
        "h743-full motor-loss#0 dbc9b9c7c0c2f5d8",
    ];

    fn hash_output(profile: &str) -> Vec<String> {
        let args = p(&["hash", "--profile", profile]).expect("valid");
        let airframe = Airframe::reference_quad().expect("airframe");
        hash_lines(&args, &airframe).expect("hash lines")
    }

    #[test]
    fn hash_prints_its_five_lines_as_before_then_one_per_logged_flight() {
        for (profile, name) in [("f405", "f405-lean"), ("h743", "h743-full")] {
            let lines = hash_output(profile);
            assert_eq!(lines.len(), 8, "{lines:#?}");
            let pinned: Vec<&str> = PINNED
                .iter()
                .copied()
                .filter(|l| l.starts_with(name))
                .collect();
            assert_eq!(
                lines[..5].iter().map(String::as_str).collect::<Vec<_>>(),
                pinned
            );
            for (line, flight) in lines[5..].iter().zip(["rc-loss-position", "gust-hold"]) {
                let w: Vec<&str> = line.split(' ').collect();
                assert_eq!(w.len(), 4, "{line}");
                assert_eq!(&w[..3], &[name, "log-normal", &format!("{flight}#0")]);
                assert_eq!(w[3].len(), 16, "{line}");
                assert!(
                    w[3].chars()
                        .all(|c| c.is_ascii_hexdigit() && !c.is_ascii_uppercase())
                );
            }
            assert_ne!(lines[5], lines[6]);
            // Then the replay-level log of rc-loss-position.
            let w: Vec<&str> = lines[7].split(' ').collect();
            assert_eq!(w.len(), 4, "{}", lines[7]);
            assert_eq!(&w[..3], &[name, "log-replay", "rc-loss-position#0"]);
            assert_eq!(w[3].len(), 16, "{}", lines[7]);
            assert!(
                w[3].chars()
                    .all(|c| c.is_ascii_hexdigit() && !c.is_ascii_uppercase())
            );
            assert_ne!(w[3], lines[5].rsplit(' ').next().expect("hash"));
        }
    }

    #[test]
    fn the_replay_level_log_hash_is_the_fnv_1a_of_a_file_that_replays() {
        for profile in [F405_LEAN, H743_FULL] {
            let a = logged_flight_bytes_at(profile, Level::Replay, "rc-loss-position", 0, &[])
                .expect("first");
            let b = logged_flight_bytes_at(profile, Level::Replay, "rc-loss-position", 0, &[])
                .expect("second");
            assert_eq!(a, b, "{}", profile.name);
            let file = oxide_log::reader::UlogFile::parse(&a).expect("parses");
            assert_eq!(file.info_str("oxide_log_level"), Some("replay"));
            assert_eq!(file.info_str("ver_sw"), Some("hash"));
            assert_eq!(file.info_str("ver_hw"), Some("SIM"));
            let report =
                oxide_sim::replay::replay(&a, &oxide_sim::replay::ReplayOptions::default())
                    .expect("replays");
            assert!(report.identical() && report.exit_code() == 0);
            let line = log_replay_hash_lines(profile, &[])
                .expect("lines")
                .remove(0);
            assert_eq!(
                line,
                format!(
                    "{} log-replay rc-loss-position#0 {:016x}",
                    profile.name,
                    fnv1a_64(&a)
                )
            );
        }
    }

    #[test]
    fn the_same_flight_gives_the_same_log_bytes_and_they_hash_as_fnv_1a() {
        for profile in [F405_LEAN, H743_FULL] {
            let a = logged_flight_bytes(profile, "rc-loss-position", 0, &[]).expect("first");
            let b = logged_flight_bytes(profile, "rc-loss-position", 0, &[]).expect("second");
            assert_eq!(a, b, "{}", profile.name);
            let file = oxide_log::reader::UlogFile::parse(&a).expect("parses");
            // The build information the hash uses, whatever the commit.
            assert_eq!(file.info_str("ver_sw"), Some("hash"));
            assert_eq!(file.info_str("ver_hw"), Some("SIM"));
            // FNV-1a 64, from the definition.
            let mut h: u64 = 0xcbf2_9ce4_8422_2325;
            for byte in &a {
                h = (h ^ u64::from(*byte)).wrapping_mul(0x0000_0100_0000_01b3);
            }
            assert_eq!(fnv1a_64(&a), h);
            let line = log_hash_lines(profile, &[]).expect("lines").remove(0);
            assert_eq!(
                line,
                format!("{} log-normal rc-loss-position#0 {h:016x}", profile.name)
            );
        }
        // Another seed or another parameter is another file.
        let a = logged_flight_bytes(F405_LEAN, "motor-loss", 0, &[]).expect("a");
        let b = logged_flight_bytes(F405_LEAN, "motor-loss", 1, &[]).expect("b");
        assert_ne!(a, b);
    }
}
