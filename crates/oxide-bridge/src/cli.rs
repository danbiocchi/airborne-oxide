// SPDX-License-Identifier: AGPL-3.0-or-later
//! The command line of `oxide-bridge`: `fly` flies one run of a scenario
//! file, `batch` flies many, each in a child process of its own.
//!
//! ```text
//! oxide-bridge fly   --scenario FILE [--index N] [--autopilot arducopter|core-f405|core-h743|core-f405-truth]
//!                    [--binary PATH] [--instance N] [--out DIR] [--speedup X]
//!                    [--param-file FILE]... [--twice]
//! oxide-bridge batch --scenario FILE --indices A..B --out DIR [--jobs J] [--instances A-B]
//!                    [--autopilot ...] [--binary PATH] [--speedup X] [--param-file FILE]...
//!                    [--twice] [--limit-s S] [--no-pin] [--first-core N]
//! oxide-bridge tune  --out DIR [--binary PATH] [--instance N] [--seed N]
//!                    [--hover-only [--gains FILE]]
//! ```
//!
//! Exit codes: 0 the work was done (`fly`: the flight was flown and judged,
//! whatever the verdict; `batch`: every run has its row), 1 a wrong use (a
//! sentence says what), 2 `fly`: the flight did not start or the bridge
//! failed, 3 `batch`: a run failed twice and the batch stopped after writing
//! what it had.

use std::io::Write as _;
use std::path::{Path, PathBuf};
use std::process::{Command, ExitCode};
use std::time::Duration;

use crate::batch::{self, BatchConfig, Event, Job, Pin};
use crate::record::{self, Row};
use crate::runner::{self, Autopilot, RunError, ScenarioFlight, Setup};
use crate::scenario::{LoadedScenario, translate};

/// The variable that holds the simulator program's path when `--binary` is
/// not given.
pub const BINARY_VARIABLE: &str = "OXIDE_ARDUCOPTER";

/// The wall limit of one child of a batch, s, when `--limit-s` is not given.
pub const DEFAULT_LIMIT_S: f64 = 600.0;

/// A wrong use: one sentence.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Usage(pub String);

impl core::fmt::Display for Usage {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for Usage {}

fn usage<T>(text: impl Into<String>) -> Result<T, Usage> {
    Err(Usage(text.into()))
}

/// What both commands take.
#[derive(Clone, Debug, PartialEq)]
pub struct Common {
    pub scenario: PathBuf,
    pub autopilot: Autopilot,
    pub binary: Option<PathBuf>,
    pub speedup: u32,
    pub param_files: Vec<PathBuf>,
    pub twice: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct FlyArgs {
    pub common: Common,
    pub index: u64,
    pub instance: u8,
    pub out: Option<PathBuf>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct BatchArgs {
    pub common: Common,
    /// `A..B`, the end not included.
    pub indices: (u64, u64),
    pub out: PathBuf,
    pub jobs: usize,
    pub instances: (u8, u8),
    pub limit_s: f64,
    pub pin: bool,
    pub first_core: usize,
}

/// What `tune` is given (ADR-0007 section 2 as changed by ADR-0009).
#[derive(Clone, Debug, PartialEq)]
pub struct TuneArgs {
    pub out: PathBuf,
    pub binary: Option<PathBuf>,
    pub instance: u8,
    pub seed: u64,
    /// Fly only the hover of the script, as a check of a gains file, and
    /// report its quality.
    pub hover_only: bool,
    /// The gains file of a hover check (default `starting-gains.parm`).
    pub gains: Option<PathBuf>,
}

#[derive(Clone, Debug, PartialEq)]
pub enum Cmd {
    Help,
    Fly(FlyArgs),
    Batch(BatchArgs),
    Tune(TuneArgs),
    /// `gate fly` and `gate verdict` ([`crate::gate::cmd`]).
    Gate(crate::gate::cmd::GateCmd),
}

/// The usage text.
pub const USAGE: &str = "\
oxide-bridge fly   --scenario FILE [--index N] [--autopilot arducopter|core-f405|core-h743|core-f405-truth]
                   [--binary PATH] [--instance N] [--out DIR] [--speedup X]
                   [--param-file FILE]... [--twice]
oxide-bridge batch --scenario FILE --indices A..B --out DIR [--jobs J] [--instances A-B]
                   [--autopilot ...] [--binary PATH] [--speedup X] [--param-file FILE]...
                   [--twice] [--limit-s S] [--no-pin] [--first-core N]
oxide-bridge tune  --out DIR [--binary PATH] [--instance N] [--seed N]
                   [--hover-only [--gains FILE]]
The simulator program is --binary, or the path in OXIDE_ARDUCOPTER. A batch runs
each flight as `oxide-bridge fly` in a child process of its own and writes
DIR/flights.csv. `tune` flies ArduCopter's tuning procedure (ADR-0007 section 2 as
changed by ADR-0009): two tuning flights from POSHOLD, roll and pitch and then yaw, with the
harmonic notch decided once, on the hover of the first flight (a peak makes that flight land
after its hover, and the tuning flights are then flown with the notch). It writes
DIR/report.txt, DIR/spectrum.csv (the hover that decides the notch), DIR/tuned-gains-1.parm
(what the roll-and-pitch flight saved) and, when every check of every flight holds,
DIR/tuned-gains.parm (the 29 gains and, when a notch was set, its six values). With --hover-only it
flies only the hover of the script and prints its quality. Exit codes: 0 done, 1 wrong use, 2 the flight did not start or
the bridge failed, 3 a run failed twice and the batch stopped.
oxide-bridge gate fly|verdict: flies gate 1 and computes its verdict (`oxide-bridge gate`).";

struct Args<'a> {
    rest: &'a [String],
    pos: usize,
}

impl<'a> Args<'a> {
    fn next(&mut self) -> Option<&'a str> {
        let a = self.rest.get(self.pos)?;
        self.pos += 1;
        Some(a.as_str())
    }

    fn value(&mut self, flag: &str) -> Result<&'a str, Usage> {
        self.next()
            .ok_or_else(|| Usage(format!("{flag} needs a value")))
    }
}

fn number<T: std::str::FromStr>(flag: &str, text: &str, what: &str) -> Result<T, Usage> {
    text.parse::<T>()
        .or_else(|_| usage(format!("{flag} is {text:?}: it must be {what}")))
}

/// `a-b`, inclusive, both 0 to 9 (the program's instance numbers).
pub fn parse_instances(text: &str) -> Result<(u8, u8), Usage> {
    let bad = || {
        Usage(format!(
            "--instances is {text:?}: it must be a range `a-b` with 0 <= a <= b <= 9"
        ))
    };
    let (a, b) = text.split_once('-').ok_or_else(bad)?;
    let digits = |p: &str| !p.is_empty() && p.bytes().all(|c| c.is_ascii_digit());
    if !digits(a) || !digits(b) {
        return Err(bad());
    }
    let (a, b) = (
        a.parse::<u8>().map_err(|_| bad())?,
        b.parse::<u8>().map_err(|_| bad())?,
    );
    if a > b || b > 9 {
        return Err(bad());
    }
    Ok((a, b))
}

/// `A..B`: the indices A up to B, B not included.
pub fn parse_indices(text: &str) -> Result<(u64, u64), Usage> {
    let bad = || {
        Usage(format!(
            "--indices is {text:?}: it must be `A..B` (A up to B, B not included) with A below B"
        ))
    };
    let (a, b) = text.split_once("..").ok_or_else(bad)?;
    let digits = |p: &str| !p.is_empty() && p.bytes().all(|c| c.is_ascii_digit());
    if !digits(a) || !digits(b) {
        return Err(bad());
    }
    let (a, b) = (
        a.parse::<u64>().map_err(|_| bad())?,
        b.parse::<u64>().map_err(|_| bad())?,
    );
    if a >= b {
        return Err(bad());
    }
    Ok((a, b))
}

/// Parses the arguments after the program's name.
pub fn parse(args: &[String]) -> Result<Cmd, Usage> {
    let Some(command) = args.first() else {
        return usage("no command: say `fly` or `batch` (oxide-bridge help lists the options)");
    };
    match command.as_str() {
        "help" | "--help" | "-h" => Ok(Cmd::Help),
        "fly" => parse_fly(args.get(1..).unwrap_or(&[])),
        "batch" => parse_batch(args.get(1..).unwrap_or(&[])),
        "tune" => parse_tune(args.get(1..).unwrap_or(&[])),
        "gate" => crate::gate::cmd::parse(args.get(1..).unwrap_or(&[])).map(Cmd::Gate),
        other => usage(format!(
            "{other:?} is not a command: say `fly`, `batch` or `tune` (oxide-bridge help lists the options)"
        )),
    }
}

struct Collected {
    scenario: Option<PathBuf>,
    autopilot: Autopilot,
    binary: Option<PathBuf>,
    speedup: u32,
    param_files: Vec<PathBuf>,
    twice: bool,
    index: Option<u64>,
    instance: Option<u8>,
    out: Option<PathBuf>,
    indices: Option<(u64, u64)>,
    jobs: usize,
    instances: Option<(u8, u8)>,
    limit_s: f64,
    pin: bool,
    first_core: usize,
}

fn collect(rest: &[String], batch: bool) -> Result<Collected, Usage> {
    let mut c = Collected {
        scenario: None,
        autopilot: Autopilot::ArduCopter,
        binary: None,
        speedup: 100,
        param_files: Vec::new(),
        twice: false,
        index: None,
        instance: None,
        out: None,
        indices: None,
        jobs: 1,
        instances: None,
        limit_s: DEFAULT_LIMIT_S,
        pin: true,
        first_core: 0,
    };
    let mut a = Args { rest, pos: 0 };
    while let Some(flag) = a.next() {
        match (flag, batch) {
            ("--scenario", _) => c.scenario = Some(PathBuf::from(a.value(flag)?)),
            ("--autopilot", _) => {
                let v = a.value(flag)?;
                c.autopilot = Autopilot::parse(v).map_or_else(
                    || {
                        usage(format!(
                            "--autopilot is {v:?}: it must be arducopter, core-f405, core-h743 or core-f405-truth"
                        ))
                    },
                    Ok,
                )?;
            }
            ("--binary", _) => c.binary = Some(PathBuf::from(a.value(flag)?)),
            ("--speedup", _) => {
                let v = a.value(flag)?;
                c.speedup = number::<u32>(flag, v, "a whole number from 1")?;
                if c.speedup == 0 {
                    return usage("--speedup is 0: it must be a whole number from 1");
                }
            }
            ("--param-file", _) => c.param_files.push(PathBuf::from(a.value(flag)?)),
            ("--twice", _) => c.twice = true,
            ("--out", _) => c.out = Some(PathBuf::from(a.value(flag)?)),
            ("--index", false) => {
                c.index = Some(number(flag, a.value(flag)?, "a whole number")?);
            }
            ("--instance", false) => {
                let v = a.value(flag)?;
                let n: u8 = number(flag, v, "a whole number from 0 to 9")?;
                if n > 9 {
                    return usage(format!("--instance is {n}: it must be from 0 to 9"));
                }
                c.instance = Some(n);
            }
            ("--indices", true) => c.indices = Some(parse_indices(a.value(flag)?)?),
            ("--jobs", true) => {
                let v = a.value(flag)?;
                c.jobs = number(flag, v, "a whole number from 1")?;
                if c.jobs == 0 {
                    return usage("--jobs is 0: it must be a whole number from 1");
                }
            }
            ("--instances", true) => c.instances = Some(parse_instances(a.value(flag)?)?),
            ("--limit-s", true) => {
                let v = a.value(flag)?;
                c.limit_s = number(flag, v, "a number of seconds above zero")?;
                if !(c.limit_s.is_finite() && c.limit_s > 0.0) {
                    return usage(format!(
                        "--limit-s is {v:?}: it must be a number of seconds above zero"
                    ));
                }
            }
            ("--no-pin", true) => c.pin = false,
            ("--first-core", true) => {
                c.first_core = number(flag, a.value(flag)?, "a whole number")?;
            }
            (other, _) => {
                return usage(format!(
                    "{other:?} is not an option of {}: oxide-bridge help lists them",
                    if batch { "batch" } else { "fly" }
                ));
            }
        }
    }
    Ok(c)
}

fn common(c: &Collected) -> Result<Common, Usage> {
    let Some(scenario) = c.scenario.clone() else {
        return usage("--scenario FILE is required: say which scenario file to fly");
    };
    Ok(Common {
        scenario,
        autopilot: c.autopilot,
        binary: c.binary.clone(),
        speedup: c.speedup,
        param_files: c.param_files.clone(),
        twice: c.twice,
    })
}

fn parse_fly(rest: &[String]) -> Result<Cmd, Usage> {
    let c = collect(rest, false)?;
    Ok(Cmd::Fly(FlyArgs {
        common: common(&c)?,
        index: c.index.unwrap_or(0),
        instance: c.instance.unwrap_or(0),
        out: c.out,
    }))
}

fn parse_tune(rest: &[String]) -> Result<Cmd, Usage> {
    let mut a = Args { rest, pos: 0 };
    let (mut out, mut binary, mut gains) = (None, None, None);
    let (mut instance, mut seed, mut hover_only) = (0u8, 0u64, false);
    while let Some(flag) = a.next() {
        match flag {
            "--out" => out = Some(PathBuf::from(a.value(flag)?)),
            "--binary" => binary = Some(PathBuf::from(a.value(flag)?)),
            "--gains" => gains = Some(PathBuf::from(a.value(flag)?)),
            "--hover-only" => hover_only = true,
            "--instance" => {
                let v = a.value(flag)?;
                instance = number(flag, v, "a whole number from 0 to 9")?;
                if instance > 9 {
                    return usage(format!("--instance is {instance}: it must be from 0 to 9"));
                }
            }
            "--seed" => seed = number(flag, a.value(flag)?, "a whole number")?,
            other => {
                return usage(format!(
                    "{other:?} is not an option of tune: oxide-bridge help lists them"
                ));
            }
        }
    }
    let Some(out) = out else {
        return usage("--out DIR is required: say where the tuning writes its files");
    };
    if gains.is_some() && !hover_only {
        return usage(
            "--gains is for --hover-only: the tuning itself starts from starting-gains.parm",
        );
    }
    Ok(Cmd::Tune(TuneArgs {
        out,
        binary,
        instance,
        seed,
        hover_only,
        gains,
    }))
}

/// `oxide-bridge tune`.
pub fn run_tune(args: &TuneArgs) -> ExitCode {
    let binary = match &args.binary {
        Some(b) => b.clone(),
        None => match std::env::var_os(BINARY_VARIABLE) {
            Some(p) if !p.is_empty() => PathBuf::from(p),
            _ => {
                return wrong_use(&Usage(format!(
                    "no simulator program: give --binary PATH or set {BINARY_VARIABLE}"
                )));
            }
        },
    };
    if args.hover_only {
        let gains = args
            .gains
            .clone()
            .unwrap_or_else(crate::parm::starting_gains_path);
        let files = vec![crate::parm::base_path(), crate::parm::setup_path(), gains];
        return match crate::tune::hover_check(&binary, &args.out, args.instance, args.seed, files) {
            Ok(f) => {
                let s = &f.stats;
                println!(
                    "hover check: tilt rms {:.4} deg, position rms {:.4} m, height rms {:.4} m \
                     (the last {} s of the hover); simulated {:.1} s, wall {:.1} s{}",
                    s.hover_tilt_rms_deg,
                    s.hover_pos_rms_m,
                    s.hover_height_rms_m,
                    crate::tune::QUALITY_S,
                    f.sim_s,
                    f.wall_s,
                    f.failure
                        .as_ref()
                        .map_or(String::new(), |e| format!("; FAILED: {e}"))
                );
                if f.failure.is_some() {
                    ExitCode::from(2)
                } else {
                    ExitCode::SUCCESS
                }
            }
            Err(e) => {
                eprintln!("error: {e}");
                ExitCode::from(2)
            }
        };
    }
    let cfg = crate::tune::TuneConfig {
        binary,
        out: args.out.clone(),
        instance: args.instance,
        seed: args.seed,
    };
    match crate::tune::tune(&cfg) {
        Ok(o) => {
            println!(
                "tuned      {} values written to {} ({} flights)",
                o.values_written(),
                args.out.join("tuned-gains.parm").display(),
                o.flights.len()
            );
            println!("report     {}", args.out.join("report.txt").display());
            ExitCode::SUCCESS
        }
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::from(2)
        }
    }
}

fn parse_batch(rest: &[String]) -> Result<Cmd, Usage> {
    let c = collect(rest, true)?;
    let common = common(&c)?;
    let Some(indices) = c.indices else {
        return usage("--indices A..B is required: say which runs to fly");
    };
    let Some(out) = c.out.clone() else {
        return usage("--out DIR is required: say where the batch writes flights.csv");
    };
    let instances = c.instances.unwrap_or((0, 9));
    let available = usize::from(instances.1 - instances.0) + 1;
    if c.jobs > available {
        return usage(format!(
            "--jobs is {}: each job needs an instance number of its own and --instances {}-{} \
             has {available}",
            c.jobs, instances.0, instances.1
        ));
    }
    Ok(Cmd::Batch(BatchArgs {
        common,
        indices,
        out,
        jobs: c.jobs,
        instances,
        limit_s: c.limit_s,
        pin: c.pin,
        first_core: c.first_core,
    }))
}

/// The simulator program's path: `--binary`, else the variable.
fn binary_of(common: &Common) -> Result<PathBuf, Usage> {
    if let Some(b) = &common.binary {
        return Ok(b.clone());
    }
    match std::env::var_os(BINARY_VARIABLE) {
        Some(p) if !p.is_empty() => Ok(PathBuf::from(p)),
        _ => usage(format!(
            "no simulator program: give --binary PATH or set {BINARY_VARIABLE}"
        )),
    }
}

/// Checks what can be checked before anything flies: the scenario loads and,
/// for ArduCopter, translates and the program is named.
fn prepare(common: &Common) -> Result<(LoadedScenario, Option<PathBuf>), Usage> {
    let src = LoadedScenario::load(&common.scenario).map_err(|e| Usage(e.to_string()))?;
    if common.autopilot == Autopilot::ArduCopter {
        translate(&src.file).map_err(|e| {
            Usage(format!(
                "scenario {:?} cannot be flown through the bridge: {e}",
                src.file.name
            ))
        })?;
        return Ok((src, Some(binary_of(common)?)));
    }
    Ok((src, None))
}

fn result_lines(f: &ScenarioFlight) -> Vec<String> {
    let v = &f.verdict;
    let mut out = Vec::new();
    if !v.violations.is_empty() {
        out.push("result     FAIL".to_string());
        out.extend(v.violations.iter().map(|x| format!("  violated: {x}")));
    } else if !v.not_judged.is_empty() {
        out.push(format!(
            "result     NOT JUDGED ({} requirement{} could not be judged)",
            v.not_judged.len(),
            if v.not_judged.len() == 1 { "" } else { "s" }
        ));
    } else {
        out.push("result     PASS".to_string());
    }
    out.extend(v.not_judged.iter().map(|n| format!("  not judged: {n}")));
    out
}

fn print_flight(f: &ScenarioFlight, dir: &Path, repeats: Option<bool>) {
    println!(
        "flight     {} run {} seed {} autopilot {}",
        f.scenario,
        f.index,
        f.seed,
        f.autopilot.name()
    );
    println!("class      {}", f.class);
    if let Some(h) = &f.verdict.hold {
        println!(
            "hold       rms_h {:.4} m  max_h {:.4} m  rms_v {} m  max_v {:.4} m  tilt {:.2} deg",
            h.rms_h_error_m,
            h.max_h_error_m,
            f.rms_v_m.map_or("-".to_string(), |v| format!("{v:.4}")),
            h.max_v_error_m,
            h.max_tilt_deg
        );
    }
    for line in result_lines(f) {
        println!("{line}");
    }
    if let Some(r) = repeats {
        println!(
            "repeats    {}",
            if r {
                "yes (the hashes of both flights are equal)"
            } else {
                "NO (the hashes of the two flights differ)"
            }
        );
    }
    println!("wall       {:.2} s", f.wall_s);
    println!("record     {}", dir.display());
}

/// How a run is flown.
#[derive(Clone, Debug)]
enum Plan {
    Core(oxide_sim::scenario_file::RunProfile),
    CoreTruth,
    Bridge(Setup),
}

fn fly_once(
    src: &LoadedScenario,
    index: u64,
    plan: &Plan,
    out: &Path,
) -> Result<ScenarioFlight, RunError> {
    match plan {
        Plan::Core(profile) => runner::fly_core(src, index, *profile),
        Plan::CoreTruth => runner::fly_core_truth(src, index),
        Plan::Bridge(setup) => {
            let mut setup = setup.clone();
            setup.out = out.to_path_buf();
            runner::fly_scenario(src, index, &setup)
        }
    }
}

/// Flies a run a second time with `second` and says whether the three hashes
/// (trace, packet, reply) of the second flight are those of `first`.
fn fly_again(
    first: &ScenarioFlight,
    second: impl FnOnce() -> Result<ScenarioFlight, RunError>,
) -> Result<(ScenarioFlight, bool), RunError> {
    let second = second()?;
    let same = second.hashes() == first.hashes();
    Ok((second, same))
}

/// `oxide-bridge fly`.
pub fn run_fly(args: &FlyArgs) -> ExitCode {
    let (src, binary) = match prepare(&args.common) {
        Ok(x) => x,
        Err(e) => return wrong_use(&e),
    };
    let autopilot = args.common.autopilot;
    let out = args.out.clone().unwrap_or_else(|| {
        std::env::temp_dir().join(format!(
            "oxide-bridge-{}-{}-{}-{}",
            src.file.name,
            args.index,
            autopilot.name(),
            std::process::id()
        ))
    });
    let plan = match (autopilot, binary) {
        (Autopilot::Core(profile), _) => Plan::Core(profile),
        (Autopilot::CoreTruth, _) => Plan::CoreTruth,
        (Autopilot::ArduCopter, binary) => {
            let mut s = Setup::new(binary.unwrap_or_default(), &out, args.instance);
            s.speedup = args.common.speedup;
            s.param_files.clone_from(&args.common.param_files);
            Plan::Bridge(s)
        }
    };
    let flown = fly_once(&src, args.index, &plan, &out);
    let flight = match flown {
        Ok(f) => f,
        Err(e) => return failed(&src, args.index, autopilot, &out, &e, 0.0),
    };
    let mut row = Row::of_flight(&flight);
    let mut repeats = None;
    if args.common.twice {
        let second_out = out.join("second");
        match fly_again(&flight, || fly_once(&src, args.index, &plan, &second_out)) {
            Ok((second, same)) => {
                if let Err(e) = record::write_files(
                    &second_out,
                    &record::audit_text(&src, &second, None),
                    &Row::of_flight(&second),
                ) {
                    eprintln!("error: cannot write the record of the second flight: {e}");
                    return ExitCode::from(2);
                }
                row.set_repeats(same);
                repeats = Some(same);
            }
            Err(e) => {
                return failed(&src, args.index, autopilot, &out, &e, flight.wall_s);
            }
        }
    }
    let audit = record::audit_text(&src, &flight, repeats);
    if let Err(e) = record::write_files(&out, &audit, &row) {
        eprintln!("error: cannot write the record into {}: {e}", out.display());
        return ExitCode::from(2);
    }
    print_flight(&flight, &out, repeats);
    ExitCode::SUCCESS
}

/// A flight that did not start or whose bridge failed: the record says why,
/// the exit code is 2.
fn failed(
    src: &LoadedScenario,
    index: u64,
    autopilot: Autopilot,
    out: &Path,
    e: &RunError,
    wall_s: f64,
) -> ExitCode {
    eprintln!("error: {e}");
    if let RunError::NotTranslatable(_) = e {
        return ExitCode::from(1);
    }
    let row = Row::not_started(
        &src.file.name,
        index,
        src.file.seed(index),
        &autopilot,
        wall_s,
        &e.to_string(),
    );
    let text = record::failure_text(src, index, &autopilot, e);
    if let Err(w) = record::write_files(out, &text, &row) {
        eprintln!("error: cannot write the record into {}: {w}", out.display());
    } else {
        eprintln!("record     {}", out.display());
    }
    ExitCode::from(2)
}

fn wrong_use(e: &Usage) -> ExitCode {
    eprintln!("error: {e}");
    ExitCode::from(1)
}

/// The command that flies one run in a child process: this program with
/// `fly` and the job's own instance, directory and index.
pub fn child_command(exe: &Path, args: &BatchArgs, binary: Option<&Path>, job: &Job) -> Command {
    let mut c = Command::new(exe);
    c.arg("fly")
        .arg("--scenario")
        .arg(&args.common.scenario)
        .arg("--index")
        .arg(job.index.to_string())
        .arg("--autopilot")
        .arg(args.common.autopilot.name())
        .arg("--instance")
        .arg(job.instance.to_string())
        .arg("--out")
        .arg(&job.out)
        .arg("--speedup")
        .arg(args.common.speedup.to_string());
    if let Some(b) = binary {
        c.arg("--binary").arg(b);
    }
    for f in &args.common.param_files {
        c.arg("--param-file").arg(f);
    }
    if args.common.twice {
        c.arg("--twice");
    }
    c
}

/// `oxide-bridge batch`.
pub fn run_batch(args: &BatchArgs) -> ExitCode {
    let (src, binary) = match prepare(&args.common) {
        Ok(x) => x,
        Err(e) => return wrong_use(&e),
    };
    let exe = match std::env::current_exe() {
        Ok(e) => e,
        Err(e) => {
            return wrong_use(&Usage(format!(
                "cannot find this program's own path to start the children: {e}"
            )));
        }
    };
    let pin = if args.pin && batch::taskset_available() {
        Pin::Taskset {
            first_core: args.first_core,
        }
    } else {
        Pin::Off
    };
    if args.pin && pin == Pin::Off {
        eprintln!("note: `taskset` was not found, so the children are not pinned to cores");
    }
    let runs: Vec<(u64, u64)> = (args.indices.0..args.indices.1)
        .map(|i| (i, src.file.seed(i)))
        .collect();
    // Twice as long: both flights of a run are one child.
    let factor = if args.common.twice { 2.0 } else { 1.0 };
    let cfg = BatchConfig {
        scenario: src.file.name.clone(),
        autopilot: args.common.autopilot,
        runs,
        jobs: args.jobs,
        instances: args.instances,
        limit: Duration::from_secs_f64(args.limit_s * factor),
        out: args.out.clone(),
        pin,
    };
    let launch = |job: &Job| child_command(&exe, args, binary.as_deref(), job);
    let say = |e: &Event| {
        let line = match e {
            Event::Spawned {
                index,
                attempt,
                pid,
                instance,
                core,
            } => format!(
                "run {index} attempt {attempt}: started, pid {pid}, instance {instance}, core {}",
                core.map_or("unpinned".to_string(), |c| c.to_string())
            ),
            Event::Failed {
                index,
                attempt,
                reason,
                will_retry,
            } => format!(
                "run {index} attempt {attempt}: failed: {reason}{}",
                if *will_retry {
                    " (flown once more)"
                } else {
                    ""
                }
            ),
            Event::Done { index, row } => format!(
                "run {index}: {} {} {}",
                if row.get("started") == "yes" {
                    row.get("class")
                } else {
                    "not started"
                },
                row.get("verdict"),
                if row.get("rms_h_m").is_empty() {
                    String::new()
                } else {
                    format!("rms_h {} m", row.get("rms_h_m"))
                }
            ),
        };
        let mut out = std::io::stdout().lock();
        let _ = writeln!(out, "{line}");
        let _ = out.flush();
    };
    match batch::run_batch(&cfg, &launch, &say) {
        Ok(result) => {
            println!(
                "table      {} ({} of {} runs)",
                args.out.join("flights.csv").display(),
                result.rows.len(),
                cfg.runs.len()
            );
            match result.stopped_at {
                None => ExitCode::SUCCESS,
                Some(i) => {
                    eprintln!(
                        "error: run {i} failed twice: the batch stopped after writing what it had"
                    );
                    ExitCode::from(3)
                }
            }
        }
        Err(e) => wrong_use(&Usage(e.to_string())),
    }
}

/// The program's main: parses, runs, returns the exit code.
pub fn main_with(args: &[String]) -> ExitCode {
    match parse(args) {
        Ok(Cmd::Help) => {
            println!("{USAGE}");
            ExitCode::SUCCESS
        }
        Ok(Cmd::Fly(a)) => run_fly(&a),
        Ok(Cmd::Batch(a)) => run_batch(&a),
        Ok(Cmd::Tune(a)) => run_tune(&a),
        Ok(Cmd::Gate(g)) => crate::gate::cmd::run(&g),
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::from(1)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A real flight to compare (our core, so that no binary is needed).
    fn a_flight() -> ScenarioFlight {
        let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../scenarios/hover-step.toml");
        let src = LoadedScenario::load(path).expect("hover-step");
        runner::fly_core(&src, 0, oxide_sim::scenario_file::RunProfile::F405).expect("flies")
    }

    #[test]
    fn a_second_flight_repeats_only_when_all_three_hashes_are_the_firsts() {
        let first = a_flight();
        let again = |edit: &dyn Fn(&mut ScenarioFlight)| {
            let mut f = first.clone();
            edit(&mut f);
            fly_again(&first, || Ok(f)).expect("flown").1
        };
        assert!(again(&|_| {}), "the same hashes repeat");
        assert!(!again(&|f| f.trace_hash ^= 1), "a trace hash differs");
        assert!(
            !again(&|f| f.packet_hash = Some(7)),
            "a packet hash differs, the others the same"
        );
        assert!(
            !again(&|f| f.reply_hash = Some(7)),
            "a reply hash differs, the others the same"
        );
        // The second flight is the one returned, and an error of it is passed on.
        let (second, _) = fly_again(&first, || {
            let mut f = first.clone();
            f.wall_s = 123.0;
            Ok(f)
        })
        .expect("flown");
        assert_eq!(second.wall_s, 123.0);
        assert!(
            fly_again(&first, || Err(RunError::Sim(
                oxide_sim::SimError::TimeBase(1)
            )))
            .is_err()
        );
    }

    fn words(line: &str) -> Vec<String> {
        line.split_whitespace().map(String::from).collect()
    }

    fn fly(line: &str) -> FlyArgs {
        match parse(&words(line)) {
            Ok(Cmd::Fly(a)) => a,
            other => panic!("{line}: {other:?}"),
        }
    }

    fn batch(line: &str) -> BatchArgs {
        match parse(&words(line)) {
            Ok(Cmd::Batch(a)) => a,
            other => panic!("{line}: {other:?}"),
        }
    }

    fn wrong(line: &str) -> String {
        match parse(&words(line)) {
            Err(Usage(s)) => s,
            other => panic!("{line}: should be a wrong use, got {other:?}"),
        }
    }

    #[test]
    fn fly_takes_its_options_and_defaults_the_rest() {
        let a = fly("fly --scenario s.toml");
        assert_eq!(a.index, 0);
        assert_eq!(a.instance, 0);
        assert_eq!(a.out, None);
        assert_eq!(a.common.autopilot, Autopilot::ArduCopter);
        assert_eq!(a.common.speedup, 100);
        assert!(!a.common.twice);
        let a = fly(
            "fly --scenario s.toml --index 7 --autopilot core-h743 --binary /x/ac --instance 5 \
             --out o --speedup 10 --param-file a.parm --param-file b.parm --twice",
        );
        assert_eq!(a.index, 7);
        assert_eq!(a.instance, 5);
        assert_eq!(a.common.autopilot.name(), "core-h743");
        assert_eq!(a.common.binary, Some(PathBuf::from("/x/ac")));
        assert_eq!(a.out, Some(PathBuf::from("o")));
        assert_eq!(a.common.speedup, 10);
        assert_eq!(
            a.common.param_files,
            vec![PathBuf::from("a.parm"), PathBuf::from("b.parm")]
        );
        assert!(a.common.twice);
    }

    #[test]
    fn batch_takes_its_options_and_defaults_the_rest() {
        let a = batch("batch --scenario s.toml --indices 0..4 --out d");
        assert_eq!(a.indices, (0, 4));
        assert_eq!(a.jobs, 1);
        assert_eq!(a.instances, (0, 9));
        assert!(a.pin);
        assert_eq!(a.limit_s, DEFAULT_LIMIT_S);
        let a = batch(
            "batch --scenario s.toml --indices 10..12 --out d --jobs 3 --instances 5-9 \
             --autopilot core-f405 --twice --limit-s 30 --no-pin --first-core 1",
        );
        assert_eq!((a.indices, a.jobs, a.instances), ((10, 12), 3, (5, 9)));
        assert!(a.common.twice && !a.pin);
        assert_eq!((a.limit_s, a.first_core), (30.0, 1));
    }

    #[test]
    fn every_wrong_use_is_one_sentence() {
        let cases = [
            ("", "no command"),
            ("run", "not a command"),
            ("fly", "--scenario FILE is required"),
            ("fly --scenario", "needs a value"),
            ("fly --scenario s.toml --nonsense", "not an option of fly"),
            ("fly --scenario s.toml --jobs 2", "not an option of fly"),
            ("fly --scenario s.toml --index x", "--index"),
            ("fly --scenario s.toml --index -1", "--index"),
            ("fly --scenario s.toml --instance 10", "from 0 to 9"),
            ("fly --scenario s.toml --instance q", "--instance"),
            (
                "fly --scenario s.toml --autopilot px4",
                "arducopter, core-f405, core-h743 or core-f405-truth",
            ),
            ("fly --scenario s.toml --speedup 0", "from 1"),
            ("fly --scenario s.toml --speedup fast", "--speedup"),
            (
                "batch --indices 0..2 --out d",
                "--scenario FILE is required",
            ),
            (
                "batch --scenario s.toml --out d",
                "--indices A..B is required",
            ),
            (
                "batch --scenario s.toml --indices 0..2",
                "--out DIR is required",
            ),
            (
                "batch --scenario s.toml --indices 3..3 --out d",
                "A below B",
            ),
            (
                "batch --scenario s.toml --indices 5..2 --out d",
                "A below B",
            ),
            ("batch --scenario s.toml --indices 5 --out d", "--indices"),
            (
                "batch --scenario s.toml --indices a..b --out d",
                "--indices",
            ),
            (
                "batch --scenario s.toml --indices 0..2 --out d --jobs 0",
                "from 1",
            ),
            (
                "batch --scenario s.toml --indices 0..2 --out d --jobs 3 --instances 0-1",
                "instance number of its own",
            ),
            (
                "batch --scenario s.toml --indices 0..2 --out d --instances 3-2",
                "--instances",
            ),
            (
                "batch --scenario s.toml --indices 0..2 --out d --instances 0-10",
                "--instances",
            ),
            (
                "batch --scenario s.toml --indices 0..2 --out d --instances 4",
                "--instances",
            ),
            (
                "batch --scenario s.toml --indices 0..2 --out d --limit-s 0",
                "above zero",
            ),
            (
                "batch --scenario s.toml --indices 0..2 --out d --limit-s nan",
                "above zero",
            ),
            (
                "batch --scenario s.toml --indices 0..2 --out d --index 1",
                "not an option of batch",
            ),
            (
                "batch --scenario s.toml --indices 0..2 --out d --instance 1",
                "not an option of batch",
            ),
        ];
        for (line, needle) in cases {
            let s = wrong(line);
            assert!(s.contains(needle), "{line:?}: {s}");
            assert!(!s.contains('\n') && !s.is_empty(), "{line:?}: {s}");
        }
    }

    #[test]
    fn help_is_asked_for_by_name() {
        for h in ["help", "--help", "-h"] {
            assert_eq!(parse(&words(h)), Ok(Cmd::Help));
        }
    }

    #[test]
    fn a_child_of_a_batch_is_this_program_flying_one_run() {
        let a = batch(
            "batch --scenario s.toml --indices 0..4 --out d --autopilot core-f405 --speedup 7 \
             --param-file p.parm --twice",
        );
        let job = Job {
            index: 3,
            instance: 6,
            attempt: 1,
            out: PathBuf::from("d/run-0003"),
        };
        let c = child_command(
            Path::new("/bin/oxide-bridge"),
            &a,
            Some(Path::new("/x/ac")),
            &job,
        );
        let args: Vec<String> = c
            .get_args()
            .map(|s| s.to_string_lossy().into_owned())
            .collect();
        assert_eq!(
            args.join(" "),
            "fly --scenario s.toml --index 3 --autopilot core-f405 --instance 6 --out d/run-0003 \
             --speedup 7 --binary /x/ac --param-file p.parm --twice"
        );
    }
}
