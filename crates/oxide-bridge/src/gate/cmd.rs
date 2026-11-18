// SPDX-License-Identifier: AGPL-3.0-or-later
//! The `gate` subcommand of `oxide-bridge`.
//!
//! ```text
//! oxide-bridge gate fly --scenarios DIR --out OUT [--binary PATH] [--jobs J]
//!                       [--instances A-B] [--allow-dirty] [--reduced]
//! oxide-bridge gate verdict --in OUT [--report FILE]
//! ```
//!
//! Exit codes of `gate fly`: 0 flown, 1 a wrong use or a refusal (uncommitted
//! changes, no binary), 3 a run failed twice and the gate stopped. Of `gate
//! verdict`: 0 passed, 4 not passed, 2 the tables are incomplete or
//! inconsistent (no verdict).

use std::path::PathBuf;
use std::process::ExitCode;

use crate::cli::{Usage, parse_instances};
use crate::gate::fly::{self, Settings};
use crate::gate::report;
use crate::gate::verdict::{self, Verdict};

/// What `gate` is asked.
#[derive(Clone, Debug, PartialEq)]
pub enum GateCmd {
    Fly(Settings),
    Verdict {
        input: PathBuf,
        report: Option<PathBuf>,
    },
}

/// The usage text of the subcommand.
pub const USAGE: &str = "\
oxide-bridge gate fly --scenarios DIR --out OUT [--binary PATH] [--jobs J] [--instances A-B]
                      [--allow-dirty] [--reduced]
oxide-bridge gate verdict --in OUT [--report FILE]
`gate fly` flies the plan of ADR-0007 from the scenario files of DIR into OUT (exit 3: a
run failed twice and the gate stopped), from a commit with no uncommitted changes unless
--allow-dirty; a directory with a gate in it is continued. `gate verdict` reads only OUT's
tables and gate.txt: exit 0 passed, 4 not passed, 2 the tables are incomplete or inconsistent.";

fn usage<T>(text: impl Into<String>) -> Result<T, Usage> {
    Err(Usage(text.into()))
}

fn take<'a>(words: &mut std::slice::Iter<'a, String>, flag: &str) -> Result<&'a str, Usage> {
    words
        .next()
        .map(String::as_str)
        .ok_or_else(|| Usage(format!("{flag} needs a value")))
}

fn parse_fly(args: &[String], rest: &[String]) -> Result<GateCmd, Usage> {
    let mut s = Settings {
        scenarios: PathBuf::new(),
        out: PathBuf::new(),
        binary: None,
        jobs: None,
        instances: (0, 9),
        allow_dirty: false,
        reduced: false,
        command_line: format!("oxide-bridge gate {}", args.join(" ")),
    };
    let (mut have_scenarios, mut have_out) = (false, false);
    let mut words = rest.iter();
    while let Some(flag) = words.next() {
        match flag.as_str() {
            "--scenarios" => {
                s.scenarios = PathBuf::from(take(&mut words, flag)?);
                have_scenarios = true;
            }
            "--out" => {
                s.out = PathBuf::from(take(&mut words, flag)?);
                have_out = true;
            }
            "--binary" => s.binary = Some(PathBuf::from(take(&mut words, flag)?)),
            "--jobs" => {
                let v = take(&mut words, flag)?;
                let n: usize = v.parse().map_err(|_| {
                    Usage(format!("--jobs is {v:?}: it must be a whole number from 1"))
                })?;
                if n == 0 {
                    return usage("--jobs is 0: it must be a whole number from 1");
                }
                s.jobs = Some(n);
            }
            "--instances" => s.instances = parse_instances(take(&mut words, flag)?)?,
            "--allow-dirty" => s.allow_dirty = true,
            "--reduced" => s.reduced = true,
            other => {
                return usage(format!(
                    "{other:?} is not an option of gate fly: oxide-bridge help lists them"
                ));
            }
        }
    }
    if !have_scenarios {
        return usage("--scenarios DIR is required: say where the scenario files are");
    }
    if !have_out {
        return usage("--out OUT is required: say where the gate writes its tables");
    }
    let available = usize::from(s.instances.1 - s.instances.0) + 1;
    if s.jobs.is_some_and(|j| j > available) {
        return usage(format!(
            "--jobs is above the {available} instance numbers of --instances {}-{}",
            s.instances.0, s.instances.1
        ));
    }
    Ok(GateCmd::Fly(s))
}

fn parse_verdict(rest: &[String]) -> Result<GateCmd, Usage> {
    let (mut input, mut report) = (None, None);
    let mut words = rest.iter();
    while let Some(flag) = words.next() {
        match flag.as_str() {
            "--in" => input = Some(PathBuf::from(take(&mut words, flag)?)),
            "--report" => report = Some(PathBuf::from(take(&mut words, flag)?)),
            other => {
                return usage(format!(
                    "{other:?} is not an option of gate verdict: oxide-bridge help lists them"
                ));
            }
        }
    }
    let Some(input) = input else {
        return usage("--in OUT is required: say which gate directory to read");
    };
    Ok(GateCmd::Verdict { input, report })
}

/// Parses the arguments after `gate`.
pub fn parse(args: &[String]) -> Result<GateCmd, Usage> {
    let Some(sub) = args.first() else {
        return usage("no gate command: say `gate fly` or `gate verdict`");
    };
    let rest = args.get(1..).unwrap_or(&[]);
    match sub.as_str() {
        "fly" => parse_fly(args, rest),
        "verdict" => parse_verdict(rest),
        other => usage(format!(
            "{other:?} is not a gate command: say `gate fly` or `gate verdict`"
        )),
    }
}

/// Runs a gate command and gives the exit code.
pub fn run(cmd: &GateCmd) -> ExitCode {
    match cmd {
        GateCmd::Fly(s) => fly::run(s),
        GateCmd::Verdict { input, report } => {
            let v = verdict::verdict(input);
            print!("{}", v.text());
            if let Some(path) = report {
                match &v {
                    Verdict::Evaluated(e) => match report::write(e, path) {
                        Ok(paths) => {
                            for p in paths {
                                println!("report     {}", p.display());
                            }
                        }
                        Err(err) => {
                            eprintln!("error: cannot write the report {}: {err}", path.display());
                            return ExitCode::from(1);
                        }
                    },
                    Verdict::Incomplete(_) => {
                        eprintln!("note: no report is written when there is no verdict");
                    }
                }
            }
            ExitCode::from(v.exit_code())
        }
    }
}
