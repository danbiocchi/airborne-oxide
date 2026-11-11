// SPDX-License-Identifier: AGPL-3.0-or-later
//! `oxide-sim`: run a scenario headless.
//!
//! ```text
//! oxide-sim run  [--profile f405|h743] [--airframe FILE] [--csv FILE]
//!                [--scenario hover-step|gust-hold] [--seed N]
//!                [--estimator truth|ekf] [--sensors ideal|realistic]
//! oxide-sim hash [same options]
//! oxide-sim montecarlo [--profile f405|h743] [--seeds N] [--threads T]
//! oxide-sim scenario list
//! oxide-sim scenario NAME [--profile f405|h743] [--seed N] [--csv FILE]
//! oxide-sim params list | json | doc
//! ```
//! `run`, `hash`, `montecarlo` and `scenario` also take `--params FILE.toml`
//! (a `[params]` table) and `--param NAME=VALUE` (repeatable): parameter
//! overrides applied through the checked `set`, the file first, then the flags
//! in order (ADR-0004). `params list` prints the table, `params json` the
//! ground-station metadata and `params doc` the text of `docs/PARAMETERS.md`.
//! `hash` prints trace hashes, which CI compares between Linux and Windows
//! (DESIGN §5.2). Without `--scenario` it prints one line for hover-step and
//! one for gust-hold run 0, so the check covers the RNG, the noise models and
//! the libm paths, then one line per fault scenario in `HASHED_FAULT_SCENARIOS`
//! (run 0, `<profile> <name>#0 <hash>`), so that it also covers ground contact,
//! landings, failsafes and a crash. `montecarlo` flies gust-hold indices 0..N (default 200) on
//! all cores (or `--threads T`) against the M1b acceptance criteria and exits
//! with 1 if any run fails. `scenario` flies a fault scenario (ADR-0003),
//! prints its event timeline, a summary and every violated expectation, and
//! exits with 1 if there are any; `--seed` is the run index.

mod params_cmd;

use std::io::Write;
use std::process::ExitCode;
use std::time::Instant;

use oxide_core::config::EstimatorSource;
use oxide_core::math::Vec3;
use oxide_core::schedule::{F405_LEAN, H743_FULL, Profile};
use oxide_core::time::Micros;
use oxide_sim::fault_scenarios;
use oxide_sim::metrics::{HoldCriteria, HoldMetrics, fly_gust_hold_with_params};
use oxide_sim::montecarlo::run_indexed;
use oxide_sim::outcome::FlightOutcome;
use oxide_sim::scenario::{GustHold, HoverStep, Pilot, gust_hold};
use oxide_sim::sensors::SensorConfig;
use oxide_sim::{Airframe, SimError, SimOptions, Simulation, StepRecord};

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
        .ok_or("missing command: run | hash | montecarlo")?;
    let mut args = Args {
        command,
        profile: F405_LEAN,
        airframe: None,
        csv: None,
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
    if !matches!(args.command.as_str(), "run" | "hash" | "montecarlo") {
        return Err(format!(
            "unknown command {} (run | hash | montecarlo)",
            args.command
        ));
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
        let Some(est) = r.estimate else { return };
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
    let est = r.estimate;
    let est_pos = est.map(|s| s.position);
    let est_euler = est.map(|s| s.attitude.cast::<f64>().to_euler());
    let g = |v: Option<f32>| v.map_or(String::new(), |x| format!("{x:.5}"));
    let h = |v: Option<f64>| v.map_or(String::new(), |x| format!("{x:.5}"));
    format!(
        "{:.4},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.4},{:.4},{:.4},{:.4},{:.3},{:.3},{},{},{},{},{},{},{},{},{},{},{:.4},{:.4},{:.4},{:?},{:?},{},{},{},{}\n",
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
        u8::from(r.status.armed),
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
        r.status.nav,
        r.status.action,
        g(est.map(|s| s.velocity.x)),
        g(est.map(|s| s.velocity.y)),
        g(est.map(|s| s.velocity.z)),
        r.status.reasons.0,
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
) -> Result<Outcome, Box<dyn std::error::Error>> {
    let duration_us = (job.duration_s * 1e6) as u64;
    let mut sim = Simulation::<_, 4>::with_options(airframe, profile, job.pilot, job.options)?;
    let mut csv = match csv_path {
        Some(p) => {
            let mut f = std::io::BufWriter::new(std::fs::File::create(p)?);
            f.write_all(CSV_HEADER)?;
            Some(f)
        }
        None => None,
    };
    let mut max_tilt = 0.0_f64;
    let mut errors = EstimateErrors::default();
    let mut io_error = None;
    let start = Instant::now();
    sim.run_until(Micros(duration_us), |r| {
        let e = r.attitude.to_euler();
        max_tilt = max_tilt.max(e.x.abs()).max(e.y.abs());
        errors.update(r);
        if let Some(f) = csv.as_mut()
            && let Err(err) = f.write_all(csv_row(r).as_bytes())
        {
            io_error.get_or_insert(err);
        }
    });
    let wall_s = start.elapsed().as_secs_f64();
    if let Some(err) = io_error {
        return Err(err.into());
    }
    if let Some(mut f) = csv {
        f.flush()?;
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
        let mut sim = fault_scenarios::build_with_params(&scenario, profile, 0, params)?;
        sim.run_until(
            Micros((scenario.script.duration_s * 1e6).round() as u64),
            |_| {},
        );
        lines.push(format!(
            "{} {}#0 {:016x}",
            profile.name,
            name,
            sim.trace_hash()
        ));
    }
    Ok(lines)
}

fn run(args: &Args) -> Result<(), Box<dyn std::error::Error>> {
    let airframe = load_airframe(args.airframe.as_deref())?;
    let profile = args.profile;
    let run_one = |kind: ScenarioKind, csv: Option<&str>| match kind {
        ScenarioKind::HoverStep => execute(&airframe, profile, hover_step_job(args), csv),
        ScenarioKind::GustHold => execute(&airframe, profile, gust_hold_job(args), csv),
    };

    if args.command == "hash" {
        let kinds: &[ScenarioKind] = match args.scenario {
            Some(ScenarioKind::HoverStep) => &[ScenarioKind::HoverStep],
            Some(ScenarioKind::GustHold) => &[ScenarioKind::GustHold],
            // Both, with default options: covers the RNG, noise and libm paths.
            None => &[ScenarioKind::HoverStep, ScenarioKind::GustHold],
        };
        for &kind in kinds {
            let o = run_one(kind, args.csv.as_deref())?;
            println!("{} {} {:016x}", profile.name, o.label, o.hash);
        }
        if args.scenario.is_none() {
            for line in fault_hash_lines_with(profile, &args.params)? {
                println!("{line}");
            }
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
        let f = m.failures(&c);
        if f.is_empty() {
            passed += 1;
        } else {
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
        worst("estimate max horizontal (m)", |m| m.est_max_h_error_m);
        worst("estimate max vertical (m)", |m| m.est_max_v_error_m);
        worst("estimate max velocity (m/s)", |m| m.est_max_vel_error_mps);
        worst("estimate max attitude (deg)", |m| m.est_max_att_error_deg);
        worst("rejected measurements", |m| f64::from(m.rejected));
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

/// `oxide-sim scenario list | NAME [--profile P] [--seed N] [--csv FILE]
/// [--params FILE] [--param NAME=VALUE]`.
fn scenario_command(
    mut it: impl Iterator<Item = String>,
) -> Result<bool, Box<dyn std::error::Error>> {
    let first = it.next().ok_or("scenario needs list or a scenario name")?;
    if first == "list" {
        for s in fault_scenarios::all()
            .into_iter()
            .chain(fault_scenarios::pairs())
        {
            println!("{:<34} {}", s.name, s.about);
        }
        return Ok(true);
    }
    let scenario = fault_scenarios::by_name(&first)
        .ok_or_else(|| format!("unknown scenario {first} (try: oxide-sim scenario list)"))?;
    let (mut profile, mut index, mut csv_path) = (F405_LEAN, 0_u64, None);
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
            "--params" => params_file = Some(value()?),
            "--param" => param_flags.push(parse_param_flag(&value()?)?),
            other => return Err(format!("unknown option {other}").into()),
        }
    }
    let params = collect_params(params_file.as_deref(), param_flags)?;
    let mut sim = fault_scenarios::build_with_params(&scenario, profile, index, &params)?;
    let mut csv = match &csv_path {
        Some(p) => {
            let mut f = std::io::BufWriter::new(std::fs::File::create(p)?);
            f.write_all(CSV_HEADER)?;
            Some(f)
        }
        None => None,
    };
    let mut outcome = FlightOutcome::new();
    let mut io_error = None;
    let end = Micros((scenario.script.duration_s * 1e6).round() as u64);
    sim.run_until(end, |r| {
        outcome.add(r);
        if let Some(f) = csv.as_mut()
            && let Err(err) = f.write_all(csv_row(r).as_bytes())
        {
            io_error.get_or_insert(err);
        }
    });
    if let Some(err) = io_error {
        return Err(err.into());
    }
    if let Some(mut f) = csv {
        f.flush()?;
    }
    println!(
        "scenario   {} (run {index}, profile {})",
        scenario.name, profile.name
    );
    println!("about      {}", scenario.about);
    println!("timeline");
    for e in &outcome.events {
        println!("  {}", oxide_sim::outcome::event_line(e));
    }
    let p = outcome.position_at_end;
    println!("navigation {:?}", outcome.nav_sequence);
    println!(
        "reasons    {}   highest action {:?}",
        oxide_sim::outcome::reason_names(outcome.reasons_seen),
        outcome.max_action
    );
    println!(
        "end        t {:.2} s, N {:.2} E {:.2} D {:.2} m, armed {}, {:?}",
        outcome.end_s, p.x, p.y, p.z, outcome.armed_at_end, outcome.nav_at_end
    );
    println!(
        "limits     max height {:.2} m, max distance from home {:.2} m, max tilt {:.1} deg, touchdown {}",
        outcome.max_height_m,
        outcome.max_distance_from_home_m,
        outcome.max_tilt_deg,
        outcome
            .touchdown_speed_mps
            .map_or("none".into(), |v| format!("{v:.2} m/s"))
    );
    println!(
        "disarmed   {} at {}",
        outcome
            .disarm_reason
            .map_or("never".into(), |r| format!("{r:?}")),
        outcome
            .disarmed_at_s
            .map_or("-".into(), |t| format!("{t:.2} s"))
    );
    println!("trace hash {:016x}", sim.trace_hash());
    let violations = scenario.expect.check(&outcome);
    if violations.is_empty() {
        println!("result     PASS");
    } else {
        println!("result     FAIL");
        for v in &violations {
            println!("  violated: {v}");
        }
    }
    Ok(violations.is_empty())
}

fn main() -> ExitCode {
    if std::env::args().nth(1).as_deref() == Some("params") {
        return match params_command(std::env::args().skip(2)) {
            Ok(()) => ExitCode::SUCCESS,
            Err(e) => {
                eprintln!("error: {e}");
                ExitCode::from(2)
            }
        };
    }
    if std::env::args().nth(1).as_deref() == Some("scenario") {
        return match scenario_command(std::env::args().skip(2)) {
            Ok(true) => ExitCode::SUCCESS,
            Ok(false) => ExitCode::FAILURE,
            Err(e) => {
                eprintln!("error: {e}");
                ExitCode::from(2)
            }
        };
    }
    let args = match parse() {
        Ok(a) => a,
        Err(e) => {
            eprintln!("error: {e}");
            eprintln!(
                "usage: oxide-sim run|hash|montecarlo|scenario|params [--profile f405|h743] [--airframe FILE] [--csv FILE] [--scenario hover-step|gust-hold] [--seed N] [--estimator truth|ekf] [--sensors ideal|realistic] [--seeds N] [--threads T] [--params FILE] [--param NAME=VALUE]; oxide-sim params list|json|doc"
            );
            return ExitCode::from(2);
        }
    };
    if args.command == "montecarlo" {
        return match montecarlo(&args) {
            Ok(true) => ExitCode::SUCCESS,
            Ok(false) => ExitCode::FAILURE,
            Err(e) => {
                eprintln!("error: {e}");
                ExitCode::FAILURE
            }
        };
    }
    match run(&args) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::FAILURE
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p(args: &[&str]) -> Result<Args, String> {
        parse_from(args.iter().map(|s| (*s).to_string()))
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
        execute(&airframe, args.profile, hover_step_job(&args), None)
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
}
