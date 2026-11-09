// SPDX-License-Identifier: AGPL-3.0-or-later
//! `oxide-sim`: run a scenario headless.
//!
//! ```text
//! oxide-sim run  [--profile f405|h743] [--airframe FILE] [--csv FILE]
//!                [--scenario hover-step|gust-hold] [--seed N]
//!                [--estimator truth|ekf] [--sensors ideal|realistic]
//! oxide-sim hash [same options]
//! ```
//! `hash` prints trace hashes, which CI compares between Linux and Windows
//! (DESIGN §5.2). Without `--scenario` it prints one line for hover-step and
//! one for gust-hold run 0, so the check covers the RNG, the noise models and
//! the libm paths.

use std::io::Write;
use std::process::ExitCode;
use std::time::Instant;

use oxide_core::config::EstimatorSource;
use oxide_core::math::Vec3;
use oxide_core::schedule::{F405_LEAN, H743_FULL, Profile};
use oxide_core::time::Micros;
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
}

fn parse() -> Result<Args, String> {
    let mut it = std::env::args().skip(1);
    let command = it.next().ok_or("missing command: run | hash")?;
    let mut args = Args {
        command,
        profile: F405_LEAN,
        airframe: None,
        csv: None,
        scenario: None,
        seed: 0,
        estimator: None,
        sensors: None,
    };
    while let Some(flag) = it.next() {
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
            other => return Err(format!("unknown option {other}")),
        }
    }
    if args.command != "run" && args.command != "hash" {
        return Err(format!("unknown command {} (run | hash)", args.command));
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
        "{:.4},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.4},{:.4},{:.4},{:.4},{:.3},{:.3},{},{},{},{},{},{},{},{},{},{},{:.4},{:.4},{:.4}\n",
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
    )
}

const CSV_HEADER: &[u8] = b"t,x,y,z,vx,vy,vz,roll,pitch,yaw,p,q,r,m0,m1,m2,m3,battery_v,battery_a,armed,sp_x,sp_y,sp_z,est_x,est_y,est_z,est_roll,est_pitch,est_yaw,wind_n,wind_e,wind_d\n";

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

fn main() -> ExitCode {
    let args = match parse() {
        Ok(a) => a,
        Err(e) => {
            eprintln!("error: {e}");
            eprintln!(
                "usage: oxide-sim run|hash [--profile f405|h743] [--airframe FILE] [--csv FILE] [--scenario hover-step|gust-hold] [--seed N] [--estimator truth|ekf] [--sensors ideal|realistic]"
            );
            return ExitCode::from(2);
        }
    };
    match run(&args) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::FAILURE
        }
    }
}
