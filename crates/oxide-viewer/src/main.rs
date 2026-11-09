// SPDX-License-Identifier: AGPL-3.0-or-later
//! `oxide-viewer`: fly the simulator in real time and watch it in Rerun.
//!
//! ```text
//! oxide-viewer [--profile f405|h743] [--scenario hover-step|gust-hold] [--seed N]
//!              [--joystick] [--speed X] [--save FILE.rrd]
//! ```
//! Without `--joystick` it flies a scripted scenario: `hover-step` (default,
//! ideal sensors, calm air) or `gust-hold` (realistic sensors, wind, EKF;
//! `--seed` picks the run index). With `--joystick` a USB joystick (the
//! Taranis in USB-joystick mode) is the pilot.
//! Rerun is a visualiser only; it never feeds back into the simulation.

#[cfg(feature = "joystick")]
mod joystick;

use std::error::Error;
use std::time::{Duration, Instant};

use oxide_core::io::{RcSample, TrajectorySetpoint};
use oxide_core::schedule::{F405_LEAN, H743_FULL, Profile};
use oxide_core::time::Micros;
use oxide_sim::scenario::{GustHold, HoverStep, Pilot, gust_hold};
use oxide_sim::{Airframe, SimOptions, Simulation, StepRecord};

/// Rerun is updated at this rate, Hz.
const VIEW_RATE_HZ: u64 = 50;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Scenario {
    HoverStep,
    GustHold,
}

struct Args {
    profile: Profile,
    scenario: Scenario,
    seed: u64,
    joystick: bool,
    speed: f64,
    save: Option<String>,
}

fn parse() -> Result<Args, String> {
    let mut args = Args {
        profile: F405_LEAN,
        scenario: Scenario::HoverStep,
        seed: 0,
        joystick: false,
        speed: 1.0,
        save: None,
    };
    let mut it = std::env::args().skip(1);
    while let Some(flag) = it.next() {
        let mut value = || it.next().ok_or(format!("{flag} needs a value"));
        match flag.as_str() {
            "--profile" => {
                args.profile = match value()?.as_str() {
                    "f405" => F405_LEAN,
                    "h743" => H743_FULL,
                    other => return Err(format!("unknown profile {other}")),
                }
            }
            "--scenario" => {
                args.scenario = match value()?.as_str() {
                    "hover-step" => Scenario::HoverStep,
                    "gust-hold" => Scenario::GustHold,
                    other => return Err(format!("unknown scenario {other}")),
                }
            }
            "--seed" => {
                args.seed = value()?
                    .parse()
                    .map_err(|_| "--seed needs a non-negative integer".to_string())?;
            }
            "--joystick" => args.joystick = true,
            "--speed" => {
                args.speed = value()?
                    .parse()
                    .map_err(|_| "--speed needs a number".to_string())?;
            }
            "--save" => args.save = Some(value()?),
            other => return Err(format!("unknown option {other}")),
        }
    }
    Ok(args)
}

/// Either the scripted scenario or a joystick.
enum AnyPilot {
    Scripted(HoverStep),
    GustHold(GustHold),
    #[cfg(feature = "joystick")]
    Joystick(Box<joystick::JoystickPilot>),
}

impl Pilot for AnyPilot {
    fn rc(&mut self, t: Micros) -> Option<RcSample> {
        match self {
            AnyPilot::Scripted(p) => p.rc(t),
            AnyPilot::GustHold(p) => p.rc(t),
            #[cfg(feature = "joystick")]
            AnyPilot::Joystick(p) => p.rc(t),
        }
    }
    fn setpoint(&mut self, t: Micros) -> Option<TrajectorySetpoint> {
        match self {
            AnyPilot::Scripted(p) => p.setpoint(t),
            AnyPilot::GustHold(p) => p.setpoint(t),
            #[cfg(feature = "joystick")]
            AnyPilot::Joystick(p) => p.setpoint(t),
        }
    }
}

/// Converts NED/FRD vectors for Rerun, whose world is set to the same axes.
fn v3(v: oxide_core::math::Vec3<f64>) -> [f32; 3] {
    let f = v.cast::<f32>();
    [f.x, f.y, f.z]
}

fn log_static_scene(rec: &rerun::RecordingStream) -> Result<(), Box<dyn Error>> {
    // World axes: x North, y East, z Down, same as the simulator.
    rec.log_static("world", &rerun::ViewCoordinates::FRD())?;
    rec.log_static(
        "world/ground",
        &rerun::Boxes3D::from_centers_and_half_sizes([[0.0, 0.0, 0.01]], [[10.0, 10.0, 0.01]])
            .with_colors([rerun::Color::from_rgb(70, 90, 70)]),
    )?;
    log_body(rec, "world/quad/body", true)?;
    log_body(rec, "world/estimate/body", false)?;
    Ok(())
}

/// Body: frame plate plus two arms, in the vehicle's FRD frame. The true
/// vehicle is orange; the estimate's box is blue so the two can be told apart.
fn log_body(rec: &rerun::RecordingStream, path: &str, truth: bool) -> Result<(), Box<dyn Error>> {
    let (plate, arms) = if truth {
        (
            rerun::Color::from_rgb(220, 120, 40),
            rerun::Color::from_rgb(60, 60, 60),
        )
    } else {
        (
            rerun::Color::from_rgb(60, 140, 230),
            rerun::Color::from_rgb(40, 80, 140),
        )
    };
    rec.log_static(
        path,
        &rerun::Boxes3D::from_centers_and_half_sizes(
            [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            [
                [0.06, 0.03, 0.015],
                [0.08, 0.008, 0.005],
                [0.008, 0.08, 0.005],
            ],
        )
        .with_rotation_axis_angles([
            rerun::RotationAxisAngle::new([0.0, 0.0, 1.0], rerun::Angle::from_degrees(0.0)),
            rerun::RotationAxisAngle::new([0.0, 0.0, 1.0], rerun::Angle::from_degrees(45.0)),
            rerun::RotationAxisAngle::new([0.0, 0.0, 1.0], rerun::Angle::from_degrees(45.0)),
        ])
        .with_colors([plate, arms, arms]),
    )?;
    Ok(())
}

/// Length of the wind arrow, m per m/s of wind.
const WIND_ARROW_SCALE: f32 = 0.3;

/// Angle of the rotation between two attitudes, degrees.
fn attitude_error_deg(truth: oxide_core::math::Quat<f64>, est: oxide_core::math::Quat<f64>) -> f64 {
    (est * truth.conj())
        .to_rotation_vector()
        .norm()
        .to_degrees()
}

fn log_step(
    rec: &rerun::RecordingStream,
    r: &StepRecord<4>,
    trail: &[[f32; 3]],
    est_trail: &[[f32; 3]],
) -> Result<(), Box<dyn Error>> {
    rec.set_duration_secs("sim_time", r.t.as_secs_f64());
    let q = r.attitude.cast::<f32>();
    rec.log(
        "world/quad",
        &rerun::Transform3D::from_translation_rotation(
            v3(r.position),
            rerun::Quaternion::from_xyzw([q.x, q.y, q.z, q.w]),
        ),
    )?;
    // Thrust arrows at the rotors (length ∝ command), drawn upwards (−z).
    let rotors = [
        [0.0795, 0.0795],
        [-0.0795, -0.0795],
        [0.0795, -0.0795],
        [-0.0795, 0.0795],
    ];
    let origins: Vec<[f32; 3]> = rotors.iter().map(|p| [p[0], p[1], 0.0]).collect();
    let vectors: Vec<[f32; 3]> = r.motors.iter().map(|m| [0.0, 0.0, -0.25 * m]).collect();
    rec.log(
        "world/quad/thrust",
        &rerun::Arrows3D::from_vectors(vectors).with_origins(origins),
    )?;
    if trail.len() >= 2 {
        rec.log("world/trail", &rerun::LineStrips3D::new([trail.to_vec()]))?;
    }
    if let Some(sp) = r.setpoint {
        rec.log(
            "world/setpoint",
            &rerun::Points3D::new([[sp.position.x, sp.position.y, sp.position.z]])
                .with_radii([0.05])
                .with_colors([rerun::Color::from_rgb(40, 200, 80)]),
        )?;
    }
    // Wind: an arrow at the vehicle, pointing where the air moves.
    let w = v3(r.wind);
    rec.log(
        "world/wind",
        &rerun::Arrows3D::from_vectors([[
            w[0] * WIND_ARROW_SCALE,
            w[1] * WIND_ARROW_SCALE,
            w[2] * WIND_ARROW_SCALE,
        ]])
        .with_origins([v3(r.position)])
        .with_colors([rerun::Color::from_rgb(90, 200, 230)]),
    )?;
    rec.log(
        "plots/wind_speed_mps",
        &rerun::Scalars::new([r.wind.norm()]),
    )?;
    // Estimate: second box, trail and error plots (only with a published estimate).
    if let Some(est) = r.estimate {
        let eq = est.attitude.cast::<f32>();
        rec.log(
            "world/estimate",
            &rerun::Transform3D::from_translation_rotation(
                [est.position.x, est.position.y, est.position.z],
                rerun::Quaternion::from_xyzw([eq.x, eq.y, eq.z, eq.w]),
            ),
        )?;
        if est_trail.len() >= 2 {
            rec.log(
                "world/estimate_trail",
                &rerun::LineStrips3D::new([est_trail.to_vec()])
                    .with_colors([rerun::Color::from_rgb(60, 140, 230)]),
            )?;
        }
        let ep = est.position.cast::<f64>() - r.position;
        rec.log(
            "plots/est_err_horizontal_m",
            &rerun::Scalars::new([oxide_core::math::Vec3::new(ep.x, ep.y, 0.0).norm()]),
        )?;
        rec.log(
            "plots/est_err_vertical_m",
            &rerun::Scalars::new([ep.z.abs()]),
        )?;
        rec.log(
            "plots/est_err_attitude_deg",
            &rerun::Scalars::new([attitude_error_deg(r.attitude, est.attitude.cast::<f64>())]),
        )?;
    }
    let e = r.attitude.to_euler();
    rec.log("plots/altitude_m", &rerun::Scalars::new([-r.position.z]))?;
    rec.log("plots/roll_deg", &rerun::Scalars::new([e.x.to_degrees()]))?;
    rec.log("plots/pitch_deg", &rerun::Scalars::new([e.y.to_degrees()]))?;
    rec.log("plots/yaw_deg", &rerun::Scalars::new([e.z.to_degrees()]))?;
    rec.log("plots/battery_v", &rerun::Scalars::new([r.battery_v]))?;
    for (i, m) in r.motors.iter().enumerate() {
        rec.log(
            format!("plots/motor/{i}"),
            &rerun::Scalars::new([f64::from(*m)]),
        )?;
    }
    Ok(())
}

fn run(args: &Args) -> Result<(), Box<dyn Error>> {
    let airframe = Airframe::reference_quad()?;
    let mut options = SimOptions::default();
    let (pilot, duration) = if args.joystick {
        #[cfg(feature = "joystick")]
        {
            (
                AnyPilot::Joystick(Box::new(joystick::JoystickPilot::new()?)),
                None,
            )
        }
        #[cfg(not(feature = "joystick"))]
        {
            return Err("built without the `joystick` feature".into());
        }
    } else {
        match args.scenario {
            Scenario::HoverStep => {
                let s = HoverStep::default();
                let d = Micros((s.duration_s * 1e6) as u64);
                (AnyPilot::Scripted(s), Some(d))
            }
            Scenario::GustHold => {
                let (s, o) = gust_hold(args.seed);
                options = o;
                let d = Micros((s.duration_s * 1e6) as u64);
                (AnyPilot::GustHold(s), Some(d))
            }
        }
    };
    let mut sim = Simulation::<_, 4>::with_options(&airframe, args.profile, pilot, options)?;

    let builder = rerun::RecordingStreamBuilder::new("airborne_oxide");
    let rec = match &args.save {
        Some(path) => builder.save(path)?,
        None => builder.spawn()?,
    };
    log_static_scene(&rec)?;

    let base_us = u64::from(args.profile.base_period_us);
    let view_every = (1_000_000 / VIEW_RATE_HZ / base_us).max(1);
    let mut trail: Vec<[f32; 3]> = Vec::new();
    let mut est_trail: Vec<[f32; 3]> = Vec::new();
    let start = Instant::now();
    let mut ticks: u64 = 0;
    loop {
        if let Some(d) = duration
            && sim.time() >= d
        {
            break;
        }
        let r = sim.step();
        ticks += 1;
        if ticks.is_multiple_of(view_every) {
            trail.push(v3(r.position));
            if trail.len() > 3000 {
                trail.remove(0);
            }
            if let Some(est) = r.estimate {
                est_trail.push([est.position.x, est.position.y, est.position.z]);
                if est_trail.len() > 3000 {
                    est_trail.remove(0);
                }
            }
            log_step(&rec, &r, &trail, &est_trail)?;
        }
        // Pace to real time × speed (0 = as fast as possible).
        if args.speed > 0.0 && ticks.is_multiple_of(view_every) {
            let target = Duration::from_secs_f64(r.t.as_secs_f64() / args.speed);
            if let Some(wait) = target.checked_sub(start.elapsed()) {
                std::thread::sleep(wait);
            }
        }
    }
    rec.flush_blocking()?;
    println!(
        "flew {:.1} s, trace hash {:016x}",
        sim.time().as_secs_f64(),
        sim.trace_hash()
    );
    Ok(())
}

fn main() -> std::process::ExitCode {
    let args = match parse() {
        Ok(a) => a,
        Err(e) => {
            eprintln!("error: {e}");
            eprintln!(
                "usage: oxide-viewer [--profile f405|h743] [--scenario hover-step|gust-hold] [--seed N] [--joystick] [--speed X] [--save FILE.rrd]"
            );
            return std::process::ExitCode::from(2);
        }
    };
    match run(&args) {
        Ok(()) => std::process::ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("error: {e}");
            std::process::ExitCode::FAILURE
        }
    }
}
