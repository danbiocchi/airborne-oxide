// SPDX-License-Identifier: AGPL-3.0-or-later
//! `oxide-viewer`: fly the simulator in real time and watch it in Rerun.
//!
//! ```text
//! oxide-viewer [--profile f405|h743] [--joystick] [--speed X] [--save FILE.rrd]
//! ```
//! Without `--joystick` it flies the scripted hover-and-step scenario. With
//! `--joystick` a USB joystick (the Taranis in USB-joystick mode) is the pilot.
//! Rerun is a visualiser only; it never feeds back into the simulation.

#[cfg(feature = "joystick")]
mod joystick;

use std::error::Error;
use std::time::{Duration, Instant};

use oxide_core::io::{RcSample, TrajectorySetpoint};
use oxide_core::schedule::{F405_LEAN, H743_FULL, Profile};
use oxide_core::time::Micros;
use oxide_sim::scenario::{HoverStep, Pilot};
use oxide_sim::{Airframe, Simulation, StepRecord};

/// Rerun is updated at this rate, Hz.
const VIEW_RATE_HZ: u64 = 50;

struct Args {
    profile: Profile,
    joystick: bool,
    speed: f64,
    save: Option<String>,
}

fn parse() -> Result<Args, String> {
    let mut args = Args {
        profile: F405_LEAN,
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
    #[cfg(feature = "joystick")]
    Joystick(Box<joystick::JoystickPilot>),
}

impl Pilot for AnyPilot {
    fn rc(&mut self, t: Micros) -> Option<RcSample> {
        match self {
            AnyPilot::Scripted(p) => p.rc(t),
            #[cfg(feature = "joystick")]
            AnyPilot::Joystick(p) => p.rc(t),
        }
    }
    fn setpoint(&mut self, t: Micros) -> Option<TrajectorySetpoint> {
        match self {
            AnyPilot::Scripted(p) => p.setpoint(t),
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
    // Body: frame plate plus two arms, in the vehicle's FRD frame.
    rec.log_static(
        "world/quad/body",
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
        .with_colors([
            rerun::Color::from_rgb(220, 120, 40),
            rerun::Color::from_rgb(60, 60, 60),
            rerun::Color::from_rgb(60, 60, 60),
        ]),
    )?;
    Ok(())
}

fn log_step(
    rec: &rerun::RecordingStream,
    r: &StepRecord<4>,
    trail: &[[f32; 3]],
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
        let s = HoverStep::default();
        let d = Micros((s.duration_s * 1e6) as u64);
        (AnyPilot::Scripted(s), Some(d))
    };
    let mut sim = Simulation::<_, 4>::new(&airframe, args.profile, pilot)?;

    let builder = rerun::RecordingStreamBuilder::new("airborne_oxide");
    let rec = match &args.save {
        Some(path) => builder.save(path)?,
        None => builder.spawn()?,
    };
    log_static_scene(&rec)?;

    let base_us = u64::from(args.profile.base_period_us);
    let view_every = (1_000_000 / VIEW_RATE_HZ / base_us).max(1);
    let mut trail: Vec<[f32; 3]> = Vec::new();
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
            log_step(&rec, &r, &trail)?;
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
                "usage: oxide-viewer [--profile f405|h743] [--joystick] [--speed X] [--save FILE.rrd]"
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
