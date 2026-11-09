// SPDX-License-Identifier: AGPL-3.0-or-later
//! `oxide-sim`: run a scenario headless.
//!
//! ```text
//! oxide-sim run  [--profile f405|h743] [--airframe FILE] [--csv FILE]
//! oxide-sim hash [--profile f405|h743] [--airframe FILE]
//! ```
//! `hash` prints only the trace hash, which CI compares between Linux and
//! Windows (DESIGN §5.2).

use std::io::Write;
use std::process::ExitCode;
use std::time::Instant;

use oxide_core::schedule::{F405_LEAN, H743_FULL, Profile};
use oxide_core::time::Micros;
use oxide_sim::scenario::HoverStep;
use oxide_sim::{Airframe, SimError, Simulation, StepRecord};

struct Args {
    command: String,
    profile: Profile,
    airframe: Option<String>,
    csv: Option<String>,
}

fn parse() -> Result<Args, String> {
    let mut it = std::env::args().skip(1);
    let command = it.next().ok_or("missing command: run | hash")?;
    let mut args = Args {
        command,
        profile: F405_LEAN,
        airframe: None,
        csv: None,
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

fn csv_row(r: &StepRecord<4>) -> String {
    let e = r.attitude.to_euler();
    let sp = r.setpoint.map(|s| s.position);
    let f = |v: Option<f32>| v.map_or(String::new(), |x| format!("{x:.4}"));
    format!(
        "{:.4},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.4},{:.4},{:.4},{:.4},{:.3},{:.3},{},{},{},{}\n",
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
    )
}

fn run(args: &Args) -> Result<(), Box<dyn std::error::Error>> {
    let airframe = load_airframe(args.airframe.as_deref())?;
    let scenario = HoverStep::default();
    let duration_us = (scenario.duration_s * 1e6) as u64;
    let mut sim = Simulation::<_, 4>::new(&airframe, args.profile, scenario)?;

    let mut csv = match &args.csv {
        Some(p) => {
            let mut f = std::io::BufWriter::new(std::fs::File::create(p)?);
            f.write_all(
                b"t,x,y,z,vx,vy,vz,roll,pitch,yaw,p,q,r,m0,m1,m2,m3,battery_v,battery_a,armed,sp_x,sp_y,sp_z\n",
            )?;
            Some(f)
        }
        None => None,
    };
    let mut max_tilt = 0.0_f64;
    let mut io_error = None;
    let start = Instant::now();
    sim.run_until(Micros(duration_us), |r| {
        let e = r.attitude.to_euler();
        max_tilt = max_tilt.max(e.x.abs()).max(e.y.abs());
        if let Some(f) = csv.as_mut()
            && let Err(err) = f.write_all(csv_row(r).as_bytes())
        {
            io_error.get_or_insert(err);
        }
    });
    let wall = start.elapsed().as_secs_f64();
    if let Some(err) = io_error {
        return Err(err.into());
    }
    if let Some(mut f) = csv {
        f.flush()?;
    }

    let hash = sim.trace_hash();
    if args.command == "hash" {
        println!("{} {:016x}", args.profile.name, hash);
        return Ok(());
    }
    let b = sim.body();
    let sim_s = sim.time().as_secs_f64();
    println!("profile          {}", args.profile.name);
    println!("airframe         {}", airframe.name);
    println!(
        "simulated        {sim_s:.1} s in {wall:.3} s wall ({:.0}x real time)",
        sim_s / wall.max(1e-9)
    );
    println!(
        "final position   N {:.3}  E {:.3}  D {:.3} m (target N 1.000  E 1.000  D -3.000)",
        b.position.x, b.position.y, b.position.z
    );
    println!(
        "final yaw        {:.2} deg (target 90.00)",
        b.attitude.yaw().to_degrees()
    );
    println!("max tilt         {:.1} deg", max_tilt.to_degrees());
    println!("trace hash       {hash:016x}");
    Ok(())
}

fn main() -> ExitCode {
    let args = match parse() {
        Ok(a) => a,
        Err(e) => {
            eprintln!("error: {e}");
            eprintln!(
                "usage: oxide-sim run|hash [--profile f405|h743] [--airframe FILE] [--csv FILE]"
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
