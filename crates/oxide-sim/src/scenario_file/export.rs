// SPDX-License-Identifier: AGPL-3.0-or-later
//! The exporter: writes a scenario file from the in-code definition. It
//! generated `scenarios/*.toml` and is kept (tests only) until the Rust
//! definitions are deleted. Numbers are written in Rust's shortest form
//! (`{:?}`), so reading them back gives the same bits.

use std::fmt::Write as _;

use oxide_core::config::{EstimatorSource, FailsafeParams};
use oxide_core::io::Reasons;
use oxide_core::params::{self, PARAMS, ParamValue};

use super::*;
use crate::fault_scenarios::{self, FaultScenario};

/// An `f64` in its shortest form that reads back as the same bits.
fn num(v: f64) -> String {
    assert!(v.is_finite(), "cannot write {v}");
    let s = format!("{v:?}");
    assert_eq!(s.parse::<f64>().ok().map(f64::to_bits), Some(v.to_bits()));
    s
}

/// An `f32` in its shortest form; the loader reads it as an `f64` and
/// narrows it, which must give the same `f32`.
fn num32(v: f32) -> String {
    assert!(v.is_finite(), "cannot write {v}");
    let s = format!("{v:?}");
    let back = s.parse::<f64>().expect("a float") as f32;
    assert_eq!(
        back.to_bits(),
        v.to_bits(),
        "{s} does not read back as {v:?}"
    );
    s
}

fn array(v: &[f64]) -> String {
    let items: Vec<String> = v.iter().map(|x| num(*x)).collect();
    format!("[{}]", items.join(", "))
}

fn quoted(s: &str) -> String {
    let mut out = String::from("\"");
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            c => out.push(c),
        }
    }
    out.push('"');
    out
}

fn name_list<T: Copy + PartialEq>(table: &[(T, &'static str)], items: &[T]) -> String {
    let names: Vec<String> = items
        .iter()
        .map(|i| quoted(name_of(table, *i).expect("a named value")))
        .collect();
    format!("[{}]", names.join(", "))
}

fn reason_list(r: Reasons) -> String {
    let names: Vec<String> = REASON_NAMES
        .iter()
        .filter(|(bit, _)| r.contains(*bit))
        .map(|(_, n)| quoted(n))
        .collect();
    format!("[{}]", names.join(", "))
}

fn fault_name(k: &FaultKind) -> &'static str {
    match k {
        FaultKind::MotorLoss { .. } => "motor-loss",
        FaultKind::GpsLoss => "gps-loss",
        FaultKind::ImuLoss => "imu-loss",
        FaultKind::BaroLoss => "baro-loss",
        FaultKind::MagLoss => "mag-loss",
        FaultKind::RcLoss => "rc-loss",
        FaultKind::RcSilent => "rc-silent",
        FaultKind::GcsLoss => "gcs-loss",
        FaultKind::BrainCrash => "brain-crash",
        FaultKind::BrainHang => "brain-hang",
    }
}

fn bool_s(b: bool) -> &'static str {
    if b { "true" } else { "false" }
}

/// The file text of `s`.
pub fn to_toml(s: &ScenarioFile) -> String {
    let mut o = String::new();
    let w = &mut o;
    macro_rules! line {
        ($($arg:tt)*) => { writeln!(w, $($arg)*).expect("write to a string") };
    }
    line!("# SPDX-License-Identifier: AGPL-3.0-or-later");
    line!("name = {}", quoted(&s.name));
    line!("about = {}", quoted(&s.about));
    line!("airframe = {}", quoted(&s.airframe));
    line!("duration_s = {}", num(s.duration_s));

    line!("\n[sim]");
    let sensors = match s.sensors {
        SensorPreset::Ideal => "ideal",
        SensorPreset::Realistic => "realistic",
    };
    let estimator = match s.estimator {
        EstimatorSource::External => "external",
        EstimatorSource::Ekf => "ekf",
    };
    line!("sensors = \"{sensors}\"");
    line!("estimator = \"{estimator}\"");
    line!("\n[sim.links]");
    line!("gcs = {}", bool_s(s.links.gcs));
    line!("brain = {}", bool_s(s.links.brain));
    line!("rc_failsafe_delay_us = {}", s.links.rc_failsafe_delay_us);

    match &s.wind {
        WindSpec::Calm => line!("\n[wind]\nkind = \"calm\""),
        WindSpec::Fixed(c) => {
            line!("\n[wind]\nkind = \"fixed\"");
            line!(
                "steady_ned = {}",
                array(&[c.steady_ned.x, c.steady_ned.y, c.steady_ned.z])
            );
            line!(
                "turbulence_sigma = {}",
                array(&[
                    c.turbulence_sigma.x,
                    c.turbulence_sigma.y,
                    c.turbulence_sigma.z
                ])
            );
            line!("turbulence_tau_s = {}", num(c.turbulence_tau_s));
            for g in &c.gusts {
                line!("\n[[wind.gust]]");
                line!("start_s = {}", num(g.start_s));
                line!("duration_s = {}", num(g.duration_s));
                line!(
                    "peak_ned = {}",
                    array(&[g.peak_ned.x, g.peak_ned.y, g.peak_ned.z])
                );
            }
        }
        WindSpec::Random(r) => {
            line!("\n[wind]\nkind = \"random\"");
            line!("steady_speed_mps = {}", array(&r.steady_speed_mps));
            line!("sigma_h_mps = {}", array(&r.sigma_h_mps));
            line!("sigma_v_ratio = {}", num(r.sigma_v_ratio));
            line!("tau_s = {}", num(r.tau_s));
            if let Some(g) = &r.gusts {
                line!("\n[wind.gusts]");
                line!("count = {}", g.count);
                line!("start_s = {}", array(&g.start_s));
                line!("duration_s = {}", array(&g.duration_s));
                line!("peak_h_mps = {}", array(&g.peak_h_mps));
                line!("peak_v_mps = {}", array(&g.peak_v_mps));
            }
        }
    }

    if let Some(b) = &s.battery {
        line!("\n[battery]");
        if let Some(c) = b.capacity_mah {
            line!("capacity_mah = {}", num(c));
        }
        if let Some(c) = b.initial_state_of_charge {
            line!("initial_state_of_charge = {}", num(c));
        }
    }

    if !s.params.is_empty() {
        line!("\n[params]");
        for (name, v) in &s.params {
            line!("{name} = {}", num(*v));
        }
    }

    for f in &s.faults {
        line!("\n[[fault]]");
        line!("kind = \"{}\"", fault_name(&f.kind));
        if let FaultKind::MotorLoss { rotor, scale } = f.kind {
            line!("rotor = {rotor}");
            line!("scale = {}", num(scale));
        }
        line!("start_s = {}", num(f.start_s));
        if let Some(e) = f.end_s {
            line!("end_s = {}", num(e));
        }
    }

    let p = &s.pilot;
    line!("\n[pilot]");
    if p.arm_at_s.is_finite() {
        line!("arm_at_s = {}", num(p.arm_at_s));
    }
    if let Some(t) = p.disarm_at_s {
        line!("disarm_at_s = {}", num(t));
    }
    if let Some(t) = p.kill_at_s {
        line!("kill_at_s = {}", num(t));
    }
    for (t, m) in &p.mode {
        line!("\n[[pilot.mode]]");
        line!("t = {}", num(*t));
        line!("switch = \"{}\"", name_of(&MODE_NAMES, *m).expect("a mode"));
    }
    for (t, st) in &p.sticks {
        line!("\n[[pilot.sticks]]");
        line!("t = {}", num(*t));
        line!("roll = {}", num32(st[0]));
        line!("pitch = {}", num32(st[1]));
        line!("yaw = {}", num32(st[2]));
        line!("throttle = {}", num32(st[3]));
    }
    for (t, sp) in &p.setpoints {
        line!("\n[[pilot.setpoint]]");
        line!("t = {}", num(*t));
        match sp {
            None => line!("silent = true"),
            Some(sp) => {
                assert_eq!(
                    sp.velocity,
                    Vec3::zero(),
                    "files carry no feed-forward velocity"
                );
                line!(
                    "position = [{}, {}, {}]",
                    num32(sp.position.x),
                    num32(sp.position.y),
                    num32(-sp.position.z)
                );
                line!("yaw = {}", num32(sp.yaw));
            }
        }
    }

    if let Some(e) = &s.expect {
        line!("\n[expect]");
        if !e.nav_contains.is_empty() {
            line!(
                "nav_contains = {}",
                name_list(&NAV_STATE_NAMES, &e.nav_contains)
            );
        }
        if !e.nav_never.is_empty() {
            line!("nav_never = {}", name_list(&NAV_STATE_NAMES, &e.nav_never));
        }
        if !e.reasons.is_empty() {
            line!("reasons = {}", reason_list(e.reasons));
        }
        if !e.reasons_never.is_empty() {
            line!("reasons_never = {}", reason_list(e.reasons_never));
        }
        if !e.events.is_empty() {
            line!("events = {}", name_list(&EVENT_NAMES, &e.events));
        }
        if e.min_action != Action::None {
            line!(
                "min_action = \"{}\"",
                name_of(&ACTION_NAMES, e.min_action).expect("an action")
            );
        }
        line!("by_s = {}", num(e.by_s));
        line!("max_touchdown_mps = {}", num(e.max_touchdown_mps));
        line!("max_tilt_deg = {}", num(e.max_tilt_deg));
        if let Some(v) = e.min_height_m {
            line!("min_height_m = {}", num(v));
        }
        if let Some(v) = e.max_height_m {
            line!("max_height_m = {}", num(v));
        }
        if let Some(v) = e.max_distance_from_home_m {
            line!("max_distance_from_home_m = {}", num(v));
        }
        line!("\n[expect.ending]");
        match e.ending {
            Ending::Flying { point, tolerance_m } => {
                line!("kind = \"flying\"");
                line!("point = {}", array(&[point.x, point.y, -point.z]));
                line!("tolerance_m = {}", num(tolerance_m));
            }
            Ending::FlyingAtHeight {
                height_m,
                tolerance_m,
            } => {
                line!("kind = \"flying-at-height\"");
                line!("height_m = {}", num(height_m));
                line!("tolerance_m = {}", num(tolerance_m));
            }
            Ending::Landed { within_m_of_home } => {
                line!("kind = \"landed\"");
                if let Some(d) = within_m_of_home {
                    line!("within_m_of_home = {}", num(d));
                }
            }
            Ending::Terminated => line!("kind = \"terminated\""),
            Ending::Disarmed(r) => {
                line!("kind = \"disarmed\"");
                line!(
                    "reason = \"{}\"",
                    name_of(&DISARM_REASON_NAMES, r).expect("a disarm reason")
                );
            }
            Ending::Down => line!("kind = \"down\""),
        }
    }

    line!("\n[run]");
    line!("seeds = {}", s.run.seeds);
    let profiles: Vec<String> = s.run.profiles.iter().map(|p| quoted(p.name())).collect();
    line!("profiles = [{}]", profiles.join(", "));
    o
}

/// The parameter overrides that turn the default configuration into one with
/// `failsafe`, by name, in table order.
fn failsafe_params(failsafe: &FailsafeParams) -> Vec<(String, f64)> {
    let airframe = Airframe::reference_quad().expect("reference airframe");
    let base = airframe.core_config::<ROTORS>().expect("reference config");
    let mut changed = base;
    changed.failsafe = *failsafe;
    let mut out = Vec::new();
    for def in PARAMS.iter() {
        let id = params::find(def.name).expect("a table entry");
        let a = params::get(&base, id).expect("a value");
        let b = params::get(&changed, id).expect("a value");
        if a != b {
            let v = match b {
                ParamValue::F32(v) => format!("{v:?}").parse::<f64>().expect("a float"),
                ParamValue::I32(v) => f64::from(v),
            };
            out.push((def.name.to_string(), v));
        }
    }
    // The reader returns a table in name order; write it that way.
    out.sort_by(|a, b| a.0.cmp(&b.0));
    // The overrides must rebuild exactly this failsafe set.
    let mut cfg = base;
    for (name, v) in &out {
        apply_param(&mut cfg, &F405_LEAN, name, *v).expect("an accepted override");
    }
    assert_eq!(
        cfg.failsafe, *failsafe,
        "some failsafe field is not a parameter"
    );
    out
}

/// A fault scenario as a file.
pub fn from_fault_scenario(s: &FaultScenario) -> ScenarioFile {
    let reference = Airframe::reference_quad()
        .expect("reference airframe")
        .battery;
    let battery = s.battery.as_ref().map(|b| {
        assert_eq!(
            (b.cells, b.internal_resistance_ohm, b.nominal_voltage_v),
            (
                reference.cells,
                reference.internal_resistance_ohm,
                reference.nominal_voltage_v
            ),
            "{}: the battery differs from the airframe's in more than capacity and charge",
            s.name
        );
        BatteryOverride {
            capacity_mah: Some(b.capacity_mah),
            initial_state_of_charge: Some(b.initial_state_of_charge),
        }
    });
    ScenarioFile {
        name: s.name.to_string(),
        about: s.about.to_string(),
        airframe: REFERENCE_AIRFRAME.to_string(),
        duration_s: s.script.duration_s,
        sensors: SensorPreset::Realistic,
        estimator: EstimatorSource::Ekf,
        links: s.links,
        // `fault_scenarios::fault_wind`.
        wind: WindSpec::Random(RandomWind {
            steady_speed_mps: [0.0, 2.0],
            sigma_h_mps: [0.1, 0.4],
            sigma_v_ratio: 0.5,
            tau_s: 2.0,
            gusts: None,
        }),
        battery,
        params: s.failsafe.as_ref().map(failsafe_params).unwrap_or_default(),
        faults: s.faults.clone(),
        pilot: s.script.clone(),
        expect: Some(s.expect.clone()),
        hold: None,
        run: RunSpec {
            seeds: 10,
            profiles: vec![RunProfile::F405, RunProfile::H743],
        },
    }
}

/// Every scenario of the suite as a file: the 20 named failures and the 21
/// pairings.
pub fn fault_files() -> Vec<ScenarioFile> {
    fault_scenarios::all()
        .iter()
        .chain(fault_scenarios::pairs().iter())
        .map(from_fault_scenario)
        .collect()
}

/// Every scenario there is.
pub fn all_files() -> Vec<ScenarioFile> {
    fault_files()
}
