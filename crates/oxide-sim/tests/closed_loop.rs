// SPDX-License-Identifier: AGPL-3.0-or-later
//! Closed-loop requirements for M1a: the core flies the reference quad on
//! true state (DESIGN §13). Both build profiles must pass.

use oxide_core::io::{RcSample, TrajectorySetpoint};
use oxide_core::math::Vec3;
use oxide_core::schedule::{F405_LEAN, H743_FULL, Profile};
use oxide_core::time::Micros;
use oxide_sim::scenario::{HoverStep, Pilot};
use oxide_sim::{Airframe, Simulation, StepRecord};

fn fly<P: Pilot>(profile: Profile, pilot: P, seconds: u64) -> (Vec<StepRecord<4>>, u64) {
    let a = Airframe::reference_quad().expect("reference airframe");
    let mut sim = Simulation::<_, 4>::new(&a, profile, pilot).expect("sim");
    let mut log = Vec::new();
    sim.run_until(Micros::from_secs(seconds), |r| log.push(*r));
    (log, sim.trace_hash())
}

fn at(log: &[StepRecord<4>], t: f64) -> &StepRecord<4> {
    let i = log.partition_point(|r| r.t.as_secs_f64() < t);
    &log[i.min(log.len() - 1)]
}

fn window(log: &[StepRecord<4>], t0: f64, t1: f64) -> impl Iterator<Item = &StepRecord<4>> {
    log.iter()
        .filter(move |r| (t0..t1).contains(&r.t.as_secs_f64()))
}

fn hover_step_meets_requirements(profile: Profile) {
    let (log, _) = fly(profile, HoverStep::default(), 20);
    let name = profile.name;

    for r in window(&log, 0.0, 1.5) {
        assert!(
            !r.status.armed && r.motors == [0.0; 4],
            "{name}: motors must be off before arming"
        );
    }
    for r in &log {
        assert!(
            r.position.is_finite() && r.attitude.is_finite(),
            "{name}: state went non-finite"
        );
        assert_eq!(r.status.timing_faults, 0, "{name}: timing fault");
        let e = r.attitude.to_euler();
        assert!(
            e.x.abs().max(e.y.abs()) < 36f64.to_radians(),
            "{name}: tilt limit exceeded at {:?}",
            r.t
        );
    }

    // Takeoff to 2 m (setpoint at t = 1 s).
    let z = at(&log, 5.9).position.z;
    assert!((z + 2.0).abs() < 0.02, "{name}: altitude {z}");
    let overshoot = window(&log, 1.0, 6.0)
        .map(|r| -r.position.z)
        .fold(0.0, f64::max)
        - 2.0;
    assert!(overshoot < 0.1, "{name}: takeoff overshoot {overshoot}");

    // 1 m step north (t = 6 s).
    let x = at(&log, 10.9).position.x;
    assert!((x - 1.0).abs() < 0.02, "{name}: north {x}");
    let peak = window(&log, 6.0, 11.0)
        .map(|r| r.position.x)
        .fold(0.0, f64::max);
    assert!(peak < 1.05, "{name}: north overshoot {peak}");

    // 90° yaw step (t = 11 s) without wandering off.
    let yaw = at(&log, 14.9).attitude.yaw().to_degrees();
    assert!((yaw - 90.0).abs() < 1.0, "{name}: yaw {yaw}");
    let drift = window(&log, 11.0, 15.0)
        .map(|r| (r.position.x - 1.0).abs())
        .fold(0.0, f64::max);
    assert!(drift < 0.02, "{name}: drift during yaw {drift}");

    // Combined step (t = 15 s).
    let err = (at(&log, 19.9).position - Vec3::new(1.0, 1.0, -3.0)).norm();
    assert!(err < 0.02, "{name}: final position error {err}");
}

#[test]
fn hover_step_f405() {
    hover_step_meets_requirements(F405_LEAN);
}

#[test]
fn hover_step_h743() {
    hover_step_meets_requirements(H743_FULL);
}

#[test]
fn runs_are_bit_identical() {
    let (_, a) = fly(F405_LEAN, HoverStep::default(), 20);
    let (_, b) = fly(F405_LEAN, HoverStep::default(), 20);
    assert_eq!(a, b);
    let (_, c) = fly(H743_FULL, HoverStep::default(), 20);
    assert_ne!(a, c, "different profiles must give different traces");
}

/// Hover-step script with the kill switch thrown at 8 s.
struct KillAt8(HoverStep);

impl Pilot for KillAt8 {
    fn rc(&mut self, t: Micros) -> Option<RcSample> {
        self.0.rc(t).map(|mut r| {
            r.kill_switch = t.as_secs_f64() >= 8.0;
            r
        })
    }
    fn setpoint(&mut self, t: Micros) -> Option<TrajectorySetpoint> {
        self.0.setpoint(t)
    }
}

#[test]
fn kill_switch_stops_motors_in_flight() {
    let (log, _) = fly(F405_LEAN, KillAt8(HoverStep::default()), 12);
    assert!(
        at(&log, 7.9).status.armed && at(&log, 7.9).position.z < -1.5,
        "should be flying before the kill"
    );
    // Three confirming supervisor runs at 50 Hz plus one LET period.
    for r in window(&log, 8.1, 12.0) {
        assert_eq!(
            r.motors, [0.0; 4],
            "motors must stay off after kill at {:?}",
            r.t
        );
        assert!(r.status.killed && !r.status.armed);
    }
    assert!(
        at(&log, 11.9).position.z.abs() < 1e-9,
        "vehicle should be on the ground"
    );
}
