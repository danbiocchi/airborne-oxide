// SPDX-License-Identifier: AGPL-3.0-or-later
//! Analytic oracles for the physics (DESIGN §5.8): cases with known answers.

use oxide_core::math::{Quat, Real, Vec3};
use oxide_core::schedule::F405_LEAN;
use oxide_core::time::Micros;
use oxide_sim::scenario::NoPilot;
use oxide_sim::vehicle::{BodyState, GRAVITY, VehicleModel};
use oxide_sim::{Airframe, Simulation};

fn model() -> VehicleModel<4> {
    let a = Airframe::reference_quad().expect("reference airframe");
    VehicleModel::from_airframe(&a).expect("model")
}

#[test]
fn free_fall_matches_closed_form() {
    let a = Airframe::reference_quad().expect("reference airframe");
    let mut sim = Simulation::<_, 4>::new(&a, F405_LEAN, NoPilot).expect("sim");
    sim.model_mut().drag_area = Vec3::zero();
    sim.set_body(BodyState {
        position: Vec3::new(0.0, 0.0, -100.0),
        ..BodyState::default()
    });
    sim.run_until(Micros::from_secs(1), |_| {});
    let z = sim.body().position.z;
    let expected = -100.0 + 0.5 * GRAVITY * 1.0 * 1.0;
    assert!((z - expected).abs() < 1e-9, "z = {z}, expected {expected}");
    assert!((sim.body().velocity.z - GRAVITY).abs() < 1e-9);
}

#[test]
fn torque_free_rotation_conserves_energy_and_momentum() {
    let mut m = model();
    m.gravity = false;
    m.drag_area = Vec3::zero();
    m.inertia = Vec3::new(1.6e-3, 2.0e-3, 2.8e-3);
    let mut s = BodyState::<4> {
        rate: Vec3::new(0.1, 3.0, 0.2),
        ..BodyState::default()
    };
    let energy = |s: &BodyState<4>| 0.5 * s.rate.dot(m.inertia.hadamard(s.rate));
    let momentum = |s: &BodyState<4>| s.attitude.rotate(m.inertia.hadamard(s.rate));
    let (e0, l0) = (energy(&s), momentum(&s));
    for _ in 0..10_000 {
        s = m.rk4(&s, &[0.0; 4], 1e-3);
    }
    let (e1, l1) = (energy(&s), momentum(&s));
    assert!(
        ((e1 - e0) / e0).abs() < 1e-6,
        "energy drift {}",
        (e1 - e0) / e0
    );
    assert!(
        (l1 - l0).norm() / l0.norm() < 1e-6,
        "momentum drift {}",
        (l1 - l0).norm() / l0.norm()
    );
}

#[test]
fn hover_thrust_balances_weight_with_no_torque() {
    let m = model();
    let n = Real::sqrt(m.mass_kg * GRAVITY / (4.0 * m.rotors[0].max_thrust_n));
    let s = BodyState::<4> {
        rotor_speed: [n; 4],
        ..BodyState::default()
    };
    let loads = m.loads(&s);
    assert!((loads.force_body.z + m.mass_kg * GRAVITY).abs() < 1e-12);
    assert!(loads.torque_body.norm() < 1e-15, "{:?}", loads.torque_body);
}

#[test]
fn motor_reaches_commanded_speed() {
    let mut m = model();
    m.gravity = false;
    let mut s = BodyState::<4>::default();
    let cmd = [0.6; 4];
    let tau = m.motor_time_constant_s;
    let steps = (10.0 * tau / 1e-3) as usize;
    for _ in 0..steps {
        s = m.rk4(&s, &cmd, 1e-3);
    }
    for v in s.rotor_speed {
        assert!((v - 0.6).abs() < 0.6 * 1e-4 * 1.1, "{v}");
    }
}

#[test]
fn resting_imu_reads_one_g_up() {
    let a = Airframe::reference_quad().expect("reference airframe");
    let mut sim = Simulation::<_, 4>::new(&a, F405_LEAN, NoPilot).expect("sim");
    sim.run_until(Micros::from_millis(50), |_| {});
    let imu = oxide_sim::sensors::imu(Micros::ZERO, sim.body(), Vec3::new(0.0, 0.0, -GRAVITY));
    assert!((imu.value.accel.z + 9.806_65).abs() < 1e-5);
    assert_eq!(sim.body().attitude, Quat::identity());
}
