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
        s = m.rk4(&s, &[0.0; 4], 1e-3, Vec3::zero());
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
    let loads = m.loads(&s, Vec3::zero());
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
        s = m.rk4(&s, &cmd, 1e-3, Vec3::zero());
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

#[test]
fn quadratic_drag_in_steady_wind_matches_closed_form() {
    let mut m = model();
    m.gravity = false;
    m.rotor_drag_coeff = 0.0;
    let wind = Vec3::new(3.0, 0.0, 0.0);
    let k = 0.5 * oxide_sim::vehicle::AIR_DENSITY * m.drag_area.x / m.mass_kg;
    let u0 = 3.0;
    let mut s = BodyState::<4>::default();
    let dt = 1e-3;
    for _ in 0..2000 {
        s = m.rk4(&s, &[0.0; 4], dt, wind);
    }
    let u = wind.x - s.velocity.x;
    let expected = u0 / (1.0 + k * u0 * 2.0);
    assert!(
        ((u - expected) / expected).abs() < 1e-6,
        "u = {u}, expected {expected}"
    );
}

#[test]
fn rotor_drag_decays_velocity_exponentially() {
    let mut m = model();
    m.gravity = false;
    m.drag_area = Vec3::zero();
    let n = 0.5;
    let v0 = 5.0;
    let mut s = BodyState::<4> {
        velocity: Vec3::new(v0, 0.0, 0.0),
        rotor_speed: [n; 4],
        ..BodyState::default()
    };
    let dt = 1e-3;
    for _ in 0..1000 {
        s = m.rk4(&s, &[n; 4], dt, Vec3::zero());
    }
    let expected = v0 * Real::exp(-m.rotor_drag_coeff * 4.0 * n * 1.0 / m.mass_kg);
    assert!(
        ((s.velocity.x - expected) / expected).abs() < 1e-6,
        "v = {}, expected {expected}",
        s.velocity.x
    );
}

// ---- The compliant ground (DESIGN §5.3) ----

/// Steps a model with all motors off.
fn fall(
    m: &VehicleModel<4>,
    mut s: BodyState<4>,
    secs: f64,
    mut each: impl FnMut(&BodyState<4>),
) -> BodyState<4> {
    let dt = 1e-3;
    for _ in 0..(secs / dt) as usize {
        s = m.rk4(&s, &[0.0; 4], dt, Vec3::zero());
        each(&s);
    }
    s
}

fn tilt_deg(s: &BodyState<4>) -> f64 {
    let d = s.attitude.rotate(Vec3::new(0.0, 0.0, 1.0));
    Real::atan2(Real::sqrt(d.x * d.x + d.y * d.y), d.z).to_degrees()
}

#[test]
fn at_rest_on_the_ground_nothing_moves() {
    let m = model();
    let s = fall(&m, BodyState::<4>::default(), 10.0, |_| {});
    assert!(s.position.norm() < 1e-9, "{:?}", s.position);
    assert!(s.velocity.norm() < 1e-9, "{:?}", s.velocity);
    assert!(tilt_deg(&s) < 1e-9);
    assert!(s.rate.norm() < 1e-9);
}

#[test]
fn touchdown_at_point_seven_is_gentle_and_settles() {
    let m = model();
    let mut s = BodyState::<4>::default();
    // Arrives at the ground at 0.7 m/s.
    s.position.z = -m.ground.sag_m();
    s.velocity.z = 0.7;
    let mut peak = 0.0_f64;
    let mut prev_v = s.velocity.z;
    let mut left_ground_after = false;
    let mut touched = false;
    let end = fall(&m, s, 1.0, |b| {
        // Net acceleration seen by the body (specific force magnitude).
        peak = peak.max(((b.velocity.z - prev_v) / 1e-3 - GRAVITY).abs());
        prev_v = b.velocity.z;
        if m.ground.in_contact(b.position.z) {
            touched = true;
        } else if touched {
            left_ground_after = true;
        }
    });
    assert!(touched);
    assert!(peak < 8.0 * GRAVITY, "peak {} g", peak / GRAVITY);
    assert!(end.position.z.abs() < 1e-3, "z = {}", end.position.z);
    assert!(!left_ground_after);
}

#[test]
fn drop_from_thirty_centimetres_stays_shallow() {
    let m = model();
    let mut s = BodyState::<4>::default();
    s.position.z = -0.3;
    let mut deepest = 0.0_f64;
    fall(&m, s, 2.0, |b| deepest = deepest.max(b.position.z));
    assert!(deepest < 0.05, "deepest {deepest} m");
}

#[test]
fn tilted_vehicle_returns_to_level() {
    let m = model();
    let s = BodyState::<4> {
        attitude: Quat::from_rotation_vector(Vec3::new(10f64.to_radians(), 0.0, 0.0)),
        ..BodyState::default()
    };
    let end = fall(&m, s, 1.0, |_| {});
    assert!(tilt_deg(&end) < 0.1, "tilt {}", tilt_deg(&end));
    assert!(end.rate.norm() < 0.05);
}

#[test]
fn sliding_stops_within_half_a_metre() {
    let m = model();
    let mut s = BodyState::<4>::default();
    s.velocity.x = 2.0;
    let end = fall(&m, s, 3.0, |_| {});
    assert!(end.position.x < 0.5, "slid {} m", end.position.x);
    assert!(end.position.x > 0.1);
    assert!(end.velocity.norm() < 1e-2);
}

#[test]
fn thrust_above_weight_lifts_off_and_below_does_not() {
    let m = model();
    let hover = Real::sqrt(m.mass_kg * GRAVITY / (4.0 * m.rotors[0].max_thrust_n));
    let run = |factor: f64| {
        let n = hover * Real::sqrt(factor);
        let mut s = BodyState::<4> {
            rotor_speed: [n; 4],
            ..BodyState::default()
        };
        for _ in 0..1000 {
            s = m.rk4(&s, &[n; 4], 1e-3, Vec3::zero());
        }
        s.position.z
    };
    assert!(run(1.05) < -0.01, "no lift-off");
    // Below weight the ground still carries the rest: it stays in contact.
    let z = run(0.95);
    assert!(
        m.ground.in_contact(z) && z.abs() < m.ground.sag_m(),
        "z = {z}"
    );
}
