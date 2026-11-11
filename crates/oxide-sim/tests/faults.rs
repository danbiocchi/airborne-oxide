// SPDX-License-Identifier: AGPL-3.0-or-later
//! Fault injection and links (M1c, ADR-0003): what the core is fed.

use oxide_core::io::{Inputs, RcSample, TrajectorySetpoint};
use oxide_core::math::{Real, Vec3};
use oxide_core::schedule::F405_LEAN;
use oxide_core::time::Micros;
use oxide_sim::faults::{Fault, FaultKind, LinkConfig};
use oxide_sim::scenario::{COMMAND_PERIOD_US, Pilot};
use oxide_sim::sensors::SensorConfig;
use oxide_sim::vehicle::{BodyState, GRAVITY, VehicleModel};
use oxide_sim::{Airframe, SimError, SimOptions, Simulation};

/// 50 Hz RC whose roll stick changes with time, and 50 Hz setpoints.
struct TestPilot;

impl Pilot for TestPilot {
    fn rc(&mut self, t: Micros) -> Option<RcSample> {
        t.0.is_multiple_of(COMMAND_PERIOD_US).then(|| RcSample {
            roll: (t.as_secs_f64() * 0.05) as f32,
            throttle: 0.5,
            arm_switch: true,
            mode_switch: 1,
            ..RcSample::default()
        })
    }
    fn setpoint(&mut self, t: Micros) -> Option<TrajectorySetpoint> {
        t.0.is_multiple_of(COMMAND_PERIOD_US)
            .then(TrajectorySetpoint::default)
    }
}

fn fault(kind: FaultKind, start_s: f64, end_s: Option<f64>) -> Fault {
    Fault {
        kind,
        start_s,
        end_s,
    }
}

/// Runs `seconds` and returns `(t, inputs fed to the core)` for every tick.
fn run(options: SimOptions, seconds: u64) -> Vec<(Micros, Inputs)> {
    let a = Airframe::reference_quad().expect("airframe");
    let mut sim = Simulation::<_, 4>::with_options(&a, F405_LEAN, TestPilot, options).expect("sim");
    let mut out = Vec::new();
    while sim.time() < Micros::from_secs(seconds) {
        let r = sim.step();
        out.push((r.t, *sim.last_inputs()));
    }
    out
}

fn with_faults(faults: Vec<Fault>) -> SimOptions {
    SimOptions {
        seed: 3,
        sensors: SensorConfig::realistic(),
        faults,
        ..SimOptions::default()
    }
}

fn secs(t: Micros) -> f64 {
    t.as_secs_f64()
}

#[test]
fn sensor_loss_drops_only_that_sensor_inside_its_window() {
    let base = run(with_faults(vec![]), 6);
    type Probe = fn(&Inputs) -> bool;
    let cases: [(FaultKind, Probe); 4] = [
        (FaultKind::ImuLoss, |i| i.imu.is_some()),
        (FaultKind::GpsLoss, |i| i.gps.is_some()),
        (FaultKind::BaroLoss, |i| i.baro.is_some()),
        (FaultKind::MagLoss, |i| i.mag.is_some()),
    ];
    for (idx, (kind, present)) in cases.iter().enumerate() {
        let faulty = run(with_faults(vec![fault(*kind, 2.0, Some(4.0))]), 6);
        let mut dropped = 0;
        for ((t, b), (_, f)) in base.iter().zip(faulty.iter()) {
            let inside = (2.0..4.0).contains(&secs(*t));
            if inside {
                assert!(!present(f), "{kind:?} present at {t:?}");
                if present(b) {
                    dropped += 1;
                }
            } else {
                // Identical to the fault-free run: random sequences unshifted.
                assert_eq!(b, f, "{kind:?} differs outside window at {t:?}");
            }
            // The other sensors are untouched everywhere.
            for (j, (_, other)) in cases.iter().enumerate() {
                if j != idx {
                    assert_eq!(other(b), other(f));
                }
            }
        }
        assert!(dropped > 0, "{kind:?}: baseline had no samples to drop");
    }
}

#[test]
fn sensor_samples_after_a_loss_match_the_fault_free_run() {
    let base = run(with_faults(vec![]), 5);
    let faulty = run(
        with_faults(vec![fault(FaultKind::GpsLoss, 1.0, Some(3.0))]),
        5,
    );
    let after = |v: &[(Micros, Inputs)]| {
        v.iter()
            .filter(|(t, _)| secs(*t) >= 3.0)
            .map(|(_, i)| i.gps)
            .collect::<Vec<_>>()
    };
    assert_eq!(after(&base), after(&faulty));
}

#[test]
fn rc_loss_flags_frame_lost_then_failsafe_after_delay_and_holds_sticks() {
    let frames = run(
        SimOptions {
            faults: vec![fault(FaultKind::RcLoss, 3.0, None)],
            ..SimOptions::default()
        },
        6,
    );
    let rc: Vec<(f64, RcSample)> = frames
        .iter()
        .filter_map(|(t, i)| i.rc.map(|r| (secs(*t), r.value)))
        .collect();
    let held_from = rc.iter().rfind(|(t, _)| *t < 3.0).expect("pre-loss").1;
    assert!(held_from.is_valid());
    assert!(held_from.roll > 0.0);
    for (t, r) in &rc {
        if *t < 3.0 {
            assert!(r.is_valid(), "valid before loss at {t}");
        } else {
            assert!(r.frame_lost, "frame_lost immediately at {t}");
            assert_eq!(r.failsafe, *t >= 4.0, "failsafe timing at {t}");
            assert_eq!(r.roll, held_from.roll, "sticks held at {t}");
            assert_eq!(r.throttle, held_from.throttle);
            assert_eq!(r.arm_switch, held_from.arm_switch);
            assert_eq!(r.mode_switch, held_from.mode_switch);
        }
    }
    // Frames keep coming at 50 Hz through the loss.
    assert_eq!(rc.iter().filter(|(t, _)| *t >= 3.0).count(), 150);
}

#[test]
fn rc_loss_failsafe_delay_is_configurable_and_loss_can_end() {
    let frames = run(
        SimOptions {
            faults: vec![fault(FaultKind::RcLoss, 1.0, Some(2.0))],
            links: LinkConfig {
                rc_failsafe_delay_us: 200_000,
                ..LinkConfig::default()
            },
            ..SimOptions::default()
        },
        3,
    );
    for (t, i) in &frames {
        let Some(r) = i.rc else { continue };
        let t = secs(*t);
        assert_eq!(r.value.frame_lost, (1.0..2.0).contains(&t), "t={t}");
        assert_eq!(r.value.failsafe, (1.2..2.0).contains(&t), "t={t}");
        if t >= 2.0 {
            assert!(r.value.is_valid());
            assert_eq!(r.value.roll, (t * 0.05) as f32, "live sticks at {t}");
        }
    }
}

#[test]
fn rc_loss_with_no_prior_frame_holds_default_sticks_with_flags() {
    let frames = run(
        SimOptions {
            faults: vec![fault(FaultKind::RcLoss, 0.0, None)],
            ..SimOptions::default()
        },
        2,
    );
    let first = frames.iter().find_map(|(_, i)| i.rc).expect("a frame");
    assert_eq!(
        first.value,
        RcSample {
            frame_lost: true,
            ..RcSample::default()
        }
    );
    assert!(
        frames
            .iter()
            .filter_map(|(t, i)| i.rc.map(|r| (secs(*t), r.value)))
            .all(|(t, r)| r.failsafe == (t >= 1.0))
    );
}

#[test]
fn rc_silent_sends_nothing_and_then_resumes() {
    let frames = run(
        SimOptions {
            faults: vec![fault(FaultKind::RcSilent, 1.0, Some(2.0))],
            ..SimOptions::default()
        },
        3,
    );
    for (t, i) in &frames {
        let t = secs(*t);
        if (1.0..2.0).contains(&t) {
            assert!(i.rc.is_none(), "frame at {t}");
        } else if ((t * 1000.0).round() as u64).is_multiple_of(20) {
            assert!(i.rc.is_some_and(|r| r.value.is_valid()), "no frame at {t}");
        }
    }
}

fn beats(frames: &[(Micros, Inputs)]) -> Vec<(u64, u32)> {
    frames
        .iter()
        .filter_map(|(t, i)| i.brain_heartbeat.map(|h| (t.0, h.value.progress)))
        .collect()
}

#[test]
fn default_links_send_brain_heartbeats_only_every_200ms_with_counting_progress() {
    let frames = run(SimOptions::default(), 3);
    assert!(frames.iter().all(|(_, i)| i.gcs_heartbeat.is_none()));
    let b = beats(&frames);
    assert_eq!(b.len(), 15);
    for (n, (t, p)) in b.iter().enumerate() {
        assert_eq!(*t, n as u64 * 200_000);
        assert_eq!(*p, n as u32 + 1);
    }
    for (t, i) in &frames {
        if let Some(h) = i.brain_heartbeat {
            assert_eq!((h.t_sample, h.t_arrival), (*t, *t));
        }
    }
}

#[test]
fn gcs_heartbeats_every_second_and_stop_under_gcs_loss() {
    let frames = run(
        SimOptions {
            links: LinkConfig {
                gcs: true,
                ..LinkConfig::default()
            },
            faults: vec![fault(FaultKind::GcsLoss, 3.0, Some(6.0))],
            ..SimOptions::default()
        },
        8,
    );
    let g: Vec<u64> = frames
        .iter()
        .filter(|(_, i)| i.gcs_heartbeat.is_some())
        .map(|(t, _)| t.0 / 1_000_000)
        .collect();
    assert_eq!(g, vec![0, 1, 2, 6, 7]);
    // Brain heartbeats are unaffected.
    assert_eq!(beats(&frames).len(), 40);
}

#[test]
fn no_links_means_no_heartbeats() {
    let frames = run(
        SimOptions {
            links: LinkConfig {
                gcs: false,
                brain: false,
                ..LinkConfig::default()
            },
            ..SimOptions::default()
        },
        2,
    );
    assert!(
        frames
            .iter()
            .all(|(_, i)| i.gcs_heartbeat.is_none() && i.brain_heartbeat.is_none())
    );
}

#[test]
fn brain_crash_stops_heartbeats_and_setpoints_then_resumes() {
    let frames = run(
        SimOptions {
            faults: vec![fault(FaultKind::BrainCrash, 2.0, Some(4.0))],
            ..SimOptions::default()
        },
        6,
    );
    let mut sp_before = 0;
    let mut sp_after = 0;
    for (t, i) in &frames {
        let ts = secs(*t);
        if (2.0..4.0).contains(&ts) {
            assert!(i.brain_heartbeat.is_none(), "heartbeat at {ts}");
            assert!(i.setpoint.is_none(), "setpoint at {ts}");
        } else if i.setpoint.is_some() {
            if ts < 2.0 {
                sp_before += 1;
            } else {
                sp_after += 1;
            }
        }
    }
    assert_eq!((sp_before, sp_after), (100, 100));
    let b = beats(&frames);
    assert_eq!(b.len(), 20);
    // The counter resumes counting after the crash (10 beats before it).
    assert_eq!(
        b.iter().find(|(t, _)| *t == 4_000_000).map(|x| x.1),
        Some(11)
    );
}

#[test]
fn brain_hang_keeps_heartbeating_with_frozen_progress_and_no_setpoints() {
    let frames = run(
        SimOptions {
            faults: vec![fault(FaultKind::BrainHang, 2.0, Some(4.0))],
            ..SimOptions::default()
        },
        6,
    );
    let b = beats(&frames);
    assert_eq!(b.len(), 30, "heartbeats continue");
    for (t, p) in &b {
        if (2_000_000..4_000_000).contains(t) {
            assert_eq!(*p, 10, "frozen at {t}");
        }
    }
    assert_eq!(
        b.iter().find(|(t, _)| *t == 4_000_000).map(|x| x.1),
        Some(11)
    );
    for (t, i) in &frames {
        if (2.0..4.0).contains(&secs(*t)) {
            assert!(i.setpoint.is_none());
        }
    }
}

#[test]
fn motor_loss_sets_rotor_scale_inside_its_window_only() {
    let a = Airframe::reference_quad().expect("airframe");
    let options = SimOptions {
        faults: vec![
            fault(
                FaultKind::MotorLoss {
                    rotor: 1,
                    scale: 0.0,
                },
                0.5,
                Some(1.0),
            ),
            fault(
                FaultKind::MotorLoss {
                    rotor: 3,
                    scale: 0.5,
                },
                0.75,
                None,
            ),
        ],
        ..SimOptions::default()
    };
    let mut sim = Simulation::<_, 4>::with_options(&a, F405_LEAN, TestPilot, options).expect("sim");
    while sim.time() < Micros::from_secs(2) {
        let r = sim.step();
        let t = secs(r.t);
        let want = [
            1.0,
            if (0.5..1.0).contains(&t) { 0.0 } else { 1.0 },
            1.0,
            if t >= 0.75 { 0.5 } else { 1.0 },
        ];
        assert_eq!(sim.model().rotor_scale, want, "t={t}");
    }
}

#[test]
fn healthy_model_has_unit_rotor_scale() {
    let a = Airframe::reference_quad().expect("airframe");
    let m = VehicleModel::<4>::from_airframe(&a).expect("model");
    assert_eq!(m.rotor_scale, [1.0; 4]);
}

#[test]
fn dead_rotor_removes_its_thrust_oracle() {
    let a = Airframe::reference_quad().expect("airframe");
    let mut m = VehicleModel::<4>::from_airframe(&a).expect("model");
    m.gravity = true;
    // Normalised speed at which four rotors exactly hover the vehicle.
    let n = Real::sqrt(m.mass_kg * GRAVITY / (4.0 * m.rotors[0].max_thrust_n));
    let state = BodyState::<4> {
        position: Vec3::new(0.0, 0.0, -10.0),
        rotor_speed: [n; 4],
        ..BodyState::default()
    };
    let calm = Vec3::zero();

    // Healthy: balanced, no net acceleration, no torque.
    let l = m.loads(&state, calm);
    assert!(l.torque_body.norm() < 1e-9, "{:?}", l.torque_body);
    assert!((l.force_body.z + m.mass_kg * GRAVITY).abs() < 1e-9);

    // One rotor dead: thrust is 3/4 of weight, so the vehicle accelerates
    // downward (NED +z) at g/4 at the first instant.
    m.rotor_scale[0] = 0.0;
    let l = m.loads(&state, calm);
    assert!((l.force_body.z + 0.75 * m.mass_kg * GRAVITY).abs() < 1e-9);
    let accel_z = GRAVITY + l.force_body.z / m.mass_kg;
    assert!(
        (accel_z - GRAVITY / 4.0).abs() < 1e-9,
        "vertical acceleration {accel_z}"
    );
    // The missing rotor unbalances the moments: the body starts rotating.
    assert!(
        l.torque_body.norm() > 0.01 * m.mass_kg,
        "{:?}",
        l.torque_body
    );
    let dt = 1e-3;
    let cmd = [n; 4];
    let next = m.rk4(&state, &cmd, dt, calm);
    assert!(next.rate.norm() > 1e-6, "rate {:?}", next.rate);
    assert!(next.velocity.z > 0.0 && (next.velocity.z - GRAVITY / 4.0 * dt).abs() < 1e-5);

    // Half scale gives half the lost thrust; shaft power scales with it.
    let healthy_power = {
        m.rotor_scale = [1.0; 4];
        m.loads(&state, calm).shaft_power_w
    };
    m.rotor_scale[0] = 0.0;
    let power = m.loads(&state, calm).shaft_power_w;
    assert!((power - 0.75 * healthy_power).abs() < 1e-9 * healthy_power.max(1.0));
}

#[test]
fn no_faults_is_bit_identical_to_default_run() {
    let a = Airframe::reference_quad().expect("airframe");
    let hash = |o: SimOptions| {
        let mut sim = Simulation::<_, 4>::with_options(&a, F405_LEAN, TestPilot, o).expect("sim");
        sim.run_until(Micros::from_secs(3), |_| {});
        sim.trace_hash()
    };
    let plain = hash(SimOptions::default());
    let explicit = hash(SimOptions {
        faults: Vec::new(),
        links: LinkConfig::default(),
        ..SimOptions::default()
    });
    assert_eq!(plain, explicit);
}

/// The error of building a simulation with these options.
fn setup_error(options: SimOptions) -> Option<SimError> {
    let a = Airframe::reference_quad().expect("airframe");
    Simulation::<_, 4>::with_options(&a, F405_LEAN, TestPilot, options).err()
}

#[test]
fn a_motor_loss_on_a_rotor_the_airframe_lacks_is_refused() {
    let bad = FaultKind::MotorLoss {
        rotor: 4,
        scale: 0.0,
    };
    let e = setup_error(with_faults(vec![fault(bad, 5.0, None)])).expect("refused");
    assert!(matches!(e, SimError::Fault { index: 0, .. }), "{e:?}");
    assert!(e.to_string().contains("rotor 4"), "{e}");
    // The last rotor that exists is fine.
    let ok = FaultKind::MotorLoss {
        rotor: 3,
        scale: 0.0,
    };
    assert_eq!(setup_error(with_faults(vec![fault(ok, 5.0, None)])), None);
}

#[test]
fn a_motor_loss_scale_that_is_not_a_fraction_is_refused() {
    for scale in [-0.1, 1.1, f64::NAN, f64::INFINITY] {
        let bad = FaultKind::MotorLoss { rotor: 0, scale };
        let e = setup_error(with_faults(vec![fault(bad, 5.0, None)])).expect("refused");
        assert!(
            matches!(e, SimError::Fault { index: 0, .. }),
            "{scale}: {e:?}"
        );
    }
    for scale in [0.0, 0.5, 1.0] {
        let ok = FaultKind::MotorLoss { rotor: 0, scale };
        assert_eq!(setup_error(with_faults(vec![fault(ok, 5.0, None)])), None);
    }
}

#[test]
fn a_fault_window_that_ends_before_it_starts_is_refused() {
    for (start, end) in [(5.0, 4.0), (5.0, 5.0), (5.0, f64::NAN)] {
        let e = setup_error(with_faults(vec![
            fault(FaultKind::GpsLoss, 1.0, None),
            fault(FaultKind::BaroLoss, start, Some(end)),
        ]))
        .expect("refused");
        assert!(
            matches!(e, SimError::Fault { index: 1, .. }),
            "{start}..{end}: {e:?}"
        );
    }
    // A non-finite start is refused too (it would never start).
    let e =
        setup_error(with_faults(vec![fault(FaultKind::GpsLoss, f64::NAN, None)])).expect("refused");
    assert!(matches!(e, SimError::Fault { index: 0, .. }), "{e:?}");
    // A window that is not empty, and a permanent fault, are fine.
    assert_eq!(
        setup_error(with_faults(vec![
            fault(FaultKind::GpsLoss, 5.0, Some(5.001)),
            fault(FaultKind::BaroLoss, 5.0, None),
        ])),
        None
    );
}

#[test]
fn link_faults_need_their_link() {
    let no_gcs = with_faults(vec![fault(FaultKind::GcsLoss, 5.0, None)]);
    let e = setup_error(no_gcs.clone()).expect("refused");
    assert!(e.to_string().contains("ground station"), "{e}");
    let mut with_gcs = no_gcs;
    with_gcs.links.gcs = true;
    assert_eq!(setup_error(with_gcs), None);

    for kind in [FaultKind::BrainCrash, FaultKind::BrainHang] {
        let mut o = with_faults(vec![fault(kind, 5.0, None)]);
        assert_eq!(setup_error(o.clone()), None, "{kind:?} with a brain");
        o.links.brain = false;
        let e = setup_error(o).expect("refused");
        assert!(e.to_string().contains("brain"), "{kind:?}: {e}");
    }
}
