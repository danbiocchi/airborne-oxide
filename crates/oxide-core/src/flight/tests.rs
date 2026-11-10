mod flow;

use super::*;
use crate::config::{EstimatorConfig, FailsafeParams, Gains, Limits, Rotor, Spin};
use crate::io::{BaroSample, GpsSample, MagSample};
use crate::schedule::F405_LEAN;

fn cfg() -> VehicleConfig<4> {
    let a = 0.08;
    let r = |x: f32, y: f32, spin| Rotor {
        x,
        y,
        spin,
        max_thrust_n: 8.0,
        torque_coeff_m: 0.016,
    };
    VehicleConfig {
        mass_kg: 0.65,
        inertia: Vec3::new(1.6e-3, 1.6e-3, 2.8e-3),
        rotors: [
            r(a, a, Spin::Ccw),
            r(-a, -a, Spin::Ccw),
            r(a, -a, Spin::Cw),
            r(-a, a, Spin::Cw),
        ],
        thrust_expo: 1.0,
        nominal_voltage_v: 22.2,
        gains: Gains::default(),
        limits: Limits::default(),
        estimator: EstimatorConfig::default(),
        failsafe: FailsafeParams::default(),
        battery_cells: 6,
        battery_resistance_ohm: 0.025,
    }
}

fn rc(arm: bool, kill: bool) -> RcSample {
    RcSample {
        arm_switch: arm,
        kill_switch: kill,
        mode_switch: 1,
        ..RcSample::default()
    }
}

/// A healthy 6S pack at rest.
fn battery(t: Micros) -> Option<Stamped<BatterySample>> {
    Some(Stamped::ideal(
        t,
        BatterySample {
            voltage_v: 23.0,
            current_a: 0.0,
        },
    ))
}

fn inputs(t: Micros, rc_sample: RcSample) -> Inputs {
    Inputs {
        state: Some(Stamped::ideal(t, StateSample::default())),
        rc: Some(Stamped::ideal(t, rc_sample)),
        battery: battery(t),
        ..Inputs::default()
    }
}

/// Runs `ticks` ticks from `start` with a fixed RC frame.
fn run(core: &mut Core<4>, start: u64, ticks: u64, rc_sample: RcSample) -> Outputs<4> {
    let mut out = Outputs::off(Micros::ZERO);
    for k in start..start + ticks {
        let t = Micros(k * 1000);
        out = core.tick(t, &inputs(t, rc_sample));
    }
    out
}

#[test]
fn outputs_are_valid_one_period_later() {
    let mut core = Core::new(cfg(), F405_LEAN).expect("valid config");
    let out = core.tick(Micros(0), &Inputs::default());
    assert_eq!(out.t_valid, Micros(1000));
    assert_eq!(out.motors, [0.0; 4]);
}

#[test]
fn arms_on_rising_edge_and_kill_latches() {
    let mut core = Core::new(cfg(), F405_LEAN).expect("valid config");
    // RC must be valid for a second before anything counts.
    let out = run(&mut core, 0, 1100, rc(false, false));
    assert!(!out.status.armed);
    let out = run(&mut core, 1100, 100, rc(true, false));
    assert!(out.status.armed);
    assert!(out.motors.iter().all(|m| *m >= 0.05));
    // Kill needs three consecutive supervisor runs (50 Hz → 60 ms).
    let out = run(&mut core, 1200, 20, rc(true, true));
    assert!(out.status.armed, "one frame of kill must not disarm");
    let out = run(&mut core, 1220, 100, rc(true, true));
    assert!(!out.status.armed && out.status.killed);
    assert_eq!(out.motors, [0.0; 4]);
    // Releasing kill alone does not re-arm or clear the latch.
    let out = run(&mut core, 1320, 100, rc(true, false));
    assert!(!out.status.armed && out.status.killed);
    // Arm switch off clears the latch; a new rising edge re-arms.
    let out = run(&mut core, 1420, 100, rc(false, false));
    assert!(!out.status.killed);
    let out = run(&mut core, 1520, 100, rc(true, false));
    assert!(out.status.armed);
}

#[test]
fn invalid_rc_frames_are_ignored() {
    let mut core = Core::new(cfg(), F405_LEAN).expect("valid config");
    run(&mut core, 0, 1100, rc(false, false));
    let bad = RcSample {
        failsafe: true,
        ..rc(true, false)
    };
    let out = run(&mut core, 1100, 100, bad);
    assert!(!out.status.armed, "a failsafe frame must not arm");
}

#[test]
fn counts_timing_faults() {
    let mut core = Core::new(cfg(), F405_LEAN).expect("valid config");
    core.tick(Micros(0), &Inputs::default());
    let out = core.tick(Micros(2500), &Inputs::default());
    assert_eq!(out.status.timing_faults, 1);
}

fn ekf_cfg() -> VehicleConfig<4> {
    let mut c = cfg();
    c.estimator.source = EstimatorSource::Ekf;
    c
}

/// Stationary level vehicle on the ground: IMU at 1 kHz, GPS at 5 Hz with
/// 150 ms latency, baro and mag at 50 Hz.
fn ekf_inputs(k: u64, rc_sample: RcSample) -> Inputs {
    let t = Micros(k * 1000);
    let mut i = Inputs {
        imu: Some(Stamped::ideal(
            t,
            ImuSample {
                gyro: Vec3::zero(),
                accel: Vec3::new(0.0, 0.0, -crate::GRAVITY),
            },
        )),
        rc: Some(Stamped::ideal(t, rc_sample)),
        battery: battery(t),
        ..Inputs::default()
    };
    if k.is_multiple_of(20) {
        i.baro = Some(Stamped::ideal(t, BaroSample { altitude_m: 40.0 }));
        i.mag = Some(Stamped::ideal(
            t,
            MagSample {
                field_ut: Vec3::new(25.0, 0.0, 43.3),
            },
        ));
    }
    // The fix measured 150 ms ago arrives now.
    if k >= 150 && (k - 150).is_multiple_of(200) {
        let ts = Micros((k - 150) * 1000);
        i.gps = Some(Stamped {
            t_sample: ts,
            t_arrival: t,
            value: GpsSample {
                h_acc_m: 0.6,
                v_acc_m: 1.0,
                s_acc_mps: 0.2,
                fix_3d: true,
                ..GpsSample::default()
            },
        });
    }
    i
}

#[test]
fn ekf_mode_aligns_estimates_and_arms_only_with_an_estimate() {
    let mut core = Core::new(ekf_cfg(), F405_LEAN).expect("valid config");
    let mut first_estimate = None;
    let mut armed_before = false;
    // Arm switch is high from the start (no rising edge to use later).
    for k in 0..3000u64 {
        let out = core.tick(Micros(k * 1000), &ekf_inputs(k, rc(true, false)));
        if core.estimate().is_none() {
            armed_before |= out.status.armed;
            assert!(out.status.no_estimate);
            assert!(!out.status.estimator.aligned || k > 500);
        } else if first_estimate.is_none() {
            first_estimate = Some(k);
        }
    }
    assert!(!armed_before, "must not arm without an estimate");
    let first = first_estimate.expect("estimate by 3 s");
    assert!(first > 900, "no estimate before alignment (k = {first})");
    let e = core.estimate().expect("estimate at 3 s");
    assert!(e.position.norm() < 0.1, "p = {:?}", e.position);
    assert!(
        e.attitude.to_rotation_vector().norm() < 1.0_f32.to_radians(),
        "attitude = {:?}",
        e.attitude
    );
    // The switch was already high when the estimate appeared: no arm.
    let out = core.tick(Micros(3_000_000), &ekf_inputs(3000, rc(true, false)));
    assert!(!out.status.armed);
    assert!(out.status.estimator.aligned && out.status.estimator.gps_aiding);
    // A fresh rising edge arms.
    let mut armed = false;
    for k in 3001..3200u64 {
        let out = core.tick(Micros(k * 1000), &ekf_inputs(k, rc(false, false)));
        armed |= out.status.armed;
    }
    assert!(!armed);
    for k in 3200..3400u64 {
        let out = core.tick(Micros(k * 1000), &ekf_inputs(k, rc(true, false)));
        armed |= out.status.armed;
    }
    assert!(armed, "arming works once the estimate exists");
}

#[test]
fn ekf_mode_ignores_the_external_state_input() {
    let mut core = Core::new(ekf_cfg(), F405_LEAN).expect("valid config");
    for k in 0..500u64 {
        let t = Micros(k * 1000);
        let mut i = ekf_inputs(k, rc(false, false));
        i.state = Some(Stamped::ideal(t, StateSample::default()));
        core.tick(t, &i);
    }
    assert!(core.estimate().is_none());
}

#[test]
fn external_mode_reports_aligned_when_estimate_is_visible() {
    let mut core = Core::new(cfg(), F405_LEAN).expect("valid config");
    let out = core.tick(Micros(0), &Inputs::default());
    assert!(!out.status.estimator.aligned);
    let out = run(&mut core, 1, 5, rc(false, false));
    assert!(out.status.estimator.aligned);
    assert_eq!(out.status.estimator.resets, 0);
}

#[test]
fn bad_estimator_params_fail_core_construction() {
    let mut c = cfg();
    c.estimator.ekf.gps_gate = f32::NAN;
    assert_eq!(
        Core::new(c, F405_LEAN).err(),
        Some(ConfigError::BadEstimator)
    );
}

#[test]
fn ekf_mode_imu_stall_drops_the_estimate() {
    let mut core = Core::new(ekf_cfg(), F405_LEAN).expect("valid config");
    for k in 0..3000u64 {
        core.tick(Micros(k * 1000), &ekf_inputs(k, rc(false, false)));
    }
    // Arm with a fresh rising edge.
    let mut out = Outputs::off(Micros::ZERO);
    for k in 3000..3300u64 {
        out = core.tick(Micros(k * 1000), &ekf_inputs(k, rc(true, false)));
    }
    assert!(out.status.armed && !out.status.no_estimate);
    // The IMU stops; everything else continues.
    let mut flagged = None;
    for k in 3300..3600u64 {
        let mut i = ekf_inputs(k, rc(true, false));
        i.imu = None;
        out = core.tick(Micros(k * 1000), &i);
        if out.status.no_estimate && flagged.is_none() {
            flagged = Some(k - 3300);
        }
    }
    let after = flagged.expect("no_estimate must be set");
    assert!(after <= 150, "took {after} ms");
    assert!(core.estimate().is_none());
}

#[test]
fn ekf_mode_arming_needs_gps_aiding() {
    let mut core = Core::new(ekf_cfg(), F405_LEAN).expect("valid config");
    let step = |core: &mut Core<4>, k: u64, arm: bool, gps: bool| {
        let mut i = ekf_inputs(k, rc(arm, false));
        if !gps {
            i.gps = None;
        }
        core.tick(Micros(k * 1000), &i)
    };
    for k in 0..3000u64 {
        step(&mut core, k, false, true);
    }
    let out = step(&mut core, 3000, false, true);
    assert!(out.status.estimator.gps_aiding);
    // GPS lost for longer than the timeout (5 s) plus a second.
    for k in 3001..9000u64 {
        step(&mut core, k, false, false);
    }
    let mut armed = false;
    for k in 9000..9150u64 {
        let out = step(&mut core, k, true, false);
        assert!(
            core.estimate().is_some(),
            "dead reckoning keeps the estimate"
        );
        assert!(!out.status.estimator.gps_aiding);
        armed |= out.status.armed;
    }
    assert!(!armed, "must not arm without GPS aiding");
    // GPS resumes; once fused, a new rising edge arms.
    for k in 9150..9800u64 {
        step(&mut core, k, false, true);
    }
    let out = step(&mut core, 9800, false, true);
    assert!(out.status.estimator.gps_aiding);
    let mut armed = false;
    for k in 9801..10000u64 {
        armed |= step(&mut core, k, true, true).status.armed;
    }
    assert!(armed, "arms again once GPS aids");
}

#[test]
fn a_future_stamped_sample_is_not_fresh() {
    let mut core = Core::new(cfg(), F405_LEAN).expect("valid config");
    run(&mut core, 0, 1100, rc(false, false));
    let mut armed = false;
    for k in 1100..1200u64 {
        let t = Micros(k * 1000);
        let ahead = t.add_us(10_000_000);
        let mut i = inputs(t, RcSample::default());
        i.rc = Some(Stamped::ideal(ahead, rc(true, false)));
        armed |= core.tick(t, &i).status.armed;
    }
    assert!(!armed, "a frame from the future must be ignored");
    // Real frames count again once RC has been valid for a second.
    run(&mut core, 1200, 1100, rc(false, false));
    let out = run(&mut core, 2300, 100, rc(true, false));
    assert!(out.status.armed, "normal frames afterwards still work");
}
