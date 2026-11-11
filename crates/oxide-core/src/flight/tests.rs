mod flow;

use super::*;
use crate::config::{EstimatorConfig, FailsafeParams, Gains, Limits, Rotor, Spin};
use crate::io::{BaroSample, DisarmReason, EventKind, GpsSample, MagSample};
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

/// One failsafe or battery field changed, and whether the result must
/// validate.
fn with(change: impl FnOnce(&mut VehicleConfig<4>)) -> Result<(), ConfigError> {
    let mut c = cfg();
    change(&mut c);
    c.validate()
}

#[test]
fn the_default_configuration_validates() {
    assert_eq!(cfg().validate(), Ok(()));
    assert_eq!(FailsafeParams::default().validate(), Ok(()));
}

#[test]
fn every_float_failsafe_field_refuses_nan_and_infinity() {
    type Set = fn(&mut FailsafeParams, f32);
    let fields: [(&str, Set); 23] = [
        ("rc_valid_min_s", |p, v| p.rc_valid_min_s = v),
        ("rc_loss_hold_s", |p, v| p.rc_loss_hold_s = v),
        ("no_link_rtl_s", |p, v| p.no_link_rtl_s = v),
        ("brain_loss_rtl_s", |p, v| p.brain_loss_rtl_s = v),
        ("gps_loss_land_s", |p, v| p.gps_loss_land_s = v),
        ("clear_hysteresis_s", |p, v| p.clear_hysteresis_s = v),
        ("battery_low_v", |p, v| p.battery_low_v = v),
        ("battery_critical_v", |p, v| p.battery_critical_v = v),
        ("battery_debounce_s", |p, v| p.battery_debounce_s = v),
        ("battery_filter_tau_s", |p, v| p.battery_filter_tau_s = v),
        ("fence_radius_m", |p, v| p.fence_radius_m = v),
        ("fence_altitude_m", |p, v| p.fence_altitude_m = v),
        ("loc_error_rad", |p, v| p.loc_error_rad = v),
        ("loc_error_s", |p, v| p.loc_error_s = v),
        ("loc_tilt_rad", |p, v| p.loc_tilt_rad = v),
        ("loc_tilt_s", |p, v| p.loc_tilt_s = v),
        ("rtl_altitude_m", |p, v| p.rtl_altitude_m = v),
        ("rtl_speed_mps", |p, v| p.rtl_speed_mps = v),
        ("land_speed_mps", |p, v| p.land_speed_mps = v),
        ("land_blind_thrust", |p, v| p.land_blind_thrust = v),
        ("touchdown_thrust", |p, v| p.touchdown_thrust = v),
        ("touchdown_speed_mps", |p, v| p.touchdown_speed_mps = v),
        ("touchdown_s", |p, v| p.touchdown_s = v),
    ];
    for (name, set) in fields {
        for bad in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
            let r = with(|c| set(&mut c.failsafe, bad));
            assert_eq!(r, Err(ConfigError::BadFailsafe), "{name} = {bad}");
        }
    }
}

#[test]
fn failsafe_values_outside_their_bounds_are_refused_and_on_the_bound_pass() {
    type Set = fn(&mut FailsafeParams, f32);
    let pi = core::f32::consts::PI;
    // (name, setter, [refused], [accepted])
    let table: [(&str, Set, &[f32], &[f32]); 22] = [
        (
            "rc_valid_min_s",
            |p, v| p.rc_valid_min_s = v,
            &[-0.001, 3600.1],
            &[0.0, 3600.0],
        ),
        (
            "rc_loss_hold_s",
            |p, v| p.rc_loss_hold_s = v,
            &[-0.001, 3600.1],
            &[0.0, 3600.0],
        ),
        (
            "no_link_rtl_s",
            |p, v| p.no_link_rtl_s = v,
            &[-0.001, 3600.1],
            &[0.0, 3600.0],
        ),
        (
            "brain_loss_rtl_s",
            |p, v| p.brain_loss_rtl_s = v,
            &[-0.001, 3600.1],
            &[0.0, 3600.0],
        ),
        (
            "gps_loss_land_s",
            |p, v| p.gps_loss_land_s = v,
            &[-0.001, 3600.1],
            &[0.0, 3600.0],
        ),
        (
            "clear_hysteresis_s",
            |p, v| p.clear_hysteresis_s = v,
            &[-0.001, 3600.1],
            &[0.0, 3600.0],
        ),
        (
            "battery_debounce_s",
            |p, v| p.battery_debounce_s = v,
            &[-0.001, 3600.1],
            &[0.0, 3600.0],
        ),
        (
            "battery_filter_tau_s",
            |p, v| p.battery_filter_tau_s = v,
            &[0.0, -1.0, 3600.1],
            &[1e-6, 3600.0],
        ),
        (
            "loc_error_s",
            |p, v| p.loc_error_s = v,
            &[-0.001, 3600.1],
            &[0.0, 3600.0],
        ),
        (
            "loc_tilt_s",
            |p, v| p.loc_tilt_s = v,
            &[-0.001, 3600.1],
            &[0.0, 3600.0],
        ),
        (
            "touchdown_s",
            |p, v| p.touchdown_s = v,
            &[0.0, -1.0, 3600.1],
            &[1e-6, 3600.0],
        ),
        (
            "battery_low_v",
            |p, v| p.battery_low_v = v,
            &[2.99, 4.21],
            &[3.3, 3.5, 4.2],
        ),
        (
            "battery_critical_v",
            |p, v| p.battery_critical_v = v,
            &[2.99, 3.51],
            &[3.0, 3.5],
        ),
        (
            "fence_radius_m",
            |p, v| p.fence_radius_m = v,
            &[-0.001],
            &[0.0, 1e6],
        ),
        (
            "fence_altitude_m",
            |p, v| p.fence_altitude_m = v,
            &[-0.001],
            &[0.0, 1e6],
        ),
        (
            "loc_error_rad",
            |p, v| p.loc_error_rad = v,
            &[0.0, -0.1, pi + 0.001],
            &[1e-6, pi],
        ),
        (
            "loc_tilt_rad",
            |p, v| p.loc_tilt_rad = v,
            &[0.0, -0.1, pi + 0.001],
            &[1e-6, pi],
        ),
        (
            "rtl_altitude_m",
            |p, v| p.rtl_altitude_m = v,
            &[-0.001],
            &[0.0, 500.0],
        ),
        (
            "rtl_speed_mps",
            |p, v| p.rtl_speed_mps = v,
            &[0.0, 0.0999, -1.0],
            &[0.1, 10.0],
        ),
        (
            "land_blind_thrust",
            |p, v| p.land_blind_thrust = v,
            &[0.0, -0.1, 1.001],
            &[1e-6, 1.0],
        ),
        (
            "touchdown_thrust",
            |p, v| p.touchdown_thrust = v,
            &[0.0, -0.1, 1.0, 1.5],
            &[1e-6, 0.999],
        ),
        (
            "touchdown_speed_mps",
            |p, v| p.touchdown_speed_mps = v,
            &[0.0, -0.1],
            &[1e-3, 0.35],
        ),
    ];
    for (name, set, refused, accepted) in table {
        for &v in refused {
            let r = with(|c| set(&mut c.failsafe, v));
            assert_eq!(
                r,
                Err(ConfigError::BadFailsafe),
                "{name} = {v} must be refused"
            );
        }
        for &v in accepted {
            let r = with(|c| set(&mut c.failsafe, v));
            assert_eq!(r, Ok(()), "{name} = {v} must be accepted");
        }
    }
}

#[test]
fn failsafe_timeouts_below_50_ms_are_refused() {
    type Set = fn(&mut FailsafeParams, u32);
    let fields: [(&str, Set); 4] = [
        ("rc_timeout_us", |p, v| p.rc_timeout_us = v),
        ("gcs_timeout_us", |p, v| p.gcs_timeout_us = v),
        ("brain_timeout_us", |p, v| p.brain_timeout_us = v),
        ("setpoint_timeout_us", |p, v| p.setpoint_timeout_us = v),
    ];
    for (name, set) in fields {
        for bad in [0, 1, 49_999] {
            let r = with(|c| set(&mut c.failsafe, bad));
            assert_eq!(r, Err(ConfigError::BadFailsafe), "{name} = {bad}");
        }
        assert_eq!(with(|c| set(&mut c.failsafe, 50_000)), Ok(()), "{name}");
    }
}

#[test]
fn critical_battery_threshold_may_not_exceed_the_low_one() {
    let r = with(|c| {
        c.failsafe.battery_low_v = 3.4;
        c.failsafe.battery_critical_v = 3.41;
    });
    assert_eq!(r, Err(ConfigError::BadFailsafe));
    let r = with(|c| {
        c.failsafe.battery_low_v = 3.4;
        c.failsafe.battery_critical_v = 3.4;
    });
    assert_eq!(r, Ok(()));
}

#[test]
fn land_speed_must_reach_the_contact_descent() {
    let contact = FailsafeParams::default().contact_descent_mps();
    assert_eq!(with(|c| c.failsafe.land_speed_mps = contact), Ok(()));
    assert_eq!(
        with(|c| c.failsafe.land_speed_mps = contact * 0.99),
        Err(ConfigError::BadFailsafe)
    );
}

#[test]
fn the_battery_description_is_validated() {
    assert_eq!(with(|c| c.battery_cells = 0), Err(ConfigError::BadFailsafe));
    assert_eq!(with(|c| c.battery_cells = 1), Ok(()));
    for bad in [f32::NAN, f32::INFINITY, -0.001] {
        assert_eq!(
            with(|c| c.battery_resistance_ohm = bad),
            Err(ConfigError::BadFailsafe),
            "resistance {bad}"
        );
    }
    assert_eq!(with(|c| c.battery_resistance_ohm = 0.0), Ok(()));
}

#[test]
fn core_new_refuses_a_bad_failsafe_configuration() {
    let mut c = cfg();
    c.failsafe.rtl_speed_mps = 0.0;
    assert_eq!(
        Core::new(c, F405_LEAN).err(),
        Some(ConfigError::BadFailsafe)
    );
    let mut c = cfg();
    c.battery_cells = 0;
    assert_eq!(
        Core::new(c, F405_LEAN).err(),
        Some(ConfigError::BadFailsafe)
    );
}

// ---- switches are confirmed on distinct frames (ADR-0003, DESIGN §4.9) ----

/// Status is published one supervisor run after the run that decided it, so
/// the tests that look at it add a trailing gap (`None`) after their frames.
///
/// Sends one RC frame per supervisor run (every 20th tick), each frame
/// arriving on the run's own tick, for the samples given. `None` is a gap:
/// no frame arrives and the stored one stays. Returns the next tick (ms)
/// and every event seen.
fn frames(
    core: &mut Core<4>,
    start_ms: u64,
    samples: &[Option<RcSample>],
) -> (u64, Vec<EventKind>, Outputs<4>) {
    let mut events = Vec::new();
    let mut out = Outputs::off(Micros::ZERO);
    let mut k = start_ms;
    for sample in samples {
        for i in 0..20 {
            let t = Micros(k * 1000);
            let mut inp = inputs(t, RcSample::default());
            inp.rc = if i == 0 {
                sample.map(|s| Stamped::ideal(t, s))
            } else {
                None
            };
            out = core.tick(t, &inp);
            events.extend(out.events.iter().map(|e| e.kind));
            k += 1;
        }
    }
    (k, events, out)
}

/// A core armed after RC settled; returns the next tick.
fn armed_core() -> (Core<4>, u64) {
    let mut core = Core::new(cfg(), F405_LEAN).expect("valid config");
    run(&mut core, 0, 1100, rc(false, false));
    let out = run(&mut core, 1100, 100, rc(true, false));
    assert!(out.status.armed);
    (core, 1200)
}

fn kill_frame() -> Option<RcSample> {
    Some(rc(true, true))
}

fn ok_frame() -> Option<RcSample> {
    Some(rc(true, false))
}

#[test]
fn one_kill_frame_then_a_gap_does_not_disarm() {
    let (mut core, k) = armed_core();
    let (k, ev, out) = frames(&mut core, k, &[kill_frame(), None]);
    assert!(out.status.armed);
    // 100 ms with no frames: the stored kill frame is still fresh.
    let (_, ev2, out) = frames(&mut core, k, &[None; 5]);
    assert!(out.status.armed && !out.status.killed);
    assert!(
        !ev.iter()
            .chain(&ev2)
            .any(|e| matches!(e, EventKind::Disarmed(_)))
    );
}

#[test]
fn three_distinct_kill_frames_disarm_with_kill() {
    let (mut core, k) = armed_core();
    let (k, _, out) = frames(&mut core, k, &[kill_frame(), kill_frame(), None]);
    assert!(out.status.armed);
    let (_, ev, out) = frames(&mut core, k, &[kill_frame(), None]);
    assert!(!out.status.armed && out.status.killed);
    assert!(ev.contains(&EventKind::Disarmed(DisarmReason::Kill)));
}

#[test]
fn a_valid_non_kill_frame_resets_the_kill_count() {
    let (mut core, k) = armed_core();
    let (_, ev, out) = frames(
        &mut core,
        k,
        &[
            kill_frame(),
            kill_frame(),
            ok_frame(),
            kill_frame(),
            kill_frame(),
            None,
        ],
    );
    assert!(out.status.armed && !out.status.killed);
    assert!(!ev.iter().any(|e| matches!(e, EventKind::Disarmed(_))));
}

#[test]
fn a_lost_frame_between_kill_frames_neither_counts_nor_resets() {
    let (mut core, k) = armed_core();
    let lost = Some(RcSample {
        frame_lost: true,
        ..rc(true, true)
    });
    let (_, ev, out) = frames(
        &mut core,
        k,
        &[kill_frame(), kill_frame(), lost, kill_frame(), None],
    );
    assert!(!out.status.armed && out.status.killed);
    assert!(ev.contains(&EventKind::Disarmed(DisarmReason::Kill)));
}

#[test]
fn a_kill_count_does_not_survive_an_rc_timeout() {
    let (mut core, k) = armed_core();
    let (k, _, _) = frames(&mut core, k, &[kill_frame(), kill_frame()]);
    // Silence longer than rc_timeout_us (500 ms), then one kill frame.
    let (k, mut ev, _) = frames(&mut core, k, &[None; 30]);
    let (_, ev2, out) = frames(&mut core, k, &[kill_frame()]);
    ev.extend(ev2);
    assert!(!out.status.killed);
    assert!(!ev.contains(&EventKind::Disarmed(DisarmReason::Kill)));
}

#[test]
fn one_arm_low_frame_in_flight_does_not_disarm() {
    let (mut core, k) = armed_core();
    let low = Some(rc(false, false));
    let (_, ev, out) = frames(
        &mut core,
        k,
        &[low, ok_frame(), ok_frame(), ok_frame(), None],
    );
    assert!(out.status.armed);
    assert!(!ev.iter().any(|e| matches!(e, EventKind::Disarmed(_))));
}

#[test]
fn three_arm_low_frames_disarm_with_switch() {
    let (mut core, k) = armed_core();
    let low = Some(rc(false, false));
    let (k, _, out) = frames(&mut core, k, &[low, low, None]);
    assert!(out.status.armed);
    let (_, ev, out) = frames(&mut core, k, &[low, None]);
    assert!(!out.status.armed);
    assert!(ev.contains(&EventKind::Disarmed(DisarmReason::Switch)));
}

#[test]
fn one_arm_high_frame_on_the_bench_does_not_arm() {
    let mut core = Core::new(cfg(), F405_LEAN).expect("valid config");
    run(&mut core, 0, 1100, rc(false, false));
    let low = Some(rc(false, false));
    let (_, ev, out) = frames(&mut core, 1100, &[ok_frame(), low, low, low, low, None]);
    assert!(!out.status.armed);
    assert!(
        !ev.iter()
            .any(|e| matches!(e, EventKind::Armed | EventKind::ArmRefused(_))),
        "events: {ev:?}"
    );
}

#[test]
fn three_arm_high_frames_arm_on_the_confirmed_edge() {
    let mut core = Core::new(cfg(), F405_LEAN).expect("valid config");
    run(&mut core, 0, 1100, rc(false, false));
    let (k, _, out) = frames(&mut core, 1100, &[ok_frame(), ok_frame(), None]);
    assert!(!out.status.armed);
    let (_, ev, out) = frames(&mut core, k, &[ok_frame(), None]);
    assert!(out.status.armed);
    assert!(ev.contains(&EventKind::Armed));
}
