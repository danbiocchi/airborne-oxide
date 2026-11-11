// SPDX-License-Identifier: AGPL-3.0-or-later
//! Changing a parameter at run time through `Core::tick` (ADR-0004 §4).

use super::*;
use crate::config::ConfigError;
use crate::io::ParamSet;
use crate::params::{ParamError, ParamId, ParamValue};

fn set(id: ParamId, v: f32) -> ParamSet {
    ParamSet {
        id,
        value: ParamValue::F32(v),
    }
}

/// One tick of the external-source rig (`inputs`), with an optional change.
fn step(core: &mut Core<4>, k: u64, rc_sample: RcSample, ps: Option<ParamSet>) -> Outputs<4> {
    let t = Micros(k * 1000);
    let mut i = inputs(t, rc_sample);
    i.param_set = ps;
    core.tick(t, &i)
}

fn stick(roll: f32) -> RcSample {
    RcSample {
        roll,
        throttle: 0.5,
        ..rc(true, false)
    }
}

/// A core settled and armed by tick 1200, external source.
fn armed_core(c: VehicleConfig<4>) -> (Core<4>, u64) {
    let mut core = Core::new(c, F405_LEAN).expect("valid config");
    run(&mut core, 0, 1100, rc(false, false));
    let out = run(&mut core, 1100, 100, rc(true, false));
    assert!(out.status.armed);
    (core, 1200)
}

fn events(out: &Outputs<4>) -> Vec<EventKind> {
    out.events.iter().map(|e| e.kind).collect()
}

fn count_changed(out: &Outputs<4>) -> usize {
    events(out)
        .iter()
        .filter(|k| matches!(k, EventKind::ParamChanged(_)))
        .count()
}

#[test]
fn live_change_in_flight_takes_effect_with_one_event() {
    let (mut a, k0) = armed_core(cfg());
    let (mut b, _) = armed_core(cfg());
    let mut changed = 0;
    let mut diverged_after = None;
    for k in k0..k0 + 300 {
        let ps = (k == k0 + 100).then(|| set(ParamId::LIM_TILT, 0.2));
        let oa = step(&mut a, k, stick(1.0), ps);
        let ob = step(&mut b, k, stick(1.0), None);
        changed += count_changed(&oa);
        assert_eq!(count_changed(&ob), 0);
        if k < k0 + 100 {
            assert_eq!(oa.motors, ob.motors, "identical before the change");
        } else if oa.motors != ob.motors && diverged_after.is_none() {
            diverged_after = Some(k - (k0 + 100));
        }
    }
    assert_eq!(changed, 1);
    let d = diverged_after.expect("the tilt limit (Altitude mode, switch 1) changes the motors");
    assert!(d <= 40, "visible within a few runs (took {d} ticks)");
    assert_eq!(a.param(ParamId::LIM_TILT), Some(ParamValue::F32(0.2)));
    assert_eq!(a.config().limits.max_tilt_rad, 0.2);
    assert!(a.config() != b.config());
}

#[test]
fn disarmed_only_parameter_is_refused_while_armed_then_accepted() {
    let (mut core, k0) = armed_core(cfg());
    let before = *core.config();
    for id in [ParamId::EKF_GYR_NOISE, ParamId::RATE_GYRO_CUTOFF] {
        let out = step(&mut core, k0, stick(0.0), Some(set(id, 0.02)));
        let ev = events(&out);
        assert_eq!(
            ev,
            vec![EventKind::ParamRefused {
                id,
                why: ParamError::Armed
            }]
        );
    }
    let out = step(
        &mut core,
        k0 + 1,
        stick(0.0),
        Some(ParamSet {
            id: ParamId::EKF_SOURCE,
            value: ParamValue::I32(1),
        }),
    );
    assert_eq!(
        events(&out),
        vec![EventKind::ParamRefused {
            id: ParamId::EKF_SOURCE,
            why: ParamError::Armed
        }]
    );
    assert!(*core.config() == before, "configuration bit-identical");
    assert!(!core.params_unsaved());

    // Disarm (switch low for long enough), then it is accepted.
    let mut k = k0 + 2;
    for _ in 0..200 {
        step(&mut core, k, rc(false, false), None);
        k += 1;
    }
    let out = step(
        &mut core,
        k,
        rc(false, false),
        Some(set(ParamId::EKF_GYR_NOISE, 0.02)),
    );
    assert!(!out.status.armed);
    assert_eq!(
        events(&out),
        vec![EventKind::ParamChanged(ParamId::EKF_GYR_NOISE)]
    );
    assert_eq!(
        core.param(ParamId::EKF_GYR_NOISE),
        Some(ParamValue::F32(0.02))
    );
}

#[test]
fn estimator_change_restarts_the_estimate_and_blocks_arming_meanwhile() {
    let mut core = Core::new(ekf_cfg(), F405_LEAN).expect("valid config");
    let mut k = 0u64;
    while k < 3000 {
        core.tick(Micros(k * 1000), &ekf_inputs(k, rc(false, false)));
        k += 1;
    }
    assert!(core.estimate().is_some(), "aligned before the change");

    let mut i = ekf_inputs(k, rc(false, false));
    i.param_set = Some(set(ParamId::EKF_GYR_NOISE, 0.02));
    let out = core.tick(Micros(k * 1000), &i);
    k += 1;
    assert_eq!(
        events(&out),
        vec![EventKind::ParamChanged(ParamId::EKF_GYR_NOISE)]
    );
    assert!(core.estimate().is_none(), "the estimate went away");
    assert!(!core.estimator().status().aligned);

    // Arm switch rises while there is no estimate: refused with ESTIMATOR.
    let mut refused = false;
    let mut armed_in_between = false;
    let mut back_at = None;
    while k < 8000 {
        // Arm switch high only from 100 ticks after the change on.
        let arm = k >= 3100;
        let out = core.tick(Micros(k * 1000), &ekf_inputs(k, rc(arm, false)));
        for e in out.events.iter() {
            if let EventKind::ArmRefused(f) = e.kind
                && f.contains(PrearmFlags::ESTIMATOR)
            {
                refused = true;
            }
        }
        if core.estimate().is_none() {
            armed_in_between |= out.status.armed;
        } else if back_at.is_none() {
            back_at = Some(k);
        }
        k += 1;
    }
    assert!(refused, "arming was refused for the estimator");
    assert!(!armed_in_between);
    assert!(back_at.is_some(), "the estimate came back");
}

#[test]
fn gyro_cutoff_change_changes_the_rate_loop_response() {
    // Alternating gyro noise at 500 Hz: a 10 Hz filter removes it.
    let noisy = |k: u64, sample: &mut Inputs| {
        if let Some(imu) = sample.imu.as_mut() {
            let s = if k.is_multiple_of(2) { 1.0 } else { -1.0 };
            imu.value.gyro = Vec3::new(s, -s, 0.0);
        }
    };
    let drive = |mut core: Core<4>, ps: Option<ParamSet>| {
        let mut motors = Vec::new();
        let mut k = 0u64;
        // Disarmed, quiet gyro; the change (if any) arrives at tick 500.
        while k < 1200 {
            let t = Micros(k * 1000);
            let mut i = inputs(t, rc(k >= 1100, false));
            i.imu = Some(Stamped::ideal(t, ImuSample::default()));
            if k == 500 {
                i.param_set = ps;
            }
            core.tick(t, &i);
            k += 1;
        }
        while k < 1400 {
            let t = Micros(k * 1000);
            let mut i = inputs(t, rc(true, false));
            i.imu = Some(Stamped::ideal(t, ImuSample::default()));
            noisy(k, &mut i);
            let out = core.tick(t, &i);
            assert!(out.status.armed);
            motors.push(out.motors);
            k += 1;
        }
        motors
    };
    let mut slow = cfg();
    slow.gains.gyro_cutoff_hz = 10.0;
    let default = drive(Core::new(cfg(), F405_LEAN).expect("valid"), None);
    let by_param = drive(
        Core::new(cfg(), F405_LEAN).expect("valid"),
        Some(set(ParamId::RATE_GYRO_CUTOFF, 10.0)),
    );
    let built = drive(Core::new(slow, F405_LEAN).expect("valid"), None);
    assert_eq!(by_param, built, "same as a core built with the cut-off");
    assert!(by_param != default, "the filter changed");
    let spread = |m: &[[f32; 4]]| {
        m.iter()
            .map(|r| (r[0] - r[1]).abs())
            .fold(0.0_f32, f32::max)
    };
    assert!(
        spread(&by_param) < 0.5 * spread(&default),
        "slow {} vs default {}",
        spread(&by_param),
        spread(&default)
    );
}

#[test]
fn refusals_name_the_reason_and_change_nothing() {
    let mut core = Core::new(cfg(), F405_LEAN).expect("valid config");
    let before = *core.config();
    let cases: [(ParamSet, ParamError); 5] = [
        (set(ParamId::LIM_TILT, 5.0), ParamError::OutOfRange),
        (set(ParamId::LIM_TILT, f32::NAN), ParamError::NotFinite),
        (
            ParamSet {
                id: ParamId::LIM_TILT,
                value: ParamValue::I32(1),
            },
            ParamError::WrongType,
        ),
        (
            set(ParamId::LAND_SPEED, 0.4),
            ParamError::Invalid(ConfigError::BadFailsafe),
        ),
        (set(ParamId(u16::MAX), 1.0), ParamError::Unknown),
    ];
    for (n, (ps, why)) in cases.into_iter().enumerate() {
        let out = step(&mut core, n as u64, rc(false, false), Some(ps));
        assert_eq!(
            events(&out),
            vec![EventKind::ParamRefused { id: ps.id, why }],
            "case {n}"
        );
        assert!(*core.config() == before, "case {n}");
        assert!(!out.status.params_unsaved);
    }
}

#[test]
fn identical_inputs_with_identical_sets_give_identical_outputs() {
    let (mut a, k0) = armed_core(cfg());
    let (mut b, _) = armed_core(cfg());
    let sets = [
        (10, set(ParamId::LIM_TILT, 0.3)),
        (20, set(ParamId::RATE_ROLL_P, 31.0)),
        (30, set(ParamId::EKF_GYR_NOISE, 0.02)), // refused: armed
        (40, set(ParamId::LIM_TILT, 9.0)),       // refused: range
        (50, set(ParamId::FS_RC_HOLD, 2.0)),
    ];
    for k in k0..k0 + 120 {
        let ps = sets.iter().find(|(d, _)| k == k0 + d).map(|(_, p)| *p);
        let r = stick(if k % 7 < 3 { 0.6 } else { -0.4 });
        let oa = step(&mut a, k, r, ps);
        let ob = step(&mut b, k, r, ps);
        assert_eq!(oa, ob, "tick {k}");
    }
    assert!(a.config() == b.config());
}

#[test]
fn unsaved_flag_rises_survives_arming_and_clears_only_when_disarmed() {
    let mut core = Core::new(cfg(), F405_LEAN).expect("valid config");
    run(&mut core, 0, 1100, rc(false, false));
    assert!(
        !step(&mut core, 1100, rc(false, false), None)
            .status
            .params_unsaved
    );
    let out = step(
        &mut core,
        1101,
        rc(false, false),
        Some(set(ParamId::LIM_TILT, 0.5)),
    );
    assert!(out.status.params_unsaved);
    // Survives arming.
    let out = run(&mut core, 1102, 100, rc(true, false));
    assert!(out.status.armed && out.status.params_unsaved);
    // clear_unsaved does nothing while armed.
    core.clear_unsaved();
    let out = step(&mut core, 1202, rc(true, false), None);
    assert!(out.status.params_unsaved && core.params_unsaved());
    // A refused change does not touch it either way; disarm, then clear.
    let out = run(&mut core, 1203, 200, rc(false, false));
    assert!(!out.status.armed && out.status.params_unsaved);
    core.clear_unsaved();
    let out = step(&mut core, 1403, rc(false, false), None);
    assert!(!out.status.params_unsaved);
}
