// SPDX-License-Identifier: AGPL-3.0-or-later
//! Changing a parameter at run time through `Core::tick` (ADR-0004 §4).

use super::*;
use crate::config::ConfigError;
use crate::io::ParamSet;
use crate::params::{LoadReport, ParamError, ParamId, ParamValue};

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
    let mut k = k0;
    for id in [ParamId::EKF_GYR_NOISE, ParamId::RATE_GYRO_CUTOFF] {
        let out = step(&mut core, k, stick(0.0), Some(set(id, 0.02)));
        k += 1;
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
        k,
        stick(0.0),
        Some(ParamSet {
            id: ParamId::EKF_SOURCE,
            value: ParamValue::I32(1),
        }),
    );
    k += 1;
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
    assert_eq!(out.status.timing_faults, 0, "no tick number was used twice");
}

/// Steps with `rc_sample` until `want` is among the events; returns the tick
/// it happened on.
fn run_until(
    core: &mut Core<4>,
    from: u64,
    rc_sample: RcSample,
    want: impl Fn(&EventKind) -> bool,
) -> u64 {
    for k in from..from + 2000 {
        let out = step(core, k, rc_sample, None);
        if events(&out).iter().any(&want) {
            return k;
        }
    }
    panic!("the event never came");
}

fn refused_armed(id: ParamId) -> Vec<EventKind> {
    vec![EventKind::ParamRefused {
        id,
        why: ParamError::Armed,
    }]
}

#[test]
fn a_disarmed_only_set_on_the_tick_after_the_arm_decision_is_refused() {
    let mut core = Core::new(cfg(), F405_LEAN).expect("valid config");
    run(&mut core, 0, 1100, rc(false, false));
    let k = run_until(&mut core, 1100, rc(true, false), |e| {
        matches!(e, EventKind::Armed)
    });
    // The decision is made but not yet published: only `pending` says armed.
    assert!(core.supervision.pending.armed);
    assert!(!core.supervision.visible.armed);
    let id = ParamId::EKF_GYR_NOISE;
    let out = step(&mut core, k + 1, rc(true, false), Some(set(id, 0.02)));
    assert_eq!(events(&out), refused_armed(id));
    assert_eq!(core.param(id), Some(ParamValue::F32(0.015)));
    assert_eq!(out.status.timing_faults, 0);
}

#[test]
fn a_disarmed_only_set_on_the_tick_after_the_disarm_decision_is_refused() {
    let (mut core, k0) = armed_core(cfg());
    let k = run_until(&mut core, k0, rc(false, false), |e| {
        matches!(e, EventKind::Disarmed(_))
    });
    // The decision is made but not yet published: only `visible` says armed.
    assert!(!core.supervision.pending.armed);
    assert!(core.supervision.visible.armed);
    let id = ParamId::EKF_GYR_NOISE;
    let out = step(&mut core, k + 1, rc(false, false), Some(set(id, 0.02)));
    assert_eq!(events(&out), refused_armed(id));
    assert_eq!(core.param(id), Some(ParamValue::F32(0.015)));
    assert_eq!(out.status.timing_faults, 0);
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

// ---- a bad load, and its acknowledgement (ADR-0004 §6) ----

fn unclean_reports() -> [(&'static str, LoadReport); 4] {
    let r = LoadReport::default;
    [
        (
            "unknown",
            LoadReport {
                unknown: 1,
                applied: 3,
                ..r()
            },
        ),
        ("dropped", LoadReport { dropped: 2, ..r() }),
        (
            "invalid",
            LoadReport {
                invalid: true,
                ..r()
            },
        ),
        (
            "unreadable",
            LoadReport {
                unreadable: true,
                ..r()
            },
        ),
    ]
}

/// One tick of the external-source rig with an acknowledgement.
fn step_ack(core: &mut Core<4>, k: u64, rc_sample: RcSample, ack: bool) -> Outputs<4> {
    let t = Micros(k * 1000);
    let mut i = inputs(t, rc_sample);
    i.param_ack = ack;
    core.tick(t, &i)
}

/// Runs `n` ticks from `k`, collecting every event; returns the next tick.
fn run_events(
    core: &mut Core<4>,
    k: u64,
    n: u64,
    rc_sample: RcSample,
    all: &mut Vec<(u64, EventKind)>,
) -> (u64, Outputs<4>) {
    let mut out = Outputs::off(Micros::ZERO);
    for j in k..k + n {
        out = step_ack(core, j, rc_sample, false);
        all.extend(out.events.iter().map(|e| (j, e.kind)));
    }
    (k + n, out)
}

#[test]
fn a_clean_report_arms_and_an_unclean_one_leaves_only_config_between_it_and_arming() {
    for (what, report) in unclean_reports() {
        let mut core = Core::new(cfg(), F405_LEAN).expect("valid config");
        core.report_param_load(report);
        assert!(core.params_fault(), "{what}");
        let mut all = Vec::new();
        let (k, _) = run_events(&mut core, 0, 1100, rc(false, false), &mut all);
        let (_, out) = run_events(&mut core, k, 100, rc(true, false), &mut all);
        assert!(!out.status.armed, "{what}: armed with a parameter fault");
        assert_eq!(out.status.prearm, PrearmFlags::CONFIG, "{what}");
        assert!(!core.is_armed());
        // The refusal names exactly that flag.
        let refusals: Vec<PrearmFlags> = all
            .iter()
            .filter_map(|(_, e)| match e {
                EventKind::ArmRefused(f) => Some(*f),
                _ => None,
            })
            .collect();
        assert_eq!(refusals, vec![PrearmFlags::CONFIG], "{what}");
        assert_eq!(out.status.timing_faults, 0, "{what}");
    }
    // The same rig with a clean report arms; so does one that applied values.
    for report in [
        LoadReport::default(),
        LoadReport {
            applied: 12,
            ..LoadReport::default()
        },
    ] {
        let mut core = Core::new(cfg(), F405_LEAN).expect("valid config");
        core.report_param_load(report);
        assert!(!core.params_fault());
        let mut all = Vec::new();
        let (k, _) = run_events(&mut core, 0, 1100, rc(false, false), &mut all);
        let (_, out) = run_events(&mut core, k, 100, rc(true, false), &mut all);
        assert!(out.status.armed && core.is_armed());
        assert!(out.status.prearm.is_empty());
        assert!(
            all.iter()
                .all(|(_, e)| !matches!(e, EventKind::ParamLoadFault(_))),
            "no event for a clean load"
        );
        assert_eq!(out.status.timing_faults, 0);
    }
}

#[test]
fn the_fault_event_comes_once_on_the_first_tick() {
    for (what, report) in unclean_reports() {
        let mut core = Core::new(cfg(), F405_LEAN).expect("valid config");
        core.report_param_load(report);
        let mut all = Vec::new();
        run_events(&mut core, 0, 3000, rc(true, false), &mut all);
        let faults: Vec<_> = all
            .iter()
            .filter(|(_, e)| matches!(e, EventKind::ParamLoadFault(_)))
            .collect();
        assert_eq!(
            faults,
            vec![&(0, EventKind::ParamLoadFault(report))],
            "{what}"
        );
    }
}

#[test]
fn an_acknowledgement_clears_the_fault_sets_unsaved_and_the_vehicle_then_arms() {
    let report = LoadReport {
        unknown: 1,
        ..LoadReport::default()
    };
    let mut core = Core::new(cfg(), F405_LEAN).expect("valid config");
    core.report_param_load(report);
    let mut all = Vec::new();
    let (mut k, _) = run_events(&mut core, 0, 1100, rc(false, false), &mut all);
    let (k2, out) = run_events(&mut core, k, 100, rc(true, false), &mut all);
    k = k2;
    assert!(!out.status.armed && !out.status.params_unsaved && !core.params_unsaved());

    let out = step_ack(&mut core, k, rc(true, false), true);
    k += 1;
    assert!(!core.params_fault());
    assert!(core.params_unsaved() && out.status.params_unsaved);
    assert_eq!(
        events(&out),
        vec![EventKind::ParamFaultAcked],
        "exactly the acknowledgement"
    );
    // A second acknowledgement does nothing more.
    let out = step_ack(&mut core, k, rc(true, false), true);
    k += 1;
    assert!(events(&out).is_empty());

    // The flag clears on the next supervisor run; a fresh rising edge arms.
    let mut all = Vec::new();
    let (k2, out) = run_events(&mut core, k, 100, rc(false, false), &mut all);
    k = k2;
    assert!(!out.status.armed);
    assert!(out.status.prearm.is_empty(), "{:?}", out.status.prearm);
    let (_, out) = run_events(&mut core, k, 100, rc(true, false), &mut all);
    assert!(out.status.armed && core.is_armed());
    assert!(out.status.params_unsaved, "unsaved survives arming");
    assert_eq!(out.status.timing_faults, 0);
}

#[test]
fn an_acknowledgement_with_no_fault_held_emits_nothing_and_does_not_set_unsaved() {
    let mut core = Core::new(cfg(), F405_LEAN).expect("valid config");
    core.report_param_load(LoadReport {
        applied: 4,
        ..LoadReport::default()
    });
    let mut out = Outputs::off(Micros::ZERO);
    for k in 0..50 {
        out = step_ack(&mut core, k, rc(false, false), true);
        assert!(events(&out).is_empty(), "tick {k}");
    }
    assert!(!core.params_unsaved() && !out.status.params_unsaved);
    assert_eq!(out.status.timing_faults, 0);
    // The same on a core never told anything.
    let mut core = Core::new(cfg(), F405_LEAN).expect("valid config");
    let out = step_ack(&mut core, 0, rc(false, false), true);
    assert!(events(&out).is_empty() && !out.status.params_unsaved);
}

#[test]
fn an_acknowledgement_while_armed_is_ignored_until_the_vehicle_is_disarmed() {
    let (mut core, mut k) = armed_core(cfg());
    // A fault reported late, on a vehicle that is already flying.
    core.report_param_load(LoadReport {
        dropped: 1,
        ..LoadReport::default()
    });
    assert!(core.is_armed() && core.params_fault());
    let out = step_ack(&mut core, k, stick(0.0), true);
    k += 1;
    assert!(core.params_fault() && !core.params_unsaved());
    assert_eq!(
        events(&out)
            .into_iter()
            .filter(|e| matches!(e, EventKind::ParamFaultAcked))
            .count(),
        0
    );
    // Disarm (switch low for long enough), then it works.
    let mut all = Vec::new();
    let (k2, _) = run_events(&mut core, k, 200, rc(false, false), &mut all);
    k = k2;
    assert!(!core.is_armed());
    let out = step_ack(&mut core, k, rc(false, false), true);
    assert!(!core.params_fault() && core.params_unsaved());
    assert!(events(&out).contains(&EventKind::ParamFaultAcked));
    assert_eq!(out.status.timing_faults, 0);
}

#[test]
fn a_parameter_set_while_the_fault_is_held_is_applied_and_stays_unsaved() {
    let mut core = Core::new(cfg(), F405_LEAN).expect("valid config");
    core.report_param_load(LoadReport {
        unreadable: true,
        ..LoadReport::default()
    });
    let out = step(
        &mut core,
        0,
        rc(false, false),
        Some(set(ParamId::LIM_TILT, 0.5)),
    );
    assert!(events(&out).contains(&EventKind::ParamChanged(ParamId::LIM_TILT)));
    assert_eq!(core.config().limits.max_tilt_rad, 0.5);
    assert!(core.params_fault(), "a change does not acknowledge");
    assert!(core.params_unsaved() && out.status.params_unsaved);
    assert_eq!(out.status.timing_faults, 0);
}

// ---- skipped records: a clean load that may hold older values ----

#[test]
fn a_clean_report_with_torn_records_holds_no_fault_and_emits_one_recovered_event() {
    let report = LoadReport {
        applied: 4,
        torn: 2,
        ..LoadReport::default()
    };
    assert!(report.clean());
    let mut core = Core::new(cfg(), F405_LEAN).expect("valid config");
    core.report_param_load(report);
    assert!(!core.params_fault(), "torn records are not a fault");
    let mut all = Vec::new();
    let (k, _) = run_events(&mut core, 0, 1100, rc(false, false), &mut all);
    let (_, out) = run_events(&mut core, k, 100, rc(true, false), &mut all);
    // Arming is not refused.
    assert!(out.status.armed && core.is_armed());
    assert!(out.status.prearm.is_empty());
    // One event, on the first tick, carrying the report; no fault event.
    let recovered: Vec<_> = all
        .iter()
        .filter(|(_, e)| matches!(e, EventKind::ParamLoadRecovered(_)))
        .collect();
    assert_eq!(recovered, vec![&(0, EventKind::ParamLoadRecovered(report))]);
    assert!(
        all.iter()
            .all(|(_, e)| !matches!(e, EventKind::ParamLoadFault(_)))
    );
    // A clean report with nothing torn emits nothing.
    let mut core = Core::new(cfg(), F405_LEAN).expect("valid config");
    core.report_param_load(LoadReport {
        applied: 4,
        ..LoadReport::default()
    });
    let mut all = Vec::new();
    run_events(&mut core, 0, 50, rc(false, false), &mut all);
    assert!(all.is_empty(), "{all:?}");
}

#[test]
fn an_unclean_report_with_torn_records_is_a_fault_and_carries_the_count() {
    let report = LoadReport {
        unknown: 1,
        torn: 5,
        ..LoadReport::default()
    };
    let mut core = Core::new(cfg(), F405_LEAN).expect("valid config");
    core.report_param_load(report);
    assert!(core.params_fault());
    let mut all = Vec::new();
    run_events(&mut core, 0, 50, rc(false, false), &mut all);
    assert_eq!(all, vec![(0, EventKind::ParamLoadFault(report))]);
}

#[test]
fn a_second_report_replaces_the_first_and_emits_nothing_more() {
    let first = LoadReport {
        unknown: 1,
        ..LoadReport::default()
    };
    let second = LoadReport {
        dropped: 2,
        ..LoadReport::default()
    };
    // Before the first tick: one event, carrying the later report.
    let mut core = Core::new(cfg(), F405_LEAN).expect("valid config");
    core.report_param_load(first);
    core.report_param_load(second);
    let mut all = Vec::new();
    run_events(&mut core, 0, 50, rc(false, false), &mut all);
    assert_eq!(all, vec![(0, EventKind::ParamLoadFault(second))]);
    // After the event went out: the fault now holds the second report (an
    // acknowledgement is still needed) but no second event follows.
    let mut core = Core::new(cfg(), F405_LEAN).expect("valid config");
    core.report_param_load(first);
    let mut all = Vec::new();
    let (k, _) = run_events(&mut core, 0, 10, rc(false, false), &mut all);
    core.report_param_load(second);
    assert!(core.params_fault());
    let (k, _) = run_events(&mut core, k, 100, rc(false, false), &mut all);
    assert_eq!(all, vec![(0, EventKind::ParamLoadFault(first))]);
    let t = Micros(k * 1000);
    let mut i = inputs(t, rc(false, false));
    i.param_ack = true;
    let out = core.tick(t, &i);
    assert!(!core.params_fault());
    assert!(
        out.events
            .iter()
            .any(|e| matches!(e.kind, EventKind::ParamFaultAcked))
    );
    // A second report with torn records after the first one's event: no
    // second recovered event.
    let torn = LoadReport {
        torn: 1,
        ..LoadReport::default()
    };
    let mut core = Core::new(cfg(), F405_LEAN).expect("valid config");
    core.report_param_load(torn);
    let mut all = Vec::new();
    let (_, _) = run_events(&mut core, 0, 10, rc(false, false), &mut all);
    core.report_param_load(torn);
    run_events(&mut core, 10, 100, rc(false, false), &mut all);
    assert_eq!(all, vec![(0, EventKind::ParamLoadRecovered(torn))]);
}
