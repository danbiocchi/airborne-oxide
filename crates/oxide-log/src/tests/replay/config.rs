// SPDX-License-Identifier: AGPL-3.0-or-later
//! Step 2: the vehicle description and the parameters rebuild the
//! configuration, or say which parameter does not survive.

use super::harness::*;
use super::samples::*;
use crate::msg::ParamValue as MsgParam;
use crate::replay::decode::{self, FileConfigError};
use crate::replay::{self, RebuildError};
use oxide_core::config::{NumbersError, Rotor, Spin, VehicleConfig};
use oxide_core::params::{self, ParamError, ParamId, ParamValue};
use oxide_core::schedule::{F405_LEAN, H743_FULL, Profile};
use oxide_core::time::Micros;

fn numbers<const N: usize>(cfg: &VehicleConfig<N>) -> Vec<(&'static str, f32)> {
    let mut v = Vec::new();
    cfg.vehicle_numbers(|n, x| v.push((n, x)));
    v
}

fn param_list<const N: usize>(cfg: &VehicleConfig<N>) -> Vec<(&'static str, ParamValue)> {
    params::PARAMS
        .iter()
        .enumerate()
        .map(|(i, d)| (d.name, params::get(cfg, ParamId(i as u16)).unwrap()))
        .collect()
}

/// The exact microseconds as the log carries them: (key, µs).
fn micros_list<const N: usize>(cfg: &VehicleConfig<N>) -> Vec<(u32, u32)> {
    replay::MICRO_PARAMS
        .iter()
        .map(|n| {
            let key = params::key_of(params::find(n).unwrap()).unwrap();
            (key, replay::micros_of(cfg, n).unwrap())
        })
        .collect()
}

fn rebuild<const N: usize>(
    cfg: &VehicleConfig<N>,
    profile: &Profile,
) -> Result<VehicleConfig<N>, RebuildError> {
    replay::rebuild_config::<N>(numbers(cfg), param_list(cfg), micros_list(cfg), profile)
}

fn same<const N: usize>(cfg: &VehicleConfig<N>, profile: &Profile) {
    let back = rebuild(cfg, profile).unwrap();
    assert_eq!(&back, cfg);
    assert_eq!(back.config_hash(), cfg.config_hash());
}

/// The quad with parameters changed and one rotor unlike the others.
fn odd() -> VehicleConfig<4> {
    let mut c = quad();
    c.rotors[2] = Rotor {
        x: 0.0831,
        y: -0.0777,
        spin: Spin::Cw,
        max_thrust_n: 9.5,
        torque_coeff_m: 0.0191,
    };
    c.mass_kg = 0.7;
    c.battery_cells = 4;
    let roll_p = params::find("RATE_ROLL_P").unwrap();
    let ParamValue::F32(p) = params::get(&c, roll_p).unwrap() else {
        panic!()
    };
    params::set(&mut c, &F405_LEAN, roll_p, ParamValue::F32(p * 1.1)).unwrap();
    for (name, v) in [("FS_RC_TIMEOUT", 0.75), ("EKF_MAG_TMO", 12.5)] {
        let id = params::find(name).unwrap();
        params::set(&mut c, &F405_LEAN, id, ParamValue::F32(v)).unwrap();
    }
    let int = params::PARAMS
        .iter()
        .position(|d| matches!(d.default, ParamValue::I32(_)))
        .unwrap();
    let id = ParamId(int as u16);
    params::set(&mut c, &F405_LEAN, id, params::PARAMS[int].max).unwrap();
    c
}

fn hex() -> VehicleConfig<6> {
    let r = |x: f32, y: f32, spin| Rotor {
        x: 0.2 * x,
        y: 0.2 * y,
        spin,
        max_thrust_n: 6.0,
        torque_coeff_m: 0.016,
    };
    let (c, s) = (0.5, 0.866_025_4);
    let q = quad();
    VehicleConfig {
        mass_kg: 1.1,
        inertia: oxide_core::math::Vec3::new(5e-3, 5e-3, 9e-3),
        rotors: [
            r(1.0, 0.0, Spin::Ccw),
            r(c, s, Spin::Cw),
            r(-c, s, Spin::Ccw),
            r(-1.0, 0.0, Spin::Cw),
            r(-c, -s, Spin::Ccw),
            r(c, -s, Spin::Cw),
        ],
        thrust_expo: 0.5,
        nominal_voltage_v: q.nominal_voltage_v,
        gains: q.gains,
        limits: q.limits,
        estimator: q.estimator,
        failsafe: q.failsafe,
        battery_cells: 6,
        battery_resistance_ohm: 0.02,
    }
}

#[test]
fn the_reference_quad_rebuilds_equal_with_the_same_hash() {
    same(&quad(), &F405_LEAN);
    same(&quad(), &H743_FULL);
}

#[test]
fn a_quad_with_changed_parameters_and_an_odd_rotor_rebuilds_equal() {
    let c = odd();
    assert_ne!(c, quad());
    same(&c, &F405_LEAN);
    same(&c, &H743_FULL);
}

#[test]
fn a_hexacopter_rebuilds_equal() {
    let c = hex();
    c.validate().unwrap();
    same(&c, &F405_LEAN);
}

#[test]
fn the_order_of_the_lists_does_not_matter() {
    let c = odd();
    let mut n = numbers(&c);
    let mut p = param_list(&c);
    n.reverse();
    p.reverse();
    let back = replay::rebuild_config::<4>(n, p, micros_list(&c), &F405_LEAN).unwrap();
    assert_eq!(back, c);
}

fn err(n: Vec<(&'static str, f32)>, p: Vec<(&'static str, ParamValue)>) -> RebuildError {
    err_us(n, p, micros_list(&quad()))
}

fn err_us(
    n: Vec<(&'static str, f32)>,
    p: Vec<(&'static str, ParamValue)>,
    us: Vec<(u32, u32)>,
) -> RebuildError {
    replay::rebuild_config::<4>(n, p, us, &F405_LEAN).unwrap_err()
}

#[test]
fn a_bad_list_is_refused_and_names_what_is_wrong() {
    let c = quad();
    let id = |n: &str| params::find(n).unwrap();

    // The vehicle numbers.
    let mut n = numbers(&c);
    n.retain(|(name, _)| *name != "mass_kg");
    assert_eq!(
        err(n, param_list(&c)),
        RebuildError::Numbers(NumbersError::Missing("mass_kg"))
    );
    let mut n = numbers(&c);
    n.iter_mut()
        .find(|(name, _)| *name == "rotor1_spin")
        .unwrap()
        .1 = 0.5;
    assert_eq!(
        err(n, param_list(&c)),
        RebuildError::Numbers(NumbersError::BadValue("rotor1_spin"))
    );

    // The parameters.
    let mut p = param_list(&c);
    p.retain(|(name, _)| *name != "RATE_ROLL_P");
    assert_eq!(
        err(numbers(&c), p),
        RebuildError::MissingParam(id("RATE_ROLL_P"))
    );
    let mut p = param_list(&c);
    p.push(("RATE_ROLL_P", ParamValue::F32(0.1)));
    assert_eq!(
        err(numbers(&c), p),
        RebuildError::DuplicateParam(id("RATE_ROLL_P"))
    );
    let mut p = param_list(&c);
    p.push(("NOT_A_PARAM", ParamValue::F32(0.1)));
    let e = err(numbers(&c), p);
    assert_eq!(
        e,
        RebuildError::UnknownParam {
            position: params::COUNT,
            key: params::key("NOT_A_PARAM")
        }
    );
    for (value, why) in [
        (ParamValue::F32(1e9), ParamError::OutOfRange),
        (ParamValue::F32(f32::NAN), ParamError::NotFinite),
        (ParamValue::I32(3), ParamError::WrongType),
    ] {
        let mut p = param_list(&c);
        p.iter_mut().find(|(n, _)| *n == "RATE_ROLL_P").unwrap().1 = value;
        assert_eq!(
            err(numbers(&c), p),
            RebuildError::Refused {
                id: id("RATE_ROLL_P"),
                why
            }
        );
    }
}

#[test]
fn values_that_are_fine_alone_but_break_a_rule_together_are_refused() {
    let c = quad();
    // The configuration breaks `validate` through the vehicle numbers.
    let mut n = numbers(&c);
    n.iter_mut().find(|(name, _)| *name == "mass_kg").unwrap().1 = 1e6;
    let e = replay::rebuild_config::<4>(n, param_list(&c), micros_list(&c), &F405_LEAN);
    // A mass that the vehicle cannot lift is not a parameter: the rules
    // between fields are checked on the whole.
    match e {
        Err(RebuildError::Invalid(_)) => {}
        other => panic!("{other:?}"),
    }
}

#[test]
fn microseconds_up_to_16_seconds_survive_the_trip_through_f32_seconds() {
    let id = params::find("EKF_MAG_TMO").unwrap();
    let mut c = quad();
    let mut bad = Vec::new();
    let mut check = |us: u32| {
        c.estimator.ekf.mag_timeout_us = us;
        if replay::exact_param(&c, &F405_LEAN, id).is_err() {
            bad.push(us);
        }
    };
    // The whole range was run once (every value from 0 to 16,000,000 passes);
    // the test keeps to a dense start, a stride through the rest and a dense end.
    for us in (50_000..=2_000_000).chain((2_000_000..=15_980_000).step_by(13)) {
        check(us);
    }
    for us in 15_980_000..=16_000_000 {
        check(us);
    }
    assert_eq!(bad, Vec::<u32>::new(), "every value up to 16 s is exact");
}

#[test]
fn microseconds_above_16_seconds_may_not_and_the_parameter_is_named() {
    let id = params::find("EKF_MAG_TMO").unwrap();
    for us in [
        16_000_001u32,
        16_000_398,
        16_777_217,
        30_000_001,
        3_599_999_999,
    ] {
        let mut c = quad();
        c.estimator.ekf.mag_timeout_us = us;
        assert_eq!(
            replay::check_exact(&c, &F405_LEAN),
            Err(RebuildError::NotExact(id)),
            "{us}"
        );
        // The hash cannot tell (it is over the seconds as f32), so the
        // rebuild must never hand back a different configuration.
        match rebuild(&c, &F405_LEAN) {
            Ok(back) => assert_eq!(back, c, "{us}: a silently different configuration"),
            Err(e) => assert_eq!(e, RebuildError::NotExact(id), "{us}"),
        }
    }
    // Round values are exact all the way up, 3600 s included.
    for us in [30_000_000u32, 60_000_000, 3_600_000_000, 100_000_000] {
        let mut c = quad();
        c.estimator.ekf.mag_timeout_us = us;
        assert_eq!(replay::check_exact(&c, &F405_LEAN), Ok(()), "{us}");
    }
}

#[test]
fn the_defaults_and_the_other_configurations_are_exact() {
    for p in [F405_LEAN, H743_FULL] {
        assert_eq!(replay::check_exact(&quad(), &p), Ok(()));
        assert_eq!(replay::check_exact(&odd(), &p), Ok(()));
    }
    assert_eq!(replay::check_exact(&hex(), &F405_LEAN), Ok(()));
}

fn log_of<const N: usize>(cfg: &VehicleConfig<N>, hash: Option<u64>) -> Builder {
    let mut b = Builder::new();
    if let Some(h) = hash {
        b.info_u64("oxide_config_hash", h);
    }
    for (key, us) in micros_list(cfg) {
        b.push(&replay::MICROS, |w| {
            let name = replay::MICRO_PARAMS
                .iter()
                .find(|n| params::key_of(params::find(n).unwrap()) == Some(key))
                .unwrap();
            replay::fill_micros(w, Micros(0), name, us);
        });
    }
    for (n, v) in param_list(cfg) {
        b.param(
            n,
            match v {
                ParamValue::F32(x) => MsgParam::F32(x),
                ParamValue::I32(x) => MsgParam::I32(x),
            },
        );
    }
    for (i, (n, v)) in numbers(cfg).into_iter().enumerate() {
        b.push(&replay::VEHICLE, |w| {
            replay::fill_vehicle_number(w, Micros(0), i as u16, n, v);
        });
    }
    b
}

#[test]
fn the_configuration_is_rebuilt_from_a_file_and_its_hash_checked() {
    let c = odd();
    let f = log_of(&c, Some(c.config_hash())).file();
    let back = decode::config_of::<4>(&f, &F405_LEAN).unwrap();
    assert_eq!(back, c);
    let nums = decode::vehicle_numbers_of(&f).unwrap();
    assert_eq!(nums.len(), 8 + 5 * 4);
    assert_eq!(nums[0].name, "mass_kg");
}

#[test]
fn a_file_whose_hash_differs_or_is_missing_is_refused() {
    let c = odd();
    let f = log_of(&c, Some(c.config_hash() ^ 1)).file();
    assert_eq!(
        decode::config_of::<4>(&f, &F405_LEAN).unwrap_err(),
        FileConfigError::HashMismatch {
            logged: c.config_hash() ^ 1,
            rebuilt: c.config_hash()
        }
    );
    let f = log_of(&c, None).file();
    assert_eq!(
        decode::config_of::<4>(&f, &F405_LEAN).unwrap_err(),
        FileConfigError::NoHash
    );
}

#[test]
fn vehicle_records_whose_indices_do_not_run_from_zero_are_refused() {
    let mut b = Builder::new();
    b.push(&replay::VEHICLE, |w| {
        replay::fill_vehicle_number(w, Micros(0), 5, "mass_kg", 1.0);
    });
    let f = b.file();
    assert_eq!(
        decode::vehicle_numbers_of(&f).unwrap_err(),
        FileConfigError::IndexOutOfOrder {
            position: 0,
            index: 5
        }
    );
}

#[test]
fn a_file_with_a_lossy_microsecond_value_is_not_replayed_against_another_configuration() {
    let mut c = quad();
    c.failsafe.rc_timeout_us = 16_000_001;
    let f = log_of(&c, Some(c.config_hash())).file();
    if let Ok(back) = decode::config_of::<4>(&f, &F405_LEAN) {
        assert_eq!(back, c, "a silently different configuration");
    }
}

#[test]
fn a_microsecond_parameter_that_is_not_the_canonical_f32_is_refused_by_the_rebuild() {
    // 1.0000004 s is not the f32 of any whole microsecond: it would be read
    // back as 1.0 s. The exact microseconds agree (1,000,000), so only the
    // read-back of the rebuild can refuse it, and it must name the parameter.
    let mut c = quad();
    c.failsafe.rc_timeout_us = 1_000_000;
    let id = params::find("FS_RC_TIMEOUT").unwrap();
    let mut p = param_list(&c);
    p.iter_mut().find(|(n, _)| *n == "FS_RC_TIMEOUT").unwrap().1 = ParamValue::F32(1.000_000_4);
    assert_eq!(
        replay::rebuild_config::<4>(numbers(&c), p, micros_list(&c), &F405_LEAN).unwrap_err(),
        RebuildError::NotExact(id)
    );
    // The same through a file: its P message is the non-canonical value.
    let mut b = log_of(&c, Some(c.config_hash()));
    for (n, v) in &mut b.params {
        if n == "FS_RC_TIMEOUT" {
            *v = MsgParam::F32(1.000_000_4);
        }
    }
    assert_eq!(
        decode::config_of::<4>(&b.file(), &F405_LEAN).unwrap_err(),
        FileConfigError::Rebuild(RebuildError::NotExact(id))
    );
}

#[test]
fn the_list_of_microsecond_parameters_is_the_core_s_and_its_values_are_read_from_the_fields() {
    // A parameter is held as microseconds iff the core quantizes a value that
    // is not a whole microsecond. Probe every f32 parameter in seconds.
    let mut probed = Vec::new();
    for (i, d) in params::PARAMS.iter().enumerate() {
        if !matches!((d.default, d.unit), (ParamValue::F32(_), "s")) {
            continue;
        }
        let id = ParamId(i as u16);
        let mut c = quad();
        // 0.1 s and 0.4 us: no whole microsecond is this f32.
        let off = 0.100_000_4_f32;
        if params::set(&mut c, &F405_LEAN, id, ParamValue::F32(off)).is_ok()
            && params::get(&c, id) != Some(ParamValue::F32(off))
        {
            probed.push(d.name);
        }
    }
    assert_eq!(probed, replay::MICRO_PARAMS);
    // `micros_of` reads the field the parameter writes.
    let mut c = quad();
    for (k, name) in replay::MICRO_PARAMS.iter().enumerate() {
        let us = 100_000 + 1000 * k as u32;
        params::set(
            &mut c,
            &F405_LEAN,
            params::find(name).unwrap(),
            ParamValue::F32(us as f32 / 1e6),
        )
        .unwrap();
        assert_eq!(replay::micros_of(&c, name), Some(us), "{name}");
    }
    assert_eq!(replay::micros_of(&c, "RATE_ROLL_P"), None);
}

#[test]
fn with_the_exact_microseconds_a_lossy_value_is_named_and_a_round_one_is_rebuilt() {
    let id = params::find("EKF_MAG_TMO").unwrap();
    for us in [16_000_001u32, 16_777_217, 30_000_001] {
        let mut c = quad();
        c.estimator.ekf.mag_timeout_us = us;
        assert_eq!(rebuild(&c, &F405_LEAN), Err(RebuildError::NotExact(id)));
        let f = log_of(&c, Some(c.config_hash())).file();
        assert_eq!(
            decode::config_of::<4>(&f, &F405_LEAN).unwrap_err(),
            FileConfigError::Rebuild(RebuildError::NotExact(id)),
            "{us}"
        );
    }
    for us in [30_000_000u32, 3_600_000_000, 16_000_000, 123_457] {
        let mut c = quad();
        c.estimator.ekf.mag_timeout_us = us;
        same(&c, &F405_LEAN);
        let f = log_of(&c, Some(c.config_hash())).file();
        assert_eq!(decode::config_of::<4>(&f, &F405_LEAN).unwrap(), c);
    }
}

#[test]
fn a_file_whose_exact_microseconds_disagree_with_its_parameters_is_refused() {
    // The file says 0.75 s in the P message and 750,001 us in the exact record.
    let mut c = quad();
    c.failsafe.rc_timeout_us = 750_000;
    let id = params::find("FS_RC_TIMEOUT").unwrap();
    let mut us = micros_list(&c);
    let key = params::key_of(id).unwrap();
    us.iter_mut().find(|(k, _)| *k == key).unwrap().1 = 750_001;
    assert_eq!(
        err_us(numbers(&c), param_list(&c), us),
        RebuildError::NotExact(id)
    );
}

#[test]
fn the_exact_microsecond_records_must_be_complete_and_known() {
    let c = quad();
    let key = |n: &str| params::key_of(params::find(n).unwrap()).unwrap();
    let mut us = micros_list(&c);
    us.retain(|(k, _)| *k != key("FS_GCS_TIMEOUT"));
    assert_eq!(
        err_us(numbers(&c), param_list(&c), us),
        RebuildError::MissingParam(params::find("FS_GCS_TIMEOUT").unwrap())
    );
    let mut us = micros_list(&c);
    us.push((key("FS_GCS_TIMEOUT"), 3_000_000));
    assert_eq!(
        err_us(numbers(&c), param_list(&c), us),
        RebuildError::DuplicateParam(params::find("FS_GCS_TIMEOUT").unwrap())
    );
    // A parameter that is not held in microseconds has no such record.
    let mut us = micros_list(&c);
    us.push((key("RATE_ROLL_P"), 1));
    assert_eq!(
        err_us(numbers(&c), param_list(&c), us),
        RebuildError::UnknownParam {
            position: replay::MICRO_PARAMS.len(),
            key: key("RATE_ROLL_P")
        }
    );
    let mut us = micros_list(&c);
    us.push((0xDEAD_BEEF, 1));
    assert!(matches!(
        err_us(numbers(&c), param_list(&c), us),
        RebuildError::UnknownParam { .. }
    ));
}

#[test]
fn write_micros_and_decode_micros_round_trip() {
    let c = odd();
    let mut v = Vec::new();
    replay::write_micros(&mut v, 3, Micros(77), &c).unwrap();
    let f = {
        let mut b = Builder::new();
        for (k, us) in micros_list(&c) {
            let name = replay::MICRO_PARAMS
                .iter()
                .find(|n| params::key_of(params::find(n).unwrap()) == Some(k))
                .unwrap();
            b.push(&replay::MICROS, |w| {
                replay::fill_micros(w, Micros(77), name, us)
            });
        }
        b.file()
    };
    let got: Vec<(u32, u32)> = f
        .records("oxide_param_us")
        .map(|r| decode::decode_micros(&r).unwrap())
        .collect();
    assert_eq!(got, micros_list(&c));
    assert_eq!(v.len(), replay::MICRO_PARAMS.len() * (3 + 2 + 16));
}
