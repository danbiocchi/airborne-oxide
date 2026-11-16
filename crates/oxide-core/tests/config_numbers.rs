// SPDX-License-Identifier: AGPL-3.0-or-later
//! The vehicle description as a list of numbers, and the configuration hash
//! (M1c part three, K1 step 4; ADR-0005 §6).

use oxide_core::config::{
    EstimatorConfig, FailsafeParams, Gains, Limits, NumbersError, Rotor, Spin, VehicleConfig,
};
use oxide_core::math::Vec3;
use oxide_core::params::{self, ParamId, ParamValue};
use oxide_core::schedule::F405_LEAN;

/// The reference 5-inch quad: 0.65 kg, four rotors of 8 N, expo 1.
fn quad() -> VehicleConfig<4> {
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

fn numbers<const N: usize>(c: &VehicleConfig<N>) -> Vec<(&'static str, f32)> {
    let mut v = Vec::new();
    c.vehicle_numbers(|name, x| v.push((name, x)));
    v
}

fn param_bits<const N: usize>(c: &VehicleConfig<N>) -> Vec<u32> {
    (0..params::PARAMS.len())
        .map(|i| match params::get(c, ParamId(i as u16)).unwrap() {
            ParamValue::F32(x) => x.to_bits(),
            ParamValue::I32(x) => x as u32,
        })
        .collect()
}

/// Rebuilds a configuration the way a replay header would: the vehicle
/// numbers, then every parameter.
fn rebuild<const N: usize>(c: &VehicleConfig<N>) -> VehicleConfig<N> {
    let nums = numbers(c);
    let mut r = VehicleConfig::<N>::from_vehicle_numbers(nums.iter().copied()).unwrap();
    let all = (0..params::PARAMS.len()).map(|i| {
        let id = ParamId(i as u16);
        (id, params::get(c, id).unwrap())
    });
    params::set_all(&mut r, &F405_LEAN, all).unwrap();
    r
}

fn set(c: &mut VehicleConfig<4>, name: &str, v: f32) {
    let id = params::find(name).unwrap();
    params::set(c, &F405_LEAN, id, ParamValue::F32(v)).unwrap();
}

/// A configuration with a non-default value in every vehicle number and in
/// several parameters.
fn modified() -> VehicleConfig<4> {
    let mut c = quad();
    c.mass_kg = 0.7;
    c.inertia = Vec3::new(1.7e-3, 1.8e-3, 2.9e-3);
    c.thrust_expo = 0.75;
    c.nominal_voltage_v = 14.8;
    c.battery_cells = 4;
    c.battery_resistance_ohm = 0.031;
    for (k, r) in c.rotors.iter_mut().enumerate() {
        r.x *= 1.0 + 0.01 * k as f32;
        r.y *= 1.0 - 0.01 * k as f32;
        r.max_thrust_n = 8.5 + k as f32 * 0.1;
        r.torque_coeff_m = 0.015 + k as f32 * 1.0e-4;
    }
    c.rotors[0].spin = Spin::Cw;
    set(&mut c, "RATE_ROLL_P", 31.0);
    set(&mut c, "FS_RC_TIMEOUT", 0.321);
    set(&mut c, "LIM_TILT", 0.5);
    c.validate().unwrap();
    c
}

#[test]
fn the_names_and_their_order_are_fixed() {
    let names: Vec<&str> = numbers(&quad()).iter().map(|(n, _)| *n).collect();
    assert_eq!(
        names,
        [
            "mass_kg",
            "inertia_x",
            "inertia_y",
            "inertia_z",
            "thrust_expo",
            "nominal_voltage_v",
            "battery_cells",
            "battery_resistance_ohm",
            "rotor0_x",
            "rotor0_y",
            "rotor0_spin",
            "rotor0_max_thrust_n",
            "rotor0_torque_coeff_m",
            "rotor1_x",
            "rotor1_y",
            "rotor1_spin",
            "rotor1_max_thrust_n",
            "rotor1_torque_coeff_m",
            "rotor2_x",
            "rotor2_y",
            "rotor2_spin",
            "rotor2_max_thrust_n",
            "rotor2_torque_coeff_m",
            "rotor3_x",
            "rotor3_y",
            "rotor3_spin",
            "rotor3_max_thrust_n",
            "rotor3_torque_coeff_m",
        ]
    );
    let q = numbers(&quad());
    let find = |n: &str| q.iter().find(|(m, _)| *m == n).unwrap().1;
    assert_eq!(find("rotor0_spin"), 1.0);
    assert_eq!(find("rotor2_spin"), -1.0);
    assert_eq!(find("battery_cells"), 6.0);
}

#[test]
fn a_configuration_rebuilt_from_its_numbers_and_parameters_is_the_original() {
    for c in [quad(), modified()] {
        let r = rebuild(&c);
        assert_eq!(r, c);
        let (a, b) = (numbers(&c), numbers(&r));
        assert_eq!(a.len(), b.len());
        for ((na, va), (nb, vb)) in a.iter().zip(b.iter()) {
            assert_eq!(na, nb);
            assert_eq!(va.to_bits(), vb.to_bits(), "{na}");
        }
        assert_eq!(param_bits(&c), param_bits(&r));
        assert_eq!(c.config_hash(), r.config_hash());
    }
    // A different vehicle is not the default one.
    assert_ne!(rebuild(&modified()), quad());
}

#[test]
fn the_numbers_may_come_in_any_order_but_not_wrong() {
    let c = modified();
    let mut nums = numbers(&c);
    nums.reverse();
    let mut r = VehicleConfig::<4>::from_vehicle_numbers(nums.iter().copied()).unwrap();
    let all = (0..params::PARAMS.len()).map(|i| {
        let id = ParamId(i as u16);
        (id, params::get(&c, id).unwrap())
    });
    params::set_all(&mut r, &F405_LEAN, all).unwrap();
    assert_eq!(r, c);

    let good = numbers(&quad());
    let build = |v: Vec<(&str, f32)>| VehicleConfig::<4>::from_vehicle_numbers(v);
    // Missing: each name in turn.
    for i in 0..good.len() {
        let mut v = good.clone();
        let (name, _) = v.remove(i);
        assert_eq!(build(v), Err(NumbersError::Missing(name)));
    }
    // Unknown names, including a rotor the vehicle does not have.
    let mut v = good.clone();
    v.push(("mass", 1.0));
    assert_eq!(build(v), Err(NumbersError::Unknown));
    let mut v = good.clone();
    v.push(("rotor4_x", 0.1));
    assert_eq!(build(v), Err(NumbersError::Unknown));
    // A name twice.
    let mut v = good.clone();
    v.push(("mass_kg", 0.5));
    assert_eq!(build(v), Err(NumbersError::Duplicate("mass_kg")));
    // Values that cannot be the field.
    for (name, bad) in [
        ("rotor1_spin", 0.0),
        ("rotor1_spin", 2.0),
        ("rotor1_spin", f32::NAN),
        ("battery_cells", 2.5),
        ("battery_cells", 256.0),
        ("battery_cells", -1.0),
        ("battery_cells", f32::NAN),
    ] {
        let mut v = good.clone();
        v.iter_mut().find(|(n, _)| *n == name).unwrap().1 = bad;
        assert_eq!(build(v), Err(NumbersError::BadValue(name)), "{name} {bad}");
    }
    // A vehicle with fewer rotors does not take a fourth.
    assert_eq!(
        VehicleConfig::<3>::from_vehicle_numbers(good.iter().copied()),
        Err(NumbersError::Unknown)
    );
}

/// The hash of a default-parameter vehicle `c` with its vehicle number
/// `index` changed to `to` (not validated: only the hash is looked at).
fn hash_with_number(c: &VehicleConfig<4>, index: usize, to: f32) -> u64 {
    assert_eq!(c.gains, Gains::default());
    let mut nums = numbers(c);
    nums[index].1 = to;
    VehicleConfig::<4>::from_vehicle_numbers(nums.iter().copied())
        .unwrap()
        .config_hash()
}

#[test]
fn changing_any_single_value_changes_the_hash() {
    let base = quad();
    let h = base.config_hash();
    // Every vehicle number: the next float up, and for the spins and the
    // cell count the other legal value.
    let nums = numbers(&base);
    for (i, (name, v)) in nums.iter().enumerate() {
        let to = if name.ends_with("_spin") {
            -v
        } else if *name == "battery_cells" {
            v + 1.0
        } else {
            f32::from_bits(v.to_bits() + 1)
        };
        assert_ne!(hash_with_number(&base, i, to), h, "{name}");
    }
    // Every parameter.
    let mut seen = std::collections::HashSet::new();
    seen.insert(h);
    for (i, d) in params::PARAMS.iter().enumerate() {
        let id = ParamId(i as u16);
        let near = match d.default {
            ParamValue::F32(x) => ParamValue::F32(x * 0.9),
            ParamValue::I32(x) => ParamValue::I32(x),
        };
        let mut changed = false;
        for v in [d.max, d.min, near] {
            let mut c = base;
            if params::set(&mut c, &F405_LEAN, id, v).is_err() || c == base {
                continue;
            }
            let hc = c.config_hash();
            assert_ne!(hc, h, "{}", d.name);
            // And every one of them gives a hash of its own.
            seen.insert(hc);
            changed = true;
            break;
        }
        assert!(changed, "{} never changes", d.name);
    }
    assert_eq!(seen.len(), 1 + params::PARAMS.len());
    // The same configuration hashes the same.
    assert_eq!(quad().config_hash(), h);
}

#[test]
fn the_hash_of_the_reference_quad_is_pinned() {
    // 28 vehicle numbers and 93 parameters, each as four little-endian bytes
    // of its bits, then the exact microseconds of the ten durations as four
    // little-endian bytes each, through 64-bit FNV-1a: also computed by an
    // independent Python script from the same list of bits. The first part
    // alone gave 0x4e22_87ac_ee8e_2c79; the pin changed only because the hash
    // now continues over the ten durations.
    assert_eq!(quad().config_hash(), 0xd5a0_f38d_7ab0_9979);
}

#[test]
fn two_durations_one_microsecond_apart_above_16_s_hash_differently() {
    let mut a = quad();
    let mut b = quad();
    // 100 s and 100 s + 1 us: the same f32 seconds.
    a.failsafe.rc_timeout_us = 100_000_000;
    b.failsafe.rc_timeout_us = 100_000_001;
    assert_eq!(
        params::get(&a, ParamId::FS_RC_TIMEOUT),
        params::get(&b, ParamId::FS_RC_TIMEOUT)
    );
    assert_ne!(a.config_hash(), b.config_hash());
    // Every duration counts.
    for id in params::micro_ids() {
        let c = quad();
        let mut d = quad();
        bump(&mut d, id);
        assert_ne!(c.config_hash(), d.config_hash(), "{id:?}");
    }
}

/// Adds one microsecond to the field of duration `id`.
fn bump(c: &mut VehicleConfig<4>, id: ParamId) {
    let (e, f) = (&mut c.estimator.ekf, &mut c.failsafe);
    let all = params::micro_ids().position(|x| x == id).unwrap();
    let field = [
        &mut e.delay_horizon_us,
        &mut e.align_time_us,
        &mut e.gps_timeout_us,
        &mut e.pos_timeout_us,
        &mut e.baro_timeout_us,
        &mut e.mag_timeout_us,
        &mut f.rc_timeout_us,
        &mut f.gcs_timeout_us,
        &mut f.brain_timeout_us,
        &mut f.setpoint_timeout_us,
    ]
    .into_iter()
    .nth(all)
    .unwrap();
    *field += 1;
}
