// SPDX-License-Identifier: AGPL-3.0-or-later
//! Rules between parameters and the vehicle (M1c part two, wave 4, task P):
//! each refuses a configuration that cannot fly, through `validate` and
//! through `params::set`, and accepts the default and the boundary.

use oxide_core::config::{
    ConfigError, EstimatorConfig, FailsafeParams, Gains, Limits, Rotor, Spin, VehicleConfig,
};
use oxide_core::math::Vec3;
use oxide_core::params::{self, ParamError, ParamId, ParamValue};
use oxide_core::schedule::{F405_LEAN, H743_FULL};

/// The reference 5-inch quad: 0.65 kg, four rotors of 8 N, expo 1.
pub fn quad() -> VehicleConfig<4> {
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

fn id(name: &str) -> ParamId {
    params::find(name).unwrap_or_else(|| panic!("no parameter {name}"))
}

/// `set` on both profiles; the configuration must be untouched on a refusal.
fn set(cfg: &mut VehicleConfig<4>, name: &str, v: f32) -> Result<(), ParamError> {
    let before = *cfg;
    let r = params::set(cfg, &F405_LEAN, id(name), ParamValue::F32(v));
    let mut other = before;
    let r2 = params::set(&mut other, &H743_FULL, id(name), ParamValue::F32(v));
    assert_eq!(r.is_ok(), r2.is_ok(), "{name}={v}: profiles disagree");
    if r.is_err() {
        assert_eq!(
            *cfg, before,
            "{name}={v}: a refusal changed the configuration"
        );
    }
    r
}

#[test]
fn a_multirotor_that_cannot_lift_itself_is_refused() {
    // 5 kg on four 8 N rotors: 32 N of thrust against 49 N of weight.
    let mut heavy = quad();
    heavy.mass_kg = 5.0;
    assert_eq!(heavy.validate(), Err(ConfigError::CannotLift));
    // The case of the review: with these values the old code passed validate
    // and panicked in the position controller.
    heavy.failsafe.land_blind_thrust = 0.95;
    heavy.failsafe.touchdown_thrust = 0.9;
    heavy.limits.min_thrust_fraction_of_hover = 0.8;
    assert_eq!(heavy.validate(), Err(ConfigError::CannotLift));
    // Through `set`: every change on such a vehicle is refused and leaves it
    // as it was.
    let before = heavy;
    assert_eq!(
        params::set(&mut heavy, &F405_LEAN, id("POS_X_P"), ParamValue::F32(1.0)),
        Err(ParamError::Invalid(ConfigError::CannotLift))
    );
    assert_eq!(heavy, before);
    // Mass in grams: 650 kg.
    let mut grams = quad();
    grams.mass_kg = 650.0;
    assert_eq!(grams.validate(), Err(ConfigError::CannotLift));
}

#[test]
fn thrust_just_above_weight_is_accepted_and_equal_is_refused() {
    let mut v = quad();
    assert_eq!(v.validate(), Ok(()));
    let total = v.total_max_thrust_n();
    // Weight equal to the total thrust: cannot climb.
    v.mass_kg = total / oxide_core::GRAVITY;
    if v.weight_n() < total {
        v.mass_kg = f32::from_bits(v.mass_kg.to_bits() + 1);
    }
    while v.weight_n() > total {
        v.mass_kg = f32::from_bits(v.mass_kg.to_bits() - 1);
    }
    while v.weight_n() < total {
        v.mass_kg = f32::from_bits(v.mass_kg.to_bits() + 1);
    }
    assert!(v.weight_n() >= total);
    assert_eq!(
        v.validate(),
        Err(ConfigError::CannotLift),
        "weight {} total {}",
        v.weight_n(),
        total
    );
    // One step lighter than that lifts, though barely.
    v.mass_kg = f32::from_bits(v.mass_kg.to_bits() - 1);
    while v.weight_n() >= total {
        v.mass_kg = f32::from_bits(v.mass_kg.to_bits() - 1);
    }
    assert!(v.weight_n() < total);
    assert_eq!(v.validate(), Ok(()));
}

#[test]
fn the_default_configuration_passes_every_rule() {
    let mut v = quad();
    assert_eq!(v.validate(), Ok(()));
    // Setting any parameter to its default is accepted.
    for (i, d) in params::PARAMS.iter().enumerate() {
        let r = params::set(&mut v, &F405_LEAN, ParamId(i as u16), d.default);
        assert_eq!(r, Ok(()), "{}", d.name);
    }
    let _ = set; // used by later tests
}
