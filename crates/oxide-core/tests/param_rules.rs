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

// ---- LIM_IDLE ----

/// The thrust fraction a motor command gives (the curve `run_rate` inverts).
fn fraction(u: f32, expo: f32) -> f32 {
    (1.0 - expo) * u + expo * u * u
}

#[test]
fn idle_must_give_at_most_half_the_hover_thrust() {
    let q = quad();
    // The table maximum is 0.5 now.
    let def = &params::PARAMS[id("LIM_IDLE").index()];
    assert_eq!(def.max, ParamValue::F32(0.5));
    assert!(def.description.contains("half"), "{}", def.description);
    // Hover is 0.65 * g / 32 N = 0.199 of full thrust per rotor; half of that
    // is reached by command sqrt(0.0996) = 0.3156 with expo 1.
    let hover_fraction = q.weight_n() / q.total_max_thrust_n();
    let mut ok = 0.0_f32;
    let mut bad = 0.5_f32;
    // Bisect the largest accepted value through `set`.
    for _ in 0..40 {
        let mid = 0.5 * (ok + bad);
        let mut c = q;
        if set(&mut c, "LIM_IDLE", mid).is_ok() {
            ok = mid;
        } else {
            bad = mid;
        }
    }
    assert!(
        (fraction(ok, 1.0) - 0.5 * hover_fraction).abs() < 1e-4,
        "largest accepted idle {ok} gives fraction {} against half hover {}",
        fraction(ok, 1.0),
        0.5 * hover_fraction
    );
    // Just above is refused with the rule's own error, below is accepted, the
    // default is accepted, and a refusal leaves the configuration alone.
    let mut c = q;
    assert_eq!(set(&mut c, "LIM_IDLE", 0.05), Ok(()));
    assert_eq!(set(&mut c, "LIM_IDLE", ok), Ok(()));
    assert_eq!(c.limits.idle_command, ok);
    let mut c = q;
    assert_eq!(
        set(&mut c, "LIM_IDLE", 0.45),
        Err(ParamError::Invalid(ConfigError::IdleTooHigh))
    );
    assert_eq!(
        set(&mut c, "LIM_IDLE", 0.6),
        Err(ParamError::OutOfRange),
        "above the table maximum"
    );
    // The old table maximum, 1, is out of range; 0.5 is not accepted on this
    // vehicle either.
    assert_eq!(
        set(&mut c, "LIM_IDLE", 0.5),
        Err(ParamError::Invalid(ConfigError::IdleTooHigh))
    );
    // validate sees it too, whatever way the value got there.
    let mut v = q;
    v.limits.idle_command = 0.45;
    assert_eq!(v.validate(), Err(ConfigError::IdleTooHigh));
    v.limits.idle_command = ok;
    assert_eq!(v.validate(), Ok(()));
}

#[test]
fn the_idle_rule_follows_the_thrust_curve_and_the_weight() {
    // Expo 0: thrust is linear in the command, so half of hover is
    // 0.5 * 0.199 = 0.0996.
    let mut lin = quad();
    lin.thrust_expo = 0.0;
    let mut c = lin;
    assert_eq!(set(&mut c, "LIM_IDLE", 0.09), Ok(()));
    assert_eq!(
        set(&mut c, "LIM_IDLE", 0.11),
        Err(ParamError::Invalid(ConfigError::IdleTooHigh))
    );
    // A heavier vehicle (hover at 0.4 of full thrust) allows a larger idle
    // command than the light one: 0.5 * 0.4 = 0.2 of thrust, command 0.447.
    let mut heavy = quad();
    heavy.mass_kg = 1.3;
    let mut c = heavy;
    assert_eq!(set(&mut c, "LIM_IDLE", 0.4), Ok(()));
    assert_eq!(
        set(&mut c, "LIM_IDLE", 0.5),
        Err(ParamError::Invalid(ConfigError::IdleTooHigh))
    );
}

// ---- EKF_OUT_TAU ----

fn set_both(
    cfg: &mut VehicleConfig<4>,
    profile: &oxide_core::schedule::Profile,
    list: &[(&str, f32)],
) -> Result<(), (Option<ParamId>, ParamError)> {
    let changes: Vec<_> = list
        .iter()
        .map(|(n, v)| (id(n), ParamValue::F32(*v)))
        .collect();
    params::set_all(cfg, profile, changes)
}

#[test]
fn the_output_time_constant_has_a_minimum_and_follows_the_delay_horizon() {
    let q = quad();
    let def = &params::PARAMS[id("EKF_OUT_TAU").index()];
    assert_eq!(def.min, ParamValue::F32(0.15));
    assert!(
        def.description.contains("EKF_HORIZON") && def.description.contains("0.15"),
        "{}",
        def.description
    );
    // Below the minimum is out of range, whatever the horizon.
    for tau in [0.14, 0.1, 0.05, 1e-3, 1e-6] {
        let mut c = q;
        assert_eq!(set(&mut c, "EKF_OUT_TAU", tau), Err(ParamError::OutOfRange));
    }
    // With the default horizon (0.2 s) a time constant below it is refused by
    // the rule between the two, with the configuration left alone.
    let mut c = q;
    assert_eq!(
        set(&mut c, "EKF_OUT_TAU", 0.15),
        Err(ParamError::Invalid(ConfigError::BadEstimator))
    );
    assert_eq!(
        set(&mut c, "EKF_OUT_TAU", 0.19),
        Err(ParamError::Invalid(ConfigError::BadEstimator))
    );
    // Equal to the horizon, and the default, are accepted.
    assert_eq!(set(&mut c, "EKF_OUT_TAU", 0.2), Ok(()));
    assert_eq!(set(&mut c, "EKF_OUT_TAU", 0.25), Ok(()));
    // The horizon cannot move above the time constant either.
    let mut c = q;
    assert_eq!(set(&mut c, "EKF_OUT_TAU", 0.2), Ok(()));
    assert_eq!(
        set(&mut c, "EKF_HORIZON", 0.21),
        Err(ParamError::Invalid(ConfigError::BadEstimator))
    );
    // Both at their lowest together: the minimum time constant with a horizon
    // that long or shorter. Equality holds at the table's upper horizon too.
    for profile in [&F405_LEAN, &H743_FULL] {
        let mut c = q;
        assert_eq!(
            set_both(
                &mut c,
                profile,
                &[("EKF_HORIZON", 0.15), ("EKF_OUT_TAU", 0.15)]
            ),
            Ok(())
        );
        let mut c = q;
        assert_eq!(
            set_both(
                &mut c,
                profile,
                &[("EKF_HORIZON", 0.248), ("EKF_OUT_TAU", 0.248)]
            ),
            Ok(())
        );
        let mut c = q;
        assert!(matches!(
            set_both(
                &mut c,
                profile,
                &[("EKF_HORIZON", 0.248), ("EKF_OUT_TAU", 0.247)]
            ),
            Err((None, ParamError::Invalid(ConfigError::BadEstimator)))
        ));
        assert_eq!(c, q);
    }
    // validate and the estimator's own check see it however the value got
    // there.
    let mut v = q;
    v.estimator.ekf.output_tau_s = 0.15;
    assert_eq!(v.validate(), Err(ConfigError::BadEstimator));
    assert_eq!(
        v.estimator.ekf.validate(&F405_LEAN),
        Err(ConfigError::BadEstimator)
    );
}

// ---- landing ----

#[test]
fn the_descent_limit_must_allow_the_landing_speed_to_count_as_a_command() {
    let q = quad();
    // Contact is declared at twice LAND_TD_SPEED (0.25 by default), so
    // LIM_SPEED_DOWN, which clamps the descent LAND_SPEED commands, must be at
    // least that.
    let mut c = q;
    assert_eq!(
        set(&mut c, "LIM_SPEED_DOWN", 0.49),
        Err(ParamError::Invalid(ConfigError::DescentLimitTooLow))
    );
    assert_eq!(
        set(&mut c, "LIM_SPEED_DOWN", 0.01),
        Err(ParamError::Invalid(ConfigError::DescentLimitTooLow))
    );
    assert_eq!(
        set(&mut c, "LIM_SPEED_DOWN", 0.5),
        Ok(()),
        "equal is accepted"
    );
    assert_eq!(set(&mut c, "LIM_SPEED_DOWN", 1.5), Ok(()), "the default");
    // The touchdown speed cannot rise past what the limit allows either.
    let mut c = q;
    assert_eq!(set(&mut c, "LIM_SPEED_DOWN", 0.6), Ok(()));
    assert_eq!(
        set(&mut c, "LAND_TD_SPEED", 0.31),
        Err(ParamError::Invalid(ConfigError::DescentLimitTooLow))
    );
    assert_eq!(set(&mut c, "LAND_TD_SPEED", 0.3), Ok(()));
    // Together, in one change: a faster landing with a limit to match.
    for profile in [&F405_LEAN, &H743_FULL] {
        let mut c = q;
        assert_eq!(
            set_both(
                &mut c,
                profile,
                &[
                    ("LAND_TD_SPEED", 1.0),
                    ("LAND_SPEED", 2.0),
                    ("LIM_SPEED_DOWN", 2.0)
                ]
            ),
            Ok(())
        );
        let mut c = q;
        assert!(matches!(
            set_both(
                &mut c,
                profile,
                &[
                    ("LAND_TD_SPEED", 1.0),
                    ("LAND_SPEED", 2.0),
                    ("LIM_SPEED_DOWN", 1.9)
                ]
            ),
            Err((None, ParamError::Invalid(ConfigError::DescentLimitTooLow)))
        ));
        assert_eq!(c, q);
    }
    let mut v = q;
    v.limits.max_speed_down = 0.4;
    assert_eq!(v.validate(), Err(ConfigError::DescentLimitTooLow));
}

#[test]
fn the_touchdown_speed_and_time_have_ranges_that_can_be_detected() {
    let q = quad();
    let td_speed = &params::PARAMS[id("LAND_TD_SPEED").index()];
    assert_eq!(td_speed.min, ParamValue::F32(0.1));
    assert!(
        td_speed.description.contains("0.1"),
        "{}",
        td_speed.description
    );
    let td_time = &params::PARAMS[id("LAND_TD_TIME").index()];
    assert_eq!(
        (td_time.min, td_time.max),
        (ParamValue::F32(0.1), ParamValue::F32(10.0))
    );
    for (name, bad) in [
        ("LAND_TD_SPEED", 0.09),
        ("LAND_TD_SPEED", 0.01),
        ("LAND_TD_SPEED", 1e-6),
        ("LAND_TD_TIME", 0.09),
        ("LAND_TD_TIME", 1e-6),
        ("LAND_TD_TIME", 10.1),
        ("LAND_TD_TIME", 3600.0),
    ] {
        let mut c = q;
        assert_eq!(
            set(&mut c, name, bad),
            Err(ParamError::OutOfRange),
            "{name} {bad}"
        );
        assert_eq!(c, q);
    }
    for (name, ok) in [
        ("LAND_TD_SPEED", 0.1),
        ("LAND_TD_TIME", 0.1),
        ("LAND_TD_TIME", 10.0),
        ("LAND_TD_TIME", 1.0),
        ("LAND_TD_SPEED", 0.25),
    ] {
        let mut c = q;
        assert_eq!(set(&mut c, name, ok), Ok(()), "{name} {ok}");
    }
    // At the new minimum of the touchdown speed the landing and descent
    // limits may go to the smallest values the rules then allow: 0.2.
    for profile in [&F405_LEAN, &H743_FULL] {
        let mut c = q;
        assert_eq!(
            set_both(
                &mut c,
                profile,
                &[
                    ("LAND_TD_SPEED", 0.1),
                    ("LAND_SPEED", 0.2),
                    ("LIM_SPEED_DOWN", 0.2)
                ]
            ),
            Ok(())
        );
        let mut c = q;
        assert!(
            set_both(
                &mut c,
                profile,
                &[
                    ("LAND_TD_SPEED", 0.1),
                    ("LAND_SPEED", 0.19),
                    ("LIM_SPEED_DOWN", 0.2)
                ]
            )
            .is_err()
        );
    }
}
