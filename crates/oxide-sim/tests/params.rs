// SPDX-License-Identifier: AGPL-3.0-or-later
//! Parameter overrides in options and in the airframe file (ADR-0004).

use oxide_core::schedule::F405_LEAN;
use oxide_core::time::Micros;
use oxide_sim::airframe::REFERENCE_QUAD_TOML;
use oxide_sim::scenario::HoverStep;
use oxide_sim::{Airframe, SimError, SimOptions, Simulation};

fn hash_with(airframe: &Airframe, params: &[(&str, f64)]) -> Result<u64, SimError> {
    let options = SimOptions {
        params: params.iter().map(|(n, v)| ((*n).to_string(), *v)).collect(),
        ..SimOptions::default()
    };
    let mut sim =
        Simulation::<_, 4>::with_options(airframe, F405_LEAN, HoverStep::default(), options)?;
    sim.run_until(Micros::from_secs(15), |_| {});
    Ok(sim.trace_hash())
}

fn reference() -> Airframe {
    Airframe::reference_quad().expect("reference airframe")
}

fn refused(params: &[(&str, f64)]) -> String {
    match hash_with(&reference(), params) {
        Err(e @ SimError::Param { .. }) => e.to_string(),
        other => panic!("expected a parameter error, got {other:?}"),
    }
}

#[test]
fn empty_override_list_does_not_change_the_flight() {
    let a = reference();
    let plain = {
        let mut sim = Simulation::<_, 4>::new(&a, F405_LEAN, HoverStep::default()).expect("sim");
        sim.run_until(Micros::from_secs(15), |_| {});
        sim.trace_hash()
    };
    assert_eq!(hash_with(&a, &[]).expect("run"), plain);
}

#[test]
fn an_override_changes_the_flight_and_is_repeatable() {
    let a = reference();
    let base = hash_with(&a, &[]).expect("base");
    let one = hash_with(&a, &[("POS_X_P", 1.2)]).expect("override");
    let again = hash_with(&a, &[("POS_X_P", 1.2)]).expect("override again");
    assert_ne!(base, one);
    assert_eq!(one, again);
}

#[test]
fn a_whole_number_is_accepted_for_a_float_and_the_source_follows_the_parameter() {
    let a = reference();
    // 3 is an integer in a TOML file; as an f64 it is fine for an f32 parameter.
    assert!(hash_with(&a, &[("RTL_ALT", 12.0)]).is_ok());
    // EKF_SOURCE = 1 on the default (truth) options still flies: the state
    // input follows the configuration.
    let ekf = hash_with(&a, &[("EKF_SOURCE", 1.0)]).expect("ekf");
    assert_ne!(ekf, hash_with(&a, &[]).expect("truth"));
}

#[test]
fn each_error_names_the_parameter_the_value_and_the_reason() {
    let e = refused(&[("NO_SUCH_PARAM", 1.0)]);
    assert!(
        e.contains("NO_SUCH_PARAM") && e.contains("no such parameter"),
        "{e}"
    );
    let e = refused(&[("EKF_SOURCE", 0.5)]);
    assert!(
        e.contains("EKF_SOURCE") && e.contains("0.5") && e.contains("whole number"),
        "{e}"
    );
    let e = refused(&[("POS_X_P", 1.0e9)]);
    assert!(
        e.contains("POS_X_P") && e.contains("1000000000") && e.contains("range"),
        "{e}"
    );
    let e = refused(&[("POS_X_P", f64::NAN)]);
    assert!(e.contains("POS_X_P") && e.contains("finite"), "{e}");
    let e = refused(&[("BAT_CRIT_V", 3.9)]);
    assert!(e.contains("BAT_CRIT_V") && e.contains("rule"), "{e}");
}

#[test]
fn overrides_apply_in_order() {
    // Raising the low level first and then the critical one is valid at each
    // step; the reverse order breaks the rule (critical above low) at the
    // first step and is refused.
    let a = reference();
    assert!(hash_with(&a, &[("BAT_LOW_V", 3.8), ("BAT_CRIT_V", 3.6)]).is_ok());
    assert!(hash_with(&a, &[("BAT_CRIT_V", 3.6), ("BAT_LOW_V", 3.8)]).is_err());
}

#[test]
fn the_airframe_params_table_is_applied_before_the_options() {
    let text = format!("{REFERENCE_QUAD_TOML}\n[params]\nPOS_X_P = 1.2\nRTL_ALT = 12\n");
    let a = Airframe::from_toml(&text).expect("airframe with params");
    assert_eq!(
        a.params,
        vec![("POS_X_P".to_string(), 1.2), ("RTL_ALT".to_string(), 12.0)]
    );
    // Same flight as the override given as an option.
    let from_file = hash_with(&a, &[]).expect("file");
    let from_option =
        hash_with(&reference(), &[("POS_X_P", 1.2), ("RTL_ALT", 12.0)]).expect("option");
    assert_eq!(from_file, from_option);
    // An option wins over the file.
    let both = hash_with(&a, &[("POS_X_P", 1.5)]).expect("both");
    let only = hash_with(&reference(), &[("RTL_ALT", 12.0)]).expect("only");
    assert_eq!(both, only, "POS_X_P is back at its default 1.5");
}

#[test]
fn a_bad_name_in_the_airframe_file_is_refused_naming_it() {
    let text = format!("{REFERENCE_QUAD_TOML}\n[params]\nPOS_TYPO = 1.0\n");
    let a = Airframe::from_toml(&text).expect("parses");
    let e = hash_with(&a, &[]).expect_err("refused");
    let m = e.to_string();
    assert!(m.contains("POS_TYPO") && m.contains("airframe file"), "{m}");
}

#[test]
fn the_reference_airframe_sets_no_parameters() {
    assert!(reference().params.is_empty());
}
