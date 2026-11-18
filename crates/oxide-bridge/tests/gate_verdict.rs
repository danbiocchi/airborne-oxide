// SPDX-License-Identifier: AGPL-3.0-or-later
//! The verdict of gate 1 on tables written by hand: every rule of ADR-0007
//! sections 5 to 7 is shown to fail and to pass, and every table that is
//! incomplete or inconsistent is refused with a sentence. No binary needed.

#[path = "support/gate_fixture.rs"]
mod fx;

use fx::{AC, F405, Fixture, H743, TRUTH};
use oxide_bridge::batch::run_dir;
use oxide_bridge::gate::stats::median;
use oxide_bridge::gate::verdict::{
    EXIT_INCOMPLETE, EXIT_NOT_PASSED, EXIT_PASSED, Evaluation, Gate, Status, Verdict, evaluate,
    read_at, table_dir, twice_dir, verdict, verdict_at,
};

/// An edit of a good fixture.
type Edit = fn(&mut Fixture);

fn run(f: &Fixture) -> Verdict {
    f.write();
    let v = verdict_at(&f.dir, Some(&f.info.commit));
    f.remove();
    v
}

/// The evaluation; the tables must be complete.
fn eval(f: &Fixture) -> Evaluation {
    match run(f) {
        Verdict::Evaluated(e) => *e,
        Verdict::Incomplete(i) => panic!("the tables should be complete: {i:?}"),
    }
}

/// The tables read and checked, for a test that edits what was read.
fn read_gate(f: &Fixture) -> Gate {
    f.write();
    let g = read_at(&f.dir, Some(&f.info.commit));
    f.remove();
    match g {
        Ok(g) => g,
        Err(i) => panic!("the tables should be complete: {i:?}"),
    }
}

/// The problems; the tables must be refused.
fn refused(f: &Fixture) -> Vec<String> {
    match run(f) {
        Verdict::Incomplete(i) => i.0,
        Verdict::Evaluated(e) => panic!("the tables should be refused:\n{}", e.text()),
    }
}

fn mentions(problems: &[String], words: &[&str]) -> bool {
    problems.iter().any(|p| words.iter().all(|w| p.contains(w)))
}

fn set_rms(f: &mut Fixture, who: &str, value: impl Fn(u64) -> f64) {
    for i in 0..20 {
        f.row("gust-hold", who, i)
            .set("rms_h_m", &format!("{}", value(i)));
    }
}

fn set_class(f: &mut Fixture, scenario: &str, who: &str, index: u64, class: &str) {
    f.row(scenario, who, index).set("class", class);
}

fn set_last(f: &mut Fixture, scenario: &str, who: &str, lasts: &[f64]) {
    for (i, t) in lasts.iter().enumerate() {
        f.row(scenario, who, i as u64)
            .set("last_motor_on_s", &format!("{t}"));
    }
}

// ---- the good tables -------------------------------------------------------

#[test]
fn good_tables_of_a_trial_plan_hold_every_criterion_and_are_still_not_a_pass() {
    let e = eval(&Fixture::small("good-small"));
    for c in [&e.a1, &e.a2, &e.b1, &e.b2, &e.b3, &e.repeats] {
        assert_eq!(c.status, Status::Holds, "{c:?}");
    }
    assert!(!e.trial_reasons.is_empty());
    assert!(!e.passed, "a trial is not the gate");
    assert_eq!(e.exit_code(), EXIT_NOT_PASSED);
    assert!(e.text().contains("NOT PASSED"));
    assert!(e.text().contains("not the gate's plan"));
}

#[test]
fn the_gate_is_passed_on_good_tables_of_the_whole_plan() {
    let f = Fixture::full("good-full");
    let e = eval(&f);
    for c in [&e.a1, &e.a2, &e.b1, &e.b2, &e.b3, &e.repeats] {
        assert_eq!(c.status, Status::Holds, "{c:?}");
    }
    assert!(e.trial_reasons.is_empty(), "{:?}", e.trial_reasons);
    assert!(e.passed);
    assert_eq!(e.exit_code(), EXIT_PASSED);
    let text = e.text();
    assert!(text.contains("gate 1: PASSED"), "{text}");
    assert_eq!(e.twice.len(), 10);
    assert_eq!(e.gust.len(), 4);
    // 500 gust flights each for the four autopilots, ours lower everywhere.
    assert_eq!(e.gust[1].flights, 500);
    assert_eq!(e.wins.ours, 500);
}

#[test]
fn the_gate_is_not_passed_when_any_one_criterion_fails_on_the_whole_plan() {
    let breaks: [(&str, Edit); 5] = [
        ("A1", |f| set_class(f, "gust-hold", F405, 17, "crash")),
        ("A2", |f| {
            for i in 0..500 {
                f.row("gust-hold", F405, i).set("rms_h_m", "0.9");
            }
        }),
        ("B1", |f| set_class(f, "geofence", F405, 0, "crash")),
        ("B2", |f| {
            fx::set_verdict(f.row("battery-low", F405, 2), "fail")
        }),
        ("B3", |f| {
            f.row("motor-loss", F405, 4).set("last_motor_on_s", "40");
            f.row("motor-loss", F405, 3).set("last_motor_on_s", "40");
            f.row("motor-loss", F405, 2).set("last_motor_on_s", "40");
            f.row("motor-loss", F405, 1).set("last_motor_on_s", "40");
            f.row("motor-loss", F405, 0).set("last_motor_on_s", "40");
        }),
    ];
    for (name, apply) in breaks {
        let mut f = Fixture::full(&format!("break-{name}"));
        apply(&mut f);
        // A crash in B1/B2's scenarios must not be matched by ArduCopter's.
        let e = eval(&f);
        assert!(!e.passed, "{name}: {}", e.text());
        assert_eq!(e.exit_code(), EXIT_NOT_PASSED, "{name}");
        let failing: Vec<&str> = [
            ("A1", &e.a1),
            ("A2", &e.a2),
            ("B1", &e.b1),
            ("B2", &e.b2),
            ("B3", &e.b3),
            ("repeats", &e.repeats),
        ]
        .iter()
        .filter(|(_, c)| c.status == Status::Fails)
        .map(|(n, _)| *n)
        .collect();
        assert!(failing.contains(&name), "{name}: failing {failing:?}");
    }
}

#[test]
fn the_gate_is_not_passed_when_the_repeats_cannot_hold_on_the_whole_plan() {
    // A flight that does not repeat gives no verdict at all (see below); a
    // Gate whose flight flown twice is edited after it was read is not
    // passed either.
    let mut g = read_gate(&Fixture::full("repeats-whole"));
    g.twice.get_mut("imu-loss").expect("twice")[0].repeats = Some(false);
    let e = evaluate(&g);
    assert!(!e.passed, "{}", e.text());
    assert_eq!(e.repeats.status, Status::Fails);
    assert_eq!(e.exit_code(), EXIT_NOT_PASSED);
}

// ---- A1 --------------------------------------------------------------------

#[test]
fn a1_holds_when_every_f405_and_h743_flight_is_safe_and_passes() {
    let e = eval(&Fixture::small("a1-good"));
    assert_eq!(e.a1.status, Status::Holds);
    assert!(e.a1.failing.is_empty());
}

#[test]
fn a1_fails_for_one_f405_flight_that_is_a_crash() {
    let mut f = Fixture::small("a1-crash");
    set_class(&mut f, "gust-hold", F405, 3, "crash");
    let e = eval(&f);
    assert_eq!(e.a1.status, Status::Fails);
    assert!(mentions(
        &e.a1.failing,
        &["gust-hold core-f405 index 3", "crash"]
    ));
}

#[test]
fn a1_fails_for_one_f405_flight_that_is_a_fly_away() {
    let mut f = Fixture::small("a1-flyaway");
    set_class(&mut f, "gust-hold", F405, 4, "fly-away");
    assert_eq!(eval(&f).a1.status, Status::Fails);
}

#[test]
fn a1_fails_for_one_f405_flight_whose_verdict_is_fail() {
    let mut f = Fixture::small("a1-verdict");
    fx::set_verdict(f.row("gust-hold", F405, 5), "fail");
    let e = eval(&f);
    assert_eq!(e.a1.status, Status::Fails);
    assert!(mentions(
        &e.a1.failing,
        &["core-f405 index 5", "verdict fail"]
    ));
}

#[test]
fn a1_fails_for_a_flight_that_was_not_judged() {
    let mut f = Fixture::small("a1-notjudged");
    fx::set_verdict(f.row("gust-hold", F405, 6), "not-judged");
    assert_eq!(eval(&f).a1.status, Status::Fails);
}

#[test]
fn a1_fails_for_one_h743_flight_whose_verdict_is_fail() {
    let mut f = Fixture::small("a1-h743");
    fx::set_verdict(f.row("gust-hold", H743, 7), "fail");
    let e = eval(&f);
    assert_eq!(e.a1.status, Status::Fails);
    assert!(mentions(&e.a1.failing, &["core-h743 index 7"]));
}

#[test]
fn a1_fails_for_one_h743_flight_that_is_a_crash() {
    let mut f = Fixture::small("a1-h743-crash");
    set_class(&mut f, "gust-hold", H743, 1, "crash");
    assert_eq!(eval(&f).a1.status, Status::Fails);
}

#[test]
fn a1_does_not_read_arducopter_or_the_true_state_core() {
    let mut f = Fixture::small("a1-others");
    set_class(&mut f, "gust-hold", AC, 0, "crash");
    fx::set_verdict(f.row("gust-hold", AC, 1), "fail");
    set_class(&mut f, "gust-hold", TRUTH, 2, "crash");
    fx::set_verdict(f.row("gust-hold", TRUTH, 3), "fail");
    assert_eq!(eval(&f).a1.status, Status::Holds);
}

// ---- A2 --------------------------------------------------------------------

#[test]
fn a2_holds_when_ours_is_below_at_both_the_median_and_the_95th_percentile() {
    let e = eval(&Fixture::small("a2-good"));
    assert_eq!(e.a2.status, Status::Holds);
    assert_eq!(e.wins.ours, 20);
    assert_eq!(e.wins.arducopter, 0);
}

#[test]
fn a2_fails_when_ours_is_lower_at_the_median_and_higher_at_the_95th_percentile() {
    let mut f = Fixture::small("a2-tail");
    set_rms(&mut f, F405, |i| if i == 19 { 10.0 } else { 0.1 });
    set_rms(&mut f, AC, |_| 0.3);
    let e = eval(&f);
    assert_eq!(e.a2.status, Status::Fails, "{}", e.text());
    assert!(mentions(&e.a2.failing, &["95th percentile"]));
    assert!(!mentions(&e.a2.failing, &["median"]));
}

#[test]
fn a2_fails_when_ours_is_higher_at_the_median_and_lower_at_the_95th_percentile() {
    let mut f = Fixture::small("a2-median");
    set_rms(&mut f, F405, |_| 0.5);
    set_rms(&mut f, AC, |i| if i < 12 { 0.1 } else { 0.9 });
    let e = eval(&f);
    assert_eq!(e.a2.status, Status::Fails, "{}", e.text());
    assert!(mentions(&e.a2.failing, &["median"]));
    assert!(!mentions(&e.a2.failing, &["95th"]));
}

#[test]
fn a2_holds_when_both_are_equal_to_the_last_bit() {
    let mut f = Fixture::small("a2-equal");
    set_rms(&mut f, F405, |_| 0.123_456_789_123_4);
    set_rms(&mut f, AC, |_| 0.123_456_789_123_4);
    assert_eq!(eval(&f).a2.status, Status::Holds);
}

#[test]
fn a2_fails_for_a_difference_in_the_last_bit_against_us_no_rounding() {
    let x = 0.123_456_789_123_4_f64;
    let worse = f64::from_bits(x.to_bits() + 1);
    assert!(worse > x && (worse - x) < 1e-15);
    let mut f = Fixture::small("a2-ulp");
    set_rms(&mut f, F405, |_| worse);
    set_rms(&mut f, AC, |_| x);
    assert_eq!(eval(&f).a2.status, Status::Fails);
    // And the same difference in our favour holds.
    let mut f = Fixture::small("a2-ulp-ok");
    set_rms(&mut f, F405, |_| x);
    set_rms(&mut f, AC, |_| worse);
    assert_eq!(eval(&f).a2.status, Status::Holds);
}

#[test]
fn a2_uses_the_interpolated_percentile_not_the_nearest_rank() {
    // Ours: 19 flights of 0.1 and one of 10: the 95th percentile at position
    // 18.05 is 0.1 + 0.05 * 9.9 = 0.595; the nearest rank would say 0.1.
    // ArduCopter's: 18 of 0.1 and two of 0.3: 0.3 (the nearest rank also).
    let mut f = Fixture::small("a2-interpolation");
    set_rms(&mut f, F405, |i| if i == 19 { 10.0 } else { 0.1 });
    set_rms(&mut f, AC, |i| if i >= 18 { 0.3 } else { 0.1 });
    let e = eval(&f);
    let ours = e.gust[1].p95.expect("p95");
    assert!((ours - 0.595).abs() < 1e-12, "{ours}");
    assert_eq!(e.a2.status, Status::Fails);
}

#[test]
fn a2_leaves_arducopters_unsafe_flights_out_and_counts_them() {
    // Ten of ArduCopter's twenty flights crashed with a tiny error: with them
    // its median would be 0.105 and ours (0.15) would fail; without them it is
    // 0.2 and ours holds.
    let mut f = Fixture::small("a2-unsafe");
    set_rms(&mut f, F405, |_| 0.15);
    set_rms(&mut f, AC, |_| 0.2);
    for i in 0..10 {
        set_class(&mut f, "gust-hold", AC, i, "crash");
        f.row("gust-hold", AC, i).set("rms_h_m", "0.01");
    }
    let e = eval(&f);
    assert_eq!(e.a2.status, Status::Holds, "{}", e.text());
    assert_eq!(e.gust[0].unsafe_flights, 10);
    assert_eq!(e.gust[0].counted, 10);
    assert_eq!(e.gust[0].median, Some(0.2));
    assert!(mentions(&e.a2.lines, &["10 of 20", "not safe"]));
    // A fly-away is left out too.
    set_class(&mut f, "gust-hold", AC, 10, "fly-away");
    f.row("gust-hold", AC, 10).set("rms_h_m", "0.001");
    let e = eval(&f);
    assert_eq!(e.gust[0].unsafe_flights, 11);
    assert_eq!(e.gust[0].median, Some(0.2));
}

#[test]
fn a2_with_no_safe_arducopter_flight_cannot_hold() {
    let mut f = Fixture::small("a2-none");
    for i in 0..20 {
        set_class(&mut f, "gust-hold", AC, i, "crash");
    }
    let e = eval(&f);
    assert_eq!(e.a2.status, Status::Fails);
    assert!(mentions(&e.a2.failing, &["no safe gust flight"]));
}

#[test]
fn a2_compares_the_f405_core_and_no_other() {
    let mut f = Fixture::small("a2-who");
    set_rms(&mut f, H743, |_| 5.0);
    set_rms(&mut f, TRUTH, |_| 5.0);
    let e = eval(&f);
    assert_eq!(e.a2.status, Status::Holds);
    // They are reported all the same.
    assert_eq!(e.gust[2].median, Some(5.0));
    assert_eq!(e.gust[3].median, Some(5.0));
}

#[test]
fn the_win_counts_are_over_every_index() {
    let mut f = Fixture::small("a2-wins");
    set_rms(&mut f, F405, |i| {
        if i < 12 {
            0.1
        } else if i < 15 {
            0.3
        } else {
            0.2
        }
    });
    set_rms(&mut f, AC, |_| 0.2);
    let e = eval(&f);
    assert_eq!((e.wins.ours, e.wins.arducopter, e.wins.equal), (12, 3, 5));
    assert_eq!(e.paired.len(), 20);
}

// ---- B1 and B2 -------------------------------------------------------------

#[test]
fn b1_fails_when_we_have_a_crash_and_arducopter_has_none() {
    let mut f = Fixture::small("b1-fail");
    set_class(&mut f, "gps-loss", F405, 1, "crash");
    let e = eval(&f);
    assert_eq!(e.b1.status, Status::Fails);
    assert!(mentions(
        &e.b1.failing,
        &["gps-loss", "core-f405 has 1", "ArduCopter 0"]
    ));
}

#[test]
fn b1_holds_when_we_have_one_crash_and_so_has_arducopter_and_b2_then_fails() {
    let mut f = Fixture::small("b1-equal");
    set_class(&mut f, "gps-loss", F405, 1, "crash");
    set_class(&mut f, "gps-loss", AC, 0, "crash");
    let e = eval(&f);
    assert_eq!(e.b1.status, Status::Holds, "{}", e.text());
    assert_eq!(e.b2.status, Status::Fails);
    assert!(mentions(
        &e.b2.failing,
        &["gps-loss core-f405 index 1", "crash"]
    ));
}

#[test]
fn b1_holds_when_we_have_fewer_crashes_than_arducopter() {
    let mut f = Fixture::small("b1-fewer");
    set_class(&mut f, "gps-loss", AC, 0, "crash");
    set_class(&mut f, "gps-loss", AC, 1, "fly-away");
    assert_eq!(eval(&f).b1.status, Status::Holds);
}

#[test]
fn b1_counts_a_fly_away_as_a_crash() {
    let mut f = Fixture::small("b1-flyaway");
    set_class(&mut f, "gps-loss", F405, 0, "fly-away");
    assert_eq!(eval(&f).b1.status, Status::Fails);
}

#[test]
fn b2_fails_for_one_crash_one_fly_away_one_verdict_fail_each_alone() {
    // ArduCopter crashes everywhere, so that B1 holds and B2 stands alone.
    let each: [(&str, Edit); 3] = [
        ("crash", |f| set_class(f, "gps-loss", F405, 0, "crash")),
        ("fly-away", |f| {
            set_class(f, "gps-loss", F405, 1, "fly-away")
        }),
        ("verdict", |f| {
            fx::set_verdict(f.row("gps-loss", F405, 2), "fail")
        }),
    ];
    for (name, apply) in each {
        let mut f = Fixture::small(&format!("b2-{name}"));
        for i in 0..3 {
            set_class(&mut f, "gps-loss", AC, i, "crash");
        }
        let e = eval(&f);
        assert_eq!(e.b2.status, Status::Holds, "{name}: before");
        let mut f = Fixture::small(&format!("b2-{name}-2"));
        for i in 0..3 {
            set_class(&mut f, "gps-loss", AC, i, "crash");
        }
        apply(&mut f);
        let e = eval(&f);
        assert_eq!(e.b2.status, Status::Fails, "{name}");
        assert_eq!(
            e.b1.status,
            Status::Holds,
            "{name}: B1 is a count of classes"
        );
    }
}

#[test]
fn b2_fails_for_a_flight_that_was_not_judged() {
    let mut f = Fixture::small("b2-notjudged");
    fx::set_verdict(f.row("gps-loss", F405, 0), "not-judged");
    assert_eq!(eval(&f).b2.status, Status::Fails);
}

#[test]
fn b_reads_the_f405_core_and_leaves_the_h743_to_the_report() {
    let mut f = Fixture::small("b-h743");
    set_class(&mut f, "gps-loss", H743, 0, "crash");
    fx::set_verdict(f.row("gps-loss", H743, 1), "fail");
    let e = eval(&f);
    assert_eq!(e.b1.status, Status::Holds);
    assert_eq!(e.b2.status, Status::Holds);
    let s = e
        .scenarios
        .iter()
        .find(|s| s.scenario == "gps-loss")
        .expect("row");
    assert_eq!(s.core_h743.crash, 1);
    assert_eq!(s.core_h743.passed, 2);
}

#[test]
fn a_scenario_that_is_not_scored_decides_nothing() {
    let mut f = Fixture::small("b-notscored");
    set_class(&mut f, "hover-step", F405, 0, "crash");
    fx::set_verdict(f.row("hover-step", F405, 1), "fail");
    let e = eval(&f);
    assert_eq!(e.b1.status, Status::Holds);
    assert_eq!(e.b2.status, Status::Holds);
    // It is in the report all the same.
    assert!(e.scenarios.iter().any(|s| s.scenario == "hover-step"));
}

#[test]
fn a_scenario_scored_on_the_time_to_motors_off_is_not_scored_on_its_outcome() {
    let mut f = Fixture::small("b-motors-outcome");
    set_class(&mut f, "motor-loss", F405, 0, "crash");
    fx::set_verdict(f.row("motor-loss", F405, 1), "fail");
    let e = eval(&f);
    assert_eq!(e.b1.status, Status::Holds);
    assert_eq!(e.b2.status, Status::Holds);
}

// ---- B3 --------------------------------------------------------------------

#[test]
fn b3_takes_the_median_of_last_motor_on_less_the_failure_start() {
    let mut f = Fixture::small("b3-hand");
    set_last(&mut f, "motor-loss", F405, &[15.1, 15.2, 15.3, 15.4, 15.5]);
    // ArduCopter's motors never stop in two flights: 59.995 s is the flight's
    // end. Deltas 1, 2, 44.995, 44.995, 2.
    set_last(
        &mut f,
        "motor-loss",
        AC,
        &[16.0, 17.0, 59.995, 59.995, 17.0],
    );
    let e = eval(&f);
    assert_eq!(e.b3.status, Status::Holds, "{}", e.text());
    let m = &e.motors[0];
    assert_eq!(m.failure_start_s, 15.0);
    assert_eq!(median(&m.arducopter), Some(2.0));
    let ours = median(&m.core_f405).expect("median");
    assert!((ours - 0.3).abs() < 1e-9, "{ours}");
}

#[test]
fn b3_fails_when_ours_is_a_millisecond_slower() {
    let mut f = Fixture::small("b3-slower");
    set_last(&mut f, "motor-loss", F405, &[17.001; 5]);
    set_last(&mut f, "motor-loss", AC, &[17.0; 5]);
    let e = eval(&f);
    assert_eq!(e.b3.status, Status::Fails);
    assert!(mentions(
        &e.b3.failing,
        &["motor-loss", "above ArduCopter's"]
    ));
    let mut f = Fixture::small("b3-equal");
    set_last(&mut f, "motor-loss", F405, &[17.0; 5]);
    set_last(&mut f, "motor-loss", AC, &[17.0; 5]);
    assert_eq!(eval(&f).b3.status, Status::Holds);
}

#[test]
fn b3_is_the_median_not_the_mean() {
    // Ours: deltas 0.1 0.1 5 5 5 (median 5, mean 2.04). ArduCopter: 3 each.
    let mut f = Fixture::small("b3-mean");
    set_last(&mut f, "motor-loss", F405, &[15.1, 15.1, 20.0, 20.0, 20.0]);
    set_last(&mut f, "motor-loss", AC, &[18.0; 5]);
    assert_eq!(eval(&f).b3.status, Status::Fails);
    // And the reverse: a median below with a mean above holds.
    let mut f = Fixture::small("b3-mean-2");
    set_last(&mut f, "motor-loss", F405, &[15.1, 15.1, 15.1, 55.0, 55.0]);
    set_last(&mut f, "motor-loss", AC, &[18.0; 5]);
    assert_eq!(eval(&f).b3.status, Status::Holds);
}

#[test]
fn b3_counts_a_flight_whose_motors_never_stop_on_either_side() {
    // ArduCopter's: deltas 1, 3 and three flights that never stopped (44.995):
    // median 44.995. Ours 3 each holds; without those flights ArduCopter's
    // median would be 2.
    let mut f = Fixture::small("b3-never-ac");
    set_last(&mut f, "motor-loss", F405, &[18.0; 5]);
    set_last(
        &mut f,
        "motor-loss",
        AC,
        &[16.0, 18.0, 59.995, 59.995, 59.995],
    );
    assert_eq!(eval(&f).b3.status, Status::Holds);
    // Ours never stop in three flights: median 44.995 against 3 fails; without
    // them it would be 1.
    let mut f = Fixture::small("b3-never-ours");
    set_last(
        &mut f,
        "motor-loss",
        F405,
        &[16.0, 16.0, 59.995, 59.995, 59.995],
    );
    set_last(&mut f, "motor-loss", AC, &[18.0; 5]);
    assert_eq!(eval(&f).b3.status, Status::Fails);
}

#[test]
fn b3_does_not_read_the_h743_core() {
    let mut f = Fixture::small("b3-h743");
    set_last(&mut f, "motor-loss", H743, &[59.995; 5]);
    let e = eval(&f);
    assert_eq!(e.b3.status, Status::Holds);
    assert_eq!(median(&e.motors[0].core_h743), Some(44.995));
}

#[test]
fn a_flight_with_no_last_motor_on_cannot_be_timed() {
    let mut f = Fixture::small("b3-empty");
    f.row("motor-loss", AC, 2).set("last_motor_on_s", "");
    let p = refused(&f);
    assert!(
        mentions(&p, &["motor-loss arducopter index 2", "last_motor_on_s"]),
        "{p:?}"
    );
}

// ---- repeats ---------------------------------------------------------------

#[test]
fn the_repeats_hold_when_every_twice_flown_flight_repeats_and_matches() {
    let e = eval(&Fixture::small("rep-good"));
    assert_eq!(e.repeats.status, Status::Holds);
    assert_eq!(e.twice.len(), 3);
}

#[test]
fn one_repeats_no_fails() {
    // On a gate that was read: the criterion itself.
    let mut g = read_gate(&Fixture::small("rep-no"));
    g.twice.get_mut("gps-loss").expect("twice")[0].repeats = Some(false);
    let e = evaluate(&g);
    assert_eq!(e.repeats.status, Status::Fails);
    assert!(mentions(
        &e.repeats.failing,
        &["gps-loss arducopter index 0", "repeats is no"]
    ));
    // From the tables: no verdict, "the result waits until the cause is known"
    // (ADR-0007 section 7), not a failed gate.
    let mut f = Fixture::small("rep-no-tables");
    f.twice_flown("gps-loss", 0).row.set("repeats", "no");
    let p = refused(&f);
    assert!(
        mentions(
            &p,
            &[
                "gps-loss arducopter index 0",
                "waits until the cause is known"
            ]
        ),
        "{p:?}"
    );
}

#[test]
fn a_repeats_that_is_empty_does_not_count_as_yes() {
    let mut g = read_gate(&Fixture::small("rep-empty"));
    g.twice.get_mut("gust-hold").expect("twice")[1].repeats = None;
    assert_eq!(evaluate(&g).repeats.status, Status::Fails);
    let mut f = Fixture::small("rep-empty-tables");
    f.twice_flown("gust-hold", 1).row.set("repeats", "");
    assert!(mentions(
        &refused(&f),
        &["gust-hold arducopter index 1", "repeats"]
    ));
}

#[test]
fn a_twice_flown_flight_whose_hash_differs_from_the_main_table_fails_each_hash() {
    for column in ["trace_hash", "packet_hash", "reply_hash"] {
        // The criterion, on a gate that was read.
        let mut g = read_gate(&Fixture::small(&format!("rep-{column}")));
        {
            let f = &mut g.twice.get_mut("gust-hold").expect("twice")[1];
            match column {
                "trace_hash" => f.trace_hash = "00000000deadbeef".into(),
                "packet_hash" => f.packet_hash = "00000000deadbeef".into(),
                _ => f.reply_hash = "00000000deadbeef".into(),
            }
        }
        let e = evaluate(&g);
        assert_eq!(e.repeats.status, Status::Fails, "{column}");
        assert!(mentions(
            &e.repeats.failing,
            &["gust-hold arducopter index 1", "main table"]
        ));
        // From the tables: no verdict, the result waits.
        let mut f = Fixture::small(&format!("rep-{column}-tables"));
        // It says it repeated; the hash is not the main table's.
        f.twice_flown("gust-hold", 1)
            .row
            .set(column, "00000000deadbeef");
        let p = refused(&f);
        assert!(
            mentions(
                &p,
                &[
                    "gust-hold arducopter index 1",
                    "waits until the cause is known"
                ]
            ),
            "{column}: {p:?}"
        );
    }
    // The other way: the main table's hash is the one that differs.
    let mut f = Fixture::small("rep-main");
    f.row("gust-hold", AC, 0)
        .set("trace_hash", "00000000deadbeef");
    let p = refused(&f);
    assert!(
        mentions(&p, &["gust-hold arducopter index 0", "main table"]),
        "{p:?}"
    );
}

#[test]
fn a_twice_table_that_holds_copies_of_the_main_rows_is_not_a_repeat() {
    // The flight did not repeat (its second flight's hashes differ and its
    // audit says no); the table was then edited to say yes and to hold the
    // main table's hashes. Its own run directory still says what happened.
    let mut f = Fixture::small("rep-forged");
    let main = f.row("gps-loss", AC, 0).clone();
    {
        let t = f.twice_flown("gps-loss", 0);
        t.row.set("trace_hash", "0000000000000001");
        t.row.set("packet_hash", "0000000000000002");
        t.row.set("reply_hash", "0000000000000003");
        t.row.set("repeats", "no");
        t.audit_repeats = Some("no".into());
        t.second = Some((
            "0000000000000011".into(),
            "0000000000000012".into(),
            "0000000000000013".into(),
        ));
        t.own = Some(t.row.clone());
        for c in ["trace_hash", "packet_hash", "reply_hash"] {
            t.row.set(c, main.get(c));
        }
        t.row.set("repeats", "yes");
    }
    let p = refused(&f);
    assert!(
        mentions(&p, &["gps-loss arducopter index 0", "own run directory"]),
        "{p:?}"
    );
}

#[test]
fn a_flight_whose_second_flight_has_other_hashes_is_not_a_repeat_whatever_the_table_says() {
    // Table and run directory agree and say yes; the second flight's record
    // holds other hashes.
    for which in 0..3 {
        let mut f = Fixture::small(&format!("rep-second-{which}"));
        let t = f.twice_flown("gps-loss", 0);
        let (tr, pk, rp) = (
            t.row.get("trace_hash").to_string(),
            t.row.get("packet_hash").to_string(),
            t.row.get("reply_hash").to_string(),
        );
        let other = "00000000cafe0000".to_string();
        t.second = Some(match which {
            0 => (other, pk, rp),
            1 => (tr, other, rp),
            _ => (tr, pk, other),
        });
        let p = refused(&f);
        assert!(
            mentions(&p, &["gps-loss arducopter index 0", "second flight"]),
            "{which}: {p:?}"
        );
    }
}

#[test]
fn a_flight_flown_twice_whose_audit_does_not_say_it_repeated_is_not_a_repeat() {
    let mut f = Fixture::small("rep-audit");
    f.twice_flown("gps-loss", 0).audit_repeats = Some("no".into());
    let p = refused(&f);
    assert!(
        mentions(
            &p,
            &["gps-loss arducopter index 0", "its audit says repeats"]
        ),
        "{p:?}"
    );
}

#[test]
fn a_flight_flown_twice_without_the_record_of_its_second_flight_is_not_a_repeat() {
    let f = Fixture::small("rep-no-second");
    f.write();
    let second = oxide_bridge::batch::run_dir(&twice_dir(&f.dir, "gps-loss"), 0).join("second");
    std::fs::remove_dir_all(second).expect("remove");
    let v = verdict_at(&f.dir, Some(&f.info.commit));
    f.remove();
    assert_eq!(v.exit_code(), EXIT_INCOMPLETE);
    assert!(
        mentions(
            &incomplete(&v),
            &["gps-loss arducopter index 0", "second flight", "missing"]
        ),
        "{}",
        v.text()
    );
}

#[test]
fn empty_hashes_on_both_sides_are_not_the_same_flight() {
    let mut f = Fixture::small("rep-empty-hashes");
    f.row("gust-hold", AC, 1).set("trace_hash", "");
    f.twice_flown("gust-hold", 1).row.set("trace_hash", "");
    let p = refused(&f);
    assert!(
        mentions(
            &p,
            &[
                "gust-hold arducopter index 1",
                "same flight in the main table"
            ]
        ),
        "{p:?}"
    );
}

fn incomplete(v: &Verdict) -> Vec<String> {
    match v {
        Verdict::Incomplete(i) => i.0.clone(),
        Verdict::Evaluated(e) => panic!("should be refused:\n{}", e.text()),
    }
}

// ---- completeness ----------------------------------------------------------

#[test]
fn a_missing_row_is_exit_2_and_names_the_row() {
    let mut f = Fixture::small("miss-row");
    f.tables
        .get_mut(&("gust-hold".to_string(), F405.to_string()))
        .expect("table")
        .retain(|r| r.row.get("index") != "3");
    f.write();
    let v = verdict_at(&f.dir, Some(&f.info.commit));
    f.remove();
    assert_eq!(v.exit_code(), EXIT_INCOMPLETE);
    let text = v.text();
    assert!(text.contains("gust-hold core-f405 index 3"), "{text}");
    assert!(
        !text.contains("PASSED") || text.contains("no verdict"),
        "{text}"
    );
    assert!(text.contains("no verdict"));
}

#[test]
fn a_missing_row_does_not_pass_a_criterion_it_would_have_failed() {
    // The core's crash row is missing instead of present: no pass.
    let mut f = Fixture::full("miss-crash");
    f.tables
        .get_mut(&("geofence".to_string(), F405.to_string()))
        .expect("table")
        .pop();
    f.write();
    let v = verdict_at(&f.dir, Some(&f.info.commit));
    f.remove();
    assert_eq!(v.exit_code(), EXIT_INCOMPLETE);
}

#[test]
fn a_doubled_row_is_exit_2_and_names_the_row() {
    let mut f = Fixture::small("double-row");
    let t = f
        .tables
        .get_mut(&("gps-loss".to_string(), F405.to_string()))
        .expect("table");
    let again = t[1].clone();
    t.push(again);
    let p = refused(&f);
    assert!(
        mentions(&p, &["gps-loss core-f405 index 1", "twice"]),
        "{p:?}"
    );
}

#[test]
fn a_row_with_another_seed_is_exit_2_and_names_the_row() {
    let mut f = Fixture::small("seed");
    f.row("gust-hold", H743, 9).set("seed", "12345");
    let p = refused(&f);
    assert!(
        mentions(&p, &["gust-hold core-h743 index 9", "seed"]),
        "{p:?}"
    );
}

#[test]
fn a_seed_that_every_autopilot_shares_is_still_the_seed_of_the_scenario_and_index() {
    let mut f = Fixture::small("seed-all");
    for who in [AC, F405, H743, TRUTH] {
        f.row("gust-hold", who, 5).set("seed", "777");
    }
    let p = refused(&f);
    assert!(
        mentions(&p, &["gust-hold core-f405 index 5", "seed"]),
        "{p:?}"
    );
    assert!(
        mentions(&p, &["gust-hold arducopter index 5", "seed"]),
        "{p:?}"
    );
}

#[test]
fn a_table_from_another_commit_is_exit_2() {
    let mut f = Fixture::small("commit");
    f.flown("gps-loss", F405, 2).commit = "f".repeat(40);
    let p = refused(&f);
    assert!(
        mentions(&p, &["gps-loss core-f405 index 2", "commit"]),
        "{p:?}"
    );
}

#[test]
fn a_row_flown_on_a_dirty_tree_is_exit_2_by_its_note_or_by_its_audit() {
    let mut f = Fixture::small("dirty-note");
    f.row("gust-hold", AC, 4)
        .set("note", "first problem: x; dirty tree");
    let p = refused(&f);
    assert!(
        mentions(&p, &["gust-hold arducopter index 4", "dirty tree"]),
        "{p:?}"
    );
    let mut f = Fixture::small("dirty-audit");
    f.flown("gust-hold", F405, 4).commit = format!("{}-dirty", fx::commit());
    let p = refused(&f);
    assert!(
        mentions(&p, &["gust-hold core-f405 index 4", "dirty tree"]),
        "{p:?}"
    );
}

#[test]
fn a_gate_flown_on_a_dirty_tree_is_exit_2_whatever_the_rows_say() {
    let mut f = Fixture::small("dirty-info");
    f.info.tree_dirty = true;
    let p = refused(&f);
    assert!(mentions(&p, &["dirty tree"]), "{p:?}");
}

#[test]
fn a_table_that_mixes_binaries_or_parameter_hashes_is_exit_2() {
    let mut f = Fixture::small("binary");
    f.flown("gps-loss", AC, 1).binary = "c".repeat(64);
    let p = refused(&f);
    assert!(
        mentions(&p, &["gps-loss arducopter index 1", "binary"]),
        "{p:?}"
    );
    let mut f = Fixture::small("params");
    f.flown("gps-loss", AC, 1).params = "another".into();
    let p = refused(&f);
    assert!(
        mentions(&p, &["gps-loss", "mixes parameter hashes"]),
        "{p:?}"
    );
    // The twice-flown flights have the parameters of their scenario's table.
    let mut f = Fixture::small("params-twice");
    f.twice_flown("gps-loss", 0).params = "another".into();
    let p = refused(&f);
    assert!(mentions(&p, &["gps-loss", "parameter hashes"]), "{p:?}");
}

#[test]
fn a_scenario_file_that_is_not_the_one_gate_txt_hashed_is_exit_2() {
    let mut f = Fixture::small("filesha");
    f.flown("gust-hold", F405, 0).file_sha = "e".repeat(64);
    let p = refused(&f);
    assert!(
        mentions(&p, &["gust-hold core-f405 index 0", "SHA-256"]),
        "{p:?}"
    );
}

#[test]
fn a_row_that_did_not_start_means_the_gate_stopped_and_gives_no_verdict() {
    let mut f = Fixture::small("stopped");
    let r = f.row("gps-loss", AC, 1);
    *r = oxide_bridge::record::Row::not_started(
        "gps-loss",
        1,
        r.get("seed").parse().expect("seed"),
        &oxide_bridge::runner::Autopilot::ArduCopter,
        1.0,
        "the program did not arm",
    );
    let p = refused(&f);
    assert!(
        mentions(&p, &["gate stopped", "gps-loss arducopter index 1"]),
        "{p:?}"
    );
}

#[test]
fn a_missing_table_is_exit_2() {
    let f = Fixture::small("no-table");
    f.write();
    std::fs::remove_dir_all(table_dir(&f.dir, "gps-loss", F405)).expect("remove");
    let v = verdict_at(&f.dir, Some(&f.info.commit));
    f.remove();
    assert_eq!(v.exit_code(), EXIT_INCOMPLETE);
    assert!(
        v.text().contains("gps-loss flown by core-f405"),
        "{}",
        v.text()
    );
}

#[test]
fn a_row_outside_the_plan_or_of_another_scenario_is_exit_2() {
    let mut f = Fixture::small("extra");
    let t = f
        .tables
        .get_mut(&("gust-hold".to_string(), F405.to_string()))
        .expect("table");
    let mut extra = t[0].clone();
    extra.row.set("index", "20");
    t.push(extra);
    let p = refused(&f);
    assert!(mentions(&p, &["index 20", "not in the plan"]), "{p:?}");
    let mut f = Fixture::small("other-scenario");
    f.row("gps-loss", F405, 0).set("scenario", "baro-loss");
    let p = refused(&f);
    assert!(mentions(&p, &["baro-loss"]), "{p:?}");
}

#[test]
fn a_cell_that_is_not_a_number_is_exit_2() {
    let mut f = Fixture::small("nan");
    f.row("gust-hold", AC, 2).set("rms_h_m", "abc");
    let p = refused(&f);
    assert!(
        mentions(&p, &["gust-hold arducopter index 2", "rms_h_m"]),
        "{p:?}"
    );
    let mut f = Fixture::small("nan2");
    f.row("gust-hold", AC, 2).set("rms_h_m", "NaN");
    assert!(mentions(&refused(&f), &["rms_h_m"]));
    let mut f = Fixture::small("noclass");
    f.row("gps-loss", AC, 0).set("class", "");
    assert!(mentions(&refused(&f), &["class"]));
}

#[test]
fn a_gate_directory_without_gate_txt_is_exit_2() {
    let dir = std::env::temp_dir().join(format!("oxide-gate-empty-{}", std::process::id()));
    std::fs::create_dir_all(&dir).expect("dir");
    let v = verdict(&dir);
    assert_eq!(v.exit_code(), EXIT_INCOMPLETE);
    assert!(v.text().contains("gate.txt"), "{}", v.text());
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn a_plan_that_claims_to_be_the_gates_and_lacks_a_scenario_is_exit_2() {
    let mut f = Fixture::full("claims-full");
    f.info.plan.entries.retain(|e| e.scenario != "mag-loss");
    f.tables.retain(|(s, _), _| s != "mag-loss");
    let p = refused(&f);
    assert!(mentions(&p, &["mag-loss"]), "{p:?}");
}

#[test]
fn a_plan_that_says_it_is_reduced_is_never_passed_even_when_it_is_complete() {
    let mut f = Fixture::full("says-reduced");
    f.info.reduced = true;
    let e = eval(&f);
    assert!(!e.passed);
    assert_eq!(e.exit_code(), EXIT_NOT_PASSED);
}

#[test]
fn a_plan_that_scores_a_scenario_otherwise_than_the_record_is_exit_2() {
    let mut f = Fixture::small("mis-scored");
    for e in &mut f.info.plan.entries {
        if e.scenario == "gps-loss" {
            e.scoring = oxide_bridge::gate::plan::Scoring::NotScored;
        }
    }
    let p = refused(&f);
    assert!(mentions(&p, &["gps-loss", "scored"]), "{p:?}");
}

// ---- the record's whole plan -----------------------------------------------

/// Takes the scenario out of a fixture of the whole plan: gate.txt's plan and
/// scenario files, the tables, and the flights flown twice.
fn drop_scenario(f: &mut Fixture, name: &str) {
    f.info.plan.entries.retain(|e| e.scenario != name);
    f.info.plan.twice.retain(|(n, _)| n != name);
    f.info.plan.pair_kinds.remove(name);
    f.info.scenario_files.retain(|(n, _)| n != name);
    f.tables.retain(|(s, _), _| s != name);
    f.twice.remove(name);
}

#[test]
fn a_pair_scenario_left_out_of_the_whole_plan_is_exit_2() {
    let mut f = Fixture::full("pair-left-out");
    drop_scenario(&mut f, "pair-rc-loss+geofence");
    let p = refused(&f);
    assert!(
        mentions(&p, &["pair-rc-loss+geofence", "not in the plan"]),
        "{p:?}"
    );
}

#[test]
fn every_pair_that_is_not_flown_twice_left_out_is_exit_2() {
    let mut f = Fixture::full("pairs-left-out");
    let pairs: Vec<String> = oxide_bridge::gate::plan::PAIR_NAMES
        .iter()
        .filter(|n| **n != "pair-gps-loss+baro-loss")
        .map(|n| (*n).to_string())
        .collect();
    assert_eq!(pairs.len(), 20);
    for n in &pairs {
        drop_scenario(&mut f, n);
    }
    let p = refused(&f);
    for n in &pairs {
        assert!(mentions(&p, &[n.as_str(), "not in the plan"]), "{n}: {p:?}");
    }
}

#[test]
fn a_scenario_the_record_does_not_name_is_exit_2_on_the_whole_plan() {
    let mut f = Fixture::full("extra-scenario");
    let mut e = f
        .info
        .plan
        .entries
        .iter()
        .find(|e| e.scenario == "gps-loss")
        .expect("entry")
        .clone();
    e.scenario = "pair-gps-loss+mag-loss".into();
    f.info.plan.entries.push(e);
    let p = refused(&f);
    assert!(
        mentions(&p, &["pair-gps-loss+mag-loss", "not in the record"]),
        "{p:?}"
    );
}

// ---- the files are the commit's --------------------------------------------

/// Gives every flight of the scenario's tables (and twice-flown flights) the
/// scenario file hash `sha`, in the audits and in gate.txt.
fn set_file_sha(f: &mut Fixture, scenario: &str, sha: &str) {
    for ((s, _), flights) in &mut f.tables {
        if s == scenario {
            for fl in flights {
                fl.file_sha = sha.to_string();
            }
        }
    }
    for fl in f.twice.get_mut(scenario).into_iter().flatten() {
        fl.file_sha = sha.to_string();
    }
    for (n, h) in &mut f.info.scenario_files {
        if n == scenario {
            *h = sha.to_string();
        }
    }
}

#[test]
fn a_gate_flown_from_an_edited_scenario_file_is_exit_2_on_the_whole_plan_and_a_trial_otherwise() {
    // gust-hold in calm air: every audit and gate.txt agree with each other
    // and none with the commit's file.
    let mut f = Fixture::full("edited-scenario");
    set_file_sha(&mut f, "gust-hold", &"7".repeat(64));
    let p = refused(&f);
    assert!(
        mentions(&p, &["gust-hold", "SHA-256", "the commit's"]),
        "{p:?}"
    );
    // A reduced gate keeps its verdict and says why it is not a pass.
    let mut f = Fixture::full("edited-scenario-reduced");
    set_file_sha(&mut f, "gust-hold", &"7".repeat(64));
    f.info.reduced = true;
    let e = eval(&f);
    assert!(!e.passed);
    assert_eq!(e.exit_code(), EXIT_NOT_PASSED);
    assert!(
        mentions(&e.trial_reasons, &["gust-hold", "the commit's"]),
        "{:?}",
        e.trial_reasons
    );
}

#[test]
fn a_gate_flown_with_other_parameter_files_is_exit_2() {
    // gate.txt's hash of a file differs from the program's.
    let mut f = Fixture::full("param-file");
    f.info.params[0].1 = "9".repeat(64);
    let p = refused(&f);
    assert!(
        mentions(&p, &["parameter files", "not the commit's"]),
        "{p:?}"
    );
    // A file missing from the list.
    let mut f = Fixture::full("param-file-missing");
    f.info.params.pop();
    assert!(mentions(&refused(&f), &["parameter files"]));
}

#[test]
fn a_table_whose_every_flight_has_another_parameter_hash_is_exit_2() {
    // Not one table that mixes them: every audit of the scenario, and of its
    // flights flown twice, agree on a set that is not the commit's.
    for scenario in ["gps-loss", "gust-hold"] {
        let mut f = Fixture::full(&format!("params-{scenario}"));
        for ((s, a), flights) in &mut f.tables {
            if s == scenario && a == AC {
                for fl in flights {
                    fl.params = "9".repeat(64);
                }
            }
        }
        for fl in f.twice.get_mut(scenario).into_iter().flatten() {
            fl.params = "9".repeat(64);
        }
        let p = refused(&f);
        assert!(
            mentions(
                &p,
                &[scenario, "parameter hash", "commit's parameter files"]
            ),
            "{scenario}: {p:?}"
        );
    }
}

#[test]
fn a_binary_that_is_not_the_pinned_one_is_exit_2_even_when_gate_txt_and_every_audit_agree() {
    let mut f = Fixture::full("binary-pinned");
    let other = "d".repeat(64);
    f.info.binary_sha256 = Some(other.clone());
    for flights in f.tables.values_mut().chain(f.twice.values_mut()) {
        for fl in flights {
            fl.binary = other.clone();
        }
    }
    let p = refused(&f);
    assert!(mentions(&p, &["simulator program", "pinned"]), "{p:?}");
}

// ---- the table is the flight's own record ----------------------------------

#[test]
fn a_class_edited_in_the_table_against_the_flights_own_record_is_exit_2() {
    let mut f = Fixture::full("class-edit");
    {
        let fl = f.flown("gps-loss", F405, 5);
        fl.row.set("class", "crash");
        fl.own = Some(fl.row.clone());
        fl.row.set("class", "safe");
    }
    let p = refused(&f);
    assert!(
        mentions(
            &p,
            &["gps-loss core-f405 index 5", "own run directory", "class"]
        ),
        "{p:?}"
    );
}

#[test]
fn a_halved_rms_in_the_table_alone_is_exit_2() {
    let mut f = Fixture::full("rms-halved");
    for i in 0..500 {
        let fl = f.flown("gust-hold", F405, i);
        let v: f64 = fl.row.get("rms_h_m").parse().expect("number");
        fl.edit_table_only("rms_h_m", &format!("{}", v / 2.0));
    }
    let p = refused(&f);
    assert!(
        mentions(
            &p,
            &[
                "gust-hold core-f405 index 0",
                "own run directory",
                "rms_h_m"
            ]
        ),
        "{p:?}"
    );
}

#[test]
fn a_table_row_and_audit_that_disagree_on_class_seed_or_hash_is_exit_2() {
    // The audit is the flight's: a table row that says another class, or a
    // reply hash the audit does not have, is not that flight's row.
    let mut f = Fixture::small("audit-class");
    f.flown("gps-loss", F405, 1).own = None;
    {
        let fl = f.flown("gps-loss", AC, 1);
        fl.own = Some(fl.row.clone());
        fl.row.set("packet_hash", "00000000000000aa");
    }
    let p = refused(&f);
    assert!(
        mentions(&p, &["gps-loss arducopter index 1", "packet_hash"]),
        "{p:?}"
    );
}

#[test]
fn a_negative_error_time_or_tilt_is_malformed_and_exit_2() {
    for (scenario, who, column) in [
        ("gust-hold", F405, "rms_h_m"),
        ("gust-hold", AC, "rms_h_m"),
        ("gust-hold", F405, "rms_v_m"),
        ("gust-hold", F405, "max_tilt_deg"),
        ("motor-loss", F405, "last_motor_on_s"),
    ] {
        let mut f = Fixture::small(&format!("negative-{column}"));
        let fl = f.flown(scenario, who, 2);
        fl.row.set(column, "-0.5");
        fl.own = None;
        let p = refused(&f);
        assert!(
            mentions(&p, &[scenario, column, "negative"]),
            "{scenario} {who} {column}: {p:?}"
        );
    }
}

#[test]
fn an_unsafe_arducopter_gust_flight_needs_no_rms_but_a_safe_one_and_ours_do() {
    let mut f = Fixture::small("rms-unsafe");
    set_class(&mut f, "gust-hold", AC, 4, "crash");
    f.row("gust-hold", AC, 4).set("rms_h_m", "");
    let e = eval(&f);
    assert_eq!(e.gust[0].unsafe_flights, 1);
    assert_eq!(e.gust[0].counted, 19);
    let mut f = Fixture::small("rms-safe");
    f.row("gust-hold", AC, 4).set("rms_h_m", "");
    assert!(mentions(
        &refused(&f),
        &["gust-hold arducopter index 4", "rms_h_m"]
    ));
    let mut f = Fixture::small("rms-ours");
    set_class(&mut f, "gust-hold", F405, 4, "crash");
    f.row("gust-hold", F405, 4).set("rms_h_m", "");
    assert!(mentions(
        &refused(&f),
        &["gust-hold core-f405 index 4", "rms_h_m"]
    ));
}

// ---- directories the plan does not name ------------------------------------

#[test]
fn a_directory_the_plan_does_not_name_is_exit_2() {
    for (name, dir, word) in [
        (
            "extra-scenario",
            "extra-scenario/arducopter",
            "extra-scenario",
        ),
        (
            "extra-autopilot",
            "gust-hold/core-f405-more",
            "core-f405-more",
        ),
        ("extra-twice", "twice/mag-loss", "mag-loss"),
        ("extra-truth", "gps-loss/core-f405-truth", "core-f405-truth"),
    ] {
        let f = Fixture::small(name);
        f.write();
        std::fs::create_dir_all(f.dir.join(dir)).expect("dir");
        std::fs::write(f.dir.join(dir).join("flights.csv"), "x\n").expect("table");
        let v = verdict_at(&f.dir, Some(&f.info.commit));
        f.remove();
        assert_eq!(v.exit_code(), EXIT_INCOMPLETE, "{dir}: {}", v.text());
        assert!(
            mentions(&incomplete(&v), &[word, "plan"]),
            "{dir}: {}",
            v.text()
        );
    }
    // A directory with no table in it (the reports of `gate verdict --report`
    // go into one when the person says so) is not a table of another plan.
    let f = Fixture::small("report-dir");
    f.write();
    std::fs::create_dir_all(f.dir.join("report/figures")).expect("dir");
    std::fs::write(f.dir.join("report/gate1.md"), "report\n").expect("report");
    let v = verdict_at(&f.dir, Some(&f.info.commit));
    f.remove();
    assert!(matches!(v, Verdict::Evaluated(_)), "{}", v.text());
    // A table a continued gate moved aside is its own leftover.
    let f = Fixture::small("interrupted");
    f.write();
    std::fs::create_dir_all(f.dir.join("gps-loss/arducopter.interrupted")).expect("dir");
    let v = verdict_at(&f.dir, Some(&f.info.commit));
    f.remove();
    assert!(matches!(v, Verdict::Evaluated(_)), "{}", v.text());
}

// ---- the rules that no other test held -------------------------------------

#[test]
fn the_seed_of_a_twice_flown_row_is_checked_like_any_other() {
    let mut f = Fixture::small("seed-twice");
    f.twice_flown("gps-loss", 0).row.set("seed", "12345");
    let p = refused(&f);
    assert!(
        mentions(
            &p,
            &["gps-loss arducopter index 0 (flown twice)", "its seed"]
        ),
        "{p:?}"
    );
}

#[test]
fn the_binary_of_a_twice_flown_flight_is_checked_like_any_other() {
    let mut f = Fixture::small("binary-twice");
    f.twice_flown("gps-loss", 0).binary = "c".repeat(64);
    let p = refused(&f);
    assert!(
        mentions(&p, &["gps-loss arducopter index 0 (flown twice)", "binary"]),
        "{p:?}"
    );
}

#[test]
fn a_started_cell_that_is_not_yes_is_exit_2_whatever_it_says() {
    for word in ["", "maybe", "YES", "true"] {
        let mut f = Fixture::small("started-word");
        f.row("gps-loss", AC, 1).set("started", word);
        f.flown("gps-loss", AC, 1).own = None;
        let p = refused(&f);
        assert!(
            mentions(&p, &["gps-loss arducopter index 1", "did not start"]),
            "{word:?}: {p:?}"
        );
    }
}

#[test]
fn arducopters_fly_aways_are_left_out_of_its_statistics_like_its_crashes() {
    // Ten of twenty fly away with a tiny error: counted, ArduCopter's median
    // would be 0.105 and ours (0.15) would fail; left out it is 0.2.
    let mut f = Fixture::small("a2-flyaways");
    set_rms(&mut f, F405, |_| 0.15);
    set_rms(&mut f, AC, |_| 0.2);
    for i in 0..10 {
        set_class(&mut f, "gust-hold", AC, i, "fly-away");
        f.row("gust-hold", AC, i).set("rms_h_m", "0.01");
    }
    let e = eval(&f);
    assert_eq!(e.a2.status, Status::Holds, "{}", e.text());
    assert_eq!(e.gust[0].median, Some(0.2));
    assert!(mentions(&e.a2.lines, &["10 of 20", "not safe"]));
}

#[test]
fn a_gate_that_says_it_stopped_gives_no_verdict_whatever_the_tables_hold() {
    let mut f = Fixture::small("stopped-line");
    f.info.stopped = Some("gps-loss/arducopter run 2 failed twice".into());
    let p = refused(&f);
    assert!(mentions(&p, &["the gate stopped", "failed twice"]), "{p:?}");
}

#[test]
fn tables_of_another_commit_than_the_programs_own_give_no_verdict() {
    // ADR-0007 section 7: "no verdict on tables flown ... at another commit
    // than its own". A program not built at a commit has none to compare.
    let f = Fixture::small("own-commit");
    f.write();
    let own = fx::commit();
    let other = "f".repeat(40);
    let same = verdict_at(&f.dir, Some(&own));
    let none = verdict_at(&f.dir, None);
    let differ = verdict_at(&f.dir, Some(&other));
    let read_differ = read_at(&f.dir, Some(&other));
    f.remove();
    assert!(matches!(same, Verdict::Evaluated(_)), "{}", same.text());
    // A program that is no commit's gives no verdict, a pass or not.
    assert_eq!(none.exit_code(), EXIT_INCOMPLETE, "{}", none.text());
    assert!(
        mentions(&incomplete(&none), &["not built at a commit"]),
        "{}",
        none.text()
    );
    assert!(!none.text().contains("PASSED"));
    assert_eq!(differ.exit_code(), EXIT_INCOMPLETE, "{}", differ.text());
    let p = read_differ.expect_err("refused").0;
    assert!(mentions(&p, &["flown at commit", &other]), "{p:?}");
}

#[test]
fn what_gate_txt_says_of_a_pair_or_a_failure_start_is_the_commits_files_or_exit_2() {
    // The failure start decides B3 (the times are counted from it) and what a
    // pair holds decides how it is scored: neither is taken on trust.
    let mut f = Fixture::full("failure-start");
    f.info.plan.failure_starts.insert("motor-loss".into(), 10.0);
    let p = refused(&f);
    assert!(
        mentions(&p, &["motor-loss", "starts at", "the commit's file"]),
        "{p:?}"
    );
    let mut f = Fixture::full("pair-kinds");
    f.info.plan.pair_kinds.insert(
        "pair-gps-loss+baro-loss".into(),
        vec!["geofence".into(), "gps-loss".into()],
    );
    let p = refused(&f);
    assert!(
        mentions(&p, &["pair-gps-loss+baro-loss", "the commit's file holds"]),
        "{p:?}"
    );
    // The same gate.txt as a trial is a trial, not a pass.
    let mut f = Fixture::full("failure-start-trial");
    f.info.plan.failure_starts.insert("motor-loss".into(), 10.0);
    f.info.reduced = true;
    let e = eval(&f);
    assert!(!e.passed, "a trial is never a pass");
}

#[test]
fn a_table_with_other_line_ends_or_an_unended_last_line_is_exit_2() {
    // Not forgeries (the values are the genuine ones) but not what `gate fly`
    // wrote either: refused rather than passed.
    for (tag, edit) in [
        (
            "crlf",
            (|t: &str| t.replace('\n', "\r\n")) as fn(&str) -> String,
        ),
        ("no-final-newline", |t: &str| {
            t.trim_end_matches('\n').to_string()
        }),
    ] {
        let f = Fixture::small(tag);
        f.write();
        let path = table_dir(&f.dir, "gps-loss", F405).join("flights.csv");
        let text = std::fs::read_to_string(&path).expect("table");
        std::fs::write(&path, edit(&text)).expect("write");
        let v = verdict_at(&f.dir, Some(&f.info.commit));
        f.remove();
        let Verdict::Incomplete(i) = v else {
            panic!("{tag}: the table should be refused");
        };
        assert!(
            mentions(&i.0, &["gps-loss", "core-f405", "line feed"]),
            "{tag}: {i:?}"
        );
    }
}

// ---- the row against the flight's own audit --------------------------------

/// The problems of tables in which one cell of one flight was edited in the
/// table and in its `row.csv` together and its audit was left as the flight
/// wrote it.
fn forged(tag: &str, scenario: &str, who: &str, index: u64, cells: &[(&str, &str)]) -> Vec<String> {
    let mut f = Fixture::small(tag);
    for (column, value) in cells {
        f.flown(scenario, who, index)
            .edit_table_and_row_only(column, value);
    }
    refused(&f)
}

#[test]
fn a_number_that_decides_a_criterion_edited_in_the_table_and_row_csv_but_not_in_the_audit_is_exit_2()
 {
    // The value is in three records of the flight; two of them agreeing is not
    // enough (ADR-0007 section 7: every row equals the flight's own record).
    let cases: [(&str, &str, u64, &str, &str); 8] = [
        ("gust-hold", F405, 3, "rms_h_m", "0.01"),
        ("gust-hold", F405, 3, "rms_v_m", "0.001"),
        ("gust-hold", F405, 3, "max_tilt_deg", "1.5"),
        ("gust-hold", F405, 3, "max_h_m", "0.1"),
        ("motor-loss", AC, 1, "last_motor_on_s", "15.05"),
        ("motor-loss", F405, 1, "last_motor_on_s", "40"),
        ("gps-loss", F405, 1, "touchdown_mps", "0.1"),
        ("gps-loss", F405, 1, "disarmed_at_s", "3"),
    ];
    for (scenario, who, index, column, value) in cases {
        let p = forged("audit-number", scenario, who, index, &[(column, value)]);
        assert!(
            mentions(
                &p,
                &[
                    &format!("{scenario} {who} index {index}"),
                    "its audit says",
                    column
                ]
            ),
            "{scenario} {who} {column}: {p:?}"
        );
    }
}

#[test]
fn a_verdict_edited_in_the_table_and_row_csv_but_not_in_the_audit_is_exit_2() {
    // The word with the audit's counts, and the counts with the word.
    let p = forged(
        "audit-verdict",
        "gps-loss",
        F405,
        1,
        &[("verdict", "fail"), ("violations", "1")],
    );
    assert!(
        mentions(&p, &["its audit says 0 violations", "its row says 1"]),
        "{p:?}"
    );
    // The word alone says something its own counts do not.
    let p = forged("audit-word", "gps-loss", F405, 1, &[("verdict", "fail")]);
    assert!(mentions(&p, &["verdict \"fail\"", "\"pass\""]), "{p:?}");
    let p = forged(
        "audit-notjudged",
        "gps-loss",
        F405,
        1,
        &[("verdict", "not-judged"), ("not_judged", "2")],
    );
    assert!(mentions(&p, &["its audit says", "not judged"]), "{p:?}");
}

#[test]
fn a_class_or_a_hash_edited_in_the_table_and_row_csv_but_not_in_the_audit_is_exit_2() {
    let p = forged("audit-class", "gps-loss", F405, 0, &[("class", "crash")]);
    assert!(
        mentions(&p, &["its audit says class \"safe\"", "\"crash\""]),
        "{p:?}"
    );
    let p = forged(
        "audit-packet",
        "gps-loss",
        AC,
        0,
        &[("packet_hash", "0123456789abcdef")],
    );
    assert!(mentions(&p, &["its audit says packet hash"]), "{p:?}");
    let p = forged(
        "audit-reply",
        "gps-loss",
        AC,
        0,
        &[("reply_hash", "0123456789abcdef")],
    );
    assert!(mentions(&p, &["its audit says reply hash"]), "{p:?}");
}

#[test]
fn a_note_edited_in_the_table_only_is_exit_2_unless_the_flight_was_retried() {
    let mut f = Fixture::small("note-differs");
    f.flown("gps-loss", F405, 0)
        .edit_table_only("note", "another note");
    let p = refused(&f);
    assert!(
        mentions(&p, &["gps-loss core-f405 index 0", "note differ"]),
        "{p:?}"
    );
    // A flight flown again has the batch's own retried and note.
    let mut f = Fixture::small("note-retried");
    let flown = f.flown("gps-loss", F405, 0);
    flown.edit_table_only("retried", "yes");
    flown.edit_table_only("note", "retried after: first attempt failed");
    assert_eq!(eval(&f).a1.status, Status::Holds);
    // But not another cell on the way.
    let mut f = Fixture::small("note-retried-other");
    let flown = f.flown("gps-loss", F405, 0);
    flown.edit_table_only("retried", "yes");
    flown.edit_table_only("class", "crash");
    let p = refused(&f);
    assert!(
        mentions(&p, &["gps-loss core-f405 index 0", "class differ"]),
        "{p:?}"
    );
}

#[test]
fn a_negative_error_or_time_or_a_word_in_any_number_column_is_exit_2() {
    let cases: [(&str, &str, u64, &str, &str); 8] = [
        ("gust-hold", F405, 7, "max_h_m", "-0.3"),
        ("gps-loss", F405, 1, "disarmed_at_s", "-4.0"),
        ("gps-loss", F405, 1, "max_h_m", "abc"),
        ("gps-loss", F405, 1, "touchdown_mps", "-0.5"),
        ("gps-loss", F405, 1, "max_height_m", "NaN"),
        ("gps-loss", F405, 1, "max_distance_m", "-1"),
        ("gps-loss", F405, 1, "wall_s", "-2"),
        ("gps-loss", F405, 1, "end_n_m", "inf"),
    ];
    for (scenario, who, index, column, value) in cases {
        let mut f = Fixture::small("bad-number");
        f.row(scenario, who, index).set(column, value);
        let p = refused(&f);
        assert!(
            mentions(&p, &[&format!("{scenario} {who} index {index}"), column]),
            "{column} {value}: {p:?}"
        );
    }
    // A position may be negative.
    let mut f = Fixture::small("negative-position");
    f.row("gps-loss", F405, 1).set("end_n_m", "-3.5");
    assert_eq!(eval(&f).a1.status, Status::Holds);
}

/// Copies a directory with everything in it.
fn copy_dir(from: &std::path::Path, to: &std::path::Path) {
    std::fs::create_dir_all(to).expect("dir");
    for e in std::fs::read_dir(from).expect("read").flatten() {
        let (src, dst) = (e.path(), to.join(e.file_name()));
        if src.is_dir() {
            copy_dir(&src, &dst);
        } else {
            std::fs::copy(&src, &dst).expect("copy");
        }
    }
}

#[test]
fn flights_flown_twice_that_are_copies_of_the_main_flights_records_are_not_repeats() {
    // The hashes of a copy agree with the main flight's by construction; the
    // copy's audit still names the main flight's directory.
    let f = Fixture::small("twice-copies");
    f.write();
    for (scenario, count) in &f.info.plan.twice {
        for i in 0..*count {
            let main = run_dir(&table_dir(&f.dir, scenario, AC), i);
            let twice = run_dir(&twice_dir(&f.dir, scenario), i);
            std::fs::remove_dir_all(&twice).expect("remove");
            copy_dir(&main, &twice);
            let audit = std::fs::read_to_string(twice.join("audit.txt")).expect("audit");
            let audit = audit.replacen(
                "  class ",
                "  repeats         yes (trace, packet and reply hashes of two flights)\n  class ",
                1,
            );
            std::fs::write(twice.join("audit.txt"), audit).expect("audit");
            copy_dir(&main, &twice.join("second"));
        }
    }
    let v = verdict_at(&f.dir, Some(&f.info.commit));
    f.remove();
    assert_eq!(v.exit_code(), EXIT_INCOMPLETE, "{}", v.text());
    let p = incomplete(&v);
    assert!(
        mentions(
            &p,
            &[
                "gust-hold arducopter index 0 (flown twice)",
                "not the record of this flight"
            ]
        ),
        "{p:?}"
    );
    assert!(!v.text().contains("PASSED"));
}

#[test]
fn a_second_flight_whose_record_is_a_copy_of_another_is_not_a_repeat() {
    let f = Fixture::small("second-copy");
    f.write();
    let main = run_dir(&table_dir(&f.dir, "gps-loss", AC), 0);
    let twice = run_dir(&twice_dir(&f.dir, "gps-loss"), 0);
    std::fs::remove_dir_all(twice.join("second")).expect("remove");
    copy_dir(&main, &twice.join("second"));
    let v = verdict_at(&f.dir, Some(&f.info.commit));
    f.remove();
    assert_eq!(v.exit_code(), EXIT_INCOMPLETE, "{}", v.text());
    assert!(
        mentions(
            &incomplete(&v),
            &[
                "gps-loss arducopter index 0",
                "does not repeat",
                "command line"
            ]
        ),
        "{}",
        v.text()
    );
}

#[test]
fn a_gate_directory_that_was_moved_is_still_read() {
    // The audits name the directory they were flown in; only the end of the
    // path, below the gate directory, is compared.
    let f = Fixture::small("moved");
    f.write();
    let moved = f.dir.with_file_name(format!(
        "{}-moved",
        f.dir.file_name().expect("name").to_string_lossy()
    ));
    let _ = std::fs::remove_dir_all(&moved);
    std::fs::rename(&f.dir, &moved).expect("move");
    let v = verdict_at(&moved, Some(&f.info.commit));
    let _ = std::fs::remove_dir_all(&moved);
    assert!(matches!(v, Verdict::Evaluated(_)), "{}", v.text());
}

#[test]
fn hover_step_left_out_of_a_gate_that_says_it_is_the_whole_plan_is_exit_2() {
    let mut f = Fixture::full("no-hover-step");
    f.info.plan.entries.retain(|e| e.scenario != "hover-step");
    f.info.scenario_files.retain(|(n, _)| n != "hover-step");
    f.tables.retain(|(s, _), _| s != "hover-step");
    let p = refused(&f);
    assert!(mentions(&p, &["hover-step", "not in the plan"]), "{p:?}");
}

#[test]
fn a_command_line_that_only_ends_like_the_flights_own_directory_is_not_its_directory() {
    // `.../mytwice/gps-loss/...` ends with the characters of `twice/gps-loss/...`
    // and is not below a directory called twice.
    let f = Fixture::small("suffix-only");
    f.write();
    let run = run_dir(&twice_dir(&f.dir, "gps-loss"), 0);
    // The record names its directory as the platform prints it, with `\` on
    // Windows. The program reads either separator, so the line is edited with
    // `/` throughout.
    let audit = std::fs::read_to_string(run.join("audit.txt"))
        .expect("audit")
        .replace('\\', "/");
    let own = format!("/twice/gps-loss/{}/sitl/defaults.parm", "run-0000");
    assert!(audit.contains(&own), "{audit}");
    std::fs::write(
        run.join("audit.txt"),
        audit.replace(&own, "/mytwice/gps-loss/run-0000/sitl/defaults.parm"),
    )
    .expect("audit");
    let v = verdict_at(&f.dir, Some(&f.info.commit));
    f.remove();
    assert_eq!(v.exit_code(), EXIT_INCOMPLETE, "{}", v.text());
    assert!(
        mentions(
            &incomplete(&v),
            &["gps-loss arducopter index 0 (flown twice)", "command line"]
        ),
        "{}",
        v.text()
    );
}
