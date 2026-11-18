// SPDX-License-Identifier: AGPL-3.0-or-later
//! The verdict of gate 1 on tables written by hand: every rule of ADR-0007
//! sections 5 to 7 is shown to fail and to pass, and every table that is
//! incomplete or inconsistent is refused with a sentence. No binary needed.

#[path = "support/gate_fixture.rs"]
mod fx;

use fx::{AC, F405, Fixture, H743, TRUTH};
use oxide_bridge::gate::stats::median;
use oxide_bridge::gate::verdict::{
    EXIT_INCOMPLETE, EXIT_NOT_PASSED, EXIT_PASSED, Evaluation, Status, Verdict, table_dir, verdict,
};

/// An edit of a good fixture.
type Edit = fn(&mut Fixture);

fn run(f: &Fixture) -> Verdict {
    f.write();
    let v = verdict(&f.dir);
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
    let breaks: [(&str, Edit); 6] = [
        ("A1", |f| set_class(f, "gust-hold", F405, 17, "crash")),
        ("A2", |f| {
            for i in 0..500 {
                f.row("gust-hold", F405, i).set("rms_h_m", "0.9");
            }
        }),
        ("B1", |f| set_class(f, "geofence", F405, 0, "crash")),
        ("B2", |f| {
            f.row("battery-low", F405, 2).set("verdict", "fail")
        }),
        ("B3", |f| {
            f.row("motor-loss", F405, 4).set("last_motor_on_s", "40");
            f.row("motor-loss", F405, 3).set("last_motor_on_s", "40");
            f.row("motor-loss", F405, 2).set("last_motor_on_s", "40");
            f.row("motor-loss", F405, 1).set("last_motor_on_s", "40");
            f.row("motor-loss", F405, 0).set("last_motor_on_s", "40");
        }),
        ("repeats", |f| {
            f.twice_flown("imu-loss", 0).row.set("repeats", "no");
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
    f.row("gust-hold", F405, 5).set("verdict", "fail");
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
    f.row("gust-hold", F405, 6).set("verdict", "not-judged");
    assert_eq!(eval(&f).a1.status, Status::Fails);
}

#[test]
fn a1_fails_for_one_h743_flight_whose_verdict_is_fail() {
    let mut f = Fixture::small("a1-h743");
    f.row("gust-hold", H743, 7).set("verdict", "fail");
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
    f.row("gust-hold", AC, 1).set("verdict", "fail");
    set_class(&mut f, "gust-hold", TRUTH, 2, "crash");
    f.row("gust-hold", TRUTH, 3).set("verdict", "fail");
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
            f.row("gps-loss", F405, 2).set("verdict", "fail")
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
    f.row("gps-loss", F405, 0).set("verdict", "not-judged");
    assert_eq!(eval(&f).b2.status, Status::Fails);
}

#[test]
fn b_reads_the_f405_core_and_leaves_the_h743_to_the_report() {
    let mut f = Fixture::small("b-h743");
    set_class(&mut f, "gps-loss", H743, 0, "crash");
    f.row("gps-loss", H743, 1).set("verdict", "fail");
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
    f.row("hover-step", F405, 1).set("verdict", "fail");
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
    f.row("motor-loss", F405, 1).set("verdict", "fail");
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
    let mut f = Fixture::small("rep-no");
    f.twice_flown("gps-loss", 0).row.set("repeats", "no");
    let e = eval(&f);
    assert_eq!(e.repeats.status, Status::Fails);
    assert!(mentions(
        &e.repeats.failing,
        &["gps-loss arducopter index 0", "repeats is no"]
    ));
}

#[test]
fn a_repeats_that_is_empty_does_not_count_as_yes() {
    let mut f = Fixture::small("rep-empty");
    f.twice_flown("gust-hold", 1).row.set("repeats", "");
    assert_eq!(eval(&f).repeats.status, Status::Fails);
}

#[test]
fn a_twice_flown_flight_whose_hash_differs_from_the_main_table_fails_each_hash() {
    for column in ["trace_hash", "packet_hash", "reply_hash"] {
        let mut f = Fixture::small(&format!("rep-{column}"));
        // It says it repeated; the hash is not the main table's.
        f.twice_flown("gust-hold", 1)
            .row
            .set(column, "00000000deadbeef");
        let e = eval(&f);
        assert_eq!(e.repeats.status, Status::Fails, "{column}");
        assert!(mentions(
            &e.repeats.failing,
            &["gust-hold arducopter index 1", "main table"]
        ));
    }
    // The other way: the main table's hash is the one that differs.
    let mut f = Fixture::small("rep-main");
    f.row("gust-hold", AC, 0)
        .set("trace_hash", "00000000deadbeef");
    assert_eq!(eval(&f).repeats.status, Status::Fails);
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
    let v = verdict(&f.dir);
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
    let v = verdict(&f.dir);
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
    f.flown("gust-hold", F405, 4).commit = format!("{}-dirty", fx::COMMIT);
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
    let v = verdict(&f.dir);
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
