// SPDX-License-Identifier: AGPL-3.0-or-later
//! The tick clock of a streamed topic (ADR-0005 §3).

use super::rates::{Tick, times};
use super::started;
use crate::header::Level;
use crate::reader::{Item, UlogFile};
use crate::recorder::Clock;
use oxide_core::time::Micros;

/// The formula before the division was taken off the common path.
fn old_advance(next: Option<u64>, now: u64, period: u64) -> u64 {
    let period = period.max(1);
    match next {
        None => now.saturating_add(period),
        Some(n) => {
            let late = now.saturating_sub(n) / period;
            n.saturating_add(period.saturating_mul(late.saturating_add(1)))
        }
    }
}

fn advanced(next: Option<u64>, now: u64, period: u64) -> u64 {
    let mut c = Clock::new(period);
    c.next = next;
    c.advance(now);
    c.next.expect("advance sets the next time")
}

#[test]
fn advance_gives_what_the_dividing_formula_gave() {
    let p = 10_000;
    let max = u64::MAX;
    // (next, now, period)
    let table: &[(Option<u64>, u64, u64)] = &[
        (None, 0, p),
        (None, 123, p),
        (None, max, p),
        (None, 5, 0),
        // exactly on time, a little late, one below a period late
        (Some(100_000), 100_000, p),
        (Some(100_000), 100_001, p),
        (Some(100_000), 109_999, p),
        // exact multiples of the period late
        (Some(100_000), 110_000, p),
        (Some(100_000), 120_000, p),
        (Some(100_000), 2_100_000, p),
        // one below a multiple
        (Some(100_000), 119_999, p),
        (Some(100_000), 2_099_999, p),
        // not yet due, but within a period
        (Some(100_000), 99_999, p),
        (Some(100_000), 90_000, p),
        // zero period behaves as one microsecond
        (Some(7), 7, 0),
        (Some(7), 12, 0),
        (Some(7), 3, 0),
        // period of one
        (Some(7), 7, 1),
        (Some(7), 9, 1),
        // near the end of the range: the sum saturates
        (Some(max - 5), max - 5, p),
        (Some(max - 5), max, p),
        (Some(max - p), max, p),
        (Some(max - p - 1), max, p),
        (Some(max), max, p),
        (Some(max - 1), max, 1),
        (Some(0), max, p),
        (Some(0), max, 1),
        (Some(0), max, max),
        (Some(1), max, max),
        (Some(max / 2), max, max / 2),
        (Some(max / 2), max - 1, max / 2),
        (Some(max / 2 + 1), max, max / 2),
        (Some(3), 3, max),
        (None, 3, max),
    ];
    // A next time more than a period ahead of `now` is a clock that stepped
    // back, which the old formula did not know (see below).
    let in_step = |next: Option<u64>, now: u64, period: u64| {
        next.is_none_or(|n| n <= now.saturating_add(period.max(1)))
    };
    for &(next, now, period) in table.iter().filter(|e| in_step(e.0, e.1, e.2)) {
        assert_eq!(
            advanced(next, now, period),
            old_advance(next, now, period),
            "next {next:?}, now {now}, period {period}"
        );
    }
    // And over a grid of small values around the multiples.
    for period in [0, 1, 2, 3, 10] {
        for next in 0..40 {
            for now in (0..80).filter(|n| in_step(Some(next), *n, period)) {
                assert_eq!(
                    advanced(Some(next), now, period),
                    old_advance(Some(next), now, period),
                    "next {next}, now {now}, period {period}"
                );
            }
        }
    }
}

/// Records the given tick times, in order, 1 ms apart or as given.
fn record_ticks(ticks: &[u64]) -> UlogFile {
    let mut file = Vec::new();
    let (mut rec, cfg) = started(&mut file, Level::Normal);
    for &t in ticks {
        let tick = Tick::at(t);
        rec.record(Micros(t), &tick.inputs, &tick.outputs, &tick.tel, &cfg);
        rec.drain(&mut file);
    }
    UlogFile::parse(&file).expect("parses")
}

fn syncs(f: &UlogFile) -> usize {
    f.items.iter().filter(|i| matches!(i, Item::Sync)).count()
}

#[test]
fn a_clock_that_steps_back_does_not_silence_the_streams() {
    // The reviewer's probe: 0..5 s at 1 kHz, then the tick clock goes back to
    // 0 and runs 0..8 s; and a tick repeats the same `now` once.
    let first: Vec<u64> = (0..5000).map(|k| k * 1000).collect();
    let second: Vec<u64> = (0..8000).map(|k| k * 1000).collect();
    let mut ticks = first.clone();
    ticks.extend(&second);
    ticks.insert(10, 9000);
    let f = record_ticks(&ticks);
    for (name, hz) in [
        ("sensor_combined", 100),
        ("vehicle_attitude", 50),
        ("vehicle_local_position", 20),
        ("manual_control_setpoint", 10),
        ("battery_status", 5),
    ] {
        // Both passes write their own rate from their own start.
        let want = (5 + 8) * hz;
        assert_eq!(times(&f, name).len() as u64, want, "{name}");
    }
    // Status topics and the logger: one a second, from each start.
    for name in ["vehicle_status", "oxide_status", "oxide_logger"] {
        assert_eq!(times(&f, name).len(), 5 + 8, "{name}");
    }
    // The repeated `now` wrote nothing twice.
    let ts = times(&f, "vehicle_attitude");
    let mut seen = std::collections::BTreeSet::new();
    let first_pass = &ts[..250];
    assert!(first_pass.iter().all(|t| seen.insert(*t)), "{first_pass:?}");
    // The synchronisation messages are 500 ms apart in each pass, and the
    // second pass starts with one at once.
    let alone = |ticks: &[u64]| syncs(&record_ticks(ticks));
    assert_eq!(syncs(&f), alone(&first) + alone(&second) + 1);
}

#[test]
fn a_step_back_of_less_than_a_period_changes_nothing() {
    // 5 ms back is within the 10 ms period of the fastest topic.
    let a = record_ticks(&[0, 4_000, 9_000, 6_000, 14_000, 19_000, 24_000]);
    let b = record_ticks(&[0, 4_000, 9_000, 14_000, 19_000, 24_000]);
    assert_eq!(times(&a, "sensor_combined"), times(&b, "sensor_combined"));
    assert_eq!(times(&a, "sensor_combined"), [0, 14_000, 24_000]);
}

#[test]
fn advance_re_anchors_a_clock_that_is_more_than_a_period_ahead() {
    let p = 10_000;
    let mut c = Clock::new(p);
    c.next = Some(5_000_000);
    assert!(c.due(0), "a clock far ahead of now is due again");
    c.advance(0);
    assert_eq!(c.next, Some(p));
    // Exactly one period ahead is not a step back.
    c.next = Some(20_000);
    assert!(!c.due(10_000));
    c.next = Some(20_001);
    assert!(c.due(10_000));
    // Near the end of the range nothing overflows.
    let mut c = Clock::new(p);
    c.next = Some(u64::MAX);
    assert!(!c.due(u64::MAX - p));
    assert!(c.due(u64::MAX - p - 1));
    c.advance(u64::MAX - p - 1);
    assert_eq!(c.next, Some(u64::MAX - 1));
}
