// SPDX-License-Identifier: AGPL-3.0-or-later
//! The tick clock of a streamed topic (ADR-0005 §3).

use crate::recorder::Clock;

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
    for &(next, now, period) in table {
        assert_eq!(
            advanced(next, now, period),
            old_advance(next, now, period),
            "next {next:?}, now {now}, period {period}"
        );
    }
    // And over a grid of small values around the multiples.
    for period in [0, 1, 2, 3, 10] {
        for next in 0..40 {
            for now in 0..80 {
                assert_eq!(
                    advanced(Some(next), now, period),
                    old_advance(Some(next), now, period),
                    "next {next}, now {now}, period {period}"
                );
            }
        }
    }
}
