// SPDX-License-Identifier: AGPL-3.0-or-later
//! The program's instance numbers for the tests that fly it, leased one to a
//! flight at a time so that tests that run side by side never share a port,
//! and so that two tasks that share the machine can each take a range of
//! their own.
//!
//! The range is `OXIDE_ARDUCOPTER_INSTANCES`, `a-b` (inclusive, `0 <= a <= b
//! <= 19`), read once; the default is `0-9`. Anything else panics with a
//! sentence that names the variable and what it holds.

use std::sync::{Condvar, Mutex, OnceLock};

/// The variable that names the range.
pub const VARIABLE: &str = "OXIDE_ARDUCOPTER_INSTANCES";

/// The range when the variable is unset.
pub const DEFAULT: (u8, u8) = (0, 9);

/// The largest instance number a range may name.
pub const LARGEST: u8 = 19;

/// Parses `a-b` (or the default for `None`). The error is a sentence.
pub fn parse_range(text: Option<&str>) -> Result<(u8, u8), String> {
    let Some(text) = text else {
        return Ok(DEFAULT);
    };
    let bad = |why: &str| {
        format!(
            "{VARIABLE} is {text:?}: {why}; it must be a range `a-b` with 0 <= a <= b <= {LARGEST} \
             (for example `0-9`)"
        )
    };
    let (a, b) = text
        .split_once('-')
        .ok_or_else(|| bad("there is no `-` between two numbers"))?;
    let number = |part: &str| -> Result<u8, String> {
        if part.is_empty() || !part.bytes().all(|c| c.is_ascii_digit()) {
            return Err(bad("each end is a whole number of digits"));
        }
        part.parse::<u8>()
            .ok()
            .filter(|n| *n <= LARGEST)
            .ok_or_else(|| bad("an end is above the largest instance number"))
    };
    let (a, b) = (number(a)?, number(b)?);
    if a > b {
        return Err(bad("the first number is above the second"));
    }
    Ok((a, b))
}

/// The range of this process: read once, panics on a bad value.
pub fn range() -> (u8, u8) {
    static RANGE: OnceLock<(u8, u8)> = OnceLock::new();
    *RANGE.get_or_init(|| {
        let text = std::env::var(VARIABLE).ok();
        parse_range(text.as_deref()).unwrap_or_else(|e| panic!("{e}"))
    })
}

static POOL: (Mutex<u32>, Condvar) = (Mutex::new(0), Condvar::new());

/// A leased instance number. A test holds its lease for all the flights it
/// flies one after the other.
pub struct Instance(pub u8);

impl Instance {
    /// The lowest free instance of the range, waiting for one when all are
    /// taken.
    pub fn lease() -> Self {
        let (low, high) = range();
        let mut taken = POOL.0.lock().expect("the pool");
        loop {
            if let Some(i) = (low..=high).find(|i| *taken & (1 << i) == 0) {
                *taken |= 1 << i;
                return Self(i);
            }
            taken = POOL.1.wait(taken).expect("the pool");
        }
    }
}

impl Drop for Instance {
    fn drop(&mut self) {
        if let Ok(mut taken) = POOL.0.lock() {
            *taken &= !(1 << self.0);
        }
        POOL.1.notify_all();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_default_is_0_to_9() {
        assert_eq!(parse_range(None), Ok((0, 9)));
    }

    #[test]
    fn a_range_is_two_inclusive_ends() {
        assert_eq!(parse_range(Some("0-9")), Ok((0, 9)));
        assert_eq!(parse_range(Some("3-3")), Ok((3, 3)));
        assert_eq!(parse_range(Some("10-19")), Ok((10, 19)));
        assert_eq!(parse_range(Some("0-19")), Ok((0, 19)));
        assert_eq!(parse_range(Some("05-07")), Ok((5, 7)));
    }

    #[test]
    fn anything_else_is_a_sentence_that_names_the_variable() {
        for bad in [
            "", "5", "-", "a-b", "3-", "-3", "5-4", "0-20", "20-20", "256-300", "1-2-3", " 1-2",
            "1 -2", "+1-2", "1,2", "1..2", "-1-5",
        ] {
            let e = parse_range(Some(bad)).unwrap_err();
            assert!(
                e.contains(VARIABLE) && e.contains("a-b") && e.contains(&format!("{bad:?}")),
                "{bad:?}: {e}"
            );
        }
    }
}
