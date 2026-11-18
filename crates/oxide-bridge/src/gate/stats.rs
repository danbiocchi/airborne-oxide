// SPDX-License-Identifier: AGPL-3.0-or-later
//! The statistics of the gate: the percentile by linear interpolation
//! (ADR-0007 section 5), the median (its own formula for an even count), and
//! the mean.
//! Values are compared as computed, never rounded.

/// The `p`-quantile of `values` (0 to 1) by linear interpolation at position
/// `p * (n - 1)` counted from 0 in the sorted values. `None` for no values,
/// a value that is not a number, or a `p` outside 0 to 1.
///
/// A position that falls on a value returns that value as it is.
pub fn percentile(values: &[f64], p: f64) -> Option<f64> {
    if values.is_empty() || !(0.0..=1.0).contains(&p) || values.iter().any(|v| v.is_nan()) {
        return None;
    }
    let mut sorted = values.to_vec();
    sorted.sort_by(f64::total_cmp);
    let position = p * (sorted.len() - 1) as f64;
    let below = position.floor();
    let fraction = position - below;
    let lo = below as usize;
    let a = *sorted.get(lo)?;
    if fraction == 0.0 {
        return Some(a);
    }
    let b = *sorted.get(lo + 1)?;
    Some(a + fraction * (b - a))
}

/// The 95th percentile (position `0.95 * (n - 1)`).
pub fn percentile_95(values: &[f64]) -> Option<f64> {
    percentile(values, 0.95)
}

/// The median: the middle value, or for an even count `(a + b) / 2` of the two
/// middle values, computed exactly so (ADR-0007 sections 5 and 6, D7.1): not
/// the interpolation `a + 0.5 * (b - a)`, which differs from it in the last
/// bit for some pairs.
pub fn median(values: &[f64]) -> Option<f64> {
    if values.is_empty() || values.iter().any(|v| v.is_nan()) {
        return None;
    }
    let mut sorted = values.to_vec();
    sorted.sort_by(f64::total_cmp);
    let n = sorted.len();
    let hi = *sorted.get(n / 2)?;
    if n % 2 == 1 {
        return Some(hi);
    }
    let lo = *sorted.get(n / 2 - 1)?;
    Some((lo + hi) / 2.0)
}

/// The arithmetic mean; `None` for no values.
pub fn mean(values: &[f64]) -> Option<f64> {
    if values.is_empty() {
        return None;
    }
    Some(values.iter().sum::<f64>() / values.len() as f64)
}
