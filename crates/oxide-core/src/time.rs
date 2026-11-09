// SPDX-License-Identifier: AGPL-3.0-or-later
//! Time: `u64` microseconds since boot (DESIGN §4.3). Never a float, and
//! never a `u32` (which wraps after 71.6 minutes).

/// Microseconds since boot.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Micros(pub u64);

impl Micros {
    pub const ZERO: Self = Self(0);

    pub const fn from_millis(ms: u64) -> Self {
        Self(ms.saturating_mul(1000))
    }
    pub const fn from_secs(s: u64) -> Self {
        Self(s.saturating_mul(1_000_000))
    }
    /// `self + us`, saturating.
    #[must_use]
    pub const fn add_us(self, us: u64) -> Self {
        Self(self.0.saturating_add(us))
    }
    /// Microseconds elapsed since `earlier` (zero if `earlier` is later).
    pub const fn since(self, earlier: Self) -> u64 {
        self.0.saturating_sub(earlier.0)
    }
    /// Seconds as f64 (for logging and the simulator, never for control timing).
    #[allow(clippy::cast_precision_loss)]
    pub fn as_secs_f64(self) -> f64 {
        self.0 as f64 * 1e-6
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn saturates() {
        assert_eq!(Micros(5).since(Micros(10)), 0);
        assert_eq!(Micros(u64::MAX).add_us(1), Micros(u64::MAX));
        assert_eq!(Micros::from_millis(3).0, 3000);
    }
}
