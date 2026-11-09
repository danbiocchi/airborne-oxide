// SPDX-License-Identifier: AGPL-3.0-or-later
//! Digital filters for the flight core (f32, no heap, deterministic).

use crate::math::{Real, Vec3};

/// Second-order Butterworth low-pass on each axis of a `Vec3<f32>`
/// (biquad, direct form II transposed). Used on the rate-loop gyro.
///
/// The filter is pass-through when disabled (invalid or out-of-range cutoff).
/// The first sample after `new` or `reset` primes the internal state to that
/// sample's steady state, so there is no start-up transient.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LowPass2 {
    b0: f32,
    b1: f32,
    b2: f32,
    a1: f32,
    a2: f32,
    z1: Vec3<f32>,
    z2: Vec3<f32>,
    enabled: bool,
    primed: bool,
}

impl LowPass2 {
    /// Pass-through (disabled) when `cutoff_hz` is not finite, ≤ 0, or
    /// ≥ 0.45 × `sample_hz`, or `sample_hz` is not finite and positive.
    pub fn new(sample_hz: f32, cutoff_hz: f32) -> Self {
        let mut f = Self::passthrough();
        if !(sample_hz.is_finite() && sample_hz > 0.0) {
            return f;
        }
        if !(cutoff_hz.is_finite() && cutoff_hz > 0.0 && cutoff_hz < 0.45 * sample_hz) {
            return f;
        }
        // Coefficients in f64 for accuracy, stored as f32.
        let fs = f64::from(sample_hz);
        let fc = f64::from(cutoff_hz);
        let q = core::f64::consts::FRAC_1_SQRT_2;
        let k = <f64 as Real>::tan(core::f64::consts::PI * fc / fs);
        let k2 = k * k;
        let norm = 1.0 / (1.0 + k / q + k2);
        let b0 = k2 * norm;
        f.b0 = b0 as f32;
        f.b1 = (2.0 * b0) as f32;
        f.b2 = b0 as f32;
        f.a1 = (2.0 * (k2 - 1.0) * norm) as f32;
        f.a2 = ((1.0 - k / q + k2) * norm) as f32;
        f.enabled = true;
        f
    }

    fn passthrough() -> Self {
        Self {
            b0: 1.0,
            b1: 0.0,
            b2: 0.0,
            a1: 0.0,
            a2: 0.0,
            z1: Vec3::zero(),
            z2: Vec3::zero(),
            enabled: false,
            primed: false,
        }
    }

    /// Filters one sample. The first sample after `new`/`reset` primes the
    /// state to that input's steady state, so there is no start-up transient.
    /// A non-finite input is returned unchanged and resets the state. If the
    /// output would overflow, the state is reset and the input is returned
    /// unchanged.
    pub fn apply(&mut self, x: Vec3<f32>) -> Vec3<f32> {
        if !self.enabled {
            return x;
        }
        if !x.is_finite() {
            self.reset();
            return x;
        }
        if !self.primed {
            self.prime(x);
        }
        let ax = step_axis(self, x.x, self.z1.x, self.z2.x);
        let ay = step_axis(self, x.y, self.z1.y, self.z2.y);
        let az = step_axis(self, x.z, self.z1.z, self.z2.z);
        let out = Vec3::new(ax.0, ay.0, az.0);
        if !out.is_finite() {
            self.reset();
            return x;
        }
        self.z1 = Vec3::new(ax.1, ay.1, az.1);
        self.z2 = Vec3::new(ax.2, ay.2, az.2);
        self.primed = true;
        out
    }

    /// Clears the filter state. The next sample primes it again.
    pub fn reset(&mut self) {
        self.z1 = Vec3::zero();
        self.z2 = Vec3::zero();
        self.primed = false;
    }

    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    /// Sets the state so that a constant input `x0` passes unchanged.
    fn prime(&mut self, x0: Vec3<f32>) {
        let z2 = Vec3::new(
            (self.b2 - self.a2) * x0.x,
            (self.b2 - self.a2) * x0.y,
            (self.b2 - self.a2) * x0.z,
        );
        let z1 = Vec3::new(
            (self.b1 - self.a1) * x0.x + z2.x,
            (self.b1 - self.a1) * x0.y + z2.y,
            (self.b1 - self.a1) * x0.z + z2.z,
        );
        self.z1 = z1;
        self.z2 = z2;
        self.primed = true;
    }
}

/// One DF2T step for one axis. Returns `(y, z1', z2')`.
fn step_axis(f: &LowPass2, x: f32, z1: f32, z2: f32) -> (f32, f32, f32) {
    let y = f.b0 * x + z1;
    let new_z1 = f.b1 * x - f.a1 * y + z2;
    let new_z2 = f.b2 * x - f.a2 * y;
    (y, new_z1, new_z2)
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;

    /// Steady amplitude of a sine at `f_in` through the filter.
    fn steady_amplitude(f: &mut LowPass2, fs: f64, f_in: f64) -> f64 {
        let n = (fs * 2.0) as usize; // 2 s
        let mut peak = 0.0_f64;
        for k in 0..n {
            let t = k as f64 / fs;
            let x = <f64 as Real>::sin(2.0 * core::f64::consts::PI * f_in * t);
            let y = f.apply(Vec3::new(x as f32, 0.0, 0.0)).x;
            if k > n / 2 {
                peak = peak.max(f64::from(y.abs()));
            }
        }
        peak
    }

    #[test]
    fn dc_gain_is_one() {
        let mut f = LowPass2::new(1000.0, 50.0);
        assert!(f.is_enabled());
        let x = Vec3::new(0.3, -0.7, 1.0);
        let mut y = Vec3::zero();
        for _ in 0..2000 {
            y = f.apply(x);
        }
        assert!((y.x - 0.3).abs() < 1e-5);
        assert!((y.y + 0.7).abs() < 1e-5);
        assert!((y.z - 1.0).abs() < 1e-5);
    }

    #[test]
    fn first_sample_has_no_transient() {
        let mut f = LowPass2::new(1000.0, 50.0);
        let x = Vec3::new(2.0, -1.0, 0.5);
        for _ in 0..3 {
            let y = f.apply(x);
            assert!((y.x - 2.0).abs() < 1e-4);
            assert!((y.y + 1.0).abs() < 1e-4);
            assert!((y.z - 0.5).abs() < 1e-4);
        }
    }

    #[test]
    fn gain_at_cutoff_is_butterworth() {
        let fs = 1000.0;
        let mut f = LowPass2::new(fs as f32, 50.0);
        let a = steady_amplitude(&mut f, fs, 50.0);
        assert!((a - 0.707).abs() < 0.02, "amplitude at fc = {a}");
    }

    #[test]
    fn gain_at_four_times_cutoff_is_small() {
        let fs = 1000.0;
        let mut f = LowPass2::new(fs as f32, 50.0);
        let a = steady_amplitude(&mut f, fs, 200.0);
        assert!(a < 0.07, "amplitude at 4 fc = {a}");
    }

    #[test]
    fn disabled_is_exact_passthrough() {
        for (fs, fc) in [
            (1000.0_f32, 0.0_f32),
            (1000.0, -5.0),
            (1000.0, 450.0),
            (1000.0, 500.0),
            (1000.0, f32::NAN),
            (f32::NAN, 50.0),
            (0.0, 50.0),
            (-1000.0, 50.0),
        ] {
            let mut f = LowPass2::new(fs, fc);
            assert!(!f.is_enabled(), "fs={fs} fc={fc}");
            let x = Vec3::new(0.123_456, -9.87, 3.3);
            assert_eq!(f.apply(x), x);
            assert_eq!(f.apply(x), x);
        }
    }

    #[test]
    fn non_finite_input_is_returned_and_recovers() {
        let mut f = LowPass2::new(1000.0, 50.0);
        for _ in 0..100 {
            f.apply(Vec3::splat(1.0));
        }
        let bad = Vec3::new(f32::NAN, 0.0, 0.0);
        let out = f.apply(bad);
        assert!(out.x.is_nan());
        assert_eq!(out.y, 0.0);
        // After the bad sample the filter primes from the next input.
        let y = f.apply(Vec3::splat(2.0));
        assert_eq!(y, Vec3::splat(2.0));
        let inf = f.apply(Vec3::new(f32::INFINITY, 0.0, 0.0));
        assert!(inf.x.is_infinite());
        let y = f.apply(Vec3::splat(-1.0));
        assert!(y.is_finite());
        assert!((y.x + 1.0).abs() < 1e-4);
    }

    #[test]
    fn reset_reprimes() {
        let mut f = LowPass2::new(1000.0, 50.0);
        for _ in 0..500 {
            f.apply(Vec3::splat(1.0));
        }
        f.reset();
        let y = f.apply(Vec3::splat(-4.0));
        assert!((y.x + 4.0).abs() < 1e-4);
    }

    proptest! {
        // The 1.1 bound is only for cutoffs up to 0.05 × fs. The worst case
        // (L1 norm of the impulse response) is 1.09 there, but 1.10 at 0.1 × fs
        // and 1.29 at 0.25 × fs, so higher cutoffs are not covered by this test.
        #[test]
        fn bounded_input_gives_bounded_finite_output(
            fc in 1.0_f32..50.0,
            xs in proptest::collection::vec((-1.0_f32..=1.0, -1.0_f32..=1.0, -1.0_f32..=1.0), 1..400),
        ) {
            let mut f = LowPass2::new(1000.0, fc);
            for (x, y, z) in xs {
                let out = f.apply(Vec3::new(x, y, z));
                prop_assert!(out.is_finite());
                prop_assert!(out.x.abs() <= 1.1 && out.y.abs() <= 1.1 && out.z.abs() <= 1.1,
                    "out = {out:?}");
            }
        }
    }
}
