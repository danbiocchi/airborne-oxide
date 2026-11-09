// SPDX-License-Identifier: AGPL-3.0-or-later
//! Control allocation: turns a body torque and collective thrust demand into
//! per-rotor thrust.
//!
//! M1a uses the pseudo-inverse of the rotor effectiveness matrix with a
//! fixed, bounded desaturation order: roll/pitch first, then collective
//! thrust, then yaw. The constrained weighted-least-squares allocator in
//! DESIGN §4.5 replaces it later behind the same interface. This version
//! always returns finite fractions in `[0, 1]`, which a property test checks
//! (and a Kani proof will, §9).

use crate::config::{ConfigError, Rotor};
use crate::math::{Real, Vec3};

/// Row order of the effectiveness matrix.
const ROLL: usize = 0;
const PITCH: usize = 1;
const YAW: usize = 2;
const THRUST: usize = 3;

type Mat4 = [[f32; 4]; 4];

fn get(m: &Mat4, r: usize, c: usize) -> f32 {
    m.get(r).and_then(|row| row.get(c)).copied().unwrap_or(0.0)
}

fn scale_row(m: &mut Mat4, r: usize, k: f32) {
    if let Some(row) = m.get_mut(r) {
        row.iter_mut().for_each(|v| *v *= k);
    }
}

/// `m[r] -= f · m[src]`.
fn sub_row(m: &mut Mat4, r: usize, src: usize, f: f32) {
    let Some(s) = m.get(src).copied() else { return };
    if let Some(row) = m.get_mut(r) {
        row.iter_mut().zip(s.iter()).for_each(|(x, y)| *x -= f * *y);
    }
}

/// Gauss-Jordan inverse with partial pivoting. `None` if singular.
fn invert4(m: Mat4) -> Option<Mat4> {
    let mut a = m;
    let mut inv: Mat4 = [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ];
    for col in 0..4 {
        let mut piv = col;
        let mut best = 0.0_f32;
        for r in col..4 {
            let v = Real::abs(get(&a, r, col));
            if v > best {
                best = v;
                piv = r;
            }
        }
        // `best` starts at 0 and only grows past finite values, so it is never NaN.
        if best <= 1e-9 || !best.is_finite() {
            return None;
        }
        a.swap(col, piv);
        inv.swap(col, piv);
        let k = 1.0 / get(&a, col, col);
        scale_row(&mut a, col, k);
        scale_row(&mut inv, col, k);
        for r in 0..4 {
            if r != col {
                let f = get(&a, r, col);
                sub_row(&mut a, r, col, f);
                sub_row(&mut inv, r, col, f);
            }
        }
    }
    Some(inv)
}

/// Effectiveness column of one rotor: torque and thrust per newton.
fn effectiveness(r: &Rotor) -> [f32; 4] {
    // Thrust acts along body −z at (x, y): torque = r × (0, 0, −T).
    [-r.y, r.x, r.spin.sign() * r.torque_coeff_m, 1.0]
}

/// Result of one allocation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Allocation<const N: usize> {
    /// Thrust per rotor as a fraction of its maximum, in `[0, 1]`.
    pub fraction: [f32; N],
    /// The full demand could not be met.
    pub saturated: bool,
}

/// Mixer for `N` rotors.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mixer<const N: usize> {
    /// Per rotor: thrust (N) produced per unit of [roll, pitch, yaw, thrust].
    pinv: [[f32; 4]; N],
    max_thrust: [f32; N],
}

impl<const N: usize> Mixer<N> {
    /// Builds the mixer from the rotor layout.
    pub fn new(rotors: &[Rotor; N]) -> Result<Self, ConfigError> {
        let cols: [[f32; 4]; N] =
            core::array::from_fn(|i| rotors.get(i).map_or([0.0; 4], effectiveness));
        // M = B·Bᵀ (4×4).
        let mut m: Mat4 = [[0.0; 4]; 4];
        for (a, row) in m.iter_mut().enumerate() {
            for (b, v) in row.iter_mut().enumerate() {
                *v = cols
                    .iter()
                    .map(|c| c.get(a).copied().unwrap_or(0.0) * c.get(b).copied().unwrap_or(0.0))
                    .sum();
            }
        }
        let minv = invert4(m).ok_or(ConfigError::UncontrollableLayout)?;
        // pinv = Bᵀ·M⁻¹ : row i = colᵢᵀ · M⁻¹.
        let pinv: [[f32; 4]; N] = core::array::from_fn(|i| {
            let c = cols.get(i).copied().unwrap_or([0.0; 4]);
            core::array::from_fn(|k| {
                (0..4)
                    .map(|j| c.get(j).copied().unwrap_or(0.0) * get(&minv, j, k))
                    .sum()
            })
        });
        let max_thrust: [f32; N] =
            core::array::from_fn(|i| rotors.get(i).map_or(1.0, |r| r.max_thrust_n));
        if pinv.iter().flatten().any(|v| !v.is_finite()) {
            return Err(ConfigError::UncontrollableLayout);
        }
        Ok(Self { pinv, max_thrust })
    }

    /// Allocates a body torque (N·m) and collective thrust (N).
    pub fn mix(&self, torque: Vec3<f32>, thrust_n: f32) -> Allocation<N> {
        let tx = torque.x.finite_or(0.0);
        let ty = torque.y.finite_or(0.0);
        let tz = torque.z.finite_or(0.0);
        let t = thrust_n.finite_or(0.0).max(0.0);
        let mut saturated = !(torque.is_finite() && thrust_n.is_finite());

        // Per-rotor contributions as fractions of that rotor's maximum.
        let mut rp = [0.0_f32; N];
        let mut yaw = [0.0_f32; N];
        let mut col = [0.0_f32; N];
        for ((((p, tmax), rp), yaw), col) in self
            .pinv
            .iter()
            .zip(self.max_thrust.iter())
            .zip(rp.iter_mut())
            .zip(yaw.iter_mut())
            .zip(col.iter_mut())
        {
            let inv = 1.0 / *tmax;
            let pr = |k: usize| p.get(k).copied().unwrap_or(0.0);
            *rp = (pr(ROLL) * tx + pr(PITCH) * ty) * inv;
            *yaw = pr(YAW) * tz * inv;
            *col = pr(THRUST) * t * inv;
        }

        // 1. Roll/pitch must fit in the [0, 1] band by itself.
        let (lo, hi) = rp
            .iter()
            .fold((f32::INFINITY, f32::NEG_INFINITY), |(l, h), v| {
                (l.min(*v), h.max(*v))
            });
        let range = hi - lo;
        if range > 1.0 {
            let k = 1.0 / range;
            rp.iter_mut().for_each(|v| *v *= k);
            saturated = true;
        }

        // 2. Scale collective so roll/pitch fit: s·colᵢ + rpᵢ ∈ [0, 1].
        let mut s_hi = f32::INFINITY;
        let mut s_lo = 0.0_f32;
        for (c, r) in col.iter().zip(rp.iter()) {
            if *c > 1e-9 {
                s_hi = s_hi.min((1.0 - *r) / *c);
                s_lo = s_lo.max(-*r / *c);
            }
        }
        let mut s = 1.0_f32;
        if s > s_hi {
            s = s_hi;
            saturated = true;
        }
        if s < s_lo {
            // Keep attitude authority by raising collective (air mode).
            s = s_lo;
            saturated = true;
        }
        let s = s.max(0.0);
        let mut base = [0.0_f32; N];
        for ((b, c), r) in base.iter_mut().zip(col.iter()).zip(rp.iter()) {
            *b = s * *c + *r;
        }

        // 3. Add as much yaw as fits.
        let mut ky = 1.0_f32;
        for (b, y) in base.iter().zip(yaw.iter()) {
            if *y > 1e-9 {
                ky = ky.min((1.0 - *b) / *y);
            } else if *y < -1e-9 {
                ky = ky.min(-*b / *y);
            }
        }
        let ky = ky.clamp(0.0, 1.0);
        if ky < 1.0 {
            saturated = true;
        }

        let mut fraction = [0.0_f32; N];
        for ((f, b), y) in fraction.iter_mut().zip(base.iter()).zip(yaw.iter()) {
            *f = (*b + ky * *y).finite_or(0.0).clamp(0.0, 1.0);
        }
        Allocation {
            fraction,
            saturated,
        }
    }
}

/// Inverts the thrust curve `f = (1 − e)·u + e·u²` to get the command `u`
/// for thrust fraction `f`. Both are in `[0, 1]`.
pub fn command_for_fraction(fraction: f32, expo: f32) -> f32 {
    let f = fraction.finite_or(0.0).clamp(0.0, 1.0);
    let e = expo.finite_or(1.0).clamp(0.0, 1.0);
    if e < 1e-6 {
        return f;
    }
    let a = 1.0 - e;
    let u = (-a + Real::sqrt(a * a + 4.0 * e * f)) / (2.0 * e);
    u.finite_or(0.0).clamp(0.0, 1.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::Spin;
    use proptest::prelude::*;

    fn quad_x() -> [Rotor; 4] {
        let a = 0.08;
        let r = |x: f32, y: f32, spin| Rotor {
            x,
            y,
            spin,
            max_thrust_n: 8.0,
            torque_coeff_m: 0.016,
        };
        [
            r(a, a, Spin::Ccw),
            r(-a, -a, Spin::Ccw),
            r(a, -a, Spin::Cw),
            r(-a, a, Spin::Cw),
        ]
    }

    #[test]
    fn hover_is_equal_split() {
        let m = Mixer::new(&quad_x()).expect("valid layout");
        let a = m.mix(Vec3::zero(), 8.0);
        for f in a.fraction {
            assert!((f - 0.25).abs() < 1e-6, "{f}");
        }
        assert!(!a.saturated);
    }

    #[test]
    fn roll_right_speeds_up_left_rotors() {
        let m = Mixer::new(&quad_x()).expect("valid layout");
        let a = m.mix(Vec3::new(0.1, 0.0, 0.0), 8.0);
        // Rotors 1 (rear-left) and 2 (front-left) are on the left (y < 0).
        assert!(a.fraction[1] > a.fraction[0] && a.fraction[2] > a.fraction[3]);
    }

    #[test]
    fn yaw_right_speeds_up_ccw_rotors() {
        let m = Mixer::new(&quad_x()).expect("valid layout");
        let a = m.mix(Vec3::new(0.0, 0.0, 0.02), 8.0);
        assert!(a.fraction[0] > a.fraction[2]);
    }

    #[test]
    fn collinear_rotors_are_rejected() {
        let mut r = quad_x();
        for rotor in &mut r {
            rotor.y = 0.0;
        }
        assert_eq!(Mixer::new(&r), Err(ConfigError::UncontrollableLayout));
    }

    #[test]
    fn expo_inverse_round_trips() {
        for e in [0.0_f32, 0.3, 0.65, 1.0] {
            for i in 0..=10 {
                let u = i as f32 / 10.0;
                let f = (1.0 - e) * u + e * u * u;
                assert!((command_for_fraction(f, e) - u).abs() < 1e-5);
            }
        }
    }

    proptest! {
        #[test]
        fn mix_is_always_finite_and_bounded(
            tx in -50.0f32..50.0, ty in -50.0f32..50.0, tz in -5.0f32..5.0, t in -10.0f32..100.0
        ) {
            let m = Mixer::new(&quad_x()).expect("valid layout");
            let a = m.mix(Vec3::new(tx, ty, tz), t);
            for f in a.fraction {
                prop_assert!(f.is_finite() && (0.0..=1.0).contains(&f));
            }
        }

        #[test]
        fn mix_survives_non_finite_input(sel in 0usize..4) {
            let m = Mixer::new(&quad_x()).expect("valid layout");
            let mut v = [0.1f32, 0.1, 0.01, 8.0];
            if let Some(x) = v.get_mut(sel) { *x = f32::NAN; }
            let a = m.mix(Vec3::new(v[0], v[1], v[2]), v[3]);
            prop_assert!(a.saturated);
            for f in a.fraction {
                prop_assert!(f.is_finite() && (0.0..=1.0).contains(&f));
            }
        }
    }
}
