// SPDX-License-Identifier: AGPL-3.0-or-later
//! Deterministic maths shared by the flight core (f32) and the simulator (f64).
//!
//! Every transcendental function goes through the pure-Rust `libm` crate so
//! results are bit-identical on every host (DESIGN §4.3, §5.2). Call the
//! functions in this module, or `Real` trait methods, never the std float
//! methods; clippy enforces this in the workspace (`clippy.toml`).

use core::ops::{Add, AddAssign, Div, Mul, MulAssign, Neg, Sub, SubAssign};

/// Scalar type usable by the core and the simulator: `f32` or `f64`.
pub trait Real:
    Copy
    + PartialOrd
    + Default
    + core::fmt::Debug
    + Add<Output = Self>
    + Sub<Output = Self>
    + Mul<Output = Self>
    + Div<Output = Self>
    + Neg<Output = Self>
    + AddAssign
    + SubAssign
    + MulAssign
{
    const ZERO: Self;
    const ONE: Self;
    const TWO: Self;
    const HALF: Self;
    const PI: Self;

    /// Converts from f64 (round to nearest, deterministic).
    fn from_f64(v: f64) -> Self;
    /// Converts to f64 (exact for f32).
    fn to_f64(self) -> f64;
    fn sqrt(self) -> Self;
    fn sin(self) -> Self;
    fn cos(self) -> Self;
    fn asin(self) -> Self;
    fn atan2(self, x: Self) -> Self;
    fn exp(self) -> Self;
    fn abs(self) -> Self;
    fn is_finite(self) -> bool;

    /// Smaller of two values. If `self` is NaN the result is NaN.
    fn min(self, other: Self) -> Self {
        if other < self { other } else { self }
    }
    /// Larger of two values. If `self` is NaN the result is NaN.
    fn max(self, other: Self) -> Self {
        if other > self { other } else { self }
    }
    /// Clamps into `[lo, hi]` (assumes `lo <= hi`).
    fn clamp(self, lo: Self, hi: Self) -> Self {
        self.max(lo).min(hi)
    }
    /// Replaces a non-finite value with `fallback`.
    fn finite_or(self, fallback: Self) -> Self {
        if self.is_finite() { self } else { fallback }
    }
}

impl Real for f32 {
    const ZERO: Self = 0.0;
    const ONE: Self = 1.0;
    const TWO: Self = 2.0;
    const HALF: Self = 0.5;
    const PI: Self = core::f32::consts::PI;

    #[allow(clippy::cast_possible_truncation)]
    fn from_f64(v: f64) -> Self {
        v as f32
    }
    fn to_f64(self) -> f64 {
        f64::from(self)
    }
    fn sqrt(self) -> Self {
        libm::sqrtf(self)
    }
    fn sin(self) -> Self {
        libm::sinf(self)
    }
    fn cos(self) -> Self {
        libm::cosf(self)
    }
    fn asin(self) -> Self {
        libm::asinf(self)
    }
    fn atan2(self, x: Self) -> Self {
        libm::atan2f(self, x)
    }
    fn exp(self) -> Self {
        libm::expf(self)
    }
    fn abs(self) -> Self {
        libm::fabsf(self)
    }
    fn is_finite(self) -> bool {
        f32::is_finite(self)
    }
}

impl Real for f64 {
    const ZERO: Self = 0.0;
    const ONE: Self = 1.0;
    const TWO: Self = 2.0;
    const HALF: Self = 0.5;
    const PI: Self = core::f64::consts::PI;

    fn from_f64(v: f64) -> Self {
        v
    }
    fn to_f64(self) -> f64 {
        self
    }
    fn sqrt(self) -> Self {
        libm::sqrt(self)
    }
    fn sin(self) -> Self {
        libm::sin(self)
    }
    fn cos(self) -> Self {
        libm::cos(self)
    }
    fn asin(self) -> Self {
        libm::asin(self)
    }
    fn atan2(self, x: Self) -> Self {
        libm::atan2(self, x)
    }
    fn exp(self) -> Self {
        libm::exp(self)
    }
    fn abs(self) -> Self {
        libm::fabs(self)
    }
    fn is_finite(self) -> bool {
        f64::is_finite(self)
    }
}

/// 3-vector. In the world frame axes are North-East-Down; in the body frame
/// Forward-Right-Down (DESIGN §4.3).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec3<T> {
    pub x: T,
    pub y: T,
    pub z: T,
}

impl<T: Real> Vec3<T> {
    pub const fn new(x: T, y: T, z: T) -> Self {
        Self { x, y, z }
    }
    pub fn zero() -> Self {
        Self::new(T::ZERO, T::ZERO, T::ZERO)
    }
    pub fn splat(v: T) -> Self {
        Self::new(v, v, v)
    }
    pub fn dot(self, o: Self) -> T {
        self.x * o.x + self.y * o.y + self.z * o.z
    }
    pub fn cross(self, o: Self) -> Self {
        Self::new(
            self.y * o.z - self.z * o.y,
            self.z * o.x - self.x * o.z,
            self.x * o.y - self.y * o.x,
        )
    }
    pub fn norm_sq(self) -> T {
        self.dot(self)
    }
    pub fn norm(self) -> T {
        self.norm_sq().sqrt()
    }
    /// Unit vector, or `None` if the length is zero or not finite.
    pub fn normalized(self) -> Option<Self> {
        let n = self.norm();
        if n > T::ZERO && n.is_finite() {
            Some(self * (T::ONE / n))
        } else {
            None
        }
    }
    /// Component-wise product.
    pub fn hadamard(self, o: Self) -> Self {
        Self::new(self.x * o.x, self.y * o.y, self.z * o.z)
    }
    pub fn map(self, f: impl Fn(T) -> T) -> Self {
        Self::new(f(self.x), f(self.y), f(self.z))
    }
    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }
    /// Component-wise clamp to `[-limit, limit]`.
    pub fn clamp_abs(self, limit: Self) -> Self {
        Self::new(
            self.x.clamp(-limit.x, limit.x),
            self.y.clamp(-limit.y, limit.y),
            self.z.clamp(-limit.z, limit.z),
        )
    }
    /// Converts between f32 and f64.
    pub fn cast<U: Real>(self) -> Vec3<U> {
        Vec3::new(
            U::from_f64(self.x.to_f64()),
            U::from_f64(self.y.to_f64()),
            U::from_f64(self.z.to_f64()),
        )
    }
}

impl<T: Real> Add for Vec3<T> {
    type Output = Self;
    fn add(self, o: Self) -> Self {
        Self::new(self.x + o.x, self.y + o.y, self.z + o.z)
    }
}
impl<T: Real> Sub for Vec3<T> {
    type Output = Self;
    fn sub(self, o: Self) -> Self {
        Self::new(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}
impl<T: Real> Neg for Vec3<T> {
    type Output = Self;
    fn neg(self) -> Self {
        Self::new(-self.x, -self.y, -self.z)
    }
}
impl<T: Real> Mul<T> for Vec3<T> {
    type Output = Self;
    fn mul(self, s: T) -> Self {
        Self::new(self.x * s, self.y * s, self.z * s)
    }
}
impl<T: Real> AddAssign for Vec3<T> {
    fn add_assign(&mut self, o: Self) {
        *self = *self + o;
    }
}
impl<T: Real> SubAssign for Vec3<T> {
    fn sub_assign(&mut self, o: Self) {
        *self = *self - o;
    }
}

/// Unit quaternion `(w, x, y, z)` rotating body-frame vectors into the world
/// frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quat<T> {
    pub w: T,
    pub x: T,
    pub y: T,
    pub z: T,
}

impl<T: Real> Default for Quat<T> {
    fn default() -> Self {
        Self::identity()
    }
}

impl<T: Real> Quat<T> {
    pub const fn new(w: T, x: T, y: T, z: T) -> Self {
        Self { w, x, y, z }
    }
    pub fn identity() -> Self {
        Self::new(T::ONE, T::ZERO, T::ZERO, T::ZERO)
    }
    pub fn vector(self) -> Vec3<T> {
        Vec3::new(self.x, self.y, self.z)
    }
    pub fn conj(self) -> Self {
        Self::new(self.w, -self.x, -self.y, -self.z)
    }
    pub fn norm(self) -> T {
        (self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }
    /// Unit quaternion; returns identity if the norm is zero or not finite.
    pub fn normalized(self) -> Self {
        let n = self.norm();
        if n > T::ZERO && n.is_finite() {
            let k = T::ONE / n;
            Self::new(self.w * k, self.x * k, self.y * k, self.z * k)
        } else {
            Self::identity()
        }
    }
    pub fn is_finite(self) -> bool {
        self.w.is_finite() && self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }
    /// Hamilton product `self ⊗ o` (also available as `self * o`).
    pub fn hamilton(self, o: Self) -> Self {
        Self::new(
            self.w * o.w - self.x * o.x - self.y * o.y - self.z * o.z,
            self.w * o.x + self.x * o.w + self.y * o.z - self.z * o.y,
            self.w * o.y - self.x * o.z + self.y * o.w + self.z * o.x,
            self.w * o.z + self.x * o.y - self.y * o.x + self.z * o.w,
        )
    }
    /// Rotates a body-frame vector into the world frame.
    pub fn rotate(self, v: Vec3<T>) -> Vec3<T> {
        let u = self.vector();
        let t = u.cross(v) * T::TWO;
        v + t * self.w + u.cross(t)
    }
    /// Rotates a world-frame vector into the body frame.
    pub fn inverse_rotate(self, v: Vec3<T>) -> Vec3<T> {
        self.conj().rotate(v)
    }
    pub fn from_axis_angle(axis: Vec3<T>, angle: T) -> Self {
        match axis.normalized() {
            Some(a) => {
                let h = angle * T::HALF;
                let s = h.sin();
                Self::new(h.cos(), a.x * s, a.y * s, a.z * s)
            }
            None => Self::identity(),
        }
    }
    /// From roll, pitch, yaw (Z-Y-X aerospace convention).
    pub fn from_euler(roll: T, pitch: T, yaw: T) -> Self {
        let (sr, cr) = ((roll * T::HALF).sin(), (roll * T::HALF).cos());
        let (sp, cp) = ((pitch * T::HALF).sin(), (pitch * T::HALF).cos());
        let (sy, cy) = ((yaw * T::HALF).sin(), (yaw * T::HALF).cos());
        Self::new(
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        )
    }
    /// Roll, pitch, yaw (Z-Y-X aerospace convention).
    pub fn to_euler(self) -> Vec3<T> {
        let (w, x, y, z) = (self.w, self.x, self.y, self.z);
        let roll = (T::TWO * (w * x + y * z)).atan2(T::ONE - T::TWO * (x * x + y * y));
        let pitch = (T::TWO * (w * y - z * x)).clamp(-T::ONE, T::ONE).asin();
        let yaw = (T::TWO * (w * z + x * y)).atan2(T::ONE - T::TWO * (y * y + z * z));
        Vec3::new(roll, pitch, yaw)
    }
    /// Yaw angle only.
    pub fn yaw(self) -> T {
        let (w, x, y, z) = (self.w, self.x, self.y, self.z);
        (T::TWO * (w * z + x * y)).atan2(T::ONE - T::TWO * (y * y + z * z))
    }
    /// From the body axes expressed in the world frame (the columns of the
    /// body-to-world rotation matrix). Shepperd's method.
    pub fn from_axes(xb: Vec3<T>, yb: Vec3<T>, zb: Vec3<T>) -> Self {
        let (r00, r10, r20) = (xb.x, xb.y, xb.z);
        let (r01, r11, r21) = (yb.x, yb.y, yb.z);
        let (r02, r12, r22) = (zb.x, zb.y, zb.z);
        let quarter = T::HALF * T::HALF;
        let trace = r00 + r11 + r22;
        let q = if trace > T::ZERO {
            let s = (trace + T::ONE).sqrt() * T::TWO;
            Self::new(
                s * quarter,
                (r21 - r12) / s,
                (r02 - r20) / s,
                (r10 - r01) / s,
            )
        } else if r00 > r11 && r00 > r22 {
            let s = (T::ONE + r00 - r11 - r22).sqrt() * T::TWO;
            Self::new(
                (r21 - r12) / s,
                s * quarter,
                (r01 + r10) / s,
                (r02 + r20) / s,
            )
        } else if r11 > r22 {
            let s = (T::ONE + r11 - r00 - r22).sqrt() * T::TWO;
            Self::new(
                (r02 - r20) / s,
                (r01 + r10) / s,
                s * quarter,
                (r12 + r21) / s,
            )
        } else {
            let s = (T::ONE + r22 - r00 - r11).sqrt() * T::TWO;
            Self::new(
                (r10 - r01) / s,
                (r02 + r20) / s,
                (r12 + r21) / s,
                s * quarter,
            )
        };
        q.normalized()
    }
    /// Time derivative for a body angular rate `omega`: `0.5 · q ⊗ (0, ω)`.
    pub fn derivative(self, omega: Vec3<T>) -> Self {
        let p = self.hamilton(Self::new(T::ZERO, omega.x, omega.y, omega.z));
        Self::new(p.w * T::HALF, p.x * T::HALF, p.y * T::HALF, p.z * T::HALF)
    }
    /// `self + k · d`, component-wise (for integrators; not normalized).
    pub fn add_scaled(self, d: Self, k: T) -> Self {
        Self::new(
            self.w + d.w * k,
            self.x + d.x * k,
            self.y + d.y * k,
            self.z + d.z * k,
        )
    }
    /// Converts between f32 and f64.
    pub fn cast<U: Real>(self) -> Quat<U> {
        Quat::new(
            U::from_f64(self.w.to_f64()),
            U::from_f64(self.x.to_f64()),
            U::from_f64(self.y.to_f64()),
            U::from_f64(self.z.to_f64()),
        )
    }
}

impl<T: Real> Mul for Quat<T> {
    type Output = Self;
    fn mul(self, o: Self) -> Self {
        self.hamilton(o)
    }
}

/// Wraps an angle into `(-π, π]`.
pub fn wrap_pi<T: Real>(a: T) -> T {
    // atan2(sin, cos) is exact enough and branch-free for any finite input.
    a.sin().atan2(a.cos())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() <= tol
    }

    #[test]
    fn euler_round_trip() {
        let q = Quat::<f64>::from_euler(0.3, -0.2, 1.1);
        let e = q.to_euler();
        assert!(close(e.x, 0.3, 1e-12) && close(e.y, -0.2, 1e-12) && close(e.z, 1.1, 1e-12));
    }

    #[test]
    fn rotate_matches_axes() {
        // 90° yaw: body x (forward) points East.
        let q = Quat::<f64>::from_euler(0.0, 0.0, core::f64::consts::FRAC_PI_2);
        let v = q.rotate(Vec3::new(1.0, 0.0, 0.0));
        assert!(close(v.x, 0.0, 1e-12) && close(v.y, 1.0, 1e-12) && close(v.z, 0.0, 1e-12));
        let back = q.inverse_rotate(v);
        assert!(close(back.x, 1.0, 1e-12));
    }

    #[test]
    fn from_axes_round_trip() {
        let q = Quat::<f64>::from_euler(-0.4, 0.25, -2.0);
        let xb = q.rotate(Vec3::new(1.0, 0.0, 0.0));
        let yb = q.rotate(Vec3::new(0.0, 1.0, 0.0));
        let zb = q.rotate(Vec3::new(0.0, 0.0, 1.0));
        let r = Quat::from_axes(xb, yb, zb);
        // Same rotation up to sign.
        let d = (r.w * q.w + r.x * q.x + r.y * q.y + r.z * q.z).abs();
        assert!(close(d, 1.0, 1e-12));
    }

    #[test]
    fn wrap() {
        assert!(close(
            wrap_pi(3.0 * core::f64::consts::PI),
            core::f64::consts::PI,
            1e-9
        ));
        assert!(close(wrap_pi(-0.5_f64), -0.5, 1e-12));
    }
}
