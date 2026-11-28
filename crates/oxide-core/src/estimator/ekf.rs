// SPDX-License-Identifier: AGPL-3.0-or-later
//! Error-state extended Kalman filter (DESIGN §4.7, ADR-0002).
//!
//! Nominal state: attitude `q` (body to world), velocity and position (world
//! NED), gyro bias and accelerometer bias (body FRD). The 15-element error
//! state is `[δp, δv, δθ, δbg, δba]`; the attitude error is a **world-frame**
//! rotation vector, `q_true = Exp(δθ) ⊗ q` (Solà 2017, "global angular
//! error"). Prediction integrates IMU deltas; measurements are fused one
//! scalar at a time with an innovation gate. The process-noise parameters
//! are PX4 EKF2's, stated for its 10 ms prediction step; the variance added
//! per prediction is `σ² · dt · 0.01 s`, the same per second at any filter
//! rate. Generic over the scalar so the
//! Jacobians can be checked in f64 while the core runs `Ekf<f32>`.

use crate::config::EkfParams;
use crate::linalg::Mat;
use crate::math::{Quat, Real, Vec3, wrap_pi};

use super::imu::ImuDelta;

/// Error-state dimension.
pub const N_ERR: usize = 15;
/// Error-state index of position.
pub const IP: usize = 0;
/// Error-state index of velocity.
pub const IV: usize = 3;
/// Error-state index of the attitude error.
pub const ITH: usize = 6;
/// Error-state index of the gyro bias.
pub const IBG: usize = 9;
/// Error-state index of the accelerometer bias.
pub const IBA: usize = 12;

/// Standard gravity, m/s².
const G: f64 = 9.806_65;
/// Largest gyro bias the filter will estimate, rad/s per axis.
pub const GYRO_BIAS_LIMIT: f64 = 0.15;
/// Largest accelerometer bias the filter will estimate, m/s² per axis.
const ACCEL_BIAS_LIMIT: f64 = 0.4;
/// Variance limits `(min, max)` per error-state group: p, v, θ, bg, ba.
const VAR_LIMITS: [(f64, f64); 5] = [
    (1e-6, 1e6),
    (1e-6, 1e4),
    (1e-9, 1.0),
    (1e-12, 0.25),
    (1e-9, 25.0),
];

type M3<T> = Mat<T, 3, 3>;
type Cov<T> = Mat<T, N_ERR, N_ERR>;

/// The nominal (large-signal) state.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NominalState<T> {
    /// Attitude, body to world.
    pub q: Quat<T>,
    /// Velocity, world NED, m/s.
    pub v: Vec3<T>,
    /// Position relative to the alignment point, world NED, m.
    pub p: Vec3<T>,
    /// Gyro bias, body, rad/s.
    pub bg: Vec3<T>,
    /// Accelerometer bias, body, m/s².
    pub ba: Vec3<T>,
}

/// Result of a measurement update.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Fusion<T> {
    /// Fused. `test_ratio` is the squared normalised innovation over the
    /// squared gate (at most 1).
    Accepted { test_ratio: T },
    /// Failed the innovation gate; nothing changed.
    Rejected { test_ratio: T },
    /// Not fused: numerical problem or unobservable geometry.
    Skipped,
}

/// The prediction step the process-noise parameters are stated for, s
/// (PX4 EKF2's 10 ms filter period, whose default values these are).
const Q_REF_DT_S: f64 = 0.01;

/// Filter parameters converted to the filter's scalar type.
#[derive(Clone, Copy, Debug)]
struct Prm<T> {
    gyro_noise: T,
    accel_noise: T,
    gyro_bias_noise: T,
    accel_bias_noise: T,
    gps_gate: T,
    baro_gate: T,
    yaw_gate: T,
    declination: T,
    /// `cos` and `sin` of the declination.
    decl_cos: T,
    decl_sin: T,
    init_tilt: T,
    init_yaw: T,
    init_vel: T,
    init_pos: T,
    init_bg: T,
    init_ba: T,
}

impl<T: Real> Prm<T> {
    fn new(p: &EkfParams) -> Self {
        let c = |x: f32| T::from_f64(f64::from(x));
        Self {
            gyro_noise: c(p.gyro_noise),
            accel_noise: c(p.accel_noise),
            gyro_bias_noise: c(p.gyro_bias_noise),
            accel_bias_noise: c(p.accel_bias_noise),
            gps_gate: c(p.gps_gate),
            baro_gate: c(p.baro_gate),
            yaw_gate: c(p.yaw_gate),
            declination: c(p.mag_declination_rad),
            decl_cos: c(p.mag_declination_rad).cos(),
            decl_sin: c(p.mag_declination_rad).sin(),
            init_tilt: c(p.init_tilt_sigma_rad),
            init_yaw: c(p.init_yaw_sigma_rad),
            init_vel: c(p.init_vel_sigma_mps),
            init_pos: c(p.init_pos_sigma_m),
            init_bg: c(p.init_gyro_bias_sigma),
            init_ba: c(p.init_accel_bias_sigma),
        }
    }

    fn initial_cov(&self) -> Cov<T> {
        let mut p = Cov::zeros();
        let sigmas = [
            self.init_pos,
            self.init_pos,
            self.init_pos,
            self.init_vel,
            self.init_vel,
            self.init_vel,
            self.init_tilt,
            self.init_tilt,
            self.init_yaw,
            self.init_bg,
            self.init_bg,
            self.init_bg,
            self.init_ba,
            self.init_ba,
            self.init_ba,
        ];
        for (i, s) in sigmas.iter().enumerate() {
            p.set(i, i, *s * *s);
        }
        p
    }
}

/// The error-state EKF.
#[derive(Clone, Copy, Debug)]
pub struct Ekf<T> {
    /// Nominal state.
    pub x: NominalState<T>,
    /// Error-state covariance.
    pub p: Cov<T>,
    prm: Prm<T>,
    faults: u32,
    /// Horizontal part (north, east) of the world-frame velocity change the
    /// last `predict` integrated, m/s: `R(q) (dvel - ba dt)`. Zero when the
    /// last delta was unusable. Read-only for the heading check.
    dvw_ne: [T; 2],
}

/// True for a finite, strictly positive value (false for NaN).
fn pos<T: Real>(x: T) -> bool {
    x.is_finite() && x > T::ZERO
}

/// True for a finite `a >= b` (false for NaN).
fn at_least<T: Real>(a: T, b: T) -> bool {
    a.is_finite() && a >= b
}

fn skew<T: Real>(v: Vec3<T>) -> M3<T> {
    let z = T::ZERO;
    Mat::from_rows([[z, -v.z, v.y], [v.z, z, -v.x], [-v.y, v.x, z]])
}

/// Rotation matrix of a (unit) quaternion, body to world.
fn rot_matrix<T: Real>(q: Quat<T>) -> M3<T> {
    let (w, x, y, z) = (q.w, q.x, q.y, q.z);
    let two = T::TWO;
    let one = T::ONE;
    Mat::from_rows([
        [
            one - two * (y * y + z * z),
            two * (x * y - w * z),
            two * (x * z + w * y),
        ],
        [
            two * (x * y + w * z),
            one - two * (x * x + z * z),
            two * (y * z - w * x),
        ],
        [
            two * (x * z - w * y),
            two * (y * z + w * x),
            one - two * (x * x + y * y),
        ],
    ])
}

fn mat3_vec<T: Real>(m: &M3<T>, v: Vec3<T>) -> Vec3<T> {
    let [x, y, z] = m.mul_vec(&[v.x, v.y, v.z]);
    Vec3::new(x, y, z)
}

fn vec_at<T: Real>(a: &[T; N_ERR], o: usize) -> Vec3<T> {
    let g = |i: usize| a.get(o + i).copied().unwrap_or(T::ZERO);
    Vec3::new(g(0), g(1), g(2))
}

/// Applies an error-state correction to a nominal state (the injection
/// convention: `q ← Exp(δθ) ⊗ q`, everything else additive).
fn injected<T: Real>(x: &NominalState<T>, dx: &[T; N_ERR]) -> NominalState<T> {
    NominalState {
        q: (Quat::from_rotation_vector(vec_at(dx, ITH)) * x.q).normalized(),
        v: x.v + vec_at(dx, IV),
        p: x.p + vec_at(dx, IP),
        bg: x.bg + vec_at(dx, IBG),
        ba: x.ba + vec_at(dx, IBA),
    }
}

impl<T: Real> Ekf<T> {
    /// At rest at the origin with attitude `q`; covariance from the init
    /// sigmas.
    pub fn new(params: &EkfParams, q: Quat<T>) -> Self {
        let prm = Prm::new(params);
        Self {
            x: NominalState {
                q: q.normalized(),
                v: Vec3::zero(),
                p: Vec3::zero(),
                bg: Vec3::zero(),
                ba: Vec3::zero(),
            },
            p: prm.initial_cov(),
            prm,
            faults: 0,
            dvw_ne: [T::ZERO; 2],
        }
    }

    /// Number of covariance resets after numerical faults.
    pub fn faults(&self) -> u32 {
        self.faults
    }

    /// Propagates the nominal state over one IMU delta and returns it with
    /// the error-state transition matrix. `None` if the delta is unusable.
    fn transition(&self, d: &ImuDelta<T>) -> Option<(NominalState<T>, Cov<T>, Vec3<T>)> {
        let dt = d.dt;
        if !pos(dt) || !d.dang.is_finite() || !d.dvel.is_finite() {
            return None;
        }
        let dth = d.dang - self.x.bg * dt;
        let dv = d.dvel - self.x.ba * dt;
        let q = (self.x.q * Quat::from_rotation_vector(dth)).normalized();
        let r = rot_matrix(q);
        let dvw = mat3_vec(&r, dv);
        let g = Vec3::new(T::ZERO, T::ZERO, T::from_f64(G));
        let v = self.x.v + dvw + g * dt;
        let p = self.x.p + (self.x.v + v) * (T::HALF * dt);
        if !(q.is_finite() && v.is_finite() && p.is_finite()) {
            return None;
        }

        // First-order right Jacobian of SO(3).
        let j = M3::identity().sub(&skew(dth).scale(T::HALF));
        let r_sdv_j = r.mul(&skew(dv)).mul(&j);
        let sdvw = skew(dvw);
        let mut f = Cov::<T>::identity();
        f.set_block3(IP, IV, M3::identity().scale(dt).rows());
        f.set_block3(IP, ITH, sdvw.scale(-T::HALF * dt).rows());
        f.set_block3(IP, IBG, r_sdv_j.scale(T::HALF * dt * dt).rows());
        f.set_block3(IP, IBA, r.scale(-T::HALF * dt * dt).rows());
        f.set_block3(IV, ITH, sdvw.scale(-T::ONE).rows());
        f.set_block3(IV, IBG, r_sdv_j.scale(dt).rows());
        f.set_block3(IV, IBA, r.scale(-dt).rows());
        f.set_block3(ITH, IBG, r.mul(&j).scale(-dt).rows());

        let next = NominalState {
            q,
            v,
            p,
            bg: self.x.bg,
            ba: self.x.ba,
        };
        Some((next, f, dvw))
    }

    /// The horizontal (north, east) world-frame velocity change the last
    /// [`Ekf::predict`] integrated, m/s (zero if that delta was unusable).
    pub fn last_dvw_ne(&self) -> [T; 2] {
        self.dvw_ne
    }

    /// Time update with one IMU delta. Ignored when `dt <= 0` or anything is
    /// not finite.
    pub fn predict(&mut self, d: &ImuDelta<T>) {
        let Some((next, f, dvw)) = self.transition(d) else {
            self.dvw_ne = [T::ZERO; 2];
            return;
        };
        self.dvw_ne = [dvw.x, dvw.y];
        let dt = d.dt;
        let mut p = f.mul(&self.p).mul(&f.transpose());
        let sq = |x: T| x * x;
        // PX4 applies these parameters as (sigma * dt)^2 per 10 ms step, which
        // is sigma^2 * 0.01 s of variance per second (sigma^2 * 0.01 * dt
        // over an interval dt). Applied per step of another length that form
        // would grow the covariance by another amount per second (0.4 of
        // PX4's at 4 ms). The variance added per step is
        // therefore sigma^2 * dt * Q_REF_DT_S: exactly PX4's at dt = 10 ms,
        // and the same per second at any filter rate.
        let qdt = dt * T::from_f64(Q_REF_DT_S);
        let noise = [
            (IV, sq(self.prm.accel_noise) * qdt),
            (ITH, sq(self.prm.gyro_noise) * qdt),
            (IBG, sq(self.prm.gyro_bias_noise) * qdt),
            (IBA, sq(self.prm.accel_bias_noise) * qdt),
        ];
        for (start, var) in noise {
            for k in 0..3 {
                p.add_at(start + k, start + k, var);
            }
        }
        self.x = next;
        self.p = p;
        self.condition();
    }

    /// Symmetrises and bounds the covariance, resets it after a numerical
    /// fault, and bounds the bias states.
    fn condition(&mut self) {
        self.p.symmetrize();
        if self.p.is_finite() {
            for i in 0..N_ERR {
                let (lo, hi) = VAR_LIMITS.get(i / 3).copied().unwrap_or((1e-9, 1e4));
                let v = self.p.get(i, i).clamp(T::from_f64(lo), T::from_f64(hi));
                self.p.set(i, i, v);
            }
        } else {
            self.p = self.prm.initial_cov();
            self.faults = self.faults.saturating_add(1);
        }
        let bg = T::from_f64(GYRO_BIAS_LIMIT);
        let ba = T::from_f64(ACCEL_BIAS_LIMIT);
        self.x.bg = self
            .x
            .bg
            .map(|e| e.finite_or(T::ZERO))
            .clamp_abs(Vec3::splat(bg));
        self.x.ba = self
            .x
            .ba
            .map(|e| e.finite_or(T::ZERO))
            .clamp_abs(Vec3::splat(ba));
    }

    /// `P·Hᵀ` and the innovation variance `S` for a sparse scalar
    /// measurement; `None` if `S` is not positive and finite.
    fn gain(&self, h: &[(usize, T)], r: T) -> Option<([T; N_ERR], T)> {
        let mut pht = [T::ZERO; N_ERR];
        for (i, e) in pht.iter_mut().enumerate() {
            let mut a = T::ZERO;
            for &(k, c) in h {
                a += self.p.get(i, k) * c;
            }
            *e = a;
        }
        let mut s = r;
        for &(k, c) in h {
            s += c * pht.get(k).copied().unwrap_or(T::ZERO);
        }
        let ok = s > T::ZERO && s.is_finite() && pht.iter().all(|e| e.is_finite());
        ok.then_some((pht, s))
    }

    /// Covariance update and state injection for a scalar measurement.
    fn apply(&mut self, pht: &[T; N_ERR], s: T, nu: T) {
        let inv_s = T::ONE / s;
        let mut dx = [T::ZERO; N_ERR];
        for (i, (d, &pi)) in dx.iter_mut().zip(pht.iter()).enumerate() {
            let ki = pi * inv_s;
            *d = ki * nu;
            for (j, &pj) in pht.iter().enumerate() {
                self.p.add_at(i, j, -(ki * pj));
            }
        }
        // The covariance-reset Jacobian `I − [½δθ]×` is neglected, as in PX4.
        self.x = injected(&self.x, &dx);
        self.condition();
    }

    /// Gated scalar update.
    fn fuse_scalar(&mut self, h: &[(usize, T)], nu: T, sigma: T, gate: T) -> Fusion<T> {
        let r = sigma * sigma;
        if !nu.is_finite() || !pos(r) {
            return Fusion::Skipped;
        }
        let Some((pht, s)) = self.gain(h, r) else {
            return Fusion::Skipped;
        };
        let ratio = nu * nu / (gate * gate * s);
        if !(ratio.is_finite() && ratio <= T::ONE) {
            return Fusion::Rejected { test_ratio: ratio };
        }
        self.apply(&pht, s, nu);
        Fusion::Accepted { test_ratio: ratio }
    }

    /// Group-gated vector update on directly observed error-state components
    /// (`H` is 1 at each index).
    fn fuse_vector<const M: usize>(
        &mut self,
        idx: [usize; M],
        z: [T; M],
        sigma: T,
        gate: T,
        get: impl Fn(&NominalState<T>) -> [T; M],
    ) -> Fusion<T> {
        let r = sigma * sigma;
        if !z.iter().all(|e| e.is_finite()) || !pos(r) {
            return Fusion::Skipped;
        }
        let h0 = get(&self.x);
        let mut worst = T::ZERO;
        for ((&i, &zi), &hi) in idx.iter().zip(z.iter()).zip(h0.iter()) {
            let Some((_, s)) = self.gain(&[(i, T::ONE)], r) else {
                return Fusion::Skipped;
            };
            let nu = zi - hi;
            let ratio = nu * nu / (gate * gate * s);
            if !ratio.is_finite() {
                return Fusion::Rejected { test_ratio: ratio };
            }
            worst = worst.max(ratio);
        }
        if worst > T::ONE {
            return Fusion::Rejected { test_ratio: worst };
        }
        let mut fused = false;
        for (k, &i) in idx.iter().enumerate() {
            let cur = get(&self.x);
            let nu = z.get(k).copied().unwrap_or(T::ZERO) - cur.get(k).copied().unwrap_or(T::ZERO);
            if let Some((pht, s)) = self.gain(&[(i, T::ONE)], r)
                && nu.is_finite()
            {
                self.apply(&pht, s, nu);
                fused = true;
            }
        }
        if fused {
            Fusion::Accepted { test_ratio: worst }
        } else {
            Fusion::Skipped
        }
    }

    /// GPS north/east position, m, relative to the alignment point.
    pub fn fuse_gps_position_ne(&mut self, ne: [T; 2], sigma_m: T) -> Fusion<T> {
        let gate = self.prm.gps_gate;
        self.fuse_vector([IP, IP + 1], ne, sigma_m, gate, |x| [x.p.x, x.p.y])
    }

    /// GPS NED velocity, m/s.
    pub fn fuse_gps_velocity(&mut self, v: Vec3<T>, sigma_mps: T) -> Fusion<T> {
        let gate = self.prm.gps_gate;
        self.fuse_vector(
            [IV, IV + 1, IV + 2],
            [v.x, v.y, v.z],
            sigma_mps,
            gate,
            |x| [x.v.x, x.v.y, x.v.z],
        )
    }

    /// Height above the alignment point, positive up (model: `h = −p.z`).
    pub fn fuse_height(&mut self, h_up: T, sigma_m: T) -> Fusion<T> {
        let nu = h_up + self.x.p.z;
        let gate = self.prm.baro_gate;
        self.fuse_scalar(&[(IP + 2, -T::ONE)], nu, sigma_m, gate)
    }

    /// Heading innovation and sparse `H` for a body-frame field; `None`
    /// when the geometry is unusable.
    ///
    /// The measurement is the horizontal direction of the field rotated
    /// into the world frame with the estimated attitude, `m_w = R(q)·m_body`,
    /// which should point along the declination: `ν = D − atan2(m_w.y,
    /// m_w.x)`. With the true field `B = Exp(δθ)·m_w ≈ m_w + δθ × m_w`,
    /// to first order `ν = δθz − m_z·(m_x·δθx + m_y·δθy)/(m_x² + m_y²)`.
    /// The tilt terms matter: a tilt error leaks into the heading in
    /// proportion to the field's dip (×1.7 at 60° inclination). Treating
    /// the innovation as a pure yaw error makes the filter overconfident
    /// in yaw.
    ///
    /// **Where `H` is evaluated.** An EKF linearises its measurement at the
    /// state estimate, so the tilt entries are taken at the field the
    /// filter predicts, not at the measured one: the measured world field
    /// turned about Down until its horizontal part points along the
    /// declination `D`, keeping the measured dip `m_z/h`. That gives
    /// `H = (−cos D·dip, −sin D·dip, 1)` for the three attitude errors.
    /// Built from the measured field, `H_E = −m_y·m_z/h²` is about
    /// `tan(I)·ν` for a field pointing north (`D = 0`; the cross-field part
    /// of the noise is `m_y`), so the gain depends on the innovation and
    /// every update adds a correction of non-zero mean about the horizontal
    /// axis perpendicular to the field (`tan(I)·P_EE·ν²/S` at `D = 0`). In hover that axis cannot be told from
    /// the accelerometer bias, which it drives. Where `ν = 0` the two forms
    /// are equal; the innovation itself is unchanged.
    fn heading_model(&self, mag_body: Vec3<T>) -> Option<(T, [(usize, T); 3])> {
        if !mag_body.is_finite() {
            return None;
        }
        let r = rot_matrix(self.x.q);
        let mw = mat3_vec(&r, mag_body);
        let h2 = mw.x * mw.x + mw.y * mw.y;
        let total = mag_body.norm();
        if !pos(total) || !at_least(h2.sqrt(), T::from_f64(0.1) * total) {
            return None;
        }
        let nu = wrap_pi(self.prm.declination - mw.y.atan2(mw.x));
        let dip = mw.z / h2.sqrt();
        let h = [
            (ITH, -self.prm.decl_cos * dip),
            (ITH + 1, -self.prm.decl_sin * dip),
            (ITH + 2, T::ONE),
        ];
        Some((nu, h))
    }

    /// Magnetic heading from a body-frame field (only its direction
    /// matters).
    pub fn fuse_mag_heading(&mut self, mag_body: Vec3<T>, sigma_rad: T) -> Fusion<T> {
        let Some((nu, h)) = self.heading_model(mag_body) else {
            return Fusion::Skipped;
        };
        let gate = self.prm.yaw_gate;
        self.fuse_scalar(&h, nu, sigma_rad, gate)
    }

    /// The heading the magnetometer implies given the current tilt: the
    /// current yaw plus the heading innovation. `None` when the geometry is
    /// unusable.
    pub fn mag_heading(&self, mag_body: Vec3<T>) -> Option<T> {
        let (nu, _) = self.heading_model(mag_body)?;
        let psi = wrap_pi(self.x.q.yaw() + nu);
        psi.is_finite().then_some(psi)
    }

    /// Rotates the attitude about the world down axis so its yaw becomes
    /// `yaw` (tilt unchanged), sets the heading variance (error index
    /// `ITH + 2`) to `sigma²` and zeroes its cross-covariances. Returns the
    /// yaw change applied (zero if an input is not finite).
    pub fn reset_yaw(&mut self, yaw: T, sigma_rad: T) -> T {
        if !yaw.is_finite() {
            return T::ZERO;
        }
        let dpsi = wrap_pi(yaw - self.x.q.yaw());
        if !dpsi.is_finite() {
            return T::ZERO;
        }
        self.x.q =
            (Quat::from_rotation_vector(Vec3::new(T::ZERO, T::ZERO, dpsi)) * self.x.q).normalized();
        self.reset_cov(&[ITH + 2], sigma_rad);
        dpsi
    }

    /// Snaps north/east to `ne`, sets their variance to `sigma²` and zeroes
    /// their cross-covariances. Returns the position change.
    pub fn reset_position_ne(&mut self, ne: [T; 2], sigma_m: T) -> Vec3<T> {
        let new = Vec3::new(
            ne.first().copied().unwrap_or(self.x.p.x),
            ne.get(1).copied().unwrap_or(self.x.p.y),
            self.x.p.z,
        );
        if !new.is_finite() {
            return Vec3::zero();
        }
        let delta = new - self.x.p;
        self.x.p = new;
        self.reset_cov(&[IP, IP + 1], sigma_m);
        delta
    }

    /// Same for velocity. Returns the velocity change.
    pub fn reset_velocity(&mut self, v: Vec3<T>, sigma_mps: T) -> Vec3<T> {
        if !v.is_finite() {
            return Vec3::zero();
        }
        let delta = v - self.x.v;
        self.x.v = v;
        self.reset_cov(&[IV, IV + 1, IV + 2], sigma_mps);
        delta
    }

    /// Sets the roll and pitch error variances (`ITH`, `ITH + 1`) back to
    /// their initial value and zeroes their cross-covariances: the heading
    /// check's correction does this so that the GPS velocity can undo within
    /// seconds the tilt error a wrong heading forced into the filter.
    pub fn reset_tilt_variance(&mut self) {
        let sigma = self.prm.init_tilt;
        self.reset_cov(&[ITH, ITH + 1], sigma);
    }

    /// Adds `dpsi` (rad) to the declination the heading model uses, with its
    /// cosine and sine: after the heading was turned by `dpsi` the field's
    /// direction in the world frame turned by `-dpsi`, and the model's
    /// declination must follow so that the magnetometer innovation stays
    /// about zero. The parameter `EKF_MAG_DECL` is not touched. A value that
    /// is not finite is ignored.
    pub fn shift_declination(&mut self, dpsi: T) {
        if !dpsi.is_finite() {
            return;
        }
        let d = wrap_pi(self.prm.declination + dpsi);
        self.prm.declination = d;
        self.prm.decl_cos = d.cos();
        self.prm.decl_sin = d.sin();
    }

    /// The declination the heading model uses now, rad.
    pub fn declination(&self) -> T {
        self.prm.declination
    }

    fn reset_cov(&mut self, idx: &[usize], sigma: T) {
        let var = (sigma * sigma).finite_or(T::ONE);
        for &i in idx {
            for k in 0..N_ERR {
                self.p.set(i, k, T::ZERO);
                self.p.set(k, i, T::ZERO);
            }
            self.p.set(i, i, var);
        }
        self.condition();
    }
}

/// Tilt from a mean specific force at rest (body): `(roll, pitch)` in rad.
/// `None` if not finite or `|a| < 1e-3`.
pub fn tilt_from_accel<T: Real>(accel: Vec3<T>) -> Option<(T, T)> {
    if !accel.is_finite() || !at_least(accel.norm(), T::from_f64(1e-3)) {
        return None;
    }
    let roll = (-accel.y).atan2(-accel.z);
    let pitch = accel
        .x
        .atan2((accel.y * accel.y + accel.z * accel.z).sqrt());
    Some((roll, pitch))
}

/// Heading from a body-frame field: `ψ = declination − atan2(m_w.y, m_w.x)`
/// with `m_w = R(roll, pitch, 0)·m_body`. `None` if the horizontal part is
/// under 10 % of `|m|` or the input is not finite.
pub fn heading_from_mag<T: Real>(
    mag_body: Vec3<T>,
    roll: T,
    pitch: T,
    declination: T,
) -> Option<T> {
    if !mag_body.is_finite() || !roll.is_finite() || !pitch.is_finite() {
        return None;
    }
    let mw = Quat::from_euler(roll, pitch, T::ZERO).rotate(mag_body);
    let horiz = (mw.x * mw.x + mw.y * mw.y).sqrt();
    if !at_least(horiz, T::from_f64(0.1) * mag_body.norm()) {
        return None;
    }
    let psi = wrap_pi(declination - mw.y.atan2(mw.x));
    psi.is_finite().then_some(psi)
}

/// Attitude from the mean accelerometer and optional mean magnetometer
/// reading at rest (yaw 0 without a usable magnetometer).
pub fn initial_attitude<T: Real>(
    accel: Vec3<T>,
    mag: Option<Vec3<T>>,
    declination: T,
) -> Option<Quat<T>> {
    let (roll, pitch) = tilt_from_accel(accel)?;
    let yaw = mag
        .and_then(|m| heading_from_mag(m, roll, pitch, declination))
        .unwrap_or(T::ZERO);
    Some(Quat::from_euler(roll, pitch, yaw))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::time::Micros;
    use proptest::prelude::*;

    /// xorshift64* in [0, 1).
    struct Lcg(u64);
    impl Lcg {
        fn next(&mut self) -> f64 {
            self.0 ^= self.0 >> 12;
            self.0 ^= self.0 << 25;
            self.0 ^= self.0 >> 27;
            let v = self.0.wrapping_mul(0x2545_F491_4F6C_DD1D);
            (v >> 11) as f64 / (1u64 << 53) as f64
        }
        fn range(&mut self, lo: f64, hi: f64) -> f64 {
            lo + (hi - lo) * self.next()
        }
        fn vec(&mut self, lim: f64) -> Vec3<f64> {
            Vec3::new(
                self.range(-lim, lim),
                self.range(-lim, lim),
                self.range(-lim, lim),
            )
        }
    }

    fn delta<T: Real>(dt: f64, dang: Vec3<f64>, dvel: Vec3<f64>) -> ImuDelta<T> {
        ImuDelta {
            t_end: Micros::default(),
            dt: T::from_f64(dt),
            dang: dang.cast(),
            dvel: dvel.cast(),
        }
    }

    const B_NED: Vec3<f64> = Vec3::new(25.0, 0.0, 43.30);

    fn err_vec(a: &NominalState<f64>, b: &NominalState<f64>, denom: f64) -> [f64; N_ERR] {
        let dth = (a.q * b.q.conj()).to_rotation_vector();
        let (dp, dv) = (a.p - b.p, a.v - b.v);
        let (dbg, dba) = (a.bg - b.bg, a.ba - b.ba);
        let mut out = [0.0; N_ERR];
        for (o, v) in out.iter_mut().zip([
            dp.x, dp.y, dp.z, dv.x, dv.y, dv.z, dth.x, dth.y, dth.z, dbg.x, dbg.y, dbg.z, dba.x,
            dba.y, dba.z,
        ]) {
            *o = v / denom;
        }
        out
    }

    #[test]
    fn predict_jacobian_matches_finite_differences() {
        let mut rng = Lcg(0x1234_5678_9abc_def1);
        let eps = 1e-6;
        for case in 0..40 {
            let tilt = rng.range(0.0, 60f64.to_radians());
            let axis = Vec3::new(rng.range(-1.0, 1.0), rng.range(-1.0, 1.0), 0.0);
            let q = Quat::from_axis_angle(axis, tilt)
                * Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), rng.range(-3.0, 3.0));
            let mut ekf = Ekf::<f64>::new(&EkfParams::default(), q);
            ekf.x.v = rng.vec(5.7);
            ekf.x.p = rng.vec(50.0);
            ekf.x.bg = rng.vec(0.075);
            ekf.x.ba = rng.vec(0.2);
            // Rates up to 5 rad/s per axis, specific force up to ~30 m/s².
            let d = delta::<f64>(0.004, rng.vec(5.0) * 0.004, rng.vec(17.0) * 0.004);
            let (_, f, _) = ekf.transition(&d).unwrap();
            for j in 0..N_ERR {
                let mut dx = [0.0; N_ERR];
                dx[j] = eps;
                let xp = Ekf {
                    x: injected(&ekf.x, &dx),
                    ..ekf
                };
                dx[j] = -eps;
                let xm = Ekf {
                    x: injected(&ekf.x, &dx),
                    ..ekf
                };
                let (np, _, _) = xp.transition(&d).unwrap();
                let (nm, _, _) = xm.transition(&d).unwrap();
                let col = err_vec(&np, &nm, 2.0 * eps);
                for (i, c) in col.iter().enumerate() {
                    let fij = f.get(i, j);
                    assert!(
                        (c - fij).abs() < 1e-6,
                        "case {case}: F[{i},{j}] analytic {fij} numeric {c}"
                    );
                }
            }
        }
    }

    #[test]
    fn heading_jacobian_matches_finite_differences() {
        let mut rng = Lcg(77);
        let eps = 1e-6;
        for _ in 0..20 {
            let q = Quat::from_euler(
                rng.range(-0.8, 0.8),
                rng.range(-0.8, 0.8),
                rng.range(-3.1, 3.1),
            );
            let ekf = Ekf::<f64>::new(&EkfParams::default(), q);
            let mag = q.inverse_rotate(B_NED);
            let (nu, h) = ekf.heading_model(mag).unwrap();
            assert!(nu.abs() < 1e-9, "consistent field gives zero innovation");
            // The innovation the filter sees when the true attitude is
            // `Exp(δθ) ⊗ q` (the error-state convention) and the sensor
            // reads the field in the true body frame.
            let nu_for = |dth: Vec3<f64>| {
                let q_true = Quat::from_rotation_vector(dth) * q;
                ekf.heading_model(q_true.inverse_rotate(B_NED)).unwrap().0
            };
            for (k, &(idx, hk)) in h.iter().enumerate() {
                assert_eq!(idx, ITH + k);
                let mut e = [0.0; 3];
                e[k] = eps;
                let d = Vec3::new(e[0], e[1], e[2]);
                let num = (nu_for(d) - nu_for(-d)) / (2.0 * eps);
                assert!(
                    (num - hk).abs() < 1e-6,
                    "H[{k}] analytic {hk} numeric {num}"
                );
            }
            // A pure yaw error shows up one to one.
            assert!((nu_for(Vec3::new(0.0, 0.0, 3e-3)) - 3e-3).abs() < 1e-5);
        }
        // The dip of the field couples tilt into heading: with the field
        // 60° below the horizon, a tilt error about north reads as a
        // heading error tan(60°) ≈ 1.73 times as large, opposite in sign.
        let ekf = Ekf::<f64>::new(&EkfParams::default(), Quat::identity());
        let (_, h) = ekf.heading_model(B_NED).unwrap();
        assert!((h[0].1 + 43.30 / 25.0).abs() < 1e-12 && h[1].1.abs() < 1e-12);
    }

    #[test]
    fn initial_attitude_recovers_euler() {
        let (roll, pitch, yaw) = (5f64.to_radians(), (-3f64).to_radians(), 40f64.to_radians());
        let q = Quat::from_euler(roll, pitch, yaw);
        let accel = q.inverse_rotate(Vec3::new(0.0, 0.0, -9.80665));
        let mag = q.inverse_rotate(B_NED);
        let got = initial_attitude(accel, Some(mag), 0.0).unwrap().to_euler();
        assert!((got.x - roll).abs() < 1e-9);
        assert!((got.y - pitch).abs() < 1e-9);
        assert!((got.z - yaw).abs() < 1e-9);
        let nomag = initial_attitude(accel, None, 0.0).unwrap().to_euler();
        assert!((nomag.x - roll).abs() < 1e-9 && (nomag.y - pitch).abs() < 1e-9);
        assert!(nomag.z.abs() < 1e-12);
        // Declination shifts the heading.
        let d: f64 = 0.2;
        let m = q.inverse_rotate(Vec3::new(25.0 * Real::cos(d), 25.0 * Real::sin(d), 43.3));
        let yaw_d = initial_attitude(accel, Some(m), d).unwrap().to_euler().z;
        assert!((yaw_d - yaw).abs() < 1e-9);
        assert!(initial_attitude(Vec3::<f64>::zero(), Some(mag), 0.0).is_none());
        assert!(initial_attitude(Vec3::new(f64::NAN, 0.0, -9.8), None, 0.0).is_none());
        // A field pointing straight down has no heading.
        assert!(heading_from_mag(Vec3::new(0.0, 0.0, 50.0), 0.0, 0.0, 0.0).is_none());
    }

    #[test]
    fn biases_converge_when_stationary() {
        let params = EkfParams::default();
        let q = Quat::<f32>::from_euler(0.05, -0.03, 0.7);
        let mut ekf = Ekf::<f32>::new(&params, q);
        let bg_true = Vec3::new(0.01_f32, -0.008, 0.005);
        let ba_true = Vec3::new(0.0_f32, 0.0, 0.1);
        let dt = 0.004_f32;
        let f_body = q.inverse_rotate(Vec3::new(0.0, 0.0, -9.80665)) + ba_true;
        let mag = q.inverse_rotate(Vec3::new(25.0, 0.0, 43.30));
        let d = ImuDelta {
            t_end: Micros::default(),
            dt,
            dang: bg_true * dt,
            dvel: f_body * dt,
        };
        for k in 1..=15_000_u32 {
            ekf.predict(&d);
            if k % 50 == 0 {
                ekf.fuse_gps_position_ne([0.0, 0.0], 0.5);
                ekf.fuse_gps_velocity(Vec3::zero(), 0.3);
            }
            if k % 5 == 0 {
                ekf.fuse_height(0.0, 0.5);
                ekf.fuse_mag_heading(mag, 0.15);
            }
        }
        let e = ekf.x.bg - bg_true;
        assert!(
            e.x.abs() < 0.002 && e.y.abs() < 0.002 && e.z.abs() < 0.002,
            "gyro bias error {e:?}"
        );
        assert!(
            (ekf.x.ba.z - 0.1).abs() < 0.03,
            "accel bias z {}",
            ekf.x.ba.z
        );
        assert_eq!(ekf.faults(), 0);
    }

    #[test]
    fn gross_outliers_are_gated() {
        let mut ekf = Ekf::<f32>::new(&EkfParams::default(), Quat::identity());
        let before = (ekf.x, ekf.p);
        let r = ekf.fuse_gps_position_ne([100.0, 0.0], 0.5);
        assert!(matches!(r, Fusion::Rejected { test_ratio } if test_ratio > 1.0));
        assert_eq!(ekf.x, before.0);
        assert_eq!(ekf.p, before.1);
        let r = ekf.fuse_gps_velocity(Vec3::new(0.0, 0.0, 50.0), 0.3);
        assert!(matches!(r, Fusion::Rejected { .. }));
        assert_eq!(ekf.x, before.0);
        let r = ekf.fuse_height(80.0, 0.5);
        assert!(matches!(r, Fusion::Rejected { .. }));
        let r = ekf.fuse_gps_position_ne([0.5, -0.5], 0.5);
        assert!(matches!(r, Fusion::Accepted { test_ratio } if test_ratio <= 1.0));
        assert!(ekf.x.p.x > 0.0 && ekf.x.p.y < 0.0);
        // Garbage in, nothing out.
        assert_eq!(ekf.fuse_height(f32::NAN, 0.5), Fusion::Skipped);
        assert_eq!(ekf.fuse_height(0.0, 0.0), Fusion::Skipped);
        assert_eq!(ekf.fuse_mag_heading(Vec3::zero(), 0.1), Fusion::Skipped);
        assert_eq!(
            ekf.fuse_mag_heading(Vec3::new(0.0, 0.0, 40.0), 0.1),
            Fusion::Skipped
        );
    }

    #[test]
    fn predict_ignores_bad_deltas() {
        let mut ekf = Ekf::<f32>::new(&EkfParams::default(), Quat::identity());
        let (x, p) = (ekf.x, ekf.p);
        ekf.predict(&delta::<f32>(0.0, Vec3::zero(), Vec3::zero()));
        ekf.predict(&delta::<f32>(-0.004, Vec3::zero(), Vec3::zero()));
        ekf.predict(&delta::<f32>(
            0.004,
            Vec3::new(f64::NAN, 0.0, 0.0),
            Vec3::zero(),
        ));
        assert_eq!(ekf.x, x);
        assert_eq!(ekf.p, p);
    }

    #[test]
    fn resets_move_state_and_decouple_covariance() {
        let mut ekf = Ekf::<f32>::new(&EkfParams::default(), Quat::identity());
        let d = delta::<f32>(0.004, Vec3::new(0.01, 0.0, 0.0), Vec3::new(0.0, 0.0, -0.04));
        for _ in 0..50 {
            ekf.predict(&d);
        }
        let ch = ekf.reset_position_ne([10.0, -4.0], 2.0);
        assert!((ekf.x.p.x - 10.0).abs() < 1e-6 && (ekf.x.p.y + 4.0).abs() < 1e-6);
        assert!((ch.x - 10.0).abs() < 1e-3);
        assert_eq!(ekf.p.get(0, 0), 4.0);
        assert_eq!(ekf.p.get(1, 1), 4.0);
        assert_eq!(ekf.p.get(0, 4), 0.0);
        assert_eq!(ekf.p.get(4, 0), 0.0);
        let ch = ekf.reset_velocity(Vec3::new(1.0, 2.0, 3.0), 0.5);
        assert_eq!(ekf.x.v, Vec3::new(1.0, 2.0, 3.0));
        assert!(ch.is_finite());
        assert_eq!(ekf.p.get(3, 3), 0.25);
        assert_eq!(ekf.p.get(3, 7), 0.0);
    }

    /// The four process-noise parameters with one scale applied to every
    /// sigma, in the order accel, gyro, gyro bias, accel bias.
    fn noise_params(accel: f32, gyro: f32, gyro_bias: f32, accel_bias: f32) -> EkfParams {
        EkfParams {
            accel_noise: accel,
            gyro_noise: gyro,
            gyro_bias_noise: gyro_bias,
            accel_bias_noise: accel_bias,
            ..EkfParams::default()
        }
    }

    /// Covariance after `n` predictions of `dt` each, at rest, from the same
    /// initial covariance, with no measurement fused.
    fn grown(params: &EkfParams, dt: f64, n: usize) -> Cov<f64> {
        let q = Quat::<f64>::from_euler(0.05, -0.03, 0.7);
        let mut ekf = Ekf::<f64>::new(params, q);
        let f = q.inverse_rotate(Vec3::new(0.0, 0.0, -G));
        let d = delta::<f64>(dt, Vec3::zero(), f * dt);
        for _ in 0..n {
            ekf.predict(&d);
        }
        ekf.p
    }

    /// The covariance the noise terms alone add: the grown covariance minus
    /// the one grown with all four noises zero.
    fn noise_added(params: &EkfParams, dt: f64, n: usize) -> Cov<f64> {
        let with = grown(params, dt, n);
        let quiet = EkfParams {
            accel_noise: 0.0,
            gyro_noise: 0.0,
            gyro_bias_noise: 0.0,
            accel_bias_noise: 0.0,
            ..*params
        };
        let without = grown(&quiet, dt, n);
        let mut out = Cov::zeros();
        for i in 0..N_ERR {
            for j in 0..N_ERR {
                out.set(i, j, with.get(i, j) - without.get(i, j));
            }
        }
        out
    }

    #[test]
    fn covariance_growth_per_second_does_not_depend_on_the_filter_step() {
        // The variance the noise terms add over one second (the grown
        // covariance minus the noiseless one) at 2.5, 4 and 10 ms (400, 250,
        // 100 steps). The noise sums to sigma^2 * 0.01 per second in every
        // case. The initial variances are above the floors of VAR_LIMITS, so
        // no clamp acts and the two runs differ only by the noise. What still
        // differs is the first-order transition matrix, which feeds attitude
        // variance into velocity by terms of order dt / (1 s): at most 6e-4
        // for velocity and 2e-5 for attitude, so 1e-3 and 1e-4 are asserted.
        // The bias groups have identity blocks in F and agree to rounding.
        // Before the fix the groups differed by the step ratio, 4x between
        // 2.5 and 10 ms.
        let params = EkfParams {
            init_tilt_sigma_rad: 1e-3,
            init_yaw_sigma_rad: 1e-3,
            init_vel_sigma_mps: 1e-2,
            init_pos_sigma_m: 1e-2,
            init_gyro_bias_sigma: 1e-4,
            init_accel_bias_sigma: 1e-3,
            ..EkfParams::default()
        };
        let steps = [(0.0025, 400), (0.004, 250), (0.01, 100)];
        let grids: Vec<Cov<f64>> = steps
            .iter()
            .map(|&(dt, n)| noise_added(&params, dt, n))
            .collect();
        for (s, tol) in [(IV, 1e-3), (ITH, 1e-4), (IBG, 1e-9), (IBA, 1e-9)] {
            for i in s..s + 3 {
                let reference = grids[2].get(i, i);
                for (g, &(dt, _)) in grids.iter().zip(&steps) {
                    let rel = (g.get(i, i) - reference).abs() / reference;
                    assert!(rel < tol, "P[{i}][{i}] at dt = {dt}: relative {rel}");
                }
            }
        }
    }

    #[test]
    fn one_step_at_ten_ms_adds_px4s_variance() {
        // PX4 EKF2 adds (sigma * dt)^2 per 10 ms step. At dt = 10 ms the
        // variance added here is exactly that, for each of the four
        // parameters and on the three axes of each, and nothing else moves.
        // (Tolerance 1e-6 relative: the difference of two f64 covariances
        // carries their rounding, about 5e-10 of the smallest variance.)
        let params = EkfParams::default();
        // A step of 4 ms adds 0.4 of it, not (sigma * 0.004)^2 = 0.16 of it.
        let four = noise_added(&params, 0.004, 1);
        for (s, sigma) in [
            (IV, params.accel_noise),
            (ITH, params.gyro_noise),
            (IBG, params.gyro_bias_noise),
            (IBA, params.accel_bias_noise),
        ] {
            let want = f64::from(sigma) * f64::from(sigma) * 0.004 * 0.01;
            assert!(
                (four.get(s, s) - want).abs() <= 1e-6 * want,
                "4 ms step at {s}: {:e}, want {want:e}",
                four.get(s, s)
            );
        }
        let a = noise_added(&params, 0.01, 1);
        let want = |sigma: f32| (f64::from(sigma) * 0.01) * (f64::from(sigma) * 0.01);
        let groups = [
            (IV, want(params.accel_noise)),
            (ITH, want(params.gyro_noise)),
            (IBG, want(params.gyro_bias_noise)),
            (IBA, want(params.accel_bias_noise)),
        ];
        for i in 0..N_ERR {
            for j in 0..N_ERR {
                let expected = groups
                    .iter()
                    .find(|&&(s, _)| i == j && (s..s + 3).contains(&i))
                    .map_or(0.0, |&(_, v)| v);
                let got = a.get(i, j);
                assert!(
                    (got - expected).abs() <= 1e-6 * expected.max(1e-12),
                    "added[{i}][{j}] = {got:e}, want {expected:e}"
                );
            }
        }
    }

    #[test]
    fn added_variance_is_linear_in_the_step_and_quadratic_in_sigma() {
        let params = EkfParams::default();
        let ten = noise_added(&params, 0.01, 1);
        let four = noise_added(&params, 0.004, 1);
        let double = noise_added(
            &noise_params(
                2.0 * params.accel_noise,
                2.0 * params.gyro_noise,
                2.0 * params.gyro_bias_noise,
                2.0 * params.accel_bias_noise,
            ),
            0.01,
            1,
        );
        for s in [IV, ITH, IBG, IBA] {
            let t = ten.get(s, s);
            assert!((four.get(s, s) / t - 0.4).abs() < 1e-6, "dt scaling at {s}");
            assert!(
                (double.get(s, s) / t - 4.0).abs() < 1e-6,
                "sigma scaling at {s}"
            );
        }
    }

    #[test]
    fn nonfinite_covariance_is_reset() {
        let mut ekf = Ekf::<f32>::new(&EkfParams::default(), Quat::identity());
        ekf.p.set(2, 2, f32::NAN);
        ekf.predict(&delta::<f32>(
            0.004,
            Vec3::zero(),
            Vec3::new(0.0, 0.0, -0.04),
        ));
        assert_eq!(ekf.faults(), 1);
        assert!(ekf.p.is_finite());
    }

    fn check_healthy(ekf: &Ekf<f32>) -> Result<(), TestCaseError> {
        prop_assert!(ekf.x.q.is_finite() && ekf.x.v.is_finite() && ekf.x.p.is_finite());
        prop_assert!(ekf.x.bg.is_finite() && ekf.x.ba.is_finite());
        prop_assert!(ekf.p.is_finite());
        prop_assert!((ekf.x.q.norm() - 1.0).abs() < 1e-3);
        for i in 0..N_ERR {
            prop_assert!(ekf.p.get(i, i) > 0.0);
            for j in 0..N_ERR {
                prop_assert_eq!(ekf.p.get(i, j), ekf.p.get(j, i));
            }
        }
        Ok(())
    }

    proptest! {
        #[test]
        fn state_and_covariance_stay_sane(
            ops in proptest::collection::vec(
                (0u8..5, 0.0005f64..0.02, -8.0f64..8.0, -8.0f64..8.0, -8.0f64..8.0, -40.0f64..40.0, -40.0f64..40.0),
                1..80,
            )
        ) {
            let mut ekf = Ekf::<f32>::new(&EkfParams::default(), Quat::from_euler(0.1, 0.2, 0.3));
            for (kind, dt, a, b, c, d, e) in ops {
                match kind {
                    0 | 1 => ekf.predict(&delta::<f32>(
                        dt,
                        Vec3::new(a, b, c) * dt,
                        Vec3::new(d, e, -9.8) * dt,
                    )),
                    2 => {
                        ekf.fuse_gps_position_ne([a as f32 * 3.0, b as f32 * 3.0], 0.5);
                        ekf.fuse_gps_velocity(Vec3::new(a, b, c * 0.3).cast(), 0.3);
                    }
                    3 => {
                        ekf.fuse_height(d as f32, 0.5);
                    }
                    _ => {
                        ekf.fuse_mag_heading(Vec3::new(d, e, a * 5.0).cast(), 0.15);
                    }
                }
                check_healthy(&ekf)?;
            }
        }
    }

    /// Standard normal draws from the test generator (Box-Muller).
    fn normal(rng: &mut Lcg) -> f64 {
        let (a, b) = (1.0 - rng.next(), rng.next());
        Real::sqrt(-2.0 * Real::ln(a)) * Real::cos(2.0 * core::f64::consts::PI * b)
    }

    fn normal3(rng: &mut Lcg, sigma: f64) -> Vec3<f64> {
        Vec3::new(
            sigma * normal(rng),
            sigma * normal(rng),
            sigma * normal(rng),
        )
    }

    /// A filter whose attitude is exactly `q` and whose covariance is the
    /// one after `secs` seconds at rest at `q` with the simulator's sensor
    /// noise (gyro, accelerometer, barometer, GPS and magnetometer 0.5 µT
    /// per axis): a realistic covariance at a known true state.
    fn settled_filter(q: Quat<f64>, secs: u32, seed: u64) -> Ekf<f64> {
        let mut rng = Lcg(seed);
        let mut ekf = Ekf::<f64>::new(&EkfParams::default(), q);
        let dt = 0.004;
        let f_body = q.inverse_rotate(Vec3::new(0.0, 0.0, -9.80665));
        let mag = q.inverse_rotate(B_NED);
        for k in 1..=(secs * 250) {
            let d = ImuDelta::<f64> {
                t_end: Micros::default(),
                dt,
                dang: normal3(&mut rng, 8.7e-5 * Real::sqrt(dt)),
                dvel: f_body * dt + normal3(&mut rng, 3.9e-3 * Real::sqrt(dt)),
            };
            ekf.predict(&d);
            if k % 5 == 0 {
                ekf.fuse_height(0.1 * normal(&mut rng), 0.5);
                ekf.fuse_mag_heading(mag + normal3(&mut rng, 0.5), 0.15);
            }
            if k % 50 == 0 {
                let n = normal3(&mut rng, 0.3);
                ekf.fuse_gps_position_ne([n.x, n.y], 0.6);
                ekf.fuse_gps_velocity(normal3(&mut rng, 0.1), 0.3);
            }
        }
        let mut at_truth = Ekf::<f64>::new(&EkfParams::default(), q);
        at_truth.p = ekf.p;
        at_truth
    }

    /// Mean and standard error of the world-frame tilt correction (about
    /// North and East) of one `fuse_mag_heading` update at a fixed true
    /// state (estimate equals truth, so ν is pure sensor noise; in f64, the f32 core
    /// gives the same means to 3 digits), over
    /// `n` noisy readings (0.5 µT per body axis on the 50 µT field).
    fn mean_tilt_correction(ekf: &Ekf<f64>, q: Quat<f64>, n: u32, seed: u64) -> [(f64, f64); 2] {
        let mut rng = Lcg(seed);
        let b_body = q.inverse_rotate(B_NED);
        let (mut s1, mut s2) = ([0.0f64; 2], [0.0f64; 2]);
        for _ in 0..n {
            let mut e = *ekf;
            let m = b_body + normal3(&mut rng, 0.5);
            assert!(matches!(
                e.fuse_mag_heading(m, 0.15),
                Fusion::Accepted { .. }
            ));
            let dth = (e.x.q * ekf.x.q.conj()).to_rotation_vector();
            for (i, v) in [dth.x, dth.y].into_iter().enumerate() {
                s1[i] += v;
                s2[i] += v * v;
            }
        }
        let nf = f64::from(n);
        let out = |i: usize| {
            let mean = s1[i] / nf;
            let var = (s2[i] / nf - mean * mean).max(0.0);
            (mean, Real::sqrt(var / nf))
        };
        [out(0), out(1)]
    }

    /// At a fixed true state the heading update has no mean tilt
    /// correction. With the Jacobian built from the noisy measured field
    /// (the code before this fix) the mean about East was +3.46e-6 rad per
    /// update here, about 450 standard errors (446 measured). Bound: each mean is within 4
    /// standard errors of zero (fixed seeds, so the test is deterministic).
    #[test]
    fn heading_update_has_no_mean_tilt_correction() {
        let q0 = Quat::<f64>::identity();
        let ekf = settled_filter(q0, 100, 1);
        let [(mn, sn), (me, se)] = mean_tilt_correction(&ekf, q0, 400_000, 42);
        assert!(sn > 0.0 && se > 0.0);
        assert!(
            mn.abs() < 4.0 * sn && me.abs() < 4.0 * se,
            "mean tilt correction N {mn:e} ± {sn:e}, E {me:e} ± {se:e}"
        );
        // A tilted, rotated attitude: the biased axis would turn with the field.
        let q1 = Quat::<f64>::from_euler(5f64.to_radians(), (-3f64).to_radians(), 0.7);
        let ekf = settled_filter(q1, 100, 2);
        let [(mn, sn), (me, se)] = mean_tilt_correction(&ekf, q1, 400_000, 43);
        assert!(
            mn.abs() < 4.0 * sn && me.abs() < 4.0 * se,
            "tilted: mean tilt correction N {mn:e} ± {sn:e}, E {me:e} ± {se:e}"
        );
    }

    /// The tilt entries of H are taken at the predicted field: turning the
    /// measured world field about Down (as heading noise does) leaves them
    /// unchanged, for zero and non-zero declination.
    #[test]
    fn heading_jacobian_tilt_entries_ignore_field_rotation_about_down() {
        let mut rng = Lcg(5);
        for decl in [0.0, 0.3, -2.0] {
            let prm = EkfParams {
                mag_declination_rad: decl as f32,
                ..EkfParams::default()
            };
            for _ in 0..20 {
                let q = Quat::from_euler(
                    rng.range(-0.8, 0.8),
                    rng.range(-0.8, 0.8),
                    rng.range(-3.1, 3.1),
                );
                let ekf = Ekf::<f64>::new(&prm, q);
                let rd = Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), decl);
                // A field with some heading error and a noisy dip.
                let m_w = Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), rng.range(-0.2, 0.2))
                    .rotate(rd.rotate(B_NED) + rng.vec(0.5));
                let (_, h0) = ekf.heading_model(q.inverse_rotate(m_w)).unwrap();
                for _ in 0..5 {
                    let phi = rng.range(-1.0, 1.0);
                    let turned = Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), phi).rotate(m_w);
                    let (nu, h) = ekf.heading_model(q.inverse_rotate(turned)).unwrap();
                    assert!(nu.is_finite());
                    for k in 0..2 {
                        assert!(
                            (h[k].1 - h0[k].1).abs() < 1e-9,
                            "decl {decl}: H[{k}] {} after a turn of {phi}, was {}",
                            h[k].1,
                            h0[k].1
                        );
                    }
                    assert_eq!(h[2].1, 1.0);
                }
            }
        }
    }

    /// At a consistent field (ν = 0) H equals the exact first-order
    /// expression `−m_x·m_z/h²`, `−m_y·m_z/h²` evaluated at the world
    /// field, also for a non-zero declination (the finite-difference test
    /// above runs at declination 0).
    #[test]
    fn heading_jacobian_at_consistent_field_is_the_measured_field_form() {
        let mut rng = Lcg(6);
        for decl in [0.0, 0.3, -1.2, 3.0] {
            let prm = EkfParams {
                mag_declination_rad: decl as f32,
                ..EkfParams::default()
            };
            let decl = f64::from(prm.mag_declination_rad);
            for k in 0..60 {
                let q = Quat::from_euler(
                    rng.range(-0.8, 0.8),
                    rng.range(-0.8, 0.8),
                    rng.range(-3.1, 3.1),
                );
                let ekf = Ekf::<f64>::new(&prm, q);
                // The dip comes from the measurement: other inclinations
                // than the simulator's 60 degrees.
                let incl = [60.0f64, 10.0, 45.0, 80.0, -30.0][k % 5].to_radians();
                let b0 = Vec3::new(50.0 * Real::cos(incl), 0.0, 50.0 * Real::sin(incl));
                let b = Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), decl).rotate(b0);
                let mw = q.rotate(q.inverse_rotate(b));
                let h2 = mw.x * mw.x + mw.y * mw.y;
                let (nu, h) = ekf.heading_model(q.inverse_rotate(b)).unwrap();
                assert!(nu.abs() < 1e-9);
                assert!((h[0].1 + mw.x * mw.z / h2).abs() < 1e-9, "decl {decl}");
                assert!((h[1].1 + mw.y * mw.z / h2).abs() < 1e-9, "decl {decl}");
            }
        }
    }
}
