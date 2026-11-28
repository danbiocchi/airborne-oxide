// SPDX-License-Identifier: AGPL-3.0-or-later
//! A check of the filter's heading against the vehicle's own motion
//! (heading-guard design D2 sections 2.1 and 2.2; stage 1, piece P1).
//!
//! If the filter's heading is wrong by `eps`, the velocity change the
//! filter integrates from the accelerometer between two GPS fixes (`A`,
//! world frame, through the filter's own attitude) is the GPS velocity change
//! over the same interval (`G`) turned by `eps` about Down. A least-squares
//! fit of that one angle over a fading window is closed form:
//! `eps = atan2(Σ cross(G, A), Σ G·A)`, and it reads any angle, ±π included.
//!
//! The verdict is acted on by the estimator (`Estimator::fuse_gps`): a
//! correction of the heading while fewer than `EKF_HDG_NMAX` have been made
//! in the flight, containment (a latch read by the validity and by the GPS
//! fusion) after that. The check itself only judges, counts and logs
//! ([`HeadingFit`]); it restarts at every correction, so one disagreement is
//! one verdict and one correction, not one per fix.
//!
//! **A fix without a valid speed accuracy** (`s_acc` not finite) is judged
//! with the filter's noise floor, as the fusion treats it (see `on_fix`);
//! the board bring-up must say that the receiver in use reports `s_acc`.
//!
//! All state is a handful of `f32` sums: no allocation, no history, no
//! division by a value that can be zero (guarded), transcendentals only
//! through [`Real`].

use crate::math::Real;
use crate::time::Micros;

/// Largest GPS speed accuracy (`s_acc`) at which an interval is used, m/s:
/// the receiver's own claim above which it distrusts its velocity (PX4's
/// `EKF2_REQ_SACC` default).
pub const SACC_MAX: f32 = 0.5;
/// Least `Sa` (m²/s², the sum of `|A|²`) at which the fit is evaluated:
/// below it the fit's own one-sigma at the noise floor,
/// `√2 · 0.3 / √Sa`, exceeds about 45° and the small-angle variance formula
/// does not hold.
pub const SA_MIN: f32 = 0.3;
/// `kappa = E / Sa` is 1 for a pure rotation. `|G|` and `|A|` measure the
/// same acceleration, so a factor of 2 either way is far outside any
/// accelerometer scale error or tilt-bias effect (healthy flights: 0.73 to
/// 1.44).
pub const KAPPA_MIN: f32 = 0.5;
/// See [`KAPPA_MIN`].
pub const KAPPA_MAX: f32 = 2.0;
/// Largest rms residual of the fit per interval, m/s: twice the residual
/// expected at the noise floor, `2 · √2 · 0.3` (healthy flights: at most
/// 0.33).
pub const RES_MAX: f32 = 0.85;
/// The magnitude gate, in standard deviations: a rotation does not change
/// `|G| - |A|`, so a difference above `MAG_GATE · √2 · sv` is not a rotation
/// (a GPS jump, a burst) and the fit restarts. The filter's own GPS gate
/// (`EKF_GPS_GATE`, 5 sigma) applied to the one quantity a rotation cannot
/// change.
pub const MAG_GATE: f32 = 5.0;
/// A gap between two fixes longer than this restarts the fit, µs: five
/// fixes at 5 Hz.
pub const GAP_MAX_US: u64 = 1_000_000;
/// An interval between two fixes is used only when it lasts at least this
/// long, µs: shorter ones have a velocity change of the order of the noise.
pub const DT_MIN_US: u64 = 100_000;
/// ... and at most this long, µs: a longer one spans a missed fix, and the
/// accelerometer integral then covers a different span than the fix pair
/// assumes.
pub const DT_MAX_US: u64 = 300_000;
/// Smallest `E` the sigma formula divides by.
const E_MIN: f32 = 1.0e-9;
/// `√2`: the standard deviation of a difference of two GPS velocities is
/// `√2 · sv` per axis.
const SQRT_2: f32 = core::f32::consts::SQRT_2;

/// The tunable values of the check (parameters `EKF_HDG_*` and the filter's
/// own GPS velocity noise floor).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HeadingConfig {
    /// `THETA`: the heading error worth acting on, rad (`EKF_HDG_THR`).
    pub threshold_rad: f32,
    /// `K`: the one-sided confidence, in sigma (`EKF_HDG_K`).
    pub k_sigma: f32,
    /// How long the disagreement must hold, µs (`EKF_HDG_T`).
    pub persist_us: u64,
    /// `TAU`: the memory of the fit, s (`EKF_HDG_TAU`).
    pub tau_s: f32,
    /// Floor of the GPS velocity noise, m/s (`EKF_GPS_VEL_N`).
    pub vel_noise_mps: f32,
}

impl Default for HeadingConfig {
    /// The parameter defaults (the same numbers as the table in `params`).
    fn default() -> Self {
        Self {
            threshold_rad: 0.3,
            k_sigma: 2.0,
            persist_us: 1_000_000,
            tau_s: 3.0,
            vel_noise_mps: 0.3,
        }
    }
}

/// What the check says at a fix.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Verdict {
    /// Nothing to say (not enough evidence, or the heading agrees).
    None,
    /// The heading disagrees with the motion by `eps` (filter minus truth,
    /// rad, any angle) with one-sigma `sigma`, and has for the persistence
    /// time. The estimator acts on it and restarts the fit.
    Correct { eps: f32, sigma: f32 },
}

/// One record per GPS fix judged while airborne: the quantities of the fit
/// after that fix, for the log. All zero when no fix has been judged or
/// while the vehicle is not airborne (`seq` keeps its value).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HeadingFit {
    /// Fixes judged while airborne since boot (saturating).
    pub seq: u32,
    /// `t_sample` of the fix, µs.
    pub t_us: u64,
    /// The interval ending at this fix was used by the fit.
    pub used: bool,
    /// The fitted heading error, rad (filter minus truth).
    pub eps: f32,
    /// Its one-sigma, rad.
    pub sigma: f32,
    /// `Sa`, the fading sum of `|A|²`, m²/s².
    pub sa: f32,
    /// `kappa = E / Sa`: the size of `G` against `A` (1 for a rotation).
    pub kappa: f32,
    /// `rho`: the rms residual per interval after turning `A` by `eps`, m/s.
    pub rho: f32,
    /// The test statistic `(|eps| - THETA) / sigma`.
    pub stat: f32,
    /// Consecutive fixes with every gate open and `stat >= K`.
    pub streak: u8,
    /// 1 when the verdict at this fix is `Correct`, else 0.
    pub verdict: u8,
    /// The filter's GPS velocity test ratio at this fix (squared normalised
    /// innovation over the squared gate; above 1 means rejected), or -1
    /// when the fix was not fused (a reset to GPS, or skipped).
    pub vel_ratio: f32,
}

/// The heading guard's per-flight state, for the log (cleared at disarm).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HeadingGuard {
    /// Corrections made in this flight.
    pub corrections: u8,
    /// The containment latch: the heading is declared untrustworthy until
    /// disarm.
    pub contained: bool,
    /// The run-time shift of the magnetometer declination made by the
    /// corrections of this flight, rad (the parameter `EKF_MAG_DECL` is
    /// never written).
    pub shift_rad: f32,
}

/// The heading check: the sums, the previous fix and the streak.
#[derive(Clone, Copy, Debug)]
pub struct HeadingCheck {
    cfg: HeadingConfig,
    /// The supervisor's `armed && !landed`.
    airborne: bool,
    /// Horizontal velocity change the filter integrated since the previous
    /// fix, world N/E, m/s.
    acc: [f32; 2],
    /// Previous usable fix: its `t_sample` and horizontal velocity.
    prev: Option<(Micros, [f32; 2])>,
    sc: f32,
    sd: f32,
    sa: f32,
    sg: f32,
    n: f32,
    streak: u8,
    /// Start (the previous fix's time) of the first interval of the streak.
    streak_from: Micros,
    /// Fixes at which the verdict was `Correct`, since boot.
    verdicts: u32,
    fit: HeadingFit,
}

impl HeadingCheck {
    pub const fn new(cfg: HeadingConfig) -> Self {
        Self {
            cfg,
            airborne: false,
            acc: [0.0; 2],
            prev: None,
            sc: 0.0,
            sd: 0.0,
            sa: 0.0,
            sg: 0.0,
            n: 0.0,
            streak: 0,
            streak_from: Micros::ZERO,
            verdicts: 0,
            fit: HeadingFit {
                seq: 0,
                t_us: 0,
                used: false,
                eps: 0.0,
                sigma: 0.0,
                sa: 0.0,
                kappa: 0.0,
                rho: 0.0,
                stat: 0.0,
                streak: 0,
                verdict: 0,
                vel_ratio: 0.0,
            },
        }
    }

    /// The latest per-fix record.
    pub const fn fit(&self) -> HeadingFit {
        self.fit
    }

    /// How many fixes since boot had the verdict `Correct`.
    pub const fn verdicts(&self) -> u32 {
        self.verdicts
    }

    /// Whether the check believes the vehicle is airborne.
    pub const fn airborne(&self) -> bool {
        self.airborne
    }

    /// The supervisor's `armed && !landed`. Leaving the air restarts the fit;
    /// on the ground nothing accumulates.
    pub fn set_airborne(&mut self, airborne: bool) {
        if self.airborne && !airborne {
            self.restart();
            self.fit = HeadingFit {
                seq: self.fit.seq,
                ..HeadingFit::default()
            };
        }
        self.airborne = airborne;
    }

    /// Forgets the fit: the sums, the streak, the half-built interval and
    /// the previous fix. Called wherever the filter's yaw is reset (the
    /// accelerometer integral of an interval that spans a yaw reset is
    /// rotated by two different headings) and when the filter is dropped.
    pub fn restart(&mut self) {
        self.restart_sums();
        self.acc = [0.0; 2];
        self.prev = None;
    }

    fn restart_sums(&mut self) {
        self.sc = 0.0;
        self.sd = 0.0;
        self.sa = 0.0;
        self.sg = 0.0;
        self.n = 0.0;
        self.streak = 0;
    }

    /// One filter step: the horizontal part of the velocity change the
    /// filter integrated (`R(q) (dvel - ba dt)`, world N/E, m/s). Ignored
    /// on the ground and when not finite.
    pub fn on_step(&mut self, dvw_ne: [f32; 2]) {
        if !self.airborne {
            return;
        }
        let [n, e] = dvw_ne;
        if n.is_finite() && e.is_finite() {
            let [an, ae] = &mut self.acc;
            *an += n;
            *ae += e;
        }
    }

    /// Records the filter's GPS velocity test ratio for the fix just judged
    /// (call after the fusion; `None` if the fix was not fused).
    pub fn set_vel_ratio(&mut self, ratio: Option<f32>) {
        if self.airborne {
            self.fit.vel_ratio = ratio.filter(|r| r.is_finite()).unwrap_or(-1.0);
        }
    }

    /// One GPS fix at its sample time `t`, in time order (the filter fuses
    /// them at the delayed horizon). `fix` is the horizontal velocity (world
    /// N/E, m/s) and the receiver's speed accuracy (m/s), or `None` when the
    /// fix is not usable (no 3D fix, not finite): the interval then simply
    /// grows to the next fix and is too long to be used.
    pub fn on_fix(&mut self, t: Micros, fix: Option<([f32; 2], f32)>) -> Verdict {
        if !self.airborne {
            // On the ground nothing accumulates.
            self.restart();
            return Verdict::None;
        }
        let Some((v, s_acc)) = fix else {
            return Verdict::None;
        };
        if !(v[0].is_finite() && v[1].is_finite()) {
            return Verdict::None;
        }
        // A receiver that reports no accuracy (`s_acc` not finite) is taken
        // as the filter's fusion takes it: zero, so the noise floor applies
        // and the fix is judged. This is a decision (planner's ruling R3):
        // a receiver that reports no accuracy must not switch the guard off,
        // an unseen wrong heading being the worse failure. The price is that
        // the receiver's own refusal of its velocity (`s_acc` above
        // `SACC_MAX`) is not heard from a receiver that never says it. The
        // board bring-up must therefore confirm that the receiver in use
        // reports its speed accuracy.
        let s_acc = s_acc.finite_or(0.0).max(0.0);
        let a = self.acc;
        self.acc = [0.0; 2];
        let Some((t_prev, v_prev)) = self.prev.replace((t, v)) else {
            // The first fix since a restart: no interval yet.
            self.restart_sums();
            return self.record(t, false, 0.0, Verdict::None);
        };
        let dt_us = t.since(t_prev);
        if dt_us > GAP_MAX_US {
            // A gap: the sums are of a different time. Start afresh.
            self.restart_sums();
            return self.record(t, false, 0.0, Verdict::None);
        }
        let g = [v[0] - v_prev[0], v[1] - v_prev[1]];
        let cr = g[0] * a[1] - g[1] * a[0];
        let dd = g[0] * a[0] + g[1] * a[1];
        let a2 = a[0] * a[0] + a[1] * a[1];
        let g2 = g[0] * g[0] + g[1] * g[1];
        let sv = s_acc.max(self.cfg.vel_noise_mps);
        let mut used = (DT_MIN_US..=DT_MAX_US).contains(&dt_us)
            && s_acc <= SACC_MAX
            && cr.is_finite()
            && dd.is_finite()
            && a2.is_finite()
            && g2.is_finite();
        if used && (Real::sqrt(g2) - Real::sqrt(a2)).abs() > MAG_GATE * SQRT_2 * sv {
            // A rotation cannot change the size: the GPS and the IMU
            // disagree on how much the vehicle accelerated.
            used = false;
            self.restart_sums();
        }
        if used {
            let lam = Real::exp(-micros_to_s(dt_us) / self.cfg.tau_s);
            self.sc = lam * self.sc + cr;
            self.sd = lam * self.sd + dd;
            self.sa = lam * self.sa + a2;
            self.sg = lam * self.sg + g2;
            self.n = lam * self.n + 1.0;
            if !(self.sc.is_finite() && self.sd.is_finite() && self.sa.is_finite()) {
                self.restart_sums();
                used = false;
            }
        }

        let mut stat = 0.0;
        let mut good = false;
        if self.n > 0.0 && self.sa > 0.0 {
            let e = Real::sqrt(self.sc * self.sc + self.sd * self.sd);
            let eps = Real::atan2(self.sc, self.sd);
            let kappa = e / self.sa;
            let rho = Real::sqrt((self.sg + self.sa - 2.0 * e).max(0.0) / self.n);
            // The noise per interval: the larger of the claimed
            // (`sqrt(2) sv`) and the fit's own residual, so a noisier
            // receiver makes the check slower, not wrong (D2 sections 8
            // and 2.1; the offline prototype does the same).
            let noise = (SQRT_2 * sv).max(rho);
            let sigma = noise * Real::sqrt(self.sa) / e.max(E_MIN);
            stat = (eps.abs() - self.cfg.threshold_rad) / sigma;
            good = used
                && self.sa >= SA_MIN
                && (KAPPA_MIN..=KAPPA_MAX).contains(&kappa)
                && rho <= RES_MAX;
            self.fit.eps = eps;
            self.fit.sigma = sigma;
            self.fit.sa = self.sa;
            self.fit.kappa = kappa;
            self.fit.rho = rho;
        } else {
            self.fit.eps = 0.0;
            self.fit.sigma = 0.0;
            self.fit.sa = 0.0;
            self.fit.kappa = 0.0;
            self.fit.rho = 0.0;
        }
        if good && stat >= self.cfg.k_sigma {
            if self.streak == 0 {
                self.streak_from = t_prev;
            }
            self.streak = self.streak.saturating_add(1);
        } else {
            self.streak = 0;
        }
        let verdict = if self.streak > 0 && t.since(self.streak_from) >= self.cfg.persist_us {
            Verdict::Correct {
                eps: self.fit.eps,
                sigma: self.fit.sigma,
            }
        } else {
            Verdict::None
        };
        self.record(t, used, stat, verdict)
    }

    /// Fills the per-fix record (the fit quantities are already in it) and
    /// returns the verdict.
    fn record(&mut self, t: Micros, used: bool, stat: f32, verdict: Verdict) -> Verdict {
        if self.n <= 0.0 || self.sa <= 0.0 {
            self.fit.eps = 0.0;
            self.fit.sigma = 0.0;
            self.fit.sa = 0.0;
            self.fit.kappa = 0.0;
            self.fit.rho = 0.0;
        }
        self.fit.seq = self.fit.seq.saturating_add(1);
        self.fit.t_us = t.0;
        self.fit.used = used;
        self.fit.stat = stat;
        self.fit.streak = self.streak;
        self.fit.verdict = u8::from(matches!(verdict, Verdict::Correct { .. }));
        if self.fit.verdict != 0 {
            self.verdicts = self.verdicts.saturating_add(1);
        }
        self.fit.vel_ratio = -1.0;
        verdict
    }
}

/// Microseconds as seconds, f32.
#[allow(clippy::cast_precision_loss)]
fn micros_to_s(us: u64) -> f32 {
    us as f32 * 1e-6
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::wrap_pi;

    const FIX_US: u64 = 200_000;

    /// A deterministic pseudo-random sequence in [-1, 1].
    struct Lcg(u32);
    impl Lcg {
        fn next(&mut self) -> f32 {
            self.0 = self.0.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (self.0 >> 8) as f32 / 8_388_608.0 - 1.0
        }
    }

    fn rot(v: [f32; 2], a: f32) -> [f32; 2] {
        let (s, c) = (Real::sin(a), Real::cos(a));
        [c * v[0] - s * v[1], s * v[0] + c * v[1]]
    }

    /// A velocity path with changing acceleration in both axes, m/s.
    fn path(k: u32) -> [f32; 2] {
        let t = k as f32 * 0.2;
        [6.0 * Real::sin(1.1 * t), 5.0 * Real::sin(0.7 * t + 1.0)]
    }

    /// Drives a check along `path` with the filter heading wrong by `eps`:
    /// `n` fixes, `A = Rz(eps) G` plus `noise` m/s of GPS velocity noise.
    struct Drive {
        chk: HeadingCheck,
        k: u32,
        t_us: u64,
        rng: Lcg,
        noise: f32,
        s_acc: f32,
        /// Scales the whole velocity path.
        amp: f32,
        last: Verdict,
        first_verdict_fix: Option<u32>,
    }

    impl Drive {
        fn new() -> Self {
            let mut chk = HeadingCheck::new(HeadingConfig::default());
            chk.set_airborne(true);
            Self {
                chk,
                k: 0,
                t_us: 1_000_000,
                rng: Lcg(7),
                noise: 0.0,
                s_acc: 0.2,
                amp: 1.0,
                last: Verdict::None,
                first_verdict_fix: None,
            }
        }

        /// One fix `dt_us` after the last, with the IMU integral `scale`
        /// times the GPS change turned by `eps`.
        fn fix_with(&mut self, dt_us: u64, eps: f32, scale: f32, extra_g: [f32; 2]) -> Verdict {
            let prev = path(self.k).map(|x| x * self.amp);
            self.k += 1;
            self.t_us += dt_us;
            let cur = path(self.k).map(|x| x * self.amp);
            let g = [cur[0] - prev[0], cur[1] - prev[1]];
            let a = rot(g, eps);
            // Several filter steps per interval.
            for _ in 0..50 {
                self.chk.on_step([a[0] * scale / 50.0, a[1] * scale / 50.0]);
            }
            let n = self.noise;
            let v = [
                cur[0] + extra_g[0] + n * self.rng.next(),
                cur[1] + extra_g[1] + n * self.rng.next(),
            ];
            let verdict = self.chk.on_fix(Micros(self.t_us), Some((v, self.s_acc)));
            if matches!(verdict, Verdict::Correct { .. }) && self.first_verdict_fix.is_none() {
                self.first_verdict_fix = Some(self.k);
            }
            self.last = verdict;
            verdict
        }

        fn fix(&mut self, eps: f32) -> Verdict {
            self.fix_with(FIX_US, eps, 1.0, [0.0; 2])
        }

        fn run(&mut self, n: u32, eps: f32) {
            for _ in 0..n {
                self.fix(eps);
            }
        }
    }

    #[test]
    fn a_pure_rotation_of_any_angle_is_found_within_its_sigma() {
        let pi = core::f32::consts::PI;
        for eps in [
            0.8, -0.8, 1.0, -1.0, 1.5, -1.5, 2.0, -2.0, 3.0, -3.0, 3.13, -3.13, pi, -pi,
        ] {
            let mut d = Drive::new();
            d.noise = 0.1;
            d.run(40, eps);
            let fit = d.chk.fit();
            let err = wrap_pi(fit.eps - eps).abs();
            assert!(fit.used, "eps {eps}");
            assert!(
                err <= fit.sigma.max(0.02),
                "eps {eps}: fitted {} sigma {}",
                fit.eps,
                fit.sigma
            );
            assert!(fit.sigma < 0.2, "eps {eps}: sigma {}", fit.sigma);
            assert!(fit.sa > 5.0, "eps {eps}");
            assert!(
                (0.8..1.25).contains(&fit.kappa),
                "eps {eps}: kappa {}",
                fit.kappa
            );
            assert!(fit.rho < 0.3, "eps {eps}: rho {}", fit.rho);
            assert!(matches!(d.last, Verdict::Correct { .. }), "eps {eps}");
        }
    }

    #[test]
    fn the_sign_of_the_angle_is_filter_minus_truth() {
        // A is G turned by +eps about Down (north towards east): eps > 0.
        let mut d = Drive::new();
        d.run(30, 1.0);
        assert!(d.chk.fit().eps > 0.9 && d.chk.fit().eps < 1.1);
        let mut d = Drive::new();
        d.run(30, -1.0);
        assert!(d.chk.fit().eps < -0.9 && d.chk.fit().eps > -1.1);
    }

    #[test]
    fn a_heading_that_agrees_or_is_slightly_off_gives_no_verdict() {
        // Below the threshold on perfect data: no verdict; and the verdict is
        // there for an error above it (the comparison is the right way
        // round).
        let mut d = Drive::new();
        d.run(60, 0.0);
        assert!(d.first_verdict_fix.is_none());
        assert!(d.chk.fit().eps.abs() < 0.01);
        let mut d = Drive::new();
        d.run(60, 0.25);
        assert!(d.first_verdict_fix.is_none(), "0.25 rad is below THETA");
        // Above it, far enough that the sigma (the claimed 0.3 m/s noise
        // floor over a window of this much motion) allows the verdict.
        let mut d = Drive::new();
        d.run(60, 0.8);
        assert!(
            d.first_verdict_fix.is_some(),
            "0.8 rad on perfect data is above THETA"
        );
        let mut d = Drive::new();
        d.run(60, -0.8);
        assert!(d.first_verdict_fix.is_some());
    }

    #[test]
    fn a_jump_in_gps_velocity_is_refused_by_the_magnitude_gate() {
        // A healthy flight, then one GPS velocity step of 6 m/s that the
        // accelerometer never felt.
        let mut d = Drive::new();
        d.run(25, 0.0);
        assert!(d.chk.fit().sa > 1.0);
        d.fix_with(FIX_US, 0.0, 1.0, [6.0, 0.0]);
        let fit = d.chk.fit();
        assert!(!fit.used, "the interval with the jump must not be used");
        assert_eq!(fit.sa, 0.0, "the fit restarts: {fit:?}");
        assert_eq!(fit.streak, 0);
        assert!(matches!(d.last, Verdict::None));
        // The same jump across a window that showed a rotation also ends
        // the streak.
        let mut d = Drive::new();
        d.run(3, 1.5);
        assert!(d.chk.fit().streak >= 1);
        d.fix_with(FIX_US, 1.5, 1.0, [0.0, 8.0]);
        assert!(!d.chk.fit().used);
        assert_eq!(d.chk.fit().streak, 0);
    }

    /// Two fixes 0.2 s apart with GPS velocity change `|G| = g` and an IMU
    /// integral of size `a` (the same direction); whether the second
    /// interval was used and whether the fit was restarted.
    fn gate_case(g: f32, a: f32, s_acc: f32) -> (bool, f32) {
        let mut chk = HeadingCheck::new(HeadingConfig::default());
        chk.set_airborne(true);
        // A first interval with agreeing sizes, so that there is something
        // to restart.
        let mut t = 1_000_000;
        let mut v = 0.0;
        chk.on_fix(Micros(t), Some(([v, 0.0], s_acc)));
        for _ in 0..3 {
            t += FIX_US;
            v += 1.0;
            chk.on_step([1.0, 0.0]);
            chk.on_fix(Micros(t), Some(([v, 0.0], s_acc)));
        }
        assert!(chk.fit().used && chk.fit().sa > 2.0);
        t += FIX_US;
        chk.on_step([a, 0.0]);
        chk.on_fix(Micros(t), Some(([v + g, 0.0], s_acc)));
        (chk.fit().used, chk.fit().sa)
    }

    #[test]
    fn the_magnitude_gate_is_the_difference_of_the_sizes_at_five_root_two_sigma() {
        // sv = max(s_acc 0.2, noise floor 0.3) = 0.3: the gate is
        // 5 sqrt(2) 0.3 = 2.1213 m/s on | |G| - |A| |.
        let gate = MAG_GATE * SQRT_2 * 0.3;
        assert!((gate - 2.1213).abs() < 1e-3);
        // A moderate acceleration, the IMU feeling less than the GPS: just
        // below the gate the interval is used, just above it is not and the
        // fit restarts. (A gate on the squares, `|G|^2 - |A|^2`, refuses the
        // first of these.)
        let (used, sa) = gate_case(3.0, 3.0 - (gate - 0.02), 0.2);
        assert!(used, "just below the gate");
        assert!(sa > 2.0);
        let (used, sa) = gate_case(3.0, 3.0 - (gate + 0.02), 0.2);
        assert!(!used, "just above the gate");
        assert_eq!(sa, 0.0, "the fit restarts");
        // The IMU feeling more than the GPS, at a small GPS change (a gate
        // on a ratio, or on the squares, gets this one wrong).
        let (used, _) = gate_case(1.0, 1.0 + gate - 0.02, 0.2);
        assert!(used, "just below the gate, A larger");
        let (used, sa) = gate_case(1.0, 1.0 + gate + 0.02, 0.2);
        assert!(!used && sa == 0.0, "just above the gate, A larger");
        // The gate widens with the receiver's own accuracy (sv = s_acc when
        // above the floor): 5 sqrt(2) 0.4 = 2.83.
        let wide = MAG_GATE * SQRT_2 * 0.4;
        let (used, _) = gate_case(3.0, 3.0 - (wide - 0.02), 0.4);
        assert!(used);
        let (used, _) = gate_case(3.0, 3.0 - (wide + 0.02), 0.4);
        assert!(!used);
    }

    #[test]
    fn a_fix_with_no_valid_speed_accuracy_is_judged_with_the_noise_floor() {
        // The fusion takes a missing `s_acc` as zero (the noise floor); so
        // does the check, so a receiver that reports none does not switch
        // the guard off.
        let run = |s_acc: f32| {
            let mut d = Drive::new();
            d.s_acc = s_acc;
            d.run(60, 1.5);
            (d.first_verdict_fix, d.chk.fit().used, d.chk.fit().sigma)
        };
        let floor = run(0.0);
        assert!(floor.0.is_some());
        for bad in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY, -1.0] {
            assert_eq!(run(bad), floor, "s_acc {bad}");
        }
    }

    #[test]
    fn a_jump_does_not_produce_a_verdict_on_a_rotated_flight() {
        // 5 fixes of a real 1.5 rad error would fire; a jump in the fifth
        // keeps it from firing then.
        let mut d = Drive::new();
        d.run(4, 1.5);
        d.fix_with(FIX_US, 1.5, 1.0, [0.0, 9.0]);
        assert!(d.first_verdict_fix.is_none());
    }

    #[test]
    fn no_motion_gives_no_verdict() {
        let mut chk = HeadingCheck::new(HeadingConfig::default());
        chk.set_airborne(true);
        let mut rng = Lcg(3);
        let mut t = 1_000_000;
        for _ in 0..200 {
            t += FIX_US;
            for _ in 0..50 {
                chk.on_step([0.002 * rng.next(), 0.002 * rng.next()]);
            }
            let v = [0.1 * rng.next(), 0.1 * rng.next()];
            assert_eq!(chk.on_fix(Micros(t), Some((v, 0.2))), Verdict::None);
        }
        assert!(chk.fit().sa < SA_MIN);
    }

    /// The steady `Sa` of the unit path (no scaling), measured.
    fn unit_sa() -> f32 {
        let mut d = Drive::new();
        d.run(60, 0.0);
        d.chk.fit().sa
    }

    /// Flies the path scaled by `scale`, with the filter heading wrong by
    /// `eps`, and returns (any verdict, largest `Sa`, whether a fix had
    /// `stat >= K` with `Sa` in `lo..hi`).
    fn fly_scaled(scale: f32, eps: f32, lo: f32, hi: f32) -> (bool, f32, bool) {
        let mut chk = HeadingCheck::new(HeadingConfig::default());
        chk.set_airborne(true);
        let mut t = 1_000_000u64;
        let mut prev = [0.0f32; 2];
        let (mut fired, mut max_sa, mut stat_in_range) = (false, 0.0f32, false);
        for k in 0..80u32 {
            t += FIX_US;
            let p = path(k);
            let cur = [scale * p[0], scale * p[1]];
            let g = [cur[0] - prev[0], cur[1] - prev[1]];
            prev = cur;
            chk.on_step(rot(g, eps));
            if matches!(
                chk.on_fix(Micros(t), Some((cur, 0.2))),
                Verdict::Correct { .. }
            ) {
                fired = true;
            }
            let f = chk.fit();
            max_sa = max_sa.max(f.sa);
            if k > 20 && f.sa >= lo && f.sa < hi && f.stat >= 2.0 {
                stat_in_range = true;
            }
        }
        (fired, max_sa, stat_in_range)
    }

    #[test]
    fn too_little_motion_is_not_judged() {
        // A heading wrong by pi seen through very little acceleration: the
        // sigma is large and the fit is not evaluated (Sa below SA_MIN).
        let scale = Real::sqrt(0.15 / unit_sa());
        let (fired, max_sa, _) = fly_scaled(scale, 3.1, 0.0, 0.0);
        assert!(!fired);
        assert!(max_sa < SA_MIN * 1.6, "Sa {max_sa}");
        // And with none at all.
        let (fired, max_sa, _) = fly_scaled(0.0, 3.1, 0.0, 0.0);
        assert!(!fired);
        assert_eq!(max_sa, 0.0);
    }

    #[test]
    fn small_sa_alone_blocks_the_verdict() {
        // eps = pi on perfect data with Sa between 0.12 and 0.3: the
        // statistic is above K (sigma about 1) but SA_MIN keeps the check
        // from judging.
        let scale = Real::sqrt(0.2 / unit_sa());
        let (fired, _, stat_in_range) = fly_scaled(scale, 3.1, 0.12, SA_MIN);
        assert!(
            stat_in_range,
            "the test must reach a state where only SA_MIN blocks"
        );
        assert!(!fired);
        // With ten times the motion the same error fires.
        let (fired, _, _) = fly_scaled(scale * 3.0, 3.1, 0.0, 0.0);
        assert!(fired);
    }

    #[test]
    fn the_size_ratio_gate_blocks_a_fit_that_is_not_a_rotation() {
        // The GPS saw 0.45 of what the IMU saw (kappa 0.45 < 0.5), with the
        // direction fitting 2 rad exactly; the magnitude gate and the
        // residual gate are both clear.
        let mut d = Drive::new();
        d.run(60, 2.0);
        assert!(d.first_verdict_fix.is_some(), "the unscaled flight fires");
        let mut d = Drive::new();
        d.amp = 0.5;
        for _ in 0..60 {
            d.fix_with(FIX_US, 2.0, 1.0 / 0.45, [0.0; 2]);
        }
        let f = d.chk.fit();
        assert!(f.kappa < KAPPA_MIN, "kappa {}", f.kappa);
        assert!(f.rho <= RES_MAX, "rho {}", f.rho);
        assert!(f.stat >= 2.0, "stat {}", f.stat);
        assert!(f.used);
        assert!(d.first_verdict_fix.is_none(), "kappa {}", f.kappa);
        // And above KAPPA_MAX (the IMU saw 0.4 of what the GPS saw).
        let mut d = Drive::new();
        for _ in 0..60 {
            d.fix_with(FIX_US, 2.0, 0.4, [0.0; 2]);
        }
        let f = d.chk.fit();
        assert!(f.kappa > KAPPA_MAX, "kappa {}", f.kappa);
        assert!(f.rho <= RES_MAX, "rho {}", f.rho);
        assert!(f.stat >= 2.0, "stat {}", f.stat);
        assert!(d.first_verdict_fix.is_none());
    }

    #[test]
    fn a_noisy_fit_is_blocked_by_the_residual_gate() {
        // The direction fits 1.5 rad on average but each interval scatters
        // by 1 m/s perpendicular to the acceleration: kappa is in range,
        // the statistic is above K, rho is above RES_MAX.
        let mut chk = HeadingCheck::new(HeadingConfig::default());
        chk.set_airborne(true);
        let mut t = 1_000_000u64;
        let mut prev = [0.0f32; 2];
        let mut fired = false;
        let mut last = HeadingFit::default();
        for k in 0..80u32 {
            t += FIX_US;
            let p = path(k);
            let cur = [2.0 * p[0], 2.0 * p[1]];
            let g = [cur[0] - prev[0], cur[1] - prev[1]];
            prev = cur;
            let gn = Real::sqrt(g[0] * g[0] + g[1] * g[1]).max(1e-6);
            let side = if k % 2 == 0 { 1.0 } else { -1.0 };
            let a = rot([g[0] - side * g[1] / gn, g[1] + side * g[0] / gn], 1.5);
            chk.on_step(a);
            if matches!(
                chk.on_fix(Micros(t), Some((cur, 0.2))),
                Verdict::Correct { .. }
            ) {
                fired = true;
            }
            last = chk.fit();
        }
        assert!(last.rho > RES_MAX, "rho {}", last.rho);
        assert!(
            (KAPPA_MIN..=KAPPA_MAX).contains(&last.kappa),
            "kappa {}",
            last.kappa
        );
        assert!(last.stat >= 2.0, "stat {}", last.stat);
        assert!(!fired);
    }

    #[test]
    fn sigma_is_the_larger_of_the_claimed_noise_and_the_fits_own_residual() {
        // Each interval scatters by 0.6 m/s perpendicular to the
        // acceleration: rho (0.6) is above the claimed sqrt(2) 0.3 (0.42)
        // and below the residual gate (0.85). The sigma follows rho.
        let mut chk = HeadingCheck::new(HeadingConfig::default());
        chk.set_airborne(true);
        let mut t = 1_000_000u64;
        let mut prev = [0.0f32; 2];
        for k in 0..60u32 {
            t += FIX_US;
            let p = path(k);
            let cur = [2.0 * p[0], 2.0 * p[1]];
            let g = [cur[0] - prev[0], cur[1] - prev[1]];
            prev = cur;
            let gn = Real::sqrt(g[0] * g[0] + g[1] * g[1]).max(1e-6);
            let side = if k % 2 == 0 { 0.6 } else { -0.6 };
            chk.on_step(rot([g[0] - side * g[1] / gn, g[1] + side * g[0] / gn], 1.5));
            let _ = chk.on_fix(Micros(t), Some((cur, 0.2)));
        }
        let f = chk.fit();
        assert!(f.rho > 0.5 && f.rho < RES_MAX, "rho {}", f.rho);
        // sigma = max(0.424, rho) sqrt(Sa) / E with E = kappa Sa.
        let want = f.rho.max(SQRT_2 * 0.3) / (f.kappa * Real::sqrt(f.sa));
        assert!(
            (f.sigma - want).abs() < 1e-4 * want,
            "sigma {} want {}",
            f.sigma,
            want
        );
        // And with the noise floor above rho it is the claimed noise.
        let cfg = HeadingConfig {
            vel_noise_mps: 1.0,
            ..HeadingConfig::default()
        };
        let mut d = Drive::new();
        d.chk = HeadingCheck::new(cfg);
        d.chk.set_airborne(true);
        d.s_acc = 0.2;
        d.run(40, 1.5);
        let f = d.chk.fit();
        let want = SQRT_2 * 1.0 / (f.kappa * Real::sqrt(f.sa));
        assert!(
            (f.sigma - want).abs() < 1e-4 * want,
            "sigma {} want {}",
            f.sigma,
            want
        );
    }

    #[test]
    fn the_receivers_own_speed_accuracy_gates_the_interval() {
        let mut d = Drive::new();
        d.s_acc = 0.8;
        d.run(60, 1.5);
        assert!(d.first_verdict_fix.is_none());
        assert!(!d.chk.fit().used);
        let mut d = Drive::new();
        d.s_acc = 0.5;
        d.run(60, 1.5);
        assert!(d.first_verdict_fix.is_some(), "0.5 m/s is still accepted");
    }

    #[test]
    fn only_intervals_of_one_to_three_tenths_of_a_second_are_used() {
        for dt in [50_000u64, 90_000, 310_000, 400_000, 600_000] {
            let mut d = Drive::new();
            for _ in 0..60 {
                d.fix_with(dt, 1.5, 1.0, [0.0; 2]);
            }
            assert!(d.first_verdict_fix.is_none(), "dt {dt}");
            assert!(!d.chk.fit().used, "dt {dt}");
        }
        for dt in [100_000u64, 150_000, 300_000] {
            let mut d = Drive::new();
            for _ in 0..80 {
                d.fix_with(dt, 1.5, 1.0, [0.0; 2]);
            }
            assert!(d.chk.fit().used, "dt {dt}");
        }
    }

    #[test]
    fn the_streak_needs_the_stated_time() {
        // A perfect 1.5 rad error from the start of the flight: the first
        // judged interval is the second fix; the verdict comes when the
        // streak has lasted 1 s, that is five fixes.
        let mut d = Drive::new();
        let mut first_ok = None;
        for k in 0..40 {
            d.fix(1.5);
            let f = d.chk.fit();
            if first_ok.is_none() && f.streak > 0 {
                first_ok = Some(k);
            }
            if let Verdict::Correct { .. } = d.last {
                let s = first_ok.expect("a streak before the verdict");
                assert_eq!(k - s, 4, "five fixes: from {s} to {k}");
                assert_eq!(f.streak, 5);
                assert_eq!(f.verdict, 1);
                return;
            }
            assert!(f.streak < 5);
        }
        panic!("no verdict");
    }

    #[test]
    fn the_persistence_time_is_the_parameter() {
        for (secs, fixes) in [(0.4f32, 2u8), (0.6, 3), (2.0, 10)] {
            let cfg = HeadingConfig {
                persist_us: (secs * 1e6) as u64,
                ..HeadingConfig::default()
            };
            let mut d = Drive::new();
            d.chk = HeadingCheck::new(cfg);
            d.chk.set_airborne(true);
            for _ in 0..60 {
                d.fix(1.5);
                if let Verdict::Correct { .. } = d.last {
                    assert_eq!(d.chk.fit().streak, fixes, "persist {secs}");
                    break;
                }
            }
            assert!(d.first_verdict_fix.is_some(), "persist {secs}");
        }
    }

    #[test]
    fn a_broken_streak_starts_again() {
        // Four disagreeing fixes, one fix with an interval that is not used
        // (too long), then the count starts over: the verdict needs five
        // more.
        let mut d = Drive::new();
        d.run(5, 1.5);
        // Find a state with a streak of exactly 3.
        let mut d = Drive::new();
        let mut guard = 0;
        while d.chk.fit().streak < 3 {
            d.fix(1.5);
            guard += 1;
            assert!(guard < 30);
        }
        assert!(d.first_verdict_fix.is_none());
        d.fix_with(450_000, 1.5, 1.0, [0.0; 2]);
        assert_eq!(d.chk.fit().streak, 0, "an unused interval ends the streak");
        let from = d.k;
        d.run(10, 1.5);
        let at = d.first_verdict_fix.expect("fires again");
        assert_eq!(at - from, 5, "five fixes after the break");
    }

    #[test]
    fn a_gap_restarts_the_fit() {
        let mut d = Drive::new();
        d.run(40, 1.5);
        assert!(d.chk.fit().sa > 10.0);
        // 1.5 s without a fix, and then the heading is wrong the other way.
        d.fix_with(1_500_000, -1.5, 1.0, [0.0; 2]);
        assert_eq!(d.chk.fit().sa, 0.0, "{:?}", d.chk.fit());
        assert_eq!(d.chk.fit().streak, 0);
        assert!(matches!(d.last, Verdict::None));
        d.run(3, -1.5);
        let f = d.chk.fit();
        assert!(
            wrap_pi(f.eps + 1.5).abs() < 0.05,
            "fit {} after the gap",
            f.eps
        );
        // And just under a second is not a gap: the sums survive (the
        // interval itself is too long to be used).
        let mut d = Drive::new();
        d.run(40, 1.5);
        d.fix_with(900_000, 1.5, 1.0, [0.0; 2]);
        assert!(d.chk.fit().sa > 10.0);
    }

    #[test]
    fn a_yaw_reset_restarts_the_fit() {
        let mut d = Drive::new();
        d.run(40, 1.5);
        assert!(d.chk.fit().sa > 10.0);
        d.first_verdict_fix = None;
        // Half an interval of steps, then the filter's yaw is reset.
        for _ in 0..25 {
            d.chk.on_step([0.1, 0.1]);
        }
        d.chk.restart();
        // The next fix has no previous fix: no interval, and nothing from
        // before.
        d.run(1, 0.0);
        let f = d.chk.fit();
        assert_eq!(f.sa, 0.0);
        assert!(!f.used);
        // The fix after it is judged on the new heading alone.
        d.run(8, 0.0);
        let f = d.chk.fit();
        assert!(f.used && f.eps.abs() < 0.01, "{f:?}");
        assert!(d.first_verdict_fix.is_none(), "restart clears the streak");
    }

    #[test]
    fn restart_clears_the_half_built_interval() {
        // Steps before a reset are of the old heading: the interval that
        // spans it is not used. After restart the first fix only sets the
        // reference.
        let mut chk = HeadingCheck::new(HeadingConfig::default());
        chk.set_airborne(true);
        for _ in 0..10 {
            chk.on_step([5.0, 5.0]);
        }
        chk.restart();
        assert_eq!(
            chk.on_fix(Micros(1_000_000), Some(([0.0, 0.0], 0.2))),
            Verdict::None
        );
        chk.on_step([0.1, 0.0]);
        let _ = chk.on_fix(Micros(1_200_000), Some(([0.1, 0.0], 0.2)));
        // |A| is 0.1, not 7: the stale 10 steps are gone.
        assert!(chk.fit().sa < 0.02, "{:?}", chk.fit());
    }

    #[test]
    fn on_the_ground_nothing_accumulates() {
        let mut chk = HeadingCheck::new(HeadingConfig::default());
        let mut d = Drive::new();
        d.chk = chk;
        d.chk.set_airborne(false);
        d.run(60, 1.5);
        assert!(d.first_verdict_fix.is_none());
        assert_eq!(d.chk.fit().seq, 0, "no fix is judged on the ground");
        assert_eq!(d.chk.fit().sa, 0.0);
        // The first airborne fix only sets the reference: a minute of
        // motion on the ground has left nothing in the sums.
        d.chk.set_airborne(true);
        d.run(1, 1.5);
        assert_eq!(d.chk.fit().sa, 0.0);
        d.run(1, 1.5);
        let f = d.chk.fit();
        assert!(f.sa < 2.0, "only one interval is in the sums: {}", f.sa);
        // Landing clears it.
        d.run(30, 1.5);
        assert!(d.chk.fit().sa > 5.0);
        d.chk.set_airborne(false);
        assert_eq!(d.chk.fit().sa, 0.0);
        assert_eq!(d.chk.fit().streak, 0);
        let seq = d.chk.fit().seq;
        chk = d.chk;
        for _ in 0..50 {
            chk.on_step([1.0, 1.0]);
        }
        let _ = chk.on_fix(Micros(100_000_000), Some(([1.0, 1.0], 0.2)));
        assert_eq!(chk.fit().seq, seq);
        assert_eq!(chk.fit().sa, 0.0);
    }

    #[test]
    fn an_unusable_fix_does_not_break_the_interval() {
        // A fix that is not 3D, in the middle: the interval across it is
        // 0.4 s long and is not used, then the fit goes on.
        let mut d = Drive::new();
        d.run(20, 1.5);
        let sa = d.chk.fit().sa;
        // A bad fix: the filter keeps stepping.
        let prev = path(d.k);
        d.k += 1;
        d.t_us += FIX_US;
        let cur = path(d.k);
        let g = [cur[0] - prev[0], cur[1] - prev[1]];
        let a = rot(g, 1.5);
        d.chk.on_step(a);
        assert_eq!(d.chk.on_fix(Micros(d.t_us), None), Verdict::None);
        assert_eq!(d.chk.fit().sa, sa, "no change from an unusable fix");
        d.fix(1.5);
        assert!(!d.chk.fit().used, "0.4 s across the missing fix");
        d.fix(1.5);
        assert!(d.chk.fit().used);
    }

    #[test]
    fn non_finite_input_is_ignored() {
        let mut chk = HeadingCheck::new(HeadingConfig::default());
        chk.set_airborne(true);
        chk.on_step([f32::NAN, 1.0]);
        chk.on_step([f32::INFINITY, 1.0]);
        assert_eq!(
            chk.on_fix(Micros(1_000_000), Some(([f32::NAN, 0.0], 0.2))),
            Verdict::None
        );
        assert_eq!(
            chk.on_fix(Micros(1_200_000), Some(([0.0, 0.0], f32::NAN))),
            Verdict::None
        );
        chk.on_step([0.0, 0.1]);
        let v = chk.on_fix(Micros(1_400_000), Some(([0.0, 0.1], 0.2)));
        assert_eq!(v, Verdict::None);
        let f = chk.fit();
        assert!(f.sa.is_finite() && f.eps.is_finite() && f.sigma.is_finite());
    }

    #[test]
    fn the_velocity_test_ratio_is_logged_with_the_fix() {
        let mut d = Drive::new();
        d.run(3, 0.0);
        assert_eq!(d.chk.fit().vel_ratio, -1.0);
        d.chk.set_vel_ratio(Some(0.37));
        assert_eq!(d.chk.fit().vel_ratio, 0.37);
        d.chk.set_vel_ratio(Some(f32::NAN));
        assert_eq!(d.chk.fit().vel_ratio, -1.0);
        d.run(1, 0.0);
        assert_eq!(d.chk.fit().vel_ratio, -1.0, "a new fix starts at not fused");
    }

    #[test]
    fn the_threshold_and_confidence_are_the_parameters() {
        // THETA 0.8: a 0.6 rad error on perfect data does not fire, a
        // 1.5 rad error does.
        let cfg = HeadingConfig {
            threshold_rad: 0.8,
            ..HeadingConfig::default()
        };
        let mut d = Drive::new();
        d.chk = HeadingCheck::new(cfg);
        d.chk.set_airborne(true);
        d.run(60, 0.6);
        assert!(d.first_verdict_fix.is_none());
        let mut d = Drive::new();
        d.chk = HeadingCheck::new(cfg);
        d.chk.set_airborne(true);
        d.run(60, 1.5);
        assert!(d.first_verdict_fix.is_some());
        // K huge: nothing fires.
        let cfg = HeadingConfig {
            k_sigma: 1.0e6,
            ..HeadingConfig::default()
        };
        let mut d = Drive::new();
        d.chk = HeadingCheck::new(cfg);
        d.chk.set_airborne(true);
        d.run(60, 1.5);
        assert!(d.first_verdict_fix.is_none());
    }

    #[test]
    fn the_memory_is_the_parameter() {
        // After a change of heading the fit follows within about TAU.
        let follow = |tau: f32| {
            let cfg = HeadingConfig {
                tau_s: tau,
                ..HeadingConfig::default()
            };
            let mut d = Drive::new();
            d.chk = HeadingCheck::new(cfg);
            d.chk.set_airborne(true);
            d.run(60, 1.5);
            let mut n = 0;
            while wrap_pi(d.chk.fit().eps + 1.5).abs() > 0.3 && n < 200 {
                d.fix(-1.5);
                n += 1;
            }
            n
        };
        assert!(
            follow(1.0) + 3 < follow(6.0),
            "{} {}",
            follow(1.0),
            follow(6.0)
        );
    }
}
