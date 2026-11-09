// SPDX-License-Identifier: AGPL-3.0-or-later
//! Wind: the velocity of the air mass, NED, m/s (direction the air moves).
//!
//! The field is uniform in space and a function of time only:
//! `steady + turbulence + gusts`.
//!
//! Turbulence is a first-order Gauss–Markov process per axis, discretised
//! exactly. It is deliberately not a Dryden model: Dryden spectra need an
//! airspeed to turn spatial frequency into temporal frequency, and a hovering
//! multirotor has none. Dryden/von Kármán come with the fixed-wing work (S2b).

use oxide_core::math::{Real, Vec3};
use oxide_core::time::Micros;

use crate::rng::Rng;

/// A discrete "1 − cosine" gust (MIL-F-8785C shape):
/// `v(t) = peak · ½(1 − cos(2π (t − start)/duration))` for
/// `start ≤ t ≤ start + duration`, else 0.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Gust {
    pub start_s: f64,
    pub duration_s: f64,
    pub peak_ned: Vec3<f64>,
}

/// Wind configuration. `Default` is calm.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct WindConfig {
    pub steady_ned: Vec3<f64>,
    /// Turbulence standard deviation per axis, m/s.
    pub turbulence_sigma: Vec3<f64>,
    /// Turbulence correlation time, s.
    pub turbulence_tau_s: f64,
    pub gusts: Vec<Gust>,
}

impl WindConfig {
    pub fn calm() -> Self {
        Self::default()
    }

    /// `true` if the wind is exactly zero at all times.
    pub fn is_calm(&self) -> bool {
        let z = Vec3::new(0.0, 0.0, 0.0);
        self.steady_ned == z
            && (self.turbulence_sigma == z || self.turbulence_tau_s <= 0.0)
            && self.gusts.iter().all(|g| g.peak_ned == z)
    }
}

/// Sum of the gusts active at `t_s`.
pub fn gust_velocity(gusts: &[Gust], t_s: f64) -> Vec3<f64> {
    let mut v = Vec3::new(0.0, 0.0, 0.0);
    for g in gusts {
        if g.duration_s > 0.0 && t_s >= g.start_s && t_s <= g.start_s + g.duration_s {
            let phase = 2.0 * core::f64::consts::PI * (t_s - g.start_s) / g.duration_s;
            let s = 0.5 * (1.0 - Real::cos(phase));
            v = Vec3::new(
                v.x + g.peak_ned.x * s,
                v.y + g.peak_ned.y * s,
                v.z + g.peak_ned.z * s,
            );
        }
    }
    v
}

/// Uniform wind field, a function of time only.
pub struct Wind {
    cfg: WindConfig,
    turbulence: Vec3<f64>,
    rng: Rng,
    last_t: Option<Micros>,
    current: Vec3<f64>,
}

impl Wind {
    pub fn new(cfg: WindConfig, rng: Rng) -> Self {
        Self {
            cfg,
            turbulence: Vec3::new(0.0, 0.0, 0.0),
            rng,
            last_t: None,
            current: Vec3::new(0.0, 0.0, 0.0),
        }
    }

    /// Advances turbulence to `t` (non-decreasing) and returns the wind at
    /// `t` = steady + turbulence + gusts.
    pub fn sample(&mut self, t: Micros) -> Vec3<f64> {
        let sigma = self.cfg.turbulence_sigma;
        let tau = self.cfg.turbulence_tau_s;
        let active = tau > 0.0;
        match self.last_t {
            None => {
                // Stationary initial distribution: no warm-up.
                let mut x = Vec3::new(0.0, 0.0, 0.0);
                if active {
                    if sigma.x != 0.0 {
                        x.x = sigma.x * self.rng.normal();
                    }
                    if sigma.y != 0.0 {
                        x.y = sigma.y * self.rng.normal();
                    }
                    if sigma.z != 0.0 {
                        x.z = sigma.z * self.rng.normal();
                    }
                }
                self.turbulence = x;
            }
            Some(prev) => {
                let dt = t.since(prev) as f64 * 1e-6;
                if active && dt > 0.0 {
                    let a = Real::exp(-dt / tau);
                    let b = Real::sqrt(1.0 - a * a);
                    let x = self.turbulence;
                    let mut n = x;
                    if sigma.x != 0.0 {
                        n.x = a * x.x + sigma.x * b * self.rng.normal();
                    }
                    if sigma.y != 0.0 {
                        n.y = a * x.y + sigma.y * b * self.rng.normal();
                    }
                    if sigma.z != 0.0 {
                        n.z = a * x.z + sigma.z * b * self.rng.normal();
                    }
                    self.turbulence = n;
                }
            }
        }
        self.last_t = Some(t);
        let g = gust_velocity(&self.cfg.gusts, t.as_secs_f64());
        let s = self.cfg.steady_ned;
        let u = self.turbulence;
        self.current = Vec3::new(s.x + u.x + g.x, s.y + u.y + g.y, s.z + u.z + g.z);
        self.current
    }

    pub fn current(&self) -> Vec3<f64> {
        self.current
    }

    pub fn config(&self) -> &WindConfig {
        &self.cfg
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn gust() -> Gust {
        Gust {
            start_s: 2.0,
            duration_s: 4.0,
            peak_ned: Vec3::new(6.0, -2.0, 0.0),
        }
    }

    #[test]
    fn gust_shape() {
        let g = [gust()];
        let at = |t| gust_velocity(&g, t);
        assert_eq!(at(1.0), Vec3::new(0.0, 0.0, 0.0));
        assert_eq!(at(7.0), Vec3::new(0.0, 0.0, 0.0));
        assert!(at(2.0).x.abs() < 1e-12);
        assert!(at(6.0).x.abs() < 1e-12);
        let mid = at(4.0);
        assert!((mid.x - 6.0).abs() < 1e-12 && (mid.y + 2.0).abs() < 1e-12);
        let quarter = at(3.0);
        assert!((quarter.x - 3.0).abs() < 1e-12);
    }

    #[test]
    fn calm_wind_is_exactly_zero() {
        let cfg = WindConfig::calm();
        assert!(cfg.is_calm());
        let mut rng = Rng::from_seed(1);
        let mut w = Wind::new(cfg, rng.clone());
        for k in 0..1000 {
            assert_eq!(w.sample(Micros(k * 1000)), Vec3::new(0.0, 0.0, 0.0));
        }
        // Consumed no random numbers.
        assert_eq!(rng.next_u64(), w.rng.next_u64());
        assert!(
            !WindConfig {
                steady_ned: Vec3::new(1.0, 0.0, 0.0),
                ..WindConfig::calm()
            }
            .is_calm()
        );
    }

    fn turb_cfg() -> WindConfig {
        WindConfig {
            turbulence_sigma: Vec3::new(1.0, 1.0, 0.5),
            turbulence_tau_s: 2.0,
            ..WindConfig::calm()
        }
    }

    #[test]
    fn determinism() {
        let mut a = Wind::new(turb_cfg(), Rng::from_seed(42));
        let mut b = Wind::new(turb_cfg(), Rng::from_seed(42));
        let mut c = Wind::new(turb_cfg(), Rng::from_seed(43));
        let mut differs = false;
        for k in 0..500 {
            let t = Micros(k * 1000);
            let (va, vb, vc) = (a.sample(t), b.sample(t), c.sample(t));
            assert_eq!(va, vb);
            differs |= va != vc;
        }
        assert!(differs);
    }

    #[test]
    fn turbulence_statistics() {
        let mut w = Wind::new(turb_cfg(), Rng::from_seed(2024));
        let n: usize = 2_400_000; // 2400 s at 1 ms
        let lag = 2000; // tau = 2 s
        let mut xs: Vec<[f64; 3]> = Vec::with_capacity(n);
        for k in 0..n {
            let v = w.sample(Micros(k as u64 * 1000));
            xs.push([v.x, v.y, v.z]);
        }
        let sigmas = [1.0, 1.0, 0.5];
        for ax in 0..3 {
            let mean = xs.iter().map(|v| v[ax]).sum::<f64>() / n as f64;
            let var = xs
                .iter()
                .map(|v| {
                    let d = v[ax] - mean;
                    d * d
                })
                .sum::<f64>()
                / n as f64;
            let std = Real::sqrt(var);
            assert!(
                (std / sigmas[ax] - 1.0).abs() < 0.05,
                "axis {ax}: std {std}"
            );
            let cov = (0..n - lag)
                .map(|i| (xs[i][ax] - mean) * (xs[i + lag][ax] - mean))
                .sum::<f64>()
                / (n - lag) as f64;
            let rho = cov / var;
            assert!(
                (rho - Real::exp(-1.0_f64)).abs() < 0.05,
                "axis {ax}: autocorrelation {rho}"
            );
        }
    }
}
