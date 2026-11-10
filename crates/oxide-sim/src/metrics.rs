// SPDX-License-Identifier: AGPL-3.0-or-later
//! Pass/fail metrics for hold scenarios: the M1b acceptance criteria
//! (DESIGN §13). World frame NED, SI units; angles in degrees where the
//! field name says so.

use oxide_core::math::{Real, Vec3};
use oxide_core::schedule::Profile;
use oxide_core::time::Micros;

use crate::scenario::gust_hold;
use crate::sim::StepRecord;
use crate::{Airframe, SimError, Simulation};

/// Pass/fail limits for a hold scenario. These are the M1b acceptance
/// criteria (DESIGN §13); they are fixed in advance and are not tuned to
/// make a run pass.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HoldCriteria {
    /// Hold is judged from here to the end, s.
    pub window_start_s: f64,
    /// Must be armed with a valid estimate from here on, s.
    pub armed_from_s: f64,
    /// Truth vs target, horizontal, max in window, m.
    pub max_h_error_m: f64,
    /// Truth vs target, horizontal, RMS in window, m.
    pub rms_h_error_m: f64,
    /// Truth vs target, vertical, max in window, m.
    pub max_v_error_m: f64,
    /// Roll or pitch (truth), whole flight, deg.
    pub max_tilt_deg: f64,
    /// Estimate vs truth, horizontal position, max in window, m.
    pub est_max_h_error_m: f64,
    /// Estimate vs truth, vertical position, max in window, m.
    pub est_max_v_error_m: f64,
    /// Estimate vs truth, velocity norm, max in window, m/s.
    pub est_max_vel_error_mps: f64,
    /// Angle between estimated and true attitude, max in window, deg.
    pub est_max_att_error_deg: f64,
}

impl Default for HoldCriteria {
    fn default() -> Self {
        Self {
            window_start_s: 15.0,
            armed_from_s: 3.5,
            max_h_error_m: 1.5,
            rms_h_error_m: 0.5,
            max_v_error_m: 1.0,
            max_tilt_deg: 35.0,
            est_max_h_error_m: 1.0,
            est_max_v_error_m: 1.0,
            est_max_vel_error_mps: 0.5,
            est_max_att_error_deg: 3.0,
        }
    }
}

/// Running statistics of one run.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HoldMetrics {
    pub max_h_error_m: f64,
    pub rms_h_error_m: f64,
    pub max_v_error_m: f64,
    pub max_tilt_deg: f64,
    pub est_max_h_error_m: f64,
    pub est_max_v_error_m: f64,
    pub est_max_vel_error_mps: f64,
    pub est_max_att_error_deg: f64,
    /// Ticks after `armed_from_s` that were not armed.
    pub unarmed_ticks: u64,
    /// Ticks after `armed_from_s` with no estimate.
    pub no_estimate_ticks: u64,
    /// Any truth or estimate value was NaN or infinite.
    pub non_finite: bool,
    pub timing_faults: u32,
    /// From the last record's `status.estimator`.
    pub rejected: u32,
    /// From the last record's `status.estimator`.
    pub resets: u32,
    /// From the last record: non-finite physics steps so far.
    pub physics_faults: u32,
    sum_sq_h: f64,
    window_ticks: u64,
}

impl Default for HoldMetrics {
    fn default() -> Self {
        Self::new()
    }
}

impl HoldMetrics {
    pub fn new() -> Self {
        Self {
            max_h_error_m: 0.0,
            rms_h_error_m: 0.0,
            max_v_error_m: 0.0,
            max_tilt_deg: 0.0,
            est_max_h_error_m: 0.0,
            est_max_v_error_m: 0.0,
            est_max_vel_error_mps: 0.0,
            est_max_att_error_deg: 0.0,
            unarmed_ticks: 0,
            no_estimate_ticks: 0,
            non_finite: false,
            timing_faults: 0,
            rejected: 0,
            resets: 0,
            physics_faults: 0,
            sum_sq_h: 0.0,
            window_ticks: 0,
        }
    }

    /// Adds one tick. `target` is the hold point (NED, m).
    pub fn add<const N: usize>(&mut self, r: &StepRecord<N>, target: Vec3<f64>, c: &HoldCriteria) {
        let t = r.t.as_secs_f64();
        let e = r.attitude.to_euler();
        let tilt = e.x.abs().max(e.y.abs()).to_degrees();
        if tilt.is_finite() {
            self.max_tilt_deg = self.max_tilt_deg.max(tilt);
        } else {
            self.non_finite = true;
        }
        self.timing_faults = r.status.timing_faults;
        self.rejected = r.status.estimator.rejected;
        self.resets = r.status.estimator.resets;
        self.physics_faults = r.physics_faults;

        if t >= c.armed_from_s {
            if !r.status.armed {
                self.unarmed_ticks += 1;
            }
            if r.estimate.is_none() || r.status.no_estimate {
                self.no_estimate_ticks += 1;
            }
        }
        if t < c.window_start_s {
            return;
        }

        let d = r.position - target;
        let dh = Real::sqrt(d.x * d.x + d.y * d.y);
        let dv = d.z.abs();
        if dh.is_finite() && dv.is_finite() {
            self.max_h_error_m = self.max_h_error_m.max(dh);
            self.max_v_error_m = self.max_v_error_m.max(dv);
            self.sum_sq_h += dh * dh;
            self.window_ticks += 1;
            self.rms_h_error_m = Real::sqrt(self.sum_sq_h / self.window_ticks as f64);
        } else {
            self.non_finite = true;
        }

        let Some(est) = r.estimate else { return };
        let p = est.position.cast::<f64>();
        let v = est.velocity.cast::<f64>();
        let (dn, de) = (p.x - r.position.x, p.y - r.position.y);
        let eh = Real::sqrt(dn * dn + de * de);
        let ev = (p.z - r.position.z).abs();
        let evel = (v - r.velocity).norm();
        let dq = est.attitude.cast::<f64>() * r.attitude.conj();
        let ea = dq.to_rotation_vector().norm().to_degrees();
        if eh.is_finite() && ev.is_finite() && evel.is_finite() && ea.is_finite() {
            self.est_max_h_error_m = self.est_max_h_error_m.max(eh);
            self.est_max_v_error_m = self.est_max_v_error_m.max(ev);
            self.est_max_vel_error_mps = self.est_max_vel_error_mps.max(evel);
            self.est_max_att_error_deg = self.est_max_att_error_deg.max(ea);
        } else {
            self.non_finite = true;
        }
    }

    /// One human-readable line per violated criterion; empty when the run passes.
    pub fn failures(&self, c: &HoldCriteria) -> Vec<String> {
        let mut out = Vec::new();
        let mut limit = |name: &str, value: f64, max: f64, unit: &str| {
            // Written so that NaN fails.
            if value.is_nan() || value > max {
                out.push(format!("{name} {value:.3} {unit} > limit {max} {unit}"));
            }
        };
        limit(
            "max horizontal error",
            self.max_h_error_m,
            c.max_h_error_m,
            "m",
        );
        limit(
            "rms horizontal error",
            self.rms_h_error_m,
            c.rms_h_error_m,
            "m",
        );
        limit(
            "max vertical error",
            self.max_v_error_m,
            c.max_v_error_m,
            "m",
        );
        limit("max tilt", self.max_tilt_deg, c.max_tilt_deg, "deg");
        limit(
            "estimate max horizontal error",
            self.est_max_h_error_m,
            c.est_max_h_error_m,
            "m",
        );
        limit(
            "estimate max vertical error",
            self.est_max_v_error_m,
            c.est_max_v_error_m,
            "m",
        );
        limit(
            "estimate max velocity error",
            self.est_max_vel_error_mps,
            c.est_max_vel_error_mps,
            "m/s",
        );
        limit(
            "estimate max attitude error",
            self.est_max_att_error_deg,
            c.est_max_att_error_deg,
            "deg",
        );
        if self.window_ticks == 0 {
            // Otherwise a run too short to reach the window would pass
            // every window limit with zeros.
            out.push(format!(
                "no ticks in the hold window (from {} s)",
                c.window_start_s
            ));
        }
        if self.unarmed_ticks > 0 {
            out.push(format!(
                "{} ticks not armed after {} s",
                self.unarmed_ticks, c.armed_from_s
            ));
        }
        if self.no_estimate_ticks > 0 {
            out.push(format!(
                "{} ticks without an estimate after {} s",
                self.no_estimate_ticks, c.armed_from_s
            ));
        }
        if self.non_finite {
            out.push("non-finite value in truth or estimate".into());
        }
        if self.timing_faults > 0 {
            out.push(format!("{} timing faults", self.timing_faults));
        }
        if self.physics_faults > 0 {
            out.push(format!("{} non-finite physics steps", self.physics_faults));
        }
        if self.resets > 1 {
            out.push(format!(
                "{} estimator resets (1 expected: first GPS fix)",
                self.resets
            ));
        }
        out
    }

    pub fn passed(&self, c: &HoldCriteria) -> bool {
        self.failures(c).is_empty()
    }
}

/// Flies gust-hold run `index` on `profile` with the reference quad and
/// returns its metrics.
pub fn fly_gust_hold(
    profile: Profile,
    index: u64,
    c: &HoldCriteria,
) -> Result<HoldMetrics, SimError> {
    let airframe = Airframe::reference_quad()?;
    let (pilot, options) = gust_hold(index);
    let duration = Micros((pilot.duration_s * 1e6) as u64);
    let target = pilot.hold.cast::<f64>();
    let mut sim = Simulation::<_, 4>::with_options(&airframe, profile, pilot, options)?;
    let mut m = HoldMetrics::new();
    sim.run_until(duration, |r| m.add(r, target, c));
    Ok(m)
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxide_core::io::{StateSample, Status};
    use oxide_core::math::Quat;

    fn rec(t_s: f64, pos: Vec3<f64>) -> StepRecord<4> {
        StepRecord {
            t: Micros((t_s * 1e6) as u64),
            position: pos,
            velocity: Vec3::zero(),
            attitude: Quat::identity(),
            rate: Vec3::zero(),
            motors: [0.0; 4],
            rotor_speed: [0.0; 4],
            battery_v: 16.0,
            battery_a: 0.0,
            status: Status {
                armed: true,
                ..Status::default()
            },
            setpoint: None,
            estimate: Some(StateSample {
                position: pos.cast::<f32>(),
                velocity: Vec3::zero(),
                attitude: Quat::identity(),
                rate: Vec3::zero(),
            }),
            wind: Vec3::zero(),
            physics_faults: 0,
        }
    }

    fn target() -> Vec3<f64> {
        Vec3::new(0.0, 0.0, -2.0)
    }

    #[test]
    fn a_run_that_never_reaches_the_window_fails() {
        let c = HoldCriteria::default();
        let m = HoldMetrics::new();
        assert!(!m.passed(&c));
        assert!(
            m.failures(&c)
                .iter()
                .any(|f| f.contains("no ticks in the hold window"))
        );
    }

    #[test]
    fn rms_and_max_and_window_edges() {
        let c = HoldCriteria::default();
        let mut m = HoldMetrics::new();
        // Before the window: huge error, must not count in the hold errors.
        m.add(&rec(14.999, Vec3::new(9.0, 0.0, -2.0)), target(), &c);
        assert_eq!(m.max_h_error_m, 0.0);
        // At the window start (inclusive): 3-4-5 horizontal.
        m.add(&rec(15.0, Vec3::new(0.3, 0.4, -2.5)), target(), &c);
        m.add(&rec(16.0, Vec3::new(0.0, 0.0, -2.0)), target(), &c);
        assert!((m.max_h_error_m - 0.5).abs() < 1e-6);
        assert!((m.max_v_error_m - 0.5).abs() < 1e-6);
        // RMS of [0.5, 0.0] = sqrt(0.125).
        assert!((m.rms_h_error_m - Real::sqrt(0.125_f64)).abs() < 1e-6);
        assert!(m.passed(&c), "{:?}", m.failures(&c));
    }

    #[test]
    fn attitude_error_angle_and_tilt() {
        let c = HoldCriteria::default();
        let mut m = HoldMetrics::new();
        let mut r = rec(20.0, target());
        // Estimate rotated 2 degrees about z relative to truth.
        r.estimate = r.estimate.map(|mut e| {
            e.attitude = Quat::from_rotation_vector(Vec3::new(0.0, 0.0, 2.0_f32.to_radians()));
            e
        });
        // Truth tilted 10 degrees in roll.
        r.attitude = Quat::from_rotation_vector(Vec3::new(10.0_f64.to_radians(), 0.0, 0.0));
        m.add(&r, target(), &c);
        assert!((m.max_tilt_deg - 10.0).abs() < 1e-6, "{}", m.max_tilt_deg);
        // Truth and estimate differ by roll 10 deg and yaw 2 deg: angle is
        // about sqrt(10^2 + 2^2) for small angles.
        let ea = m.est_max_att_error_deg;
        assert!((ea - 10.2).abs() < 0.1, "att error {ea}");
    }

    #[test]
    fn estimate_errors_velocity_and_missing_estimate() {
        let c = HoldCriteria::default();
        let mut m = HoldMetrics::new();
        let mut r = rec(20.0, target());
        r.estimate = r.estimate.map(|mut e| {
            e.position = Vec3::new(0.6, 0.8, -2.0 + 0.25);
            e.velocity = Vec3::new(0.0, 0.3, 0.4);
            e
        });
        m.add(&r, target(), &c);
        assert!((m.est_max_h_error_m - 1.0).abs() < 1e-6);
        assert!((m.est_max_v_error_m - 0.25).abs() < 1e-6);
        assert!((m.est_max_vel_error_mps - 0.5).abs() < 1e-6);
        let mut none = rec(21.0, target());
        none.estimate = None;
        m.add(&none, target(), &c);
        assert_eq!(m.no_estimate_ticks, 1);
        assert!(!m.passed(&c));
    }

    #[test]
    fn a_physics_fault_fails_the_run() {
        let c = HoldCriteria::default();
        let mut m = HoldMetrics::new();
        m.add(&rec(16.0, target()), target(), &c);
        assert!(m.passed(&c), "{:?}", m.failures(&c));
        let mut bad = rec(17.0, target());
        bad.physics_faults = 1;
        m.add(&bad, target(), &c);
        assert_eq!(m.physics_faults, 1);
        assert!(
            m.failures(&c)
                .iter()
                .any(|f| f.contains("non-finite physics"))
        );
    }

    #[test]
    fn armed_and_fault_accounting() {
        let c = HoldCriteria::default();
        let mut m = HoldMetrics::new();
        let mut early = rec(3.4, target());
        early.status.armed = false;
        early.estimate = None;
        m.add(&early, target(), &c);
        assert_eq!((m.unarmed_ticks, m.no_estimate_ticks), (0, 0));
        let mut late = rec(3.5, target());
        late.status.armed = false;
        late.status.timing_faults = 2;
        late.status.estimator.resets = 2;
        m.add(&late, target(), &c);
        assert_eq!(m.unarmed_ticks, 1);
        assert_eq!(m.timing_faults, 2);
        // Not armed, timing faults, resets, and no tick in the hold window.
        let f = m.failures(&c);
        assert_eq!(f.len(), 4, "{f:?}");
        let mut nan = rec(20.0, target());
        nan.position.x = f64::NAN;
        m.add(&nan, target(), &c);
        assert!(m.non_finite);
    }
}
