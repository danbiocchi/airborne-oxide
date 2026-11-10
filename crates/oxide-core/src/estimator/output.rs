// SPDX-License-Identifier: AGPL-3.0-or-later
//! Output predictor (DESIGN §4.7, ADR-0002): a current-time state integrated
//! with every IMU sample and pulled towards the delayed-horizon EKF by
//! comparing the two at the same (delayed) time. Same structure as PX4
//! EKF2's output predictor.

use super::ekf::NominalState;
use super::ring::Ring;
use crate::GRAVITY;
use crate::math::{Quat, Vec3};
use crate::time::Micros;

/// History depth: one entry per EKF step over the delay horizon.
const HISTORY: usize = 64;

/// The predictor's state. World frame NED, body frame FRD.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct OutputState {
    /// Time of the last IMU sample integrated.
    pub t: Micros,
    /// Attitude, body to world.
    pub q: Quat<f32>,
    /// Velocity, m/s.
    pub v: Vec3<f32>,
    /// Position, m.
    pub p: Vec3<f32>,
}

/// Integrates every IMU sample and tracks the EKF with first-order
/// corrections.
#[derive(Clone, Debug)]
pub struct OutputPredictor {
    state: OutputState,
    history: Ring<OutputState, HISTORY>,
    /// Attitude correction rate, world frame, rad/s.
    att_corr: Vec3<f32>,
    /// Correction time constant, s.
    tau: f32,
    /// Bias-corrected body rate of the latest sample, rad/s.
    rate: Vec3<f32>,
}

impl OutputPredictor {
    /// `tau_s`: correction time constant, s (floored at 1 ms; a non-finite
    /// value gives 0.25 s).
    pub fn new(tau_s: f32) -> Self {
        let tau = if tau_s.is_finite() {
            tau_s.max(1e-3)
        } else {
            0.25
        };
        Self {
            state: OutputState::default(),
            history: Ring::new(),
            att_corr: Vec3::zero(),
            tau,
            rate: Vec3::zero(),
        }
    }

    /// Restarts at the given state; clears history and corrections.
    pub fn reset(&mut self, t: Micros, q: Quat<f32>, v: Vec3<f32>, p: Vec3<f32>) {
        self.state = OutputState {
            t,
            q: q.normalized(),
            v,
            p,
        };
        self.history.clear();
        self.att_corr = Vec3::zero();
        self.rate = Vec3::zero();
    }

    /// One IMU sample (body gyro rad/s, specific force m/s²) measured at `t`
    /// and held for `dt` s, with the EKF's current bias estimates. Samples
    /// that are not finite, or `dt <= 0`, are skipped.
    pub fn propagate(
        &mut self,
        t: Micros,
        gyro: Vec3<f32>,
        accel: Vec3<f32>,
        dt: f32,
        bg: Vec3<f32>,
        ba: Vec3<f32>,
    ) {
        let w = gyro - bg;
        let a = accel - ba;
        if !(dt > 0.0 && dt.is_finite()) || !w.is_finite() || !a.is_finite() {
            return;
        }
        let corr = Quat::from_rotation_vector(self.att_corr * dt);
        let dq = Quat::from_rotation_vector(w * dt);
        let q = (corr * self.state.q * dq).normalized();
        let v_prev = self.state.v;
        let v = v_prev + q.rotate(a) * dt + Vec3::new(0.0, 0.0, GRAVITY) * dt;
        self.state.q = q;
        self.state.v = v;
        self.state.p += (v_prev + v) * (0.5 * dt);
        self.state.t = t;
        self.rate = w;
    }

    /// Saves the current state in the history (call when an IMU delta closes,
    /// so history times match the EKF's delta end times).
    pub fn record(&mut self) {
        self.history.push(self.state);
    }

    /// Saves the current state in the history stamped `t`, without changing
    /// the current state (used to seed the history at alignment).
    pub fn record_at(&mut self, t: Micros) {
        self.history.push(OutputState { t, ..self.state });
    }

    /// Pulls towards the EKF state `x` valid at `t_ekf`. `filter_dt` is the
    /// EKF step, s. Does nothing unless the history holds an entry at exactly
    /// `t_ekf`.
    pub fn correct(&mut self, t_ekf: Micros, x: &NominalState<f32>, filter_dt: f32) {
        while self.history.front().is_some_and(|o| o.t < t_ekf) {
            self.history.pop_front();
        }
        let Some(o) = self.history.front().copied().filter(|o| o.t == t_ekf) else {
            return;
        };
        let e = (x.q * o.q.conj()).to_rotation_vector();
        let k = (filter_dt / self.tau).clamp(0.0, 1.0);
        let dv = (x.v - o.v) * k;
        let dp = (x.p - o.p) * k;
        if !(e.is_finite() && dv.is_finite() && dp.is_finite() && k.is_finite()) {
            return;
        }
        self.att_corr = e * (1.0 / self.tau);
        self.shift(dp, dv);
    }

    /// Applies a state reset made by the EKF (position/velocity jump) to the
    /// current state and the whole history.
    pub fn shift(&mut self, dp: Vec3<f32>, dv: Vec3<f32>) {
        if !(dp.is_finite() && dv.is_finite()) {
            return;
        }
        self.state.p += dp;
        self.state.v += dv;
        self.history.for_each_mut(|s| {
            s.p += dp;
            s.v += dv;
        });
    }

    /// Applies a world-frame yaw change made by the EKF (a heading reset)
    /// to the current attitude and every history attitude. Position and
    /// velocity are world-frame and do not change.
    pub fn rotate_yaw(&mut self, dpsi: f32) {
        if !dpsi.is_finite() {
            return;
        }
        let r = Quat::from_rotation_vector(Vec3::new(0.0, 0.0, dpsi));
        self.state.q = (r * self.state.q).normalized();
        self.history.for_each_mut(|s| s.q = (r * s.q).normalized());
    }

    pub fn state(&self) -> &OutputState {
        &self.state
    }

    /// Bias-corrected body rate from the latest sample, rad/s.
    pub fn rate(&self) -> Vec3<f32> {
        self.rate
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TAU: f32 = 0.25;
    const STEP_US: u64 = 4000;

    fn rest_accel() -> Vec3<f32> {
        Vec3::new(0.0, 0.0, -GRAVITY)
    }

    /// Runs 4 ms steps at rest (1 kHz samples, a record per step) against a
    /// fixed EKF state `x`, applying the correction 200 ms behind.
    fn run_against(op: &mut OutputPredictor, x: &NominalState<f32>, steps: u64) {
        let delay_steps = 50;
        for k in 0..steps {
            for j in 1..=4u64 {
                let t = Micros((k * 4 + j) * 1000);
                op.propagate(
                    t,
                    Vec3::zero(),
                    rest_accel(),
                    0.001,
                    Vec3::zero(),
                    Vec3::zero(),
                );
            }
            op.record();
            if k >= delay_steps {
                let t_ekf = Micros((k - delay_steps + 1) * STEP_US);
                op.correct(t_ekf, x, 0.004);
            }
        }
    }

    fn nominal(q: Quat<f32>, v: Vec3<f32>, p: Vec3<f32>) -> NominalState<f32> {
        NominalState {
            q,
            v,
            p,
            bg: Vec3::zero(),
            ba: Vec3::zero(),
        }
    }

    #[test]
    fn rest_stays_at_rest() {
        let mut op = OutputPredictor::new(TAU);
        op.reset(Micros::ZERO, Quat::identity(), Vec3::zero(), Vec3::zero());
        for k in 1..=2000u64 {
            op.propagate(
                Micros(k * 1000),
                Vec3::zero(),
                rest_accel(),
                0.001,
                Vec3::zero(),
                Vec3::zero(),
            );
        }
        assert!(op.state().v.norm() < 1e-3, "v = {:?}", op.state().v);
        assert!(op.state().p.norm() < 1e-2, "p = {:?}", op.state().p);
        assert_eq!(op.state().t, Micros(2_000_000));
    }

    #[test]
    fn non_finite_and_zero_dt_are_skipped() {
        let mut op = OutputPredictor::new(TAU);
        op.reset(Micros::ZERO, Quat::identity(), Vec3::zero(), Vec3::zero());
        let before = *op.state();
        let z = Vec3::zero();
        op.propagate(Micros(1000), Vec3::new(f32::NAN, 0.0, 0.0), z, 0.001, z, z);
        op.propagate(Micros(1000), z, z, 0.0, z, z);
        op.propagate(
            Micros(1000),
            z,
            z,
            0.001,
            Vec3::new(f32::INFINITY, 0.0, 0.0),
            z,
        );
        assert_eq!(*op.state(), before);
    }

    #[test]
    fn integrates_rotation_and_bias() {
        let mut op = OutputPredictor::new(TAU);
        op.reset(Micros::ZERO, Quat::identity(), Vec3::zero(), Vec3::zero());
        // 1 rad/s yaw measured, 0.5 rad/s of it bias: 0.5 rad over 1 s.
        let bg = Vec3::new(0.0, 0.0, 0.5);
        for k in 1..=1000u64 {
            op.propagate(
                Micros(k * 1000),
                Vec3::new(0.0, 0.0, 1.0),
                rest_accel(),
                0.001,
                bg,
                Vec3::zero(),
            );
        }
        assert!((op.state().q.yaw() - 0.5).abs() < 1e-3);
        assert!((op.rate().z - 0.5).abs() < 1e-6);
    }

    #[test]
    fn correct_without_matching_entry_does_nothing() {
        let mut op = OutputPredictor::new(TAU);
        op.reset(Micros::ZERO, Quat::identity(), Vec3::zero(), Vec3::zero());
        op.record_at(Micros(4000));
        let x = nominal(Quat::identity(), Vec3::zero(), Vec3::new(1.0, 0.0, 0.0));
        op.correct(Micros(2000), &x, 0.004);
        assert_eq!(op.state().p, Vec3::zero());
        // The entry at 4000 was dropped above only if older than 2000: it is
        // still there, so a correction at 4000 applies.
        op.correct(Micros(4000), &x, 0.004);
        assert!((op.state().p.x - 0.004 / TAU).abs() < 1e-6);
    }

    #[test]
    fn tracks_the_ekf_with_time_constant_tau() {
        let mut op = OutputPredictor::new(TAU);
        op.reset(Micros::ZERO, Quat::identity(), Vec3::zero(), Vec3::zero());
        let q = Quat::from_euler(0.0, 0.0, 0.2);
        let x = nominal(q, Vec3::zero(), Vec3::new(1.0, -2.0, 0.5));
        run_against(&mut op, &x, 1250); // 5 s
        let s = op.state();
        assert!((s.p - x.p).norm() < 0.02, "p = {:?}", s.p);
        assert!(s.v.norm() < 0.05, "v = {:?}", s.v);
        assert!((s.q.yaw() - 0.2).abs() < 0.01, "yaw = {}", s.q.yaw());
    }

    #[test]
    fn shift_moves_state_and_history() {
        let mut op = OutputPredictor::new(TAU);
        op.reset(Micros::ZERO, Quat::identity(), Vec3::zero(), Vec3::zero());
        op.record_at(Micros(4000));
        op.shift(Vec3::new(10.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0));
        assert_eq!(op.state().p.x, 10.0);
        assert_eq!(op.state().v.y, 1.0);
        // The history entry moved too: a correction to the shifted value
        // sees no error.
        let x = nominal(
            Quat::identity(),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(10.0, 0.0, 0.0),
        );
        op.correct(Micros(4000), &x, 0.004);
        assert!((op.state().p.x - 10.0).abs() < 1e-6);
        assert!((op.state().v.y - 1.0).abs() < 1e-6);
    }
}
