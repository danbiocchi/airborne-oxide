// SPDX-License-Identifier: AGPL-3.0-or-later
//! IMU integration into delta angles and delta velocities (DESIGN §4.7).

use crate::math::Vec3;
use crate::time::Micros;

/// IMU deltas over one interval: what the EKF predicts with. Generic so the
/// filter can be tested in f64; the core uses f32.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ImuDelta<T> {
    /// End of the interval: the time of the last sample included.
    pub t_end: Micros,
    /// Interval length, s.
    pub dt: T,
    /// Integrated body angular rate, rad.
    pub dang: Vec3<T>,
    /// Integrated body specific force, m/s.
    pub dvel: Vec3<T>,
}

/// Accumulates per-tick IMU samples into deltas for the EKF.
///
/// Rectangular integration: each sample is held over the interval since the
/// previous sample. There is no coning or sculling correction.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImuAccumulator {
    /// Deltas since the last `take`. `acc.dt` is the accumulated interval.
    acc: ImuDelta<f32>,
    /// Time of the last valid sample, kept across `take` calls.
    last_t: Option<Micros>,
    /// Expected sample interval, s (the base tick).
    nominal_dt: f32,
    /// Valid samples added since the last `take`.
    samples: u32,
    /// Non-finite samples skipped, since construction.
    skipped: u32,
}

impl ImuAccumulator {
    /// `nominal_dt`: the expected sample interval, s (the base tick).
    pub fn new(nominal_dt: f32) -> Self {
        Self {
            acc: ImuDelta::default(),
            last_t: None,
            nominal_dt,
            samples: 0,
            skipped: 0,
        }
    }

    /// Adds one sample measured at `t`, held over the interval since the
    /// previous sample (rectangular integration, no coning/sculling
    /// correction). The interval is `t − last_t` when that is in
    /// (0, 4·nominal_dt]; otherwise (first sample, a gap, or time not
    /// increasing) it is `nominal_dt`. Non-finite samples are skipped and
    /// counted, and do not advance `last_t`.
    pub fn push(&mut self, t: Micros, gyro: Vec3<f32>, accel: Vec3<f32>) {
        if !gyro.is_finite() || !accel.is_finite() {
            self.skipped = self.skipped.saturating_add(1);
            return;
        }
        let dt = self.interval(t);
        self.acc.dang += gyro * dt;
        self.acc.dvel += accel * dt;
        self.acc.dt += dt;
        self.acc.t_end = t;
        self.samples = self.samples.saturating_add(1);
        self.last_t = Some(t);
    }

    /// The interval a sample at `t` is held for, s.
    fn interval(&self, t: Micros) -> f32 {
        let Some(last) = self.last_t else {
            return self.nominal_dt;
        };
        if t <= last {
            return self.nominal_dt;
        }
        let dt = (t.since(last) as f32) * 1e-6;
        if dt <= 4.0 * self.nominal_dt {
            dt
        } else {
            self.nominal_dt
        }
    }

    /// The accumulated delta (t_end = time of the last sample added), and
    /// starts a new one. `None` if no sample was added since the last take.
    pub fn take(&mut self) -> Option<ImuDelta<f32>> {
        if self.samples == 0 {
            return None;
        }
        let out = self.acc;
        self.acc = ImuDelta::default();
        self.samples = 0;
        Some(out)
    }

    /// Accumulated interval since the last `take`, s.
    pub fn accumulated_dt(&self) -> f32 {
        self.acc.dt
    }

    /// Non-finite samples skipped since construction.
    pub fn skipped(&self) -> u32 {
        self.skipped
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 0.001;

    fn us(t: u64) -> Micros {
        Micros(t)
    }

    #[test]
    fn constant_rates_integrate_to_rate_times_time() {
        let mut acc = ImuAccumulator::new(DT);
        assert_eq!(acc.take(), None);
        let w = Vec3::new(0.1_f32, -0.2, 0.3);
        let a = Vec3::new(0.0_f32, 0.5, -9.8);
        // First sample is held for nominal_dt; each later one for the 1 ms
        // since the previous sample. 11 samples at 0..=10 ms give 11 ms.
        for k in 0..=10u64 {
            acc.push(us(k * 1000), w, a);
        }
        let d = acc.take().expect("samples were added");
        let total = 11.0 * DT;
        assert!((d.dt - total).abs() < 1e-6, "dt = {}", d.dt);
        assert!((d.dang.x - 0.1 * total).abs() < 1e-6);
        assert!((d.dang.y + 0.2 * total).abs() < 1e-6);
        assert!((d.dang.z - 0.3 * total).abs() < 1e-6);
        assert!((d.dvel.y - 0.5 * total).abs() < 1e-6);
        assert!((d.dvel.z + 9.8 * total).abs() < 1e-5);
        assert_eq!(d.t_end, us(10_000));
    }

    #[test]
    fn take_resets_and_keeps_interval_continuity() {
        let mut acc = ImuAccumulator::new(DT);
        acc.push(us(0), Vec3::splat(1.0), Vec3::zero());
        let first = acc.take().expect("one sample");
        assert!((first.dt - DT).abs() < 1e-9);
        assert_eq!(acc.accumulated_dt(), 0.0);
        assert_eq!(acc.take(), None);
        // 1 ms after the last sample, so it is held for 1 ms.
        acc.push(us(1000), Vec3::splat(2.0), Vec3::zero());
        assert!((acc.accumulated_dt() - DT).abs() < 1e-9);
        let second = acc.take().expect("one sample");
        assert!((second.dang.x - 2.0 * DT).abs() < 1e-9);
        assert_eq!(second.t_end, us(1000));
    }

    #[test]
    fn gap_uses_nominal_interval() {
        let mut acc = ImuAccumulator::new(DT);
        acc.push(us(0), Vec3::zero(), Vec3::zero());
        // 4 ms is still within 4·nominal and is accepted.
        acc.push(us(4000), Vec3::zero(), Vec3::zero());
        assert!((acc.accumulated_dt() - 5.0 * DT).abs() < 1e-7);
        let _ = acc.take();
        // 5 ms since the last sample is a gap: held for nominal_dt.
        acc.push(us(9001), Vec3::zero(), Vec3::zero());
        assert!((acc.accumulated_dt() - DT).abs() < 1e-9);
    }

    #[test]
    fn time_not_increasing_uses_nominal_interval() {
        let mut acc = ImuAccumulator::new(DT);
        acc.push(us(5000), Vec3::zero(), Vec3::zero());
        acc.push(us(5000), Vec3::zero(), Vec3::zero());
        acc.push(us(3000), Vec3::zero(), Vec3::zero());
        assert!((acc.accumulated_dt() - 3.0 * DT).abs() < 1e-7);
    }

    #[test]
    fn non_finite_samples_are_skipped_and_do_not_advance_time() {
        let mut acc = ImuAccumulator::new(DT);
        acc.push(us(0), Vec3::splat(1.0), Vec3::zero());
        acc.push(us(1000), Vec3::new(f32::NAN, 0.0, 0.0), Vec3::zero());
        acc.push(us(2000), Vec3::zero(), Vec3::new(0.0, f32::INFINITY, 0.0));
        assert_eq!(acc.skipped(), 2);
        // Held over 3 ms from the last valid sample at 0 ms.
        acc.push(us(3000), Vec3::splat(1.0), Vec3::zero());
        let d = acc.take().expect("valid samples");
        assert!((d.dt - 4.0 * DT).abs() < 1e-7, "dt = {}", d.dt);
        assert!(d.dang.is_finite());
        assert_eq!(d.t_end, us(3000));
    }

    #[test]
    fn only_skipped_samples_yield_nothing() {
        let mut acc = ImuAccumulator::new(DT);
        acc.push(us(0), Vec3::new(f32::NAN, 0.0, 0.0), Vec3::zero());
        assert_eq!(acc.take(), None);
        assert_eq!(acc.skipped(), 1);
    }
}
