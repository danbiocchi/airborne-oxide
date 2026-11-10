// SPDX-License-Identifier: AGPL-3.0-or-later
//! State estimation (DESIGN §4.7, ADR-0002): an error-state EKF on a delayed
//! time horizon, fed by IMU deltas, with an output predictor that brings the
//! state forward to the current tick.
//!
//! [`Estimator`] orchestrates the pieces. Sensor samples are pushed as they
//! arrive; [`Estimator::update`] runs once per base tick. Until the vehicle
//! has sat still for `align_time_us` the filter does not exist and IMU deltas
//! are only buffered. After alignment the EKF runs `delay_horizon_us` behind
//! real time so that every sensor (GPS above all) can be fused at the time it
//! was measured, and the output predictor carries the result forward.

pub mod ekf;
pub mod imu;
pub mod output;
pub mod ring;

use crate::GRAVITY;
use crate::config::{ConfigError, EkfParams};
use crate::io::{
    BaroSample, EstimatorStatus, GpsSample, ImuSample, MagSample, Stamped, StateSample,
};
use crate::math::{Quat, Real, Vec3};
use crate::schedule::Profile;
use crate::time::Micros;
use ekf::{Ekf, Fusion, GYRO_BIAS_LIMIT, heading_from_mag, tilt_from_accel};
use imu::{ImuAccumulator, ImuDelta};
use output::OutputPredictor;
use ring::Ring;

/// IMU delta ring capacity (also the output predictor's history depth).
pub const IMU_BUFFER: usize = 64;
/// GPS sample ring capacity.
pub const GPS_BUFFER: usize = 8;
/// Barometer and magnetometer ring capacity.
pub const OBS_BUFFER: usize = 32;

/// Longest gap between accepted IMU samples before the estimate is dropped
/// (ADR-0001 item 4: state staleness), µs.
pub const IMU_TIMEOUT_US: u64 = 100_000;

/// Stillness test for alignment: mean specific force within these multiples
/// of g, and mean rate under `STILL_RATE` rad/s.
const STILL_ACCEL_LO: f32 = 0.9;
const STILL_ACCEL_HI: f32 = 1.1;
const STILL_RATE: f32 = 0.05;

/// Sums over the current stillness window.
#[derive(Clone, Copy, Debug, Default)]
struct AlignWindow {
    /// Window length, s.
    dt: f32,
    /// Summed delta velocity, m/s.
    dvel: Vec3<f32>,
    /// Summed delta angle, rad (its mean rate is the gyro bias at rest).
    dang: Vec3<f32>,
    /// Running mean of the magnetometer, µT, and its sample count.
    mag_mean: Vec3<f32>,
    mag_n: u32,
    /// Running mean of the barometric altitude, m, and its sample count.
    baro_mean: f32,
    baro_n: u32,
}

/// GPS aiding bookkeeping.
#[derive(Clone, Copy, Debug, Default)]
struct GpsTrack {
    /// GPS has been fused (or used to reset the state) at least once.
    fused: bool,
    /// `t_sample` of the last accepted fix.
    last_accept: Micros,
    /// `t_sample` of the first fix of the current run of rejections.
    reject_since: Option<Micros>,
    /// Same, counting only the position part of the fix.
    pos_reject_since: Option<Micros>,
}

/// The estimator: alignment, the delayed-horizon EKF and the output
/// predictor.
#[derive(Clone, Debug)]
pub struct Estimator {
    params: EkfParams,
    horizon_us: u64,
    gps_timeout_us: u64,
    /// Filter period in base ticks (at least 1).
    period_ticks: u64,
    /// Filter period, s.
    filter_dt: f32,
    /// Base tick, s (for the IMU accumulator).
    base_dt: f32,
    ticks: u64,

    acc: ImuAccumulator,
    imu_ring: Ring<ImuDelta<f32>, IMU_BUFFER>,
    gps_ring: Ring<Stamped<GpsSample>, GPS_BUFFER>,
    baro_ring: Ring<Stamped<BaroSample>, OBS_BUFFER>,
    mag_ring: Ring<Stamped<MagSample>, OBS_BUFFER>,

    window: AlignWindow,
    ekf: Option<Ekf<f32>>,
    output: OutputPredictor,
    /// Barometer altitude at the alignment point, m.
    baro_ref: Option<f32>,
    /// End time of the last delta the EKF consumed (its current time).
    t_ekf: Micros,
    gps: GpsTrack,
    /// The yaw comes from the magnetometer (alignment saw one, or the first
    /// usable reading since reset the heading).
    heading_ok: bool,
    /// `t_arrival` of the last IMU sample accepted.
    last_imu_arrival: Option<Micros>,
    raw_gyro: Vec3<f32>,
    rejected: u32,
    resets: u32,
}

/// True for a finite, strictly positive value (false for NaN).
fn pos(x: f32) -> bool {
    x.is_finite() && x > 0.0
}

impl Estimator {
    /// Fails with [`ConfigError::BadEstimator`] if a parameter is non-finite
    /// or non-positive where it must be positive (noises, gates, horizon,
    /// time constant, align time, GPS timeout, initial sigmas), or if the
    /// delay horizon does not fit the IMU buffer: `ceil(delay_horizon_us /
    /// filter period) + 2 > IMU_BUFFER`. A malformed profile fails with
    /// [`ConfigError::BadProfile`].
    pub fn new(params: &EkfParams, profile: &Profile) -> Result<Self, ConfigError> {
        profile.validate()?;
        let p = params;
        let positive = [
            p.gyro_noise,
            p.accel_noise,
            p.gyro_bias_noise,
            p.accel_bias_noise,
            p.gps_pos_noise_m,
            p.gps_vel_noise_mps,
            p.baro_noise_m,
            p.yaw_noise_rad,
            p.gps_gate,
            p.baro_gate,
            p.yaw_gate,
            p.output_tau_s,
            p.init_tilt_sigma_rad,
            p.init_yaw_sigma_rad,
            p.init_vel_sigma_mps,
            p.init_pos_sigma_m,
            p.init_gyro_bias_sigma,
            p.init_accel_bias_sigma,
        ];
        if !positive.iter().all(|x| pos(*x))
            || !p.mag_declination_rad.is_finite()
            || p.delay_horizon_us == 0
            || p.align_time_us == 0
            || p.gps_timeout_us == 0
        {
            return Err(ConfigError::BadEstimator);
        }
        let period_us = u64::from(profile.filter_period_ticks) * u64::from(profile.base_period_us);
        let needed = u64::from(p.delay_horizon_us).div_ceil(period_us.max(1)) + 2;
        if period_us == 0 || needed > IMU_BUFFER as u64 {
            return Err(ConfigError::BadEstimator);
        }
        Ok(Self {
            params: *p,
            horizon_us: u64::from(p.delay_horizon_us),
            gps_timeout_us: u64::from(p.gps_timeout_us),
            period_ticks: u64::from(profile.filter_period_ticks),
            filter_dt: profile.filter_dt(),
            base_dt: profile.base_dt(),
            ticks: 0,
            acc: ImuAccumulator::new(profile.base_dt()),
            imu_ring: Ring::new(),
            gps_ring: Ring::new(),
            baro_ring: Ring::new(),
            mag_ring: Ring::new(),
            window: AlignWindow::default(),
            ekf: None,
            output: OutputPredictor::new(p.output_tau_s),
            baro_ref: None,
            t_ekf: Micros::ZERO,
            gps: GpsTrack::default(),
            heading_ok: false,
            last_imu_arrival: None,
            raw_gyro: Vec3::zero(),
            rejected: 0,
            resets: 0,
        })
    }

    /// Adds an IMU sample. Once aligned it also advances the output
    /// predictor, with exactly the interval the accumulator used.
    pub fn push_imu(&mut self, s: &Stamped<ImuSample>) {
        let v = s.value;
        let Some(dt) = self.acc.push(s.t_sample, v.gyro, v.accel) else {
            return;
        };
        self.raw_gyro = v.gyro;
        self.last_imu_arrival = Some(s.t_arrival);
        if let Some(ekf) = &self.ekf {
            self.output
                .propagate(s.t_sample, v.gyro, v.accel, dt, ekf.x.bg, ekf.x.ba);
        }
    }

    /// Queues a GPS fix (`t_sample` is when it was measured).
    pub fn push_gps(&mut self, s: &Stamped<GpsSample>) {
        if self.too_late(s.t_sample) {
            return;
        }
        self.gps_ring.push(*s);
    }

    /// Queues a barometer sample.
    pub fn push_baro(&mut self, s: &Stamped<BaroSample>) {
        if self.too_late(s.t_sample) {
            return;
        }
        self.baro_ring.push(*s);
    }

    /// Queues a magnetometer sample.
    pub fn push_mag(&mut self, s: &Stamped<MagSample>) {
        if self.too_late(s.t_sample) {
            return;
        }
        self.mag_ring.push(*s);
    }

    /// After alignment, a sample older than the EKF's current time arrived
    /// later than the delay horizon allows: count it and drop it.
    fn too_late(&mut self, t_sample: Micros) -> bool {
        if self.ekf.is_some() && t_sample < self.t_ekf {
            self.rejected = self.rejected.saturating_add(1);
            true
        } else {
            false
        }
    }

    /// Once per base tick, after this tick's samples were pushed. Every
    /// `filter_period_ticks`-th call runs the filter step.
    pub fn update(&mut self, now: Micros) {
        if self.ekf.is_some()
            && self
                .last_imu_arrival
                .is_none_or(|t| now.since(t) > IMU_TIMEOUT_US)
        {
            self.drop_to_unaligned();
        }
        self.ticks = self.ticks.saturating_add(1);
        if !self.ticks.is_multiple_of(self.period_ticks) {
            return;
        }
        let Some(delta) = self.acc.take() else {
            return;
        };
        if self.ekf.is_some() {
            self.run_filter(now, delta);
        } else {
            self.align_step(now, delta);
        }
    }

    /// Current-time estimate: `Some` once aligned, the heading is
    /// initialised and GPS has been fused at least once, and everything is
    /// finite.
    pub fn estimate(&self) -> Option<StateSample> {
        if self.ekf.is_none() || !self.heading_ok || !self.gps.fused {
            return None;
        }
        let o = self.output.state();
        let est = StateSample {
            position: o.p,
            velocity: o.v,
            attitude: o.q,
            rate: self.output.rate(),
        };
        (est.position.is_finite()
            && est.velocity.is_finite()
            && est.attitude.is_finite()
            && est.rate.is_finite())
        .then_some(est)
    }

    /// Estimated gyro bias, rad/s; zero before alignment.
    pub fn gyro_bias(&self) -> Vec3<f32> {
        self.ekf.as_ref().map_or_else(Vec3::zero, |e| e.x.bg)
    }

    /// The latest finite raw gyro sample, rad/s.
    pub fn raw_gyro(&self) -> Vec3<f32> {
        self.raw_gyro
    }

    pub fn status(&self) -> EstimatorStatus {
        let aiding = self.gps.fused
            && self.t_ekf.since(self.gps.last_accept) <= self.gps_timeout_us
            && self.ekf.is_some();
        EstimatorStatus {
            aligned: self.ekf.is_some() && self.heading_ok,
            gps_aiding: aiding,
            rejected: self.rejected,
            resets: self.resets,
        }
    }

    /// The delayed-horizon filter, for tests and logging.
    pub fn ekf(&self) -> Option<&Ekf<f32>> {
        self.ekf.as_ref()
    }

    // ---- alignment ------------------------------------------------------

    fn restart_alignment(&mut self) {
        self.window = AlignWindow::default();
        self.imu_ring.clear();
        // Fixes measured before the vehicle was still say nothing about
        // where it is now.
        self.gps_ring.clear();
    }

    /// An IMU stall made the dead-reckoned state untrustworthy: back to the
    /// unaligned state, counted as a reset.
    fn drop_to_unaligned(&mut self) {
        self.ekf = None;
        self.acc = ImuAccumulator::new(self.base_dt);
        self.imu_ring.clear();
        self.gps_ring.clear();
        self.baro_ring.clear();
        self.mag_ring.clear();
        self.window = AlignWindow::default();
        self.baro_ref = None;
        self.gps = GpsTrack::default();
        self.heading_ok = false;
        self.last_imu_arrival = None;
        self.resets = self.resets.saturating_add(1);
    }

    /// Drains the mag and baro rings into the running means.
    fn drain_alignment_obs(&mut self, accumulate: bool) {
        while let Some(m) = self.mag_ring.pop_front() {
            if accumulate && m.value.field_ut.is_finite() {
                let w = &mut self.window;
                w.mag_n = w.mag_n.saturating_add(1);
                w.mag_mean += (m.value.field_ut - w.mag_mean) * (1.0 / count_f32(w.mag_n));
            }
        }
        while let Some(b) = self.baro_ring.pop_front() {
            if accumulate && b.value.altitude_m.is_finite() {
                let w = &mut self.window;
                w.baro_n = w.baro_n.saturating_add(1);
                w.baro_mean += (b.value.altitude_m - w.baro_mean) * (1.0 / count_f32(w.baro_n));
            }
        }
    }

    fn align_step(&mut self, now: Micros, delta: ImuDelta<f32>) {
        let dt = delta.dt;
        let mean_accel = delta.dvel * (1.0 / dt);
        let mean_rate = delta.dang * (1.0 / dt);
        let f = mean_accel.norm();
        let still = dt > 0.0
            && f.is_finite()
            && (STILL_ACCEL_LO * GRAVITY..=STILL_ACCEL_HI * GRAVITY).contains(&f)
            && mean_rate.norm() < STILL_RATE;
        if !still {
            self.restart_alignment();
            self.drain_alignment_obs(false);
            return;
        }
        // Keep the ring within the delay horizon so that it fits from the
        // first filter step on.
        while self
            .imu_ring
            .front()
            .is_some_and(|f| f.t_end.add_us(self.horizon_us) <= delta.t_end)
        {
            self.imu_ring.pop_front();
        }
        self.imu_ring.push(delta);
        self.window.dt += dt;
        self.window.dvel += delta.dvel;
        self.window.dang += delta.dang;
        self.drain_alignment_obs(true);

        let need = micros_to_s(u64::from(self.params.align_time_us));
        if self.window.dt < need {
            return;
        }
        let w = self.window;
        let accel = w.dvel * (1.0 / w.dt);
        let mag = (w.mag_n > 0).then_some(w.mag_mean);
        let Some((roll, pitch)) = tilt_from_accel(accel) else {
            self.restart_alignment();
            return;
        };
        let heading =
            mag.and_then(|m| heading_from_mag(m, roll, pitch, self.params.mag_declination_rad));
        let q0 = Quat::from_euler(roll, pitch, heading.unwrap_or(0.0));
        let mut ekf = Ekf::new(&self.params, q0);
        // At rest the mean gyro rate is the turn-on bias (Earth rate is
        // negligible).
        let limit = <f32 as Real>::from_f64(GYRO_BIAS_LIMIT);
        ekf.x.bg = (w.dang * (1.0 / w.dt)).clamp_abs(Vec3::splat(limit));
        self.ekf = Some(ekf);
        self.baro_ref = (w.baro_n > 0).then_some(w.baro_mean);
        self.output.reset(now, q0, Vec3::zero(), Vec3::zero());
        for d in self.imu_ring.iter() {
            self.output.record_at(d.t_end);
        }
        // The first predict integrates the front delta, so the initial
        // state is valid at the start of that delta.
        self.t_ekf = self.imu_ring.front().map_or(now, |d| {
            Micros(d.t_end.0.saturating_sub(secs_to_micros(d.dt)))
        });
        self.heading_ok = heading.is_some();
        // Drop fixes measured before the filter's start time.
        for _ in 0..self.gps_ring.len() {
            if let Some(g) = self.gps_ring.pop_front()
                && g.t_sample >= self.t_ekf
            {
                self.gps_ring.push(g);
            }
        }
        self.window = AlignWindow::default();
    }

    // ---- filter ----------------------------------------------------------

    fn run_filter(&mut self, now: Micros, delta: ImuDelta<f32>) {
        let Some(mut ekf) = self.ekf.take() else {
            return;
        };
        self.imu_ring.push(delta);
        self.output.record();
        while let Some(d) = self.imu_ring.front().copied() {
            if d.t_end.add_us(self.horizon_us) > now {
                break;
            }
            self.imu_ring.pop_front();
            ekf.predict(&d);
            self.t_ekf = d.t_end;
            self.fuse_due(&mut ekf, d.t_end);
            self.output.correct(d.t_end, &ekf.x, self.filter_dt);
        }
        self.ekf = Some(ekf);
    }

    /// Fuses every buffered observation measured at or before `t`.
    fn fuse_due(&mut self, ekf: &mut Ekf<f32>, t: Micros) {
        while let Some(s) = self.baro_ring.front().copied() {
            if s.t_sample > t {
                break;
            }
            self.baro_ring.pop_front();
            self.fuse_baro(ekf, &s);
        }
        while let Some(s) = self.mag_ring.front().copied() {
            if s.t_sample > t {
                break;
            }
            self.mag_ring.pop_front();
            self.fuse_mag(ekf, &s);
        }
        while let Some(s) = self.gps_ring.front().copied() {
            if s.t_sample > t {
                break;
            }
            self.gps_ring.pop_front();
            self.fuse_gps(ekf, &s);
        }
    }

    /// Counts a rejected fusion; returns the result for inspection.
    fn note(&mut self, f: Fusion<f32>) -> Fusion<f32> {
        if matches!(f, Fusion::Rejected { .. }) {
            self.rejected = self.rejected.saturating_add(1);
        }
        f
    }

    fn fuse_baro(&mut self, ekf: &mut Ekf<f32>, s: &Stamped<BaroSample>) {
        let alt = s.value.altitude_m;
        if !alt.is_finite() {
            return;
        }
        match self.baro_ref {
            // No barometer reading at alignment: the first one defines zero.
            None => self.baro_ref = Some(alt),
            Some(r) => {
                let f = ekf.fuse_height(alt - r, self.params.baro_noise_m);
                self.note(f);
            }
        }
    }

    fn fuse_mag(&mut self, ekf: &mut Ekf<f32>, s: &Stamped<MagSample>) {
        if !s.value.field_ut.is_finite() {
            return;
        }
        if !self.heading_ok {
            // No heading at alignment: take the first usable reading as the
            // heading instead of gating it against a guess.
            if let Some(psi) = ekf.mag_heading(s.value.field_ut) {
                let dpsi = ekf.reset_yaw(psi, self.params.init_yaw_sigma_rad);
                self.output.rotate_yaw(dpsi);
                self.heading_ok = true;
                self.resets = self.resets.saturating_add(1);
            }
            return;
        }
        let f = ekf.fuse_mag_heading(s.value.field_ut, self.params.yaw_noise_rad);
        self.note(f);
    }

    fn fuse_gps(&mut self, ekf: &mut Ekf<f32>, s: &Stamped<GpsSample>) {
        let g = s.value;
        if !g.fix_3d || !g.position.is_finite() || !g.velocity.is_finite() {
            return;
        }
        let sigma_p = g.h_acc_m.finite_or(0.0).max(self.params.gps_pos_noise_m);
        let sigma_v = g
            .s_acc_mps
            .finite_or(0.0)
            .max(self.params.gps_vel_noise_mps);
        let t = s.t_sample;

        let timed_out = self.gps.fused && t.since(self.gps.last_accept) > self.gps_timeout_us;
        if !self.gps.fused || timed_out {
            self.reset_to_gps(ekf, s, sigma_p, sigma_v);
            return;
        }
        let pos = self.note(ekf.fuse_gps_position_ne([g.position.x, g.position.y], sigma_p));
        let vel = self.note(ekf.fuse_gps_velocity(g.velocity, sigma_v));
        let accepted = |f: Fusion<f32>| matches!(f, Fusion::Accepted { .. });
        let rejected = |f: Fusion<f32>| matches!(f, Fusion::Rejected { .. });
        if accepted(pos) || accepted(vel) {
            self.gps.last_accept = t;
            self.gps.reject_since = None;
        }
        // Two timers: the whole fix rejected (nothing accepted), and the
        // position alone rejected. The second catches a position offset that
        // comes with consistent velocity, which the first never sees because
        // the velocity keeps being accepted.
        let all_rejected = !(accepted(pos) || accepted(vel)) && (rejected(pos) || rejected(vel));
        let mut stuck = false;
        if all_rejected {
            let since = *self.gps.reject_since.get_or_insert(t);
            stuck |= t.since(since) > self.gps_timeout_us;
        }
        if rejected(pos) {
            let since = *self.gps.pos_reject_since.get_or_insert(t);
            stuck |= t.since(since) > self.gps_timeout_us;
        } else if accepted(pos) {
            self.gps.pos_reject_since = None;
        }
        if stuck {
            self.reset_to_gps(ekf, s, sigma_p, sigma_v);
        }
    }

    /// Snaps position (N/E) and velocity to a GPS fix.
    fn reset_to_gps(
        &mut self,
        ekf: &mut Ekf<f32>,
        s: &Stamped<GpsSample>,
        sigma_p: f32,
        sigma_v: f32,
    ) {
        let g = s.value;
        let dp = ekf.reset_position_ne([g.position.x, g.position.y], sigma_p);
        let dv = ekf.reset_velocity(g.velocity, sigma_v);
        self.output.shift(dp, dv);
        self.resets = self.resets.saturating_add(1);
        self.gps.fused = true;
        self.gps.last_accept = s.t_sample;
        self.gps.reject_since = None;
        self.gps.pos_reject_since = None;
    }
}

/// A sample count as f32 (counts here stay far below 2^24).
#[allow(clippy::cast_precision_loss)]
fn count_f32(n: u32) -> f32 {
    n as f32
}

/// Seconds as whole microseconds (rounded; non-negative).
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
fn secs_to_micros(s: f32) -> u64 {
    (f64::from(s) * 1e6 + 0.5).max(0.0) as u64
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
    use crate::schedule::{F405_LEAN, H743_FULL};

    const MAG: Vec3<f32> = Vec3::new(25.0, 0.0, 43.3);

    /// Drives an estimator with synthetic stationary sensors at 1 kHz.
    struct Rig {
        est: Estimator,
        k: u64,
        accel: Vec3<f32>,
        gyro: Vec3<f32>,
        gps_pos: Vec3<f32>,
        gps_vel: Vec3<f32>,
        gps_latency_us: u64,
        /// Body-frame magnetic field the rig reports.
        mag_field: Vec3<f32>,
        /// No magnetometer samples before this time, µs.
        mag_from_us: u64,
        /// False stops the IMU samples (updates continue).
        imu_on: bool,
        /// Pending GPS fixes as (arrival, sample).
        pending: [Option<(Micros, Stamped<GpsSample>)>; 4],
    }

    impl Rig {
        fn new(params: &EkfParams) -> Self {
            Self {
                est: Estimator::new(params, &F405_LEAN).unwrap(),
                k: 0,
                accel: Vec3::new(0.0, 0.0, -GRAVITY),
                gyro: Vec3::zero(),
                gps_pos: Vec3::zero(),
                gps_vel: Vec3::zero(),
                gps_latency_us: 150_000,
                mag_field: MAG,
                mag_from_us: 0,
                imu_on: true,
                pending: [None; 4],
            }
        }

        fn now(&self) -> Micros {
            Micros(self.k * 1000)
        }

        /// One base tick.
        fn tick(&mut self) {
            let t = self.now();
            let imu = ImuSample {
                gyro: self.gyro,
                accel: self.accel,
            };
            if self.imu_on {
                self.est.push_imu(&Stamped::ideal(t, imu));
            }
            if t.0.is_multiple_of(20_000) {
                let alt = BaroSample { altitude_m: 123.0 };
                self.est.push_baro(&Stamped::ideal(t, alt));
                if t.0 >= self.mag_from_us {
                    let m = MagSample {
                        field_ut: self.mag_field,
                    };
                    self.est.push_mag(&Stamped::ideal(t, m));
                }
            }
            if t.0.is_multiple_of(200_000) {
                let g = GpsSample {
                    position: self.gps_pos,
                    velocity: self.gps_vel,
                    h_acc_m: 0.6,
                    v_acc_m: 1.0,
                    s_acc_mps: 0.2,
                    fix_3d: true,
                };
                let s = Stamped {
                    t_sample: t,
                    t_arrival: t.add_us(self.gps_latency_us),
                    value: g,
                };
                if let Some(slot) = self.pending.iter_mut().find(|p| p.is_none()) {
                    *slot = Some((s.t_arrival, s));
                }
            }
            for slot in &mut self.pending {
                if let Some((arrive, s)) = *slot
                    && arrive <= t
                {
                    self.est.push_gps(&s);
                    *slot = None;
                }
            }
            self.est.update(t);
            self.k += 1;
        }

        fn run_until(&mut self, secs: f64) {
            while self.now().as_secs_f64() < secs {
                self.tick();
            }
        }
    }

    #[test]
    fn rejects_bad_parameters() {
        let ok = EkfParams::default();
        assert!(Estimator::new(&ok, &F405_LEAN).is_ok());
        assert!(Estimator::new(&ok, &H743_FULL).is_ok());
        let bad = |f: fn(&mut EkfParams)| {
            let mut p = ok;
            f(&mut p);
            Estimator::new(&p, &F405_LEAN).err()
        };
        let e = Some(ConfigError::BadEstimator);
        assert_eq!(bad(|p| p.gyro_noise = f32::NAN), e);
        assert_eq!(bad(|p| p.accel_noise = 0.0), e);
        assert_eq!(bad(|p| p.gps_gate = -1.0), e);
        assert_eq!(bad(|p| p.baro_gate = f32::INFINITY), e);
        assert_eq!(bad(|p| p.output_tau_s = 0.0), e);
        assert_eq!(bad(|p| p.delay_horizon_us = 0), e);
        assert_eq!(bad(|p| p.align_time_us = 0), e);
        assert_eq!(bad(|p| p.gps_timeout_us = 0), e);
        assert_eq!(bad(|p| p.mag_declination_rad = f32::NAN), e);
    }

    #[test]
    fn horizon_must_fit_the_imu_buffer() {
        // 250 Hz filter: 4 ms per delta. 62 deltas + 2 = 64 fits; 63 does not.
        let mut p = EkfParams {
            delay_horizon_us: 62 * 4000,
            ..EkfParams::default()
        };
        assert!(Estimator::new(&p, &F405_LEAN).is_ok());
        p.delay_horizon_us = 62 * 4000 + 1;
        assert_eq!(
            Estimator::new(&p, &F405_LEAN).err(),
            Some(ConfigError::BadEstimator)
        );
        p.delay_horizon_us = 63 * 4000;
        assert_eq!(
            Estimator::new(&p, &F405_LEAN).err(),
            Some(ConfigError::BadEstimator)
        );
    }

    #[test]
    fn malformed_profile_is_rejected() {
        let mut prof = F405_LEAN;
        prof.filter_period_ticks = 0;
        assert_eq!(
            Estimator::new(&EkfParams::default(), &prof).err(),
            Some(ConfigError::BadProfile)
        );
    }

    #[test]
    fn aligns_after_stillness_and_estimates_after_first_gps() {
        let mut rig = Rig::new(&EkfParams::default());
        rig.run_until(0.9);
        assert!(!rig.est.status().aligned);
        assert!(rig.est.estimate().is_none());
        assert!(rig.est.ekf().is_none());
        rig.run_until(1.2);
        assert!(rig.est.status().aligned);
        rig.run_until(3.0);
        let e = rig.est.estimate().expect("estimate after alignment + GPS");
        assert!(e.position.norm() < 0.05, "p = {:?}", e.position);
        assert!(e.velocity.norm() < 0.05, "v = {:?}", e.velocity);
        let att = e.attitude.to_rotation_vector().norm();
        assert!(att < 1.0_f32.to_radians(), "attitude error {att}");
        assert!(rig.est.status().gps_aiding);
        assert_eq!(rig.est.gyro_bias(), rig.est.ekf().unwrap().x.bg);
    }

    #[test]
    fn alignment_restarts_when_moved() {
        // Undisturbed, alignment happens at about 1.0 s.
        let mut rig = Rig::new(&EkfParams::default());
        rig.run_until(0.6);
        // Shake for 20 ms: strong specific force.
        rig.accel = Vec3::new(8.0, 0.0, -15.0);
        rig.run_until(0.62);
        rig.accel = Vec3::new(0.0, 0.0, -GRAVITY);
        rig.run_until(1.4);
        assert!(!rig.est.status().aligned, "window must have restarted");
        rig.run_until(1.8);
        assert!(rig.est.status().aligned);
    }

    #[test]
    fn rotating_vehicle_does_not_align() {
        let mut est = Estimator::new(&EkfParams::default(), &F405_LEAN).unwrap();
        for k in 0..3000u64 {
            let t = Micros(k * 1000);
            let imu = ImuSample {
                gyro: Vec3::new(0.0, 0.0, 0.2),
                accel: Vec3::new(0.0, 0.0, -GRAVITY),
            };
            est.push_imu(&Stamped::ideal(t, imu));
            est.update(t);
        }
        assert!(!est.status().aligned);
    }

    #[test]
    fn late_samples_are_dropped_and_counted() {
        let mut rig = Rig::new(&EkfParams::default());
        rig.run_until(2.0);
        assert_eq!(rig.est.status().rejected, 0);
        let now = rig.now();
        let old = now.0 - 300_000;
        let t_old = Micros(old);
        rig.est.push_gps(&Stamped {
            t_sample: t_old,
            t_arrival: now,
            value: GpsSample {
                position: Vec3::new(100.0, 0.0, 0.0),
                fix_3d: true,
                ..GpsSample::default()
            },
        });
        rig.est.push_baro(&Stamped {
            t_sample: t_old,
            t_arrival: now,
            value: BaroSample { altitude_m: 500.0 },
        });
        rig.est.push_mag(&Stamped {
            t_sample: t_old,
            t_arrival: now,
            value: MagSample { field_ut: MAG },
        });
        assert_eq!(rig.est.status().rejected, 3);
        rig.run_until(3.0);
        let e = rig.est.estimate().unwrap();
        assert!(e.position.norm() < 0.1, "dropped samples must not be used");
        // A sample inside the horizon is accepted.
        let before = rig.est.status().rejected;
        let recent = rig.now().add_us(0);
        rig.est.push_baro(&Stamped::ideal(
            Micros(recent.0 - 100_000),
            BaroSample { altitude_m: 123.0 },
        ));
        assert_eq!(rig.est.status().rejected, before);
    }

    #[test]
    fn first_gps_fix_resets_position() {
        let mut rig = Rig::new(&EkfParams::default());
        rig.gps_pos = Vec3::new(10.0, -5.0, 0.0);
        rig.run_until(3.0);
        let st = rig.est.status();
        assert_eq!(st.resets, 1);
        assert_eq!(st.rejected, 0);
        let e = rig.est.estimate().unwrap();
        assert!((e.position.x - 10.0).abs() < 0.1, "p = {:?}", e.position);
        assert!((e.position.y + 5.0).abs() < 0.1, "p = {:?}", e.position);
    }

    #[test]
    fn gps_jump_is_rejected_then_reset_after_timeout() {
        let params = EkfParams {
            gps_timeout_us: 2_000_000,
            ..EkfParams::default()
        };
        let mut rig = Rig::new(&params);
        rig.run_until(3.0);
        assert_eq!(rig.est.status().resets, 1);
        rig.gps_pos = Vec3::new(50.0, 0.0, 0.0);
        rig.run_until(4.0);
        let st = rig.est.status();
        assert!(st.rejected > 0, "jump must be gated");
        assert_eq!(st.resets, 1);
        let e = rig.est.estimate().unwrap();
        assert!(e.position.x < 1.0, "p = {:?}", e.position);
        rig.run_until(8.0);
        let st = rig.est.status();
        assert_eq!(st.resets, 2, "position must reset after the timeout");
        let e = rig.est.estimate().unwrap();
        assert!((e.position.x - 50.0).abs() < 0.5, "p = {:?}", e.position);
        assert!(st.gps_aiding);
    }

    #[test]
    fn output_tracks_ekf_closely_at_rest() {
        let mut rig = Rig::new(&EkfParams::default());
        rig.run_until(5.0);
        let ekf = rig.est.ekf().unwrap();
        let e = rig.est.estimate().unwrap();
        assert!((e.position - ekf.x.p).norm() < 0.05);
        let dq = (e.attitude * ekf.x.q.conj()).to_rotation_vector().norm();
        assert!(dq < 0.01, "attitude difference {dq}");
    }

    #[test]
    fn h743_profile_runs() {
        let mut est = Estimator::new(&EkfParams::default(), &H743_FULL).unwrap();
        for k in 0..16_000u64 {
            let t = Micros(k * 250);
            let imu = ImuSample {
                gyro: Vec3::zero(),
                accel: Vec3::new(0.0, 0.0, -GRAVITY),
            };
            est.push_imu(&Stamped::ideal(t, imu));
            if k % 80 == 0 {
                est.push_mag(&Stamped::ideal(t, MagSample { field_ut: MAG }));
            }
            est.update(t);
        }
        assert!(est.status().aligned);
    }

    #[test]
    fn alignment_learns_gyro_bias_from_still_window() {
        let bias = Vec3::new(0.01_f32, -0.008, 0.012);
        let mut rig = Rig::new(&EkfParams::default());
        rig.gyro = bias;
        while !rig.est.status().aligned {
            rig.tick();
            assert!(rig.now().0 < 3_000_000, "never aligned");
        }
        let err = (rig.est.gyro_bias() - bias).norm();
        assert!(err < 1e-3, "bias error right after alignment {err}");
        // Attitude stays at truth (identity) for the next 10 s.
        let end = rig.now().0 + 10_000_000;
        let mut worst = 0.0_f32;
        while rig.now().0 < end {
            rig.tick();
            if let Some(e) = rig.est.estimate() {
                worst = worst.max(e.attitude.to_rotation_vector().norm());
            }
        }
        assert!(worst < 0.5_f32.to_radians(), "attitude error {worst} rad");
        assert!(rig.est.estimate().is_some());
    }

    #[test]
    fn imu_ring_fits_from_the_first_filter_step() {
        let params = EkfParams::default();
        let mut rig = Rig::new(&params);
        let period_us = 4000u64;
        let cap = u64::from(params.delay_horizon_us).div_ceil(period_us) as usize + 1;
        let mut prev: Option<Micros> = None;
        let mut steps = 0;
        while rig.now().0 < 4_000_000 {
            rig.tick();
            if !rig.est.status().aligned {
                continue;
            }
            // Only filter-step ticks change the ring.
            if !rig.est.ticks.is_multiple_of(rig.est.period_ticks) {
                continue;
            }
            assert!(
                rig.est.imu_ring.len() <= cap,
                "len {}",
                rig.est.imu_ring.len()
            );
            let t = rig.est.t_ekf;
            if let Some(p) = prev {
                assert_eq!(t.since(p), period_us, "EKF time step at {:?}", rig.now());
                steps += 1;
            }
            prev = Some(t);
        }
        assert!(steps > 100);
    }

    /// Truth for the motion test, t seconds after the start of motion:
    /// yaw rate 0.5 rad/s for 6 s, then 2 m/s^2 north for 2 s, then coast.
    fn motion_truth(t: f64) -> (Vec3<f64>, Vec3<f64>, Vec3<f64>, f64) {
        // (position, velocity, world acceleration, yaw)
        let yaw = 0.5 * t.min(6.0);
        let ta = (t - 6.0).clamp(0.0, 2.0);
        let acc = if (6.0..8.0).contains(&t) { 2.0 } else { 0.0 };
        let v = 2.0 * ta;
        let p = 0.5 * 2.0 * ta * ta + 4.0 * (t - 8.0).max(0.0);
        (
            Vec3::new(p, 0.0, 0.0),
            Vec3::new(v, 0.0, 0.0),
            Vec3::new(acc, 0.0, 0.0),
            yaw,
        )
    }

    #[test]
    fn tracks_a_moving_vehicle_with_consistent_sensors() {
        use crate::math::Quat;
        let b_ned = Vec3::new(25.0_f64, 0.0, 43.3);
        let g = f64::from(GRAVITY);
        let mut est = Estimator::new(&EkfParams::default(), &F405_LEAN).unwrap();
        let t0_us = 3_000_000u64;
        let mut pending: [Option<(u64, Stamped<GpsSample>)>; 4] = [None; 4];
        let (mut worst_p, mut worst_v, mut worst_a) = (0.0_f64, 0.0_f64, 0.0_f64);
        for k in 0..16_000u64 {
            let t_us = k * 1000;
            let t = Micros(t_us);
            let ts = t_us.saturating_sub(t0_us) as f64 * 1e-6;
            let (p, v, a, yaw) = motion_truth(ts);
            let q = Quat::<f64>::from_euler(0.0, 0.0, yaw);
            let rate = if (0.0..6.0).contains(&ts) && t_us >= t0_us {
                0.5
            } else {
                0.0
            };
            let f_body = q.inverse_rotate(a - Vec3::new(0.0, 0.0, g));
            let imu = ImuSample {
                gyro: Vec3::new(0.0, 0.0, rate),
                accel: f_body.cast(),
            };
            est.push_imu(&Stamped::ideal(t, imu));
            if t_us % 20_000 == 0 {
                est.push_baro(&Stamped::ideal(t, BaroSample { altitude_m: 123.0 }));
                let m = q.inverse_rotate(b_ned);
                est.push_mag(&Stamped::ideal(t, MagSample { field_ut: m.cast() }));
            }
            if t_us % 200_000 == 0 {
                let s = Stamped {
                    t_sample: t,
                    t_arrival: t.add_us(150_000),
                    value: GpsSample {
                        position: p.cast(),
                        velocity: v.cast(),
                        h_acc_m: 0.6,
                        v_acc_m: 1.0,
                        s_acc_mps: 0.2,
                        fix_3d: true,
                    },
                };
                if let Some(slot) = pending.iter_mut().find(|x| x.is_none()) {
                    *slot = Some((s.t_arrival.0, s));
                }
            }
            for slot in &mut pending {
                if let Some((arrive, s)) = *slot
                    && arrive <= t_us
                {
                    est.push_gps(&s);
                    *slot = None;
                }
            }
            est.update(t);
            if t_us >= t0_us {
                let e = est.estimate().expect("estimate during motion");
                let ep = (e.position.cast::<f64>() - p).norm();
                let ev = (e.velocity.cast::<f64>() - v).norm();
                let dq = (e.attitude.cast::<f64>() * q.conj())
                    .to_rotation_vector()
                    .norm();
                worst_p = worst_p.max(ep);
                worst_v = worst_v.max(ev);
                worst_a = worst_a.max(dq);
            }
        }
        eprintln!(
            "motion worst: p {worst_p} m, v {worst_v} m/s, att {} deg",
            worst_a.to_degrees()
        );
        assert!(worst_p < 0.15, "position error {worst_p} m");
        assert!(worst_v < 0.1, "velocity error {worst_v} m/s");
        assert!(
            worst_a < 1.0_f64.to_radians(),
            "attitude error {} deg",
            worst_a.to_degrees()
        );
    }

    #[test]
    fn heading_is_initialised_from_the_first_magnetometer_sample() {
        use crate::math::Quat;
        let yaw = 120.0_f32.to_radians();
        let mut rig = Rig::new(&EkfParams::default());
        rig.mag_field = Quat::from_euler(0.0, 0.0, yaw).inverse_rotate(MAG);
        // Alignment would complete at about 1 s; the magnetometer starts at 3 s.
        rig.mag_from_us = 3_000_000;
        while rig.now().0 < 3_000_000 {
            rig.tick();
            assert!(rig.est.estimate().is_none(), "no estimate without heading");
            assert!(!rig.est.status().aligned, "not aligned without heading");
        }
        rig.run_until(4.0);
        let e = rig.est.estimate().expect("estimate within 1 s of the mag");
        let err = wrap_pi(e.attitude.yaw() - yaw).abs();
        assert!(err < 2.0_f32.to_radians(), "yaw error {err} rad");
        assert!(rig.est.status().aligned);
        let rejected = rig.est.status().rejected;
        rig.run_until(9.0);
        assert_eq!(rig.est.status().rejected, rejected, "mag must not be gated");
        let e = rig.est.estimate().unwrap();
        assert!(wrap_pi(e.attitude.yaw() - yaw).abs() < 2.0_f32.to_radians());
    }

    #[test]
    fn imu_stall_invalidates_the_estimate_and_realigns() {
        let mut rig = Rig::new(&EkfParams::default());
        rig.run_until(3.0);
        assert!(rig.est.estimate().is_some());
        let resets = rig.est.status().resets;
        rig.imu_on = false;
        rig.run_until(3.15);
        assert!(rig.est.estimate().is_none());
        assert!(!rig.est.status().aligned);
        assert!(!rig.est.status().gps_aiding);
        assert_eq!(rig.est.status().resets, resets + 1);
        // IMU resumes with the rig still: aligns again, then GPS gives an estimate.
        rig.imu_on = true;
        rig.run_until(3.15 + 0.5);
        assert!(!rig.est.status().aligned, "needs the full align time");
        rig.run_until(3.15 + 1.3);
        assert!(rig.est.status().aligned);
        rig.run_until(3.15 + 3.0);
        assert!(rig.est.estimate().is_some(), "estimate after the next fix");
        assert!(rig.est.status().gps_aiding);
    }

    #[test]
    fn gps_fixes_from_before_the_still_window_are_not_fused() {
        let mut rig = Rig::new(&EkfParams::default());
        // Moved around for 1 s: alignment keeps restarting.
        rig.accel = Vec3::new(8.0, 0.0, -15.0);
        rig.gps_pos = Vec3::new(10.0, 0.0, 0.0);
        rig.gps_vel = Vec3::new(3.0, 0.0, 0.0);
        rig.run_until(1.0);
        // Set down.
        rig.accel = Vec3::new(0.0, 0.0, -GRAVITY);
        rig.gps_pos = Vec3::new(30.0, 0.0, 0.0);
        rig.gps_vel = Vec3::zero();
        let mut first = None;
        while rig.now().as_secs_f64() < 9.0 {
            rig.tick();
            if let Some(e) = rig.est.estimate() {
                first.get_or_insert(rig.now());
                let dp = (e.position - Vec3::new(30.0, 0.0, 0.0)).norm();
                assert!(dp < 1.0, "p = {:?} at {:?}", e.position, rig.now());
                assert!(e.velocity.norm() < 0.3, "v = {:?}", e.velocity);
            }
        }
        assert!(first.is_some(), "estimate must appear");
    }
}
