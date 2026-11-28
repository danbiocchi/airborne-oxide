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
//!
//! The process-noise parameters (`EKF_GYR_NOISE`, `EKF_ACC_NOISE`,
//! `EKF_GYR_BNOISE`, `EKF_ACC_BNOISE`) are stated, as PX4 EKF2 states them,
//! for a 10 ms prediction step; the filter applies the same noise per second
//! at any filter rate (see [`ekf::Ekf::predict`]).

pub mod ekf;
pub mod heading_check;
pub mod imu;
pub mod output;
pub mod ring;

use crate::GRAVITY;
use crate::config::{ConfigError, EkfParams};
use crate::io::{
    BaroSample, EstimatorStatus, GpsSample, ImuSample, MagSample, Stamped, StateSample, Validity,
};
use crate::math::{Quat, Real, Vec3, wrap_pi};
use crate::schedule::Profile;
use crate::time::Micros;
use ekf::{Ekf, Fusion, GYRO_BIAS_LIMIT, heading_from_mag, tilt_from_accel};
use heading_check::{HeadingCheck, HeadingConfig, HeadingFit, Verdict};
use imu::{ImuAccumulator, ImuDelta};
use output::OutputPredictor;
use ring::Ring;

/// The least sigma given to the yaw reset of a heading correction, rad: not
/// smaller than this, so that the magnetometer and the GPS can still refine
/// the corrected heading (D2 section 12).
const HDG_RESET_SIGMA_MIN_RAD: f32 = 0.1;

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
    /// Same, counting only the velocity part of the fix.
    vel_reject_since: Option<Micros>,
    /// `t_sample` of the last fix whose velocity was accepted (or reset to).
    vel_accept: Option<Micros>,
    /// `t_sample` of the last fix whose position was accepted (or reset to).
    pos_accept: Option<Micros>,
}

/// The estimator: alignment, the delayed-horizon EKF and the output
/// predictor.
#[derive(Clone, Debug)]
pub struct Estimator {
    params: EkfParams,
    horizon_us: u64,
    gps_timeout_us: u64,
    pos_timeout_us: u64,
    baro_timeout_us: u64,
    mag_timeout_us: u64,
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
    /// `t_sample` of the last accepted barometer fusion.
    baro_accept: Option<Micros>,
    /// `t_sample` of the last accepted magnetometer fusion (or heading reset).
    mag_accept: Option<Micros>,
    /// Covariance resets of filters that were since dropped.
    dropped_faults: u32,
    /// `t_arrival` of the last IMU sample accepted.
    last_imu_arrival: Option<Micros>,
    raw_gyro: Vec3<f32>,
    rejected: u32,
    resets: u32,
    /// The check of the heading against the vehicle's own motion. In this
    /// version it only watches: its verdict is counted and logged, and
    /// nothing reads it to act.
    hdg: HeadingCheck,
    /// Corrections made in this flight (cleared at disarm).
    hdg_corrections: u8,
    /// Corrections allowed per flight (`EKF_HDG_NMAX`).
    hdg_max: u8,
    /// The containment latch: the check said correct with the budget used
    /// up. Read by [`Estimator::validity`] and by `fuse_gps`; it never
    /// clears `heading_ok` (the next magnetometer sample would then reset
    /// the yaw to the wrong magnetometer heading and call it good). Cleared
    /// at disarm.
    hdg_latched: bool,
    /// Sum of the heading corrections of this flight, rad (wrapped): the run-time
    /// shift of the declination. Held here, not only in the filter, because
    /// a new filter (`drop_to_unaligned`) reads the declination from the
    /// parameters again. `EKF_MAG_DECL` itself is never written.
    decl_shift: f32,
    /// The supervisor's `armed` (a fall clears the per-flight state).
    armed: bool,
    /// The last correction not yet reported (rad), for the core's event.
    pending_corrected: Option<f32>,
    /// The containment latch was set and not yet reported.
    pending_contained: bool,
}

impl Estimator {
    /// Fails with [`ConfigError::BadEstimator`] if a parameter is outside its
    /// range in the parameter table (non-finite, or non-positive where it
    /// must be positive), or if the delay horizon does not fit the IMU
    /// buffer (see [`EkfParams::validate`]). A malformed profile fails with
    /// [`ConfigError::BadProfile`].
    pub fn new(params: &EkfParams, profile: &Profile) -> Result<Self, ConfigError> {
        profile.validate()?;
        params.validate(profile)?;
        let p = params;
        Ok(Self {
            params: *p,
            horizon_us: u64::from(p.delay_horizon_us),
            gps_timeout_us: u64::from(p.gps_timeout_us),
            pos_timeout_us: u64::from(p.pos_timeout_us),
            baro_timeout_us: u64::from(p.baro_timeout_us),
            mag_timeout_us: u64::from(p.mag_timeout_us),
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
            baro_accept: None,
            mag_accept: None,
            dropped_faults: 0,
            last_imu_arrival: None,
            raw_gyro: Vec3::zero(),
            rejected: 0,
            resets: 0,
            hdg: HeadingCheck::new(heading_config(p)),
            hdg_corrections: 0,
            hdg_max: u8::try_from(p.hdg_max_corrections).unwrap_or(u8::MAX),
            hdg_latched: false,
            decl_shift: 0.0,
            armed: false,
            pending_corrected: None,
            pending_contained: false,
        })
    }

    /// The supervisor's `armed && !landed`: the heading check judges and
    /// accumulates only while airborne.
    pub fn set_airborne(&mut self, airborne: bool) {
        self.hdg.set_airborne(airborne);
    }

    /// The supervisor's `armed`. Falling to disarmed ends the flight: the
    /// correction count, the containment latch and the declination shift
    /// are cleared. If any of them was set the filter's heading and the
    /// magnetometer now disagree by the shift, so the filter is dropped and
    /// aligns again from the magnetometer with the declination parameter
    /// (the estimator's own re-alignment, as after an IMU stall): the
    /// vehicle cannot be armed until that is done (no estimate meanwhile).
    pub fn set_armed(&mut self, armed: bool) {
        let fell = self.armed && !armed;
        self.armed = armed;
        if !fell {
            return;
        }
        let dirty = self.hdg_corrections != 0 || self.hdg_latched || self.decl_shift != 0.0;
        self.hdg_corrections = 0;
        self.hdg_latched = false;
        self.decl_shift = 0.0;
        self.pending_corrected = None;
        self.pending_contained = false;
        if dirty && self.ekf.is_some() {
            self.drop_to_unaligned();
        }
    }

    /// Heading corrections made in this flight.
    pub fn heading_corrections(&self) -> u8 {
        self.hdg_corrections
    }

    /// The containment latch (the heading is declared untrustworthy until
    /// disarm).
    pub fn heading_contained(&self) -> bool {
        self.hdg_latched
    }

    /// The run-time shift of the declination, rad (sum of the corrections of
    /// this flight).
    pub fn declination_shift(&self) -> f32 {
        self.decl_shift
    }

    /// The last heading correction (rad) made since the previous call, if
    /// any: the core reports it as an event.
    pub fn take_heading_corrected(&mut self) -> Option<f32> {
        self.pending_corrected.take()
    }

    /// True once after the containment latch was set.
    pub fn take_heading_contained(&mut self) -> bool {
        core::mem::take(&mut self.pending_contained)
    }

    /// The heading check's record for the latest GPS fix it judged while
    /// airborne (all zero before the first).
    pub fn heading_fit(&self) -> HeadingFit {
        self.hdg.fit()
    }

    /// How many fixes since boot had the heading check's verdict
    /// `Correct` (nothing acts on it).
    pub fn heading_verdicts(&self) -> u32 {
        self.hdg.verdicts()
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

    /// Which parts of the estimate are valid, evaluated at the filter's
    /// time (ADR-0003 §1). All false until the filter exists.
    pub fn validity(&self) -> Validity {
        if self.ekf.is_none() {
            return Validity::NONE;
        }
        let recent =
            |t: Option<Micros>, timeout: u64| t.is_some_and(|t| self.t_ekf.since(t) <= timeout);
        let heading =
            self.heading_ok && !self.hdg_latched && recent(self.mag_accept, self.mag_timeout_us);
        let altitude = recent(self.baro_accept, self.baro_timeout_us);
        let velocity = heading && recent(self.gps.vel_accept, self.gps_timeout_us);
        let position = velocity && recent(self.gps.pos_accept, self.pos_timeout_us);
        Validity {
            attitude: true,
            heading,
            altitude,
            velocity,
            position,
        }
    }

    /// The current-time state whenever the filter is running, even if
    /// position or heading is not valid; `None` if it holds a non-finite
    /// value. Use [`Estimator::validity`] to know what to trust.
    pub fn state(&self) -> Option<StateSample> {
        self.ekf.as_ref()?;
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

    /// Numerical faults (covariance resets) reported by the filter so far,
    /// including those of filters since dropped.
    pub fn faults(&self) -> u32 {
        self.dropped_faults
            .saturating_add(self.ekf.as_ref().map_or(0, Ekf::faults))
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
            heading_corrections: self.hdg_corrections,
            heading_contained: self.hdg_latched,
            heading_shift_rad: self.decl_shift,
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
        if let Some(e) = &self.ekf {
            self.dropped_faults = self.dropped_faults.saturating_add(e.faults());
        }
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
        self.baro_accept = None;
        self.mag_accept = None;
        self.last_imu_arrival = None;
        self.hdg.restart();
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
        // The declination of the heading model: the parameter and this
        // flight's shift (zero unless the filter was dropped in flight).
        let declination = wrap_pi(self.params.mag_declination_rad + self.decl_shift);
        let heading = mag.and_then(|m| heading_from_mag(m, roll, pitch, declination));
        let q0 = Quat::from_euler(roll, pitch, heading.unwrap_or(0.0));
        let mut ekf = Ekf::new(&self.params, q0);
        // A new filter reads the declination from the parameters: apply the
        // shift of the corrections again.
        ekf.shift_declination(self.decl_shift);
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
        // The alignment readings count as accepted ones, so that the
        // sensor timeouts (and the recovery after them) run from here even
        // if no later reading is ever accepted.
        self.mag_accept = heading.map(|_| self.t_ekf);
        self.baro_accept = self.baro_ref.map(|_| self.t_ekf);
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
            self.hdg.on_step(ekf.last_dvw_ne());
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
            Some(_) if self.baro_timed_out() => {
                // The height estimate was dead-reckoned through the gap, so
                // keep it and re-reference the barometer to it:
                // altitude - baro_ref = -p.z.
                self.baro_ref = Some(alt + ekf.x.p.z);
                self.baro_accept = Some(s.t_sample);
                self.resets = self.resets.saturating_add(1);
            }
            Some(r) => {
                let f = ekf.fuse_height(alt - r, self.params.baro_noise_m);
                if matches!(self.note(f), Fusion::Accepted { .. }) {
                    self.baro_accept = Some(s.t_sample);
                }
            }
        }
    }

    fn fuse_mag(&mut self, ekf: &mut Ekf<f32>, s: &Stamped<MagSample>) {
        if !s.value.field_ut.is_finite() {
            return;
        }
        // Without a heading, or after the magnetometer was silent for longer
        // than its timeout (the yaw may have drifted outside any gate):
        // take the first usable reading as the heading instead of gating
        // it against a guess.
        if !self.heading_ok || self.mag_timed_out() {
            if let Some(psi) = ekf.mag_heading(s.value.field_ut) {
                let dpsi = ekf.reset_yaw(psi, self.params.init_yaw_sigma_rad);
                self.output.rotate_yaw(dpsi);
                // The heading the check judged is gone.
                self.hdg.restart();
                self.heading_ok = true;
                self.mag_accept = Some(s.t_sample);
                self.resets = self.resets.saturating_add(1);
            }
            return;
        }
        let f = ekf.fuse_mag_heading(s.value.field_ut, self.params.yaw_noise_rad);
        if matches!(self.note(f), Fusion::Accepted { .. }) {
            self.mag_accept = Some(s.t_sample);
        }
    }

    /// The heading was last confirmed longer ago than `mag_timeout_us`
    /// (at the filter's time), as [`Estimator::validity`] sees it.
    fn mag_timed_out(&self) -> bool {
        self.mag_accept
            .is_some_and(|t| self.t_ekf.since(t) > self.mag_timeout_us)
    }

    /// Same for the barometer and `baro_timeout_us`.
    fn baro_timed_out(&self) -> bool {
        self.baro_accept
            .is_some_and(|t| self.t_ekf.since(t) > self.baro_timeout_us)
    }

    fn fuse_gps(&mut self, ekf: &mut Ekf<f32>, s: &Stamped<GpsSample>) {
        let g = s.value;
        if !g.fix_3d || !g.position.is_finite() || !g.velocity.is_finite() {
            let _ = self.hdg.on_fix(s.t_sample, None);
            return;
        }
        // Without a heading, GPS velocity would be attributed to the wrong
        // body axes and corrupt the tilt. Wait for the magnetometer.
        if !self.heading_ok {
            let _ = self.hdg.on_fix(s.t_sample, None);
            return;
        }
        // The heading check judges the fix (with the velocity change the
        // filter integrated since the previous one) before the fusion
        // changes anything. Its verdict is counted and logged, not acted on.
        let verdict = self.hdg.on_fix(
            s.t_sample,
            Some(([g.velocity.x, g.velocity.y], g.s_acc_mps)),
        );
        let sigma_p = g.h_acc_m.finite_or(0.0).max(self.params.gps_pos_noise_m);
        let sigma_v = g
            .s_acc_mps
            .finite_or(0.0)
            .max(self.params.gps_vel_noise_mps);
        let t = s.t_sample;

        // Contained: the heading is declared untrustworthy until disarm, so
        // GPS fusion stops (the check goes on judging, for the log).
        if self.hdg_latched {
            return;
        }
        if let Verdict::Correct { eps, sigma } = verdict {
            if self.hdg_corrections < self.hdg_max {
                self.correct_heading(ekf, s, eps, sigma, sigma_p, sigma_v);
            } else {
                // The budget is used up (or is zero): the next
                // disagreement is containment, through the existing
                // `ESTIMATOR_UNHEALTHY` and `LandNow` (`validity()`).
                self.hdg_latched = true;
                self.pending_contained = true;
            }
            return;
        }

        let timed_out = self.gps.fused && t.since(self.gps.last_accept) > self.gps_timeout_us;
        if !self.gps.fused || timed_out {
            self.reset_to_gps(ekf, s, sigma_p, sigma_v);
            return;
        }
        let pos = self.note(ekf.fuse_gps_position_ne([g.position.x, g.position.y], sigma_p));
        let vel = self.note(ekf.fuse_gps_velocity(g.velocity, sigma_v));
        self.hdg.set_vel_ratio(match vel {
            Fusion::Accepted { test_ratio } | Fusion::Rejected { test_ratio } => Some(test_ratio),
            Fusion::Skipped => None,
        });
        let accepted = |f: Fusion<f32>| matches!(f, Fusion::Accepted { .. });
        let rejected = |f: Fusion<f32>| matches!(f, Fusion::Rejected { .. });
        // Position and velocity validity are tracked apart: a fix whose
        // position was rejected must not keep the position valid.
        if accepted(pos) {
            self.gps.pos_accept = Some(t);
        }
        if accepted(vel) {
            self.gps.vel_accept = Some(t);
        }
        // The velocity alone rejected while the position is accepted: a
        // third timer, since neither of the others can see it.
        let mut vel_stuck = false;
        if rejected(vel) {
            let since = *self.gps.vel_reject_since.get_or_insert(t);
            vel_stuck = accepted(pos) && t.since(since) > self.gps_timeout_us;
        } else if accepted(vel) {
            self.gps.vel_reject_since = None;
        }
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
        } else if vel_stuck {
            let dv = ekf.reset_velocity(g.velocity, sigma_v);
            self.output.shift(Vec3::zero(), dv);
            self.resets = self.resets.saturating_add(1);
            self.gps.vel_accept = Some(t);
            self.gps.last_accept = t;
            self.gps.vel_reject_since = None;
        }
    }

    /// The heading check says the filter's heading is wrong by `eps` (filter
    /// minus truth, one-sigma `sigma`) and the budget allows a correction
    /// (D2 2.3), at the delayed horizon, in this order:
    ///
    /// 1. the yaw is reset by the fitted angle with sigma `max(sigma, 0.1)`
    ///    and the output predictor is turned by the same angle, as the
    ///    magnetometer's own heading reset does;
    /// 2. position (N/E) and velocity snap to this fix (`reset_to_gps`, which
    ///    also counts the reset that guidance re-bases its held point and yaw
    ///    set-point on) so that GPS fusion resumes at once;
    /// 3. the tilt variances go back to their initial value;
    /// 4. the declination of the heading model is shifted by the correction
    ///    (and the shift kept here), so that the magnetometer goes on holding
    ///    the corrected heading;
    /// 5. the fit restarts, the corrections are counted and the event is
    ///    queued.
    fn correct_heading(
        &mut self,
        ekf: &mut Ekf<f32>,
        s: &Stamped<GpsSample>,
        eps: f32,
        sigma: f32,
        sigma_p: f32,
        sigma_v: f32,
    ) {
        let yaw = ekf.x.q.yaw();
        let dpsi = ekf.reset_yaw(yaw - eps, sigma.max(HDG_RESET_SIGMA_MIN_RAD));
        self.output.rotate_yaw(dpsi);
        self.reset_to_gps(ekf, s, sigma_p, sigma_v);
        ekf.reset_tilt_variance();
        ekf.shift_declination(dpsi);
        self.decl_shift = wrap_pi(self.decl_shift + dpsi);
        self.hdg.restart();
        self.hdg_corrections = self.hdg_corrections.saturating_add(1);
        self.pending_corrected = Some(dpsi);
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
        self.gps.vel_accept = Some(s.t_sample);
        self.gps.pos_accept = Some(s.t_sample);
        self.gps.last_accept = s.t_sample;
        self.gps.reject_since = None;
        self.gps.pos_reject_since = None;
        self.gps.vel_reject_since = None;
    }
}

/// The heading check's settings from the filter's parameters.
fn heading_config(p: &EkfParams) -> HeadingConfig {
    HeadingConfig {
        threshold_rad: p.hdg_threshold_rad,
        k_sigma: p.hdg_k_sigma,
        persist_us: u64::from(p.hdg_persist_us),
        tau_s: p.hdg_tau_s,
        vel_noise_mps: p.gps_vel_noise_mps,
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
    use crate::math::{Quat, wrap_pi};
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
        /// Altitude the barometer reports, m.
        baro_alt: f32,
        /// No magnetometer samples before this time, µs.
        mag_from_us: u64,
        /// False stops the IMU samples (updates continue).
        imu_on: bool,
        /// False stops the barometer, magnetometer and GPS samples.
        baro_on: bool,
        mag_on: bool,
        gps_on: bool,
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
                baro_alt: 123.0,
                mag_from_us: 0,
                imu_on: true,
                baro_on: true,
                mag_on: true,
                gps_on: true,
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
                let alt = BaroSample {
                    altitude_m: self.baro_alt,
                };
                if self.baro_on {
                    self.est.push_baro(&Stamped::ideal(t, alt));
                }
                if self.mag_on && t.0 >= self.mag_from_us {
                    let m = MagSample {
                        field_ut: self.mag_field,
                    };
                    self.est.push_mag(&Stamped::ideal(t, m));
                }
            }
            if self.gps_on && t.0.is_multiple_of(200_000) {
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
    fn validity_all_true_in_steady_state() {
        let mut rig = Rig::new(&EkfParams::default());
        assert_eq!(rig.est.validity(), Validity::NONE);
        rig.run_until(4.0);
        assert_eq!(rig.est.validity(), Validity::ALL);
        assert!(rig.est.state().is_some());
        assert_eq!(rig.est.faults(), 0);
    }

    #[test]
    fn validity_all_false_before_alignment() {
        let mut rig = Rig::new(&EkfParams::default());
        rig.run_until(0.9);
        assert_eq!(rig.est.validity(), Validity::NONE);
        assert!(rig.est.state().is_none());
    }

    #[test]
    fn state_exists_without_gps() {
        let mut rig = Rig::new(&EkfParams::default());
        rig.gps_on = false;
        rig.run_until(3.0);
        assert!(rig.est.estimate().is_none());
        assert!(rig.est.state().is_some());
        let v = rig.est.validity();
        assert!(v.attitude && v.heading && v.altitude);
        assert!(!v.velocity && !v.position);
    }

    #[test]
    fn validity_gps_loss_drops_position_then_velocity() {
        let mut rig = Rig::new(&EkfParams::default());
        rig.run_until(4.0);
        rig.gps_on = false;
        // Fixes already in flight (150 ms latency) still arrive; the last
        // accepted sample time is about 4.0 s.
        rig.run_until(4.0 + 1.0);
        let v = rig.est.validity();
        assert!(v.position && v.velocity, "{v:?}");
        rig.run_until(4.0 + 1.9);
        let v = rig.est.validity();
        assert!(!v.position, "{v:?}");
        assert!(v.velocity && v.heading && v.altitude && v.attitude, "{v:?}");
        rig.run_until(4.0 + 4.5);
        assert!(rig.est.validity().velocity);
        rig.run_until(4.0 + 5.6);
        let v = rig.est.validity();
        assert!(!v.velocity && !v.position, "{v:?}");
        assert!(v.attitude && v.heading && v.altitude, "{v:?}");
        assert!(rig.est.state().is_some());
        // GPS returns: everything returns.
        rig.gps_on = true;
        rig.run_until(4.0 + 7.0);
        assert_eq!(rig.est.validity(), Validity::ALL);
    }

    #[test]
    fn validity_baro_loss_drops_altitude() {
        let mut rig = Rig::new(&EkfParams::default());
        rig.run_until(4.0);
        rig.baro_on = false;
        rig.run_until(4.0 + 4.5);
        assert!(rig.est.validity().altitude);
        rig.run_until(4.0 + 5.5);
        let v = rig.est.validity();
        assert!(!v.altitude, "{v:?}");
        assert!(v.attitude && v.heading && v.velocity && v.position, "{v:?}");
        assert!(rig.est.state().is_some());
        rig.baro_on = true;
        rig.run_until(4.0 + 6.5);
        assert_eq!(rig.est.validity(), Validity::ALL);
    }

    #[test]
    fn validity_mag_loss_drops_heading_velocity_position() {
        let mut rig = Rig::new(&EkfParams::default());
        rig.run_until(4.0);
        rig.mag_on = false;
        rig.run_until(4.0 + 29.0);
        assert_eq!(rig.est.validity(), Validity::ALL);
        rig.run_until(4.0 + 31.0);
        let v = rig.est.validity();
        assert!(v.attitude && v.altitude, "{v:?}");
        assert!(!v.heading && !v.velocity && !v.position, "{v:?}");
        assert!(rig.est.state().is_some());
        rig.mag_on = true;
        rig.run_until(4.0 + 33.0);
        assert_eq!(rig.est.validity(), Validity::ALL);
    }

    #[test]
    fn validity_clears_when_the_filter_is_dropped() {
        let mut rig = Rig::new(&EkfParams::default());
        rig.run_until(4.0);
        assert_eq!(rig.est.validity(), Validity::ALL);
        rig.imu_on = false;
        rig.run_until(4.3);
        assert_eq!(rig.est.validity(), Validity::NONE);
        assert!(rig.est.state().is_none());
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
        assert_eq!(bad(|p| p.pos_timeout_us = 0), e);
        assert_eq!(bad(|p| p.baro_timeout_us = 0), e);
        assert_eq!(bad(|p| p.mag_timeout_us = 0), e);
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

    #[test]
    fn heading_resets_when_the_magnetometer_returns_after_its_timeout() {
        use crate::math::Quat;
        let mut rig = Rig::new(&EkfParams::default());
        rig.run_until(4.0);
        assert_eq!(rig.est.validity(), Validity::ALL);
        let resets = rig.est.status().resets;
        rig.mag_on = false;
        rig.run_until(4.0 + 35.0);
        assert!(!rig.est.validity().heading);
        // The vehicle was turned 90 degrees meanwhile: the old gate would
        // reject the magnetometer for ever.
        let yaw = 90.0_f32.to_radians();
        rig.mag_field = Quat::from_euler(0.0, 0.0, yaw).inverse_rotate(MAG);
        rig.mag_on = true;
        rig.run_until(4.0 + 36.0);
        assert_eq!(rig.est.validity(), Validity::ALL);
        let e = rig.est.estimate().unwrap();
        let err = wrap_pi(e.attitude.yaw() - yaw).abs();
        assert!(err < 2.0_f32.to_radians(), "yaw error {err} rad");
        assert_eq!(rig.est.status().resets, resets + 1);
        rig.run_until(4.0 + 40.0);
        assert_eq!(rig.est.status().resets, resets + 1, "one reset only");
    }

    #[test]
    fn height_rereferences_when_the_barometer_returns_after_its_timeout() {
        let mut rig = Rig::new(&EkfParams::default());
        rig.run_until(4.0);
        let resets = rig.est.status().resets;
        rig.baro_on = false;
        rig.run_until(4.0 + 8.0);
        assert!(!rig.est.validity().altitude);
        let z0 = rig.est.state().unwrap().position.z;
        rig.baro_alt = 143.0;
        rig.baro_on = true;
        rig.run_until(4.0 + 9.0);
        assert!(rig.est.validity().altitude);
        let z1 = rig.est.state().unwrap().position.z;
        assert!((z1 - z0).abs() < 0.2, "height moved {} m", z1 - z0);
        assert_eq!(rig.est.status().resets, resets + 1);
        rig.run_until(4.0 + 15.0);
        assert_eq!(rig.est.validity(), Validity::ALL);
        assert!((rig.est.state().unwrap().position.z - z0).abs() < 0.2);
        assert_eq!(rig.est.status().resets, resets + 1, "one reset only");
    }

    #[test]
    fn velocity_resets_when_gps_velocity_alone_is_rejected_for_the_timeout() {
        let mut rig = Rig::new(&EkfParams::default());
        rig.run_until(4.0);
        let resets = rig.est.status().resets;
        assert_eq!(resets, 1);
        // Position stays consistent; velocity is 5 m/s off.
        rig.gps_vel = Vec3::new(5.0, 0.0, 0.0);
        rig.run_until(4.0 + 3.0);
        assert_eq!(rig.est.status().resets, resets, "not before the timeout");
        rig.run_until(4.0 + 6.0);
        assert_eq!(rig.est.status().resets, resets + 1);
        let v = rig.est.validity();
        assert!(v.velocity && v.position, "{v:?}");
        // Snapped to the fix (the stationary rig's position then pulls it
        // back, so look soon after the reset).
        let e = rig.est.estimate().unwrap();
        assert!(e.velocity.x > 3.5, "v = {:?}", e.velocity);
        rig.run_until(4.0 + 9.0);
        assert_eq!(rig.est.status().resets, resets + 1, "one reset only");
    }

    // ---- the heading check, wired into the filter -------------------------

    /// Moves the rig along a horizontal velocity path from `t0` (s): north
    /// `6 sin(1.1 t)`, east `5 sin(0.7 t)` m/s, the body frame being the
    /// world frame (true heading 0 against the rig's magnetic field), until
    /// `until` (s). The accelerometer and the GPS both follow the path.
    fn fly_path(rig: &mut Rig, t0: f64, until: f64) {
        let (w1, w2) = (1.1f32, 0.7f32);
        while rig.now().as_secs_f64() < until {
            let t = (rig.now().as_secs_f64() - t0).max(0.0) as f32;
            rig.accel = Vec3::new(
                6.0 * w1 * Real::cos(w1 * t),
                5.0 * w2 * Real::cos(w2 * t),
                -GRAVITY,
            );
            rig.gps_vel = Vec3::new(6.0 * Real::sin(w1 * t), 5.0 * Real::sin(w2 * t), 0.0);
            rig.gps_pos = Vec3::new(
                6.0 * (1.0 - Real::cos(w1 * t)) / w1,
                5.0 * (1.0 - Real::cos(w2 * t)) / w2,
                0.0,
            );
            rig.tick();
        }
    }

    #[test]
    fn the_heading_check_judges_nothing_on_the_ground() {
        let mut rig = Rig::new(&EkfParams::default());
        // Moving but not airborne (the supervisor has not said so).
        fly_path(&mut rig, 3.0, 12.0);
        assert_eq!(rig.est.heading_fit().seq, 0);
        assert_eq!(rig.est.heading_fit().sa, 0.0);
        assert_eq!(rig.est.heading_verdicts(), 0);
    }

    #[test]
    fn the_heading_check_agrees_with_a_right_heading_and_flags_a_wrong_one() {
        // Right heading (declination 0 and the rig's field north).
        let mut rig = Rig::new(&EkfParams::default());
        rig.run_until(3.0);
        rig.est.set_airborne(true);
        fly_path(&mut rig, 3.0, 14.0);
        let f = rig.est.heading_fit();
        assert!(f.seq > 30, "fixes judged: {}", f.seq);
        assert!(f.used && f.sa > 1.0, "{f:?}");
        assert!(f.eps.abs() < 0.2, "eps {}", f.eps);
        assert_eq!(rig.est.heading_verdicts(), 0);
        assert_eq!(rig.est.heading_corrections(), 0);
        assert_eq!(rig.est.validity(), Validity::ALL);
        // The fix's own record: its time is a GPS sample time, and the
        // filter's velocity test ratio is there (the fix was fused).
        assert_eq!(f.t_us % 200_000, 0);
        assert!((0.0..=1.0).contains(&f.vel_ratio), "ratio {}", f.vel_ratio);

        // The same flight with a declination of 1 rad the site does not
        // have: the filter's heading is wrong by +1 rad. The fit reads it
        // at the fix where the verdict is first correct.
        let mut rig = wrong_rig(1.0, 3.0);
        let mut at_verdict = None;
        while rig.now().as_secs_f64() < 14.0 && at_verdict.is_none() {
            let next = rig.now().as_secs_f64() + 0.001;
            fly_path(&mut rig, 3.0, next);
            if rig.est.heading_verdicts() > 0 {
                at_verdict = Some(rig.est.heading_fit());
            }
        }
        let f = at_verdict.expect("a verdict");
        assert_eq!(f.verdict, 1);
        assert!((f.eps - 1.0).abs() < 0.5, "eps {} (true +1.0)", f.eps);
    }

    /// A rig with the declination `decl` the site does not have, aligned
    /// and told it is airborne at `t0` (s).
    fn wrong_rig(decl: f32, t0: f64) -> Rig {
        wrong_rig_with(
            &EkfParams {
                mag_declination_rad: decl,
                ..EkfParams::default()
            },
            t0,
        )
    }

    fn wrong_rig_with(params: &EkfParams, t0: f64) -> Rig {
        let mut rig = Rig::new(params);
        rig.run_until(t0);
        rig.est.set_armed(true);
        rig.est.set_airborne(true);
        rig
    }

    /// `fly_path` with the vehicle's true heading `psi(t)` (rad, `t` in s
    /// from the start of the path) against the world: the accelerometer and
    /// the magnetometer read in the body frame, the GPS in the world frame.
    fn fly_path_heading(rig: &mut Rig, t0: f64, until: f64, psi: impl Fn(f32) -> f32) {
        let (w1, w2) = (1.1f32, 0.7f32);
        while rig.now().as_secs_f64() < until {
            let t = (rig.now().as_secs_f64() - t0).max(0.0) as f32;
            let (an, ae) = (6.0 * w1 * Real::cos(w1 * t), 5.0 * w2 * Real::cos(w2 * t));
            let (s, c) = (Real::sin(psi(t)), Real::cos(psi(t)));
            // Body = world rotated by -psi about Down.
            rig.accel = Vec3::new(c * an + s * ae, -s * an + c * ae, -GRAVITY);
            rig.mag_field = Vec3::new(c * MAG.x + s * MAG.y, -s * MAG.x + c * MAG.y, MAG.z);
            rig.gps_vel = Vec3::new(6.0 * Real::sin(w1 * t), 5.0 * Real::sin(w2 * t), 0.0);
            rig.gps_pos = Vec3::new(
                6.0 * (1.0 - Real::cos(w1 * t)) / w1,
                5.0 * (1.0 - Real::cos(w2 * t)) / w2,
                0.0,
            );
            rig.tick();
        }
    }

    /// Flies `wrong_rig(decl, 3.0)` along the path until the first
    /// correction (or `limit` s); returns the rig, the correction angle and
    /// the check's record at the correcting fix.
    fn until_first_correction(mut rig: Rig, limit: f64) -> (Rig, f32, HeadingFit) {
        let mut fit = rig.est.heading_fit();
        while rig.now().as_secs_f64() < limit {
            let next = rig.now().as_secs_f64() + 0.001;
            fly_path(&mut rig, 3.0, next);
            if rig.est.heading_verdicts() > 0 {
                fit = rig.est.heading_fit();
            }
            if let Some(dpsi) = rig.est.take_heading_corrected() {
                return (rig, dpsi, fit);
            }
        }
        panic!("no correction by {limit} s");
    }

    #[test]
    fn a_wrong_declination_is_corrected_once_and_the_heading_stays_valid() {
        let mut rig = wrong_rig(1.0, 3.0);
        let mut corrections = 0;
        let mut yaw_err_at_end = f32::NAN;
        while rig.now().as_secs_f64() < 16.0 {
            let before = rig.est.state().map(|s| s.attitude.yaw());
            let next = rig.now().as_secs_f64() + 0.001;
            fly_path(&mut rig, 3.0, next);
            // The heading is valid in every tick, the correction included.
            assert!(rig.est.validity().heading, "t = {:?}", rig.now());
            if let Some(dpsi) = rig.est.take_heading_corrected() {
                corrections += 1;
                // The published attitude was turned in the same step.
                let after = rig.est.state().unwrap().attitude.yaw();
                let turned = wrap_pi(after - before.unwrap());
                assert!((turned - dpsi).abs() < 0.02, "turned {turned}, dpsi {dpsi}");
                assert!(dpsi < -0.5, "the filter was +1 rad wrong, dpsi {dpsi}");
            }
            yaw_err_at_end = wrap_pi(rig.est.state().unwrap().attitude.yaw());
        }
        assert_eq!(corrections, 1, "one correction");
        assert_eq!(rig.est.heading_corrections(), 1);
        assert!(!rig.est.heading_contained());
        // The heading error after it is within the fit's one-sigma band
        // (the rig's body frame is the world frame: the true yaw is 0).
        let sigma = rig.est.heading_fit().sigma.max(0.1);
        assert!(
            yaw_err_at_end.abs() < 0.2 + sigma,
            "yaw error {yaw_err_at_end}"
        );
        assert_eq!(rig.est.validity(), Validity::ALL);
        assert!(rig.est.status().gps_aiding);
    }

    #[test]
    fn the_correction_snaps_to_the_fix_and_gps_is_fused_again_at_once() {
        // Before the correction the wrong heading has the filter's position
        // and velocity away from the fixes and the fixes rejected.
        let rig = wrong_rig(1.0, 3.0);
        let mut rig = rig;
        let mut resets_before = 0;
        let mut rejected_before = 0;
        let mut corrected = None;
        while rig.now().as_secs_f64() < 14.0 && corrected.is_none() {
            resets_before = rig.est.status().resets;
            rejected_before = rig.est.status().rejected;
            let next = rig.now().as_secs_f64() + 0.001;
            fly_path(&mut rig, 3.0, next);
            corrected = rig.est.take_heading_corrected();
        }
        corrected.expect("a correction");
        // Counted as a reset, exactly one (guidance re-bases on the change).
        assert_eq!(rig.est.status().resets, resets_before + 1);
        // Position and velocity are the fix's (the filter's own delayed
        // state: look at it, not at the output a tick later).
        let ekf = rig.est.ekf().unwrap();
        let fit = rig.est.heading_fit();
        let fix_t = fit.t_us;
        let w1 = 1.1f32;
        let w2 = 0.7f32;
        let t = (fix_t as f64 * 1e-6 - 3.0) as f32;
        let want_v = [6.0 * Real::sin(w1 * t), 5.0 * Real::sin(w2 * t)];
        // The filter has stepped on to the end of the 4 ms step that
        // contained the fix: at most a few mm/s of acceleration times 4 ms.
        assert!(
            (ekf.x.v.x - want_v[0]).abs() < 0.2,
            "v {:?} want {want_v:?}",
            ekf.x.v
        );
        assert!(
            (ekf.x.v.y - want_v[1]).abs() < 0.2,
            "v {:?} want {want_v:?}",
            ekf.x.v
        );
        // No rejection at the very next fixes and the heading is right.
        let until = rig.now().as_secs_f64() + 2.0;
        fly_path(&mut rig, 3.0, until);
        assert!(rig.est.status().rejected <= rejected_before + 1);
        assert_eq!(rig.est.validity(), Validity::ALL);
        assert!(rig.est.status().gps_aiding);
    }

    #[test]
    fn the_correction_sets_the_yaw_sigma_to_the_fits_but_not_below_a_tenth() {
        let var_after = |sigma: f32| {
            let mut rig = Rig::new(&EkfParams::default());
            rig.run_until(3.0);
            let mut ekf = rig.est.ekf.take().unwrap();
            let fix = Stamped::ideal(
                Micros(3_000_000),
                GpsSample {
                    h_acc_m: 0.6,
                    s_acc_mps: 0.2,
                    fix_3d: true,
                    ..GpsSample::default()
                },
            );
            rig.est
                .correct_heading(&mut ekf, &fix, 1.0, sigma, 0.6, 0.3);
            ekf.p.get(ekf::ITH + 2, ekf::ITH + 2)
        };
        // A fit sigma below 0.1 rad: the reset's sigma is 0.1.
        assert!((var_after(0.02) - 0.01).abs() < 1e-4);
        assert!((var_after(0.099) - 0.01).abs() < 1e-4);
        // Above it, the fit's own.
        assert!((var_after(0.3) - 0.09).abs() < 1e-3);
        assert!((var_after(0.15) - 0.0225).abs() < 1e-3);
    }

    #[test]
    fn the_correction_gives_the_tilt_variances_back_their_initial_value() {
        let init = EkfParams::default().init_tilt_sigma_rad;
        let rig = wrong_rig(1.0, 3.0);
        // Just before: the filter has learnt its tilt, variance below the
        // initial one.
        let mut rig = rig;
        let mut before = f32::NAN;
        let mut corrected = None;
        while rig.now().as_secs_f64() < 14.0 && corrected.is_none() {
            before = rig.est.ekf().unwrap().p.get(ekf::ITH, ekf::ITH);
            let next = rig.now().as_secs_f64() + 0.001;
            fly_path(&mut rig, 3.0, next);
            corrected = rig.est.take_heading_corrected();
        }
        corrected.expect("a correction");
        assert!(
            before < 0.5 * init * init,
            "variance before {before}, init {}",
            init * init
        );
        for i in [ekf::ITH, ekf::ITH + 1] {
            let after = rig.est.ekf().unwrap().p.get(i, i);
            assert!(
                (after - init * init).abs() < 0.1 * init * init,
                "tilt variance {i}: {after}, init {}",
                init * init
            );
        }
    }

    #[test]
    fn the_declination_shift_keeps_the_magnetometer_innovation_near_zero() {
        let rig = wrong_rig(1.0, 3.0);
        let (mut rig, dpsi, _) = until_first_correction(rig, 14.0);
        let innovation = |rig: &Rig| {
            let e = rig.est.ekf().unwrap();
            wrap_pi(e.mag_heading(rig.mag_field).unwrap() - e.x.q.yaw())
        };
        assert!(
            innovation(&rig).abs() < 0.1,
            "innovation {}",
            innovation(&rig)
        );
        assert!((rig.est.declination_shift() - dpsi).abs() < 1e-6);
        assert!((rig.est.ekf().unwrap().declination() - (1.0 + dpsi)).abs() < 1e-5);
        assert_eq!(
            rig.est.status().heading_shift_rad,
            rig.est.declination_shift()
        );
        // And it stays there: the magnetometer holds the corrected heading
        // instead of dragging it back or being rejected.
        let rejected = rig.est.status().rejected;
        let until = rig.now().as_secs_f64() + 6.0;
        fly_path(&mut rig, 3.0, until);
        assert!(innovation(&rig).abs() < 0.1);
        assert_eq!(rig.est.status().rejected, rejected, "nothing rejected");
        assert_eq!(rig.est.heading_corrections(), 1);
    }

    #[test]
    fn the_fit_restarts_at_a_correction_and_one_verdict_is_one_correction() {
        let rig = wrong_rig(1.0, 3.0);
        let (mut rig, _, _) = until_first_correction(rig, 14.0);
        // The record is the correcting fix's; the next fix is the first of a
        // new fit: nothing of the old one is left.
        let seq = rig.est.heading_fit().seq;
        let verdicts = rig.est.heading_verdicts();
        while rig.est.heading_fit().seq == seq {
            let next = rig.now().as_secs_f64() + 0.001;
            fly_path(&mut rig, 3.0, next);
        }
        let f = rig.est.heading_fit();
        assert!(!f.used && f.sa == 0.0 && f.streak == 0, "{f:?}");
        let until = rig.now().as_secs_f64() + 8.0;
        fly_path(&mut rig, 3.0, until);
        let f = rig.est.heading_fit();
        assert!(f.seq > seq + 20, "fixes were judged after it");
        // The fit rebuilt from nothing reads the right heading.
        assert!(f.eps.abs() < 0.3, "eps after the correction {}", f.eps);
        assert_eq!(rig.est.heading_verdicts(), verdicts, "no more verdicts");
        assert_eq!(rig.est.heading_corrections(), 1);
    }

    #[test]
    fn the_budget_two_corrections_then_the_third_verdict_is_containment() {
        // The true heading steps by 0.9 rad at 10 s and by another 0.9 at
        // 25 s with no gyro to show it (the magnetometer disagrees and is
        // rejected): each is a wrong heading the check finds, after the
        // declination fault's own correction at 4 s.
        let psi = |t: f32| {
            if t >= 22.0 {
                1.8
            } else if t >= 7.0 {
                0.9
            } else {
                0.0
            }
        };
        let mut rig = wrong_rig(1.0, 3.0);
        let mut seen = Vec::new();
        let mut contained_at = None;
        while rig.now().as_secs_f64() < 40.0 {
            let next = rig.now().as_secs_f64() + 0.001;
            fly_path_heading(&mut rig, 3.0, next, psi);
            if let Some(d) = rig.est.take_heading_corrected() {
                seen.push((rig.now().as_secs_f64(), d));
                assert!(!rig.est.heading_contained());
                assert!(rig.est.validity().heading);
            }
            if rig.est.take_heading_contained() {
                contained_at = Some(rig.now().as_secs_f64());
            }
        }
        // The declination fault (1 rad), then the two heading steps: the
        // budget allows two corrections; the third verdict is containment.
        assert_eq!(seen.len(), 2, "corrections {seen:?}");
        assert_eq!(rig.est.heading_corrections(), 2);
        assert!(contained_at.is_some_and(|t| t > seen[1].0));
        assert!(rig.est.heading_contained());
        assert!(rig.est.status().heading_contained);
        // The second correction is right: it undoes the first step (+0.9 rad
        // of true heading), as seen from the heading the first correction
        // had made right (to within the fit's bias, a few tenths).
        assert!(
            seen[1].1 > 0.4 && seen[1].1 < 1.3,
            "second correction {:?}",
            seen[1]
        );
    }

    #[test]
    fn with_a_budget_of_zero_the_first_verdict_is_containment() {
        let params = EkfParams {
            mag_declination_rad: 1.0,
            hdg_max_corrections: 0,
            ..EkfParams::default()
        };
        let mut rig = wrong_rig_with(&params, 3.0);
        let mut when = None;
        while rig.now().as_secs_f64() < 14.0 && when.is_none() {
            let next = rig.now().as_secs_f64() + 0.001;
            fly_path(&mut rig, 3.0, next);
            if rig.est.take_heading_contained() {
                when = Some(rig.now());
            }
        }
        assert!(when.is_some(), "contained");
        assert_eq!(rig.est.heading_corrections(), 0);
        assert!(rig.est.take_heading_corrected().is_none());
        assert_eq!(rig.est.declination_shift(), 0.0);
    }

    #[test]
    fn contained_the_heading_velocity_and_position_are_invalid_and_gps_is_not_fused() {
        let params = EkfParams {
            mag_declination_rad: 1.0,
            hdg_max_corrections: 0,
            ..EkfParams::default()
        };
        let mut rig = wrong_rig_with(&params, 3.0);
        while rig.now().as_secs_f64() < 14.0 && !rig.est.heading_contained() {
            let next = rig.now().as_secs_f64() + 0.001;
            fly_path(&mut rig, 3.0, next);
        }
        assert!(rig.est.heading_contained());
        assert!(rig.est.take_heading_contained(), "the event, once");
        let v = rig.est.validity();
        assert!(!v.heading && !v.velocity && !v.position, "{v:?}");
        assert!(v.attitude && v.altitude, "{v:?}");
        // The latch is not `heading_ok`: that stays true, or the next
        // magnetometer sample would reset the yaw to the (wrong)
        // magnetometer heading and declare it good.
        assert!(rig.est.heading_ok);
        let resets = rig.est.status().resets;
        let t_accept = rig.est.gps.last_accept;
        let pos_accept = rig.est.gps.pos_accept;
        let until = rig.now().as_secs_f64() + 6.0;
        fly_path(&mut rig, 3.0, until);
        assert!(rig.est.heading_ok);
        assert_eq!(rig.est.status().resets, resets, "no reset of any kind");
        // GPS is not fused: nothing accepted since the latch, so its
        // timeout runs out.
        assert_eq!(rig.est.gps.last_accept, t_accept);
        assert_eq!(rig.est.gps.pos_accept, pos_accept);
        assert!(!rig.est.status().gps_aiding);
        // Still judged, for the log.
        assert!(rig.est.heading_fit().seq > 0);
        // And no second event, however many verdict fixes follow.
        assert!(!rig.est.take_heading_contained());
    }

    #[test]
    fn a_magnetometer_sample_does_not_reset_the_yaw_of_a_contained_filter() {
        let params = EkfParams {
            mag_declination_rad: 1.0,
            hdg_max_corrections: 0,
            ..EkfParams::default()
        };
        let mut rig = wrong_rig_with(&params, 3.0);
        while rig.now().as_secs_f64() < 14.0 && !rig.est.heading_contained() {
            let next = rig.now().as_secs_f64() + 0.001;
            fly_path(&mut rig, 3.0, next);
        }
        let resets = rig.est.status().resets;
        // The magnetometer now says a heading 2 rad away from the yaw: a
        // filter whose `heading_ok` was cleared would take it as the
        // heading at the next sample.
        rig.mag_field = Quat::from_euler(0.0, 0.0, 2.0).inverse_rotate(MAG);
        let until = rig.now().as_secs_f64() + 1.0;
        fly_path(&mut rig, 3.0, until);
        assert!(rig.est.heading_ok);
        assert_eq!(rig.est.status().resets, resets);
        // The magnetometer disagrees by 2 rad: gated, not taken as the
        // heading.
        assert!(rig.est.ekf().unwrap().x.q.yaw().abs() < 1.5);
        assert!(!rig.est.validity().heading);
    }

    #[test]
    fn the_shift_survives_a_new_filter_and_is_applied_again() {
        let rig = wrong_rig(1.0, 3.0);
        let (mut rig, dpsi, _) = until_first_correction(rig, 14.0);
        // An IMU stall in flight: the filter is dropped, the vehicle then
        // sits still and the estimator aligns afresh.
        rig.imu_on = false;
        let t = rig.now().as_secs_f64();
        fly_path(&mut rig, 3.0, t + 0.3);
        assert!(rig.est.ekf().is_none());
        assert!(
            (rig.est.declination_shift() - dpsi).abs() < 1e-6,
            "kept outside the filter"
        );
        rig.imu_on = true;
        rig.accel = Vec3::new(0.0, 0.0, -GRAVITY);
        rig.mag_field = MAG;
        rig.gps_vel = Vec3::zero();
        rig.run_until(t + 4.0);
        assert_eq!(rig.est.heading_corrections(), 1, "not cleared by the drop");
        let e = rig.est.ekf().expect("aligned again");
        assert!(
            (e.declination() - (1.0 + dpsi)).abs() < 1e-5,
            "declination {}",
            e.declination()
        );
        // The heading is the corrected one (declination 1 rad plus the
        // correction), not the wrong declination's 1 rad.
        assert!(
            wrap_pi(e.x.q.yaw() - (1.0 + dpsi)).abs() < 0.05,
            "yaw {}",
            e.x.q.yaw()
        );
        assert_eq!(rig.est.validity(), Validity::ALL);
    }

    #[test]
    fn disarm_clears_the_count_the_latch_and_the_shift_and_the_filter_aligns_again() {
        let rig = wrong_rig(1.0, 3.0);
        let (mut rig, _, _) = until_first_correction(rig, 14.0);
        assert_eq!(rig.est.heading_corrections(), 1);
        assert!(rig.est.declination_shift() != 0.0);
        // The vehicle lands and is disarmed, at rest.
        rig.accel = Vec3::new(0.0, 0.0, -GRAVITY);
        rig.gps_vel = Vec3::zero();
        rig.mag_field = MAG;
        rig.est.set_airborne(false);
        rig.est.set_armed(false);
        let t_disarm = rig.now().as_secs_f64();
        assert_eq!(rig.est.heading_corrections(), 0);
        assert!(!rig.est.heading_contained());
        assert_eq!(rig.est.declination_shift(), 0.0);
        let s = rig.est.status();
        assert!(s.heading_corrections == 0 && !s.heading_contained && s.heading_shift_rad == 0.0);
        assert!(rig.est.take_heading_corrected().is_none());
        // The filter's heading (the truth) and the magnetometer (with the
        // parameter's declination) now disagree: the filter is dropped, and
        // it aligns again from the magnetometer: it does not carry the
        // disagreement and it is not left rejecting the magnetometer.
        assert!(rig.est.ekf().is_none());
        assert_eq!(rig.est.validity(), Validity::NONE);
        let mut valid_after = None;
        while rig.now().as_secs_f64() < t_disarm + 8.0 {
            rig.tick();
            if valid_after.is_none() && rig.est.validity() == Validity::ALL {
                valid_after = Some(rig.now().as_secs_f64() - t_disarm);
            }
        }
        let valid_after = valid_after.expect("valid again");
        assert!(valid_after < 5.0, "valid again after {valid_after} s");
        assert!(
            rig.est.estimate().is_some(),
            "an estimate: the vehicle can be armed again"
        );
        let e = rig.est.ekf().unwrap();
        assert_eq!(e.declination(), 1.0, "the parameter again");
        // Consistent with the magnetometer: no disagreement left.
        let nu = wrap_pi(e.mag_heading(MAG).unwrap() - e.x.q.yaw());
        assert!(nu.abs() < 0.05, "innovation {nu}");
        // A flight that was not touched is not disturbed by a disarm.
        let mut quiet = Rig::new(&EkfParams::default());
        quiet.run_until(4.0);
        quiet.est.set_armed(true);
        quiet.run_until(5.0);
        quiet.est.set_armed(false);
        quiet.run_until(5.5);
        assert!(quiet.est.ekf().is_some());
        assert_eq!(quiet.est.validity(), Validity::ALL);
    }

    #[test]
    fn disarm_clears_a_latch_and_a_new_flight_can_correct_again() {
        let params = EkfParams {
            mag_declination_rad: 1.0,
            hdg_max_corrections: 0,
            ..EkfParams::default()
        };
        let mut rig = wrong_rig_with(&params, 3.0);
        while rig.now().as_secs_f64() < 14.0 && !rig.est.heading_contained() {
            let next = rig.now().as_secs_f64() + 0.001;
            fly_path(&mut rig, 3.0, next);
        }
        assert!(rig.est.heading_contained());
        rig.est.set_airborne(false);
        rig.est.set_armed(false);
        assert!(!rig.est.heading_contained());
        assert!(!rig.est.status().heading_contained);
        rig.accel = Vec3::new(0.0, 0.0, -GRAVITY);
        rig.gps_vel = Vec3::zero();
        rig.gps_pos = Vec3::zero();
        rig.mag_field = MAG;
        let t = rig.now().as_secs_f64();
        rig.run_until(t + 6.0);
        assert!(rig.est.validity().heading, "{:?}", rig.est.validity());
    }

    #[test]
    fn nothing_is_corrected_or_contained_on_the_ground() {
        let mut rig = Rig::new(&EkfParams {
            mag_declination_rad: 1.0,
            ..EkfParams::default()
        });
        rig.run_until(3.0);
        rig.est.set_armed(true);
        // Armed, on the ground (not airborne): the vehicle moves as in the
        // flights above, and nothing acts.
        fly_path(&mut rig, 3.0, 14.0);
        assert_eq!(rig.est.heading_corrections(), 0);
        assert!(!rig.est.heading_contained());
        assert_eq!(rig.est.heading_verdicts(), 0);
        assert_eq!(rig.est.declination_shift(), 0.0);
    }

    #[test]
    fn the_heading_check_restarts_where_the_yaw_is_reset() {
        let mut rig = Rig::new(&EkfParams::default());
        rig.run_until(3.0);
        rig.est.set_airborne(true);
        fly_path(&mut rig, 3.0, 8.0);
        assert!(rig.est.heading_fit().sa > 1.0);
        // The magnetometer is silent for longer than its timeout, then
        // returns: the filter resets its yaw to it (`fuse_mag`).
        rig.mag_on = false;
        fly_path(&mut rig, 3.0, 8.0 + 31.0);
        assert!(rig.est.heading_fit().sa > 1.0, "the fit goes on without it");
        let resets = rig.est.status().resets;
        rig.mag_on = true;
        // The magnetometer sample is fused 0.2 s after it was taken (the
        // filter runs on a delayed horizon); wait for the reset.
        let sa_before = rig.est.heading_fit().sa;
        let mut reset_tick = None;
        let mut first_after = None;
        let mut prev_seq = rig.est.heading_fit().seq;
        while rig.now().as_secs_f64() < 8.0 + 36.0 {
            let next = rig.now().as_secs_f64() + 0.001;
            fly_path(&mut rig, 3.0, next);
            let f = rig.est.heading_fit();
            if reset_tick.is_none() && rig.est.status().resets > resets {
                reset_tick = Some(f.seq);
            }
            // The first fix judged at or after the reset: the reset and the
            // fix can fall in the same tick (the reset is first).
            if reset_tick.is_some() && first_after.is_none() && f.seq != prev_seq {
                first_after = Some(f);
            }
            prev_seq = f.seq;
        }
        assert!(reset_tick.is_some(), "the yaw was reset");
        // That fix is the first of a new fit: no interval, nothing carried
        // over.
        let f = first_after.expect("a fix after the reset");
        assert!(sa_before > 2.0);
        assert!(!f.used && f.sa == 0.0, "{f:?}");
        // And it builds up again afterwards.
        assert!(rig.est.heading_fit().sa > 1.0);
    }

    #[test]
    fn the_heading_check_restarts_when_the_filter_is_dropped() {
        let mut rig = Rig::new(&EkfParams::default());
        rig.run_until(3.0);
        rig.est.set_airborne(true);
        fly_path(&mut rig, 3.0, 8.0);
        assert!(rig.est.heading_fit().sa > 1.0);
        // An IMU stall: the filter is dropped, and everything restarts.
        rig.imu_on = false;
        let t = rig.now().as_secs_f64();
        fly_path(&mut rig, 3.0, t + 0.3);
        assert!(rig.est.ekf().is_none());
        rig.imu_on = true;
        rig.accel = Vec3::new(0.0, 0.0, -GRAVITY);
        rig.gps_vel = Vec3::zero();
        // (The unaligned filter takes no fixes; the fit is cleared when it
        // does come back and the vehicle is still airborne.)
        rig.run_until(t + 4.0);
        let f = rig.est.heading_fit();
        assert!(f.sa < 0.5, "{f:?}");
    }
}
