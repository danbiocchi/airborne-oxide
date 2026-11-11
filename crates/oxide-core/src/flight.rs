// SPDX-License-Identifier: AGPL-3.0-or-later
//! The flight core: one `tick` per base period (DESIGN §4.2, ADR-0001).
//!
//! The supervisor (arming, failsafes, navigation state) lives in
//! `supervisor`, the guidance task in `guidance` (ADR-0003).

mod guidance;
mod supervisor;
#[cfg(test)]
mod tests;

use crate::allocation::{Mixer, command_for_fraction};
use crate::config::{ConfigError, EstimatorSource, VehicleConfig};
use crate::control::{
    AttitudeTarget, PositionController, RateController, RateTarget, attitude_rate_setpoint,
};
use crate::estimator::ring::Ring;
use crate::estimator::{Estimator, IMU_TIMEOUT_US};
use crate::filters::LowPass2;
use crate::io::{
    Action, BatterySample, BrainHeartbeat, EVENTS_PER_TICK, EstimatorStatus, Event, EventKind,
    EventList, FlightMode, GcsHeartbeat, ImuSample, Inputs, NavState, Outputs, PrearmFlags,
    RcSample, Reasons, Stamped, StateSample, Status, TrajectorySetpoint, Validity,
};
use crate::math::{Real, Vec3};
use crate::schedule::{Profile, TaskId};
use crate::time::Micros;

use guidance::GuidanceState;
use supervisor::SupervisorState;

/// How old a sample may be before it is treated as missing, µs (the same
/// 100 ms the estimator applies to the IMU).
const STATE_TIMEOUT_US: u64 = IMU_TIMEOUT_US;
const BATTERY_TIMEOUT_US: u64 = 1_000_000;
/// Consecutive supervisor runs that must see the kill switch (DESIGN §4.9).
const KILL_CONFIRMATIONS: u8 = 3;
/// Throttle must be below this to arm.
const ARM_MAX_THROTTLE: f32 = 0.1;
/// Body rate must be below this to arm, rad/s.
const ARM_MAX_RATE: f32 = 0.1;
/// Position mode keeps its setpoint within this distance of the vehicle, m.
const POSITION_LEASH_M: f32 = 2.0;
/// Capacity of the event queue.
const EVENT_QUEUE: usize = 32;

/// Output of a task under the logical-execution-time rule: computed into
/// `pending`, visible to other tasks only after the next release.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct Let<T: Copy> {
    pending: T,
    visible: T,
}

impl<T: Copy> Let<T> {
    fn publish(&mut self) {
        self.visible = self.pending;
    }
}

/// The published estimate: a state and which parts of it to trust.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct Estimate {
    state: Option<StateSample>,
    valid: Validity,
    /// The estimator's state resets so far (zero for the external source).
    /// A change means the estimate jumped rather than moved.
    resets: u32,
}

/// Supervisor decisions.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct Supervision {
    armed: bool,
    killed: bool,
    /// The pilot's mode.
    mode: FlightMode,
    nav: NavState,
    home: Option<Vec3<f32>>,
    reasons: Reasons,
    action: Action,
    prearm: PrearmFlags,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct Periods {
    estimator: u64,
    supervisor: u64,
    guidance: u64,
    attitude: u64,
}

/// The flight core for a vehicle with `N` rotors.
#[derive(Clone, Debug)]
pub struct Core<const N: usize> {
    cfg: VehicleConfig<N>,
    mixer: Mixer<N>,
    profile: Profile,
    periods: Periods,
    ticks: u64,
    last_now: Option<Micros>,
    timing_faults: u32,

    imu: Option<Stamped<ImuSample>>,
    state: Option<Stamped<StateSample>>,
    rc: Option<Stamped<RcSample>>,
    battery: Option<Stamped<BatterySample>>,
    setpoint: Option<Stamped<TrajectorySetpoint>>,
    gcs: Option<Stamped<GcsHeartbeat>>,
    /// The brain's last progress value, and the arrival time at which it
    /// last changed (the first heartbeat counts as a change).
    brain_progress: Option<u32>,
    brain_changed_at: Option<Micros>,

    estimator: Estimator,
    /// Rate-loop gyro low-pass, and its latest output (body, rad/s).
    gyro_lpf: LowPass2,
    gyro_filtered: Vec3<f32>,

    estimate: Let<Estimate>,
    supervision: Let<Supervision>,
    guidance: Let<AttitudeTarget>,
    attitude: Let<RateTarget>,

    sup_state: SupervisorState,
    guide_state: GuidanceState,
    position_ctrl: PositionController,
    rate_ctrl: RateController,

    events: Ring<Option<Event>, EVENT_QUEUE>,
    events_dropped: u32,
}

impl<const N: usize> Core<N> {
    /// Builds a core. Fails if the vehicle or profile is malformed.
    pub fn new(cfg: VehicleConfig<N>, profile: Profile) -> Result<Self, ConfigError> {
        cfg.validate()?;
        profile.validate()?;
        let mixer = Mixer::new(&cfg.rotors)?;
        let p = |id| {
            profile
                .period(id)
                .map(u64::from)
                .ok_or(ConfigError::BadProfile)
        };
        let periods = Periods {
            estimator: p(TaskId::Estimator)?,
            supervisor: p(TaskId::Supervisor)?,
            guidance: p(TaskId::Guidance)?,
            attitude: p(TaskId::Attitude)?,
        };
        let estimator = Estimator::new(&cfg.estimator.ekf, &profile)?;
        let gyro_lpf = LowPass2::new(1.0 / profile.base_dt(), cfg.gains.gyro_cutoff_hz);
        Ok(Self {
            cfg,
            mixer,
            profile,
            periods,
            ticks: 0,
            last_now: None,
            timing_faults: 0,
            imu: None,
            state: None,
            rc: None,
            battery: None,
            setpoint: None,
            gcs: None,
            brain_progress: None,
            brain_changed_at: None,
            estimator,
            gyro_lpf,
            gyro_filtered: Vec3::zero(),
            estimate: Let::default(),
            supervision: Let::default(),
            guidance: Let::default(),
            attitude: Let::default(),
            sup_state: SupervisorState::new(),
            guide_state: GuidanceState::default(),
            position_ctrl: PositionController::default(),
            rate_ctrl: RateController::default(),
            events: Ring::new(),
            events_dropped: 0,
        })
    }

    pub fn config(&self) -> &VehicleConfig<N> {
        &self.cfg
    }

    pub fn profile(&self) -> &Profile {
        &self.profile
    }

    /// The estimator, for logging and tests (its filter state, covariance
    /// and health counters).
    pub fn estimator(&self) -> &Estimator {
        &self.estimator
    }

    /// The state estimate the controllers currently see (the published
    /// one), or `None` while attitude is not valid. Which parts of the state
    /// to trust is in `Status::valid`: with the EKF, position or heading can
    /// be stale or wrong while this is `Some`.
    pub fn estimate(&self) -> Option<StateSample> {
        let e = self.estimate.visible;
        e.state.filter(|_| e.valid.attitude)
    }

    /// Home, set at arming when position was valid.
    pub fn home(&self) -> Option<Vec3<f32>> {
        self.supervision.visible.home
    }

    /// Events dropped because the queue (32 events) overflowed.
    pub fn events_dropped(&self) -> u32 {
        self.events_dropped
    }

    /// Runs one base tick. Call exactly once per base period.
    pub fn tick(&mut self, now: Micros, inputs: &Inputs) -> Outputs<N> {
        let base_us = u64::from(self.profile.base_period_us);
        if let Some(last) = self.last_now
            && now.since(last) != base_us
        {
            self.timing_faults = self.timing_faults.saturating_add(1);
        }
        self.absorb(inputs);

        let k = self.ticks;
        let released = |period: u64| period > 0 && k.is_multiple_of(period);
        let (est, sup, gui, att) = (
            released(self.periods.estimator),
            released(self.periods.supervisor),
            released(self.periods.guidance),
            released(self.periods.attitude),
        );

        // Publish what each released task computed in its previous period…
        if est {
            self.estimate.publish();
        }
        if sup {
            self.supervision.publish();
        }
        if gui {
            self.guidance.publish();
        }
        if att {
            self.attitude.publish();
        }
        // …then run them on the inputs as they are now.
        let base_dt = self.profile.base_dt();
        if est {
            self.run_estimator(now);
        }
        if sup {
            #[allow(clippy::cast_precision_loss)]
            self.run_supervisor(now, base_dt * self.periods.supervisor as f32);
        }
        if gui {
            #[allow(clippy::cast_precision_loss)]
            self.run_guidance(now, base_dt * self.periods.guidance as f32);
        }
        if att {
            self.run_attitude();
        }
        let out = self.run_rate(now, base_dt, base_us);

        self.ticks = self.ticks.saturating_add(1);
        self.last_now = Some(now);
        out
    }

    /// Keeps the newest sample of each kind.
    fn absorb(&mut self, inputs: &Inputs) {
        let ekf = self.cfg.estimator.source == EstimatorSource::Ekf;
        if let Some(imu) = inputs.imu
            && imu.value.gyro.is_finite()
            && imu.value.accel.is_finite()
        {
            self.gyro_filtered = self.gyro_lpf.apply(imu.value.gyro);
            self.imu = Some(imu);
            if ekf {
                self.estimator.push_imu(&imu);
            }
        }
        if ekf {
            if let Some(s) = &inputs.gps {
                self.estimator.push_gps(s);
            }
            if let Some(s) = &inputs.baro {
                self.estimator.push_baro(s);
            }
            if let Some(s) = &inputs.mag {
                self.estimator.push_mag(s);
            }
        }
        if inputs.state.is_some() {
            self.state = inputs.state;
        }
        if let Some(rc) = inputs.rc {
            self.rc = Some(rc);
            if !rc.value.is_valid() {
                // A bad frame is a gap, even if no supervisor run sees it.
                self.sup_state.rc_valid_since = None;
            }
        }
        if inputs.battery.is_some() {
            self.battery = inputs.battery;
        }
        if let Some(sp) = inputs.setpoint {
            // Gateway, M1a subset: drop non-finite setpoints.
            let v = sp.value;
            if v.position.is_finite() && v.velocity.is_finite() && v.yaw.is_finite() {
                self.setpoint = Some(sp);
            }
        }
        if inputs.gcs_heartbeat.is_some() {
            self.gcs = inputs.gcs_heartbeat;
        }
        if let Some(hb) = inputs.brain_heartbeat {
            let BrainHeartbeat { progress } = hb.value;
            if self.brain_progress != Some(progress) {
                self.brain_progress = Some(progress);
                self.brain_changed_at = Some(hb.t_arrival);
            }
        }
    }

    fn fresh<T: Copy>(sample: Option<Stamped<T>>, now: Micros, timeout_us: u64) -> Option<T> {
        // A sample stamped in the future is corrupt, not fresh.
        sample
            .filter(|s| s.t_arrival <= now && now.since(s.t_arrival) <= timeout_us)
            .map(|s| s.value)
    }

    fn valid_rc(&self, now: Micros) -> Option<RcSample> {
        Self::fresh(self.rc, now, u64::from(self.cfg.failsafe.rc_timeout_us))
            .filter(RcSample::is_valid)
    }

    fn run_estimator(&mut self, now: Micros) {
        if self.cfg.estimator.source == EstimatorSource::Ekf {
            self.estimator.update(now);
            let state = self.estimator.state();
            let valid = if state.is_some() {
                self.estimator.validity()
            } else {
                Validity::NONE
            };
            self.estimate.pending = Estimate {
                state,
                valid,
                resets: self.estimator.status().resets,
            };
            return;
        }
        // External: the estimate is the `state` input (simulator ground
        // truth, motion capture).
        let state = Self::fresh(self.state, now, STATE_TIMEOUT_US).filter(|s| {
            s.position.is_finite()
                && s.velocity.is_finite()
                && s.attitude.is_finite()
                && s.rate.is_finite()
        });
        self.estimate.pending = Estimate {
            state,
            valid: if state.is_some() {
                Validity::ALL
            } else {
                Validity::NONE
            },
            resets: 0,
        };
    }

    fn estimator_status(&self) -> EstimatorStatus {
        match self.cfg.estimator.source {
            EstimatorSource::Ekf => self.estimator.status(),
            EstimatorSource::External => EstimatorStatus {
                aligned: self.estimate.visible.valid.attitude,
                ..EstimatorStatus::default()
            },
        }
    }

    /// Estimated gyro bias, rad/s (zero for the external source).
    fn gyro_bias(&self) -> Vec3<f32> {
        if self.cfg.estimator.source == EstimatorSource::Ekf {
            self.estimator.gyro_bias()
        } else {
            Vec3::zero()
        }
    }

    /// The body rate the rate loop uses: the filtered, bias-corrected gyro
    /// while the IMU is fresh, else the estimate's rate.
    fn loop_gyro(&self, now: Micros) -> Option<Vec3<f32>> {
        Self::fresh(self.imu, now, STATE_TIMEOUT_US)
            .map(|_| self.gyro_filtered - self.gyro_bias())
            .or_else(|| self.estimate.visible.state.map(|e| e.rate))
    }

    fn emit(&mut self, now: Micros, kind: EventKind) {
        let dropped = self.events.push(Some(Event { t: now, kind }));
        if dropped.is_some() {
            self.events_dropped = self.events_dropped.saturating_add(1);
        }
    }

    fn run_attitude(&mut self) {
        let sup = self.supervision.visible;
        let target = self.guidance.visible;
        self.attitude.pending = match self.estimate.visible.state {
            Some(est) if sup.armed && self.estimate.visible.valid.attitude => RateTarget {
                rate: attitude_rate_setpoint(
                    est.attitude,
                    target.attitude,
                    &self.cfg.gains,
                    &self.cfg.limits,
                ),
                thrust_n: target.thrust_n,
            },
            _ => RateTarget::default(),
        };
    }

    fn run_rate(&mut self, now: Micros, dt: f32, base_us: u64) -> Outputs<N> {
        let sup = self.supervision.visible;
        let est = self.estimate.visible;
        let status = Status {
            armed: sup.armed,
            killed: sup.killed,
            mode: sup.mode,
            saturated: false,
            timing_faults: self.timing_faults,
            no_estimate: !est.valid.attitude,
            estimator: self.estimator_status(),
            nav: sup.nav,
            valid: est.valid,
            reasons: sup.reasons,
            action: sup.action,
            prearm: sup.prearm,
            home_set: sup.home.is_some(),
        };
        let mut events = EventList::new();
        for _ in 0..EVENTS_PER_TICK {
            match self.events.pop_front() {
                Some(Some(e)) => {
                    events.push(e);
                }
                _ => break,
            }
        }
        let t_valid = now.add_us(base_us);
        let mut out = Outputs {
            t_valid,
            motors: [0.0; N],
            status,
            events,
        };
        if !sup.armed {
            self.rate_ctrl.reset();
            return out;
        }
        let target = self.attitude.visible;
        let gyro = self.loop_gyro(now).unwrap_or_else(Vec3::zero);
        // Don't wind up the integrator while sitting on the ground.
        let airborne_ish = target.thrust_n >= 0.5 * self.cfg.weight_n();
        let alpha = self.rate_ctrl.update(
            target.rate,
            gyro,
            dt,
            &self.cfg.gains,
            &self.cfg.limits,
            airborne_ish,
        );
        let torque = self.cfg.inertia.hadamard(alpha);
        let alloc = self.mixer.mix(torque, target.thrust_n);
        out.status.saturated = alloc.saturated;

        // Battery sag lowers motor speed for a given command; compensate.
        let comp = Self::fresh(self.battery, now, BATTERY_TIMEOUT_US)
            .map(|b| b.voltage_v)
            .filter(|v| v.is_finite() && *v > 0.5 * self.cfg.nominal_voltage_v)
            .map_or(1.0, |v| {
                let r = self.cfg.nominal_voltage_v / v;
                r * r
            });
        let idle = self.cfg.limits.idle_command;
        for (m, f) in out.motors.iter_mut().zip(alloc.fraction.iter()) {
            let frac = (*f * comp).clamp(0.0, 1.0);
            *m = command_for_fraction(frac, self.cfg.thrust_expo).max(idle);
        }
        out
    }
}

/// A stick position with the deadband `db` (`Limits::stick_deadband`) cut
/// out and the rest rescaled to the full range.
fn deadband(v: f32, db: f32) -> f32 {
    let v = v.finite_or(0.0).clamp(-1.0, 1.0);
    if Real::abs(v) < db {
        0.0
    } else if v > 0.0 {
        (v - db) / (1.0 - db)
    } else {
        (v + db) / (1.0 - db)
    }
}

/// Keeps `sp` within `radius` of `pos`.
fn leash(sp: Vec3<f32>, pos: Vec3<f32>, radius: f32) -> Vec3<f32> {
    let d = sp - pos;
    let n = d.norm();
    if n > radius && n > 0.0 {
        pos + d * (radius / n)
    } else {
        sp
    }
}
