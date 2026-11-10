// SPDX-License-Identifier: AGPL-3.0-or-later
//! The flight core: one `tick` per base period (DESIGN §4.2, ADR-0001).

use crate::allocation::{Mixer, command_for_fraction};
use crate::config::{ConfigError, EstimatorSource, VehicleConfig};
use crate::control::{
    AttitudeTarget, PositionController, RateController, RateTarget, attitude_rate_setpoint,
};
use crate::estimator::{Estimator, IMU_TIMEOUT_US};
use crate::filters::LowPass2;
use crate::io::{
    BatterySample, EstimatorStatus, EventList, FlightMode, ImuSample, Inputs, Outputs, RcSample,
    Stamped, StateSample, Status, TrajectorySetpoint,
};
use crate::math::{Quat, Real, Vec3, wrap_pi};
use crate::schedule::{Profile, TaskId};
use crate::time::Micros;

/// How old a sample may be before it is treated as missing, µs (the same
/// 100 ms the estimator applies to the IMU).
const STATE_TIMEOUT_US: u64 = IMU_TIMEOUT_US;
const RC_TIMEOUT_US: u64 = 500_000;
const SETPOINT_TIMEOUT_US: u64 = 500_000;
const BATTERY_TIMEOUT_US: u64 = 1_000_000;
/// Consecutive supervisor runs that must see the kill switch (DESIGN §4.9).
const KILL_CONFIRMATIONS: u8 = 3;
/// Throttle must be below this to arm.
const ARM_MAX_THROTTLE: f32 = 0.1;
/// Stick deadband.
const DEADBAND: f32 = 0.05;
/// Position mode keeps its setpoint within this distance of the vehicle, m.
const POSITION_LEASH_M: f32 = 2.0;

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

/// Supervisor decisions.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct Supervision {
    armed: bool,
    killed: bool,
    mode: FlightMode,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct SupervisorState {
    kill_count: u8,
    prev_arm_switch: bool,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct GuidanceState {
    position_sp: Vec3<f32>,
    yaw_sp: f32,
    last_mode: Option<FlightMode>,
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

    estimator: Estimator,
    /// Rate-loop gyro low-pass, and its latest output (body, rad/s).
    gyro_lpf: LowPass2,
    gyro_filtered: Vec3<f32>,

    estimate: Let<Option<StateSample>>,
    supervision: Let<Supervision>,
    guidance: Let<AttitudeTarget>,
    attitude: Let<RateTarget>,

    sup_state: SupervisorState,
    guide_state: GuidanceState,
    position_ctrl: PositionController,
    rate_ctrl: RateController,
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
            estimator,
            gyro_lpf,
            gyro_filtered: Vec3::zero(),
            estimate: Let::default(),
            supervision: Let::default(),
            guidance: Let::default(),
            attitude: Let::default(),
            sup_state: SupervisorState::default(),
            guide_state: GuidanceState::default(),
            position_ctrl: PositionController::default(),
            rate_ctrl: RateController::default(),
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
    /// one), or `None` before the estimator has a valid solution.
    pub fn estimate(&self) -> Option<StateSample> {
        self.estimate.visible
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
            self.run_supervisor(now);
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
        if inputs.rc.is_some() {
            self.rc = inputs.rc;
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
    }

    fn fresh<T: Copy>(sample: Option<Stamped<T>>, now: Micros, timeout_us: u64) -> Option<T> {
        // A sample stamped in the future is corrupt, not fresh.
        sample
            .filter(|s| s.t_arrival <= now && now.since(s.t_arrival) <= timeout_us)
            .map(|s| s.value)
    }

    fn valid_rc(&self, now: Micros) -> Option<RcSample> {
        Self::fresh(self.rc, now, RC_TIMEOUT_US).filter(RcSample::is_valid)
    }

    fn run_estimator(&mut self, now: Micros) {
        if self.cfg.estimator.source == EstimatorSource::Ekf {
            self.estimator.update(now);
            self.estimate.pending = self.estimator.estimate();
            return;
        }
        // External: the estimate is the `state` input (simulator ground
        // truth, motion capture).
        self.estimate.pending = Self::fresh(self.state, now, STATE_TIMEOUT_US).filter(|s| {
            s.position.is_finite()
                && s.velocity.is_finite()
                && s.attitude.is_finite()
                && s.rate.is_finite()
        });
    }

    fn estimator_status(&self) -> EstimatorStatus {
        match self.cfg.estimator.source {
            EstimatorSource::Ekf => self.estimator.status(),
            EstimatorSource::External => EstimatorStatus {
                aligned: self.estimate.visible.is_some(),
                ..EstimatorStatus::default()
            },
        }
    }

    fn run_supervisor(&mut self, now: Micros) {
        let mut s = self.supervision.visible;
        // M1a: without a valid RC frame, decisions stay as they are. RC-loss
        // failsafes arrive in M1c.
        if let Some(rc) = self.valid_rc(now) {
            self.sup_state.kill_count = if rc.kill_switch {
                self.sup_state
                    .kill_count
                    .saturating_add(1)
                    .min(KILL_CONFIRMATIONS)
            } else {
                0
            };
            if self.sup_state.kill_count >= KILL_CONFIRMATIONS {
                s.killed = true;
                s.armed = false;
            }
            if s.killed && !rc.kill_switch && !rc.arm_switch {
                s.killed = false;
            }
            let rising = rc.arm_switch && !self.sup_state.prev_arm_switch;
            let have_estimate = self.estimate.visible.is_some()
                && (self.cfg.estimator.source != EstimatorSource::Ekf
                    || self.estimator.status().gps_aiding);
            if !s.armed && rising && !s.killed && rc.throttle < ARM_MAX_THROTTLE && have_estimate {
                s.armed = true;
            }
            if s.armed && !rc.arm_switch {
                s.armed = false;
            }
            s.mode = match rc.mode_switch {
                0 => FlightMode::Stabilized,
                1 => FlightMode::Position,
                _ => FlightMode::Offboard,
            };
            self.sup_state.prev_arm_switch = rc.arm_switch;
        }
        self.supervision.pending = s;
    }

    fn run_guidance(&mut self, now: Micros, dt: f32) {
        let sup = self.supervision.visible;
        let Some(est) = self.estimate.visible else {
            self.position_ctrl.reset();
            self.guidance.pending = AttitudeTarget::default();
            return;
        };
        let current_yaw = est.attitude.yaw();
        if !sup.armed {
            self.position_ctrl.reset();
            self.guide_state = GuidanceState {
                position_sp: est.position,
                yaw_sp: current_yaw,
                last_mode: None,
            };
            self.guidance.pending = AttitudeTarget {
                attitude: est.attitude,
                thrust_n: 0.0,
            };
            return;
        }
        if self.guide_state.last_mode != Some(sup.mode) {
            // Entering a mode: hold where we are.
            self.guide_state.position_sp = est.position;
            self.guide_state.yaw_sp = current_yaw;
            self.position_ctrl.reset();
            self.guide_state.last_mode = Some(sup.mode);
        }
        let rc = self.valid_rc(now);
        let l = self.cfg.limits;
        let target = match (sup.mode, rc) {
            (FlightMode::Stabilized | FlightMode::Altitude, Some(rc)) => {
                self.guide_state.yaw_sp =
                    wrap_pi(self.guide_state.yaw_sp + deadband(rc.yaw) * l.max_yaw_rate * dt);
                self.guide_state.position_sp = est.position;
                let roll = rc.roll.clamp(-1.0, 1.0) * l.max_tilt_stabilized_rad;
                let pitch = -rc.pitch.clamp(-1.0, 1.0) * l.max_tilt_stabilized_rad;
                AttitudeTarget {
                    attitude: Quat::from_euler(roll, pitch, self.guide_state.yaw_sp),
                    thrust_n: rc.throttle.clamp(0.0, 1.0) * self.cfg.total_max_thrust_n(),
                }
            }
            (FlightMode::Position, Some(rc)) => {
                let v = self.stick_velocity(&rc, current_yaw);
                self.guide_state.yaw_sp =
                    wrap_pi(self.guide_state.yaw_sp + deadband(rc.yaw) * l.max_yaw_rate * dt);
                let next = self.guide_state.position_sp + v * dt;
                self.guide_state.position_sp = leash(next, est.position, POSITION_LEASH_M);
                self.position_ctrl.update(
                    self.guide_state.position_sp,
                    v,
                    self.guide_state.yaw_sp,
                    &est,
                    dt,
                    &self.cfg,
                )
            }
            (FlightMode::Offboard, _) => {
                let mut vff = Vec3::zero();
                if let Some(sp) = Self::fresh(self.setpoint, now, SETPOINT_TIMEOUT_US) {
                    self.guide_state.position_sp = sp.position;
                    self.guide_state.yaw_sp = wrap_pi(sp.yaw);
                    vff = sp.velocity;
                }
                self.position_ctrl.update(
                    self.guide_state.position_sp,
                    vff,
                    self.guide_state.yaw_sp,
                    &est,
                    dt,
                    &self.cfg,
                )
            }
            // No valid RC in a stick mode: hold position (M1a; M1c adds the
            // RC-loss failsafe).
            (FlightMode::Stabilized | FlightMode::Altitude | FlightMode::Position, None) => {
                self.position_ctrl.update(
                    self.guide_state.position_sp,
                    Vec3::zero(),
                    self.guide_state.yaw_sp,
                    &est,
                    dt,
                    &self.cfg,
                )
            }
        };
        self.guidance.pending = target;
    }

    /// Stick velocity demand in NED for position mode.
    fn stick_velocity(&self, rc: &RcSample, yaw: f32) -> Vec3<f32> {
        let l = &self.cfg.limits;
        let fwd = deadband(rc.pitch) * l.max_speed_xy;
        let right = deadband(rc.roll) * l.max_speed_xy;
        let (s, c) = (Real::sin(yaw), Real::cos(yaw));
        let climb = deadband((rc.throttle - 0.5) * 2.0);
        let vz = if climb > 0.0 {
            -climb * l.max_speed_up
        } else {
            -climb * l.max_speed_down
        };
        Vec3::new(fwd * c - right * s, fwd * s + right * c, vz)
    }

    fn run_attitude(&mut self) {
        let sup = self.supervision.visible;
        let target = self.guidance.visible;
        self.attitude.pending = match self.estimate.visible {
            Some(est) if sup.armed => RateTarget {
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
        let mut status = Status {
            armed: sup.armed,
            killed: sup.killed,
            mode: sup.mode,
            saturated: false,
            timing_faults: self.timing_faults,
            no_estimate: self.estimate.visible.is_none(),
            estimator: self.estimator_status(),
            ..Status::default()
        };
        let t_valid = now.add_us(base_us);
        if !sup.armed {
            self.rate_ctrl.reset();
            return Outputs {
                t_valid,
                motors: [0.0; N],
                status,
                events: EventList::new(),
            };
        }
        let target = self.attitude.visible;
        let bias = if self.cfg.estimator.source == EstimatorSource::Ekf {
            self.estimator.gyro_bias()
        } else {
            Vec3::zero()
        };
        let gyro = Self::fresh(self.imu, now, STATE_TIMEOUT_US)
            .map(|_| self.gyro_filtered - bias)
            .or_else(|| self.estimate.visible.map(|e| e.rate))
            .unwrap_or_else(Vec3::zero);
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
        status.saturated = alloc.saturated;

        // Battery sag lowers motor speed for a given command; compensate.
        let comp = Self::fresh(self.battery, now, BATTERY_TIMEOUT_US)
            .map(|b| b.voltage_v)
            .filter(|v| v.is_finite() && *v > 0.5 * self.cfg.nominal_voltage_v)
            .map_or(1.0, |v| {
                let r = self.cfg.nominal_voltage_v / v;
                r * r
            });
        let idle = self.cfg.limits.idle_command;
        let mut motors = [0.0_f32; N];
        for (m, f) in motors.iter_mut().zip(alloc.fraction.iter()) {
            let frac = (*f * comp).clamp(0.0, 1.0);
            *m = command_for_fraction(frac, self.cfg.thrust_expo).max(idle);
        }
        Outputs {
            t_valid,
            motors,
            status,
            events: EventList::new(),
        }
    }
}

fn deadband(v: f32) -> f32 {
    let v = v.finite_or(0.0).clamp(-1.0, 1.0);
    if Real::abs(v) < DEADBAND {
        0.0
    } else if v > 0.0 {
        (v - DEADBAND) / (1.0 - DEADBAND)
    } else {
        (v + DEADBAND) / (1.0 - DEADBAND)
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::{EstimatorConfig, FailsafeParams, Gains, Limits, Rotor, Spin};
    use crate::io::{BaroSample, GpsSample, MagSample};
    use crate::schedule::F405_LEAN;

    fn cfg() -> VehicleConfig<4> {
        let a = 0.08;
        let r = |x: f32, y: f32, spin| Rotor {
            x,
            y,
            spin,
            max_thrust_n: 8.0,
            torque_coeff_m: 0.016,
        };
        VehicleConfig {
            mass_kg: 0.65,
            inertia: Vec3::new(1.6e-3, 1.6e-3, 2.8e-3),
            rotors: [
                r(a, a, Spin::Ccw),
                r(-a, -a, Spin::Ccw),
                r(a, -a, Spin::Cw),
                r(-a, a, Spin::Cw),
            ],
            thrust_expo: 1.0,
            nominal_voltage_v: 22.2,
            gains: Gains::default(),
            limits: Limits::default(),
            estimator: EstimatorConfig::default(),
            failsafe: FailsafeParams::default(),
            battery_cells: 6,
            battery_resistance_ohm: 0.025,
        }
    }

    fn rc(arm: bool, kill: bool) -> RcSample {
        RcSample {
            arm_switch: arm,
            kill_switch: kill,
            mode_switch: 2,
            ..RcSample::default()
        }
    }

    fn inputs(t: Micros, rc_sample: RcSample) -> Inputs {
        Inputs {
            state: Some(Stamped::ideal(t, StateSample::default())),
            rc: Some(Stamped::ideal(t, rc_sample)),
            ..Inputs::default()
        }
    }

    /// Runs `ticks` ticks from `start` with a fixed RC frame.
    fn run(core: &mut Core<4>, start: u64, ticks: u64, rc_sample: RcSample) -> Outputs<4> {
        let mut out = Outputs::off(Micros::ZERO);
        for k in start..start + ticks {
            let t = Micros(k * 1000);
            out = core.tick(t, &inputs(t, rc_sample));
        }
        out
    }

    #[test]
    fn outputs_are_valid_one_period_later() {
        let mut core = Core::new(cfg(), F405_LEAN).expect("valid config");
        let out = core.tick(Micros(0), &Inputs::default());
        assert_eq!(out.t_valid, Micros(1000));
        assert_eq!(out.motors, [0.0; 4]);
    }

    #[test]
    fn arms_on_rising_edge_and_kill_latches() {
        let mut core = Core::new(cfg(), F405_LEAN).expect("valid config");
        let out = run(&mut core, 0, 100, rc(false, false));
        assert!(!out.status.armed);
        let out = run(&mut core, 100, 100, rc(true, false));
        assert!(out.status.armed);
        assert!(out.motors.iter().all(|m| *m >= 0.05));
        // Kill needs three consecutive supervisor runs (50 Hz → 60 ms).
        let out = run(&mut core, 200, 20, rc(true, true));
        assert!(out.status.armed, "one frame of kill must not disarm");
        let out = run(&mut core, 220, 100, rc(true, true));
        assert!(!out.status.armed && out.status.killed);
        assert_eq!(out.motors, [0.0; 4]);
        // Releasing kill alone does not re-arm or clear the latch.
        let out = run(&mut core, 320, 100, rc(true, false));
        assert!(!out.status.armed && out.status.killed);
        // Arm switch off clears the latch; a new rising edge re-arms.
        let out = run(&mut core, 420, 100, rc(false, false));
        assert!(!out.status.killed);
        let out = run(&mut core, 520, 100, rc(true, false));
        assert!(out.status.armed);
    }

    #[test]
    fn invalid_rc_frames_are_ignored() {
        let mut core = Core::new(cfg(), F405_LEAN).expect("valid config");
        run(&mut core, 0, 100, rc(false, false));
        let bad = RcSample {
            failsafe: true,
            ..rc(true, false)
        };
        let out = run(&mut core, 100, 100, bad);
        assert!(!out.status.armed, "a failsafe frame must not arm");
    }

    #[test]
    fn counts_timing_faults() {
        let mut core = Core::new(cfg(), F405_LEAN).expect("valid config");
        core.tick(Micros(0), &Inputs::default());
        let out = core.tick(Micros(2500), &Inputs::default());
        assert_eq!(out.status.timing_faults, 1);
    }

    fn ekf_cfg() -> VehicleConfig<4> {
        let mut c = cfg();
        c.estimator.source = EstimatorSource::Ekf;
        c
    }

    /// Stationary level vehicle on the ground: IMU at 1 kHz, GPS at 5 Hz with
    /// 150 ms latency, baro and mag at 50 Hz.
    fn ekf_inputs(k: u64, rc_sample: RcSample) -> Inputs {
        let t = Micros(k * 1000);
        let mut i = Inputs {
            imu: Some(Stamped::ideal(
                t,
                ImuSample {
                    gyro: Vec3::zero(),
                    accel: Vec3::new(0.0, 0.0, -crate::GRAVITY),
                },
            )),
            rc: Some(Stamped::ideal(t, rc_sample)),
            ..Inputs::default()
        };
        if k.is_multiple_of(20) {
            i.baro = Some(Stamped::ideal(t, BaroSample { altitude_m: 40.0 }));
            i.mag = Some(Stamped::ideal(
                t,
                MagSample {
                    field_ut: Vec3::new(25.0, 0.0, 43.3),
                },
            ));
        }
        // The fix measured 150 ms ago arrives now.
        if k >= 150 && (k - 150).is_multiple_of(200) {
            let ts = Micros((k - 150) * 1000);
            i.gps = Some(Stamped {
                t_sample: ts,
                t_arrival: t,
                value: GpsSample {
                    h_acc_m: 0.6,
                    v_acc_m: 1.0,
                    s_acc_mps: 0.2,
                    fix_3d: true,
                    ..GpsSample::default()
                },
            });
        }
        i
    }

    #[test]
    fn ekf_mode_aligns_estimates_and_arms_only_with_an_estimate() {
        let mut core = Core::new(ekf_cfg(), F405_LEAN).expect("valid config");
        let mut first_estimate = None;
        let mut armed_before = false;
        // Arm switch is high from the start (no rising edge to use later).
        for k in 0..3000u64 {
            let out = core.tick(Micros(k * 1000), &ekf_inputs(k, rc(true, false)));
            if core.estimate().is_none() {
                armed_before |= out.status.armed;
                assert!(out.status.no_estimate);
                assert!(!out.status.estimator.aligned || k > 500);
            } else if first_estimate.is_none() {
                first_estimate = Some(k);
            }
        }
        assert!(!armed_before, "must not arm without an estimate");
        let first = first_estimate.expect("estimate by 3 s");
        assert!(first > 900, "no estimate before alignment (k = {first})");
        let e = core.estimate().expect("estimate at 3 s");
        assert!(e.position.norm() < 0.1, "p = {:?}", e.position);
        assert!(
            e.attitude.to_rotation_vector().norm() < 1.0_f32.to_radians(),
            "attitude = {:?}",
            e.attitude
        );
        // The switch was already high when the estimate appeared: no arm.
        let out = core.tick(Micros(3_000_000), &ekf_inputs(3000, rc(true, false)));
        assert!(!out.status.armed);
        assert!(out.status.estimator.aligned && out.status.estimator.gps_aiding);
        // A fresh rising edge arms.
        let mut armed = false;
        for k in 3001..3200u64 {
            let out = core.tick(Micros(k * 1000), &ekf_inputs(k, rc(false, false)));
            armed |= out.status.armed;
        }
        assert!(!armed);
        for k in 3200..3400u64 {
            let out = core.tick(Micros(k * 1000), &ekf_inputs(k, rc(true, false)));
            armed |= out.status.armed;
        }
        assert!(armed, "arming works once the estimate exists");
    }

    #[test]
    fn ekf_mode_ignores_the_external_state_input() {
        let mut core = Core::new(ekf_cfg(), F405_LEAN).expect("valid config");
        for k in 0..500u64 {
            let t = Micros(k * 1000);
            let mut i = ekf_inputs(k, rc(false, false));
            i.state = Some(Stamped::ideal(t, StateSample::default()));
            core.tick(t, &i);
        }
        assert!(core.estimate().is_none());
    }

    #[test]
    fn external_mode_reports_aligned_when_estimate_is_visible() {
        let mut core = Core::new(cfg(), F405_LEAN).expect("valid config");
        let out = core.tick(Micros(0), &Inputs::default());
        assert!(!out.status.estimator.aligned);
        let out = run(&mut core, 1, 5, rc(false, false));
        assert!(out.status.estimator.aligned);
        assert_eq!(out.status.estimator.resets, 0);
    }

    #[test]
    fn bad_estimator_params_fail_core_construction() {
        let mut c = cfg();
        c.estimator.ekf.gps_gate = f32::NAN;
        assert_eq!(
            Core::new(c, F405_LEAN).err(),
            Some(ConfigError::BadEstimator)
        );
    }

    #[test]
    fn ekf_mode_imu_stall_drops_the_estimate() {
        let mut core = Core::new(ekf_cfg(), F405_LEAN).expect("valid config");
        for k in 0..3000u64 {
            core.tick(Micros(k * 1000), &ekf_inputs(k, rc(false, false)));
        }
        // Arm with a fresh rising edge.
        let mut out = Outputs::off(Micros::ZERO);
        for k in 3000..3300u64 {
            out = core.tick(Micros(k * 1000), &ekf_inputs(k, rc(true, false)));
        }
        assert!(out.status.armed && !out.status.no_estimate);
        // The IMU stops; everything else continues.
        let mut flagged = None;
        for k in 3300..3600u64 {
            let mut i = ekf_inputs(k, rc(true, false));
            i.imu = None;
            out = core.tick(Micros(k * 1000), &i);
            if out.status.no_estimate && flagged.is_none() {
                flagged = Some(k - 3300);
            }
        }
        let after = flagged.expect("no_estimate must be set");
        assert!(after <= 150, "took {after} ms");
        assert!(core.estimate().is_none());
    }

    #[test]
    fn ekf_mode_arming_needs_gps_aiding() {
        let mut core = Core::new(ekf_cfg(), F405_LEAN).expect("valid config");
        let step = |core: &mut Core<4>, k: u64, arm: bool, gps: bool| {
            let mut i = ekf_inputs(k, rc(arm, false));
            if !gps {
                i.gps = None;
            }
            core.tick(Micros(k * 1000), &i)
        };
        for k in 0..3000u64 {
            step(&mut core, k, false, true);
        }
        let out = step(&mut core, 3000, false, true);
        assert!(out.status.estimator.gps_aiding);
        // GPS lost for longer than the timeout (5 s) plus a second.
        for k in 3001..9000u64 {
            step(&mut core, k, false, false);
        }
        let mut armed = false;
        for k in 9000..9150u64 {
            let out = step(&mut core, k, true, false);
            assert!(
                core.estimate().is_some(),
                "dead reckoning keeps the estimate"
            );
            assert!(!out.status.estimator.gps_aiding);
            armed |= out.status.armed;
        }
        assert!(!armed, "must not arm without GPS aiding");
        // GPS resumes; once fused, a new rising edge arms.
        for k in 9150..9800u64 {
            step(&mut core, k, false, true);
        }
        let out = step(&mut core, 9800, false, true);
        assert!(out.status.estimator.gps_aiding);
        let mut armed = false;
        for k in 9801..10000u64 {
            armed |= step(&mut core, k, true, true).status.armed;
        }
        assert!(armed, "arms again once GPS aids");
    }

    #[test]
    fn a_future_stamped_sample_is_not_fresh() {
        let mut core = Core::new(cfg(), F405_LEAN).expect("valid config");
        run(&mut core, 0, 100, rc(false, false));
        let mut armed = false;
        for k in 100..200u64 {
            let t = Micros(k * 1000);
            let ahead = t.add_us(10_000_000);
            let mut i = inputs(t, RcSample::default());
            i.rc = Some(Stamped::ideal(ahead, rc(true, false)));
            armed |= core.tick(t, &i).status.armed;
        }
        assert!(!armed, "a frame from the future must be ignored");
        let out = run(&mut core, 200, 100, rc(true, false));
        assert!(out.status.armed, "normal frames afterwards still work");
    }
}
