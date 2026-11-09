// SPDX-License-Identifier: AGPL-3.0-or-later
//! The flight core: one `tick` per base period (DESIGN §4.2, ADR-0001).

use crate::allocation::{Mixer, command_for_fraction};
use crate::config::{ConfigError, VehicleConfig};
use crate::control::{
    AttitudeTarget, PositionController, RateController, RateTarget, attitude_rate_setpoint,
};
use crate::io::{
    BatterySample, EstimatorStatus, FlightMode, ImuSample, Inputs, Outputs, RcSample, Stamped,
    StateSample, Status, TrajectorySetpoint,
};
use crate::math::{Quat, Real, Vec3, wrap_pi};
use crate::schedule::{Profile, TaskId};
use crate::time::Micros;

/// How old a sample may be before it is treated as missing, µs.
const STATE_TIMEOUT_US: u64 = 100_000;
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
        if inputs.imu.is_some() {
            self.imu = inputs.imu;
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
        sample
            .filter(|s| now.since(s.t_arrival) <= timeout_us)
            .map(|s| s.value)
    }

    fn valid_rc(&self, now: Micros) -> Option<RcSample> {
        Self::fresh(self.rc, now, RC_TIMEOUT_US).filter(RcSample::is_valid)
    }

    fn run_estimator(&mut self, now: Micros) {
        // M1a: the estimate is the external state (simulator ground truth).
        // The EKF (DESIGN §4.7) replaces this in M1b.
        self.estimate.pending = Self::fresh(self.state, now, STATE_TIMEOUT_US).filter(|s| {
            s.position.is_finite()
                && s.velocity.is_finite()
                && s.attitude.is_finite()
                && s.rate.is_finite()
        });
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
            let have_estimate = self.estimate.visible.is_some();
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
            (FlightMode::Stabilized, Some(rc)) => {
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
            (FlightMode::Stabilized | FlightMode::Position, None) => self.position_ctrl.update(
                self.guide_state.position_sp,
                Vec3::zero(),
                self.guide_state.yaw_sp,
                &est,
                dt,
                &self.cfg,
            ),
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
            estimator: EstimatorStatus::default(),
        };
        let t_valid = now.add_us(base_us);
        if !sup.armed {
            self.rate_ctrl.reset();
            return Outputs {
                t_valid,
                motors: [0.0; N],
                status,
            };
        }
        let target = self.attitude.visible;
        let gyro = Self::fresh(self.imu, now, STATE_TIMEOUT_US)
            .map(|i| i.gyro)
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
    use crate::config::{EstimatorConfig, Gains, Limits, Rotor, Spin};
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
}
