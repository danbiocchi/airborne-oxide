// SPDX-License-Identifier: AGPL-3.0-or-later
//! The lockstep loop: the simulator owns the clock and drives the flight core
//! through `Core::tick`, exactly as firmware will (DESIGN §4.2, §5.1).
//!
//! Per tick `k` at time `t = k·base`:
//! 1. sample sensors from the true state at `t`;
//! 2. run `Core::tick(t)`; its outputs are valid from `t + base` (LET);
//! 3. integrate physics over `[t, t + base]` with the command valid at `t`.

use oxide_core::Core;
use oxide_core::config::{EstimatorSource, FailsafeParams};
use oxide_core::io::{
    BrainHeartbeat, EventList, GcsHeartbeat, Inputs, Outputs, RcSample, Stamped, StateSample,
    Status, TrajectorySetpoint,
};
use oxide_core::math::{Quat, Vec3};
use oxide_core::schedule::Profile;
use oxide_core::time::Micros;

use crate::SimError;
use crate::airframe::{Airframe, BatterySpec};
use crate::battery::Battery;
use crate::faults::{
    BRAIN_HEARTBEAT_PERIOD_US, Fault, FaultKind, GCS_HEARTBEAT_PERIOD_US, LinkConfig, active_at,
};
use crate::hash::TraceHash;
use crate::rng::{Rng, Stream};
use crate::scenario::Pilot;
use crate::sensors::{self, SensorConfig, SensorSuite, Truth};
use crate::vehicle::{BodyState, GRAVITY, VehicleModel};
use crate::wind::{Wind, WindConfig};

/// Everything about a run that is not the airframe, profile or pilot.
#[derive(Clone, Debug, PartialEq)]
pub struct SimOptions {
    /// Run seed (`rng::run_seed` for scenario runs).
    pub seed: u64,
    pub sensors: SensorConfig,
    pub wind: WindConfig,
    /// Replaces the airframe's estimator source. With `External` the
    /// simulator also sends ground truth as the `state` input; with `Ekf` it
    /// does not.
    pub estimator: EstimatorSource,
    /// Injected faults (none by default).
    pub faults: Vec<Fault>,
    /// Ground-station and brain links (brain only by default).
    pub links: LinkConfig,
    /// Replaces the airframe's failsafe parameters (None = the defaults).
    pub failsafe: Option<FailsafeParams>,
    /// Replaces the airframe's battery (for low-battery scenarios).
    pub battery: Option<BatterySpec>,
}

impl Default for SimOptions {
    /// Seed 0, ideal sensors, calm wind, truth state.
    fn default() -> Self {
        Self {
            seed: 0,
            sensors: SensorConfig::ideal(),
            wind: WindConfig::calm(),
            estimator: EstimatorSource::External,
            faults: Vec::new(),
            links: LinkConfig::default(),
            failsafe: None,
            battery: None,
        }
    }
}

impl SimOptions {
    /// Realistic sensors, calm wind, EKF.
    pub fn realistic(seed: u64) -> Self {
        Self {
            seed,
            sensors: SensorConfig::realistic(),
            wind: WindConfig::calm(),
            estimator: EstimatorSource::Ekf,
            faults: Vec::new(),
            links: LinkConfig::default(),
            failsafe: None,
            battery: None,
        }
    }
}

/// What happened in one tick, for logging, viewers and tests.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StepRecord<const N: usize> {
    /// Time of the tick (state is the state at this time).
    pub t: Micros,
    pub position: Vec3<f64>,
    pub velocity: Vec3<f64>,
    pub attitude: Quat<f64>,
    pub rate: Vec3<f64>,
    /// Motor commands produced at this tick (applied from `t + base`).
    pub motors: [f32; N],
    pub rotor_speed: [f64; N],
    pub battery_v: f64,
    pub battery_a: f64,
    pub status: Status,
    pub setpoint: Option<TrajectorySetpoint>,
    /// The core's published state estimate after this tick.
    pub estimate: Option<StateSample>,
    /// Wind (air-mass velocity, NED, m/s) acting during this tick.
    pub wind: Vec3<f64>,
    /// Physics steps so far that produced a non-finite state (the vehicle
    /// was frozen in place instead). Must be 0 in a healthy run.
    pub physics_faults: u32,
    /// Events the core reported with this tick's outputs.
    pub events: EventList,
}

pub struct Simulation<P: Pilot, const N: usize> {
    core: Core<N>,
    model: VehicleModel<N>,
    body: BodyState<N>,
    battery: Battery,
    pilot: P,
    tick: u64,
    base_us: u64,
    dt: f64,
    sensors: SensorSuite<N>,
    wind: Wind,
    truth_state: bool,
    active: Outputs<N>,
    pending: Option<Outputs<N>>,
    specific_force_world: Vec3<f64>,
    hash: TraceHash,
    physics_faults: u32,
    faults: Vec<Fault>,
    links: LinkConfig,
    last_rc: Option<RcSample>,
    brain_progress: u32,
    last_inputs: Inputs,
}

impl<P: Pilot, const N: usize> Simulation<P, N> {
    /// Ideal sensors, calm wind, truth state (`SimOptions::default()`).
    pub fn new(airframe: &Airframe, profile: Profile, pilot: P) -> Result<Self, SimError> {
        Self::with_options(airframe, profile, pilot, SimOptions::default())
    }

    pub fn with_options(
        airframe: &Airframe,
        profile: Profile,
        pilot: P,
        options: SimOptions,
    ) -> Result<Self, SimError> {
        let mut airframe = airframe.clone();
        if let Some(b) = options.battery {
            airframe.battery = b;
        }
        let airframe = &airframe;
        let model = VehicleModel::<N>::from_airframe(airframe)?;
        let mut cfg = airframe.core_config::<N>()?;
        cfg.estimator.source = options.estimator;
        if let Some(f) = options.failsafe {
            cfg.failsafe = f;
        }
        let core = Core::new(cfg, profile).map_err(SimError::Core)?;
        let base_us = u64::from(profile.base_period_us);
        // One time base: every period must be a whole number of ticks.
        if base_us == 0 || !crate::scenario::COMMAND_PERIOD_US.is_multiple_of(base_us) {
            return Err(SimError::TimeBase(base_us));
        }
        options.links.validate(base_us)?;
        crate::faults::validate_faults(&options.faults, N, &options.links)?;
        let sensors = SensorSuite::new(options.sensors, options.seed, base_us)?;
        let wind = Wind::new(options.wind, Rng::stream(options.seed, Stream::Wind));
        Ok(Self {
            core,
            model,
            body: BodyState::default(),
            battery: Battery::new(&airframe.battery),
            pilot,
            tick: 0,
            base_us,
            dt: Micros(base_us).as_secs_f64(),
            sensors,
            wind,
            truth_state: options.estimator == EstimatorSource::External,
            active: Outputs::off(Micros::ZERO),
            pending: None,
            specific_force_world: Vec3::new(0.0, 0.0, -GRAVITY),
            hash: TraceHash::default(),
            physics_faults: 0,
            faults: options.faults,
            links: options.links,
            last_rc: None,
            brain_progress: 0,
            last_inputs: Inputs::default(),
        })
    }

    pub fn time(&self) -> Micros {
        Micros(self.tick.saturating_mul(self.base_us))
    }
    pub fn body(&self) -> &BodyState<N> {
        &self.body
    }
    pub fn model(&self) -> &VehicleModel<N> {
        &self.model
    }
    pub fn model_mut(&mut self) -> &mut VehicleModel<N> {
        &mut self.model
    }
    pub fn set_body(&mut self, body: BodyState<N>) {
        self.body = body;
    }
    pub fn pilot_mut(&mut self) -> &mut P {
        &mut self.pilot
    }
    pub fn core(&self) -> &Core<N> {
        &self.core
    }
    pub fn sensors(&self) -> &SensorSuite<N> {
        &self.sensors
    }
    pub fn wind(&self) -> &Wind {
        &self.wind
    }
    /// Physics steps so far that produced a non-finite state.
    pub fn physics_faults(&self) -> u32 {
        self.physics_faults
    }
    /// What the last `step()` fed the core (after faults).
    pub fn last_inputs(&self) -> &Inputs {
        &self.last_inputs
    }
    /// Hash of every state so far (DESIGN §5.2 item 6).
    pub fn trace_hash(&self) -> u64 {
        self.hash.value()
    }

    /// Advances one base tick.
    pub fn step(&mut self) -> StepRecord<N> {
        let t = self.time();

        // 1. Wind and sensors at t.
        let wind = self.wind.sample(t);
        let frame = self.sensors.sample(
            t,
            &Truth {
                body: &self.body,
                specific_force_world: self.specific_force_world,
                battery_v: self.battery.voltage_v,
                battery_a: self.battery.current_a,
                max_omega: self.model.max_omega,
            },
        );
        let pilot_setpoint = self.pilot.setpoint(t);
        let setpoint = if self.fault_active(t, |k| {
            matches!(k, FaultKind::BrainCrash | FaultKind::BrainHang)
        }) {
            None
        } else {
            pilot_setpoint
        };
        let rc = self.rc_frame(t);
        let (gcs_heartbeat, brain_heartbeat) = self.heartbeats(t);
        let drop_gps = self.fault_active(t, |k| matches!(k, FaultKind::GpsLoss));
        let drop_imu = self.fault_active(t, |k| matches!(k, FaultKind::ImuLoss));
        let drop_baro = self.fault_active(t, |k| matches!(k, FaultKind::BaroLoss));
        let drop_mag = self.fault_active(t, |k| matches!(k, FaultKind::MagLoss));
        self.apply_motor_faults(t);
        let inputs = Inputs {
            imu: frame.imu.filter(|_| !drop_imu),
            gps: frame.gps.filter(|_| !drop_gps),
            baro: frame.baro.filter(|_| !drop_baro),
            mag: frame.mag.filter(|_| !drop_mag),
            battery: frame.battery,
            state: self.truth_state.then(|| sensors::state(t, &self.body)),
            rc,
            setpoint: setpoint.map(|s| Stamped::ideal(t, s)),
            gcs_heartbeat,
            brain_heartbeat,
        };
        self.last_inputs = inputs;

        // 2. Core tick.
        let out = self.core.tick(t, &inputs);

        // 3. Physics over [t, t + base] with the command valid at t.
        if let Some(p) = self.pending.take() {
            if p.t_valid <= t {
                self.active = p;
            } else {
                self.pending = Some(p);
            }
        }
        let record = StepRecord {
            t,
            position: self.body.position,
            velocity: self.body.velocity,
            attitude: self.body.attitude,
            rate: self.body.rate,
            motors: out.motors,
            rotor_speed: self.body.rotor_speed,
            battery_v: self.battery.voltage_v,
            battery_a: self.battery.current_a,
            status: out.status,
            setpoint,
            estimate: self.core.estimate(),
            wind,
            physics_faults: self.physics_faults,
            events: out.events,
        };
        self.integrate(wind);
        self.pending = Some(out);
        self.hash_state();
        self.tick = self.tick.saturating_add(1);
        record
    }

    fn fault_active(&self, t: Micros, pred: fn(&FaultKind) -> bool) -> bool {
        active_at(&self.faults, t, pred).next().is_some()
    }

    /// The RC frame the receiver delivers at `t`, after RC faults.
    fn rc_frame(&mut self, t: Micros) -> Option<Stamped<RcSample>> {
        // Always ask the pilot, so a stateful pilot advances under faults.
        let sent = self.pilot.rc(t);
        if self.fault_active(t, |k| matches!(k, FaultKind::RcSilent)) {
            return None;
        }
        let start = active_at(&self.faults, t, |k| matches!(k, FaultKind::RcLoss))
            .map(|f| f.start_us())
            .min();
        match (start, sent) {
            (Some(start), Some(_)) => {
                let mut held = self.last_rc.unwrap_or_default();
                held.frame_lost = true;
                held.failsafe = t.0 >= start.saturating_add(self.links.rc_failsafe_delay_us);
                Some(Stamped::ideal(t, held))
            }
            (None, Some(r)) => {
                self.last_rc = Some(r);
                Some(Stamped::ideal(t, r))
            }
            (_, None) => None,
        }
    }

    /// Heartbeats due at `t`, after link faults.
    fn heartbeats(
        &mut self,
        t: Micros,
    ) -> (
        Option<Stamped<GcsHeartbeat>>,
        Option<Stamped<BrainHeartbeat>>,
    ) {
        let due = |p: u64| t.0.checked_rem(p) == Some(0);
        let gcs = (self.links.gcs
            && due(GCS_HEARTBEAT_PERIOD_US)
            && !self.fault_active(t, |k| matches!(k, FaultKind::GcsLoss)))
        .then(|| Stamped::ideal(t, GcsHeartbeat));
        let mut brain = None;
        if self.links.brain
            && due(BRAIN_HEARTBEAT_PERIOD_US)
            && !self.fault_active(t, |k| matches!(k, FaultKind::BrainCrash))
        {
            if !self.fault_active(t, |k| matches!(k, FaultKind::BrainHang)) {
                self.brain_progress = self.brain_progress.wrapping_add(1);
            }
            brain = Some(Stamped::ideal(
                t,
                BrainHeartbeat {
                    progress: self.brain_progress,
                },
            ));
        }
        (gcs, brain)
    }

    /// Sets the model's rotor scales from the active motor faults. With no
    /// faults configured the model is left alone (tests may set it directly).
    fn apply_motor_faults(&mut self, t: Micros) {
        if self.faults.is_empty() {
            return;
        }
        self.model.rotor_scale = [1.0; N];
        for f in active_at(&self.faults, t, |k| {
            matches!(k, FaultKind::MotorLoss { .. })
        }) {
            if let FaultKind::MotorLoss { rotor, scale } = f.kind
                && let Some(s) = self.model.rotor_scale.get_mut(rotor)
            {
                // Overlapping faults on one rotor: the worst applies.
                let scale = if scale.is_finite() {
                    scale.clamp(0.0, 1.0)
                } else {
                    0.0
                };
                *s = s.min(scale);
            }
        }
    }

    fn integrate(&mut self, wind: Vec3<f64>) {
        let v = self.battery.voltage_v;
        let cmd: [f64; N] = core::array::from_fn(|i| {
            let u = self.active.motors.get(i).copied().map_or(0.0, f64::from);
            self.model.speed_command(u, v)
        });
        let v_before = self.body.velocity;
        let mut next = self.model.rk4(&self.body, &cmd, self.dt, wind);
        if !next.is_finite() {
            // Never propagate NaN: freeze, and count it so the run can fail.
            self.physics_faults = self.physics_faults.saturating_add(1);
            next = self.body;
            next.velocity = Vec3::zero();
            next.rate = Vec3::zero();
        }
        let g = if self.model.gravity {
            Vec3::new(0.0, 0.0, GRAVITY)
        } else {
            Vec3::zero()
        };
        self.specific_force_world = (next.velocity - v_before) * (1.0 / self.dt) - g;
        self.body = next;
        let loads = self.model.loads(&self.body, wind);
        let electrical = loads.shaft_power_w / self.model.efficiency + self.model.idle_power_w;
        self.battery.draw(electrical, self.dt);
    }

    fn hash_state(&mut self) {
        let b = &self.body;
        let h = &mut self.hash;
        h.write_u64(self.tick);
        for v in [b.position, b.velocity, b.rate] {
            h.write_f64(v.x);
            h.write_f64(v.y);
            h.write_f64(v.z);
        }
        for q in [b.attitude.w, b.attitude.x, b.attitude.y, b.attitude.z] {
            h.write_f64(q);
        }
        for s in b.rotor_speed {
            h.write_f64(s);
        }
        for m in self.active.motors {
            h.write_f32(m);
        }
        h.write_f64(self.battery.voltage_v);
    }

    /// Runs until `t_end`, calling `on_step` for every tick.
    pub fn run_until(&mut self, t_end: Micros, mut on_step: impl FnMut(&StepRecord<N>)) {
        while self.time() < t_end {
            let r = self.step();
            on_step(&r);
        }
    }
}
