// SPDX-License-Identifier: AGPL-3.0-or-later
//! The lockstep loop: the simulator owns the clock and drives an autopilot (by
//! default our flight core, through `Core::tick`, exactly as firmware will:
//! DESIGN §4.2, §5.1) through the [`Autopilot`] seam.
//!
//! Per tick `k` at time `t = k·base`:
//! 1. sample sensors from the true state at `t`;
//! 2. run the autopilot's tick (for our core `Core::tick(t)`); its outputs are
//!    valid from `t + base` (LET);
//! 3. integrate physics over `[t, t + base]` with the command valid at `t`.

use std::collections::VecDeque;

use oxide_core::Core;
use oxide_core::config::{EstimatorSource, FailsafeParams, VehicleConfig};
use oxide_core::io::ParamSet;
use oxide_core::io::{
    BrainHeartbeat, EventList, GcsHeartbeat, Inputs, RcSample, Stamped, StateSample, Status,
    TrajectorySetpoint,
};
use oxide_core::math::{Quat, Vec3};
use oxide_core::params::{self, LoadReport, ParamError, ParamKind, ParamValue};
use oxide_core::schedule::Profile;
use oxide_core::time::Micros;

use crate::SimError;
use crate::airframe::{Airframe, BatterySpec};
use crate::autopilot::{Autopilot, AutopilotInput, CoreAutopilot};
use crate::battery::Battery;
use crate::faults::{
    ActiveFaults, BRAIN_HEARTBEAT_PERIOD_US, Fault, FaultKind, GCS_HEARTBEAT_PERIOD_US, LinkConfig,
    active_at,
};
use crate::flight_log::{Level, LogBuild, LogError, LogWriter, Stats};
use crate::hash::TraceHash;
use crate::rng::{Rng, Stream};
use crate::scenario::Pilot;
use crate::sensors::{SensorConfig, SensorSuite, Truth};
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
    /// Parameter overrides by name (ADR-0004: the scenario's `[params]`, then
    /// the command line's). They follow the airframe's own `[params]` table
    /// and the `estimator`, `failsafe` and `battery` overrides, and are applied
    /// together with the airframe's as one change (`apply_params`): a later
    /// entry for a name wins, and only the final configuration must be valid.
    pub params: Vec<(String, f64)>,
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
            params: Vec::new(),
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
            params: Vec::new(),
        }
    }
}

fn describe(e: ParamError) -> String {
    match e {
        ParamError::Unknown => "no such parameter (try: oxide-sim params list)".into(),
        ParamError::WrongType => "wrong type: this parameter needs a whole number".into(),
        ParamError::NotFinite => "the value is not a finite number".into(),
        ParamError::OutOfRange => "the value is outside the parameter's range".into(),
        ParamError::Invalid(c) => format!("the configuration would break a rule: {c:?}"),
        ParamError::Armed => "cannot be changed while armed".into(),
    }
}

/// The rule a joint failure names, in words.
fn describe_rule(c: oxide_core::config::ConfigError) -> String {
    use oxide_core::config::ConfigError;
    match c {
        ConfigError::BadFailsafe => "a failsafe, battery or landing rule (BAT_CRIT_V must not \
             exceed BAT_LOW_V; LAND_SPEED must be at least twice LAND_TD_SPEED; LIM_MIN_THRUST \
             must stay below LAND_TD_THRUST, which must stay below LAND_BLIND_THR)"
            .into(),
        ConfigError::BadEstimator => {
            "an estimator rule (the delay horizon must fit the profile's buffers, and \
             EKF_OUT_TAU must be at least EKF_HORIZON)"
                .into()
        }
        ConfigError::CannotLift => {
            "the vehicle cannot lift itself (the rotors' total thrust must exceed the weight)"
                .into()
        }
        ConfigError::IdleTooHigh => "LIM_IDLE gives more than half of the hover thrust of a \
             rotor (it is a floor on every motor)"
            .into(),
        ConfigError::DescentLimitTooLow => "LIM_SPEED_DOWN must be at least twice LAND_TD_SPEED, \
             or the landing would be clamped below the speed that counts as a command to move"
            .into(),
        ConfigError::LossOfControlTooTight => "FS_LOC_TILT must be at least 0.175 rad above both \
             LIM_TILT and LIM_TILT_STAB, or the tilt the controller commands would count as a \
             loss of control"
            .into(),
        ConfigError::FilterCutoffTooHigh => {
            "RATE_GYRO_CUTOFF must be zero (off) or below 0.45 times \
             the profile's rate-loop frequency (450 Hz at 1 kHz, 1800 Hz at 4 kHz)"
                .into()
        }
        other => format!("{other:?}"),
    }
}

/// One `(name, value)` as the core's parameter and value. An `i32` parameter
/// needs a whole number (one too large for an `i32` is out of range); an
/// `f32` one takes `value as f32`.
#[allow(clippy::cast_possible_truncation)]
fn to_change(name: &str, value: f64) -> Result<(params::ParamId, ParamValue), ParamError> {
    let id = params::find(name).ok_or(ParamError::Unknown)?;
    let def = params::PARAMS.get(id.index()).ok_or(ParamError::Unknown)?;
    let v = match def.kind {
        ParamKind::F32 => ParamValue::F32(value as f32),
        ParamKind::I32 => {
            if !value.is_finite() || value.fract() != 0.0 {
                return Err(ParamError::WrongType);
            }
            // A whole number, but one an i32 cannot hold.
            if value < f64::from(i32::MIN) || value > f64::from(i32::MAX) {
                return Err(ParamError::OutOfRange);
            }
            ParamValue::I32(value as i32)
        }
    };
    Ok((id, v))
}

/// Applies the parameters `list` to `cfg` as one change through
/// `oxide_core::params::set_all`: each value is checked on its own, a later
/// entry for a name overrides an earlier one, and the rules between fields are
/// checked once, on the result. So any valid combination can be stated in any
/// order. All or nothing: a refused list leaves `cfg` untouched.
///
/// On an error, the index of the entry at fault comes with it. When the values
/// together break a rule, each entry is tried alone on the starting
/// configuration: if exactly one breaks a rule alone, and the list is valid
/// without it, it is blamed (name, value, reason, as for a bad single value);
/// otherwise the index is `None` and the message is the joint one.
pub fn apply_params_at<const N: usize>(
    cfg: &mut VehicleConfig<N>,
    profile: &Profile,
    list: &[(String, f64)],
) -> Result<(), (Option<usize>, SimError)> {
    let fail = |name: &str, value: f64, e: ParamError| SimError::Param {
        name: name.to_string(),
        reason: format!("value {value}: {}", describe(e)),
    };
    let mut changes = Vec::with_capacity(list.len());
    for (i, (name, value)) in list.iter().enumerate() {
        let change = to_change(name, *value).map_err(|e| (Some(i), fail(name, *value, e)))?;
        // On its own (type, finite, range). A rule between fields may need
        // the other entries, so only a failure of this value counts here.
        let mut scratch = *cfg;
        if let Err((Some(_), e)) = params::set_all(&mut scratch, profile, [change]) {
            return Err((Some(i), fail(name, *value, e)));
        }
        changes.push(change);
    }
    let start = *cfg;
    params::set_all(cfg, profile, changes.iter().copied()).map_err(|(_, e)| {
        // Which entries break a rule even alone, on the starting
        // configuration? Exactly one: it is the culprit, reported as a
        // single value is.
        let mut culprits = Vec::new();
        for (i, ((name, value), change)) in list.iter().zip(&changes).enumerate() {
            let mut scratch = start;
            if let Err((_, single)) = params::set_all(&mut scratch, profile, [*change]) {
                culprits.push((i, fail(name, *value, single)));
            }
        }
        // It is the cause only if the rest of the list is fine without it:
        // an entry that fails alone can be made valid by another entry,
        // while a different pair is what breaks the rule.
        if culprits.len() == 1
            && let Some((i, err)) = culprits.pop()
        {
            let rest = changes
                .iter()
                .enumerate()
                .filter(|(k, _)| *k != i)
                .map(|(_, c)| *c);
            let mut scratch = start;
            if params::set_all(&mut scratch, profile, rest).is_ok() {
                return (Some(i), err);
            }
        }
        let rule = match e {
            ParamError::Invalid(c) => describe_rule(c),
            other => describe(other),
        };
        let mut names: Vec<&str> = Vec::new();
        for (name, _) in list {
            if !names.contains(&name.as_str()) {
                names.push(name);
            }
        }
        (
            None,
            SimError::Param {
                name: names.join(", "),
                reason: format!("these values together break a rule: {rule}"),
            },
        )
    })
}

/// [`apply_params_at`] without the index of the entry at fault.
pub fn apply_params<const N: usize>(
    cfg: &mut VehicleConfig<N>,
    profile: &Profile,
    list: &[(String, f64)],
) -> Result<(), SimError> {
    apply_params_at(cfg, profile, list).map_err(|(_, e)| e)
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
    /// Whether the autopilot reports itself armed after this tick (from its
    /// output; for our core, `status.armed`).
    pub armed: bool,
    pub setpoint: Option<TrajectorySetpoint>,
    /// Wind (air-mass velocity, NED, m/s) acting during this tick.
    pub wind: Vec3<f64>,
    /// Physics steps so far that produced a non-finite state (the vehicle
    /// was frozen in place instead). Must be 0 in a healthy run.
    pub physics_faults: u32,
    /// What only our flight core has; `None` for another autopilot.
    pub core: Option<CoreRecord>,
}

/// What only our flight core reports about a tick.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CoreRecord {
    pub status: Status,
    /// Events the core reported with this tick's outputs.
    pub events: EventList,
    /// The core's published state estimate after this tick.
    pub estimate: Option<StateSample>,
}

impl<const N: usize> StepRecord<N> {
    /// The core's status, if the record has the core part.
    pub fn status(&self) -> Option<&Status> {
        self.core.as_ref().map(|c| &c.status)
    }
    /// The core's published estimate (`None` without the core part, too).
    pub fn estimate(&self) -> Option<StateSample> {
        self.core.as_ref().and_then(|c| c.estimate)
    }
}

/// A command the autopilot produced, waiting for its `t_valid`.
#[derive(Clone, Copy, Debug)]
struct PendingCommand<const N: usize> {
    t_valid: Micros,
    motors: [f32; N],
}

/// The simulator: physics, sensors, wind, faults and a pilot around an
/// [`Autopilot`] (our flight core by default).
pub struct Simulation<P: Pilot, const N: usize, A: Autopilot<N> = CoreAutopilot<N>> {
    autopilot: A,
    model: VehicleModel<N>,
    body: BodyState<N>,
    battery: Battery,
    pilot: P,
    tick: u64,
    base_us: u64,
    dt: f64,
    sensors: SensorSuite<N>,
    wind: Wind,
    /// The motor commands acting over the current step.
    active: [f32; N],
    /// Commands that have not fallen due yet, oldest issued first (one
    /// entry for our core, whose commands are valid one tick later).
    pending: VecDeque<PendingCommand<N>>,
    specific_force_world: Vec3<f64>,
    hash: TraceHash,
    physics_faults: u32,
    faults: Vec<Fault>,
    links: LinkConfig,
    last_rc: Option<RcSample>,
    brain_progress: u32,
    /// A parameter change for the next tick's inputs.
    next_param_set: Option<ParamSet>,
    /// An acknowledgement of a parameter fault for the next tick's inputs.
    next_param_ack: bool,
}

impl<P: Pilot, const N: usize> Simulation<P, N, CoreAutopilot<N>> {
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
        Self::with_boot(airframe, profile, pilot, options, |_| LoadReport::default())
    }

    /// Like [`Simulation::with_options`], with a boot step: `boot` receives the
    /// configuration after the airframe's and options' parameters were
    /// applied, and before the core is built, and may change it (this is where
    /// firmware applies the values it loaded from flash, `ParamStore::boot`).
    /// The [`LoadReport`] it returns is handed to `Core::report_param_load`
    /// before the first tick. A boot step that changes nothing and returns a
    /// clean report gives exactly the simulation `with_options` builds.
    pub fn with_boot(
        airframe: &Airframe,
        profile: Profile,
        pilot: P,
        options: SimOptions,
        boot: impl FnOnce(&mut VehicleConfig<N>) -> LoadReport,
    ) -> Result<Self, SimError> {
        let mut airframe = airframe.clone();
        if let Some(b) = options.battery.clone() {
            airframe.battery = b;
        }
        let airframe = &airframe;
        let model = VehicleModel::<N>::from_airframe(airframe)?;
        let mut cfg = airframe.core_config::<N>()?;
        cfg.estimator.source = options.estimator;
        if let Some(f) = options.failsafe {
            cfg.failsafe = f;
        }
        // The airframe's `[params]`, then the options' (the scenario's, then
        // the command line's), as one change: later entries override earlier
        // ones and only the final configuration has to be valid.
        let from_airframe = airframe.params.len();
        let mut list = airframe.params.clone();
        list.extend_from_slice(&options.params);
        apply_params_at(&mut cfg, &profile, &list).map_err(|(at, e)| match (at, e) {
            (Some(i), SimError::Param { name, reason }) if i < from_airframe => SimError::Param {
                name,
                reason: format!("{reason} (set in the airframe file {})", airframe.name),
            },
            (_, other) => other,
        })?;
        let report = boot(&mut cfg);
        // A parameter (or a stored value) may have changed the source: the
        // state input follows it.
        let source = cfg.estimator.source;
        let mut core = Core::new(cfg, profile).map_err(SimError::Core)?;
        core.report_param_load(report);
        let autopilot = CoreAutopilot::new(core, source, report);
        Self::from_parts(airframe, model, autopilot, pilot, options)
    }

    pub fn core(&self) -> &Core<N> {
        self.autopilot.core()
    }
    /// The core, between ticks (for the parameter store's `service`, which
    /// tells it that values are saved).
    pub fn core_mut(&mut self) -> &mut Core<N> {
        self.autopilot.core_mut()
    }
    /// The report of the stored-parameter load that the boot step returned
    /// (the value given to `Core::report_param_load` before the first tick).
    pub fn load_report(&self) -> LoadReport {
        self.autopilot.load_report()
    }

    /// Starts a log: writes its header to `writer` and, from then on, hands
    /// every tick to the recorder and drains the ring to the writer. Only
    /// before the first tick, and once. At the replay level the boot report
    /// and the profile are recorded too, and a configuration that could never
    /// replay is refused ([`LogError::NotReplayable`]). A writer that fails later never changes the flight;
    /// [`finish_log`](Self::finish_log) returns the first error.
    pub fn start_log(
        &mut self,
        level: Level,
        build: &LogBuild,
        writer: LogWriter,
    ) -> Result<(), LogError> {
        if self.autopilot.has_log() {
            return Err(LogError::AlreadyStarted);
        }
        if self.tick != 0 {
            return Err(LogError::AfterFirstTick);
        }
        self.autopilot.start_log(level, build, writer)
    }

    /// The logger's counters, or `None` when there is no log.
    pub fn log_stats(&self) -> Option<Stats> {
        self.autopilot.log_stats()
    }

    /// Flushes the log and returns its writer, or the first I/O error. The
    /// simulation has no log afterwards.
    pub fn finish_log(&mut self) -> Result<LogWriter, LogError> {
        self.autopilot.finish_log()
    }

    /// What the last `step()` fed the core (after faults).
    pub fn last_inputs(&self) -> &Inputs {
        self.autopilot.last_inputs()
    }

    /// Advances one base tick.
    pub fn step(&mut self) -> StepRecord<N> {
        match self.try_step() {
            Ok(r) => r,
            Err(never) => match never {},
        }
    }

    /// Runs until `t_end`, calling `on_step` for every tick.
    pub fn run_until(&mut self, t_end: Micros, mut on_step: impl FnMut(&StepRecord<N>)) {
        while self.time() < t_end {
            let r = self.step();
            on_step(&r);
        }
    }
}

impl<P: Pilot, const N: usize, A: Autopilot<N>> Simulation<P, N, A> {
    /// A simulation around another autopilot. `options` is [`SimOptions`]
    /// as for our core, and what it says about the world applies: `seed`,
    /// `sensors` (our sensor models run and their frame goes to the
    /// autopilot, which may ignore it), `wind`, `faults`, `links`, `battery`.
    /// What configures our flight core is ignored: `estimator`, `failsafe`,
    /// `params`. The tick period, which is also the physics step, is the
    /// autopilot's [`period_us`](Autopilot::period_us): it must divide the
    /// command period, and the sensors' and links' periods must be whole
    /// multiples of it, else [`SimError::TimeBase`], as for a profile.
    pub fn with_autopilot(
        airframe: &Airframe,
        autopilot: A,
        pilot: P,
        options: SimOptions,
    ) -> Result<Self, SimError> {
        let mut airframe = airframe.clone();
        if let Some(b) = options.battery.clone() {
            airframe.battery = b;
        }
        let model = VehicleModel::<N>::from_airframe(&airframe)?;
        Self::from_parts(&airframe, model, autopilot, pilot, options)
    }

    /// The part of construction every autopilot shares: the time base, the
    /// links and faults, the sensors and the wind.
    fn from_parts(
        airframe: &Airframe,
        model: VehicleModel<N>,
        autopilot: A,
        pilot: P,
        options: SimOptions,
    ) -> Result<Self, SimError> {
        let base_us = autopilot.period_us();
        // One time base: every period must be a whole number of ticks.
        if base_us == 0 || !crate::scenario::COMMAND_PERIOD_US.is_multiple_of(base_us) {
            return Err(SimError::TimeBase(base_us));
        }
        options.links.validate(base_us)?;
        crate::faults::validate_faults(&options.faults, N, &options.links)?;
        let sensors = SensorSuite::new(options.sensors, options.seed, base_us)?;
        let wind = Wind::new(options.wind, Rng::stream(options.seed, Stream::Wind));
        Ok(Self {
            autopilot,
            model,
            body: BodyState::default(),
            battery: Battery::new(&airframe.battery),
            pilot,
            tick: 0,
            base_us,
            dt: Micros(base_us).as_secs_f64(),
            sensors,
            wind,
            active: [0.0; N],
            pending: VecDeque::new(),
            specific_force_world: Vec3::new(0.0, 0.0, -GRAVITY),
            hash: TraceHash::default(),
            physics_faults: 0,
            faults: options.faults,
            links: options.links,
            last_rc: None,
            brain_progress: 0,
            next_param_set: None,
            next_param_ack: false,
        })
    }

    /// Puts a parameter change into the next tick's inputs (at most one per
    /// tick: a later call before the tick replaces it). Only an autopilot with
    /// parameters of our kind (our core) does anything with it; it stays
    /// queued if the tick fails.
    pub fn queue_param_set(&mut self, set: ParamSet) {
        self.next_param_set = Some(set);
    }
    /// Puts an operator's acknowledgement of a parameter fault into the next
    /// tick's inputs.
    pub fn queue_param_ack(&mut self) {
        self.next_param_ack = true;
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
    pub fn autopilot(&self) -> &A {
        &self.autopilot
    }
    pub fn autopilot_mut(&mut self) -> &mut A {
        &mut self.autopilot
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
    /// Hash of every state so far (DESIGN §5.2 item 6).
    pub fn trace_hash(&self) -> u64 {
        self.hash.value()
    }

    /// Advances one tick. If the autopilot's tick fails, its error is
    /// returned and the tick did not happen as far as the simulation's
    /// state goes: the tick counter, the trace hash and the vehicle's truth
    /// are as they were, and a queued parameter change is still queued. The
    /// world's random streams, the pilot and the heartbeat counter did
    /// advance for that tick, so a caller stops (or abandons the run) rather
    /// than retrying it.
    pub fn try_step(&mut self) -> Result<StepRecord<N>, A::Error> {
        let t = self.time();

        // 1. Wind and sensors at t.
        let wind = self.wind.sample(t);
        let mut frame = self.sensors.sample(
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
        let active = ActiveFaults::at(&self.faults, t);
        let setpoint = if active.brain_crash || active.brain_hang {
            None
        } else {
            pilot_setpoint
        };
        let rc = self.rc_frame(t);
        let (gcs_heartbeat, brain_heartbeat) = self.heartbeats(t);
        self.apply_motor_faults(t);
        // Sensor faults: a lost sensor delivers nothing.
        if active.imu_loss {
            frame.imu = None;
        }
        if active.gps_loss {
            frame.gps = None;
        }
        if active.baro_loss {
            frame.baro = None;
        }
        if active.mag_loss {
            frame.mag = None;
        }
        let param_set = self.next_param_set.take();
        let param_ack = core::mem::take(&mut self.next_param_ack);

        // 2. The autopilot's tick.
        let input = AutopilotInput {
            body: &self.body,
            specific_force_world: self.specific_force_world,
            battery_v: self.battery.voltage_v,
            battery_a: self.battery.current_a,
            wind,
            sensors: &frame,
            rc,
            setpoint,
            gcs_heartbeat,
            brain_heartbeat,
            param_set,
            param_ack,
            faults: active,
            rotor_scale: self.model.rotor_scale,
        };
        let out = match self.autopilot.tick(t, &input) {
            Ok(out) => out,
            Err(e) => {
                self.next_param_set = param_set;
                self.next_param_ack = param_ack;
                return Err(e);
            }
        };

        // 3. Physics over [t, t + base] with the commands valid at t: this
        // tick's command joins the queue, and every command that is due
        // (`t_valid <= t`, this tick's own included) takes effect, the one
        // issued last winning; those not due yet wait.
        self.pending.push_back(PendingCommand {
            t_valid: out.t_valid,
            motors: out.motors,
        });
        for _ in 0..self.pending.len() {
            if let Some(p) = self.pending.pop_front() {
                if p.t_valid <= t {
                    self.active = p.motors;
                } else {
                    self.pending.push_back(p);
                }
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
            armed: out.armed,
            setpoint,
            wind,
            physics_faults: self.physics_faults,
            core: out.core,
        };
        self.integrate(wind);
        self.hash_state();
        self.tick = self.tick.saturating_add(1);
        Ok(record)
    }

    /// Runs until `t_end`, calling `on_step` for every tick; stops at the
    /// first error from the autopilot.
    pub fn try_run_until(
        &mut self,
        t_end: Micros,
        mut on_step: impl FnMut(&StepRecord<N>),
    ) -> Result<(), A::Error> {
        while self.time() < t_end {
            let r = self.try_step()?;
            on_step(&r);
        }
        Ok(())
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
            let u = self.active.get(i).copied().map_or(0.0, f64::from);
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
        for m in self.active {
            h.write_f32(m);
        }
        h.write_f64(self.battery.voltage_v);
    }
}
