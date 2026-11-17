// SPDX-License-Identifier: AGPL-3.0-or-later
//! The seam between the simulator and whatever flies the vehicle (DESIGN §5,
//! §8): the [`Autopilot`] trait, and [`CoreAutopilot`], our flight core behind
//! it.
//!
//! Per tick the simulator builds an [`AutopilotInput`] (the physics truth, our
//! sensor models' frame after sensor faults, the pilot's radio and setpoint
//! after radio and brain faults, the heartbeats, which fault kinds are
//! active), calls [`Autopilot::tick`], and flies the motor commands it returns
//! from `t_valid` on (logical execution time, DESIGN §4.2). Our own core
//! reads what it always read, and nothing else: [`CoreAutopilot`] turns the
//! input back into `oxide_core::io::Inputs` exactly as the loop did before the
//! seam existed.
//!
//! World frame NED, body frame FRD, SI units, time in µs.

use core::convert::Infallible;

use oxide_core::Core;
use oxide_core::config::EstimatorSource;
use oxide_core::io::{
    BrainHeartbeat, GcsHeartbeat, Inputs, ParamSet, RcSample, Stamped, TrajectorySetpoint,
};
use oxide_core::math::Vec3;
use oxide_core::params::LoadReport;
use oxide_core::time::Micros;

use crate::faults::ActiveFaults;
use crate::flight_log::{FlightLog, Level, LogBuild, LogError, LogWriter, Stats};
use crate::sensors::{self, SensorFrame};
use crate::sim::CoreRecord;
use crate::vehicle::BodyState;

/// Whatever flies the vehicle: our flight core, or another autopilot behind a
/// bridge.
pub trait Autopilot<const N: usize> {
    /// Why a tick can fail (a bridge loses its process; our core cannot fail).
    type Error: core::fmt::Debug + core::fmt::Display;
    /// The tick period, which is also the physics step, in microseconds.
    fn period_us(&self) -> u64;
    /// One tick at `t`: sees what the simulator knows at `t`, returns the motor
    /// commands and the time from which they are valid.
    fn tick(
        &mut self,
        t: Micros,
        input: &AutopilotInput<'_, N>,
    ) -> Result<AutopilotOutput<N>, Self::Error>;
}

/// What the simulator knows at one tick, `t`: everything the loop has at the
/// point where it calls the autopilot.
#[derive(Clone, Copy, Debug)]
pub struct AutopilotInput<'a, const N: usize> {
    /// The physics truth at `t`.
    pub body: &'a BodyState<N>,
    /// Specific force in the world frame, m/s² (what the physics step before
    /// this tick left: the acceleration without gravity, contact included).
    pub specific_force_world: Vec3<f64>,
    /// Battery voltage and current as the physics has them, V and A.
    pub battery_v: f64,
    pub battery_a: f64,
    /// The wind (air-mass velocity, NED, m/s) acting during this tick.
    pub wind: Vec3<f64>,
    /// Our sensor models' output at `t`, after sensor faults (a lost sensor
    /// is `None`). Another autopilot with its own sensors may ignore it.
    pub sensors: &'a SensorFrame,
    /// The receiver's frame after radio faults.
    pub rc: Option<Stamped<RcSample>>,
    /// The brain's setpoint after brain faults.
    pub setpoint: Option<TrajectorySetpoint>,
    /// The heartbeats due at `t`, after link faults.
    pub gcs_heartbeat: Option<Stamped<GcsHeartbeat>>,
    pub brain_heartbeat: Option<Stamped<BrainHeartbeat>>,
    /// A parameter change and an acknowledgement for this tick (only our core
    /// has parameters of this kind).
    pub param_set: Option<ParamSet>,
    pub param_ack: bool,
    /// Which fault kinds are active at `t`.
    pub faults: ActiveFaults,
}

/// What an autopilot returns from one tick.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AutopilotOutput<const N: usize> {
    /// The commands act from this time (LET: the tick's own time plus the
    /// period, for our core).
    pub t_valid: Micros,
    /// Normalised motor commands, 0 to 1.
    pub motors: [f32; N],
    /// Whether the autopilot reports itself armed after this tick.
    pub armed: bool,
    /// What only our flight core has; `None` for another autopilot.
    pub core: Option<CoreRecord>,
}

/// Our flight core as the simulator's autopilot: owns the `Core`, whether the
/// estimator is replaced by the truth, the stored-parameter load report and
/// the flight log. Its tick assembles `Inputs`, runs `Core::tick`, and records
/// the log, and nothing else.
pub struct CoreAutopilot<const N: usize> {
    core: Core<N>,
    /// The estimator source is `External`: the truth goes in as `state`.
    truth_state: bool,
    load_report: LoadReport,
    log: Option<FlightLog>,
    last_inputs: Inputs,
}

impl<const N: usize> CoreAutopilot<N> {
    /// `core` has had `report` handed to `Core::report_param_load`;
    /// `source` is the estimator source of the configuration it was built with.
    pub fn new(core: Core<N>, source: EstimatorSource, report: LoadReport) -> Self {
        Self {
            core,
            truth_state: source == EstimatorSource::External,
            load_report: report,
            log: None,
            last_inputs: Inputs::default(),
        }
    }

    pub fn core(&self) -> &Core<N> {
        &self.core
    }
    pub fn core_mut(&mut self) -> &mut Core<N> {
        &mut self.core
    }
    /// The report of the stored-parameter load that the boot step returned.
    pub fn load_report(&self) -> LoadReport {
        self.load_report
    }
    /// What the last tick fed the core (after faults).
    pub fn last_inputs(&self) -> &Inputs {
        &self.last_inputs
    }
    /// A log is running.
    pub fn has_log(&self) -> bool {
        self.log.is_some()
    }
    /// Starts the log (the checks on when are the simulation's).
    pub fn start_log(
        &mut self,
        level: Level,
        build: &LogBuild,
        writer: LogWriter,
    ) -> Result<(), LogError> {
        let log = FlightLog::start(
            level,
            build,
            self.core.profile(),
            &self.load_report,
            self.core.config(),
            writer,
        )?;
        self.log = Some(log);
        Ok(())
    }
    pub fn log_stats(&self) -> Option<Stats> {
        self.log.as_ref().map(FlightLog::stats)
    }
    pub fn finish_log(&mut self) -> Result<LogWriter, LogError> {
        self.log.take().ok_or(LogError::NotStarted)?.finish()
    }
}

impl<const N: usize> Autopilot<N> for CoreAutopilot<N> {
    type Error = Infallible;

    fn period_us(&self) -> u64 {
        u64::from(self.core.profile().base_period_us)
    }

    fn tick(
        &mut self,
        t: Micros,
        input: &AutopilotInput<'_, N>,
    ) -> Result<AutopilotOutput<N>, Infallible> {
        let f = input.sensors;
        let inputs = Inputs {
            imu: f.imu,
            gps: f.gps,
            baro: f.baro,
            mag: f.mag,
            battery: f.battery,
            state: self.truth_state.then(|| sensors::state(t, input.body)),
            rc: input.rc,
            setpoint: input.setpoint.map(|s| Stamped::ideal(t, s)),
            gcs_heartbeat: input.gcs_heartbeat,
            brain_heartbeat: input.brain_heartbeat,
            param_set: input.param_set,
            param_ack: input.param_ack,
        };
        self.last_inputs = inputs;

        // The log needs to know whether the store has made its "saved" call
        // since the last tick: the flag is read before the tick.
        let unsaved_before = self.core.params_unsaved();
        let out = self.core.tick(t, &inputs);
        if let Some(log) = self.log.as_mut() {
            log.record(
                t,
                unsaved_before,
                &inputs,
                &out,
                &self.core.telemetry(),
                self.core.config(),
            );
        }
        Ok(AutopilotOutput {
            t_valid: out.t_valid,
            motors: out.motors,
            armed: out.status.armed,
            core: Some(CoreRecord {
                status: out.status,
                events: out.events,
                estimate: self.core.estimate(),
            }),
        })
    }
}
