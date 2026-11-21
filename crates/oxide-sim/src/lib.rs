// SPDX-License-Identifier: AGPL-3.0-or-later
//! Airborne Oxide simulator (DESIGN §5).
//!
//! Deterministic: single-threaded per run, f64 physics through
//! `oxide_core::math` (libm), no hash-map iteration, no wall-clock in the
//! simulation state (DESIGN §5.2).

pub mod airframe;
pub mod autopilot;
pub mod battery;
pub mod fault_scenarios;
pub mod faults;
pub mod flight_log;
pub mod hash;
pub mod metrics;
pub mod montecarlo;
pub mod outcome;
pub mod replay;
pub mod rng;
pub mod scenario;
pub mod scenario_file;
pub mod sensors;
pub mod sim;
pub mod vehicle;
pub mod wind;

pub use airframe::Airframe;
pub use autopilot::{Autopilot, AutopilotInput, AutopilotOutput, CoreAutopilot, PrerollInput};
pub use scenario_file::{ScenarioError, ScenarioFile};
pub use sim::{CoreRecord, PrerollError, SimOptions, Simulation, StepRecord};

use core::fmt;

#[derive(Debug, Clone, PartialEq)]
pub enum SimError {
    /// The airframe file is malformed or out of range.
    Airframe(String),
    /// The flight core rejected its configuration.
    Core(oxide_core::config::ConfigError),
    /// A period is not a whole number of base ticks (µs value given).
    TimeBase(u64),
    /// The wind's sampling period is not a whole multiple of the step.
    WindPeriod { period_us: u64, step_us: u64 },
    /// The IMU model cannot run at this step (the step is not a whole
    /// multiple of its internal period) or its filter cannot be built.
    Imu(String),
    /// Fault number `index` of the options can never happen as written.
    Fault { index: usize, reason: String },
    /// A parameter override was refused (ADR-0004).
    Param { name: String, reason: String },
    /// A scenario file is malformed or asks for something that cannot be.
    Scenario(ScenarioError),
}

impl fmt::Display for SimError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SimError::Airframe(m) => write!(f, "airframe: {m}"),
            SimError::Core(e) => write!(f, "flight core configuration: {e:?}"),
            SimError::TimeBase(us) => write!(
                f,
                "periods must be whole multiples of the {us} µs base tick"
            ),
            SimError::WindPeriod { period_us, step_us } => write!(
                f,
                "the wind period of {period_us} µs must be a whole multiple of the {step_us} µs step"
            ),
            SimError::Imu(m) => write!(f, "IMU model: {m}"),
            SimError::Fault { index, reason } => write!(f, "fault {index}: {reason}"),
            SimError::Param { name, reason } => write!(f, "parameter {name}: {reason}"),
            SimError::Scenario(e) => write!(f, "scenario: {e}"),
        }
    }
}

impl std::error::Error for SimError {}

impl From<ScenarioError> for SimError {
    fn from(e: ScenarioError) -> Self {
        SimError::Scenario(e)
    }
}
