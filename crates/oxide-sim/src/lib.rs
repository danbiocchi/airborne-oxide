// SPDX-License-Identifier: AGPL-3.0-or-later
//! Airborne Oxide simulator (DESIGN §5).
//!
//! Deterministic: single-threaded per run, f64 physics through
//! `oxide_core::math` (libm), no hash-map iteration, no wall-clock in the
//! simulation state (DESIGN §5.2).

pub mod airframe;
pub mod battery;
pub mod hash;
pub mod montecarlo;
pub mod rng;
pub mod scenario;
pub mod sensors;
pub mod sim;
pub mod vehicle;
pub mod wind;

pub use airframe::Airframe;
pub use sim::{SimOptions, Simulation, StepRecord};

use core::fmt;

#[derive(Debug, Clone, PartialEq)]
pub enum SimError {
    /// The airframe file is malformed or out of range.
    Airframe(String),
    /// The flight core rejected its configuration.
    Core(oxide_core::config::ConfigError),
    /// A period is not a whole number of base ticks (µs value given).
    TimeBase(u64),
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
        }
    }
}

impl std::error::Error for SimError {}
