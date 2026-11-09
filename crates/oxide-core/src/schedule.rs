// SPDX-License-Identifier: AGPL-3.0-or-later
//! The core owns its multi-rate schedule (DESIGN §4.2, ADR-0001).
//!
//! Each build profile fixes a base tick and every task's period as a whole
//! number of base ticks. Tasks follow the logical-execution-time rule: a task
//! reads its inputs when it is released and its outputs become visible to
//! other tasks at the end of its period (its next release).

use crate::config::ConfigError;

/// The core's tasks, in the order they run within a tick.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TaskId {
    /// State estimate. M1a passes through an external (simulator) state.
    Estimator,
    /// Arming, kill switch and mode selection.
    Supervisor,
    /// Position/velocity control or stick handling → attitude and thrust target.
    Guidance,
    /// Attitude control → body-rate target.
    Attitude,
    /// Rate control and allocation → motor commands. Runs every tick.
    Rate,
}

/// One task and its period.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TaskSpec {
    pub id: TaskId,
    pub period_ticks: u32,
}

/// A build profile: base tick and task table.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Profile {
    pub name: &'static str,
    pub base_period_us: u32,
    pub tasks: [TaskSpec; 5],
}

const fn spec(id: TaskId, period_ticks: u32) -> TaskSpec {
    TaskSpec { id, period_ticks }
}

/// STM32F405 "lean" profile: 1 kHz base tick.
pub const F405_LEAN: Profile = Profile {
    name: "f405-lean",
    base_period_us: 1000,
    tasks: [
        spec(TaskId::Estimator, 1),
        spec(TaskId::Supervisor, 20), // 50 Hz
        spec(TaskId::Guidance, 20),   // 50 Hz
        spec(TaskId::Attitude, 4),    // 250 Hz
        spec(TaskId::Rate, 1),        // 1 kHz
    ],
};

/// STM32H743 "full" profile: 4 kHz base tick.
pub const H743_FULL: Profile = Profile {
    name: "h743-full",
    base_period_us: 250,
    tasks: [
        spec(TaskId::Estimator, 1),
        spec(TaskId::Supervisor, 80), // 50 Hz
        spec(TaskId::Guidance, 40),   // 100 Hz
        spec(TaskId::Attitude, 8),    // 500 Hz
        spec(TaskId::Rate, 1),        // 4 kHz
    ],
};

impl Profile {
    /// Period of a task in base ticks.
    pub fn period(&self, id: TaskId) -> Option<u32> {
        self.tasks
            .iter()
            .find(|t| t.id == id)
            .map(|t| t.period_ticks)
    }

    /// Base tick in seconds.
    #[allow(clippy::cast_precision_loss)]
    pub fn base_dt(&self) -> f32 {
        self.base_period_us as f32 * 1e-6
    }

    /// Checks the table: every task present exactly once, periods ≥ 1, and
    /// the rate task runs every tick.
    pub fn validate(&self) -> Result<(), ConfigError> {
        let ids = [
            TaskId::Estimator,
            TaskId::Supervisor,
            TaskId::Guidance,
            TaskId::Attitude,
            TaskId::Rate,
        ];
        for id in ids {
            if self.tasks.iter().filter(|t| t.id == id).count() != 1 {
                return Err(ConfigError::BadProfile);
            }
        }
        if self.base_period_us == 0 || self.tasks.iter().any(|t| t.period_ticks == 0) {
            return Err(ConfigError::BadProfile);
        }
        if self.period(TaskId::Rate) != Some(1) {
            return Err(ConfigError::BadProfile);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shipped_profiles_are_valid() {
        assert_eq!(F405_LEAN.validate(), Ok(()));
        assert_eq!(H743_FULL.validate(), Ok(()));
    }

    #[test]
    fn rejects_slow_rate_task() {
        let mut p = F405_LEAN;
        if let Some(t) = p.tasks.iter_mut().find(|t| t.id == TaskId::Rate) {
            t.period_ticks = 2;
        }
        assert_eq!(p.validate(), Err(ConfigError::BadProfile));
    }
}
