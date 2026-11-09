// SPDX-License-Identifier: AGPL-3.0-or-later
//! Pilots: what drives the RC and setpoint inputs. Scripted scenarios for
//! tests and CI, a joystick for pilot-in-the-loop (in `oxide-viewer`).
//! TOML scenario files arrive in M1c.

use oxide_core::io::{RcSample, TrajectorySetpoint};
use oxide_core::math::Vec3;
use oxide_core::time::Micros;

/// Source of pilot and tier-2 inputs.
pub trait Pilot {
    /// RC frame sent at `t`, if any.
    fn rc(&mut self, t: Micros) -> Option<RcSample>;
    /// Trajectory setpoint sent at `t`, if any.
    fn setpoint(&mut self, t: Micros) -> Option<TrajectorySetpoint>;
}

/// RC frame and setpoint period, µs (50 Hz).
pub const COMMAND_PERIOD_US: u64 = 20_000;

/// A point the scripted vehicle should reach.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Waypoint {
    /// When this waypoint becomes the setpoint, s.
    pub from_s: f64,
    pub position: Vec3<f32>,
    pub yaw: f32,
}

/// Arms on the ground, takes off to 2 m, then steps in x, yaw and xyz.
#[derive(Clone, Debug, PartialEq)]
pub struct HoverStep {
    pub waypoints: Vec<Waypoint>,
    pub arm_at_s: f64,
    pub duration_s: f64,
}

impl Default for HoverStep {
    fn default() -> Self {
        let wp = |from_s, x, y, z, yaw| Waypoint {
            from_s,
            position: Vec3::new(x, y, z),
            yaw,
        };
        Self {
            waypoints: vec![
                wp(0.0, 0.0, 0.0, 0.0, 0.0),
                wp(1.0, 0.0, 0.0, -2.0, 0.0),
                wp(6.0, 1.0, 0.0, -2.0, 0.0),
                wp(11.0, 1.0, 0.0, -2.0, core::f32::consts::FRAC_PI_2),
                wp(15.0, 1.0, 1.0, -3.0, core::f32::consts::FRAC_PI_2),
            ],
            arm_at_s: 0.5,
            duration_s: 20.0,
        }
    }
}

impl HoverStep {
    /// The waypoint active at time `t_s`.
    pub fn active(&self, t_s: f64) -> Option<&Waypoint> {
        self.waypoints.iter().rev().find(|w| w.from_s <= t_s)
    }
}

impl Pilot for HoverStep {
    fn rc(&mut self, t: Micros) -> Option<RcSample> {
        if !t.0.is_multiple_of(COMMAND_PERIOD_US) {
            return None;
        }
        Some(RcSample {
            arm_switch: t.as_secs_f64() >= self.arm_at_s,
            mode_switch: 2,
            throttle: 0.0,
            ..RcSample::default()
        })
    }

    fn setpoint(&mut self, t: Micros) -> Option<TrajectorySetpoint> {
        if !t.0.is_multiple_of(COMMAND_PERIOD_US) {
            return None;
        }
        self.active(t.as_secs_f64()).map(|w| TrajectorySetpoint {
            position: w.position,
            velocity: Vec3::zero(),
            yaw: w.yaw,
        })
    }
}

/// Sends nothing: for open-loop and analytic tests.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct NoPilot;

impl Pilot for NoPilot {
    fn rc(&mut self, _t: Micros) -> Option<RcSample> {
        None
    }
    fn setpoint(&mut self, _t: Micros) -> Option<TrajectorySetpoint> {
        None
    }
}
