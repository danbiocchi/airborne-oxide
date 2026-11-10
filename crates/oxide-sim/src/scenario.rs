// SPDX-License-Identifier: AGPL-3.0-or-later
//! Pilots: what drives the RC and setpoint inputs. Scripted scenarios for
//! tests and CI, a joystick for pilot-in-the-loop (in `oxide-viewer`).
//! TOML scenario files arrive in M1c.

use oxide_core::io::{RcSample, TrajectorySetpoint};
use oxide_core::math::Vec3;
use oxide_core::time::Micros;

use crate::rng::{Rng, Stream, run_seed};
use crate::sim::SimOptions;
use crate::wind::{Gust, WindConfig};

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
            arm_at_s: 1.5,
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

/// Scenario id of [`GustHold`], the name its run seeds are derived from.
pub const GUST_HOLD_ID: &str = "gust-hold";

/// The M1b acceptance scenario: arm on the ground, climb to 2 m, hold in
/// gusty wind.
#[derive(Clone, Debug, PartialEq)]
pub struct GustHold {
    pub arm_at_s: f64,
    pub takeoff_at_s: f64,
    /// Held position after `takeoff_at_s`, m NED.
    pub hold: Vec3<f32>,
    pub yaw: f32,
    pub duration_s: f64,
}

impl Default for GustHold {
    fn default() -> Self {
        Self {
            arm_at_s: 3.0,
            takeoff_at_s: 4.0,
            hold: Vec3::new(0.0, 0.0, -2.0),
            yaw: 0.0,
            duration_s: 40.0,
        }
    }
}

impl Pilot for GustHold {
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
        let position = if t.as_secs_f64() >= self.takeoff_at_s {
            self.hold
        } else {
            Vec3::zero()
        };
        Some(TrajectorySetpoint {
            position,
            velocity: Vec3::zero(),
            yaw: self.yaw,
        })
    }
}

/// Wind for run seed `seed`, drawn from `Rng::stream(seed, Stream::Scenario)`
/// in a fixed order: steady speed U(0, 4) m/s with a bearing from
/// `horizontal_direction`; turbulence σ_h U(0.2, 0.8) m/s (σ = (σ_h, σ_h,
/// 0.5·σ_h), τ = 2 s); then three gusts, each with start U(15, 35) s,
/// duration U(1, 4) s, horizontal peak U(1, 4) m/s at a random bearing and
/// vertical peak U(−0.5, 0.5) m/s.
pub fn gust_hold_wind(seed: u64) -> WindConfig {
    let mut rng = Rng::stream(seed, Stream::Scenario);
    let speed = rng.range(0.0, 4.0);
    let steady_ned = rng.horizontal_direction() * speed;
    let sigma_h = rng.range(0.2, 0.8);
    let mut gusts = Vec::with_capacity(3);
    for _ in 0..3 {
        let start_s = rng.range(15.0, 35.0);
        let duration_s = rng.range(1.0, 4.0);
        let peak_h = rng.range(1.0, 4.0);
        let dir = rng.horizontal_direction();
        let peak_d = rng.range(-0.5, 0.5);
        gusts.push(Gust {
            start_s,
            duration_s,
            peak_ned: Vec3::new(dir.x * peak_h, dir.y * peak_h, peak_d),
        });
    }
    WindConfig {
        steady_ned,
        turbulence_sigma: Vec3::new(sigma_h, sigma_h, 0.5 * sigma_h),
        turbulence_tau_s: 2.0,
        gusts,
    }
}

/// Everything for run `index` of the gust-hold scenario: the pilot and the
/// options (seed from `run_seed`, realistic sensors, scenario wind, EKF).
pub fn gust_hold(index: u64) -> (GustHold, SimOptions) {
    let seed = run_seed(GUST_HOLD_ID, index);
    let options = SimOptions {
        wind: gust_hold_wind(seed),
        ..SimOptions::realistic(seed)
    };
    (GustHold::default(), options)
}

/// A scripted flight: what the pilot's radio and the brain send over time.
#[derive(Clone, Debug, PartialEq)]
pub struct Script {
    /// The arm switch goes up at this time…
    pub arm_at_s: f64,
    /// …and down at this time, if any.
    pub disarm_at_s: Option<f64>,
    /// The kill switch goes on at this time, if any.
    pub kill_at_s: Option<f64>,
    /// Mode switch position from each time on (sorted; the first entry
    /// should be at 0).
    pub mode: Vec<(f64, u8)>,
    /// Sticks [roll, pitch, yaw, throttle] from each time on (default
    /// centred, throttle 0).
    pub sticks: Vec<(f64, [f32; 4])>,
    /// Tier-2 setpoint from each time on; `None` = the brain sends no
    /// setpoints from then.
    pub setpoints: Vec<(f64, Option<TrajectorySetpoint>)>,
    pub duration_s: f64,
}

/// The value of the last entry whose time is at or before `t_s`.
fn latest<T: Copy>(list: &[(f64, T)], t_s: f64) -> Option<T> {
    list.iter()
        .rev()
        .find(|(from, _)| *from <= t_s)
        .map(|e| e.1)
}

impl Default for Script {
    /// Nothing happens: mode 0, no arming, no setpoints, 10 s.
    fn default() -> Self {
        Self {
            arm_at_s: f64::INFINITY,
            disarm_at_s: None,
            kill_at_s: None,
            mode: vec![(0.0, 0)],
            sticks: Vec::new(),
            setpoints: Vec::new(),
            duration_s: 10.0,
        }
    }
}

impl Pilot for Script {
    fn rc(&mut self, t: Micros) -> Option<RcSample> {
        if !t.0.is_multiple_of(COMMAND_PERIOD_US) {
            return None;
        }
        let ts = t.as_secs_f64();
        let [roll, pitch, yaw, throttle] = latest(&self.sticks, ts).unwrap_or([0.0; 4]);
        Some(RcSample {
            roll,
            pitch,
            yaw,
            throttle,
            arm_switch: ts >= self.arm_at_s && self.disarm_at_s.is_none_or(|d| ts < d),
            kill_switch: self.kill_at_s.is_some_and(|k| ts >= k),
            mode_switch: latest(&self.mode, ts).unwrap_or(0),
            ..RcSample::default()
        })
    }

    fn setpoint(&mut self, t: Micros) -> Option<TrajectorySetpoint> {
        if !t.0.is_multiple_of(COMMAND_PERIOD_US) {
            return None;
        }
        latest(&self.setpoints, t.as_secs_f64()).flatten()
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

#[cfg(test)]
mod tests {
    use super::*;

    fn sp(x: f32) -> TrajectorySetpoint {
        TrajectorySetpoint {
            position: Vec3::new(x, 0.0, 0.0),
            ..TrajectorySetpoint::default()
        }
    }

    #[test]
    fn script_follows_its_timeline() {
        let mut s = Script {
            arm_at_s: 1.0,
            disarm_at_s: Some(3.0),
            kill_at_s: Some(4.0),
            mode: vec![(0.0, 1), (2.0, 2)],
            sticks: vec![(0.5, [0.1, 0.2, 0.3, 0.4])],
            setpoints: vec![(0.0, Some(sp(1.0))), (2.0, None), (3.0, Some(sp(2.0)))],
            duration_s: 5.0,
        };
        let at = |s: &mut Script, t_s: f64| s.rc(Micros((t_s * 1e6) as u64)).expect("frame");
        let r = at(&mut s, 0.0);
        assert!(!r.arm_switch && r.mode_switch == 1 && r.throttle == 0.0);
        let r = at(&mut s, 1.0);
        assert!(r.arm_switch && r.roll == 0.1 && r.throttle == 0.4);
        assert_eq!(at(&mut s, 2.0).mode_switch, 2);
        assert!(!at(&mut s, 3.0).arm_switch);
        assert!(!at(&mut s, 3.98).kill_switch);
        assert!(at(&mut s, 4.0).kill_switch);
        let sp_at = |s: &mut Script, t_s: f64| s.setpoint(Micros((t_s * 1e6) as u64));
        assert_eq!(sp_at(&mut s, 1.0), Some(sp(1.0)));
        assert_eq!(sp_at(&mut s, 2.5), None);
        assert_eq!(sp_at(&mut s, 3.0), Some(sp(2.0)));
        // Off the command grid nothing is sent.
        assert!(s.rc(Micros(1_010_000)).is_none());
        assert!(s.setpoint(Micros(1_010_000)).is_none());
    }
}
