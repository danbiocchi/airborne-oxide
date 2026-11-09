// SPDX-License-Identifier: AGPL-3.0-or-later
//! A USB joystick as the RC pilot (DESIGN §5.1, Q20).
//!
//! EdgeTX/OpenTX radios in USB-joystick mode report the first channels as
//! axes, in channel order. We take the joystick's axes sorted by their raw
//! code and read them as channels, AETR order by default:
//!
//! | channel | default use |
//! |---|---|
//! | 1 | roll |
//! | 2 | pitch |
//! | 3 | throttle |
//! | 4 | yaw |
//! | 5 | arm switch (high = armed) |
//! | 6 | mode switch (low / mid / high = stabilized / position / offboard) |
//! | 7 | kill switch (high = kill) |
//!
//! Untested with real hardware yet: the first run prints every axis so the
//! mapping can be checked and fixed. Set `OXIDE_JOYSTICK_ORDER` to a
//! permutation such as `1,0,2,3,4,5,6` to remap without rebuilding.

use std::collections::BTreeMap;
use std::error::Error;

use gilrs::{EventType, Gilrs};
use oxide_core::io::{RcSample, TrajectorySetpoint};
use oxide_core::time::Micros;
use oxide_sim::scenario::{COMMAND_PERIOD_US, Pilot};

pub struct JoystickPilot {
    gilrs: Gilrs,
    /// Raw axis code → latest value in [−1, 1]. BTreeMap: stable order.
    axes: BTreeMap<u32, f32>,
    order: Vec<usize>,
    announced: bool,
}

impl JoystickPilot {
    pub fn new() -> Result<Self, Box<dyn Error>> {
        let gilrs = Gilrs::new().map_err(|e| format!("joystick init failed: {e}"))?;
        let mut found = false;
        for (_, g) in gilrs.gamepads() {
            println!("joystick: {} (connected: {})", g.name(), g.is_connected());
            found = true;
        }
        if !found {
            return Err("no joystick found: plug the radio in and pick USB joystick mode".into());
        }
        let order = std::env::var("OXIDE_JOYSTICK_ORDER")
            .ok()
            .map(|s| s.split(',').filter_map(|p| p.trim().parse().ok()).collect())
            .unwrap_or_else(|| (0..8).collect());
        Ok(Self {
            gilrs,
            axes: BTreeMap::new(),
            order,
            announced: false,
        })
    }

    fn poll(&mut self) {
        while let Some(ev) = self.gilrs.next_event() {
            if let EventType::AxisChanged(_, value, code) = ev.event {
                self.axes.insert(code.into_u32(), value.clamp(-1.0, 1.0));
            }
        }
    }

    /// Channel `n` (0-based) after remapping, in [−1, 1].
    fn channel(&self, n: usize) -> f32 {
        let idx = self.order.get(n).copied().unwrap_or(n);
        self.axes.values().nth(idx).copied().unwrap_or(-1.0)
    }
}

impl Pilot for JoystickPilot {
    fn rc(&mut self, t: Micros) -> Option<RcSample> {
        if !t.0.is_multiple_of(COMMAND_PERIOD_US) {
            return None;
        }
        self.poll();
        if !self.announced && self.axes.len() >= 4 {
            println!(
                "joystick axes (raw code → channel): {:?}",
                self.axes.keys().collect::<Vec<_>>()
            );
            self.announced = true;
        }
        let mode = self.channel(5);
        Some(RcSample {
            roll: self.channel(0),
            pitch: self.channel(1),
            throttle: (self.channel(2) + 1.0) * 0.5,
            yaw: self.channel(3),
            arm_switch: self.channel(4) > 0.5,
            mode_switch: if mode < -0.33 {
                0
            } else if mode > 0.33 {
                2
            } else {
                1
            },
            kill_switch: self.channel(6) > 0.5,
            // A joystick that stops updating is caught by the core's RC timeout.
            failsafe: false,
            frame_lost: false,
        })
    }

    fn setpoint(&mut self, _t: Micros) -> Option<TrajectorySetpoint> {
        None
    }
}
