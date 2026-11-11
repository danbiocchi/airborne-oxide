// SPDX-License-Identifier: AGPL-3.0-or-later
//! The supervisor task: RC, arming, monitors, failsafes and the navigation
//! state (ADR-0003 §3 to §7).

use super::{
    ARM_MAX_RATE, ARM_MAX_THROTTLE, BATTERY_TIMEOUT_US, Core, KILL_CONFIRMATIONS, Supervision,
};
use crate::failsafe::{Conditions, Failsafe};
use crate::io::{Action, DisarmReason, EventKind, FlightMode, NavState, PrearmFlags, Reasons};
use crate::math::Quat;
use crate::monitor::{BatteryMonitor, ControlMonitor, LandDetector, geofence_breached};
use crate::time::Micros;

/// How many distinct valid frames in a row have shown one switch position.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct SwitchRun {
    /// The position the last examined valid frame showed.
    position: bool,
    /// Consecutive examined valid frames that showed it (saturates at
    /// `KILL_CONFIRMATIONS`).
    count: u8,
}

impl SwitchRun {
    const fn new() -> Self {
        Self {
            position: false,
            count: 0,
        }
    }

    fn observe(&mut self, position: bool) {
        if self.count > 0 && self.position == position {
            self.count = self.count.saturating_add(1).min(KILL_CONFIRMATIONS);
        } else {
            self.position = position;
            self.count = 1;
        }
    }

    /// The switch has shown `position` in `KILL_CONFIRMATIONS` frames in a row.
    const fn confirmed(&self, position: bool) -> bool {
        self.count >= KILL_CONFIRMATIONS && self.position == position
    }
}

/// The supervisor's memory between runs.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(super) struct SupervisorState {
    /// `t_arrival` of the RC frame examined last (each frame counts once).
    last_rc_frame: Option<Micros>,
    kill_run: SwitchRun,
    arm_run: SwitchRun,
    /// The arm-switch position confirmed on distinct frames (starts low).
    arm_confirmed: bool,
    prev_arm_switch: bool,
    /// Since when RC has been valid without a gap.
    pub(super) rc_valid_since: Option<Micros>,
    /// The mode-switch position in effect (updated only while RC is settled).
    switch_in_effect: Option<u8>,
    /// Set by a `Terminated` failsafe; cleared when the arm switch is
    /// released on a valid frame.
    terminated_latch: bool,
    battery: BatteryMonitor,
    land: LandDetector,
    control: ControlMonitor,
    failsafe: Failsafe,
    prev_reasons: Reasons,
    prev_action: Action,
    /// What a degraded pilot mode was flown as at the last run, if it was.
    degraded: Option<NavState>,
    /// Estimator fault count when the vehicle was last disarmed or armed.
    fault_baseline: u32,
    /// Since when a pilot state has been sitting still at low thrust.
    ground_since: Option<Micros>,
    /// The pilot was last seen sitting on the ground (still, low thrust).
    /// Frozen while a failsafe state flies, cleared on disarm.
    ground_hint: bool,
}

impl SupervisorState {
    pub(super) const fn new() -> Self {
        Self {
            last_rc_frame: None,
            kill_run: SwitchRun::new(),
            arm_run: SwitchRun::new(),
            arm_confirmed: false,
            prev_arm_switch: false,
            rc_valid_since: None,
            switch_in_effect: None,
            terminated_latch: false,
            battery: BatteryMonitor::new(),
            land: LandDetector::new(),
            control: ControlMonitor::new(),
            failsafe: Failsafe::new(),
            prev_reasons: Reasons::empty(),
            prev_action: Action::None,
            degraded: None,
            fault_baseline: 0,
            ground_since: None,
            ground_hint: false,
        }
    }
}

/// Thrust below this fraction of hover, with no vertical motion, for
/// `GROUND_HINT_US`, means a pilot state is sitting on the ground.
const GROUND_HINT_THRUST: f32 = 0.5;
const GROUND_HINT_US: u64 = 200_000;

/// A duration in seconds as whole microseconds (negative and NaN give 0).
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
fn secs_to_us(s: f32) -> u64 {
    (f64::from(s) * 1e6 + 0.5) as u64
}

/// The navigation state a pilot mode asks for.
const fn requested_nav(mode: FlightMode) -> NavState {
    match mode {
        FlightMode::Stabilized => NavState::Stabilized,
        FlightMode::Altitude => NavState::Altitude,
        FlightMode::Position => NavState::Position,
        FlightMode::Offboard => NavState::Offboard,
    }
}

impl<const N: usize> Core<N> {
    /// Disarms with `reason` if armed. Home is forgotten.
    fn disarm(&mut self, now: Micros, s: &mut Supervision, reason: DisarmReason) {
        if s.armed {
            s.armed = false;
            s.home = None;
            self.emit(now, EventKind::Disarmed(reason));
        }
    }

    #[allow(clippy::too_many_lines)]
    pub(super) fn run_supervisor(&mut self, now: Micros, dt: f32) {
        let p = self.cfg.failsafe;
        let mut s = self.supervision.pending;
        let est = self.estimate.visible;
        let valid = est.valid;
        let state = est.state.filter(|_| valid.attitude);
        let rc = self.valid_rc(now);

        // 1. RC.
        let rc_valid = rc.is_some();
        if rc_valid {
            self.sup_state.rc_valid_since.get_or_insert(now);
        } else {
            self.sup_state.rc_valid_since = None;
        }
        let rc_settled = self
            .sup_state
            .rc_valid_since
            .is_some_and(|t| now.since(t) >= secs_to_us(p.rc_valid_min_s));

        // 2. Kill and arm switches. A stored frame is examined once, by its
        // arrival time; a position takes effect after `KILL_CONFIRMATIONS`
        // distinct valid frames in a row agree. A flagged frame is no
        // evidence either way; with no fresh frame at all the runs restart.
        let latest = Self::fresh(self.rc, now, u64::from(p.rc_timeout_us));
        match (latest, self.rc.map(|f| f.t_arrival)) {
            (Some(frame), Some(arrival)) => {
                if self.sup_state.last_rc_frame != Some(arrival) {
                    self.sup_state.last_rc_frame = Some(arrival);
                    if frame.is_valid() {
                        self.sup_state.kill_run.observe(frame.kill_switch);
                        self.sup_state.arm_run.observe(frame.arm_switch);
                        if self.sup_state.arm_run.count >= KILL_CONFIRMATIONS {
                            self.sup_state.arm_confirmed = self.sup_state.arm_run.position;
                        }
                    }
                }
            }
            _ => {
                self.sup_state.kill_run = SwitchRun::new();
                self.sup_state.arm_run = SwitchRun::new();
            }
        }
        if let Some(rc) = rc {
            if self.sup_state.kill_run.confirmed(true) {
                s.killed = true;
                self.disarm(now, &mut s, DisarmReason::Kill);
            }
            if s.killed && !rc.kill_switch && !rc.arm_switch {
                s.killed = false;
            }
            if self.sup_state.terminated_latch && !rc.arm_switch {
                self.sup_state.terminated_latch = false;
            }
        }

        // 3. The switch position in effect.
        let mut mode_switch_changed = false;
        if rc_settled
            && let Some(rc) = rc
            && self.sup_state.switch_in_effect != Some(rc.mode_switch)
        {
            mode_switch_changed = self.sup_state.switch_in_effect.is_some();
            self.sup_state.switch_in_effect = Some(rc.mode_switch);
        }
        let pilot_mode = match self.sup_state.switch_in_effect {
            None | Some(0) => FlightMode::Stabilized,
            Some(1) => FlightMode::Position,
            Some(_) => FlightMode::Offboard,
        };
        s.mode = pilot_mode;

        // 4. Monitors.
        let battery_sample = Self::fresh(self.battery, now, BATTERY_TIMEOUT_US);
        self.sup_state.battery.update(
            now,
            battery_sample,
            self.cfg.battery_cells,
            self.cfg.battery_resistance_ohm,
            s.armed,
            &p,
            dt,
        );
        // The estimate's vertical velocity is valid under either flag.
        let vertical_speed = if valid.altitude || valid.velocity {
            state.map(|e| -e.velocity.z)
        } else {
            None
        };
        let landed = self.sup_state.land.update(
            now,
            s.armed,
            self.guidance.visible.thrust_n,
            self.cfg.weight_n(),
            vertical_speed,
            // Where the pilot commands the thrust, low thrust alone is not
            // evidence of ground contact; in a landing state the controller
            // lowers it only on contact or after the blind-landing timeout.
            matches!(s.nav, NavState::Rtl | NavState::Land | NavState::LandNow),
            &p,
        );
        // A pilot state at low thrust and standing still is the vehicle on
        // the ground, even before the touchdown detector has held for
        // `touchdown_s` (just after arming, say).
        if !s.armed {
            self.sup_state.ground_since = None;
            self.sup_state.ground_hint = false;
        } else if matches!(
            s.nav,
            NavState::Stabilized | NavState::Altitude | NavState::Position
        ) {
            let still = vertical_speed.is_some_and(|v| v.abs() < p.touchdown_speed_mps);
            let low = self.guidance.visible.thrust_n < GROUND_HINT_THRUST * self.cfg.weight_n();
            if still && low {
                let since = *self.sup_state.ground_since.get_or_insert(now);
                self.sup_state.ground_hint = now.since(since) >= GROUND_HINT_US;
            } else {
                self.sup_state.ground_since = None;
                self.sup_state.ground_hint = false;
            }
        }
        let loss_of_control = match state {
            Some(e) => self.sup_state.control.update(
                now,
                s.armed && !landed,
                e.attitude,
                self.guidance.visible.attitude,
                &p,
            ),
            None => {
                self.sup_state
                    .control
                    .update(now, false, Quat::identity(), Quat::identity(), &p)
            }
        };
        let breached = match (s.armed && valid.position, s.home, state) {
            (true, Some(home), Some(e)) => {
                geofence_breached(e.position, home, p.fence_radius_m, p.fence_altitude_m)
            }
            _ => false,
        };

        // 5. Links.
        let gcs_alive = Self::fresh(self.gcs, now, u64::from(p.gcs_timeout_us)).is_some();
        let brain_alive = self
            .brain_changed_at
            .is_some_and(|t| t <= now && now.since(t) <= u64::from(p.brain_timeout_us));
        let setpoint_fresh =
            Self::fresh(self.setpoint, now, u64::from(p.setpoint_timeout_us)).is_some();

        // 6. Pre-arm flags.
        let mut flags = PrearmFlags::empty();
        flags.set(
            PrearmFlags::KILL,
            rc.is_some_and(|r| r.kill_switch) || s.killed || self.sup_state.terminated_latch,
        );
        flags.set(
            PrearmFlags::THROTTLE,
            rc.is_some_and(|r| !r.throttle.is_finite() || r.throttle >= ARM_MAX_THROTTLE),
        );
        flags.set(PrearmFlags::RC, !rc_settled);
        flags.set(
            PrearmFlags::NOT_STILL,
            self.loop_gyro(now)
                .is_some_and(|g| !g.norm().is_finite() || g.norm() > ARM_MAX_RATE),
        );
        flags.set(
            PrearmFlags::ESTIMATOR,
            !(valid.attitude && valid.heading && valid.altitude),
        );
        flags.set(
            PrearmFlags::POSITION,
            matches!(pilot_mode, FlightMode::Position | FlightMode::Offboard) && !valid.position,
        );
        // Arming into Offboard needs a live brain, or the failsafe would be
        // in force on the first run.
        flags.set(
            PrearmFlags::LINK,
            pilot_mode == FlightMode::Offboard && !(brain_alive && setpoint_fresh),
        );
        flags.set(
            PrearmFlags::BATTERY,
            !(battery_sample.is_some()
                && self.sup_state.battery.plausible()
                && !self.sup_state.battery.low()),
        );
        // CONFIG never fails here: `Core::new` refuses an invalid
        // configuration, so a running core has a valid one.

        // 7. Arm / disarm (valid frames only).
        let fault_count = self.estimator_faults();
        if rc.is_some() {
            let arm_switch = self.sup_state.arm_confirmed;
            let rising = arm_switch && !self.sup_state.prev_arm_switch;
            if rising && !s.armed {
                if flags.is_empty() {
                    s.armed = true;
                    s.home = match (valid.position, state) {
                        (true, Some(e)) => Some(e.position),
                        _ => None,
                    };
                    self.sup_state.fault_baseline = fault_count;
                    self.emit(now, EventKind::Armed);
                    if s.home.is_some() {
                        self.emit(now, EventKind::HomeSet);
                    }
                } else {
                    self.emit(now, EventKind::ArmRefused(flags));
                }
            } else if s.armed && !arm_switch {
                self.disarm(now, &mut s, DisarmReason::Switch);
            }
            self.sup_state.prev_arm_switch = arm_switch;
        }
        if !s.armed {
            self.sup_state.fault_baseline = fault_count;
        } else if s.home.is_none()
            && valid.position
            && let Some(e) = state
        {
            // Armed without a position: home is the first position fix
            // after arming, so that Rtl becomes available.
            s.home = Some(e.position);
            self.emit(now, EventKind::HomeSet);
        }

        // 8. Failsafe. A failsafe that begins with the pilot's vehicle on
        // the ground disarms it (ADR-0003 §2).
        let landed_fs =
            landed || (self.sup_state.ground_hint && self.sup_state.prev_action == Action::None);
        let conditions = Conditions {
            armed: s.armed,
            landed: landed_fs,
            mode: pilot_mode,
            rc_valid,
            rc_settled,
            mode_switch_changed,
            gcs_alive,
            brain_alive,
            setpoint_fresh,
            battery_low: self.sup_state.battery.low(),
            battery_critical: self.sup_state.battery.critical(),
            valid,
            estimator_fault: fault_count > self.sup_state.fault_baseline,
            geofence_breached: breached,
            loss_of_control,
        };
        let decision = self.sup_state.failsafe.update(now, &conditions, &p);
        if s.armed {
            let prev = self.sup_state.prev_reasons;
            for i in 0..u16::BITS {
                let bit = Reasons(1 << i);
                if decision.reasons.difference(prev).intersects(bit) {
                    self.emit(now, EventKind::ReasonRaised(bit));
                }
                if prev.difference(decision.reasons).intersects(bit) {
                    self.emit(now, EventKind::ReasonCleared(bit));
                }
            }
            if decision.action != self.sup_state.prev_action {
                self.emit(
                    now,
                    EventKind::ActionChanged {
                        from: self.sup_state.prev_action,
                        to: decision.action,
                    },
                );
            }
            if decision.overridden {
                self.emit(now, EventKind::PilotOverride(decision.acknowledged));
            }
        }
        let prev_action = self.sup_state.prev_action;
        self.sup_state.prev_reasons = if s.armed {
            decision.reasons
        } else {
            Reasons::empty()
        };
        self.sup_state.prev_action = if s.armed {
            decision.action
        } else {
            Action::None
        };

        // 9. and 11. Landed with a failsafe in force: disarm. In a landing
        // action that was already under way this is the touchdown.
        if s.armed && decision.action != Action::None && landed_fs {
            let landing = matches!(
                decision.action,
                Action::Rtl | Action::Land | Action::LandNow
            );
            if landing && prev_action != Action::None {
                self.emit(now, EventKind::Touchdown);
            }
            self.disarm(now, &mut s, DisarmReason::Touchdown);
        }

        // 10. Terminate.
        if decision.action == Action::Terminate {
            self.disarm(now, &mut s, DisarmReason::Terminated);
            self.sup_state.terminated_latch = true;
        }

        // 12. Navigation state.
        let nav = if !s.armed {
            if self.sup_state.terminated_latch {
                NavState::Terminated
            } else {
                NavState::Disarmed
            }
        } else {
            match decision.action {
                Action::Hold => NavState::Hold,
                Action::Rtl => NavState::Rtl,
                Action::Land => NavState::Land,
                Action::LandNow => NavState::LandNow,
                Action::Terminate => NavState::Terminated,
                Action::None => {
                    let requested = requested_nav(pilot_mode);
                    let altitude_ok = valid.attitude && valid.altitude;
                    let degrade = if altitude_ok {
                        NavState::Altitude
                    } else {
                        NavState::Stabilized
                    };
                    let flown = match requested {
                        NavState::Position | NavState::Offboard => {
                            if valid.full() && !decision.restricted {
                                requested
                            } else {
                                degrade
                            }
                        }
                        NavState::Altitude => degrade,
                        other => other,
                    };
                    let degraded = (flown != requested).then_some(flown);
                    if degraded.is_some() && degraded != self.sup_state.degraded {
                        self.emit(
                            now,
                            EventKind::ModeDegraded {
                                requested: pilot_mode,
                                flown,
                            },
                        );
                    }
                    self.sup_state.degraded = degraded;
                    flown
                }
            }
        };
        if !s.armed || decision.action != Action::None {
            self.sup_state.degraded = None;
        }
        if nav != s.nav {
            self.emit(
                now,
                EventKind::NavChanged {
                    from: s.nav,
                    to: nav,
                },
            );
        }
        s.nav = nav;
        s.reasons = decision.reasons;
        s.action = decision.action;
        s.prearm = flags;
        self.supervision.pending = s;
    }

    /// Numerical faults reported by the filter (zero for the external source).
    fn estimator_faults(&self) -> u32 {
        if self.cfg.estimator.source == crate::config::EstimatorSource::Ekf {
            self.estimator.faults()
        } else {
            0
        }
    }
}
