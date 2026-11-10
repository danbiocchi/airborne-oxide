// SPDX-License-Identifier: AGPL-3.0-or-later
//! Failsafe state machine (ADR-0003 §3–§5).
//!
//! A pure function of the supervisor's [`Conditions`] and a small state. It
//! has no clock of its own: tick time comes in as `now`.

use crate::config::FailsafeParams;
use crate::io::{Action, FlightMode, Reasons, Validity};
use crate::time::Micros;

/// Facts about this instant, gathered by the supervisor.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Conditions {
    pub armed: bool,
    pub landed: bool,
    /// The mode the pilot's switch selects (before degradation).
    pub mode: FlightMode,
    /// A valid RC frame arrived within the RC timeout.
    pub rc_valid: bool,
    /// RC has been valid without a gap for `rc_valid_min_s`.
    pub rc_settled: bool,
    /// The mode switch moved since the previous run (compared between valid frames).
    pub mode_switch_changed: bool,
    /// The ground station was heard within its timeout (false if it never connected).
    pub gcs_alive: bool,
    /// The brain's progress counter advanced within its timeout.
    pub brain_alive: bool,
    /// A tier-2 setpoint arrived within its timeout.
    pub setpoint_fresh: bool,
    /// Latched by the battery monitor.
    pub battery_low: bool,
    pub battery_critical: bool,
    pub valid: Validity,
    /// The filter reported a numerical fault.
    pub estimator_fault: bool,
    pub geofence_breached: bool,
    pub loss_of_control: bool,
}

/// The outcome of one supervisor run.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Decision {
    /// The failsafe action in force (None = fly the pilot's mode).
    pub action: Action,
    /// Reasons that are true now.
    pub reasons: Reasons,
    /// Reasons the pilot has acknowledged by overriding and that are still true.
    pub acknowledged: Reasons,
    /// The pilot took over on this run.
    pub overridden: bool,
    /// The pilot may only fly Stabilized or Altitude (ADR-0003 §5 "Restricted").
    pub restricted: bool,
}

const N: usize = 11;

/// Every reason, in bit order.
const REASONS: [Reasons; N] = [
    Reasons::RC_LOST,
    Reasons::NO_HUMAN_LINK,
    Reasons::BRAIN_LOST,
    Reasons::SETPOINT_STALE,
    Reasons::BATTERY_LOW,
    Reasons::BATTERY_CRITICAL,
    Reasons::GPS_LOST,
    Reasons::ESTIMATOR_UNHEALTHY,
    Reasons::IMU_FAILURE,
    Reasons::GEOFENCE,
    Reasons::LOSS_OF_CONTROL,
];

/// The failsafe state machine.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Failsafe {
    /// When each reason became true (indexed like [`REASONS`]).
    since: [Option<Micros>; N],
    /// The action each acknowledged reason had when it was acknowledged.
    ack_action: [Action; N],
    acked: Reasons,
    /// Highest action at or above `Rtl` seen so far.
    latched: Action,
    /// The action returned by the previous run.
    action: Action,
    /// When the candidate first fell to `None` during a Hold.
    clear_since: Option<Micros>,
}

impl Failsafe {
    pub const fn new() -> Self {
        Self {
            since: [None; N],
            ack_action: [Action::None; N],
            acked: Reasons::empty(),
            latched: Action::None,
            action: Action::None,
            clear_since: None,
        }
    }

    /// Back to the initial state.
    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// One supervisor run at tick time `now` (non-decreasing).
    pub fn update(&mut self, now: Micros, c: &Conditions, p: &FailsafeParams) -> Decision {
        if !c.armed {
            self.reset();
            return Decision::default();
        }
        let prev = self.action;
        let reasons = evaluate(c, prev != Action::None);

        for (bit, since) in REASONS.iter().zip(self.since.iter_mut()) {
            if reasons.contains(*bit) {
                if since.is_none() {
                    *since = Some(now);
                }
            } else {
                *since = None;
            }
        }
        // A reason leaves the acknowledged set when it stops being true.
        self.acked = self.acked.intersection(reasons);

        let candidate = self.candidate(now, reasons, p);
        let mut action = self.arbitrate(now, prev, candidate, p);

        let mut overridden = false;
        // Override acts on a failsafe that was already in force; a reason
        // raised on this very run is not overridden by the same switch move.
        if prev != Action::None
            && action != Action::None
            && action != Action::Terminate
            && c.rc_settled
            && c.mode_switch_changed
        {
            for ((bit, since), ack) in REASONS
                .iter()
                .zip(self.since.iter())
                .zip(self.ack_action.iter_mut())
            {
                if reasons.contains(*bit) {
                    *ack = reason_action(*bit, elapsed_s(now, *since), p);
                }
            }
            self.acked = reasons;
            self.latched = Action::None;
            self.clear_since = None;
            let candidate = self.candidate(now, reasons, p);
            // Recomputed as if no Hold were in force, so no hysteresis.
            action = self.arbitrate(now, Action::None, candidate, p);
            overridden = true;
        }

        if prev == Action::Terminate {
            action = Action::Terminate;
        }
        self.action = action;

        Decision {
            action,
            reasons,
            acknowledged: self.acked,
            overridden,
            restricted: self
                .acked
                .intersects(Reasons::BATTERY_CRITICAL.union(Reasons::ESTIMATOR_UNHEALTHY)),
        }
    }

    /// Highest contribution of the true reasons, honouring acknowledgement.
    fn candidate(&self, now: Micros, reasons: Reasons, p: &FailsafeParams) -> Action {
        let mut best = Action::None;
        for ((bit, since), ack) in REASONS
            .iter()
            .zip(self.since.iter())
            .zip(self.ack_action.iter())
        {
            if !reasons.contains(*bit) {
                continue;
            }
            let a = reason_action(*bit, elapsed_s(now, *since), p);
            if self.acked.contains(*bit) && a <= *ack {
                continue;
            }
            best = best.max(a);
        }
        best
    }

    /// Latch and Hold hysteresis (rules 3 and 4).
    fn arbitrate(
        &mut self,
        now: Micros,
        prev: Action,
        candidate: Action,
        p: &FailsafeParams,
    ) -> Action {
        if candidate >= Action::Rtl {
            self.latched = self.latched.max(candidate);
        }
        let mut action = candidate.max(self.latched);
        if action == Action::None && prev == Action::Hold {
            let since = *self.clear_since.get_or_insert(now);
            if elapsed_s(now, Some(since)) < p.clear_hysteresis_s {
                action = Action::Hold;
            } else {
                self.clear_since = None;
            }
        } else {
            self.clear_since = None;
        }
        action
    }
}

/// Which reasons are true (ADR-0003 §3).
fn evaluate(c: &Conditions, in_failsafe: bool) -> Reasons {
    let stick = matches!(
        c.mode,
        FlightMode::Stabilized | FlightMode::Altitude | FlightMode::Position
    );
    let offboard = c.mode == FlightMode::Offboard;
    let mut r = Reasons::empty();
    r.set(Reasons::RC_LOST, !c.rc_valid && stick);
    r.set(
        Reasons::NO_HUMAN_LINK,
        !c.rc_valid && !c.gcs_alive && offboard,
    );
    r.set(Reasons::BRAIN_LOST, !c.brain_alive && offboard);
    r.set(Reasons::SETPOINT_STALE, !c.setpoint_fresh && offboard);
    r.set(Reasons::BATTERY_LOW, c.battery_low);
    r.set(Reasons::BATTERY_CRITICAL, c.battery_critical);
    r.set(
        Reasons::GPS_LOST,
        !c.valid.position
            && (offboard || in_failsafe || (c.mode == FlightMode::Position && !c.rc_valid)),
    );
    r.set(
        Reasons::ESTIMATOR_UNHEALTHY,
        c.valid.attitude && (!c.valid.altitude || !c.valid.heading || c.estimator_fault),
    );
    r.set(Reasons::IMU_FAILURE, !c.valid.attitude);
    r.set(Reasons::GEOFENCE, c.geofence_breached);
    r.set(Reasons::LOSS_OF_CONTROL, c.loss_of_control && !c.landed);
    r
}

/// Seconds from `since` to `now` (zero when `since` is unset).
#[allow(clippy::cast_precision_loss)]
fn elapsed_s(now: Micros, since: Option<Micros>) -> f32 {
    match since {
        Some(s) => now.since(s) as f32 / 1_000_000.0,
        None => 0.0,
    }
}

/// The action a single reason calls for after being true for `elapsed_s`.
///
/// `reason` must be exactly one bit; anything else calls for no action.
pub fn reason_action(reason: Reasons, elapsed_s: f32, p: &FailsafeParams) -> Action {
    let delayed = |first: Action, after: Action, limit_s: f32| {
        if elapsed_s >= limit_s { after } else { first }
    };
    if reason == Reasons::RC_LOST {
        delayed(Action::Hold, Action::Rtl, p.rc_loss_hold_s)
    } else if reason == Reasons::NO_HUMAN_LINK {
        delayed(Action::None, Action::Rtl, p.no_link_rtl_s)
    } else if reason == Reasons::BRAIN_LOST {
        delayed(Action::Hold, Action::Rtl, p.brain_loss_rtl_s)
    } else if reason == Reasons::SETPOINT_STALE {
        Action::Hold
    } else if reason == Reasons::BATTERY_LOW || reason == Reasons::GEOFENCE {
        Action::Rtl
    } else if reason == Reasons::BATTERY_CRITICAL || reason == Reasons::ESTIMATOR_UNHEALTHY {
        Action::LandNow
    } else if reason == Reasons::GPS_LOST {
        delayed(Action::Hold, Action::Land, p.gps_loss_land_s)
    } else if reason == Reasons::IMU_FAILURE || reason == Reasons::LOSS_OF_CONTROL {
        Action::Terminate
    } else {
        Action::None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;

    fn p() -> FailsafeParams {
        FailsafeParams::default()
    }

    fn healthy() -> Conditions {
        Conditions {
            armed: true,
            mode: FlightMode::Stabilized,
            rc_valid: true,
            rc_settled: true,
            gcs_alive: true,
            brain_alive: true,
            setpoint_fresh: true,
            valid: Validity::ALL,
            ..Conditions::default()
        }
    }

    fn offboard() -> Conditions {
        Conditions {
            mode: FlightMode::Offboard,
            ..healthy()
        }
    }

    fn no_pos() -> Validity {
        Validity {
            position: false,
            ..Validity::ALL
        }
    }

    fn at(s: f64) -> Micros {
        Micros((s * 1e6) as u64)
    }

    fn run(f: &mut Failsafe, s: f64, c: &Conditions) -> Decision {
        f.update(at(s), c, &p())
    }

    #[test]
    fn healthy_stays_none() {
        let mut f = Failsafe::new();
        for i in 0..10 {
            let d = run(&mut f, f64::from(i), &healthy());
            assert_eq!(d.action, Action::None);
            assert!(d.reasons.is_empty());
        }
    }

    #[test]
    fn reason_action_table() {
        let q = p();
        let a = |r, e| reason_action(r, e, &q);
        assert_eq!(a(Reasons::RC_LOST, 0.0), Action::Hold);
        assert_eq!(a(Reasons::RC_LOST, 2.99), Action::Hold);
        assert_eq!(a(Reasons::RC_LOST, 3.0), Action::Rtl);
        assert_eq!(a(Reasons::NO_HUMAN_LINK, 59.9), Action::None);
        assert_eq!(a(Reasons::NO_HUMAN_LINK, 60.0), Action::Rtl);
        assert_eq!(a(Reasons::BRAIN_LOST, 9.9), Action::Hold);
        assert_eq!(a(Reasons::BRAIN_LOST, 10.0), Action::Rtl);
        assert_eq!(a(Reasons::SETPOINT_STALE, 1000.0), Action::Hold);
        assert_eq!(a(Reasons::BATTERY_LOW, 0.0), Action::Rtl);
        assert_eq!(a(Reasons::BATTERY_CRITICAL, 0.0), Action::LandNow);
        assert_eq!(a(Reasons::GPS_LOST, 29.9), Action::Hold);
        assert_eq!(a(Reasons::GPS_LOST, 30.0), Action::Land);
        assert_eq!(a(Reasons::ESTIMATOR_UNHEALTHY, 0.0), Action::LandNow);
        assert_eq!(a(Reasons::IMU_FAILURE, 0.0), Action::Terminate);
        assert_eq!(a(Reasons::GEOFENCE, 0.0), Action::Rtl);
        assert_eq!(a(Reasons::LOSS_OF_CONTROL, 0.0), Action::Terminate);
        assert_eq!(a(Reasons::empty(), 0.0), Action::None);
    }

    #[test]
    fn rc_lost_hold_then_rtl_at_boundary() {
        let mut f = Failsafe::new();
        let c = Conditions {
            rc_valid: false,
            rc_settled: false,
            ..healthy()
        };
        let d = run(&mut f, 10.0, &c);
        assert_eq!(d.action, Action::Hold);
        assert!(d.reasons.contains(Reasons::RC_LOST));
        assert_eq!(run(&mut f, 12.999, &c).action, Action::Hold);
        assert_eq!(run(&mut f, 13.0, &c).action, Action::Rtl);
    }

    #[test]
    fn rc_lost_does_not_apply_in_offboard() {
        let mut f = Failsafe::new();
        let c = Conditions {
            rc_valid: false,
            ..offboard()
        };
        let d = run(&mut f, 0.0, &c);
        assert!(!d.reasons.contains(Reasons::RC_LOST));
    }

    #[test]
    fn no_human_link_none_then_rtl() {
        let mut f = Failsafe::new();
        let c = Conditions {
            rc_valid: false,
            gcs_alive: false,
            ..offboard()
        };
        let d = run(&mut f, 0.0, &c);
        assert!(d.reasons.contains(Reasons::NO_HUMAN_LINK));
        assert_eq!(d.action, Action::None);
        assert_eq!(run(&mut f, 59.9, &c).action, Action::None);
        assert_eq!(run(&mut f, 60.0, &c).action, Action::Rtl);
    }

    #[test]
    fn no_human_link_needs_both_links_lost() {
        let mut f = Failsafe::new();
        let rc_only = Conditions {
            rc_valid: false,
            ..offboard()
        };
        assert!(run(&mut f, 0.0, &rc_only).reasons.is_empty());
        let gcs_only = Conditions {
            gcs_alive: false,
            ..offboard()
        };
        assert!(run(&mut f, 1.0, &gcs_only).reasons.is_empty());
    }

    #[test]
    fn brain_lost_hold_then_rtl() {
        let mut f = Failsafe::new();
        let c = Conditions {
            brain_alive: false,
            ..offboard()
        };
        assert_eq!(run(&mut f, 0.0, &c).action, Action::Hold);
        assert_eq!(run(&mut f, 9.999, &c).action, Action::Hold);
        assert_eq!(run(&mut f, 10.0, &c).action, Action::Rtl);
        // Brain lost does not apply to a pilot mode.
        let mut g = Failsafe::new();
        let s = Conditions {
            brain_alive: false,
            ..healthy()
        };
        assert!(run(&mut g, 0.0, &s).reasons.is_empty());
    }

    #[test]
    fn setpoint_stale_holds_in_offboard_only() {
        let mut f = Failsafe::new();
        let c = Conditions {
            setpoint_fresh: false,
            ..offboard()
        };
        let d = run(&mut f, 0.0, &c);
        assert_eq!(d.action, Action::Hold);
        assert!(d.reasons.contains(Reasons::SETPOINT_STALE));
        let mut g = Failsafe::new();
        let s = Conditions {
            setpoint_fresh: false,
            ..healthy()
        };
        assert!(run(&mut g, 0.0, &s).reasons.is_empty());
    }

    #[test]
    fn battery_low_rtl_and_critical_land_now() {
        let mut f = Failsafe::new();
        let low = Conditions {
            battery_low: true,
            ..healthy()
        };
        let d = run(&mut f, 0.0, &low);
        assert_eq!(d.action, Action::Rtl);
        assert!(d.reasons.contains(Reasons::BATTERY_LOW));
        let crit = Conditions {
            battery_low: true,
            battery_critical: true,
            ..healthy()
        };
        assert_eq!(run(&mut f, 1.0, &crit).action, Action::LandNow);
    }

    #[test]
    fn gps_lost_offboard_hold_then_land() {
        let mut f = Failsafe::new();
        let c = Conditions {
            valid: no_pos(),
            ..offboard()
        };
        let d = run(&mut f, 0.0, &c);
        assert!(d.reasons.contains(Reasons::GPS_LOST));
        assert_eq!(d.action, Action::Hold);
        assert_eq!(run(&mut f, 29.999, &c).action, Action::Hold);
        assert_eq!(run(&mut f, 30.0, &c).action, Action::Land);
    }

    #[test]
    fn gps_lost_with_pilot_flying_is_not_a_reason() {
        let mut f = Failsafe::new();
        let c = Conditions {
            mode: FlightMode::Position,
            valid: no_pos(),
            ..healthy()
        };
        let d = run(&mut f, 0.0, &c);
        assert!(d.reasons.is_empty());
        assert_eq!(d.action, Action::None);
    }

    #[test]
    fn gps_lost_in_position_with_rc_lost() {
        let mut f = Failsafe::new();
        let c = Conditions {
            mode: FlightMode::Position,
            rc_valid: false,
            valid: no_pos(),
            ..healthy()
        };
        let d = run(&mut f, 0.0, &c);
        assert!(d.reasons.contains(Reasons::GPS_LOST));
        assert!(d.reasons.contains(Reasons::RC_LOST));
    }

    #[test]
    fn gps_lost_counts_while_in_failsafe() {
        let mut f = Failsafe::new();
        // Stabilized with RC lost: Hold, but position invalid is not yet a reason.
        let nopos = Conditions {
            rc_valid: false,
            valid: no_pos(),
            ..healthy()
        };
        let d = run(&mut f, 0.0, &nopos);
        assert_eq!(d.action, Action::Hold);
        assert!(!d.reasons.contains(Reasons::GPS_LOST));
        // Next run the vehicle is in failsafe, so GPS loss now counts.
        let d = run(&mut f, 0.1, &nopos);
        assert!(d.reasons.contains(Reasons::GPS_LOST));
    }

    #[test]
    fn estimator_unhealthy_variants_land_now() {
        let cases = [
            Validity {
                altitude: false,
                ..Validity::ALL
            },
            Validity {
                heading: false,
                ..Validity::ALL
            },
        ];
        for v in cases {
            let mut f = Failsafe::new();
            let c = Conditions {
                valid: v,
                ..healthy()
            };
            let d = run(&mut f, 0.0, &c);
            assert!(d.reasons.contains(Reasons::ESTIMATOR_UNHEALTHY));
            assert_eq!(d.action, Action::LandNow);
        }
        let mut f = Failsafe::new();
        let c = Conditions {
            estimator_fault: true,
            ..healthy()
        };
        let d = run(&mut f, 0.0, &c);
        assert!(d.reasons.contains(Reasons::ESTIMATOR_UNHEALTHY));
        assert_eq!(d.action, Action::LandNow);
    }

    #[test]
    fn imu_failure_terminates_and_is_not_estimator_unhealthy() {
        let mut f = Failsafe::new();
        let c = Conditions {
            valid: Validity::NONE,
            ..healthy()
        };
        let d = run(&mut f, 0.0, &c);
        assert_eq!(d.action, Action::Terminate);
        assert!(d.reasons.contains(Reasons::IMU_FAILURE));
        assert!(!d.reasons.contains(Reasons::ESTIMATOR_UNHEALTHY));
    }

    #[test]
    fn geofence_rtl() {
        let mut f = Failsafe::new();
        let c = Conditions {
            geofence_breached: true,
            ..healthy()
        };
        assert_eq!(run(&mut f, 0.0, &c).action, Action::Rtl);
    }

    #[test]
    fn loss_of_control_terminates_unless_landed() {
        let mut f = Failsafe::new();
        let c = Conditions {
            loss_of_control: true,
            landed: true,
            ..healthy()
        };
        assert_eq!(run(&mut f, 0.0, &c).action, Action::None);
        let c = Conditions { landed: false, ..c };
        assert_eq!(run(&mut f, 0.1, &c).action, Action::Terminate);
    }

    #[test]
    fn disarmed_resets_and_returns_default() {
        let mut f = Failsafe::new();
        let bad = Conditions {
            battery_critical: true,
            ..healthy()
        };
        assert_eq!(run(&mut f, 0.0, &bad).action, Action::LandNow);
        let off = Conditions {
            armed: false,
            ..bad
        };
        assert_eq!(run(&mut f, 1.0, &off), Decision::default());
        assert_eq!(f, Failsafe::new());
        assert_eq!(run(&mut f, 2.0, &healthy()).action, Action::None);
    }

    #[test]
    fn highest_contribution_wins() {
        let mut f = Failsafe::new();
        let c = Conditions {
            rc_valid: false,
            geofence_breached: true,
            battery_critical: true,
            ..healthy()
        };
        assert_eq!(run(&mut f, 0.0, &c).action, Action::LandNow);
    }

    #[test]
    fn rtl_latch_never_drops() {
        let mut f = Failsafe::new();
        let c = Conditions {
            geofence_breached: true,
            ..healthy()
        };
        assert_eq!(run(&mut f, 0.0, &c).action, Action::Rtl);
        for i in 1..20 {
            assert_eq!(run(&mut f, f64::from(i), &healthy()).action, Action::Rtl);
        }
        // A higher candidate raises the latch.
        let crit = Conditions {
            battery_critical: true,
            ..healthy()
        };
        assert_eq!(run(&mut f, 30.0, &crit).action, Action::LandNow);
        assert_eq!(run(&mut f, 31.0, &healthy()).action, Action::LandNow);
    }

    #[test]
    fn hold_hysteresis_then_none() {
        let mut f = Failsafe::new();
        let bad = Conditions {
            setpoint_fresh: false,
            ..offboard()
        };
        assert_eq!(run(&mut f, 0.0, &bad).action, Action::Hold);
        let ok = offboard();
        assert_eq!(run(&mut f, 1.0, &ok).action, Action::Hold);
        assert_eq!(run(&mut f, 1.999, &ok).action, Action::Hold);
        assert_eq!(run(&mut f, 2.0, &ok).action, Action::None);
        assert_eq!(run(&mut f, 2.1, &ok).action, Action::None);
    }

    #[test]
    fn hold_hysteresis_restarts_when_reason_returns() {
        let mut f = Failsafe::new();
        let bad = Conditions {
            setpoint_fresh: false,
            ..offboard()
        };
        let ok = offboard();
        run(&mut f, 0.0, &bad);
        assert_eq!(run(&mut f, 1.0, &ok).action, Action::Hold);
        assert_eq!(run(&mut f, 1.5, &bad).action, Action::Hold);
        assert_eq!(run(&mut f, 2.0, &ok).action, Action::Hold);
        // Clear since 2.0, not 1.0.
        assert_eq!(run(&mut f, 2.9, &ok).action, Action::Hold);
        assert_eq!(run(&mut f, 3.0, &ok).action, Action::None);
    }

    fn rc_back(settled: bool, changed: bool) -> Conditions {
        Conditions {
            rc_valid: true,
            rc_settled: settled,
            mode_switch_changed: changed,
            ..healthy()
        }
    }

    #[test]
    fn override_needs_settled_rc_and_switch_change() {
        let mut f = Failsafe::new();
        let lost = Conditions {
            rc_valid: false,
            rc_settled: false,
            ..healthy()
        };
        assert_eq!(run(&mut f, 0.0, &lost).action, Action::Hold);
        assert_eq!(run(&mut f, 3.0, &lost).action, Action::Rtl);
        // RC back, settled, but no switch change: nothing happens.
        assert_eq!(run(&mut f, 5.0, &rc_back(true, false)).action, Action::Rtl);
        // Switch change but RC not settled: nothing happens.
        assert_eq!(run(&mut f, 5.1, &rc_back(false, true)).action, Action::Rtl);
        let d = run(&mut f, 5.2, &rc_back(true, true));
        assert!(d.overridden);
        assert_eq!(d.action, Action::None);
        // Latch is gone: the next run stays None.
        let d = run(&mut f, 5.3, &rc_back(true, false));
        assert!(!d.overridden);
        assert_eq!(d.action, Action::None);
    }

    #[test]
    fn override_acknowledges_true_reasons() {
        let mut f = Failsafe::new();
        let bad = Conditions {
            battery_low: true,
            ..healthy()
        };
        assert_eq!(run(&mut f, 0.0, &bad).action, Action::Rtl);
        let sw = Conditions {
            mode_switch_changed: true,
            ..bad
        };
        let d = run(&mut f, 1.0, &sw);
        assert!(d.overridden);
        assert_eq!(d.action, Action::None);
        assert_eq!(d.acknowledged, Reasons::BATTERY_LOW);
        assert!(!d.restricted);
        // Still true, still acknowledged: no new action.
        let d = run(&mut f, 2.0, &bad);
        assert_eq!(d.action, Action::None);
        assert_eq!(d.acknowledged, Reasons::BATTERY_LOW);
    }

    #[test]
    fn override_from_hold_goes_straight_to_none() {
        let mut f = Failsafe::new();
        let bad = Conditions {
            setpoint_fresh: false,
            ..offboard()
        };
        assert_eq!(run(&mut f, 0.0, &bad).action, Action::Hold);
        let sw = Conditions {
            mode_switch_changed: true,
            ..bad
        };
        let d = run(&mut f, 0.5, &sw);
        assert!(d.overridden);
        assert_eq!(d.action, Action::None);
    }

    #[test]
    fn acknowledgement_drops_when_reason_clears_and_recurs() {
        let mut f = Failsafe::new();
        let lost = Conditions {
            rc_valid: false,
            rc_settled: false,
            ..healthy()
        };
        assert_eq!(run(&mut f, 0.0, &lost).action, Action::Hold);
        // The switch moves with RC back: the reason is no longer true, so
        // the override has nothing to acknowledge.
        let back = Conditions {
            mode_switch_changed: true,
            ..healthy()
        };
        let d = run(&mut f, 1.0, &back);
        assert!(d.overridden);
        assert_eq!(d.action, Action::None);
        // The reason recurs and acts again from Hold.
        let d = run(&mut f, 1.1, &lost);
        assert_eq!(d.action, Action::Hold);
        assert!(d.acknowledged.is_empty());
    }

    #[test]
    fn acknowledged_reason_escalating_beyond_ack_level_acts() {
        let mut f = Failsafe::new();
        // GPS lost in Offboard acknowledged at Hold, escalates to Land at 30 s.
        let c = Conditions {
            valid: no_pos(),
            ..offboard()
        };
        assert_eq!(run(&mut f, 0.0, &c).action, Action::Hold);
        let sw = Conditions {
            mode_switch_changed: true,
            ..c
        };
        let d = run(&mut f, 1.0, &sw);
        assert!(d.overridden);
        assert_eq!(d.action, Action::None);
        assert!(d.acknowledged.contains(Reasons::GPS_LOST));
        assert_eq!(run(&mut f, 20.0, &c).action, Action::None);
        assert_eq!(run(&mut f, 29.9, &c).action, Action::None);
        assert_eq!(run(&mut f, 30.0, &c).action, Action::Land);
    }

    #[test]
    fn new_reason_after_override_acts() {
        let mut f = Failsafe::new();
        let bad = Conditions {
            battery_low: true,
            ..healthy()
        };
        run(&mut f, 0.0, &bad);
        let sw = Conditions {
            mode_switch_changed: true,
            ..bad
        };
        assert!(run(&mut f, 1.0, &sw).overridden);
        let worse = Conditions {
            geofence_breached: true,
            ..bad
        };
        assert_eq!(run(&mut f, 2.0, &worse).action, Action::Rtl);
    }

    #[test]
    fn restricted_for_critical_and_unhealthy_overrides() {
        for c in [
            Conditions {
                battery_critical: true,
                ..healthy()
            },
            Conditions {
                valid: Validity {
                    heading: false,
                    ..Validity::ALL
                },
                ..healthy()
            },
        ] {
            let mut f = Failsafe::new();
            assert!(!run(&mut f, 0.0, &c).restricted);
            let sw = Conditions {
                mode_switch_changed: true,
                ..c
            };
            let d = run(&mut f, 1.0, &sw);
            assert!(d.overridden);
            assert_eq!(d.action, Action::None);
            assert!(d.restricted);
            // Clearing the reason lifts the restriction.
            let d = run(&mut f, 2.0, &healthy());
            assert!(!d.restricted);
            assert!(d.acknowledged.is_empty());
        }
    }

    #[test]
    fn terminate_is_absorbing_and_not_overridable() {
        let mut f = Failsafe::new();
        let c = Conditions {
            loss_of_control: true,
            ..healthy()
        };
        assert_eq!(run(&mut f, 0.0, &c).action, Action::Terminate);
        let sw = Conditions {
            mode_switch_changed: true,
            ..healthy()
        };
        let d = run(&mut f, 1.0, &sw);
        assert!(!d.overridden);
        assert_eq!(d.action, Action::Terminate);
        assert_eq!(run(&mut f, 2.0, &healthy()).action, Action::Terminate);
        // Disarm then rearm starts clean.
        let off = Conditions {
            armed: false,
            ..healthy()
        };
        run(&mut f, 3.0, &off);
        assert_eq!(run(&mut f, 4.0, &healthy()).action, Action::None);
    }

    #[test]
    fn same_step_reason_is_not_overridden() {
        let mut f = Failsafe::new();
        let c = Conditions {
            setpoint_fresh: false,
            mode_switch_changed: true,
            ..offboard()
        };
        let d = run(&mut f, 0.0, &c);
        assert!(!d.overridden);
        assert_eq!(d.action, Action::Hold);
    }

    // ---- property tests ----

    fn mode_strategy() -> impl Strategy<Value = FlightMode> {
        prop_oneof![
            Just(FlightMode::Stabilized),
            Just(FlightMode::Altitude),
            Just(FlightMode::Position),
            Just(FlightMode::Offboard),
        ]
    }

    fn validity_strategy() -> impl Strategy<Value = Validity> {
        // Biased towards fully valid so that long histories are interesting.
        (
            any::<bool>(),
            any::<bool>(),
            any::<bool>(),
            any::<bool>(),
            any::<bool>(),
            0u8..4,
        )
            .prop_map(|(a, h, al, v, po, all)| {
                if all != 0 {
                    Validity::ALL
                } else {
                    Validity {
                        attitude: a,
                        heading: h,
                        altitude: al,
                        velocity: v,
                        position: po,
                    }
                }
            })
    }

    fn bool_mostly(true_weight: u32) -> impl Strategy<Value = bool> {
        prop_oneof![true_weight => Just(true), 1 => Just(false)]
    }

    fn conditions_strategy() -> impl Strategy<Value = Conditions> {
        let a = (
            any::<bool>(),
            mode_strategy(),
            bool_mostly(4),
            any::<bool>(),
            any::<bool>(),
            bool_mostly(2),
            bool_mostly(4),
            bool_mostly(4),
        );
        let b = (
            any::<bool>(),
            any::<bool>(),
            validity_strategy(),
            any::<bool>(),
            any::<bool>(),
            any::<bool>(),
        );
        (a, b).prop_map(|(a, b)| Conditions {
            armed: true,
            landed: a.0,
            mode: a.1,
            rc_valid: a.2,
            rc_settled: a.3,
            mode_switch_changed: a.4,
            gcs_alive: a.5,
            brain_alive: a.6,
            setpoint_fresh: a.7,
            battery_low: b.0 && b.1,
            battery_critical: b.1 && b.5,
            valid: b.2,
            estimator_fault: b.3 && b.5,
            geofence_breached: b.4 && b.5,
            // Rare so that Terminate is not reached on nearly every sequence.
            loss_of_control: b.0 && b.3 && b.4 && b.5,
        })
    }

    fn steps_strategy() -> impl Strategy<Value = Vec<(u64, Conditions)>> {
        prop::collection::vec((0u64..2_500_000, conditions_strategy()), 1..120)
    }

    proptest! {
        #[test]
        fn prop_action_never_decreases_without_override(steps in steps_strategy()) {
            let mut f = Failsafe::new();
            let mut t = 0u64;
            let mut prev = Action::None;
            for (dt, c) in steps {
                t += dt;
                let d = f.update(Micros(t), &c, &p());
                if !d.overridden && d.action < prev {
                    prop_assert_eq!(prev, Action::Hold);
                    prop_assert_eq!(d.action, Action::None);
                }
                prev = d.action;
            }
        }

        #[test]
        fn prop_action_at_least_every_unacknowledged_reason(steps in steps_strategy()) {
            let q = p();
            let mut f = Failsafe::new();
            let mut t = 0u64;
            let mut since: [Option<u64>; N] = [None; N];
            for (dt, c) in steps {
                t += dt;
                let d = f.update(Micros(t), &c, &q);
                for (bit, s) in REASONS.iter().zip(since.iter_mut()) {
                    if d.reasons.contains(*bit) {
                        let start = *s.get_or_insert(t);
                        if !d.acknowledged.contains(*bit) {
                            let e = (t - start) as f32 / 1_000_000.0;
                            prop_assert!(d.action >= reason_action(*bit, e, &q));
                        }
                    } else {
                        *s = None;
                    }
                }
                prop_assert!(d.acknowledged.difference(d.reasons).is_empty());
            }
        }

        #[test]
        fn prop_override_requires_its_conditions(steps in steps_strategy()) {
            let mut f = Failsafe::new();
            let mut t = 0u64;
            let mut prev = Action::None;
            for (dt, c) in steps {
                t += dt;
                let d = f.update(Micros(t), &c, &p());
                if d.overridden {
                    prop_assert!(c.rc_settled && c.mode_switch_changed);
                    prop_assert!(prev != Action::None && prev != Action::Terminate);
                }
                prev = d.action;
            }
        }

        #[test]
        fn prop_terminate_is_absorbing(steps in steps_strategy()) {
            let mut f = Failsafe::new();
            let mut t = 0u64;
            let mut terminated = false;
            for (dt, c) in steps {
                t += dt;
                let d = f.update(Micros(t), &c, &p());
                if terminated {
                    prop_assert_eq!(d.action, Action::Terminate);
                    prop_assert!(!d.overridden);
                }
                terminated |= d.action == Action::Terminate;
            }
        }

        #[test]
        fn prop_disarm_resets(steps in steps_strategy(), extra in conditions_strategy()) {
            let mut f = Failsafe::new();
            let mut t = 0u64;
            for (dt, c) in &steps {
                t += dt;
                f.update(Micros(t), c, &p());
            }
            let off = Conditions { armed: false, ..extra };
            prop_assert_eq!(f.update(Micros(t + 1), &off, &p()), Decision::default());
            // The next armed run matches a fresh machine.
            let mut fresh = Failsafe::new();
            let a = f.update(Micros(t + 2), &extra, &p());
            let b = fresh.update(Micros(t + 2), &extra, &p());
            prop_assert_eq!(a, b);
        }
    }
}
