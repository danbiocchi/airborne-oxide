// SPDX-License-Identifier: AGPL-3.0-or-later
//! Failsafes, navigation states and arming through the whole core
//! (ADR-0003).

use super::*;
use crate::flight::guidance::RtlPhase;
use crate::io::DisarmReason;
use crate::monitor::tilt_angle;

/// A scripted world around one core: feeds every input kind each tick and
/// collects the events.
struct Rig {
    core: Core<4>,
    k: u64,
    ekf: bool,
    rc: RcSample,
    rc_on: bool,
    /// External state (ignored with the EKF).
    state: Option<StateSample>,
    battery: Option<BatterySample>,
    brain_on: bool,
    brain_frozen: bool,
    brain_progress: u32,
    setpoint: Option<TrajectorySetpoint>,
    gps_on: bool,
    imu_on: bool,
    events: Vec<Event>,
    out: Outputs<4>,
}

fn at_height(h: f32) -> StateSample {
    StateSample {
        position: Vec3::new(0.0, 0.0, -h),
        ..StateSample::default()
    }
}

impl Rig {
    fn new(ekf: bool, mode_switch: u8) -> Self {
        let cfg = if ekf { ekf_cfg() } else { cfg() };
        Self {
            core: Core::new(cfg, F405_LEAN).expect("valid config"),
            k: 0,
            ekf,
            rc: RcSample {
                mode_switch,
                ..RcSample::default()
            },
            rc_on: true,
            state: Some(at_height(5.0)),
            battery: Some(BatterySample {
                voltage_v: 23.0,
                current_a: 0.0,
            }),
            brain_on: true,
            brain_frozen: false,
            brain_progress: 0,
            setpoint: Some(TrajectorySetpoint {
                position: Vec3::new(0.0, 0.0, -5.0),
                velocity: Vec3::zero(),
                yaw: 0.0,
            }),
            gps_on: true,
            imu_on: true,
            events: Vec::new(),
            out: Outputs::off(Micros::ZERO),
        }
    }

    /// External state, hovering at 5 m.
    fn external(mode_switch: u8) -> Self {
        Self::new(false, mode_switch)
    }

    /// EKF with synthetic sensors of a vehicle at rest.
    fn ekf(mode_switch: u8) -> Self {
        Self::new(true, mode_switch)
    }

    fn step(&mut self) {
        let t = Micros(self.k * 1000);
        let mut i = if self.ekf {
            ekf_inputs(self.k, RcSample::default())
        } else {
            Inputs {
                state: self.state.map(|s| Stamped::ideal(t, s)),
                ..Inputs::default()
            }
        };
        i.rc = self.rc_on.then(|| Stamped::ideal(t, self.rc));
        i.battery = self.battery.map(|b| Stamped::ideal(t, b));
        if !self.gps_on {
            i.gps = None;
        }
        if !self.imu_on {
            i.imu = None;
        }
        if self.brain_on && self.k.is_multiple_of(50) {
            if !self.brain_frozen {
                self.brain_progress += 1;
            }
            i.brain_heartbeat = Some(Stamped::ideal(
                t,
                BrainHeartbeat {
                    progress: self.brain_progress.max(1),
                },
            ));
        }
        if self.k.is_multiple_of(20) {
            i.setpoint = self.setpoint.map(|s| Stamped::ideal(t, s));
        }
        self.out = self.core.tick(t, &i);
        self.events.extend(self.out.events.iter().copied());
        self.k += 1;
    }

    fn run(&mut self, secs: f64) {
        for _ in 0..(secs * 1000.0).round() as u64 {
            self.step();
        }
    }

    /// Runs until `cond` holds or `max_s` pass; true if it held.
    fn run_until(&mut self, max_s: f64, mut cond: impl FnMut(&Self) -> bool) -> bool {
        for _ in 0..(max_s * 1000.0).round() as u64 {
            self.step();
            if cond(self) {
                return true;
            }
        }
        false
    }

    /// Settles RC (and, with the EKF, the filter), then raises the arm
    /// switch and runs a little.
    fn arm(&mut self) {
        self.rc.arm_switch = false;
        self.run(if self.ekf { 3.0 } else { 1.2 });
        self.rc.arm_switch = true;
        self.run(0.3);
        assert!(
            self.out.status.armed,
            "arming failed: {:?}",
            self.out.status
        );
    }

    fn nav(&self) -> NavState {
        self.out.status.nav
    }

    fn saw(&self, kind: EventKind) -> bool {
        self.events.iter().any(|e| e.kind == kind)
    }

    fn count(&self, f: impl Fn(&EventKind) -> bool) -> usize {
        self.events.iter().filter(|e| f(&e.kind)).count()
    }
}

#[test]
fn baseline_arms_and_reports_armed_and_home() {
    let mut rig = Rig::external(1);
    rig.arm();
    assert!(rig.out.status.prearm.is_empty());
    assert!(rig.saw(EventKind::Armed) && rig.saw(EventKind::HomeSet));
    assert!(rig.out.status.home_set);
    assert_eq!(rig.core.home(), Some(Vec3::new(0.0, 0.0, -5.0)));
    assert_eq!(rig.nav(), NavState::Position);
    assert_eq!(rig.out.status.mode, FlightMode::Position);
    assert_eq!(rig.out.status.valid, Validity::ALL);
    // A mode that is fully valid is not reported as degraded.
    assert_eq!(
        rig.count(|k| matches!(k, EventKind::ModeDegraded { .. })),
        0
    );
}

/// Tries to arm a rig that has one failing pre-arm check, and checks that
/// the attempt is refused with exactly `bit`.
fn refused_with(mut rig: Rig, bit: PrearmFlags, settle_s: f64) {
    rig.rc.arm_switch = false;
    rig.run(settle_s);
    rig.rc.arm_switch = true;
    rig.run(0.5);
    assert!(!rig.out.status.armed, "{bit:?} must block arming");
    assert_eq!(rig.out.status.prearm, bit, "only {bit:?} should be set");
    let refusals: Vec<_> = rig
        .events
        .iter()
        .filter_map(|e| match e.kind {
            EventKind::ArmRefused(f) => Some(f),
            _ => None,
        })
        .collect();
    assert_eq!(refusals, [bit], "one refusal, carrying the failing bit");
}

#[test]
fn each_prearm_bit_alone_blocks_arming_and_is_reported() {
    // KILL: the switch is on.
    let mut rig = Rig::external(1);
    rig.rc.kill_switch = true;
    refused_with(rig, PrearmFlags::KILL, 1.2);
    // THROTTLE.
    let mut rig = Rig::external(1);
    rig.rc.throttle = 0.5;
    refused_with(rig, PrearmFlags::THROTTLE, 1.2);
    // RC: valid for less than a second.
    refused_with(Rig::external(1), PrearmFlags::RC, 0.3);
    // NOT_STILL: body rate 0.5 rad/s.
    let mut rig = Rig::external(1);
    rig.state = Some(StateSample {
        rate: Vec3::new(0.5, 0.0, 0.0),
        ..at_height(5.0)
    });
    refused_with(rig, PrearmFlags::NOT_STILL, 1.2);
    // ESTIMATOR: no state at all, in a mode that does not need a position.
    let mut rig = Rig::external(0);
    rig.state = None;
    refused_with(rig, PrearmFlags::ESTIMATOR, 1.2);
    // POSITION: the EKF has no GPS, the switch selects Position.
    let mut rig = Rig::ekf(1);
    rig.gps_on = false;
    refused_with(rig, PrearmFlags::POSITION, 3.0);
    // BATTERY: no reading, then an implausible one.
    let mut rig = Rig::external(1);
    rig.battery = None;
    refused_with(rig, PrearmFlags::BATTERY, 1.2);
    let mut rig = Rig::external(1);
    rig.battery = Some(BatterySample {
        voltage_v: 12.0,
        current_a: 0.0,
    });
    refused_with(rig, PrearmFlags::BATTERY, 1.2);
}

#[test]
fn the_config_prearm_bit_is_never_set_on_a_running_core() {
    // `Core::new` refuses an invalid configuration, so CONFIG cannot be
    // observed failing; it must not be set spuriously either.
    let mut rig = Rig::external(1);
    rig.arm();
    assert!(!rig.out.status.prearm.contains(PrearmFlags::CONFIG));
    let bad = {
        let mut c = cfg();
        c.mass_kg = -1.0;
        c
    };
    assert!(Core::new(bad, F405_LEAN).is_err());
}

#[test]
fn a_refused_attempt_is_not_retried_while_the_switch_stays_up() {
    let mut rig = Rig::external(1);
    rig.battery = None;
    rig.rc.arm_switch = false;
    rig.run(1.2);
    rig.rc.arm_switch = true;
    rig.run(0.5);
    assert!(!rig.out.status.armed);
    // The check now passes, but the switch is still up.
    rig.battery = Some(BatterySample {
        voltage_v: 23.0,
        current_a: 0.0,
    });
    rig.run(1.0);
    assert!(rig.out.status.prearm.is_empty());
    assert!(!rig.out.status.armed, "no retry without a new rising edge");
    assert_eq!(rig.count(|k| matches!(k, EventKind::ArmRefused(_))), 1);
    rig.rc.arm_switch = false;
    rig.run(0.2);
    rig.rc.arm_switch = true;
    rig.run(0.2);
    assert!(rig.out.status.armed);
}

#[test]
fn arming_without_a_position_sets_no_home() {
    // Stabilized needs no position: the EKF has no GPS, arming works, and
    // there is no home.
    let mut rig = Rig::ekf(0);
    rig.gps_on = false;
    rig.arm();
    assert!(rig.saw(EventKind::Armed));
    assert!(!rig.saw(EventKind::HomeSet));
    assert!(!rig.out.status.home_set);
    assert_eq!(rig.core.home(), None);
}

#[test]
fn rc_loss_in_position_holds_then_returns_and_rc_back_changes_nothing() {
    let mut rig = Rig::external(1);
    rig.arm();
    rig.rc_on = false;
    // RC counts as lost after rc_timeout_us (0.5 s); the supervisor runs
    // at 50 Hz.
    assert!(rig.run_until(0.9, |r| r.nav() == NavState::Hold));
    assert!(rig.out.status.reasons.contains(Reasons::RC_LOST));
    assert_eq!(rig.out.status.action, Action::Hold);
    assert!(rig.saw(EventKind::ReasonRaised(Reasons::RC_LOST)));
    assert!(rig.saw(EventKind::ActionChanged {
        from: Action::None,
        to: Action::Hold
    }));
    // Hold becomes Rtl rc_loss_hold_s (3 s) after the reason began.
    assert!(!rig.run_until(2.5, |r| r.nav() == NavState::Rtl));
    assert!(rig.run_until(1.0, |r| r.nav() == NavState::Rtl));
    assert_eq!(rig.out.status.action, Action::Rtl);
    // RC returns: nothing changes, Rtl never drops.
    rig.rc_on = true;
    rig.run(3.0);
    assert!(!rig.out.status.reasons.contains(Reasons::RC_LOST));
    assert_eq!(rig.nav(), NavState::Rtl);
    assert_eq!(rig.count(|k| matches!(k, EventKind::PilotOverride(_))), 0);
}

#[test]
fn a_switch_moved_while_the_link_was_down_counts_once_rc_is_settled() {
    let mut rig = Rig::external(1);
    rig.arm();
    rig.rc_on = false;
    assert!(rig.run_until(5.0, |r| r.nav() == NavState::Rtl));
    rig.rc_on = true;
    rig.rc.mode_switch = 0;
    // RC is back but not settled for a second: the move is not seen yet.
    rig.run(0.8);
    assert_eq!(rig.nav(), NavState::Rtl);
    // Once settled, the move counts as a change: the pilot has the
    // vehicle, in the mode the switch selects.
    assert!(rig.run_until(0.6, |r| r.nav() == NavState::Stabilized));
    assert_eq!(rig.count(|k| matches!(k, EventKind::PilotOverride(_))), 1);
    assert_eq!(rig.out.status.action, Action::None);
    assert_eq!(rig.out.status.mode, FlightMode::Stabilized);
    // A later explicit move is an ordinary mode change.
    rig.rc.mode_switch = 1;
    rig.run(0.3);
    assert_eq!(rig.nav(), NavState::Position);
}

#[test]
fn moving_the_switch_during_a_failsafe_overrides_and_sticks_do_not() {
    let mut rig = Rig::external(2);
    rig.arm();
    rig.run(1.2);
    assert_eq!(rig.nav(), NavState::Offboard);
    // A reason that does not depend on RC: the brain stops.
    rig.brain_frozen = true;
    assert!(rig.run_until(3.0, |r| r.nav() == NavState::Hold));
    rig.rc.roll = 0.5;
    rig.run(0.3);
    assert_eq!(rig.nav(), NavState::Hold, "sticks do nothing");
    rig.rc.mode_switch = 1;
    rig.run(0.1);
    assert_eq!(rig.nav(), NavState::Position);
    assert_eq!(rig.count(|k| matches!(k, EventKind::PilotOverride(_))), 1);
    // The reason is acknowledged: it does not act again.
    rig.run(2.0);
    assert_eq!(rig.nav(), NavState::Position);
}

#[test]
fn rc_loss_on_the_ground_disarms_and_does_not_take_off() {
    let mut rig = Rig::external(0);
    rig.state = Some(StateSample::default());
    rig.arm();
    // Idle for longer than touchdown_s: the vehicle counts as landed.
    rig.run(1.5);
    assert!(rig.out.status.armed);
    rig.rc_on = false;
    let mut max_motor = 0.0_f32;
    let done = rig.run_until(1.5, |r| {
        max_motor = max_motor.max(r.out.motors.iter().copied().fold(0.0, f32::max));
        !r.out.status.armed
    });
    assert!(done, "must disarm");
    rig.run(0.2);
    assert!(rig.saw(EventKind::Disarmed(DisarmReason::Touchdown)));
    assert!(max_motor < 0.1, "motors rose to {max_motor}");
    assert!(!rig.out.status.armed);
    assert_eq!(rig.out.motors, [0.0; 4]);
}

#[test]
fn offboard_setpoints_stopping_gives_hold_and_returning_restores_offboard() {
    let mut rig = Rig::external(2);
    rig.arm();
    rig.run(0.5);
    assert_eq!(rig.nav(), NavState::Offboard);
    let sp = rig.setpoint;
    rig.setpoint = None;
    assert!(rig.run_until(1.0, |r| r.nav() == NavState::Hold));
    assert!(rig.out.status.reasons.contains(Reasons::SETPOINT_STALE));
    rig.setpoint = sp;
    // Back to Offboard after the reason has been clear for clear_hysteresis_s.
    let t0 = rig.k;
    assert!(rig.run_until(2.0, |r| r.nav() == NavState::Offboard));
    let took = (rig.k - t0) as f64 / 1000.0;
    assert!((0.9..1.4).contains(&took), "back after {took} s");
    assert!(rig.saw(EventKind::ReasonCleared(Reasons::SETPOINT_STALE)));
    assert_eq!(rig.out.status.action, Action::None);
}

#[test]
fn a_frozen_brain_holds_then_returns() {
    let mut rig = Rig::external(2);
    rig.arm();
    rig.run(0.5);
    rig.brain_frozen = true;
    // Progress unchanged for brain_timeout_us (2 s).
    let t0 = rig.k;
    assert!(rig.run_until(3.0, |r| r.nav() == NavState::Hold));
    let after = (rig.k - t0) as f64 / 1000.0;
    assert!((1.9..2.4).contains(&after), "BRAIN_LOST after {after} s");
    assert!(rig.out.status.reasons.contains(Reasons::BRAIN_LOST));
    // brain_loss_rtl_s (10 s) later: Rtl.
    let t1 = rig.k;
    assert!(rig.run_until(11.0, |r| r.nav() == NavState::Rtl));
    let later = (rig.k - t1) as f64 / 1000.0;
    assert!((9.5..10.5).contains(&later), "Rtl after {later} s more");
}

#[test]
fn gps_loss_in_position_with_rc_degrades_to_altitude_without_a_failsafe() {
    let mut rig = Rig::ekf(1);
    rig.arm();
    assert_eq!(rig.nav(), NavState::Position);
    rig.gps_on = false;
    assert!(rig.run_until(4.0, |r| r.nav() == NavState::Altitude));
    assert!(rig.saw(EventKind::ModeDegraded {
        requested: FlightMode::Position,
        flown: NavState::Altitude,
    }));
    assert_eq!(rig.out.status.action, Action::None);
    assert!(rig.out.status.reasons.is_empty());
    // Stays so with GPS gone for good (velocity goes invalid too).
    rig.run(8.0);
    assert_eq!(rig.nav(), NavState::Altitude);
    assert_eq!(rig.out.status.action, Action::None);
    assert!(rig.out.status.valid.attitude && rig.out.status.valid.altitude);
    assert!(!rig.out.status.valid.position && !rig.out.status.valid.velocity);
    assert_eq!(
        rig.count(|k| matches!(k, EventKind::ModeDegraded { .. })),
        1,
        "reported once"
    );
}

#[test]
fn gps_loss_in_offboard_holds_then_lands() {
    let mut rig = Rig::ekf(2);
    rig.arm();
    assert_eq!(rig.nav(), NavState::Offboard);
    rig.gps_on = false;
    assert!(rig.run_until(4.0, |r| r.nav() == NavState::Hold));
    assert!(rig.out.status.reasons.contains(Reasons::GPS_LOST));
    assert_eq!(rig.out.status.action, Action::Hold);
    // gps_loss_land_s (30 s) after the reason began.
    assert!(!rig.run_until(27.0, |r| r.nav() == NavState::Land));
    assert!(rig.run_until(6.0, |r| r.nav() == NavState::Land));
    assert_eq!(rig.out.status.action, Action::Land);
}

#[test]
fn imu_stopping_while_armed_terminates_and_blocks_arming_until_released() {
    let mut rig = Rig::ekf(1);
    rig.arm();
    rig.imu_on = false;
    assert!(rig.run_until(1.0, |r| r.nav() == NavState::Terminated));
    assert!(rig.saw(EventKind::Disarmed(DisarmReason::Terminated)));
    rig.run(0.1);
    assert!(!rig.out.status.armed);
    assert_eq!(rig.out.motors, [0.0; 4]);
    assert!(rig.out.status.prearm.contains(PrearmFlags::KILL));
    assert_eq!(rig.nav(), NavState::Terminated);
    // The IMU returns; the switch is still up: blocked, still terminated.
    rig.imu_on = true;
    rig.run(4.0);
    assert!(rig.out.status.prearm.contains(PrearmFlags::KILL));
    assert_eq!(rig.nav(), NavState::Terminated);
    assert!(!rig.out.status.armed);
    // Releasing the arm switch clears the latch.
    rig.rc.arm_switch = false;
    rig.run(0.2);
    assert!(!rig.out.status.prearm.contains(PrearmFlags::KILL));
    assert_eq!(rig.nav(), NavState::Disarmed);
}

#[test]
fn land_now_on_the_ground_touches_down_and_disarms() {
    // A critical battery lands now. The vehicle sits still on the ground,
    // so the command winds down and the touchdown condition then has to
    // hold for touchdown_s.
    let mut rig = Rig::external(1);
    rig.state = Some(StateSample::default());
    rig.arm();
    rig.battery = Some(BatterySample {
        voltage_v: 19.0,
        current_a: 0.0,
    });
    assert!(rig.run_until(10.0, |r| r.nav() == NavState::LandNow));
    assert!(rig.out.status.reasons.contains(Reasons::BATTERY_CRITICAL));
    assert!(rig.run_until(40.0, |r| !r.out.status.armed));
    assert!(rig.saw(EventKind::Touchdown));
    assert!(rig.saw(EventKind::Disarmed(DisarmReason::Touchdown)));
    assert_eq!(rig.out.motors, [0.0; 4]);
}

#[test]
fn land_after_gps_loss_touches_down() {
    let mut rig = Rig::ekf(2);
    rig.arm();
    rig.gps_on = false;
    assert!(rig.run_until(40.0, |r| r.nav() == NavState::Land));
    assert!(rig.run_until(60.0, |r| !r.out.status.armed));
    assert!(rig.saw(EventKind::Touchdown));
    assert!(rig.saw(EventKind::Disarmed(DisarmReason::Touchdown)));
}

fn set_pos(rig: &mut Rig, x: f32, y: f32, z: f32) {
    rig.state = Some(StateSample {
        position: Vec3::new(x, y, z),
        ..StateSample::default()
    });
}

fn force_rtl(rig: &mut Rig) {
    rig.rc_on = false;
    assert!(rig.run_until(6.0, |r| r.nav() == NavState::Rtl));
}

#[test]
fn rtl_climbs_returns_and_lands_over_home() {
    let mut rig = Rig::external(1);
    set_pos(&mut rig, 0.0, 0.0, -1.0);
    rig.arm();
    assert_eq!(rig.core.home(), Some(Vec3::new(0.0, 0.0, -1.0)));
    // Fly away 30 m, then lose the radio.
    set_pos(&mut rig, 30.0, 0.0, -1.0);
    rig.run(0.5);
    force_rtl(&mut rig);
    rig.run(0.1);
    let g = rig.core.guide_state;
    assert_eq!(g.rtl, RtlPhase::Climb);
    // rtl_altitude_m (10 m) above home: NED z = -1 - 10.
    assert!((g.position_sp.z + 11.0).abs() < 1e-4, "{:?}", g.position_sp);
    assert!((g.position_sp.x - 30.0).abs() < 1e-4);
    // Not there yet: still climbing.
    rig.run(0.5);
    assert_eq!(rig.core.guide_state.rtl, RtlPhase::Climb);
    // Reaching the height starts the return at rtl_speed_mps (3 m/s).
    set_pos(&mut rig, 30.0, 0.0, -11.0);
    rig.run(0.1);
    assert_eq!(rig.core.guide_state.rtl, RtlPhase::Return);
    // The vehicle follows the setpoint exactly; guidance runs at 50 Hz, so
    // 3 m/s is 0.06 m per run.
    let mut prev = rig.core.guide_state.position_sp.x;
    let mut runs = 0;
    while rig.core.guide_state.rtl == RtlPhase::Return && runs < 2000 {
        let sp = rig.core.guide_state.position_sp;
        set_pos(&mut rig, sp.x, sp.y, -11.0);
        rig.run(0.02);
        let now = rig.core.guide_state.position_sp.x;
        if rig.core.guide_state.rtl == RtlPhase::Return && now > 0.1 {
            assert!(
                ((prev - now) - 0.06).abs() < 2e-3,
                "step {} at x = {now}",
                prev - now
            );
        }
        prev = now;
        runs += 1;
    }
    // 30 m at 3 m/s is 10 s = 500 runs (plus a few for the first settle).
    assert!((480..560).contains(&runs), "{runs} runs");
    assert_eq!(rig.core.guide_state.rtl, RtlPhase::Landing);
    let g = rig.core.guide_state;
    assert!(g.position_sp.x.abs() < 1e-4 && g.position_sp.y.abs() < 1e-4);
    assert_eq!(rig.nav(), NavState::Rtl);
    // Landing descends: the vertical command is a velocity, so the thrust
    // is below hover while the (scripted, stationary) vehicle is not moving.
    rig.run(1.0);
    assert!(rig.core.guidance.pending.thrust_n < rig.core.config().weight_n());
}

#[test]
fn rtl_within_one_metre_of_home_skips_the_climb() {
    let mut rig = Rig::external(1);
    set_pos(&mut rig, 0.0, 0.0, -2.0);
    rig.arm();
    set_pos(&mut rig, 0.6, 0.0, -2.0);
    rig.run(0.2);
    force_rtl(&mut rig);
    rig.run(0.1);
    assert_eq!(rig.core.guide_state.rtl, RtlPhase::Landing);
}

#[test]
fn rtl_without_a_position_behaves_as_hold() {
    let mut rig = Rig::ekf(1);
    rig.arm();
    assert!(rig.core.home().is_some());
    let height = rig.core.estimate().expect("estimate").position.z;
    rig.gps_on = false;
    rig.rc_on = false;
    assert!(rig.run_until(8.0, |r| r.nav() == NavState::Rtl));
    rig.run(1.0);
    // No climb, no return: the phase never starts and the height is held.
    assert_eq!(rig.core.guide_state.rtl, RtlPhase::Pending);
    assert!((rig.core.guide_state.position_sp.z - height).abs() < 1.0);
}

#[test]
fn altitude_mode_holds_height_climbs_with_throttle_and_tilts_with_sticks() {
    let mut rig = Rig::ekf(1);
    rig.arm();
    rig.gps_on = false;
    assert!(rig.run_until(4.0, |r| r.nav() == NavState::Altitude));
    let weight = rig.core.config().weight_n();
    // Throttle centred: the height setpoint holds, the thrust is near hover.
    rig.rc.throttle = 0.5;
    rig.run(0.5);
    let hold_sp = rig.core.guide_state.position_sp.z;
    rig.run(0.5);
    assert_eq!(rig.core.guide_state.position_sp.z, hold_sp);
    let thrust = rig.core.guidance.pending.thrust_n;
    assert!((thrust / weight - 1.0).abs() < 0.2, "thrust {thrust}");
    // Throttle high: a climb is commanded and the setpoint follows.
    rig.rc.throttle = 0.9;
    rig.run(0.3);
    let thrust = rig.core.guidance.pending.thrust_n;
    assert!(thrust > 1.1 * weight, "climb thrust {thrust} vs {weight}");
    let z = rig.core.estimate().expect("estimate").position.z;
    assert!((rig.core.guide_state.position_sp.z - z).abs() < 1e-4);
    // Throttle low: descent.
    rig.rc.throttle = 0.1;
    rig.run(0.3);
    let thrust = rig.core.guidance.pending.thrust_n;
    assert!(thrust < 0.9 * weight, "descent thrust {thrust}");
    // Roll stick: tilt follows, scaled by max_tilt_rad.
    rig.rc.throttle = 0.5;
    rig.rc.roll = 0.5;
    rig.run(0.2);
    let tilt = tilt_angle(rig.core.guidance.pending.attitude);
    let want = 0.5 * rig.core.config().limits.max_tilt_rad;
    assert!((tilt - want).abs() < 0.01, "tilt {tilt}, wanted {want}");
}

#[test]
fn events_beyond_the_per_tick_limit_arrive_later_in_order() {
    // Arming into Offboard with no brain and no setpoints raises several
    // events on one run, and the failsafe disarms at once (the vehicle is
    // on the ground).
    let mut rig = Rig::external(2);
    rig.state = Some(StateSample::default());
    rig.brain_on = false;
    rig.setpoint = None;
    rig.rc.arm_switch = false;
    rig.run(1.2);
    rig.rc.arm_switch = true;
    let mut per_tick = Vec::new();
    for _ in 0..60 {
        rig.step();
        per_tick.push(rig.out.events.len());
    }
    assert!(per_tick.iter().all(|n| *n <= EVENTS_PER_TICK));
    assert!(per_tick.contains(&EVENTS_PER_TICK), "{per_tick:?}");
    let kinds: Vec<_> = rig.events.iter().map(|e| e.kind).collect();
    assert!(
        kinds.len() > EVENTS_PER_TICK,
        "need a burst larger than one tick: {kinds:?}"
    );
    assert_eq!(kinds.first(), Some(&EventKind::Armed));
    assert_eq!(kinds.get(1), Some(&EventKind::HomeSet));
    assert!(matches!(
        kinds.last(),
        Some(EventKind::Disarmed(DisarmReason::Touchdown))
    ));
    // All stamped with the tick time of the run that produced them.
    let t = rig.events.first().map(|e| e.t);
    assert!(rig.events.iter().all(|e| Some(e.t) == t));
    assert_eq!(rig.core.events_dropped(), 0);
}

#[test]
fn event_queue_overflow_drops_the_oldest_and_counts() {
    let mut core = Core::new(cfg(), F405_LEAN).expect("valid config");
    for i in 0..40u64 {
        core.emit(Micros(i), EventKind::Armed);
    }
    assert_eq!(core.events_dropped(), 8);
    // The 32 newest survive, oldest first, four per tick.
    let mut times = Vec::new();
    for k in 0..10u64 {
        let out = core.tick(Micros(k * 1000), &Inputs::default());
        times.extend(out.events.iter().map(|e| e.t.0));
    }
    assert_eq!(times, (8..40).collect::<Vec<u64>>());
}
