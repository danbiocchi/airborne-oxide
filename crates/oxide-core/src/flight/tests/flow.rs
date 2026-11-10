// SPDX-License-Identifier: AGPL-3.0-or-later
//! Failsafes, navigation states and arming through the whole core
//! (ADR-0003).

use super::*;
use crate::flight::guidance::RtlPhase;
use crate::io::DisarmReason;
use crate::math::Quat;
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
    baro_on: bool,
    mag_on: bool,
    gcs_on: bool,
    /// Feed one absurd (finite, but blowing up the covariance) accelerometer sample at the next step.
    imu_glitch: bool,
    imu_on: bool,
    /// After arming, centre the throttle of a rig that hovers.
    hover_after_arm: bool,
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
            baro_on: true,
            mag_on: true,
            gcs_on: false,
            imu_glitch: false,
            imu_on: true,
            hover_after_arm: true,
            events: Vec::new(),
            out: Outputs::off(Micros::ZERO),
        }
    }

    /// External state, hovering at 5 m.
    /// The same rig with another configuration.
    fn with_cfg(mut self, c: VehicleConfig<4>) -> Self {
        self.core = Core::new(c, F405_LEAN).expect("valid config");
        self
    }

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
        if !self.baro_on {
            i.baro = None;
        }
        if !self.mag_on {
            i.mag = None;
        }
        if self.gcs_on && self.k.is_multiple_of(100) {
            i.gcs_heartbeat = Some(Stamped::ideal(t, GcsHeartbeat));
        }
        if self.imu_glitch {
            self.imu_glitch = false;
            if let Some(imu) = i.imu.as_mut() {
                imu.value.accel = Vec3::new(3.0e38, 0.0, 0.0);
            }
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
        // Airborne rigs hover: a centred throttle stick. (Low throttle in
        // Position or Altitude is a commanded descent, which on a
        // motionless state means the ground.)
        if self.hover_after_arm && !self.ekf && self.state.is_some_and(|s| s.position.z < -0.5) {
            self.rc.throttle = 0.5;
            self.run(0.6);
        }
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
    rig.hover_after_arm = false;
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
    // LINK: Offboard with no brain heartbeat and no setpoints.
    let mut rig = Rig::external(2);
    rig.brain_on = false;
    rig.setpoint = None;
    refused_with(rig, PrearmFlags::LINK, 1.2);
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
    // Throttle centred: a hold at the ground, not a commanded descent.
    rig.rc.throttle = 0.5;
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
    // Hovering, not sitting on the ground: a failsafe would disarm at once.
    rig.rc.throttle = 0.5;
    rig.run(1.0);
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
    // A burst of six events raised on one run (arming into an active
    // failsafe used to do this; arming is now refused, see LINK).
    let mut rig = Rig::external(2);
    rig.rc.arm_switch = false;
    rig.run(1.2);
    rig.events.clear();
    let burst = [
        EventKind::Armed,
        EventKind::HomeSet,
        EventKind::ReasonRaised(Reasons::BRAIN_LOST),
        EventKind::ReasonRaised(Reasons::SETPOINT_STALE),
        EventKind::ActionChanged {
            from: Action::None,
            to: Action::Hold,
        },
        EventKind::Disarmed(DisarmReason::Touchdown),
    ];
    let t0 = Micros(rig.k * 1000);
    for kind in burst {
        rig.core.emit(t0, kind);
    }
    let mut per_tick = Vec::new();
    for _ in 0..10 {
        rig.step();
        per_tick.push(rig.out.events.len());
    }
    assert!(per_tick.iter().all(|n| *n <= EVENTS_PER_TICK));
    assert!(per_tick.contains(&EVENTS_PER_TICK), "{per_tick:?}");
    let kinds: Vec<_> = rig.events.iter().map(|e| e.kind).collect();
    assert_eq!(kinds, burst, "all arrive, in order");
    // All stamped with the time of the run that produced them.
    assert!(rig.events.iter().all(|e| e.t == t0));
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

#[test]
fn rc_loss_on_the_ground_in_position_mode_disarms_without_lifting() {
    // Position with the throttle stick down commands a slow descent, so
    // the idle thrust sits above the touchdown threshold.
    let mut rig = Rig::external(1);
    rig.state = Some(StateSample::default());
    rig.arm();
    rig.run(1.0);
    assert!(rig.out.status.armed);
    rig.rc_on = false;
    let mut max_motor = 0.0_f32;
    let done = rig.run_until(1.5, |r| {
        max_motor = max_motor.max(r.out.motors.iter().copied().fold(0.0, f32::max));
        !r.out.status.armed
    });
    assert!(done, "must disarm, nav {:?}", rig.nav());
    rig.run(0.2);
    assert!(rig.saw(EventKind::Disarmed(DisarmReason::Touchdown)));
    // The minimum thrust (0.1 x weight) is a motor command of about 0.14
    // with the square-root thrust curve: not lift.
    assert!(max_motor < 0.15, "motors rose to {max_motor}");
    assert_eq!(rig.out.motors, [0.0; 4]);
}

#[test]
fn rc_loss_soon_after_arming_on_the_ground_disarms_without_lifting() {
    // The touchdown detector has not held for touchdown_s yet.
    let mut rig = Rig::external(0);
    rig.state = Some(StateSample::default());
    rig.arm();
    rig.rc_on = false;
    let mut max_motor = 0.0_f32;
    let done = rig.run_until(1.5, |r| {
        max_motor = max_motor.max(r.out.motors.iter().copied().fold(0.0, f32::max));
        !r.out.status.armed
    });
    assert!(done, "must disarm, nav {:?}", rig.nav());
    assert!(rig.saw(EventKind::Disarmed(DisarmReason::Touchdown)));
    assert!(max_motor < 0.1, "motors rose to {max_motor}");
}

#[test]
fn position_lost_during_the_rtl_landing_phase_holds_height_instead_of_descending() {
    // ADR-0003 §2: Rtl without a position behaves as Hold, also when the
    // position goes away part-way through the landing. A long touchdown
    // time keeps the (motionless) rig from disarming first.
    let mut c = ekf_cfg();
    c.failsafe.touchdown_s = 8.0;
    let mut rig = Rig::ekf(1);
    rig.core = Core::new(c, F405_LEAN).expect("valid config");
    rig.arm();
    rig.rc.throttle = 0.5;
    rig.run(1.0);
    rig.rc_on = false;
    assert!(rig.run_until(6.0, |r| r.nav() == NavState::Rtl));
    rig.run(0.1);
    assert_eq!(rig.core.guide_state.rtl, RtlPhase::Landing);
    rig.gps_on = false;
    assert!(rig.run_until(4.0, |r| !r.out.status.valid.position));
    rig.run(1.0);
    let mut min_motor = 1.0_f32;
    rig.run_until(3.0, |r| {
        min_motor = min_motor.min(r.out.motors.iter().copied().fold(1.0, f32::min));
        false
    });
    assert!(rig.out.status.armed, "must keep holding, not touch down");
    assert_eq!(rig.nav(), NavState::Rtl);
    assert!(min_motor > 0.3, "hover thrust expected, motors {min_motor}");
}

#[test]
fn arming_into_offboard_with_a_dead_brain_is_refused_without_a_burst_of_events() {
    let mut rig = Rig::external(2);
    rig.brain_on = false;
    rig.setpoint = None;
    rig.rc.arm_switch = false;
    rig.run(1.2);
    rig.rc.arm_switch = true;
    rig.run(0.5);
    assert!(!rig.out.status.armed);
    assert!(rig.out.status.prearm.contains(PrearmFlags::LINK));
    assert_eq!(rig.count(|k| matches!(k, EventKind::ArmRefused(_))), 1);
    for kind in [
        EventKind::Armed,
        EventKind::HomeSet,
        EventKind::Disarmed(DisarmReason::Touchdown),
    ] {
        assert!(!rig.saw(kind), "{kind:?}");
    }
    assert_eq!(rig.count(|k| matches!(k, EventKind::ReasonRaised(_))), 0);
}

#[test]
fn a_fully_blind_landing_eventually_drops_thrust_and_disarms() {
    // No GPS and no barometer: the landing flies a fixed thrust. It must
    // not stay armed at 0.9 x hover for ever.
    let mut rig = Rig::ekf(1);
    rig.arm();
    rig.rc.throttle = 0.5;
    rig.run(1.0);
    rig.gps_on = false;
    rig.baro_on = false;
    assert!(rig.run_until(15.0, |r| r.nav() == NavState::LandNow));
    assert!(!rig.out.status.valid.altitude && !rig.out.status.valid.velocity);
    // Still flying the blind thrust well into the descent…
    rig.run(30.0);
    assert!(rig.out.status.armed);
    assert!(
        rig.out.motors.iter().all(|m| *m > 0.3),
        "{:?}",
        rig.out.motors
    );
    // …then the thrust drops and the touchdown is detected.
    assert!(rig.run_until(60.0, |r| !r.out.status.armed));
    assert!(rig.saw(EventKind::Disarmed(DisarmReason::Touchdown)));
}

#[test]
fn home_is_set_at_the_first_position_after_arming_without_one() {
    let mut rig = Rig::ekf(0);
    rig.gps_on = false;
    rig.arm();
    // Not sitting on the ground: a failsafe would disarm at once.
    rig.rc.throttle = 0.5;
    // Let the position validity lapse, then GPS comes back.
    rig.run(3.0);
    assert!(!rig.out.status.valid.position);
    assert_eq!(rig.core.home(), None);
    rig.gps_on = true;
    assert!(rig.run_until(5.0, |r| r.out.status.valid.position));
    rig.run(0.1);
    assert!(rig.out.status.home_set);
    assert_eq!(rig.count(|k| matches!(k, EventKind::HomeSet)), 1);
    // It stays: a later position change does not move it.
    let home = rig.core.home();
    assert!(home.is_some());
    rig.run(1.0);
    assert_eq!(rig.core.home(), home);
    // RC loss now returns instead of holding for ever.
    rig.rc_on = false;
    assert!(rig.run_until(6.0, |r| r.nav() == NavState::Rtl));
}

/// A hovering external-state rig (5 m, throttle centred, armed).
fn hovering(mode_switch: u8, c: VehicleConfig<4>) -> Rig {
    let mut rig = Rig::external(mode_switch).with_cfg(c);
    rig.arm();
    rig.run(0.5);
    assert!(rig.out.status.armed);
    rig
}

fn tilted(roll_deg: f32) -> StateSample {
    StateSample {
        attitude: Quat::from_euler(roll_deg.to_radians(), 0.0, 0.0),
        ..at_height(5.0)
    }
}

#[test]
fn critical_battery_acknowledged_limits_the_pilot_to_altitude_or_stabilized() {
    let mut rig = hovering(1, cfg());
    rig.battery = Some(BatterySample {
        voltage_v: 19.0,
        current_a: 0.0,
    });
    assert!(rig.run_until(10.0, |r| r.nav() == NavState::LandNow));
    // The pilot moves the switch to Offboard: they take over, but a
    // restricted pilot gets Altitude, not the mode asked for.
    rig.rc.mode_switch = 2;
    assert!(rig.run_until(1.0, |r| r.out.status.action == Action::None));
    assert!(rig.events.iter().any(|e| matches!(
        e.kind,
        EventKind::PilotOverride(r) if r.contains(Reasons::BATTERY_CRITICAL)
    )));
    assert_eq!(rig.nav(), NavState::Altitude);
    assert!(rig.saw(EventKind::ModeDegraded {
        requested: FlightMode::Offboard,
        flown: NavState::Altitude
    }));
    rig.run(1.0);
    assert_eq!(rig.nav(), NavState::Altitude, "stays restricted");
    // Stabilized is allowed.
    rig.rc.mode_switch = 0;
    rig.run(1.0);
    assert_eq!(rig.nav(), NavState::Stabilized);
}

#[test]
fn geofence_breach_returns_and_needs_home_and_a_limit() {
    let mut c = cfg();
    c.failsafe.fence_radius_m = 10.0;
    c.failsafe.fence_altitude_m = 20.0;
    // Inside both limits: nothing.
    let mut rig = hovering(1, c);
    set_pos(&mut rig, 9.0, 0.0, -19.0);
    rig.run(1.0);
    assert_eq!(rig.nav(), NavState::Position);
    assert!(!rig.out.status.reasons.contains(Reasons::GEOFENCE));
    // Too far (home is the arming point: 0, 0, -5).
    set_pos(&mut rig, 11.0, 0.0, -5.0);
    assert!(rig.run_until(1.0, |r| r.nav() == NavState::Rtl));
    assert!(rig.out.status.reasons.contains(Reasons::GEOFENCE));
    // Too high, on a fresh vehicle.
    let mut c = cfg();
    c.failsafe.fence_radius_m = 10.0;
    c.failsafe.fence_altitude_m = 20.0;
    let mut rig = hovering(1, c);
    set_pos(&mut rig, 0.0, 0.0, -26.0);
    assert!(rig.run_until(1.0, |r| r.nav() == NavState::Rtl));
    assert!(rig.out.status.reasons.contains(Reasons::GEOFENCE));
    // Zero disables both limits.
    let mut rig = hovering(1, cfg());
    set_pos(&mut rig, 500.0, 0.0, -300.0);
    rig.run(2.0);
    assert_eq!(rig.nav(), NavState::Position);
    // No position, no geofence: Altitude-only flight far from home.
    let mut c = ekf_cfg();
    c.failsafe.fence_radius_m = 0.5;
    let mut rig = Rig::ekf(0).with_cfg(c);
    rig.gps_on = false;
    rig.arm();
    rig.rc.throttle = 0.5;
    rig.run(8.0);
    assert!(!rig.out.status.valid.position);
    assert!(!rig.out.status.reasons.contains(Reasons::GEOFENCE));
}

#[test]
fn loss_of_control_terminates_in_the_air_but_not_on_the_ground() {
    // Airborne: tilt of 80 degrees for longer than 0.25 s.
    let mut rig = hovering(1, cfg());
    rig.state = Some(tilted(80.0));
    assert!(rig.run_until(1.0, |r| r.nav() == NavState::Terminated));
    assert!(rig.out.status.reasons.contains(Reasons::LOSS_OF_CONTROL));
    assert!(rig.saw(EventKind::Disarmed(DisarmReason::Terminated)));
    rig.run(0.1);
    assert_eq!(rig.out.motors, [0.0; 4]);
    // On the ground (landed): the same attitude is a vehicle tipped over,
    // not a loss of control in flight.
    let mut rig = Rig::external(0);
    rig.state = Some(StateSample::default());
    rig.arm();
    rig.run(1.5);
    rig.state = Some(StateSample {
        attitude: Quat::from_euler(80.0_f32.to_radians(), 0.0, 0.0),
        ..StateSample::default()
    });
    rig.run(2.0);
    assert!(rig.out.status.armed);
    assert_ne!(rig.nav(), NavState::Terminated);
    assert!(!rig.out.status.reasons.contains(Reasons::LOSS_OF_CONTROL));
}

#[test]
fn offboard_with_no_human_link_returns_after_the_countdown_unless_the_gcs_is_alive() {
    // RC lost alone changes nothing in Offboard, and the ground station
    // being alive keeps it that way.
    let mut rig = hovering(2, cfg());
    rig.gcs_on = true;
    rig.run(2.0);
    rig.rc_on = false;
    rig.run(65.0);
    assert_eq!(rig.nav(), NavState::Offboard);
    assert!(!rig.out.status.reasons.contains(Reasons::NO_HUMAN_LINK));
    // The ground station goes too: no_link_rtl_s (60 s) later, Rtl.
    rig.gcs_on = false;
    assert!(rig.run_until(5.0, |r| {
        r.out.status.reasons.contains(Reasons::NO_HUMAN_LINK)
    }));
    assert_eq!(rig.nav(), NavState::Offboard);
    rig.run(55.0);
    assert_eq!(rig.nav(), NavState::Offboard);
    assert!(rig.run_until(10.0, |r| r.nav() == NavState::Rtl));
    // A ground station that never connected counts as absent.
    let mut rig = hovering(2, cfg());
    rig.rc_on = false;
    assert!(!rig.run_until(55.0, |r| r.nav() == NavState::Rtl));
    assert!(rig.run_until(10.0, |r| r.nav() == NavState::Rtl));
}

#[test]
fn losing_baro_or_mag_makes_the_estimator_unhealthy_and_lands_now() {
    let mut rig = Rig::ekf(1);
    rig.arm();
    rig.rc.throttle = 0.5;
    rig.run(1.0);
    rig.baro_on = false;
    assert!(rig.run_until(10.0, |r| r.nav() == NavState::LandNow));
    assert!(!rig.out.status.valid.altitude);
    assert!(
        rig.out
            .status
            .reasons
            .contains(Reasons::ESTIMATOR_UNHEALTHY)
    );
    // Heading: the magnetometer times out after 30 s.
    let mut rig = Rig::ekf(1);
    rig.arm();
    rig.rc.throttle = 0.5;
    rig.run(1.0);
    rig.mag_on = false;
    rig.run(25.0);
    assert_eq!(rig.nav(), NavState::Position);
    // (A static rig counts as sitting on the ground, so the landing ends in
    // a touchdown at once: look at the events.)
    rig.run(12.0);
    assert!(!rig.out.status.valid.heading);
    assert!(rig.saw(EventKind::ReasonRaised(Reasons::ESTIMATOR_UNHEALTHY)));
    assert!(rig.saw(EventKind::ActionChanged {
        from: Action::None,
        to: Action::LandNow
    }));
}

#[test]
fn an_estimator_fault_in_flight_lands_now_but_one_before_arming_does_not() {
    // A fault before arming is the baseline.
    let mut rig = Rig::ekf(0);
    rig.run(1.5);
    rig.imu_glitch = true;
    rig.run(1.5);
    let faults = rig.core.estimator.faults();
    assert!(faults > 0, "the glitch must register as a fault");
    rig.arm();
    rig.rc.throttle = 0.5;
    rig.run(3.0);
    assert!(rig.out.status.armed);
    assert_eq!(rig.nav(), NavState::Stabilized);
    assert!(
        !rig.out
            .status
            .reasons
            .contains(Reasons::ESTIMATOR_UNHEALTHY)
    );
    // One in flight.
    rig.imu_glitch = true;
    assert!(rig.run_until(2.0, |r| {
        r.out.status.reasons.contains(Reasons::ESTIMATOR_UNHEALTHY)
    }));
    assert!(rig.run_until(1.0, |r| r.nav() == NavState::LandNow));
}

#[test]
fn home_is_forgotten_on_disarm_kept_in_flight_and_set_again_on_rearming() {
    let mut rig = Rig::external(1);
    rig.arm();
    let first = Vec3::new(0.0, 0.0, -5.0);
    assert_eq!(rig.core.home(), Some(first));
    // Flying elsewhere does not move it.
    set_pos(&mut rig, 4.0, 3.0, -6.0);
    rig.run(1.0);
    assert_eq!(rig.core.home(), Some(first));
    assert_eq!(rig.count(|k| matches!(k, EventKind::HomeSet)), 1);
    // Disarm forgets it.
    rig.rc.arm_switch = false;
    rig.run(0.3);
    assert!(!rig.out.status.armed);
    assert_eq!(rig.core.home(), None);
    assert!(!rig.out.status.home_set);
    // Re-arming sets the new place.
    rig.rc.throttle = 0.0;
    rig.hover_after_arm = false;
    rig.arm();
    assert_eq!(rig.core.home(), Some(Vec3::new(4.0, 3.0, -6.0)));
    assert_eq!(rig.count(|k| matches!(k, EventKind::HomeSet)), 2);
}
