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
    /// East coordinate of every GPS fix in EKF mode, m.
    gps_east: f32,
    baro_on: bool,
    mag_on: bool,
    /// Field every magnetometer sample reads in EKF mode, µT (body frame).
    mag_field: Vec3<f32>,
    gcs_on: bool,
    /// Feed one absurd (finite, but blowing up the covariance) accelerometer sample at the next step.
    imu_glitch: bool,
    imu_on: bool,
    /// After arming, centre the throttle of a rig that hovers.
    hover_after_arm: bool,
    /// A responsive vertical world: the vehicle accelerates as the motor
    /// commands say (external state: the state's vertical speed and height
    /// follow; EKF: the accelerometer reads the thrust). Off by default, so
    /// the state stays where the test put it.
    world: bool,
    /// Extra accelerometer reading on the down axis in `world` EKF mode, m/s^2.
    accel_bias_z: f32,
    mass_kg: f32,
    max_thrust_n: f32,
    nominal_v: f32,
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
            gps_east: 0.0,
            baro_on: true,
            mag_on: true,
            mag_field: Vec3::new(25.0, 0.0, 43.3),
            gcs_on: false,
            imu_glitch: false,
            imu_on: true,
            hover_after_arm: true,
            world: false,
            accel_bias_z: 0.0,
            mass_kg: cfg.mass_kg,
            max_thrust_n: cfg.rotors[0].max_thrust_n,
            nominal_v: cfg.nominal_voltage_v,
            events: Vec::new(),
            out: Outputs::off(Micros::ZERO),
        }
    }

    /// External state, hovering at 5 m.
    /// The same rig with another configuration.
    fn with_cfg(mut self, c: VehicleConfig<4>) -> Self {
        self.mass_kg = c.mass_kg;
        self.max_thrust_n = c.rotors[0].max_thrust_n;
        self.nominal_v = c.nominal_voltage_v;
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

    /// Total rotor thrust of the last motor commands, N (square-law rotors,
    /// full expo, at the rig's battery voltage).
    fn thrust_n(&self) -> f32 {
        let v = self.battery.map_or(self.nominal_v, |b| b.voltage_v);
        let k = (v / self.nominal_v) * (v / self.nominal_v);
        self.out
            .motors
            .iter()
            .map(|u| self.max_thrust_n * u * u * k)
            .sum()
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
        if self.world
            && self.ekf
            && let Some(imu) = i.imu.as_mut()
        {
            imu.value.accel = Vec3::new(
                0.0,
                0.0,
                -self.thrust_n() / self.mass_kg + self.accel_bias_z,
            );
        }
        i.rc = self.rc_on.then(|| Stamped::ideal(t, self.rc));
        i.battery = self.battery.map(|b| Stamped::ideal(t, b));
        if let Some(g) = i.gps.as_mut() {
            g.value.position.y = self.gps_east;
        }
        if !self.gps_on {
            i.gps = None;
        }
        if !self.baro_on {
            i.baro = None;
        }
        if let Some(m) = i.mag.as_mut() {
            m.value.field_ut = self.mag_field;
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
        if self.world && !self.ekf {
            let a = crate::GRAVITY - self.thrust_n() / self.mass_kg;
            let thrust = self.thrust_n();
            if let Some(s) = self.state.as_mut() {
                s.velocity.z += a * 1.0e-3;
                s.position.z += s.velocity.z * 1.0e-3;
                if s.position.z >= 0.0 && thrust < self.mass_kg * crate::GRAVITY {
                    s.position.z = 0.0;
                    s.velocity.z = 0.0;
                }
            }
        }
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
    rig.world = true;
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
    // hold for touchdown_s. The low level is set equal to the critical one
    // (the configuration refuses a critical level above the low one) so that
    // both latch together and LandNow, not the low-battery return, is in
    // force: this test is about LandNow.
    let mut c = cfg();
    c.failsafe.battery_low_v = c.failsafe.battery_critical_v;
    let mut rig = Rig::external(0).with_cfg(c);
    rig.state = Some(StateSample::default());
    rig.arm();
    // Stabilized with the throttle held up: the vehicle is not on the ground
    // as far as the pilot-state ground rule is concerned (its thrust is not
    // low), until LandNow takes it down.
    rig.rc.throttle = 0.3;
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
    // The accelerometer reads the thrust the core commands, so the
    // inertial vertical speed responds to the descent it flies.
    rig.world = true;
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
    rig.world = true;
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

// ---- Ground contact by motion (guidance.rs, ADR-0003 §2) ----

/// Lowest motor command: the minimum thrust (0.1 x weight) is a command of
/// about 0.14; hover is about 0.5.
const CUT: f32 = 0.2;

fn min_motor(rig: &Rig) -> f32 {
    rig.out.motors.iter().copied().fold(f32::MAX, f32::min)
}

/// A Stabilized rig at 5 m with the throttle up (so it does not count as on
/// the ground) and a critical battery, run until LandNow starts. The low
/// level equals the critical one (it may not be lower), so both latch
/// together and LandNow is in force.
fn critical_battery_rig() -> Rig {
    let mut c = cfg();
    c.failsafe.battery_low_v = c.failsafe.battery_critical_v;
    let mut rig = Rig::external(0).with_cfg(c);
    rig.arm();
    rig.rc.throttle = 0.3;
    rig.battery = Some(BatterySample {
        voltage_v: 19.0,
        current_a: 0.0,
    });
    assert!(rig.run_until(10.0, |r| r.nav() == NavState::LandNow));
    rig
}

#[test]
fn contact_is_declared_on_a_still_vehicle_whatever_the_horizontal_error() {
    let mut rig = critical_battery_rig();
    // The state does not respond to the descent, and the held point is 5 m
    // away: the position loop demands tilt and thrust, which says nothing
    // about contact.
    if let Some(s) = rig.state.as_mut() {
        s.position.x += 5.0;
    }
    assert!(rig.run_until(0.4, |r| min_motor(r) > CUT), "not yet");
    assert!(
        rig.run_until(1.0, |r| min_motor(r) < CUT),
        "no contact: motors {:?}",
        rig.out.motors
    );
    // The attitude target is level, not leaning towards the held point.
    let down = rig
        .core
        .guidance
        .visible
        .attitude
        .rotate(Vec3::new(0.0, 0.0, 1.0));
    assert!(down.x.abs() < 1e-4 && down.y.abs() < 1e-4, "{down:?}");
    // And the touchdown follows.
    assert!(rig.run_until(5.0, |r| !r.out.status.armed));
    assert!(rig.saw(EventKind::Disarmed(DisarmReason::Touchdown)));
}

#[test]
fn contact_is_not_declared_in_the_first_second_of_a_descent_from_hover() {
    let mut rig = Rig::external(1);
    rig.arm();
    // From here the state accelerates as the motors command.
    rig.world = true;
    rig.run(0.5);
    rig.rc_on = false;
    assert!(rig.run_until(6.0, |r| r.nav() == NavState::Rtl));
    // Rtl at home lands at once: a descent from hover.
    let mut min_seen = f32::MAX;
    for _ in 0..1000 {
        rig.step();
        min_seen = min_seen.min(min_motor(&rig));
    }
    assert!(min_seen > CUT, "thrust cut in the air: {min_seen}");
    let vz = rig.state.map_or(0.0, |s| s.velocity.z);
    assert!(vz > 0.3, "not descending: {vz}");
}

#[test]
fn contact_ends_as_soon_as_the_state_moves() {
    let mut rig = critical_battery_rig();
    assert!(rig.run_until(1.0, |r| min_motor(r) < CUT), "no contact");
    // The ground gives way: the state starts falling.
    if let Some(s) = rig.state.as_mut() {
        s.velocity.z = 1.0;
    }
    assert!(
        rig.run_until(0.2, |r| min_motor(r) > CUT),
        "contact did not end: {:?}",
        rig.out.motors
    );
}

#[test]
fn a_blind_landing_descends_on_the_inertial_vertical_speed() {
    let mut rig = Rig::ekf(1);
    rig.arm();
    rig.rc.throttle = 0.5;
    rig.run(1.0);
    rig.world = true;
    rig.gps_on = false;
    rig.baro_on = false;
    assert!(rig.run_until(15.0, |r| r.nav() == NavState::LandNow));
    assert!(!rig.out.status.valid.altitude && !rig.out.status.valid.velocity);
    // A fixed thrust would hold 0.9 x hover; the velocity loop on the
    // accelerometer-derived speed settles on the descent rate instead.
    rig.run(8.0);
    let vz = rig.core.estimate().map_or(0.0, |e| e.velocity.z);
    assert!((vz - 0.7).abs() < 0.3, "inertial descent rate {vz}");
    assert!(rig.out.status.armed);
}

#[test]
fn a_blind_landing_never_commands_more_than_hover_thrust() {
    // The accelerometer's vertical bias was not fully calibrated when the
    // barometer went: the dead-reckoned speed over-reads the descent, and a
    // velocity loop on it would raise thrust above hover and climb.
    let mut rig = Rig::ekf(1);
    rig.arm();
    rig.rc.throttle = 0.5;
    rig.run(1.0);
    rig.world = true;
    rig.accel_bias_z = 2.0;
    rig.gps_on = false;
    rig.baro_on = false;
    assert!(rig.run_until(15.0, |r| r.nav() == NavState::LandNow));
    // The state change takes a tick to reach guidance.
    rig.run(0.01);
    // `thrust_n` scales with the battery voltage; so does the limit.
    let kv = rig.battery.map_or(1.0, |b| {
        (b.voltage_v / rig.nominal_v) * (b.voltage_v / rig.nominal_v)
    });
    let limit = rig.mass_kg * crate::GRAVITY * kv * 1.02;
    let mut peak = 0.0_f32;
    for _ in 0..20_000 {
        rig.step();
        peak = peak.max(rig.thrust_n());
    }
    assert!(peak <= limit, "blind thrust {peak} N above hover {limit} N");
}

/// Thrust of the last motor commands as a fraction of the vehicle's weight.
fn thrust_over_weight(rig: &Rig) -> f32 {
    let kv = rig.battery.map_or(1.0, |b| {
        (b.voltage_v / rig.nominal_v) * (b.voltage_v / rig.nominal_v)
    });
    rig.thrust_n() / kv / (rig.mass_kg * crate::GRAVITY)
}

#[test]
fn stick_held_down_on_the_ground_in_position_leaves_nothing_wound_up() {
    // The pilot holds the throttle down on the ground: contact is declared.
    // The held point must not keep sinking into the ground meanwhile, or
    // the centred stick that follows would still command a descent.
    let mut rig = Rig::external(1);
    rig.state = Some(StateSample::default());
    rig.rc.throttle = 0.0;
    rig.arm();
    assert!(rig.run_until(3.0, |r| min_motor(r) < CUT), "no contact");
    rig.run(6.0);
    rig.rc.throttle = 0.5;
    rig.run(0.5);
    assert!(rig.out.status.armed);
    let t = thrust_over_weight(&rig);
    assert!(
        (0.85..1.15).contains(&t),
        "thrust {t} x weight after release"
    );
}

#[test]
fn land_now_from_a_centred_position_hold_on_the_ground_disarms() {
    let mut c = cfg();
    c.failsafe.battery_low_v = c.failsafe.battery_critical_v;
    let mut rig = Rig::external(1).with_cfg(c);
    rig.state = Some(StateSample::default());
    rig.arm();
    rig.rc.throttle = 0.5;
    rig.battery = Some(BatterySample {
        voltage_v: 19.0,
        current_a: 0.0,
    });
    assert!(rig.run_until(40.0, |r| !r.out.status.armed));
    assert!(rig.saw(EventKind::Disarmed(DisarmReason::Touchdown)));
    assert_eq!(rig.out.motors, [0.0; 4]);
}

#[test]
fn contact_is_not_declared_on_a_vehicle_rising() {
    // An up-gust carries the vehicle up while a descent is commanded: an
    // upward speed is motion, not a vehicle held by the ground.
    let mut rig = critical_battery_rig();
    if let Some(s) = rig.state.as_mut() {
        s.velocity.z = -0.6;
    }
    let mut min_seen = f32::MAX;
    for _ in 0..1500 {
        rig.step();
        min_seen = min_seen.min(min_motor(&rig));
    }
    assert!(min_seen > CUT, "thrust cut on a rising vehicle: {min_seen}");
}

#[test]
fn zero_throttle_with_no_height_or_speed_is_not_landed_when_a_failsafe_starts() {
    // Stabilized and descending with the throttle at zero, the barometer and
    // GPS gone (height and speed invalid). The pilot overrides the estimator
    // failsafe, so the pilot's state flies; the thrust is the pilot's, and
    // low thrust says nothing about the ground. When the battery then turns
    // critical, LandNow must start in the air, not disarm as "landed".
    let mut rig = Rig::ekf(0);
    rig.arm();
    rig.rc.throttle = 0.5;
    rig.run(1.0);
    rig.world = true;
    rig.baro_on = false;
    rig.gps_on = false;
    rig.rc.throttle = 0.0;
    assert!(rig.run_until(15.0, |r| r.nav() == NavState::LandNow));
    assert!(!rig.out.status.valid.altitude && !rig.out.status.valid.velocity);
    // The pilot takes over (flips the mode switch) and keeps the throttle off.
    rig.rc.mode_switch = 1;
    assert!(
        rig.run_until(5.0, |r| r.nav() == NavState::Stabilized),
        "the override flies the pilot's state"
    );
    assert!(
        rig.events
            .iter()
            .any(|e| matches!(e.kind, EventKind::PilotOverride(_)))
    );
    rig.run(3.0);
    assert!(rig.out.status.armed && rig.nav() == NavState::Stabilized);
    // A new failsafe starts.
    rig.battery = Some(BatterySample {
        voltage_v: 19.0,
        current_a: 0.0,
    });
    assert!(rig.run_until(15.0, |r| r.nav() == NavState::LandNow
        || !r.out.status.armed));
    assert!(
        rig.out.status.armed,
        "disarmed in the air: {:?}",
        rig.events
    );
    rig.run(0.3);
    assert!(
        rig.out.status.armed,
        "disarmed in the air: {:?}",
        rig.events
    );
    assert!(!rig.saw(EventKind::Disarmed(DisarmReason::Touchdown)));
}

/// Runs `secs` and returns the largest commanded tilt seen, rad.
fn max_commanded_tilt(rig: &mut Rig, secs: f64) -> f32 {
    let mut max = 0.0_f32;
    rig.run_until(secs, |r| {
        max = max.max(tilt_angle(r.core.guidance.visible.attitude));
        false
    });
    max
}

#[test]
fn hold_does_not_chase_the_old_point_when_gps_returns_with_a_position_reset() {
    let mut rig = Rig::ekf(2);
    rig.arm();
    rig.run(1.0);
    rig.gps_on = false;
    rig.run(8.0);
    assert_eq!(rig.nav(), NavState::Hold);
    assert!(!rig.out.status.valid.position);
    // The brain's setpoint is where the vehicle really is, so that the end of
    // the failsafe (back to Offboard after a second) asks for nothing either.
    rig.gps_east = 20.0;
    rig.setpoint = Some(TrajectorySetpoint {
        position: Vec3::new(0.0, 20.0, -5.0),
        velocity: Vec3::zero(),
        yaw: 0.0,
    });
    rig.gps_on = true;
    let max = max_commanded_tilt(&mut rig, 4.0);
    assert!(rig.out.status.valid.position, "position must be back");
    assert!(
        max < 5.0_f32.to_radians(),
        "commanded tilt {} deg after the fix",
        max.to_degrees()
    );
    let e = rig.core.estimate().expect("estimate");
    assert!(
        (e.position.y - 20.0).abs() < 1.0,
        "estimate {:?}",
        e.position
    );
    assert!(
        (rig.core.guide_state.position_sp.y - e.position.y).abs() < 1.0,
        "setpoint {:?}",
        rig.core.guide_state.position_sp
    );
}

/// A bare core whose guidance is driven directly with a made-up estimate.
fn guidance_core() -> Core<4> {
    Core::new(cfg(), F405_LEAN).expect("valid config")
}

/// One guidance run (4 ms) in `nav` with the given validity, position, yaw and
/// home. The supervisor's armed flag is set; nothing else is simulated.
fn guide(
    core: &mut Core<4>,
    k: u64,
    nav: NavState,
    valid: Validity,
    position: Vec3<f32>,
    yaw: f32,
    home: Option<Vec3<f32>>,
) {
    core.supervision.visible = Supervision {
        armed: true,
        nav,
        home,
        ..Supervision::default()
    };
    core.estimate.visible = Estimate {
        state: Some(StateSample {
            position,
            velocity: Vec3::new(0.0, 0.0, 0.7),
            attitude: Quat::from_euler(0.0, 0.0, yaw),
            ..StateSample::default()
        }),
        valid,
    };
    core.run_guidance(Micros(k * 4000), 0.004);
}

const FLYING_STATES: [NavState; 5] = [
    NavState::Hold,
    NavState::Land,
    NavState::LandNow,
    NavState::Rtl,
    NavState::Offboard,
];

#[test]
fn every_flying_state_takes_the_new_yaw_when_the_heading_returns() {
    let no_heading = Validity {
        heading: false,
        velocity: false,
        position: false,
        ..Validity::ALL
    };
    let home = Some(Vec3::new(0.0, 0.0, 0.0));
    for nav in FLYING_STATES {
        let mut core = guidance_core();
        let p = Vec3::new(0.0, 0.0, -5.0);
        guide(&mut core, 0, nav, Validity::ALL, p, 0.0, home);
        guide(&mut core, 1, nav, Validity::ALL, p, 0.0, home);
        // Without a heading the setpoint follows the yaw.
        guide(&mut core, 2, nav, no_heading, p, 0.3, home);
        assert!((core.guide_state.yaw_sp - 0.3).abs() < 1e-6, "{nav:?}");
        // The magnetometer is back and the estimator reset its yaw.
        guide(&mut core, 3, nav, Validity::ALL, p, 1.4, home);
        assert!(
            (core.guide_state.yaw_sp - 1.4).abs() < 1e-6,
            "{nav:?}: yaw_sp {}",
            core.guide_state.yaw_sp
        );
        // Afterwards the setpoint is a setpoint again, not the yaw.
        guide(&mut core, 4, nav, Validity::ALL, p, 1.5, home);
        assert!(
            (core.guide_state.yaw_sp - 1.4).abs() < 1e-6 || nav == NavState::Offboard,
            "{nav:?}"
        );
    }
}

#[test]
fn every_flying_state_takes_the_fix_as_its_point_when_the_position_returns() {
    let no_position = Validity {
        position: false,
        ..Validity::ALL
    };
    let home = Some(Vec3::new(0.0, 0.0, 0.0));
    for nav in [
        NavState::Hold,
        NavState::Land,
        NavState::LandNow,
        NavState::Offboard,
    ] {
        let mut core = guidance_core();
        guide(
            &mut core,
            0,
            nav,
            Validity::ALL,
            Vec3::new(0.0, 0.0, -5.0),
            0.0,
            home,
        );
        guide(
            &mut core,
            1,
            nav,
            no_position,
            Vec3::new(3.0, 0.0, -5.0),
            0.0,
            home,
        );
        guide(
            &mut core,
            2,
            nav,
            Validity::ALL,
            Vec3::new(25.0, 4.0, -5.0),
            0.0,
            home,
        );
        let sp = core.guide_state.position_sp;
        assert!(
            (sp.x - 25.0).abs() < 1e-6 && (sp.y - 4.0).abs() < 1e-6,
            "{nav:?}: {sp:?}"
        );
    }
}

#[test]
fn every_flying_state_takes_the_current_height_when_the_altitude_returns() {
    let no_altitude = Validity {
        altitude: false,
        ..Validity::ALL
    };
    let home = Some(Vec3::new(0.0, 0.0, 0.0));
    for nav in [
        NavState::Hold,
        NavState::Land,
        NavState::LandNow,
        NavState::Rtl,
    ] {
        let mut core = guidance_core();
        guide(
            &mut core,
            0,
            nav,
            Validity::ALL,
            Vec3::new(0.0, 0.0, -5.0),
            0.0,
            home,
        );
        guide(
            &mut core,
            1,
            nav,
            no_altitude,
            Vec3::new(0.0, 0.0, -5.0),
            0.0,
            home,
        );
        // Away from the point the setpoint last followed.
        guide(
            &mut core,
            2,
            nav,
            Validity::ALL,
            Vec3::new(0.0, 0.0, -8.0),
            0.0,
            home,
        );
        let z = core.guide_state.position_sp.z;
        assert!((z + 8.0).abs() < 1e-6, "{nav:?}: z {z}");
    }
}

#[test]
fn the_rtl_return_height_survives_a_barometer_outage() {
    let no_altitude = Validity {
        altitude: false,
        ..Validity::ALL
    };
    let home = Some(Vec3::new(0.0, 0.0, 0.0));
    let mut core = guidance_core();
    // 10 m out, 3 m up: Rtl climbs to 10 m.
    guide(
        &mut core,
        0,
        NavState::Rtl,
        Validity::ALL,
        Vec3::new(10.0, 0.0, -3.0),
        0.0,
        home,
    );
    guide(
        &mut core,
        1,
        NavState::Rtl,
        Validity::ALL,
        Vec3::new(10.0, 0.0, -3.0),
        0.0,
        home,
    );
    assert_eq!(core.guide_state.rtl, RtlPhase::Climb);
    assert!((core.guide_state.position_sp.z + 10.0).abs() < 1e-6);
    guide(
        &mut core,
        2,
        NavState::Rtl,
        no_altitude,
        Vec3::new(10.0, 0.0, -3.0),
        0.0,
        home,
    );
    guide(
        &mut core,
        3,
        NavState::Rtl,
        Validity::ALL,
        Vec3::new(10.0, 0.0, -6.0),
        0.0,
        home,
    );
    assert_eq!(core.guide_state.rtl, RtlPhase::Climb);
    let z = core.guide_state.position_sp.z;
    assert!((z + 10.0).abs() < 1e-6, "return height lost: {z}");
}

#[test]
fn an_rtl_that_reached_its_landing_does_not_climb_when_the_position_returns() {
    // Same set-up as the position-loss test above, then the position comes
    // back 3 m from home (and, after the long gap, as a reset).
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
    rig.run(5.0);
    assert_eq!(rig.nav(), NavState::Rtl);
    rig.gps_east = 3.0;
    rig.gps_on = true;
    assert!(rig.run_until(3.0, |r| r.out.status.valid.position));
    let z_back = rig.core.estimate().expect("estimate").position.z;
    let mut highest = 0.0_f32;
    let mut phases = Vec::new();
    rig.run_until(3.0, |r| {
        highest = highest.min(r.core.guide_state.position_sp.z);
        if phases.last() != Some(&r.core.guide_state.rtl) {
            phases.push(r.core.guide_state.rtl);
        }
        false
    });
    assert!(
        highest >= z_back - 0.05,
        "height setpoint {highest} above the height {z_back} at the return"
    );
    assert!(!phases.contains(&RtlPhase::Climb), "{phases:?}");
    assert_eq!(rig.core.guide_state.rtl, RtlPhase::Return);
}

#[test]
fn an_rtl_that_reached_its_landing_returns_at_the_current_height_and_lands() {
    let home = Some(Vec3::new(0.0, 0.0, 0.0));
    let no_position = Validity {
        position: false,
        velocity: false,
        ..Validity::ALL
    };
    let mut core = guidance_core();
    let mut k = 0;
    let mut run = |core: &mut Core<4>, v: Validity, p: Vec3<f32>| {
        guide(core, k, NavState::Rtl, v, p, 0.0, home);
        k += 1;
    };
    // Entered over home: straight to the landing.
    run(&mut core, Validity::ALL, Vec3::new(0.0, 0.0, -4.0));
    run(&mut core, Validity::ALL, Vec3::new(0.0, 0.0, -4.0));
    assert_eq!(core.guide_state.rtl, RtlPhase::Landing);
    // The position goes, the vehicle drifts and sinks a little.
    run(&mut core, no_position, Vec3::new(1.0, 0.0, -3.5));
    run(&mut core, no_position, Vec3::new(2.0, 0.0, -3.0));
    // It returns 3 m from home, 3 m up: no climb to 10 m.
    let mut lowest_sp = 0.0_f32;
    for _ in 0..50 {
        run(&mut core, Validity::ALL, Vec3::new(3.0, 0.0, -3.0));
        lowest_sp = lowest_sp.min(core.guide_state.position_sp.z);
        assert_eq!(core.guide_state.rtl, RtlPhase::Return);
    }
    assert!(
        (lowest_sp + 3.0).abs() < 1e-6,
        "height setpoint {lowest_sp}"
    );
    // Over home and still: the landing, as usual.
    for _ in 0..3 {
        run(&mut core, Validity::ALL, Vec3::new(0.0, 0.0, -3.0));
    }
    assert_eq!(core.guide_state.rtl, RtlPhase::Return);
    // The setpoint walks to home at the return speed; wait for it.
    for _ in 0..400 {
        run(&mut core, Validity::ALL, Vec3::new(0.0, 0.0, -3.0));
    }
    assert_eq!(core.guide_state.rtl, RtlPhase::Landing);
    assert!((core.guide_state.position_sp.z + 3.0).abs() < 1e-6);
}
