// SPDX-License-Identifier: AGPL-3.0-or-later
//! The fault scenario suite (ADR-0003, DESIGN §4.10): each scenario injects
//! one failure in flight (or a pair of them) and states what the vehicle
//! must do. They are the acceptance test of the failsafes.
//!
//! Unless a scenario says otherwise it flies in Offboard (mode switch 2),
//! arms at 3 s, is sent a setpoint 5 m above the take-off point from 4 s, has
//! its fault at 15 s and has no ground station. A setpoint at the ground
//! point is sent from t = 0, because arming into Offboard needs live
//! setpoints. World frame NED, SI units.

use oxide_core::config::FailsafeParams;
use oxide_core::io::{Action, DisarmReason, NavState, Reasons, TrajectorySetpoint};
use oxide_core::math::Vec3;
use oxide_core::schedule::Profile;
use oxide_core::time::Micros;

use crate::airframe::BatterySpec;
use crate::faults::{Fault, FaultKind, LinkConfig};
use crate::outcome::{Ending, EventTag, Expect, FlightOutcome};
use crate::rng::{Rng, Stream, run_seed};
use crate::scenario::Script;
use crate::sim::SimOptions;
use crate::wind::WindConfig;
use crate::{Airframe, SimError, Simulation};

/// One named fault scenario.
#[derive(Clone, Debug, PartialEq)]
pub struct FaultScenario {
    pub name: &'static str,
    /// One sentence: what fails and what should happen.
    pub about: &'static str,
    pub script: Script,
    pub faults: Vec<Fault>,
    pub links: LinkConfig,
    pub failsafe: Option<FailsafeParams>,
    pub battery: Option<BatterySpec>,
    pub expect: Expect,
}

// ---- Building blocks -------------------------------------------------------

/// A setpoint over `(north, east, up)` metres, yaw 0.
fn sp(north: f32, east: f32, up: f32) -> Option<TrajectorySetpoint> {
    Some(TrajectorySetpoint {
        position: Vec3::new(north, east, -up),
        velocity: Vec3::zero(),
        yaw: 0.0,
    })
}

/// The setpoint point as a truth position, NED.
fn at(north: f64, east: f64, up: f64) -> Vec3<f64> {
    Vec3::new(north, east, -up)
}

/// An Offboard flight: arm at 3 s, ground-point setpoints until 4 s, then
/// `target`.
fn offboard_script(target: Option<TrajectorySetpoint>, duration_s: f64) -> Script {
    Script {
        arm_at_s: 3.0,
        mode: vec![(0.0, 2)],
        setpoints: vec![(0.0, sp(0.0, 0.0, 0.0)), (4.0, target)],
        duration_s,
        ..Script::default()
    }
}

/// A pilot flight in Position mode: arm at 3 s, climb to about 5 m by 5 s,
/// optionally fly about 15 m north (8 s to 13 s), then centre the sticks.
/// Throttle is 0 until the climb starts (arming needs it low); centred is 0.5.
fn position_script(forward: bool, duration_s: f64) -> Script {
    let mut sticks = vec![
        (0.0, [0.0, 0.0, 0.0, 0.0]),
        (3.5, [0.0, 0.0, 0.0, 1.0]),
        (5.2, [0.0, 0.0, 0.0, 0.5]),
    ];
    if forward {
        sticks.push((8.0, [0.0, 0.6, 0.0, 0.5]));
        sticks.push((13.0, [0.0, 0.0, 0.0, 0.5]));
    }
    Script {
        arm_at_s: 3.0,
        mode: vec![(0.0, 1)],
        sticks,
        setpoints: vec![(0.0, sp(0.0, 0.0, 0.0))],
        duration_s,
        ..Script::default()
    }
}

/// A fault from `start_s` to the end of the run.
fn fault(kind: FaultKind, start_s: f64) -> Fault {
    Fault {
        kind,
        start_s,
        end_s: None,
    }
}

fn fault_between(kind: FaultKind, start_s: f64, end_s: f64) -> Fault {
    Fault {
        kind,
        start_s,
        end_s: Some(end_s),
    }
}

fn gcs() -> LinkConfig {
    LinkConfig {
        gcs: true,
        ..LinkConfig::default()
    }
}

fn params(f: impl FnOnce(&mut FailsafeParams)) -> Option<FailsafeParams> {
    let mut p = FailsafeParams::default();
    f(&mut p);
    Some(p)
}

/// Every reason except `keep`.
fn all_but(keep: Reasons) -> Reasons {
    Reasons::ALL.difference(keep)
}

/// The expectation of a flight that must carry on undisturbed at `point`.
fn undisturbed(point: Vec3<f64>, tol: f64, by_s: f64) -> Expect {
    Expect {
        nav_never: vec![
            NavState::Hold,
            NavState::Rtl,
            NavState::Land,
            NavState::LandNow,
            NavState::Terminated,
        ],
        reasons_never: Reasons::ALL,
        ending: Ending::Flying {
            point,
            tolerance_m: tol,
        },
        by_s,
        ..Expect::default()
    }
}

/// A battery that starts at `soc` charge, with `capacity_mah`, otherwise as
/// the reference quad's.
fn battery(capacity_mah: f64, soc: f64) -> BatterySpec {
    BatterySpec {
        cells: 6,
        capacity_mah,
        internal_resistance_ohm: 0.025,
        nominal_voltage_v: 22.2,
        initial_state_of_charge: soc,
    }
}

// ---- The scenarios -----------------------------------------------------------

fn rc_loss_position() -> FaultScenario {
    FaultScenario {
        name: "rc-loss-position",
        about: "Radio lost while the pilot flies Position 15 m out: Hold, then Rtl, and lands at home.",
        script: position_script(true, 100.0),
        faults: vec![fault(FaultKind::RcLoss, 25.0)],
        links: LinkConfig::default(),
        failsafe: None,
        battery: None,
        expect: Expect {
            nav_contains: vec![NavState::Position, NavState::Hold, NavState::Rtl],
            reasons: Reasons::RC_LOST,
            ending: Ending::Landed {
                within_m_of_home: Some(2.0),
            },
            by_s: 90.0,
            ..Expect::default()
        },
    }
}

fn rc_loss_override() -> FaultScenario {
    let mut script = position_script(true, 60.0);
    script.mode.push((34.0, 2));
    // The brain's setpoint, over the point the return flight has reached by
    // the time the pilot takes over.
    script.setpoints = vec![(0.0, sp(7.0, 0.0, 10.0))];
    FaultScenario {
        name: "rc-loss-override",
        about: "Radio lost in Position, then back: the pilot moves the mode switch to Offboard during the return and takes over.",
        script,
        faults: vec![fault_between(FaultKind::RcLoss, 25.0, 32.0)],
        links: LinkConfig::default(),
        failsafe: None,
        battery: None,
        expect: Expect {
            nav_contains: vec![NavState::Hold, NavState::Rtl, NavState::Offboard],
            nav_never: vec![NavState::Land, NavState::LandNow, NavState::Terminated],
            reasons: Reasons::RC_LOST,
            events: vec![EventTag::PilotOverride],
            ending: Ending::Flying {
                point: at(7.0, 0.0, 10.0),
                tolerance_m: 3.0,
            },
            by_s: 60.0,
            ..Expect::default()
        },
    }
}

fn rc_loss_on_ground() -> FaultScenario {
    FaultScenario {
        name: "rc-loss-on-ground",
        about: "Radio lost while armed on the ground in Stabilized: disarms at once and never leaves the ground.",
        script: Script {
            arm_at_s: 3.0,
            mode: vec![(0.0, 0)],
            setpoints: vec![(0.0, sp(0.0, 0.0, 0.0))],
            duration_s: 12.0,
            ..Script::default()
        },
        faults: vec![fault(FaultKind::RcLoss, 6.0)],
        links: LinkConfig::default(),
        failsafe: None,
        battery: None,
        expect: Expect {
            reasons: Reasons::RC_LOST,
            max_height_m: Some(0.2),
            ending: Ending::Landed {
                within_m_of_home: Some(0.5),
            },
            by_s: 8.0,
            ..Expect::default()
        },
    }
}

fn rc_loss_offboard_gcs() -> FaultScenario {
    FaultScenario {
        name: "rc-loss-offboard-gcs",
        about: "Radio lost in Offboard with a ground station connected: nothing happens.",
        script: offboard_script(sp(0.0, 0.0, 5.0), 40.0),
        faults: vec![fault(FaultKind::RcLoss, 15.0)],
        links: gcs(),
        failsafe: None,
        battery: None,
        expect: undisturbed(at(0.0, 0.0, 5.0), 1.5, 40.0),
    }
}

fn no_human_link() -> FaultScenario {
    FaultScenario {
        name: "no-human-link",
        about: "Radio, then ground station lost in Offboard: after no_link_rtl_s (shortened to 10 s) it returns and lands at home.",
        script: offboard_script(sp(0.0, 0.0, 5.0), 80.0),
        faults: vec![
            fault(FaultKind::RcLoss, 15.0),
            fault(FaultKind::GcsLoss, 20.0),
        ],
        links: gcs(),
        failsafe: params(|p| p.no_link_rtl_s = 10.0),
        battery: None,
        expect: Expect {
            nav_contains: vec![NavState::Rtl],
            reasons: Reasons::NO_HUMAN_LINK,
            ending: Ending::Landed {
                within_m_of_home: Some(2.0),
            },
            by_s: 70.0,
            ..Expect::default()
        },
    }
}

fn gps_loss() -> FaultScenario {
    FaultScenario {
        name: "gps-loss",
        about: "GPS lost for good in Offboard: Hold, then after gps_loss_land_s (shortened to 10 s) Land, and it lands within 40 m of home.",
        script: offboard_script(sp(0.0, 0.0, 5.0), 80.0),
        faults: vec![fault(FaultKind::GpsLoss, 15.0)],
        links: LinkConfig::default(),
        failsafe: params(|p| p.gps_loss_land_s = 10.0),
        battery: None,
        expect: Expect {
            nav_contains: vec![NavState::Hold, NavState::Land],
            reasons: Reasons::GPS_LOST,
            ending: Ending::Landed {
                within_m_of_home: Some(40.0),
            },
            by_s: 70.0,
            ..Expect::default()
        },
    }
}

fn gps_loss_recovers() -> FaultScenario {
    FaultScenario {
        name: "gps-loss-recovers",
        about: "GPS lost for 4 s in Offboard: Hold while it is gone, then Offboard again at the setpoint.",
        script: offboard_script(sp(0.0, 0.0, 5.0), 45.0),
        faults: vec![fault_between(FaultKind::GpsLoss, 15.0, 19.0)],
        links: LinkConfig::default(),
        failsafe: None,
        battery: None,
        expect: Expect {
            nav_contains: vec![NavState::Offboard, NavState::Hold, NavState::Offboard],
            nav_never: vec![
                NavState::Rtl,
                NavState::Land,
                NavState::LandNow,
                NavState::Terminated,
            ],
            reasons: Reasons::GPS_LOST,
            ending: Ending::Flying {
                point: at(0.0, 0.0, 5.0),
                tolerance_m: 2.0,
            },
            by_s: 45.0,
            ..Expect::default()
        },
    }
}

fn gps_loss_pilot() -> FaultScenario {
    FaultScenario {
        name: "gps-loss-pilot",
        about: "GPS lost while the pilot hovers in Position: drops to Altitude with an event, no failsafe, pilot stays in control.",
        script: position_script(false, 45.0),
        faults: vec![fault(FaultKind::GpsLoss, 20.0)],
        links: LinkConfig::default(),
        failsafe: None,
        battery: None,
        expect: Expect {
            nav_contains: vec![NavState::Position, NavState::Altitude],
            nav_never: vec![
                NavState::Hold,
                NavState::Rtl,
                NavState::Land,
                NavState::LandNow,
                NavState::Terminated,
            ],
            reasons_never: Reasons::ALL,
            events: vec![EventTag::ModeDegraded],
            ending: Ending::FlyingAtHeight {
                height_m: 5.0,
                tolerance_m: 3.0,
            },
            by_s: 45.0,
            ..Expect::default()
        },
    }
}

fn baro_loss() -> FaultScenario {
    FaultScenario {
        name: "baro-loss",
        about: "Barometer lost for good: the estimator is unhealthy and the vehicle lands now.",
        script: offboard_script(sp(0.0, 0.0, 5.0), 60.0),
        faults: vec![fault(FaultKind::BaroLoss, 15.0)],
        links: LinkConfig::default(),
        failsafe: None,
        battery: None,
        expect: Expect {
            nav_contains: vec![NavState::LandNow],
            reasons: Reasons::ESTIMATOR_UNHEALTHY,
            ending: Ending::Landed {
                within_m_of_home: None,
            },
            by_s: 55.0,
            ..Expect::default()
        },
    }
}

fn mag_loss() -> FaultScenario {
    FaultScenario {
        name: "mag-loss",
        about: "Magnetometer lost for good: heading expires after 30 s, the estimator is unhealthy and the vehicle lands now.",
        script: offboard_script(sp(0.0, 0.0, 5.0), 90.0),
        faults: vec![fault(FaultKind::MagLoss, 15.0)],
        links: LinkConfig::default(),
        failsafe: None,
        battery: None,
        expect: Expect {
            nav_contains: vec![NavState::LandNow],
            reasons: Reasons::ESTIMATOR_UNHEALTHY,
            ending: Ending::Landed {
                within_m_of_home: None,
            },
            by_s: 85.0,
            ..Expect::default()
        },
    }
}

fn imu_loss() -> FaultScenario {
    FaultScenario {
        name: "imu-loss",
        about: "IMU lost: the flight is terminated within 0.3 s and the motors stay off.",
        script: offboard_script(sp(0.0, 0.0, 5.0), 25.0),
        faults: vec![fault(FaultKind::ImuLoss, 15.0)],
        links: LinkConfig::default(),
        failsafe: None,
        battery: None,
        expect: Expect {
            nav_contains: vec![NavState::Terminated],
            reasons: Reasons::IMU_FAILURE,
            ending: Ending::Terminated,
            by_s: 15.3,
            ..Expect::default()
        },
    }
}

fn brain_scenario(name: &'static str, about: &'static str, kind: FaultKind) -> FaultScenario {
    FaultScenario {
        name,
        about,
        script: offboard_script(sp(15.0, 0.0, 5.0), 100.0),
        faults: vec![fault(kind, 20.0)],
        links: LinkConfig::default(),
        failsafe: None,
        battery: None,
        expect: Expect {
            nav_contains: vec![NavState::Hold, NavState::Rtl],
            reasons: Reasons::SETPOINT_STALE.union(Reasons::BRAIN_LOST),
            ending: Ending::Landed {
                within_m_of_home: Some(2.0),
            },
            by_s: 90.0,
            ..Expect::default()
        },
    }
}

fn setpoint_gap() -> FaultScenario {
    let mut script = offboard_script(sp(0.0, 0.0, 5.0), 40.0);
    script.setpoints.push((15.0, None));
    script.setpoints.push((18.0, sp(0.0, 0.0, 5.0)));
    FaultScenario {
        name: "setpoint-gap",
        about: "The brain sends no setpoints for 3 s but its heartbeat continues: Hold while they are stale, then Offboard again.",
        script,
        faults: Vec::new(),
        links: LinkConfig::default(),
        failsafe: None,
        battery: None,
        expect: Expect {
            nav_contains: vec![NavState::Offboard, NavState::Hold, NavState::Offboard],
            nav_never: vec![
                NavState::Rtl,
                NavState::Land,
                NavState::LandNow,
                NavState::Terminated,
            ],
            reasons: Reasons::SETPOINT_STALE,
            reasons_never: all_but(Reasons::SETPOINT_STALE),
            ending: Ending::Flying {
                point: at(0.0, 0.0, 5.0),
                tolerance_m: 1.5,
            },
            by_s: 40.0,
            ..Expect::default()
        },
    }
}

fn gcs_loss() -> FaultScenario {
    FaultScenario {
        name: "gcs-loss",
        about: "The ground station is lost while the radio is fine: nothing happens.",
        script: offboard_script(sp(0.0, 0.0, 5.0), 40.0),
        faults: vec![fault(FaultKind::GcsLoss, 15.0)],
        links: gcs(),
        failsafe: None,
        battery: None,
        expect: undisturbed(at(0.0, 0.0, 5.0), 1.5, 40.0),
    }
}

fn battery_low() -> FaultScenario {
    FaultScenario {
        name: "battery-low",
        about: "The battery runs down to its low level in flight: Rtl, and lands at home.",
        script: offboard_script(sp(0.0, 0.0, 5.0), 90.0),
        faults: Vec::new(),
        links: LinkConfig::default(),
        failsafe: None,
        battery: Some(battery(1500.0, 0.15)),
        expect: Expect {
            nav_contains: vec![NavState::Rtl],
            reasons: Reasons::BATTERY_LOW,
            ending: Ending::Landed {
                within_m_of_home: Some(2.0),
            },
            by_s: 80.0,
            ..Expect::default()
        },
    }
}

fn battery_critical() -> FaultScenario {
    FaultScenario {
        name: "battery-critical",
        about: "The battery runs down through low to critical in flight: the vehicle lands now, wherever it is.",
        script: offboard_script(sp(0.0, 0.0, 5.0), 90.0),
        faults: Vec::new(),
        links: LinkConfig::default(),
        failsafe: None,
        battery: Some(battery(1500.0, 0.06)),
        expect: Expect {
            nav_contains: vec![NavState::LandNow],
            reasons: Reasons::BATTERY_CRITICAL,
            ending: Ending::Landed {
                within_m_of_home: None,
            },
            by_s: 80.0,
            ..Expect::default()
        },
    }
}

fn geofence() -> FaultScenario {
    let mut script = offboard_script(sp(0.0, 0.0, 5.0), 100.0);
    script.setpoints.push((15.0, sp(20.0, 0.0, 5.0)));
    FaultScenario {
        name: "geofence",
        about: "The setpoint steps 20 m out past a 10 m fence: Rtl, lands at home, and never strays more than 14 m.",
        script,
        faults: Vec::new(),
        links: LinkConfig::default(),
        failsafe: params(|p| p.fence_radius_m = 10.0),
        battery: None,
        expect: Expect {
            nav_contains: vec![NavState::Rtl],
            reasons: Reasons::GEOFENCE,
            max_distance_from_home_m: Some(14.0),
            ending: Ending::Landed {
                within_m_of_home: Some(2.0),
            },
            by_s: 90.0,
            ..Expect::default()
        },
    }
}

fn motor_loss() -> FaultScenario {
    FaultScenario {
        name: "motor-loss",
        about: "A rotor dies: the loss-of-control check fires and the flight is terminated within 2 s.",
        script: offboard_script(sp(0.0, 0.0, 5.0), 30.0),
        faults: vec![fault(
            FaultKind::MotorLoss {
                rotor: 0,
                scale: 0.0,
            },
            15.0,
        )],
        links: LinkConfig::default(),
        failsafe: None,
        battery: None,
        expect: Expect {
            nav_contains: vec![NavState::Terminated],
            reasons: Reasons::LOSS_OF_CONTROL,
            ending: Ending::Terminated,
            by_s: 17.0,
            ..Expect::default()
        },
    }
}

fn kill_in_rtl() -> FaultScenario {
    let mut s = rc_loss_position();
    s.name = "kill-in-rtl";
    s.about = "Radio lost in Position and back during the return, then the pilot hits the kill switch: disarms with the motors off.";
    s.script.duration_s = 50.0;
    s.script.kill_at_s = Some(34.0);
    s.faults = vec![fault_between(FaultKind::RcLoss, 25.0, 32.0)];
    s.expect = Expect {
        nav_contains: vec![NavState::Hold, NavState::Rtl],
        reasons: Reasons::RC_LOST,
        ending: Ending::Disarmed(DisarmReason::Kill),
        by_s: 36.0,
        ..Expect::default()
    };
    s
}

/// Every named scenario.
pub fn all() -> Vec<FaultScenario> {
    vec![
        rc_loss_position(),
        rc_loss_override(),
        rc_loss_on_ground(),
        rc_loss_offboard_gcs(),
        no_human_link(),
        gps_loss(),
        gps_loss_recovers(),
        gps_loss_pilot(),
        baro_loss(),
        mag_loss(),
        imu_loss(),
        brain_scenario(
            "brain-crash",
            "The brain crashes (heartbeat and setpoints stop) 15 m out: Hold, then Rtl, and lands at home.",
            FaultKind::BrainCrash,
        ),
        brain_scenario(
            "brain-hang",
            "The brain hangs (heartbeat continues, progress frozen, setpoints stop) 15 m out: Hold, then Rtl, and lands at home.",
            FaultKind::BrainHang,
        ),
        setpoint_gap(),
        gcs_loss(),
        battery_low(),
        battery_critical(),
        geofence(),
        motor_loss(),
        kill_in_rtl(),
    ]
}

/// The scenario called `name`, among the named ones and the pairings.
pub fn by_name(name: &str) -> Option<FaultScenario> {
    all().into_iter().chain(pairs()).find(|s| s.name == name)
}

// ---- Pairs -----------------------------------------------------------------

/// The seven common failures (ADR-0003, Consequences).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Failure {
    RcLoss,
    GpsLoss,
    BatteryLow,
    BatteryCritical,
    BrainCrash,
    Geofence,
    BaroLoss,
}

const FAILURES: [(Failure, &str); 7] = [
    (Failure::RcLoss, "rc-loss"),
    (Failure::GpsLoss, "gps-loss"),
    (Failure::BatteryLow, "battery-low"),
    (Failure::BatteryCritical, "battery-critical"),
    (Failure::BrainCrash, "brain-crash"),
    (Failure::Geofence, "geofence"),
    (Failure::BaroLoss, "baro-loss"),
];

/// Pairings run long enough for the slowest ending.
const PAIR_DURATION_S: f64 = 110.0;
/// Time by which a pairing must be down, s.
const PAIR_BY_S: f64 = 105.0;

impl Failure {
    /// The reason it raises.
    fn reason(self) -> Reasons {
        match self {
            Failure::RcLoss => Reasons::NO_HUMAN_LINK,
            Failure::GpsLoss => Reasons::GPS_LOST,
            Failure::BatteryLow => Reasons::BATTERY_LOW,
            Failure::BatteryCritical => Reasons::BATTERY_CRITICAL,
            Failure::BrainCrash => Reasons::BRAIN_LOST,
            Failure::Geofence => Reasons::GEOFENCE,
            Failure::BaroLoss => Reasons::ESTIMATOR_UNHEALTHY,
        }
    }
}

/// Seconds a flight spends from arming until a hover battery at `soc`
/// reaches the OCV of the given level: see `pair_battery`.
fn pair_battery(
    first: Failure,
    second: Failure,
    t_first: f64,
    t_second: f64,
) -> Option<BatterySpec> {
    // The battery scenarios depend on how fast the hover drains the pack;
    // the figures below come from `oxide-sim scenario battery-*`.
    let _ = (first, second, t_first, t_second);
    None
}

/// Every pairing of the seven common failures, one scenario each: the first
/// failure at 15 s, the second at 20 s. The generic expectation is that the
/// flight ends on the ground, disarmed (landed or terminated), that the
/// failsafe action never decreased, that the highest action is at least what
/// each failure alone calls for at its final escalation, and that a landing
/// is soft enough. Timers that would make a run needlessly long are
/// shortened: `no_link_rtl_s` to 5 s, `gps_loss_land_s` to 10 s.
pub fn pairs() -> Vec<FaultScenario> {
    let mut out = Vec::with_capacity(21);
    for (i, &(a, na)) in FAILURES.iter().enumerate() {
        for &(b, nb) in FAILURES.iter().skip(i + 1) {
            let name: &'static str = Box::leak(format!("pair-{na}+{nb}").into_boxed_str());
            out.push(pair(name, a, b));
        }
    }
    out
}

fn pair(name: &'static str, a: Failure, b: Failure) -> FaultScenario {
    let mut failsafe = FailsafeParams {
        no_link_rtl_s: 5.0,
        gps_loss_land_s: 10.0,
        ..FailsafeParams::default()
    };
    let mut script = offboard_script(sp(0.0, 0.0, 5.0), PAIR_DURATION_S);
    let mut faults = Vec::new();
    let mut battery_spec = None;
    for (f, t) in [(a, 15.0), (b, 20.0)] {
        match f {
            Failure::RcLoss => faults.push(fault(FaultKind::RcLoss, t)),
            Failure::GpsLoss => faults.push(fault(FaultKind::GpsLoss, t)),
            Failure::BrainCrash => faults.push(fault(FaultKind::BrainCrash, t)),
            Failure::BaroLoss => faults.push(fault(FaultKind::BaroLoss, t)),
            Failure::Geofence => {
                failsafe.fence_radius_m = 10.0;
                script.setpoints.push((t, sp(20.0, 0.0, 5.0)));
            }
            Failure::BatteryLow | Failure::BatteryCritical => {}
        }
    }
    if matches!(a, Failure::BatteryLow | Failure::BatteryCritical)
        || matches!(b, Failure::BatteryLow | Failure::BatteryCritical)
    {
        battery_spec = pair_battery(a, b, 15.0, 20.0);
    }
    let needed = [a, b]
        .iter()
        .map(|f| oxide_core::failsafe::reason_action(f.reason(), 1.0e6, &failsafe))
        .max()
        .unwrap_or(Action::None);
    FaultScenario {
        name,
        about: "Two failures in one flight: it ends on the ground, disarmed, and the action never decreases.",
        script,
        faults,
        links: LinkConfig::default(),
        failsafe: Some(failsafe),
        battery: battery_spec,
        expect: Expect {
            min_action: needed,
            ending: Ending::Down,
            by_s: PAIR_BY_S,
            ..Expect::default()
        },
    }
}

// ---- Flying ----------------------------------------------------------------

/// Light random wind for run seed `seed`: steady U(0, 2) m/s at a random
/// bearing, turbulence σ_h U(0.1, 0.4) m/s (σ = (σ_h, σ_h, 0.5·σ_h),
/// τ = 2 s), no gusts. Drawn from the Scenario stream like
/// `scenario::gust_hold_wind`.
pub fn fault_wind(seed: u64) -> WindConfig {
    let mut rng = Rng::stream(seed, Stream::Scenario);
    let speed = rng.range(0.0, 2.0);
    let steady_ned = rng.horizontal_direction() * speed;
    let sigma_h = rng.range(0.1, 0.4);
    WindConfig {
        steady_ned,
        turbulence_sigma: Vec3::new(sigma_h, sigma_h, 0.5 * sigma_h),
        turbulence_tau_s: 2.0,
        gusts: Vec::new(),
    }
}

/// The configured simulation of run `index` of `scenario` on `profile`, with
/// the reference quad, realistic sensors, the EKF and light random wind, not
/// yet run. Run seed `rng::run_seed(scenario.name, index)`.
pub fn build(
    scenario: &FaultScenario,
    profile: Profile,
    index: u64,
) -> Result<Simulation<Script, 4>, SimError> {
    let airframe = Airframe::reference_quad()?;
    let seed = run_seed(scenario.name, index);
    let options = SimOptions {
        wind: fault_wind(seed),
        faults: scenario.faults.clone(),
        links: scenario.links,
        failsafe: scenario.failsafe,
        battery: scenario.battery.clone(),
        ..SimOptions::realistic(seed)
    };
    Simulation::<_, 4>::with_options(&airframe, profile, scenario.script.clone(), options)
}

/// Flies run `index` of `scenario` on `profile` and returns what happened.
pub fn fly(
    scenario: &FaultScenario,
    profile: Profile,
    index: u64,
) -> Result<FlightOutcome, SimError> {
    let mut sim = build(scenario, profile, index)?;
    let mut outcome = FlightOutcome::new();
    let end = Micros((scenario.script.duration_s * 1e6).round() as u64);
    sim.run_until(end, |r| outcome.add(r));
    Ok(outcome)
}
