// SPDX-License-Identifier: AGPL-3.0-or-later
//! Inputs, outputs and their timestamps: the contract between the core and
//! whatever drives it, board firmware or the simulator (DESIGN §4.2,
//! ADR-0001).

use crate::math::{Quat, Vec3};
use crate::time::Micros;

/// A sample with when it was measured and when the core received it.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Stamped<T> {
    pub t_sample: Micros,
    pub t_arrival: Micros,
    pub value: T,
}

impl<T> Stamped<T> {
    /// A sample measured and received at the same instant (ideal sensor).
    pub const fn ideal(t: Micros, value: T) -> Self {
        Self {
            t_sample: t,
            t_arrival: t,
            value,
        }
    }
}

/// IMU sample in the body frame (Forward-Right-Down).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ImuSample {
    /// Angular rate, rad/s.
    pub gyro: Vec3<f32>,
    /// Specific force (what an accelerometer measures), m/s².
    pub accel: Vec3<f32>,
}

/// A full vehicle state from an external source. In M1a the simulator feeds
/// ground truth here; later this carries motion-capture or similar.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct StateSample {
    /// Position, m, local North-East-Down.
    pub position: Vec3<f32>,
    /// Velocity, m/s, North-East-Down.
    pub velocity: Vec3<f32>,
    /// Body-to-world attitude.
    pub attitude: Quat<f32>,
    /// Body angular rate, rad/s.
    pub rate: Vec3<f32>,
}

/// Decoded RC input. Raw protocol decoding (SBUS) happens in firmware.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RcSample {
    /// Roll stick, −1 (left) to 1 (right).
    pub roll: f32,
    /// Pitch stick, −1 (back) to 1 (forward).
    pub pitch: f32,
    /// Yaw stick, −1 (left) to 1 (right).
    pub yaw: f32,
    /// Throttle stick, 0 to 1.
    pub throttle: f32,
    pub arm_switch: bool,
    pub kill_switch: bool,
    /// Mode switch position: 0, 1 or 2.
    pub mode_switch: u8,
    /// The receiver reports failsafe (link lost).
    pub failsafe: bool,
    /// The receiver reports a lost frame.
    pub frame_lost: bool,
}

impl RcSample {
    /// A frame is valid when the receiver reports neither failsafe nor a
    /// lost frame (DESIGN §4.9).
    pub const fn is_valid(&self) -> bool {
        !self.failsafe && !self.frame_lost
    }
}

/// Battery voltage and current.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BatterySample {
    pub voltage_v: f32,
    pub current_a: f32,
}

/// GNSS fix. M1b carries position already converted to the local NED frame
/// around the origin; latitude/longitude arrive with the GPS driver.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct GpsSample {
    /// Antenna position, m, local North-East-Down.
    pub position: Vec3<f32>,
    /// Velocity, m/s, North-East-Down.
    pub velocity: Vec3<f32>,
    /// Receiver-reported 1σ horizontal position accuracy, m.
    pub h_acc_m: f32,
    /// Receiver-reported 1σ vertical position accuracy, m.
    pub v_acc_m: f32,
    /// Receiver-reported 1σ speed accuracy, m/s.
    pub s_acc_mps: f32,
    /// The receiver has a 3D fix. Samples without one are not fused.
    pub fix_3d: bool,
}

/// Barometric altitude.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BaroSample {
    /// Pressure altitude, m, positive up, from an arbitrary reference. The
    /// estimator only uses changes relative to its value at alignment.
    pub altitude_m: f32,
}

/// Magnetometer reading.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MagSample {
    /// Magnetic field in the body frame (Forward-Right-Down), µT.
    pub field_ut: Vec3<f32>,
}

/// The ground station is connected and talking.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct GcsHeartbeat;

/// The brain's heartbeat (DESIGN §4.8).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct BrainHeartbeat {
    /// Planner-progress counter. A brain that is alive but stuck sends
    /// heartbeats with this unchanged, which counts as lost.
    pub progress: u32,
}

/// Tier-2 trajectory setpoint (DESIGN §4.8).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TrajectorySetpoint {
    /// Target position, m, North-East-Down.
    pub position: Vec3<f32>,
    /// Velocity feedforward, m/s, North-East-Down.
    pub velocity: Vec3<f32>,
    /// Target yaw, rad.
    pub yaw: f32,
}

/// Everything that arrived since the previous tick. `None` means no new
/// sample; the core keeps the last one and judges staleness itself.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Inputs {
    pub imu: Option<Stamped<ImuSample>>,
    pub state: Option<Stamped<StateSample>>,
    pub rc: Option<Stamped<RcSample>>,
    pub battery: Option<Stamped<BatterySample>>,
    pub setpoint: Option<Stamped<TrajectorySetpoint>>,
    pub gps: Option<Stamped<GpsSample>>,
    pub baro: Option<Stamped<BaroSample>>,
    pub mag: Option<Stamped<MagSample>>,
    pub gcs_heartbeat: Option<Stamped<GcsHeartbeat>>,
    pub brain_heartbeat: Option<Stamped<BrainHeartbeat>>,
}

/// Flight mode.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum FlightMode {
    /// Sticks command tilt angles, yaw rate and collective thrust.
    #[default]
    Stabilized,
    /// Sticks command tilt angles and yaw rate; the throttle commands climb
    /// rate, and centred it holds height. Not on the mode switch: it is
    /// what Position degrades to without a position (ADR-0003 §2).
    Altitude,
    /// Sticks command velocity; centred sticks hold position.
    Position,
    /// Follows tier-2 trajectory setpoints; holds position if they go stale.
    Offboard,
}

/// Which parts of the estimate can be trusted (ADR-0003 §1).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Validity {
    /// Roll, pitch and body rates.
    pub attitude: bool,
    /// Yaw.
    pub heading: bool,
    /// Height and vertical speed.
    pub altitude: bool,
    /// Horizontal velocity.
    pub velocity: bool,
    /// Horizontal position.
    pub position: bool,
}

impl Validity {
    pub const NONE: Self = Self {
        attitude: false,
        heading: false,
        altitude: false,
        velocity: false,
        position: false,
    };
    pub const ALL: Self = Self {
        attitude: true,
        heading: true,
        altitude: true,
        velocity: true,
        position: true,
    };

    /// Everything a position controller needs.
    pub const fn full(&self) -> bool {
        self.attitude && self.heading && self.altitude && self.velocity && self.position
    }
}

/// What the vehicle is doing (ADR-0003 §2).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum NavState {
    #[default]
    Disarmed,
    Stabilized,
    Altitude,
    Position,
    Offboard,
    /// Failsafe: hold the point where it was entered.
    Hold,
    /// Failsafe: climb, fly home, land.
    Rtl,
    /// Failsafe: descend holding horizontal position.
    Land,
    /// Failsafe: descend in place at once, on attitude alone if need be.
    LandNow,
    /// Failsafe: motors off, latched.
    Terminated,
}

/// A failsafe action. The order is the ranking: a later variant outranks
/// an earlier one (ADR-0003 §4).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub enum Action {
    #[default]
    None,
    Hold,
    Rtl,
    Land,
    LandNow,
    Terminate,
}

/// Defines a set of named bits with the usual set operations.
macro_rules! bitset {
    ($(#[$doc:meta])* $name:ident: $repr:ty { $($(#[$fdoc:meta])* $flag:ident = $bit:expr;)* }) => {
        $(#[$doc])*
        #[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
        pub struct $name(pub $repr);

        impl $name {
            $($(#[$fdoc])* pub const $flag: Self = Self(1 << $bit);)*
            /// Every defined bit.
            pub const ALL: Self = Self(0 $(| (1 << $bit))*);

            pub const fn empty() -> Self {
                Self(0)
            }
            pub const fn is_empty(self) -> bool {
                self.0 == 0
            }
            /// True when every bit of `other` is set here.
            pub const fn contains(self, other: Self) -> bool {
                self.0 & other.0 == other.0
            }
            /// True when any bit of `other` is set here.
            pub const fn intersects(self, other: Self) -> bool {
                self.0 & other.0 != 0
            }
            pub const fn union(self, other: Self) -> Self {
                Self(self.0 | other.0)
            }
            pub const fn intersection(self, other: Self) -> Self {
                Self(self.0 & other.0)
            }
            /// The bits set here and not in `other`.
            pub const fn difference(self, other: Self) -> Self {
                Self(self.0 & !other.0)
            }
            pub fn insert(&mut self, other: Self) {
                self.0 |= other.0;
            }
            pub fn remove(&mut self, other: Self) {
                self.0 &= !other.0;
            }
            /// Sets or clears the bits of `other`.
            pub fn set(&mut self, other: Self, on: bool) {
                if on {
                    self.insert(other);
                } else {
                    self.remove(other);
                }
            }
        }
    };
}

bitset! {
    /// Failsafe reasons that are currently true, one bit each
    /// (ADR-0003 §3). Bit positions are stable: they are logged.
    Reasons: u16 {
        RC_LOST = 0;
        NO_HUMAN_LINK = 1;
        BRAIN_LOST = 2;
        SETPOINT_STALE = 3;
        BATTERY_LOW = 4;
        BATTERY_CRITICAL = 5;
        GPS_LOST = 6;
        ESTIMATOR_UNHEALTHY = 7;
        IMU_FAILURE = 8;
        GEOFENCE = 9;
        LOSS_OF_CONTROL = 10;
    }
}

bitset! {
    /// Pre-arm checks that are failing, one bit each (ADR-0003 §6). Bit
    /// positions are stable: they are logged and shown to the pilot.
    PrearmFlags: u16 {
        /// The kill switch is on or its latch is set.
        KILL = 0;
        /// Throttle is not low.
        THROTTLE = 1;
        /// RC has not been valid for long enough.
        RC = 2;
        /// The vehicle is moving.
        NOT_STILL = 3;
        /// Attitude, heading or height is not valid.
        ESTIMATOR = 4;
        /// The selected mode needs a position and there is none.
        POSITION = 5;
        /// No battery reading, an implausible voltage, or battery low.
        BATTERY = 6;
        /// The configuration failed validation.
        CONFIG = 7;
    }
}

/// Why the vehicle disarmed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DisarmReason {
    /// The pilot's arm switch.
    Switch,
    /// The kill switch.
    Kill,
    /// Touchdown after an automatic landing.
    Touchdown,
    /// A failsafe terminated the flight.
    Terminated,
}

/// Something that happened (ADR-0003 §7).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EventKind {
    Armed,
    Disarmed(DisarmReason),
    /// Arming was refused; the failing checks.
    ArmRefused(PrearmFlags),
    NavChanged {
        from: NavState,
        to: NavState,
    },
    /// The pilot's mode could not be flown as requested.
    ModeDegraded {
        requested: FlightMode,
        flown: NavState,
    },
    /// A failsafe reason became true (one bit).
    ReasonRaised(Reasons),
    /// A failsafe reason stopped being true (one bit).
    ReasonCleared(Reasons),
    ActionChanged {
        from: Action,
        to: Action,
    },
    /// The pilot took over; the reasons acknowledged.
    PilotOverride(Reasons),
    Touchdown,
    HomeSet,
}

/// An event with the tick time it happened at.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Event {
    pub t: Micros,
    pub kind: EventKind,
}

/// Most events reported with one tick's outputs. More than this are held
/// back and reported on the following ticks.
pub const EVENTS_PER_TICK: usize = 4;

/// The events reported with one tick's outputs, oldest first.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct EventList {
    items: [Option<Event>; EVENTS_PER_TICK],
}

impl EventList {
    pub const fn new() -> Self {
        Self {
            items: [None; EVENTS_PER_TICK],
        }
    }
    /// Adds an event; `false` if the list is full.
    pub fn push(&mut self, e: Event) -> bool {
        match self.items.iter_mut().find(|s| s.is_none()) {
            Some(slot) => {
                *slot = Some(e);
                true
            }
            None => false,
        }
    }
    pub fn iter(&self) -> impl Iterator<Item = &Event> + '_ {
        self.items.iter().flatten()
    }
    pub fn len(&self) -> usize {
        self.iter().count()
    }
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

/// Estimator health (ADR-0002).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct EstimatorStatus {
    /// Tilt and heading have been initialised.
    pub aligned: bool,
    /// GPS is correcting horizontal position and velocity.
    pub gps_aiding: bool,
    /// Measurements rejected by the innovation gates since boot.
    pub rejected: u32,
    /// State resets since boot (position or velocity snapped to GPS).
    pub resets: u32,
}

/// Health and status flags reported with every output.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Status {
    pub armed: bool,
    /// The kill switch fired; latched until the arm switch is released.
    pub killed: bool,
    pub mode: FlightMode,
    /// The mixer could not meet the full demand this tick.
    pub saturated: bool,
    /// Ticks that arrived at the wrong time (logged fault, DESIGN §4.2).
    pub timing_faults: u32,
    /// No usable estimate: not even attitude.
    pub no_estimate: bool,
    pub estimator: EstimatorStatus,
    /// What the vehicle is doing.
    pub nav: NavState,
    /// Which parts of the estimate are valid.
    pub valid: Validity,
    /// Failsafe reasons currently true.
    pub reasons: Reasons,
    /// The failsafe action in force.
    pub action: Action,
    /// Pre-arm checks currently failing.
    pub prearm: PrearmFlags,
    /// Home was set at arming.
    pub home_set: bool,
}

/// Actuator commands and status. `motors` are normalised commands in
/// `[0, 1]`, valid from `t_valid` (one rate-loop period after the tick that
/// produced them, per the logical-execution-time rule).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Outputs<const N: usize> {
    pub t_valid: Micros,
    pub motors: [f32; N],
    pub status: Status,
    /// What happened this tick.
    pub events: EventList,
}

impl<const N: usize> Outputs<N> {
    /// All motors off.
    pub fn off(t_valid: Micros) -> Self {
        Self {
            t_valid,
            motors: [0.0; N],
            status: Status::default(),
            events: EventList::new(),
        }
    }
}
