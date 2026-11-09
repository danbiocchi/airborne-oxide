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
}

/// Flight mode.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum FlightMode {
    /// Sticks command tilt angles, yaw rate and collective thrust.
    #[default]
    Stabilized,
    /// Sticks command velocity; centred sticks hold position.
    Position,
    /// Follows tier-2 trajectory setpoints; holds position if they go stale.
    Offboard,
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
    /// No valid state estimate yet.
    pub no_estimate: bool,
    pub estimator: EstimatorStatus,
}

/// Actuator commands and status. `motors` are normalised commands in
/// `[0, 1]`, valid from `t_valid` (one rate-loop period after the tick that
/// produced them, per the logical-execution-time rule).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Outputs<const N: usize> {
    pub t_valid: Micros,
    pub motors: [f32; N],
    pub status: Status,
}

impl<const N: usize> Outputs<N> {
    /// All motors off.
    pub fn off(t_valid: Micros) -> Self {
        Self {
            t_valid,
            motors: [0.0; N],
            status: Status::default(),
        }
    }
}
