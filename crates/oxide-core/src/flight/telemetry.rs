// SPDX-License-Identifier: AGPL-3.0-or-later
//! Read-only telemetry: values the core already computes and a log needs
//! (ADR-0005 §3).
//!
//! Nothing here is computed for the log. Each field is a value the flight
//! path holds anyway, copied out; where the flight path only had a local
//! variable, it now stores it in a field that nothing reads back
//! (`RateCommand`, `Setpoints`). Reading [`Core::telemetry`] changes nothing,
//! which a test proves by comparing two cores' outputs bit for bit.

use super::{Core, guidance::LAND_CONTACT_S};
use crate::control::{HorizontalForm, VerticalForm, YawForm};
use crate::estimator::heading_check::HeadingFit;
use crate::io::{StateSample, Validity};
use crate::math::{Quat, Vec3};

/// What the rate loop commanded on its last tick, before the mixer. Kept by
/// the rate loop (task `Rate`, every base tick); all zero while disarmed.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RateCommand {
    /// Torque demand about the body x, y and z axes, N·m, frame FRD: the
    /// rate controller's angular-acceleration demand times the principal
    /// moments of inertia, per axis (`inertia.hadamard(alpha)`). The mixer's
    /// input; no cross-coupling term is added.
    pub torque_nm: Vec3<f32>,
    /// Collective thrust handed to the mixer, N, positive up: the attitude
    /// task's thrust target, unchanged.
    pub thrust_n: f32,
    /// The mixer could not meet the demand on this tick.
    pub saturated: bool,
}

/// What guidance is flying: the setpoints and the forms in force. Kept by
/// the guidance task (every guidance release); absent while guidance is not
/// controlling (disarmed, terminated, or no attitude estimate).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Setpoints {
    /// Position setpoint, m, world frame NED. In `Rtl` the horizontal part
    /// is the moving point and the vertical part the return height. It is
    /// only followed where `horizontal` and `vertical` are `Position`.
    pub position: Vec3<f32>,
    /// The velocity setpoint the velocity loop tracked, m/s NED, after the
    /// speed limits. An axis without feedback holds the estimate's own
    /// velocity (zero error). `None` in `Stabilized`, which flies roll and
    /// pitch angles and a thrust fraction with no velocity loop.
    pub velocity: Option<Vec3<f32>>,
    /// Yaw setpoint, rad. Followed only where `yaw_form` is `Heading`.
    pub yaw: f32,
    /// The horizontal controller form in force (ADR-0003 §2); `Tilt` in
    /// `Stabilized`.
    pub horizontal: HorizontalForm,
    /// The vertical controller form in force; `Thrust` in `Stabilized`.
    pub vertical: VerticalForm,
    /// The yaw form in force; `Heading` in `Stabilized` (the stick turns the
    /// yaw setpoint and the attitude target carries it).
    pub yaw_form: YawForm,
}

/// A snapshot of values inside the core. Every field names the task that
/// published it; all are the latest values at the moment of the call (after
/// the last `tick`). Frames: world NED, body FRD; SI units.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Telemetry {
    /// The published estimate (estimator task) as the controllers see it:
    /// position m, velocity m/s, attitude body to world, body rate rad/s.
    /// `None` until the estimator has published a state. Unlike
    /// [`Core::estimate`] it is also present when attitude is not valid
    /// (then `valid.attitude` is false and the controllers do not use it).
    pub estimate: Option<StateSample>,
    /// Which parts of `estimate` to trust.
    pub valid: Validity,
    /// Attitude target, body to world (guidance task; what the attitude
    /// controller flies, one guidance period behind `setpoints`).
    pub attitude_target: Quat<f32>,
    /// Collective thrust target, N, positive up (guidance task).
    pub thrust_target_n: f32,
    /// Body-rate target, rad/s (attitude task).
    pub rate_target: Vec3<f32>,
    /// What the rate loop commanded on its last tick (rate task).
    pub rate_command: RateCommand,
    /// The setpoints and controller forms guidance is flying (guidance
    /// task), or `None` when guidance is not controlling.
    pub setpoints: Option<Setpoints>,
    /// The landing detector's flag (supervisor task): true while disarmed,
    /// and while armed once thrust and vertical speed have stayed below the
    /// touchdown thresholds for `touchdown_s`.
    pub landed: bool,
    /// Guidance has seen ground contact (false whenever `setpoints` is
    /// `None`, so a stale count is never reported): a descent is commanded and the
    /// vehicle has not moved for 0.5 s (guidance task). The attitude and
    /// thrust targets are then overridden to level at minimum thrust.
    pub ground_contact: bool,
    /// The rate loop's gyro after its low-pass, rad/s body (rate-loop gyro
    /// filter; the estimator's bias is not subtracted here).
    pub gyro_filtered: Vec3<f32>,
    /// Filtered, load-compensated voltage per cell, V (battery monitor,
    /// supervisor task); `None` before the first reading. The core holds no
    /// filtered pack voltage: this per-cell value is what it filters.
    pub battery_cell_v: Option<f32>,
    /// The low-battery latch.
    pub battery_low: bool,
    /// The critical-battery latch.
    pub battery_critical: bool,
    /// The last battery sample received, unfiltered, and whether it exists:
    /// pack voltage, V.
    pub battery_voltage_v: Option<f32>,
    /// Current of the last battery sample received, A.
    pub battery_current_a: Option<f32>,
    /// Events dropped because the queue overflowed.
    pub events_dropped: u32,
    /// The heading check's record for the latest GPS fix it judged while
    /// airborne (estimator task; all zero for the external source).
    pub heading_fit: HeadingFit,
}

impl<const N: usize> Core<N> {
    /// Values the core holds that a log needs, read without side effects.
    pub fn telemetry(&self) -> Telemetry {
        let est = self.estimate.visible;
        let target = self.guidance.visible;
        let battery = &self.sup_state.battery;
        Telemetry {
            estimate: est.state,
            valid: est.valid,
            attitude_target: target.attitude,
            thrust_target_n: target.thrust_n,
            rate_target: self.attitude.visible.rate,
            rate_command: self.rate_command,
            setpoints: self.setpoints,
            landed: self.sup_state.land.landed(),
            ground_contact: self.setpoints.is_some()
                && self.guide_state.contact_s >= LAND_CONTACT_S,
            gyro_filtered: self.gyro_filtered,
            battery_cell_v: battery.cell_voltage(),
            battery_low: battery.low(),
            battery_critical: battery.critical(),
            battery_voltage_v: self.battery.map(|b| b.value.voltage_v),
            battery_current_a: self.battery.map(|b| b.value.current_a),
            events_dropped: self.events_dropped,
            heading_fit: self.heading_fit(),
        }
    }
}
