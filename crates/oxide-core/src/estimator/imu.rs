// SPDX-License-Identifier: AGPL-3.0-or-later
//! IMU integration into delta angles and delta velocities (DESIGN §4.7).

use crate::math::Vec3;
use crate::time::Micros;

/// IMU deltas over one interval: what the EKF predicts with. Generic so the
/// filter can be tested in f64; the core uses f32.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ImuDelta<T> {
    /// End of the interval: the time of the last sample included.
    pub t_end: Micros,
    /// Interval length, s.
    pub dt: T,
    /// Integrated body angular rate, rad.
    pub dang: Vec3<T>,
    /// Integrated body specific force, m/s.
    pub dvel: Vec3<T>,
}
