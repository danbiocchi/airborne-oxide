// SPDX-License-Identifier: AGPL-3.0-or-later
//! State estimation (DESIGN §4.7, ADR-0002): an error-state EKF on a delayed
//! time horizon, fed by IMU deltas, with an output predictor that brings the
//! state forward to the current tick.

pub mod ekf;
pub mod imu;
pub mod output;
pub mod ring;
