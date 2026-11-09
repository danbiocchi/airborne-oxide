// SPDX-License-Identifier: AGPL-3.0-or-later
//! Ideal sensors for M1a: exact values, no noise, no latency. Noise, bias,
//! latency and dropouts arrive in M1b (DESIGN §5.4 S1).

use oxide_core::io::{BatterySample, ImuSample, Stamped, StateSample};
use oxide_core::math::Vec3;
use oxide_core::time::Micros;

use crate::vehicle::BodyState;

/// IMU sample: body rate and specific force (the acceleration an
/// accelerometer feels, i.e. without gravity), both in the body frame.
pub fn imu<const N: usize>(
    t: Micros,
    s: &BodyState<N>,
    specific_force_world: Vec3<f64>,
) -> Stamped<ImuSample> {
    let accel = s.attitude.inverse_rotate(specific_force_world);
    Stamped::ideal(
        t,
        ImuSample {
            gyro: s.rate.cast(),
            accel: accel.cast(),
        },
    )
}

/// Full-state sample (stands in for the estimator in M1a).
pub fn state<const N: usize>(t: Micros, s: &BodyState<N>) -> Stamped<StateSample> {
    Stamped::ideal(
        t,
        StateSample {
            position: s.position.cast(),
            velocity: s.velocity.cast(),
            attitude: s.attitude.cast(),
            rate: s.rate.cast(),
        },
    )
}

#[allow(clippy::cast_possible_truncation)]
pub fn battery(t: Micros, voltage_v: f64, current_a: f64) -> Stamped<BatterySample> {
    Stamped::ideal(
        t,
        BatterySample {
            voltage_v: voltage_v as f32,
            current_a: current_a as f32,
        },
    )
}
