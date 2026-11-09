// SPDX-License-Identifier: AGPL-3.0-or-later
//! Vehicle configuration. On hardware this comes from parameters (§4.14);
//! in the simulator it is built from the airframe file.

use crate::math::Vec3;

/// Propeller spin direction, seen from above.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Spin {
    /// Counter-clockwise: reaction torque is positive yaw (nose right).
    Ccw,
    /// Clockwise: reaction torque is negative yaw.
    Cw,
}

impl Spin {
    pub const fn sign(self) -> f32 {
        match self {
            Spin::Ccw => 1.0,
            Spin::Cw => -1.0,
        }
    }
}

/// One rotor, thrusting along body −z (up).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Rotor {
    /// Position from the centre of mass, m (x forward, y right).
    pub x: f32,
    pub y: f32,
    pub spin: Spin,
    /// Maximum thrust at the nominal battery voltage, N.
    pub max_thrust_n: f32,
    /// Reaction torque per newton of thrust, m.
    pub torque_coeff_m: f32,
}

/// Controller gains, in physical units.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Gains {
    /// Rate loop: angular acceleration demand per rad/s of error, 1/s.
    pub rate_p: Vec3<f32>,
    /// Rate loop integral, 1/s².
    pub rate_i: Vec3<f32>,
    /// Rate loop derivative (on measurement), dimensionless.
    pub rate_d: Vec3<f32>,
    /// Derivative low-pass cutoff, Hz.
    pub rate_d_cutoff_hz: f32,
    /// Attitude loop: rate demand per rad of error, 1/s.
    pub attitude_p: Vec3<f32>,
    /// Velocity loop: acceleration demand per m/s of error, 1/s (x, y, z).
    pub velocity_p: Vec3<f32>,
    /// Velocity loop integral, 1/s².
    pub velocity_i: Vec3<f32>,
    /// Velocity loop derivative (on measurement), dimensionless.
    pub velocity_d: Vec3<f32>,
    /// Position loop: velocity demand per m of error, 1/s.
    pub position_p: Vec3<f32>,
}

impl Default for Gains {
    /// Starting gains for a small multirotor; auto-tuned per airframe later.
    ///
    /// The outer loops were chosen on a reduced model (position P → velocity
    /// PI → acceleration with a 0.12 s lag, 50 Hz, one-period delay) for
    /// ~1 % overshoot and ~1.1 s settling on a 1 m step. PX4-style defaults
    /// (0.95 / 1.8 / 0.4) leave a slow closed-loop pole near −0.2 rad/s that
    /// gives a 14 % overshoot and a 4 s tail without trajectory smoothing.
    fn default() -> Self {
        Self {
            rate_p: Vec3::new(28.0, 28.0, 12.0),
            rate_i: Vec3::new(40.0, 40.0, 10.0),
            rate_d: Vec3::new(0.3, 0.3, 0.0),
            rate_d_cutoff_hz: 60.0,
            attitude_p: Vec3::new(6.5, 6.5, 3.0),
            velocity_p: Vec3::new(4.0, 4.0, 4.0),
            velocity_i: Vec3::new(0.4, 0.4, 0.8),
            velocity_d: Vec3::new(0.0, 0.0, 0.0),
            position_p: Vec3::new(1.5, 1.5, 1.5),
        }
    }
}

/// Flight limits.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Limits {
    /// Maximum tilt in position and offboard modes, rad.
    pub max_tilt_rad: f32,
    /// Maximum tilt in stabilized mode, rad.
    pub max_tilt_stabilized_rad: f32,
    /// Maximum body rates, rad/s.
    pub max_rate: Vec3<f32>,
    /// Maximum horizontal speed, m/s.
    pub max_speed_xy: f32,
    pub max_speed_up: f32,
    pub max_speed_down: f32,
    /// Yaw rate at full yaw stick, rad/s.
    pub max_yaw_rate: f32,
    /// Limit on the rate integral's contribution, rad/s².
    pub rate_integral_limit: f32,
    /// Limit on the velocity integral's contribution, m/s².
    pub velocity_integral_limit: f32,
    /// Minimum collective thrust while flying, as a fraction of hover.
    pub min_thrust_fraction_of_hover: f32,
    /// Motor command while armed with no demand (keeps props spinning).
    pub idle_command: f32,
}

impl Default for Limits {
    fn default() -> Self {
        Self {
            max_tilt_rad: 35.0_f32.to_radians(),
            max_tilt_stabilized_rad: 45.0_f32.to_radians(),
            max_rate: Vec3::new(3.8, 3.8, 3.5),
            max_speed_xy: 5.0,
            max_speed_up: 3.0,
            max_speed_down: 1.5,
            max_yaw_rate: 2.0,
            rate_integral_limit: 30.0,
            velocity_integral_limit: 3.0,
            min_thrust_fraction_of_hover: 0.1,
            idle_command: 0.05,
        }
    }
}

/// Everything the core needs to know about the vehicle.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VehicleConfig<const N: usize> {
    pub mass_kg: f32,
    /// Principal moments of inertia, kg·m².
    pub inertia: Vec3<f32>,
    pub rotors: [Rotor; N],
    /// Thrust curve: thrust fraction = (1 − e)·u + e·u² for command u.
    pub thrust_expo: f32,
    /// Battery voltage at which `max_thrust_n` applies, V.
    pub nominal_voltage_v: f32,
    pub gains: Gains,
    pub limits: Limits,
}

/// Configuration problems found when the core is built.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConfigError {
    /// Mass or inertia not positive and finite.
    BadMassProperties,
    /// A rotor has a non-positive or non-finite thrust or position.
    BadRotor,
    /// The rotor layout cannot produce independent roll, pitch, yaw and thrust.
    UncontrollableLayout,
    /// Thrust expo outside `[0, 1]`.
    BadThrustExpo,
    /// The scheduling profile is malformed.
    BadProfile,
}

impl<const N: usize> VehicleConfig<N> {
    pub fn validate(&self) -> Result<(), ConfigError> {
        let pos = |v: f32| v.is_finite() && v > 0.0;
        if !pos(self.mass_kg)
            || !pos(self.inertia.x)
            || !pos(self.inertia.y)
            || !pos(self.inertia.z)
        {
            return Err(ConfigError::BadMassProperties);
        }
        if self.rotors.iter().any(|r| {
            !pos(r.max_thrust_n)
                || !r.x.is_finite()
                || !r.y.is_finite()
                || !r.torque_coeff_m.is_finite()
        }) {
            return Err(ConfigError::BadRotor);
        }
        if !(0.0..=1.0).contains(&self.thrust_expo) {
            return Err(ConfigError::BadThrustExpo);
        }
        if !pos(self.nominal_voltage_v) {
            return Err(ConfigError::BadMassProperties);
        }
        Ok(())
    }

    /// Total thrust available at nominal voltage, N.
    pub fn total_max_thrust_n(&self) -> f32 {
        self.rotors.iter().map(|r| r.max_thrust_n).sum()
    }

    /// Weight, N.
    pub fn weight_n(&self) -> f32 {
        self.mass_kg * crate::GRAVITY
    }
}
