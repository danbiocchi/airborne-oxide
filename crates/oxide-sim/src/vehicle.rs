// SPDX-License-Identifier: AGPL-3.0-or-later
//! Multirotor physics: 6-DOF rigid body, first-order motors, quadratic body
//! drag, gravity and a simple ground contact. f64, fixed-step RK4
//! (DESIGN §5.3). All maths goes through `oxide_core::math` (libm).

use oxide_core::math::{Quat, Real, Vec3};

use crate::SimError;
use crate::airframe::{Airframe, SpinSpec};

/// Standard gravity, m/s².
pub const GRAVITY: f64 = 9.806_65;
/// Sea-level air density, kg/m³.
pub const AIR_DENSITY: f64 = 1.225;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RotorModel {
    /// Position from the centre of mass, m (body frame).
    pub position: Vec3<f64>,
    /// +1 for counter-clockwise seen from above, −1 for clockwise.
    pub spin: f64,
    pub max_thrust_n: f64,
}

/// Physical parameters of the simulated vehicle (the "truth").
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VehicleModel<const N: usize> {
    pub mass_kg: f64,
    pub inertia: Vec3<f64>,
    pub drag_area: Vec3<f64>,
    /// Rotor (induced / blade-flapping) drag: N per m/s of in-plane
    /// air-relative velocity per unit of summed normalised rotor speed.
    pub rotor_drag_coeff: f64,
    pub rotors: [RotorModel; N],
    /// Per-rotor fraction of normal thrust (and so reaction torque and shaft
    /// power). 1.0 = healthy, 0.0 = dead. Set by the simulation's faults.
    pub rotor_scale: [f64; N],
    pub torque_coeff_m: f64,
    pub motor_time_constant_s: f64,
    pub thrust_expo: f64,
    /// Rotor speed at full command and nominal voltage, rad/s.
    pub max_omega: f64,
    pub efficiency: f64,
    pub idle_power_w: f64,
    pub nominal_voltage_v: f64,
    /// Gravity on (false only for analytic tests).
    pub gravity: bool,
}

impl<const N: usize> VehicleModel<N> {
    pub fn from_airframe(a: &Airframe) -> Result<Self, SimError> {
        let specs = a.rotor_array::<N>()?;
        let p = &a.propulsion;
        let i = a.body.inertia_kg_m2;
        let d = a.body.drag_area_m2;
        Ok(Self {
            mass_kg: a.body.mass_kg,
            inertia: Vec3::new(i[0], i[1], i[2]),
            drag_area: Vec3::new(d[0], d[1], d[2]),
            rotor_drag_coeff: p.rotor_drag_coeff,
            rotors: specs.map(|r| RotorModel {
                position: Vec3::new(r.position_m[0], r.position_m[1], 0.0),
                spin: match r.spin {
                    SpinSpec::Ccw => 1.0,
                    SpinSpec::Cw => -1.0,
                },
                max_thrust_n: p.max_thrust_n,
            }),
            rotor_scale: [1.0; N],
            torque_coeff_m: p.torque_coeff_m,
            motor_time_constant_s: p.motor_time_constant_s,
            thrust_expo: p.thrust_expo,
            max_omega: p.max_rpm * 2.0 * core::f64::consts::PI / 60.0,
            efficiency: p.efficiency,
            idle_power_w: p.idle_power_w,
            nominal_voltage_v: a.battery.nominal_voltage_v,
            gravity: true,
        })
    }

    /// Steady-state normalised rotor speed for command `u` at `voltage`.
    pub fn speed_command(&self, u: f64, voltage: f64) -> f64 {
        let u = if u.is_finite() {
            u.clamp(0.0, 1.0)
        } else {
            0.0
        };
        let e = self.thrust_expo;
        let frac = (1.0 - e) * u + e * u * u;
        Real::sqrt(frac) * (voltage / self.nominal_voltage_v).max(0.0)
    }
}

/// Rigid-body state plus normalised rotor speeds (ω/ω_max at nominal voltage).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BodyState<const N: usize> {
    /// Position, m, NED.
    pub position: Vec3<f64>,
    /// Velocity, m/s, NED.
    pub velocity: Vec3<f64>,
    /// Body-to-world attitude.
    pub attitude: Quat<f64>,
    /// Body rate, rad/s (FRD).
    pub rate: Vec3<f64>,
    pub rotor_speed: [f64; N],
}

impl<const N: usize> Default for BodyState<N> {
    fn default() -> Self {
        Self {
            position: Vec3::zero(),
            velocity: Vec3::zero(),
            attitude: Quat::identity(),
            rate: Vec3::zero(),
            rotor_speed: [0.0; N],
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct Derivative<const N: usize> {
    position: Vec3<f64>,
    velocity: Vec3<f64>,
    attitude: Quat<f64>,
    rate: Vec3<f64>,
    rotor_speed: [f64; N],
}

impl<const N: usize> BodyState<N> {
    fn add(&self, d: &Derivative<N>, h: f64) -> Self {
        let mut rotor_speed = self.rotor_speed;
        for (s, ds) in rotor_speed.iter_mut().zip(d.rotor_speed.iter()) {
            *s += ds * h;
        }
        Self {
            position: self.position + d.position * h,
            velocity: self.velocity + d.velocity * h,
            attitude: self.attitude.add_scaled(d.attitude, h),
            rate: self.rate + d.rate * h,
            rotor_speed,
        }
    }

    pub fn is_finite(&self) -> bool {
        self.position.is_finite()
            && self.velocity.is_finite()
            && self.attitude.is_finite()
            && self.rate.is_finite()
            && self.rotor_speed.iter().all(|v| v.is_finite())
    }
}

/// Forces and moments at one instant.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Loads {
    /// Non-gravitational force, body frame, N.
    pub force_body: Vec3<f64>,
    /// Torque about the centre of mass, body frame, N·m.
    pub torque_body: Vec3<f64>,
    /// Mechanical shaft power of all rotors, W.
    pub shaft_power_w: f64,
}

impl<const N: usize> VehicleModel<N> {
    /// Loads at state `s` in a uniform wind (`wind_ned`, m/s, the velocity of
    /// the air mass). Drag acts on the air-relative velocity.
    pub fn loads(&self, s: &BodyState<N>, wind_ned: Vec3<f64>) -> Loads {
        let mut force = Vec3::zero();
        let mut torque = Vec3::zero();
        let mut power = 0.0;
        let mut speed_sum = 0.0;
        for ((r, n), k) in self
            .rotors
            .iter()
            .zip(s.rotor_speed.iter())
            .zip(self.rotor_scale.iter())
        {
            let n = n.max(0.0);
            speed_sum += n;
            let thrust = r.max_thrust_n * n * n * k;
            let f = Vec3::new(0.0, 0.0, -thrust);
            force += f;
            torque +=
                r.position.cross(f) + Vec3::new(0.0, 0.0, r.spin * self.torque_coeff_m * thrust);
            power += self.torque_coeff_m * thrust * n * self.max_omega;
        }
        // Quadratic body drag per axis plus linear rotor drag in the rotor
        // plane, both on the air-relative velocity.
        let vb = s.attitude.inverse_rotate(s.velocity - wind_ned);
        let drag = Vec3::new(vb.x * vb.x.abs(), vb.y * vb.y.abs(), vb.z * vb.z.abs())
            .hadamard(self.drag_area)
            * (-0.5 * AIR_DENSITY);
        let rotor_drag = Vec3::new(vb.x, vb.y, 0.0) * (-self.rotor_drag_coeff * speed_sum);
        Loads {
            force_body: force + drag + rotor_drag,
            torque_body: torque,
            shaft_power_w: power,
        }
    }

    fn derivative(
        &self,
        s: &BodyState<N>,
        speed_cmd: &[f64; N],
        wind_ned: Vec3<f64>,
    ) -> Derivative<N> {
        let loads = self.loads(s, wind_ned);
        let g = if self.gravity {
            Vec3::new(0.0, 0.0, GRAVITY)
        } else {
            Vec3::zero()
        };
        let accel = g + s.attitude.rotate(loads.force_body) * (1.0 / self.mass_kg);
        let jw = self.inertia.hadamard(s.rate);
        let ang = (loads.torque_body - s.rate.cross(jw)).hadamard(self.inertia.map(|v| 1.0 / v));
        let mut rotor_speed = [0.0; N];
        for ((d, n), c) in rotor_speed
            .iter_mut()
            .zip(s.rotor_speed.iter())
            .zip(speed_cmd.iter())
        {
            *d = (c - n) / self.motor_time_constant_s;
        }
        Derivative {
            position: s.velocity,
            velocity: accel,
            attitude: s.attitude.derivative(s.rate),
            rate: ang,
            rotor_speed,
        }
    }

    /// One RK4 step of `dt` with rotor-speed commands and the wind
    /// (`wind_ned`, m/s) held constant.
    pub fn rk4(
        &self,
        s: &BodyState<N>,
        speed_cmd: &[f64; N],
        dt: f64,
        wind_ned: Vec3<f64>,
    ) -> BodyState<N> {
        let k1 = self.derivative(s, speed_cmd, wind_ned);
        let k2 = self.derivative(&s.add(&k1, 0.5 * dt), speed_cmd, wind_ned);
        let k3 = self.derivative(&s.add(&k2, 0.5 * dt), speed_cmd, wind_ned);
        let k4 = self.derivative(&s.add(&k3, dt), speed_cmd, wind_ned);
        let mut out = s.add(&k1, dt / 6.0);
        out = out.add(&k2, dt / 3.0);
        out = out.add(&k3, dt / 3.0);
        out = out.add(&k4, dt / 6.0);
        out.attitude = out.attitude.normalized();
        out
    }
}

/// Ground plane at NED z = 0. Placeholder until collisions arrive in S2:
/// touching down stops all motion, and the vehicle rests until thrust lifts it.
pub fn ground_contact<const N: usize>(s: &mut BodyState<N>) -> bool {
    if s.position.z >= 0.0 {
        s.position.z = 0.0;
        if s.velocity.z >= 0.0 {
            s.velocity = Vec3::zero();
            s.rate = Vec3::zero();
        }
        true
    } else {
        false
    }
}
