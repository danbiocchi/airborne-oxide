// SPDX-License-Identifier: AGPL-3.0-or-later
//! Multirotor physics: 6-DOF rigid body, first-order motors, quadratic body
//! drag, gravity and a compliant ground. f64, fixed-step RK4
//! (DESIGN §5.3). All maths goes through `oxide_core::math` (libm).

use oxide_core::math::{Quat, Real, Vec3};

use crate::SimError;
use crate::airframe::{Airframe, SpinSpec};

/// Shortest time in which the ground's rate-proportional torques may stop a
/// body rate (a numerical safeguard for the explicit integrator), s.
const DAMPING_STEP_S: f64 = 0.5e-3;
/// Standard gravity, m/s².
pub const GRAVITY: f64 = 9.806_65;
/// Sea-level air density, kg/m³.
pub const AIR_DENSITY: f64 = 1.225;

/// The ground: a compliant plane at NED z = 0 that the vehicle rests on.
/// Contact forces are part of the physics derivative, so RK4 integrates them;
/// they are a function of the state only (deterministic). The ground exists
/// only while gravity is on.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GroundModel {
    /// Natural frequency of the vertical spring-damper, rad/s.
    pub omega_rad_s: f64,
    /// Damping ratio of the vertical spring-damper.
    pub zeta: f64,
    /// Coulomb friction coefficient (horizontal, and yaw through the legs).
    pub friction: f64,
    /// Horizontal speed over which friction ramps up to full, m/s.
    pub friction_speed_mps: f64,
    /// Yaw rate over which yaw friction ramps up to full, rad/s.
    pub yaw_friction_rate_rad_s: f64,
    /// Tilt at which the legs' restoring torque saturates, rad.
    pub tilt_ref_rad: f64,
}

impl Default for GroundModel {
    fn default() -> Self {
        Self {
            omega_rad_s: 30.0,
            zeta: 1.0,
            friction: 0.8,
            friction_speed_mps: 0.05,
            yaw_friction_rate_rad_s: 0.1,
            tilt_ref_rad: 0.05,
        }
    }
}

impl GroundModel {
    /// Static sag, m: the spring is in balance with the weight at z = 0.
    pub fn sag_m(&self) -> f64 {
        GRAVITY / (self.omega_rad_s * self.omega_rad_s)
    }

    /// True while the centre of mass at NED height `z` touches the ground.
    pub fn in_contact(&self, z: f64) -> bool {
        z > -self.sag_m()
    }

    /// Upward normal force on a body of `mass_kg` at NED `z` moving down at
    /// `vz`, N. Never negative: the ground cannot pull.
    pub fn normal_force_n(&self, mass_kg: f64, z: f64, vz: f64) -> f64 {
        if !self.in_contact(z) {
            return 0.0;
        }
        let w = self.omega_rad_s;
        (mass_kg * (w * w * (z + self.sag_m()) + 2.0 * self.zeta * w * vz)).max(0.0)
    }
}

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
    pub ground: GroundModel,
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
            ground: GroundModel::default(),
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

    /// Mean horizontal distance of the rotors from the centre of mass, m
    /// (the legs' lever arm).
    pub fn lever_arm_m(&self) -> f64 {
        let n = self.rotors.len().max(1) as f64;
        self.rotors
            .iter()
            .map(|r| Real::sqrt(r.position.x * r.position.x + r.position.y * r.position.y))
            .sum::<f64>()
            / n
    }

    /// Ground contact: force on the centre of mass (world frame, N) and torque
    /// (body frame, N·m). Zero when airborne.
    fn ground_loads(&self, s: &BodyState<N>) -> (Vec3<f64>, Vec3<f64>) {
        let gm = &self.ground;
        if !self.gravity {
            return (Vec3::zero(), Vec3::zero());
        }
        let fn_n = gm.normal_force_n(self.mass_kg, s.position.z, s.velocity.z);
        if fn_n <= 0.0 {
            return (Vec3::zero(), Vec3::zero());
        }
        // Friction opposes the horizontal velocity.
        let vxy = Vec3::new(s.velocity.x, s.velocity.y, 0.0);
        let f_fric = vxy * (-gm.friction * fn_n / (vxy.norm() + gm.friction_speed_mps));
        let force = Vec3::new(f_fric.x, f_fric.y, -fn_n);

        // Legs: restoring torque towards level (while upright), saturating at F_n·L, plus
        // damping sized for critical damping at the static load, all scaled
        // by F_n so they vanish at lift-off.
        let l = self.lever_arm_m();
        let down = s.attitude.rotate(Vec3::new(0.0, 0.0, 1.0));
        // Rotation vector (world) that turns the body's down axis onto the
        // vertical: axis = down × ẑ, angle = tilt.
        let axis = Vec3::new(down.y, -down.x, 0.0);
        let sin_t = axis.norm();
        let tilt = Real::atan2(sin_t, down.z);
        let tilt_vec = if sin_t > 1e-12 {
            axis * (tilt / sin_t)
        } else {
            Vec3::zero()
        };
        // The legs only right a vehicle whose up axis is above the horizon
        // (tilt below 90 deg); one that lies upside down is not turned over.
        let restoring = if down.z > 0.0 {
            tilt_vec * (fn_n * l / gm.tilt_ref_rad)
        } else {
            Vec3::zero()
        };
        let tb = s.attitude.inverse_rotate(restoring);
        let limit = fn_n * l;
        let w = self.mass_kg * GRAVITY;
        let k0 = w * l / gm.tilt_ref_rad;
        let scale = fn_n / w;
        // Rate-proportional torques are capped so that none can reverse a
        // body rate within `DAMPING_STEP_S`: with the contact force many
        // times the weight (a hard landing) they would otherwise be stiffer
        // than the 1 kHz base step can integrate. Below the cap they are
        // exactly the formulas above.
        let damp = |inertia: f64, rate: f64| {
            let t = 2.0 * Real::sqrt(k0 * inertia) * scale * rate;
            let cap = inertia * rate.abs() / DAMPING_STEP_S;
            t.clamp(-cap, cap)
        };
        let tx = tb.x.clamp(-limit, limit) - damp(self.inertia.x, s.rate.x);
        let ty = tb.y.clamp(-limit, limit) - damp(self.inertia.y, s.rate.y);
        let yaw = gm.friction * fn_n * l * s.rate.z / (s.rate.z.abs() + gm.yaw_friction_rate_rad_s);
        let cap = self.inertia.z * s.rate.z.abs() / DAMPING_STEP_S;
        let tz = -yaw.clamp(-cap, cap);
        (force, Vec3::new(tx, ty, tz))
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
        let (ground_force, ground_torque) = self.ground_loads(s);
        let accel = g + (s.attitude.rotate(loads.force_body) + ground_force) * (1.0 / self.mass_kg);
        let jw = self.inertia.hadamard(s.rate);
        let ang = (loads.torque_body + ground_torque - s.rate.cross(jw))
            .hadamard(self.inertia.map(|v| 1.0 / v));
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
