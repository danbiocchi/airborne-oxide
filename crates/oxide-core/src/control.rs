// SPDX-License-Identifier: AGPL-3.0-or-later
//! Cascaded control: position → velocity → attitude → body rate (DESIGN §4.6).

use crate::GRAVITY;
use crate::config::{Gains, Limits, VehicleConfig};
use crate::io::StateSample;
use crate::math::{Quat, Real, Vec3};

/// Attitude and collective-thrust target, produced by guidance.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AttitudeTarget {
    pub attitude: Quat<f32>,
    /// Collective thrust, N.
    pub thrust_n: f32,
}

impl Default for AttitudeTarget {
    fn default() -> Self {
        Self {
            attitude: Quat::identity(),
            thrust_n: 0.0,
        }
    }
}

/// Body-rate and collective-thrust target, produced by attitude control.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RateTarget {
    pub rate: Vec3<f32>,
    pub thrust_n: f32,
}

/// First-order low-pass coefficient for one step of `dt` at `cutoff_hz`.
fn lowpass_alpha(dt: f32, cutoff_hz: f32) -> f32 {
    if cutoff_hz.is_nan() || cutoff_hz <= 0.0 {
        return 1.0;
    }
    let rc = 1.0 / (2.0 * core::f32::consts::PI * cutoff_hz);
    (dt / (dt + rc)).clamp(0.0, 1.0)
}

/// Body-rate PID. Output is an angular-acceleration demand, rad/s².
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RateController {
    integral: Vec3<f32>,
    prev_rate: Option<Vec3<f32>>,
    d_filtered: Vec3<f32>,
}

impl RateController {
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// `integrate` is false on the ground or while the mixer is saturated
    /// in the direction that would wind up.
    pub fn update(
        &mut self,
        rate_sp: Vec3<f32>,
        rate: Vec3<f32>,
        dt: f32,
        gains: &Gains,
        limits: &Limits,
        integrate: bool,
    ) -> Vec3<f32> {
        let err = rate_sp - rate;
        if integrate {
            let lim = Vec3::splat(limits.rate_integral_limit);
            self.integral = (self.integral + gains.rate_i.hadamard(err) * dt).clamp_abs(lim);
        }
        let raw_d = match self.prev_rate {
            Some(prev) if dt > 0.0 => (prev - rate) * (1.0 / dt),
            _ => Vec3::zero(),
        };
        self.prev_rate = Some(rate);
        let a = lowpass_alpha(dt, gains.rate_d_cutoff_hz);
        self.d_filtered += (raw_d - self.d_filtered) * a;
        let out =
            gains.rate_p.hadamard(err) + self.integral + gains.rate_d.hadamard(self.d_filtered);
        if out.is_finite() {
            out
        } else {
            self.reset();
            Vec3::zero()
        }
    }
}

/// Attitude P controller on the quaternion error. Returns a body-rate target.
pub fn attitude_rate_setpoint(
    q: Quat<f32>,
    q_sp: Quat<f32>,
    gains: &Gains,
    limits: &Limits,
) -> Vec3<f32> {
    let mut e = q.conj() * q_sp;
    if e.w < 0.0 {
        e = Quat::new(-e.w, -e.x, -e.y, -e.z);
    }
    let rate = gains
        .attitude_p
        .hadamard(e.vector() * 2.0)
        .clamp_abs(limits.max_rate);
    if rate.is_finite() { rate } else { Vec3::zero() }
}

/// Builds an attitude whose body z axis is `zb` (world frame, pointing
/// "down" through the vehicle) with heading `yaw`.
pub fn attitude_from_z_and_yaw(zb: Vec3<f32>, yaw: f32) -> Quat<f32> {
    let xc = Vec3::new(Real::cos(yaw), Real::sin(yaw), 0.0);
    let yb = match zb.cross(xc).normalized() {
        Some(y) => y,
        // zb is horizontal along the heading: pick the heading's right side.
        None => Vec3::new(-Real::sin(yaw), Real::cos(yaw), 0.0),
    };
    let xb = yb.cross(zb);
    Quat::from_axes(xb, yb, zb)
}

/// Position and velocity controller (one task).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PositionController {
    velocity_integral: Vec3<f32>,
    prev_velocity: Option<Vec3<f32>>,
}

impl PositionController {
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// Tracks `position_sp` with velocity feedforward `velocity_ff`.
    pub fn update<const N: usize>(
        &mut self,
        position_sp: Vec3<f32>,
        velocity_ff: Vec3<f32>,
        yaw_sp: f32,
        est: &StateSample,
        dt: f32,
        cfg: &VehicleConfig<N>,
    ) -> AttitudeTarget {
        let g = &cfg.gains;
        let l = &cfg.limits;

        // Position → velocity.
        let mut vel_sp = g.position_p.hadamard(position_sp - est.position) + velocity_ff;
        let vxy = Real::sqrt(vel_sp.x * vel_sp.x + vel_sp.y * vel_sp.y);
        let xy_speed_limited = vxy > l.max_speed_xy && vxy > 0.0;
        if xy_speed_limited {
            let k = l.max_speed_xy / vxy;
            vel_sp.x *= k;
            vel_sp.y *= k;
        }
        let vz = vel_sp.z.clamp(-l.max_speed_up, l.max_speed_down);
        let z_speed_limited = vz != vel_sp.z;
        vel_sp.z = vz;

        // Velocity → acceleration (integral applied after the limit checks).
        let err = vel_sp - est.velocity;
        let d = match self.prev_velocity {
            Some(prev) if dt > 0.0 => (prev - est.velocity) * (1.0 / dt),
            _ => Vec3::zero(),
        };
        self.prev_velocity = Some(est.velocity);
        let unintegrated = g.velocity_p.hadamard(err) + g.velocity_d.hadamard(d);
        let mut acc = unintegrated + self.velocity_integral;

        // Horizontal acceleration is limited by the tilt limit.
        let tan_tilt = Real::sin(l.max_tilt_rad) / Real::cos(l.max_tilt_rad);
        let max_h = GRAVITY * tan_tilt;
        let axy = Real::sqrt(acc.x * acc.x + acc.y * acc.y);
        let xy_accel_limited = axy > max_h && axy > 0.0;
        if xy_accel_limited {
            let k = max_h / axy;
            acc.x *= k;
            acc.y *= k;
        }

        let (target, thrust_limited) = thrust_target_with_limit(acc, yaw_sp, cfg);

        // Anti-windup: an axis integrates only while its demand is not being
        // clipped by a speed, tilt or thrust limit.
        let lim = Vec3::splat(l.velocity_integral_limit);
        let step = g.velocity_i.hadamard(err) * dt;
        let xy_ok = !(xy_speed_limited || xy_accel_limited);
        let z_ok = !(z_speed_limited || thrust_limited);
        let gated = Vec3::new(
            if xy_ok { step.x } else { 0.0 },
            if xy_ok { step.y } else { 0.0 },
            if z_ok { step.z } else { 0.0 },
        );
        self.velocity_integral = (self.velocity_integral + gated).clamp_abs(lim);
        if target.attitude.is_finite() && target.thrust_n.is_finite() {
            target
        } else {
            self.reset();
            AttitudeTarget {
                attitude: attitude_from_z_and_yaw(Vec3::new(0.0, 0.0, 1.0), yaw_sp),
                thrust_n: cfg.weight_n(),
            }
        }
    }
}

/// Attitude and thrust that produce world acceleration `acc` (NED).
pub fn thrust_target<const N: usize>(
    acc: Vec3<f32>,
    yaw_sp: f32,
    cfg: &VehicleConfig<N>,
) -> AttitudeTarget {
    thrust_target_with_limit(acc, yaw_sp, cfg).0
}

/// As [`thrust_target`], also reporting whether thrust or tilt was clipped.
pub fn thrust_target_with_limit<const N: usize>(
    acc: Vec3<f32>,
    yaw_sp: f32,
    cfg: &VehicleConfig<N>,
) -> (AttitudeTarget, bool) {
    let l = &cfg.limits;
    // Specific force the thrust must supply: f = a − g (g points down).
    let f = acc - Vec3::new(0.0, 0.0, GRAVITY);
    let down = Vec3::new(0.0, 0.0, 1.0);
    let cos_max = Real::cos(l.max_tilt_rad);
    let sin_max = Real::sin(l.max_tilt_rad);

    // Body z must point along −f.
    let mut zb = (-f).normalized().unwrap_or(down);
    let mut limited = false;
    if f.z >= 0.0 {
        // Demand is to accelerate down faster than gravity: stay level.
        zb = down;
        limited = true;
    } else if zb.z < cos_max {
        let h = Vec3::new(zb.x, zb.y, 0.0)
            .normalized()
            .unwrap_or(Vec3::zero());
        zb = Vec3::new(h.x * sin_max, h.y * sin_max, cos_max);
        limited = true;
    }

    let min_thrust = l.min_thrust_fraction_of_hover * cfg.weight_n();
    let raw = cfg.mass_kg * (-f).dot(zb);
    let thrust = raw.clamp(min_thrust, cfg.total_max_thrust_n());
    limited |= thrust != raw;
    (
        AttitudeTarget {
            attitude: attitude_from_z_and_yaw(zb, yaw_sp),
            thrust_n: thrust,
        },
        limited,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn level_attitude_has_no_rate_demand() {
        let g = Gains::default();
        let l = Limits::default();
        let q = Quat::from_euler(0.0, 0.0, 0.7);
        assert_eq!(attitude_rate_setpoint(q, q, &g, &l), Vec3::zero());
    }

    #[test]
    fn roll_error_gives_roll_rate() {
        let g = Gains::default();
        let l = Limits::default();
        let r = attitude_rate_setpoint(Quat::identity(), Quat::from_euler(0.2, 0.0, 0.0), &g, &l);
        assert!(r.x > 0.0 && r.y.abs() < 1e-6 && r.z.abs() < 1e-6);
    }

    #[test]
    fn attitude_from_level_z_keeps_yaw() {
        let q = attitude_from_z_and_yaw(Vec3::new(0.0, 0.0, 1.0), 1.0);
        let e = q.to_euler();
        assert!(e.x.abs() < 1e-5 && e.y.abs() < 1e-5 && (e.z - 1.0).abs() < 1e-5);
    }

    #[test]
    fn rate_controller_resets_on_nan() {
        let mut c = RateController::default();
        let out = c.update(
            Vec3::splat(f32::NAN),
            Vec3::zero(),
            0.001,
            &Gains::default(),
            &Limits::default(),
            true,
        );
        assert_eq!(out, Vec3::zero());
    }
}
