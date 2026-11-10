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

/// Horizontal command.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Horizontal {
    /// Hold a north/east position, m, with velocity feed-forward, m/s.
    Position { sp: [f32; 2], vel_ff: [f32; 2] },
    /// Track a north/east velocity, m/s.
    Velocity([f32; 2]),
    /// Fly these roll and pitch angles, rad (no horizontal feedback).
    Tilt { roll: f32, pitch: f32 },
}

/// Vertical command (NED: down is positive).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Vertical {
    /// Hold a down position, m, with velocity feed-forward, m/s.
    Position { sp: f32, vel_ff: f32 },
    /// Track a down velocity, m/s.
    Velocity(f32),
    /// Collective thrust, N (no vertical feedback).
    Thrust(f32),
}

/// Yaw command.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Yaw {
    /// Hold this heading, rad.
    Heading(f32),
    /// Keep the current estimated yaw (no heading correction).
    Free,
}

/// Position and velocity controller (one task).
///
/// Horizontal and vertical control are chosen independently each call, so
/// the controller works on partial estimates (ADR-0003 section 2).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PositionController {
    /// x, y: horizontal integrator; z: vertical integrator.
    velocity_integral: Vec3<f32>,
    prev_velocity: Option<Vec3<f32>>,
}

impl PositionController {
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// Tracks `position_sp` with velocity feedforward `velocity_ff`.
    /// Equivalent to [`Self::update_axes`] with position control on both axes.
    pub fn update<const N: usize>(
        &mut self,
        position_sp: Vec3<f32>,
        velocity_ff: Vec3<f32>,
        yaw_sp: f32,
        est: &StateSample,
        dt: f32,
        cfg: &VehicleConfig<N>,
    ) -> AttitudeTarget {
        self.update_axes(
            Horizontal::Position {
                sp: [position_sp.x, position_sp.y],
                vel_ff: [velocity_ff.x, velocity_ff.y],
            },
            Vertical::Position {
                sp: position_sp.z,
                vel_ff: velocity_ff.z,
            },
            Yaw::Heading(yaw_sp),
            est,
            dt,
            cfg,
        )
    }

    /// One control step with the horizontal, vertical and yaw axes each in
    /// the form given.
    ///
    /// Safety choices where the specification is open: with
    /// [`Horizontal::Tilt`], roll and pitch are each clamped to
    /// `max_tilt_rad`; the resulting thrust (including [`Vertical::Thrust`])
    /// is clamped to the usual thrust limits.
    pub fn update_axes<const N: usize>(
        &mut self,
        h: Horizontal,
        v: Vertical,
        yaw: Yaw,
        est: &StateSample,
        dt: f32,
        cfg: &VehicleConfig<N>,
    ) -> AttitudeTarget {
        let g = &cfg.gains;
        let l = &cfg.limits;
        let yaw_sp = match yaw {
            Yaw::Heading(y) => y,
            Yaw::Free => est.attitude.yaw(),
        };
        let h_fb = !matches!(h, Horizontal::Tilt { .. });
        let v_fb = !matches!(v, Vertical::Thrust(_));

        // Position → velocity. Axes without feedback track the estimate, so
        // their error is zero.
        let mut vel_sp = est.velocity;
        match h {
            Horizontal::Position { sp, vel_ff } => {
                vel_sp.x = g.position_p.x * (sp[0] - est.position.x) + vel_ff[0];
                vel_sp.y = g.position_p.y * (sp[1] - est.position.y) + vel_ff[1];
            }
            Horizontal::Velocity(vel) => {
                vel_sp.x = vel[0];
                vel_sp.y = vel[1];
            }
            Horizontal::Tilt { .. } => {}
        }
        match v {
            Vertical::Position { sp, vel_ff } => {
                vel_sp.z = g.position_p.z * (sp - est.position.z) + vel_ff;
            }
            Vertical::Velocity(vel) => vel_sp.z = vel,
            Vertical::Thrust(_) => {}
        }
        let mut xy_speed_limited = false;
        if h_fb {
            let vxy = Real::sqrt(vel_sp.x * vel_sp.x + vel_sp.y * vel_sp.y);
            xy_speed_limited = vxy > l.max_speed_xy && vxy > 0.0;
            if xy_speed_limited {
                let k = l.max_speed_xy / vxy;
                vel_sp.x *= k;
                vel_sp.y *= k;
            }
        }
        let mut z_speed_limited = false;
        if v_fb {
            let vz = vel_sp.z.clamp(-l.max_speed_up, l.max_speed_down);
            z_speed_limited = vz != vel_sp.z;
            vel_sp.z = vz;
        }

        // Velocity → acceleration (integral applied after the limit checks).
        let err = vel_sp - est.velocity;
        let d = match self.prev_velocity {
            Some(prev) if dt > 0.0 => (prev - est.velocity) * (1.0 / dt),
            _ => Vec3::zero(),
        };
        self.prev_velocity = Some(est.velocity);
        let unintegrated = g.velocity_p.hadamard(err) + g.velocity_d.hadamard(d);
        let mut acc = unintegrated + self.velocity_integral;
        if !h_fb {
            acc.x = 0.0;
            acc.y = 0.0;
        }
        if let Vertical::Thrust(n) = v {
            // Vertical acceleration that makes the thrust n (a_z = g (1 - n / (m g))).
            let w = cfg.weight_n();
            acc.z = GRAVITY * (1.0 - n / w);
        }

        // Horizontal acceleration is limited by the tilt limit.
        let mut xy_accel_limited = false;
        if h_fb {
            let tan_tilt = Real::sin(l.max_tilt_rad) / Real::cos(l.max_tilt_rad);
            let max_h = GRAVITY * tan_tilt;
            let axy = Real::sqrt(acc.x * acc.x + acc.y * acc.y);
            xy_accel_limited = axy > max_h && axy > 0.0;
            if xy_accel_limited {
                let k = max_h / axy;
                acc.x *= k;
                acc.y *= k;
            }
        }

        let (target, thrust_limited) = if let Horizontal::Tilt { roll, pitch } = h {
            let roll = roll.clamp(-l.max_tilt_rad, l.max_tilt_rad);
            let pitch = pitch.clamp(-l.max_tilt_rad, l.max_tilt_rad);
            let min_thrust = l.min_thrust_fraction_of_hover * cfg.weight_n();
            let max_thrust = cfg.total_max_thrust_n();
            let raw = match v {
                Vertical::Thrust(n) => n,
                _ => {
                    let c = (Real::cos(roll) * Real::cos(pitch)).max(0.5);
                    cfg.mass_kg * (GRAVITY - acc.z) / c
                }
            };
            let thrust = raw.clamp(min_thrust, max_thrust);
            (
                AttitudeTarget {
                    attitude: Quat::from_euler(roll, pitch, yaw_sp),
                    thrust_n: thrust,
                },
                thrust != raw,
            )
        } else {
            thrust_target_with_limit(acc, yaw_sp, cfg)
        };

        // Anti-windup: an axis integrates only while its demand is not being
        // clipped by a speed, tilt or thrust limit. An axis that is not under
        // feedback holds no integral.
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
        if !h_fb {
            self.velocity_integral.x = 0.0;
            self.velocity_integral.y = 0.0;
        }
        if !v_fb {
            self.velocity_integral.z = 0.0;
        }
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

    // ---- position controller by axis ----

    use crate::config::{EstimatorConfig, FailsafeParams, Rotor, Spin};

    fn cfg() -> VehicleConfig<4> {
        let a = 0.08;
        let r = |x: f32, y: f32, spin| Rotor {
            x,
            y,
            spin,
            max_thrust_n: 8.0,
            torque_coeff_m: 0.016,
        };
        VehicleConfig {
            mass_kg: 0.65,
            inertia: Vec3::new(1.6e-3, 1.6e-3, 2.8e-3),
            rotors: [
                r(a, a, Spin::Ccw),
                r(-a, -a, Spin::Ccw),
                r(a, -a, Spin::Cw),
                r(-a, a, Spin::Cw),
            ],
            thrust_expo: 1.0,
            nominal_voltage_v: 22.2,
            gains: Gains::default(),
            limits: Limits::default(),
            estimator: EstimatorConfig::default(),
            failsafe: FailsafeParams::default(),
            battery_cells: 6,
            battery_resistance_ohm: 0.025,
        }
    }

    /// The controller as it was before the axis split, kept as the reference.
    #[derive(Default)]
    struct OldController {
        velocity_integral: Vec3<f32>,
        prev_velocity: Option<Vec3<f32>>,
    }

    impl OldController {
        fn update<const N: usize>(
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
            let err = vel_sp - est.velocity;
            let d = match self.prev_velocity {
                Some(prev) if dt > 0.0 => (prev - est.velocity) * (1.0 / dt),
                _ => Vec3::zero(),
            };
            self.prev_velocity = Some(est.velocity);
            let unintegrated = g.velocity_p.hadamard(err) + g.velocity_d.hadamard(d);
            let mut acc = unintegrated + self.velocity_integral;
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
                *self = Self::default();
                AttitudeTarget {
                    attitude: attitude_from_z_and_yaw(Vec3::new(0.0, 0.0, 1.0), yaw_sp),
                    thrust_n: cfg.weight_n(),
                }
            }
        }
    }

    struct Lcg(u64);
    impl Lcg {
        fn next(&mut self) -> f32 {
            self.0 = self
                .0
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            ((self.0 >> 40) as f32 / (1u64 << 24) as f32) * 2.0 - 1.0
        }
        fn vec(&mut self, scale: f32) -> Vec3<f32> {
            Vec3::new(
                self.next() * scale,
                self.next() * scale,
                self.next() * scale,
            )
        }
    }

    fn quat_bits(q: Quat<f32>) -> [u32; 4] {
        [q.w.to_bits(), q.x.to_bits(), q.y.to_bits(), q.z.to_bits()]
    }

    #[test]
    fn update_axes_position_is_bit_identical_to_old_update() {
        let cfg = cfg();
        for seed in 0..20u64 {
            let mut rng = Lcg(seed + 1);
            let mut old = OldController::default();
            let mut new = PositionController::default();
            let mut via_update = PositionController::default();
            for step in 0..300 {
                // Mix of small and limit-hitting magnitudes.
                let scale = if step % 7 == 0 { 50.0 } else { 3.0 };
                let sp = rng.vec(scale);
                let ff = rng.vec(scale * 0.3);
                let yaw = rng.next() * 3.0;
                let est = StateSample {
                    position: rng.vec(scale),
                    velocity: rng.vec(scale * 0.5),
                    attitude: Quat::from_euler(rng.next() * 0.5, rng.next() * 0.5, rng.next()),
                    rate: rng.vec(1.0),
                };
                let dt = 0.02;
                let a = old.update(sp, ff, yaw, &est, dt, &cfg);
                let b = new.update_axes(
                    Horizontal::Position {
                        sp: [sp.x, sp.y],
                        vel_ff: [ff.x, ff.y],
                    },
                    Vertical::Position {
                        sp: sp.z,
                        vel_ff: ff.z,
                    },
                    Yaw::Heading(yaw),
                    &est,
                    dt,
                    &cfg,
                );
                let c = via_update.update(sp, ff, yaw, &est, dt, &cfg);
                assert_eq!(quat_bits(a.attitude), quat_bits(b.attitude));
                assert_eq!(a.thrust_n.to_bits(), b.thrust_n.to_bits());
                assert_eq!(quat_bits(a.attitude), quat_bits(c.attitude));
                assert_eq!(a.thrust_n.to_bits(), c.thrust_n.to_bits());
                assert_eq!(old.velocity_integral, new.velocity_integral);
                assert_eq!(old.prev_velocity, new.prev_velocity);
            }
        }
    }

    /// Point mass pushed by the commanded thrust along the target body axis.
    fn point_mass_step(s: &mut StateSample, t: AttitudeTarget, cfg: &VehicleConfig<4>, dt: f32) {
        let thrust_acc = t
            .attitude
            .rotate(Vec3::new(0.0, 0.0, -t.thrust_n / cfg.mass_kg));
        let acc = thrust_acc + Vec3::new(0.0, 0.0, GRAVITY);
        s.velocity += acc * dt;
        s.position += s.velocity * dt;
        s.attitude = t.attitude;
    }

    fn hover_state() -> StateSample {
        StateSample {
            attitude: Quat::identity(),
            ..StateSample::default()
        }
    }

    #[test]
    fn horizontal_velocity_tracks_a_step() {
        let cfg = cfg();
        let mut c = PositionController::default();
        let mut s = hover_state();
        for _ in 0..150 {
            let t = c.update_axes(
                Horizontal::Velocity([3.0, -2.0]),
                Vertical::Velocity(0.0),
                Yaw::Heading(0.0),
                &s,
                0.02,
                &cfg,
            );
            point_mass_step(&mut s, t, &cfg, 0.02);
        }
        assert!((s.velocity.x - 3.0).abs() < 0.1, "vx {}", s.velocity.x);
        assert!((s.velocity.y + 2.0).abs() < 0.1, "vy {}", s.velocity.y);
        assert!(s.velocity.z.abs() < 0.1, "vz {}", s.velocity.z);
    }

    #[test]
    fn horizontal_velocity_setpoint_is_clamped_to_max_speed() {
        let cfg = cfg();
        let mut c = PositionController::default();
        let mut s = hover_state();
        for _ in 0..400 {
            let t = c.update_axes(
                Horizontal::Velocity([100.0, 0.0]),
                Vertical::Velocity(0.0),
                Yaw::Heading(0.0),
                &s,
                0.02,
                &cfg,
            );
            point_mass_step(&mut s, t, &cfg, 0.02);
        }
        assert!(
            (s.velocity.x - cfg.limits.max_speed_xy).abs() < 0.3,
            "vx {}",
            s.velocity.x
        );
    }

    #[test]
    fn vertical_velocity_tracks_a_step() {
        let cfg = cfg();
        for target in [-1.5_f32, 1.0] {
            let mut c = PositionController::default();
            let mut s = hover_state();
            for _ in 0..150 {
                let t = c.update_axes(
                    Horizontal::Velocity([0.0, 0.0]),
                    Vertical::Velocity(target),
                    Yaw::Heading(0.0),
                    &s,
                    0.02,
                    &cfg,
                );
                point_mass_step(&mut s, t, &cfg, 0.02);
            }
            assert!(
                (s.velocity.z - target).abs() < 0.1,
                "vz {} target {target}",
                s.velocity.z
            );
        }
    }

    #[test]
    fn tilt_with_hover_thrust_is_level_and_exact() {
        let cfg = cfg();
        let mut c = PositionController::default();
        let t = c.update_axes(
            Horizontal::Tilt {
                roll: 0.0,
                pitch: 0.0,
            },
            Vertical::Thrust(cfg.weight_n()),
            Yaw::Heading(0.0),
            &hover_state(),
            0.02,
            &cfg,
        );
        assert_eq!(t.thrust_n, cfg.weight_n());
        assert_eq!(
            quat_bits(t.attitude),
            quat_bits(Quat::from_euler(0.0, 0.0, 0.0))
        );
    }

    #[test]
    fn thrust_with_position_hold_is_level_at_hover_thrust() {
        let cfg = cfg();
        let mut c = PositionController::default();
        let t = c.update_axes(
            Horizontal::Position {
                sp: [0.0, 0.0],
                vel_ff: [0.0, 0.0],
            },
            Vertical::Thrust(cfg.weight_n()),
            Yaw::Heading(0.4),
            &hover_state(),
            0.02,
            &cfg,
        );
        assert!((t.thrust_n - cfg.weight_n()).abs() < 1e-4);
        let e = t.attitude.to_euler();
        assert!(e.x.abs() < 1e-5 && e.y.abs() < 1e-5 && (e.z - 0.4).abs() < 1e-5);
    }

    #[test]
    fn tilt_compensation_keeps_vertical_acceleration_zero() {
        let cfg = cfg();
        let roll = 20.0_f32.to_radians();
        let mut c = PositionController::default();
        let t = c.update_axes(
            Horizontal::Tilt { roll, pitch: 0.0 },
            Vertical::Velocity(0.0),
            Yaw::Heading(0.0),
            &hover_state(),
            0.02,
            &cfg,
        );
        let e = t.attitude.to_euler();
        assert!((e.x - roll).abs() < 1e-5);
        assert!(t.thrust_n > cfg.weight_n());
        let thrust_acc = t
            .attitude
            .rotate(Vec3::new(0.0, 0.0, -t.thrust_n / cfg.mass_kg));
        assert!((thrust_acc.z + GRAVITY).abs() < 1e-3, "{}", thrust_acc.z);
    }

    #[test]
    fn yaw_free_uses_estimated_yaw() {
        let cfg = cfg();
        let mut c = PositionController::default();
        let est = StateSample {
            attitude: Quat::from_euler(0.05, -0.03, 1.2),
            ..StateSample::default()
        };
        let t = c.update_axes(
            Horizontal::Velocity([0.0, 0.0]),
            Vertical::Velocity(0.0),
            Yaw::Free,
            &est,
            0.02,
            &cfg,
        );
        assert!((t.attitude.yaw() - est.attitude.yaw()).abs() < 1e-4);
        let t = c.update_axes(
            Horizontal::Tilt {
                roll: 0.1,
                pitch: 0.0,
            },
            Vertical::Velocity(0.0),
            Yaw::Free,
            &est,
            0.02,
            &cfg,
        );
        assert!((t.attitude.yaw() - est.attitude.yaw()).abs() < 1e-4);
    }

    #[test]
    fn integrators_reset_when_an_axis_leaves_feedback() {
        let cfg = cfg();
        let est = hover_state();
        let mut c = PositionController::default();
        for _ in 0..20 {
            c.update_axes(
                Horizontal::Velocity([0.5, 0.5]),
                Vertical::Velocity(0.5),
                Yaw::Heading(0.0),
                &est,
                0.02,
                &cfg,
            );
        }
        assert!(c.velocity_integral.x > 0.0 && c.velocity_integral.y > 0.0);
        assert!(c.velocity_integral.z > 0.0);

        // Horizontal leaves feedback: only x, y reset.
        let mut h = c;
        h.update_axes(
            Horizontal::Tilt {
                roll: 0.0,
                pitch: 0.0,
            },
            Vertical::Velocity(0.5),
            Yaw::Heading(0.0),
            &est,
            0.02,
            &cfg,
        );
        assert_eq!(h.velocity_integral.x, 0.0);
        assert_eq!(h.velocity_integral.y, 0.0);
        assert!(h.velocity_integral.z > 0.0);

        // Vertical leaves feedback: only z resets.
        let mut v = c;
        v.update_axes(
            Horizontal::Velocity([0.5, 0.5]),
            Vertical::Thrust(cfg.weight_n()),
            Yaw::Heading(0.0),
            &est,
            0.02,
            &cfg,
        );
        assert_eq!(v.velocity_integral.z, 0.0);
        assert!(v.velocity_integral.x > 0.0 && v.velocity_integral.y > 0.0);
    }

    #[test]
    fn tilt_is_clamped_and_non_finite_input_is_safe() {
        let cfg = cfg();
        let mut c = PositionController::default();
        let t = c.update_axes(
            Horizontal::Tilt {
                roll: 1.5,
                pitch: 0.0,
            },
            Vertical::Velocity(0.0),
            Yaw::Heading(0.0),
            &hover_state(),
            0.02,
            &cfg,
        );
        assert!((t.attitude.to_euler().x - cfg.limits.max_tilt_rad).abs() < 1e-4);
        assert!(t.thrust_n <= cfg.total_max_thrust_n());
        let t = c.update_axes(
            Horizontal::Tilt {
                roll: f32::NAN,
                pitch: 0.0,
            },
            Vertical::Thrust(f32::NAN),
            Yaw::Heading(0.0),
            &hover_state(),
            0.02,
            &cfg,
        );
        assert!(t.attitude.is_finite() && t.thrust_n.is_finite());
    }
}
