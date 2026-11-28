// SPDX-License-Identifier: AGPL-3.0-or-later
//! A tilt reference for the heading check (heading-guard stage 1, fourth
//! part; candidate T1a of the measurement in `report-HM.md`, section A1).
//!
//! The heading check compares the GPS velocity change over an interval with
//! the velocity change the accelerometer integrates, turned into the world
//! frame. The filter's own roll and pitch are pulled wrong by a wrong
//! heading itself (the GPS velocity is attributed to the wrong body axes),
//! so the check's angle read 0.18 rad high on average: the filter's tilt was
//! 6 degrees wrong on average, up to 16, at the moment the check spoke. This
//! reference is a small attitude state of its own that the GPS fusion never
//! touches: the unit vector Down in the body frame `u` and a gyro bias `b`.
//! It is propagated with the gyro and pulled toward the accelerometer's
//! direction (taken as "up" when the vehicle does not accelerate), the pull
//! weakening as the accelerometer's size leaves one g. It is used for one
//! thing, the input of the fit: the horizontal velocity change is the delta
//! velocity turned with **this** roll and pitch and **the filter's yaw**
//! (the yaw is the thing being checked, so it comes from the filter, as
//! it is). Nothing else reads it.
//!
//! Start: `u` from the filter's roll and pitch, `b` from the filter's own
//! gyro-bias estimate at that moment (the alignment's mean rate at rest;
//! PX4 seeds its reference the same way). From then on the reference takes
//! nothing from the filter but its yaw: no GPS reset, no heading
//! correction, no magnetometer reset touches `u` or `b`.
//!
//! `no_std`, `f32`, no allocation; transcendentals only through [`Real`].

use super::imu::ImuDelta;
use crate::GRAVITY;
use crate::math::{Real, Vec3};

/// Rate of the pull toward the accelerometer's direction, 1/s, where the
/// accelerometer reads exactly one g. Measured (HM, A3): of the two gains
/// tried, 0.2 meets the selection rule (a verdict in all 208 flights, none
/// in 123 healthy ones, later than the core's by more than 0.4 s in at most
/// 10 of 208) and 0.05 does not (43 flights late); it takes the 90th
/// percentile of the angle's error at the verdict from 0.328 to 0.234 rad.
/// A tilt state pulled slowly enough to ride out a manoeuvre is tilt-right
/// but, measured, makes the check speak late near 0.8 to 1.0 rad.
pub const PULL_GAIN: f32 = 0.2;
/// The gyro bias is learned at this fraction of the pull's rate, as PX4's
/// `ATT_W_GYRO_BIAS` is read: `b' = BIAS_GAIN * k * (u x u_acc)`. Measured
/// with the pull gain above (HM).
pub const BIAS_GAIN: f32 = 0.04;
/// The pull's weight falls to zero as the accelerometer's size leaves one g,
/// at this fraction of g: `max(0, 1 - ((|f| - g) / (BAND * g))^2)`. A
/// vehicle that accelerates hard reads a specific force that is not the
/// direction of "up".
pub const BAND: f32 = 0.5;
/// Smallest accelerometer size that has a direction, m/s^2.
const F_MIN: f32 = 1.0e-6;
/// Below this the horizontal part of `u` has no roll (a vehicle on its nose
/// or tail): roll is taken as zero.
const CH_MIN: f32 = 1.0e-9;

/// The tilt reference: the unit vector Down in the body frame and the gyro
/// bias estimate.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TiltRef {
    /// Down, unit length, body frame (FRD): level is `(0, 0, 1)`.
    u: Vec3<f32>,
    /// Gyro bias estimate, rad/s (body frame).
    b: Vec3<f32>,
}

impl TiltRef {
    /// A reference at `roll` and `pitch` (rad; the filter's, at the moment
    /// the filter comes into being) with the gyro bias `bias` (rad/s; the
    /// filter's own estimate at that moment).
    pub fn new(roll: f32, pitch: f32, bias: Vec3<f32>) -> Self {
        let (sp, cp) = (Real::sin(pitch), Real::cos(pitch));
        let (sr, cr) = (Real::sin(roll), Real::cos(roll));
        Self {
            u: Vec3::new(-sp, cp * sr, cp * cr),
            b: bias,
        }
    }

    /// Down in the body frame (unit length).
    pub const fn down(&self) -> Vec3<f32> {
        self.u
    }

    /// The gyro bias estimate, rad/s.
    pub const fn bias(&self) -> Vec3<f32> {
        self.b
    }

    /// Roll, rad (the convention of the filter's attitude).
    pub fn roll(&self) -> f32 {
        Real::atan2(self.u.y, self.u.z)
    }

    /// Pitch, rad.
    pub fn pitch(&self) -> f32 {
        Real::asin((-self.u.x).clamp(-1.0, 1.0))
    }

    /// One filter step with the same IMU delta the filter's prediction takes:
    /// propagate with the gyro less the bias, then pull toward the
    /// accelerometer's direction and learn the bias. A delta that is not
    /// usable (no time, not finite) changes nothing.
    pub fn step(&mut self, d: &ImuDelta<f32>) {
        let dt = d.dt;
        if !(dt > 0.0 && dt.is_finite() && d.dang.is_finite() && d.dvel.is_finite()) {
            return;
        }
        // Propagate: du = -(w x u) dt, w = gyro - bias.
        let turn = d.dang - self.b * dt;
        let Some(u) = (self.u - turn.cross(self.u)).normalized() else {
            return;
        };
        self.u = u;
        // Pull toward "up" as the accelerometer sees it, with a weight that
        // is zero when its size is half a g or more from g.
        let f = d.dvel * (1.0 / dt);
        let fnorm = f.norm();
        if !(fnorm > F_MIN && fnorm.is_finite()) {
            return;
        }
        let q = (fnorm - GRAVITY) / (BAND * GRAVITY);
        let weight = 1.0 - q * q;
        if weight <= 0.0 {
            return;
        }
        let target = f * (-1.0 / fnorm);
        let e = self.u.cross(target);
        let k = PULL_GAIN * weight;
        let along = self.u.dot(target);
        let pulled = self.u + (target - self.u * along) * (k * dt);
        if let Some(u) = pulled.normalized() {
            self.u = u;
        }
        self.b += e * (BIAS_GAIN * k * dt);
    }

    /// The horizontal (north, east) world-frame velocity change of a delta
    /// velocity `dv` (body frame, m/s, the accelerometer's bias already
    /// taken off) turned with this reference's roll and pitch and a yaw
    /// whose cosine and sine are `yaw_cos_sin` (the filter's).
    pub fn horizontal(&self, dv: Vec3<f32>, yaw_cos_sin: [f32; 2]) -> [f32; 2] {
        let u = self.u;
        let ch = Real::sqrt(u.y * u.y + u.z * u.z);
        let sth = -u.x;
        let (sph, cph) = if ch > CH_MIN {
            (u.y / ch, u.z / ch)
        } else {
            (0.0, 1.0)
        };
        // Pitch about the body y axis and roll about x, applied to `dv`:
        // the level-frame velocity change before the yaw.
        let r1 = ch * dv.x + sth * sph * dv.y + sth * cph * dv.z;
        let r2 = cph * dv.y - sph * dv.z;
        let [cy, sy] = yaw_cos_sin;
        [cy * r1 - sy * r2, sy * r1 + cy * r2]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::time::Micros;

    const DT: f32 = 0.004;

    fn delta(gyro: Vec3<f32>, accel: Vec3<f32>) -> ImuDelta<f32> {
        ImuDelta {
            t_end: Micros::ZERO,
            dt: DT,
            dang: gyro * DT,
            dvel: accel * DT,
        }
    }

    /// The accelerometer reading of a vehicle at rest at `roll`, `pitch`
    /// (specific force: minus gravity in the body frame).
    fn rest_accel(roll: f32, pitch: f32) -> Vec3<f32> {
        let down = TiltRef::new(roll, pitch, Vec3::zero()).down();
        down * (-GRAVITY)
    }

    fn angle_between(a: Vec3<f32>, b: Vec3<f32>) -> f32 {
        Real::atan2(a.cross(b).norm(), a.dot(b))
    }

    #[test]
    fn a_level_still_vehicle_stays_level() {
        let mut r = TiltRef::new(0.0, 0.0, Vec3::zero());
        for _ in 0..5000 {
            r.step(&delta(Vec3::zero(), rest_accel(0.0, 0.0)));
        }
        assert!(r.roll().abs() < 1e-6 && r.pitch().abs() < 1e-6);
        assert!(r.bias().norm() < 1e-6);
        assert!((r.down().norm() - 1.0).abs() < 1e-6);
    }

    #[test]
    fn the_start_is_the_roll_and_pitch_it_is_given() {
        for (roll, pitch) in [(0.3_f32, -0.2_f32), (-1.0, 0.7), (2.0, 0.1)] {
            let r = TiltRef::new(roll, pitch, Vec3::new(0.01, 0.0, 0.0));
            assert!((r.roll() - roll).abs() < 1e-5, "{roll}");
            assert!((r.pitch() - pitch).abs() < 1e-5, "{pitch}");
            assert_eq!(r.bias(), Vec3::new(0.01, 0.0, 0.0));
        }
    }

    #[test]
    fn a_constant_gyro_bias_is_learned_and_the_tilt_comes_back() {
        // A still, level vehicle whose gyro reads 0.02 rad/s on x and -0.01
        // on y, the reference's own bias starting at zero: without the
        // pull the tilt would drift 0.02 rad/s; with it the tilt stays
        // small and the bias is learned.
        let gyro = Vec3::new(0.02, -0.01, 0.0);
        let mut r = TiltRef::new(0.0, 0.0, Vec3::zero());
        let mut worst_after_10_s = 0.0_f32;
        for k in 0..(200 * 250) {
            r.step(&delta(gyro, rest_accel(0.0, 0.0)));
            if k > 10 * 250 {
                worst_after_10_s =
                    worst_after_10_s.max(angle_between(r.down(), Vec3::new(0.0, 0.0, 1.0)));
            }
        }
        // The tilt is held near level the whole time (the equilibrium of a
        // pull of 0.2 /s against 0.02 rad/s is 0.1 rad before learning).
        assert!(worst_after_10_s < 0.1, "tilt error {worst_after_10_s}");
        // And the bias is learned: most of it after 200 s.
        let learned = r.bias();
        assert!((learned.x - 0.02).abs() < 0.004, "bias x {}", learned.x);
        assert!((learned.y + 0.01).abs() < 0.004, "bias y {}", learned.y);
        // The tilt itself has come back.
        assert!(angle_between(r.down(), Vec3::new(0.0, 0.0, 1.0)) < 0.01);
    }

    #[test]
    fn a_tilted_vehicle_at_rest_converges_to_its_tilt_at_the_stated_rate() {
        let (roll, pitch) = (0.2_f32, -0.15_f32);
        let truth = TiltRef::new(roll, pitch, Vec3::zero()).down();
        let mut r = TiltRef::new(0.0, 0.0, Vec3::zero());
        let e0 = angle_between(r.down(), truth);
        let mut at_5s = 0.0;
        for k in 1..=(30 * 250) {
            r.step(&delta(Vec3::zero(), rest_accel(roll, pitch)));
            if k == 5 * 250 {
                at_5s = angle_between(r.down(), truth);
            }
        }
        // 0.2 per second: the error after 5 s is exp(-1) of the start (a
        // little less, the bias term also pulls).
        let ratio = at_5s / e0;
        assert!(
            (0.3..0.4).contains(&ratio),
            "after 5 s the error is {ratio} of the start (exp(-1) = 0.368)"
        );
        // The bias learned from the transient (about 0.003 rad/s from a
        // start 0.25 rad off) leaves a residual that unlearns slowly: the
        // measured behaviour of the candidate, not a flaw of the port.
        assert!(angle_between(r.down(), truth) < 0.03);
    }

    #[test]
    fn the_pull_is_zero_when_the_accelerometer_is_half_a_g_or_more_from_g() {
        let truth_dir = TiltRef::new(0.4, 0.3, Vec3::zero()).down();
        for scale in [0.5_f32, 0.3, 1.5, 2.0, 3.0] {
            let mut r = TiltRef::new(0.0, 0.0, Vec3::zero());
            for _ in 0..2000 {
                r.step(&delta(Vec3::zero(), truth_dir * (-GRAVITY * scale)));
            }
            assert_eq!(r.down(), Vec3::new(0.0, 0.0, 1.0), "scale {scale}");
            assert_eq!(r.bias(), Vec3::zero(), "scale {scale}");
        }
        // Just inside the band the pull acts.
        let mut r = TiltRef::new(0.0, 0.0, Vec3::zero());
        for _ in 0..2000 {
            r.step(&delta(Vec3::zero(), truth_dir * (-GRAVITY * 1.2)));
        }
        assert!(angle_between(r.down(), Vec3::new(0.0, 0.0, 1.0)) > 0.05);
        // And at 0.5 g from g the weight is exactly zero.
        let mut r = TiltRef::new(0.0, 0.0, Vec3::zero());
        r.step(&delta(Vec3::zero(), truth_dir * (-GRAVITY * 1.5)));
        assert_eq!(r.down(), Vec3::new(0.0, 0.0, 1.0));
    }

    #[test]
    fn the_gyro_turns_it_the_right_way() {
        // Rolling at 1 rad/s about the body x axis (right wing down) for a
        // quarter of a second with the pull off (a big acceleration): roll
        // goes positive by about 0.25 rad.
        let mut r = TiltRef::new(0.0, 0.0, Vec3::zero());
        for _ in 0..62 {
            r.step(&delta(Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 0.0, -40.0)));
        }
        assert!((r.roll() - 0.248).abs() < 0.01, "roll {}", r.roll());
        assert!(r.pitch().abs() < 1e-4);
        // Pitching up at 1 rad/s about y: pitch goes positive.
        let mut r = TiltRef::new(0.0, 0.0, Vec3::zero());
        for _ in 0..62 {
            r.step(&delta(Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, 0.0, -40.0)));
        }
        assert!((r.pitch() - 0.248).abs() < 0.01, "pitch {}", r.pitch());
        assert!(r.roll().abs() < 1e-4);
    }

    #[test]
    fn the_yaw_is_the_one_it_is_given_and_does_not_touch_the_state() {
        let r = TiltRef::new(0.25, -0.1, Vec3::zero());
        let dv = Vec3::new(0.3, -0.2, -0.04);
        let before = r;
        let psi = 0.7_f32;
        let a = r.horizontal(dv, [Real::cos(psi), Real::sin(psi)]);
        assert_eq!(r, before, "reading the reference changes it");
        // A different yaw turns the answer about Down by the difference.
        let phi = 2.1_f32;
        let b = r.horizontal(dv, [Real::cos(psi + phi), Real::sin(psi + phi)]);
        let (c, s) = (Real::cos(phi), Real::sin(phi));
        assert!((b[0] - (c * a[0] - s * a[1])).abs() < 1e-6);
        assert!((b[1] - (s * a[0] + c * a[1])).abs() < 1e-6);
        // At zero yaw and level it is the delta velocity itself.
        let level = TiltRef::new(0.0, 0.0, Vec3::zero());
        let h = level.horizontal(dv, [1.0, 0.0]);
        assert!((h[0] - dv.x).abs() < 1e-7 && (h[1] - dv.y).abs() < 1e-7);
    }

    #[test]
    fn the_horizontal_change_is_the_delta_velocity_turned_by_roll_and_pitch() {
        // The same thing through the rotation the filter uses (a quaternion
        // from the Euler angles with zero yaw): the north/east part of
        // `R dv`.
        use crate::math::Quat;
        for (roll, pitch) in [(0.3_f32, -0.2_f32), (-0.8, 0.5), (1.2, 0.1)] {
            let r = TiltRef::new(roll, pitch, Vec3::zero());
            let dv = Vec3::new(0.11, -0.07, -0.039);
            let want = Quat::from_euler(roll, pitch, 0.0).rotate(dv);
            let got = r.horizontal(dv, [1.0, 0.0]);
            assert!((got[0] - want.x).abs() < 1e-5, "{roll} {pitch}");
            assert!((got[1] - want.y).abs() < 1e-5, "{roll} {pitch}");
        }
    }

    #[test]
    fn an_unusable_delta_changes_nothing() {
        let mut r = TiltRef::new(0.2, 0.1, Vec3::new(0.01, 0.0, 0.0));
        let before = r;
        for d in [
            ImuDelta {
                dt: 0.0,
                ..delta(Vec3::zero(), rest_accel(0.0, 0.0))
            },
            ImuDelta {
                dt: f32::NAN,
                ..delta(Vec3::zero(), rest_accel(0.0, 0.0))
            },
            delta(Vec3::new(f32::NAN, 0.0, 0.0), rest_accel(0.0, 0.0)),
            delta(Vec3::zero(), Vec3::new(0.0, f32::INFINITY, 0.0)),
        ] {
            r.step(&d);
            assert_eq!(r, before);
        }
        // No accelerometer direction (zero specific force): propagated, not pulled.
        r.step(&delta(Vec3::zero(), Vec3::zero()));
        assert_eq!(r.bias(), before.bias());
    }

    // ---- the cross-check against the measured code ----

    /// The made-up sequence of the cross-check: 3000 steps of 4 ms in which
    /// the vehicle turns, rolls and pitches, accelerates in the horizontal
    /// plane and, from 8 s to 10 s, accelerates hard enough that the
    /// accelerometer leaves the band around one g (the pull is off); the
    /// gyro has a constant bias. Returns the gyro and accelerometer readings
    /// and the filter's yaw (the truth plus 0.3 rad) of step `k`.
    fn made_up(k: usize) -> (Vec3<f32>, Vec3<f32>, f32) {
        let t = (k as f32 + 0.5) * DT;
        let (s, c) = (|x: f32| Real::sin(x), |x: f32| Real::cos(x));
        let roll = 0.5 * s(0.9 * t) + 0.1 * s(3.0 * t);
        let pitch = 0.4 * s(0.6 * t + 1.0);
        let yaw = 0.8 * t + 0.5 * s(0.4 * t);
        let droll = 0.45 * c(0.9 * t) + 0.3 * c(3.0 * t);
        let dpitch = 0.24 * c(0.6 * t + 1.0);
        let dyaw = 0.8 + 0.2 * c(0.4 * t);
        let rates = Vec3::new(
            droll - dyaw * s(pitch),
            dpitch * c(roll) + dyaw * s(roll) * c(pitch),
            -dpitch * s(roll) + dyaw * c(roll) * c(pitch),
        );
        let gyro = rates + Vec3::new(0.02, -0.015, 0.01);
        let big = if (8.0..10.0).contains(&t) { 1.0 } else { 0.0 };
        let a_w = Vec3::new(
            4.0 * s(1.3 * t) + big * 7.0 * s(5.0 * t),
            3.0 * s(0.8 * t + 2.0) + big * 6.0,
            0.3 * s(2.1 * t) + big * -5.0,
        ) - Vec3::new(0.0, 0.0, GRAVITY);
        // f = R^T a_w, R = Rz(yaw) Ry(pitch) Rx(roll).
        let (cr, sr, cp, sp, cy, sy) = (c(roll), s(roll), c(pitch), s(pitch), c(yaw), s(yaw));
        let col = |r0: f32, r1: f32, r2: f32| r0 * a_w.x + r1 * a_w.y + r2 * a_w.z;
        let accel = Vec3::new(
            col(cy * cp, sy * cp, -sp),
            col(cy * sp * sr - sy * cr, sy * sp * sr + cy * cr, cp * sr),
            col(cy * sp * cr + sy * sr, sy * sp * cr - cy * sr, cp * cr),
        );
        (gyro, accel, yaw + 0.3)
    }

    /// The output of `hm/tools/ref.c` (the measured code, candidate T1a:
    /// mode 2, pull 0.2, bias gain 0.04, in double) on that sequence, with
    /// the start `roll 0.10, pitch -0.05, bias (0.005, -0.004, 0)`: every
    /// 250 steps the state `u`, the bias, and the running sum of the
    /// horizontal velocity change `A` (the delta velocity turned with the
    /// reference's roll and pitch and the yaw above). The C code was built
    /// and run for this table; the table is the committed test data.
    /// One checkpoint of the C run: the step count, `u`, the bias and the
    /// running sum of `A` (north, east).
    type Checkpoint = (usize, [f32; 3], [f32; 3], [f32; 2]);

    #[rustfmt::skip]
    const FROM_C: [Checkpoint; 12] = [
        (250, [-0.18227321, 0.19681354, 0.9633488], [0.00489735, -0.008271745, 0.0007320028], [4.189892, 4.2502933]),
        (500, [-0.3125263, 0.25554883, 0.9148891], [0.0013025538, -0.00952761, -8.542533e-05], [9.173886, 6.632624]),
        (750, [-0.22769117, 0.13100085, 0.96488106], [0.0014028199, -0.008615753, -0.0003993306], [10.549956, 5.6115794]),
        (1000, [-0.09104221, -0.27061135, 0.958374], [0.0025054459, -0.010350262, -0.0004789341], [8.982458, 3.0578923]),
        (1250, [0.15282995, -0.2757792, 0.94899356], [0.0031747064, -0.010822744, -0.0006572645], [9.387495, 2.3774343]),
        (1500, [0.35466772, -0.2962974, 0.8868025], [0.0062535102, -0.009832455, -0.0012406005], [12.927321, 5.028178]),
        (1750, [0.36678764, 0.13159704, 0.92095006], [0.009610064, -0.009832709, -0.0025834376], [15.507474, 8.978437]),
        (2000, [0.24496481, 0.26533738, 0.9325172], [0.011659782, -0.008934157, -0.0034789126], [13.935765, 11.532171]),
        (2250, [0.00020186897, 0.50163007, 0.8650822], [0.011659782, -0.008934157, -0.0034789126], [7.5940123, 17.372612]),
        (2500, [-0.243011, 0.037064712, 0.9693152], [0.011659782, -0.008934157, -0.0034789126], [5.6112328, 22.390383]),
        (2750, [-0.33628052, -0.13307267, 0.9323127], [0.009426609, -0.007089271, -0.0040879245], [9.708756, 21.409227]),
        (3000, [-0.29184675, -0.5752063, 0.7641749], [0.009478652, -0.005272324, -0.003406314], [12.441358, 19.940113]),
    ];

    /// The cost of the reference per filter step (the step and the
    /// horizontal change, as the estimator calls them) and its state in bytes
    /// (a measurement, not a pass condition): `cargo test --release -p
    /// oxide-core --lib tilt_ref::tests::cost -- --ignored --nocapture`.
    #[test]
    #[ignore = "a timing measurement: run it by hand, in release"]
    fn cost_of_the_reference_per_filter_step() {
        use std::time::Instant;
        let mut r = TiltRef::new(0.1, -0.05, Vec3::new(0.005, -0.004, 0.0));
        let seq: Vec<(Vec3<f32>, Vec3<f32>, f32)> = (0..3000).map(made_up).collect();
        let reps = 20_000_u32;
        let t0 = Instant::now();
        let mut x = 0.0_f32;
        for k in 0..reps * 3000 {
            let (gyro, accel, yaw) = seq[(k % 3000) as usize];
            let d = delta(gyro, accel);
            r.step(&d);
            let [n, e] = r.horizontal(d.dvel, [Real::cos(yaw), Real::sin(yaw)]);
            x += n + e;
        }
        let per_step = t0.elapsed().as_secs_f64() / f64::from(reps * 3000) * 1e9;
        // The test's own cos and sin of the yaw are in that figure; the
        // estimator takes them from the filter's quaternion with a square
        // root and two divisions.
        let t0 = Instant::now();
        let mut y = 0.0_f32;
        for k in 0..reps * 3000 {
            let (_, _, yaw) = seq[(k % 3000) as usize];
            y += Real::cos(yaw) + Real::sin(yaw);
        }
        let own = t0.elapsed().as_secs_f64() / f64::from(reps * 3000) * 1e9;
        println!(
            "tilt reference: {per_step:.1} ns per filter step with the yaw's cos and sin ({own:.1} ns of it the test's own), state {} bytes, checksum {x} {y}",
            core::mem::size_of::<TiltRef>()
        );
    }

    #[test]
    fn the_port_gives_what_the_measured_code_gives_on_3000_steps() {
        let mut r = TiltRef::new(0.10, -0.05, Vec3::new(0.005, -0.004, 0.0));
        let (mut a_n, mut a_e) = (0.0_f32, 0.0_f32);
        let mut worst_u = 0.0_f32;
        let mut worst_b = 0.0_f32;
        let mut worst_a = 0.0_f32;
        let mut checked = 0;
        for k in 0..3000 {
            let (gyro, accel, yaw) = made_up(k);
            let d = delta(gyro, accel);
            r.step(&d);
            let [n, e] = r.horizontal(d.dvel, [Real::cos(yaw), Real::sin(yaw)]);
            a_n += n;
            a_e += e;
            if let Some((_, u, b, a)) = FROM_C.iter().find(|row| row.0 == k + 1) {
                worst_u = worst_u.max((r.down() - Vec3::new(u[0], u[1], u[2])).norm());
                worst_b = worst_b.max((r.bias() - Vec3::new(b[0], b[1], b[2])).norm());
                worst_a = worst_a.max((a_n - a[0]).abs()).max((a_e - a[1]).abs());
                checked += 1;
            }
        }
        assert_eq!(checked, 12);
        // f32 against double over 3000 steps: rounding only.
        assert!(worst_u < 2.0e-5, "state u differs by {worst_u}");
        assert!(worst_b < 1.0e-6, "bias differs by {worst_b}");
        assert!(worst_a < 5.0e-4, "A differs by {worst_a}");
    }
}
