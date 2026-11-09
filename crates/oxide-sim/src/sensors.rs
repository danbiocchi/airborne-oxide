// SPDX-License-Identifier: AGPL-3.0-or-later
//! Sensor models (DESIGN §5.4 S1): IMU with noise, bias random walk,
//! rotor-synchronous vibration and saturation; GPS with noise, latency and
//! dropouts; barometer, magnetometer and battery monitor. The free functions
//! [`imu`], [`state`] and [`battery`] are the exact, noise-free sensors.
//!
//! Frames: world NED, body FRD. Time is `Micros`; SI units (magnetic field
//! in µT). Each sensor draws from its own [`Stream`], always in a fixed
//! order, and always the same number of draws per sample regardless of the
//! configured sigmas, so changing one parameter never shifts the sequence.

use std::collections::VecDeque;

use oxide_core::io::{
    BaroSample, BatterySample, GpsSample, ImuSample, MagSample, Stamped, StateSample,
};
use oxide_core::math::{Real, Vec3};
use oxide_core::time::Micros;

use crate::SimError;
use crate::rng::{Rng, Stream};
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

const TWO_PI: f64 = 2.0 * core::f64::consts::PI;

/// IMU error model, one sample per simulator tick.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImuModel {
    /// Gyro white-noise density, rad/s/√Hz.
    pub gyro_noise_density: f64,
    /// Accelerometer white-noise density, m/s²/√Hz.
    pub accel_noise_density: f64,
    /// Gyro turn-on bias, 1σ per axis, rad/s.
    pub gyro_bias_sigma: f64,
    /// Accelerometer turn-on bias, 1σ per axis, m/s².
    pub accel_bias_sigma: f64,
    /// Gyro bias random walk, rad/s/√s.
    pub gyro_bias_walk: f64,
    /// Accelerometer bias random walk, m/s²/√s.
    pub accel_bias_walk: f64,
    /// Broadband gyro vibration at the reference rotor speed, rad/s rms.
    pub gyro_vibration_rms: f64,
    /// Broadband accelerometer vibration at the reference speed, m/s² rms.
    pub accel_vibration_rms: f64,
    /// Rotor-synchronous gyro amplitude per rotor at full speed, rad/s.
    pub gyro_harmonic_amp: f64,
    /// Rotor-synchronous accelerometer amplitude per rotor at full speed, m/s².
    pub accel_harmonic_amp: f64,
    /// Normalised rotor speed the rms values refer to.
    pub vibration_ref_speed: f64,
    /// Gyro saturation, rad/s.
    pub gyro_range: f64,
    /// Accelerometer saturation, m/s².
    pub accel_range: f64,
}

/// GPS receiver model.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GpsModel {
    /// Sample period, µs (samples at `first_fix_us + k·period`).
    pub period_us: u64,
    /// Delay from measurement to arrival, µs.
    pub latency_us: u64,
    /// Time of the first fix, µs.
    pub first_fix_us: u64,
    /// Horizontal position noise, m (white, per sample).
    pub pos_sigma_h_m: f64,
    /// Vertical position noise, m.
    pub pos_sigma_v_m: f64,
    /// Velocity noise per axis, m/s.
    pub vel_sigma_mps: f64,
    /// Horizontal accuracy the receiver reports, m.
    pub reported_h_acc_m: f64,
    /// Vertical accuracy the receiver reports, m.
    pub reported_v_acc_m: f64,
    /// Speed accuracy the receiver reports, m/s.
    pub reported_s_acc_mps: f64,
    /// Probability that a sample is lost.
    pub dropout_probability: f64,
}

/// Barometric altimeter model.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BaroModel {
    /// Sample period, µs.
    pub period_us: u64,
    /// White noise, m.
    pub noise_sigma_m: f64,
    /// Constant reference offset, 1σ, m (drawn once).
    pub offset_sigma_m: f64,
    /// Drift random walk, m/√s.
    pub drift_walk_m: f64,
}

/// Magnetometer model.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MagModel {
    /// Sample period, µs.
    pub period_us: u64,
    /// Earth field in NED, µT.
    pub field_ned_ut: Vec3<f64>,
    /// White noise per axis, µT.
    pub noise_sigma_ut: f64,
}

/// Battery voltage and current monitor.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BatterySensorModel {
    /// Sample period, µs.
    pub period_us: u64,
    /// Voltage noise, V.
    pub voltage_sigma_v: f64,
    /// Current noise, A.
    pub current_sigma_a: f64,
}

/// All sensor models of one vehicle.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SensorConfig {
    pub imu: ImuModel,
    pub gps: GpsModel,
    pub baro: BaroModel,
    pub mag: MagModel,
    pub battery: BatterySensorModel,
}

impl SensorConfig {
    /// Typical hobby-grade hardware (MPU-6000 class IMU, 5 Hz GPS, ...).
    pub fn realistic() -> Self {
        Self {
            imu: ImuModel {
                gyro_noise_density: 8.7e-5,
                accel_noise_density: 3.9e-3,
                gyro_bias_sigma: 0.005,
                accel_bias_sigma: 0.05,
                gyro_bias_walk: 1e-4,
                accel_bias_walk: 5e-4,
                gyro_vibration_rms: 0.02,
                accel_vibration_rms: 0.5,
                gyro_harmonic_amp: 0.01,
                accel_harmonic_amp: 1.0,
                vibration_ref_speed: 0.45,
                gyro_range: 34.9,
                accel_range: 156.9,
            },
            gps: GpsModel {
                period_us: 200_000,
                latency_us: 150_000,
                first_fix_us: 0,
                pos_sigma_h_m: 0.3,
                pos_sigma_v_m: 0.5,
                vel_sigma_mps: 0.1,
                reported_h_acc_m: 0.6,
                reported_v_acc_m: 1.0,
                reported_s_acc_mps: 0.2,
                dropout_probability: 0.0,
            },
            baro: BaroModel {
                period_us: 20_000,
                noise_sigma_m: 0.1,
                offset_sigma_m: 5.0,
                drift_walk_m: 0.01,
            },
            mag: MagModel {
                period_us: 20_000,
                field_ned_ut: Vec3::new(25.0, 0.0, 43.30),
                noise_sigma_ut: 0.5,
            },
            battery: BatterySensorModel {
                period_us: 10_000,
                voltage_sigma_v: 0.02,
                current_sigma_a: 0.1,
            },
        }
    }

    /// Same rates and ranges as [`SensorConfig::realistic`]; every noise,
    /// bias, drift, offset, vibration, dropout and latency is zero.
    pub fn ideal() -> Self {
        let r = Self::realistic();
        Self {
            imu: ImuModel {
                gyro_noise_density: 0.0,
                accel_noise_density: 0.0,
                gyro_bias_sigma: 0.0,
                accel_bias_sigma: 0.0,
                gyro_bias_walk: 0.0,
                accel_bias_walk: 0.0,
                gyro_vibration_rms: 0.0,
                accel_vibration_rms: 0.0,
                gyro_harmonic_amp: 0.0,
                accel_harmonic_amp: 0.0,
                ..r.imu
            },
            gps: GpsModel {
                latency_us: 0,
                pos_sigma_h_m: 0.0,
                pos_sigma_v_m: 0.0,
                vel_sigma_mps: 0.0,
                dropout_probability: 0.0,
                ..r.gps
            },
            baro: BaroModel {
                noise_sigma_m: 0.0,
                offset_sigma_m: 0.0,
                drift_walk_m: 0.0,
                ..r.baro
            },
            mag: MagModel {
                noise_sigma_ut: 0.0,
                ..r.mag
            },
            battery: BatterySensorModel {
                voltage_sigma_v: 0.0,
                current_sigma_a: 0.0,
                ..r.battery
            },
        }
    }
}

impl Default for SensorConfig {
    fn default() -> Self {
        Self::realistic()
    }
}

/// What the sensors produced this tick (`None` = nothing new).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SensorFrame {
    pub imu: Option<Stamped<ImuSample>>,
    pub gps: Option<Stamped<GpsSample>>,
    pub baro: Option<Stamped<BaroSample>>,
    pub mag: Option<Stamped<MagSample>>,
    pub battery: Option<Stamped<BatterySample>>,
}

/// True values the sensors sample at one tick.
#[derive(Clone, Copy, Debug)]
pub struct Truth<'a, const N: usize> {
    pub body: &'a BodyState<N>,
    /// Specific force (acceleration without gravity), m/s², world frame.
    pub specific_force_world: Vec3<f64>,
    pub battery_v: f64,
    pub battery_a: f64,
    /// Rotor speed at normalised speed 1, rad/s (for the vibration phase).
    pub max_omega: f64,
}

/// `v + term`, leaving `v` bit-identical when the term is exactly zero
/// (so a zeroed error model reproduces the ideal sensor exactly).
fn add_term(v: f64, term: f64) -> f64 {
    if term == 0.0 { v } else { v + term }
}

fn add_vec(v: Vec3<f64>, t: Vec3<f64>) -> Vec3<f64> {
    Vec3::new(add_term(v.x, t.x), add_term(v.y, t.y), add_term(v.z, t.z))
}

fn unit_normals(rng: &mut Rng) -> Vec3<f64> {
    let x = rng.normal();
    let y = rng.normal();
    let z = rng.normal();
    Vec3::new(x, y, z)
}

fn due(t_us: u64, period_us: u64) -> bool {
    period_us != 0 && t_us.is_multiple_of(period_us)
}

fn period_s(period_us: u64) -> f64 {
    Micros(period_us).as_secs_f64()
}

/// All sensors of one simulated vehicle.
pub struct SensorSuite<const N: usize> {
    config: SensorConfig,
    imu_rng: Rng,
    gps_rng: Rng,
    baro_rng: Rng,
    mag_rng: Rng,
    batt_rng: Rng,
    gyro_bias: Vec3<f64>,
    accel_bias: Vec3<f64>,
    rotor_phase: [f64; N],
    baro_offset: f64,
    baro_drift: f64,
    gps_queue: VecDeque<Stamped<GpsSample>>,
    base_us: u64,
}

impl<const N: usize> SensorSuite<N> {
    /// Fails with [`SimError::TimeBase`] if any period, the GPS latency or
    /// the first-fix time is not a whole multiple of `base_us`, or a period
    /// is zero.
    pub fn new(config: SensorConfig, run_seed: u64, base_us: u64) -> Result<Self, SimError> {
        let ok = |us: u64| base_us != 0 && us.is_multiple_of(base_us);
        let periods = [
            config.gps.period_us,
            config.baro.period_us,
            config.mag.period_us,
            config.battery.period_us,
        ];
        if base_us == 0
            || periods.iter().any(|&p| p == 0 || !ok(p))
            || !ok(config.gps.latency_us)
            || !ok(config.gps.first_fix_us)
        {
            return Err(SimError::TimeBase(base_us));
        }
        let mut imu_rng = Rng::stream(run_seed, Stream::Imu);
        let gyro_bias = unit_normals(&mut imu_rng) * config.imu.gyro_bias_sigma;
        let accel_bias = unit_normals(&mut imu_rng) * config.imu.accel_bias_sigma;
        let mut rotor_phase = [0.0; N];
        for p in &mut rotor_phase {
            *p = imu_rng.range(0.0, TWO_PI);
        }
        let mut baro_rng = Rng::stream(run_seed, Stream::Baro);
        let baro_offset = baro_rng.gaussian(config.baro.offset_sigma_m);
        Ok(Self {
            config,
            imu_rng,
            gps_rng: Rng::stream(run_seed, Stream::Gps),
            baro_rng,
            mag_rng: Rng::stream(run_seed, Stream::Mag),
            batt_rng: Rng::stream(run_seed, Stream::Battery),
            gyro_bias,
            accel_bias,
            rotor_phase,
            baro_offset,
            baro_drift: 0.0,
            gps_queue: VecDeque::new(),
            base_us,
        })
    }

    /// Samples every sensor due at `t`. Call once per tick, `t` increasing
    /// by `base_us`.
    pub fn sample(&mut self, t: Micros, truth: &Truth<'_, N>) -> SensorFrame {
        SensorFrame {
            imu: Some(self.sample_imu(t, truth)),
            gps: self.sample_gps(t, truth),
            baro: self.sample_baro(t, truth),
            mag: self.sample_mag(t, truth),
            battery: self.sample_battery(t, truth),
        }
    }

    /// Current gyro bias (truth, for diagnostics), rad/s body.
    pub fn gyro_bias(&self) -> Vec3<f64> {
        self.gyro_bias
    }

    /// Current accelerometer bias (truth, for diagnostics), m/s² body.
    pub fn accel_bias(&self) -> Vec3<f64> {
        self.accel_bias
    }

    pub fn config(&self) -> &SensorConfig {
        &self.config
    }

    fn sample_imu(&mut self, t: Micros, truth: &Truth<'_, N>) -> Stamped<ImuSample> {
        let m = self.config.imu;
        let dt = period_s(self.base_us);
        let sqrt_dt = Real::sqrt(dt);
        let inv_sqrt_dt = Real::sqrt(1.0 / dt);
        let rng = &mut self.imu_rng;

        // Draw order: bias walk (gyro, accel), white noise (gyro, accel),
        // broadband vibration (gyro, accel).
        self.gyro_bias = add_vec(
            self.gyro_bias,
            unit_normals(rng) * (m.gyro_bias_walk * sqrt_dt),
        );
        self.accel_bias = add_vec(
            self.accel_bias,
            unit_normals(rng) * (m.accel_bias_walk * sqrt_dt),
        );
        let gyro_white = unit_normals(rng) * (m.gyro_noise_density * inv_sqrt_dt);
        let accel_white = unit_normals(rng) * (m.accel_noise_density * inv_sqrt_dt);
        let gyro_bb = unit_normals(rng);
        let accel_bb = unit_normals(rng);

        // Vibration scales with the square of the mean rotor speed.
        let speeds = truth.body.rotor_speed.map(|n| n.max(0.0));
        #[allow(clippy::cast_precision_loss)]
        let mean = if N == 0 {
            0.0
        } else {
            speeds.iter().sum::<f64>() / N as f64
        };
        let scale = if m.vibration_ref_speed > 0.0 {
            let r = mean / m.vibration_ref_speed;
            r * r
        } else {
            0.0
        };
        let mut gyro_vib = gyro_bb * (m.gyro_vibration_rms * scale);
        let mut accel_vib = accel_bb * (m.accel_vibration_rms * scale);
        for (phase, &n) in self.rotor_phase.iter_mut().zip(speeds.iter()) {
            *phase += n * truth.max_omega * dt;
            *phase -= TWO_PI * (*phase / TWO_PI).floor();
            let (s, c) = (Real::sin(*phase), Real::cos(*phase));
            let n2 = n * n;
            gyro_vib += Vec3::new(c, s, 0.0) * (m.gyro_harmonic_amp * n2);
            accel_vib += Vec3::new(0.5 * c, 0.5 * s, s) * (m.accel_harmonic_amp * n2);
        }

        let gyro = add_vec(
            add_vec(add_vec(truth.body.rate, self.gyro_bias), gyro_white),
            gyro_vib,
        );
        let f_body = truth
            .body
            .attitude
            .inverse_rotate(truth.specific_force_world);
        let accel = add_vec(
            add_vec(add_vec(f_body, self.accel_bias), accel_white),
            accel_vib,
        );
        let gyro = gyro.clamp_abs(Vec3::splat(m.gyro_range));
        let accel = accel.clamp_abs(Vec3::splat(m.accel_range));
        Stamped::ideal(
            t,
            ImuSample {
                gyro: gyro.cast(),
                accel: accel.cast(),
            },
        )
    }

    #[allow(clippy::cast_possible_truncation)]
    fn sample_gps(&mut self, t: Micros, truth: &Truth<'_, N>) -> Option<Stamped<GpsSample>> {
        let m = self.config.gps;
        if t.0 >= m.first_fix_us && due(t.0 - m.first_fix_us, m.period_us) {
            // Fixed draw order: position, velocity, dropout.
            let pos_noise = self.gps_rng.gaussian_vec(Vec3::new(
                m.pos_sigma_h_m,
                m.pos_sigma_h_m,
                m.pos_sigma_v_m,
            ));
            let vel_noise = self.gps_rng.gaussian_vec(Vec3::splat(m.vel_sigma_mps));
            let lost = self.gps_rng.chance(m.dropout_probability);
            if !lost {
                self.gps_queue.push_back(Stamped {
                    t_sample: t,
                    t_arrival: t.add_us(m.latency_us),
                    value: GpsSample {
                        position: add_vec(truth.body.position, pos_noise).cast(),
                        velocity: add_vec(truth.body.velocity, vel_noise).cast(),
                        h_acc_m: m.reported_h_acc_m as f32,
                        v_acc_m: m.reported_v_acc_m as f32,
                        s_acc_mps: m.reported_s_acc_mps as f32,
                        fix_3d: true,
                    },
                });
            }
        }
        if self.gps_queue.front().is_some_and(|s| s.t_arrival <= t) {
            self.gps_queue.pop_front()
        } else {
            None
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    fn sample_baro(&mut self, t: Micros, truth: &Truth<'_, N>) -> Option<Stamped<BaroSample>> {
        let m = self.config.baro;
        if !due(t.0, m.period_us) {
            return None;
        }
        let walk = self.baro_rng.normal();
        let noise = self.baro_rng.normal();
        self.baro_drift = add_term(
            self.baro_drift,
            m.drift_walk_m * Real::sqrt(period_s(m.period_us)) * walk,
        );
        let alt = add_term(-truth.body.position.z, self.baro_offset);
        let alt = add_term(alt, self.baro_drift);
        let alt = add_term(alt, m.noise_sigma_m * noise);
        Some(Stamped::ideal(
            t,
            BaroSample {
                altitude_m: alt as f32,
            },
        ))
    }

    fn sample_mag(&mut self, t: Micros, truth: &Truth<'_, N>) -> Option<Stamped<MagSample>> {
        let m = self.config.mag;
        if !due(t.0, m.period_us) {
            return None;
        }
        let noise = unit_normals(&mut self.mag_rng) * m.noise_sigma_ut;
        let field = truth.body.attitude.inverse_rotate(m.field_ned_ut);
        Some(Stamped::ideal(
            t,
            MagSample {
                field_ut: add_vec(field, noise).cast(),
            },
        ))
    }

    fn sample_battery(
        &mut self,
        t: Micros,
        truth: &Truth<'_, N>,
    ) -> Option<Stamped<BatterySample>> {
        let m = self.config.battery;
        if !due(t.0, m.period_us) {
            return None;
        }
        let nv = self.batt_rng.normal();
        let na = self.batt_rng.normal();
        Some(battery(
            t,
            add_term(truth.battery_v, m.voltage_sigma_v * nv),
            add_term(truth.battery_a, m.current_sigma_a * na),
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxide_core::math::Quat;

    const BASE: u64 = 1000;

    fn tilted_body() -> BodyState<4> {
        BodyState {
            position: Vec3::new(1.0, -2.0, -3.5),
            velocity: Vec3::new(0.5, 0.25, -0.1),
            attitude: Quat::from_axis_angle(Vec3::new(0.3, 0.5, 0.8), 0.7),
            rate: Vec3::new(0.1, -0.2, 0.3),
            rotor_speed: [0.0; 4],
        }
    }

    fn truth(body: &BodyState<4>) -> Truth<'_, 4> {
        Truth {
            body,
            specific_force_world: Vec3::new(0.3, -0.4, -9.8),
            battery_v: 15.2,
            battery_a: 12.5,
            max_omega: 2000.0,
        }
    }

    fn run(cfg: SensorConfig, seed: u64, body: &BodyState<4>, ticks: u64) -> Vec<SensorFrame> {
        let mut s = SensorSuite::<4>::new(cfg, seed, BASE).unwrap();
        (0..ticks)
            .map(|k| s.sample(Micros(k * BASE), &truth(body)))
            .collect()
    }

    #[test]
    fn ideal_imu_matches_free_function_exactly() {
        let body = tilted_body();
        let tr = truth(&body);
        let mut s = SensorSuite::<4>::new(SensorConfig::ideal(), 5, BASE).unwrap();
        for k in 0..20 {
            let t = Micros(k * BASE);
            let got = s.sample(t, &tr).imu.unwrap();
            let want = imu(t, &body, tr.specific_force_world);
            assert_eq!(got, want);
            assert_eq!(got.value.gyro.x.to_bits(), want.value.gyro.x.to_bits());
            assert_eq!(got.value.accel.z.to_bits(), want.value.accel.z.to_bits());
        }
    }

    #[test]
    fn ideal_other_sensors_equal_truth_at_right_instants() {
        let body = tilted_body();
        let cfg = SensorConfig::ideal();
        let frames = run(cfg, 1, &body, 500);
        for (k, f) in frames.iter().enumerate() {
            let t = (k as u64) * BASE;
            assert_eq!(f.gps.is_some(), t.is_multiple_of(200_000), "gps at {t}");
            assert_eq!(f.baro.is_some(), t.is_multiple_of(20_000));
            assert_eq!(f.mag.is_some(), t.is_multiple_of(20_000));
            assert_eq!(f.battery.is_some(), t.is_multiple_of(10_000));
            if let Some(g) = f.gps {
                assert_eq!(g.t_sample, g.t_arrival);
                assert_eq!(g.value.position, body.position.cast());
                assert_eq!(g.value.velocity, body.velocity.cast());
                assert!(g.value.fix_3d);
            }
            if let Some(b) = f.baro {
                assert_eq!(b.value.altitude_m, 3.5);
            }
            if let Some(m) = f.mag {
                let want = body.attitude.inverse_rotate(cfg.mag.field_ned_ut);
                assert_eq!(m.value.field_ut, want.cast());
            }
            if let Some(b) = f.battery {
                assert_eq!(b.value.voltage_v, 15.2);
                assert_eq!(b.value.current_a, 12.5);
            }
        }
    }

    #[test]
    fn gyro_white_noise_matches_density() {
        let mut cfg = SensorConfig::ideal();
        cfg.imu.gyro_noise_density = 8.7e-5;
        let body = BodyState::<4>::default();
        let tr = truth(&body);
        let mut s = SensorSuite::<4>::new(cfg, 11, BASE).unwrap();
        let n = 100_000_u32;
        let (mut sum, mut sum2) = (0.0, 0.0);
        for k in 0..u64::from(n) {
            let g = f64::from(s.sample(Micros(k * BASE), &tr).imu.unwrap().value.gyro.x);
            sum += g;
            sum2 += g * g;
        }
        let mean = sum / f64::from(n);
        let var = sum2 / f64::from(n) - mean * mean;
        let std = Real::sqrt(var);
        let want = 8.7e-5 / Real::sqrt(BASE as f64 * 1e-6);
        assert!((std / want - 1.0).abs() < 0.05, "std {std} want {want}");
    }

    #[test]
    fn gps_latency_delivers_in_later_frame() {
        let body = tilted_body();
        let mut cfg = SensorConfig::ideal();
        cfg.gps.latency_us = 150_000;
        let frames = run(cfg, 1, &body, 600);
        let arrivals: Vec<_> = frames
            .iter()
            .enumerate()
            .filter_map(|(k, f)| f.gps.map(|g| (k as u64 * BASE, g)))
            .collect();
        let (frame_t, g) = arrivals
            .iter()
            .find(|(_, g)| g.t_sample == Micros(200_000))
            .unwrap();
        assert_eq!(*frame_t, 350_000);
        assert_eq!(g.t_arrival, Micros(350_000));
        // 150 ms latency, 200 ms period: one arrival per period, none lost.
        assert_eq!(arrivals.len(), 3);
        assert_eq!(arrivals[0].0, 150_000);
    }

    #[test]
    fn gps_dropout_probability_one_gives_nothing() {
        let body = tilted_body();
        let mut cfg = SensorConfig::ideal();
        cfg.gps.dropout_probability = 1.0;
        assert!(run(cfg, 1, &body, 1000).iter().all(|f| f.gps.is_none()));
    }

    #[test]
    fn gps_dropout_does_not_shift_noise_sequence() {
        let body = tilted_body();
        let mut a = SensorConfig::realistic();
        a.gps.latency_us = 0;
        let mut b = a;
        b.gps.dropout_probability = 0.5;
        let fa = run(a, 3, &body, 3000);
        let fb = run(b, 3, &body, 3000);
        let mut matched = 0;
        for (x, y) in fa.iter().zip(&fb) {
            if let (Some(x), Some(y)) = (x.gps, y.gps) {
                assert_eq!(x, y);
                matched += 1;
            }
        }
        assert!(matched > 0);
    }

    #[test]
    fn vibration_zero_when_stopped_present_when_spinning() {
        let mut cfg = SensorConfig::ideal();
        cfg.imu.gyro_vibration_rms = 0.02;
        cfg.imu.accel_vibration_rms = 0.5;
        cfg.imu.gyro_harmonic_amp = 0.01;
        cfg.imu.accel_harmonic_amp = 1.0;
        let mut body = BodyState::<4>::default();
        let tr = truth(&body);
        let mut s = SensorSuite::<4>::new(cfg, 2, BASE).unwrap();
        let ideal = imu(Micros(0), &body, tr.specific_force_world).value;
        for k in 0..100 {
            let v = s.sample(Micros(k * BASE), &tr).imu.unwrap().value;
            assert_eq!(v, ideal);
        }
        body.rotor_speed = [0.6; 4];
        let tr = truth(&body);
        let mut dev = 0.0_f64;
        for k in 100..300 {
            let v = s.sample(Micros(k * BASE), &tr).imu.unwrap().value;
            dev = dev.max(f64::from((v.accel.x - ideal.accel.x).abs()));
        }
        assert!(dev > 0.1, "vibration {dev}");
    }

    #[test]
    fn imu_saturates() {
        let body = BodyState::<4> {
            rate: Vec3::new(100.0, -100.0, 0.0),
            ..Default::default()
        };
        let cfg = SensorConfig::ideal();
        let tr = Truth {
            specific_force_world: Vec3::new(0.0, 0.0, -1000.0),
            ..truth(&body)
        };
        let mut s = SensorSuite::<4>::new(cfg, 2, BASE).unwrap();
        let v = s.sample(Micros(0), &tr).imu.unwrap().value;
        assert_eq!(v.gyro.x, 34.9_f32);
        assert_eq!(v.gyro.y, -34.9_f32);
        assert_eq!(v.accel.z, -156.9_f32);
    }

    #[test]
    fn deterministic_per_seed() {
        let mut body = tilted_body();
        body.rotor_speed = [0.5; 4];
        let cfg = SensorConfig::realistic();
        let a = run(cfg, 9, &body, 10_000);
        let b = run(cfg, 9, &body, 10_000);
        let c = run(cfg, 10, &body, 10_000);
        assert_eq!(a, b);
        assert_ne!(a, c);
    }

    #[test]
    fn bad_periods_rejected() {
        let mk = |f: fn(&mut SensorConfig)| {
            let mut c = SensorConfig::realistic();
            f(&mut c);
            SensorSuite::<4>::new(c, 1, BASE).map(|_| ())
        };
        assert!(mk(|_| {}).is_ok());
        assert_eq!(mk(|c| c.gps.period_us = 0), Err(SimError::TimeBase(BASE)));
        assert_eq!(
            mk(|c| c.baro.period_us = 20_500),
            Err(SimError::TimeBase(BASE))
        );
        assert_eq!(mk(|c| c.mag.period_us = 0), Err(SimError::TimeBase(BASE)));
        assert_eq!(
            mk(|c| c.battery.period_us = 1500),
            Err(SimError::TimeBase(BASE))
        );
        assert_eq!(
            mk(|c| c.gps.latency_us = 150_500),
            Err(SimError::TimeBase(BASE))
        );
    }

    #[test]
    fn biases_start_random_and_walk() {
        let body = BodyState::<4>::default();
        let tr = truth(&body);
        let mut s = SensorSuite::<4>::new(SensorConfig::realistic(), 4, BASE).unwrap();
        let b0 = s.gyro_bias();
        assert!(b0.norm() > 0.0);
        for k in 0..1000 {
            s.sample(Micros(k * BASE), &tr);
        }
        assert_ne!(s.gyro_bias(), b0);
        assert!(s.accel_bias().is_finite());
    }
}
