// SPDX-License-Identifier: AGPL-3.0-or-later
//! Sensor models (DESIGN §5.4 S1): IMU with noise, bias random walk,
//! rotor-synchronous vibration and saturation; GPS with noise, latency and
//! dropouts; barometer, magnetometer and battery monitor. The free functions
//! [`imu`], [`state`] and [`battery`] are the exact, noise-free sensors.
//!
//! Frames: world NED, body FRD. Time is `Micros`; SI units (magnetic field
//! in µT). Each sensor draws from its own [`Stream`], always in a fixed
//! order, and always the same number of draws per sample (for the IMU, per
//! internal sample: see [`ImuModel`]) regardless of the configured sigmas,
//! so changing one parameter never shifts the sequence.

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

/// IMU error model.
///
/// # How a sample is made
///
/// The IMU runs on a clock of its own, as a real one does, whoever reads it.
/// Every [`internal_period_us`](ImuModel::internal_period_us) it draws its
/// bias walk and its noise, and the noise passes its anti-alias low-pass
/// filter. A reader is given the latest filtered sample at its own
/// [`period_us`](ImuModel::period_us) (decimation), with the truth, the bias
/// and the rotor-synchronous lines of that instant added. So the noise a
/// reader sees has the same density in the band the filter passes whatever
/// its period: a reader at 1 kHz and one at 4 kHz are given the same internal
/// stream, one every eighth and one every second sample of it (with the same
/// seed and the same truth, the 1 kHz samples' noise is every fourth 4 kHz
/// sample's noise, bit for bit). Only what lies above the reader's Nyquist
/// frequency and still
/// passes the filter folds back: in 0 to 200 Hz a 1 kHz reader gets 0.45 %
/// more noise amplitude than a 4 kHz reader, worked out from the filter
/// below.
///
/// A density `D` here means what it has always meant for the white noise:
/// samples of it taken at a rate `f`, unfiltered, would have a standard
/// deviation of `D·√f`; filtered, the power at each frequency is `D²·|H|²`.
///
/// # The values of [`SensorConfig::realistic`], and why
///
/// - **Internal rate 8 kHz** (125 µs): the gyro sample rate of the
///   MPU-6000, the class of IMU this model stands for, with its widest
///   low-pass setting (`DLPF_CFG` 0 in its register map). It divides every
///   step the simulator flies (250 µs for the H743 profile, 500 µs for a
///   bridged flight, 1000 µs for the F405 profile), and it puts the
///   filter's band far below its own Nyquist frequency.
/// - **Anti-alias filter: second-order Butterworth, −3 dB at 256 Hz**, by
///   the bilinear transform at the internal rate. 256 Hz is the MPU-6000's
///   gyro bandwidth at that setting. It is half the Nyquist frequency of the
///   slowest reader (500 Hz at 1 kHz), so little folds back (above), and it
///   is above the band the loops use (our core's gyro filter and
///   ArduCopter's `INS_GYRO_FILTER` are both at 80 Hz), so what they use is
///   the density itself (at 100 Hz the filter passes 97.7 % of the power).
///   The datasheet gives a bandwidth and a delay, not a shape; second order
///   is the lowest that keeps the fold-back that small. The filter acts on
///   the noise only. The truth, the bias and the rotor lines reach the
///   reader unfiltered: the delay the filter would put on the signal (about
///   0.88 ms below 100 Hz for this one) is the separate S2 item
///   "output-filter delay" of DESIGN §5.4. Its power gain on white noise is
///   that of a band of 565.6 Hz (two-sided), so a density `D` reaches every
///   reader as a standard deviation of `D·√565.6`.
/// - **Broadband vibration: a density, flat from 0 to 4 kHz** (the internal
///   Nyquist frequency), at the reference rotor speed: gyro
///   6.325e-4 rad/s/√Hz, accelerometer 1.581e-2 m/s²/√Hz, scaled with the
///   square of the mean rotor speed. These are 0.02 rad/s and 0.5 m/s² rms
///   at 1 kHz, the figures the model had as a fixed rms per sample: the
///   density the F405 profile and ArduCopter (on the same 1000 µs grid)
///   have flown on until now, kept where the loops use it so that the
///   change takes away the dependence on the reader's rate and nothing else.
///   ArduCopter's tuning flight (ADR-0007 §2) was flown on it too. At the
///   H743's 4 kHz the old fixed rms was half this density. The 4 kHz band
///   matters to what a reader gets only by being far above 256 Hz: the
///   filter passes the density below its cut-off and removes the rest.
///   Delivered at hover, the vibration is 0.0150 rad/s and 0.376 m/s² rms
///   per sample, at every reader's rate.
/// - **White noise** keeps its MPU-6000 densities and now passes the filter
///   too: 2.1e-3 rad/s and 0.093 m/s² rms per sample at every rate, where
///   it was `D·√f` at the reader's rate `f` (the same density below 256 Hz).
///
/// Every internal sample draws the same number of variates in the same
/// order (bias walk, gyro then accelerometer; noise, gyro then
/// accelerometer; the white noise and the vibration share one draw per
/// axis, with the variance of both), whatever the densities, so changing
/// one never shifts the sequence.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImuModel {
    /// The reader's sample period, µs: 0 gives a sample at every simulator
    /// tick; otherwise a whole multiple of the tick, and a sample is given at
    /// the ticks where `t` is a multiple of it. The rotor lines' phase
    /// advances by this period (or the tick) from one sample to the next. The
    /// IMU's own clock, bias walk and noise run whatever this is. Our flight
    /// core needs a sample on every tick, so only another autopilot may set
    /// it.
    pub period_us: u64,
    /// The IMU's internal sampling period, µs: every simulator tick must be a
    /// whole multiple of it.
    pub internal_period_us: u64,
    /// The anti-alias filter's −3 dB frequency, Hz (second-order
    /// Butterworth at the internal rate, on the noise), below the internal
    /// Nyquist frequency.
    pub filter_cutoff_hz: f64,
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
    /// Broadband gyro vibration density at the reference rotor speed,
    /// rad/s/√Hz, flat up to the internal Nyquist frequency.
    pub gyro_vibration_density: f64,
    /// Broadband accelerometer vibration density at the reference speed,
    /// m/s²/√Hz, flat up to the internal Nyquist frequency.
    pub accel_vibration_density: f64,
    /// Rotor-synchronous gyro amplitude per rotor at full speed, rad/s, as
    /// delivered (not filtered; a line above the reader's Nyquist frequency
    /// folds back, as it would on an IMU with no filter: at the reference
    /// quad's 30,000 rpm the line reaches 500 Hz at full speed, 225 Hz at
    /// hover).
    pub gyro_harmonic_amp: f64,
    /// Rotor-synchronous accelerometer amplitude per rotor at full speed,
    /// m/s², as delivered (as the gyro's).
    pub accel_harmonic_amp: f64,
    /// Normalised rotor speed the vibration densities refer to.
    pub vibration_ref_speed: f64,
    /// Gyro saturation, rad/s.
    pub gyro_range: f64,
    /// Accelerometer saturation, m/s².
    pub accel_range: f64,
}

impl ImuModel {
    /// The anti-alias filter's coefficients `[b0, b1, b2, a1, a2]`
    /// (second-order Butterworth by the bilinear transform with the cut-off
    /// prewarped), or `None` when the internal period is zero or the cut-off
    /// is not between 0 and the internal Nyquist frequency.
    pub fn filter_coefficients(&self) -> Option<[f64; 5]> {
        if self.internal_period_us == 0 {
            return None;
        }
        let fs = 1.0 / period_s(self.internal_period_us);
        let fc = self.filter_cutoff_hz;
        if !(fc.is_finite() && fc > 0.0 && fc < 0.5 * fs) {
            return None;
        }
        let k = Real::tan(core::f64::consts::PI * fc / fs);
        let r2 = core::f64::consts::SQRT_2;
        let kk = k * k;
        let n = 1.0 / (1.0 + r2 * k + kk);
        let b0 = kk * n;
        Some([
            b0,
            2.0 * b0,
            b0,
            2.0 * (kk - 1.0) * n,
            (1.0 - r2 * k + kk) * n,
        ])
    }

    /// The filter's power gain on white noise as a two-sided bandwidth, Hz:
    /// a density `D` reaches the reader as a standard deviation of
    /// `D·√(this)`. Summed over the impulse response (`None` as for
    /// [`filter_coefficients`](Self::filter_coefficients)).
    pub fn noise_bandwidth_hz(&self) -> Option<f64> {
        let c = self.filter_coefficients()?;
        let mut f = Lowpass::default();
        let mut sum = 0.0;
        for i in 0..16_384 {
            let x = if i == 0 { 1.0 } else { 0.0 };
            let y = f.step(&c, Vec3::splat(x)).x;
            sum += y * y;
        }
        Some(sum / period_s(self.internal_period_us))
    }
}

/// A second-order section per axis, transposed direct form II.
#[derive(Clone, Copy, Debug, Default)]
struct Lowpass {
    z1: Vec3<f64>,
    z2: Vec3<f64>,
}

impl Lowpass {
    /// One input sample through the filter; returns the output. Zero in and
    /// zero state give exactly zero out.
    fn step(&mut self, c: &[f64; 5], x: Vec3<f64>) -> Vec3<f64> {
        let [b0, b1, b2, a1, a2] = *c;
        let y = x * b0 + self.z1;
        self.z1 = x * b1 - y * a1 + self.z2;
        self.z2 = x * b2 - y * a2;
        y
    }
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
                period_us: 0,
                internal_period_us: 125,
                filter_cutoff_hz: 256.0,
                gyro_noise_density: 8.7e-5,
                accel_noise_density: 3.9e-3,
                gyro_bias_sigma: 0.005,
                accel_bias_sigma: 0.05,
                gyro_bias_walk: 1e-4,
                accel_bias_walk: 5e-4,
                // 0.02 rad/s and 0.5 m/s² rms at 1 kHz (see ImuModel).
                gyro_vibration_density: 6.324_555_320_336_759e-4,
                accel_vibration_density: 1.581_138_830_084_189_6e-2,
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
                gyro_vibration_density: 0.0,
                accel_vibration_density: 0.0,
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
    /// The IMU's own clock: the time of the next tick, µs, counted from the
    /// first tick (pre-roll included) and advanced by the tick.
    imu_clock_us: u64,
    /// Index of the next internal sample: internal sample `j` is at
    /// `j · internal_period_us` on the IMU's clock.
    imu_next: u64,
    /// Anti-alias filter coefficients and state, gyro and accelerometer.
    imu_filter: [f64; 5],
    gyro_lp: Lowpass,
    accel_lp: Lowpass,
    /// The latest filtered noise, what a reader is given.
    gyro_noise: Vec3<f64>,
    accel_noise: Vec3<f64>,
    baro_offset: f64,
    baro_drift: f64,
    gps_queue: VecDeque<Stamped<GpsSample>>,
    base_us: u64,
}

impl<const N: usize> SensorSuite<N> {
    /// Fails with [`SimError::TimeBase`] if any period, the GPS latency or
    /// the first-fix time is not a whole multiple of `base_us`, or a period
    /// is zero; with [`SimError::Imu`] if `base_us` is not a whole multiple
    /// of the IMU's internal period or its filter cannot be built
    /// ([`ImuModel::filter_coefficients`]).
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
            || !ok(config.imu.period_us)
            || !ok(config.gps.latency_us)
            || !ok(config.gps.first_fix_us)
        {
            return Err(SimError::TimeBase(base_us));
        }
        let ip = config.imu.internal_period_us;
        if ip == 0 || !base_us.is_multiple_of(ip) {
            return Err(SimError::Imu(format!(
                "the {base_us} µs step is not a whole multiple of the IMU's internal period of {ip} µs"
            )));
        }
        let Some(imu_filter) = config.imu.filter_coefficients() else {
            return Err(SimError::Imu(format!(
                "the IMU filter's cut-off of {} Hz is not between 0 and the internal Nyquist frequency",
                config.imu.filter_cutoff_hz
            )));
        };
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
            imu_clock_us: 0,
            imu_next: 0,
            imu_filter,
            gyro_lp: Lowpass::default(),
            accel_lp: Lowpass::default(),
            gyro_noise: Vec3::zero(),
            accel_noise: Vec3::zero(),
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
            imu: self.sample_imu(t, truth),
            gps: self.sample_gps(t, truth),
            baro: self.sample_baro(t, truth),
            mag: self.sample_mag(t, truth),
            battery: self.sample_battery(t, truth),
        }
    }

    /// The IMU sample of pre-roll step `k` (the steps count from 0, one per
    /// tick) for a vehicle at rest: `None` when the IMU's period has no
    /// sample due at `k` ticks, else the sample, built by the code `sample`
    /// uses. Advances only the IMU's state (its own clock, random stream,
    /// bias walk, filter and rotor phases), by one tick: no other sensor and
    /// no queue.
    pub fn preroll_imu(&mut self, k: u64, truth: &Truth<'_, N>) -> Option<ImuSample> {
        self.advance_imu(truth);
        let p = self.config.imu.period_us;
        if p != 0 && !due(k.saturating_mul(self.base_us), p) {
            return None;
        }
        Some(self.imu_sample(truth))
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

    /// The IMU sample due at `t`, or `None` between the samples of a
    /// coarser period. The IMU's own clock advances by one tick either way.
    fn sample_imu(&mut self, t: Micros, truth: &Truth<'_, N>) -> Option<Stamped<ImuSample>> {
        self.advance_imu(truth);
        let p = self.config.imu.period_us;
        if p != 0 && !due(t.0, p) {
            return None;
        }
        Some(Stamped::ideal(t, self.imu_sample(truth)))
    }

    /// Runs the IMU's internal sampling up to this tick: every internal
    /// sample due at or before the tick's time on the IMU's clock (the first
    /// tick makes sample 0), each a bias-walk step and one noise sample
    /// through the filter. The noise's density is set from the rotor speeds
    /// of this tick. Then the clock moves on by one tick.
    fn advance_imu(&mut self, truth: &Truth<'_, N>) {
        let m = self.config.imu;
        let ip = m.internal_period_us;
        let dt = period_s(ip);
        let sqrt_dt = Real::sqrt(dt);
        let sqrt_rate = Real::sqrt(1.0 / dt);
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
        // White noise and broadband vibration are independent and both white
        // at the internal rate: one draw per axis with the variance of both.
        let density = |white: f64, vib: f64| {
            let v = vib * scale;
            Real::sqrt(white * white + v * v)
        };
        let gyro_sigma = density(m.gyro_noise_density, m.gyro_vibration_density) * sqrt_rate;
        let accel_sigma = density(m.accel_noise_density, m.accel_vibration_density) * sqrt_rate;
        let last = self.imu_clock_us / ip;
        let rng = &mut self.imu_rng;
        while self.imu_next <= last {
            // Draw order: bias walk (gyro, accel), noise (gyro, accel).
            self.gyro_bias = add_vec(
                self.gyro_bias,
                unit_normals(rng) * (m.gyro_bias_walk * sqrt_dt),
            );
            self.accel_bias = add_vec(
                self.accel_bias,
                unit_normals(rng) * (m.accel_bias_walk * sqrt_dt),
            );
            let g = unit_normals(rng) * gyro_sigma;
            let a = unit_normals(rng) * accel_sigma;
            self.gyro_noise = self.gyro_lp.step(&self.imu_filter, g);
            self.accel_noise = self.accel_lp.step(&self.imu_filter, a);
            self.imu_next += 1;
        }
        self.imu_clock_us = self.imu_clock_us.saturating_add(self.base_us);
    }

    /// The sample a reader is given now: the truth, the bias, the latest
    /// filtered noise and the rotor lines, saturated. Advances the rotor
    /// phases (by the reader's period, or the tick when it has none) and
    /// nothing else.
    fn imu_sample(&mut self, truth: &Truth<'_, N>) -> ImuSample {
        let m = self.config.imu;
        let dt = period_s(if m.period_us == 0 {
            self.base_us
        } else {
            m.period_us
        });
        let speeds = truth.body.rotor_speed.map(|n| n.max(0.0));
        let mut gyro_line = Vec3::zero();
        let mut accel_line = Vec3::zero();
        for (phase, &n) in self.rotor_phase.iter_mut().zip(speeds.iter()) {
            *phase += n * truth.max_omega * dt;
            *phase -= TWO_PI * (*phase / TWO_PI).floor();
            let (s, c) = (Real::sin(*phase), Real::cos(*phase));
            let n2 = n * n;
            gyro_line += Vec3::new(c, s, 0.0) * (m.gyro_harmonic_amp * n2);
            accel_line += Vec3::new(0.5 * c, 0.5 * s, s) * (m.accel_harmonic_amp * n2);
        }

        let gyro = add_vec(
            add_vec(add_vec(truth.body.rate, self.gyro_bias), self.gyro_noise),
            gyro_line,
        );
        let f_body = truth
            .body
            .attitude
            .inverse_rotate(truth.specific_force_world);
        let accel = add_vec(
            add_vec(add_vec(f_body, self.accel_bias), self.accel_noise),
            accel_line,
        );
        let gyro = gyro.clamp_abs(Vec3::splat(m.gyro_range));
        let accel = accel.clamp_abs(Vec3::splat(m.accel_range));
        ImuSample {
            gyro: gyro.cast(),
            accel: accel.cast(),
        }
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

    /// The filter's power gain on white noise, as a two-sided band: the
    /// integral of |H|² over the internal band, worked out numerically from
    /// the coefficients (`2·∫₀^{4 kHz} |H(f)|² df`), independently of
    /// `noise_bandwidth_hz`.
    const NOISE_BAND_HZ: f64 = 565.576;

    #[test]
    fn the_filter_is_the_documented_one() {
        let m = SensorConfig::realistic().imu;
        assert_eq!(m.internal_period_us, 125);
        assert_eq!(m.filter_cutoff_hz, 256.0);
        let [b0, b1, b2, a1, a2] = m.filter_coefficients().expect("filter");
        // Unit gain at DC, as a low-pass must have.
        assert!(((b0 + b1 + b2) / (1.0 + a1 + a2) - 1.0).abs() < 1e-12);
        let band = m.noise_bandwidth_hz().expect("band");
        assert!((band - NOISE_BAND_HZ).abs() < 0.01, "band {band}");
        // The vibration densities are the 1 kHz rms figures as densities.
        let want = |rms: f64| rms / Real::sqrt(1000.0);
        assert!((m.gyro_vibration_density / want(0.02) - 1.0).abs() < 1e-12);
        assert!((m.accel_vibration_density / want(0.5) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn a_step_or_filter_that_does_not_fit_is_refused() {
        let mk = |f: fn(&mut SensorConfig), base: u64| {
            let mut c = SensorConfig::realistic();
            f(&mut c);
            SensorSuite::<4>::new(c, 1, base).map(|_| ())
        };
        assert!(mk(|_| {}, 250).is_ok());
        assert!(mk(|_| {}, 500).is_ok());
        assert!(matches!(mk(|_| {}, 200), Err(SimError::Imu(_))));
        assert!(matches!(
            mk(|c| c.imu.internal_period_us = 0, 1000),
            Err(SimError::Imu(_))
        ));
        assert!(matches!(
            mk(|c| c.imu.filter_cutoff_hz = 4000.0, 1000),
            Err(SimError::Imu(_))
        ));
        assert!(matches!(
            mk(|c| c.imu.filter_cutoff_hz = 0.0, 1000),
            Err(SimError::Imu(_))
        ));
        assert!(matches!(
            mk(|c| c.imu.filter_cutoff_hz = f64::NAN, 1000),
            Err(SimError::Imu(_))
        ));
    }

    /// The white noise reaches every reader through the filter, with the
    /// same standard deviation per sample: `D·√565.6`.
    #[test]
    fn gyro_white_noise_matches_density() {
        let mut cfg = SensorConfig::ideal();
        cfg.imu.gyro_noise_density = 8.7e-5;
        let body = BodyState::<4>::default();
        let tr = truth(&body);
        let want = 8.7e-5 * Real::sqrt(NOISE_BAND_HZ);
        for base in [1000_u64, 250] {
            let mut s = SensorSuite::<4>::new(cfg, 11, base).unwrap();
            let n = 100_000_u32;
            let (mut sum, mut sum2) = (0.0, 0.0);
            for k in 0..u64::from(n) {
                let g = f64::from(s.sample(Micros(k * base), &tr).imu.unwrap().value.gyro.x);
                sum += g;
                sum2 += g * g;
            }
            let mean = sum / f64::from(n);
            let var = sum2 / f64::from(n) - mean * mean;
            let std = Real::sqrt(var);
            assert!(
                (std / want - 1.0).abs() < 0.05,
                "step {base}: std {std} want {want}"
            );
        }
    }

    /// A vehicle hovering at the reference rotor speed, with no rotor lines,
    /// bias or walk: the IMU's noise alone (white and vibration).
    fn noise_only() -> (SensorConfig, BodyState<4>) {
        let r = SensorConfig::realistic().imu;
        let mut cfg = SensorConfig::ideal();
        cfg.imu.gyro_noise_density = r.gyro_noise_density;
        cfg.imu.accel_noise_density = r.accel_noise_density;
        cfg.imu.gyro_vibration_density = r.gyro_vibration_density;
        cfg.imu.accel_vibration_density = r.accel_vibration_density;
        let body = BodyState::<4> {
            rotor_speed: [r.vibration_ref_speed; 4],
            ..BodyState::default()
        };
        (cfg, body)
    }

    /// The gyro and accelerometer samples of `seconds` of a reader at the
    /// step `base` (its period).
    fn noise_samples(seed: u64, base: u64, seconds: u64) -> Vec<[f64; 6]> {
        let (cfg, body) = noise_only();
        let tr = truth(&body);
        let mut s = SensorSuite::<4>::new(cfg, seed, base).unwrap();
        (0..seconds * 1_000_000 / base)
            .map(|k| {
                let v = s.sample(Micros(k * base), &tr).imu.unwrap().value;
                let (g, a) = (v.gyro, v.accel);
                [g.x, g.y, g.z, a.x, a.y, a.z].map(f64::from)
            })
            .collect()
    }

    /// Mean two-sided power density of channel `c` over the whole-hertz
    /// bins `lo..=hi`, from periodograms of one-second segments (Goertzel per
    /// bin): `|X|² / (N · fs)`, which is `D²` for white noise of density `D`.
    fn mean_density2(x: &[[f64; 6]], fs: usize, c: usize, lo: usize, hi: usize) -> f64 {
        let segments = x.len() / fs;
        let mut total = 0.0;
        for seg in x.chunks_exact(fs) {
            for k in lo..=hi {
                #[allow(clippy::cast_precision_loss)]
                let coeff = 2.0 * Real::cos(TWO_PI * k as f64 / fs as f64);
                let (mut s1, mut s2) = (0.0, 0.0);
                for v in seg {
                    let s0 = v[c] + coeff * s1 - s2;
                    s2 = s1;
                    s1 = s0;
                }
                total += s1 * s1 + s2 * s2 - coeff * s1 * s2;
            }
        }
        #[allow(clippy::cast_precision_loss)]
        let n = (segments * (hi - lo + 1)) as f64;
        #[allow(clippy::cast_precision_loss)]
        let norm = (fs * fs) as f64;
        total / n / norm
    }

    /// The goal of the IMU's filter and decimation: the noise a reader is
    /// given has the same density at 1 kHz and at 4 kHz (the F405 and H743
    /// steps). Measured on independent seeds over 64 s, in 1 to 100 Hz
    /// (against the configured density too) and in 100 to 250 Hz, within
    /// 3 % in amplitude. What the filter lets fold back into a 1 kHz
    /// reader is 0.4 % in 1 to 100 Hz and 0.8 % in 100 to 250 Hz (worked
    /// out from the filter); each estimate scatters by about 0.4 % (19,200
    /// periodogram values in the low band). The model this replaced gave the
    /// 1 kHz reader twice the 4 kHz reader's vibration density.
    #[test]
    fn noise_density_is_the_same_at_1_khz_and_4_khz() {
        let (cfg, _) = noise_only();
        let m = cfg.imu;
        let hypot = |a: f64, b: f64| Real::sqrt(a * a + b * b);
        let d = [
            hypot(m.gyro_noise_density, m.gyro_vibration_density),
            hypot(m.accel_noise_density, m.accel_vibration_density),
        ];
        let slow = noise_samples(21, 1000, 64);
        let fast = noise_samples(22, 250, 64);
        for (sensor, channels) in [(0, 0..3), (1, 3..6)] {
            let want = d[sensor];
            let band = |x: &[[f64; 6]], fs, lo, hi| {
                let p = channels
                    .clone()
                    .map(|c| mean_density2(x, fs, c, lo, hi))
                    .sum::<f64>()
                    / 3.0;
                Real::sqrt(p)
            };
            let (s_lo, f_lo) = (band(&slow, 1000, 1, 100), band(&fast, 4000, 1, 100));
            let (s_hi, f_hi) = (band(&slow, 1000, 100, 250), band(&fast, 4000, 100, 250));
            for (name, got) in [("1 kHz", s_lo), ("4 kHz", f_lo)] {
                assert!(
                    (got / want - 1.0).abs() < 0.03,
                    "sensor {sensor}, {name}, 1-100 Hz: {got:e} against the density {want:e}"
                );
            }
            assert!(
                (s_lo / f_lo - 1.0).abs() < 0.03,
                "sensor {sensor}, 1-100 Hz: 1 kHz {s_lo:e}, 4 kHz {f_lo:e}"
            );
            assert!(
                (s_hi / f_hi - 1.0).abs() < 0.03,
                "sensor {sensor}, 100-250 Hz: 1 kHz {s_hi:e}, 4 kHz {f_hi:e}"
            );
        }
    }

    /// With the same seed the two readers are given the same internal
    /// stream: the noise (and bias) of every 1 kHz sample is that of every
    /// fourth 4 kHz sample, bit for bit.
    #[test]
    fn a_1_khz_reader_gets_every_fourth_sample_of_a_4_khz_reader() {
        let (mut cfg, body) = noise_only();
        let r = SensorConfig::realistic().imu;
        cfg.imu.gyro_bias_sigma = r.gyro_bias_sigma;
        cfg.imu.accel_bias_sigma = r.accel_bias_sigma;
        cfg.imu.gyro_bias_walk = r.gyro_bias_walk;
        cfg.imu.accel_bias_walk = r.accel_bias_walk;
        let tr = truth(&body);
        let mut slow = SensorSuite::<4>::new(cfg, 5, 1000).unwrap();
        let mut fast = SensorSuite::<4>::new(cfg, 5, 250).unwrap();
        let mut compared = 0;
        for k in 0..8000_u64 {
            let f = fast.sample(Micros(k * 250), &tr).imu.unwrap().value;
            if k % 4 == 0 {
                let s = slow.sample(Micros(k / 4 * 1000), &tr).imu.unwrap().value;
                assert_eq!(s, f, "sample at {} us", k * 250);
                assert_eq!(slow.gyro_bias(), fast.gyro_bias());
                compared += 1;
            }
        }
        assert_eq!(compared, 2000);
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
        cfg.imu.gyro_vibration_density = 6.3e-4;
        cfg.imu.accel_vibration_density = 1.6e-2;
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
