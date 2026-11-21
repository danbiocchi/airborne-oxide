// SPDX-License-Identifier: AGPL-3.0-or-later
//! The IMU has a sampling period of its own: another autopilot at a 500 us
//! step is given the IMU our F405 core is given (the same model, sampled
//! every 1000 us), and nothing else about the sensors moves.

use core::convert::Infallible;

use oxide_core::io::{ImuSample, Stamped};
use oxide_core::math::Real;
use oxide_core::schedule::F405_LEAN;
use oxide_core::time::Micros;
use oxide_sim::scenario::NoPilot;
use oxide_sim::sensors::{SensorConfig, SensorFrame};
use oxide_sim::{
    Airframe, Autopilot, AutopilotInput, AutopilotOutput, SimError, SimOptions, Simulation,
};

/// Never arms, records the sensor frame of every tick.
struct Recorder {
    period_us: u64,
    frames: Vec<SensorFrame>,
}

impl Recorder {
    fn new(period_us: u64) -> Self {
        Self {
            period_us,
            frames: Vec::new(),
        }
    }
}

impl Autopilot<4> for Recorder {
    type Error = Infallible;
    fn period_us(&self) -> u64 {
        self.period_us
    }
    fn tick(
        &mut self,
        t: Micros,
        input: &AutopilotInput<'_, 4>,
    ) -> Result<AutopilotOutput<4>, Infallible> {
        self.frames.push(*input.sensors);
        Ok(AutopilotOutput {
            t_valid: Micros(t.0 + self.period_us),
            motors: [0.0; 4],
            armed: false,
            core: None,
        })
    }
}

fn airframe() -> Airframe {
    Airframe::reference_quad().expect("reference airframe")
}

fn options(seed: u64, imu_period_us: u64) -> SimOptions {
    let mut o = SimOptions::realistic(seed);
    o.sensors.imu.period_us = imu_period_us;
    o
}

/// The frames of a recorder flight of `seconds` on the ground.
fn stub_frames(seed: u64, imu_period_us: u64, step_us: u64, seconds: u64) -> Vec<SensorFrame> {
    let mut sim = Simulation::with_autopilot(
        &airframe(),
        Recorder::new(step_us),
        NoPilot,
        options(seed, imu_period_us),
    )
    .expect("builds");
    sim.try_run_until(Micros::from_secs(seconds), |_| {})
        .expect("cannot fail");
    sim.into_autopilot().frames
}

fn bits(s: &Stamped<ImuSample>) -> [u32; 7] {
    let (g, a) = (s.value.gyro, s.value.accel);
    [
        g.x.to_bits(),
        g.y.to_bits(),
        g.z.to_bits(),
        a.x.to_bits(),
        a.y.to_bits(),
        a.z.to_bits(),
        s.t_sample.0 as u32,
    ]
}

/// The IMU samples our F405 core is given on a ground flight that never arms.
fn core_imu(seed: u64, seconds: u64) -> Vec<Stamped<ImuSample>> {
    let mut sim = Simulation::<_, 4>::with_options(
        &airframe(),
        F405_LEAN,
        NoPilot,
        SimOptions::realistic(seed),
    )
    .expect("builds");
    let mut out = Vec::new();
    while sim.time() < Micros::from_secs(seconds) {
        let r = sim.step();
        assert!(!r.armed);
        out.push(
            sim.last_inputs()
                .imu
                .expect("the core gets a sample every tick"),
        );
    }
    out
}

#[test]
fn the_stub_is_given_the_samples_the_f405_core_is_given() {
    let core = core_imu(7, 3);
    let stub = stub_frames(7, 1000, 500, 3);
    assert_eq!(core.len(), 3000);
    assert_eq!(stub.len(), 6000);
    for (i, f) in stub.iter().enumerate() {
        if i % 2 == 0 {
            let want = core.get(i / 2).expect("core sample");
            let got = f.imu.expect("a sample at a 1000 us instant");
            assert_eq!(bits(&got), bits(want), "sample at {} us", i * 500);
        } else {
            assert!(f.imu.is_none(), "no sample between: {} us", i * 500);
        }
    }
    // A different seed gives different samples.
    let other = stub_frames(8, 1000, 500, 3);
    assert_ne!(
        stub.first().and_then(|f| f.imu).map(|s| bits(&s)),
        other.first().and_then(|f| f.imu).map(|s| bits(&s))
    );
    let differing = stub
        .iter()
        .zip(&other)
        .filter(|(a, b)| a.imu.map(|s| bits(&s)) != b.imu.map(|s| bits(&s)))
        .count();
    assert!(differing >= 2900, "only {differing} samples differ");
}

/// Standard deviation of the gyro x, y and z samples about their means.
fn gyro_std(frames: &[SensorFrame]) -> [f64; 3] {
    let g: Vec<[f64; 3]> = frames
        .iter()
        .filter_map(|f| f.imu)
        .map(|s| {
            [
                f64::from(s.value.gyro.x),
                f64::from(s.value.gyro.y),
                f64::from(s.value.gyro.z),
            ]
        })
        .collect();
    let n = g.len() as f64;
    core::array::from_fn(|a| {
        let mean = g.iter().map(|v| v[a]).sum::<f64>() / n;
        Real::sqrt(g.iter().map(|v| (v[a] - mean) * (v[a] - mean)).sum::<f64>() / n)
    })
}

/// The IMU's noise passes its filter on its own clock (ADR-0008), so a
/// reader every 1000 us and one every 500 us step get the same noise per
/// sample: the white-noise density times the root of the filter's noise
/// band (565.6 Hz). At rest there is no vibration.
#[test]
fn the_noise_per_sample_is_the_same_at_1000_us_and_at_the_500_us_step() {
    let m = SensorConfig::realistic().imu;
    let want = m.gyro_noise_density * Real::sqrt(m.noise_bandwidth_hz().expect("filter"));
    for period in [1000, 0] {
        let std = gyro_std(&stub_frames(11, period, 500, 3));
        for s in std {
            assert!(
                (s / want - 1.0).abs() < 0.10,
                "period {period}: std {s:e} vs {want:e}"
            );
        }
    }
}

#[test]
fn the_other_sensors_do_not_move() {
    let a = stub_frames(5, 0, 500, 3);
    let b = stub_frames(5, 1000, 500, 3);
    assert_eq!(a.len(), b.len());
    let (mut gps, mut baro, mut mag, mut batt) = (0, 0, 0, 0);
    for (x, y) in a.iter().zip(&b) {
        assert_eq!(x.gps, y.gps);
        assert_eq!(x.baro, y.baro);
        assert_eq!(x.mag, y.mag);
        assert_eq!(x.battery, y.battery);
        gps += usize::from(x.gps.is_some());
        baro += usize::from(x.baro.is_some());
        mag += usize::from(x.mag.is_some());
        batt += usize::from(x.battery.is_some());
    }
    assert!(gps > 5 && baro > 100 && mag > 100 && batt > 100);
}

#[test]
fn a_period_that_is_not_a_multiple_of_the_step_is_refused() {
    let r = Simulation::with_autopilot(&airframe(), Recorder::new(500), NoPilot, options(1, 700));
    assert!(matches!(r, Err(SimError::TimeBase(500))));
}

#[test]
fn our_core_refuses_a_period_other_than_its_step() {
    for period in [500, 2000, 700] {
        let r =
            Simulation::<_, 4>::with_options(&airframe(), F405_LEAN, NoPilot, options(1, period));
        match r {
            Err(SimError::TimeBase(_)) => assert_eq!(period, 500, "500 is not a multiple"),
            Err(e @ SimError::Param { .. }) => {
                let text = e.to_string();
                assert!(text.contains("every tick"), "says why: {text}");
            }
            Err(e) => panic!("period {period}: unexpected {e:?}"),
            Ok(_) => panic!("period {period} was accepted by the core simulation"),
        }
    }
    // The step itself is the same as no period.
    let ok = Simulation::<_, 4>::with_options(&airframe(), F405_LEAN, NoPilot, options(1, 1000));
    assert!(ok.is_ok());
}

/// The harmonics of spinning rotors advance by the rotor angle of one IMU
/// period (not of one step): the same suite at period 1000 us and step 500 us
/// gives, bit for bit, what it gives at period 0 and step 1000 us.
#[test]
fn the_rotor_phase_advances_by_the_imu_period() {
    use oxide_sim::sensors::{SensorSuite, Truth};
    use oxide_sim::vehicle::BodyState;

    let body = BodyState::<4> {
        rotor_speed: [0.7, 0.55, 0.8, 0.6],
        ..BodyState::default()
    };
    let truth = Truth {
        body: &body,
        specific_force_world: oxide_core::math::Vec3::new(0.0, 0.0, -9.80665),
        battery_v: 16.0,
        battery_a: 10.0,
        max_omega: 2000.0,
    };
    let mut fine_cfg = SensorConfig::realistic();
    fine_cfg.imu.period_us = 1000;
    let mut fine = SensorSuite::<4>::new(fine_cfg, 3, 500).expect("fine");
    let mut coarse = SensorSuite::<4>::new(SensorConfig::realistic(), 3, 1000).expect("coarse");
    let mut n = 0;
    for i in 0..600u64 {
        let f = fine.sample(Micros(i * 500), &truth).imu;
        if i % 2 == 0 {
            let c = coarse.sample(Micros(i / 2 * 1000), &truth).imu;
            assert_eq!(f.map(|s| bits(&s)), c.map(|s| bits(&s)), "sample {i}");
            n += 1;
        } else {
            assert!(f.is_none());
        }
    }
    assert_eq!(n, 300);
}
