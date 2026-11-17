// SPDX-License-Identifier: AGPL-3.0-or-later
//! The pre-roll: the vehicle rests on the ground before the flight, and an
//! autopilot that is watching sees the IMU model's turn-on bias.

use core::fmt;

use oxide_core::io::ImuSample;
use oxide_core::math::Vec3;
use oxide_core::schedule::F405_LEAN;
use oxide_core::time::Micros;
use oxide_sim::scenario::{NoPilot, gust_hold_wind};
use oxide_sim::sensors::SensorFrame;
use oxide_sim::vehicle::BodyState;
use oxide_sim::{
    Airframe, Autopilot, AutopilotInput, AutopilotOutput, PrerollError, PrerollInput, SimOptions,
    Simulation, StepRecord,
};

#[derive(Debug, PartialEq)]
struct Died {
    k: u64,
}

impl std::error::Error for Died {}
impl fmt::Display for Died {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "died at {}", self.k)
    }
}

/// Never arms; records its pre-roll and the frames of its flight.
struct Watcher {
    period_us: u64,
    fail_at_k: Option<u64>,
    preroll_calls: u64,
    ks: Vec<u64>,
    bodies: Vec<BodyState<4>>,
    forces: Vec<Vec3<f64>>,
    batteries: Vec<(f64, f64)>,
    imus: Vec<Option<ImuSample>>,
    frames: Vec<SensorFrame>,
}

impl Watcher {
    fn new(period_us: u64) -> Self {
        Self {
            period_us,
            fail_at_k: None,
            preroll_calls: 0,
            ks: Vec::new(),
            bodies: Vec::new(),
            forces: Vec::new(),
            batteries: Vec::new(),
            imus: Vec::new(),
            frames: Vec::new(),
        }
    }
}

impl Autopilot<4> for Watcher {
    type Error = Died;
    fn period_us(&self) -> u64 {
        self.period_us
    }
    fn tick(
        &mut self,
        t: Micros,
        input: &AutopilotInput<'_, 4>,
    ) -> Result<AutopilotOutput<4>, Died> {
        self.frames.push(*input.sensors);
        Ok(AutopilotOutput {
            t_valid: Micros(t.0 + self.period_us),
            motors: [0.0; 4],
            armed: false,
            core: None,
        })
    }
    fn preroll_tick(&mut self, k: u64, input: &PrerollInput<'_, 4>) -> Result<(), Died> {
        self.preroll_calls += 1;
        if self.fail_at_k == Some(k) {
            return Err(Died { k });
        }
        self.ks.push(k);
        self.bodies.push(*input.body);
        self.forces.push(input.specific_force_world);
        self.batteries.push((input.battery_v, input.battery_a));
        self.imus.push(input.imu);
        Ok(())
    }
}

fn airframe() -> Airframe {
    Airframe::reference_quad().expect("reference airframe")
}

fn options(seed: u64) -> SimOptions {
    let mut o = SimOptions::realistic(seed);
    o.sensors.imu.period_us = 1000;
    o.wind = gust_hold_wind(seed);
    o
}

fn sim(w: Watcher, seed: u64) -> Simulation<NoPilot, 4, Watcher> {
    Simulation::with_autopilot(&airframe(), w, NoPilot, options(seed)).expect("builds")
}

fn bits(s: &ImuSample) -> [u32; 6] {
    [
        s.gyro.x.to_bits(),
        s.gyro.y.to_bits(),
        s.gyro.z.to_bits(),
        s.accel.x.to_bits(),
        s.accel.y.to_bits(),
        s.accel.z.to_bits(),
    ]
}

#[test]
fn the_autopilot_sees_the_rest_and_the_turn_on_bias() {
    let mut s = sim(Watcher::new(500), 3);
    let bg = s.sensors().gyro_bias();
    let ba = s.sensors().accel_bias();
    s.preroll(20_000).expect("preroll");
    let w = s.autopilot();
    assert_eq!(w.preroll_calls, 20_000);
    assert!(w.ks.iter().copied().eq(0..20_000));
    let rest = BodyState::<4>::default();
    for b in &w.bodies {
        assert_eq!(*b, rest);
    }
    for f in &w.forces {
        assert_eq!(*f, Vec3::new(0.0, 0.0, -9.80665));
    }
    let (v0, a0) = w.batteries[0];
    assert!(v0 > 5.0 && a0.abs() < 1e-9, "resting battery {v0} V {a0} A");
    assert!(w.batteries.iter().all(|&b| b == (v0, a0)));
    // An IMU sample on every second call (period 1000 at a 500 us step).
    for (k, i) in w.imus.iter().enumerate() {
        assert_eq!(i.is_some(), k % 2 == 0, "call {k}");
    }
    let got: Vec<&ImuSample> = w.imus.iter().flatten().collect();
    assert_eq!(got.len(), 10_000);
    let n = got.len() as f64;
    let mean = |f: &dyn Fn(&ImuSample) -> f32| got.iter().map(|s| f64::from(f(s))).sum::<f64>() / n;
    assert!(
        got.iter()
            .all(|s| bits(s).iter().all(|b| f32::from_bits(*b).is_finite()))
    );
    let g = [
        mean(&|s| s.gyro.x),
        mean(&|s| s.gyro.y),
        mean(&|s| s.gyro.z),
    ];
    let a = [
        mean(&|s| s.accel.x),
        mean(&|s| s.accel.y),
        mean(&|s| s.accel.z),
    ];
    for (m, b) in g.iter().zip([bg.x, bg.y, bg.z]) {
        assert!((m - b).abs() < 1e-3, "gyro mean {m} vs bias {b}");
    }
    for (m, b) in a.iter().zip([ba.x, ba.y, ba.z - 9.80665]) {
        assert!((m - b).abs() < 8e-3, "accel mean {m} vs {b}");
    }
}

#[test]
fn the_bias_carries_over_into_the_flight() {
    let mut o = options(9);
    o.sensors.imu.gyro_noise_density = 0.0;
    o.sensors.imu.accel_noise_density = 0.0;
    let mut s =
        Simulation::with_autopilot(&airframe(), Watcher::new(500), NoPilot, o).expect("builds");
    let turn_on = (s.sensors().gyro_bias(), s.sensors().accel_bias());
    s.preroll(90_000).expect("preroll");
    let (bg, ba) = (s.sensors().gyro_bias(), s.sensors().accel_bias());
    let moved = (bg - turn_on.0).norm() + (ba - turn_on.1).norm();
    assert!(moved > 1e-4, "the walk ran: moved {moved}");
    s.try_step().expect("step");
    s.try_step().expect("step");
    // The first sample is built on the bias after the pre-roll plus one walk
    // step (a 1000 us step of 1e-4 rad/s/sqrt(s): about 3e-6): no jump back.
    let first = s.autopilot().frames[0].imu.expect("sample at t = 0").value;
    let diff = (Vec3::new(
        f64::from(first.gyro.x),
        f64::from(first.gyro.y),
        f64::from(first.gyro.z),
    ) - bg)
        .norm();
    assert!(diff < 3e-5, "first sample is {diff} from the carried bias");
    let back = (Vec3::new(
        f64::from(first.gyro.x),
        f64::from(first.gyro.y),
        f64::from(first.gyro.z),
    ) - turn_on.0)
        .norm();
    assert!(back > 10.0 * diff, "no jump back to the turn-on value");
    let adiff = f64::from(first.accel.z) - (ba.z - 9.80665);
    assert!(adiff.abs() < 1e-4, "accel: {adiff}");
}

fn ground_flight(preroll: u64) -> (Vec<StepRecord<4>>, Vec<SensorFrame>, u64) {
    let mut s = sim(Watcher::new(500), 21);
    if preroll > 0 {
        s.preroll(preroll).expect("preroll");
        assert_eq!(s.time(), Micros(0));
        assert_eq!(
            s.trace_hash(),
            0xcbf2_9ce4_8422_2325,
            "the hash is untouched"
        );
    }
    let mut rec = Vec::new();
    s.try_run_until(Micros::from_secs(3), |r| rec.push(*r))
        .expect("flies");
    let hash = s.trace_hash();
    (rec, s.into_autopilot().frames, hash)
}

#[test]
fn nothing_but_the_imu_moves() {
    let (ra, fa, ha) = ground_flight(0);
    let (rb, fb, hb) = ground_flight(90_000);
    assert_eq!(ha, hb);
    assert_eq!(ra, rb, "records (time, wind, truth) are identical");
    assert_eq!(ra.first().map(|r| r.t), Some(Micros(0)));
    let mut imu_differs = 0;
    for (x, y) in fa.iter().zip(&fb) {
        assert_eq!(x.gps, y.gps);
        assert_eq!(x.baro, y.baro);
        assert_eq!(x.mag, y.mag);
        assert_eq!(x.battery, y.battery);
        assert_eq!(x.imu.is_some(), y.imu.is_some());
        if let (Some(p), Some(q)) = (x.imu, y.imu)
            && bits(&p.value) != bits(&q.value)
        {
            imu_differs += 1;
        }
    }
    assert_eq!(imu_differs, 3000, "every IMU sample differs");
}

#[test]
fn preroll_is_refused_after_a_step_and_the_second_time() {
    let mut s = sim(Watcher::new(500), 1);
    s.try_step().expect("step");
    let e = s.preroll(10).expect_err("after a step");
    assert!(matches!(e, PrerollError::AfterStep { .. }), "{e:?}");
    assert_eq!(s.autopilot().preroll_calls, 0);

    let mut s = sim(Watcher::new(500), 1);
    s.preroll(10).expect("first");
    let e = s.preroll(10).expect_err("second");
    assert!(matches!(e, PrerollError::AlreadyRun), "{e:?}");
    assert_eq!(s.autopilot().preroll_calls, 10);
    assert_ne!(
        PrerollError::<Died>::AfterStep { tick: 1 }.to_string(),
        PrerollError::<Died>::AlreadyRun.to_string()
    );
}

#[test]
fn a_failing_preroll_tick_stops_the_preroll() {
    let mut w = Watcher::new(500);
    w.fail_at_k = Some(7);
    let mut s = sim(w, 1);
    let e = s.preroll(100).expect_err("fails");
    match &e {
        PrerollError::Autopilot { k, source } => {
            assert_eq!(*k, 7);
            assert_eq!(*source, Died { k: 7 });
        }
        other => panic!("{other:?}"),
    }
    assert!(std::error::Error::source(&e).is_some());
    assert!(e.to_string().contains('7'));
    assert_eq!(s.autopilot().preroll_calls, 8);
}

#[test]
fn our_core_that_never_prerolls_flies_the_known_flight() {
    use oxide_sim::scenario::HoverStep;
    let mut sim = Simulation::<_, 4>::with_options(
        &airframe(),
        F405_LEAN,
        HoverStep::default(),
        SimOptions::default(),
    )
    .expect("builds");
    sim.run_until(Micros::from_secs(20), |_| {});
    assert_eq!(format!("{:016x}", sim.trace_hash()), "3de16d7a9847deaf");
}
