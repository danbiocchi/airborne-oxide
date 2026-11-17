// SPDX-License-Identifier: AGPL-3.0-or-later
//! The wind on a coarser grid (ADR-0006 §7): a 500 µs flight with a 1000 µs
//! wind period sees, sample for sample, the wind of the 1000 µs flight of the
//! same seed, and holds it in between.

use core::convert::Infallible;

use oxide_core::schedule::F405_LEAN;
use oxide_core::time::Micros;
use oxide_sim::scenario::{NoPilot, gust_hold};
use oxide_sim::{
    Airframe, Autopilot, AutopilotInput, AutopilotOutput, SimError, SimOptions, Simulation,
    StepRecord,
};

/// Four equal motors at a fixed command, valid at once.
struct Steady {
    period_us: u64,
}

impl Autopilot<4> for Steady {
    type Error = Infallible;

    fn period_us(&self) -> u64 {
        self.period_us
    }

    fn tick(
        &mut self,
        t: Micros,
        _input: &AutopilotInput<'_, 4>,
    ) -> Result<AutopilotOutput<4>, Infallible> {
        Ok(AutopilotOutput {
            t_valid: t,
            motors: [0.4; 4],
            armed: true,
            core: None,
        })
    }
}

fn airframe() -> Airframe {
    Airframe::reference_quad().expect("reference airframe")
}

fn options(index: u64, wind_period_us: Option<u64>) -> SimOptions {
    let (_, o) = gust_hold(index);
    SimOptions {
        wind_period_us,
        ..o
    }
}

fn fly_stub(
    step_us: u64,
    opts: SimOptions,
    seconds: u64,
) -> Result<(Vec<StepRecord<4>>, u64), SimError> {
    let mut sim =
        Simulation::with_autopilot(&airframe(), Steady { period_us: step_us }, NoPilot, opts)?;
    let mut log = Vec::new();
    sim.try_run_until(Micros::from_secs(seconds), |r| log.push(*r))
        .expect("cannot fail");
    Ok((log, sim.trace_hash()))
}

fn fly_core(opts: SimOptions, seconds: u64) -> Vec<StepRecord<4>> {
    let (pilot, _) = gust_hold(0);
    let mut sim =
        Simulation::<_, 4>::with_options(&airframe(), F405_LEAN, pilot, opts).expect("builds");
    let mut log = Vec::new();
    sim.try_run_until(Micros::from_secs(seconds), |r| log.push(*r))
        .expect("our core cannot fail");
    log
}

fn bits(v: oxide_core::math::Vec3<f64>) -> [u64; 3] {
    [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()]
}

#[test]
fn a_500_us_flight_on_a_1000_us_wind_grid_has_the_wind_of_the_1000_us_flight() {
    let core = fly_core(options(0, None), 40);
    let (stub, _) = fly_stub(500, options(0, Some(1_000)), 40).expect("builds");
    assert_eq!(core.len(), 40_000);
    assert_eq!(stub.len(), 80_000);
    let mut moved = false;
    for (k, c) in core.iter().enumerate() {
        let on = &stub[2 * k];
        let between = &stub[2 * k + 1];
        assert_eq!(on.t, c.t, "record {k}");
        assert_eq!(
            bits(on.wind),
            bits(c.wind),
            "sample at {} s",
            c.t.as_secs_f64()
        );
        assert_eq!(
            bits(between.wind),
            bits(on.wind),
            "held between samples after {} s",
            c.t.as_secs_f64()
        );
        moved |= k > 0 && bits(c.wind) != bits(core[k - 1].wind);
    }
    assert!(moved, "the wind must actually vary");
}

#[test]
fn a_different_seed_gives_a_different_wind_and_trace() {
    let (a, ha) = fly_stub(500, options(0, Some(1_000)), 5).expect("builds");
    let (b, hb) = fly_stub(500, options(1, Some(1_000)), 5).expect("builds");
    assert_ne!(ha, hb);
    assert!(a.iter().zip(&b).any(|(x, y)| bits(x.wind) != bits(y.wind)));
}

#[test]
fn the_default_and_a_period_equal_to_the_step_are_the_same_flight() {
    let (_, none) = fly_stub(500, options(0, None), 5).expect("builds");
    let (_, same) = fly_stub(500, options(0, Some(500)), 5).expect("builds");
    assert_eq!(none, same);
}

#[test]
fn a_wind_period_that_is_not_a_whole_multiple_of_the_step_is_refused_naming_both() {
    for bad in [750, 250, 0] {
        let e = fly_stub(500, options(0, Some(bad)), 1).expect_err("refused");
        assert_eq!(
            e,
            SimError::WindPeriod {
                period_us: bad,
                step_us: 500
            }
        );
        let text = e.to_string();
        assert!(
            text.contains(&bad.to_string()) && text.contains("500"),
            "{text}"
        );
    }
}

#[test]
fn our_core_flies_on_a_coarser_wind_grid_too() {
    let flown = |wind_period_us| {
        let (pilot, _) = gust_hold(0);
        let mut sim = Simulation::<_, 4>::with_options(
            &airframe(),
            F405_LEAN,
            pilot,
            options(0, wind_period_us),
        )
        .expect("builds");
        let mut log = Vec::new();
        sim.try_run_until(Micros::from_secs(3), |r| log.push(r.wind))
            .expect("cannot fail");
        log
    };
    let fine = flown(None);
    let coarse = flown(Some(4_000));
    for (k, w) in coarse.iter().enumerate() {
        assert_eq!(bits(*w), bits(coarse[k - k % 4]), "held at step {k}");
    }
    // The first draw is the stationary one, the same on any grid; later
    // samples are different draws of the same process.
    assert_eq!(
        bits(coarse[0]),
        bits(fine[0]),
        "same seed, same first sample"
    );
    assert_ne!(bits(coarse[4]), bits(fine[4]));
}
