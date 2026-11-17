// SPDX-License-Identifier: AGPL-3.0-or-later
//! The simulator hosts an autopilot that is not our flight core: a test-only
//! `Stub` flies through the `Autopilot` seam with a tick period neither core
//! profile has, and our own core flown through the same seam gives the flight
//! it always gave.

use core::fmt;

use oxide_core::io::ParamSet;
use oxide_core::math::Vec3;
use oxide_core::params::ParamValue;
use oxide_core::schedule::{F405_LEAN, Profile};
use oxide_core::time::Micros;
use oxide_sim::metrics::{HoldCriteria, HoldMetrics};
use oxide_sim::outcome::{Ending, EventTag, Expect, FlightOutcome, NEEDS_CORE};
use oxide_sim::scenario::{HoverStep, NoPilot};
use oxide_sim::{
    Airframe, Autopilot, AutopilotInput, AutopilotOutput, SimError, SimOptions, Simulation,
    StepRecord,
};

/// The stub's tick failed.
#[derive(Debug, PartialEq)]
struct StubError {
    tick: u64,
}

impl std::error::Error for StubError {}

impl fmt::Display for StubError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "the stub's process died at its tick {}", self.tick)
    }
}

/// Arms at a given time and holds a height on the truth it is given: one
/// collective command from a PD on height, no attitude control (it starts
/// level in calm air, and four equal rotors keep it so).
struct Stub {
    period_us: u64,
    arm_at_s: f64,
    hold_m: f64,
    kp: f64,
    kd: f64,
    /// The call (counted from 0) that fails.
    fail_at_call: Option<u64>,
    calls: u64,
    /// The commands are valid this many periods after the tick's time (1 is
    /// the logical execution time of our core).
    valid_after_periods: u64,
    /// The parameter changes and acknowledgements it was given.
    seen_sets: Vec<ParamSet>,
    seen_acks: u32,
    /// The rotor scales it was given at each tick.
    seen_scales: Vec<[f64; 4]>,
}

impl Stub {
    fn new(period_us: u64) -> Self {
        Self {
            period_us,
            arm_at_s: 0.5,
            hold_m: 2.0,
            kp: 0.36,
            kd: 0.18,
            fail_at_call: None,
            calls: 0,
            valid_after_periods: 1,
            seen_sets: Vec::new(),
            seen_acks: 0,
            seen_scales: Vec::new(),
        }
    }
}

impl Autopilot<4> for Stub {
    type Error = StubError;

    fn period_us(&self) -> u64 {
        self.period_us
    }

    fn tick(
        &mut self,
        t: Micros,
        input: &AutopilotInput<'_, 4>,
    ) -> Result<AutopilotOutput<4>, StubError> {
        let call = self.calls;
        self.calls += 1;
        if self.fail_at_call == Some(call) {
            return Err(StubError { tick: call });
        }
        self.seen_sets.extend(input.param_set);
        self.seen_acks += u32::from(input.param_ack);
        self.seen_scales.push(input.rotor_scale);
        let armed = t.as_secs_f64() >= self.arm_at_s;
        let height = -input.body.position.z;
        let climb = -input.body.velocity.z;
        // The command that hovers at full charge is about 0.40.
        let u = 0.40 + self.kp * (self.hold_m - height) - self.kd * climb;
        let u = if armed { u.clamp(0.0, 1.0) as f32 } else { 0.0 };
        Ok(AutopilotOutput {
            t_valid: Micros(t.0 + self.valid_after_periods * self.period_us),
            motors: [u; 4],
            armed,
            core: None,
        })
    }
}

fn airframe() -> Airframe {
    Airframe::reference_quad().expect("reference airframe")
}

fn stub_sim(stub: Stub) -> Simulation<NoPilot, 4, Stub> {
    Simulation::with_autopilot(&airframe(), stub, NoPilot, SimOptions::default()).expect("builds")
}

/// Flies the stub for `seconds`, collecting the records.
fn fly_stub(stub: Stub, seconds: u64) -> (Vec<StepRecord<4>>, u64) {
    let mut sim = stub_sim(stub);
    let mut log = Vec::new();
    sim.try_run_until(Micros::from_secs(seconds), |r| log.push(*r))
        .expect("the stub does not fail");
    (log, sim.trace_hash())
}

#[test]
fn a_stub_autopilot_flies_five_seconds_and_holds_its_height() {
    let (log, _) = fly_stub(Stub::new(500), 5);
    assert_eq!(log.len(), 10_000, "5 s at the stub's 500 us period");
    let max = log.iter().map(|r| -r.position.z).fold(0.0, f64::max);
    assert!(max > 1.0, "the vehicle left the ground: max height {max}");
    for r in log.iter().filter(|r| r.t.as_secs_f64() >= 3.0) {
        let h = -r.position.z;
        assert!(
            (h - 2.0).abs() < 1.0,
            "{:.2} s: height {h:.2} m is not within a metre of 2 m",
            r.t.as_secs_f64()
        );
    }
    // `armed` is the stub's, and there is no core part.
    for r in &log {
        assert_eq!(r.armed, r.t.as_secs_f64() >= 0.5, "t {}", r.t.as_secs_f64());
        assert!(r.core.is_none() && r.status().is_none() && r.estimate().is_none());
    }
}

#[test]
fn the_same_stub_flight_hashes_the_same_and_a_different_gain_does_not() {
    let (_, a) = fly_stub(Stub::new(500), 5);
    let (_, b) = fly_stub(Stub::new(500), 5);
    assert_eq!(a, b, "the same flight twice");
    let mut other = Stub::new(500);
    other.kp = 0.30;
    let (_, c) = fly_stub(other, 5);
    assert_ne!(a, c, "a different gain flies a different flight");
}

fn outcome_of(log: &[StepRecord<4>]) -> FlightOutcome {
    let mut o = FlightOutcome::new();
    for r in log {
        o.add(r);
    }
    o
}

#[test]
fn the_outcome_of_a_stub_flight_has_the_truth_part_and_no_core_part() {
    let (log, _) = fly_stub(Stub::new(500), 5);
    let o = outcome_of(&log);
    assert!(o.core.is_none());
    assert!(o.max_height_m > 0.0);
    assert!(o.armed_at_end);
    let first_armed = log.iter().find(|r| r.armed).expect("armed at some time");
    assert_eq!(o.home, Some(first_armed.position));
    assert!((o.end_s - log.last().expect("records").t.as_secs_f64()).abs() < 1e-12);
    assert_eq!(o.physics_faults, 0);
    assert!(!o.non_finite);
}

#[test]
fn expect_judges_the_truth_and_names_what_needs_the_core() {
    let (log, _) = fly_stub(Stub::new(500), 5);
    let o = outcome_of(&log);
    let holds = Expect {
        ending: Ending::FlyingAtHeight {
            height_m: 2.0,
            tolerance_m: 1.0,
        },
        by_s: 4.5,
        min_height_m: Some(1.0),
        max_height_m: Some(5.0),
        ..Expect::default()
    };
    let j = holds.check(&o);
    assert!(j.violations.is_empty(), "{:?}", j.violations);

    // A truth limit it broke is a violation.
    let low = Expect {
        max_height_m: Some(0.5),
        ..holds.clone()
    };
    let j = low.check(&o);
    assert!(
        j.violations.iter().any(|v| v.contains("limit 0.5 m")),
        "{:?}",
        j.violations
    );

    // What needs the core's status is neither passed nor violated.
    let core_keys = Expect {
        nav_contains: vec![oxide_core::io::NavState::Hold],
        reasons: oxide_core::io::Reasons::RC_LOST,
        events: vec![EventTag::Armed],
        min_action: oxide_core::io::Action::Hold,
        ..holds
    };
    let j = core_keys.check(&o);
    assert!(j.violations.is_empty(), "{:?}", j.violations);
    let keys: Vec<&str> = j.not_judged.iter().map(|n| n.key).collect();
    for want in ["nav_contains", "reasons", "events", "min_action"] {
        assert!(keys.contains(&want), "{want} is not in {keys:?}");
    }
    assert!(j.not_judged.iter().all(|n| n.reason == NEEDS_CORE));
}

#[test]
fn hold_metrics_have_truth_errors_and_leave_the_estimator_criteria_unjudged() {
    let (log, _) = fly_stub(Stub::new(500), 5);
    let c = HoldCriteria {
        window_start_s: 3.0,
        armed_from_s: 1.0,
        ..HoldCriteria::default()
    };
    let target = Vec3::new(0.0, 0.0, -2.0);
    let mut m = HoldMetrics::new();
    for r in &log {
        m.add(r, target, &c);
    }
    assert!(m.max_h_error_m.is_finite() && m.rms_h_error_m.is_finite());
    assert!(
        m.max_v_error_m.is_finite() && m.max_v_error_m < 1.0,
        "{m:?}"
    );
    assert!(m.max_tilt_deg.is_finite());
    assert_eq!(m.unarmed_ticks, 0, "armed from 0.5 s");
    assert!(m.core.is_none());
    let j = m.judge(&c);
    let keys: Vec<&str> = j.not_judged.iter().map(|n| n.key).collect();
    assert!(
        keys.contains(&"hold: estimate max horizontal error"),
        "{keys:?}"
    );
    assert!(!m.passed(&c), "not passed while criteria are unjudged");
}

#[test]
fn a_period_that_does_not_fit_is_refused_like_a_bad_profile() {
    // The period must divide the 20 ms command period.
    let bad = Simulation::<NoPilot, 4, Stub>::with_autopilot(
        &airframe(),
        Stub::new(3_000),
        NoPilot,
        SimOptions::default(),
    );
    assert!(matches!(bad, Err(SimError::TimeBase(3_000))));
    // The error a bad profile gets for the same period.
    let profile = Profile {
        base_period_us: 3_000,
        ..F405_LEAN
    };
    let via_profile = Simulation::<NoPilot, 4>::with_options(
        &airframe(),
        profile,
        NoPilot,
        SimOptions::default(),
    );
    assert_eq!(via_profile.err(), Some(SimError::TimeBase(3_000)));
    // Zero is refused too.
    let zero = Simulation::<NoPilot, 4, Stub>::with_autopilot(
        &airframe(),
        Stub::new(0),
        NoPilot,
        SimOptions::default(),
    );
    assert!(matches!(zero, Err(SimError::TimeBase(0))));
    // A period that divides the command period but not a sensor's (the
    // battery sensor's 10 ms) is refused as well.
    let sensor = Simulation::<NoPilot, 4, Stub>::with_autopilot(
        &airframe(),
        Stub::new(32),
        NoPilot,
        SimOptions::default(),
    );
    assert!(matches!(sensor, Err(SimError::TimeBase(32))));
}

#[test]
fn a_failing_tick_leaves_the_simulation_as_it_was() {
    let mut stub = Stub::new(500);
    stub.fail_at_call = Some(1_000);
    let mut sim = stub_sim(stub);
    for _ in 0..1_000 {
        sim.try_step().expect("the first 1000 ticks succeed");
    }
    let (time, hash, body) = (sim.time(), sim.trace_hash(), *sim.body());
    assert_eq!(time, Micros(500_000));
    let err = sim.try_step().expect_err("the 1001st tick fails");
    assert_eq!(err, StubError { tick: 1_000 });
    assert_eq!(sim.time(), time, "the tick counter did not move");
    assert_eq!(sim.trace_hash(), hash, "the hash did not move");
    assert_eq!(*sim.body(), body, "the truth did not move");
    assert_eq!(sim.physics_faults(), 0);
    // `try_run_until` stops at the error and returns it.
    let mut stub = Stub::new(500);
    stub.fail_at_call = Some(10);
    let mut sim = stub_sim(stub);
    let mut seen = 0;
    let r = sim.try_run_until(Micros::from_secs(1), |_| seen += 1);
    assert_eq!(r, Err(StubError { tick: 10 }));
    assert_eq!(seen, 10);
}

#[test]
fn our_core_through_the_seam_flies_the_flight_it_always_flew() {
    // hover-step on the F405 profile with default options: the `hash`
    // command's first line pins its trace hash.
    let mut sim = Simulation::<_, 4>::with_options(
        &airframe(),
        F405_LEAN,
        HoverStep::default(),
        SimOptions::default(),
    )
    .expect("builds");
    let mut records = 0;
    sim.try_run_until(Micros::from_secs(20), |r| {
        records += 1;
        let core = r.core.as_ref().expect("our core fills the core part");
        assert_eq!(r.armed, core.status.armed, "t {}", r.t.as_secs_f64());
    })
    .expect("our core cannot fail");
    assert_eq!(records, 20_000);
    assert_eq!(format!("{:016x}", sim.trace_hash()), "3de16d7a9847deaf");
}

fn max_height(log: &[StepRecord<4>]) -> f64 {
    log.iter().map(|r| -r.position.z).fold(0.0, f64::max)
}

#[test]
fn a_command_is_applied_from_its_t_valid_whenever_that_is() {
    let mut heights = Vec::new();
    let mut hashes = Vec::new();
    for k in 0..4 {
        let mut stub = Stub::new(500);
        stub.valid_after_periods = k;
        let (log, hash) = fly_stub(stub, 3);
        heights.push(max_height(&log));
        hashes.push(hash);
    }
    // A command valid later than the next tick is not lost: the vehicle
    // still flies (it never left the ground when each new command replaced
    // the one that had not fallen due yet).
    for (k, h) in heights.iter().enumerate() {
        assert!(*h > 1.0, "valid {k} periods after the tick: max height {h}");
    }
    // Every delay is a different flight: a command valid at the tick itself
    // acts one period earlier than one valid a period later, and so on.
    for a in 0..hashes.len() {
        for b in a + 1..hashes.len() {
            assert_ne!(hashes[a], hashes[b], "delays {a} and {b} flew the same");
        }
    }
}

#[test]
fn a_flight_with_requirements_it_could_not_judge_has_not_passed() {
    let (log, _) = fly_stub(Stub::new(500), 5);
    let text = "name = \"x\"\nduration_s = 5.0\n[expect]\nreasons = [\"RC_LOST\"]\nby_s = 4.0\n\
                [expect.ending]\nkind = \"flying-at-height\"\nheight_m = 2.0\ntolerance_m = 1.0\n";
    let file = oxide_sim::scenario_file::load_str(text).expect("loads");
    let mut checker = oxide_sim::scenario_file::Checker::new(&file);
    for r in &log {
        checker.add(r);
    }
    let verdict = checker.finish();
    assert!(verdict.violations.is_empty(), "{:?}", verdict.violations);
    assert!(!verdict.not_judged.is_empty());
    assert!(!verdict.passed(), "unjudged requirements are not a pass");
    let first = verdict.first_problem().expect("something stops the pass");
    assert!(first.starts_with("not judged: "), "{first}");
    // The same on the expectation's own answer.
    let o = outcome_of(&log);
    let expect = file.expect.expect("the file states expectations");
    assert!(!expect.check(&o).passed());
    // A truth-only expectation that holds is not violated.
    let truth = Expect {
        ending: Ending::FlyingAtHeight {
            height_m: 2.0,
            tolerance_m: 1.0,
        },
        by_s: 4.5,
        max_height_m: Some(5.0),
        ..Expect::default()
    };
    let j = truth.check(&o);
    assert!(j.violations.is_empty(), "{:?}", j.violations);
    // (the invariants that need the core are still listed: not a pass)
    assert!(!j.not_judged.is_empty() && !j.passed());
}

/// Commands every rotor at one fixed value from the first tick.
struct Fixed(f32);

impl Autopilot<4> for Fixed {
    type Error = StubError;

    fn period_us(&self) -> u64 {
        500
    }

    fn tick(
        &mut self,
        t: Micros,
        _input: &AutopilotInput<'_, 4>,
    ) -> Result<AutopilotOutput<4>, StubError> {
        Ok(AutopilotOutput {
            t_valid: Micros(t.0 + 500),
            motors: [self.0; 4],
            armed: true,
            core: None,
        })
    }
}

#[test]
fn the_hash_covers_the_commands_even_when_the_truth_is_the_same() {
    // The physics clamps a command to 0..1, so 1.0 and 2.0 fly the same
    // flight: only the commands themselves differ.
    let fly = |u: f32| {
        let mut sim =
            Simulation::with_autopilot(&airframe(), Fixed(u), NoPilot, SimOptions::default())
                .expect("builds");
        let mut truth = Vec::new();
        sim.try_run_until(Micros::from_secs(1), |r| {
            truth.push((r.position, r.velocity, r.attitude, r.rate, r.rotor_speed));
        })
        .expect("cannot fail");
        (truth, sim.trace_hash())
    };
    let (full, hash_full) = fly(1.0);
    let (over, hash_over) = fly(2.0);
    assert_eq!(full, over, "the truth is the same");
    assert!(full.last().is_some_and(|t| t.0.z < -0.1), "and it flew");
    assert_ne!(hash_full, hash_over, "the hash covers the commands");
}

#[test]
fn a_failing_tick_leaves_a_queued_parameter_change_and_ack_queued() {
    let set = ParamSet {
        id: oxide_core::params::find("BAT_LOW_V").expect("a parameter"),
        value: ParamValue::F32(14.0),
    };
    let mut stub = Stub::new(500);
    stub.fail_at_call = Some(3);
    let mut sim = stub_sim(stub);
    for _ in 0..3 {
        sim.try_step().expect("the first ticks succeed");
    }
    sim.queue_param_set(set);
    sim.queue_param_ack();
    sim.try_step().expect_err("the 4th tick fails");
    assert!(
        sim.autopilot().seen_sets.is_empty() && sim.autopilot().seen_acks == 0,
        "the failing tick never delivered them"
    );
    // The next tick (the stub fails only once) gets both, exactly once.
    sim.try_step().expect("the next tick succeeds");
    assert_eq!(sim.autopilot().seen_sets, vec![set]);
    assert_eq!(sim.autopilot().seen_acks, 1);
    sim.try_step().expect("and the next");
    assert_eq!(sim.autopilot().seen_sets, vec![set]);
    assert_eq!(sim.autopilot().seen_acks, 1);
}

#[test]
fn the_autopilot_is_told_how_much_thrust_each_rotor_has_left() {
    use oxide_sim::faults::{Fault, FaultKind};
    let options = SimOptions {
        faults: vec![
            Fault {
                kind: FaultKind::MotorLoss {
                    rotor: 1,
                    scale: 0.4,
                },
                start_s: 0.01,
                end_s: Some(0.02),
            },
            // Overlapping on one rotor: the worst applies.
            Fault {
                kind: FaultKind::MotorLoss {
                    rotor: 1,
                    scale: 0.7,
                },
                start_s: 0.015,
                end_s: Some(0.03),
            },
            Fault {
                kind: FaultKind::MotorLoss {
                    rotor: 3,
                    scale: 0.0,
                },
                start_s: 0.025,
                end_s: None,
            },
        ],
        ..SimOptions::default()
    };
    let mut sim =
        Simulation::with_autopilot(&airframe(), Stub::new(500), NoPilot, options).expect("builds");
    sim.try_run_until(Micros(40_000), |_| {})
        .expect("cannot fail");
    let seen = &sim.autopilot().seen_scales;
    let at = |us: usize| seen[us / 500];
    assert_eq!(at(0), [1.0; 4]);
    assert_eq!(at(9_500), [1.0; 4]);
    assert_eq!(at(10_000), [1.0, 0.4, 1.0, 1.0]);
    assert_eq!(at(15_000), [1.0, 0.4, 1.0, 1.0], "the worst scale wins");
    assert_eq!(at(20_000), [1.0, 0.7, 1.0, 1.0]);
    assert_eq!(at(25_000), [1.0, 0.7, 1.0, 0.0]);
    assert_eq!(at(30_000), [1.0, 1.0, 1.0, 0.0]);
    assert_eq!(at(39_500), [1.0, 1.0, 1.0, 0.0]);
}

fn fly_with_question_mark(stub: Stub) -> Result<u64, Box<dyn std::error::Error>> {
    let mut sim = stub_sim(stub);
    for _ in 0..20 {
        sim.try_step()?;
    }
    Ok(sim.into_autopilot().calls)
}

#[test]
fn an_autopilot_error_is_a_std_error_and_the_autopilot_can_be_taken_back() {
    assert_eq!(fly_with_question_mark(Stub::new(500)).expect("flies"), 20);
    let mut failing = Stub::new(500);
    failing.fail_at_call = Some(5);
    let e = fly_with_question_mark(failing).expect_err("the 6th tick fails");
    assert_eq!(e.to_string(), "the stub's process died at its tick 5");
    assert!(e.downcast_ref::<StubError>().is_some());
    // Our own core's error is one too (it cannot happen).
    fn is_std_error<E: std::error::Error + 'static>() {}
    is_std_error::<<oxide_sim::CoreAutopilot<4> as Autopilot<4>>::Error>();
}
