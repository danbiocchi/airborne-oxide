# ADR-0001: Flight-core interface

**Status:** Accepted (M1a) · **Date:** 2026-11-09 · **Design reference:** DESIGN §4.2, §5.2, §5.3

## Context

The simulator has to test the *real* multi-rate behaviour of the flight core, not a harness of its own. Board firmware and the simulator therefore have to drive the core in the same way, and the results must not depend on how RTIC happens to preempt tasks.

## Decision

1. **One entry point.** `Core::tick(now: Micros, inputs: &Inputs) -> Outputs<N>` is called exactly once per base tick, by the firmware's highest-priority periodic task or by the simulator. A tick that arrives at the wrong time increments `Status::timing_faults`.

2. **The core owns the schedule.** A `Profile` fixes the base tick and the period of every task, in whole base ticks:

   | Task | F405 "lean" (1 kHz base) | H743 "full" (4 kHz base) |
   |---|---|---|
   | Estimator | every tick | every tick |
   | Supervisor (arming, kill, mode) | 50 Hz | 50 Hz |
   | Guidance (position/velocity, sticks) | 50 Hz | 100 Hz |
   | Attitude | 250 Hz | 500 Hz |
   | Rate + allocation | every tick | every tick |

   `Profile::validate` rejects tables with a missing or duplicated task, a zero period, or a rate task slower than the base tick.

3. **Logical-execution-time rule.** When a task is released, it first publishes what it computed in its previous period, then computes again from the inputs as they are now. Other tasks only ever see published values. Each task's latency is therefore fixed at one period. Motor commands computed at tick *k* carry `t_valid = t_k + base` and are applied from then on.

4. **Timestamped inputs.** Every input is a `Stamped<T> { t_sample, t_arrival, value }`. `None` means no new sample this tick. The core keeps the newest sample of each kind and judges staleness on `t_arrival` against its own clock: state 100 ms, RC 500 ms, setpoint 500 ms, battery 1 s.

5. **Units and types.** Time is `u64` microseconds (`Micros`). Physical values are SI. The world frame is NED and the body frame FRD. `Inputs`, `Outputs`, `Status` and the sample types live in `oxide-core::io`.

6. **Simulator side.** The simulator tick equals the profile's base tick. Physics integrates `[t, t + base]` with the command valid at `t`. Every sensor and command period must be a whole number of ticks, or `Simulation::new` fails with `SimError::TimeBase`.

## Consequences

- The same task table runs on the chip and in CI. Both profiles are flown by the closed-loop tests (`crates/oxide-sim/tests/closed_loop.rs`).
- Results are bit-identical run to run. CI also compares trace hashes between Linux and Windows.
- Cascaded loops pick up one period of latency per level. That latency is deterministic, and the controllers are tuned with it included.
- The compute-and-output latency model in DESIGN §4.2 is the fixed one-period default for now. A measured per-task latency replaces it after the F405 bring-up.

## Deferred

- The estimator task passes through an external state (simulator truth) until the EKF lands in M1b.
- RC-loss and other failsafes (M1c). Until then, an invalid RC frame is ignored and the previous decisions stand.
- The full command gateway and link authority (M3). M1a only drops non-finite setpoints.
