// SPDX-License-Identifier: AGPL-3.0-or-later
//! The lockstep loop: the simulator owns the clock and drives the flight core
//! through `Core::tick`, exactly as firmware will (DESIGN §4.2, §5.1).
//!
//! Per tick `k` at time `t = k·base`:
//! 1. sample sensors from the true state at `t`;
//! 2. run `Core::tick(t)`; its outputs are valid from `t + base` (LET);
//! 3. integrate physics over `[t, t + base]` with the command valid at `t`.

use oxide_core::Core;
use oxide_core::io::{Inputs, Outputs, Status, TrajectorySetpoint};
use oxide_core::math::{Quat, Vec3};
use oxide_core::schedule::Profile;
use oxide_core::time::Micros;

use crate::SimError;
use crate::airframe::Airframe;
use crate::battery::Battery;
use crate::hash::TraceHash;
use crate::scenario::Pilot;
use crate::sensors;
use crate::vehicle::{BodyState, GRAVITY, VehicleModel, ground_contact};

/// Battery monitor sample period, µs.
const BATTERY_PERIOD_US: u64 = 10_000;

/// What happened in one tick, for logging, viewers and tests.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StepRecord<const N: usize> {
    /// Time of the tick (state is the state at this time).
    pub t: Micros,
    pub position: Vec3<f64>,
    pub velocity: Vec3<f64>,
    pub attitude: Quat<f64>,
    pub rate: Vec3<f64>,
    /// Motor commands produced at this tick (applied from `t + base`).
    pub motors: [f32; N],
    pub rotor_speed: [f64; N],
    pub battery_v: f64,
    pub battery_a: f64,
    pub status: Status,
    pub setpoint: Option<TrajectorySetpoint>,
}

pub struct Simulation<P: Pilot, const N: usize> {
    core: Core<N>,
    model: VehicleModel<N>,
    body: BodyState<N>,
    battery: Battery,
    pilot: P,
    tick: u64,
    base_us: u64,
    dt: f64,
    battery_every: u64,
    active: Outputs<N>,
    pending: Option<Outputs<N>>,
    specific_force_world: Vec3<f64>,
    hash: TraceHash,
}

impl<P: Pilot, const N: usize> Simulation<P, N> {
    pub fn new(airframe: &Airframe, profile: Profile, pilot: P) -> Result<Self, SimError> {
        let model = VehicleModel::<N>::from_airframe(airframe)?;
        let core = Core::new(airframe.core_config::<N>()?, profile).map_err(SimError::Core)?;
        let base_us = u64::from(profile.base_period_us);
        // One time base: every period must be a whole number of ticks.
        if base_us == 0
            || !BATTERY_PERIOD_US.is_multiple_of(base_us)
            || !crate::scenario::COMMAND_PERIOD_US.is_multiple_of(base_us)
        {
            return Err(SimError::TimeBase(base_us));
        }
        Ok(Self {
            core,
            model,
            body: BodyState::default(),
            battery: Battery::new(&airframe.battery),
            pilot,
            tick: 0,
            base_us,
            dt: Micros(base_us).as_secs_f64(),
            battery_every: BATTERY_PERIOD_US / base_us,
            active: Outputs::off(Micros::ZERO),
            pending: None,
            specific_force_world: Vec3::new(0.0, 0.0, -GRAVITY),
            hash: TraceHash::default(),
        })
    }

    pub fn time(&self) -> Micros {
        Micros(self.tick.saturating_mul(self.base_us))
    }
    pub fn body(&self) -> &BodyState<N> {
        &self.body
    }
    pub fn model(&self) -> &VehicleModel<N> {
        &self.model
    }
    pub fn model_mut(&mut self) -> &mut VehicleModel<N> {
        &mut self.model
    }
    pub fn set_body(&mut self, body: BodyState<N>) {
        self.body = body;
    }
    pub fn pilot_mut(&mut self) -> &mut P {
        &mut self.pilot
    }
    pub fn core(&self) -> &Core<N> {
        &self.core
    }
    /// Hash of every state so far (DESIGN §5.2 item 6).
    pub fn trace_hash(&self) -> u64 {
        self.hash.value()
    }

    /// Advances one base tick.
    pub fn step(&mut self) -> StepRecord<N> {
        let t = self.time();

        // 1. Sensors at t.
        let setpoint = self.pilot.setpoint(t);
        let inputs = Inputs {
            imu: Some(sensors::imu(t, &self.body, self.specific_force_world)),
            state: Some(sensors::state(t, &self.body)),
            rc: self
                .pilot
                .rc(t)
                .map(|r| oxide_core::io::Stamped::ideal(t, r)),
            battery: self
                .tick
                .is_multiple_of(self.battery_every)
                .then(|| sensors::battery(t, self.battery.voltage_v, self.battery.current_a)),
            setpoint: setpoint.map(|s| oxide_core::io::Stamped::ideal(t, s)),
            ..Inputs::default()
        };

        // 2. Core tick.
        let out = self.core.tick(t, &inputs);

        // 3. Physics over [t, t + base] with the command valid at t.
        if let Some(p) = self.pending.take() {
            if p.t_valid <= t {
                self.active = p;
            } else {
                self.pending = Some(p);
            }
        }
        let record = StepRecord {
            t,
            position: self.body.position,
            velocity: self.body.velocity,
            attitude: self.body.attitude,
            rate: self.body.rate,
            motors: out.motors,
            rotor_speed: self.body.rotor_speed,
            battery_v: self.battery.voltage_v,
            battery_a: self.battery.current_a,
            status: out.status,
            setpoint,
        };
        self.integrate();
        self.pending = Some(out);
        self.hash_state();
        self.tick = self.tick.saturating_add(1);
        record
    }

    fn integrate(&mut self) {
        let v = self.battery.voltage_v;
        let cmd: [f64; N] = core::array::from_fn(|i| {
            let u = self.active.motors.get(i).copied().map_or(0.0, f64::from);
            self.model.speed_command(u, v)
        });
        let v_before = self.body.velocity;
        let mut next = self.model.rk4(&self.body, &cmd, self.dt);
        ground_contact(&mut next);
        if !next.is_finite() {
            // Never propagate NaN: freeze and let tests catch the anomaly.
            next = self.body;
            next.velocity = Vec3::zero();
            next.rate = Vec3::zero();
        }
        let g = if self.model.gravity {
            Vec3::new(0.0, 0.0, GRAVITY)
        } else {
            Vec3::zero()
        };
        self.specific_force_world = (next.velocity - v_before) * (1.0 / self.dt) - g;
        self.body = next;
        let loads = self.model.loads(&self.body);
        let electrical = loads.shaft_power_w / self.model.efficiency + self.model.idle_power_w;
        self.battery.draw(electrical, self.dt);
    }

    fn hash_state(&mut self) {
        let b = &self.body;
        let h = &mut self.hash;
        h.write_u64(self.tick);
        for v in [b.position, b.velocity, b.rate] {
            h.write_f64(v.x);
            h.write_f64(v.y);
            h.write_f64(v.z);
        }
        for q in [b.attitude.w, b.attitude.x, b.attitude.y, b.attitude.z] {
            h.write_f64(q);
        }
        for s in b.rotor_speed {
            h.write_f64(s);
        }
        for m in self.active.motors {
            h.write_f32(m);
        }
        h.write_f64(self.battery.voltage_v);
    }

    /// Runs until `t_end`, calling `on_step` for every tick.
    pub fn run_until(&mut self, t_end: Micros, mut on_step: impl FnMut(&StepRecord<N>)) {
        while self.time() < t_end {
            let r = self.step();
            on_step(&r);
        }
    }
}
