// SPDX-License-Identifier: AGPL-3.0-or-later
//! Scenario files (ADR-0004 §7, DESIGN §5.5): one TOML file per scenario.
//!
//! A file holds the flight (pilot, faults, wind, battery, parameters), what
//! must happen (`[expect]`) and how often CI flies it (`[run]`). Unknown
//! keys are errors; states, reasons, actions and modes are written by name;
//! heights are up-positive and the loader converts them to NED. Times are
//! seconds of simulation time.
//!
//! ```toml
//! # Required: name (also the key the run seeds are derived from) and
//! # duration_s. Everything else has the default noted beside it.
//! name = "gps-loss"
//! about = "GPS lost for good in Offboard: ..."   # default ""
//! airframe = "quad-x-5in"                        # default; the embedded reference
//! duration_s = 70.0
//!
//! [sim]                       # default: ideal sensors, external estimator, brain link only
//! sensors = "realistic"       # "ideal" | "realistic"
//! estimator = "ekf"           # "external" (truth) | "ekf"
//! [sim.links]
//! gcs = false                 # a ground station sends heartbeats (1 Hz)
//! brain = true                # a brain sends heartbeats (5 Hz)
//! rc_failsafe_delay_us = 1000000  # from the start of an RC loss to the receiver's failsafe flag
//!
//! [wind]                      # default: calm
//! kind = "random"             # "calm" | "fixed" | "random"
//! steady_speed_mps = [0.0, 2.0]   # random: drawn U(lo, hi) from the Scenario stream, then a bearing,
//! sigma_h_mps = [0.1, 0.4]        # then sigma_h, then each gust (start, duration, peak_h, bearing, peak_d)
//! sigma_v_ratio = 0.5             # default 0.5: sigma_z = ratio * sigma_h
//! tau_s = 2.0
//! # [wind.gusts]              # optional: count, start_s, duration_s, peak_h_mps, peak_v_mps (ranges)
//! # fixed: steady_ned = [n, e, d], turbulence_sigma = [n, e, d], turbulence_tau_s,
//! #        and [[wind.gust]] start_s, duration_s, peak_ned = [n, e, d] (m/s, NED)
//!
//! [battery]                   # optional: replaces these fields of the airframe's battery
//! capacity_mah = 4000.0
//! initial_state_of_charge = 0.0614
//!
//! [params]                    # parameter overrides by name, applied in name order
//! FS_GPS_LAND = 10.0
//!
//! [[fault]]                   # kind: motor-loss (rotor, scale), gps-loss, imu-loss, baro-loss,
//! kind = "gps-loss"           #       mag-loss, rc-loss, rc-silent, gcs-loss, brain-crash, brain-hang
//! start_s = 15.0
//! # end_s = 19.0              # absent = to the end of the run
//!
//! [pilot]                     # default: never arms, Stabilized, no setpoints
//! arm_at_s = 3.0              # absent = never; disarm_at_s and kill_at_s likewise
//! [[pilot.mode]]              # switch from t on: "stabilized" | "position" | "offboard"
//! t = 0.0
//! switch = "offboard"
//! [[pilot.sticks]]            # roll, pitch, yaw in -1..1, throttle in 0..1 (default 0)
//! t = 0.0
//! throttle = 0.0
//! [[pilot.setpoint]]          # the brain's setpoint from t on; height is up-positive
//! t = 0.0
//! position = [0.0, 0.0, 0.0]  # [north, east, up] m
//! yaw = 0.0                   # rad, default 0
//! [[pilot.setpoint]]
//! t = 15.0
//! silent = true               # the brain sends nothing from t on
//!
//! [expect]                    # default: nothing required (see `Expect::default`)
//! nav_contains = ["hold", "land"]     # in this order
//! nav_never = []
//! reasons = ["GPS_LOST"]              # all of these raised
//! reasons_never = []
//! events = []                         # armed, arm-refused, mode-degraded, pilot-override, touchdown, home-set
//! min_action = "none"                 # none, hold, rtl, land, land-now, terminate
//! by_s = 65.0
//! max_touchdown_mps = 1.5
//! max_tilt_deg = 40.0
//! min_height_m = 4.0                  # also max_height_m, max_distance_from_home_m
//! [expect.ending]             # kind: flying (point [n, e, up], tolerance_m), flying-at-height (height_m,
//! kind = "landed"             #       tolerance_m), landed (within_m_of_home), terminated,
//! within_m_of_home = 40.0     #       disarmed (reason: switch, kill, touchdown, terminated), down
//! # [expect.hold]             # the hold criteria of `metrics::HoldCriteria` (gust-hold)
//!
//! [run]                       # for the Monte-Carlo job
//! seeds = 10                  # default 1
//! profiles = ["f405", "h743"] # default both
//! ```

use std::fmt;
use std::path::Path;

use oxide_core::config::EstimatorSource;
use oxide_core::io::{Action, DisarmReason, NavState, Reasons, TrajectorySetpoint};
use oxide_core::math::Vec3;
use oxide_core::schedule::{F405_LEAN, H743_FULL, Profile};
use oxide_core::time::Micros;
use serde::Deserialize;

use crate::SimError;
use crate::airframe::{Airframe, BatterySpec, ordered_params};
use crate::faults::{Fault, FaultKind, LinkConfig, validate_faults};
use crate::metrics::{HoldCriteria, HoldMetrics};
use crate::outcome::{Ending, EventTag, Expect, FlightOutcome, REASON_NAMES};
use crate::rng::{Rng, Stream, run_seed};
use crate::scenario::Script;
use crate::sensors::SensorConfig;
use crate::sim::{SimOptions, Simulation, StepRecord, apply_param};
use crate::wind::{Gust, WindConfig};

#[cfg(test)]
mod tests;

/// The airframe every scenario flies until files can name another.
pub const REFERENCE_AIRFRAME: &str = "quad-x-5in";

/// The rotor count of the airframe (compile-time size of the simulation).
const ROTORS: usize = 4;

// ---- Errors ----------------------------------------------------------------

/// Why a scenario file was refused: the file (when known), the key and the
/// reason.
#[derive(Clone, Debug, PartialEq)]
pub struct ScenarioError {
    /// The path the text came from, if it came from a file.
    pub file: Option<String>,
    /// The key at fault, e.g. `pilot.mode[1].switch`; empty when none applies.
    pub key: String,
    pub reason: String,
}

impl ScenarioError {
    fn new(key: impl Into<String>, reason: impl Into<String>) -> Self {
        Self {
            file: None,
            key: key.into(),
            reason: reason.into(),
        }
    }

    fn in_file(mut self, file: &str) -> Self {
        self.file = Some(file.to_string());
        self
    }
}

impl fmt::Display for ScenarioError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if let Some(file) = &self.file {
            write!(f, "{file}: ")?;
        }
        if !self.key.is_empty() {
            write!(f, "`{}`: ", self.key)?;
        }
        write!(f, "{}", self.reason)
    }
}

impl std::error::Error for ScenarioError {}

type Result<T> = core::result::Result<T, ScenarioError>;

// ---- The loaded scenario -----------------------------------------------------

/// Which sensor error models a scenario flies with.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SensorPreset {
    Ideal,
    Realistic,
}

impl SensorPreset {
    pub fn config(self) -> SensorConfig {
        match self {
            SensorPreset::Ideal => SensorConfig::ideal(),
            SensorPreset::Realistic => SensorConfig::realistic(),
        }
    }
}

/// A flight-controller profile a scenario is flown on in the Monte-Carlo job.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RunProfile {
    F405,
    H743,
}

impl RunProfile {
    pub fn profile(self) -> Profile {
        match self {
            RunProfile::F405 => F405_LEAN,
            RunProfile::H743 => H743_FULL,
        }
    }

    /// The name used in files and on the command line.
    pub fn name(self) -> &'static str {
        match self {
            RunProfile::F405 => "f405",
            RunProfile::H743 => "h743",
        }
    }
}

/// `[run]`: how CI flies the scenario.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RunSpec {
    /// Run indices `0..seeds`.
    pub seeds: u64,
    pub profiles: Vec<RunProfile>,
}

impl Default for RunSpec {
    fn default() -> Self {
        Self {
            seeds: 1,
            profiles: vec![RunProfile::F405, RunProfile::H743],
        }
    }
}

/// Gusts drawn per run: `count` gusts, each with start, duration, horizontal
/// peak, bearing and vertical peak drawn in this order.
#[derive(Clone, Debug, PartialEq)]
pub struct RandomGusts {
    pub count: u32,
    pub start_s: [f64; 2],
    pub duration_s: [f64; 2],
    pub peak_h_mps: [f64; 2],
    pub peak_v_mps: [f64; 2],
}

/// Wind drawn per run from the run seed's `Scenario` stream, in this order:
/// steady speed, its bearing, turbulence `sigma_h`, then the gusts.
#[derive(Clone, Debug, PartialEq)]
pub struct RandomWind {
    pub steady_speed_mps: [f64; 2],
    pub sigma_h_mps: [f64; 2],
    /// Vertical turbulence is this fraction of the horizontal.
    pub sigma_v_ratio: f64,
    pub tau_s: f64,
    pub gusts: Option<RandomGusts>,
}

impl RandomWind {
    /// The wind of the run with seed `seed`.
    pub fn generate(&self, seed: u64) -> WindConfig {
        let mut rng = Rng::stream(seed, Stream::Scenario);
        let speed = rng.range(self.steady_speed_mps[0], self.steady_speed_mps[1]);
        let steady_ned = rng.horizontal_direction() * speed;
        let sigma_h = rng.range(self.sigma_h_mps[0], self.sigma_h_mps[1]);
        let mut gusts = Vec::new();
        if let Some(g) = &self.gusts {
            for _ in 0..g.count {
                let start_s = rng.range(g.start_s[0], g.start_s[1]);
                let duration_s = rng.range(g.duration_s[0], g.duration_s[1]);
                let peak_h = rng.range(g.peak_h_mps[0], g.peak_h_mps[1]);
                let dir = rng.horizontal_direction();
                let peak_d = rng.range(g.peak_v_mps[0], g.peak_v_mps[1]);
                gusts.push(Gust {
                    start_s,
                    duration_s,
                    peak_ned: Vec3::new(dir.x * peak_h, dir.y * peak_h, peak_d),
                });
            }
        }
        WindConfig {
            steady_ned,
            turbulence_sigma: Vec3::new(sigma_h, sigma_h, self.sigma_v_ratio * sigma_h),
            turbulence_tau_s: self.tau_s,
            gusts,
        }
    }
}

/// `[wind]`.
#[derive(Clone, Debug, PartialEq)]
pub enum WindSpec {
    Calm,
    /// The same wind in every run.
    Fixed(WindConfig),
    /// Drawn per run.
    Random(RandomWind),
}

impl WindSpec {
    /// The wind of the run with seed `seed`.
    pub fn config(&self, seed: u64) -> WindConfig {
        match self {
            WindSpec::Calm => WindConfig::calm(),
            WindSpec::Fixed(w) => w.clone(),
            WindSpec::Random(r) => r.generate(seed),
        }
    }
}

/// `[battery]`: the fields of the airframe's battery this scenario replaces.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BatteryOverride {
    pub capacity_mah: Option<f64>,
    pub initial_state_of_charge: Option<f64>,
}

impl BatteryOverride {
    /// The airframe's battery with this override applied.
    pub fn apply(&self, base: &BatterySpec) -> BatterySpec {
        let mut b = base.clone();
        if let Some(c) = self.capacity_mah {
            b.capacity_mah = c;
        }
        if let Some(s) = self.initial_state_of_charge {
            b.initial_state_of_charge = s;
        }
        b
    }
}

/// The point a hold scenario must hold (NED, m): the position of the last
/// setpoint the brain sends.
fn hold_target(pilot: &Script) -> Option<Vec3<f64>> {
    pilot
        .setpoints
        .iter()
        .rev()
        .find_map(|(_, sp)| sp.map(|sp| sp.position.cast::<f64>()))
}

/// What one flight did against what its file asks: [`Checker`]'s result.
#[derive(Clone, Debug, PartialEq)]
pub struct Verdict {
    pub outcome: FlightOutcome,
    /// The hold statistics, when the file has `[expect.hold]`.
    pub hold: Option<HoldMetrics>,
    /// Every expectation the flight broke, in words; empty = passed. A file
    /// with neither `[expect]` nor `[expect.hold]` has none to break.
    pub violations: Vec<String>,
}

/// Collects a flight's outcome and hold statistics step by step, then judges
/// them against the file's `[expect]` and `[expect.hold]`.
#[derive(Clone, Debug)]
pub struct Checker {
    outcome: FlightOutcome,
    hold: Option<(HoldMetrics, HoldCriteria, Vec3<f64>)>,
    expect: Option<Expect>,
}

impl Checker {
    pub fn new(file: &ScenarioFile) -> Self {
        let hold = file
            .hold
            .and_then(|c| hold_target(&file.pilot).map(|target| (HoldMetrics::new(), c, target)));
        Self {
            outcome: FlightOutcome::new(),
            hold,
            expect: file.expect.clone(),
        }
    }

    pub fn add(&mut self, r: &StepRecord<ROTORS>) {
        self.outcome.add(r);
        if let Some((m, c, target)) = &mut self.hold {
            m.add(r, *target, c);
        }
    }

    pub fn finish(self) -> Verdict {
        let mut violations = self
            .expect
            .as_ref()
            .map_or_else(Vec::new, |e| e.check(&self.outcome));
        let hold = self.hold.map(|(m, c, _)| {
            violations.extend(m.failures(&c));
            m
        });
        Verdict {
            outcome: self.outcome,
            hold,
            violations,
        }
    }
}

/// A scenario as loaded from a file.
#[derive(Clone, Debug, PartialEq)]
pub struct ScenarioFile {
    /// Also the key the run seeds are derived from (`rng::run_seed`).
    pub name: String,
    pub about: String,
    pub airframe: String,
    pub duration_s: f64,
    pub sensors: SensorPreset,
    pub estimator: EstimatorSource,
    pub links: LinkConfig,
    pub wind: WindSpec,
    pub battery: Option<BatteryOverride>,
    /// Parameter overrides by name, in the order they are applied (the TOML
    /// reader yields a table sorted by key, so: name order).
    pub params: Vec<(String, f64)>,
    pub faults: Vec<Fault>,
    /// What the radio and the brain send. `pilot.duration_s` is
    /// `duration_s`.
    pub pilot: Script,
    /// What must happen; `None` when the file has no `[expect]` keys.
    pub expect: Option<Expect>,
    /// `[expect.hold]`, the criteria of a hold scenario.
    pub hold: Option<HoldCriteria>,
    pub run: RunSpec,
}

impl ScenarioFile {
    /// The seed of run `index`.
    pub fn seed(&self, index: u64) -> u64 {
        run_seed(&self.name, index)
    }

    /// The simulation options of run `index`, for `airframe`.
    pub fn options(&self, airframe: &Airframe, index: u64) -> SimOptions {
        self.options_with(airframe, index, &[])
    }

    /// Like [`options`](Self::options), with `extra` parameter overrides
    /// applied after the file's own `[params]` (the command line, ADR-0004
    /// §5).
    pub fn options_with(
        &self,
        airframe: &Airframe,
        index: u64,
        extra: &[(String, f64)],
    ) -> SimOptions {
        let seed = self.seed(index);
        let mut params = self.params.clone();
        params.extend_from_slice(extra);
        SimOptions {
            seed,
            sensors: self.sensors.config(),
            wind: self.wind.config(seed),
            estimator: self.estimator,
            faults: self.faults.clone(),
            links: self.links,
            failsafe: None,
            battery: self.battery.map(|b| b.apply(&airframe.battery)),
            params,
        }
    }

    /// The configured simulation of run `index` on `profile`, not yet run.
    pub fn build(
        &self,
        profile: Profile,
        index: u64,
    ) -> core::result::Result<Simulation<Script, ROTORS>, SimError> {
        self.build_with(profile, index, |w| w, &[])
    }

    /// Like [`build`](Self::build), with `adjust` applied to the run's wind
    /// (for example to add an updraft).
    pub fn build_with_wind(
        &self,
        profile: Profile,
        index: u64,
        adjust: impl FnOnce(WindConfig) -> WindConfig,
    ) -> core::result::Result<Simulation<Script, ROTORS>, SimError> {
        self.build_with(profile, index, adjust, &[])
    }

    /// Like [`build`](Self::build), with parameter overrides by name
    /// (ADR-0004) applied after the file's own.
    pub fn build_with_params(
        &self,
        profile: Profile,
        index: u64,
        params: &[(String, f64)],
    ) -> core::result::Result<Simulation<Script, ROTORS>, SimError> {
        self.build_with(profile, index, |w| w, params)
    }

    fn build_with(
        &self,
        profile: Profile,
        index: u64,
        adjust: impl FnOnce(WindConfig) -> WindConfig,
        params: &[(String, f64)],
    ) -> core::result::Result<Simulation<Script, ROTORS>, SimError> {
        let airframe = Airframe::reference_quad()?;
        let mut options = self.options_with(&airframe, index, params);
        options.wind = adjust(options.wind);
        Simulation::<_, ROTORS>::with_options(&airframe, profile, self.pilot.clone(), options)
    }

    /// The time the run ends.
    pub fn end(&self) -> Micros {
        Micros((self.duration_s * 1e6).round() as u64)
    }

    /// Flies run `index` on `profile` to the end, with `params` applied after
    /// the file's own, and judges it against `[expect]` and `[expect.hold]`.
    pub fn fly_checked(
        &self,
        profile: Profile,
        index: u64,
        params: &[(String, f64)],
    ) -> core::result::Result<Verdict, SimError> {
        let mut sim = self.build_with_params(profile, index, params)?;
        let mut checker = Checker::new(self);
        sim.run_until(self.end(), |r| checker.add(r));
        Ok(checker.finish())
    }

    /// Flies run `index` on `profile` to the end and returns what happened.
    pub fn fly(
        &self,
        profile: Profile,
        index: u64,
    ) -> core::result::Result<FlightOutcome, SimError> {
        let mut sim = self.build(profile, index)?;
        let mut outcome = FlightOutcome::new();
        let end = Micros((self.duration_s * 1e6).round() as u64);
        sim.run_until(end, |r| outcome.add(r));
        Ok(outcome)
    }
}

// ---- Name tables -------------------------------------------------------------

pub const NAV_STATE_NAMES: [(NavState, &str); 10] = [
    (NavState::Disarmed, "disarmed"),
    (NavState::Stabilized, "stabilized"),
    (NavState::Altitude, "altitude"),
    (NavState::Position, "position"),
    (NavState::Offboard, "offboard"),
    (NavState::Hold, "hold"),
    (NavState::Rtl, "rtl"),
    (NavState::Land, "land"),
    (NavState::LandNow, "land-now"),
    (NavState::Terminated, "terminated"),
];

pub const ACTION_NAMES: [(Action, &str); 6] = [
    (Action::None, "none"),
    (Action::Hold, "hold"),
    (Action::Rtl, "rtl"),
    (Action::Land, "land"),
    (Action::LandNow, "land-now"),
    (Action::Terminate, "terminate"),
];

pub const DISARM_REASON_NAMES: [(DisarmReason, &str); 4] = [
    (DisarmReason::Switch, "switch"),
    (DisarmReason::Kill, "kill"),
    (DisarmReason::Touchdown, "touchdown"),
    (DisarmReason::Terminated, "terminated"),
];

pub const EVENT_NAMES: [(EventTag, &str); 6] = [
    (EventTag::Armed, "armed"),
    (EventTag::ArmRefused, "arm-refused"),
    (EventTag::ModeDegraded, "mode-degraded"),
    (EventTag::PilotOverride, "pilot-override"),
    (EventTag::Touchdown, "touchdown"),
    (EventTag::HomeSet, "home-set"),
];

/// The mode switch positions, as the supervisor reads them (0 Stabilized,
/// 1 Position, anything higher Offboard).
pub const MODE_NAMES: [(u8, &str); 3] = [(0, "stabilized"), (1, "position"), (2, "offboard")];

/// The name of `value` in `table`.
pub fn name_of<T: Copy + PartialEq>(table: &[(T, &'static str)], value: T) -> Option<&'static str> {
    table.iter().find(|(v, _)| *v == value).map(|(_, n)| *n)
}

fn lookup<T: Copy>(table: &[(T, &str)], what: &str, key: &str, name: &str) -> Result<T> {
    table
        .iter()
        .find(|(_, n)| *n == name)
        .map(|(v, _)| *v)
        .ok_or_else(|| {
            let known: Vec<&str> = table.iter().map(|(_, n)| *n).collect();
            ScenarioError::new(
                key,
                format!("unknown {what} `{name}` (one of: {})", known.join(", ")),
            )
        })
}

fn lookup_reasons(key: &str, names: &[String]) -> Result<Reasons> {
    let mut out = Reasons::empty();
    for (i, n) in names.iter().enumerate() {
        out.insert(lookup(&REASON_NAMES, "reason", &format!("{key}[{i}]"), n)?);
    }
    Ok(out)
}

// ---- The file as serde sees it ----------------------------------------------

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RawFile {
    name: String,
    about: Option<String>,
    airframe: Option<String>,
    duration_s: f64,
    sim: Option<RawSim>,
    wind: Option<RawWind>,
    battery: Option<RawBattery>,
    #[serde(default, deserialize_with = "ordered_params")]
    params: Vec<(String, f64)>,
    #[serde(default)]
    fault: Vec<RawFault>,
    pilot: Option<RawPilot>,
    expect: Option<RawExpect>,
    run: Option<RawRun>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RawSim {
    sensors: Option<String>,
    estimator: Option<String>,
    links: Option<RawLinks>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RawLinks {
    gcs: Option<bool>,
    brain: Option<bool>,
    rc_failsafe_delay_us: Option<u64>,
}

#[derive(Deserialize)]
#[serde(tag = "kind", rename_all = "kebab-case", deny_unknown_fields)]
enum RawWind {
    // A struct variant, so that deny_unknown_fields applies to it.
    Calm {},
    Fixed {
        steady_ned: Option<[f64; 3]>,
        turbulence_sigma: Option<[f64; 3]>,
        turbulence_tau_s: Option<f64>,
        gust: Option<Vec<RawGust>>,
    },
    Random {
        steady_speed_mps: [f64; 2],
        sigma_h_mps: [f64; 2],
        sigma_v_ratio: Option<f64>,
        tau_s: f64,
        gusts: Option<RawGusts>,
    },
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RawGust {
    start_s: f64,
    duration_s: f64,
    peak_ned: [f64; 3],
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RawGusts {
    count: u32,
    start_s: [f64; 2],
    duration_s: [f64; 2],
    peak_h_mps: [f64; 2],
    peak_v_mps: [f64; 2],
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RawBattery {
    capacity_mah: Option<f64>,
    initial_state_of_charge: Option<f64>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RawFault {
    kind: String,
    start_s: f64,
    end_s: Option<f64>,
    rotor: Option<usize>,
    scale: Option<f64>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RawPilot {
    arm_at_s: Option<f64>,
    disarm_at_s: Option<f64>,
    kill_at_s: Option<f64>,
    #[serde(default)]
    mode: Vec<RawMode>,
    #[serde(default)]
    sticks: Vec<RawSticks>,
    #[serde(default)]
    setpoint: Vec<RawSetpoint>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RawMode {
    t: f64,
    switch: String,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RawSticks {
    t: f64,
    roll: Option<f32>,
    pitch: Option<f32>,
    yaw: Option<f32>,
    throttle: Option<f32>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RawSetpoint {
    t: f64,
    position: Option<[f32; 3]>,
    yaw: Option<f32>,
    silent: Option<bool>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RawExpect {
    nav_contains: Option<Vec<String>>,
    nav_never: Option<Vec<String>>,
    reasons: Option<Vec<String>>,
    reasons_never: Option<Vec<String>>,
    events: Option<Vec<String>>,
    min_action: Option<String>,
    ending: Option<RawEnding>,
    by_s: Option<f64>,
    max_touchdown_mps: Option<f64>,
    max_tilt_deg: Option<f64>,
    min_height_m: Option<f64>,
    max_height_m: Option<f64>,
    max_distance_from_home_m: Option<f64>,
    hold: Option<RawHold>,
}

#[derive(Deserialize)]
#[serde(tag = "kind", rename_all = "kebab-case", deny_unknown_fields)]
enum RawEnding {
    Flying { point: [f64; 3], tolerance_m: f64 },
    FlyingAtHeight { height_m: f64, tolerance_m: f64 },
    Landed { within_m_of_home: Option<f64> },
    Terminated {},
    Disarmed { reason: String },
    Down {},
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RawHold {
    window_start_s: Option<f64>,
    armed_from_s: Option<f64>,
    max_h_error_m: Option<f64>,
    rms_h_error_m: Option<f64>,
    max_v_error_m: Option<f64>,
    max_tilt_deg: Option<f64>,
    est_max_h_error_m: Option<f64>,
    est_max_v_error_m: Option<f64>,
    est_max_vel_error_mps: Option<f64>,
    est_max_att_error_deg: Option<f64>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RawRun {
    seeds: Option<u64>,
    profiles: Option<Vec<String>>,
}

// ---- Loading ------------------------------------------------------------------

/// Parses and checks a scenario file's text.
pub fn load_str(text: &str) -> Result<ScenarioFile> {
    let raw: RawFile = toml::from_str(text).map_err(|e| from_toml_error(&e, text))?;
    convert(raw)
}

/// Reads, parses and checks the scenario file at `path`. An error names the
/// path.
pub fn load_path(path: impl AsRef<Path>) -> Result<ScenarioFile> {
    let path = path.as_ref();
    let shown = path.display().to_string();
    let text = std::fs::read_to_string(path).map_err(|e| {
        ScenarioError::new("", format!("cannot read the file: {e}")).in_file(&shown)
    })?;
    load_str(&text).map_err(|e| e.in_file(&shown))
}

/// Turns a TOML-layer error into a `ScenarioError`. The span says where the
/// text went wrong; `key_path_at` maps it back to the dotted key path
/// (`fault[0].start_s`). An unknown field is named by the field itself (the
/// first back-quoted word of the message); a missing field is named by the
/// table the span points at plus the field. Everything else (wrong type,
/// bad value, unknown variant) is named by the key the span lies in.
fn from_toml_error(e: &toml::de::Error, text: &str) -> ScenarioError {
    let msg = e.message();
    let quoted = || msg.split('`').nth(1).map(str::to_string);
    let at = e.span().map(|span| key_path_at(text, span.start));
    let join = |table: &str, key: &str| {
        if table.is_empty() {
            key.to_string()
        } else {
            format!("{table}.{key}")
        }
    };
    let key = if msg.starts_with("unknown field") {
        quoted().unwrap_or_default()
    } else if msg.starts_with("missing field") {
        let field = quoted().unwrap_or_default();
        join(at.as_ref().map_or("", |(table, _)| table), &field)
    } else {
        at.map_or_else(String::new, |(table, key)| match key {
            Some(key) => join(&table, &key),
            None => table,
        })
    };
    let reason = match e.span() {
        Some(span) => {
            let line = text
                .get(..span.start)
                .map_or(1, |s| s.matches('\n').count() + 1);
            format!("{msg} (line {line})")
        }
        None => msg.to_string(),
    };
    ScenarioError::new(key, reason)
}

/// A `key = value` line's key, unquoted, or `None` for any other line.
fn assigned_key(line: &str) -> Option<String> {
    let line = line.trim_start();
    let (k, _) = line.split_once('=')?;
    let k = k.trim();
    let ok = !k.is_empty()
        && k.chars()
            .all(|c| c.is_ascii_alphanumeric() || matches!(c, '_' | '-' | '.' | '"' | '\''));
    ok.then(|| k.replace(['"', '\''], ""))
}

/// The dotted key path of the text at byte offset `offset`: the table
/// header in force (`[[fault]]` counted as `fault[i]`), then the key of the
/// assignment the offset lies in, returned as `(table, key)`; the key is
/// `None` when the offset lies on a table header.
fn key_path_at(text: &str, offset: usize) -> (String, Option<String>) {
    let offset = offset.min(text.len());
    let line_end = text
        .get(offset..)
        .and_then(|r| r.find('\n'))
        .map_or(text.len(), |n| offset + n);
    let lines: Vec<&str> = text.get(..line_end).unwrap_or(text).lines().collect();
    let last = lines.len().saturating_sub(1);
    let mut table = String::new();
    let mut counts: Vec<(String, usize)> = Vec::new();
    let mut key: Option<String> = None;
    let mut on_header = false;
    for (i, raw) in lines.iter().enumerate() {
        let line = raw.trim();
        if let Some(h) = line.strip_prefix("[[").and_then(|l| l.split("]]").next()) {
            let h = h.trim().to_string();
            let n = match counts.iter_mut().find(|(k, _)| *k == h) {
                Some((_, n)) => {
                    *n += 1;
                    *n
                }
                None => {
                    counts.push((h.clone(), 0));
                    0
                }
            };
            table = format!("{h}[{n}]");
            key = None;
            on_header = i == last;
        } else if let Some(h) = line.strip_prefix('[').and_then(|l| l.split(']').next()) {
            if !line.starts_with("[[") && line.ends_with(']') {
                table = h.trim().to_string();
                key = None;
                on_header = i == last;
            }
        } else if let Some(k) = assigned_key(line) {
            key = Some(k);
            on_header = false;
        }
    }
    (table, key.filter(|_| !on_header))
}

fn finite(key: &str, v: f64) -> Result<f64> {
    if v.is_finite() {
        Ok(v)
    } else {
        Err(ScenarioError::new(key, "must be a finite number"))
    }
}

fn time(key: &str, v: f64) -> Result<f64> {
    if v.is_finite() && v >= 0.0 {
        Ok(v)
    } else {
        Err(ScenarioError::new(
            key,
            "must be a time of zero or more seconds",
        ))
    }
}

fn range(key: &str, r: [f64; 2]) -> Result<[f64; 2]> {
    if !(r[0].is_finite() && r[1].is_finite()) || r[0] > r[1] {
        return Err(ScenarioError::new(
            key,
            "must be [low, high] with finite numbers and low <= high",
        ));
    }
    Ok(r)
}

/// Entry times must be finite, not negative and strictly increasing.
fn sorted(key: &str, times: impl Iterator<Item = f64>) -> Result<()> {
    let mut prev: Option<f64> = None;
    for (i, t) in times.enumerate() {
        time(&format!("{key}[{i}].t"), t)?;
        if prev.is_some_and(|p| t <= p) {
            return Err(ScenarioError::new(
                format!("{key}[{i}].t"),
                "the entries must be in strictly increasing order of time",
            ));
        }
        prev = Some(t);
    }
    Ok(())
}

fn convert(raw: RawFile) -> Result<ScenarioFile> {
    if raw.name.is_empty()
        || !raw
            .name
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || matches!(c, '-' | '_' | '+' | '.'))
    {
        return Err(ScenarioError::new(
            "name",
            "must be non-empty and use only letters, digits and - _ + .",
        ));
    }
    let airframe = raw
        .airframe
        .unwrap_or_else(|| REFERENCE_AIRFRAME.to_string());
    if airframe != REFERENCE_AIRFRAME {
        return Err(ScenarioError::new(
            "airframe",
            format!(
                "unknown airframe `{airframe}` (only the embedded `{REFERENCE_AIRFRAME}` exists; paths come later)"
            ),
        ));
    }
    if !(raw.duration_s.is_finite() && raw.duration_s > 0.0) {
        return Err(ScenarioError::new(
            "duration_s",
            "must be more than zero seconds",
        ));
    }

    let (sensors, estimator, links) = convert_sim(raw.sim)?;
    let wind = raw.wind.map_or(Ok(WindSpec::Calm), convert_wind)?;
    let battery = raw
        .battery
        .map(|b| -> Result<BatteryOverride> {
            if let Some(c) = b.capacity_mah
                && !(c.is_finite() && c > 0.0)
            {
                return Err(ScenarioError::new(
                    "battery.capacity_mah",
                    "must be more than zero",
                ));
            }
            if let Some(s) = b.initial_state_of_charge
                && !(s.is_finite() && (0.0..=1.0).contains(&s))
            {
                return Err(ScenarioError::new(
                    "battery.initial_state_of_charge",
                    "must be between 0 and 1",
                ));
            }
            Ok(BatteryOverride {
                capacity_mah: b.capacity_mah,
                initial_state_of_charge: b.initial_state_of_charge,
            })
        })
        .transpose()?;
    check_params(&raw.params)?;
    let faults = convert_faults(&raw.fault, &links)?;
    let pilot = convert_pilot(raw.pilot, raw.duration_s)?;
    let (expect, hold) = match raw.expect {
        Some(e) => convert_expect(e)?,
        None => (None, None),
    };
    let run = convert_run(raw.run)?;
    if hold.is_some() && hold_target(&pilot).is_none() {
        return Err(ScenarioError::new(
            "expect.hold",
            "needs a [[pilot.setpoint]] with a position: that is the point to hold",
        ));
    }
    Ok(ScenarioFile {
        name: raw.name,
        about: raw.about.unwrap_or_default(),
        airframe,
        duration_s: raw.duration_s,
        sensors,
        estimator,
        links,
        wind,
        battery,
        params: raw.params,
        faults,
        pilot,
        expect,
        hold,
        run,
    })
}

fn convert_sim(raw: Option<RawSim>) -> Result<(SensorPreset, EstimatorSource, LinkConfig)> {
    let mut sensors = SensorPreset::Ideal;
    let mut estimator = EstimatorSource::External;
    let mut links = LinkConfig::default();
    if let Some(s) = raw {
        if let Some(n) = s.sensors {
            sensors = lookup(
                &[
                    (SensorPreset::Ideal, "ideal"),
                    (SensorPreset::Realistic, "realistic"),
                ],
                "sensor preset",
                "sim.sensors",
                &n,
            )?;
        }
        if let Some(n) = s.estimator {
            estimator = lookup(
                &[
                    (EstimatorSource::External, "external"),
                    (EstimatorSource::Ekf, "ekf"),
                ],
                "estimator",
                "sim.estimator",
                &n,
            )?;
        }
        if let Some(l) = s.links {
            links.gcs = l.gcs.unwrap_or(links.gcs);
            links.brain = l.brain.unwrap_or(links.brain);
            links.rc_failsafe_delay_us =
                l.rc_failsafe_delay_us.unwrap_or(links.rc_failsafe_delay_us);
        }
    }
    Ok((sensors, estimator, links))
}

fn vec3(key: &str, v: [f64; 3]) -> Result<Vec3<f64>> {
    for x in v {
        finite(key, x)?;
    }
    Ok(Vec3::new(v[0], v[1], v[2]))
}

fn convert_wind(raw: RawWind) -> Result<WindSpec> {
    Ok(match raw {
        RawWind::Calm {} => WindSpec::Calm,
        RawWind::Fixed {
            steady_ned,
            turbulence_sigma,
            turbulence_tau_s,
            gust,
        } => {
            let mut gusts = Vec::new();
            for (i, g) in gust.unwrap_or_default().iter().enumerate() {
                let key = format!("wind.gust[{i}]");
                gusts.push(Gust {
                    start_s: time(&format!("{key}.start_s"), g.start_s)?,
                    duration_s: finite(&format!("{key}.duration_s"), g.duration_s)?,
                    peak_ned: vec3(&format!("{key}.peak_ned"), g.peak_ned)?,
                });
            }
            WindSpec::Fixed(WindConfig {
                steady_ned: vec3("wind.steady_ned", steady_ned.unwrap_or([0.0; 3]))?,
                turbulence_sigma: vec3(
                    "wind.turbulence_sigma",
                    turbulence_sigma.unwrap_or([0.0; 3]),
                )?,
                turbulence_tau_s: finite("wind.turbulence_tau_s", turbulence_tau_s.unwrap_or(0.0))?,
                gusts,
            })
        }
        RawWind::Random {
            steady_speed_mps,
            sigma_h_mps,
            sigma_v_ratio,
            tau_s,
            gusts,
        } => WindSpec::Random(RandomWind {
            steady_speed_mps: range("wind.steady_speed_mps", steady_speed_mps)?,
            sigma_h_mps: range("wind.sigma_h_mps", sigma_h_mps)?,
            sigma_v_ratio: finite("wind.sigma_v_ratio", sigma_v_ratio.unwrap_or(0.5))?,
            tau_s: finite("wind.tau_s", tau_s)?,
            gusts: gusts
                .map(|g| -> Result<RandomGusts> {
                    Ok(RandomGusts {
                        count: g.count,
                        start_s: range("wind.gusts.start_s", g.start_s)?,
                        duration_s: range("wind.gusts.duration_s", g.duration_s)?,
                        peak_h_mps: range("wind.gusts.peak_h_mps", g.peak_h_mps)?,
                        peak_v_mps: range("wind.gusts.peak_v_mps", g.peak_v_mps)?,
                    })
                })
                .transpose()?,
        }),
    })
}

/// Every override must be a real parameter and an acceptable value: tried in
/// order on a scratch configuration of the reference airframe.
fn check_params(params: &[(String, f64)]) -> Result<()> {
    if params.is_empty() {
        return Ok(());
    }
    let key = |name: &str| format!("params.{name}");
    let airframe =
        Airframe::reference_quad().map_err(|e| ScenarioError::new("airframe", e.to_string()))?;
    let mut cfg = airframe
        .core_config::<ROTORS>()
        .map_err(|e| ScenarioError::new("airframe", e.to_string()))?;
    for (name, value) in &airframe.params {
        apply_param(&mut cfg, &F405_LEAN, name, *value)
            .map_err(|e| ScenarioError::new("airframe", e.to_string()))?;
    }
    for (name, value) in params {
        apply_param(&mut cfg, &F405_LEAN, name, *value).map_err(|e| match e {
            SimError::Param { reason, .. } => ScenarioError::new(key(name), reason),
            other => ScenarioError::new(key(name), other.to_string()),
        })?;
    }
    Ok(())
}

/// Fault kinds by file name.
pub const FAULT_KIND_NAMES: [&str; 10] = [
    "motor-loss",
    "gps-loss",
    "imu-loss",
    "baro-loss",
    "mag-loss",
    "rc-loss",
    "rc-silent",
    "gcs-loss",
    "brain-crash",
    "brain-hang",
];

fn convert_faults(raw: &[RawFault], links: &LinkConfig) -> Result<Vec<Fault>> {
    let mut out = Vec::with_capacity(raw.len());
    for (i, f) in raw.iter().enumerate() {
        let key = format!("fault[{i}]");
        let kind = match f.kind.as_str() {
            "motor-loss" => {
                let rotor = f.rotor.ok_or_else(|| {
                    ScenarioError::new(format!("{key}.rotor"), "a motor-loss needs a rotor")
                })?;
                let scale = f.scale.ok_or_else(|| {
                    ScenarioError::new(format!("{key}.scale"), "a motor-loss needs a scale")
                })?;
                FaultKind::MotorLoss { rotor, scale }
            }
            other => {
                if f.rotor.is_some() {
                    return Err(ScenarioError::new(
                        format!("{key}.rotor"),
                        "only a motor-loss has a rotor",
                    ));
                }
                if f.scale.is_some() {
                    return Err(ScenarioError::new(
                        format!("{key}.scale"),
                        "only a motor-loss has a scale",
                    ));
                }
                match other {
                    "gps-loss" => FaultKind::GpsLoss,
                    "imu-loss" => FaultKind::ImuLoss,
                    "baro-loss" => FaultKind::BaroLoss,
                    "mag-loss" => FaultKind::MagLoss,
                    "rc-loss" => FaultKind::RcLoss,
                    "rc-silent" => FaultKind::RcSilent,
                    "gcs-loss" => FaultKind::GcsLoss,
                    "brain-crash" => FaultKind::BrainCrash,
                    "brain-hang" => FaultKind::BrainHang,
                    _ => {
                        return Err(ScenarioError::new(
                            format!("{key}.kind"),
                            format!(
                                "unknown fault kind `{other}` (one of: {})",
                                FAULT_KIND_NAMES.join(", ")
                            ),
                        ));
                    }
                }
            }
        };
        out.push(Fault {
            kind,
            start_s: time(&format!("{key}.start_s"), f.start_s)?,
            end_s: f
                .end_s
                .map(|e| time(&format!("{key}.end_s"), e))
                .transpose()?,
        });
    }
    validate_faults(&out, ROTORS, links).map_err(|e| match e {
        SimError::Fault { index, reason } => ScenarioError::new(format!("fault[{index}]"), reason),
        other => ScenarioError::new("fault", other.to_string()),
    })?;
    Ok(out)
}

fn convert_pilot(raw: Option<RawPilot>, duration_s: f64) -> Result<Script> {
    let mut script = Script {
        duration_s,
        ..Script::default()
    };
    let Some(p) = raw else {
        return Ok(script);
    };
    if let Some(t) = p.arm_at_s {
        script.arm_at_s = time("pilot.arm_at_s", t)?;
    }
    script.disarm_at_s = p
        .disarm_at_s
        .map(|t| time("pilot.disarm_at_s", t))
        .transpose()?;
    script.kill_at_s = p
        .kill_at_s
        .map(|t| time("pilot.kill_at_s", t))
        .transpose()?;

    sorted("pilot.mode", p.mode.iter().map(|m| m.t))?;
    if !p.mode.is_empty() {
        script.mode = p
            .mode
            .iter()
            .enumerate()
            .map(|(i, m)| {
                lookup(
                    &MODE_NAMES,
                    "mode",
                    &format!("pilot.mode[{i}].switch"),
                    &m.switch,
                )
                .map(|v| (m.t, v))
            })
            .collect::<Result<_>>()?;
    }

    sorted("pilot.sticks", p.sticks.iter().map(|s| s.t))?;
    for (i, s) in p.sticks.iter().enumerate() {
        let get = |name: &str, v: Option<f32>, lo: f32| -> Result<f32> {
            let v = v.unwrap_or(0.0);
            if v.is_finite() && (lo..=1.0).contains(&v) {
                Ok(v)
            } else {
                Err(ScenarioError::new(
                    format!("pilot.sticks[{i}].{name}"),
                    format!("must be between {lo} and 1"),
                ))
            }
        };
        script.sticks.push((
            s.t,
            [
                get("roll", s.roll, -1.0)?,
                get("pitch", s.pitch, -1.0)?,
                get("yaw", s.yaw, -1.0)?,
                get("throttle", s.throttle, 0.0)?,
            ],
        ));
    }

    sorted("pilot.setpoint", p.setpoint.iter().map(|s| s.t))?;
    for (i, s) in p.setpoint.iter().enumerate() {
        let key = format!("pilot.setpoint[{i}]");
        let sp = if s.silent == Some(true) {
            if s.position.is_some() || s.yaw.is_some() {
                return Err(ScenarioError::new(
                    format!("{key}.silent"),
                    "a silent entry (the brain sends nothing) has no position or yaw",
                ));
            }
            None
        } else {
            let pos = s.position.ok_or_else(|| {
                ScenarioError::new(
                    format!("{key}.position"),
                    "needs a position [north, east, up] (or `silent = true`)",
                )
            })?;
            let yaw = s.yaw.unwrap_or(0.0);
            if !(pos.iter().all(|v| v.is_finite()) && yaw.is_finite()) {
                return Err(ScenarioError::new(
                    format!("{key}.position"),
                    "position and yaw must be finite",
                ));
            }
            Some(TrajectorySetpoint {
                position: Vec3::new(pos[0], pos[1], -pos[2]),
                velocity: Vec3::zero(),
                yaw,
            })
        };
        script.setpoints.push((s.t, sp));
    }
    Ok(script)
}

fn names<T: Copy>(
    key: &str,
    list: Option<Vec<String>>,
    table: &[(T, &str)],
    what: &str,
) -> Result<Vec<T>> {
    list.unwrap_or_default()
        .iter()
        .enumerate()
        .map(|(i, n)| lookup(table, what, &format!("{key}[{i}]"), n))
        .collect()
}

fn convert_expect(raw: RawExpect) -> Result<(Option<Expect>, Option<HoldCriteria>)> {
    let has_expect = raw.nav_contains.is_some()
        || raw.nav_never.is_some()
        || raw.reasons.is_some()
        || raw.reasons_never.is_some()
        || raw.events.is_some()
        || raw.min_action.is_some()
        || raw.ending.is_some()
        || raw.by_s.is_some()
        || raw.max_touchdown_mps.is_some()
        || raw.max_tilt_deg.is_some()
        || raw.min_height_m.is_some()
        || raw.max_height_m.is_some()
        || raw.max_distance_from_home_m.is_some();
    let hold = raw.hold.map(convert_hold).transpose()?;
    if !has_expect {
        return Ok((None, hold));
    }
    let d = Expect::default();
    let opt = |key: &str, v: Option<f64>| v.map(|v| finite(key, v)).transpose();
    let ending = match raw.ending {
        None => d.ending,
        Some(RawEnding::Flying { point, tolerance_m }) => {
            let p = vec3("expect.ending.point", point)?;
            Ending::Flying {
                point: Vec3::new(p.x, p.y, -p.z),
                tolerance_m: finite("expect.ending.tolerance_m", tolerance_m)?,
            }
        }
        Some(RawEnding::FlyingAtHeight {
            height_m,
            tolerance_m,
        }) => Ending::FlyingAtHeight {
            height_m: finite("expect.ending.height_m", height_m)?,
            tolerance_m: finite("expect.ending.tolerance_m", tolerance_m)?,
        },
        Some(RawEnding::Landed { within_m_of_home }) => Ending::Landed {
            within_m_of_home: opt("expect.ending.within_m_of_home", within_m_of_home)?,
        },
        Some(RawEnding::Terminated {}) => Ending::Terminated,
        Some(RawEnding::Disarmed { reason }) => Ending::Disarmed(lookup(
            &DISARM_REASON_NAMES,
            "disarm reason",
            "expect.ending.reason",
            &reason,
        )?),
        Some(RawEnding::Down {}) => Ending::Down,
    };
    let expect = Expect {
        nav_contains: names(
            "expect.nav_contains",
            raw.nav_contains,
            &NAV_STATE_NAMES,
            "navigation state",
        )?,
        nav_never: names(
            "expect.nav_never",
            raw.nav_never,
            &NAV_STATE_NAMES,
            "navigation state",
        )?,
        reasons: lookup_reasons("expect.reasons", &raw.reasons.unwrap_or_default())?,
        reasons_never: lookup_reasons(
            "expect.reasons_never",
            &raw.reasons_never.unwrap_or_default(),
        )?,
        events: names("expect.events", raw.events, &EVENT_NAMES, "event")?,
        min_action: match raw.min_action {
            Some(n) => lookup(&ACTION_NAMES, "action", "expect.min_action", &n)?,
            None => d.min_action,
        },
        ending,
        by_s: finite("expect.by_s", raw.by_s.unwrap_or(d.by_s))?,
        max_touchdown_mps: finite(
            "expect.max_touchdown_mps",
            raw.max_touchdown_mps.unwrap_or(d.max_touchdown_mps),
        )?,
        max_tilt_deg: finite(
            "expect.max_tilt_deg",
            raw.max_tilt_deg.unwrap_or(d.max_tilt_deg),
        )?,
        min_height_m: opt("expect.min_height_m", raw.min_height_m)?,
        max_height_m: opt("expect.max_height_m", raw.max_height_m)?,
        max_distance_from_home_m: opt(
            "expect.max_distance_from_home_m",
            raw.max_distance_from_home_m,
        )?,
    };
    Ok((Some(expect), hold))
}

fn convert_hold(raw: RawHold) -> Result<HoldCriteria> {
    let d = HoldCriteria::default();
    let f = |name: &str, v: Option<f64>, default: f64| {
        finite(&format!("expect.hold.{name}"), v.unwrap_or(default))
    };
    Ok(HoldCriteria {
        window_start_s: f("window_start_s", raw.window_start_s, d.window_start_s)?,
        armed_from_s: f("armed_from_s", raw.armed_from_s, d.armed_from_s)?,
        max_h_error_m: f("max_h_error_m", raw.max_h_error_m, d.max_h_error_m)?,
        rms_h_error_m: f("rms_h_error_m", raw.rms_h_error_m, d.rms_h_error_m)?,
        max_v_error_m: f("max_v_error_m", raw.max_v_error_m, d.max_v_error_m)?,
        max_tilt_deg: f("max_tilt_deg", raw.max_tilt_deg, d.max_tilt_deg)?,
        est_max_h_error_m: f(
            "est_max_h_error_m",
            raw.est_max_h_error_m,
            d.est_max_h_error_m,
        )?,
        est_max_v_error_m: f(
            "est_max_v_error_m",
            raw.est_max_v_error_m,
            d.est_max_v_error_m,
        )?,
        est_max_vel_error_mps: f(
            "est_max_vel_error_mps",
            raw.est_max_vel_error_mps,
            d.est_max_vel_error_mps,
        )?,
        est_max_att_error_deg: f(
            "est_max_att_error_deg",
            raw.est_max_att_error_deg,
            d.est_max_att_error_deg,
        )?,
    })
}

fn convert_run(raw: Option<RawRun>) -> Result<RunSpec> {
    let mut run = RunSpec::default();
    let Some(r) = raw else {
        return Ok(run);
    };
    if let Some(n) = r.seeds {
        if n == 0 {
            return Err(ScenarioError::new("run.seeds", "must be at least 1"));
        }
        run.seeds = n;
    }
    if let Some(list) = r.profiles {
        if list.is_empty() {
            return Err(ScenarioError::new(
                "run.profiles",
                "must name at least one profile",
            ));
        }
        let table = [(RunProfile::F405, "f405"), (RunProfile::H743, "h743")];
        let mut out: Vec<RunProfile> = Vec::new();
        for (i, n) in list.iter().enumerate() {
            let p = lookup(&table, "profile", &format!("run.profiles[{i}]"), n)?;
            if out.contains(&p) {
                return Err(ScenarioError::new(
                    format!("run.profiles[{i}]"),
                    format!("profile `{n}` is listed twice"),
                ));
            }
            out.push(p);
        }
        run.profiles = out;
    }
    Ok(run)
}
