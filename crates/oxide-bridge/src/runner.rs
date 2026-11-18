// SPDX-License-Identifier: AGPL-3.0-or-later
//! One flight from a scenario file: through the bridge ([`fly_scenario`]) or
//! on our own core ([`fly_core`]), judged by the same [`Checker`] and classed
//! by the same code, so that one table serves both sides of the benchmark.

use core::fmt;
use std::path::PathBuf;
use std::time::Instant;

use oxide_core::config::{EstimatorSource, FailsafeParams};
use oxide_core::math::Real;
use oxide_sim::outcome::FlightClass;
use oxide_sim::scenario_file::{Checker, RunProfile, ScenarioFile, Verdict};
use oxide_sim::{Airframe, SimError, StepRecord};

use crate::arducopter::{ArduCopterConfig, FlightAudit, FlyError, fly};
use crate::faults::{HOVER_POWER_W, battery_failsafe_params};
use crate::parm;
use crate::scenario::{LoadedScenario, Translation, Untranslatable, translate};

/// Who flies.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Autopilot {
    /// The simulator program, through the bridge.
    ArduCopter,
    /// Our flight core, on a profile.
    Core(RunProfile),
    /// Our F405 core on the true state: the same file and index as
    /// `core-f405` with the estimator source set to the true state. Reported
    /// beside the gate and deciding nothing (ADR-0007 section 5).
    CoreTruth,
}

impl Autopilot {
    /// The name in tables and on the command line.
    pub fn name(&self) -> String {
        match self {
            Self::ArduCopter => "arducopter".into(),
            Self::Core(p) => format!("core-{}", p.name()),
            Self::CoreTruth => "core-f405-truth".into(),
        }
    }

    /// `arducopter`, `core-f405`, `core-h743` or `core-f405-truth`.
    pub fn parse(text: &str) -> Option<Self> {
        match text {
            "arducopter" => Some(Self::ArduCopter),
            "core-f405" => Some(Self::Core(RunProfile::F405)),
            "core-h743" => Some(Self::Core(RunProfile::H743)),
            "core-f405-truth" => Some(Self::CoreTruth),
            _ => None,
        }
    }
}

/// Where and how a bridged flight runs.
#[derive(Clone, Debug)]
pub struct Setup {
    /// The simulator program.
    pub binary: PathBuf,
    /// The flight's directory: the record goes here and the program's own
    /// directory (`sitl`) is made inside it, so it must not hold one.
    pub out: PathBuf,
    /// The program's instance number, 0 to 9.
    pub instance: u8,
    /// The program's own speed-up.
    pub speedup: u32,
    /// Parameter files read after the committed ones, in order.
    pub param_files: Vec<PathBuf>,
}

impl Setup {
    pub fn new(binary: impl Into<PathBuf>, out: impl Into<PathBuf>, instance: u8) -> Self {
        Self {
            binary: binary.into(),
            out: out.into(),
            instance,
            speedup: 100,
            param_files: Vec::new(),
        }
    }
}

/// Why a flight did not produce a verdict.
#[derive(Debug)]
pub enum RunError {
    /// The scenario cannot be flown through the bridge.
    NotTranslatable(Untranslatable),
    /// The simulation could not be built (our core's route).
    Sim(SimError),
    /// The bridged flight did not start or failed.
    Fly(FlyError),
}

impl RunError {
    /// What the failed bridged flight left of its audit.
    pub fn audit(&self) -> Option<&FlightAudit> {
        match self {
            Self::Fly(FlyError::Flight { audit, .. }) => audit.as_deref(),
            _ => None,
        }
    }
}

impl fmt::Display for RunError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NotTranslatable(e) => write!(f, "{e}"),
            Self::Sim(e) => write!(f, "cannot set up the simulation: {e}"),
            Self::Fly(e @ FlyError::Flight { .. }) => write!(f, "the flight failed: {e}"),
            Self::Fly(e) => write!(f, "the flight did not start: {e}"),
        }
    }
}

impl std::error::Error for RunError {}

/// One flight, judged.
#[derive(Clone, Debug)]
pub struct ScenarioFlight {
    pub scenario: String,
    pub index: u64,
    pub seed: u64,
    pub autopilot: Autopilot,
    pub verdict: Verdict,
    /// From the physics alone ([`FlightOutcome::class`](oxide_sim::outcome::FlightOutcome::class)).
    pub class: FlightClass,
    /// The RMS vertical distance from the hold point over the hold window,
    /// m; with `[expect.hold]` only.
    pub rms_v_m: Option<f64>,
    /// `Simulation::trace_hash` at the end.
    pub trace_hash: u64,
    /// The hashes of the exchange; a bridged flight only.
    pub packet_hash: Option<u64>,
    pub reply_hash: Option<u64>,
    /// The bridge's audit of the flight; a bridged flight only.
    pub audit: Option<FlightAudit>,
    /// Wall time of the whole flight, pre-roll included, s.
    pub wall_s: f64,
    /// Flight time of the last record with a motor command above zero, s.
    pub last_motor_on_s: Option<f64>,
    /// The scenario's parameters that are ours only (not sent).
    pub ours_only: Vec<(String, f64)>,
    /// The parameter files the program started with, as paths.
    pub param_files: Vec<String>,
    /// The parameters given after the files.
    pub extra_params: Vec<(String, f64)>,
}

impl ScenarioFlight {
    /// The row's note: the first problem of the verdict, then what is ours
    /// only.
    pub fn note(&self) -> String {
        let mut parts = Vec::new();
        if let Some(p) = self.verdict.first_problem() {
            parts.push(format!("first problem: {p}"));
        }
        if !self.ours_only.is_empty() {
            let list: Vec<String> = self
                .ours_only
                .iter()
                .map(|(n, v)| format!("{n}={v}"))
                .collect();
            parts.push(format!("ours only: {}", list.join(" ")));
        }
        parts.join("; ")
    }

    /// The three hashes that must repeat, as a key.
    pub fn hashes(&self) -> (u64, Option<u64>, Option<u64>) {
        (self.trace_hash, self.packet_hash, self.reply_hash)
    }
}

/// Collects what the step records say beyond the verdict: the vertical hold
/// error and the last motor command.
struct Recorder {
    checker: Checker,
    /// Hold point z (NED), the window's start and the running sum.
    vertical: Option<(f64, f64, f64, u64)>,
    last_motor_on_s: Option<f64>,
}

impl Recorder {
    /// `bridged`: the flight is another autopilot's, and the hold's
    /// armed-from limit, which is our core's own start-up, is left out
    /// (listed as not judged: ADR-0007 section 5).
    fn new(file: &ScenarioFile, bridged: bool) -> Self {
        let target_z = file
            .pilot
            .setpoints
            .iter()
            .rev()
            .find_map(|(_, sp)| sp.map(|sp| f64::from(sp.position.z)));
        let vertical = match (file.hold, target_z) {
            (Some(c), Some(z)) => Some((z, c.window_start_s, 0.0, 0)),
            _ => None,
        };
        let checker = Checker::new(file);
        Self {
            checker: if bridged {
                checker.without_armed_limit()
            } else {
                checker
            },
            vertical,
            last_motor_on_s: None,
        }
    }

    fn add(&mut self, r: &StepRecord<4>) {
        self.checker.add(r);
        if r.motors.iter().any(|m| *m > 0.0) {
            self.last_motor_on_s = Some(r.t.as_secs_f64());
        }
        match &mut self.vertical {
            Some((z, from, sum, n)) if r.t.as_secs_f64() >= *from => {
                let dz = r.position.z - *z;
                *sum += dz * dz;
                *n += 1;
            }
            _ => {}
        }
    }

    fn rms_v_m(&self) -> Option<f64> {
        self.vertical
            .filter(|(_, _, _, n)| *n > 0)
            .map(|(_, _, sum, n)| Real::sqrt(sum / n as f64))
    }
}

/// What every bridged flight is given, whatever the scenario: the committed
/// files (`base.parm`, `setup.parm`, `tuned-gains.parm`, `failsafes.parm`, in
/// this order; the starting gains are the tuning flights' own) and our core's
/// battery thresholds as ArduCopter's own ([`battery_failsafe_params`]).
pub fn committed_params(airframe: &Airframe) -> (Vec<PathBuf>, Vec<(String, f64)>) {
    let files = vec![
        parm::base_path(),
        parm::setup_path(),
        parm::tuned_gains_path(),
        parm::failsafes_path(),
    ];
    let cells = u8::try_from(airframe.battery.cells).unwrap_or(u8::MAX);
    let extras = battery_failsafe_params(
        &FailsafeParams::default(),
        cells,
        airframe.battery.internal_resistance_ohm,
        HOVER_POWER_W,
    );
    (files, extras)
}

/// The gate's low-battery threshold, V: the `BATT_LOW_VOLT` every bridged
/// flight is given ([`committed_params`]).
pub fn battery_low_volt() -> f64 {
    let airframe = Airframe::reference_quad().expect("the reference airframe loads");
    committed_params(&airframe)
        .1
        .iter()
        .find(|(n, _)| n == "BATT_LOW_VOLT")
        .map_or(f64::NAN, |(_, v)| *v)
}

/// The parameter files and the extra parameters of a bridged flight, in the
/// order they are applied: the committed files, then the setup's own files,
/// then the battery thresholds, then the scenario's (the fence).
pub fn arducopter_params(
    translation: &Translation,
    setup: &Setup,
    airframe: &Airframe,
) -> (Vec<PathBuf>, Vec<(String, f64)>) {
    let (mut files, mut extras) = committed_params(airframe);
    files.extend(setup.param_files.iter().cloned());
    extras.extend(translation.extra_params.iter().cloned());
    (files, extras)
}

/// The simulation options of run `index` for a bridged flight: the file's
/// own, without its `[params]`, which are our core's and never reach the other
/// autopilot as they are ([`translate`] decides what becomes of each).
fn bridged_scenario_options(
    file: &ScenarioFile,
    airframe: &Airframe,
    index: u64,
) -> oxide_sim::SimOptions {
    let mut options = file.options(airframe, index);
    options.params.clear();
    options
}

/// Flies run `index` of `src` through the bridge, judges it, and returns
/// what happened. Nothing is written: see [`crate::record`].
pub fn fly_scenario(
    src: &LoadedScenario,
    index: u64,
    setup: &Setup,
) -> Result<ScenarioFlight, RunError> {
    let file = &src.file;
    let translation = translate(file).map_err(RunError::NotTranslatable)?;
    let airframe = Airframe::reference_quad().map_err(RunError::Sim)?;
    let options = bridged_scenario_options(file, &airframe, index);
    let (param_files, extra_params) = arducopter_params(&translation, setup, &airframe);
    let mut config = ArduCopterConfig::new(&setup.binary, &setup.out, setup.instance);
    config.flight_name = Some("sitl".into());
    config.speedup = setup.speedup;
    config.param_files.clone_from(&param_files);
    config.extra_params.clone_from(&extra_params);
    config.brain = translation.brain;
    let mut rec = Recorder::new(file, true);
    let wall = Instant::now();
    let flown = fly(
        &airframe,
        translation.pilot.clone(),
        options,
        config,
        file.end(),
        |r| rec.add(r),
    )
    .map_err(RunError::Fly)?;
    let wall_s = wall.elapsed().as_secs_f64();
    let rms_v_m = rec.rms_v_m();
    let last_motor_on_s = rec.last_motor_on_s;
    let verdict = rec.checker.finish();
    Ok(ScenarioFlight {
        scenario: file.name.clone(),
        index,
        seed: file.seed(index),
        autopilot: Autopilot::ArduCopter,
        class: verdict.outcome.class(),
        verdict,
        rms_v_m,
        trace_hash: flown.trace_hash,
        packet_hash: Some(flown.audit.packet_hash),
        reply_hash: Some(flown.audit.reply_hash),
        audit: Some(flown.audit),
        wall_s,
        last_motor_on_s,
        ours_only: translation.ours_only,
        param_files: param_files
            .iter()
            .map(|p| p.display().to_string())
            .collect(),
        extra_params,
    })
}

/// Flies run `index` of `src` on our own core, on `profile`, with the file's
/// own parameters, and judges it by the same [`Checker`].
pub fn fly_core(
    src: &LoadedScenario,
    index: u64,
    profile: RunProfile,
) -> Result<ScenarioFlight, RunError> {
    fly_core_as(src, index, profile, Autopilot::Core(profile))
}

/// Flies run `index` of `src` on our F405 core with the estimator source set
/// to the true state (`core-f405-truth`): the file is otherwise as it is, so
/// the seed and the wind are those of the `core-f405` flight.
pub fn fly_core_truth(src: &LoadedScenario, index: u64) -> Result<ScenarioFlight, RunError> {
    let mut truth = src.clone();
    truth.file.estimator = EstimatorSource::External;
    fly_core_as(&truth, index, RunProfile::F405, Autopilot::CoreTruth)
}

fn fly_core_as(
    src: &LoadedScenario,
    index: u64,
    profile: RunProfile,
    autopilot: Autopilot,
) -> Result<ScenarioFlight, RunError> {
    let file = &src.file;
    let mut sim = file
        .build_with_params(profile.profile(), index, &[])
        .map_err(RunError::Sim)?;
    let mut rec = Recorder::new(file, false);
    let wall = Instant::now();
    sim.run_until(file.end(), |r| rec.add(r));
    let wall_s = wall.elapsed().as_secs_f64();
    let trace_hash = sim.trace_hash();
    let rms_v_m = rec.rms_v_m();
    let last_motor_on_s = rec.last_motor_on_s;
    let verdict = rec.checker.finish();
    Ok(ScenarioFlight {
        scenario: file.name.clone(),
        index,
        seed: file.seed(index),
        autopilot,
        class: verdict.outcome.class(),
        verdict,
        rms_v_m,
        trace_hash,
        packet_hash: None,
        reply_hash: None,
        audit: None,
        wall_s,
        last_motor_on_s,
        ours_only: Vec::new(),
        param_files: Vec::new(),
        extra_params: Vec::new(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scenario::LoadedScenario;
    use oxide_core::math::{Quat, Vec3};
    use oxide_core::time::Micros;

    fn scenario(name: &str) -> LoadedScenario {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../scenarios")
            .join(format!("{name}.toml"));
        LoadedScenario::load(path).expect("the scenario file")
    }

    /// A record of another autopilot's tick (truth and the armed flag only).
    fn record(t_s: f64, z: f64, motor: f32, armed: bool) -> StepRecord<4> {
        StepRecord {
            t: Micros((t_s * 1e6).round() as u64),
            position: Vec3::new(0.0, 0.0, z),
            velocity: Vec3::zero(),
            attitude: Quat::identity(),
            rate: Vec3::zero(),
            motors: [motor, 0.0, 0.0, 0.0],
            rotor_speed: [0.0; 4],
            battery_v: 22.0,
            battery_a: 0.0,
            armed,
            setpoint: None,
            wind: Vec3::zero(),
            physics_faults: 0,
            core: None,
        }
    }

    #[test]
    fn our_cores_parameters_are_not_in_the_options_of_a_bridged_flight() {
        let af = Airframe::reference_quad().expect("airframe");
        let src = scenario("geofence");
        let ours = src.file.options(&af, 0);
        assert!(
            ours.params.iter().any(|(n, _)| n == "FS_FENCE_RAD"),
            "the file sets our core's fence radius: {:?}",
            ours.params
        );
        let bridged = bridged_scenario_options(&src.file, &af, 0);
        assert!(bridged.params.is_empty(), "{:?}", bridged.params);
        // Nothing else of the options is touched.
        assert_eq!(bridged.seed, ours.seed);
    }

    #[test]
    fn the_faults_of_a_bridged_flight_keep_the_files_times_only_the_pilot_is_delayed() {
        let af = Airframe::reference_quad().expect("airframe");
        let src = scenario("rc-loss-override");
        let ours = src.file.options(&af, 0);
        assert!(!ours.faults.is_empty(), "the file has a radio fault");
        let bridged = bridged_scenario_options(&src.file, &af, 0);
        assert_eq!(bridged.faults, ours.faults);
        let t = translate(&src.file).expect("translatable");
        assert_ne!(t.pilot, src.file.pilot, "the pilot's script is delayed");
        assert_eq!(t.pilot.arm_at_s, src.file.pilot.arm_at_s);
    }

    #[test]
    fn the_last_motor_on_is_the_time_of_the_last_record_with_a_command_above_zero() {
        let src = scenario("hover-step");
        let mut rec = Recorder::new(&src.file, true);
        assert_eq!(rec.last_motor_on_s, None);
        // Zero commands are not "on": a command of exactly zero is off.
        for k in 0..10 {
            rec.add(&record(f64::from(k) * 0.5, 0.0, 0.0, false));
        }
        assert_eq!(rec.last_motor_on_s, None, "motors at exactly zero");
        rec.add(&record(5.0, -1.0, 0.25, true));
        assert_eq!(rec.last_motor_on_s, Some(5.0));
        rec.add(&record(5.5, -1.0, 0.0, true));
        rec.add(&record(6.0, -1.0, 0.0, true));
        assert_eq!(rec.last_motor_on_s, Some(5.0), "later zeros do not move it");
        rec.add(&record(7.0, -1.0, 1e-9, true));
        assert_eq!(rec.last_motor_on_s, Some(7.0));
        // Only the first of the four motors was above zero in these; any one
        // counts.
        let mut r = record(8.0, -1.0, 0.0, true);
        r.motors = [0.0, 0.0, 0.0, 0.1];
        rec.add(&r);
        assert_eq!(rec.last_motor_on_s, Some(8.0));
    }

    #[test]
    fn the_armed_from_limit_is_ours_only_a_bridged_flight_is_not_judged_on_it() {
        let src = scenario("gust-hold");
        let verdict = |bridged: bool| {
            let mut rec = Recorder::new(&src.file, bridged);
            // Armed only from 5.2 s, as ArduCopter does.
            for k in 0..400 {
                let t = f64::from(k) * 0.05;
                rec.add(&record(t, -2.0, 0.5, t >= 5.2));
            }
            rec.checker.finish()
        };
        let ours = verdict(false);
        assert!(
            ours.violations
                .iter()
                .any(|v| v.contains("not armed after")),
            "{:?}",
            ours.violations
        );
        let bridged = verdict(true);
        assert!(
            !bridged.violations.iter().any(|v| v.contains("not armed")),
            "{:?}",
            bridged.violations
        );
        assert!(
            bridged
                .not_judged
                .iter()
                .any(|n| n.key == "hold: armed from armed_from_s"),
            "{:?}",
            bridged.not_judged
        );
    }

    #[test]
    fn the_vertical_error_is_the_rms_from_the_window_start_inclusive() {
        let src = scenario("gust-hold");
        let from = src.file.hold.expect("a hold").window_start_s;
        assert_eq!(from, 15.0);
        // The hold point's z is the file's last setpoint.
        let target = src
            .file
            .pilot
            .setpoints
            .iter()
            .rev()
            .find_map(|(_, sp)| sp.map(|sp| f64::from(sp.position.z)))
            .expect("a setpoint");
        let mut rec = Recorder::new(&src.file, false);
        assert_eq!(rec.rms_v_m(), None, "no tick in the window yet");
        // Before the window: far off, must not count.
        rec.add(&record(from - 0.001, target + 10.0, 0.5, true));
        assert_eq!(rec.rms_v_m(), None);
        // At the start (inclusive), then later: errors 1, 1 and 3.
        rec.add(&record(from, target + 1.0, 0.5, true));
        rec.add(&record(from + 1.0, target - 1.0, 0.5, true));
        rec.add(&record(from + 6.0, target + 3.0, 0.5, true));
        let want = Real::sqrt((1.0 + 1.0 + 9.0) / 3.0);
        let got = rec.rms_v_m().expect("a value");
        assert!((got - want).abs() < 1e-9, "{got} against {want}");
    }

    #[test]
    fn the_parameters_of_a_bridged_flight_are_the_committed_files_the_battery_then_the_fence() {
        let af = Airframe::reference_quad().expect("airframe");
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../scenarios/geofence.toml");
        let src = LoadedScenario::load(path).expect("geofence");
        let t = translate(&src.file).expect("translatable");
        let mut setup = Setup::new("/x", "/y", 0);
        setup.param_files.push(PathBuf::from("mine.parm"));
        let (files, extras) = arducopter_params(&t, &setup, &af);
        let names: Vec<String> = files
            .iter()
            .map(|p| p.file_name().expect("name").to_string_lossy().into_owned())
            .collect();
        assert_eq!(
            names,
            [
                "base.parm",
                "setup.parm",
                "tuned-gains.parm",
                "failsafes.parm",
                "mine.parm"
            ]
        );
        let extra_names: Vec<&str> = extras.iter().map(|(n, _)| n.as_str()).collect();
        assert_eq!(
            extra_names,
            [
                "BATT_LOW_VOLT",
                "BATT_CRT_VOLT",
                "FENCE_ENABLE",
                "FENCE_TYPE",
                "FENCE_RADIUS",
                "FENCE_ACTION"
            ]
        );
        let value = |n: &str| extras.iter().find(|(m, _)| m == n).expect("present").1;
        assert!((value("BATT_LOW_VOLT") - 20.767).abs() < 0.001);
        assert!((value("BATT_CRT_VOLT") - 19.553).abs() < 0.001);
    }
}
