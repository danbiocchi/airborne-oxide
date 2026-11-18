// SPDX-License-Identifier: AGPL-3.0-or-later
//! ArduCopter flying our quad (ADR-0006): the autopilot that starts the
//! simulator program, runs the pre-roll, and flies offboard.
//!
//! [`ArduCopter`] implements `oxide_sim::Autopilot<4>` with a period of
//! 500 µs. One tick is one exchange ([`crate::bridge::Exchange`]); around it:
//!
//! - **The pre-roll** ([`ArduCopter::preroll_tick`](oxide_sim::Autopilot::preroll_tick)):
//!   the vehicle at rest, our IMU model's samples, no GPS error, the motor
//!   pulses never above 1000 µs.
//! - **The flight tick**: the truth, our IMU model's sample (latched between
//!   its samples), the battery, the radio, and the GPS error added to the
//!   north and east position of the reply.
//! - **Offboard**: our setpoint stream becomes GUIDED commands on the brain
//!   link (take-off, then position targets); the brain's heartbeat goes out
//!   at 1 Hz.
//! - **Faults**: the faults active at the tick go through
//!   [`FaultTranslator`], and each parameter write it returns is sent on the
//!   instrument's link before the tick's exchange and recorded in the audit
//!   ([`SentKind::ParamSet`]); the echo is recorded when it is read. The
//!   faults that need no write need no code: the pilot's radio, the brain's
//!   and the ground station's links are forwarded only while they arrive.
//! - **The instrument**: a client on serial 0 that sends a heartbeat at 1 Hz
//!   as a system id that is not the ground station's, requests a fixed set of
//!   telemetry messages at fixed steps, and reads. It is the measuring
//!   instrument, not a ground station.
//!
//! # Time and repeatability
//!
//! Everything this autopilot sends to the program, or to the physics, is
//! decided from the simulated time and from lockstep data only: the truth,
//! the setpoints, and the motor pulses of the previous exchange. Everything
//! read from MAVLink goes into the [`FlightAudit`] and nowhere else. A
//! command goes out *before* the reply of its exchange, so that it is in the
//! program's socket when the program reads the next frame.
//!
//! The repeat tests of `tests/arducopter_fly.rs` do not prove the rule: the
//! program writes its TCP answers before its next motor packet, so most data
//! read from MAVLink arrives in time to repeat anyway (a decision taken from
//! the vehicle's heartbeat instead of the pulses repeats bit for bit). The rule
//! is held by the lib test
//! `no_decision_path_reads_what_was_read_from_mavlink`, which reads this file.
//!
//! The simulated time of an event in the audit is `exchange number × step`,
//! counted from the first exchange of the pre-roll ("program time"). The
//! flight starts at `preroll_steps × step`.

use core::fmt;
use std::io;
use std::path::PathBuf;
use std::time::Duration;

use oxide_core::io::{RcSample, TrajectorySetpoint};
use oxide_core::math::Real;
use oxide_core::time::Micros;
use oxide_sim::autopilot::{Autopilot, AutopilotInput, AutopilotOutput, PrerollInput};
use oxide_sim::faults::ActiveFaults;
use oxide_sim::rng::{Rng, Stream};
use oxide_sim::scenario::Pilot;
use oxide_sim::sim::{PrerollError, SimOptions, StepRecord};
use oxide_sim::{Airframe, SimError, Simulation};

use crate::bridge::{
    BridgeConfig, BridgeError, ConfigError, Exchange, RcLatch, armed_by_pulses, reply_of_body,
};
use crate::faults::FaultTranslator;
use crate::link::Link;
use crate::mav::{self, MavClient, MavError, MavEvent};
use crate::motors::{DEFAULT_SPIN_MAX, DEFAULT_SPIN_MIN, MotorMap};
use crate::parm::{self, ParmError};
use crate::reply::Reply;
use crate::scenario::MODE_OFFBOARD;
use crate::sitl::{Sitl, SitlConfig, SitlError, json_port_of};

/// The step of a bridged flight, µs.
pub const STEP_US: u64 = 500;
/// The steps of the pre-roll: 45 s. ArduCopter is ready to arm in a mode that
/// uses GPS after 41.2 s at rest.
pub const PREROLL_STEPS: u64 = 90_000;
/// Steps in one second.
const SECOND_STEPS: u64 = 1_000_000 / STEP_US;
/// The instrument's system id: not 255, the ground station's.
pub const INSTRUMENT_SYSTEM_ID: u8 = 250;
/// The brain's system id (a companion computer).
pub const BRAIN_SYSTEM_ID: u8 = 251;
/// The ground station's system id.
pub const GCS_SYSTEM_ID: u8 = 255;
/// The exchange at which the brain's and the ground station's clients are
/// connected. The program opens its serial 1 and 2 only once it has run for a
/// few steps (measured: they accept a connection from step 30 on, and refuse
/// one before the first exchange), so the connection is made at a fixed step
/// well after that. It changes nothing the physics sees.
pub const LINKS_CONNECT_STEP: u64 = 1_000;
/// How long the arm switch may be on with no pulse above 1000 µs, steps.
const ARM_TIMEOUT_STEPS: u64 = 5 * SECOND_STEPS;
/// The setpoint height above which the vehicle is to take off, m.
const AIRBORNE_SETPOINT_M: f32 = 0.2;
/// The true height under which the vehicle is still on the ground, m.
const GROUND_M: f64 = 0.3;
/// How close to the take-off height the vehicle must be before position
/// targets go out, m.
const TAKEOFF_REACHED_M: f64 = 0.3;
/// A setpoint that moved by more than this in position, m, is sent at once.
const TARGET_POSITION_STEP_M: f32 = 0.001;
/// A setpoint that moved by more than this in yaw, rad, is sent at once.
const TARGET_YAW_STEP_RAD: f32 = 0.001;
/// How many arm-related status texts the error and the audit keep.
const ARM_TEXTS_KEPT: usize = 8;

/// Which MAVLink link carries what.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LinkName {
    /// Serial 0: the instrument.
    Instrument,
    /// Serial 1: the companion computer.
    Brain,
    /// Serial 2: the ground station.
    Gcs,
}

impl fmt::Display for LinkName {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::Instrument => "instrument (serial 0)",
            Self::Brain => "brain (serial 1)",
            Self::Gcs => "ground station (serial 2)",
        })
    }
}

/// Where the brain's commands go.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BrainLink {
    /// No brain: setpoints are not translated.
    None,
    /// A client of its own on serial 1 (the default).
    Serial1,
    /// The instrument's link (a switch kept for the case that serial 1 does
    /// not work). The brain's heartbeat is then not sent.
    Instrument,
}

/// A telemetry message to request, at a fixed pre-roll step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TelemetryRequest {
    /// The MAVLink message id ([`mav::id`]).
    pub message_id: u32,
    /// Messages per second.
    pub hz: f32,
    /// The program-time step (exchange number) at which the request is sent.
    pub at_step: u64,
}

/// The GPS lag of the program, µs: `SIM_GPS1_LAG_MS 150` in `base.parm` (a
/// unit test of [`crate::parm`] pins that value, and building the
/// configuration checks the files against [`GpsError::lag_us`]).
pub const GPS_LAG_US: u64 = 150_000;

/// How the bridge's held errors are turned into the error ArduCopter's GPS
/// reports (ADR-0006 section 4).
///
/// The program's GPS lag reads the position `lag` back, between two stored
/// samples one GPS period apart, so each fix it reports is
/// `a sent(j-2) + b sent(j-1)` of the errors the bridge sent, with `a = lag /
/// period` and `b = 1 - a` (measured on V4.7.2 for 150 ms and 200 ms: 0.007 m
/// rms residual). A lag beyond one period only delays that by whole periods:
/// `a` is the part of `lag / period` after the whole periods (measured at
/// 250 ms: the unblended sequence is reported with 0.79 of its standard
/// deviation and a correlation of 0.3). The bridge sends the sequence
/// `sent(k) = (n(k) - lo sent(k-1)) / hi` of independent draws `n`, `hi` the
/// larger and `lo` the smaller of `a` and `b`: the blend of that sequence is
/// `n` through a filter that changes no frequency's size, so the reported
/// error is white with the standard deviation of `n`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GpsBlend {
    /// The larger of the two weights.
    pub hi: f64,
    /// The smaller of the two weights.
    pub lo: f64,
}

impl GpsBlend {
    /// The standard deviation of `sent` over that of the reported error:
    /// `1 / sqrt(hi² - lo²)` (1.414 for 150 ms and 200 ms).
    pub fn sent_over_reported(&self) -> f64 {
        1.0 / Real::sqrt(self.hi * self.hi - self.lo * self.lo)
    }
}

/// A lag whose blend cannot be undone: its part beyond whole periods is within
/// 5 % of half the period, the two weights are nearly equal and the pre-filter's pole is on the unit circle.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct GpsLagUnusable {
    pub lag_us: u64,
    pub period_us: u64,
}

impl fmt::Display for GpsLagUnusable {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "the GPS lag of {} us is within 5 % of half the GPS period of {} us (the part of \
             the lag beyond whole periods is what counts): the two samples the program blends \
             then weigh alike and the blend cannot be undone; use another lag",
            self.lag_us, self.period_us
        )
    }
}

impl std::error::Error for GpsLagUnusable {}

/// The GPS horizontal error of ADR-0006 §4.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GpsError {
    /// Switched off when false (the flight then has none).
    pub enabled: bool,
    /// The standard deviation per axis of the error ArduCopter's GPS reports,
    /// m: our GPS model's own value ([`bridged_options`] sets it to
    /// `pos_sigma_h_m`, undivided). The errors the bridge sends have
    /// [`GpsBlend::sent_over_reported`] times that.
    pub sigma_m: f64,
    /// A new draw every period, µs of flight time.
    pub period_us: u64,
    /// The draws are at `first_fix_us + k × period`.
    pub first_fix_us: u64,
    /// The program's GPS lag, µs: `SIM_GPS1_LAG_MS` times 1000. It is checked
    /// against the parameter files when the autopilot is built. Any lag the
    /// program accepts is undone except one whose part beyond whole periods is
    /// within 5 % of half a period ([`GpsError::blend`]).
    pub lag_us: u64,
}

impl GpsError {
    /// Our realistic GPS model's values.
    pub fn ours() -> Self {
        Self {
            enabled: true,
            sigma_m: 0.3,
            period_us: 200_000,
            first_fix_us: 0,
            lag_us: GPS_LAG_US,
        }
    }

    /// The blend to undo. The program reads the position `lag` back between
    /// two stored samples, so only the part of the lag beyond whole periods
    /// sets the weights: `a = (lag mod period) / period` (measured on V4.7.2
    /// at 250 ms with a 200 ms period: the report is blended, not white).
    /// `None` for a lag of 0 or of a whole number of periods (the program
    /// then reads one stored sample, and `sent = n`); an error for a part
    /// within 5 % of half the period.
    pub fn blend(&self) -> Result<Option<GpsBlend>, GpsLagUnusable> {
        if self.lag_us == 0 || self.period_us == 0 {
            return Ok(None);
        }
        let part_us = self.lag_us % self.period_us;
        if part_us == 0 {
            return Ok(None);
        }
        let a = part_us as f64 / self.period_us as f64;
        // |part - period / 2| <= 5 % of period / 2, in whole microseconds.
        if part_us.abs_diff(self.period_us - part_us) * 20 <= self.period_us {
            return Err(GpsLagUnusable {
                lag_us: self.lag_us,
                period_us: self.period_us,
            });
        }
        let b = 1.0 - a;
        Ok(Some(GpsBlend {
            hi: a.max(b),
            lo: a.min(b),
        }))
    }
}

/// How the autopilot is set up.
#[derive(Clone, Debug)]
pub struct ArduCopterConfig {
    /// The simulator program.
    pub binary: PathBuf,
    /// The directory under which the flight's own directory is made.
    pub root: PathBuf,
    /// The flight's directory name; `None` for a new one.
    pub flight_name: Option<String>,
    /// 0 to 9: shifts every port.
    pub instance: u8,
    /// The program's own speed-up (how long it sleeps; its clock is ours).
    pub speedup: u32,
    /// Parameter files in order; the default is the two committed ones.
    pub param_files: Vec<PathBuf>,
    /// Parameters written after the files.
    pub extra_params: Vec<(String, f64)>,
    /// The run seed (the GPS error's stream; recorded in the audit).
    pub seed: u64,
    pub gps_error: GpsError,
    pub brain: BrainLink,
    /// A ground-station client on serial 2 that sends heartbeats as system 255
    /// while ours come ([`fly`] sets it from `options.links.gcs`).
    pub gcs: bool,
    /// Requested at fixed steps, in this order.
    pub telemetry: Vec<TelemetryRequest>,
    /// A test switch: the instrument's heartbeats stop at this flight time, s.
    pub instrument_heartbeats_stop_at_s: Option<f64>,
    /// The instrument's system id ([`INSTRUMENT_SYSTEM_ID`], which is not the
    /// ground station's). A test switch: with 255 the instrument *is* the
    /// ground station, which shows that the ground-station failsafe is armed
    /// when a test says it sees none.
    pub instrument_system_id: u8,
    /// How long to wait for each next packet.
    pub reply_timeout: Duration,
}

impl ArduCopterConfig {
    /// The default setup: the committed parameter files of the gains
    /// (`base.parm`, `setup.parm`, `tuned-gains.parm`; the failsafes and the
    /// battery thresholds are given by the runner), the brain on
    /// serial 1, the GPS error ours, `SYSTEM_TIME` at 10 Hz and
    /// `LOCAL_POSITION_NED` at 10 Hz requested 2 s into the pre-roll.
    pub fn new(binary: impl Into<PathBuf>, root: impl Into<PathBuf>, instance: u8) -> Self {
        Self {
            binary: binary.into(),
            root: root.into(),
            flight_name: None,
            instance,
            speedup: 100,
            param_files: vec![
                parm::base_path(),
                parm::setup_path(),
                parm::tuned_gains_path(),
            ],
            extra_params: Vec::new(),
            seed: 0,
            gps_error: GpsError::ours(),
            brain: BrainLink::Serial1,
            gcs: false,
            telemetry: vec![
                TelemetryRequest {
                    message_id: mav::id::SYSTEM_TIME,
                    hz: 10.0,
                    at_step: 4_000,
                },
                TelemetryRequest {
                    message_id: mav::id::LOCAL_POSITION_NED,
                    hz: 10.0,
                    at_step: 4_001,
                },
            ],
            instrument_heartbeats_stop_at_s: None,
            instrument_system_id: INSTRUMENT_SYSTEM_ID,
            reply_timeout: Duration::from_secs(10),
        }
    }
}

/// A simulated time, for the errors and the audit.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SimTime {
    /// Program time: exchange number × step, µs.
    pub program_us: u64,
    /// Program time of the first flight tick, µs; `None` in the pre-roll.
    pub flight_start_us: Option<u64>,
}

impl SimTime {
    /// Flight time in seconds, negative in the pre-roll; `None` before the
    /// flight's start is known.
    #[allow(clippy::cast_precision_loss)]
    pub fn flight_s(&self) -> Option<f64> {
        let start = self.flight_start_us?;
        Some((self.program_us as f64 - start as f64) * 1.0e-6)
    }
}

impl fmt::Display for SimTime {
    #[allow(clippy::cast_precision_loss)]
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "simulated time {:.4} s", self.program_us as f64 * 1.0e-6)?;
        match self.flight_s() {
            Some(s) => write!(f, " (flight time {s:.4} s)"),
            None => write!(f, " (pre-roll)"),
        }
    }
}

/// Why the autopilot failed.
#[derive(Debug)]
pub enum ArduCopterError {
    /// The configuration is not usable.
    Config(ConfigError),
    /// The instance number is above 9.
    Instance(u8),
    /// The GPS lag cannot be undone by the pre-filter.
    GpsLag(GpsLagUnusable),
    /// The lag the bridge assumes is not the one the parameter files set.
    GpsLagMismatch {
        /// `GpsError::lag_us`.
        assumed_us: u64,
        /// `SIM_GPS1_LAG_MS` of the files and extra parameters (last value
        /// wins), `None` when none sets it.
        program_ms: Option<f64>,
    },
    /// A parameter file does not parse (its values are read for the motor
    /// map).
    ParamFile { path: PathBuf, source: ParmError },
    /// The UDP port could not be bound.
    Bind { port: u16, source: io::Error },
    /// The simulator program could not be started.
    Start(SitlError),
    /// A MAVLink link could not be connected.
    Connect { link: LinkName, source: MavError },
    /// The exchange failed.
    Exchange { at: SimTime, source: BridgeError },
    /// A MAVLink link failed.
    Mav {
        at: SimTime,
        link: LinkName,
        source: MavError,
    },
    /// The simulator program has ended.
    Process { at: SimTime, source: SitlError },
    /// A motor pulse rose above 1000 µs during the pre-roll: the vehicle armed
    /// itself.
    ArmedInPreroll {
        at: SimTime,
        output: usize,
        pulse: u16,
    },
    /// The arm switch was on for 5 s and no pulse rose above 1000 µs.
    DidNotArm {
        at: SimTime,
        /// The last `PreArm` and `Arm` status texts read, with the program
        /// time of the step that read each, µs.
        texts: Vec<(u64, String)>,
    },
    /// The program could not be stopped.
    Finish(SitlError),
}

impl fmt::Display for ArduCopterError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Config(e) => write!(f, "the ArduCopter configuration is not usable: {e}"),
            Self::Instance(i) => write!(f, "instance {i} is not between 0 and 9"),
            Self::GpsLag(e) => write!(f, "the GPS error is not usable: {e}"),
            Self::GpsLagMismatch {
                assumed_us,
                program_ms: Some(ms),
            } => write!(
                f,
                "the bridge assumes a GPS lag of {assumed_us} us (GpsError::lag_us) but the \
                 parameters set SIM_GPS1_LAG_MS {ms} ({} us): the GPS error is undone for \
                 the lag it assumes, so the two must be the same",
                ms * 1000.0
            ),
            Self::GpsLagMismatch {
                assumed_us,
                program_ms: None,
            } => write!(
                f,
                "the bridge assumes a GPS lag of {assumed_us} us (GpsError::lag_us) but no \
                 parameter file sets SIM_GPS1_LAG_MS: set it to {} so that the two are the same",
                *assumed_us as f64 / 1000.0
            ),
            Self::ParamFile { path, source } => {
                write!(f, "start: {}: {source}", path.display())
            }
            Self::Bind { port, source } => {
                write!(f, "start: cannot bind UDP port {port}: {source}")?;
                if source.kind() == io::ErrorKind::AddrInUse {
                    let instance = port.saturating_sub(9002) / 10;
                    write!(
                        f,
                        " (instance number {instance} is in use by another program, perhaps \
                         another batch with an overlapping --instances range: give each its own \
                         instance numbers)"
                    )?;
                }
                Ok(())
            }
            Self::Start(e) => write!(f, "start: the simulator program did not start: {e}"),
            Self::Connect { link, source } => {
                write!(f, "start: cannot connect the {link} link: {source}")
            }
            Self::Exchange { at, source } => write!(f, "at {at}: the exchange failed: {source}"),
            Self::Mav { at, link, source } => {
                write!(f, "at {at}: the {link} link failed: {source}")
            }
            Self::Process { at, source } => write!(f, "at {at}: {source}"),
            Self::ArmedInPreroll { at, output, pulse } => write!(
                f,
                "at {at}: output {output} went to {pulse} us during the pre-roll: \
                 the vehicle armed itself"
            ),
            Self::DidNotArm { at, texts } => {
                write!(
                    f,
                    "ArduCopter did not arm: at {at} the arm switch had been on for 5 s and \
                     no motor pulse was above 1000 us; "
                )?;
                if texts.is_empty() {
                    write!(f, "no PreArm or Arm status text was read")
                } else {
                    write!(f, "the last PreArm/Arm status texts read:")?;
                    for (us, text) in texts {
                        write!(f, " [{:.3} s] {text:?}", *us as f64 * 1.0e-6)?;
                    }
                    Ok(())
                }
            }
            Self::Finish(e) => write!(f, "the simulator program could not be stopped: {e}"),
        }
    }
}

impl std::error::Error for ArduCopterError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Config(e) => Some(e),
            Self::GpsLag(e) => Some(e),
            Self::ParamFile { source, .. } => Some(source),
            Self::Bind { source, .. } => Some(source),
            Self::Start(e) | Self::Finish(e) => Some(e),
            Self::Connect { source, .. } | Self::Mav { source, .. } => Some(source),
            Self::Exchange { source, .. } => Some(source),
            Self::Process { source, .. } => Some(source),
            _ => None,
        }
    }
}

/// What was sent, by this autopilot, to the program.
#[derive(Clone, Debug, PartialEq)]
pub enum SentKind {
    Heartbeat,
    MessageInterval {
        message_id: u32,
        hz: f32,
    },
    TakeOff {
        height_m: f32,
    },
    PositionTarget {
        n: f32,
        e: f32,
        d: f32,
        yaw: f32,
    },
    /// A parameter written for a fault (`param_set` on the instrument's
    /// link): the name and the value as sent.
    ParamSet {
        name: String,
        value: f32,
    },
}

/// One thing sent, with the program time of the exchange it went before.
#[derive(Clone, Debug, PartialEq)]
pub struct Sent {
    pub at_us: u64,
    pub link: LinkName,
    /// The system id in the message header.
    pub system_id: u8,
    pub what: SentKind,
}

/// One thing read, with the program time of the step that read it. The time
/// of a read depends on the wall clock: it is not repeatable to the step.
#[derive(Clone, Debug, PartialEq)]
pub struct Heard {
    pub at_us: u64,
    pub link: LinkName,
    pub event: MavEvent,
}

/// What a bridged flight leaves behind (ADR-0006 §11). The two hashes repeat
/// exactly; the lists of things read do not repeat to the step.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct FlightAudit {
    /// SHA-256 of the program, hexadecimal.
    pub binary_sha256: String,
    /// The program's path and arguments.
    pub command_line: Vec<String>,
    /// SHA-256 of the effective parameter file.
    pub params_sha256: String,
    /// The run seed.
    pub seed: u64,
    /// The step, µs.
    pub step_us: u64,
    /// The pre-roll's steps (the flight starts at this exchange number).
    pub preroll_steps: u64,
    /// Exchanges done.
    pub exchanges: u64,
    /// Everything sent, in order, each with its program time.
    pub sent: Vec<Sent>,
    /// Status texts read: program time of the read, severity, text.
    pub status_texts: Vec<(u64, u8, String)>,
    /// Mode changes read from the vehicle's heartbeat: program time of the
    /// read, the vehicle's own mode number.
    pub mode_changes: Vec<(u64, u32)>,
    /// Armed changes read from the vehicle's heartbeat: program time of the
    /// read, armed.
    pub armed_changes: Vec<(u64, bool)>,
    /// The vehicle's heartbeats read.
    pub vehicle_heartbeats: u64,
    /// The highest pulse on outputs 1 to 4 during the pre-roll, µs.
    pub preroll_max_pulse_us: u16,
    /// Every other message read (telemetry), in order.
    pub heard: Vec<Heard>,
    /// FNV-1a over every motor packet received.
    pub packet_hash: u64,
    /// FNV-1a over every reply line sent.
    pub reply_hash: u64,
    /// The time stamp of the last reply, ns: `exchanges` steps, each one step
    /// and 1 ns (see `reply::timestamp_ns`).
    pub last_stamp_ns: u64,
    /// Replies whose time stamp was not exactly one step after the one before
    /// (the pre-roll's last to the flight's first included). Zero: the
    /// program's clock never jumped or ran back.
    pub stamp_breaks: u64,
    /// The program had ended before it was stopped, and the end of its error
    /// output (filled by `finish`).
    pub sitl_ended_before_stop: Option<bool>,
    pub sitl_stderr_tail: String,
}

impl FlightAudit {
    /// Heartbeats sent as this system id.
    pub fn heartbeats_sent_as(&self, system_id: u8) -> usize {
        self.sent
            .iter()
            .filter(|s| s.what == SentKind::Heartbeat && s.system_id == system_id)
            .count()
    }

    /// Anything sent as this system id.
    pub fn sent_as(&self, system_id: u8) -> usize {
        self.sent
            .iter()
            .filter(|s| s.system_id == system_id)
            .count()
    }

    /// The parameter writes sent: program time of the exchange each went
    /// before, name, value.
    pub fn param_writes(&self) -> Vec<(u64, &str, f32)> {
        self.sent
            .iter()
            .filter_map(|s| match &s.what {
                SentKind::ParamSet { name, value } => Some((s.at_us, name.as_str(), *value)),
                _ => None,
            })
            .collect()
    }

    /// The echoes (`PARAM_VALUE`) heard: program time of the read, name,
    /// value. A read time is not repeatable to the step.
    pub fn param_echoes(&self) -> Vec<(u64, &str, f32)> {
        self.heard
            .iter()
            .filter_map(|h| match &h.event {
                MavEvent::ParamValue { name, value, .. } => Some((h.at_us, name.as_str(), *value)),
                _ => None,
            })
            .collect()
    }

    /// Program time of a flight time, µs.
    pub fn program_us_of_flight_s(&self, flight_s: f64) -> u64 {
        self.preroll_steps * self.step_us + (flight_s * 1.0e6).round() as u64
    }

    /// The status texts that contain `needle`.
    pub fn texts_containing<'a>(
        &'a self,
        needle: &'a str,
    ) -> impl Iterator<Item = &'a (u64, u8, String)> {
        self.status_texts
            .iter()
            .filter(move |(_, _, t)| t.contains(needle))
    }

    fn time(&self, program_us: u64) -> String {
        let start = self.preroll_steps * self.step_us;
        let t = program_us as f64 * 1.0e-6;
        if program_us >= start {
            format!(
                "{t:9.4} s (flight {:8.4} s)",
                (program_us - start) as f64 * 1.0e-6
            )
        } else {
            format!("{t:9.4} s (pre-roll)")
        }
    }
}

impl fmt::Display for FlightAudit {
    #[allow(clippy::cast_precision_loss)]
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "flight audit")?;
        writeln!(f, "  binary sha256   {}", self.binary_sha256)?;
        writeln!(f, "  command line    {}", self.command_line.join(" "))?;
        writeln!(f, "  params sha256   {}", self.params_sha256)?;
        writeln!(
            f,
            "  seed {}, step {} us, {} pre-roll steps, {} exchanges",
            self.seed, self.step_us, self.preroll_steps, self.exchanges
        )?;
        writeln!(
            f,
            "  packet hash {:016x}, reply hash {:016x}",
            self.packet_hash, self.reply_hash
        )?;
        writeln!(
            f,
            "  last time stamp {} ns, {} stamp breaks",
            self.last_stamp_ns, self.stamp_breaks
        )?;
        let mut sent_other = 0;
        writeln!(f, "  sent (heartbeats not listed):")?;
        for s in &self.sent {
            if s.what == SentKind::Heartbeat {
                continue;
            }
            sent_other += 1;
            writeln!(
                f,
                "    {} {} as system {}: {:?}",
                self.time(s.at_us),
                s.link,
                s.system_id,
                s.what
            )?;
        }
        writeln!(
            f,
            "    ({} sent in all, {} heartbeats; {sent_other} others)",
            self.sent.len(),
            self.sent.len() - sent_other
        )?;
        writeln!(f, "  read (times are not repeatable to the step):")?;
        for (t, sev, text) in &self.status_texts {
            writeln!(f, "    {} status [{sev}] {text}", self.time(*t))?;
        }
        for (t, m) in &self.mode_changes {
            writeln!(f, "    {} mode {m}", self.time(*t))?;
        }
        for (t, a) in &self.armed_changes {
            writeln!(f, "    {} armed {a}", self.time(*t))?;
        }
        writeln!(
            f,
            "    {} vehicle heartbeats, {} other messages",
            self.vehicle_heartbeats,
            self.heard.len()
        )?;
        if let Some(ended) = self.sitl_ended_before_stop {
            writeln!(
                f,
                "  program ended before stop: {ended}; error output tail: {:?}",
                self.sitl_stderr_tail
            )?;
        }
        Ok(())
    }
}

/// A heartbeat is forwarded on the second, and only while the one of ours that
/// it stands for arrives (`arriving`): a lost ground station or a crashed
/// brain stops its heartbeat on the link as well.
fn heartbeat_due<T>(arriving: &Option<T>, t: Micros) -> bool {
    arriving.is_some() && t.0.is_multiple_of(1_000_000)
}

/// The GPS error's state: the held offset, redrawn every period.
///
/// Per axis, at each fix instant: `sent(k) = (n(k) - lo sent(k-1)) / hi` for
/// the independent draw `n(k)` ([`GpsBlend`]), or `sent = n` without a blend.
/// The draws come from `Stream::GpsError` in this order: first the two
/// starting values `sent(-1)` (north, east; standard deviation
/// `sigma / sqrt(hi² - lo²)`, so that the sequence is steady from its first
/// fix), then at each fix instant north, then east.
#[derive(Clone, Debug)]
pub struct GpsErrorState {
    config: GpsError,
    blend: Option<GpsBlend>,
    rng: Rng,
    /// `sent(-1)`.
    start: [f64; 2],
    /// The last `sent`, held until the next fix instant.
    offset: [f64; 2],
    /// A fix instant has passed.
    started: bool,
}

impl GpsErrorState {
    /// A lag that [`GpsError::blend`] refuses is treated as no blend here;
    /// [`ArduCopter::start`] refuses it before.
    pub fn new(config: GpsError, seed: u64) -> Self {
        let blend = config.blend().ok().flatten();
        let mut rng = Rng::stream(seed, Stream::GpsError);
        let mut start = [0.0; 2];
        if config.enabled {
            let sd = config.sigma_m * blend.map_or(1.0, |b| b.sent_over_reported());
            start = [rng.gaussian(sd), rng.gaussian(sd)];
        }
        Self {
            config,
            blend,
            rng,
            start,
            offset: [0.0; 2],
            started: false,
        }
    }

    /// The value `sent` had before the first fix instant (`sent(-1)`).
    pub fn before_first_fix(&self) -> [f64; 2] {
        self.start
    }

    /// The offset (north, east) in metres at flight time `t_us`: a new value
    /// at every sample instant of the GPS model, held between. Zero when
    /// switched off.
    pub fn at(&mut self, t_us: u64) -> [f64; 2] {
        let c = self.config;
        if !c.enabled || c.period_us == 0 {
            return [0.0; 2];
        }
        if t_us >= c.first_fix_us && (t_us - c.first_fix_us).is_multiple_of(c.period_us) {
            let n = self.rng.gaussian(c.sigma_m);
            let e = self.rng.gaussian(c.sigma_m);
            let prev = if self.started {
                self.offset
            } else {
                self.start
            };
            self.started = true;
            self.offset = match self.blend {
                None => [n, e],
                Some(b) => [(n - b.lo * prev[0]) / b.hi, (e - b.lo * prev[1]) / b.hi],
            };
        }
        self.offset
    }
}

/// A command of the brain.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum BrainCommand {
    TakeOff {
        height_m: f32,
    },
    Target {
        /// North, east, down, m from the start point.
        position: [f32; 3],
        yaw: f32,
    },
}

/// Turns our setpoint stream into GUIDED commands, while the pilot's mode
/// switch says offboard. Every decision is from the exchange number, the
/// setpoint, the true height, whether the pulses of the previous exchange say
/// the vehicle is armed, and the mode switch of the pilot's last valid radio
/// frame (lockstep data, not the vehicle's own mode).
///
/// Outside offboard nothing is sent, and the state starts afresh when the
/// switch comes back: a take-off from the ground, or, when offboard begins
/// in the air, no take-off and targets at once.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Brain {
    /// The mode switch of the last valid radio frame says offboard. Before
    /// any frame, and in the absence of a valid one, it keeps its value.
    offboard: bool,
    takeoff_height_m: Option<f64>,
    last_takeoff: Option<u64>,
    targets_on: bool,
    last_target: Option<([f32; 3], f32)>,
    last_target_at: Option<u64>,
}

impl Brain {
    /// Notes the pilot's mode switch from the radio frame that arrived at
    /// this tick, if any. A frame with failsafe or a lost frame carries no
    /// switch: the last one stands.
    pub fn on_radio(&mut self, frame: Option<&RcSample>) {
        if let Some(rc) = frame.filter(|rc| rc.is_valid()) {
            self.offboard = rc.mode_switch >= MODE_OFFBOARD;
        }
    }

    /// Whether the pilot's mode switch says offboard.
    pub fn offboard(&self) -> bool {
        self.offboard
    }

    /// Starts afresh, keeping what the switch says.
    fn restart(&mut self) {
        *self = Self {
            offboard: self.offboard,
            ..Self::default()
        };
    }

    /// The commands to send before exchange `n`, when a setpoint arrived at
    /// this tick. `armed` is the pulses' verdict of the previous exchange,
    /// `height_m` the true height above the start point.
    pub fn on_setpoint(
        &mut self,
        n: u64,
        armed: bool,
        height_m: f64,
        sp: &TrajectorySetpoint,
        out: &mut Vec<BrainCommand>,
    ) {
        if !armed || !self.offboard {
            self.restart();
            return;
        }
        let sp_height = -sp.position.z;
        // Take-off: at the first armed tick with an airborne setpoint, and
        // again each second while still on the ground.
        let due = self
            .last_takeoff
            .is_none_or(|l| n.saturating_sub(l) >= SECOND_STEPS);
        if !self.targets_on {
            if sp_height > AIRBORNE_SETPOINT_M && height_m < GROUND_M && due {
                self.takeoff_height_m = Some(f64::from(sp_height));
                self.last_takeoff = Some(n);
                out.push(BrainCommand::TakeOff {
                    height_m: sp_height,
                });
            }
            if let Some(h) = self.takeoff_height_m
                && (height_m - h).abs() <= TAKEOFF_REACHED_M
            {
                self.targets_on = true;
            }
            // Offboard begins in the air (nothing asked for a take-off, and
            // the vehicle is off the ground): no take-off, targets at once.
            if self.takeoff_height_m.is_none() && height_m >= GROUND_M {
                self.targets_on = true;
            }
        }
        if self.targets_on {
            let target = ([sp.position.x, sp.position.y, sp.position.z], sp.yaw);
            let moved = self.last_target.is_none_or(|(p, y)| {
                p.iter()
                    .zip(&target.0)
                    .any(|(a, b)| (a - b).abs() > TARGET_POSITION_STEP_M)
                    || (y - target.1).abs() > TARGET_YAW_STEP_RAD
            });
            let stale = self
                .last_target_at
                .is_none_or(|l| n.saturating_sub(l) >= SECOND_STEPS);
            if moved || stale {
                self.last_target = Some(target);
                self.last_target_at = Some(n);
                out.push(BrainCommand::Target {
                    position: target.0,
                    yaw: target.1,
                });
            }
        }
    }
}

/// Watches for a vehicle that does not arm.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ArmWatch {
    on_since: Option<u64>,
    ever_armed: bool,
}

impl ArmWatch {
    /// After exchange `n` (counting this one): the arm switch's state in the
    /// reply, and whether a pulse is above 1000 µs. True when the switch has
    /// been on for 5 s with no pulse ever above 1000 µs.
    pub fn update(&mut self, n: u64, arm_switch_on: bool, armed_by_pulses: bool) -> bool {
        self.ever_armed |= armed_by_pulses;
        if !arm_switch_on {
            self.on_since = None;
            return false;
        }
        let since = *self.on_since.get_or_insert(n);
        !self.ever_armed && n.saturating_sub(since) >= ARM_TIMEOUT_STEPS
    }
}

/// The motor parameters the program will run with: the last value of
/// `MOT_SPIN_MIN`, `MOT_SPIN_MAX`, `MOT_PWM_MIN` and `MOT_PWM_MAX` among the
/// parameter files (in order) and then the extra parameters, with the
/// program's defaults for those that are set nowhere.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorParams {
    pub spin_min: f64,
    pub spin_max: f64,
    pub pwm_min: f64,
    pub pwm_max: f64,
}

impl Default for MotorParams {
    fn default() -> Self {
        Self {
            spin_min: f64::from(DEFAULT_SPIN_MIN),
            spin_max: f64::from(DEFAULT_SPIN_MAX),
            pwm_min: 1000.0,
            pwm_max: 2000.0,
        }
    }
}

impl MotorParams {
    /// Applies one parameter: a name that is not a motor parameter changes
    /// nothing.
    pub fn apply(&mut self, name: &str, value: f64) {
        match name {
            "MOT_SPIN_MIN" => self.spin_min = value,
            "MOT_SPIN_MAX" => self.spin_max = value,
            "MOT_PWM_MIN" => self.pwm_min = value,
            "MOT_PWM_MAX" => self.pwm_max = value,
            _ => {}
        }
    }

    /// The effective motor parameters of a configuration: reads the parameter
    /// files, then applies the extra parameters.
    pub fn of_config(config: &ArduCopterConfig) -> Result<Self, ArduCopterError> {
        let mut p = Self::default();
        for (name, value) in all_params(config)? {
            p.apply(&name, value);
        }
        Ok(p)
    }
}

/// Every parameter the program is started with, in order: the files, then
/// the extra parameters.
pub fn all_params(config: &ArduCopterConfig) -> Result<Vec<(String, f64)>, ArduCopterError> {
    let mut out = Vec::new();
    for path in &config.param_files {
        let text = std::fs::read_to_string(path).map_err(|source| {
            ArduCopterError::Start(SitlError::ParamFileUnreadable {
                path: path.clone(),
                source,
            })
        })?;
        let values = parm::parse(&text).map_err(|source| ArduCopterError::ParamFile {
            path: path.clone(),
            source,
        })?;
        out.extend(values);
    }
    out.extend(config.extra_params.iter().cloned());
    Ok(out)
}

/// Refuses a GPS error whose lag cannot be undone, or whose lag is not the
/// program's: `SIM_GPS1_LAG_MS` of the files and extra parameters (last
/// value wins) times 1000 must equal [`GpsError::lag_us`].
fn check_gps_lag(config: &ArduCopterConfig) -> Result<(), ArduCopterError> {
    config.gps_error.blend().map_err(ArduCopterError::GpsLag)?;
    let program_ms = all_params(config)?
        .into_iter()
        .rev()
        .find(|(n, _)| n == "SIM_GPS1_LAG_MS")
        .map(|(_, v)| v);
    match program_ms {
        Some(ms) if (ms * 1000.0 - config.gps_error.lag_us as f64).abs() < 0.5 => Ok(()),
        program_ms => Err(ArduCopterError::GpsLagMismatch {
            assumed_us: config.gps_error.lag_us,
            program_ms,
        }),
    }
}

/// The exchange's configuration for `config`: the step, the time-out and the
/// motor map's range, which is the program's own `MOT_SPIN_MIN` and
/// `MOT_SPIN_MAX` ([`MotorParams`]), so that the map follows the parameter
/// files and the extra parameters. A pulse range other than 1000 to 2000 µs
/// is refused, and so is a GPS lag that is not the program's or cannot be
/// undone ([`GpsError`]).
pub fn bridge_config(config: &ArduCopterConfig) -> Result<BridgeConfig, ArduCopterError> {
    check_gps_lag(config)?;
    let motor = MotorParams::of_config(config)?;
    if motor.pwm_min != 1000.0 || motor.pwm_max != 2000.0 {
        return Err(ArduCopterError::Config(ConfigError::PwmRange {
            min: motor.pwm_min,
            max: motor.pwm_max,
        }));
    }
    let mut c = BridgeConfig::new(STEP_US);
    c.reply_timeout = config.reply_timeout;
    c.spin_min = motor.spin_min as f32;
    c.spin_max = motor.spin_max as f32;
    MotorMap::new(c.spin_min, c.spin_max)
        .map_err(|e| ArduCopterError::Config(ConfigError::Motors(e)))?;
    Ok(c)
}

/// ArduCopter as the simulator's autopilot.
pub struct ArduCopter {
    ex: Exchange,
    sitl: Option<Sitl>,
    instrument: MavClient,
    brain_client: Option<MavClient>,
    gcs_client: Option<MavClient>,
    config: ArduCopterConfig,
    /// The IMU sample last given, gyro and specific force, body FRD.
    imu: Option<([f64; 3], [f64; 3])>,
    gps_error: GpsErrorState,
    rc: RcLatch,
    brain: Brain,
    faults: FaultTranslator,
    arm_watch: ArmWatch,
    /// The pulses of the previous exchange say armed.
    armed: bool,
    /// Exchanges done in the pre-roll.
    preroll_steps: u64,
    /// The first flight tick has been seen.
    in_flight: bool,
    commands: Vec<BrainCommand>,
    scratch: Vec<MavEvent>,
    arm_texts: Vec<(u64, String)>,
    vehicle_mode: Option<u32>,
    vehicle_armed: Option<bool>,
    audit: FlightAudit,
}

impl fmt::Debug for ArduCopter {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ArduCopter")
            .field("exchanges", &self.ex.frames())
            .field("preroll_steps", &self.preroll_steps)
            .field("armed", &self.armed)
            .finish_non_exhaustive()
    }
}

impl ArduCopter {
    /// Binds the exchange's port, starts the program with the two parameter
    /// files and the extra parameters, and connects the instrument client to
    /// its serial 0. The brain's client (serial 1) and the ground station's
    /// (serial 2), when the configuration has them, are connected at exchange
    /// [`LINKS_CONNECT_STEP`]: the program opens those ports only after it has
    /// run a few steps. A failure says which step failed and leaves nothing
    /// running.
    pub fn start(config: ArduCopterConfig) -> Result<Self, ArduCopterError> {
        let port =
            json_port_of(config.instance).ok_or(ArduCopterError::Instance(config.instance))?;
        let bridge = bridge_config(&config)?;
        // The socket is bound before the program starts, as it sends to it.
        let link = Link::bind(port).map_err(|source| ArduCopterError::Bind { port, source })?;
        let ex = Exchange::new(link, bridge).map_err(ArduCopterError::Config)?;
        let mut sc = SitlConfig::new(&config.binary, &config.root);
        sc.instance = config.instance;
        sc.speedup = config.speedup;
        sc.param_files.clone_from(&config.param_files);
        sc.extra_params.clone_from(&config.extra_params);
        sc.flight_name.clone_from(&config.flight_name);
        let sitl = Sitl::start(sc).map_err(ArduCopterError::Start)?;
        let limit = sitl.startup_limit();
        let connect = |serial: u8, system: u8, component: u8, link: LinkName| {
            let port = sitl.mavlink_port(serial).unwrap_or(0);
            MavClient::connect(port, system, component, limit)
                .map_err(|source| ArduCopterError::Connect { link, source })
        };
        let instrument = connect(0, config.instrument_system_id, 190, LinkName::Instrument)?;
        let audit = FlightAudit {
            binary_sha256: sitl.binary_sha256().to_string(),
            command_line: sitl.command_line().to_vec(),
            params_sha256: sitl.params_sha256().to_string(),
            seed: config.seed,
            step_us: STEP_US,
            ..FlightAudit::default()
        };
        Ok(Self {
            ex,
            sitl: Some(sitl),
            instrument,
            brain_client: None,
            gcs_client: None,
            gps_error: GpsErrorState::new(config.gps_error, config.seed),
            config,
            imu: None,
            rc: RcLatch::new(),
            brain: Brain::default(),
            faults: FaultTranslator::new(),
            arm_watch: ArmWatch::default(),
            armed: false,
            preroll_steps: 0,
            in_flight: false,
            commands: Vec::new(),
            scratch: Vec::new(),
            arm_texts: Vec::new(),
            vehicle_mode: None,
            vehicle_armed: None,
            audit,
        })
    }

    /// The audit so far.
    pub fn audit(&self) -> &FlightAudit {
        &self.audit
    }

    /// The program's process id, while it is held.
    pub fn pid(&self) -> Option<u32> {
        self.sitl.as_ref().and_then(Sitl::pid)
    }

    /// The program's directory.
    pub fn dir(&self) -> Option<&std::path::Path> {
        self.sitl.as_ref().map(Sitl::dir)
    }

    /// The instrument's MAVLink client, for a probe that sends something of its
    /// own between two ticks (what is sent is not in the audit's `sent` list,
    /// which is the autopilot's own sending). Nothing in a flight uses it.
    pub fn instrument(&self) -> &MavClient {
        &self.instrument
    }

    /// The pulses of the previous exchange say armed.
    pub fn armed(&self) -> bool {
        self.armed
    }

    /// Exchanges done.
    pub fn exchanges(&self) -> u64 {
        self.ex.frames()
    }

    /// Ends the program and returns the audit.
    pub fn finish(mut self) -> Result<FlightAudit, ArduCopterError> {
        self.sync_hashes();
        let mut audit = core::mem::take(&mut self.audit);
        if let Some(sitl) = self.sitl.take() {
            let exit = sitl.stop().map_err(ArduCopterError::Finish)?;
            audit.sitl_ended_before_stop = Some(exit.ended_before_stop);
            audit.sitl_stderr_tail = exit.stderr_tail;
        }
        Ok(audit)
    }

    fn sync_hashes(&mut self) {
        self.audit.packet_hash = self.ex.packet_hash();
        self.audit.reply_hash = self.ex.reply_hash();
        self.audit.last_stamp_ns = self.ex.last_stamp_ns();
        self.audit.stamp_breaks = self.ex.stamp_breaks();
        self.audit.exchanges = self.ex.frames();
        self.audit.preroll_steps = self.preroll_steps;
    }

    fn now(&self) -> SimTime {
        SimTime {
            program_us: self.ex.frames() * STEP_US,
            flight_start_us: self.in_flight.then_some(self.preroll_steps * STEP_US),
        }
    }

    /// The error for a failed MAVLink call: the program's end if it has
    /// ended, else the call's own error.
    fn mav_error(&mut self, link: LinkName, source: MavError) -> ArduCopterError {
        let at = self.now();
        if let Some(sitl) = self.sitl.as_mut()
            && let Err(e) = sitl.check()
        {
            return ArduCopterError::Process { at, source: e };
        }
        ArduCopterError::Mav { at, link, source }
    }

    fn record(&mut self, link: LinkName, system_id: u8, what: SentKind) {
        self.audit.sent.push(Sent {
            at_us: self.ex.frames() * STEP_US,
            link,
            system_id,
            what,
        });
    }

    /// Connects the brain's and the ground station's clients, at exchange
    /// [`LINKS_CONNECT_STEP`].
    fn connect_links(&mut self) -> Result<(), ArduCopterError> {
        let Some(sitl) = self.sitl.as_ref() else {
            return Ok(());
        };
        let limit = sitl.startup_limit();
        let connect = |serial: u8, system: u8, component: u8, link: LinkName| {
            let port = sitl.mavlink_port(serial).unwrap_or(0);
            MavClient::connect(port, system, component, limit)
                .map_err(|source| ArduCopterError::Connect { link, source })
        };
        let brain = match self.config.brain {
            BrainLink::Serial1 => Some(connect(1, BRAIN_SYSTEM_ID, 191, LinkName::Brain)?),
            BrainLink::None | BrainLink::Instrument => None,
        };
        let gcs = if self.config.gcs {
            Some(connect(2, GCS_SYSTEM_ID, 190, LinkName::Gcs)?)
        } else {
            None
        };
        self.brain_client = brain;
        self.gcs_client = gcs;
        Ok(())
    }

    /// Everything sent before exchange `n`: the instrument's heartbeat at 1 Hz,
    /// the telemetry requests at their steps.
    fn send_instrument(&mut self) -> Result<(), ArduCopterError> {
        let n = self.ex.frames();
        if n == LINKS_CONNECT_STEP {
            self.connect_links()?;
        }
        let stopped = self
            .config
            .instrument_heartbeats_stop_at_s
            .is_some_and(|s| {
                self.in_flight && {
                    let from_flight = n.saturating_sub(self.preroll_steps);
                    from_flight as f64 * STEP_US as f64 * 1.0e-6 >= s
                }
            });
        if n.is_multiple_of(SECOND_STEPS) && !stopped {
            self.instrument
                .heartbeat()
                .map_err(|e| self.mav_error(LinkName::Instrument, e))?;
            self.record(
                LinkName::Instrument,
                self.config.instrument_system_id,
                SentKind::Heartbeat,
            );
        }
        for i in 0..self.config.telemetry.len() {
            let Some(req) = self.config.telemetry.get(i).copied() else {
                continue;
            };
            if req.at_step == n {
                self.instrument
                    .set_message_interval(req.message_id, req.hz)
                    .map_err(|e| self.mav_error(LinkName::Instrument, e))?;
                self.record(
                    LinkName::Instrument,
                    self.config.instrument_system_id,
                    SentKind::MessageInterval {
                        message_id: req.message_id,
                        hz: req.hz,
                    },
                );
            }
        }
        Ok(())
    }

    /// Everything the brain and the ground station send before exchange `n`
    /// of a flight tick.
    fn send_flight(
        &mut self,
        t: Micros,
        input: &AutopilotInput<'_, 4>,
    ) -> Result<(), ArduCopterError> {
        let n = self.ex.frames();
        // The brain's heartbeat, while ours come.
        if heartbeat_due(&input.brain_heartbeat, t)
            && self.config.brain == BrainLink::Serial1
            && let Some(c) = self.brain_client.as_ref()
        {
            c.onboard_heartbeat()
                .map_err(|e| self.mav_error(LinkName::Brain, e))?;
            self.record(LinkName::Brain, BRAIN_SYSTEM_ID, SentKind::Heartbeat);
        }
        // The ground station's, while ours come.
        if heartbeat_due(&input.gcs_heartbeat, t)
            && let Some(c) = self.gcs_client.as_ref()
        {
            c.heartbeat()
                .map_err(|e| self.mav_error(LinkName::Gcs, e))?;
            self.record(LinkName::Gcs, GCS_SYSTEM_ID, SentKind::Heartbeat);
        }
        // Our setpoints, as GUIDED commands.
        if self.config.brain == BrainLink::None {
            return Ok(());
        }
        self.brain
            .on_radio(input.rc.as_ref().map(|frame| &frame.value));
        if let Some(sp) = input.setpoint {
            self.commands.clear();
            let height = -input.body.position.z;
            self.brain
                .on_setpoint(n, self.armed, height, &sp, &mut self.commands);
            let on_instrument = self.config.brain == BrainLink::Instrument;
            let (link, system) = if on_instrument {
                (LinkName::Instrument, self.config.instrument_system_id)
            } else {
                (LinkName::Brain, BRAIN_SYSTEM_ID)
            };
            for i in 0..self.commands.len() {
                let Some(cmd) = self.commands.get(i).copied() else {
                    continue;
                };
                let client = if on_instrument {
                    Some(&self.instrument)
                } else {
                    self.brain_client.as_ref()
                };
                let Some(client) = client else { continue };
                let (sent, what) = match cmd {
                    BrainCommand::TakeOff { height_m } => {
                        (client.takeoff(height_m), SentKind::TakeOff { height_m })
                    }
                    BrainCommand::Target { position, yaw } => (
                        client.position_target_local_ned(
                            position[0],
                            position[1],
                            position[2],
                            yaw,
                        ),
                        SentKind::PositionTarget {
                            n: position[0],
                            e: position[1],
                            d: position[2],
                            yaw,
                        },
                    ),
                };
                sent.map_err(|e| self.mav_error(link, e))?;
                self.record(link, system, what);
            }
        }
        Ok(())
    }

    /// The parameter writes of the faults that start or end at this tick,
    /// sent on the instrument's link before the exchange and recorded. Decided
    /// from the fault list and the tick alone ([`FaultTranslator`]): the echo
    /// of a write is not waited for.
    fn send_faults(&mut self, active: &ActiveFaults) -> Result<(), ArduCopterError> {
        for w in self.faults.step(active) {
            self.instrument
                .param_set(w.name, w.value)
                .map_err(|e| self.mav_error(LinkName::Instrument, e))?;
            self.record(
                LinkName::Instrument,
                self.config.instrument_system_id,
                SentKind::ParamSet {
                    name: w.name.to_string(),
                    value: w.value,
                },
            );
        }
        Ok(())
    }

    /// Reads everything that is waiting on every link, into the audit.
    fn drain(&mut self) -> Result<(), ArduCopterError> {
        let at_us = self.ex.frames() * STEP_US;
        for link in [LinkName::Instrument, LinkName::Brain, LinkName::Gcs] {
            let client = match link {
                LinkName::Instrument => Some(&mut self.instrument),
                LinkName::Brain => self.brain_client.as_mut(),
                LinkName::Gcs => self.gcs_client.as_mut(),
            };
            let Some(client) = client else { continue };
            self.scratch.clear();
            if let Err(e) = client.poll(&mut self.scratch) {
                return Err(self.mav_error(link, e));
            }
            for i in 0..self.scratch.len() {
                let Some(event) = self.scratch.get(i).cloned() else {
                    continue;
                };
                self.hear(at_us, link, event);
            }
        }
        Ok(())
    }

    fn hear(&mut self, at_us: u64, link: LinkName, event: MavEvent) {
        match event {
            MavEvent::Heartbeat {
                system_id,
                component_id,
                mode,
                armed,
                ..
            } => {
                // The vehicle's autopilot: system 1, component 1.
                if system_id == 1 && component_id == 1 {
                    self.audit.vehicle_heartbeats += 1;
                    if self.vehicle_mode != Some(mode) {
                        self.vehicle_mode = Some(mode);
                        self.audit.mode_changes.push((at_us, mode));
                    }
                    if self.vehicle_armed != Some(armed) {
                        self.vehicle_armed = Some(armed);
                        self.audit.armed_changes.push((at_us, armed));
                    }
                }
            }
            MavEvent::StatusText { severity, text } => {
                if text.starts_with("PreArm") || text.starts_with("Arm") {
                    self.arm_texts.push((at_us, text.clone()));
                    if self.arm_texts.len() > ARM_TEXTS_KEPT {
                        self.arm_texts.remove(0);
                    }
                }
                self.audit.status_texts.push((at_us, severity, text));
            }
            other => self.audit.heard.push(Heard {
                at_us,
                link,
                event: other,
            }),
        }
    }

    /// Sends the reply for exchange `n`, reads the pulses, drains the links.
    fn exchange(&mut self, reply: Reply) -> Result<[u16; 4], ArduCopterError> {
        // The program's end is noticed within 100 ms, not at the reply
        // time-out.
        let mut alive = || self.sitl.as_mut().is_none_or(|s| s.check().is_ok());
        let packet = match self.ex.exchange_watched(reply, Some(&mut alive)) {
            Ok(p) => p,
            Err(source) => {
                let at = self.now();
                if let Some(sitl) = self.sitl.as_mut()
                    && let Err(e) = sitl.check()
                {
                    return Err(ArduCopterError::Process { at, source: e });
                }
                return Err(ArduCopterError::Exchange { at, source });
            }
        };
        self.sync_hashes();
        Ok(Exchange::pulses(&packet))
    }

    fn latch_imu(&mut self, gyro: [f64; 3], accel: [f64; 3]) {
        self.imu = Some((gyro, accel));
    }
}

fn triple(v: oxide_core::math::Vec3<f32>) -> [f64; 3] {
    [f64::from(v.x), f64::from(v.y), f64::from(v.z)]
}

impl Autopilot<4> for ArduCopter {
    type Error = ArduCopterError;

    fn period_us(&self) -> u64 {
        STEP_US
    }

    fn preroll_tick(
        &mut self,
        _k: u64,
        input: &PrerollInput<'_, 4>,
    ) -> Result<(), ArduCopterError> {
        if let Some(s) = input.imu {
            self.latch_imu(triple(s.gyro), triple(s.accel));
        }
        self.send_instrument()?;
        let mut reply = reply_of_body(
            0,
            input.body,
            input.specific_force_world,
            (input.battery_v, input.battery_a),
            true,
            None,
        );
        if let Some((gyro, accel)) = self.imu {
            reply.gyro = gyro;
            reply.accel_body = accel;
        }
        let pulses = self.exchange(reply)?;
        self.preroll_steps += 1;
        self.sync_hashes();
        for &p in &pulses {
            self.audit.preroll_max_pulse_us = self.audit.preroll_max_pulse_us.max(p);
        }
        if let Some((output, &pulse)) = pulses.iter().enumerate().find(|(_, p)| **p > 1000) {
            return Err(ArduCopterError::ArmedInPreroll {
                at: self.now(),
                output,
                pulse,
            });
        }
        self.drain()
    }

    fn tick(
        &mut self,
        t: Micros,
        input: &AutopilotInput<'_, 4>,
    ) -> Result<AutopilotOutput<4>, ArduCopterError> {
        self.in_flight = true;
        if let Some(s) = input.sensors.imu {
            self.latch_imu(triple(s.value.gyro), triple(s.value.accel));
        }
        self.send_instrument()?;
        self.send_flight(t, input)?;
        self.send_faults(&input.faults)?;
        let rc = self.rc.at(t, &input.rc);
        let mut reply = reply_of_body(
            0,
            input.body,
            input.specific_force_world,
            (input.battery_v, input.battery_a),
            true,
            rc,
        );
        if let Some((gyro, accel)) = self.imu {
            reply.gyro = gyro;
            reply.accel_body = accel;
        }
        let [north, east] = self.gps_error.at(t.0);
        reply.position[0] += north;
        reply.position[1] += east;
        let arm_on = rc.is_some_and(|c| c.get(5).is_some_and(|&p| p > 1500));
        let pulses = self.exchange(reply)?;
        self.armed = armed_by_pulses(&pulses);
        self.drain()?;
        if self.arm_watch.update(self.ex.frames(), arm_on, self.armed) {
            return Err(ArduCopterError::DidNotArm {
                at: self.now(),
                texts: self.arm_texts.clone(),
            });
        }
        Ok(AutopilotOutput {
            t_valid: t,
            motors: self.ex.rotors_from_pulses(&pulses),
            armed: self.armed,
            core: None,
        })
    }
}

/// Why [`fly`] failed.
#[derive(Debug)]
pub enum FlyError {
    /// The simulation could not be built (the options are not usable for a
    /// bridged flight, or the airframe is not).
    Setup(SimError),
    /// The autopilot could not start.
    Start(ArduCopterError),
    /// The pre-roll failed.
    Preroll(PrerollError<ArduCopterError>),
    /// The flight failed; the audit is what the failed flight left, when the
    /// program could be stopped.
    Flight {
        source: ArduCopterError,
        audit: Option<Box<FlightAudit>>,
    },
    /// The program could not be stopped at the end.
    Finish(ArduCopterError),
}

impl fmt::Display for FlyError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Setup(e) => write!(f, "cannot set up the simulation: {e}"),
            Self::Start(e) => write!(f, "{e}"),
            Self::Preroll(e) => write!(f, "pre-roll: {e}"),
            Self::Flight { source, .. } => write!(f, "{source}"),
            Self::Finish(e) => write!(f, "{e}"),
        }
    }
}

impl std::error::Error for FlyError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Setup(e) => Some(e),
            Self::Start(e) | Self::Finish(e) => Some(e),
            Self::Preroll(e) => Some(e),
            Self::Flight { source, .. } => Some(source),
        }
    }
}

/// What a bridged flight returns.
#[derive(Clone, Debug)]
pub struct Flown {
    /// `Simulation::trace_hash` at the end.
    pub trace_hash: u64,
    pub audit: FlightAudit,
}

/// Sets what a bridged flight needs in the options (the IMU sampled every
/// 1000 µs and the wind every 1000 µs, ADR-0006 §4 and §7) and in the
/// configuration (the seed, the ground station, the GPS error's period and
/// its standard deviation: our GPS model's own, undivided), and returns the
/// options. Nothing else is changed.
pub fn bridged_options(mut options: SimOptions, config: &mut ArduCopterConfig) -> SimOptions {
    options.sensors.imu.period_us = 1000;
    options.wind_period_us = Some(1000);
    config.seed = options.seed;
    config.gcs = options.links.gcs;
    config.gps_error.sigma_m = options.sensors.gps.pos_sigma_h_m;
    config.gps_error.period_us = options.sensors.gps.period_us;
    config.gps_error.first_fix_us = options.sensors.gps.first_fix_us;
    options
}

/// Flies a bridged flight: sets what it needs (see [`bridged_options`]) and nothing else
/// (`options.sensors.imu.period_us = 1000`, `options.wind_period_us =
/// Some(1000)` (ADR-0006 §7), the GPS error's standard deviation (see
/// [`bridged_options`]) and period from `options.sensors.gps`, the seed from `options.seed`, the ground
/// station from `options.links.gcs`), starts the program, runs the pre-roll
/// of [`PREROLL_STEPS`] steps and then the flight to `duration`, calling
/// `on_step` for every tick. Options that configure only our own core are
/// refused as `Simulation::with_autopilot` refuses them.
pub fn fly<P: Pilot>(
    airframe: &Airframe,
    pilot: P,
    options: SimOptions,
    mut config: ArduCopterConfig,
    duration: Micros,
    on_step: impl FnMut(&StepRecord<4>),
) -> Result<Flown, FlyError> {
    let options = bridged_options(options, &mut config);
    // Refuse bad options before the program starts.
    let probe = Simulation::<NoOpPilot, 4, NullAutopilot>::with_autopilot(
        airframe,
        NullAutopilot,
        NoOpPilot,
        options.clone(),
    )
    .map(|_| ());
    probe.map_err(FlyError::Setup)?;
    let ap = ArduCopter::start(config).map_err(FlyError::Start)?;
    let mut sim =
        Simulation::with_autopilot(airframe, ap, pilot, options).map_err(FlyError::Setup)?;
    sim.preroll(PREROLL_STEPS).map_err(FlyError::Preroll)?;
    let run = sim.try_run_until(duration, on_step);
    let trace_hash = sim.trace_hash();
    let ap = sim.into_autopilot();
    match run {
        Ok(()) => {
            let audit = ap.finish().map_err(FlyError::Finish)?;
            Ok(Flown { trace_hash, audit })
        }
        Err(source) => {
            let audit = ap.finish().ok().map(Box::new);
            Err(FlyError::Flight { source, audit })
        }
    }
}

/// A stand-in autopilot to check the options before anything starts.
struct NullAutopilot;

impl Autopilot<4> for NullAutopilot {
    type Error = core::convert::Infallible;
    fn period_us(&self) -> u64 {
        STEP_US
    }
    fn tick(
        &mut self,
        t: Micros,
        _input: &AutopilotInput<'_, 4>,
    ) -> Result<AutopilotOutput<4>, Self::Error> {
        Ok(AutopilotOutput {
            t_valid: t,
            motors: [0.0; 4],
            armed: false,
            core: None,
        })
    }
}

struct NoOpPilot;

impl Pilot for NoOpPilot {
    fn rc(&mut self, _t: Micros) -> Option<oxide_core::io::RcSample> {
        None
    }
    fn setpoint(&mut self, _t: Micros) -> Option<TrajectorySetpoint> {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxide_core::math::{Real, Vec3};

    fn sp(x: f32, y: f32, z: f32, yaw: f32) -> TrajectorySetpoint {
        TrajectorySetpoint {
            position: Vec3::new(x, y, z),
            velocity: Vec3::zero(),
            yaw,
        }
    }

    fn brain(
        b: &mut Brain,
        n: u64,
        armed: bool,
        height: f64,
        s: &TrajectorySetpoint,
    ) -> Vec<BrainCommand> {
        let mut out = Vec::new();
        b.on_setpoint(n, armed, height, s, &mut out);
        out
    }

    /// A brain whose pilot's switch says offboard.
    fn offboard_brain() -> Brain {
        let mut b = Brain::default();
        switch(&mut b, MODE_OFFBOARD);
        b
    }

    fn switch(b: &mut Brain, position: u8) {
        b.on_radio(Some(&RcSample {
            mode_switch: position,
            ..RcSample::default()
        }));
    }

    #[test]
    fn nothing_is_sent_while_the_vehicle_is_not_armed_by_its_pulses() {
        let mut b = offboard_brain();
        for n in 0..100 {
            assert!(brain(&mut b, n, false, 0.0, &sp(0.0, 0.0, -2.0, 0.0)).is_empty());
        }
    }

    #[test]
    fn the_take_off_goes_out_at_the_first_armed_tick_with_an_airborne_setpoint() {
        let mut b = offboard_brain();
        // Armed but the setpoint is on the ground (0.2 m is not above 0.2 m).
        assert!(brain(&mut b, 10, true, 0.0, &sp(0.0, 0.0, -0.2, 0.0)).is_empty());
        assert!(brain(&mut b, 11, true, 0.0, &sp(0.0, 0.0, 0.0, 0.0)).is_empty());
        // The first tick with the setpoint above 0.2 m.
        let c = brain(&mut b, 12, true, 0.0, &sp(0.0, 0.0, -2.0, 0.0));
        assert_eq!(c, vec![BrainCommand::TakeOff { height_m: 2.0 }]);
        // Not again until a second of simulated time has passed.
        let s = sp(0.0, 0.0, -2.0, 0.0);
        assert!(brain(&mut b, 12 + SECOND_STEPS - 1, true, 0.1, &s).is_empty());
        assert_eq!(
            brain(&mut b, 12 + SECOND_STEPS, true, 0.1, &s),
            vec![BrainCommand::TakeOff { height_m: 2.0 }]
        );
        // Not once the vehicle is off the ground (height 0.3 m or more).
        assert!(brain(&mut b, 12 + 2 * SECOND_STEPS, true, 0.3, &s).is_empty());
    }

    #[test]
    fn position_targets_start_within_0_3_m_of_the_take_off_height_and_follow_the_rules() {
        let mut b = offboard_brain();
        let s = sp(1.0, 0.0, -2.0, 0.0);
        assert_eq!(brain(&mut b, 0, true, 0.0, &s).len(), 1, "the take-off");
        // Climbing: no target yet at 1.69 m, the first one at 1.7 m.
        assert!(brain(&mut b, 100, true, 1.69, &s).is_empty());
        let c = brain(&mut b, 101, true, 1.71, &s);
        assert_eq!(
            c,
            vec![BrainCommand::Target {
                position: [1.0, 0.0, -2.0],
                yaw: 0.0
            }]
        );
        // The same setpoint: nothing until a second has passed.
        assert!(brain(&mut b, 102, true, 2.0, &s).is_empty());
        assert!(brain(&mut b, 101 + SECOND_STEPS - 1, true, 2.0, &s).is_empty());
        assert_eq!(brain(&mut b, 101 + SECOND_STEPS, true, 2.0, &s).len(), 1);
        // A change of 1 mm or less is not a change; more is, at once.
        let n = 101 + SECOND_STEPS + 10;
        assert!(brain(&mut b, n, true, 2.0, &sp(1.0, 0.001, -2.0, 0.0)).is_empty());
        assert_eq!(
            brain(&mut b, n + 1, true, 2.0, &sp(1.0, 0.0011, -2.0, 0.0)).len(),
            1
        );
        assert!(brain(&mut b, n + 2, true, 2.0, &sp(1.0, 0.0011, -2.0, 0.001)).is_empty());
        let c = brain(&mut b, n + 3, true, 2.0, &sp(1.0, 0.0011, -2.0, 0.0011));
        assert_eq!(
            c,
            vec![BrainCommand::Target {
                position: [1.0, 0.0011, -2.0],
                yaw: 0.0011
            }]
        );
        // Disarmed (by the pulses): the state starts again.
        assert!(brain(&mut b, n + 4, false, 0.0, &s).is_empty());
        assert_eq!(
            brain(&mut b, n + 5, true, 0.0, &s).len(),
            1,
            "a new take-off"
        );
    }

    #[test]
    fn a_brain_whose_switch_was_never_seen_or_is_not_offboard_sends_nothing() {
        let s = sp(1.0, 0.0, -2.0, 0.0);
        let mut b = Brain::default();
        assert!(!b.offboard());
        for n in 0..50 {
            assert!(brain(&mut b, n, true, 0.0, &s).is_empty(), "no frame yet");
        }
        // Stabilized (0) and Position (1): armed, on the ground, an airborne
        // setpoint, and still nothing, and in the air nothing either.
        for position in [0, 1] {
            switch(&mut b, position);
            assert!(!b.offboard());
            for (n, h) in [(100, 0.0), (101, 1.0), (102, 2.0)] {
                assert!(
                    brain(&mut b, n, true, h, &s).is_empty(),
                    "switch {position}"
                );
            }
        }
        switch(&mut b, MODE_OFFBOARD);
        assert!(b.offboard());
        assert_eq!(brain(&mut b, 200, true, 0.0, &s).len(), 1, "the take-off");
    }

    #[test]
    fn the_state_starts_afresh_when_the_switch_comes_back_to_offboard() {
        let mut b = offboard_brain();
        let s = sp(1.0, 0.0, -2.0, 0.0);
        assert_eq!(
            brain(&mut b, 0, true, 0.0, &s),
            vec![BrainCommand::TakeOff { height_m: 2.0 }]
        );
        assert_eq!(brain(&mut b, 100, true, 1.9, &s).len(), 1, "a target");
        // The pilot leaves offboard; the vehicle is in the air: nothing goes.
        switch(&mut b, 1);
        assert!(brain(&mut b, 200, true, 1.9, &s).is_empty());
        assert!(brain(&mut b, 200 + SECOND_STEPS, true, 1.9, &s).is_empty());
        // The pilot lands (still armed, on the ground) and comes back: a new
        // take-off, not the old target.
        assert!(brain(&mut b, 300, true, 0.0, &s).is_empty());
        switch(&mut b, MODE_OFFBOARD);
        assert_eq!(
            brain(&mut b, 301, true, 0.0, &s),
            vec![BrainCommand::TakeOff { height_m: 2.0 }]
        );
        assert!(
            brain(&mut b, 302, true, 0.5, &s).is_empty(),
            "no target yet"
        );
    }

    #[test]
    fn offboard_that_begins_in_the_air_sends_no_take_off_and_targets_at_once() {
        let mut b = Brain::default();
        let s = sp(7.0, 0.0, -10.0, 0.0);
        switch(&mut b, 1);
        assert!(brain(&mut b, 0, true, 10.0, &s).is_empty(), "Position");
        switch(&mut b, MODE_OFFBOARD);
        let c = brain(&mut b, 1, true, 10.0, &s);
        assert_eq!(
            c,
            vec![BrainCommand::Target {
                position: [7.0, 0.0, -10.0],
                yaw: 0.0
            }],
            "the first command in the air is a target"
        );
        // No take-off ever, and the rules of the targets are the usual ones.
        assert!(brain(&mut b, 2, true, 10.0, &s).is_empty());
        assert_eq!(brain(&mut b, 1 + SECOND_STEPS, true, 10.0, &s).len(), 1);
        let c = brain(
            &mut b,
            2 + SECOND_STEPS,
            true,
            10.0,
            &sp(8.0, 0.0, -10.0, 0.0),
        );
        assert!(
            matches!(c.as_slice(), [BrainCommand::Target { .. }]),
            "{c:?}"
        );
        // Just above the ground (0.3 m) counts as the air; under it is the ground.
        let mut b = offboard_brain();
        assert!(brain(&mut b, 0, true, 0.29, &sp(0.0, 0.0, 0.0, 0.0)).is_empty());
        let mut b = offboard_brain();
        assert_eq!(
            brain(&mut b, 0, true, 0.3, &sp(0.0, 0.0, -1.0, 0.0)).len(),
            1
        );
    }

    #[test]
    fn a_frame_with_failsafe_or_a_lost_frame_does_not_move_the_switch() {
        let mut b = offboard_brain();
        for rc in [
            RcSample {
                failsafe: true,
                mode_switch: 0,
                ..RcSample::default()
            },
            RcSample {
                frame_lost: true,
                mode_switch: 1,
                ..RcSample::default()
            },
        ] {
            b.on_radio(Some(&rc));
            assert!(b.offboard(), "{rc:?}");
        }
        b.on_radio(None);
        assert!(b.offboard(), "no frame at this tick");
        let mut b = Brain::default();
        b.on_radio(Some(&RcSample {
            failsafe: true,
            mode_switch: 2,
            ..RcSample::default()
        }));
        assert!(!b.offboard(), "a failsafe frame never says offboard");
    }

    #[test]
    fn the_arm_watch_trips_after_5_s_of_switch_with_no_pulse_and_never_after_one() {
        let mut w = ArmWatch::default();
        assert!(!w.update(10, false, false));
        let mut tripped = None;
        for n in 20..(20 + ARM_TIMEOUT_STEPS + 5) {
            if w.update(n, true, false) {
                tripped = Some(n);
                break;
            }
        }
        assert_eq!(tripped, Some(20 + ARM_TIMEOUT_STEPS));
        // A switch that goes off starts the count again.
        let mut w = ArmWatch::default();
        assert!(!w.update(0, true, false));
        assert!(!w.update(ARM_TIMEOUT_STEPS - 1, true, false));
        assert!(!w.update(ARM_TIMEOUT_STEPS, false, false));
        assert!(!w.update(ARM_TIMEOUT_STEPS + 1, true, false));
        assert!(!w.update(ARM_TIMEOUT_STEPS + 1 + ARM_TIMEOUT_STEPS - 1, true, false));
        assert!(w.update(ARM_TIMEOUT_STEPS + 1 + ARM_TIMEOUT_STEPS, true, false));
        // A pulse above 1000 at any time ends the watch for good.
        let mut w = ArmWatch::default();
        assert!(!w.update(0, true, false));
        assert!(!w.update(5, true, true));
        assert!(!w.update(10 * ARM_TIMEOUT_STEPS, true, false));
    }

    #[test]
    fn the_gps_error_is_drawn_every_period_held_between_and_off_when_off() {
        let on = GpsError {
            enabled: true,
            sigma_m: 0.3,
            period_us: 200_000,
            first_fix_us: 0,
            lag_us: 150_000,
        };
        let mut g = GpsErrorState::new(on, 7);
        let a = g.at(0);
        assert_ne!(a, [0.0; 2], "a draw at the first instant");
        for t in (500..200_000).step_by(500) {
            assert_eq!(g.at(t), a, "held at {t}");
        }
        let b = g.at(200_000);
        assert_ne!(b, a);
        // Repeats for a seed, differs for another.
        let mut again = GpsErrorState::new(on, 7);
        assert_eq!(again.at(0), a);
        assert_eq!(again.at(200_000), b);
        assert_ne!(GpsErrorState::new(on, 8).at(0), a);
        // Off: zero always.
        let mut off = GpsErrorState::new(
            GpsError {
                enabled: false,
                ..on
            },
            7,
        );
        assert_eq!(off.at(0), [0.0; 2]);
        assert_eq!(off.at(200_000), [0.0; 2]);
    }

    /// The error ArduCopter's GPS reports for the sent values of `state`:
    /// fix `j` is `0.75 sent(j-2) + 0.25 sent(j-1)` (measured on the binary,
    /// test 6 of `arducopter_fly`), from `j = 1`, which uses `sent(-1)`. Per
    /// axis, `count` fixes.
    fn reported_fixes(config: GpsError, seed: u64, count: usize) -> [Vec<f64>; 2] {
        let mut g = GpsErrorState::new(config, seed);
        let mut sent = vec![g.before_first_fix()];
        for k in 0..count as u64 {
            sent.push(g.at(k * config.period_us));
        }
        // sent[i] is sent(i - 1); fix j (from 1) uses sent(j-2), sent(j-1),
        // which are sent[j-1] and sent[j].
        let fix = |axis: usize| -> Vec<f64> {
            sent.windows(2)
                .map(|w| 0.75 * w[0][axis] + 0.25 * w[1][axis])
                .collect()
        };
        [fix(0), fix(1)]
    }

    fn sd_of(v: &[f64]) -> f64 {
        let n = v.len() as f64;
        let m = v.iter().sum::<f64>() / n;
        Real::sqrt(v.iter().map(|x| (x - m) * (x - m)).sum::<f64>() / n)
    }

    /// The correlation at `lag` of a series about its own mean.
    fn corr(v: &[f64], lag: usize) -> f64 {
        let n = v.len() as f64;
        let m = v.iter().sum::<f64>() / n;
        let var = v.iter().map(|x| (x - m) * (x - m)).sum::<f64>() / n;
        let cov = v
            .iter()
            .zip(v.iter().skip(lag))
            .map(|(a, b)| (a - m) * (b - m))
            .sum::<f64>()
            / (n - lag as f64);
        cov / var
    }

    #[test]
    fn the_error_arducopters_gps_reports_is_white_with_our_models_sd() {
        let c = GpsError::ours();
        for seed in [5, 6] {
            let axes = reported_fixes(c, seed, 40_000);
            for (axis, v) in axes.iter().enumerate() {
                let sd = sd_of(v);
                assert!(
                    (sd / c.sigma_m - 1.0).abs() < 0.02,
                    "seed {seed} axis {axis}: sd {sd}"
                );
                for lag in 1..=5 {
                    let r = corr(v, lag);
                    assert!(
                        r.abs() < 0.03,
                        "seed {seed} axis {axis}: lag {lag} corr {r}"
                    );
                }
            }
        }
    }

    /// The sequence is steady from the first fix: 200 fixes, wider bounds
    /// (the standard error of a standard deviation is `sigma / sqrt(2 n)` =
    /// 5 %, of a correlation `1 / sqrt(n)` = 0.07; three of them).
    #[test]
    fn the_first_200_fixes_are_already_white_with_our_models_sd() {
        let c = GpsError::ours();
        for seed in 1..=6 {
            let axes = reported_fixes(c, seed, 200);
            for (axis, v) in axes.iter().enumerate() {
                let sd = sd_of(v);
                assert!(
                    (sd / c.sigma_m - 1.0).abs() < 0.16,
                    "seed {seed} axis {axis}: sd {sd}"
                );
                for lag in 1..=5 {
                    let r = corr(v, lag);
                    assert!(
                        r.abs() < 0.22,
                        "seed {seed} axis {axis}: lag {lag} corr {r}"
                    );
                }
            }
        }
    }

    /// What 200 fixes of one seed cannot show: the first fix of the flight
    /// (`0.75 sent(-1) + 0.25 sent(0)`) over many seeds has our model's
    /// standard deviation only when `sent(-1)` is a draw of the steady
    /// sequence (left at zero it would be a ninth of the variance), and so
    /// has the second.
    #[test]
    fn the_first_fixes_over_many_seeds_have_our_models_sd() {
        let c = GpsError::ours();
        let mut first = Vec::new();
        let mut second = Vec::new();
        for seed in 0..3000 {
            let [n, e] = reported_fixes(c, seed, 2);
            first.extend([n[0], e[0]]);
            second.extend([n[1], e[1]]);
        }
        for (name, v) in [("first", &first), ("second", &second)] {
            let sd = sd_of(v);
            // 6000 values: a standard error of 0.9 %.
            assert!((sd / c.sigma_m - 1.0).abs() < 0.04, "{name} fix: sd {sd}");
        }
    }

    #[test]
    fn what_is_sent_has_the_standard_deviation_sigma_over_sqrt_half() {
        let c = GpsError::ours();
        let mut g = GpsErrorState::new(c, 9);
        let sent: Vec<[f64; 2]> = (0..40_000).map(|k| g.at(k * c.period_us)).collect();
        let want = c.sigma_m / Real::sqrt(0.5);
        assert!((want - 0.4243).abs() < 1e-4);
        for axis in 0..2 {
            let v: Vec<f64> = sent.iter().map(|s| s[axis]).collect();
            let sd = sd_of(&v);
            assert!((sd / want - 1.0).abs() < 0.03, "axis {axis}: sd {sd}");
            // The pre-filter's own correlation: -lo/hi = -1/3 at lag 1.
            assert!((corr(&v, 1) + 1.0 / 3.0).abs() < 0.03, "{}", corr(&v, 1));
        }
        // The start value is a draw of the same size.
        let starts: Vec<f64> = (0..3000)
            .flat_map(|seed| GpsErrorState::new(c, seed).before_first_fix())
            .collect();
        assert!(
            (sd_of(&starts) / want - 1.0).abs() < 0.04,
            "{}",
            sd_of(&starts)
        );
    }

    #[test]
    fn the_draws_come_in_the_order_start_north_east_then_north_east_per_fix() {
        let c = GpsError::ours();
        let blend = c.blend().unwrap().unwrap();
        assert_eq!((blend.hi, blend.lo), (0.75, 0.25));
        let mut r = Rng::stream(21, Stream::GpsError);
        let sd = c.sigma_m * blend.sent_over_reported();
        let start = [r.gaussian(sd), r.gaussian(sd)];
        let mut g = GpsErrorState::new(c, 21);
        assert_eq!(g.before_first_fix(), start);
        let (n0, e0) = (r.gaussian(c.sigma_m), r.gaussian(c.sigma_m));
        let s0 = [(n0 - 0.25 * start[0]) / 0.75, (e0 - 0.25 * start[1]) / 0.75];
        assert_eq!(g.at(0), s0);
        let (n1, e1) = (r.gaussian(c.sigma_m), r.gaussian(c.sigma_m));
        assert_eq!(
            g.at(200_000),
            [(n1 - 0.25 * s0[0]) / 0.75, (e1 - 0.25 * s0[1]) / 0.75]
        );
    }

    #[test]
    fn a_port_in_use_names_the_instance_number_and_the_overlap() {
        let in_use = ArduCopterError::Bind {
            port: 9052,
            source: io::Error::from(io::ErrorKind::AddrInUse),
        }
        .to_string();
        assert!(
            in_use.contains("instance number 5 is in use by another program")
                && in_use.contains("overlapping --instances"),
            "{in_use}"
        );
        let other = ArduCopterError::Bind {
            port: 9052,
            source: io::Error::from(io::ErrorKind::PermissionDenied),
        }
        .to_string();
        assert!(!other.contains("instance number"), "{other}");
    }

    #[test]
    fn a_heartbeat_is_forwarded_on_the_second_and_only_while_ours_arrives() {
        let (some, none): (Option<u8>, Option<u8>) = (Some(1), None);
        assert!(heartbeat_due(&some, Micros(7_000_000)));
        assert!(heartbeat_due(&some, Micros(0)));
        assert!(!heartbeat_due(&some, Micros(7_000_500)));
        assert!(
            !heartbeat_due(&none, Micros(7_000_000)),
            "a lost link sends none"
        );
        assert!(!heartbeat_due(&none, Micros(7_000_500)));
    }

    #[test]
    fn a_lag_that_is_zero_or_a_period_or_more_is_not_blended() {
        // A whole number of periods reads one stored sample.
        for lag_us in [0, 200_000, 400_000, 600_000] {
            let c = GpsError {
                lag_us,
                ..GpsError::ours()
            };
            assert_eq!(c.blend(), Ok(None), "lag {lag_us}");
            let mut g = GpsErrorState::new(c, 4);
            let mut want = Rng::stream(4, Stream::GpsError);
            let _ = (want.gaussian(0.3), want.gaussian(0.3));
            for k in 0..5 {
                let got = g.at(k * 200_000);
                assert_eq!(
                    got,
                    [want.gaussian(0.3), want.gaussian(0.3)],
                    "lag {lag_us}"
                );
            }
        }
        // A lag beyond a period is blended by its part beyond whole periods
        // (the program reads between two stored samples whatever the lag):
        // 250 ms is 0.25 of a period more than one, 350 ms is 0.75 more.
        for (lag_us, hi, lo) in [
            (250_000, 0.75, 0.25),
            (350_000, 0.75, 0.25),
            (450_000, 0.75, 0.25),
            (270_000, 0.65, 0.35),
        ] {
            let c = GpsError {
                lag_us,
                ..GpsError::ours()
            };
            let b = c.blend().unwrap().unwrap_or_else(|| panic!("{lag_us}"));
            assert!(
                (b.hi - hi).abs() < 1e-12 && (b.lo - lo).abs() < 1e-12,
                "{lag_us}"
            );
            // The sequence it sends is the pre-filtered one, not the draws.
            let mut g = GpsErrorState::new(c, 4);
            let mut want = Rng::stream(4, Stream::GpsError);
            let sd = c.sigma_m * b.sent_over_reported();
            let start = [want.gaussian(sd), want.gaussian(sd)];
            let (n, e) = (want.gaussian(0.3), want.gaussian(0.3));
            assert_eq!(
                g.at(0),
                [(n - b.lo * start[0]) / b.hi, (e - b.lo * start[1]) / b.hi],
                "{lag_us}"
            );
        }
        // A lag of a quarter of the period has weights 0.25 and 0.75.
        let c = GpsError {
            lag_us: 50_000,
            ..GpsError::ours()
        };
        let b = c.blend().unwrap().unwrap();
        assert_eq!((b.hi, b.lo), (0.75, 0.25));
    }

    #[test]
    fn a_lag_within_5_percent_of_half_the_period_is_refused_with_a_sentence() {
        for lag_us in [95_000, 100_000, 104_000, 295_000, 300_000, 504_000] {
            let c = GpsError {
                lag_us,
                ..GpsError::ours()
            };
            let e = c.blend().unwrap_err();
            let s = e.to_string();
            assert!(s.contains("half") && s.contains(&lag_us.to_string()), "{s}");
        }
        // Just outside the band both sides.
        for lag_us in [94_000, 106_000, 294_000, 306_000] {
            let c = GpsError {
                lag_us,
                ..GpsError::ours()
            };
            assert!(c.blend().unwrap().is_some(), "{lag_us}");
        }
        // The autopilot is refused at build, before anything starts.
        let mut c = ArduCopterConfig::new("/nonexistent", "/nonexistent", 0);
        c.gps_error.lag_us = 100_000;
        let e = ArduCopter::start(c).unwrap_err();
        assert!(matches!(e, ArduCopterError::GpsLag(_)), "{e}");
        assert!(e.to_string().contains("half"), "{e}");
    }

    #[test]
    fn the_lag_the_bridge_assumes_must_be_the_programs() {
        // The committed files set 150 ms: the default configuration passes.
        bridge_config(&config_with(Vec::new())).expect("the default lag");
        // A later parameter wins, as in the program.
        let e = bridge_config(&config_with(vec![("SIM_GPS1_LAG_MS".into(), 100.0)])).unwrap_err();
        let s = e.to_string();
        assert!(
            matches!(e, ArduCopterError::GpsLagMismatch { .. })
                && s.contains("150000")
                && s.contains("SIM_GPS1_LAG_MS 100")
                && s.contains("100000"),
            "{s}"
        );
        // Unless the assumption follows.
        let mut c = config_with(vec![("SIM_GPS1_LAG_MS".into(), 100.0)]);
        c.gps_error.lag_us = 100_000;
        let e = bridge_config(&c).unwrap_err();
        assert!(
            matches!(e, ArduCopterError::GpsLag(_)),
            "100 ms is half: {e}"
        );
        let mut c = config_with(vec![("SIM_GPS1_LAG_MS".into(), 60.0)]);
        c.gps_error.lag_us = 60_000;
        bridge_config(&c).expect("60 ms assumed and set");
        // The last value wins.
        let c = config_with(vec![
            ("SIM_GPS1_LAG_MS".into(), 100.0),
            ("SIM_GPS1_LAG_MS".into(), 150.0),
        ]);
        bridge_config(&c).expect("last wins");
        // A file with the other lag is refused.
        let dir = std::env::temp_dir().join(format!("oxide-lag-test-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let file = dir.join("lag.parm");
        std::fs::write(&file, "SIM_GPS1_LAG_MS 100\n").unwrap();
        let mut c = ArduCopterConfig::new("/nonexistent", "/nonexistent", 0);
        c.param_files.push(file);
        let e = bridge_config(&c).unwrap_err();
        let _ = std::fs::remove_dir_all(&dir);
        assert!(matches!(e, ArduCopterError::GpsLagMismatch { .. }), "{e}");
        // Not set anywhere: the assumption cannot be checked.
        let mut c = ArduCopterConfig::new("/nonexistent", "/nonexistent", 0);
        c.param_files = vec![parm::starting_gains_path()];
        let e = bridge_config(&c).unwrap_err();
        assert!(e.to_string().contains("no parameter file sets"), "{e}");
    }

    #[test]
    fn the_gps_error_stream_is_not_the_gps_models() {
        // The draws come from the stream of their own: the first draw is not
        // the GPS model's first draw for the same seed.
        let mut ours = Rng::stream(3, Stream::GpsError);
        let mut models = Rng::stream(3, Stream::Gps);
        assert_ne!(ours.gaussian(1.0), models.gaussian(1.0));
    }

    #[test]
    fn the_errors_name_the_simulated_time() {
        let at = SimTime {
            program_us: 47_250_000,
            flight_start_us: Some(45_000_000),
        };
        let e = ArduCopterError::DidNotArm {
            at,
            texts: vec![(
                46_000_000,
                "PreArm: Motors: Check frame class and type".into(),
            )],
        };
        let s = e.to_string();
        assert!(s.starts_with("ArduCopter did not arm"), "{s}");
        assert!(
            s.contains("47.25") && s.contains("2.25") && s.contains("PreArm: Motors"),
            "{s}"
        );
        let e = ArduCopterError::ArmedInPreroll {
            at: SimTime {
                program_us: 1_000_000,
                flight_start_us: None,
            },
            output: 2,
            pulse: 1100,
        };
        let s = e.to_string();
        assert!(
            s.contains("1.0") && s.contains("pre-roll") && s.contains("1100"),
            "{s}"
        );
        let e = ArduCopterError::Connect {
            link: LinkName::Brain,
            source: MavError::Closed,
        };
        assert!(e.to_string().contains("start") && e.to_string().contains("brain"));
    }

    #[test]
    fn bridged_options_set_what_a_bridged_flight_needs_and_nothing_else() {
        let mut c = ArduCopterConfig::new("/nonexistent", "/nonexistent", 0);
        let before = SimOptions::realistic(5);
        let after = bridged_options(before.clone(), &mut c);
        assert_eq!(after.sensors.imu.period_us, 1000);
        assert_eq!(after.wind_period_us, Some(1000));
        assert_eq!(c.seed, 5);
        assert!(!c.gcs, "the default scenario has no ground station");
        // Our model's own value, undivided.
        assert_eq!(c.gps_error.sigma_m, before.sensors.gps.pos_sigma_h_m);
        assert_eq!(c.gps_error.sigma_m, 0.3);
        assert_eq!(c.gps_error.lag_us, 150_000);
        assert_eq!(c.gps_error.period_us, 200_000);
        let mut expected = before;
        expected.sensors.imu.period_us = 1000;
        expected.wind_period_us = Some(1000);
        assert_eq!(after, expected);
        let mut with_gcs = SimOptions::realistic(5);
        with_gcs.links.gcs = true;
        let _ = bridged_options(with_gcs, &mut c);
        assert!(c.gcs);
    }

    fn config_with(extra: Vec<(String, f64)>) -> ArduCopterConfig {
        let mut c = ArduCopterConfig::new("/nonexistent", "/nonexistent", 0);
        c.extra_params = extra;
        c
    }

    #[test]
    fn the_motor_map_is_built_from_the_parameter_files() {
        // The committed files set neither: the program's defaults, which are
        // the map's constants (read from the binary: 0.15, 0.95, 1000, 2000).
        let b = bridge_config(&config_with(Vec::new())).expect("config");
        assert_eq!(b.spin_min, 0.15);
        assert_eq!(b.spin_max, 0.95);
        assert_eq!(
            MotorParams::of_config(&config_with(Vec::new()))
                .unwrap()
                .pwm_max,
            2000.0
        );
    }

    #[test]
    fn extra_parameters_that_name_the_motor_range_reach_the_motor_map() {
        let b = bridge_config(&config_with(vec![
            ("MOT_SPIN_MAX".into(), 0.9),
            ("MOT_SPIN_MIN".into(), 0.12),
        ]))
        .expect("config");
        assert_eq!((b.spin_min, b.spin_max), (0.12, 0.9));
        // The last value wins, as it does in the program's file.
        let b = bridge_config(&config_with(vec![
            ("MOT_SPIN_MAX".into(), 0.9),
            ("MOT_SPIN_MAX".into(), 0.8),
        ]))
        .expect("config");
        assert_eq!(b.spin_max, 0.8);
        // A range the map refuses is a configuration error.
        let e = bridge_config(&config_with(vec![("MOT_SPIN_MAX".into(), 1.2)])).unwrap_err();
        assert!(
            matches!(e, ArduCopterError::Config(ConfigError::Motors(_))),
            "{e}"
        );
    }

    #[test]
    fn a_pulse_range_other_than_1000_to_2000_is_refused() {
        let e = bridge_config(&config_with(vec![("MOT_PWM_MAX".into(), 1900.0)])).unwrap_err();
        assert!(
            matches!(e, ArduCopterError::Config(ConfigError::PwmRange { .. })),
            "{e}"
        );
        assert!(e.to_string().contains("MOT_PWM_MAX 1900"), "{e}");
    }

    /// The decision paths (the brain, `send_flight`, `send_instrument`, the
    /// pre-roll and flight ticks) never read what came from MAVLink: the
    /// audit's lists and the vehicle's own armed flag and mode. Reading this
    /// file is crude, and what it guards is the determinism rule of the module
    /// documentation.
    #[test]
    fn no_decision_path_reads_what_was_read_from_mavlink() {
        let src = include_str!("arducopter.rs");
        let between = |from: &str, to: &str| -> &str {
            let a = src.find(from).unwrap_or_else(|| panic!("{from} not found"));
            let b = a + src[a..]
                .find(to)
                .unwrap_or_else(|| panic!("{to} not found"));
            &src[a..b]
        };
        let regions = [
            (
                "Brain",
                between("impl Brain {", "/// Watches for a vehicle"),
            ),
            (
                "GpsErrorState",
                between("impl GpsErrorState {", "/// A command of the brain."),
            ),
            (
                "send_instrument and send_flight",
                between(
                    "    fn send_instrument(",
                    "    /// Reads everything that is waiting on every link",
                ),
            ),
            (
                "the fault translator",
                include_str!("faults.rs")
                    .split("#[cfg(test)]")
                    .next()
                    .unwrap_or(""),
            ),
            (
                "the pre-roll and flight ticks",
                between(
                    "impl Autopilot<4> for ArduCopter {",
                    "/// Why [`fly`] failed.",
                ),
            ),
        ];
        let heard = [
            "vehicle_armed",
            "vehicle_mode",
            "self.audit.status_texts",
            "self.audit.armed_changes",
            "self.audit.mode_changes",
            "self.audit.heard",
            "self.audit.vehicle_heartbeats",
            "param_echoes",
            "MavEvent",
        ];
        for (name, text) in regions {
            for token in heard {
                assert!(
                    !text.contains(token),
                    "{name} reads `{token}`: what comes from MAVLink is audit data only"
                );
            }
        }
    }
}
