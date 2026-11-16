// SPDX-License-Identifier: AGPL-3.0-or-later
//! Replay of a replay-level log (ADR-0005 §5): the flight core is rebuilt
//! from the file's header, fed the recorded ticks in file order, and every
//! output is compared with the recorded one, bit for bit.
//!
//! [`replay`] returns a value, never text: a [`ReplayReport`] (how many ticks
//! were replayed, the first [`Difference`], and why the walk [`Stop`]ped), or a
//! [`ReplayError`] when the file cannot be replayed at all. Nothing here
//! panics on a damaged or hostile file.
//!
//! The walk, per tick record (`oxide_in_tick`): the records up to the next tick
//! record, or up to a dropout, are the tick's group. The group is complete when
//! it has a record for every input the tick record says is present, one
//! `oxide_out`, and as many `oxide_event` records as `oxide_out` says. A
//! complete group is replayed: `Core::clear_unsaved` if the tick record says
//! the store saved, `Core::tick`, then the comparison. An incomplete group ends
//! the walk: at a dropout it is the gap, at the end of the file it is a file
//! cut short, anywhere else the file is malformed.

use std::fmt;

use oxide_core::Core;
use oxide_core::config::{ConfigError, VehicleConfig};
use oxide_core::io::{Event, Inputs, Outputs};
use oxide_core::schedule::{F405_LEAN, H743_FULL, Profile};
use oxide_core::time::Micros;
use oxide_log::reader::{Item, ReadError, RecordView, UlogFile};
use oxide_log::replay::decode::{
    self, DecodeError, FileConfigError, InputsError, OutRecord, TickRecord,
};

use crate::SimError;
use crate::sim::apply_params;

/// The rotor count this build flies.
const ROTORS: usize = 4;

/// What to do beyond the plain replay.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ReplayOptions {
    /// Parameter overrides by name, applied to the rebuilt configuration
    /// before the first tick (replay through a changed setting).
    pub params: Vec<(String, f64)>,
    /// Go on after a difference and count the ticks that differ. Without it
    /// the walk stops at the first difference.
    pub keep_going: bool,
}

/// Why a file cannot be replayed at all.
#[derive(Debug)]
pub enum ReplayError {
    /// The bytes are not a ULog file this reader accepts.
    Read(ReadError),
    /// The file has no `oxide_log_level` information, or it is not `replay`.
    NotReplayLevel(Option<String>),
    /// The file has no (or more than one) `oxide_boot` record.
    NoBoot(usize),
    /// The `oxide_boot` record is damaged.
    Boot(DecodeError),
    /// The file names a profile this build does not have.
    UnknownProfile(Option<String>),
    /// The boot record's rates are not the named profile's.
    ProfileMismatch(&'static str),
    /// A start record (`oxide_vehicle`, `oxide_param_us`) is not stamped with
    /// the time of the `oxide_boot` record.
    StartTime(&'static str),
    /// The vehicle has another rotor count than this build flies.
    Rotors(u8),
    /// The vehicle numbers and parameters do not rebuild the configuration
    /// the header states (including its `config_hash`).
    Config(FileConfigError),
    /// A parameter override was refused.
    Override(SimError),
    /// The core refused the rebuilt configuration.
    Core(ConfigError),
}

impl fmt::Display for ReplayError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ReplayError::Read(e) => write!(f, "not a readable log: {e}"),
            ReplayError::NotReplayLevel(None) => {
                write!(f, "not a replay-level log: it has no oxide_log_level")
            }
            ReplayError::NotReplayLevel(Some(l)) => write!(
                f,
                "not a replay-level log: oxide_log_level is {l:?} (record with --log-level replay)"
            ),
            ReplayError::NoBoot(0) => {
                write!(f, "not a replay-level log: it has no oxide_boot record")
            }
            ReplayError::NoBoot(n) => write!(f, "{n} oxide_boot records, one expected"),
            ReplayError::Boot(e) => write!(f, "oxide_boot: {e}"),
            ReplayError::UnknownProfile(Some(p)) => {
                write!(f, "unknown profile {p:?} (f405-lean | h743-full)")
            }
            ReplayError::UnknownProfile(None) => write!(f, "the file names no profile"),
            ReplayError::ProfileMismatch(p) => write!(
                f,
                "the boot record's rates are not those of profile {p} in this build"
            ),
            ReplayError::StartTime(topic) => {
                write!(f, "{topic} has another timestamp than oxide_boot")
            }
            ReplayError::Rotors(n) => write!(
                f,
                "the vehicle has {n} rotors; this build replays vehicles with {ROTORS}"
            ),
            ReplayError::Config(e) => write!(f, "cannot rebuild the configuration: {e}"),
            ReplayError::Override(e) => write!(f, "{e}"),
            ReplayError::Core(e) => write!(f, "the flight core refused the configuration: {e:?}"),
        }
    }
}

impl std::error::Error for ReplayError {}

/// The first value of a tick's outputs that differs.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Difference {
    /// The tick's sequence number.
    pub seq: u64,
    /// The tick's time, µs.
    pub t_us: u64,
    /// The field: `t_valid`, `motors[2]`, `armed`, `reasons`, `events`,
    /// `events[0]`, ...
    pub field: String,
    /// The recorded value as text.
    pub recorded: String,
    /// The replayed value as text.
    pub replayed: String,
    /// The recorded value's bits, for a float field.
    pub recorded_bits: Option<u32>,
    /// The replayed value's bits, for a float field.
    pub replayed_bits: Option<u32>,
}

impl fmt::Display for Difference {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "tick {} (t = {} us): {}: recorded {}",
            self.seq, self.t_us, self.field, self.recorded
        )?;
        if let Some(b) = self.recorded_bits {
            write!(f, " (bits {b:#010x})")?;
        }
        write!(f, ", replayed {}", self.replayed)?;
        if let Some(b) = self.replayed_bits {
            write!(f, " (bits {b:#010x})")?;
        }
        Ok(())
    }
}

/// Why the walk ended.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Stop {
    /// Every tick of the file was replayed.
    End,
    /// A difference was found and `keep_going` was not set.
    Difference,
    /// The file is cut short: a message is cut by its end, or the last tick's
    /// records are not all there. The ticks before it were replayed.
    Cut,
    /// The log has a dropout (records were lost): nothing after it can be
    /// shown.
    Dropout {
        /// The gap the logger measured, ms.
        ms: u16,
    },
    /// A tick record's sequence number is not the next one.
    SequenceGap {
        /// The number that should have come.
        expected: u64,
        /// The number that did.
        found: u64,
    },
    /// The file is damaged where a cut was expected: its last message declares
    /// a size no message of the file has, so the end of the file is not a power
    /// loss. The ticks before it were replayed.
    Damaged {
        /// What is wrong.
        why: String,
    },
    /// A tick's records contradict each other or are damaged.
    Malformed {
        /// The tick record's sequence number, if it could be read.
        seq: Option<u64>,
        /// What is wrong.
        why: String,
    },
}

/// The result of a replay.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ReplayReport {
    /// The profile the file names.
    pub profile: &'static str,
    /// Ticks replayed and compared.
    pub ticks: u64,
    /// Ticks whose outputs differ (at most 1 unless `keep_going`).
    pub differing: u64,
    /// The first difference.
    pub first_difference: Option<Difference>,
    /// Sequence number and time (µs) of the last tick replayed.
    pub last_tick: Option<(u64, u64)>,
    /// Why the walk ended.
    pub stop: Stop,
}

impl ReplayReport {
    /// Every tick replayed has identical outputs.
    pub fn identical(&self) -> bool {
        self.differing == 0
    }

    /// The command's exit code: 0 when every tick replayed is identical and
    /// the walk ended at the end of the file or in a cut, and at least one tick
    /// was replayed; 1 on a difference, a gap, a missing sequence number, a
    /// malformed tick or a damaged end (nothing after those can be proven).
    pub fn exit_code(&self) -> u8 {
        match (&self.stop, self.identical()) {
            // A replay of no tick shows nothing.
            (Stop::End | Stop::Cut, true) if self.ticks > 0 => 0,
            _ => 1,
        }
    }

    /// The lines the command prints.
    pub fn lines(&self) -> Vec<String> {
        let n = self.ticks;
        let plural = if n == 1 { "tick" } else { "ticks" };
        let mut lines = Vec::new();
        let head = match (&self.first_difference, self.differing) {
            (None, _) => format!("replayed {n} {plural} ({}): identical", self.profile),
            (Some(d), 1) => {
                format!(
                    "replayed {n} {plural} ({}): first difference at {d}",
                    self.profile
                )
            }
            (Some(d), k) => format!(
                "replayed {n} {plural} ({}): {k} differ, the first at {d}",
                self.profile
            ),
        };
        lines.push(head);
        let after = self.last_tick.map_or_else(
            || "before the first tick".to_string(),
            |(s, _)| format!("after tick {s}"),
        );
        match &self.stop {
            Stop::End => {}
            Stop::Difference => lines.push("stopped at the first difference".to_string()),
            Stop::Cut => lines.push(format!(
                "the file ends in a cut {after}: the ticks before it were replayed"
            )),
            Stop::Dropout { ms } => lines.push(format!(
                "the log has a gap ({ms} ms dropout) {after}: nothing after it can be shown"
            )),
            Stop::SequenceGap { expected, found } => lines.push(format!(
                "a tick record is missing {after}: expected number {expected}, found {found}: \
                 nothing after it can be shown"
            )),
            Stop::Damaged { why } => {
                lines.push(format!("the file is damaged {after}: {why}"));
            }
            Stop::Malformed { seq: Some(s), why } => {
                lines.push(format!("tick {s} is malformed: {why}"));
            }
            Stop::Malformed { seq: None, why } => {
                lines.push(format!("a tick record is malformed {after}: {why}"));
            }
        }
        lines
    }
}

/// Replays the bytes of a log.
pub fn replay(bytes: &[u8], options: &ReplayOptions) -> Result<ReplayReport, ReplayError> {
    let file = UlogFile::parse(bytes).map_err(ReplayError::Read)?;
    replay_file(&file, options)
}

fn profile_named(name: &str) -> Option<Profile> {
    [F405_LEAN, H743_FULL].into_iter().find(|p| p.name == name)
}

/// The state of a replay in progress.
struct Walk {
    core: Core<ROTORS>,
    keep_going: bool,
    report: ReplayReport,
}

/// Replays a parsed log.
pub fn replay_file(file: &UlogFile, options: &ReplayOptions) -> Result<ReplayReport, ReplayError> {
    match file.info_str("oxide_log_level") {
        Some("replay") => {}
        other => return Err(ReplayError::NotReplayLevel(other.map(str::to_string))),
    }
    let mut boots = file.records("oxide_boot");
    let (Some(boot), None) = (boots.next(), boots.next()) else {
        return Err(ReplayError::NoBoot(file.records("oxide_boot").count()));
    };
    let boot_time = boot.u64("timestamp");
    let boot = decode::decode_boot(&boot).map_err(ReplayError::Boot)?;
    for topic in ["oxide_vehicle", "oxide_param_us"] {
        if file.records(topic).any(|r| r.u64("timestamp") != boot_time) {
            return Err(ReplayError::StartTime(topic));
        }
    }
    let name = file.info_str("oxide_profile");
    let profile = name
        .and_then(profile_named)
        .ok_or_else(|| ReplayError::UnknownProfile(name.map(str::to_string)))?;
    let rates_match = boot.base_period_us == profile.base_period_us
        && boot.filter_period_ticks == profile.filter_period_ticks
        && profile
            .tasks
            .iter()
            .zip(&boot.task_period_ticks)
            .all(|(t, p)| t.period_ticks == *p);
    if !rates_match {
        return Err(ReplayError::ProfileMismatch(profile.name));
    }
    if usize::from(boot.rotors) != ROTORS {
        return Err(ReplayError::Rotors(boot.rotors));
    }
    let mut cfg: VehicleConfig<ROTORS> =
        decode::config_of(file, &profile).map_err(ReplayError::Config)?;
    apply_params(&mut cfg, &profile, &options.params).map_err(ReplayError::Override)?;
    let mut core = Core::new(cfg, profile).map_err(ReplayError::Core)?;
    core.report_param_load(boot.report);
    let mut walk = Walk {
        core,
        keep_going: options.keep_going,
        report: ReplayReport {
            profile: profile.name,
            ticks: 0,
            differing: 0,
            first_difference: None,
            last_tick: None,
            stop: Stop::End,
        },
    };
    walk.run(file);
    Ok(walk.report)
}

/// A tick whose records are all there, read.
struct Complete {
    tick: TickRecord,
    inputs: Inputs,
    out: OutRecord<ROTORS>,
    events: Vec<Event>,
}

/// A tick's group of records, read.
enum Group {
    /// Everything the tick needs is there.
    Complete(Box<Complete>),
    /// Something is missing (nothing is wrong with what is there).
    Incomplete,
    /// The records are damaged or contradict each other.
    Bad(Option<u64>, String),
}

fn read_group<'a>(tick: &RecordView<'a>, data: &[RecordView<'a>]) -> Group {
    let (tk, inputs) = match decode::inputs_from_records(tick, data.iter().copied()) {
        Ok(v) => v,
        Err(InputsError::Missing(_)) => return Group::Incomplete,
        Err(e) => return Group::Bad(decode::decode_tick(tick).ok().map(|t| t.seq), e.to_string()),
    };
    let bad = |why: String| Group::Bad(Some(tk.seq), why);
    let mut outs = data.iter().filter(|r| r.topic() == "oxide_out");
    let Some(out) = outs.next() else {
        return Group::Incomplete;
    };
    if outs.next().is_some() {
        return bad("two oxide_out records".to_string());
    }
    let out = match decode::decode_out::<ROTORS>(out) {
        Ok(o) => o,
        Err(e) => return bad(e.to_string()),
    };
    if out.t != tk.t {
        return bad(format!(
            "oxide_out has time {} in the tick at {}",
            out.t.0, tk.t.0
        ));
    }
    let mut events = Vec::new();
    for r in data.iter().filter(|r| r.topic() == "oxide_event") {
        match oxide_log::topics::decode_event(r) {
            Ok(e) => events.push(e),
            Err(e) => return bad(format!("oxide_event: {e}")),
        }
    }
    let want = usize::from(out.events);
    if data
        .iter()
        .filter(|r| r.topic() == "oxide_event")
        .any(|r| r.u64("timestamp") != Some(tk.t.0))
    {
        return bad(format!(
            "oxide_event has another time than the tick at {}",
            tk.t.0
        ));
    }
    if events.len() < want {
        return Group::Incomplete;
    }
    if events.len() > want {
        return bad(format!(
            "{} oxide_event records, oxide_out says {want}",
            events.len()
        ));
    }
    Group::Complete(Box::new(Complete {
        tick: tk,
        inputs,
        out,
        events,
    }))
}

impl Walk {
    fn run(&mut self, file: &UlogFile) {
        // Where each tick record is among the items.
        let ticks: Vec<usize> = file
            .items
            .iter()
            .enumerate()
            .filter_map(|(i, item)| match item {
                Item::Data(r) if file.view(r).topic() == "oxide_in_tick" => Some(i),
                _ => None,
            })
            .collect();
        // A dropout before the first tick (after the start records).
        let first = ticks.first().copied().unwrap_or(file.items.len());
        if let Some(ms) = file.items.iter().take(first).find_map(dropout) {
            self.report.stop = Stop::Dropout { ms };
            return;
        }
        // Replay records belong to a tick's group; before the first tick only
        // the start records (vehicle, parameters, boot) may be.
        for item in file.items.iter().take(first) {
            if let Item::Data(r) = item {
                let topic = file.view(r).topic();
                if topic.starts_with("oxide_in_") || topic == "oxide_out" || topic == "oxide_event"
                {
                    self.report.stop = Stop::Malformed {
                        seq: None,
                        why: format!("{topic} record before the first tick"),
                    };
                    return;
                }
            }
        }
        if ticks.is_empty() {
            self.report.stop = if file.truncated.is_some() {
                cut_stop(file)
            } else {
                Stop::Malformed {
                    seq: None,
                    why: "the file has no ticks".to_string(),
                }
            };
            return;
        }
        let mut expected: u64 = 0;
        for (n, &at) in ticks.iter().enumerate() {
            let end = ticks.get(n + 1).copied().unwrap_or(file.items.len());
            let Some(Item::Data(rec)) = file.items.get(at) else {
                return;
            };
            let tick_view = file.view(rec);
            let items = file.items.get(at + 1..end).unwrap_or(&[]);
            // The group ends at the first dropout: what follows it is not
            // known to belong to this tick.
            let gap = items.iter().position(|i| matches!(i, Item::Dropout { .. }));
            let visible = items.get(..gap.unwrap_or(items.len())).unwrap_or(&[]);
            let data: Vec<RecordView<'_>> = visible
                .iter()
                .filter_map(|i| match i {
                    Item::Data(r) => Some(file.view(r)),
                    _ => None,
                })
                .collect();
            let dropout_ms = gap.and_then(|g| items.get(g)).and_then(dropout);
            let last = n + 1 == ticks.len();
            // The sequence number comes first: a gap is a gap whatever the
            // rest of the group looks like.
            match decode::decode_tick(&tick_view) {
                Ok(t) if t.seq != expected => {
                    self.report.stop = Stop::SequenceGap {
                        expected,
                        found: t.seq,
                    };
                    return;
                }
                Ok(_) => {}
                Err(e) => {
                    self.report.stop = Stop::Malformed {
                        seq: None,
                        why: e.to_string(),
                    };
                    return;
                }
            }
            match read_group(&tick_view, &data) {
                Group::Complete(c) => {
                    self.tick(&c.tick, &c.inputs, &c.out, &c.events);
                    expected = c.tick.seq.wrapping_add(1);
                    if !self.keep_going && self.report.differing > 0 {
                        self.report.stop = Stop::Difference;
                        return;
                    }
                }
                Group::Incomplete => {
                    self.report.stop = match dropout_ms {
                        Some(ms) => Stop::Dropout { ms },
                        None if last => cut_stop(file),
                        None => Stop::Malformed {
                            seq: Some(expected),
                            why: "records of the tick are missing".to_string(),
                        },
                    };
                    return;
                }
                Group::Bad(seq, why) => {
                    self.report.stop = Stop::Malformed { seq, why };
                    return;
                }
            }
            if let Some(ms) = dropout_ms {
                self.report.stop = Stop::Dropout { ms };
                return;
            }
        }
        if file.truncated.is_some() {
            self.report.stop = cut_stop(file);
        }
    }

    /// Replays one tick and compares.
    fn tick(
        &mut self,
        tick: &TickRecord,
        inputs: &Inputs,
        out: &OutRecord<ROTORS>,
        events: &[Event],
    ) {
        if tick.saved {
            self.core.clear_unsaved();
        }
        let got: Outputs<ROTORS> = self.core.tick(tick.t, inputs);
        self.report.ticks += 1;
        self.report.last_tick = Some((tick.seq, tick.t.0));
        if let Some(d) = compare(tick.seq, tick.t, out, events, &got) {
            self.report.differing += 1;
            if self.report.first_difference.is_none() {
                self.report.first_difference = Some(d);
            }
        }
    }
}

/// How the walk ends when the file ends in the middle of something: a cut,
/// as a power loss leaves it, **if** the message the end of the file cuts could
/// be one this file holds. A message that declares more bytes than any
/// complete message of the data section is not a cut but a damaged size (a
/// flipped bit in a size makes the reader lose its place and run into the
/// end), so the replay says so and exits 1.
fn cut_stop(file: &UlogFile) -> Stop {
    match file.truncated {
        Some(t) if t.needed > 3 && t.needed - 3 > file.largest_data_message => Stop::Damaged {
            why: format!(
                "the last message, at byte {}, declares {} bytes but the file ends {} bytes \
                 into it, and no complete message of the file has more than {}: damage, \
                 not a cut",
                t.offset,
                t.needed - 3,
                t.available,
                file.largest_data_message
            ),
        },
        _ => Stop::Cut,
    }
}

fn dropout(i: &Item) -> Option<u16> {
    match i {
        Item::Dropout { ms } => Some(*ms),
        _ => None,
    }
}

/// The first field of the replayed outputs that differs from the recorded
/// ones, in the order: `t_valid`, the motor commands, the status fields, the
/// events.
fn compare(
    seq: u64,
    t: Micros,
    rec: &OutRecord<ROTORS>,
    rec_events: &[Event],
    got: &Outputs<ROTORS>,
) -> Option<Difference> {
    let diff = |field: &str, recorded: String, replayed: String| Difference {
        seq,
        t_us: t.0,
        field: field.to_string(),
        recorded,
        replayed,
        recorded_bits: None,
        replayed_bits: None,
    };
    let float = |field: &str, a: f32, b: f32| {
        (a.to_bits() != b.to_bits()).then(|| Difference {
            recorded_bits: Some(a.to_bits()),
            replayed_bits: Some(b.to_bits()),
            ..diff(field, format!("{a:?}"), format!("{b:?}"))
        })
    };
    fn ne<T: PartialEq + fmt::Debug>(
        field: &str,
        a: T,
        b: T,
        mk: &dyn Fn(&str, String, String) -> Difference,
    ) -> Option<Difference> {
        (a != b).then(|| mk(field, format!("{a:?}"), format!("{b:?}")))
    }
    if let Some(d) = ne("t_valid", rec.t_valid.0, got.t_valid.0, &diff) {
        return Some(d);
    }
    for (i, (a, b)) in rec.motors.iter().zip(&got.motors).enumerate() {
        if let Some(d) = float(&format!("motors[{i}]"), *a, *b) {
            return Some(d);
        }
    }
    let (a, b) = (&rec.status, &got.status);
    let status = [
        ne("armed", a.armed, b.armed, &diff),
        ne("killed", a.killed, b.killed, &diff),
        ne("mode", a.mode, b.mode, &diff),
        ne("saturated", a.saturated, b.saturated, &diff),
        ne("timing_faults", a.timing_faults, b.timing_faults, &diff),
        ne("no_estimate", a.no_estimate, b.no_estimate, &diff),
        ne("aligned", a.estimator.aligned, b.estimator.aligned, &diff),
        ne(
            "gps_aiding",
            a.estimator.gps_aiding,
            b.estimator.gps_aiding,
            &diff,
        ),
        ne(
            "rejected",
            a.estimator.rejected,
            b.estimator.rejected,
            &diff,
        ),
        ne("resets", a.estimator.resets, b.estimator.resets, &diff),
        ne("nav", a.nav, b.nav, &diff),
        ne("valid_attitude", a.valid.attitude, b.valid.attitude, &diff),
        ne("valid_heading", a.valid.heading, b.valid.heading, &diff),
        ne("valid_altitude", a.valid.altitude, b.valid.altitude, &diff),
        ne("valid_velocity", a.valid.velocity, b.valid.velocity, &diff),
        ne("valid_position", a.valid.position, b.valid.position, &diff),
        ne("reasons", a.reasons, b.reasons, &diff),
        ne("action", a.action, b.action, &diff),
        ne("prearm", a.prearm, b.prearm, &diff),
        ne("home_set", a.home_set, b.home_set, &diff),
        ne("params_unsaved", a.params_unsaved, b.params_unsaved, &diff),
    ];
    if let Some(d) = status.into_iter().flatten().next() {
        return Some(d);
    }
    let replayed: Vec<Event> = got.events.iter().copied().collect();
    if let Some(d) = ne("events", rec_events.len(), replayed.len(), &diff) {
        return Some(d);
    }
    for (i, (a, b)) in rec_events.iter().zip(&replayed).enumerate() {
        if let Some(d) = ne(&format!("events[{i}]"), *a, *b, &diff) {
            return Some(d);
        }
    }
    // The rebuilt outputs must be what `same_bits` calls equal.
    let rebuilt = rec.clone().into_outputs(events_list(rec_events));
    if !rebuilt.same_bits(got) {
        return Some(diff("outputs", "(as recorded)".into(), "(differ)".into()));
    }
    None
}

fn events_list(events: &[Event]) -> oxide_core::io::EventList {
    let mut list = oxide_core::io::EventList::new();
    for e in events {
        list.push(*e);
    }
    list
}
