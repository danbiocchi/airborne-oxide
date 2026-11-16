// SPDX-License-Identifier: AGPL-3.0-or-later
//! The simulator's log (ADR-0005 §7): a [`Recorder`] and a writer that a
//! [`Simulation`](crate::Simulation) feeds after every tick.
//!
//! The recorder's ring is drained into the writer after every tick, so
//! nothing drops. A writer that fails does not stop or change the flight: the
//! first error is kept, the rest of the log is discarded, and
//! [`Simulation::finish_log`](crate::Simulation::finish_log) returns the
//! error.

use std::fmt;
use std::io::Write;
use std::sync::{Arc, Mutex, PoisonError};

use oxide_core::config::VehicleConfig;
use oxide_core::flight::Telemetry;
use oxide_core::io::{Inputs, Outputs};
use oxide_core::params::LoadReport;
use oxide_core::schedule::Profile;
use oxide_core::time::Micros;
use oxide_log::header::BuildInfo;
pub use oxide_log::header::Level;
pub use oxide_log::logger::Stats;
use oxide_log::recorder::{Boot, Recorder, StartError};

/// Bytes of the recorder's ring (about 66 kB, kept on the heap).
pub const RING_BYTES: usize = 65_536;

/// Where the log goes: any writer that can be sent to another thread.
pub type LogWriter = Box<dyn Write + Send>;

/// Who built this: `ver_sw` and `ver_hw` of the log's header.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LogBuild {
    /// The git hash of the software.
    pub git_hash: String,
    /// The board: `SIM` in the simulator.
    pub board: String,
}

impl LogBuild {
    /// This git hash on the board `SIM`.
    pub fn sim(git_hash: impl Into<String>) -> Self {
        Self {
            git_hash: git_hash.into(),
            board: "SIM".to_string(),
        }
    }
}

/// Why a log could not be started or finished.
#[derive(Debug)]
pub enum LogError {
    /// The simulation has already run a tick.
    AfterFirstTick,
    /// The simulation already has a log.
    AlreadyStarted,
    /// There is no log to finish.
    NotStarted,
    /// The configuration could never replay (the text names the parameter).
    NotReplayable(String),
    /// The header did not fit the recorder's format limits.
    Format(oxide_log::Error),
    /// The writer failed (the first error, kept).
    Io(std::io::Error),
}

impl fmt::Display for LogError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            LogError::AfterFirstTick => {
                write!(f, "a log must be started before the first tick")
            }
            LogError::AlreadyStarted => write!(f, "the simulation already has a log"),
            LogError::NotStarted => write!(f, "the simulation has no log"),
            LogError::NotReplayable(why) => write!(f, "cannot log at the replay level: {why}"),
            LogError::Format(e) => write!(f, "log header: {e:?}"),
            LogError::Io(e) => write!(f, "log: {e}"),
        }
    }
}

impl std::error::Error for LogError {}

/// A writer into memory that can be read while the simulation still owns it:
/// clones share one buffer.
#[derive(Clone, Debug, Default)]
pub struct SharedBuffer(Arc<Mutex<Vec<u8>>>);

impl SharedBuffer {
    /// An empty buffer.
    pub fn new() -> Self {
        Self::default()
    }

    /// A copy of the bytes written so far.
    pub fn bytes(&self) -> Vec<u8> {
        self.0
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .clone()
    }
}

impl Write for SharedBuffer {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        self.0
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .extend_from_slice(buf);
        Ok(buf.len())
    }

    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}

/// A started log: the recorder, the writer and the first error.
pub(crate) struct FlightLog {
    recorder: Box<Recorder<RING_BYTES>>,
    writer: LogWriter,
    error: Option<std::io::Error>,
    /// `Status::params_unsaved` of the previous tick's outputs: what the
    /// core held when the tick began, unless the store's saved call came in
    /// between.
    was_unsaved: bool,
}

impl FlightLog {
    /// Writes the header to `writer` and makes the log ready.
    pub(crate) fn start<const N: usize>(
        level: Level,
        build: &LogBuild,
        profile: &Profile,
        report: &LoadReport,
        cfg: &VehicleConfig<N>,
        mut writer: LogWriter,
    ) -> Result<Self, LogError> {
        let mut recorder = Box::new(Recorder::<RING_BYTES>::new());
        let mut head: Vec<u8> = Vec::new();
        let info = BuildInfo {
            git_hash: &build.git_hash,
            board: &build.board,
        };
        recorder
            .start(
                &mut head,
                Micros::ZERO,
                info,
                Boot {
                    profile,
                    report: *report,
                },
                level,
                cfg,
            )
            .map_err(|e| match e {
                StartError::Write(e) => LogError::Format(e),
                other => LogError::NotReplayable(describe_start(&other)),
            })?;
        writer.write_all(&head).map_err(LogError::Io)?;
        Ok(Self {
            recorder,
            writer,
            error: None,
            was_unsaved: false,
        })
    }

    /// Hands the tick to the recorder and drains the ring to the writer.
    ///
    /// `unsaved_before` is `Core::params_unsaved()` read just before the tick.
    /// The store's one call between ticks, `Core::clear_unsaved`, is not
    /// intercepted but detected: `params_unsaved` was true in the previous
    /// tick's status and is false now, before this tick ran. Nothing else
    /// clears it between ticks (`Core::params_unsaved` is cleared only by
    /// `clear_unsaved`, and set again only by a tick), and `clear_unsaved`
    /// when it is already false changes nothing and need not be replayed.
    pub(crate) fn record<const N: usize>(
        &mut self,
        now: Micros,
        unsaved_before: bool,
        inputs: &Inputs,
        outputs: &Outputs<N>,
        telemetry: &Telemetry,
        cfg: &VehicleConfig<N>,
    ) {
        if self.was_unsaved && !unsaved_before {
            self.recorder.note_saved();
        }
        self.was_unsaved = outputs.status.params_unsaved;
        self.recorder.record(now, inputs, outputs, telemetry, cfg);
        let (writer, error) = (&mut self.writer, &mut self.error);
        self.recorder.drain_with(|chunk| {
            // After the first failure the rest is discarded: the flight does
            // not wait for a writer that is gone.
            if error.is_none()
                && let Err(e) = writer.write_all(chunk)
            {
                *error = Some(e);
            }
            chunk.len()
        });
    }

    pub(crate) fn stats(&self) -> Stats {
        self.recorder.stats()
    }

    /// Flushes and returns the writer, or the first error.
    pub(crate) fn finish(mut self) -> Result<LogWriter, LogError> {
        if let Some(e) = self.error.take() {
            return Err(LogError::Io(e));
        }
        self.writer.flush().map_err(LogError::Io)?;
        Ok(self.writer)
    }
}

/// Why a replay-level log of this configuration is refused, naming the
/// parameter.
fn describe_start(e: &StartError) -> String {
    use oxide_core::params::{self, ParamId};
    use oxide_log::replay::RebuildError;
    let name = |id: ParamId| params::PARAMS.get(id.index()).map_or("?", |d| d.name);
    match e {
        StartError::Write(e) => format!("{e:?}"),
        StartError::TooManyRotors(t) => {
            format!("{} rotors, a record carries at most {}", t.got, t.max)
        }
        StartError::NotReplayable(RebuildError::NotExact(id)) => format!(
            "parameter {} does not survive being logged (its value is not the exact f32 \
             number of seconds it reads back as)",
            name(*id)
        ),
        StartError::NotReplayable(RebuildError::Refused { id, why }) => {
            format!("parameter {} is refused by the core: {why:?}", name(*id))
        }
        StartError::NotReplayable(other) => format!("{other:?}"),
    }
}
