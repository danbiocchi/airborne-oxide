// SPDX-License-Identifier: AGPL-3.0-or-later
//! Many flights (ADR-0006 section 9): each run is a child process of its own,
//! `J` at a time, each on its own instance number and, where `taskset`
//! exists, on its own core. The rows come out in index order whatever order
//! the children finished in.
//!
//! The batch knows nothing of how a child is made: the caller gives it a
//! function from a [`Job`] to a [`Command`], and the child must write the
//! `row.csv` of [`crate::record`] into the job's directory and exit with 0
//! (flown and judged), or 2 (did not start, or the bridge failed). The wall
//! clock decides one thing only: a child that runs past the limit is ended.
//!
//! A run whose child did not produce a row is flown once more, in a new
//! directory of its own; the first attempt's directory is kept as
//! `run-NNNN.failed`. When the second attempt fails too, the batch flies no
//! more runs (the ones running finish), writes what it has and says so.

use std::path::{Path, PathBuf};
use std::process::{Child, Command, ExitStatus, Stdio};
use std::sync::Mutex;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::time::{Duration, Instant};

use crate::record::{self, Row};
use crate::runner::Autopilot;

/// One attempt at one run.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Job {
    pub index: u64,
    /// The program's instance number: no other child running at the same
    /// time has it.
    pub instance: u8,
    /// 1 for the first attempt, 2 for the second.
    pub attempt: u32,
    /// The child's own directory (it exists, and holds `child.log`).
    pub out: PathBuf,
}

/// What the batch tells its caller as it goes.
#[derive(Clone, Debug, PartialEq)]
pub enum Event {
    /// A child was started.
    Spawned {
        index: u64,
        attempt: u32,
        pid: u32,
        instance: u8,
        core: Option<usize>,
    },
    /// An attempt failed; `will_retry` is whether a second one follows.
    Failed {
        index: u64,
        attempt: u32,
        reason: String,
        will_retry: bool,
    },
    /// A run has its row.
    Done { index: u64, row: Row },
}

/// How the children are pinned.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Pin {
    Off,
    /// `taskset -c N`, N counted from `first_core`, one core to a worker.
    Taskset {
        first_core: usize,
    },
}

/// What a batch flies.
#[derive(Clone, Debug)]
pub struct BatchConfig {
    pub scenario: String,
    pub autopilot: Autopilot,
    /// The runs, in index order, with the seed of each.
    pub runs: Vec<(u64, u64)>,
    /// Children at a time.
    pub jobs: usize,
    /// The instance numbers, inclusive. Worker `w` flies on `first + w`.
    pub instances: (u8, u8),
    /// The wall-clock limit of one child.
    pub limit: Duration,
    /// The batch's directory: `flights.csv` and `run-NNNN/` go here.
    pub out: PathBuf,
    pub pin: Pin,
}

/// What a batch did.
#[derive(Clone, Debug)]
pub struct BatchResult {
    /// One row for each run that was flown, in index order.
    pub rows: Vec<Row>,
    /// The index of the run whose second attempt failed, which stopped the
    /// batch.
    pub stopped_at: Option<u64>,
}

/// Why a batch could not be run at all.
#[derive(Debug)]
pub struct BatchError(pub String);

impl core::fmt::Display for BatchError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for BatchError {}

/// The directory of run `index`.
pub fn run_dir(out: &Path, index: u64) -> PathBuf {
    out.join(format!("run-{index:04}"))
}

/// True when the `taskset` command can be found on the path.
pub fn taskset_available() -> bool {
    if !cfg!(unix) {
        return false;
    }
    std::env::var_os("PATH")
        .is_some_and(|paths| std::env::split_paths(&paths).any(|d| d.join("taskset").is_file()))
}

/// `command` run under `taskset -c core`.
pub fn pinned(command: &Command, core: usize) -> Command {
    let mut out = Command::new("taskset");
    out.arg("-c").arg(core.to_string());
    out.arg(command.get_program());
    out.args(command.get_args());
    for (k, v) in command.get_envs() {
        match v {
            Some(v) => out.env(k, v),
            None => out.env_remove(k),
        };
    }
    if let Some(d) = command.get_current_dir() {
        out.current_dir(d);
    }
    out
}

/// Ends the simulator program a killed child left running, found by the
/// `sitl.pid` the bridge writes in its own directory (`dir/sitl`): only when
/// the process with that pid is the program named there.
pub fn end_orphan(dir: &Path) -> bool {
    let Ok(text) = std::fs::read_to_string(dir.join("sitl").join("sitl.pid")) else {
        return false;
    };
    let mut lines = text.lines();
    let (Some(pid), Some(binary)) = (lines.next(), lines.next()) else {
        return false;
    };
    if pid.parse::<u32>().is_err() {
        return false;
    }
    end_process_named(pid, binary)
}

#[cfg(unix)]
fn end_process_named(pid: &str, binary: &str) -> bool {
    let Ok(out) = Command::new("ps").args(["-p", pid, "-o", "args="]).output() else {
        return false;
    };
    if !String::from_utf8_lossy(&out.stdout).contains(binary) {
        return false;
    }
    Command::new("kill")
        .args(["-KILL", pid])
        .status()
        .is_ok_and(|s| s.success())
}

#[cfg(not(unix))]
fn end_process_named(pid: &str, binary: &str) -> bool {
    let name = Path::new(binary)
        .file_name()
        .map(|n| n.to_string_lossy().into_owned())
        .unwrap_or_default();
    let Ok(out) = Command::new("tasklist")
        .args(["/FI", &format!("PID eq {pid}"), "/NH"])
        .output()
    else {
        return false;
    };
    if name.is_empty() || !String::from_utf8_lossy(&out.stdout).contains(&name) {
        return false;
    }
    Command::new("taskkill")
        .args(["/F", "/PID", pid])
        .status()
        .is_ok_and(|s| s.success())
}

/// How an attempt ended, in words.
fn describe(status: ExitStatus) -> String {
    #[cfg(unix)]
    {
        use std::os::unix::process::ExitStatusExt;
        if let Some(sig) = status.signal() {
            return format!("the child was ended by signal {sig}");
        }
    }
    match status.code() {
        Some(c) => format!("the child exited with code {c}"),
        None => "the child ended without an exit code".to_string(),
    }
}

/// Waits for `child` up to `limit`; ends it at the limit. `None` is the
/// limit.
fn wait_limited(child: &mut Child, limit: Duration) -> std::io::Result<Option<ExitStatus>> {
    let start = Instant::now();
    loop {
        if let Some(status) = child.try_wait()? {
            return Ok(Some(status));
        }
        if start.elapsed() >= limit {
            let _ = child.kill();
            let _ = child.wait();
            return Ok(None);
        }
        std::thread::sleep(Duration::from_millis(20));
    }
}

struct Shared<'a> {
    cfg: &'a BatchConfig,
    launch: &'a (dyn Fn(&Job) -> Command + Sync),
    observe: &'a (dyn Fn(&Event) + Sync),
    next: AtomicUsize,
    stop: AtomicBool,
    stopped_at: Mutex<Option<u64>>,
    rows: Mutex<Vec<Option<Row>>>,
}

impl Shared<'_> {
    /// One attempt: the row, or why there is none.
    fn attempt(&self, job: &Job, worker: usize) -> Result<Row, String> {
        std::fs::create_dir_all(&job.out)
            .map_err(|e| format!("cannot make {}: {e}", job.out.display()))?;
        let log = std::fs::File::create(job.out.join("child.log"))
            .map_err(|e| format!("cannot write the child's log: {e}"))?;
        let log2 = log
            .try_clone()
            .map_err(|e| format!("cannot write the child's log: {e}"))?;
        let mut command = (self.launch)(job);
        let core = match self.cfg.pin {
            Pin::Off => None,
            Pin::Taskset { first_core } => {
                let n = std::thread::available_parallelism().map_or(1, usize::from);
                Some((first_core + worker) % n)
            }
        };
        if let Some(core) = core {
            command = pinned(&command, core);
        }
        command
            .stdin(Stdio::null())
            .stdout(Stdio::from(log))
            .stderr(Stdio::from(log2));
        let mut child = command
            .spawn()
            .map_err(|e| format!("cannot start the child: {e}"))?;
        (self.observe)(&Event::Spawned {
            index: job.index,
            attempt: job.attempt,
            pid: child.id(),
            instance: job.instance,
            core,
        });
        let status = match wait_limited(&mut child, self.cfg.limit) {
            Ok(s) => s,
            Err(e) => {
                let _ = child.kill();
                let _ = child.wait();
                end_orphan(&job.out);
                return Err(format!("cannot wait for the child: {e}"));
            }
        };
        let Some(status) = status else {
            end_orphan(&job.out);
            return Err(format!(
                "the wall limit of {} s was exceeded and the child was ended",
                self.cfg.limit.as_secs_f64()
            ));
        };
        if !status.success() {
            end_orphan(&job.out);
        }
        let row = record::read_row(&job.out.join("row.csv"));
        match (status.code(), row) {
            (Some(0), Ok(row)) if row.get("started") == "yes" => Ok(row),
            // The child says why in the row it left.
            (Some(2), Ok(row)) if !row.get("note").is_empty() => Err(row.get("note").to_string()),
            (_, Ok(row)) if row.get("started") == "no" && !row.get("note").is_empty() => {
                Err(format!("{}: {}", describe(status), row.get("note")))
            }
            (Some(0), Ok(_)) => Err("the child's row says the flight did not start".into()),
            (Some(0), Err(e)) => Err(format!("the child left no usable row: {e}")),
            _ => Err(describe(status)),
        }
    }

    /// Both attempts of one run, then the row.
    fn run(&self, position: usize, worker: usize) {
        let (index, seed) = match self.cfg.runs.get(position) {
            Some(r) => *r,
            None => return,
        };
        let instance = self.cfg.instances.0 + u8::try_from(worker).unwrap_or(0);
        let dir = run_dir(&self.cfg.out, index);
        let job = |attempt, out: &Path| Job {
            index,
            instance,
            attempt,
            out: out.to_path_buf(),
        };
        let start = Instant::now();
        let first = self.attempt(&job(1, &dir), worker);
        let row = match first {
            Ok(row) => row,
            Err(r1) => {
                (self.observe)(&Event::Failed {
                    index,
                    attempt: 1,
                    reason: r1.clone(),
                    will_retry: true,
                });
                let mut failed = dir.clone().into_os_string();
                failed.push(".failed");
                let failed = PathBuf::from(failed);
                let _ = std::fs::remove_dir_all(&failed);
                let _ = std::fs::rename(&dir, &failed);
                match self.attempt(&job(2, &dir), worker) {
                    Ok(mut row) => {
                        row.set("retried", "yes");
                        let note = format!("first attempt failed: {r1}; {}", row.get("note"));
                        row.set("note", &note);
                        row
                    }
                    Err(r2) => {
                        (self.observe)(&Event::Failed {
                            index,
                            attempt: 2,
                            reason: r2.clone(),
                            will_retry: false,
                        });
                        self.stop.store(true, Ordering::SeqCst);
                        if let Ok(mut s) = self.stopped_at.lock() {
                            s.get_or_insert(index);
                        }
                        let mut row = Row::not_started(
                            &self.cfg.scenario,
                            index,
                            seed,
                            &self.cfg.autopilot,
                            start.elapsed().as_secs_f64(),
                            &format!("{r2}; first attempt: {r1}"),
                        );
                        row.set("retried", "yes");
                        row
                    }
                }
            }
        };
        (self.observe)(&Event::Done {
            index,
            row: row.clone(),
        });
        if let Ok(mut rows) = self.rows.lock()
            && let Some(slot) = rows.get_mut(position)
        {
            *slot = Some(row);
        }
    }
}

/// Flies every run of `cfg`, `cfg.jobs` at a time, and writes
/// `out/flights.csv`: the header, then one row for each run flown, in index
/// order. `launch` makes the child's command for a job; `observe` hears what
/// happens as it happens (from several threads).
pub fn run_batch(
    cfg: &BatchConfig,
    launch: &(dyn Fn(&Job) -> Command + Sync),
    observe: &(dyn Fn(&Event) + Sync),
) -> Result<BatchResult, BatchError> {
    let (lo, hi) = cfg.instances;
    let available = usize::from(hi.saturating_sub(lo)) + 1;
    if cfg.jobs == 0 || cfg.jobs > available {
        return Err(BatchError(format!(
            "{} jobs need {} instance numbers of their own and the range {lo}-{hi} has {available}",
            cfg.jobs, cfg.jobs
        )));
    }
    for (index, _) in &cfg.runs {
        let d = run_dir(&cfg.out, *index);
        if d.exists() {
            return Err(BatchError(format!(
                "{} exists already: a batch needs a directory of its own",
                d.display()
            )));
        }
    }
    std::fs::create_dir_all(&cfg.out)
        .map_err(|e| BatchError(format!("cannot make {}: {e}", cfg.out.display())))?;
    let shared = Shared {
        cfg,
        launch,
        observe,
        next: AtomicUsize::new(0),
        stop: AtomicBool::new(false),
        stopped_at: Mutex::new(None),
        rows: Mutex::new(vec![None; cfg.runs.len()]),
    };
    std::thread::scope(|scope| {
        for worker in 0..cfg.jobs.min(cfg.runs.len()) {
            let shared = &shared;
            scope.spawn(move || {
                loop {
                    if shared.stop.load(Ordering::SeqCst) {
                        break;
                    }
                    let position = shared.next.fetch_add(1, Ordering::SeqCst);
                    if position >= shared.cfg.runs.len() {
                        break;
                    }
                    shared.run(position, worker);
                }
            });
        }
    });
    let rows: Vec<Row> = shared
        .rows
        .into_inner()
        .unwrap_or_default()
        .into_iter()
        .flatten()
        .collect();
    let stopped_at = shared.stopped_at.into_inner().ok().flatten();
    let mut text = record::header();
    text.push('\n');
    for r in &rows {
        text.push_str(&r.line());
        text.push('\n');
    }
    let path = cfg.out.join("flights.csv");
    std::fs::write(&path, text)
        .map_err(|e| BatchError(format!("cannot write {}: {e}", path.display())))?;
    Ok(BatchResult { rows, stopped_at })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_pinned_command_runs_the_same_program_under_taskset() {
        let mut c = Command::new("some-program");
        c.args(["fly", "--index", "3"])
            .env("A", "1")
            .env_remove("B");
        let p = pinned(&c, 2);
        assert_eq!(p.get_program(), "taskset");
        let args: Vec<_> = p
            .get_args()
            .map(|a| a.to_string_lossy().into_owned())
            .collect();
        assert_eq!(args, ["-c", "2", "some-program", "fly", "--index", "3"]);
        let envs: Vec<_> = p.get_envs().collect();
        assert!(
            envs.iter()
                .any(|(k, v)| *k == "A" && v.is_some_and(|v| v == "1"))
        );
        assert!(envs.iter().any(|(k, v)| *k == "B" && v.is_none()));
    }

    #[test]
    fn the_run_directories_are_numbered_to_four_digits() {
        assert_eq!(run_dir(Path::new("o"), 7), Path::new("o").join("run-0007"));
        assert_eq!(
            run_dir(Path::new("o"), 12345),
            Path::new("o").join("run-12345")
        );
    }

    #[test]
    fn an_orphan_is_ended_only_when_the_pid_is_the_named_program() {
        let dir = std::env::temp_dir().join(format!("oxide-orphan-test-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        // No file: nothing to do.
        assert!(!end_orphan(&dir));
        std::fs::create_dir_all(dir.join("sitl")).expect("dir");
        // Our own pid with a program name that is not ours: left alone.
        std::fs::write(
            dir.join("sitl").join("sitl.pid"),
            format!("{}\n/no/such/arducopter\n", std::process::id()),
        )
        .expect("write");
        assert!(!end_orphan(&dir));
        std::fs::write(dir.join("sitl").join("sitl.pid"), "not-a-pid\nx\n").expect("write");
        assert!(!end_orphan(&dir));
        let _ = std::fs::remove_dir_all(&dir);
    }
}
