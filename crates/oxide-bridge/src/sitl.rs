// SPDX-License-Identifier: AGPL-3.0-or-later
//! The simulator program as a child process (ADR-0006 §1, §9).
//!
//! [`Sitl::start`] checks the binary against [`PINNED`], makes a new directory
//! for the flight, writes the effective parameter file into it and starts the
//! program there. It neither binds the UDP port nor connects MAVLink: the
//! caller owns the [`crate::link::Link`] and the [`crate::mav::MavClient`].
//! The child is ended by [`Sitl::stop`] and, whatever happens, by `Drop`.

use core::fmt;
use std::io;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, ExitStatus, Stdio};
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Duration;

use crate::sha256;

/// The binary the bridge is written against.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Pinned {
    pub version: &'static str,
    pub commit: &'static str,
    pub size_bytes: u64,
    pub sha256: &'static str,
}

/// ArduCopter V4.7.2, the project's own prebuilt `SITL_x86_64_linux_gnu`
/// binary (ADR-0006).
pub const PINNED: Pinned = Pinned {
    version: "V4.7.2",
    commit: "fb773567896014da007bca58358704d4a0d8516f",
    size_bytes: 7_235_704,
    sha256: "caed35e4cb796ec06cac40b09566a18251771486b1cbac5e2bf16b7a5ed5045d",
};

/// The first UDP port (instance 0): the program sends motor packets here.
const JSON_PORT_BASE: u16 = 9002;
/// The first MAVLink TCP port (instance 0, serial 0).
const MAVLINK_PORT_BASE: u16 = 5760;
/// How many lines of the child's error output an error carries.
const TAIL_LINES: usize = 10;

/// The UDP port the program sends motor packets to for `instance`, which the
/// caller binds before starting it. `None` for an instance above 9.
pub fn json_port_of(instance: u8) -> Option<u16> {
    (instance <= 9).then(|| JSON_PORT_BASE + 10 * u16::from(instance))
}

/// The MAVLink TCP port of `serial` (0, 1 or 2) for `instance`: 5760, 5762
/// and 5763 plus 10 per instance. `None` for anything else.
pub fn mavlink_port_of(instance: u8, serial: u8) -> Option<u16> {
    let offset = match serial {
        0 => 0,
        1 => 2,
        2 => 3,
        _ => return None,
    };
    (instance <= 9).then(|| MAVLINK_PORT_BASE + 10 * u16::from(instance) + offset)
}

/// What to start and how.
#[derive(Clone, Debug)]
pub struct SitlConfig {
    /// The program's path.
    pub binary: PathBuf,
    /// Skip the checksum comparison (the digest found is kept anyway).
    pub accept_any_binary: bool,
    /// 0 to 9; shifts every port by 10 per instance.
    pub instance: u8,
    /// The simulator's own speed-up (its clock is ours; this only sets how
    /// long it sleeps).
    pub speedup: u32,
    /// `latitude,longitude,altitude_m,heading_deg`, as the program's `--home`.
    pub home: String,
    /// Unix seconds for its `--start-time`.
    pub start_time: u64,
    /// Parameter files, concatenated in this order.
    pub param_files: Vec<PathBuf>,
    /// Parameters written after the files, one `NAME VALUE` line each.
    pub extra_params: Vec<(String, f64)>,
    /// The directory under which the flight's own directory is made (made
    /// if it is missing).
    pub root: PathBuf,
    /// The flight directory's name under `root`; it must not exist yet. When
    /// `None`, a name that is new for this process and this instance.
    pub flight_name: Option<String>,
    /// Wall-clock limit for the start-up, which the caller applies to its
    /// first-packet and MAVLink waits.
    pub startup_limit: Duration,
}

impl SitlConfig {
    pub fn new(binary: impl Into<PathBuf>, root: impl Into<PathBuf>) -> Self {
        Self {
            binary: binary.into(),
            accept_any_binary: false,
            instance: 0,
            speedup: 100,
            home: "-35.363261,149.165230,584,0".to_string(),
            start_time: 1_700_000_000,
            param_files: Vec::new(),
            extra_params: Vec::new(),
            root: root.into(),
            flight_name: None,
            startup_limit: Duration::from_secs(20),
        }
    }
}

/// Why the program was not started, or has ended.
#[derive(Debug)]
pub enum SitlError {
    /// The instance number is above 9.
    BadInstance { instance: u8 },
    /// There is no file at the binary's path.
    BinaryMissing { path: PathBuf },
    /// The binary could not be read.
    BinaryUnreadable { path: PathBuf, source: io::Error },
    /// The binary is not the pinned one.
    ChecksumMismatch {
        path: PathBuf,
        found: String,
        expected: &'static str,
    },
    /// The flight's directory exists already; a directory is never reused.
    DirExists { path: PathBuf },
    /// The flight's directory could not be made.
    Dir { path: PathBuf, source: io::Error },
    /// A parameter file could not be read.
    ParamFileUnreadable { path: PathBuf, source: io::Error },
    /// A parameter name or value would break the `NAME VALUE` line format.
    BadParam { name: String, reason: &'static str },
    /// The effective parameter file or an output file could not be written.
    Write { path: PathBuf, source: io::Error },
    /// The program could not be started.
    Spawn { path: PathBuf, source: io::Error },
    /// The program could not be waited for or ended.
    Wait { source: io::Error },
    /// The program has ended.
    Exited {
        status: ExitStatus,
        stderr_tail: String,
    },
}

impl fmt::Display for SitlError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::BadInstance { instance } => {
                write!(f, "instance {instance} is not between 0 and 9")
            }
            Self::BinaryMissing { path } => {
                write!(f, "the simulator binary {} does not exist", path.display())
            }
            Self::BinaryUnreadable { path, source } => {
                write!(
                    f,
                    "the simulator binary {} cannot be read: {source}",
                    path.display()
                )
            }
            Self::ChecksumMismatch {
                path,
                found,
                expected,
            } => write!(
                f,
                "the simulator binary {} has SHA-256 {found}, not the pinned {expected}",
                path.display()
            ),
            Self::DirExists { path } => write!(
                f,
                "the flight directory {} exists already and is never reused",
                path.display()
            ),
            Self::Dir { path, source } => {
                write!(f, "cannot make the directory {}: {source}", path.display())
            }
            Self::ParamFileUnreadable { path, source } => {
                write!(
                    f,
                    "cannot read the parameter file {}: {source}",
                    path.display()
                )
            }
            Self::BadParam { name, reason } => {
                write!(f, "parameter {name:?} is refused: {reason}")
            }
            Self::Write { path, source } => {
                write!(f, "cannot write {}: {source}", path.display())
            }
            Self::Spawn { path, source } => {
                write!(f, "cannot start {}: {source}", path.display())
            }
            Self::Wait { source } => write!(f, "cannot wait for the simulator process: {source}"),
            Self::Exited {
                status,
                stderr_tail,
            } => write!(
                f,
                "the simulator process has ended ({status}); the end of its error output: {stderr_tail:?}"
            ),
        }
    }
}

impl std::error::Error for SitlError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::BinaryUnreadable { source, .. }
            | Self::Dir { source, .. }
            | Self::ParamFileUnreadable { source, .. }
            | Self::Write { source, .. }
            | Self::Spawn { source, .. }
            | Self::Wait { source } => Some(source),
            _ => None,
        }
    }
}

/// What is known when the program is stopped.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SitlExit {
    /// How it ended.
    pub status: ExitStatus,
    /// It had ended on its own before `stop` was called.
    pub ended_before_stop: bool,
    /// The last lines of its error output.
    pub stderr_tail: String,
}

/// The running program.
#[derive(Debug)]
pub struct Sitl {
    child: Option<Child>,
    instance: u8,
    dir: PathBuf,
    binary_sha256: String,
    params_sha256: String,
    command_line: Vec<String>,
}

/// Why a parameter name is refused, if it is: it must be non-empty, printable
/// ASCII, with no space and no `#`.
fn check_name(name: &str) -> Result<(), SitlError> {
    let bad = |reason| SitlError::BadParam {
        name: name.to_string(),
        reason,
    };
    if name.is_empty() {
        return Err(bad("the name is empty"));
    }
    if !name.bytes().all(|b| b.is_ascii_graphic() && b != b'#') {
        return Err(bad(
            "the name holds whitespace, a control or non-ASCII character, or `#`",
        ));
    }
    Ok(())
}

/// The effective parameter file: the given files in order (a newline is
/// added after a file that lacks one), then one `NAME VALUE` line per extra
/// parameter. A name that would break the line format, or a value that is not
/// finite, is refused.
pub fn effective_params(files: &[Vec<u8>], extra: &[(String, f64)]) -> Result<Vec<u8>, SitlError> {
    let mut out = Vec::new();
    for f in files {
        out.extend_from_slice(f);
        if !f.is_empty() && f.last() != Some(&b'\n') {
            out.push(b'\n');
        }
    }
    for (name, value) in extra {
        check_name(name)?;
        if !value.is_finite() {
            return Err(SitlError::BadParam {
                name: name.clone(),
                reason: "the value is not a finite number",
            });
        }
        out.extend_from_slice(format!("{name} {value}\n").as_bytes());
    }
    Ok(out)
}

/// The last `TAIL_LINES` lines of a file, empty if it cannot be read.
fn tail_of(path: &Path) -> String {
    let text = std::fs::read(path)
        .map(|b| String::from_utf8_lossy(&b).into_owned())
        .unwrap_or_default();
    let lines: Vec<&str> = text.lines().collect();
    let from = lines.len().saturating_sub(TAIL_LINES);
    lines.get(from..).unwrap_or(&[]).join("\n")
}

/// Starts the process; a Linux executable that was written a moment ago can
/// report "text file busy" while another thread forks, so that is retried.
fn spawn(cmd: &mut Command, path: &Path) -> Result<Child, SitlError> {
    let mut tries = 0;
    loop {
        match cmd.spawn() {
            Ok(c) => return Ok(c),
            Err(e) if is_text_busy(&e) && tries < 50 => {
                tries += 1;
                std::thread::sleep(Duration::from_millis(20));
            }
            Err(source) => {
                return Err(SitlError::Spawn {
                    path: path.to_path_buf(),
                    source,
                });
            }
        }
    }
}

fn is_text_busy(e: &io::Error) -> bool {
    // ETXTBSY is 26 on Linux.
    cfg!(unix) && e.raw_os_error() == Some(26)
}

fn absolute(p: &Path) -> PathBuf {
    std::path::absolute(p).unwrap_or_else(|_| p.to_path_buf())
}

static FLIGHTS: AtomicU64 = AtomicU64::new(0);

impl Sitl {
    /// Checks the binary, makes the flight's directory, writes the parameter
    /// file into it and starts the program there. See the module notes for
    /// what it leaves to the caller. The parameters are checked before any
    /// directory is made.
    pub fn start(config: SitlConfig) -> Result<Self, SitlError> {
        if json_port_of(config.instance).is_none() {
            return Err(SitlError::BadInstance {
                instance: config.instance,
            });
        }
        // 1. The binary.
        let binary = absolute(&config.binary);
        let binary_sha256 = match sha256::of_file(&binary) {
            Ok(d) => d,
            Err(e) if e.kind() == io::ErrorKind::NotFound => {
                return Err(SitlError::BinaryMissing { path: binary });
            }
            Err(source) => {
                return Err(SitlError::BinaryUnreadable {
                    path: binary,
                    source,
                });
            }
        };
        if !config.accept_any_binary && binary_sha256 != PINNED.sha256 {
            return Err(SitlError::ChecksumMismatch {
                path: binary,
                found: binary_sha256,
                expected: PINNED.sha256,
            });
        }
        // The parameters, before anything is made.
        let mut files = Vec::with_capacity(config.param_files.len());
        for path in &config.param_files {
            files.push(
                std::fs::read(path).map_err(|source| SitlError::ParamFileUnreadable {
                    path: path.clone(),
                    source,
                })?,
            );
        }
        let params = effective_params(&files, &config.extra_params)?;
        // 2. The flight's own, new directory.
        let name = config.flight_name.clone().unwrap_or_else(|| {
            format!(
                "sitl-i{}-p{}-{}",
                config.instance,
                std::process::id(),
                FLIGHTS.fetch_add(1, Ordering::Relaxed)
            )
        });
        let root = absolute(&config.root);
        std::fs::create_dir_all(&root).map_err(|source| SitlError::Dir {
            path: root.clone(),
            source,
        })?;
        let dir = root.join(name);
        match std::fs::create_dir(&dir) {
            Ok(()) => {}
            Err(e) if e.kind() == io::ErrorKind::AlreadyExists => {
                return Err(SitlError::DirExists { path: dir });
            }
            Err(source) => return Err(SitlError::Dir { path: dir, source }),
        }
        // 3. The effective parameter file.
        let params_path = dir.join("defaults.parm");
        std::fs::write(&params_path, &params).map_err(|source| SitlError::Write {
            path: params_path.clone(),
            source,
        })?;
        let params_sha256 = sha256::hex(&sha256::digest(&params));
        // 4. The child.
        let args: Vec<String> = vec![
            "--model".into(),
            "JSON:127.0.0.1".into(),
            "--speedup".into(),
            config.speedup.to_string(),
            "--defaults".into(),
            params_path.display().to_string(),
            "--home".into(),
            config.home.clone(),
            "--start-time".into(),
            config.start_time.to_string(),
            "-I".into(),
            config.instance.to_string(),
            "-w".into(),
        ];
        let open = |file: &str| {
            let path = dir.join(file);
            std::fs::File::create(&path).map_err(|source| SitlError::Write { path, source })
        };
        let out = open("stdout.txt")?;
        let err = open("stderr.txt")?;
        let mut cmd = Command::new(&binary);
        cmd.args(&args)
            .current_dir(&dir)
            .stdin(Stdio::null())
            .stdout(Stdio::from(out))
            .stderr(Stdio::from(err));
        let child = spawn(&mut cmd, &binary)?;
        let mut command_line = vec![binary.display().to_string()];
        command_line.extend(args);
        Ok(Self {
            child: Some(child),
            instance: config.instance,
            dir,
            binary_sha256,
            params_sha256,
            command_line,
        })
    }

    /// The UDP port the program sends motor packets to (the caller binds it
    /// before the program starts).
    pub fn json_port(&self) -> u16 {
        json_port_of(self.instance).unwrap_or(0)
    }

    /// The MAVLink TCP port of serial 0, 1 or 2; `None` for another number.
    pub fn mavlink_port(&self, serial: u8) -> Option<u16> {
        mavlink_port_of(self.instance, serial)
    }

    pub fn instance(&self) -> u8 {
        self.instance
    }

    /// The flight's own directory (absolute).
    pub fn dir(&self) -> &Path {
        &self.dir
    }

    /// SHA-256 of the binary as found, hexadecimal.
    pub fn binary_sha256(&self) -> &str {
        &self.binary_sha256
    }

    /// SHA-256 of the effective parameter file, hexadecimal.
    pub fn params_sha256(&self) -> &str {
        &self.params_sha256
    }

    /// The program's path, then its arguments.
    pub fn command_line(&self) -> &[String] {
        &self.command_line
    }

    /// The process id, while the child has not been reaped.
    pub fn pid(&self) -> Option<u32> {
        self.child.as_ref().map(Child::id)
    }

    fn stderr_tail(&self) -> String {
        tail_of(&self.dir.join("stderr.txt"))
    }

    /// `Ok` while the child runs; once it has ended, an error with its exit
    /// status and the last lines of its error output.
    pub fn check(&mut self) -> Result<(), SitlError> {
        let Some(child) = self.child.as_mut() else {
            return Ok(());
        };
        match child.try_wait() {
            Ok(None) => Ok(()),
            Ok(Some(status)) => Err(SitlError::Exited {
                status,
                stderr_tail: self.stderr_tail(),
            }),
            Err(source) => Err(SitlError::Wait { source }),
        }
    }

    /// Ends the child (kill, then wait) and says how it ended. A child that
    /// had ended already is reported as such.
    pub fn stop(mut self) -> Result<SitlExit, SitlError> {
        let Some(mut child) = self.child.take() else {
            return Err(SitlError::Wait {
                source: io::Error::other("already stopped"),
            });
        };
        let ended_before_stop = matches!(child.try_wait(), Ok(Some(_)));
        if !ended_before_stop {
            // The child may end between the check and the kill: not an error.
            let _ = child.kill();
        }
        let status = child.wait().map_err(|source| SitlError::Wait { source })?;
        Ok(SitlExit {
            status,
            ended_before_stop,
            stderr_tail: self.stderr_tail(),
        })
    }
}

impl Drop for Sitl {
    fn drop(&mut self) {
        if let Some(mut child) = self.child.take() {
            let _ = child.kill();
            let _ = child.wait();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;

    /// A fresh empty directory under the system's temporary directory.
    fn scratch(tag: &str) -> PathBuf {
        static N: AtomicU64 = AtomicU64::new(0);
        let n = N.fetch_add(1, Ordering::Relaxed);
        let d =
            std::env::temp_dir().join(format!("oxide-sitl-test-{}-{tag}-{n}", std::process::id()));
        let _ = fs::remove_dir_all(&d);
        fs::create_dir_all(&d).expect("scratch");
        d
    }

    /// A file with `bytes`, standing in for the binary (never run).
    fn fake_binary(dir: &Path, bytes: &[u8]) -> PathBuf {
        let p = dir.join("arducopter");
        fs::write(&p, bytes).expect("write");
        p
    }

    #[test]
    fn ports_follow_the_instance() {
        assert_eq!(json_port_of(0), Some(9002));
        assert_eq!(json_port_of(1), Some(9012));
        assert_eq!(json_port_of(9), Some(9092));
        assert_eq!(json_port_of(10), None);
        assert_eq!(mavlink_port_of(0, 0), Some(5760));
        assert_eq!(mavlink_port_of(0, 1), Some(5762));
        assert_eq!(mavlink_port_of(0, 2), Some(5763));
        assert_eq!(mavlink_port_of(1, 0), Some(5770));
        assert_eq!(mavlink_port_of(1, 2), Some(5773));
        assert_eq!(mavlink_port_of(9, 0), Some(5850));
        assert_eq!(mavlink_port_of(9, 1), Some(5852));
        assert_eq!(mavlink_port_of(0, 3), None);
        assert_eq!(mavlink_port_of(10, 0), None);
    }

    #[test]
    fn a_missing_binary_is_named() {
        let root = scratch("missing");
        let bin = root.join("no-such-binary");
        let e = Sitl::start(SitlConfig::new(&bin, root.join("flights"))).unwrap_err();
        assert!(matches!(e, SitlError::BinaryMissing { .. }), "{e:?}");
        assert!(e.to_string().contains(&*bin.display().to_string()), "{e}");
        let _ = fs::remove_dir_all(&root);
    }

    #[test]
    fn another_checksum_is_refused_with_both_digests_and_accepted_on_request() {
        let root = scratch("sum");
        let bin = fake_binary(&root, b"not the simulator\n");
        // A shell-free check: only the checksum step is under test, so give
        // a directory that exists to stop the start before anything runs.
        let found = sha256::of_file(&bin).expect("digest");
        let e = Sitl::start(SitlConfig::new(&bin, root.join("flights"))).unwrap_err();
        assert!(matches!(e, SitlError::ChecksumMismatch { .. }), "{e:?}");
        let msg = e.to_string();
        assert!(msg.contains(&found), "{msg}");
        assert!(msg.contains(PINNED.sha256), "{msg}");
        assert!(msg.contains(&*bin.display().to_string()), "{msg}");
        assert!(!root.join("flights").exists(), "nothing was made");

        // Accepted: the start goes on to the directory step.
        let mut c = SitlConfig::new(&bin, root.join("flights"));
        c.accept_any_binary = true;
        c.flight_name = Some("f".into());
        fs::create_dir_all(root.join("flights").join("f")).expect("dir");
        let e = Sitl::start(c).unwrap_err();
        assert!(matches!(e, SitlError::DirExists { .. }), "{e:?}");
        let _ = fs::remove_dir_all(&root);
    }

    #[test]
    fn a_directory_that_exists_is_refused_and_left_alone() {
        let root = scratch("exists");
        let bin = fake_binary(&root, b"x");
        let mut c = SitlConfig::new(&bin, root.join("flights"));
        c.accept_any_binary = true;
        c.flight_name = Some("again".into());
        let dir = root.join("flights").join("again");
        fs::create_dir_all(&dir).expect("dir");
        fs::write(dir.join("keep"), b"1").expect("file");
        let e = Sitl::start(c).unwrap_err();
        assert!(matches!(e, SitlError::DirExists { .. }), "{e:?}");
        assert!(e.to_string().contains(&*dir.display().to_string()), "{e}");
        assert_eq!(fs::read(dir.join("keep")).expect("kept"), b"1");
        let _ = fs::remove_dir_all(&root);
    }

    #[test]
    fn the_effective_parameter_file_is_these_exact_bytes() {
        let files = vec![b"FRAME_CLASS 1\nFRAME_TYPE 1\n".to_vec(), b"A 2".to_vec()];
        let extra = vec![
            ("SIM_BARO_RND".to_string(), 0.173),
            ("LOG_BACKEND_TYPE".to_string(), 0.0),
            ("X".to_string(), -2.5),
        ];
        let got = effective_params(&files, &extra).expect("params");
        assert_eq!(
            String::from_utf8(got).expect("utf8"),
            "FRAME_CLASS 1\nFRAME_TYPE 1\nA 2\nSIM_BARO_RND 0.173\nLOG_BACKEND_TYPE 0\nX -2.5\n"
        );
        assert_eq!(effective_params(&[], &[]).expect("empty"), Vec::<u8>::new());
    }

    #[test]
    fn a_parameter_that_would_break_the_line_format_is_refused() {
        for name in ["", "A B", "A\nB", "A\tB", "#C", "NAMÉ"] {
            let e = effective_params(&[], &[(name.to_string(), 1.0)]).unwrap_err();
            assert!(matches!(e, SitlError::BadParam { .. }), "{name:?}: {e:?}");
        }
        for v in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let e = effective_params(&[], &[("OK".to_string(), v)]).unwrap_err();
            assert!(matches!(e, SitlError::BadParam { .. }), "{v}: {e:?}");
        }
    }

    #[test]
    fn a_bad_extra_parameter_stops_the_start_before_a_directory_is_made() {
        let root = scratch("badparam");
        let bin = fake_binary(&root, b"x");
        let mut c = SitlConfig::new(&bin, root.join("flights"));
        c.accept_any_binary = true;
        c.extra_params = vec![("A B".into(), 1.0)];
        let e = Sitl::start(c).unwrap_err();
        assert!(matches!(e, SitlError::BadParam { .. }), "{e:?}");
        assert!(e.to_string().contains("A B"), "{e}");
        assert!(!root.join("flights").exists());
        let _ = fs::remove_dir_all(&root);
    }

    #[test]
    fn an_instance_above_nine_is_refused() {
        let root = scratch("inst");
        let bin = fake_binary(&root, b"x");
        let mut c = SitlConfig::new(&bin, root.join("flights"));
        c.accept_any_binary = true;
        c.instance = 10;
        let e = Sitl::start(c).unwrap_err();
        assert!(
            matches!(e, SitlError::BadInstance { instance: 10 }),
            "{e:?}"
        );
        let _ = fs::remove_dir_all(&root);
    }

    #[cfg(unix)]
    mod with_a_script {
        use super::*;
        use std::os::unix::fs::PermissionsExt;
        use std::time::Instant;

        /// A shell script standing in for the binary.
        fn script(dir: &Path, body: &str) -> PathBuf {
            let p = dir.join("fake-arducopter");
            fs::write(&p, format!("#!/bin/sh\n{body}\n")).expect("write");
            fs::set_permissions(&p, fs::Permissions::from_mode(0o755)).expect("chmod");
            p
        }

        fn config(bin: &Path, root: &Path) -> SitlConfig {
            let mut c = SitlConfig::new(bin, root.join("flights"));
            c.accept_any_binary = true;
            c.instance = 2;
            c
        }

        fn wait_for(path: &Path) -> String {
            let t = Instant::now();
            while t.elapsed() < Duration::from_secs(10) {
                if let Ok(s) = fs::read_to_string(path)
                    && !s.is_empty()
                {
                    return s;
                }
                std::thread::sleep(Duration::from_millis(10));
            }
            panic!("{} never got content", path.display());
        }

        fn alive(pid: u32) -> bool {
            // A zombie still has a /proc entry; a reaped process does not.
            Path::new(&format!("/proc/{pid}")).exists()
        }

        #[test]
        fn the_child_runs_in_the_new_directory_with_the_expected_arguments() {
            let root = scratch("args");
            let bin = script(
                &root,
                "pwd > where.txt\nfor a in \"$@\"; do echo \"$a\"; done > args.txt\n\
                 echo to-stdout\necho to-stderr 1>&2\nexec sleep 30",
            );
            let params = root.join("base.parm");
            fs::write(&params, "FRAME_CLASS 1\n").expect("params");
            let mut c = config(&bin, &root);
            c.param_files = vec![params];
            c.extra_params = vec![("LOG_BACKEND_TYPE".into(), 0.0)];
            let sitl = Sitl::start(c).expect("start");
            assert_eq!(sitl.json_port(), 9022);
            assert_eq!(sitl.mavlink_port(0), Some(5780));
            assert_eq!(sitl.mavlink_port(2), Some(5783));
            assert_eq!(sitl.binary_sha256(), sha256::of_file(&bin).expect("sum"));

            let dir = sitl.dir().to_path_buf();
            assert!(dir.starts_with(root.join("flights")));
            let where_ = wait_for(&dir.join("where.txt"));
            let want = fs::canonicalize(&dir).expect("canon");
            assert_eq!(Path::new(where_.trim()), want);
            let args = wait_for(&dir.join("args.txt"));
            let lines: Vec<&str> = args.lines().collect();
            let defaults = dir.join("defaults.parm");
            let defaults = defaults.to_str().expect("utf8").to_string();
            assert_eq!(
                lines,
                [
                    "--model",
                    "JSON:127.0.0.1",
                    "--speedup",
                    "100",
                    "--defaults",
                    &defaults,
                    "--home",
                    "-35.363261,149.165230,584,0",
                    "--start-time",
                    "1700000000",
                    "-I",
                    "2",
                    "-w",
                ]
            );
            assert_eq!(sitl.command_line()[1..], lines[..]);
            // The effective parameter file and its checksum.
            let eff = fs::read(&defaults).expect("defaults");
            assert_eq!(eff, b"FRAME_CLASS 1\nLOG_BACKEND_TYPE 0\n");
            assert_eq!(sitl.params_sha256(), sha256::hex(&sha256::digest(&eff)));
            // Output goes to two files.
            assert_eq!(wait_for(&dir.join("stdout.txt")).trim(), "to-stdout");
            assert_eq!(wait_for(&dir.join("stderr.txt")).trim(), "to-stderr");
            drop(sitl);
            let _ = fs::remove_dir_all(&root);
        }

        #[test]
        fn check_reports_a_child_that_has_exited_with_its_status_and_error_output() {
            let root = scratch("exit");
            let bin = script(&root, "echo one 1>&2\necho two 1>&2\nexit 3");
            let mut sitl = Sitl::start(config(&bin, &root)).expect("start");
            let t = Instant::now();
            let e = loop {
                match sitl.check() {
                    Ok(()) => {
                        assert!(t.elapsed() < Duration::from_secs(10), "child never ended");
                        std::thread::sleep(Duration::from_millis(10));
                    }
                    Err(e) => break e,
                }
            };
            let SitlError::Exited {
                status,
                ref stderr_tail,
            } = e
            else {
                panic!("{e:?}");
            };
            assert_eq!(status.code(), Some(3));
            assert!(stderr_tail.contains("one") && stderr_tail.contains("two"));
            assert!(e.to_string().contains("two"), "{e}");
            // And again: it stays ended.
            assert!(sitl.check().is_err());
            let exit = sitl.stop().expect("stop");
            assert!(exit.ended_before_stop);
            assert_eq!(exit.status.code(), Some(3));
            let _ = fs::remove_dir_all(&root);
        }

        #[test]
        fn dropping_the_value_ends_a_child_that_would_run_for_a_minute() {
            let root = scratch("drop");
            let bin = script(&root, "exec sleep 60");
            let sitl = Sitl::start(config(&bin, &root)).expect("start");
            let pid = sitl.pid().expect("pid");
            assert!(alive(pid));
            drop(sitl);
            assert!(!alive(pid), "the child is gone and reaped");
            let _ = fs::remove_dir_all(&root);
        }

        #[test]
        fn stop_ends_a_running_child_and_says_so() {
            let root = scratch("stop");
            let bin = script(&root, "exec sleep 60");
            let mut sitl = Sitl::start(config(&bin, &root)).expect("start");
            sitl.check().expect("running");
            let pid = sitl.pid().expect("pid");
            let exit = sitl.stop().expect("stop");
            assert!(!exit.ended_before_stop);
            assert!(!exit.status.success());
            assert!(!alive(pid));
            let _ = fs::remove_dir_all(&root);
        }

        #[test]
        fn a_binary_that_cannot_be_started_is_named() {
            let root = scratch("spawn");
            let bin = root.join("not-executable");
            fs::write(&bin, "#!/bin/sh\n").expect("write");
            fs::set_permissions(&bin, fs::Permissions::from_mode(0o644)).expect("chmod");
            let e = Sitl::start(config(&bin, &root)).unwrap_err();
            assert!(matches!(e, SitlError::Spawn { .. }), "{e:?}");
            assert!(e.to_string().contains(&*bin.display().to_string()), "{e}");
            let _ = fs::remove_dir_all(&root);
        }
    }
}
