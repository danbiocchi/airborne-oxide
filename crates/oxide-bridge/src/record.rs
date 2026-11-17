// SPDX-License-Identifier: AGPL-3.0-or-later
//! A flight's record as files (ADR-0006 section 11): `audit.txt` and the one
//! row of `row.csv`, in the table format that serves both sides of the
//! benchmark (our core's flights and ArduCopter's).
//!
//! A row has [`COLUMNS`], in this order. A value that does not exist is
//! empty. Numbers are written with every digit needed to read them back
//! (Rust's shortest round-trip form). A text field never holds a comma or a
//! line break ([`clean`]).

use core::fmt::Write as _;
use std::path::Path;

use crate::runner::{Autopilot, RunError, ScenarioFlight};
use crate::scenario::LoadedScenario;

/// The columns of a row, in order.
pub const COLUMNS: [&str; 29] = [
    "scenario",
    "index",
    "seed",
    "autopilot",
    "started",
    "class",
    "verdict",
    "violations",
    "not_judged",
    "rms_h_m",
    "max_h_m",
    "rms_v_m",
    "max_v_m",
    "max_tilt_deg",
    "max_height_m",
    "max_distance_m",
    "touchdown_mps",
    "disarmed_at_s",
    "last_motor_on_s",
    "end_n_m",
    "end_e_m",
    "end_d_m",
    "trace_hash",
    "packet_hash",
    "reply_hash",
    "repeats",
    "wall_s",
    "retried",
    "note",
];

/// The header line of a table.
pub fn header() -> String {
    COLUMNS.join(",")
}

/// Text that is safe in a field: commas become semicolons, line breaks and
/// other control characters become spaces, runs of spaces become one, and the
/// ends are trimmed.
pub fn clean(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    let mut space = true;
    for c in text.chars() {
        let c = match c {
            ',' => ';',
            c if c.is_control() => ' ',
            c => c,
        };
        if c == ' ' {
            if !space {
                out.push(' ');
            }
            space = true;
        } else {
            out.push(c);
            space = false;
        }
    }
    out.truncate(out.trim_end().len());
    out
}

/// Why a line is not a row.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RowError(pub String);

impl core::fmt::Display for RowError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for RowError {}

/// One row of the table: a cell of text for each of [`COLUMNS`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Row {
    cells: Vec<String>,
}

impl Default for Row {
    fn default() -> Self {
        Self::empty()
    }
}

fn number(v: f64) -> String {
    format!("{v}")
}

fn optional(v: Option<f64>) -> String {
    v.map(number).unwrap_or_default()
}

fn yes(v: bool) -> &'static str {
    if v { "yes" } else { "no" }
}

impl Row {
    /// A row with every cell empty.
    pub fn empty() -> Self {
        Self {
            cells: vec![String::new(); COLUMNS.len()],
        }
    }

    /// The cell of column `name`; empty for a name that is not a column.
    pub fn get(&self, name: &str) -> &str {
        COLUMNS
            .iter()
            .position(|c| *c == name)
            .and_then(|i| self.cells.get(i))
            .map_or("", String::as_str)
    }

    /// Sets the cell of column `name` ([`clean`]ed). A name that is not a
    /// column is a programming error and is ignored.
    pub fn set(&mut self, name: &str, value: &str) {
        let cell = COLUMNS
            .iter()
            .position(|c| *c == name)
            .and_then(|i| self.cells.get_mut(i));
        if let Some(cell) = cell {
            *cell = clean(value);
        }
    }

    /// The row as one line, without the line break.
    pub fn line(&self) -> String {
        self.cells.join(",")
    }

    /// Reads a line written by [`line`](Self::line).
    pub fn parse(line: &str) -> Result<Self, RowError> {
        let cells: Vec<String> = line
            .trim_end_matches(['\r', '\n'])
            .split(',')
            .map(String::from)
            .collect();
        if cells.len() != COLUMNS.len() {
            return Err(RowError(format!(
                "a row has {} cells and the table has {} columns",
                cells.len(),
                COLUMNS.len()
            )));
        }
        Ok(Self { cells })
    }

    /// The row of a flight that was flown and judged.
    pub fn of_flight(f: &ScenarioFlight) -> Self {
        let o = &f.verdict.outcome;
        let mut r = Self::empty();
        r.set("scenario", &f.scenario);
        r.set("index", &f.index.to_string());
        r.set("seed", &f.seed.to_string());
        r.set("autopilot", &f.autopilot.name());
        r.set("started", "yes");
        r.set("class", f.class.name());
        r.set(
            "verdict",
            if !f.verdict.violations.is_empty() {
                "fail"
            } else if !f.verdict.not_judged.is_empty() {
                "not-judged"
            } else {
                "pass"
            },
        );
        r.set("violations", &f.verdict.violations.len().to_string());
        r.set("not_judged", &f.verdict.not_judged.len().to_string());
        if let Some(h) = &f.verdict.hold {
            r.set("rms_h_m", &number(h.rms_h_error_m));
            r.set("max_h_m", &number(h.max_h_error_m));
            r.set("max_v_m", &number(h.max_v_error_m));
            r.set("rms_v_m", &optional(f.rms_v_m));
        }
        r.set("max_tilt_deg", &number(o.max_tilt_deg));
        r.set("max_height_m", &number(o.max_height_m));
        r.set("max_distance_m", &number(o.max_distance_from_home_m));
        r.set("touchdown_mps", &optional(o.touchdown_speed_mps));
        r.set("disarmed_at_s", &optional(o.disarmed_at_s));
        r.set("last_motor_on_s", &optional(f.last_motor_on_s));
        r.set("end_n_m", &number(o.position_at_end.x));
        r.set("end_e_m", &number(o.position_at_end.y));
        r.set("end_d_m", &number(o.position_at_end.z));
        r.set("trace_hash", &format!("{:016x}", f.trace_hash));
        if let Some(h) = f.packet_hash {
            r.set("packet_hash", &format!("{h:016x}"));
        }
        if let Some(h) = f.reply_hash {
            r.set("reply_hash", &format!("{h:016x}"));
        }
        r.set("wall_s", &number(f.wall_s));
        r.set("retried", "no");
        r.set("note", &f.note());
        r
    }

    /// The row of a flight that did not start, or whose bridge failed: the
    /// identity columns, `started` no and the reason in `note`.
    pub fn not_started(
        scenario: &str,
        index: u64,
        seed: u64,
        autopilot: &Autopilot,
        wall_s: f64,
        reason: &str,
    ) -> Self {
        let mut r = Self::empty();
        r.set("scenario", scenario);
        r.set("index", &index.to_string());
        r.set("seed", &seed.to_string());
        r.set("autopilot", &autopilot.name());
        r.set("started", "no");
        r.set("wall_s", &number(wall_s));
        r.set("retried", "no");
        r.set("note", reason);
        r
    }

    /// Records the answer of `--twice`.
    pub fn set_repeats(&mut self, repeats: bool) {
        self.set("repeats", yes(repeats));
    }
}

/// The text of `audit.txt`: what the run was (the scenario file's hash, the
/// index and seed, our commit), what was translated, the outcome and the
/// verdict, then the bridge's audit of the flight (ADR-0006 section 11).
pub fn audit_text(src: &LoadedScenario, flight: &ScenarioFlight, repeats: Option<bool>) -> String {
    let mut s = String::new();
    let _ = writeln!(s, "oxide-bridge flight record");
    let _ = writeln!(s, "  scenario        {}", flight.scenario);
    let _ = writeln!(s, "  scenario file   {}", src.path.display());
    let _ = writeln!(s, "  file sha256     {}", src.sha256);
    let _ = writeln!(s, "  run index       {}", flight.index);
    let _ = writeln!(s, "  seed            {}", flight.seed);
    let _ = writeln!(s, "  autopilot       {}", flight.autopilot.name());
    let _ = writeln!(s, "  our commit      {}", env!("OXIDE_GIT_HASH"));
    let _ = writeln!(s, "  wall time       {} s", flight.wall_s);
    if let Some(r) = repeats {
        let _ = writeln!(
            s,
            "  repeats         {} (trace, packet and reply hashes of two flights)",
            yes(r)
        );
    }
    if flight.ours_only.is_empty() {
        let _ = writeln!(s, "  ours only       (none)");
    } else {
        let list: Vec<String> = flight
            .ours_only
            .iter()
            .map(|(n, v)| format!("{n}={v}"))
            .collect();
        let _ = writeln!(
            s,
            "  ours only       {} (parameters of our core with no counterpart, not sent)",
            list.join(" ")
        );
    }
    if !flight.param_files.is_empty() {
        let _ = writeln!(s, "  parameter files {}", flight.param_files.join(" "));
    }
    if !flight.extra_params.is_empty() {
        let list: Vec<String> = flight
            .extra_params
            .iter()
            .map(|(n, v)| format!("{n}={v}"))
            .collect();
        let _ = writeln!(s, "  extra params    {}", list.join(" "));
    }
    let o = &flight.verdict.outcome;
    let _ = writeln!(s, "  class           {}", flight.class.name());
    let _ = writeln!(
        s,
        "  outcome         armed at end {}, ended {:.3} s at {:?}, home {:?}, max height {} m, \
         max distance {} m, max tilt {} deg, touchdown {:?} m/s, disarmed at {:?} s, \
         last motor on {:?} s, physics faults {}",
        o.armed_at_end,
        o.end_s,
        o.position_at_end,
        o.home,
        o.max_height_m,
        o.max_distance_from_home_m,
        o.max_tilt_deg,
        o.touchdown_speed_mps,
        o.disarmed_at_s,
        flight.last_motor_on_s,
        o.physics_faults
    );
    if let Some(h) = &flight.verdict.hold {
        let _ = writeln!(s, "  hold            {h:?}");
        let _ = writeln!(s, "  hold vertical   rms {:?} m", flight.rms_v_m);
    }
    let _ = writeln!(
        s,
        "  verdict         {} violations, {} not judged",
        flight.verdict.violations.len(),
        flight.verdict.not_judged.len()
    );
    for v in &flight.verdict.violations {
        let _ = writeln!(s, "    violated: {v}");
    }
    for n in &flight.verdict.not_judged {
        let _ = writeln!(s, "    not judged: {n}");
    }
    if let Some(a) = &flight.audit {
        let _ = writeln!(s);
        let _ = write!(s, "{a}");
    }
    s
}

/// The text of `audit.txt` of a flight that did not start or whose bridge
/// failed: the identity, the reason, and the audit the failed flight left.
pub fn failure_text(
    src: &LoadedScenario,
    index: u64,
    autopilot: &Autopilot,
    e: &RunError,
) -> String {
    let mut s = String::new();
    let _ = writeln!(
        s,
        "oxide-bridge flight record (the flight did not complete)"
    );
    let _ = writeln!(s, "  scenario        {}", src.file.name);
    let _ = writeln!(s, "  scenario file   {}", src.path.display());
    let _ = writeln!(s, "  file sha256     {}", src.sha256);
    let _ = writeln!(s, "  run index       {index}");
    let _ = writeln!(s, "  seed            {}", src.file.seed(index));
    let _ = writeln!(s, "  autopilot       {}", autopilot.name());
    let _ = writeln!(s, "  our commit      {}", env!("OXIDE_GIT_HASH"));
    let _ = writeln!(s, "  reason          {e}");
    if let Some(a) = e.audit() {
        let _ = writeln!(s);
        let _ = write!(s, "{a}");
    }
    s
}

/// Writes `audit.txt` and `row.csv` (the header line, then the row) into
/// `dir`, which is made when it does not exist.
pub fn write_files(dir: &Path, audit: &str, row: &Row) -> std::io::Result<()> {
    std::fs::create_dir_all(dir)?;
    std::fs::write(dir.join("audit.txt"), audit)?;
    std::fs::write(
        dir.join("row.csv"),
        format!("{}\n{}\n", header(), row.line()),
    )
}

/// Reads the row of a `row.csv` written by [`write_files`].
pub fn read_row(path: &Path) -> Result<Row, RowError> {
    let text = std::fs::read_to_string(path)
        .map_err(|e| RowError(format!("cannot read {}: {e}", path.display())))?;
    let mut lines = text.lines();
    if lines.next() != Some(header().as_str()) {
        return Err(RowError(format!(
            "{} does not start with the table's header",
            path.display()
        )));
    }
    let line = lines
        .next()
        .ok_or_else(|| RowError(format!("{} has no row", path.display())))?;
    Row::parse(line)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_header_names_the_columns_in_the_order_of_the_specification() {
        assert_eq!(
            header(),
            "scenario,index,seed,autopilot,started,class,verdict,violations,not_judged,rms_h_m,\
             max_h_m,rms_v_m,max_v_m,max_tilt_deg,max_height_m,max_distance_m,touchdown_mps,\
             disarmed_at_s,last_motor_on_s,end_n_m,end_e_m,end_d_m,trace_hash,packet_hash,\
             reply_hash,repeats,wall_s,retried,note"
        );
    }

    #[test]
    fn text_never_holds_a_comma_or_a_line_break() {
        assert_eq!(clean("a, b\nc\r\n d\t e"), "a; b c d e");
        assert_eq!(clean("  x  "), "x");
        assert_eq!(clean(""), "");
        let mut r = Row::empty();
        r.set("note", "one, two\nthree");
        assert_eq!(r.get("note"), "one; two three");
        assert!(!r.line().contains('\n'));
        assert_eq!(r.line().matches(',').count(), COLUMNS.len() - 1);
    }

    #[test]
    fn a_row_reads_back_what_was_written() {
        let mut r = Row::empty();
        r.set("scenario", "gust-hold");
        r.set("rms_h_m", &number(0.1 + 0.2));
        let back = Row::parse(&r.line()).expect("parses");
        assert_eq!(back, r);
        assert_eq!(
            back.get("rms_h_m").parse::<f64>().expect("a number"),
            0.1 + 0.2
        );
        assert!(Row::parse("a,b").is_err());
        assert_eq!(Row::empty().get("nonsense"), "");
    }
    use oxide_core::math::Vec3;
    use oxide_sim::metrics::HoldMetrics;
    use oxide_sim::outcome::{FlightClass, FlightOutcome, NotJudged};
    use oxide_sim::scenario_file::Verdict;

    fn known_flight() -> ScenarioFlight {
        let mut outcome = FlightOutcome::new();
        outcome.max_tilt_deg = 8.5;
        outcome.max_height_m = 2.25;
        outcome.max_distance_from_home_m = 0.75;
        outcome.position_at_end = Vec3::new(0.1, -0.2, -2.0);
        let mut hold = HoldMetrics::new();
        hold.rms_h_error_m = 0.125;
        hold.max_h_error_m = 0.5;
        hold.max_v_error_m = 0.0625;
        ScenarioFlight {
            scenario: "gust-hold".into(),
            index: 3,
            seed: 77,
            autopilot: Autopilot::ArduCopter,
            verdict: Verdict {
                outcome,
                hold: Some(hold),
                violations: vec!["max error, too big".into()],
                not_judged: vec![NotJudged {
                    key: "k",
                    reason: "r",
                }],
            },
            class: FlightClass::Safe,
            rms_v_m: Some(0.03125),
            trace_hash: 0x9712_d375_90a3_17dc,
            packet_hash: Some(1),
            reply_hash: Some(0xabc),
            audit: None,
            wall_s: 9.5,
            last_motor_on_s: Some(39.995),
            ours_only: vec![("FS_GPS_LAND".into(), 10.0)],
            param_files: vec!["base.parm".into()],
            extra_params: vec![("FENCE_RADIUS".into(), 10.0)],
        }
    }

    #[test]
    fn the_row_of_a_known_outcome_is_this_line_byte_for_byte() {
        let line = Row::of_flight(&known_flight()).line();
        assert_eq!(
            line,
            "gust-hold,3,77,arducopter,yes,safe,fail,1,1,0.125,0.5,0.03125,0.0625,8.5,2.25,0.75,,,\
             39.995,0.1,-0.2,-2,9712d37590a317dc,0000000000000001,0000000000000abc,,9.5,no,\
             first problem: max error; too big; ours only: FS_GPS_LAND=10"
        );
        assert_eq!(line.split(',').count(), COLUMNS.len());
    }

    #[test]
    fn a_row_without_hold_numbers_or_hashes_leaves_them_empty() {
        let mut f = known_flight();
        f.verdict.hold = None;
        f.verdict.violations.clear();
        f.verdict.not_judged.clear();
        f.ours_only.clear();
        f.packet_hash = None;
        f.reply_hash = None;
        f.autopilot = Autopilot::Core(oxide_sim::scenario_file::RunProfile::H743);
        f.verdict.outcome.touchdown_speed_mps = Some(0.5);
        f.verdict.outcome.disarmed_at_s = Some(12.25);
        let r = Row::of_flight(&f);
        assert_eq!(r.get("autopilot"), "core-h743");
        assert_eq!(r.get("verdict"), "pass");
        for empty in [
            "rms_h_m",
            "max_h_m",
            "rms_v_m",
            "max_v_m",
            "packet_hash",
            "reply_hash",
            "repeats",
            "note",
        ] {
            assert_eq!(r.get(empty), "", "{empty}");
        }
        assert_eq!(r.get("touchdown_mps"), "0.5");
        assert_eq!(r.get("disarmed_at_s"), "12.25");
        let mut r = r;
        r.set_repeats(true);
        assert_eq!(r.get("repeats"), "yes");
    }

    #[test]
    fn a_row_that_did_not_start_says_so_and_why() {
        let r = Row::not_started(
            "gust-hold",
            4,
            9,
            &Autopilot::ArduCopter,
            1.5,
            "the program did not arm,\nafter 5 s",
        );
        assert_eq!(
            r.line(),
            "gust-hold,4,9,arducopter,no,,,,,,,,,,,,,,,,,,,,,,1.5,no,the program did not arm; after 5 s"
        );
    }

    #[test]
    fn the_files_hold_the_audit_and_the_row() {
        let dir = std::env::temp_dir().join(format!("oxide-record-test-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let src = LoadedScenario::from_text("name = \"x\"\nduration_s = 1.0\n").expect("loads");
        let f = known_flight();
        let text = audit_text(&src, &f, Some(true));
        for needle in [
            &src.sha256,
            "run index       3",
            "seed            77",
            "autopilot       arducopter",
            env!("OXIDE_GIT_HASH"),
            "FS_GPS_LAND=10",
            "FENCE_RADIUS=10",
            "repeats         yes",
            "violated: max error, too big",
            "not judged:",
        ] {
            assert!(text.contains(needle), "{needle}\n{text}");
        }
        let row = Row::of_flight(&f);
        write_files(&dir, &text, &row).expect("writes");
        assert_eq!(
            std::fs::read_to_string(dir.join("audit.txt")).expect("audit"),
            text
        );
        assert_eq!(read_row(&dir.join("row.csv")).expect("row"), row);
        std::fs::write(dir.join("bad.csv"), "nope\n").expect("writes");
        assert!(read_row(&dir.join("bad.csv")).is_err());
        let _ = std::fs::remove_dir_all(&dir);
    }
}
