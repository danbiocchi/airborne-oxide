// SPDX-License-Identifier: AGPL-3.0-or-later
//! The parameter files of a bridged ArduCopter flight (ADR-0006 section 4,
//! ADR-0007 section 2), committed in `arducopter/`.
//!
//! A file is `NAME VALUE` per line; `#` starts a comment. The program reads
//! the files as its defaults ([`crate::sitl::SitlConfig::param_files`]).

use core::fmt;
use std::path::PathBuf;

/// `arducopter/base.parm`: frame, thrust curve, sensors, battery, log, radio
/// and mode mapping. No gains.
pub const BASE: &str = include_str!("../arducopter/base.parm");
/// `arducopter/setup.parm`: the settings of ArduPilot's tuning guide that are
/// not in `base.parm`.
pub const SETUP: &str = include_str!("../arducopter/setup.parm");
/// `arducopter/starting-gains.parm`: the hand step before the tuner.
pub const STARTING_GAINS: &str = include_str!("../arducopter/starting-gains.parm");
/// `arducopter/failsafes.parm`: ADR-0007 section 3, each failsafe switched on
/// with its usual action. Not in the default file list of
/// [`crate::arducopter::ArduCopterConfig::new`]: a flight takes it on request.
pub const FAILSAFES: &str = include_str!("../arducopter/failsafes.parm");

/// The path of `base.parm` in the source tree this crate was built from.
pub fn base_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("arducopter/base.parm")
}

/// The path of `setup.parm` in the source tree this crate was built from.
pub fn setup_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("arducopter/setup.parm")
}

/// The path of `starting-gains.parm` in the source tree this crate was built
/// from.
pub fn starting_gains_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("arducopter/starting-gains.parm")
}

/// The path of `failsafes.parm` in the source tree this crate was built from.
pub fn failsafes_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("arducopter/failsafes.parm")
}

/// A line that is not a parameter.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ParmError {
    /// 1-based line number.
    pub line: usize,
    pub reason: String,
}

impl fmt::Display for ParmError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "parameter file line {}: {}", self.line, self.reason)
    }
}

impl std::error::Error for ParmError {}

/// The `(name, value)` pairs of a file, in order. Blank lines and comment
/// lines are skipped; a line is a name, white space and one finite number,
/// and nothing else.
pub fn parse(text: &str) -> Result<Vec<(String, f64)>, ParmError> {
    let mut out = Vec::new();
    for (i, raw) in text.lines().enumerate() {
        let line = i + 1;
        let body = raw.split('#').next().unwrap_or("").trim();
        if body.is_empty() {
            continue;
        }
        let bad = |reason: &str| ParmError {
            line,
            reason: reason.to_string(),
        };
        let mut parts = body.split_whitespace();
        let (Some(name), Some(value), None) = (parts.next(), parts.next(), parts.next()) else {
            return Err(bad("expected `NAME VALUE`"));
        };
        if !name
            .bytes()
            .all(|b| b.is_ascii_uppercase() || b.is_ascii_digit() || b == b'_')
        {
            return Err(bad("a name is upper-case letters, digits and `_`"));
        }
        let value: f64 = value
            .parse()
            .map_err(|_| bad("the value is not a number"))?;
        if !value.is_finite() {
            return Err(bad("the value is not finite"));
        }
        out.push((name.to_string(), value));
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeMap;

    fn all() -> Vec<(String, f64)> {
        let mut v = parse(BASE).expect("base.parm parses");
        v.extend(parse(SETUP).expect("setup.parm parses"));
        v.extend(parse(STARTING_GAINS).expect("starting-gains.parm parses"));
        v
    }

    #[test]
    fn the_files_on_disk_are_the_ones_compiled_in() {
        assert_eq!(std::fs::read_to_string(base_path()).expect("base"), BASE);
        assert_eq!(std::fs::read_to_string(setup_path()).expect("setup"), SETUP);
        assert_eq!(
            std::fs::read_to_string(starting_gains_path()).expect("gains"),
            STARTING_GAINS
        );
        assert_eq!(
            std::fs::read_to_string(failsafes_path()).expect("failsafes"),
            FAILSAFES
        );
    }

    #[test]
    fn the_failsafes_file_holds_the_five_settings_of_the_gate_record_and_nothing_else() {
        let f: Vec<(String, f64)> = parse(FAILSAFES).expect("failsafes.parm parses");
        let want = [
            ("FS_THR_ENABLE", 1.0),
            ("FS_EKF_ACTION", 1.0),
            ("BATT_FS_LOW_ACT", 2.0),
            ("BATT_FS_CRT_ACT", 1.0),
            ("FS_GCS_ENABLE", 1.0),
        ];
        let got: Vec<(&str, f64)> = f.iter().map(|(n, v)| (n.as_str(), *v)).collect();
        assert_eq!(got, want);
        // Each line has a comment that names its row (the line before).
        for (name, _) in want {
            let at = FAILSAFES.find(name).expect("the name is in the text");
            let before = &FAILSAFES[..at];
            let last_line = before.trim_end().lines().last().unwrap_or("");
            assert!(
                last_line.starts_with('#'),
                "{name} has no comment before it"
            );
        }
        // None of them is in the other two files (a later file would win).
        for (name, _) in want {
            assert!(
                !all().iter().any(|(n, _)| n == name),
                "{name} is also in base or gains"
            );
        }
    }

    #[test]
    fn no_name_appears_twice_across_base_setup_and_starting_gains() {
        let mut seen = BTreeMap::new();
        for (name, value) in all() {
            assert!(
                seen.insert(name.clone(), value).is_none(),
                "{name} appears twice"
            );
        }
    }

    #[test]
    fn the_names_the_specification_lists_are_there_with_its_values() {
        let map: BTreeMap<String, f64> = all().into_iter().collect();
        let base: BTreeMap<String, f64> = parse(BASE).expect("base").into_iter().collect();
        let gains: BTreeMap<String, f64> =
            parse(STARTING_GAINS).expect("gains").into_iter().collect();
        let want_base = [
            ("FRAME_CLASS", 1.0),
            ("FRAME_TYPE", 1.0),
            ("MOT_THST_EXPO", 1.0),
            ("MOT_THST_HOVER", 0.16),
            ("SIM_DRIFT_SPEED", 0.0),
            ("SIM_BARO_RND", 0.173),
            ("SIM_MAG_RND", 27.0),
            ("SIM_GPS1_LAG_MS", 150.0),
            ("SIM_GPS1_VERR_X", 0.1),
            ("SIM_GPS1_VERR_Y", 0.1),
            ("SIM_GPS1_VERR_Z", 0.1),
            ("SIM_GPS1_NOISE", 0.3),
            ("BATT_MONITOR", 4.0),
            ("LOG_BACKEND_TYPE", 0.0),
            ("SIM_FLOAT_EXCEPT", 0.0),
            ("FLTMODE_CH", 5.0),
            ("FLTMODE1", 0.0),
            ("FLTMODE4", 5.0),
            ("FLTMODE6", 4.0),
            ("RC6_OPTION", 153.0),
            ("GUID_OPTIONS", 1.0),
            ("RC7_OPTION", 31.0),
        ];
        for (name, value) in want_base {
            assert_eq!(base.get(name), Some(&value), "base.parm {name}");
        }
        let want_gains = [
            ("ATC_RAT_RLL_P", 0.05),
            ("ATC_RAT_RLL_I", 0.05),
            ("ATC_RAT_RLL_D", 0.0012),
            ("ATC_RAT_PIT_P", 0.05),
            ("ATC_RAT_PIT_I", 0.05),
            ("ATC_RAT_PIT_D", 0.0012),
            ("ATC_RAT_YAW_P", 0.065),
            ("ATC_RAT_YAW_I", 0.0065),
        ];
        for (name, value) in want_gains {
            assert_eq!(gains.get(name), Some(&value), "starting-gains.parm {name}");
        }
        assert_eq!(gains.len(), 8, "the gains file holds the eight gains only");
        // No gain in the base file.
        assert!(
            !base.keys().any(|n| n.starts_with("ATC_")),
            "no gain in base.parm"
        );
        assert_eq!(map.len(), base.len() + gains.len() + 10);
    }

    #[test]
    fn setup_parm_holds_the_tuning_guides_settings_each_with_a_comment() {
        let want = [
            ("MOT_BAT_VOLT_MAX", 25.2),
            ("MOT_BAT_VOLT_MIN", 19.8),
            ("INS_ACCEL_FILTER", 10.0),
            ("INS_GYRO_FILTER", 80.0),
            ("ATC_RAT_RLL_FLTT", 40.0),
            ("ATC_RAT_RLL_FLTD", 40.0),
            ("ATC_RAT_PIT_FLTT", 40.0),
            ("ATC_RAT_PIT_FLTD", 40.0),
            ("ATC_RAT_YAW_FLTT", 40.0),
            ("ATC_RAT_YAW_FLTE", 2.0),
        ];
        let got: Vec<(String, f64)> = parse(SETUP).expect("setup.parm parses");
        let got: Vec<(&str, f64)> = got.iter().map(|(n, v)| (n.as_str(), *v)).collect();
        assert_eq!(got, want);
        // Each is in a block that has a comment line above it.
        for (name, _) in want {
            let at = SETUP
                .find(&format!("\n{name} "))
                .expect("the name is in the text");
            let before = &SETUP[..at];
            let block: Vec<&str> = before
                .lines()
                .rev()
                .take_while(|l| !l.trim().is_empty())
                .collect();
            assert!(
                block.iter().any(|l| l.starts_with('#')),
                "{name} has no comment"
            );
        }
        // What the guide leaves to the defaults or to the airframe is not set.
        let names: Vec<&str> = want.iter().map(|(n, _)| *n).collect();
        for left in [
            "MOT_THST_EXPO",
            "MOT_THST_HOVER",
            "MOT_SPIN_ARM",
            "MOT_SPIN_MIN",
            "MOT_SPIN_MAX",
            "ATC_ACC_R_MAX",
            "ATC_ACC_P_MAX",
            "ATC_ACC_Y_MAX",
            "AUTOTUNE_AGGR",
            "AUTOTUNE_MIN_D",
        ] {
            assert!(!names.contains(&left), "{left} stays out of setup.parm");
        }
    }

    #[test]
    fn a_line_that_is_not_a_parameter_is_refused_with_its_number() {
        assert_eq!(parse("# c\n\nA_B 1.5 # x\nC 2\n").expect("ok").len(), 2);
        for bad in ["A", "A 1 2", "a 1", "A x", "A inf", "A NaN"] {
            let e = parse(&format!("# ok\n{bad}\n")).expect_err(bad);
            assert_eq!(e.line, 2, "{bad}");
        }
    }
}
