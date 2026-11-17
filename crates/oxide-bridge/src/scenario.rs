// SPDX-License-Identifier: AGPL-3.0-or-later
//! A scenario file as a bridged flight (ADR-0007): what of the file ArduCopter
//! can be given, and the reason when it cannot.
//!
//! [`translate`] decides. The file's `[params]` are our core's parameters and
//! never reach ArduCopter as they are: [`PARAMS`] says what each name
//! becomes. The pilot must be offboard throughout (stick flying is a later
//! part). Faults are translated: the fault translator writes them to
//! the program ([`FAULTS_TRANSLATABLE`]).

use core::fmt;
use std::path::{Path, PathBuf};

use oxide_sim::scenario_file::{self, ScenarioFile};

use crate::arducopter::BrainLink;
use crate::sha256;

/// The airframe the bridge flies: the reference quad. ArduCopter's frame,
/// thrust curve and mass are set for it in `base.parm`.
pub const AIRFRAME: &str = "quad-x-5in";

/// The value of the pilot's mode switch that means offboard.
pub const MODE_OFFBOARD: u8 = 2;

/// Whether a scenario with faults is translatable: the faults are applied to
/// the program as parameter writes (`faults::FaultTranslator`). This flag is
/// the one place that says so.
pub const FAULTS_TRANSLATABLE: bool = true;

/// What a scenario parameter becomes for ArduCopter.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ParamRoute {
    /// The distance of the geofence, m: ArduCopter's circle fence
    /// ([`fence_params`]).
    Fence,
    /// A timing of our core's own response, with no counterpart: listed in
    /// the flight's record as "ours only" and otherwise ignored.
    OursOnly,
}

/// The table: the parameter names a scenario file may carry and what each
/// becomes. A name that is not here makes the scenario untranslatable.
pub const PARAMS: [(&str, ParamRoute); 3] = [
    ("FS_FENCE_RAD", ParamRoute::Fence),
    ("FS_GPS_LAND", ParamRoute::OursOnly),
    ("FS_NOLINK_RTL", ParamRoute::OursOnly),
];

/// ArduCopter's fence for a radius above zero, m: enabled, circle only, action 1 (RTL,
/// or land when RTL cannot be done).
pub fn fence_params(radius_m: f64) -> Vec<(String, f64)> {
    vec![
        ("FENCE_ENABLE".into(), 1.0),
        ("FENCE_TYPE".into(), 2.0),
        ("FENCE_RADIUS".into(), radius_m),
        ("FENCE_ACTION".into(), 1.0),
    ]
}

/// Why a scenario cannot be flown through the bridge. Each is a sentence that
/// names the cause.
#[derive(Clone, Debug, PartialEq)]
pub enum Untranslatable {
    /// The file names another airframe.
    Airframe(String),
    /// The pilot's mode switch is at some time anything but offboard.
    StickFlying {
        /// The first mode switch position that is not offboard, and when it
        /// starts, s (`None` for no mode switch at all, which is stabilized).
        first: Option<(f64, u8)>,
    },
    /// A parameter that is not in the table.
    Param(String),
    /// A parameter whose value cannot be used.
    ParamValue { name: String, value: f64 },
    /// The file has faults and [`FAULTS_TRANSLATABLE`] is off.
    Faults(usize),
}

impl fmt::Display for Untranslatable {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Airframe(a) => write!(
                f,
                "the airframe {a:?} is not {AIRFRAME:?}, the only one the bridge flies"
            ),
            Self::StickFlying { first: None } => write!(
                f,
                "stick flying: a later part (the file has no mode switch, which is stabilized)"
            ),
            Self::StickFlying {
                first: Some((t, m)),
            } => write!(
                f,
                "stick flying: a later part (the mode switch is {m} from {t} s, not offboard)"
            ),
            Self::Param(name) => write!(
                f,
                "the parameter {name} has no counterpart in ArduCopter and is not on the list of \
                 parameters that are ours only"
            ),
            Self::ParamValue { name, value } => write!(
                f,
                "the parameter {name} is {value}: the fence radius must be a finite number, not negative"
            ),
            Self::Faults(n) => write!(f, "faults are not translatable (the file has {n})"),
        }
    }
}

impl std::error::Error for Untranslatable {}

/// What a scenario file becomes for ArduCopter.
#[derive(Clone, Debug, PartialEq)]
pub struct Translation {
    /// Parameters given at start, after the files (the fence).
    pub extra_params: Vec<(String, f64)>,
    /// The scenario's parameters that are our core's only: listed in the
    /// record, otherwise ignored.
    pub ours_only: Vec<(String, f64)>,
    /// Where the brain's commands go: nowhere when the file has no brain.
    pub brain: BrainLink,
}

/// Decides what of `file` ArduCopter is given, or why it cannot fly it. The
/// first reason is returned, checked in this order: airframe, mode switch,
/// parameters, faults.
pub fn translate(file: &ScenarioFile) -> Result<Translation, Untranslatable> {
    if file.airframe != AIRFRAME {
        return Err(Untranslatable::Airframe(file.airframe.clone()));
    }
    let off = file.pilot.mode.iter().find(|(_, m)| *m != MODE_OFFBOARD);
    if file.pilot.mode.is_empty() || off.is_some() {
        return Err(Untranslatable::StickFlying {
            first: off.copied(),
        });
    }
    let mut extra_params = Vec::new();
    let mut ours_only = Vec::new();
    for (name, value) in &file.params {
        let route = PARAMS
            .iter()
            .find(|(n, _)| n == name)
            .map(|(_, r)| *r)
            .ok_or_else(|| Untranslatable::Param(name.clone()))?;
        match route {
            ParamRoute::Fence => {
                if !(value.is_finite() && *value >= 0.0) {
                    return Err(Untranslatable::ParamValue {
                        name: name.clone(),
                        value: *value,
                    });
                }
                // Our core reads 0 as "no fence".
                if *value == 0.0 {
                    extra_params.push(("FENCE_ENABLE".into(), 0.0));
                } else {
                    extra_params.extend(fence_params(*value));
                }
            }
            ParamRoute::OursOnly => ours_only.push((name.clone(), *value)),
        }
    }
    if !FAULTS_TRANSLATABLE && !file.faults.is_empty() {
        return Err(Untranslatable::Faults(file.faults.len()));
    }
    Ok(Translation {
        extra_params,
        ours_only,
        brain: if file.links.brain {
            BrainLink::Serial1
        } else {
            BrainLink::None
        },
    })
}

/// A scenario file read from disk, with what the record says about it.
#[derive(Clone, Debug)]
pub struct LoadedScenario {
    pub file: ScenarioFile,
    /// The path it was read from, as given.
    pub path: PathBuf,
    /// SHA-256 of the file's bytes, hexadecimal.
    pub sha256: String,
}

/// Why a scenario file could not be loaded.
#[derive(Debug)]
pub enum LoadError {
    Read {
        path: PathBuf,
        source: std::io::Error,
    },
    Parse(scenario_file::ScenarioError),
}

impl fmt::Display for LoadError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Read { path, source } => {
                write!(
                    f,
                    "cannot read the scenario file {}: {source}",
                    path.display()
                )
            }
            Self::Parse(e) => write!(f, "{e}"),
        }
    }
}

impl std::error::Error for LoadError {}

impl LoadedScenario {
    /// Reads and parses `path`.
    pub fn load(path: impl AsRef<Path>) -> Result<Self, LoadError> {
        let path = path.as_ref();
        let bytes = std::fs::read(path).map_err(|source| LoadError::Read {
            path: path.to_path_buf(),
            source,
        })?;
        let sha256 = sha256::hex(&sha256::digest(&bytes));
        let file = scenario_file::load_path(path).map_err(LoadError::Parse)?;
        Ok(Self {
            file,
            path: path.to_path_buf(),
            sha256,
        })
    }

    /// A scenario that is not from a file on disk (the hash is of its text).
    pub fn from_text(text: &str) -> Result<Self, LoadError> {
        let file = scenario_file::load_str(text).map_err(LoadError::Parse)?;
        Ok(Self {
            file,
            path: PathBuf::from("<text>"),
            sha256: sha256::hex(&sha256::digest(text.as_bytes())),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scenarios_dir() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../scenarios")
    }

    const HEAD: &str = "name = \"t\"\nduration_s = 10.0\n";
    const OFFBOARD: &str =
        "[pilot]\narm_at_s = 1.0\n[[pilot.mode]]\nt = 0.0\nswitch = \"offboard\"\n";

    fn load(text: &str) -> ScenarioFile {
        scenario_file::load_str(text).unwrap_or_else(|e| panic!("{e}\n{text}"))
    }

    #[test]
    fn an_offboard_scenario_without_parameters_translates_to_nothing_extra() {
        let t = translate(&load(&format!("{HEAD}{OFFBOARD}"))).expect("translatable");
        assert!(t.extra_params.is_empty());
        assert!(t.ours_only.is_empty());
        assert_eq!(t.brain, BrainLink::Serial1);
    }

    #[test]
    fn the_fence_radius_becomes_the_circle_fence() {
        let t = translate(&load(&format!(
            "{HEAD}[params]\nFS_FENCE_RAD = 12.5\n{OFFBOARD}"
        )))
        .expect("translatable");
        assert_eq!(
            t.extra_params,
            vec![
                ("FENCE_ENABLE".to_string(), 1.0),
                ("FENCE_TYPE".to_string(), 2.0),
                ("FENCE_RADIUS".to_string(), 12.5),
                ("FENCE_ACTION".to_string(), 1.0),
            ]
        );
        assert!(t.ours_only.is_empty());
    }

    #[test]
    fn the_parameters_that_are_ours_only_are_listed_and_not_sent() {
        let t = translate(&load(&format!(
            "{HEAD}[params]\nFS_GPS_LAND = 10.0\nFS_NOLINK_RTL = 5.0\n{OFFBOARD}"
        )))
        .expect("translatable");
        assert!(t.extra_params.is_empty(), "{t:?}");
        assert_eq!(
            t.ours_only,
            vec![
                ("FS_GPS_LAND".to_string(), 10.0),
                ("FS_NOLINK_RTL".to_string(), 5.0)
            ]
        );
    }

    #[test]
    fn every_name_of_the_table_is_translated_and_none_is_dropped() {
        // One file with all three: the fence is sent, the others listed.
        let t = translate(&load(&format!(
            "{HEAD}[params]\nFS_FENCE_RAD = 10.0\nFS_GPS_LAND = 1.0\nFS_NOLINK_RTL = 2.0\n{OFFBOARD}"
        )))
        .expect("translatable");
        assert_eq!(t.extra_params.len(), 4);
        assert_eq!(t.ours_only.len(), 2);
        for (name, route) in PARAMS {
            let seen = match route {
                ParamRoute::Fence => t.extra_params.iter().any(|(n, _)| n == "FENCE_RADIUS"),
                ParamRoute::OursOnly => t.ours_only.iter().any(|(n, _)| n == name),
            };
            assert!(seen, "{name} reached neither ArduCopter nor the list");
        }
    }

    #[test]
    fn an_unknown_parameter_makes_the_scenario_untranslatable_and_is_named() {
        let e = translate(&load(&format!(
            "{HEAD}[params]\nFS_GPS_LAND = 1.0\nRATE_ROLL_P = 30.0\n{OFFBOARD}"
        )))
        .unwrap_err();
        assert_eq!(e, Untranslatable::Param("RATE_ROLL_P".into()));
        assert!(e.to_string().contains("RATE_ROLL_P"), "{e}");
        // A hand-built file can carry a name our core does not have either.
        let mut f = load(&format!("{HEAD}{OFFBOARD}"));
        f.params.push(("SOME_GAIN".into(), 2.0));
        assert_eq!(
            translate(&f),
            Err(Untranslatable::Param("SOME_GAIN".into()))
        );
    }

    #[test]
    fn a_fence_radius_that_cannot_be_used_is_refused_and_zero_means_no_fence() {
        for bad in [-3.0, f64::NAN, f64::INFINITY] {
            let mut f = load(&format!("{HEAD}{OFFBOARD}"));
            f.params.push(("FS_FENCE_RAD".into(), bad));
            let e = translate(&f).unwrap_err();
            assert!(matches!(e, Untranslatable::ParamValue { .. }), "{e:?}");
        }
        let t = translate(&load(&format!(
            "{HEAD}[params]\nFS_FENCE_RAD = 0.0\n{OFFBOARD}"
        )))
        .expect("translatable");
        assert_eq!(t.extra_params, vec![("FENCE_ENABLE".to_string(), 0.0)]);
    }

    #[test]
    fn a_stick_scenario_is_not_translatable_in_this_part() {
        for pilot in [
            "[pilot]\narm_at_s = 1.0\n[[pilot.mode]]\nt = 0.0\nswitch = \"stabilized\"\n",
            "[pilot]\narm_at_s = 1.0\n[[pilot.mode]]\nt = 0.0\nswitch = \"offboard\"\n[[pilot.mode]]\nt = 5.0\nswitch = \"position\"\n",
            "[pilot]\narm_at_s = 1.0\n",
        ] {
            let e = translate(&load(&format!("{HEAD}{pilot}"))).unwrap_err();
            assert!(matches!(e, Untranslatable::StickFlying { .. }), "{e:?}");
            assert!(e.to_string().contains("stick flying: a later part"), "{e}");
        }
        let e = translate(&load(&format!(
            "{HEAD}[pilot]\narm_at_s = 1.0\n[[pilot.mode]]\nt = 0.0\nswitch = \"offboard\"\n[[pilot.mode]]\nt = 5.0\nswitch = \"position\"\n"
        )))
        .unwrap_err();
        assert_eq!(
            e,
            Untranslatable::StickFlying {
                first: Some((5.0, 1))
            }
        );
    }

    #[test]
    fn a_scenario_with_a_fault_is_translatable_because_the_faults_are_applied() {
        let faulted = load(&format!(
            "{HEAD}{OFFBOARD}[[fault]]\nkind = \"gps-loss\"\nstart_s = 5.0\n"
        ));
        const { assert!(FAULTS_TRANSLATABLE) };
        assert!(translate(&faulted).is_ok());
    }

    #[test]
    fn another_airframe_is_refused() {
        let mut f = load(&format!("{HEAD}{OFFBOARD}"));
        f.airframe = "hex".into();
        assert!(matches!(translate(&f), Err(Untranslatable::Airframe(_))));
    }

    #[test]
    fn a_file_without_a_brain_sends_no_commands() {
        let t = translate(&load(&format!(
            "{HEAD}[sim.links]\nbrain = false\n{OFFBOARD}"
        )))
        .expect("translatable");
        assert_eq!(t.brain, BrainLink::None);
    }

    #[test]
    fn the_scenarios_of_the_repository_translate_or_say_why() {
        let mut ok = Vec::new();
        let mut why = Vec::new();
        let mut names: Vec<_> = std::fs::read_dir(scenarios_dir())
            .expect("scenarios")
            .map(|e| e.expect("entry").path())
            .filter(|p| p.extension().is_some_and(|e| e == "toml"))
            .collect();
        names.sort();
        for p in names {
            let loaded = LoadedScenario::load(&p).expect("loads");
            match translate(&loaded.file) {
                Ok(_) => ok.push(loaded.file.name.clone()),
                Err(e) => why.push((loaded.file.name.clone(), e)),
            }
        }
        // Without faults: gust-hold, hover-step, geofence, the battery ones,
        // setpoint-gap and the pairs of two non-faults.
        for must in [
            "gust-hold",
            "hover-step",
            "geofence",
            "battery-low",
            "battery-critical",
            "setpoint-gap",
            "gps-loss",
            "imu-loss",
            "motor-loss",
            "brain-crash",
        ] {
            assert!(ok.iter().any(|n| n == must), "{must}: {why:?}");
        }
        // Every refusal is for sticks (the pilot files); none is for a fault
        // or for a parameter name outside the table.
        for (name, e) in &why {
            assert!(
                matches!(e, Untranslatable::StickFlying { .. }),
                "{name}: {e}"
            );
        }
        assert!(why.iter().any(
            |(n, e)| n == "rc-loss-position" && matches!(e, Untranslatable::StickFlying { .. })
        ));
    }

    #[test]
    fn a_loaded_scenario_carries_the_hash_of_its_bytes() {
        let p = scenarios_dir().join("hover-step.toml");
        let loaded = LoadedScenario::load(&p).expect("loads");
        let bytes = std::fs::read(&p).expect("reads");
        assert_eq!(loaded.sha256, sha256::hex(&sha256::digest(&bytes)));
        assert_eq!(loaded.sha256.len(), 64);
        assert_eq!(loaded.file.name, "hover-step");
        let e = LoadedScenario::load(scenarios_dir().join("nothing.toml")).unwrap_err();
        assert!(e.to_string().contains("nothing.toml"), "{e}");
    }
}
