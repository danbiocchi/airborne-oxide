// SPDX-License-Identifier: AGPL-3.0-or-later
//! ArduCopter's tuning procedure as a script (ADR-0007 section 2 as changed by
//! ADR-0009): `oxide-bridge tune`.
//!
//! The script flies ArduCopter on the reference quad in calm air on a full
//! pack, with the gate's sensing (our IMU model on the 1000 us grid, the white
//! GPS error, the 45 s pre-roll) and with `base.parm`, `setup.parm` and
//! `starting-gains.parm` (the only flights that fly the starting gains). Every
//! tuning flight is these stages:
//!
//! 1. *Hover and learn.* Mode POSHOLD by a MAVLink mode command at a fixed
//!    time (the radio's mode switch has no POSHOLD position), the arm switch,
//!    a climb on the throttle stick, and a hover of [`HOVER_S`] seconds in
//!    which the program learns its hover thrust.
//! 2. *Spectrum.* An amplitude spectrum per axis of the gyro samples this
//!    script saw our IMU model send during the last [`SPECTRUM_S`] seconds of
//!    the hover ([`spectrum`]), and the verdict of the rule of ADR-0007 step 4
//!    ([`judge`]). The spectrum of every hover is taken and reported; only
//!    the hover of the first flight flown decides (below).
//! 3. *AutoTune.* Mode AUTOTUNE, the sticks centred, for [`AUTOTUNE_S`]
//!    seconds; then the descent with the throttle stick at [`DESCENT_STICK`]
//!    until the true height is at most [`GROUND_M`], and the stick fully low:
//!    the vehicle disarms itself inside AUTOTUNE, and the tuner saves the
//!    gains.
//! 4. *Read back.* After the disarm every parameter the tuner may have
//!    changed ([`GAIN_NAMES`]) is asked for in groups of 6 a second apart.
//! 5. *Check.* The tuner's messages for the axes the flight tunes, the
//!    set-up file as read back before the flight, the read-back, the gate's
//!    outcome class of the flight, and the pack's voltage before the descent.
//!
//! The procedure is two tuning flights, each a new start of the program on a
//! full pack ([`tune`]):
//!
//! - **The roll-and-pitch flight** flies the base file, the set-up file and the
//!   starting gains with `AUTOTUNE_AXES` 3. Its saved gains are read back and
//!   written to `tuned-gains-1.parm`.
//! - **The yaw flight** flies the same three files, then `tuned-gains-1.parm`,
//!   with `AUTOTUNE_AXES` 4. Its saved gains are the procedure's result. It
//!   must change nothing but yaw ([`check_carried`]).
//!
//! **The notch is decided once**, on the hover of the first flight flown,
//! which flies the starting gains. With a peak that flight lands after its
//! hover; the roll-and-pitch flight is then flown from the start with the
//! guide's throttle-based harmonic notch ([`notch_params`], which needs a
//! restart of the program) and the learned hover thrust as `MOT_THST_HOVER`,
//! and the yaw flight flies the same notch. With no peak no flight has a
//! notch. `tuned-gains.parm` lists [`GAIN_NAMES`] and, when a notch was set,
//! the six `INS_HNTCH_*` values read back after the last flight, so that every
//! later flight of the file flies the notch its gains were tuned with.
//!
//! # Where the rule of the bridge holds
//!
//! Every stick, every command and every parameter request is decided by the
//! simulated time, the physics truth (the height), the motor pulses of the
//! previous exchange (whether the vehicle is armed) and the gyro samples this
//! script itself gave the program: [`Script`] and [`Tuner`]. What the program
//! says (status texts, parameter values) is read only after the flight, by the
//! functions after the marker line in this file, into the report and the
//! parameter files: the one place where values read from MAVLink are written
//! out. Two exceptions, both between flights and both stated: the learned
//! hover thrust, which is the notch's reference and the roll-and-pitch
//! flight's `MOT_THST_HOVER` when a peak was found (see
//! [`Outcome::notch`]); and the gains the roll-and-pitch flight
//! saved, which the yaw flight is given as a parameter file. In both, a value
//! read from the program decides a parameter of the next flight, and nothing
//! decides what a flight does while it flies.

use core::fmt;
use std::collections::{BTreeMap, VecDeque};
use std::fmt::Write as _;
use std::path::{Path, PathBuf};
use std::time::Instant;

use oxide_core::io::{RcSample, Stamped};
use oxide_core::math::{Real, Vec3};
use oxide_core::time::Micros;
use oxide_sim::autopilot::{Autopilot, AutopilotInput, AutopilotOutput, PrerollInput};
use oxide_sim::outcome::{
    CRASH_TILT_DEG, CRASH_TOUCHDOWN_MPS, FLY_AWAY_DISTANCE_M, FLY_AWAY_HEIGHT_M, FlightClass,
    FlightOutcome,
};
use oxide_sim::scenario::{COMMAND_PERIOD_US, Pilot};
use oxide_sim::sim::{PrerollError, SimOptions, StepRecord};
use oxide_sim::{Airframe, SimError, Simulation};

use crate::arducopter::{
    ArduCopter, ArduCopterConfig, ArduCopterError, BrainLink, FlightAudit, PREROLL_STEPS, STEP_US,
    bridged_options,
};
use crate::mav::MavError;
use crate::parm;

// ---- The script's times, in flight microseconds -------------------------------

const S: u64 = 1_000_000;
/// ArduCopter's mode numbers: every tuning flight starts in POSHOLD and the
/// tuner is AUTOTUNE.
pub const MODE_AUTOTUNE: u32 = 15;
pub const MODE_POSHOLD: u32 = 16;
/// The mode command (POSHOLD) goes out at this time.
const START_MODE_AT_US: u64 = 2 * S;
/// The first group of parameter reads before the flight, and how many names
/// are asked for at once (more than about 20 at once are partly not answered).
const READ_BEFORE_AT_US: u64 = 3 * S;
const READ_GROUP: usize = 6;
/// Time from the last group to the arm switch.
const ARM_GAP_US: u64 = 3 * S;
/// Time from the arm switch to the throttle going up: the program keeps its
/// motors still for 2.0 s after it arms (live 2.233 s after the switch).
const CLIMB_DELAY_US: u64 = 3 * S;
/// The throttle stick of the climb (0 to 1; above 0.55 is up in POSHOLD, which
/// reads the stick as ALT_HOLD does).
const CLIMB_STICK: f32 = 0.8;
/// The stick goes back to centre when the true height passes this, m.
pub const CLIMB_STOP_M: f64 = 9.5;
/// A climb that takes longer than this does not reach the height.
const CLIMB_LIMIT_US: u64 = 45 * S;
/// The hover, s, and the last part of it that the spectrum is taken from.
pub const HOVER_S: u64 = 60;
pub const SPECTRUM_S: u64 = 30;
/// The last part of the hover the hover quality is measured on, s.
pub const QUALITY_S: u64 = 20;
/// From the end of the hover (the spectrum) to the mode change.
const AFTER_SPECTRUM_US: u64 = S;
/// How long AUTOTUNE is given before the descent begins, s. The tuner
/// needed 439 s on seed 0; the pack is empty about 650 s into the flight
/// (ADR-0007 section 2). A fixed number: nothing read from MAVLink decides
/// when the pilot lands.
pub const AUTOTUNE_S: u64 = 480;
/// The throttle stick of the landing (ADR-0007 section 2): ArduCopter reads it
/// as a slow descent. The stick goes fully low when the vehicle is on the
/// ground.
pub const DESCENT_STICK: f32 = 0.30;
/// The vehicle is on the ground when its true height is at or below this, m.
pub const GROUND_M: f64 = 0.05;
/// The vehicle must be on the ground (and then disarmed, by its pulses)
/// within this time of the descent beginning or of the throttle going low,
/// each.
const LAND_LIMIT_US: u64 = 180 * S;
/// From the pulses saying disarmed to the read-back.
const DISARM_SETTLE_US: u64 = 5 * S;
/// After the last read-back group, before the flight ends.
const READ_END_US: u64 = 4 * S;
/// The pack voltage under load is averaged over this long before the throttle
/// goes low.
pub const PACK_WINDOW_US: u64 = S;

// ---- The names -------------------------------------------------------------

/// What the tuner may change, read back after the disarm and written to
/// `tuned-gains.parm`: the rate controllers, the angle controllers, the
/// acceleration limits, the feed-forward switch and the learned hover thrust.
/// The roll-and-pitch flight tunes the names with `RLL` or `PIT` and
/// `ATC_ACC_R_MAX` and `ATC_ACC_P_MAX`; the yaw flight those with `YAW` and
/// `ATC_ACC_Y_MAX`; both may move `MOT_THST_HOVER`.
pub const GAIN_NAMES: [&str; 29] = [
    "ATC_ACC_P_MAX",
    "ATC_ACC_R_MAX",
    "ATC_ACC_Y_MAX",
    "ATC_ANG_PIT_P",
    "ATC_ANG_RLL_P",
    "ATC_ANG_YAW_P",
    "ATC_RATE_FF_ENAB",
    "ATC_RAT_PIT_D",
    "ATC_RAT_PIT_FF",
    "ATC_RAT_PIT_FLTD",
    "ATC_RAT_PIT_FLTE",
    "ATC_RAT_PIT_FLTT",
    "ATC_RAT_PIT_I",
    "ATC_RAT_PIT_P",
    "ATC_RAT_RLL_D",
    "ATC_RAT_RLL_FF",
    "ATC_RAT_RLL_FLTD",
    "ATC_RAT_RLL_FLTE",
    "ATC_RAT_RLL_FLTT",
    "ATC_RAT_RLL_I",
    "ATC_RAT_RLL_P",
    "ATC_RAT_YAW_D",
    "ATC_RAT_YAW_FF",
    "ATC_RAT_YAW_FLTD",
    "ATC_RAT_YAW_FLTE",
    "ATC_RAT_YAW_FLTT",
    "ATC_RAT_YAW_I",
    "ATC_RAT_YAW_P",
    "MOT_THST_HOVER",
];

/// The names of the harmonic notch ([`notch_params`], in its order): read back
/// after the last flight when a notch was set, and written after the gains of
/// `tuned-gains.parm`.
pub const NOTCH_NAMES: [&str; 6] = [
    "INS_HNTCH_ENABLE",
    "INS_HNTCH_MODE",
    "INS_HNTCH_FREQ",
    "INS_HNTCH_BW",
    "INS_HNTCH_REF",
    "INS_HNTCH_ATT",
];

/// What `setup.parm` sets: read before the flight, to show that the program
/// took every name and value.
pub const SETUP_NAMES: [&str; 10] = [
    "ATC_RAT_PIT_FLTD",
    "ATC_RAT_PIT_FLTT",
    "ATC_RAT_RLL_FLTD",
    "ATC_RAT_RLL_FLTT",
    "ATC_RAT_YAW_FLTE",
    "ATC_RAT_YAW_FLTT",
    "INS_ACCEL_FILTER",
    "INS_GYRO_FILTER",
    "MOT_BAT_VOLT_MAX",
    "MOT_BAT_VOLT_MIN",
];

/// Read before the flight and not after: what the guide's set-up leaves at
/// ArduCopter's default and the tuner's own settings.
const DEFAULT_NAMES: [&str; 8] = [
    "AUTOTUNE_AGGR",
    "AUTOTUNE_AXES",
    "AUTOTUNE_MIN_D",
    "MOT_HOVER_LEARN",
    "MOT_SPIN_ARM",
    "MOT_SPIN_MAX",
    "MOT_SPIN_MIN",
    "MOT_THST_EXPO",
];

/// Every name read before the flight, in the order of the groups.
pub fn before_names() -> Vec<&'static str> {
    let mut v: Vec<&'static str> = Vec::new();
    v.extend(SETUP_NAMES);
    v.extend(GAIN_NAMES);
    v.extend(DEFAULT_NAMES);
    v.sort_unstable();
    v.dedup();
    v
}

/// The groups of names the read-back asks for, in order.
fn groups(names: &[&'static str]) -> Vec<Vec<&'static str>> {
    names.chunks(READ_GROUP).map(<[_]>::to_vec).collect()
}

// ---- The spectrum -------------------------------------------------------------

/// The Welch segment, samples: 1024 at 1000 Hz, a line every 0.977 Hz.
pub const SEGMENT: usize = 1024;
/// The band of the rule, Hz.
pub const BAND_HZ: (f64, f64) = (40.0, 400.0);
/// A line is a peak at this power ratio to the band's median (10 dB).
pub const PEAK_RATIO: f64 = 10.0;

/// An amplitude spectrum per axis.
#[derive(Clone, Debug, PartialEq)]
pub struct Spectrum {
    /// Samples per second.
    pub sample_hz: f64,
    /// The width of a bin, Hz.
    pub bin_hz: f64,
    /// Amplitude per bin, rad/s, bins 0 to `SEGMENT / 2`, for x, y and z.
    pub amplitude: [Vec<f64>; 3],
    /// The samples used, and the segments averaged.
    pub samples: usize,
    pub segments: usize,
}

/// The largest line of one axis in the band.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Line {
    pub axis: usize,
    pub freq_hz: f64,
    /// The line's power over the band's median power.
    pub ratio: f64,
}

/// What the rule says.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Verdict {
    /// The largest line of each axis.
    pub lines: [Line; 3],
    /// The line with the largest ratio, when that ratio is at least
    /// [`PEAK_RATIO`].
    pub peak: Option<Line>,
}

fn fft(re: &mut [f64], im: &mut [f64]) {
    let n = re.len();
    // Bit reversal.
    let mut j = 0usize;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j |= bit;
        if i < j {
            re.swap(i, j);
            im.swap(i, j);
        }
    }
    let mut len = 2;
    while len <= n {
        let ang = -2.0 * core::f64::consts::PI / len as f64;
        for start in (0..n).step_by(len) {
            for k in 0..len / 2 {
                let (wr, wi) = (
                    <f64 as Real>::cos(ang * k as f64),
                    <f64 as Real>::sin(ang * k as f64),
                );
                let (a, b) = (start + k, start + k + len / 2);
                let (xr, xi) = (re[b] * wr - im[b] * wi, re[b] * wi + im[b] * wr);
                re[b] = re[a] - xr;
                im[b] = im[a] - xi;
                re[a] += xr;
                im[a] += xi;
            }
        }
        len <<= 1;
    }
}

/// The amplitude spectrum of each axis of `samples` (gyro, rad/s): the mean of
/// the periodograms of overlapping Hann-windowed segments of [`SEGMENT`]
/// samples (half overlap, the mean of each segment removed). `None` when there
/// are fewer samples than one segment.
///
/// Welch's average, not one periodogram of the whole window: the power of a
/// bin of a single periodogram of noise is exponentially distributed, and
/// about one bin in a thousand lies above ten times the median, so a band of
/// 10 000 bins would always hold a "peak". The average of 57 segments puts the
/// noise's bins within a few percent of one another, so that the rule's ratio
/// of 10 is a line and not a fluctuation.
pub fn spectrum(samples: &[[f32; 3]], sample_hz: f64) -> Option<Spectrum> {
    if samples.len() < SEGMENT {
        return None;
    }
    let n = SEGMENT;
    let window: Vec<f64> = (0..n)
        .map(|i| 0.5 - 0.5 * <f64 as Real>::cos(2.0 * core::f64::consts::PI * i as f64 / n as f64))
        .collect();
    let wsum: f64 = window.iter().sum();
    let bins = n / 2 + 1;
    let mut amplitude: [Vec<f64>; 3] = [vec![0.0; bins], vec![0.0; bins], vec![0.0; bins]];
    let mut segments = 0;
    let mut start = 0;
    while start + n <= samples.len() {
        let seg = &samples[start..start + n];
        for (axis, acc) in amplitude.iter_mut().enumerate() {
            let mean = seg.iter().map(|s| f64::from(s[axis])).sum::<f64>() / n as f64;
            let mut re: Vec<f64> = seg
                .iter()
                .zip(&window)
                .map(|(s, w)| (f64::from(s[axis]) - mean) * w)
                .collect();
            let mut im = vec![0.0; n];
            fft(&mut re, &mut im);
            for (k, a) in acc.iter_mut().enumerate() {
                *a += re[k] * re[k] + im[k] * im[k];
            }
        }
        segments += 1;
        start += n / 2;
    }
    for acc in &mut amplitude {
        for a in acc.iter_mut() {
            *a = 2.0 * Real::sqrt(*a / segments as f64) / wsum;
        }
    }
    Some(Spectrum {
        sample_hz,
        bin_hz: sample_hz / n as f64,
        amplitude,
        samples: samples.len(),
        segments,
    })
}

/// The rule of ADR-0007 step 4 on a spectrum: a peak is a line between 40 Hz
/// and 400 Hz whose power is at least ten times the median power of that
/// band, on any axis.
pub fn judge(s: &Spectrum) -> Verdict {
    let mut lines = [Line {
        axis: 0,
        freq_hz: 0.0,
        ratio: 0.0,
    }; 3];
    for (axis, (line, amp)) in lines.iter_mut().zip(&s.amplitude).enumerate() {
        let band: Vec<(usize, f64)> = amp
            .iter()
            .enumerate()
            .filter(|(k, _)| {
                let f = *k as f64 * s.bin_hz;
                f >= BAND_HZ.0 && f <= BAND_HZ.1
            })
            .map(|(k, a)| (k, a * a))
            .collect();
        let mut sorted: Vec<f64> = band.iter().map(|(_, p)| *p).collect();
        sorted.sort_by(f64::total_cmp);
        let median = match sorted.len() {
            0 => 0.0,
            n if n % 2 == 1 => sorted[n / 2],
            n => 0.5 * (sorted[n / 2 - 1] + sorted[n / 2]),
        };
        let (kmax, pmax) = band
            .iter()
            .copied()
            .fold((0, 0.0), |m, c| if c.1 > m.1 { c } else { m });
        let ratio = if median > 0.0 {
            pmax / median
        } else if pmax > 0.0 {
            f64::INFINITY
        } else {
            0.0
        };
        *line = Line {
            axis,
            freq_hz: kmax as f64 * s.bin_hz,
            ratio,
        };
    }
    let best = lines
        .iter()
        .copied()
        .fold(lines[0], |m, l| if l.ratio > m.ratio { l } else { m });
    Verdict {
        lines,
        peak: (best.ratio >= PEAK_RATIO).then_some(best),
    }
}

/// The guide's throttle-based harmonic notch for a peak at `freq_hz`:
/// `INS_HNTCH_ENABLE 1`, `INS_HNTCH_MODE 1`, `INS_HNTCH_FREQ` the peak's
/// frequency, `INS_HNTCH_BW` half of it, `INS_HNTCH_REF` the learned hover
/// thrust, `INS_HNTCH_ATT 40`.
pub fn notch_params(freq_hz: f64, hover_thrust: f64) -> Vec<(String, f64)> {
    [1.0, 1.0, freq_hz, freq_hz / 2.0, hover_thrust, 40.0]
        .into_iter()
        .zip(NOTCH_NAMES)
        .map(|(v, n)| (n.to_string(), v))
        .collect()
}

/// `spectrum.csv`: the frequency and the amplitude of each axis, one line per
/// bin.
pub fn spectrum_csv(s: &Spectrum) -> String {
    let mut out = String::from("freq_hz,gyro_x_rad_s,gyro_y_rad_s,gyro_z_rad_s\n");
    for k in 0..s.amplitude[0].len() {
        let _ = writeln!(
            out,
            "{},{:e},{:e},{:e}",
            k as f64 * s.bin_hz,
            s.amplitude[0][k],
            s.amplitude[1][k],
            s.amplitude[2][k]
        );
    }
    out
}

// ---- The script ---------------------------------------------------------------

/// Where the script is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Stage {
    /// On the ground until the throttle goes up.
    Ground,
    /// Climbing since this time.
    Climb {
        since: u64,
    },
    /// Hovering since this time; `analysed` once the spectrum has been taken.
    Hover {
        since: u64,
        analysed: Option<u64>,
    },
    /// In AUTOTUNE since this time.
    Autotune {
        since: u64,
    },
    /// Descending with the throttle stick at [`DESCENT_STICK`] since this time,
    /// until the true height is at or below [`GROUND_M`].
    Descend {
        since: u64,
        then_read: bool,
    },
    /// The throttle fully low since this time (the vehicle is on the ground);
    /// the pulses say disarmed since the other time.
    Land {
        since: u64,
        disarmed: Option<u64>,
        then_read: bool,
    },
    /// Reading back since this time; `centre`: the throttle stick is held at
    /// centre (the flight was stopped in the air).
    ReadBack {
        since: u64,
        centre: bool,
    },
    Done,
}

/// The times at which the stages began, flight microseconds.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Marks {
    pub arm: u64,
    pub climb: Option<u64>,
    pub hover: Option<u64>,
    pub spectrum: Option<u64>,
    pub autotune: Option<u64>,
    /// The descent began: the throttle stick went to [`DESCENT_STICK`].
    pub descent: Option<u64>,
    /// The vehicle was on the ground and the throttle stick went fully low.
    pub throttle_low: Option<u64>,
    pub disarmed: Option<u64>,
    pub read_back: Option<u64>,
    /// The time the flight was stopped by the box.
    pub aborted: Option<u64>,
    pub end: Option<u64>,
}

/// What the script does at one tick.
#[derive(Clone, Debug, PartialEq)]
pub struct Action {
    /// The throttle stick, 0 to 1.
    pub throttle: f32,
    /// The arm switch.
    pub arm: bool,
    /// A mode command to send.
    pub set_mode: Option<u32>,
    /// Parameters to ask for.
    pub reads: Vec<&'static str>,
    /// The spectrum is to be taken now, and [`Script::set_peak`] called.
    pub analyse: bool,
}

/// The tuning flight's script, a state machine of the simulated time, the true
/// height and whether the pulses say armed.
#[derive(Clone, Debug)]
pub struct Script {
    stage: Stage,
    before: Vec<Vec<&'static str>>,
    after: Vec<Vec<&'static str>>,
    arm_at: u64,
    peak: Option<bool>,
    stop_on_peak: bool,
    stop_after_hover: bool,
    autotune_us: u64,
    marks: Marks,
}

impl Script {
    /// `stop_on_peak`: a peak ends the flight by landing (the first flight
    /// flown, whose hover decides the notch); `stop_after_hover`: the
    /// flight ends when the spectrum is taken (a hover check).
    pub fn new(stop_on_peak: bool, stop_after_hover: bool, autotune_s: u64) -> Self {
        let before = groups(&before_names());
        let arm_at = READ_BEFORE_AT_US + before.len() as u64 * S + ARM_GAP_US;
        Self {
            stage: Stage::Ground,
            after: groups(&GAIN_NAMES),
            before,
            arm_at,
            peak: None,
            stop_on_peak,
            stop_after_hover,
            autotune_us: autotune_s * S,
            marks: Marks {
                arm: arm_at,
                ..Marks::default()
            },
        }
    }

    /// The read-back after the disarm asks for the six names of the notch
    /// after the gains (a flight that flies a notch).
    #[must_use]
    pub fn with_notch_read(mut self) -> Self {
        self.after.extend(groups(&NOTCH_NAMES));
        self
    }

    pub fn marks(&self) -> Marks {
        self.marks
    }

    pub fn done(&self) -> bool {
        self.stage == Stage::Done
    }

    /// The hover's spectrum window, flight microseconds, once the hover has
    /// begun.
    pub fn spectrum_window(&self) -> Option<(u64, u64)> {
        match self.stage {
            Stage::Hover { since, .. } => {
                Some((since + (HOVER_S - SPECTRUM_S) * S, since + HOVER_S * S))
            }
            _ => None,
        }
    }

    /// Tells the script what the rule said about the spectrum.
    pub fn set_peak(&mut self, peak: bool) {
        self.peak = Some(peak);
    }

    /// One tick at flight time `t_us`: the true `height_m` and whether the
    /// previous exchange's pulses say `armed`. An error is a sentence: the
    /// procedure cannot go on.
    pub fn tick(&mut self, t_us: u64, height_m: f64, armed: bool) -> Result<Action, String> {
        let mut a = Action {
            throttle: 0.0,
            arm: t_us >= self.arm_at,
            set_mode: None,
            reads: Vec::new(),
            analyse: false,
        };
        if t_us == START_MODE_AT_US {
            a.set_mode = Some(MODE_POSHOLD);
        }
        if t_us >= READ_BEFORE_AT_US && (t_us - READ_BEFORE_AT_US).is_multiple_of(S) {
            let k = ((t_us - READ_BEFORE_AT_US) / S) as usize;
            if let Some(g) = self.before.get(k) {
                a.reads.clone_from(g);
            }
        }
        match self.stage {
            Stage::Ground => {
                if t_us >= self.arm_at + CLIMB_DELAY_US {
                    self.marks.climb = Some(t_us);
                    self.stage = Stage::Climb { since: t_us };
                    a.throttle = CLIMB_STICK;
                }
            }
            Stage::Climb { since } => {
                if height_m >= CLIMB_STOP_M {
                    self.marks.hover = Some(t_us);
                    self.stage = Stage::Hover {
                        since: t_us,
                        analysed: None,
                    };
                    a.throttle = 0.5;
                } else if t_us - since > CLIMB_LIMIT_US {
                    return Err(format!(
                        "the climb did not reach {CLIMB_STOP_M} m in {} s (height {height_m:.2} m \
                         at flight time {:.3} s)",
                        CLIMB_LIMIT_US / S,
                        t_us as f64 * 1e-6
                    ));
                } else {
                    a.throttle = CLIMB_STICK;
                }
            }
            Stage::Hover { since, analysed } => {
                a.throttle = 0.5;
                match analysed {
                    None if t_us >= since + HOVER_S * S => {
                        self.marks.spectrum = Some(t_us);
                        self.stage = Stage::Hover {
                            since,
                            analysed: Some(t_us),
                        };
                        a.analyse = true;
                        // The learned thrust, for the notch's reference.
                        a.reads.push("MOT_THST_HOVER");
                    }
                    Some(at) if t_us >= at + AFTER_SPECTRUM_US => {
                        let peak = self.peak.unwrap_or(false);
                        if self.stop_after_hover {
                            self.finish(t_us);
                        } else if peak && self.stop_on_peak {
                            self.begin_descent(t_us, false);
                            a.throttle = DESCENT_STICK;
                        } else {
                            self.marks.autotune = Some(t_us);
                            self.stage = Stage::Autotune { since: t_us };
                            a.set_mode = Some(MODE_AUTOTUNE);
                        }
                    }
                    _ => {}
                }
            }
            Stage::Autotune { since } => {
                a.throttle = 0.5;
                if t_us >= since + self.autotune_us {
                    self.begin_descent(t_us, true);
                    a.throttle = DESCENT_STICK;
                }
            }
            Stage::Descend { since, then_read } => {
                a.throttle = DESCENT_STICK;
                if height_m <= GROUND_M || !armed {
                    self.begin_land(t_us, then_read);
                    a.throttle = 0.0;
                } else if t_us - since > LAND_LIMIT_US {
                    return Err(format!(
                        "the vehicle was not on the ground {} s after the descent began (flight \
                         time {:.3} s, height {height_m:.2} m)",
                        LAND_LIMIT_US / S,
                        t_us as f64 * 1e-6
                    ));
                }
            }
            Stage::Land {
                since,
                disarmed,
                then_read,
            } => {
                a.throttle = 0.0;
                match disarmed {
                    None if !armed => {
                        self.marks.disarmed = Some(t_us);
                        self.stage = Stage::Land {
                            since,
                            disarmed: Some(t_us),
                            then_read,
                        };
                    }
                    None if t_us - since > LAND_LIMIT_US => {
                        return Err(format!(
                            "the vehicle was not disarmed {} s after the throttle went low (flight \
                             time {:.3} s, height {height_m:.2} m)",
                            LAND_LIMIT_US / S,
                            t_us as f64 * 1e-6
                        ));
                    }
                    Some(d) if t_us >= d + DISARM_SETTLE_US => {
                        if then_read {
                            self.marks.read_back = Some(t_us);
                            self.stage = Stage::ReadBack {
                                since: t_us,
                                centre: false,
                            };
                            a.reads.clone_from(&self.after[0]);
                        } else {
                            self.finish(t_us);
                        }
                    }
                    _ => {}
                }
            }
            Stage::ReadBack { since, centre } => {
                a.throttle = if centre { 0.5 } else { 0.0 };
                let k = ((t_us - since) / S) as usize;
                if (t_us - since).is_multiple_of(S)
                    && let Some(g) = self.after.get(k)
                {
                    a.reads.clone_from(g);
                }
                if t_us >= since + self.after.len() as u64 * S + READ_END_US {
                    self.finish(t_us);
                }
            }
            Stage::Done => {}
        }
        Ok(a)
    }

    /// The vehicle left the box: the flight is stopped where it is. The
    /// gains in force are read, with the sticks centred, so that the report
    /// can say what they were at that moment.
    pub fn abort(&mut self, t_us: u64) {
        if matches!(self.stage, Stage::Done | Stage::ReadBack { .. }) {
            return;
        }
        // A descent that is stopped holds the stick low like a landing does:
        // it is the ground the vehicle is about to be on.
        let in_air = !matches!(
            self.stage,
            Stage::Ground | Stage::Descend { .. } | Stage::Land { .. }
        );
        self.marks.aborted = Some(t_us);
        self.marks.read_back = Some(t_us);
        self.stage = Stage::ReadBack {
            since: t_us,
            centre: in_air,
        };
    }

    fn begin_descent(&mut self, t_us: u64, then_read: bool) {
        self.marks.descent = Some(t_us);
        self.stage = Stage::Descend {
            since: t_us,
            then_read,
        };
    }

    fn begin_land(&mut self, t_us: u64, then_read: bool) {
        self.marks.throttle_low = Some(t_us);
        self.stage = Stage::Land {
            since: t_us,
            disarmed: None,
            then_read,
        };
    }

    fn finish(&mut self, t_us: u64) {
        self.marks.end = Some(t_us);
        self.stage = Stage::Done;
    }
}

// ---- The autopilot --------------------------------------------------------------

/// Why a tuning flight stopped.
#[derive(Debug)]
pub enum FlightError {
    /// ArduCopter's autopilot failed.
    Autopilot(ArduCopterError),
    /// A command to the program could not be sent.
    Send { at_us: u64, source: MavError },
    /// The script cannot go on.
    Script { at_us: u64, reason: String },
}

impl fmt::Display for FlightError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Autopilot(e) => write!(f, "{e}"),
            Self::Send { at_us, source } => write!(
                f,
                "a command could not be sent at flight time {:.3} s: {source}",
                *at_us as f64 * 1e-6
            ),
            Self::Script { reason, .. } => write!(f, "{reason}"),
        }
    }
}

impl std::error::Error for FlightError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Autopilot(e) => Some(e),
            Self::Send { source, .. } => Some(source),
            Self::Script { .. } => None,
        }
    }
}

/// ArduCopter, flown by the script: the radio and the commands come from
/// [`Script`], the exchange is [`ArduCopter`]'s own.
struct Tuner {
    inner: ArduCopter,
    script: Script,
    armed: bool,
    /// The gyro samples of the spectrum window, as sent.
    imu: Vec<[f32; 3]>,
    spectrum: Option<(Spectrum, Verdict)>,
    commands: Vec<(u64, String)>,
}

impl Autopilot<4> for Tuner {
    type Error = FlightError;

    fn period_us(&self) -> u64 {
        STEP_US
    }

    fn preroll_tick(&mut self, k: u64, input: &PrerollInput<'_, 4>) -> Result<(), FlightError> {
        self.inner
            .preroll_tick(k, input)
            .map_err(FlightError::Autopilot)
    }

    fn tick(
        &mut self,
        t: Micros,
        input: &AutopilotInput<'_, 4>,
    ) -> Result<AutopilotOutput<4>, FlightError> {
        let at_us = t.0;
        let height = -input.body.position.z;
        let act = self
            .script
            .tick(at_us, height, self.armed)
            .map_err(|reason| FlightError::Script { at_us, reason })?;
        let send =
            |r: Result<(), MavError>| r.map_err(|source| FlightError::Send { at_us, source });
        if let Some(mode) = act.set_mode {
            send(self.inner.instrument().set_mode(mode))?;
            self.commands.push((at_us, format!("set mode {mode}")));
        }
        for name in &act.reads {
            send(self.inner.instrument().param_request(name))?;
        }
        if !act.reads.is_empty() {
            self.commands
                .push((at_us, format!("read {}", act.reads.join(" "))));
        }
        if let (Some(s), Some((from, to))) = (input.sensors.imu, self.script.spectrum_window())
            && at_us >= from
            && at_us < to
        {
            let g = s.value.gyro;
            self.imu.push([g.x, g.y, g.z]);
        }
        if act.analyse {
            if let Some(s) = spectrum(&self.imu, 1.0e6 / 1000.0) {
                let v = judge(&s);
                self.script.set_peak(v.peak.is_some());
                self.spectrum = Some((s, v));
            } else {
                return Err(FlightError::Script {
                    at_us,
                    reason: format!("only {} gyro samples for the spectrum", self.imu.len()),
                });
            }
        }
        let frame = at_us.is_multiple_of(COMMAND_PERIOD_US).then(|| Stamped {
            t_sample: t,
            t_arrival: t,
            value: RcSample {
                throttle: act.throttle,
                arm_switch: act.arm,
                ..RcSample::default()
            },
        });
        let mut shown = *input;
        shown.rc = frame;
        shown.setpoint = None;
        let out = self.inner.tick(t, &shown).map_err(FlightError::Autopilot)?;
        self.armed = out.armed;
        Ok(out)
    }
}

struct NoPilot;

impl Pilot for NoPilot {
    fn rc(&mut self, _t: Micros) -> Option<RcSample> {
        None
    }
    fn setpoint(&mut self, _t: Micros) -> Option<oxide_core::io::TrajectorySetpoint> {
        None
    }
}

/// What a flight measured of the vehicle: the box, the hover's quality and the
/// excursion in AUTOTUNE.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Stats {
    /// The furthest the vehicle was from where it started, m, and when.
    pub max_horizontal_m: f64,
    pub max_horizontal_at_s: f64,
    /// The highest, m.
    pub max_up_m: f64,
    /// Over the last [`QUALITY_S`] seconds of the hover: RMS tilt, deg, and
    /// the RMS horizontal and vertical distance from the window's mean
    /// position, m.
    pub hover_tilt_rms_deg: f64,
    pub hover_pos_rms_m: f64,
    pub hover_height_rms_m: f64,
    /// In AUTOTUNE: the furthest from the point where it began, m, and the
    /// largest change of height, m, and the largest tilt, deg.
    pub tune_max_horizontal_m: f64,
    pub tune_max_height_change_m: f64,
    pub tune_max_tilt_deg: f64,
    /// The first time the flight was not class safe (ADR-0007 section 4), s,
    /// if it was, and its class.
    pub class_exit_s: Option<f64>,
    /// The class of the flight so far (`None` before the first record).
    pub class: Option<FlightClass>,
    /// From the physics outcome (the gate's own): the furthest from where the
    /// vehicle armed, m, the highest, m, and the largest tilt while armed,
    /// deg.
    pub furthest_from_home_m: f64,
    pub highest_m: f64,
    pub largest_tilt_deg: f64,
    /// The downward speed at the first ground contact after the vehicle had
    /// been in the air, and the greatest at any entry into contact, m/s.
    pub touchdown_speed_mps: Option<f64>,
    pub worst_impact_mps: Option<f64>,
    /// The pack voltage under load, V, averaged over the last
    /// [`PACK_WINDOW_US`] before the descent began.
    pub pack_before_landing_v: Option<f64>,
    /// The descent: the true height when it began, m, and the greatest
    /// downward speed from then until the vehicle was on the ground, m/s.
    pub descent_from_m: Option<f64>,
    pub descent_max_down_mps: Option<f64>,
    /// The true height and the downward speed at the tick the vehicle was
    /// first at or below [`GROUND_M`] after the descent began.
    pub ground_height_m: Option<f64>,
    pub ground_down_mps: Option<f64>,
}

/// The pack at one mark of the flight.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PackAt {
    pub t_s: f64,
    pub volts: f64,
    pub amps: f64,
    pub state_of_charge: f64,
}

/// What a [`Tracker`] ends with: the stats, the track and the pack at the marks.
type TrackerResult = (
    Stats,
    Vec<(f64, f64, f64, f64)>,
    Vec<(&'static str, PackAt)>,
);

/// Accumulates [`Stats`] and watches the class.
#[derive(Debug, Default)]
struct Tracker {
    start: Option<Vec3<f64>>,
    stats: Stats,
    hover_samples: Vec<(f64, f64, f64, f64)>,
    tune_origin: Option<Vec3<f64>>,
    /// Flight time s, north, east, up in m from the start, every 10 s.
    track: Vec<(f64, f64, f64, f64)>,
    /// The gate's own judgement of the flight (ADR-0007 section 4).
    outcome: FlightOutcome,
    /// The last [`PACK_WINDOW_US`] of (flight microseconds, pack volts).
    volts: VecDeque<(u64, f64)>,
    /// The pack at each mark, in the order the marks came.
    pack: Vec<(&'static str, PackAt)>,
}

/// The marks the pack is reported at: a name and the flight time of the mark.
fn pack_marks(m: &Marks) -> [(&'static str, Option<u64>); 7] {
    [
        ("hover reached", m.hover),
        (
            "spectrum window start",
            m.hover.map(|h| h + (HOVER_S - SPECTRUM_S) * S),
        ),
        ("spectrum taken", m.spectrum),
        ("AUTOTUNE start", m.autotune),
        ("descent start", m.descent),
        ("on the ground", m.throttle_low),
        ("disarmed", m.disarmed),
    ]
}

impl Tracker {
    fn tilt_deg(r: &StepRecord<4>) -> f64 {
        let z = r.attitude.rotate(Vec3::new(0.0, 0.0, 1.0));
        let h = Real::sqrt(z.x * z.x + z.y * z.y);
        Real::atan2(h, z.z).to_degrees()
    }

    /// Adds a record (`soc`: the pack's state of charge at it); an error is
    /// the sentence of the flight leaving class safe.
    fn add(&mut self, r: &StepRecord<4>, m: &Marks, soc: f64) -> Result<(), String> {
        let start = *self.start.get_or_insert(r.position);
        let d = r.position - start;
        let horizontal = Real::sqrt(d.x * d.x + d.y * d.y);
        let up = -d.z;
        let t_s = r.t.as_secs_f64();
        self.outcome.add(r);
        let class = self.outcome.class();
        self.stats.class = Some(class);
        self.stats.furthest_from_home_m = self.outcome.max_distance_from_home_m;
        self.stats.highest_m = self.outcome.max_height_m;
        self.stats.largest_tilt_deg = self.outcome.max_tilt_deg;
        self.stats.touchdown_speed_mps = self.outcome.touchdown_speed_mps;
        self.stats.worst_impact_mps = self.outcome.worst_impact_mps;
        // The pack under load: the window before the descent begins, the
        // sample of the tick itself not included.
        if let Some(low) = m.descent
            && self.stats.pack_before_landing_v.is_none()
        {
            let from = low.saturating_sub(PACK_WINDOW_US);
            let (sum, n) = self
                .volts
                .iter()
                .filter(|(t, _)| *t >= from && *t < low)
                .fold((0.0, 0u32), |(s, n), (_, v)| (s + v, n + 1));
            if n > 0 {
                self.stats.pack_before_landing_v = Some(sum / f64::from(n));
            }
        }
        self.volts.push_back((r.t.0, r.battery_v));
        while self
            .volts
            .front()
            .is_some_and(|(t, _)| *t + 2 * PACK_WINDOW_US < r.t.0)
        {
            self.volts.pop_front();
        }
        for (name, at) in pack_marks(m) {
            if let Some(at) = at
                && r.t.0 >= at
                && !self.pack.iter().any(|(n, _)| *n == name)
            {
                self.pack.push((
                    name,
                    PackAt {
                        t_s,
                        volts: r.battery_v,
                        amps: r.battery_a,
                        state_of_charge: soc,
                    },
                ));
            }
        }
        if horizontal > self.stats.max_horizontal_m {
            self.stats.max_horizontal_m = horizontal;
            self.stats.max_horizontal_at_s = t_s;
        }
        self.stats.max_up_m = self.stats.max_up_m.max(up);
        let us = r.t.0;
        if us.is_multiple_of(10 * S) {
            self.track.push((t_s, d.x, d.y, up));
        }
        let mut violation = None;
        if class != FlightClass::Safe && self.stats.class_exit_s.is_none() {
            self.stats.class_exit_s = Some(t_s);
            violation = Some(format!(
                "the flight is not class safe (ADR-0007 section 4: no crash, never more than \
                 {FLY_AWAY_DISTANCE_M} m from where it armed, never more than {FLY_AWAY_HEIGHT_M} m \
                 high): class {class} at flight time {t_s:.3} s, {:.2} m from where it armed, \
                 {:.2} m up, tilt up to {:.1} deg, touchdown speed {:?} m/s, worst impact {:?} m/s \
                 (a crash is more than {CRASH_TOUCHDOWN_MPS} m/s or more than {CRASH_TILT_DEG} deg \
                 of tilt while armed)",
                self.outcome.max_distance_from_home_m,
                self.outcome.max_height_m,
                self.outcome.max_tilt_deg,
                self.outcome.touchdown_speed_mps,
                self.outcome.worst_impact_mps
            ));
        }
        if let Some(h0) = m.hover
            && us >= h0 + (HOVER_S - QUALITY_S) * S
            && us < h0 + HOVER_S * S
        {
            self.hover_samples.push((d.x, d.y, -d.z, Self::tilt_deg(r)));
        }
        if let Some(d) = m.descent {
            // The descent: from the tick it began to the first tick at or
            // below the ground height.
            if r.t.0 >= d && self.stats.ground_height_m.is_none() {
                let down = r.velocity.z;
                self.stats.descent_from_m.get_or_insert(-r.position.z);
                self.stats.descent_max_down_mps = Some(
                    self.stats
                        .descent_max_down_mps
                        .map_or(down, |x| x.max(down)),
                );
                if -r.position.z <= GROUND_M {
                    self.stats.ground_height_m = Some(-r.position.z);
                    self.stats.ground_down_mps = Some(down);
                }
            }
        }
        if m.autotune.is_some() && m.descent.is_none() {
            let o = *self.tune_origin.get_or_insert(r.position);
            let e = r.position - o;
            self.stats.tune_max_horizontal_m = self
                .stats
                .tune_max_horizontal_m
                .max(Real::sqrt(e.x * e.x + e.y * e.y));
            self.stats.tune_max_height_change_m =
                self.stats.tune_max_height_change_m.max(e.z.abs());
            self.stats.tune_max_tilt_deg = self.stats.tune_max_tilt_deg.max(Self::tilt_deg(r));
        }
        violation.map_or(Ok(()), Err)
    }

    fn finish(mut self) -> TrackerResult {
        let n = self.hover_samples.len() as f64;
        if n > 0.0 {
            let mean = |f: fn(&(f64, f64, f64, f64)) -> f64| {
                self.hover_samples.iter().map(f).sum::<f64>() / n
            };
            let (mx, my, mz) = (mean(|s| s.0), mean(|s| s.1), mean(|s| s.2));
            let pos = self
                .hover_samples
                .iter()
                .map(|s| (s.0 - mx) * (s.0 - mx) + (s.1 - my) * (s.1 - my))
                .sum::<f64>()
                / n;
            let height = self
                .hover_samples
                .iter()
                .map(|s| (s.2 - mz) * (s.2 - mz))
                .sum::<f64>()
                / n;
            let tilt = self.hover_samples.iter().map(|s| s.3 * s.3).sum::<f64>() / n;
            self.stats.hover_pos_rms_m = Real::sqrt(pos);
            self.stats.hover_height_rms_m = Real::sqrt(height);
            self.stats.hover_tilt_rms_deg = Real::sqrt(tilt);
        }
        (self.stats, self.track, self.pack)
    }
}

/// What one flight needs.
#[derive(Clone, Debug)]
pub struct FlightPlan {
    /// The program, the directory the flight's own directory is made in, and
    /// the instance number.
    pub binary: PathBuf,
    pub root: PathBuf,
    pub flight_name: String,
    pub instance: u8,
    pub seed: u64,
    /// The parameter files in order and the parameters after them.
    pub files: Vec<PathBuf>,
    pub extra: Vec<(String, f64)>,
    pub stop_on_peak: bool,
    pub stop_after_hover: bool,
    /// Whether the read-back after the disarm also asks for the six names of
    /// the notch ([`NOTCH_NAMES`]): the last flight of a procedure with a
    /// notch.
    pub read_notch: bool,
    /// How long AUTOTUNE is given, s ([`AUTOTUNE_S`] in the procedure).
    pub autotune_s: u64,
    /// Whether leaving class safe ends the flight (it does in the procedure).
    pub enforce_class: bool,
}

/// What a flight left behind. `failure` is the sentence of a flight that
/// ended early (the box, the climb, no disarm); everything it had until then is
/// kept.
#[derive(Clone, Debug)]
pub struct FlightResult {
    pub plan: FlightPlan,
    pub audit: FlightAudit,
    pub trace_hash: u64,
    pub marks: Marks,
    pub commands: Vec<(u64, String)>,
    pub spectrum: Option<(Spectrum, Verdict)>,
    pub stats: Stats,
    /// The pack at the marks of the flight: voltage under load, current and
    /// state of charge.
    pub pack: Vec<(&'static str, PackAt)>,
    /// Flight time s, then north, east and up in m from where the flight
    /// started, every 10 s.
    pub track: Vec<(f64, f64, f64, f64)>,
    /// Simulated flight time at the end, s, and the wall time of the flight.
    pub sim_s: f64,
    pub wall_s: f64,
    pub failure: Option<String>,
}

/// Why a flight could not be run at all.
#[derive(Debug)]
pub enum TuneError {
    /// The simulation could not be built.
    Setup(SimError),
    /// The program could not be started, or its pre-roll failed.
    Start(String),
    /// The flight failed in a way that left no result.
    Flight(String),
    /// A file could not be written.
    Write {
        path: PathBuf,
        source: std::io::Error,
    },
    /// The procedure ended without a result; the report says why.
    Failed(String),
}

impl fmt::Display for TuneError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Setup(e) => write!(f, "cannot set up the simulation: {e}"),
            Self::Start(e) => write!(f, "the flight did not start: {e}"),
            Self::Flight(e) => write!(f, "the flight failed: {e}"),
            Self::Write { path, source } => {
                write!(f, "cannot write {}: {source}", path.display())
            }
            Self::Failed(e) => write!(f, "the tuning failed: {e}"),
        }
    }
}

impl std::error::Error for TuneError {}

/// Flies one tuning flight of `plan`.
pub fn fly_flight(plan: &FlightPlan) -> Result<FlightResult, TuneError> {
    let airframe = Airframe::reference_quad().map_err(TuneError::Setup)?;
    let mut config = ArduCopterConfig::new(&plan.binary, &plan.root, plan.instance);
    config.flight_name = Some(plan.flight_name.clone());
    config.param_files.clone_from(&plan.files);
    config.extra_params.clone_from(&plan.extra);
    config.brain = BrainLink::None;
    let options = bridged_options(SimOptions::realistic(plan.seed), &mut config);
    let inner = ArduCopter::start(config).map_err(|e| TuneError::Start(e.to_string()))?;
    let tuner = Tuner {
        inner,
        script: {
            let script = Script::new(plan.stop_on_peak, plan.stop_after_hover, plan.autotune_s);
            if plan.read_notch {
                script.with_notch_read()
            } else {
                script
            }
        },
        armed: false,
        imu: Vec::new(),
        spectrum: None,
        commands: Vec::new(),
    };
    let mut sim =
        Simulation::with_autopilot(&airframe, tuner, NoPilot, options).map_err(TuneError::Setup)?;
    let wall = Instant::now();
    sim.preroll(PREROLL_STEPS)
        .map_err(|e: PrerollError<FlightError>| TuneError::Start(format!("pre-roll: {e}")))?;
    let mut tracker = Tracker::default();
    let mut failure = None;
    let mut error = None;
    // A tuning flight is about 1100 s; a flight that is still going at
    // 1500 s has lost its script.
    let limit = Micros(1500 * S);
    while !sim.autopilot().script.done() && sim.time() < limit {
        match sim.try_step() {
            Ok(r) => {
                let marks = sim.autopilot().script.marks();
                let soc = sim.battery_state_of_charge();
                if let Err(e) = tracker.add(&r, &marks, soc)
                    && plan.enforce_class
                {
                    failure = Some(e);
                    let now = sim.time().0;
                    sim.autopilot_mut().script.abort(now);
                }
            }
            Err(FlightError::Script { reason, .. }) => {
                failure = Some(reason);
                break;
            }
            Err(e) => {
                error = Some(e);
                break;
            }
        }
    }
    if failure.is_none() && error.is_none() && !sim.autopilot().script.done() {
        failure = Some("the script did not finish within 1500 s of flight time".to_string());
    }
    let sim_s = sim.time().as_secs_f64();
    let trace_hash = sim.trace_hash();
    let tuner = sim.into_autopilot();
    let Tuner {
        inner,
        script,
        spectrum,
        commands,
        ..
    } = tuner;
    let audit = inner.finish();
    let (stats, track, pack) = tracker.finish();
    if let Some(e) = error {
        return Err(TuneError::Flight(e.to_string()));
    }
    let audit = audit.map_err(|e| TuneError::Flight(e.to_string()))?;
    Ok(FlightResult {
        plan: plan.clone(),
        audit,
        trace_hash,
        marks: script.marks(),
        commands,
        spectrum,
        stats,
        pack,
        track,
        sim_s,
        wall_s: wall.elapsed().as_secs_f64(),
        failure,
    })
}

// ---- Everything below reads what the program said: after the flight ---------

/// A flight time, s, of a program time, µs.
fn flight_s(program_us: u64) -> f64 {
    (program_us as f64 - (PREROLL_STEPS * STEP_US) as f64) * 1e-6
}

/// The status texts of a flight with their flight times, s.
pub fn texts(audit: &FlightAudit) -> Vec<(f64, String)> {
    audit
        .status_texts
        .iter()
        .map(|(t, _, s)| (flight_s(*t), s.clone()))
        .collect()
}

/// The last value of each parameter the program reported with a program time
/// in `[from_us, to_us)`.
pub fn echoes(audit: &FlightAudit, from_us: u64, to_us: u64) -> BTreeMap<String, f32> {
    let mut m = BTreeMap::new();
    for h in &audit.heard {
        if h.at_us >= from_us
            && h.at_us < to_us
            && let crate::mav::MavEvent::ParamValue { name, value, .. } = &h.event
        {
            m.insert(name.clone(), *value);
        }
    }
    m
}

/// Program time of a flight time in microseconds.
pub fn program_us(us: u64) -> u64 {
    PREROLL_STEPS * STEP_US + us
}

/// The values read before the flight and after the disarm: only the names
/// asked for (the program also reports parameters of its own accord when it
/// saves them).
pub fn read_back(r: &FlightResult) -> (BTreeMap<String, f32>, BTreeMap<String, f32>) {
    let mut before = echoes(&r.audit, 0, program_us(r.marks.arm));
    let asked = before_names();
    before.retain(|n, _| asked.contains(&n.as_str()));
    let mut after = r
        .marks
        .read_back
        .map_or_else(BTreeMap::new, |t| echoes(&r.audit, program_us(t), u64::MAX));
    after.retain(|n, _| GAIN_NAMES.contains(&n.as_str()));
    (before, after)
}

/// The axes the tuner reports on, as its messages name them: the roll-and-pitch
/// flight (`AUTOTUNE_AXES` 3) tunes the first two, the yaw flight (4) the last.
pub const ROLL_PITCH_AXES: [&str; 2] = ["Roll", "Pitch"];
pub const YAW_AXES: [&str; 1] = ["Yaw"];
const AXES_ROLL_PITCH: f64 = 3.0;
const AXES_YAW: f64 = 4.0;

/// The gate's low-battery threshold, V: the runner's `BATT_LOW_VOLT` (a pilot
/// lands before the low-battery alarm).
pub fn low_battery_volt() -> f64 {
    crate::runner::battery_low_volt()
}

/// The check on one tuning flight's texts and read-back: the tuner's success
/// message for each of `axes` and its saved-gains message, a value for every
/// name of [`GAIN_NAMES`], and the pack: its voltage under load, averaged over
/// the last second before the descent began (`pack_before_landing_v`), above
/// `low_battery_v`. An error is a sentence.
pub fn check_axes(
    texts: &[(f64, String)],
    after: &BTreeMap<String, f32>,
    pack_before_landing_v: Option<f64>,
    low_battery_v: f64,
    axes: &[&str],
) -> Result<(), String> {
    for axis in axes {
        // The yaw axis names its stage: "AutoTune: Yaw(E) complete".
        let head = format!("AutoTune: {axis}");
        if !texts
            .iter()
            .any(|(_, t)| t.starts_with(&head) && t.ends_with(" complete"))
        {
            return Err(format!(
                "the tuner did not print \"{head}... complete\" (the success message of that axis)"
            ));
        }
    }
    if !texts.iter().any(|(_, t)| t == "AutoTune: Success") {
        return Err("the tuner did not print \"AutoTune: Success\"".into());
    }
    if !texts
        .iter()
        .any(|(_, t)| t.starts_with("AutoTune: Saved gains for"))
    {
        return Err("the tuner did not print \"AutoTune: Saved gains for ...\"".into());
    }
    let missing: Vec<&str> = GAIN_NAMES
        .iter()
        .copied()
        .filter(|n| !after.contains_key(*n))
        .collect();
    if !missing.is_empty() {
        return Err(format!(
            "the read-back has no value for {}",
            missing.join(", ")
        ));
    }
    match pack_before_landing_v {
        None => Err("the pack voltage before the landing was not measured".into()),
        Some(v) if v.is_nan() || low_battery_v.is_nan() || v <= low_battery_v => Err(format!(
            "the pack voltage under load, averaged over the last second before the descent began, \
             was {v:.3} V: not above the low-battery threshold {low_battery_v:.3} V (the \
             vehicle landed on a pack that was too empty)"
        )),
        Some(_) => Ok(()),
    }
}

/// The whole check of one tuning flight: [`check_axes`] and [`check_setup`] on
/// what that flight read back before it flew.
pub fn check_flight(
    texts: &[(f64, String)],
    before: &BTreeMap<String, f32>,
    after: &BTreeMap<String, f32>,
    pack_before_landing_v: Option<f64>,
    low_battery_v: f64,
    axes: &[&str],
) -> Result<(), String> {
    check_axes(texts, after, pack_before_landing_v, low_battery_v, axes)
        .and_then(|()| check_setup(before))
}

/// The yaw flight changes nothing but yaw: every one of the 29 values read
/// back after it (`second`) equals what the roll-and-pitch flight saved
/// (`first`), except the yaw axis's own (the names that contain `YAW`, and
/// `ATC_ACC_Y_MAX`) and `MOT_THST_HOVER`. A value missing from either is a
/// difference.
pub fn check_carried(
    first: &BTreeMap<String, f32>,
    second: &BTreeMap<String, f32>,
) -> Result<(), String> {
    for name in GAIN_NAMES {
        if name.contains("YAW") || name == "ATC_ACC_Y_MAX" || name == "MOT_THST_HOVER" {
            continue;
        }
        match (first.get(name), second.get(name)) {
            (Some(a), Some(b)) if a == b => {}
            (a, b) => {
                return Err(format!(
                    "{name} was {a:?} after the roll-and-pitch flight and {b:?} after the yaw \
                     flight: the yaw flight must change nothing but yaw"
                ));
            }
        }
    }
    Ok(())
}

/// Each of the six notch values read back after the last flight equals the one
/// that was set, as a 32-bit float (the program holds 32-bit floats).
pub fn check_notch_read(set: &[(String, f64)], read: &BTreeMap<String, f32>) -> Result<(), String> {
    for (name, value) in set {
        #[allow(clippy::cast_possible_truncation)] // the program holds 32-bit floats
        let want = *value as f32;
        match read.get(name) {
            Some(got) if *got == want => {}
            Some(got) => {
                return Err(format!(
                    "{name} was set to {want} and read back as {got}: the notch is not what the \
                     flights were told"
                ));
            }
            None => return Err(format!("{name} was not read back")),
        }
    }
    Ok(())
}

/// The check that the program took `setup.parm`: every name of it, read back
/// before the flight (`before`), has the file's value. A program that ignored a
/// line of the guide's set-up (the 80 Hz gyro filter, say) would otherwise
/// tune and be flown on another set-up than the files say.
pub fn check_setup(before: &BTreeMap<String, f32>) -> Result<(), String> {
    let setup = parm::parse(parm::SETUP).map_err(|e| format!("setup.parm: {e}"))?;
    let mut wrong = Vec::new();
    for (name, want) in setup {
        #[allow(clippy::cast_possible_truncation)] // the program holds 32-bit floats
        let want = want as f32;
        match before.get(&name) {
            Some(got) if *got == want => {}
            Some(got) => wrong.push(format!("{name} read back {got}, setup.parm says {want}")),
            None => wrong.push(format!("{name} was not read back")),
        }
    }
    if wrong.is_empty() {
        Ok(())
    } else {
        Err(format!(
            "the program did not take setup.parm: {}",
            wrong.join("; ")
        ))
    }
}

/// The header of a file the procedure writes: what made it, and the binary.
fn header(
    s: &mut String,
    title: &str,
    what: &str,
    binary_sha256: &str,
    seed: u64,
    time_of: &str,
    flight_s: f64,
) {
    let _ = writeln!(s, "# {title}");
    let _ = writeln!(
        s,
        "# Written by `oxide-bridge tune --seed {seed}` (crates/oxide-bridge/src/tune.rs): {what}"
    );
    let _ = writeln!(s, "# binary sha256 {binary_sha256}");
    let _ = writeln!(s, "# seed {seed}");
    let _ = writeln!(s, "# simulated flight time of {time_of} {flight_s:.3} s");
}

/// The text of `tuned-gains.parm`: a header that says what made it, every
/// name of the read-back with its value as the program reported it (the
/// shortest decimal that gives the same 32-bit float), sorted by name, one per
/// line, and, when a notch was set (`notch` is not empty), the six values of
/// [`NOTCH_NAMES`] read back after the last flight, in that order, after the
/// gains. Every value in it was read back from the program.
///
/// The header holds no version of the program (the checksum names it) and
/// nothing that differs between runs of the same procedure
/// (no commit, no wall time), so a fresh run writes the committed file byte
/// for byte, header included. `flight_s` is the simulated time of every
/// tuning flight together.
pub fn tuned_file(
    after: &BTreeMap<String, f32>,
    notch: &BTreeMap<String, f32>,
    binary_sha256: &str,
    seed: u64,
    flight_s: f64,
) -> String {
    let mut s = String::new();
    header(
        &mut s,
        "ArduCopter parameters after the tuning of ADR-0009.",
        "the values read back from the program after the automatic tuner saved its gains (roll and pitch in one flight, yaw in a second flight that started from the gains of the first).",
        binary_sha256,
        seed,
        "the procedure",
        flight_s,
    );
    let _ = writeln!(
        s,
        "# harmonic notch: {}",
        if notch.is_empty() {
            "none"
        } else {
            "set, its six values follow the gains"
        }
    );
    let _ = writeln!(
        s,
        "# Never edited by hand: a test flies the procedure again and compares."
    );
    for (name, value) in after {
        let _ = writeln!(s, "{name} {value}");
    }
    for name in NOTCH_NAMES {
        if let Some(value) = notch.get(name) {
            let _ = writeln!(s, "{name} {value}");
        }
    }
    s
}

/// The text of `tuned-gains-1.parm`: what the roll-and-pitch flight saved, read
/// back, for the yaw flight to load after the starting gains.
pub fn roll_pitch_file(
    after: &BTreeMap<String, f32>,
    binary_sha256: &str,
    seed: u64,
    flight_s: f64,
) -> String {
    let mut s = String::new();
    header(
        &mut s,
        "ArduCopter parameters after the roll-and-pitch flight of ADR-0009's tuning.",
        "the values read back from the program after the tuner saved its gains for roll and pitch. The yaw flight loads this file after the starting gains.",
        binary_sha256,
        seed,
        "the roll-and-pitch flight",
        flight_s,
    );
    for (name, value) in after {
        let _ = writeln!(s, "{name} {value}");
    }
    s
}

/// A path of the output directory.
pub fn out_path(dir: &Path, name: &str) -> PathBuf {
    dir.join(name)
}

// ---- The procedure -------------------------------------------------------------

/// What `oxide-bridge tune` is given.
#[derive(Clone, Debug)]
pub struct TuneConfig {
    pub binary: PathBuf,
    pub out: PathBuf,
    pub instance: u8,
    pub seed: u64,
}

/// The notch the hover of the first flight flown decided.
#[derive(Clone, Debug, PartialEq)]
pub struct Notch {
    /// The flight (counted from 1) whose hover decided it.
    pub flight: usize,
    /// The line that was a peak.
    pub line: Line,
    /// The learned hover thrust read at the end of that hover: the notch's
    /// reference and the roll-and-pitch flight's `MOT_THST_HOVER`.
    pub reference: f64,
    /// [`notch_params`] of the line and the reference.
    pub params: Vec<(String, f64)>,
}

/// What the procedure produced.
#[derive(Clone, Debug)]
pub struct Outcome {
    /// The flights, in order.
    pub flights: Vec<FlightResult>,
    /// The values read before the first flight and after the last disarm.
    pub before: BTreeMap<String, f32>,
    pub after: BTreeMap<String, f32>,
    /// The six notch values read back after the last flight (empty when no
    /// notch was set).
    pub notch_read: BTreeMap<String, f32>,
    /// The text of `tuned-gains.parm`.
    pub tuned: String,
    /// The notch the first hover decided, if it showed a peak. Its reference,
    /// the learned hover thrust read from the program, decides a parameter of
    /// the flights after the first.
    pub notch: Option<Notch>,
    pub report: String,
}

impl Outcome {
    /// How many values `tuned-gains.parm` holds: its lines that are not
    /// comments.
    pub fn values_written(&self) -> usize {
        self.tuned
            .lines()
            .filter(|l| !l.is_empty() && !l.starts_with('#'))
            .count()
    }
}

/// The parameter files of the tuning flights, in order: the set-up of the
/// airframe and its sensors, the guide's set-up, and the starting gains.
pub fn tuning_files() -> Vec<PathBuf> {
    vec![
        parm::base_path(),
        parm::setup_path(),
        parm::starting_gains_path(),
    ]
}

/// A hover check: the hover of the script, of `files`, and its quality.
pub fn hover_check(
    binary: &Path,
    root: &Path,
    instance: u8,
    seed: u64,
    files: Vec<PathBuf>,
) -> Result<FlightResult, TuneError> {
    let flight = fly_flight(&FlightPlan {
        binary: binary.to_path_buf(),
        root: root.to_path_buf(),
        flight_name: "hover-check".into(),
        instance,
        seed,
        files,
        extra: Vec::new(),
        stop_on_peak: false,
        stop_after_hover: true,
        read_notch: false,
        autotune_s: AUTOTUNE_S,
        enforce_class: true,
    })?;
    let cfg = TuneConfig {
        binary: binary.to_path_buf(),
        out: root.to_path_buf(),
        instance,
        seed,
    };
    let (before, _) = read_back(&flight);
    let text = report_text(
        &cfg,
        &Summary {
            flights: core::slice::from_ref(&flight),
            before: &before,
            after: &BTreeMap::new(),
            notch: None,
            notch_read: &BTreeMap::new(),
            problem: flight.failure.as_deref(),
            wall_s: flight.wall_s,
        },
    );
    let path = out_path(root, "report.txt");
    std::fs::write(&path, text).map_err(|source| TuneError::Write { path, source })?;
    Ok(flight)
}

/// Which of the two tuning flights a flight is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Part<'a> {
    /// Roll and pitch (`AUTOTUNE_AXES` 3), from the starting gains.
    RollPitch,
    /// Yaw (`AUTOTUNE_AXES` 4), after the starting gains with the gains the
    /// roll-and-pitch flight saved (`roll_pitch_file`).
    Yaw { roll_pitch_file: &'a Path },
}

impl Part<'_> {
    /// The axes the flight tunes, as the tuner's messages name them.
    pub fn axes(&self) -> &'static [&'static str] {
        match self {
            Self::RollPitch => &ROLL_PITCH_AXES,
            Self::Yaw { .. } => &YAW_AXES,
        }
    }
}

/// The plan of flight number `number` of the procedure: which part it is and
/// the notch the first hover decided (`None` while it is not decided or when
/// there was no peak).
///
/// - **Files:** the base file, the set-up file and the starting gains; the yaw
///   flight adds the roll-and-pitch flight's saved gains after them.
/// - **Extra parameters:** `AUTOTUNE_AXES`; then, with a notch, its six
///   values; the roll-and-pitch flight adds the learned hover thrust as
///   `MOT_THST_HOVER` (the yaw flight has it from the file it loads).
/// - **Stops on a peak** only where the notch is not decided yet: the
///   roll-and-pitch flight without a notch, which is the first flight flown.
///   Every later hover is reported and decides nothing.
/// - **Reads the notch back** after the last flight, the yaw flight, when a
///   notch was set.
pub fn flight_plan(
    cfg: &TuneConfig,
    number: usize,
    part: Part<'_>,
    notch: Option<&Notch>,
) -> FlightPlan {
    let mut files = tuning_files();
    let mut extra = Vec::new();
    match part {
        Part::RollPitch => {
            extra.push(("AUTOTUNE_AXES".to_string(), AXES_ROLL_PITCH));
            if let Some(n) = notch {
                extra.extend(n.params.iter().cloned());
                extra.push(("MOT_THST_HOVER".to_string(), n.reference));
            }
        }
        Part::Yaw { roll_pitch_file } => {
            files.push(roll_pitch_file.to_path_buf());
            extra.push(("AUTOTUNE_AXES".to_string(), AXES_YAW));
            if let Some(n) = notch {
                extra.extend(n.params.iter().cloned());
            }
        }
    }
    FlightPlan {
        binary: cfg.binary.clone(),
        root: cfg.out.clone(),
        flight_name: format!("flight-{number}"),
        instance: cfg.instance,
        seed: cfg.seed,
        files,
        extra,
        stop_on_peak: part == Part::RollPitch && notch.is_none(),
        stop_after_hover: false,
        read_notch: matches!(part, Part::Yaw { .. }) && notch.is_some(),
        autotune_s: AUTOTUNE_S,
        enforce_class: true,
    }
}

/// The learned hover thrust a flight read at the end of its hover.
fn learned_thrust(flight: &FlightResult) -> Option<f64> {
    flight.marks.spectrum.and_then(|t| {
        echoes(
            &flight.audit,
            program_us(t),
            program_us(t + AFTER_SPECTRUM_US),
        )
        .get("MOT_THST_HOVER")
        .map(|v| f64::from(*v))
    })
}

/// The six notch values a flight read back after its disarm.
fn notch_back(r: &FlightResult) -> BTreeMap<String, f32> {
    let mut m = r
        .marks
        .read_back
        .map_or_else(BTreeMap::new, |t| echoes(&r.audit, program_us(t), u64::MAX));
    m.retain(|n, _| NOTCH_NAMES.contains(&n.as_str()));
    m
}

/// The whole procedure, flown by [`fly_flight`]. Files are written into
/// `cfg.out`: `report.txt`, `spectrum.csv` (the hover that decided the notch),
/// `tuned-gains-1.parm` once the roll-and-pitch flight is accepted and, when
/// every check of every flight holds, `tuned-gains.parm`; the flights' own
/// directories are made there too.
pub fn tune(cfg: &TuneConfig) -> Result<Outcome, TuneError> {
    procedure(cfg, &mut fly_flight)
}

/// [`tune`] with the flights flown by `fly`.
fn procedure(
    cfg: &TuneConfig,
    fly: &mut dyn FnMut(&FlightPlan) -> Result<FlightResult, TuneError>,
) -> Result<Outcome, TuneError> {
    std::fs::create_dir_all(&cfg.out).map_err(|source| TuneError::Write {
        path: cfg.out.clone(),
        source,
    })?;
    let wall = Instant::now();
    let write = |name: &str, text: &str| {
        let path = out_path(&cfg.out, name);
        std::fs::write(&path, text).map_err(|source| TuneError::Write { path, source })
    };
    let mut flights: Vec<FlightResult> = Vec::new();
    let mut notch: Option<Notch> = None;
    let mut problem: Option<String> = None;
    // What the roll-and-pitch flight saved, and the file it was written to.
    let mut saved: Option<(BTreeMap<String, f32>, PathBuf)> = None;
    let mut after = BTreeMap::new();
    let mut notch_read = BTreeMap::new();
    loop {
        let number = flights.len() + 1;
        let part = match &saved {
            None => Part::RollPitch,
            Some((_, path)) => Part::Yaw {
                roll_pitch_file: path,
            },
        };
        let plan = flight_plan(cfg, number, part, notch.as_ref());
        let flight = fly(&plan)?;
        let (before, a) = read_back(&flight);
        let line = flight.spectrum.as_ref().and_then(|(_, v)| v.peak);
        let thrust = learned_thrust(&flight);
        // The first flight flown lands after its hover when the hover shows a
        // peak: nothing of it is checked but what main's procedure checked
        // of it, the set-up file.
        let stopped = plan.stop_on_peak && flight.failure.is_none() && line.is_some();
        let mut p = flight.failure.clone();
        if p.is_none() {
            p = if stopped {
                check_setup(&before)
            } else {
                check_flight(
                    &texts(&flight.audit),
                    &before,
                    &a,
                    flight.stats.pack_before_landing_v,
                    low_battery_volt(),
                    part.axes(),
                )
            }
            .err();
        }
        let read = notch_back(&flight);
        if p.is_none()
            && let (Part::Yaw { .. }, Some((first, _))) = (part, &saved)
        {
            p = check_carried(first, &a).err();
        }
        if p.is_none()
            && plan.read_notch
            && let Some(n) = &notch
        {
            p = check_notch_read(&n.params, &read).err();
        }
        let sha = flight.audit.binary_sha256.clone();
        let sim_s = flight.sim_s;
        flights.push(flight);
        if let Some(p) = p {
            problem = Some(format!("flight {number}: {p}"));
            after = a;
            break;
        }
        if stopped {
            let (Some(line), Some(thrust)) = (line, thrust) else {
                problem = Some(format!(
                    "flight {number}: a peak was found but the learned hover thrust was not read"
                ));
                break;
            };
            notch = Some(Notch {
                flight: number,
                line,
                reference: thrust,
                params: notch_params(line.freq_hz, thrust),
            });
            continue;
        }
        match part {
            Part::RollPitch => {
                let name = "tuned-gains-1.parm";
                write(name, &roll_pitch_file(&a, &sha, cfg.seed, sim_s))?;
                saved = Some((a, out_path(&cfg.out, name)));
            }
            Part::Yaw { .. } => {
                after = a;
                notch_read = read;
                break;
            }
        }
    }
    if let Some((s, _)) = flights.first().and_then(|f| f.spectrum.as_ref()) {
        write("spectrum.csv", &spectrum_csv(s))?;
    }
    let before = flights.first().map(|f| read_back(f).0).unwrap_or_default();
    let flight_s: f64 = flights.iter().map(|f| f.sim_s).sum();
    let binary_sha = flights
        .last()
        .map(|f| f.audit.binary_sha256.clone())
        .unwrap_or_default();
    let tuned = tuned_file(&after, &notch_read, &binary_sha, cfg.seed, flight_s);
    let report = report_text(
        cfg,
        &Summary {
            flights: &flights,
            before: &before,
            after: &after,
            notch: notch.as_ref(),
            notch_read: &notch_read,
            problem: problem.as_deref(),
            wall_s: wall.elapsed().as_secs_f64(),
        },
    );
    write("report.txt", &report)?;
    if let Some(p) = problem {
        return Err(TuneError::Failed(format!(
            "{p} (the report is {})",
            out_path(&cfg.out, "report.txt").display()
        )));
    }
    write("tuned-gains.parm", &tuned)?;
    Ok(Outcome {
        flights,
        before,
        after,
        notch_read,
        tuned,
        notch,
        report,
    })
}

fn secs(us: Option<u64>) -> String {
    us.map_or_else(|| "-".to_string(), |u| format!("{:.3} s", u as f64 * 1e-6))
}

/// What the report is of.
struct Summary<'a> {
    flights: &'a [FlightResult],
    /// Read before the first flight, and after the last disarm.
    before: &'a BTreeMap<String, f32>,
    after: &'a BTreeMap<String, f32>,
    notch: Option<&'a Notch>,
    notch_read: &'a BTreeMap<String, f32>,
    problem: Option<&'a str>,
    wall_s: f64,
}

/// The axes a flight was asked to tune, from its `AUTOTUNE_AXES`.
fn axes_of(plan: &FlightPlan) -> &'static str {
    match plan.extra.iter().find(|(n, _)| n == "AUTOTUNE_AXES") {
        Some((_, v)) if *v == AXES_ROLL_PITCH => "Roll Pitch (AUTOTUNE_AXES 3)",
        Some((_, v)) if *v == AXES_YAW => "Yaw (AUTOTUNE_AXES 4)",
        _ => "all (the program's own AUTOTUNE_AXES)",
    }
}

fn report_text(cfg: &TuneConfig, sum: &Summary<'_>) -> String {
    let Summary {
        flights,
        before,
        after,
        notch,
        notch_read,
        problem,
        wall_s,
    } = *sum;
    let mut s = String::new();
    let _ = writeln!(s, "oxide-bridge tune: report");
    let _ = writeln!(s, "  commit {}", env!("OXIDE_GIT_HASH"));
    let _ = writeln!(s, "  seed {}, instance {}", cfg.seed, cfg.instance);
    if let Some(f) = flights.first() {
        let _ = writeln!(s, "  binary sha256 {}", f.audit.binary_sha256);
    }
    match problem {
        Some(p) => {
            let _ = writeln!(s, "  RESULT: FAILED: {p}");
        }
        None => {
            let _ = writeln!(
                s,
                "  RESULT: the tuner ran to its success and saved-gains messages"
            );
        }
    }
    let _ = writeln!(
        s,
        "  procedure: two tuning flights from POSHOLD (roll and pitch, then yaw on the gains the \
         first saved), the notch decided once, on the hover of the first flight flown"
    );
    match notch {
        Some(n) => {
            let _ = writeln!(
                s,
                "  notch: decided by the hover of flight {}: PEAK at {:.2} Hz (axis {}, ratio {:.2}); \
                 that flight landed after its hover; the roll-and-pitch flight was flown again from \
                 the start with the notch and MOT_THST_HOVER {}, and the yaw flight flies the same \
                 notch",
                n.flight,
                n.line.freq_hz,
                ["x", "y", "z"][n.line.axis],
                n.line.ratio,
                n.reference
            );
        }
        None => {
            let _ = writeln!(
                s,
                "  notch: none; the hover of flight 1 showed no peak, and no flight has a notch"
            );
        }
    }
    let _ = writeln!(
        s,
        "  the tuner's settings as read back from the program (flight 1): {}",
        DEFAULT_NAMES
            .iter()
            .filter(|n| n.starts_with("AUTOTUNE_"))
            .map(|n| format!(
                "{n} {}",
                before
                    .get(*n)
                    .map_or_else(|| "-".to_string(), ToString::to_string)
            ))
            .collect::<Vec<_>>()
            .join(", ")
    );
    let sim: f64 = flights.iter().map(|f| f.sim_s).sum();
    let _ = writeln!(
        s,
        "  simulated flight time of the procedure {sim:.1} s (and {} s of pre-roll per flight), \
         wall time {wall_s:.1} s",
        PREROLL_STEPS * STEP_US / S
    );
    for (i, f) in flights.iter().enumerate() {
        let m = &f.marks;
        let _ = writeln!(s, "flight {} ({})", i + 1, f.plan.flight_name);
        let _ = writeln!(
            s,
            "  tunes: {}",
            if f.plan.stop_after_hover {
                "nothing (a hover check)".to_string()
            } else if f.plan.stop_on_peak {
                format!(
                    "{}; a peak in its hover lands it after the hover instead",
                    axes_of(&f.plan)
                )
            } else {
                axes_of(&f.plan).to_string()
            }
        );
        let _ = writeln!(
            s,
            "  its hover decides the notch: {}",
            if f.plan.stop_on_peak {
                "yes (a peak lands it after the hover)"
            } else {
                "no (its spectrum is reported and decides nothing)"
            }
        );
        let _ = writeln!(
            s,
            "  parameter files {}",
            f.plan
                .files
                .iter()
                .map(|p| p
                    .file_name()
                    .map_or(String::new(), |n| n.to_string_lossy().into_owned()))
                .collect::<Vec<_>>()
                .join(" ")
        );
        if !f.plan.extra.is_empty() {
            let _ = writeln!(
                s,
                "  extra parameters {}",
                f.plan
                    .extra
                    .iter()
                    .map(|(n, v)| format!("{n}={v}"))
                    .collect::<Vec<_>>()
                    .join(" ")
            );
        }
        let _ = writeln!(
            s,
            "  trace hash {:016x}, simulated {:.3} s, wall {:.1} s",
            f.trace_hash, f.sim_s, f.wall_s
        );
        let _ = writeln!(
            s,
            "  stages (flight time): POSHOLD command {}, arm switch {}, throttle up {}, hover {}, spectrum {}, AUTOTUNE {}, descent {}, on the ground (throttle low) {}, disarmed (pulses) {}, read-back {}, end {}",
            secs(Some(START_MODE_AT_US)),
            secs(Some(m.arm)),
            secs(m.climb),
            secs(m.hover),
            secs(m.spectrum),
            secs(m.autotune),
            secs(m.descent),
            secs(m.throttle_low),
            secs(m.disarmed),
            secs(m.read_back),
            secs(m.end)
        );
        let st = &f.stats;
        let _ = writeln!(
            s,
            "  furthest {:.2} m out (at {:.1} s), highest {:.2} m; hover (last {QUALITY_S} s): tilt rms {:.3} deg, \
             position rms {:.3} m about its mean, height rms {:.3} m; in AUTOTUNE: furthest {:.2} m from where it began, \
             height change up to {:.2} m, tilt up to {:.2} deg",
            st.max_horizontal_m,
            st.max_horizontal_at_s,
            st.max_up_m,
            st.hover_tilt_rms_deg,
            st.hover_pos_rms_m,
            st.hover_height_rms_m,
            st.tune_max_horizontal_m,
            st.tune_max_height_change_m,
            st.tune_max_tilt_deg
        );
        let _ = writeln!(
            s,
            "  physics outcome (the gate's own): furthest from where it armed {:.2} m (limit {FLY_AWAY_DISTANCE_M} m), \
             highest {:.2} m (limit {FLY_AWAY_HEIGHT_M} m), largest tilt while armed {:.2} deg, \
             touchdown speed {:?} m/s, worst impact {:?} m/s",
            st.furthest_from_home_m,
            st.highest_m,
            st.largest_tilt_deg,
            st.touchdown_speed_mps,
            st.worst_impact_mps
        );
        match (st.class_exit_s, st.class) {
            (Some(t), Some(c)) => {
                let _ = writeln!(
                    s,
                    "  the flight first left class safe at {t:.3} s (class {c})"
                );
            }
            (None, Some(c)) => {
                let _ = writeln!(s, "  class of the flight: {c}");
            }
            _ => {}
        }
        match (m.descent, m.throttle_low) {
            (Some(d), Some(g)) => {
                let secs_down = (g - d) as f64 * 1e-6;
                let from = st.descent_from_m.unwrap_or(f64::NAN);
                let _ = writeln!(
                    s,
                    "  the descent (throttle stick {DESCENT_STICK}): began at {:.3} s from {from:.2} m, \
                     on the ground (height at most {GROUND_M} m) {secs_down:.3} s later, mean rate {:.3} m/s, \
                     greatest downward speed {:.3} m/s, downward speed at the first tick on the ground \
                     {:?} m/s (height {:?} m), touchdown speed {:?} m/s",
                    d as f64 * 1e-6,
                    from / secs_down,
                    st.descent_max_down_mps.unwrap_or(f64::NAN),
                    st.ground_down_mps,
                    st.ground_height_m,
                    st.touchdown_speed_mps
                );
            }
            _ => {
                let _ = writeln!(s, "  the descent: not completed");
            }
        }
        match st.pack_before_landing_v {
            Some(v) => {
                let _ = writeln!(
                    s,
                    "  pack voltage under load, mean of the last second before the descent began: {v:.3} V (the low-battery threshold is {:.3} V)",
                    low_battery_volt()
                );
            }
            None => {
                let _ = writeln!(s, "  pack voltage before the landing: not measured");
            }
        }
        let _ = writeln!(
            s,
            "  pack at the marks (flight time; voltage under load, current, state of charge):"
        );
        for (name, p) in &f.pack {
            let _ = writeln!(
                s,
                "    {name:<22} {:9.3} s  {:6.3} V  {:5.2} A  {:5.1} %",
                p.t_s,
                p.volts,
                p.amps,
                100.0 * p.state_of_charge
            );
        }
        let _ = writeln!(
            s,
            "  track every 10 s (flight time, north, east, up in m from where it started):"
        );
        for (t, n, e, u) in &f.track {
            let _ = writeln!(s, "    {t:7.1} s  {n:8.2} {e:8.2} {u:7.2}");
        }
        if let Some((sp, v)) = &f.spectrum {
            let _ = writeln!(
                s,
                "  spectrum: {} gyro samples as sent, {} segments of {SEGMENT}, bin {:.4} Hz; band {}-{} Hz",
                sp.samples, sp.segments, sp.bin_hz, BAND_HZ.0, BAND_HZ.1
            );
            for l in &v.lines {
                let _ = writeln!(
                    s,
                    "    axis {}: largest line {:.2} Hz at {:.2} times the band's median power",
                    ["x", "y", "z"][l.axis],
                    l.freq_hz,
                    l.ratio
                );
            }
            match v.peak {
                Some(p) => {
                    let _ = writeln!(
                        s,
                        "    verdict: PEAK at {:.2} Hz (axis {}), ratio {:.2}",
                        p.freq_hz,
                        ["x", "y", "z"][p.axis],
                        p.ratio
                    );
                }
                None => {
                    let _ = writeln!(
                        s,
                        "    verdict: no peak (the rule: a line of 40 to 400 Hz at 10 times the band's median power or more)"
                    );
                }
            }
        }
        if let Some(t) = m.spectrum {
            let learned = echoes(&f.audit, program_us(t), program_us(t + AFTER_SPECTRUM_US));
            if let Some(v) = learned.get("MOT_THST_HOVER") {
                let _ = writeln!(
                    s,
                    "  learned hover thrust (MOT_THST_HOVER read at the end of the hover): {v}"
                );
            }
        }
        let _ = writeln!(s, "  commands sent by the script:");
        for (t, c) in &f.commands {
            let _ = writeln!(s, "    {:9.3} s  {c}", *t as f64 * 1e-6);
        }
        let _ = writeln!(s, "  status texts of the program:");
        for (t, text) in texts(&f.audit) {
            let _ = writeln!(s, "    {t:9.3} s  {text}");
        }
        let _ = writeln!(s, "  mode changes read from the heartbeat:");
        for (t, mode) in &f.audit.mode_changes {
            let _ = writeln!(s, "    {:9.3} s  mode {mode}", flight_s(*t));
        }
        for (t, a) in &f.audit.armed_changes {
            let _ = writeln!(s, "    {:9.3} s  armed {a}", flight_s(*t));
        }
    }
    let stopped = flights.last().is_some_and(|f| f.marks.aborted.is_some());
    let _ = writeln!(
        s,
        "gains: before the flight, and {}",
        if stopped {
            "as read at the moment the flight was stopped (not saved by the tuner)"
        } else {
            "after the tuner's save"
        }
    );
    let mut names: Vec<&String> = before.keys().chain(after.keys()).collect();
    names.sort();
    names.dedup();
    for n in names {
        let b = before
            .get(n)
            .map_or_else(|| "-".to_string(), ToString::to_string);
        let a = after
            .get(n)
            .map_or_else(|| "-".to_string(), ToString::to_string);
        let mark = if b != a && after.contains_key(n) {
            "  *"
        } else {
            ""
        };
        let _ = writeln!(s, "  {n:<18} {b:>12} {a:>12}{mark}");
    }
    if let Some(n) = notch {
        let _ = writeln!(s, "notch reference (learned hover thrust) {}", n.reference);
        let _ = writeln!(
            s,
            "notch as set: {}",
            n.params
                .iter()
                .map(|(name, v)| format!("{name}={v}"))
                .collect::<Vec<_>>()
                .join(" ")
        );
    }
    if !notch_read.is_empty() {
        let _ = writeln!(
            s,
            "notch as read back after the last flight: {}",
            NOTCH_NAMES
                .iter()
                .filter_map(|n| notch_read.get(*n).map(|v| format!("{n}={v}")))
                .collect::<Vec<_>>()
                .join(" ")
        );
    }
    s
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A deterministic white-ish noise: a linear congruential generator.
    fn noise(n: usize, amp: f32, seed: u64) -> Vec<f32> {
        let mut x = seed;
        (0..n)
            .map(|_| {
                x = x
                    .wrapping_mul(6_364_136_223_846_793_005)
                    .wrapping_add(1_442_695_040_888_963_407);
                let u = ((x >> 33) as f64 / (1u64 << 31) as f64) - 0.5;
                amp * u as f32 * 3.4
            })
            .collect()
    }

    fn samples(n: usize, f: impl Fn(usize) -> [f32; 3]) -> Vec<[f32; 3]> {
        (0..n).map(f).collect()
    }

    fn sine(i: usize, hz: f64, amp: f64) -> f32 {
        (amp * <f64 as Real>::sin(2.0 * core::f64::consts::PI * hz * i as f64 / 1000.0)) as f32
    }

    #[test]
    fn the_names_are_sendable_unique_and_the_setup_names_are_those_of_setup_parm() {
        let all = before_names();
        for n in all.iter().chain(GAIN_NAMES.iter()) {
            assert!(
                !n.is_empty() && n.len() <= 16,
                "{n}: a name is sent in 16 bytes"
            );
        }
        let mut sorted = all.clone();
        sorted.sort_unstable();
        sorted.dedup();
        assert_eq!(sorted, all, "sorted and unique");
        let mut g = GAIN_NAMES.to_vec();
        g.sort_unstable();
        assert_eq!(
            g,
            GAIN_NAMES.to_vec(),
            "the read-back list is in the file's order"
        );
        for n in SETUP_NAMES.iter().chain(GAIN_NAMES.iter()) {
            assert!(all.contains(n), "{n} is read before the flight");
        }
        let setup: Vec<String> = parm::parse(parm::SETUP)
            .expect("setup.parm")
            .into_iter()
            .map(|(n, _)| n)
            .collect();
        let mut want: Vec<&str> = SETUP_NAMES.to_vec();
        want.sort_unstable();
        let mut got: Vec<&str> = setup.iter().map(String::as_str).collect();
        got.sort_unstable();
        assert_eq!(
            got, want,
            "every name of setup.parm is read back, and no other"
        );
        // The tuner may change every rate gain, filter, angle gain and
        // acceleration limit, and the learned hover thrust.
        for axis in ["RLL", "PIT", "YAW"] {
            for g in ["P", "I", "D", "FF", "FLTT", "FLTD", "FLTE"] {
                let n = format!("ATC_RAT_{axis}_{g}");
                assert!(GAIN_NAMES.contains(&n.as_str()), "{n}");
            }
            let n = format!("ATC_ANG_{axis}_P");
            assert!(GAIN_NAMES.contains(&n.as_str()), "{n}");
        }
        for n in [
            "ATC_ACC_R_MAX",
            "ATC_ACC_P_MAX",
            "ATC_ACC_Y_MAX",
            "MOT_THST_HOVER",
        ] {
            assert!(GAIN_NAMES.contains(&n), "{n}");
        }
    }

    #[test]
    fn the_tuning_flights_load_base_setup_and_the_starting_gains_in_this_order() {
        let names: Vec<String> = tuning_files()
            .iter()
            .map(|p| p.file_name().expect("name").to_string_lossy().into_owned())
            .collect();
        assert_eq!(names, ["base.parm", "setup.parm", "starting-gains.parm"]);
        for p in tuning_files() {
            assert!(p.exists(), "{}", p.display());
        }
    }

    #[test]
    fn white_noise_has_no_peak_and_a_line_in_the_band_has() {
        let n = 30_000;
        let [a, b, c] = [noise(n, 0.003, 1), noise(n, 0.003, 2), noise(n, 0.003, 3)];
        let quiet = samples(n, |i| [a[i], b[i], c[i]]);
        let v = judge(&spectrum(&quiet, 1000.0).expect("spectrum"));
        assert!(v.peak.is_none(), "{v:?}");
        for l in &v.lines {
            assert!(
                l.ratio < 3.0,
                "noise: the largest line is within a few times the median: {l:?}"
            );
        }
        // The same noise and a line of 0.002 rad/s at 150 Hz on the y axis.
        let loud = samples(n, |i| [a[i], b[i] + sine(i, 150.0, 0.002), c[i]]);
        let v = judge(&spectrum(&loud, 1000.0).expect("spectrum"));
        let p = v.peak.expect("a line well above the noise is a peak");
        assert_eq!(p.axis, 1);
        assert!((p.freq_hz - 150.0).abs() < 1.5, "{p:?}");
        assert!(p.ratio >= PEAK_RATIO);
    }

    #[test]
    fn the_mean_of_each_segment_is_removed_before_the_transform() {
        // A constant of 100 rad/s and a line of 0.002 rad/s at 150 Hz. With
        // the mean removed the zero bin holds (almost) nothing; without it the
        // zero bin would hold the constant.
        let n = 30_000;
        let s = samples(n, |i| [100.0 + sine(i, 150.0, 0.002), 100.0, -100.0]);
        let sp = spectrum(&s, 1000.0).expect("spectrum");
        for axis in 0..3 {
            assert!(
                sp.amplitude[axis][0] < 1e-3,
                "axis {axis}: the zero bin is {}",
                sp.amplitude[axis][0]
            );
        }
    }

    #[test]
    fn the_amplitude_of_a_line_is_its_amplitude_and_the_band_is_40_to_400() {
        let n = 30_000;
        // A line at a bin's centre: 100 bins of 0.9765625 Hz.
        let hz = 100.0 * 1000.0 / SEGMENT as f64;
        let s = spectrum(&samples(n, |i| [sine(i, hz, 0.5), 0.0, 0.0]), 1000.0).expect("spectrum");
        let k = 100;
        let a = s.amplitude[0][k];
        assert!((a - 0.5).abs() < 0.03, "amplitude {a}");
        // Lines outside 40 to 400 Hz are not judged: 20 Hz, 450 Hz.
        for out in [20.0, 450.0] {
            let floor = noise(n, 0.003, 9);
            let loud = samples(n, |i| [floor[i] + sine(i, out, 0.5), 0.0, 0.0]);
            let v = judge(&spectrum(&loud, 1000.0).expect("spectrum"));
            assert!(v.peak.is_none(), "{out} Hz is outside the band: {v:?}");
        }
        // Just inside either end is.
        for inside in [45.0, 395.0] {
            let floor = noise(n, 0.003, 9);
            let loud = samples(n, |i| [floor[i] + sine(i, inside, 0.5), 0.0, 0.0]);
            let v = judge(&spectrum(&loud, 1000.0).expect("spectrum"));
            assert!(v.peak.is_some(), "{inside} Hz is inside the band");
        }
        assert!(
            spectrum(&[[0.0; 3]; SEGMENT - 1], 1000.0).is_none(),
            "too short"
        );
        // A silent signal has no peak (no division by a zero median).
        let v = judge(&spectrum(&[[0.0; 3]; 4096], 1000.0).expect("spectrum"));
        assert!(v.peak.is_none());
    }

    #[test]
    fn the_notch_is_the_guides_throttle_based_one() {
        let p = notch_params(150.0, 0.18);
        let want = [
            ("INS_HNTCH_ENABLE", 1.0),
            ("INS_HNTCH_MODE", 1.0),
            ("INS_HNTCH_FREQ", 150.0),
            ("INS_HNTCH_BW", 75.0),
            ("INS_HNTCH_REF", 0.18),
            ("INS_HNTCH_ATT", 40.0),
        ];
        let got: Vec<(&str, f64)> = p.iter().map(|(n, v)| (n.as_str(), *v)).collect();
        assert_eq!(got, want);
    }

    /// What the script did, tick by tick.
    #[derive(Default)]
    struct Run {
        modes: Vec<(u64, u32)>,
        reads: Vec<(u64, Vec<&'static str>)>,
        analysed: Vec<u64>,
        throttle_changes: Vec<(u64, f32)>,
        arm: Option<u64>,
        end: Option<u64>,
        error: Option<String>,
    }

    /// Drives a script on a model vehicle: the height rises at 1.4 m/s while
    /// the throttle is above 0.55 and falls at 1 m/s while it is at the
    /// descent stick; the pulses say
    /// armed until `disarm_at`.
    fn drive(mut sc: Script, peak: bool, disarm_at: Option<u64>) -> Run {
        let mut run = Run::default();
        let (mut h, mut thr, mut t) = (0.0f64, -1.0f32, 0u64);
        while t < 3000 * S && !sc.done() {
            let armed = disarm_at.is_none_or(|d| t < d);
            match sc.tick(t, h, armed) {
                Ok(a) => {
                    if let Some(m) = a.set_mode {
                        run.modes.push((t, m));
                    }
                    if !a.reads.is_empty() {
                        run.reads.push((t, a.reads.clone()));
                    }
                    if a.analyse {
                        run.analysed.push(t);
                        sc.set_peak(peak);
                    }
                    if (a.throttle - thr).abs() > 1e-6 {
                        run.throttle_changes.push((t, a.throttle));
                        thr = a.throttle;
                    }
                    if a.arm && run.arm.is_none() {
                        run.arm = Some(t);
                    }
                    if a.throttle > 0.55 {
                        h += 1.4 * 0.0005;
                    }
                    // The descent stick sinks the model at 1 m/s.
                    if a.throttle == DESCENT_STICK {
                        h = (h - 1.0 * 0.0005).max(0.0);
                    }
                }
                Err(e) => {
                    run.error = Some(e);
                    break;
                }
            }
            t += 500;
        }
        run.end = sc.marks().end;
        run
    }

    #[test]
    fn the_script_flies_its_stages_at_the_times_it_states() {
        assert_eq!(AUTOTUNE_S, 480, "ADR-0007 section 2: the wait is 480 s");
        let sc = Script::new(true, false, AUTOTUNE_S);
        let arm = sc.marks().arm;
        let run = drive(sc, false, Some(600 * S));
        assert!(run.error.is_none(), "{:?}", run.error);
        // The mode command: POSHOLD (16) once at 2 s; AUTOTUNE (15) once, 61 s
        // after the hover began.
        assert_eq!((MODE_POSHOLD, MODE_AUTOTUNE), (16, 15));
        let m = &run.modes;
        assert_eq!(m.len(), 2, "{m:?}");
        assert_eq!(m[0], (2 * S, MODE_POSHOLD));
        // Arm switch on at its stated time; throttle up 3 s later; back to
        // centre at 9.5 m (the stick of 0.8 climbs 1.4 m/s: 6.8 s).
        assert_eq!(run.arm, Some(arm));
        let th = &run.throttle_changes;
        assert_eq!(th[0], (0, 0.0));
        assert_eq!(th[1], (arm + 3 * S, CLIMB_STICK));
        let hover_at = th[2].0;
        assert_eq!(th[2].1, 0.5);
        assert!(
            (hover_at - (arm + 3 * S)) as f64 * 1e-6 > 6.5
                && (hover_at - (arm + 3 * S)) as f64 * 1e-6 < 7.1,
            "{hover_at}"
        );
        // The spectrum is taken 60 s into the hover, the tuner started 1 s
        // after, the descent begins 480 s after (the stick at 0.30) and the
        // stick goes fully low when the height is at most 0.05 m: 9.45 to
        // 9.6 s later at the model's 1 m/s from 9.5 m.
        assert_eq!(run.analysed, vec![hover_at + HOVER_S * S]);
        assert_eq!(m[1], (hover_at + (HOVER_S + 1) * S, MODE_AUTOTUNE));
        let low = hover_at + (HOVER_S + 1 + AUTOTUNE_S) * S;
        assert_eq!(DESCENT_STICK, 0.30, "ADR-0007 section 2: the stick is 0.30");
        assert_eq!(th[3], (low, DESCENT_STICK));
        assert_eq!(th[4].1, 0.0);
        let down = (th[4].0 - low) as f64 * 1e-6;
        assert!(
            (9.4..9.7).contains(&down),
            "{down} s from the descent to the ground"
        );
        assert_eq!(th.len(), 5, "{th:?}");
        // Reads: groups of at most 6, a second apart, the first group at 3 s.
        let before: Vec<_> = run.reads.iter().filter(|(t, _)| *t < arm).collect();
        assert_eq!(before[0].0, READ_BEFORE_AT_US);
        for w in before.windows(2) {
            assert_eq!(w[1].0 - w[0].0, S);
        }
        assert!(run.reads.iter().all(|(_, g)| g.len() <= READ_GROUP));
        let asked: Vec<&str> = before.iter().flat_map(|(_, g)| g.iter().copied()).collect();
        assert_eq!(asked, before_names());
        // After the disarm (600 s): nothing read between the throttle going
        // low and 5 s after the pulses said disarmed; then the five groups.
        let after: Vec<_> = run.reads.iter().filter(|(t, _)| *t > low).collect();
        assert!(
            after
                .iter()
                .filter(|(t, _)| *t > hover_at + 100 * S)
                .all(|(t, _)| *t >= 605 * S),
            "{after:?}"
        );
        let names: Vec<&str> = after.iter().flat_map(|(_, g)| g.iter().copied()).collect();
        assert_eq!(names, GAIN_NAMES.to_vec());
        assert_eq!(after[0].0, 605 * S);
        for w in after.windows(2) {
            assert_eq!(w[1].0 - w[0].0, S);
        }
        assert_eq!(run.end, Some(605 * S + 5 * S + READ_END_US));
    }

    #[test]
    fn the_read_back_waits_for_the_disarm_and_a_vehicle_that_stays_armed_is_an_error() {
        // Never disarmed: no read of the gains is ever made, and the script
        // stops with a sentence 180 s after the throttle went low.
        let run = drive(Script::new(true, false, 10), false, None);
        let e = run.error.expect("an error");
        assert!(
            e.contains("not disarmed 180 s after the throttle went low"),
            "{e}"
        );
        let gains_asked = run
            .reads
            .iter()
            .filter(|(_, g)| g.contains(&"ATC_ACC_P_MAX") && g.len() == READ_GROUP)
            .count();
        assert_eq!(
            gains_asked, 1,
            "only the first group of the read before the flight"
        );
    }

    #[test]
    fn a_peak_lands_and_ends_the_first_flight_and_a_hover_check_ends_at_the_spectrum() {
        let run = drive(Script::new(true, false, AUTOTUNE_S), true, Some(200 * S));
        assert!(run.error.is_none(), "{:?}", run.error);
        assert_eq!(
            run.modes.len(),
            1,
            "no AUTOTUNE after a peak: {:?}",
            run.modes
        );
        // It lands (the throttle goes low) and ends 5 s after the disarm,
        // without reading the gains.
        let end = run.end.expect("ends");
        assert_eq!(end, 205 * S);
        let names: Vec<&str> = run
            .reads
            .iter()
            .filter(|(t, _)| *t > 50 * S)
            .flat_map(|(_, g)| g.iter().copied())
            .collect();
        assert_eq!(names, vec!["MOT_THST_HOVER"]);
        // A second flight (the notch given) goes on to AUTOTUNE although the
        // spectrum still shows the peak.
        let run = drive(Script::new(false, false, 5), true, Some(400 * S));
        assert_eq!(run.modes.len(), 2);
        // A hover check ends after the spectrum.
        let run = drive(Script::new(false, true, AUTOTUNE_S), false, None);
        assert_eq!(run.modes.len(), 1);
        assert!(run.end.expect("ends") - run.analysed[0] <= S);
    }

    #[test]
    fn a_climb_that_never_arrives_is_an_error_and_an_abort_reads_the_gains() {
        let mut sc = Script::new(false, false, 900);
        let mut t = 0;
        let err = loop {
            match sc.tick(t, 0.0, true) {
                Ok(_) => t += 500,
                Err(e) => break e,
            }
        };
        assert!(err.contains("did not reach 9.5 m in 45 s"), "{err}");
        // Aborted in the air: the sticks stay centred, the 29 names are read
        // in 5 groups from the next whole second.
        let mut sc = Script::new(false, false, 900);
        let mut t = 0;
        while sc.marks().autotune.is_none() {
            let h = if t > 20 * S { 9.6 } else { 0.0 };
            sc.tick(t, h, true).expect("tick");
            t += 500;
        }
        sc.abort(t);
        let mut groups_read = Vec::new();
        while !sc.done() {
            let a = sc.tick(t, 9.6, true).expect("tick");
            assert_eq!(a.throttle, 0.5);
            if !a.reads.is_empty() {
                groups_read.push((t, a.reads.clone()));
            }
            t += 500;
        }
        assert_eq!(groups_read.len(), 5);
        assert_eq!(groups_read[0].0, sc.marks().aborted.expect("aborted"));
        let names: Vec<&str> = groups_read
            .iter()
            .flat_map(|(_, g)| g.iter().copied())
            .collect();
        assert_eq!(names, GAIN_NAMES.to_vec());
    }

    /// The check of a flight that tunes all three axes (the old procedure's
    /// one flight; the axes of each of the two flights are tested below).
    fn check(
        texts: &[(f64, String)],
        after: &BTreeMap<String, f32>,
        pack: Option<f64>,
        low: f64,
    ) -> Result<(), String> {
        check_axes(texts, after, pack, low, &["Roll", "Pitch", "Yaw"])
    }

    fn check_run(
        texts: &[(f64, String)],
        before: &BTreeMap<String, f32>,
        after: &BTreeMap<String, f32>,
        pack: Option<f64>,
        low: f64,
    ) -> Result<(), String> {
        check_flight(texts, before, after, pack, low, &["Roll", "Pitch", "Yaw"])
    }

    fn texts_ok() -> Vec<(f64, String)> {
        [
            "AutoTune: Started",
            "AutoTune: Roll complete",
            "AutoTune: Pitch complete",
            "AutoTune: Yaw(E) complete",
            "AutoTune: Success",
            "AutoTune: Saved gains for Roll Pitch Yaw(E)",
        ]
        .iter()
        .enumerate()
        .map(|(i, t)| (i as f64, (*t).to_string()))
        .collect()
    }

    fn all_gains() -> BTreeMap<String, f32> {
        GAIN_NAMES.iter().map(|n| ((*n).to_string(), 1.5)).collect()
    }

    #[test]
    fn the_check_wants_every_axis_the_success_the_save_and_every_value() {
        let low = 20.767;
        assert_eq!(check(&texts_ok(), &all_gains(), Some(21.5), low), Ok(()));
        for missing in [
            "AutoTune: Roll complete",
            "AutoTune: Pitch complete",
            "AutoTune: Yaw(E) complete",
            "AutoTune: Success",
            "AutoTune: Saved gains for Roll Pitch Yaw(E)",
        ] {
            let t: Vec<_> = texts_ok()
                .into_iter()
                .filter(|(_, x)| x != missing)
                .collect();
            let e = check(&t, &all_gains(), Some(21.5), low).expect_err(missing);
            assert!(e.contains("did not print"), "{e}");
        }
        for name in GAIN_NAMES {
            let mut g = all_gains();
            g.remove(name);
            let e = check(&texts_ok(), &g, Some(21.5), low).expect_err(name);
            assert!(e.contains(name), "{e}");
        }
        // The tuner started and stopped is not a success.
        let t = vec![
            (0.0, "AutoTune: Started".to_string()),
            (1.0, "AutoTune: Stopped".to_string()),
        ];
        assert!(check(&t, &all_gains(), Some(21.5), low).is_err());
    }

    #[test]
    fn the_tuned_file_is_sorted_and_keeps_every_digit_of_every_value() {
        let mut g = BTreeMap::new();
        for (n, v) in [
            ("ATC_RAT_RLL_P", 0.054_990_08_f32),
            ("ATC_ACC_P_MAX", 4839.1997),
            ("MOT_THST_HOVER", 0.182_364_7),
            ("ATC_ANG_YAW_P", 2.060_773),
            ("ATC_RAT_YAW_FLTE", 5.0),
            ("ATC_RAT_RLL_D", 0.001_233_720_1),
        ] {
            g.insert(n.to_string(), v);
        }
        let text = tuned_file(&g, &BTreeMap::new(), "ff00", 7, 1022.5);
        let lines: Vec<&str> = text.lines().filter(|l| !l.starts_with('#')).collect();
        let mut sorted = lines.clone();
        sorted.sort_unstable();
        assert_eq!(lines, sorted, "sorted by name");
        assert!(
            text.contains("# binary sha256 ff00")
                && text.contains("# seed 7")
                && text.contains("# ArduCopter parameters")
                && text.contains("simulated flight time of the procedure 1022.500 s")
                && text.contains("Never edited by hand")
        );
        assert!(text.contains("`oxide-bridge tune --seed 7`"));
        // The version is not written: nothing reads it from the program, and
        // the checksum names the binary.
        assert!(
            !text.contains("V4."),
            "no version a constant could mislabel"
        );
        assert!(
            !text.contains("commit"),
            "nothing that differs between runs"
        );
        let parsed = parm::parse(&text).expect("it is a parameter file");
        assert_eq!(parsed.len(), g.len());
        for (n, v) in parsed {
            assert_eq!(
                v as f32, g[&n],
                "{n}: the value comes back to the same 32-bit float"
            );
        }
        assert!(text.contains("ATC_ACC_P_MAX 4839.1997\n"), "{text}");
        assert!(text.contains("ATC_RAT_RLL_P 0.05499008\n"), "{text}");
        assert!(text.contains("ATC_RAT_YAW_FLTE 5\n"), "{text}");
    }

    fn rec(t_s: f64, north: f64, up: f64, volts: f64, armed: bool) -> StepRecord<4> {
        StepRecord {
            t: Micros((t_s * 1e6).round() as u64),
            position: Vec3::new(north, 0.0, -up),
            velocity: Vec3::zero(),
            attitude: oxide_core::math::Quat::identity(),
            rate: Vec3::zero(),
            motors: [0.2; 4],
            rotor_speed: [0.0; 4],
            battery_v: volts,
            battery_a: 8.0,
            armed,
            setpoint: None,
            wind: Vec3::zero(),
            physics_faults: 0,
            core: None,
        }
    }

    #[test]
    fn the_flight_is_stopped_by_the_gates_class_limits_and_not_before() {
        let marks = Marks::default();
        // Just inside the fly-away limits: still safe, no violation.
        let mut t = Tracker::default();
        t.add(&rec(0.0, 0.0, 0.0, 25.0, false), &marks, 1.0)
            .expect("ground");
        t.add(&rec(1.0, 0.0, 10.0, 25.0, true), &marks, 1.0)
            .expect("armed");
        t.add(
            &rec(
                2.0,
                FLY_AWAY_DISTANCE_M - 0.5,
                FLY_AWAY_HEIGHT_M - 0.5,
                25.0,
                true,
            ),
            &marks,
            1.0,
        )
        .expect("inside the limits of the gate's class");
        assert_eq!(t.stats.class, Some(FlightClass::Safe));
        assert_eq!(t.stats.class_exit_s, None);
        // 150 m out: not safe, a sentence that names the class and the time.
        let e = t
            .add(&rec(3.0, 150.0, 10.0, 25.0, true), &marks, 1.0)
            .expect_err("150 m out is a fly-away");
        assert!(e.contains("class fly-away at flight time 3.000 s"), "{e}");
        assert!(e.contains("150.00 m from where it armed"), "{e}");
        assert_eq!(t.stats.class_exit_s, Some(3.0));
        // The limit applies to the height, too.
        let mut t = Tracker::default();
        t.add(&rec(0.0, 0.0, 0.0, 25.0, false), &marks, 1.0)
            .expect("ground");
        t.add(&rec(1.0, 0.0, 0.0, 25.0, true), &marks, 1.0)
            .expect("armed");
        let e = t
            .add(
                &rec(2.0, 0.0, FLY_AWAY_HEIGHT_M + 1.0, 25.0, true),
                &marks,
                1.0,
            )
            .expect_err("61 m up");
        assert!(e.contains("class fly-away"), "{e}");
        // Only the first crossing is reported.
        assert!(t.add(&rec(3.0, 0.0, 70.0, 25.0, true), &marks, 1.0).is_ok());
    }

    #[test]
    fn the_pack_is_the_mean_voltage_of_the_second_before_the_descent_began() {
        let mut t = Tracker::default();
        let mut m = Marks::default();
        // 2 s at 22.0 V, then 1 s (until the descent begins) at 21.0 V
        // with a dip in the first half of that second, then 19 V (after the
        // landing began, and at the disarm).
        for i in 0..4000u64 {
            let ts = i as f64 * 0.001;
            let v = if ts < 2.0 {
                22.0
            } else if ts < 3.0 {
                21.0
            } else {
                19.0
            };
            if i == 3000 {
                m.descent = Some(3_000_000);
            }
            if i == 3900 {
                m.disarmed = Some(3_900_000);
            }
            t.add(&rec(ts, 0.0, 5.0, v, true), &m, 0.5).expect("safe");
        }
        let v = t.stats.pack_before_landing_v.expect("measured");
        assert!(
            (v - 21.0).abs() < 1e-9,
            "{v}: the second before the descent began"
        );
        // The pack at the marks: the voltage at the disarm is another one.
        let disarmed = t
            .pack
            .iter()
            .find(|(n, _)| *n == "disarmed")
            .expect("mark")
            .1;
        assert_eq!(disarmed.volts, 19.0);
        assert!((disarmed.t_s - 3.9).abs() < 1e-9);
        let low = t
            .pack
            .iter()
            .find(|(n, _)| *n == "descent start")
            .expect("mark")
            .1;
        assert_eq!(low.state_of_charge, 0.5);
        assert_eq!(low.amps, 8.0);
        // Not measured while the descent has not begun.
        let mut t = Tracker::default();
        t.add(&rec(0.0, 0.0, 5.0, 22.0, true), &Marks::default(), 1.0)
            .expect("safe");
        assert_eq!(t.stats.pack_before_landing_v, None);
    }

    #[test]
    fn the_sample_of_the_tick_the_descent_began_at_is_not_in_the_pack_mean() {
        // The mark is read one tick after the tick it names, so the sample of
        // that tick (19 V, the descent) is already in the window's reach: it
        // is left out ("the last second before the descent begins").
        let mut t = Tracker::default();
        let mut m = Marks::default();
        for i in 0..3100u64 {
            let ts = i as f64 * 0.001;
            let v = if ts < 3.0 { 21.0 } else { 19.0 };
            if i == 3001 {
                m.descent = Some(3_000_000);
            }
            t.add(&rec(ts, 0.0, 5.0, v, true), &m, 0.5).expect("safe");
        }
        let v = t.stats.pack_before_landing_v.expect("measured");
        assert!((v - 21.0).abs() < 1e-9, "{v}");
    }

    #[test]
    fn the_pack_is_reported_at_every_mark_at_the_time_the_mark_names() {
        let mut t = Tracker::default();
        let m = Marks {
            hover: Some(10 * S),
            spectrum: Some(70 * S),
            autotune: Some(71 * S),
            descent: Some(90 * S),
            throttle_low: Some(92 * S),
            disarmed: Some(95 * S),
            ..Marks::default()
        };
        for i in 0..=100u64 {
            let ts = i as f64;
            t.add(
                &rec(ts, 0.0, 5.0, 25.0 - ts * 0.01, true),
                &m,
                1.0 - ts / 200.0,
            )
            .expect("safe");
        }
        let at = |name: &str| {
            t.pack
                .iter()
                .find(|(n, _)| *n == name)
                .unwrap_or_else(|| panic!("no mark {name}"))
                .1
        };
        // The spectrum window starts SPECTRUM_S before the end of the hover.
        let start = at("spectrum window start");
        assert_eq!(start.t_s, 10.0 + (HOVER_S - SPECTRUM_S) as f64);
        assert_eq!(start.t_s, 40.0);
        assert!((start.volts - 24.6).abs() < 1e-9, "{}", start.volts);
        assert!((start.state_of_charge - 0.8).abs() < 1e-9);
        let taken = at("spectrum taken");
        assert_eq!(taken.t_s, 70.0);
        assert!((taken.state_of_charge - 0.65).abs() < 1e-9);
        assert_eq!(at("hover reached").t_s, 10.0);
        assert_eq!(at("AUTOTUNE start").t_s, 71.0);
        assert_eq!(at("descent start").t_s, 90.0);
        assert_eq!(at("on the ground").t_s, 92.0);
        assert_eq!(at("disarmed").t_s, 95.0);
        assert_eq!(t.pack.len(), 7);
    }

    #[test]
    fn the_descent_is_measured_from_its_start_to_the_first_tick_on_the_ground() {
        let mut t = Tracker::default();
        let m = Marks {
            descent: Some(10 * S),
            ..Marks::default()
        };
        // 10 m at the start of the descent, down at 1.25 m/s, on the ground
        // (0.04 m) at 17.97 s, a spike of 1.5 m/s at 17.9 s.
        for i in 0..=2000u64 {
            let ts = i as f64 * 0.01;
            let up = (10.0 - 1.25 * (ts - 10.0)).max(0.04);
            let mut r = rec(ts, 0.0, if ts < 10.0 { 10.0 } else { up }, 25.0, true);
            r.velocity = Vec3::new(0.0, 0.0, if ts < 10.0 || up <= 0.04 { 0.0 } else { 1.25 });
            if i == 1790 {
                r.velocity = Vec3::new(0.0, 0.0, 1.5);
            }
            t.add(&r, &m, 0.5).expect("safe");
        }
        let st = t.stats;
        assert_eq!(st.descent_from_m, Some(10.0));
        assert_eq!(st.descent_max_down_mps, Some(1.5));
        assert!(st.ground_height_m.expect("on the ground") <= GROUND_M);
        assert_eq!(st.ground_down_mps, Some(1.25));
        // Nothing before the descent began.
        let mut t = Tracker::default();
        t.add(&rec(0.0, 0.0, 5.0, 22.0, true), &Marks::default(), 1.0)
            .expect("safe");
        assert_eq!(t.stats.descent_from_m, None);
    }

    #[test]
    fn a_descent_that_does_not_reach_the_ground_is_an_error_and_a_disarm_in_the_air_ends_it() {
        // The model never sinks (no drive of the height): 180 s after the
        // descent began the script gives up with a sentence.
        let mut sc = Script::new(false, false, 5);
        let mut t = 0u64;
        let err = loop {
            let h = if t > 20 * S { 9.6 } else { 0.0 };
            match sc.tick(t, h, true) {
                Ok(_) => t += 500,
                Err(e) => break e,
            }
        };
        assert!(
            err.contains("not on the ground 180 s after the descent began"),
            "{err}"
        );
        assert!(sc.marks().descent.is_some() && sc.marks().throttle_low.is_none());
    }

    #[test]
    fn a_disarm_in_the_air_during_the_descent_ends_it_with_the_stick_low() {
        // The vehicle disarms itself high up (a failsafe, say): the descent
        // does not go on to wait for a ground it will fall to.
        let mut sc = Script::new(false, false, 5);
        let mut t = 0u64;
        while sc.marks().descent.is_none() {
            sc.tick(t, 9.6, true).expect("flying");
            t += 500;
        }
        sc.tick(t, 9.6, true).expect("descending");
        assert!(
            sc.marks().throttle_low.is_none(),
            "armed, high: still descending"
        );
        t += 500;
        let a = sc.tick(t, 9.6, false).expect("disarmed in the air");
        assert_eq!(a.throttle, 0.0);
        assert_eq!(sc.marks().throttle_low, Some(t));
    }

    #[test]
    fn a_pack_that_is_not_above_the_low_battery_threshold_before_the_landing_fails_the_check() {
        let low = low_battery_volt();
        assert!(
            (low - 20.767).abs() < 0.001,
            "the runner's BATT_LOW_VOLT: {low}"
        );
        assert_eq!(
            check(&texts_ok(), &all_gains(), Some(low + 0.01), low),
            Ok(())
        );
        for v in [low, low - 0.5, 17.7, f64::NAN] {
            let e = check(&texts_ok(), &all_gains(), Some(v), low).expect_err("too empty");
            assert!(
                e.contains("pack voltage under load") && e.contains("low-battery threshold"),
                "{e}"
            );
        }
        // A threshold that is not a number (BATT_LOW_VOLT missing from the
        // runner's list) is no threshold: no pack is above it.
        let e = check(&texts_ok(), &all_gains(), Some(24.0), f64::NAN).expect_err("no threshold");
        assert!(e.contains("low-battery threshold"), "{e}");
        let e = check(&texts_ok(), &all_gains(), None, low).expect_err("not measured");
        assert!(e.contains("not measured"), "{e}");
    }

    #[test]
    fn the_program_must_have_taken_every_line_of_setup_parm() {
        let setup = parm::parse(parm::SETUP).expect("setup.parm");
        let all: BTreeMap<String, f32> =
            setup.iter().map(|(n, v)| (n.clone(), *v as f32)).collect();
        assert_eq!(all.len(), SETUP_NAMES.len());
        assert_eq!(check_setup(&all), Ok(()));
        for (name, _) in &setup {
            let mut missing = all.clone();
            missing.remove(name);
            let e = check_setup(&missing).expect_err(name);
            assert!(
                e.contains(name.as_str()) && e.contains("not read back"),
                "{e}"
            );
            let mut other = all.clone();
            other.insert(name.clone(), all[name] + 1.0);
            let e = check_setup(&other).expect_err(name);
            assert!(
                e.contains(name.as_str()) && e.contains("setup.parm says"),
                "{e}"
            );
        }
        // The guide's 80 Hz gyro filter, by name.
        assert_eq!(all.get("INS_GYRO_FILTER"), Some(&80.0));
        // The procedure's check holds the program to it.
        let low = low_battery_volt();
        assert_eq!(
            check_run(&texts_ok(), &all, &all_gains(), Some(low + 1.0), low),
            Ok(())
        );
        let mut ignored = all.clone();
        ignored.insert("INS_GYRO_FILTER".into(), 20.0);
        let e = check_run(&texts_ok(), &ignored, &all_gains(), Some(low + 1.0), low)
            .expect_err("a program that kept its 20 Hz filter");
        assert!(
            e.contains("INS_GYRO_FILTER") && e.contains("setup.parm"),
            "{e}"
        );
    }

    /// The decision path never reads what came from MAVLink, and takes the
    /// spectrum from the samples it sent, not from the truth.
    #[test]
    fn no_decision_path_reads_what_was_read_from_mavlink_and_the_spectrum_is_of_the_sent_samples() {
        let src = include_str!("tune.rs");
        let marker = "// ---- Everything below reads what the program said";
        let decide = src.split(marker).next().expect("the marker is in the file");
        assert!(decide.len() < src.len(), "the marker splits the file");
        // The test's own text is after the marker.
        for token in [
            "status_texts",
            "param_echoes",
            "MavEvent",
            "audit.heard",
            "audit.mode_changes",
            "audit.armed_changes",
            "vehicle_armed",
            "vehicle_mode",
            "echoes(",
            "texts(",
            "read_back(",
        ] {
            let at = decide
                .lines()
                .filter(|l| !l.trim_start().starts_with("//"))
                .any(|l| l.contains(token));
            // `echoes(` and `read_back(` are used by the procedure driver,
            // which is after the marker; the script and the tuner are before.
            assert!(!at, "the decision path reads `{token}`");
        }
        // The spectrum's samples are the IMU model's, as sent.
        let tick = decide
            .split("impl Autopilot<4> for Tuner")
            .nth(1)
            .and_then(|t| t.split("struct NoPilot").next())
            .expect("the Tuner's tick");
        assert!(
            tick.contains("input.sensors.imu"),
            "the samples are the IMU model's"
        );
        assert!(
            !tick.contains("body.rate") && !tick.contains(".rate"),
            "the spectrum is not taken from the truth"
        );
    }
}
