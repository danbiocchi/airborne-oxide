// SPDX-License-Identifier: AGPL-3.0-or-later
//! ArduCopter's tuning procedure as a script (ADR-0007 section 2): `oxide-bridge
//! tune`.
//!
//! The script flies ArduCopter on the reference quad in calm air on a full
//! pack, with the gate's sensing (our IMU model on the 1000 us grid, the white
//! GPS error, the 45 s pre-roll) and with `base.parm`, `setup.parm` and
//! `starting-gains.parm`, in these stages:
//!
//! 1. *Hover and learn.* Mode ALT_HOLD by a MAVLink mode command at a fixed
//!    time (the radio's mode switch has no ALT_HOLD position), the arm switch,
//!    a climb on the throttle stick, and a hover of [`HOVER_S`] seconds in
//!    which the program learns its hover thrust.
//! 2. *Spectrum.* An amplitude spectrum per axis of the gyro samples this
//!    script saw our IMU model send during the last [`SPECTRUM_S`] seconds of
//!    the hover ([`spectrum`]), and the verdict of the rule of ADR-0007 step 4
//!    ([`judge`]). With a peak the guide's throttle-based harmonic notch is set
//!    ([`notch_params`]), which needs a restart of the program: the flight
//!    lands, ends, and the hover and the next stage are flown again as a new
//!    flight with the notch parameters and the learned hover thrust added.
//! 3. *AutoTune.* Mode AUTOTUNE, the sticks centred, for [`AUTOTUNE_S`]
//!    seconds; then the throttle stick low: the vehicle descends inside
//!    AUTOTUNE, lands, disarms itself, and the tuner saves the gains.
//! 4. *Read back.* After the disarm every parameter the tuner may have
//!    changed ([`GAIN_NAMES`]) is asked for in groups of 6 a second apart, and
//!    `tuned-gains.parm` is written.
//! 5. *Check.* The tuner's messages, the read-back, the box.
//!
//! # Where the rule of the bridge holds
//!
//! Every stick, every command and every parameter request is decided by the
//! simulated time, the physics truth (the height), the motor pulses of the
//! previous exchange (whether the vehicle is armed) and the gyro samples this
//! script itself gave the program: [`Script`] and [`Tuner`]. What the program
//! says (status texts, parameter values) is read only after the flight, by the
//! functions after the marker line in this file, into the report and the
//! parameter file: the one place where values read from MAVLink are written
//! out. The one exception is the notch's reference, the learned hover thrust,
//! which the second flight of a peak's path is given as a parameter: a value
//! read from the program decides a parameter of the next flight there (see
//! [`Outcome::notch_reference`]).

use core::fmt;
use std::collections::BTreeMap;
use std::fmt::Write as _;
use std::path::{Path, PathBuf};
use std::time::Instant;

use oxide_core::io::{RcSample, Stamped};
use oxide_core::math::{Real, Vec3};
use oxide_core::time::Micros;
use oxide_sim::autopilot::{Autopilot, AutopilotInput, AutopilotOutput, PrerollInput};
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
/// ArduCopter's mode numbers.
const MODE_ALT_HOLD: u32 = 2;
const MODE_AUTOTUNE: u32 = 15;
/// The mode command goes out at this time.
const ALT_HOLD_AT_US: u64 = 2 * S;
/// The first group of parameter reads before the flight, and how many names
/// are asked for at once (more than about 20 at once are partly not answered).
const READ_BEFORE_AT_US: u64 = 3 * S;
const READ_GROUP: usize = 6;
/// Time from the last group to the arm switch.
const ARM_GAP_US: u64 = 3 * S;
/// Time from the arm switch to the throttle going up: the program keeps its
/// motors still for 2.0 s after it arms (live 2.233 s after the switch).
const CLIMB_DELAY_US: u64 = 3 * S;
/// The throttle stick of the climb (0 to 1; above 0.55 is up in ALT_HOLD).
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
/// How long AUTOTUNE is given before the throttle goes low, s.
pub const AUTOTUNE_S: u64 = 900;
/// The vehicle must be disarmed (by its pulses) within this time of the
/// throttle going low.
const LAND_LIMIT_US: u64 = 180 * S;
/// From the pulses saying disarmed to the read-back.
const DISARM_SETTLE_US: u64 = 5 * S;
/// After the last read-back group, before the flight ends.
const READ_END_US: u64 = 4 * S;
/// The box: the vehicle may not go further than this from where it started,
/// m, nor higher.
pub const BOX_HORIZONTAL_M: f64 = 20.0;
pub const BOX_UP_M: f64 = 30.0;

// ---- The names -------------------------------------------------------------

/// What the tuner may change, read back after the disarm and written to
/// `tuned-gains.parm`: the rate controllers, the angle controllers, the
/// acceleration limits, the feed-forward switch and the learned hover thrust.
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
    vec![
        ("INS_HNTCH_ENABLE".into(), 1.0),
        ("INS_HNTCH_MODE".into(), 1.0),
        ("INS_HNTCH_FREQ".into(), freq_hz),
        ("INS_HNTCH_BW".into(), freq_hz / 2.0),
        ("INS_HNTCH_REF".into(), hover_thrust),
        ("INS_HNTCH_ATT".into(), 40.0),
    ]
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
    /// The throttle low since this time; the pulses say disarmed since the
    /// other time.
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
    /// `stop_on_peak`: a peak ends the flight by landing (the first flight of
    /// a procedure whose second flight has the notch); `stop_after_hover`: the
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
        if t_us == ALT_HOLD_AT_US {
            a.set_mode = Some(MODE_ALT_HOLD);
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
                        // The learned thrust, for a second flight's notch.
                        a.reads.push("MOT_THST_HOVER");
                    }
                    Some(at) if t_us >= at + AFTER_SPECTRUM_US => {
                        let peak = self.peak.unwrap_or(false);
                        if self.stop_after_hover {
                            self.finish(t_us);
                        } else if peak && self.stop_on_peak {
                            self.begin_land(t_us, false);
                            a.throttle = 0.0;
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
                    self.begin_land(t_us, true);
                    a.throttle = 0.0;
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
        let in_air = !matches!(self.stage, Stage::Ground | Stage::Land { .. });
        self.marks.aborted = Some(t_us);
        self.marks.read_back = Some(t_us);
        self.stage = Stage::ReadBack {
            since: t_us,
            centre: in_air,
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
    /// The first time the vehicle was outside the box, s, if it was.
    pub box_exit_s: Option<f64>,
}

/// Accumulates [`Stats`] and watches the box.
#[derive(Debug, Default)]
struct Tracker {
    start: Option<Vec3<f64>>,
    stats: Stats,
    hover_samples: Vec<(f64, f64, f64, f64)>,
    tune_origin: Option<Vec3<f64>>,
    /// Flight time s, north, east, up in m from the start, every 10 s.
    track: Vec<(f64, f64, f64, f64)>,
}

impl Tracker {
    fn tilt_deg(r: &StepRecord<4>) -> f64 {
        let z = r.attitude.rotate(Vec3::new(0.0, 0.0, 1.0));
        let h = Real::sqrt(z.x * z.x + z.y * z.y);
        Real::atan2(h, z.z).to_degrees()
    }

    /// Adds a record; an error is the sentence of a box violation.
    fn add(&mut self, r: &StepRecord<4>, m: &Marks) -> Result<(), String> {
        let start = *self.start.get_or_insert(r.position);
        let d = r.position - start;
        let horizontal = Real::sqrt(d.x * d.x + d.y * d.y);
        let up = -d.z;
        let t_s = r.t.as_secs_f64();
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
        if (horizontal > BOX_HORIZONTAL_M || up > BOX_UP_M) && self.stats.box_exit_s.is_none() {
            self.stats.box_exit_s = Some(t_s);
            violation = Some(format!(
                "the vehicle left the box ({BOX_HORIZONTAL_M} m out, {BOX_UP_M} m up) at flight \
                 time {t_s:.3} s: {horizontal:.2} m from where it started, {up:.2} m up"
            ));
        }
        if let Some(h0) = m.hover
            && us >= h0 + (HOVER_S - QUALITY_S) * S
            && us < h0 + HOVER_S * S
        {
            self.hover_samples.push((d.x, d.y, -d.z, Self::tilt_deg(r)));
        }
        if m.autotune.is_some() && m.throttle_low.is_none() {
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

    fn finish(mut self) -> (Stats, Vec<(f64, f64, f64, f64)>) {
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
        (self.stats, self.track)
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
    /// How long AUTOTUNE is given, s ([`AUTOTUNE_S`] in the procedure).
    pub autotune_s: u64,
    /// Whether leaving the box ends the flight (it does in the procedure).
    pub enforce_box: bool,
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
        script: Script::new(plan.stop_on_peak, plan.stop_after_hover, plan.autotune_s),
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
                if let Err(e) = tracker.add(&r, &marks)
                    && plan.enforce_box
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
    let (stats, track) = tracker.finish();
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

/// The tuner's messages the check wants, as the program printed them.
pub const AXES: [&str; 3] = ["Roll", "Pitch", "Yaw"];

/// The check on a finished procedure's texts and read-back: the tuner's
/// success message for each axis and its saved-gains message, and a value for
/// every name of [`GAIN_NAMES`]. An error is a sentence.
pub fn check(texts: &[(f64, String)], after: &BTreeMap<String, f32>) -> Result<(), String> {
    for axis in AXES {
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
    Ok(())
}

/// The text of `tuned-gains.parm`: a header that says where it came from, and
/// every name of the read-back with its value as the program reported it (the
/// shortest decimal that gives the same 32-bit float), sorted by name, one
/// per line.
pub fn tuned_file(
    after: &BTreeMap<String, f32>,
    commit: &str,
    binary_sha256: &str,
    seed: u64,
) -> String {
    let mut s = String::new();
    let _ = writeln!(
        s,
        "# ArduCopter V4.7.2 parameters after the tuning of ADR-0007 section 2."
    );
    let _ = writeln!(
        s,
        "# The output of `oxide-bridge tune` (crates/oxide-bridge/src/tune.rs): the"
    );
    let _ = writeln!(
        s,
        "# values read back from the program after the automatic tuner saved its gains."
    );
    let _ = writeln!(s, "# commit {commit}");
    let _ = writeln!(s, "# binary sha256 {binary_sha256}");
    let _ = writeln!(s, "# seed {seed}");
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

/// What the procedure produced.
#[derive(Clone, Debug)]
pub struct Outcome {
    /// The flights, in order.
    pub flights: Vec<FlightResult>,
    /// The values read before the first flight and after the last disarm.
    pub before: BTreeMap<String, f32>,
    pub after: BTreeMap<String, f32>,
    /// The text of `tuned-gains.parm`.
    pub tuned: String,
    /// The learned hover thrust read at the end of the first hover, for the
    /// notch's reference (a value read from the program that decides a
    /// parameter of the second flight).
    pub notch_reference: Option<f64>,
    pub report: String,
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
        autotune_s: AUTOTUNE_S,
        enforce_box: true,
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
        core::slice::from_ref(&flight),
        &before,
        &BTreeMap::new(),
        None,
        flight.failure.as_deref(),
        flight.wall_s,
    );
    let path = out_path(root, "report.txt");
    std::fs::write(&path, text).map_err(|source| TuneError::Write { path, source })?;
    Ok(flight)
}

/// The whole procedure. Files are written into `cfg.out`: `report.txt`,
/// `spectrum.csv` and, when it succeeded, `tuned-gains.parm`; the flights'
/// own directories are made there too.
pub fn tune(cfg: &TuneConfig) -> Result<Outcome, TuneError> {
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
    let mut plan = FlightPlan {
        binary: cfg.binary.clone(),
        root: cfg.out.clone(),
        flight_name: "flight-1".into(),
        instance: cfg.instance,
        seed: cfg.seed,
        files: tuning_files(),
        extra: Vec::new(),
        stop_on_peak: true,
        stop_after_hover: false,
        autotune_s: AUTOTUNE_S,
        enforce_box: true,
    };
    let mut notch_reference = None;
    for n in 1..=2 {
        plan.flight_name = format!("flight-{n}");
        let flight = fly_flight(&plan)?;
        let failed = flight.failure.is_some();
        let peak = flight.spectrum.as_ref().and_then(|(_, v)| v.peak);
        let hover_thrust = flight.marks.spectrum.and_then(|t| {
            echoes(
                &flight.audit,
                program_us(t),
                program_us(t + AFTER_SPECTRUM_US),
            )
            .get("MOT_THST_HOVER")
            .map(|v| f64::from(*v))
        });
        flights.push(flight);
        if failed {
            break;
        }
        match (n, peak, hover_thrust) {
            (1, Some(line), Some(thrust)) => {
                notch_reference = Some(thrust);
                plan.extra = notch_params(line.freq_hz, thrust);
                plan.extra.push(("MOT_THST_HOVER".into(), thrust));
                plan.stop_on_peak = false;
            }
            (1, Some(_), None) => {
                return finish_failed(
                    cfg,
                    flights,
                    "a peak was found but the learned hover thrust was not read".into(),
                );
            }
            _ => break,
        }
    }
    let spectrum_flight = flights.last().and_then(|f| f.spectrum.clone());
    if let Some((s, _)) = &spectrum_flight {
        write("spectrum.csv", &spectrum_csv(s))?;
    }
    let last = flights
        .last()
        .ok_or_else(|| TuneError::Failed("no flight".into()))?;
    let before = flights.first().map(|f| read_back(f).0).unwrap_or_default();
    let after = read_back(last).1;
    let texts_all = texts(&last.audit);
    let mut problem = last.failure.clone();
    if problem.is_none() {
        problem = check(&texts_all, &after).err();
    }
    let commit = env!("OXIDE_GIT_HASH");
    let tuned = tuned_file(&after, commit, &last.audit.binary_sha256, cfg.seed);
    let report = report_text(
        cfg,
        &flights,
        &before,
        &after,
        notch_reference,
        problem.as_deref(),
        wall.elapsed().as_secs_f64(),
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
        tuned,
        notch_reference,
        report,
    })
}

fn finish_failed(
    cfg: &TuneConfig,
    flights: Vec<FlightResult>,
    problem: String,
) -> Result<Outcome, TuneError> {
    let report = report_text(
        cfg,
        &flights,
        &BTreeMap::new(),
        &BTreeMap::new(),
        None,
        Some(&problem),
        0.0,
    );
    let path = out_path(&cfg.out, "report.txt");
    std::fs::write(&path, report).map_err(|source| TuneError::Write { path, source })?;
    Err(TuneError::Failed(problem))
}

fn secs(us: Option<u64>) -> String {
    us.map_or_else(|| "-".to_string(), |u| format!("{:.3} s", u as f64 * 1e-6))
}

fn report_text(
    cfg: &TuneConfig,
    flights: &[FlightResult],
    before: &BTreeMap<String, f32>,
    after: &BTreeMap<String, f32>,
    notch_reference: Option<f64>,
    problem: Option<&str>,
    wall_s: f64,
) -> String {
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
            "  stages (flight time): ALT_HOLD command {}, arm switch {}, throttle up {}, hover {}, spectrum {}, AUTOTUNE {}, throttle low {}, disarmed (pulses) {}, read-back {}, end {}",
            secs(Some(ALT_HOLD_AT_US)),
            secs(Some(m.arm)),
            secs(m.climb),
            secs(m.hover),
            secs(m.spectrum),
            secs(m.autotune),
            secs(m.throttle_low),
            secs(m.disarmed),
            secs(m.read_back),
            secs(m.end)
        );
        let st = &f.stats;
        let _ = writeln!(
            s,
            "  box: furthest {:.2} m out (at {:.1} s), highest {:.2} m; hover (last {QUALITY_S} s): tilt rms {:.3} deg, \
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
        match st.box_exit_s {
            Some(t) => {
                let _ = writeln!(s, "  the vehicle first left the box at {t:.3} s");
            }
            None => {
                let _ = writeln!(s, "  the vehicle stayed inside the box");
            }
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
    if let Some(t) = notch_reference {
        let _ = writeln!(s, "notch reference (learned hover thrust) {t}");
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
    /// the throttle is above 0.55 and the vehicle is armed; the pulses say
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
                    if a.throttle == 0.0 && h > 0.0 && sc.marks().throttle_low.is_some() {
                        h = (h - 2.0 * 0.0005).max(0.0);
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
        let sc = Script::new(true, false, 900);
        let arm = sc.marks().arm;
        let run = drive(sc, false, Some(1_000 * S));
        assert!(run.error.is_none(), "{:?}", run.error);
        // The mode command: ALT_HOLD once at 2 s; AUTOTUNE once, 61 s after
        // the hover began.
        let m = &run.modes;
        assert_eq!(m.len(), 2, "{m:?}");
        assert_eq!(m[0], (2 * S, 2));
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
        // after, and the throttle goes low 900 s after.
        assert_eq!(run.analysed, vec![hover_at + HOVER_S * S]);
        assert_eq!(m[1], (hover_at + (HOVER_S + 1) * S, 15));
        let low = hover_at + (HOVER_S + 1 + AUTOTUNE_S) * S;
        assert_eq!(th[3], (low, 0.0));
        // Reads: groups of at most 6, a second apart, the first group at 3 s.
        let before: Vec<_> = run.reads.iter().filter(|(t, _)| *t < arm).collect();
        assert_eq!(before[0].0, READ_BEFORE_AT_US);
        for w in before.windows(2) {
            assert_eq!(w[1].0 - w[0].0, S);
        }
        assert!(run.reads.iter().all(|(_, g)| g.len() <= READ_GROUP));
        let asked: Vec<&str> = before.iter().flat_map(|(_, g)| g.iter().copied()).collect();
        assert_eq!(asked, before_names());
        // After the disarm (1000 s): nothing read between the throttle going
        // low and 5 s after the pulses said disarmed; then the five groups.
        let after: Vec<_> = run.reads.iter().filter(|(t, _)| *t > low).collect();
        assert!(
            after
                .iter()
                .filter(|(t, _)| *t > hover_at + 100 * S)
                .all(|(t, _)| *t >= 1_005 * S),
            "{after:?}"
        );
        let names: Vec<&str> = after.iter().flat_map(|(_, g)| g.iter().copied()).collect();
        assert_eq!(names, GAIN_NAMES.to_vec());
        assert_eq!(after[0].0, 1_005 * S);
        for w in after.windows(2) {
            assert_eq!(w[1].0 - w[0].0, S);
        }
        assert_eq!(run.end, Some(1_005 * S + 5 * S + READ_END_US));
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
        let run = drive(Script::new(true, false, 900), true, Some(200 * S));
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
        let run = drive(Script::new(false, true, 900), false, None);
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
        assert_eq!(check(&texts_ok(), &all_gains()), Ok(()));
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
            let e = check(&t, &all_gains()).expect_err(missing);
            assert!(e.contains("did not print"), "{e}");
        }
        for name in GAIN_NAMES {
            let mut g = all_gains();
            g.remove(name);
            let e = check(&texts_ok(), &g).expect_err(name);
            assert!(e.contains(name), "{e}");
        }
        // The tuner started and stopped is not a success.
        let t = vec![
            (0.0, "AutoTune: Started".to_string()),
            (1.0, "AutoTune: Stopped".to_string()),
        ];
        assert!(check(&t, &all_gains()).is_err());
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
        let text = tuned_file(&g, "abc123", "ff00", 7);
        let lines: Vec<&str> = text.lines().filter(|l| !l.starts_with('#')).collect();
        let mut sorted = lines.clone();
        sorted.sort_unstable();
        assert_eq!(lines, sorted, "sorted by name");
        assert!(
            text.contains("# commit abc123")
                && text.contains("# binary sha256 ff00")
                && text.contains("# seed 7")
        );
        assert!(text.contains("oxide-bridge tune"));
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
