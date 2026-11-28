// SPDX-License-Identifier: AGPL-3.0-or-later
//! The text of an event, put together from fixed pieces on the stack without
//! `core::fmt` (ADR-0005 §3), and the ULog level it is logged at.
//!
//! The wording: `Armed`, `Disarmed: touchdown`, `Arming refused: THROTTLE,
//! RC`, `Nav state: POSITION -> HOLD`, `Mode degraded: POSITION flown as
//! ALTITUDE`, `Reason raised: RC_LOST`, `Action: LAND`, `Parameter changed:
//! RATE_ROLL_P`, `Parameter refused: LIM_TILT (out of range)`. A bit this
//! code has no name for is written `BIT<n>`; a parameter outside the table
//! `#<index>`.

use oxide_core::io::{Action, DisarmReason, EventKind, FlightMode, NavState, Reasons};
use oxide_core::params::{self, LoadReport, ParamError, ParamId};

/// ULog levels (the ASCII digit is `b'0' + level`).
pub mod level {
    /// The vehicle is terminated or has lost control.
    pub const EMERGENCY: u8 = 0;
    /// A reason raised, arming refused, a parameter refused, a parameter-load fault.
    pub const WARNING: u8 = 4;
    /// Everything else.
    pub const INFO: u8 = 6;
}

/// Bytes the longest text needs, with room to spare: all 16 bits of a flag
/// set named, or a load fault with every counter at its largest.
const TEXT_MAX: usize = 256;

/// A text under construction: a fixed buffer, ASCII only. What does not fit
/// is cut and [`truncated`](Self::truncated) says so.
#[derive(Debug, Clone)]
pub struct TextBuf {
    bytes: [u8; TEXT_MAX],
    len: usize,
    cut: bool,
}

impl Default for TextBuf {
    fn default() -> Self {
        Self::new()
    }
}

impl TextBuf {
    /// An empty buffer.
    pub const fn new() -> Self {
        Self {
            bytes: [0; TEXT_MAX],
            len: 0,
            cut: false,
        }
    }

    /// Appends `s` (ASCII), or as much of it as fits.
    pub fn push(&mut self, s: &str) -> &mut Self {
        for &b in s.as_bytes() {
            match self.bytes.get_mut(self.len) {
                Some(slot) => {
                    *slot = b;
                    self.len += 1;
                }
                None => self.cut = true,
            }
        }
        self
    }

    /// Appends a number in decimal.
    pub fn push_num(&mut self, v: u64) -> &mut Self {
        let mut digits = [0u8; 20];
        self.push(decimal(v, &mut digits))
    }

    /// The text.
    pub fn as_str(&self) -> &str {
        self.bytes
            .get(..self.len)
            .and_then(|b| core::str::from_utf8(b).ok())
            .unwrap_or("")
    }

    /// True if something did not fit.
    pub const fn truncated(&self) -> bool {
        self.cut
    }
}

/// `v` in decimal, written into `buf`.
pub fn decimal(v: u64, buf: &mut [u8; 20]) -> &str {
    let mut at = buf.len();
    let mut rest = v;
    loop {
        at -= 1;
        if let Some(slot) = buf.get_mut(at) {
            *slot = b'0' + (rest % 10) as u8;
        }
        rest /= 10;
        if rest == 0 || at == 0 {
            break;
        }
    }
    buf.get(at..)
        .and_then(|b| core::str::from_utf8(b).ok())
        .unwrap_or("")
}

const REASON_NAMES: [&str; 11] = [
    "RC_LOST",
    "NO_HUMAN_LINK",
    "BRAIN_LOST",
    "SETPOINT_STALE",
    "BATTERY_LOW",
    "BATTERY_CRITICAL",
    "GPS_LOST",
    "ESTIMATOR_UNHEALTHY",
    "IMU_FAILURE",
    "GEOFENCE",
    "LOSS_OF_CONTROL",
];

const PREARM_NAMES: [&str; 9] = [
    "KILL",
    "THROTTLE",
    "RC",
    "NOT_STILL",
    "ESTIMATOR",
    "POSITION",
    "BATTERY",
    "CONFIG",
    "LINK",
];

fn mode_name(m: FlightMode) -> &'static str {
    match m {
        FlightMode::Stabilized => "STABILIZED",
        FlightMode::Altitude => "ALTITUDE",
        FlightMode::Position => "POSITION",
        FlightMode::Offboard => "OFFBOARD",
    }
}

fn nav_name(n: NavState) -> &'static str {
    match n {
        NavState::Disarmed => "DISARMED",
        NavState::Stabilized => "STABILIZED",
        NavState::Altitude => "ALTITUDE",
        NavState::Position => "POSITION",
        NavState::Offboard => "OFFBOARD",
        NavState::Hold => "HOLD",
        NavState::Rtl => "RTL",
        NavState::Land => "LAND",
        NavState::LandNow => "LAND_NOW",
        NavState::Terminated => "TERMINATED",
    }
}

fn action_name(a: Action) -> &'static str {
    match a {
        Action::None => "NONE",
        Action::Hold => "HOLD",
        Action::Rtl => "RTL",
        Action::Land => "LAND",
        Action::LandNow => "LAND_NOW",
        Action::Terminate => "TERMINATE",
    }
}

fn disarm_name(r: DisarmReason) -> &'static str {
    match r {
        DisarmReason::Switch => "switch",
        DisarmReason::Kill => "kill",
        DisarmReason::Touchdown => "touchdown",
        DisarmReason::Terminated => "terminated",
    }
}

fn param_why(b: &mut TextBuf, why: ParamError) {
    match why {
        ParamError::Unknown => b.push("unknown parameter"),
        ParamError::WrongType => b.push("wrong type"),
        ParamError::NotFinite => b.push("not finite"),
        ParamError::OutOfRange => b.push("out of range"),
        ParamError::Invalid(_) => b
            .push("invalid combination, code ")
            .push_num(u64::from(why.detail_code())),
        ParamError::Armed => b.push("armed"),
    };
}

/// The names of the set bits of `mask`, separated by `, `, after `lead`.
fn names(b: &mut TextBuf, lead: &str, mask: u16, table: &[&str]) {
    b.push(lead);
    let mut first = true;
    for bit in 0..16usize {
        if mask & (1 << bit) == 0 {
            continue;
        }
        b.push(if first { ": " } else { ", " });
        first = false;
        match table.get(bit) {
            Some(n) => b.push(n),
            None => b.push("BIT").push_num(bit as u64),
        };
    }
}

fn param_name(b: &mut TextBuf, id: ParamId) {
    match params::PARAMS.get(id.index()) {
        Some(def) => b.push(def.name),
        None => b.push("#").push_num(u64::from(id.0)),
    };
}

fn load_report(b: &mut TextBuf, r: &LoadReport, with_counts: bool) {
    b.push(" applied ").push_num(u64::from(r.applied));
    if with_counts {
        b.push(", unknown ").push_num(u64::from(r.unknown));
        b.push(", dropped ").push_num(u64::from(r.dropped));
    }
    b.push(", torn ").push_num(u64::from(r.torn));
    if r.invalid {
        b.push(", invalid");
    }
    if r.unreadable {
        b.push(", unreadable");
    }
}

/// Puts the text of an event into `b`.
pub fn event_text(kind: &EventKind, b: &mut TextBuf) {
    match kind {
        EventKind::Armed => {
            b.push("Armed");
        }
        EventKind::Disarmed(r) => {
            b.push("Disarmed: ").push(disarm_name(*r));
        }
        EventKind::ArmRefused(f) => names(b, "Arming refused", f.0, &PREARM_NAMES),
        EventKind::NavChanged { from, to } => {
            b.push("Nav state: ")
                .push(nav_name(*from))
                .push(" -> ")
                .push(nav_name(*to));
        }
        EventKind::ModeDegraded { requested, flown } => {
            b.push("Mode degraded: ")
                .push(mode_name(*requested))
                .push(" flown as ")
                .push(nav_name(*flown));
        }
        EventKind::ReasonRaised(r) => names(b, "Reason raised", r.0, &REASON_NAMES),
        EventKind::ReasonCleared(r) => names(b, "Reason cleared", r.0, &REASON_NAMES),
        EventKind::ActionChanged { to, .. } => {
            b.push("Action: ").push(action_name(*to));
        }
        EventKind::PilotOverride(r) => names(b, "Pilot override", r.0, &REASON_NAMES),
        EventKind::Touchdown => {
            b.push("Touchdown");
        }
        EventKind::HomeSet => {
            b.push("Home set");
        }
        EventKind::ParamChanged(id) => {
            b.push("Parameter changed: ");
            param_name(b, *id);
        }
        EventKind::ParamRefused { id, why } => {
            b.push("Parameter refused: ");
            param_name(b, *id);
            b.push(" (");
            param_why(b, *why);
            b.push(")");
        }
        EventKind::ParamLoadFault(r) => {
            b.push("Parameter load fault:");
            load_report(b, r, true);
        }
        EventKind::ParamLoadRecovered(r) => {
            b.push("Parameter load recovered:");
            load_report(b, r, false);
        }
        EventKind::ParamFaultAcked => {
            b.push("Parameter fault acknowledged");
        }
        EventKind::HeadingCorrected { angle_mrad, count } => {
            b.push("Heading corrected by ");
            if *angle_mrad < 0 {
                b.push("-");
            }
            b.push_num(u64::from(angle_mrad.unsigned_abs()));
            b.push(" mrad (correction ");
            b.push_num(u64::from(*count));
            b.push(")");
        }
        EventKind::HeadingContained => {
            b.push("Heading untrustworthy: contained until disarm");
        }
        EventKind::GpsSnapRepeated => {
            b.push("GPS rejected again soon after a reset to it");
        }
    }
}

/// The ULog level of an event: [`level::EMERGENCY`] for a termination and
/// for loss of control, [`level::WARNING`] for a reason raised, a refused
/// arming, a refused parameter and a parameter-load fault, [`level::INFO`]
/// for the rest. Every variant is named: a new one must be given a level.
pub fn event_level(kind: &EventKind) -> u8 {
    match kind {
        EventKind::Disarmed(DisarmReason::Terminated)
        | EventKind::NavChanged {
            to: NavState::Terminated,
            ..
        }
        | EventKind::ActionChanged {
            to: Action::Terminate,
            ..
        } => level::EMERGENCY,
        EventKind::ReasonRaised(r) if r.contains(Reasons::LOSS_OF_CONTROL) => level::EMERGENCY,
        EventKind::ReasonRaised(_)
        | EventKind::HeadingContained
        | EventKind::ArmRefused(_)
        | EventKind::ParamRefused { .. }
        | EventKind::ParamLoadFault(_) => level::WARNING,
        EventKind::Armed
        | EventKind::Disarmed(_)
        | EventKind::NavChanged { .. }
        | EventKind::ModeDegraded { .. }
        | EventKind::ReasonCleared(_)
        | EventKind::ActionChanged { .. }
        | EventKind::PilotOverride(_)
        | EventKind::Touchdown
        | EventKind::HomeSet
        | EventKind::ParamChanged(_)
        | EventKind::ParamLoadRecovered(_)
        | EventKind::HeadingCorrected { .. }
        | EventKind::GpsSnapRepeated
        | EventKind::ParamFaultAcked => level::INFO,
    }
}
