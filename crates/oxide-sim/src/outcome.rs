// SPDX-License-Identifier: AGPL-3.0-or-later
//! What happened in a flight, and what a fault scenario requires of it
//! (ADR-0003). [`FlightOutcome`] is gathered from the step records;
//! [`Expect`] turns a list of requirements into one line per violation.
//!
//! World frame NED, SI units, angles in degrees where the name says so.

use oxide_core::io::{Action, DisarmReason, Event, EventKind, NavState, Reasons};
use oxide_core::math::{Real, Vec3};

use oxide_core::time::Micros;

use crate::sim::{CoreRecord, StepRecord};
use crate::vehicle::GroundModel;

/// Height above the ground below which the vehicle counts as touching it, m:
/// the ground model's contact condition (`z > -sag`).
fn ground_m() -> f64 {
    GroundModel::default().sag_m()
}
/// The vehicle must have been above this to have "come down", m.
const AIRBORNE_M: f64 = 0.3;

/// `value > limit`, or `value` is NaN (so that NaN fails every limit).
/// How long after a pilot override the action dropping to None counts as
/// the takeover, s (the status lags the event by a status period, 20 ms).
const OVERRIDE_WINDOW_S: f64 = 0.1;

fn exceeds(value: f64, limit: f64) -> bool {
    value.is_nan() || value > limit
}

/// The reasons that cause a Hold (ADR-0003); a Hold must not end while one of
/// them is still set.
const HOLD_REASONS: Reasons = Reasons::RC_LOST
    .union(Reasons::BRAIN_LOST)
    .union(Reasons::SETPOINT_STALE)
    .union(Reasons::GPS_LOST);

/// What happened in one run, gathered from the step records. The first
/// part is judged from the physics truth and the autopilot's `armed` flag,
/// whatever autopilot flew; the second ([`CoreOutcome`]) exists only when the
/// records carried our flight core's status.
#[derive(Clone, Debug, PartialEq)]
pub struct FlightOutcome {
    pub armed_at_end: bool,
    pub position_at_end: Vec3<f64>,
    /// Time of the last record, s.
    pub end_s: f64,
    /// Home as the truth position at the first arming.
    pub home: Option<Vec3<f64>>,
    pub max_height_m: f64,
    /// Greatest horizontal distance from home, m.
    pub max_distance_from_home_m: f64,
    /// Greatest angle between the body's down axis and the vertical while
    /// armed (truth), deg.
    pub max_tilt_deg: f64,
    /// Downward speed (truth) at the first ground contact after having been
    /// above 0.3 m; None if it never came down.
    pub touchdown_speed_mps: Option<f64>,
    /// When the vehicle was last disarmed, s. With the core part it is the
    /// time of the core's last Disarmed event (the supervisor decides at that
    /// tick, and the status shows it one supervisor period later: the flag's
    /// change would be a later tick). Without it, the time of the last record
    /// at which the armed flag changed from true to false.
    pub disarmed_at_s: Option<f64>,
    /// Records with the vehicle disarmed and a motor command above zero.
    pub motor_ticks_while_disarmed: u32,
    pub non_finite: bool,
    pub physics_faults: u32,
    /// What only our flight core reports; `None` when the records had no
    /// core part.
    pub core: Option<CoreOutcome>,
    // Recorder state.
    was_above: bool,
    touched_down: bool,
    prev_down_speed: f64,
    prev_armed: bool,
}

/// The part of a [`FlightOutcome`] that comes from our flight core's status
/// and events.
#[derive(Clone, Debug, PartialEq)]
pub struct CoreOutcome {
    /// Every event, in order.
    pub events: Vec<Event>,
    /// Navigation states in the order they were entered (consecutive
    /// duplicates removed). The first is the state of the first record.
    pub nav_sequence: Vec<NavState>,
    /// Every reason that was raised at some point.
    pub reasons_seen: Reasons,
    /// The highest failsafe action reached.
    pub max_action: Action,
    /// True if, while armed, the action ever dropped to a lower-ranked one
    /// other than Hold to None once no Hold reason is left, unless it dropped to None within
    /// `OVERRIDE_WINDOW_S` after a pilot override.
    pub action_decreased: bool,
    pub nav_at_end: NavState,
    /// The last Disarmed event.
    pub disarm_reason: Option<DisarmReason>,
    pub timing_faults: u32,
    // Recorder state.
    /// Until this time, s, an action dropping to None is a pilot takeover.
    override_until_s: f64,
    prev_action: Action,
}

impl Default for FlightOutcome {
    fn default() -> Self {
        Self::new()
    }
}

impl CoreOutcome {
    fn new() -> Self {
        Self {
            events: Vec::new(),
            nav_sequence: Vec::new(),
            reasons_seen: Reasons::empty(),
            max_action: Action::None,
            action_decreased: false,
            nav_at_end: NavState::Disarmed,
            disarm_reason: None,
            timing_faults: 0,
            override_until_s: f64::NEG_INFINITY,
            prev_action: Action::None,
        }
    }

    /// Adds one tick's core part; `prev_armed` is the armed flag of the
    /// record before. Returns the time of the tick's last Disarmed event.
    fn add(&mut self, core: &CoreRecord, t: Micros, prev_armed: bool) -> Option<f64> {
        let s = &core.status;
        let mut disarmed_at = None;
        for e in core.events.iter() {
            self.events.push(*e);
            match e.kind {
                EventKind::Disarmed(reason) => {
                    self.disarm_reason = Some(reason);
                    disarmed_at = Some(e.t.as_secs_f64());
                }
                EventKind::PilotOverride(_) => {
                    self.override_until_s = e.t.as_secs_f64() + OVERRIDE_WINDOW_S;
                }
                _ => {}
            }
        }
        if self.nav_sequence.last() != Some(&s.nav) {
            self.nav_sequence.push(s.nav);
        }
        self.reasons_seen.insert(s.reasons);
        self.max_action = self.max_action.max(s.action);

        // Escalation only: judged while armed (disarming resets everything).
        if s.armed && prev_armed && s.action < self.prev_action {
            // A Hold may end when none of its reasons is left in this record
            // (a pilot override is excused by `taken_over` below).
            let hold_ended = self.prev_action == Action::Hold
                && s.action == Action::None
                && !s.reasons.intersects(HOLD_REASONS);
            // The pilot's takeover ends the action (to None); the status
            // follows the event by up to a status period, so a short window
            // after the event excuses that one decrease.
            let taken_over = s.action == Action::None && t.as_secs_f64() <= self.override_until_s;
            if !hold_ended && !taken_over {
                self.action_decreased = true;
            }
        }
        self.prev_action = s.action;
        self.nav_at_end = s.nav;
        self.timing_faults = s.timing_faults;
        disarmed_at
    }
}

impl FlightOutcome {
    pub fn new() -> Self {
        Self {
            armed_at_end: false,
            position_at_end: Vec3::zero(),
            end_s: 0.0,
            home: None,
            max_height_m: 0.0,
            max_distance_from_home_m: 0.0,
            max_tilt_deg: 0.0,
            touchdown_speed_mps: None,
            disarmed_at_s: None,
            motor_ticks_while_disarmed: 0,
            non_finite: false,
            physics_faults: 0,
            core: None,
            was_above: false,
            touched_down: false,
            prev_down_speed: 0.0,
            prev_armed: false,
        }
    }

    /// The core part, created empty if the outcome has none yet (for tests
    /// that set a field by hand).
    #[cfg(test)]
    pub(crate) fn core_mut(&mut self) -> &mut CoreOutcome {
        self.core.get_or_insert_with(CoreOutcome::new)
    }

    /// Adds one tick.
    pub fn add<const N: usize>(&mut self, r: &StepRecord<N>) {
        let disarmed_at = if let Some(core) = r.core.as_ref() {
            self.core
                .get_or_insert_with(CoreOutcome::new)
                .add(core, r.t, self.prev_armed)
        } else if self.prev_armed && !r.armed {
            Some(r.t.as_secs_f64())
        } else {
            None
        };
        if disarmed_at.is_some() {
            self.disarmed_at_s = disarmed_at;
        }
        self.prev_armed = r.armed;

        if r.armed && self.home.is_none() {
            self.home = Some(r.position);
        }
        self.armed_at_end = r.armed;
        self.position_at_end = r.position;
        self.end_s = r.t.as_secs_f64();
        self.physics_faults = r.physics_faults;
        if !r.armed && r.motors.iter().any(|m| *m > 0.0) {
            self.motor_ticks_while_disarmed = self.motor_ticks_while_disarmed.saturating_add(1);
        }

        let height = -r.position.z;
        if !(r.position.is_finite() && r.velocity.is_finite() && r.attitude.is_finite()) {
            self.non_finite = true;
            return;
        }
        self.max_height_m = self.max_height_m.max(height);
        if let Some(h) = self.home {
            let d = r.position - h;
            let dh = Real::sqrt(d.x * d.x + d.y * d.y);
            self.max_distance_from_home_m = self.max_distance_from_home_m.max(dh);
        }
        if r.armed {
            let down = r.attitude.rotate(Vec3::new(0.0, 0.0, 1.0));
            let tilt = Real::atan2(Real::sqrt(down.x * down.x + down.y * down.y), down.z);
            if tilt.is_finite() {
                self.max_tilt_deg = self.max_tilt_deg.max(tilt.to_degrees());
            } else {
                self.non_finite = true;
            }
        }
        // Touchdown speed: the downward speed of the record before the first
        // one at the ground (in contact the ground decelerates the vehicle).
        if !self.touched_down {
            if height > AIRBORNE_M {
                self.was_above = true;
            } else if self.was_above && height < ground_m() {
                self.touched_down = true;
                self.touchdown_speed_mps = Some(self.prev_down_speed.max(r.velocity.z).max(0.0));
            }
        }
        self.prev_down_speed = r.velocity.z;
    }
}

/// How a run should end.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Ending {
    /// Still flying, armed, within `tolerance_m` of `point` (NED, 3D).
    Flying { point: Vec3<f64>, tolerance_m: f64 },
    /// Still flying, armed, with its height within `tolerance_m` of
    /// `height_m` above the ground (horizontal position not judged).
    FlyingAtHeight { height_m: f64, tolerance_m: f64 },
    /// On the ground and disarmed by touchdown; within this distance of home
    /// if given.
    Landed { within_m_of_home: Option<f64> },
    /// The failsafe terminated the flight.
    Terminated,
    /// On the ground and disarmed for the given reason (kill switch, pilot).
    Disarmed(DisarmReason),
    /// On the ground and disarmed, by touchdown or because the flight was
    /// terminated. Either is a safe end for a pairing of failures.
    Down,
}

/// An event kind, without its data.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EventTag {
    Armed,
    ArmRefused,
    ModeDegraded,
    PilotOverride,
    Touchdown,
    HomeSet,
}

impl EventTag {
    pub fn matches(self, k: &EventKind) -> bool {
        matches!(
            (self, k),
            (EventTag::Armed, EventKind::Armed)
                | (EventTag::ArmRefused, EventKind::ArmRefused(_))
                | (EventTag::ModeDegraded, EventKind::ModeDegraded { .. })
                | (EventTag::PilotOverride, EventKind::PilotOverride(_))
                | (EventTag::Touchdown, EventKind::Touchdown)
                | (EventTag::HomeSet, EventKind::HomeSet)
        )
    }
}

/// What a scenario requires of its outcome.
#[derive(Clone, Debug, PartialEq)]
pub struct Expect {
    /// These navigation states must appear in this order (others may come
    /// between).
    pub nav_contains: Vec<NavState>,
    /// These must never appear.
    pub nav_never: Vec<NavState>,
    /// Every one of these reasons must have been raised.
    pub reasons: Reasons,
    /// None of these may have been raised.
    pub reasons_never: Reasons,
    /// Each of these events must have occurred.
    pub events: Vec<EventTag>,
    /// The highest action reached must be at least this.
    pub min_action: Action,
    pub ending: Ending,
    /// The ending must have been reached by this time, s: the disarm time
    /// for the ground endings; for the flying endings the run must last at
    /// least this long.
    pub by_s: f64,
    /// Touchdown speed limit for `Landed` and `Down` endings, m/s.
    pub max_touchdown_mps: f64,
    /// Tilt limit while armed, deg (not checked for `Terminated`).
    pub max_tilt_deg: f64,
    /// The vehicle must have been at least this high at some time, m
    /// (proves that the flight the fault hits really took place).
    pub min_height_m: Option<f64>,
    /// The vehicle must never have been higher than this, m.
    pub max_height_m: Option<f64>,
    /// The vehicle must never have been further than this from home
    /// (horizontally), m.
    pub max_distance_from_home_m: Option<f64>,
}

impl Default for Expect {
    /// Nothing required but a run that stays armed at its start point, with
    /// the default limits (touchdown 1.5 m/s, tilt 40 deg).
    fn default() -> Self {
        Self {
            nav_contains: Vec::new(),
            nav_never: Vec::new(),
            reasons: Reasons::empty(),
            reasons_never: Reasons::empty(),
            events: Vec::new(),
            min_action: Action::None,
            ending: Ending::Flying {
                point: Vec3::zero(),
                tolerance_m: 1.0,
            },
            by_s: 0.0,
            max_touchdown_mps: 1.5,
            max_tilt_deg: 40.0,
            min_height_m: None,
            max_height_m: None,
            max_distance_from_home_m: None,
        }
    }
}

/// Every failsafe reason with its name, in bit order. The names are the ones
/// scenario files use.
pub const REASON_NAMES: [(Reasons, &str); 11] = [
    (Reasons::RC_LOST, "RC_LOST"),
    (Reasons::NO_HUMAN_LINK, "NO_HUMAN_LINK"),
    (Reasons::BRAIN_LOST, "BRAIN_LOST"),
    (Reasons::SETPOINT_STALE, "SETPOINT_STALE"),
    (Reasons::BATTERY_LOW, "BATTERY_LOW"),
    (Reasons::BATTERY_CRITICAL, "BATTERY_CRITICAL"),
    (Reasons::GPS_LOST, "GPS_LOST"),
    (Reasons::ESTIMATOR_UNHEALTHY, "ESTIMATOR_UNHEALTHY"),
    (Reasons::IMU_FAILURE, "IMU_FAILURE"),
    (Reasons::GEOFENCE, "GEOFENCE"),
    (Reasons::LOSS_OF_CONTROL, "LOSS_OF_CONTROL"),
];

/// Names of the bits set in `r`, e.g. "RC_LOST|GPS_LOST".
pub fn reason_names(r: Reasons) -> String {
    let v: Vec<&str> = REASON_NAMES
        .iter()
        .filter(|(bit, _)| r.contains(*bit))
        .map(|(_, n)| *n)
        .collect();
    if v.is_empty() {
        "none".into()
    } else {
        v.join("|")
    }
}

/// An event as one line: "  16.520 s  ReasonRaised(GPS_LOST)".
pub fn event_line(e: &Event) -> String {
    let what = match e.kind {
        EventKind::ReasonRaised(r) => format!("ReasonRaised({})", reason_names(r)),
        EventKind::ReasonCleared(r) => format!("ReasonCleared({})", reason_names(r)),
        EventKind::PilotOverride(r) => format!("PilotOverride({})", reason_names(r)),
        other => format!("{other:?}"),
    };
    format!("{:8.3} s  {what}", e.t.as_secs_f64())
}

/// Why an expectation or invariant cannot be judged without the core part.
pub const NEEDS_CORE: &str = "needs the flight core's status";

/// A requirement the outcome cannot answer: it is neither passed nor
/// violated.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct NotJudged {
    /// The `[expect]` key or invariant, with the part of it that needs the
    /// core where only a part does.
    pub key: &'static str,
    pub reason: &'static str,
}

impl NotJudged {
    pub(crate) const fn core(key: &'static str) -> Self {
        Self {
            key,
            reason: NEEDS_CORE,
        }
    }
}

impl core::fmt::Display for NotJudged {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{}: {}", self.key, self.reason)
    }
}

/// The answer to "does this outcome meet the requirements": what is
/// violated, and what could not be judged. For an outcome with the core part
/// `not_judged` is empty.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Judgement {
    /// One line per violation.
    pub violations: Vec<String>,
    pub not_judged: Vec<NotJudged>,
}

impl Judgement {
    /// True when nothing is violated and nothing is left unjudged: a
    /// requirement that could not be judged is not a pass.
    pub fn passed(&self) -> bool {
        self.violations.is_empty() && self.not_judged.is_empty()
    }
}

impl FlightOutcome {
    /// The checks every flight gets, whether or not its file states an
    /// `[expect]`: no non-finite truth, no physics or timing faults, the
    /// action never decreased while armed, no motor command while disarmed.
    /// The timing-fault and action checks read the core's status: without the
    /// core part they are listed as not judged.
    pub fn invariants(&self) -> Judgement {
        let o = self;
        let mut j = Judgement::default();
        let v = &mut j.violations;
        if o.non_finite {
            v.push("non-finite value in truth".into());
        }
        if o.physics_faults > 0 {
            v.push(format!("{} non-finite physics steps", o.physics_faults));
        }
        match &o.core {
            Some(c) => {
                if c.timing_faults > 0 {
                    v.push(format!("{} timing faults", c.timing_faults));
                }
                if c.action_decreased {
                    v.push("failsafe action decreased while armed".into());
                }
            }
            None => {
                j.not_judged
                    .push(NotJudged::core("invariant: no timing faults"));
                j.not_judged.push(NotJudged::core(
                    "invariant: failsafe action never decreases while armed",
                ));
            }
        }
        if o.motor_ticks_while_disarmed > 0 {
            j.violations.push(format!(
                "motors commanded on for {} ticks while disarmed",
                o.motor_ticks_while_disarmed
            ));
        }
        j
    }
}

impl Expect {
    /// What is violated and what cannot be judged. Always also the flight's
    /// invariants ([`FlightOutcome::invariants`]), listed first.
    ///
    /// Judged from the truth (any autopilot): `min_height_m`, `max_height_m`,
    /// `max_distance_from_home_m`, `max_tilt_deg`, `by_s`, `max_touchdown_mps`
    /// and every `ending` as far as it reads position, the armed flag, the
    /// disarm time and the touchdown. Needing the core's status
    /// ([`NEEDS_CORE`]): `nav_contains`, `nav_never`, `reasons`,
    /// `reasons_never`, `events`, `min_action` (each only when it requires
    /// something), and the parts of the endings that name a disarm reason or
    /// a navigation state. A key left at its empty default requires nothing
    /// and is not listed.
    pub fn check(&self, o: &FlightOutcome) -> Judgement {
        let mut j = o.invariants();
        let v = &mut j.violations;

        let Some(c) = &o.core else {
            let nj = &mut j.not_judged;
            if !self.nav_contains.is_empty() {
                nj.push(NotJudged::core("nav_contains"));
            }
            if !self.nav_never.is_empty() {
                nj.push(NotJudged::core("nav_never"));
            }
            if !self.reasons.is_empty() {
                nj.push(NotJudged::core("reasons"));
            }
            if !self.reasons_never.is_empty() {
                nj.push(NotJudged::core("reasons_never"));
            }
            if !self.events.is_empty() {
                nj.push(NotJudged::core("events"));
            }
            if self.min_action > Action::None {
                nj.push(NotJudged::core("min_action"));
            }
            self.check_limits(o, &mut j.violations);
            self.check_ending(o, &mut j);
            return j;
        };

        // Navigation states, in order.
        let mut seq = c.nav_sequence.iter();
        for want in &self.nav_contains {
            if !seq.any(|n| n == want) {
                v.push(format!(
                    "navigation never reached {want:?} in the required order (saw {:?})",
                    c.nav_sequence
                ));
                break;
            }
        }
        for bad in &self.nav_never {
            if c.nav_sequence.contains(bad) {
                v.push(format!("navigation entered {bad:?}"));
            }
        }

        let missing = self.reasons.difference(c.reasons_seen);
        if !missing.is_empty() {
            v.push(format!("reason never raised: {}", reason_names(missing)));
        }
        let unwanted = c.reasons_seen.intersection(self.reasons_never);
        if !unwanted.is_empty() {
            v.push(format!("reason raised: {}", reason_names(unwanted)));
        }
        for tag in &self.events {
            if !c.events.iter().any(|e| tag.matches(&e.kind)) {
                v.push(format!("no {tag:?} event"));
            }
        }
        if c.max_action < self.min_action {
            v.push(format!(
                "highest action {:?} below the required {:?}",
                c.max_action, self.min_action
            ));
        }
        self.check_limits(o, &mut j.violations);
        self.check_ending(o, &mut j);
        j
    }

    /// The limits that read the truth only.
    fn check_limits(&self, o: &FlightOutcome, v: &mut Vec<String>) {
        if let Some(limit) = self.min_height_m
            && o.max_height_m < limit
        {
            v.push(format!(
                "only reached {:.2} m high, required {limit} m",
                o.max_height_m
            ));
        }
        if let Some(limit) = self.max_height_m
            && exceeds(o.max_height_m, limit)
        {
            v.push(format!(
                "reached {:.2} m high, limit {limit} m",
                o.max_height_m
            ));
        }
        if let Some(limit) = self.max_distance_from_home_m
            && exceeds(o.max_distance_from_home_m, limit)
        {
            v.push(format!(
                "went {:.2} m from home, limit {limit} m",
                o.max_distance_from_home_m
            ));
        }
        if self.ending != Ending::Terminated && exceeds(o.max_tilt_deg, self.max_tilt_deg) {
            v.push(format!(
                "tilt {:.1} deg, limit {} deg",
                o.max_tilt_deg, self.max_tilt_deg
            ));
        }
    }

    fn check_ending(&self, o: &FlightOutcome, j: &mut Judgement) {
        let v = &mut j.violations;
        let height_end = -o.position_at_end.z;
        let on_ground = height_end < AIRBORNE_M;
        let by_s = self.by_s;
        let disarm_reason = o.core.as_ref().and_then(|c| c.disarm_reason);
        // The disarm checks shared by the ground endings.
        let disarmed_by = |v: &mut Vec<String>, allowed: &[DisarmReason]| {
            if o.armed_at_end {
                v.push("still armed at the end".into());
                return;
            }
            let check_time = |v: &mut Vec<String>, t: f64| {
                if t > by_s {
                    v.push(format!("disarmed at {t:.2} s, later than {by_s} s"));
                }
            };
            if o.core.is_some() {
                match (disarm_reason, o.disarmed_at_s) {
                    (Some(r), Some(t)) => {
                        if !allowed.contains(&r) {
                            v.push(format!("disarmed for {r:?}, expected one of {allowed:?}"));
                        }
                        check_time(v, t);
                    }
                    _ => v.push("never disarmed".into()),
                }
            } else {
                // The reason is the core's to give; the time is the flag's.
                match o.disarmed_at_s {
                    Some(t) => check_time(v, t),
                    None => v.push("never disarmed".into()),
                }
            }
            if !on_ground {
                v.push(format!("disarmed but {height_end:.1} m above the ground"));
            }
        };
        let touchdown_speed = |v: &mut Vec<String>| match o.touchdown_speed_mps {
            Some(s) if s <= self.max_touchdown_mps => {}
            Some(s) => v.push(format!(
                "touchdown at {s:.2} m/s, limit {} m/s",
                self.max_touchdown_mps
            )),
            None if o.max_height_m > AIRBORNE_M => {
                v.push("never came down to the ground".into());
            }
            None => {}
        };
        let has_core = o.core.is_some();

        match self.ending {
            Ending::Flying { point, tolerance_m } => {
                let d = (o.position_at_end - point).norm();
                self.check_flying(o, j);
                if exceeds(d, tolerance_m) {
                    j.violations.push(format!(
                        "ended {d:.2} m from the point (limit {tolerance_m} m)"
                    ));
                }
            }
            Ending::FlyingAtHeight {
                height_m,
                tolerance_m,
            } => {
                self.check_flying(o, j);
                let e = (height_end - height_m).abs();
                if exceeds(e, tolerance_m) {
                    j.violations.push(format!(
                        "ended at {height_end:.2} m, {e:.2} m off {height_m} m (limit {tolerance_m} m)"
                    ));
                }
            }
            Ending::Landed { within_m_of_home } => {
                disarmed_by(v, &[DisarmReason::Touchdown]);
                touchdown_speed(v);
                if let (Some(limit), Some(home)) = (within_m_of_home, o.home) {
                    let d = o.position_at_end - home;
                    let dh = Real::sqrt(d.x * d.x + d.y * d.y);
                    if exceeds(dh, limit) {
                        v.push(format!("landed {dh:.2} m from home (limit {limit} m)"));
                    }
                }
                if !has_core {
                    j.not_judged
                        .push(NotJudged::core("ending landed: disarm reason Touchdown"));
                }
            }
            Ending::Terminated => {
                disarmed_by(v, &[DisarmReason::Terminated]);
                match &o.core {
                    Some(c) => {
                        if !c.nav_sequence.contains(&NavState::Terminated) {
                            v.push("navigation never reached Terminated".into());
                        }
                    }
                    None => {
                        j.not_judged.push(NotJudged::core(
                            "ending terminated: disarm reason Terminated and navigation state",
                        ));
                    }
                }
            }
            Ending::Disarmed(reason) => {
                disarmed_by(v, &[reason]);
                if !has_core {
                    j.not_judged
                        .push(NotJudged::core("ending disarmed: disarm reason"));
                }
            }
            Ending::Down => {
                disarmed_by(v, &[DisarmReason::Touchdown, DisarmReason::Terminated]);
                if disarm_reason == Some(DisarmReason::Touchdown) {
                    touchdown_speed(v);
                }
                if !has_core {
                    j.not_judged.push(NotJudged::core(
                        "ending down: disarm reason, and the touchdown speed of a touchdown disarm",
                    ));
                }
            }
        }
    }

    fn check_flying(&self, o: &FlightOutcome, j: &mut Judgement) {
        let v = &mut j.violations;
        if !o.armed_at_end {
            v.push(format!(
                "not armed at the end (disarmed {:?} at {:?} s)",
                o.core.as_ref().and_then(|c| c.disarm_reason),
                o.disarmed_at_s
            ));
        }
        match &o.core {
            Some(c) => {
                if c.nav_at_end == NavState::Terminated {
                    v.push("terminated".into());
                }
            }
            None => j
                .not_judged
                .push(NotJudged::core("ending flying: navigation not Terminated")),
        }
        if o.end_s < self.by_s - 0.05 {
            j.violations.push(format!(
                "the run ended at {:.2} s, before {} s",
                o.end_s, self.by_s
            ));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxide_core::io::{EventList, Status};
    use oxide_core::math::Quat;
    use oxide_core::time::Micros;

    fn status(armed: bool, nav: NavState, action: Action, reasons: Reasons) -> Status {
        Status {
            armed,
            nav,
            action,
            reasons,
            ..Status::default()
        }
    }

    fn rec(t_s: f64, pos: Vec3<f64>, st: Status) -> StepRecord<4> {
        StepRecord {
            t: Micros((t_s * 1e6) as u64),
            position: pos,
            velocity: Vec3::zero(),
            attitude: Quat::identity(),
            rate: Vec3::zero(),
            motors: [0.0; 4],
            rotor_speed: [0.0; 4],
            battery_v: 22.0,
            battery_a: 0.0,
            armed: st.armed,
            setpoint: None,
            wind: Vec3::zero(),
            physics_faults: 0,
            core: Some(crate::CoreRecord {
                status: st,
                events: EventList::new(),
                estimate: None,
            }),
        }
    }

    fn with_event(mut r: StepRecord<4>, kind: EventKind) -> StepRecord<4> {
        let t = r.t;
        let core = r.core.as_mut().expect("the record has a core part");
        assert!(core.events.push(Event { t, kind }));
        r
    }

    fn flying(t: f64, pos: Vec3<f64>, action: Action) -> StepRecord<4> {
        rec(
            t,
            pos,
            status(true, NavState::Offboard, action, Reasons::empty()),
        )
    }

    fn up(m: f64) -> Vec3<f64> {
        Vec3::new(0.0, 0.0, -m)
    }

    #[test]
    fn nav_sequence_drops_consecutive_duplicates() {
        let mut o = FlightOutcome::new();
        for (t, nav) in [
            (0.0, NavState::Disarmed),
            (0.1, NavState::Disarmed),
            (0.2, NavState::Offboard),
            (0.3, NavState::Offboard),
            (0.4, NavState::Hold),
            (0.5, NavState::Offboard),
        ] {
            o.add(&rec(
                t,
                up(1.0),
                status(true, nav, Action::None, Reasons::empty()),
            ));
        }
        assert_eq!(
            o.core.as_ref().expect("core part").nav_sequence,
            vec![
                NavState::Disarmed,
                NavState::Offboard,
                NavState::Hold,
                NavState::Offboard
            ]
        );
        assert_eq!(
            o.core.as_ref().expect("core part").nav_at_end,
            NavState::Offboard
        );
    }

    #[test]
    fn reasons_and_max_action_accumulate() {
        let mut o = FlightOutcome::new();
        o.add(&rec(
            0.0,
            up(1.0),
            status(true, NavState::Hold, Action::Hold, Reasons::RC_LOST),
        ));
        o.add(&rec(
            0.1,
            up(1.0),
            status(true, NavState::Rtl, Action::Rtl, Reasons::GPS_LOST),
        ));
        assert_eq!(
            o.core.as_ref().expect("core part").reasons_seen,
            Reasons::RC_LOST.union(Reasons::GPS_LOST)
        );
        assert_eq!(o.core.as_ref().expect("core part").max_action, Action::Rtl);
        assert!(!o.core.as_ref().expect("core part").action_decreased);
    }

    #[test]
    fn action_decrease_rules() {
        let run = |actions: &[(Action, bool)], override_at: Option<usize>| {
            let mut o = FlightOutcome::new();
            for (i, (a, armed)) in actions.iter().enumerate() {
                let mut r = rec(
                    i as f64 * 0.1,
                    up(1.0),
                    status(*armed, NavState::Offboard, *a, Reasons::empty()),
                );
                if override_at == Some(i) {
                    r = with_event(r, EventKind::PilotOverride(Reasons::RC_LOST));
                }
                o.add(&r);
            }
            o.core.as_ref().expect("core part").action_decreased
        };
        let t = true;
        // Hold to None is a normal end of a hold.
        assert!(!run(&[(Action::Hold, t), (Action::None, t)], None));
        // Rtl back to Hold or None is not.
        assert!(run(&[(Action::Rtl, t), (Action::Hold, t)], None));
        assert!(run(&[(Action::Rtl, t), (Action::None, t)], None));
        // Land to Rtl neither.
        assert!(run(&[(Action::Land, t), (Action::Rtl, t)], None));
        // A pilot override (here in the same record) excuses the drop to None.
        assert!(!run(&[(Action::Rtl, t), (Action::None, t)], Some(1)));
        // The status may lag the event by a few records.
        assert!(!run(
            &[(Action::Rtl, t), (Action::Rtl, t), (Action::None, t)],
            Some(1)
        ));
        // An override that leaves Rtl in force excuses no drop to Hold.
        assert!(run(
            &[(Action::Rtl, t), (Action::Rtl, t), (Action::Hold, t)],
            Some(1)
        ));
        // Nor a drop to None long after it (records are 0.1 s apart here).
        assert!(run(
            &[
                (Action::Rtl, t),
                (Action::Rtl, t),
                (Action::Rtl, t),
                (Action::None, t)
            ],
            Some(0)
        ));
        // Disarming resets everything.
        assert!(!run(&[(Action::Land, t), (Action::None, false)], None));
        // The excuse ends when the action returns to None.
        assert!(run(
            &[
                (Action::Rtl, t),
                (Action::None, t),
                (Action::Rtl, t),
                (Action::Hold, t)
            ],
            Some(1)
        ));
    }

    #[test]
    fn hold_ending_while_its_reason_is_still_set_is_a_decrease() {
        let run = |reasons_after: Reasons, override_on_second: bool| {
            let mut o = FlightOutcome::new();
            o.add(&rec(
                0.0,
                up(1.0),
                status(true, NavState::Hold, Action::Hold, Reasons::RC_LOST),
            ));
            let mut r = rec(
                0.1,
                up(1.0),
                status(true, NavState::Offboard, Action::None, reasons_after),
            );
            if override_on_second {
                r = with_event(r, EventKind::PilotOverride(Reasons::RC_LOST));
            }
            o.add(&r);
            o.core.as_ref().expect("core part").action_decreased
        };
        // Every reason that causes a Hold, still set: the Hold ended early.
        for reason in [
            Reasons::RC_LOST,
            Reasons::BRAIN_LOST,
            Reasons::SETPOINT_STALE,
            Reasons::GPS_LOST,
        ] {
            assert!(run(reason, false), "{reason:?}");
            // ... unless the pilot took over on that tick.
            assert!(!run(reason, true), "{reason:?} with override");
        }
        // The reason cleared, or only an unrelated reason is left: legitimate.
        assert!(!run(Reasons::empty(), false));
        assert!(!run(Reasons::BATTERY_LOW, false));
    }

    #[test]
    fn home_distance_height_and_tilt() {
        let mut o = FlightOutcome::new();
        o.add(&rec(
            0.0,
            Vec3::new(1.0, 2.0, 0.0),
            status(false, NavState::Disarmed, Action::None, Reasons::empty()),
        ));
        assert_eq!(o.home, None);
        // First armed record sets home, whatever came before.
        o.add(&flying(0.1, Vec3::new(1.0, 2.0, 0.0), Action::None));
        assert_eq!(o.home, Some(Vec3::new(1.0, 2.0, 0.0)));
        // 3-4-5 horizontally, 7 m up: distance is horizontal.
        o.add(&flying(0.2, Vec3::new(4.0, 6.0, -7.0), Action::None));
        o.add(&flying(0.3, Vec3::new(1.0, 2.0, -3.0), Action::None));
        assert!((o.max_distance_from_home_m - 5.0).abs() < 1e-9);
        assert!((o.max_height_m - 7.0).abs() < 1e-9);
        // Tilt: 30 deg of roll while armed counts, 60 deg disarmed does not.
        let mut r = flying(0.4, up(1.0), Action::None);
        r.attitude = Quat::from_rotation_vector(Vec3::new(30.0_f64.to_radians(), 0.0, 0.0));
        o.add(&r);
        let mut r = rec(
            0.5,
            up(1.0),
            status(false, NavState::Disarmed, Action::None, Reasons::empty()),
        );
        r.attitude = Quat::from_rotation_vector(Vec3::new(60.0_f64.to_radians(), 0.0, 0.0));
        o.add(&r);
        assert!((o.max_tilt_deg - 30.0).abs() < 1e-6, "{}", o.max_tilt_deg);
        // Combined pitch and roll: the angle from vertical, not the maximum.
        let mut r = flying(0.6, up(1.0), Action::None);
        r.attitude = Quat::from_rotation_vector(Vec3::new(0.0, 0.0, 0.0));
        let q = Quat::from_rotation_vector(Vec3::new(0.0, 0.6, 0.0))
            * Quat::from_rotation_vector(Vec3::new(0.6, 0.0, 0.0));
        r.attitude = q;
        o.add(&r);
        assert!(o.max_tilt_deg > 40.0, "{}", o.max_tilt_deg);
    }

    #[test]
    fn touchdown_speed_is_taken_before_the_ground_slows_it() {
        let mut o = FlightOutcome::new();
        let mut a = flying(1.0, up(5.0), Action::Land);
        a.velocity = Vec3::new(0.0, 0.0, 0.7);
        o.add(&a);
        let mut b = flying(1.1, up(0.05), Action::Land);
        b.velocity = Vec3::new(0.0, 0.0, 1.2);
        o.add(&b);
        assert_eq!(o.touchdown_speed_mps, None);
        let g = flying(1.2, Vec3::zero(), Action::Land);
        o.add(&g);
        assert_eq!(o.touchdown_speed_mps, Some(1.2));
        // Only the first contact counts.
        let mut c = flying(2.0, up(2.0), Action::Land);
        c.velocity = Vec3::new(0.0, 0.0, 9.0);
        o.add(&c);
        o.add(&flying(2.1, Vec3::zero(), Action::Land));
        assert_eq!(o.touchdown_speed_mps, Some(1.2));
        // A run that never left the ground has none.
        let mut o = FlightOutcome::new();
        o.add(&flying(0.0, Vec3::zero(), Action::None));
        o.add(&flying(0.1, up(0.1), Action::None));
        o.add(&flying(0.2, Vec3::zero(), Action::None));
        assert_eq!(o.touchdown_speed_mps, None);
    }

    #[test]
    fn disarm_events_and_motors() {
        let mut o = FlightOutcome::new();
        o.add(&flying(1.0, up(1.0), Action::None));
        let r = with_event(
            rec(
                2.0,
                Vec3::zero(),
                status(false, NavState::Disarmed, Action::None, Reasons::empty()),
            ),
            EventKind::Disarmed(DisarmReason::Touchdown),
        );
        o.add(&r);
        assert_eq!(
            o.core.as_ref().expect("core part").disarm_reason,
            Some(DisarmReason::Touchdown)
        );
        assert_eq!(o.disarmed_at_s, Some(2.0));
        assert!(!o.armed_at_end);
        assert_eq!(o.core.as_ref().expect("core part").events.len(), 1);
        assert_eq!(o.motor_ticks_while_disarmed, 0);
        let mut m = rec(
            2.1,
            Vec3::zero(),
            status(false, NavState::Disarmed, Action::None, Reasons::empty()),
        );
        m.motors = [0.0, 0.1, 0.0, 0.0];
        o.add(&m);
        assert_eq!(o.motor_ticks_while_disarmed, 1);
    }

    #[test]
    fn non_finite_and_fault_counters() {
        let mut o = FlightOutcome::new();
        let mut r = flying(0.0, up(1.0), Action::None);
        r.physics_faults = 2;
        r.core
            .as_mut()
            .expect("the record has a core part")
            .status
            .timing_faults = 3;
        o.add(&r);
        assert_eq!(
            (
                o.physics_faults,
                o.core.as_ref().expect("core part").timing_faults
            ),
            (2, 3)
        );
        assert!(!o.non_finite);
        let mut bad = flying(0.1, up(1.0), Action::None);
        bad.position.x = f64::NAN;
        o.add(&bad);
        assert!(o.non_finite);
    }

    // ---- Expect ----------------------------------------------------------

    /// A flight that hovers at 5 m, is held, returns and lands at home.
    fn landed_outcome() -> FlightOutcome {
        let mut o = FlightOutcome::new();
        let seq = [
            (1.0, NavState::Offboard, Action::None, Reasons::empty(), 5.0),
            (2.0, NavState::Hold, Action::Hold, Reasons::RC_LOST, 5.0),
            (3.0, NavState::Rtl, Action::Rtl, Reasons::RC_LOST, 5.0),
        ];
        for (t, nav, a, rs, h) in seq {
            o.add(&rec(t, up(h), status(true, nav, a, rs)));
        }
        let mut near = rec(
            4.0,
            up(0.05),
            status(true, NavState::Rtl, Action::Rtl, Reasons::RC_LOST),
        );
        near.velocity = Vec3::new(0.0, 0.0, 0.5);
        o.add(&near);
        o.add(&rec(
            4.1,
            Vec3::zero(),
            status(true, NavState::Rtl, Action::Rtl, Reasons::RC_LOST),
        ));
        o.add(&with_event(
            rec(
                4.2,
                Vec3::zero(),
                status(false, NavState::Disarmed, Action::None, Reasons::empty()),
            ),
            EventKind::Disarmed(DisarmReason::Touchdown),
        ));
        o
    }

    fn landed_expect() -> Expect {
        Expect {
            nav_contains: vec![NavState::Hold, NavState::Rtl],
            reasons: Reasons::RC_LOST,
            ending: Ending::Landed {
                within_m_of_home: Some(2.0),
            },
            by_s: 5.0,
            ..Expect::default()
        }
    }

    fn violations(e: &Expect, o: &FlightOutcome) -> String {
        e.check(o).violations.join(" | ")
    }

    #[test]
    fn a_good_landing_passes() {
        let o = landed_outcome();
        assert_eq!(landed_expect().check(&o).violations, Vec::<String>::new());
    }

    #[test]
    fn nav_rules() {
        let o = landed_outcome();
        let mut e = landed_expect();
        e.nav_contains = vec![NavState::Rtl, NavState::Hold];
        assert!(violations(&e, &o).contains("required order"));
        e.nav_contains = vec![NavState::Offboard, NavState::Rtl];
        assert!(e.check(&o).violations.is_empty());
        e.nav_contains = vec![NavState::Land];
        assert!(violations(&e, &o).contains("Land"));
        e.nav_contains = vec![];
        e.nav_never = vec![NavState::Hold];
        assert!(violations(&e, &o).contains("entered Hold"));
        e.nav_never = vec![NavState::Land, NavState::LandNow];
        assert!(e.check(&o).violations.is_empty());
    }

    #[test]
    fn reason_rules() {
        let o = landed_outcome();
        let mut e = landed_expect();
        e.reasons = Reasons::RC_LOST.union(Reasons::GPS_LOST);
        assert!(violations(&e, &o).contains("GPS_LOST"));
        e.reasons = Reasons::RC_LOST;
        e.reasons_never = Reasons::RC_LOST.union(Reasons::GEOFENCE);
        assert!(violations(&e, &o).contains("reason raised: RC_LOST"));
        e.reasons_never = Reasons::GEOFENCE;
        assert!(e.check(&o).violations.is_empty());
    }

    #[test]
    fn event_height_distance_and_action_rules() {
        let mut o = landed_outcome();
        let mut e = landed_expect();
        e.events = vec![EventTag::PilotOverride];
        assert!(violations(&e, &o).contains("PilotOverride"));
        o.core.as_mut().expect("core part").events.push(Event {
            t: Micros(1),
            kind: EventKind::PilotOverride(Reasons::RC_LOST),
        });
        assert!(e.check(&o).violations.is_empty());
        e.min_action = Action::Land;
        assert!(violations(&e, &o).contains("below the required Land"));
        e.min_action = Action::Rtl;
        e.max_height_m = Some(4.0);
        assert!(violations(&e, &o).contains("5.00 m high"));
        e.max_height_m = Some(5.0);
        e.max_distance_from_home_m = Some(-1.0);
        assert!(violations(&e, &o).contains("from home, limit"));
        e.max_distance_from_home_m = Some(0.0);
        assert!(e.check(&o).violations.is_empty());
    }

    #[test]
    fn tilt_limit_and_terminated_exemption() {
        let mut o = landed_outcome();
        let mut e = landed_expect();
        o.max_tilt_deg = 41.0;
        assert!(violations(&e, &o).contains("tilt 41.0"));
        e.max_tilt_deg = 45.0;
        assert!(e.check(&o).violations.is_empty());
        e.max_tilt_deg = 10.0;
        e.ending = Ending::Terminated;
        // Not checked for a terminated flight (its other rules still fail).
        assert!(!violations(&e, &o).contains("tilt"));
    }

    #[test]
    fn landed_rules() {
        let mut e = landed_expect();
        // Too hard.
        let mut o = landed_outcome();
        o.touchdown_speed_mps = Some(2.0);
        assert!(violations(&e, &o).contains("touchdown at 2.00"));
        // Too late.
        let mut o = landed_outcome();
        o.disarmed_at_s = Some(6.0);
        assert!(violations(&e, &o).contains("later than 5"));
        // Too far from home.
        let mut o = landed_outcome();
        o.position_at_end = Vec3::new(3.0, 0.0, 0.0);
        assert!(violations(&e, &o).contains("from home"));
        // Wrong reason.
        let mut o = landed_outcome();
        o.core.as_mut().expect("core part").disarm_reason = Some(DisarmReason::Switch);
        assert!(violations(&e, &o).contains("disarmed for Switch"));
        // Still armed.
        let mut o = landed_outcome();
        o.armed_at_end = true;
        assert!(violations(&e, &o).contains("still armed"));
        // Never disarmed.
        let mut o = landed_outcome();
        o.core.as_mut().expect("core part").disarm_reason = None;
        o.disarmed_at_s = None;
        assert!(violations(&e, &o).contains("never disarmed"));
        // Never came down although it went up.
        let mut o = landed_outcome();
        o.touchdown_speed_mps = None;
        assert!(violations(&e, &o).contains("never came down"));
        // Never left the ground: fine.
        o.max_height_m = 0.1;
        assert!(!violations(&e, &o).contains("never came down"));
        // No distance limit.
        e.ending = Ending::Landed {
            within_m_of_home: None,
        };
        let mut o = landed_outcome();
        o.position_at_end = Vec3::new(30.0, 0.0, 0.0);
        assert!(e.check(&o).violations.is_empty());
        // Disarmed in the air is not landed.
        o.position_at_end = Vec3::new(0.0, 0.0, -4.0);
        assert!(violations(&e, &o).contains("above the ground"));
    }

    #[test]
    fn flying_rules() {
        let mut o = FlightOutcome::new();
        o.add(&flying(20.0, up(5.0), Action::None));
        let e = Expect {
            ending: Ending::Flying {
                point: up(5.0),
                tolerance_m: 1.0,
            },
            by_s: 20.0,
            ..Expect::default()
        };
        assert!(
            e.check(&o).violations.is_empty(),
            "{:?}",
            e.check(&o).violations
        );
        // Off the point.
        let mut far = FlightOutcome::new();
        far.add(&flying(20.0, Vec3::new(2.0, 0.0, -5.0), Action::None));
        assert!(violations(&e, &far).contains("from the point"));
        // Run too short.
        let mut short = FlightOutcome::new();
        short.add(&flying(10.0, up(5.0), Action::None));
        assert!(violations(&e, &short).contains("before 20"));
        // Not armed, or terminated.
        let mut off = FlightOutcome::new();
        off.add(&rec(
            20.0,
            up(5.0),
            status(false, NavState::Disarmed, Action::None, Reasons::empty()),
        ));
        assert!(violations(&e, &off).contains("not armed"));
        let mut term = FlightOutcome::new();
        term.add(&rec(
            20.0,
            up(5.0),
            status(
                true,
                NavState::Terminated,
                Action::Terminate,
                Reasons::empty(),
            ),
        ));
        assert!(violations(&e, &term).contains("terminated"));
    }

    #[test]
    fn min_height_proves_the_flight_happened() {
        let e = Expect {
            min_height_m: Some(4.0),
            ending: Ending::Flying {
                point: Vec3::zero(),
                tolerance_m: 1.0,
            },
            ..Expect::default()
        };
        let mut grounded = FlightOutcome::new();
        grounded.add(&flying(1.0, up(0.1), Action::None));
        assert!(violations(&e, &grounded).contains("only reached"));
        let mut flew = FlightOutcome::new();
        flew.add(&flying(1.0, up(5.0), Action::None));
        flew.add(&flying(2.0, up(0.0), Action::None));
        assert!(!violations(&e, &flew).contains("only reached"));
    }

    #[test]
    fn flying_at_height_ignores_the_horizontal() {
        let mut o = FlightOutcome::new();
        o.add(&flying(30.0, Vec3::new(40.0, -20.0, -5.5), Action::None));
        let mut e = Expect {
            ending: Ending::FlyingAtHeight {
                height_m: 5.0,
                tolerance_m: 1.0,
            },
            by_s: 30.0,
            ..Expect::default()
        };
        assert!(e.check(&o).violations.is_empty());
        e.ending = Ending::FlyingAtHeight {
            height_m: 5.0,
            tolerance_m: 0.4,
        };
        assert!(violations(&e, &o).contains("off 5 m"));
    }

    #[test]
    fn terminated_and_disarmed_endings() {
        let mut o = FlightOutcome::new();
        o.add(&flying(15.0, up(5.0), Action::None));
        o.add(&rec(
            15.1,
            up(5.0),
            status(
                true,
                NavState::Terminated,
                Action::Terminate,
                Reasons::IMU_FAILURE,
            ),
        ));
        o.add(&with_event(
            rec(
                15.2,
                up(4.0),
                status(false, NavState::Disarmed, Action::None, Reasons::empty()),
            ),
            EventKind::Disarmed(DisarmReason::Terminated),
        ));
        // Falls to the ground.
        o.add(&rec(
            17.0,
            Vec3::zero(),
            status(false, NavState::Disarmed, Action::None, Reasons::empty()),
        ));
        let mut e = Expect {
            reasons: Reasons::IMU_FAILURE,
            ending: Ending::Terminated,
            by_s: 15.3,
            ..Expect::default()
        };
        assert!(
            e.check(&o).violations.is_empty(),
            "{:?}",
            e.check(&o).violations
        );
        e.by_s = 15.1;
        assert!(violations(&e, &o).contains("later than 15.1"));
        e.by_s = 15.3;
        e.ending = Ending::Disarmed(DisarmReason::Kill);
        assert!(violations(&e, &o).contains("disarmed for Terminated"));
        e.ending = Ending::Disarmed(DisarmReason::Terminated);
        assert!(e.check(&o).violations.is_empty());
        // Down accepts either reason.
        e.ending = Ending::Down;
        assert!(e.check(&o).violations.is_empty());
        let l = landed_outcome();
        let down = Expect {
            ending: Ending::Down,
            by_s: 5.0,
            ..Expect::default()
        };
        assert!(down.check(&l).violations.is_empty());
    }

    #[test]
    fn universal_rules_always_apply() {
        let mut o = landed_outcome();
        let e = landed_expect();
        o.core.as_mut().expect("core part").action_decreased = true;
        assert!(violations(&e, &o).contains("action decreased"));
        o.core.as_mut().expect("core part").action_decreased = false;
        o.non_finite = true;
        o.physics_faults = 1;
        o.core.as_mut().expect("core part").timing_faults = 1;
        o.motor_ticks_while_disarmed = 4;
        let v = violations(&e, &o);
        assert!(v.contains("non-finite value"));
        assert!(v.contains("physics"));
        assert!(v.contains("timing"));
        assert!(v.contains("while disarmed"));
    }

    #[test]
    fn reason_names_lists_bits() {
        assert_eq!(reason_names(Reasons::empty()), "none");
        assert_eq!(
            reason_names(Reasons::RC_LOST.union(Reasons::LOSS_OF_CONTROL)),
            "RC_LOST|LOSS_OF_CONTROL"
        );
    }

    // ---- Records without the core part (another autopilot) ----

    /// A record as another autopilot's tick gives it: truth and the armed
    /// flag, no core part.
    fn plain(t_s: f64, pos: Vec3<f64>, armed: bool) -> StepRecord<4> {
        let mut r = rec(
            t_s,
            pos,
            status(armed, NavState::Offboard, Action::None, Reasons::empty()),
        );
        r.core = None;
        r
    }

    /// Up, hold, come down, disarm (the flag falls at 4.2 s).
    fn plain_landed_outcome() -> FlightOutcome {
        let mut o = FlightOutcome::new();
        o.add(&plain(0.0, Vec3::zero(), false));
        for (t, h) in [(1.0, 5.0), (2.0, 5.0), (3.0, 5.0)] {
            o.add(&plain(t, up(h), true));
        }
        let mut near = plain(4.0, up(0.05), true);
        near.velocity = Vec3::new(0.0, 0.0, 0.5);
        o.add(&near);
        o.add(&plain(4.1, Vec3::zero(), true));
        o.add(&plain(4.2, Vec3::zero(), false));
        o
    }

    #[test]
    fn records_without_the_core_part_give_the_truth_part_only() {
        let o = plain_landed_outcome();
        assert!(o.core.is_none());
        assert!(!o.armed_at_end);
        assert_eq!(o.home, Some(up(5.0)));
        assert!((o.max_height_m - 5.0).abs() < 1e-12);
        assert_eq!(o.touchdown_speed_mps, Some(0.5));
        // The time the armed flag went from true to false.
        assert_eq!(o.disarmed_at_s, Some(4.2));
        assert!((o.end_s - 4.2).abs() < 1e-12);
    }

    #[test]
    fn with_the_core_part_the_disarm_time_is_the_events_not_the_flags() {
        // The supervisor disarms at 4.0 s (the event); the status shows it
        // one supervisor period later, at 4.02 s. Today's value is the
        // event's time and stays so for our core.
        let mut o = FlightOutcome::new();
        o.add(&flying(3.9, up(5.0), Action::None));
        o.add(&with_event(
            flying(4.0, up(5.0), Action::None),
            EventKind::Disarmed(DisarmReason::Kill),
        ));
        o.add(&rec(
            4.02,
            up(5.0),
            status(false, NavState::Disarmed, Action::None, Reasons::empty()),
        ));
        assert_eq!(o.disarmed_at_s, Some(4.0));
    }

    #[test]
    fn expectations_that_need_the_core_are_not_judged_without_it() {
        let o = plain_landed_outcome();
        let e = Expect {
            nav_contains: vec![NavState::Hold],
            nav_never: vec![NavState::Terminated],
            reasons: Reasons::RC_LOST,
            reasons_never: Reasons::GPS_LOST,
            events: vec![EventTag::Armed],
            min_action: Action::Rtl,
            ending: Ending::Landed {
                within_m_of_home: Some(2.0),
            },
            by_s: 5.0,
            ..Expect::default()
        };
        let j = e.check(&o);
        // Not violated: the truth part of all of it holds.
        assert!(j.violations.is_empty(), "{:?}", j.violations);
        // Not passed either: each is named, with the reason.
        let keys: Vec<&str> = j.not_judged.iter().map(|n| n.key).collect();
        for want in [
            "nav_contains",
            "nav_never",
            "reasons",
            "reasons_never",
            "events",
            "min_action",
        ] {
            assert!(keys.contains(&want), "{want} missing from {keys:?}");
        }
        assert!(keys.iter().any(|k| k.starts_with("ending landed")));
        assert!(keys.iter().any(|k| k.starts_with("invariant: no timing")));
        assert!(
            keys.iter()
                .any(|k| k.starts_with("invariant: failsafe action"))
        );
        assert!(j.not_judged.iter().all(|n| n.reason == NEEDS_CORE));
    }

    #[test]
    fn keys_a_file_does_not_state_are_not_listed_as_not_judged() {
        let o = plain_landed_outcome();
        let j = landed_expect_truth().check(&o);
        let keys: Vec<&str> = j.not_judged.iter().map(|n| n.key).collect();
        assert!(!keys.contains(&"nav_contains") && !keys.contains(&"min_action"));
    }

    fn landed_expect_truth() -> Expect {
        Expect {
            ending: Ending::Landed {
                within_m_of_home: Some(2.0),
            },
            by_s: 5.0,
            ..Expect::default()
        }
    }

    #[test]
    fn truth_expectations_are_judged_without_the_core() {
        let o = plain_landed_outcome();
        assert!(landed_expect_truth().check(&o).violations.is_empty());
        let mut e = landed_expect_truth();
        e.max_height_m = Some(3.0);
        e.by_s = 4.0;
        let v = e.check(&o).violations.join(" | ");
        assert!(v.contains("reached 5.00 m high, limit 3 m"), "{v}");
        assert!(v.contains("later than 4 s"), "{v}");
        // Truth invariants still apply.
        let mut bad = plain_landed_outcome();
        bad.motor_ticks_while_disarmed = 2;
        bad.physics_faults = 1;
        let v = landed_expect_truth().check(&bad).violations.join(" | ");
        assert!(v.contains("while disarmed") && v.contains("physics"), "{v}");
    }

    #[test]
    fn a_flight_with_the_core_part_has_nothing_unjudged() {
        let j = landed_expect().check(&landed_outcome());
        assert!(j.not_judged.is_empty());
        assert!(landed_outcome().invariants().not_judged.is_empty());
    }
}
