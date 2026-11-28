// SPDX-License-Identifier: AGPL-3.0-or-later
//! What is written after each tick, and when (ADR-0005 §3 and §7).

pub mod text;

use crate::header::{BuildInfo, Level};
use crate::logger::{Class, Logger, MessageWriter, Stats};
use crate::msg::{self, Error, ParamValue, Sink};
use crate::record::{self, RecordWriter, Topic};
use crate::replay;
use crate::topics::{self, Scales, id};
use oxide_core::codes::{px4_arming_state, px4_nav_state};
use oxide_core::config::{EstimatorSource, VehicleConfig};
use oxide_core::flight::Telemetry;
use oxide_core::io::{
    Action, BaroSample, BatterySample, Event, EventKind, ImuSample, Inputs, MagSample, NavState,
    Outputs, RcSample, Reasons, Stamped, Status,
};
use oxide_core::params::{self, LoadReport, ParamValue as CoreValue};
use oxide_core::schedule::Profile;
use oxide_core::time::Micros;
use text::{TextBuf, event_level, event_text};

/// Microseconds between records of a topic, by rate (ADR-0005 §3).
mod period {
    pub const HZ_100: u64 = 10_000;
    pub const HZ_50: u64 = 20_000;
    pub const HZ_20: u64 = 50_000;
    pub const HZ_10: u64 = 100_000;
    pub const HZ_5: u64 = 200_000;
    /// The longest a status topic and `oxide_logger` go without a record.
    pub const SECOND: u64 = 1_000_000;
}

/// When a streamed topic is next due, on the tick clock.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Clock {
    period: u64,
    pub(crate) next: Option<u64>,
}

impl Clock {
    pub(crate) const fn new(period: u64) -> Self {
        Self { period, next: None }
    }

    /// Due on the first look, then whenever `now` has reached the next time.
    /// Also due when the next time is more than a period ahead of `now`: the
    /// tick clock stepped back, and waiting for it to pass the old time
    /// would silence the topic ([`advance`](Self::advance) re-anchors).
    pub(crate) fn due(&self, now: u64) -> bool {
        match self.next {
            None => true,
            Some(n) => now >= n || n > now.saturating_add(self.period.max(1)),
        }
    }

    /// Sets the next time to the first on the period grid after `now`: a
    /// pause does not make the clock write a burst to catch up, and the
    /// phase of the grid is kept. The grid starts at the first record.
    ///
    /// A next time more than a period ahead of `now` (the clock stepped
    /// back) starts a new grid at `now`.
    ///
    /// Less than a period late is the common case and takes no division
    /// (a library call on the Cortex-M4F and M7).
    pub(crate) fn advance(&mut self, now: u64) {
        let period = self.period.max(1);
        self.next = Some(match self.next {
            None => now.saturating_add(period),
            Some(n) => {
                let on_grid = n.saturating_add(period);
                if n > now.saturating_add(period) {
                    now.saturating_add(period)
                } else if now < on_grid {
                    on_grid
                } else {
                    let late = (now - n) / period;
                    n.saturating_add(period.saturating_mul(late.saturating_add(1)))
                }
            }
        });
    }
}

/// A status-like topic: written when what it carries changes, and at least
/// once a second after its last record.
#[derive(Clone, Copy, Debug)]
struct OnChange<K> {
    last: Option<K>,
    next: u64,
}

impl<K: Copy + PartialEq> OnChange<K> {
    const fn new() -> Self {
        Self {
            last: None,
            next: 0,
        }
    }

    /// Also due when the next time is more than a second ahead of `now`: the
    /// tick clock stepped back.
    fn due(&self, key: K, now: u64) -> bool {
        self.last != Some(key) || now >= self.next || self.next > now.saturating_add(period::SECOND)
    }

    fn wrote(&mut self, key: K, now: u64) {
        self.last = Some(key);
        self.next = now.saturating_add(period::SECOND);
    }
}

/// What `vehicle_status` carries: PX4's arming state, nav state and the
/// pilot's intention, "a failsafe action is in force" and "no pre-arm check
/// fails".
pub(crate) fn vehicle_status_key(s: &Status) -> (u8, u8, u8, bool, bool) {
    (
        px4_arming_state(s.armed),
        px4_nav_state(s.nav, s.mode),
        px4_nav_state(NavState::Disarmed, s.mode),
        s.action != Action::None,
        s.prearm.is_empty(),
    )
}

/// What `failsafe_flags` carries, one bit per flag it writes.
pub(crate) fn failsafe_flags_key(s: &Status) -> u16 {
    let r = |b: Reasons| s.reasons.intersects(b);
    let battery = if r(Reasons::BATTERY_CRITICAL) {
        2
    } else {
        u16::from(r(Reasons::BATTERY_LOW))
    };
    u16::from(!s.valid.attitude)
        | u16::from(!s.valid.altitude) << 1
        | u16::from(!s.valid.position) << 2
        | u16::from(!s.valid.velocity) << 3
        | u16::from(r(Reasons::BRAIN_LOST) || r(Reasons::SETPOINT_STALE)) << 4
        | u16::from(!s.home_set) << 5
        | u16::from(r(Reasons::RC_LOST)) << 6
        | battery << 7
        | u16::from(r(Reasons::LOSS_OF_CONTROL)) << 9
        | u16::from(r(Reasons::GEOFENCE)) << 10
        | u16::from(r(Reasons::GPS_LOST)) << 11
}

/// What `oxide_status` carries.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct OxideStatusKey {
    timing_faults: u32,
    events_dropped: u32,
    reasons: u16,
    prearm: u16,
    codes: [u8; 5],
    flags: u8,
}

pub(crate) fn oxide_status_key(
    s: &Status,
    source: EstimatorSource,
    events_dropped: u32,
    landed: bool,
    ground_contact: bool,
) -> OxideStatusKey {
    OxideStatusKey {
        timing_faults: s.timing_faults,
        events_dropped,
        reasons: s.reasons.0,
        prearm: s.prearm.0,
        codes: [
            s.mode.code(),
            s.nav.code(),
            s.action.code(),
            source.code(),
            topics::validity_bits(&s.valid),
        ],
        flags: u8::from(s.armed)
            | u8::from(s.killed) << 1
            | u8::from(s.saturated) << 2
            | u8::from(s.no_estimate) << 3
            | u8::from(s.home_set) << 4
            | u8::from(s.params_unsaved) << 5
            | u8::from(landed) << 6
            | u8::from(ground_contact) << 7,
    }
}

/// True, and the clock moves on, when `clock` is due at `now`.
fn take(clock: &mut Clock, now: u64) -> bool {
    let due = clock.due(now);
    if due {
        clock.advance(now);
    }
    due
}

/// Message ids of the replay level's topics: they follow the normal level's
/// (a topic's id is its position in [`topic_list`]), in the order of
/// [`replay::TOPICS`].
pub mod replay_id {
    use crate::topics::NORMAL;

    /// The id of the first replay topic: the number of normal topics.
    pub const FIRST: u16 = NORMAL.len() as u16;
    /// `oxide_vehicle`.
    pub const VEHICLE: u16 = FIRST;
    /// `oxide_param_us`.
    pub const MICROS: u16 = FIRST + 1;
    /// `oxide_boot`.
    pub const BOOT: u16 = FIRST + 2;
    /// `oxide_in_tick`.
    pub const IN_TICK: u16 = FIRST + 3;
    /// `oxide_in_imu`.
    pub const IN_IMU: u16 = FIRST + 4;
    /// `oxide_in_state`.
    pub const IN_STATE: u16 = FIRST + 5;
    /// `oxide_in_rc`.
    pub const IN_RC: u16 = FIRST + 6;
    /// `oxide_in_battery`.
    pub const IN_BATTERY: u16 = FIRST + 7;
    /// `oxide_in_setpoint`.
    pub const IN_SETPOINT: u16 = FIRST + 8;
    /// `oxide_in_gps`.
    pub const IN_GPS: u16 = FIRST + 9;
    /// `oxide_in_baro`.
    pub const IN_BARO: u16 = FIRST + 10;
    /// `oxide_in_mag`.
    pub const IN_MAG: u16 = FIRST + 11;
    /// `oxide_in_gcs`.
    pub const IN_GCS: u16 = FIRST + 12;
    /// `oxide_in_brain`.
    pub const IN_BRAIN: u16 = FIRST + 13;
    /// `oxide_in_param_set`.
    pub const IN_PARAM_SET: u16 = FIRST + 14;
    /// `oxide_out`.
    pub const OUT: u16 = FIRST + 15;
}

const REPLAY_LEVEL_LEN: usize = topics::NORMAL.len() + replay::TOPICS.len();

/// [`topics::NORMAL`] followed by [`replay::TOPICS`].
#[allow(clippy::indexing_slicing)] // evaluated at compile time: a bad index fails the build
const fn replay_level_topics() -> [Topic; REPLAY_LEVEL_LEN] {
    let mut out = [replay::VEHICLE; REPLAY_LEVEL_LEN];
    let mut i = 0;
    while i < topics::NORMAL.len() {
        out[i] = topics::NORMAL[i];
        i += 1;
    }
    let mut j = 0;
    while j < replay::TOPICS.len() {
        out[topics::NORMAL.len() + j] = replay::TOPICS[j];
        j += 1;
    }
    out
}

const REPLAY_LEVEL: [Topic; REPLAY_LEVEL_LEN] = replay_level_topics();

/// The topics a log of this level declares, in message-id order: a topic's id
/// is its position. The replay level declares the normal topics, with the
/// ids they have at the normal level, and then [`replay::TOPICS`]
/// ([`replay_id`]).
pub fn topic_list(level: Level) -> &'static [Topic] {
    match level {
        Level::Normal => topics::NORMAL,
        Level::Replay => &REPLAY_LEVEL,
    }
}

/// What the core was booted with: the scheduling profile and the report of
/// the stored-parameter load it was given before its first tick. The normal
/// level uses only the profile's name; the replay level writes both.
#[derive(Debug, Clone, Copy)]
pub struct Boot<'a> {
    /// The scheduling profile the core was built with.
    pub profile: &'a Profile,
    /// The report given to `Core::report_param_load`.
    pub report: LoadReport,
}

impl<'a> Boot<'a> {
    /// A boot with a clean parameter load.
    pub fn clean(profile: &'a Profile) -> Self {
        Self {
            profile,
            report: LoadReport::default(),
        }
    }
}

/// Why a log could not be started.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StartError {
    /// The sink refused the definitions.
    Write(Error),
    /// A replay-level log of this configuration could never replay: the
    /// parameter does not survive being logged (`replay::check_exact`).
    NotReplayable(crate::replay::RebuildError),
    /// More rotors than an `oxide_out` record carries.
    TooManyRotors(crate::replay::TooManyRotors),
    /// The recorder has already started. Nothing was written and nothing
    /// changed: the first log goes on with its clocks, counters and ring.
    AlreadyStarted,
}

impl From<Error> for StartError {
    fn from(e: Error) -> Self {
        Self::Write(e)
    }
}

/// Records what the core did, tick by tick, into a [`Logger`] ring.
#[derive(Debug)]
pub struct Recorder<const CAP: usize> {
    logger: Logger<CAP>,
    level: Level,
    /// `start` has succeeded.
    started: bool,
    /// Calls of `record` since `start`: the next tick's sequence number.
    ticks: u64,
    /// The store's saved call happened since the last tick.
    saved: bool,
    scales: Option<Scales>,
    imu: Option<Stamped<ImuSample>>,
    imu_dt_us: u32,
    rc: Option<Stamped<RcSample>>,
    mag: Option<Stamped<MagSample>>,
    baro: Option<Stamped<BaroSample>>,
    battery: Option<Stamped<BatterySample>>,
    /// A sample of each input has arrived since its topic's last record.
    fresh: Fresh,
    clocks: Clocks,
    /// `seq` of the heading check's record last written to `oxide_estimator`:
    /// a newer one is written at once (every fix judged is logged), and the
    /// topic's clock moves on as for any record.
    fit_seq: u32,
    vehicle_status: OnChange<(u8, u8, u8, bool, bool)>,
    failsafe_flags: OnChange<u16>,
    oxide_status: OnChange<OxideStatusKey>,
}

/// Which inputs have delivered a sample since their topic's last record. A
/// topic made from an input sample is written when it is due and its input
/// is fresh, so a sensor that stops leaves a gap in its topic.
#[derive(Clone, Copy, Debug)]
struct Fresh {
    imu: bool,
    rc: bool,
    mag: bool,
    baro: bool,
    battery: bool,
}

impl Fresh {
    const fn new() -> Self {
        Self {
            imu: false,
            rc: false,
            mag: false,
            baro: false,
            battery: false,
        }
    }
}

/// One clock per streamed topic.
#[derive(Clone, Copy, Debug)]
struct Clocks {
    sensor_combined: Clock,
    angular_velocity: Clock,
    rates_setpoint: Clock,
    actuator_motors: Clock,
    thrust_setpoint: Clock,
    torque_setpoint: Clock,
    attitude: Clock,
    attitude_setpoint: Clock,
    local_position: Clock,
    local_position_setpoint: Clock,
    manual_control: Clock,
    magnetometer: Clock,
    air_data: Clock,
    battery: Clock,
    estimator: Clock,
    logger: Clock,
}

impl Clocks {
    const fn new() -> Self {
        Self {
            sensor_combined: Clock::new(period::HZ_100),
            angular_velocity: Clock::new(period::HZ_100),
            rates_setpoint: Clock::new(period::HZ_100),
            actuator_motors: Clock::new(period::HZ_100),
            thrust_setpoint: Clock::new(period::HZ_100),
            torque_setpoint: Clock::new(period::HZ_100),
            attitude: Clock::new(period::HZ_50),
            attitude_setpoint: Clock::new(period::HZ_50),
            local_position: Clock::new(period::HZ_20),
            local_position_setpoint: Clock::new(period::HZ_20),
            manual_control: Clock::new(period::HZ_10),
            magnetometer: Clock::new(period::HZ_10),
            air_data: Clock::new(period::HZ_10),
            battery: Clock::new(period::HZ_5),
            estimator: Clock::new(period::HZ_5),
            logger: Clock::new(period::SECOND),
        }
    }
}

impl<const CAP: usize> Default for Recorder<CAP> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const CAP: usize> Recorder<CAP> {
    /// A recorder that has not started; a quarter of the ring is kept for
    /// events, parameters and status.
    pub const fn new() -> Self {
        Self {
            logger: Logger::new(CAP / 4),
            level: Level::Normal,
            started: false,
            ticks: 0,
            saved: false,
            scales: None,
            imu: None,
            imu_dt_us: 0,
            rc: None,
            mag: None,
            baro: None,
            battery: None,
            fresh: Fresh::new(),
            clocks: Clocks::new(),
            fit_seq: 0,
            vehicle_status: OnChange::new(),
            failsafe_flags: OnChange::new(),
            oxide_status: OnChange::new(),
        }
    }

    /// Writes the definitions to `sink` and makes the recorder ready.
    ///
    /// A recorder starts once: a second call changes nothing, writes
    /// nothing and returns [`StartError::AlreadyStarted`].
    ///
    /// At the replay level it first checks that the log can be replayed
    /// ([`StartError::TooManyRotors`] for more than [`replay::MAX_MOTORS`]
    /// rotors, [`StartError::NotReplayable`] when `replay::check_exact`
    /// refuses a parameter, which it names) and, right after the
    /// subscriptions, writes the `oxide_vehicle` records, the `oxide_param_us`
    /// records and the `oxide_boot` record straight to `sink`.
    pub fn start<S: Sink + ?Sized, const N: usize>(
        &mut self,
        sink: &mut S,
        now: Micros,
        build: BuildInfo<'_>,
        boot: Boot<'_>,
        level: Level,
        cfg: &VehicleConfig<N>,
    ) -> Result<(), StartError> {
        if self.started {
            return Err(StartError::AlreadyStarted);
        }
        if level == Level::Replay {
            if N > replay::MAX_MOTORS {
                return Err(StartError::TooManyRotors(replay::TooManyRotors {
                    got: N,
                    max: replay::MAX_MOTORS,
                }));
            }
            replay::check_exact(cfg, boot.profile).map_err(StartError::NotReplayable)?;
        }
        let h = crate::header::Header {
            start_us: now.0,
            build,
            profile: boot.profile.name,
            level,
            nested: topics::NESTED,
        };
        crate::header::write_definitions(sink, &h, cfg, topic_list(level))?;
        if level == Level::Replay {
            // What the core was built from, once: the vehicle, the exact
            // microseconds of the duration parameters, the boot report.
            replay::write_vehicle(sink, replay_id::VEHICLE, now, cfg)?;
            replay::write_micros(sink, replay_id::MICROS, now, cfg)?;
            record::data(sink, replay_id::BOOT, &replay::BOOT, |w| {
                // N is at most 8 here: the rotor count always fits.
                let _ = replay::fill_boot(w, now, &boot.report, N, boot.profile);
            })?;
        }
        self.started = true;
        self.level = level;
        self.ticks = 0;
        self.saved = false;
        self.scales = Some(Scales::new(cfg));
        Ok(())
    }

    /// Tells the recorder that the store's "saved" call
    /// (`Core::clear_unsaved`) happened since the previous tick: the next
    /// tick record carries the flag.
    pub fn note_saved(&mut self) {
        self.saved = true;
    }

    /// Writes what is due after a tick: streamed topics at their rates, on
    /// the tick clock; a topic made from an input sample only when a sample
    /// has arrived since its last record (a sensor that stops leaves a gap,
    /// and its first sample after the gap is written at once); the status
    /// topics when they change and at least once a second; `oxide_logger`
    /// once a second. Nothing is written before [`start`](Self::start). A
    /// record that does not fit the ring is dropped and counted.
    pub fn record<const N: usize>(
        &mut self,
        now: Micros,
        inputs: &Inputs,
        outputs: &Outputs<N>,
        telemetry: &Telemetry,
        cfg: &VehicleConfig<N>,
    ) {
        let Some(scales) = self.scales else { return };
        let seq = self.ticks;
        self.ticks = seq.saturating_add(1);
        let saved = core::mem::take(&mut self.saved);
        self.observe(inputs);
        if self.level == Level::Replay {
            self.record_replay(now, seq, saved, inputs, outputs);
        }
        let t = now.0;
        let t_sample = self.t_sample(now);
        let status = &outputs.status;
        let tel = telemetry;
        let source = cfg.estimator.source;

        for e in outputs.events.iter() {
            self.event(now, e, cfg);
        }

        // Status: written when it changes. A record that did not fit is
        // tried again on the next tick.
        let key = vehicle_status_key(status);
        if self.vehicle_status.due(key, t)
            && self.must(t, id::VEHICLE_STATUS, &topics::VEHICLE_STATUS, |w| {
                topics::vehicle_status(w, now, status)
            })
        {
            self.vehicle_status.wrote(key, t);
        }
        let key = failsafe_flags_key(status);
        if self.failsafe_flags.due(key, t)
            && self.must(t, id::FAILSAFE_FLAGS, &topics::FAILSAFE_FLAGS, |w| {
                topics::failsafe_flags(w, now, status)
            })
        {
            self.failsafe_flags.wrote(key, t);
        }
        let key = oxide_status_key(
            status,
            source,
            tel.events_dropped,
            tel.landed,
            tel.ground_contact,
        );
        if self.oxide_status.due(key, t)
            && self.must(t, id::OXIDE_STATUS, &topics::OXIDE_STATUS, |w| {
                topics::oxide_status(
                    w,
                    now,
                    status,
                    source,
                    tel.events_dropped,
                    tel.landed,
                    tel.ground_contact,
                )
            })
        {
            self.oxide_status.wrote(key, t);
        }

        // Streams, in the order of their message ids.
        let dt = self.imu_dt_us;
        if let Some(imu) = self.imu
            && self.fresh.imu
            && take(&mut self.clocks.sensor_combined, t)
        {
            self.fresh.imu = false;
            self.stream(t, id::SENSOR_COMBINED, &topics::SENSOR_COMBINED, |w| {
                topics::sensor_combined(w, now, &imu, dt)
            });
        }
        if let Some(e) = tel.estimate
            && take(&mut self.clocks.angular_velocity, t)
        {
            self.stream(
                t,
                id::VEHICLE_ANGULAR_VELOCITY,
                &topics::VEHICLE_ANGULAR_VELOCITY,
                |w| topics::vehicle_angular_velocity(w, now, t_sample, e.rate),
            );
        }
        if take(&mut self.clocks.rates_setpoint, t) {
            self.stream(
                t,
                id::VEHICLE_RATES_SETPOINT,
                &topics::VEHICLE_RATES_SETPOINT,
                |w| {
                    topics::vehicle_rates_setpoint(
                        w,
                        now,
                        tel.rate_target,
                        tel.thrust_target_n,
                        &scales,
                    )
                },
            );
        }
        if take(&mut self.clocks.actuator_motors, t) {
            self.stream(t, id::ACTUATOR_MOTORS, &topics::ACTUATOR_MOTORS, |w| {
                topics::actuator_motors(w, now, t_sample, &outputs.motors)
            });
        }
        if take(&mut self.clocks.thrust_setpoint, t) {
            self.stream(
                t,
                id::VEHICLE_THRUST_SETPOINT,
                &topics::VEHICLE_THRUST_SETPOINT,
                |w| {
                    topics::vehicle_thrust_setpoint(
                        w,
                        now,
                        t_sample,
                        tel.rate_command.thrust_n,
                        &scales,
                    )
                },
            );
        }
        if take(&mut self.clocks.torque_setpoint, t) {
            self.stream(
                t,
                id::VEHICLE_TORQUE_SETPOINT,
                &topics::VEHICLE_TORQUE_SETPOINT,
                |w| {
                    topics::vehicle_torque_setpoint(
                        w,
                        now,
                        t_sample,
                        tel.rate_command.torque_nm,
                        &scales,
                    )
                },
            );
        }
        if let Some(e) = tel.estimate
            && take(&mut self.clocks.attitude, t)
        {
            self.stream(t, id::VEHICLE_ATTITUDE, &topics::VEHICLE_ATTITUDE, |w| {
                topics::vehicle_attitude(w, now, t_sample, e.attitude)
            });
        }
        if take(&mut self.clocks.attitude_setpoint, t) {
            self.stream(
                t,
                id::VEHICLE_ATTITUDE_SETPOINT,
                &topics::VEHICLE_ATTITUDE_SETPOINT,
                |w| {
                    topics::vehicle_attitude_setpoint(
                        w,
                        now,
                        tel.attitude_target,
                        tel.thrust_target_n,
                        &scales,
                    )
                },
            );
        }
        if let Some(e) = tel.estimate
            && take(&mut self.clocks.local_position, t)
        {
            let gps = status.estimator.gps_aiding;
            self.stream(
                t,
                id::VEHICLE_LOCAL_POSITION,
                &topics::VEHICLE_LOCAL_POSITION,
                |w| topics::vehicle_local_position(w, now, t_sample, &e, &tel.valid, gps),
            );
        }
        if take(&mut self.clocks.local_position_setpoint, t) {
            self.stream(
                t,
                id::VEHICLE_LOCAL_POSITION_SETPOINT,
                &topics::VEHICLE_LOCAL_POSITION_SETPOINT,
                |w| {
                    topics::vehicle_local_position_setpoint(
                        w,
                        now,
                        tel.setpoints.as_ref(),
                        tel.attitude_target,
                        tel.thrust_target_n,
                        &scales,
                    )
                },
            );
        }
        if let Some(rc) = self.rc
            && self.fresh.rc
            && take(&mut self.clocks.manual_control, t)
        {
            self.fresh.rc = false;
            self.stream(
                t,
                id::MANUAL_CONTROL_SETPOINT,
                &topics::MANUAL_CONTROL_SETPOINT,
                |w| topics::manual_control_setpoint(w, now, &rc),
            );
        }
        if let Some(mag) = self.mag
            && self.fresh.mag
            && take(&mut self.clocks.magnetometer, t)
        {
            self.fresh.mag = false;
            self.stream(
                t,
                id::VEHICLE_MAGNETOMETER,
                &topics::VEHICLE_MAGNETOMETER,
                |w| topics::vehicle_magnetometer(w, now, &mag),
            );
        }
        if let Some(baro) = self.baro
            && self.fresh.baro
            && take(&mut self.clocks.air_data, t)
        {
            self.fresh.baro = false;
            self.stream(t, id::VEHICLE_AIR_DATA, &topics::VEHICLE_AIR_DATA, |w| {
                topics::vehicle_air_data(w, now, &baro)
            });
        }
        if let Some(b) = self.battery
            && self.fresh.battery
            && take(&mut self.clocks.battery, t)
        {
            self.fresh.battery = false;
            self.stream(t, id::BATTERY_STATUS, &topics::BATTERY_STATUS, |w| {
                topics::battery_status(
                    w,
                    now,
                    &b.value,
                    tel.battery_cell_v,
                    tel.battery_low,
                    tel.battery_critical,
                    &scales,
                )
            });
        }
        // Due on its clock, or at once for a GPS fix the heading check has
        // judged since the last record: one record per fix. A record written
        // for a fix starts the clock again from that moment, so the topic
        // stays at its rate (5 Hz, one record per fix at the fix rate) and
        // the clock does not write a second record beside it.
        let fit_fresh = tel.heading_fit.seq != self.fit_seq;
        let due = if fit_fresh {
            self.clocks.estimator = Clock::new(period::HZ_5);
            self.clocks.estimator.advance(t);
            true
        } else {
            take(&mut self.clocks.estimator, t)
        };
        if due {
            self.fit_seq = tel.heading_fit.seq;
            self.stream(t, id::OXIDE_ESTIMATOR, &topics::OXIDE_ESTIMATOR, |w| {
                topics::oxide_estimator(
                    w,
                    now,
                    &status.estimator,
                    tel.estimate.is_some(),
                    tel.gyro_filtered,
                    &tel.heading_fit,
                )
            });
        }
        if take(&mut self.clocks.logger, t) {
            let stats = self.logger.stats();
            self.must(t, id::OXIDE_LOGGER, &topics::OXIDE_LOGGER, |w| {
                topics::oxide_logger(w, now, &stats)
            });
        }
    }

    /// The replay level's records of a tick, written before everything else
    /// of the tick, in this order: `oxide_in_tick`, one record per input
    /// present (in the order of [`replay::bit`]), `oxide_out`. All are
    /// `Stream`: a ring that is too full drops them and the next record that
    /// fits is preceded by a dropout.
    fn record_replay<const N: usize>(
        &mut self,
        now: Micros,
        seq: u64,
        saved: bool,
        inputs: &Inputs,
        outputs: &Outputs<N>,
    ) {
        let t = now.0;
        self.stream(t, replay_id::IN_TICK, &replay::IN_TICK, |w| {
            replay::fill_tick(w, now, seq, inputs, saved);
        });
        if let Some(s) = &inputs.imu {
            self.stream(t, replay_id::IN_IMU, &replay::IN_IMU, |w| {
                replay::fill_imu(w, now, s);
            });
        }
        if let Some(s) = &inputs.state {
            self.stream(t, replay_id::IN_STATE, &replay::IN_STATE, |w| {
                replay::fill_state(w, now, s);
            });
        }
        if let Some(s) = &inputs.rc {
            self.stream(t, replay_id::IN_RC, &replay::IN_RC, |w| {
                replay::fill_rc(w, now, s);
            });
        }
        if let Some(s) = &inputs.battery {
            self.stream(t, replay_id::IN_BATTERY, &replay::IN_BATTERY, |w| {
                replay::fill_battery(w, now, s);
            });
        }
        if let Some(s) = &inputs.setpoint {
            self.stream(t, replay_id::IN_SETPOINT, &replay::IN_SETPOINT, |w| {
                replay::fill_setpoint(w, now, s);
            });
        }
        if let Some(s) = &inputs.gps {
            self.stream(t, replay_id::IN_GPS, &replay::IN_GPS, |w| {
                replay::fill_gps(w, now, s);
            });
        }
        if let Some(s) = &inputs.baro {
            self.stream(t, replay_id::IN_BARO, &replay::IN_BARO, |w| {
                replay::fill_baro(w, now, s);
            });
        }
        if let Some(s) = &inputs.mag {
            self.stream(t, replay_id::IN_MAG, &replay::IN_MAG, |w| {
                replay::fill_mag(w, now, s);
            });
        }
        if let Some(s) = &inputs.gcs_heartbeat {
            self.stream(t, replay_id::IN_GCS, &replay::IN_GCS, |w| {
                replay::fill_gcs(w, now, s);
            });
        }
        if let Some(s) = &inputs.brain_heartbeat {
            self.stream(t, replay_id::IN_BRAIN, &replay::IN_BRAIN, |w| {
                replay::fill_brain(w, now, s);
            });
        }
        if let Some(s) = &inputs.param_set {
            self.stream(t, replay_id::IN_PARAM_SET, &replay::IN_PARAM_SET, |w| {
                replay::fill_param_set(w, now, s);
            });
        }
        self.stream(t, replay_id::OUT, &replay::OUT, |w| {
            // `start` refused more than `MAX_MOTORS` rotors.
            let _ = replay::fill_out(w, now, outputs);
        });
    }

    /// One event: as a logged string, then as an `oxide_event` record, both
    /// `Must`; a parameter change also writes the parameter's new value.
    fn event<const N: usize>(&mut self, now: Micros, e: &Event, cfg: &VehicleConfig<N>) {
        let t = now.0;
        let mut text = TextBuf::new();
        event_text(&e.kind, &mut text);
        let level = event_level(&e.kind);
        self.logger.write(Class::Must, t, |w| {
            msg::logged_string(w, level, e.t.0, text.as_str())
        });
        self.must(t, id::OXIDE_EVENT, &topics::OXIDE_EVENT, |w| {
            topics::oxide_event(w, now, e)
        });
        if let EventKind::ParamChanged(pid) = e.kind
            && let Some(def) = params::PARAMS.get(pid.index())
            && let Some(v) = params::get(cfg, pid)
        {
            let value = match v {
                CoreValue::F32(x) => ParamValue::F32(x),
                CoreValue::I32(x) => ParamValue::I32(x),
            };
            self.logger
                .write(Class::Must, t, |w| msg::parameter(w, def.name, value));
        }
    }

    /// Remembers the newest sample of each input that feeds a topic.
    fn observe(&mut self, inputs: &Inputs) {
        self.fresh.imu |= inputs.imu.is_some();
        self.fresh.rc |= inputs.rc.is_some();
        self.fresh.mag |= inputs.mag.is_some();
        self.fresh.baro |= inputs.baro.is_some();
        self.fresh.battery |= inputs.battery.is_some();
        if let Some(s) = inputs.imu {
            self.imu_dt_us = match self.imu {
                Some(prev) => u32::try_from(s.t_sample.since(prev.t_sample)).unwrap_or(u32::MAX),
                None => 0,
            };
            self.imu = Some(s);
        }
        if let Some(s) = inputs.rc {
            self.rc = Some(s);
        }
        if let Some(s) = inputs.mag {
            self.mag = Some(s);
        }
        if let Some(s) = inputs.baro {
            self.baro = Some(s);
        }
        if let Some(s) = inputs.battery {
            self.battery = Some(s);
        }
    }

    /// The time of the data the estimate and the commands are based on: the
    /// newest IMU sample's, or the tick's before one has been seen.
    fn t_sample(&self, now: Micros) -> Micros {
        self.imu.map_or(now, |s| s.t_sample)
    }

    fn stream(
        &mut self,
        now_us: u64,
        id: u16,
        topic: &Topic,
        fill: impl FnOnce(&mut RecordWriter<'_, MessageWriter<'_>>),
    ) -> bool {
        self.logger
            .write(Class::Stream, now_us, |w| record::data(w, id, topic, fill))
    }

    fn must(
        &mut self,
        now_us: u64,
        id: u16,
        topic: &Topic,
        fill: impl FnOnce(&mut RecordWriter<'_, MessageWriter<'_>>),
    ) -> bool {
        self.logger
            .write(Class::Must, now_us, |w| record::data(w, id, topic, fill))
    }

    /// Hands the oldest bytes of the ring to `sink`; returns how many it took.
    pub fn drain<S: Sink + ?Sized>(&mut self, sink: &mut S) -> usize {
        self.logger.drain(|chunk| match sink.put(chunk) {
            Ok(()) => chunk.len(),
            Err(_) => 0,
        })
    }

    /// Like [`drain`](Self::drain) for a sink that may take part of what it
    /// is offered: `sink` returns how many bytes of its argument it took.
    pub fn drain_with(&mut self, sink: impl FnMut(&[u8]) -> usize) -> usize {
        self.logger.drain(sink)
    }

    /// The logger's counters.
    pub fn stats(&self) -> Stats {
        self.logger.stats()
    }
}
