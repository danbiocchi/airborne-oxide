// SPDX-License-Identifier: AGPL-3.0-or-later
//! The chain simulator -> file -> reader -> pyulog (ADR-0005 §9): what a log
//! holds equals what the simulator knew at the same tick, for every one of
//! the twenty topics of the normal level and every field of each, and
//! `pyulog`, the strictest reader, reads the same bits as the crate's own
//! reader.
//!
//! The first half needs no Python. A flight is flown with a normal-level
//! log; at every tick that wrote something, what the simulator knew is kept:
//! the inputs, `Core::telemetry()`, the status, the motor commands, the
//! events. Every record of the file is then compared, field by field and bit
//! for bit, with a value recomputed here from those sources. The
//! conversions (normalisation of thrust and torque, gauss, the zero-centred
//! throttle, PX4's numbers for states, the "unknown" markers) are written out
//! again in this file from the ADR and the topics' documentation; the
//! encoders are not called. The expected record of a topic must name every
//! field its format has (a field nobody checks fails the test), either as a
//! value computed from the flight or as a constant.
//!
//! The pyulog tests are skipped, with a printed notice, unless the
//! environment variable `OXIDE_PYTHON` names a Python that has `pyulog`.

use std::collections::BTreeMap;
use std::process::Command;
use std::sync::{Arc, Mutex, PoisonError};

use oxide_core::config::VehicleConfig;
use oxide_core::control::{HorizontalForm, VerticalForm, YawForm};
use oxide_core::flight::Telemetry;
use oxide_core::io::{
    Action, BaroSample, BatterySample, DisarmReason, Event, EventKind, FlightMode, ImuSample,
    MagSample, NavState, ParamSet, PrearmFlags, RcSample, Reasons, Stamped, Status,
};
use oxide_core::math::Vec3;
use oxide_core::params::{self, LoadReport, ParamId, ParamValue};
use oxide_core::schedule::{F405_LEAN, H743_FULL, Profile};
use oxide_core::time::Micros;
use oxide_log::format::{FieldType, Layout, Scalar};
use oxide_log::reader::{Item, RecordView, UlogFile};
use oxide_log::topics::NORMAL;
use oxide_sim::Simulation;
use oxide_sim::fault_scenarios;
use oxide_sim::flight_log::{Level, LogBuild, SharedBuffer};
use oxide_sim::scenario::Script;

const BOTH: [Profile; 2] = [F405_LEAN, H743_FULL];

type Sim = Simulation<Script, 4>;

/// A writer into memory whose length can be read between ticks.
#[derive(Clone, Default)]
struct Grow(Arc<Mutex<Vec<u8>>>);

impl Grow {
    fn len(&self) -> usize {
        self.0.lock().unwrap_or_else(PoisonError::into_inner).len()
    }
    fn bytes(&self) -> Vec<u8> {
        self.0
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .clone()
    }
}

impl std::io::Write for Grow {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        self.0
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .extend_from_slice(buf);
        Ok(buf.len())
    }
    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}

/// The newest sample of each input that feeds a topic, up to a tick, and the
/// interval between the IMU's two newest samples.
#[derive(Clone, Copy, Default)]
struct Newest {
    imu: Option<Stamped<ImuSample>>,
    imu_dt_us: u32,
    rc: Option<Stamped<RcSample>>,
    mag: Option<Stamped<MagSample>>,
    baro: Option<Stamped<BaroSample>>,
    battery: Option<Stamped<BatterySample>>,
}

impl Newest {
    fn take(&mut self, i: &oxide_core::io::Inputs) {
        if let Some(s) = i.imu {
            self.imu_dt_us = self.imu.map_or(0, |p| {
                u32::try_from(s.t_sample.0.saturating_sub(p.t_sample.0)).unwrap_or(u32::MAX)
            });
            self.imu = Some(s);
        }
        self.rc = i.rc.or(self.rc);
        self.mag = i.mag.or(self.mag);
        self.baro = i.baro.or(self.baro);
        self.battery = i.battery.or(self.battery);
    }
}

/// What the simulator knew at one tick that wrote something.
struct Tick {
    newest: Newest,
    tel: Telemetry,
    status: Status,
    motors: [f32; 4],
    events: Vec<Event>,
}

/// Numbers of the vehicle the encoders scale by, worked out here from the
/// configuration.
struct Vehicle {
    /// The rotors' total maximum thrust, N.
    thrust_max_n: f32,
    /// The largest torque about each body axis, N·m.
    torque_max_nm: [f32; 3],
    cells: u8,
    nominal_v: f32,
    cell_resistance_ohm: f32,
    source_code: u8,
}

fn vehicle(cfg: &VehicleConfig<4>) -> Vehicle {
    let mut thrust = 0.0_f32;
    for r in &cfg.rotors {
        thrust += r.max_thrust_n;
    }
    // About one axis: every rotor whose torque coefficient is positive at its
    // maximum, or every rotor whose coefficient is negative at its maximum,
    // whichever is larger. Roll: -y, pitch: x, yaw: spin sign * coefficient.
    let axis = |c: fn(&oxide_core::config::Rotor) -> f32| {
        let (mut up, mut down) = (0.0_f32, 0.0_f32);
        for r in &cfg.rotors {
            let v = c(r) * r.max_thrust_n;
            if v > 0.0 {
                up += v;
            } else if v < 0.0 {
                down -= v;
            }
        }
        up.max(down)
    };
    let cells = cfg.battery_cells;
    Vehicle {
        thrust_max_n: thrust,
        torque_max_nm: [
            axis(|r| -r.y),
            axis(|r| r.x),
            axis(|r| r.spin.sign() * r.torque_coeff_m),
        ],
        cells,
        nominal_v: cfg.nominal_voltage_v,
        cell_resistance_ohm: cfg.battery_resistance_ohm / f32::from(cells),
        source_code: cfg.estimator.source.code(),
    }
}

struct Flight {
    ctx: String,
    bytes: Vec<u8>,
    ticks: BTreeMap<u64, Tick>,
    vehicle: Vehicle,
}

/// Something done to the simulation before a tick, for flights that need an
/// input the scenario does not give (a parameter change).
type Script2 = dyn Fn(&mut Sim, Micros) + Sync;

/// Flies `name` with a normal-level log; returns the file and what the
/// simulator knew at every tick that wrote something.
fn fly(name: &str, profile: Profile) -> Flight {
    fly_with(name, profile, &|_, _| {})
}

fn fly_with(name: &str, profile: Profile, before_tick: &Script2) -> Flight {
    let scenario = fault_scenarios::by_name(name).expect("scenario");
    let mut sim = scenario.build(profile, 0).expect("builds");
    let buf = Grow::default();
    sim.start_log(Level::Normal, &LogBuild::sim("hash"), Box::new(buf.clone()))
        .expect("log starts");
    let veh = vehicle(sim.core().config());
    let mut ticks = BTreeMap::new();
    let mut newest = Newest::default();
    let mut written = buf.len();
    while sim.time() < scenario.end() {
        let now = sim.time();
        before_tick(&mut sim, now);
        let r = sim.step();
        newest.take(sim.last_inputs());
        let now = buf.len();
        if now != written {
            written = now;
            ticks.insert(
                r.t.0,
                Tick {
                    newest,
                    tel: sim.core().telemetry(),
                    status: *r.status().expect("core part"),
                    motors: r.motors,
                    events: r
                        .core
                        .as_ref()
                        .expect("core part")
                        .events
                        .iter()
                        .copied()
                        .collect(),
                },
            );
        }
    }
    sim.finish_log().expect("finishes");
    Flight {
        ctx: format!("{name} on {}", profile.name),
        bytes: buf.bytes(),
        ticks,
        vehicle: veh,
    }
}

// ---- the expected record of a topic ----

/// A field value as little-endian bytes.
trait Le: Copy {
    const KIND: &'static str;
    fn le(self) -> ([u8; 8], usize);
}

macro_rules! le_int {
    ($($t:ty => $k:literal),*) => {$(
        impl Le for $t {
            const KIND: &'static str = $k;
            fn le(self) -> ([u8; 8], usize) {
                let mut b = [0u8; 8];
                let s = self.to_le_bytes();
                b[..s.len()].copy_from_slice(&s);
                (b, s.len())
            }
        }
    )*};
}
le_int!(u8 => "u8", u16 => "u16", u32 => "u32", u64 => "u64", i32 => "i32");

impl Le for f32 {
    const KIND: &'static str = "f32";
    fn le(self) -> ([u8; 8], usize) {
        let mut b = [0u8; 8];
        b[..4].copy_from_slice(&self.to_bits().to_le_bytes());
        (b, 4)
    }
}

impl Le for f64 {
    const KIND: &'static str = "f64";
    fn le(self) -> ([u8; 8], usize) {
        (self.to_bits().to_le_bytes(), 8)
    }
}

impl Le for bool {
    const KIND: &'static str = "bool";
    fn le(self) -> ([u8; 8], usize) {
        ([u8::from(self), 0, 0, 0, 0, 0, 0, 0], 1)
    }
}

/// How a field is compared.
#[derive(Clone, Copy, PartialEq, Eq)]
enum How {
    /// A value computed from the flight.
    Computed,
    /// A constant of the encoder: PX4's "unknown", a fixed code.
    Constant,
    /// Every byte of the field is zero (the "unknown" of an integer or flag
    /// field of any width).
    Zero,
}

struct Cell {
    name: String,
    want: [u8; 8],
    len: usize,
    kind: &'static str,
    how: How,
}

/// Which values a flag took, over the flights: `[false seen, true seen]`.
type Seen = BTreeMap<String, [bool; 2]>;

/// The values of a numeric field that were seen, for the fields that are
/// codes or counts.
type Values = BTreeMap<String, std::collections::BTreeSet<u64>>;

#[derive(Default)]
struct Coverage {
    flags: Seen,
    values: Values,
    /// Constants asserted, by topic: the names and their kind.
    constants: BTreeMap<&'static str, BTreeMap<String, &'static str>>,
}

struct Exp<'a> {
    topic: &'static str,
    cells: Vec<Cell>,
    cov: &'a mut Coverage,
}

impl Exp<'_> {
    fn put<T: Le>(&mut self, name: &str, x: T, how: How) {
        let (want, len) = x.le();
        self.cells.push(Cell {
            name: name.to_string(),
            want,
            len,
            kind: T::KIND,
            how,
        });
    }
    /// A value computed from the flight.
    fn v<T: Le>(&mut self, name: &str, x: T) {
        self.put(name, x, How::Computed);
    }
    /// An array of values computed from the flight.
    fn va<T: Le>(&mut self, name: &str, xs: &[T]) {
        for (i, x) in xs.iter().enumerate() {
            self.put(&format!("{name}[{i}]"), *x, How::Computed);
        }
    }
    /// A constant.
    fn k<T: Le>(&mut self, name: &str, x: T) {
        self.put(name, x, How::Constant);
    }
    /// An array of `n` copies of a constant.
    fn kn<T: Le>(&mut self, name: &str, n: usize, x: T) {
        for i in 0..n {
            self.put(&format!("{name}[{i}]"), x, How::Constant);
        }
    }
    /// Fields that are zero: whitespace separated names, `name*N` for an
    /// array of N.
    fn z(&mut self, names: &str) {
        for tok in names.split_whitespace() {
            let (name, n) = match tok.split_once('*') {
                Some((n, c)) => (n, Some(c.parse::<usize>().unwrap())),
                None => (tok, None),
            };
            let mut push = |name: String| {
                self.cells.push(Cell {
                    name,
                    want: [0; 8],
                    len: 0,
                    kind: "zero",
                    how: How::Zero,
                });
            };
            match n {
                None => push(name.to_string()),
                Some(n) => (0..n).for_each(|i| push(format!("{name}[{i}]"))),
            }
        }
    }
    /// A flag computed from the flight; it must be seen both ways over the
    /// flights.
    fn flag(&mut self, name: &str, on: bool) {
        self.v(name, on);
        let key = format!("{}.{name}", self.topic);
        self.cov.flags.entry(key).or_default()[usize::from(on)] = true;
    }
    /// Records that a condition was seen true or false, without a field.
    fn track(&mut self, name: &str, on: bool) {
        let key = format!("{}.{name}", self.topic);
        self.cov.flags.entry(key).or_default()[usize::from(on)] = true;
    }
    /// Records a value that was seen, without a field.
    fn track_value(&mut self, name: &str, x: u64) {
        let key = format!("{}.{name}", self.topic);
        self.cov.values.entry(key).or_default().insert(x);
    }
    /// Like `flag` for the NaN-ness of a float.
    fn nan_flag(&mut self, name: &str, x: f32) {
        self.v(name, x);
        let key = format!("{}.{name} is NaN", self.topic);
        self.cov.flags.entry(key).or_default()[usize::from(x.is_nan())] = true;
    }
    /// A bit field, bit by bit: every bit is a flag.
    fn bits(&mut self, name: &str, value: u16, nbits: u32, bytes: usize) {
        if bytes == 1 {
            self.v(name, u8::try_from(value).unwrap());
        } else {
            self.v(name, value);
        }
        for b in 0..nbits {
            let key = format!("{}.{name} bit {b}", self.topic);
            self.cov.flags.entry(key).or_default()[usize::from(value >> b & 1 == 1)] = true;
        }
    }
    /// A code or count: the values it took are recorded.
    fn code(&mut self, name: &str, x: u8) {
        self.v(name, x);
        let key = format!("{}.{name}", self.topic);
        self.cov.values.entry(key).or_default().insert(u64::from(x));
    }
}

fn hex(c: &Cell, bytes: &[u8]) -> String {
    let h: String = bytes.iter().rev().map(|b| format!("{b:02x}")).collect();
    match (c.kind, bytes.len()) {
        ("f32", 4) => {
            let v = f32::from_bits(u32::from_le_bytes(bytes.try_into().unwrap()));
            format!("{v:?} (0x{h})")
        }
        ("f64", 8) => {
            let v = f64::from_bits(u64::from_le_bytes(bytes.try_into().unwrap()));
            format!("{v:?} (0x{h})")
        }
        _ => format!("0x{h}"),
    }
}

/// The scalar and array-element names of a layout, in order, padding left out.
fn elements(layout: &Layout) -> Vec<String> {
    let mut out = Vec::new();
    for f in layout.fields.iter().filter(|f| !f.is_padding()) {
        match f.array {
            Some(n) => out.extend((0..n).map(|i| format!("{}[{i}]", f.name))),
            None => out.push(f.name.clone()),
        }
    }
    out
}

/// Compares the record with the expected cells, bit for bit. `first` also
/// requires that the cells name every field of the format exactly once.
fn compare(
    file: &UlogFile,
    r: &RecordView<'_>,
    topic: &str,
    cells: &[Cell],
    first: bool,
    ctx: &str,
    t: u64,
) {
    if first {
        let sub = file
            .subscriptions
            .iter()
            .find(|s| s.name == topic)
            .expect("subscribed");
        let mut have = elements(&file.formats[sub.format].layout);
        let mut named: Vec<String> = cells.iter().map(|c| c.name.clone()).collect();
        have.sort();
        named.sort();
        let dups = named.windows(2).any(|w| w[0] == w[1]);
        assert!(!dups, "{ctx} {topic}: a field is named twice in the test");
        assert_eq!(
            named, have,
            "{ctx} {topic}: the test names these fields, the format has those"
        );
    }
    for c in cells {
        let got = r
            .field_bytes(&c.name)
            .unwrap_or_else(|| panic!("{ctx} {topic}: no field {}", c.name));
        let ok = match c.how {
            How::Zero => got.iter().all(|b| *b == 0),
            _ => got == &c.want[..c.len],
        };
        if !ok {
            let want = if c.how == How::Zero {
                "all zero".to_string()
            } else {
                hex(c, &c.want[..c.len])
            };
            panic!(
                "{ctx}: {topic} at t={t} us, field {}: file {}, expected {want}",
                c.name,
                hex(c, got)
            );
        }
    }
}

/// The expected record of one topic at one tick.
type Expect = fn(u64, &Tick, &Vehicle, &mut Exp<'_>);

/// Checks every record of a topic against its expected record; returns how
/// many there are.
fn check_topic(
    fl: &Flight,
    file: &UlogFile,
    topic: &'static str,
    expect: Expect,
    cov: &mut Coverage,
) -> usize {
    let mut n = 0;
    for r in file.records(topic) {
        let t = r.u64("timestamp").expect("timestamp");
        let tick = fl
            .ticks
            .get(&t)
            .unwrap_or_else(|| panic!("{}: {topic} has a record at t={t}, no tick wrote", fl.ctx));
        let mut e = Exp {
            topic,
            cells: Vec::new(),
            cov,
        };
        expect(t, tick, &fl.vehicle, &mut e);
        let cells = e.cells;
        compare(file, &r, topic, &cells, n == 0, &fl.ctx, t);
        for c in cells.iter().filter(|_| n == 0) {
            if c.how != How::Computed {
                cov.constants
                    .entry(topic)
                    .or_default()
                    .insert(c.name.clone(), c.kind);
            }
        }
        n += 1;
    }
    n
}

// ---- what each topic must hold ----

const NAN: f32 = f32::NAN;

/// `-(v / scale)`, with a zero written as plus zero.
fn neg_ratio(v: f32, scale: f32) -> f32 {
    0.0 - v / scale
}

/// The time of the data the estimate and the commands are based on.
fn t_sample(t: u64, k: &Tick) -> u64 {
    k.newest.imu.map_or(t, |s| s.t_sample.0)
}

/// PX4's `nav_state` numbers (`NAVIGATION_STATE_*`) for the closest state.
fn px4_nav(nav: NavState, mode: FlightMode) -> u8 {
    match nav {
        NavState::Disarmed => match mode {
            FlightMode::Stabilized => 15,
            FlightMode::Altitude => 1,
            FlightMode::Position => 2,
            FlightMode::Offboard => 14,
        },
        NavState::Stabilized => 15,
        NavState::Altitude => 1,
        NavState::Position => 2,
        NavState::Offboard => 14,
        NavState::Hold => 4,
        NavState::Rtl => 5,
        NavState::Land => 18,
        NavState::LandNow => 12,
        NavState::Terminated => 13,
    }
}

fn sensor_combined(t: u64, k: &Tick, _: &Vehicle, e: &mut Exp<'_>) {
    let s = k.newest.imu.expect("an IMU sample");
    let rel = i64::try_from(s.t_sample.0).unwrap() - i64::try_from(t).unwrap();
    e.v("timestamp", t);
    e.va(
        "gyro_rad",
        &[s.value.gyro.x, s.value.gyro.y, s.value.gyro.z],
    );
    e.v("gyro_integral_dt", k.newest.imu_dt_us);
    e.v(
        "accelerometer_timestamp_relative",
        i32::try_from(rel).unwrap(),
    );
    e.va(
        "accelerometer_m_s2",
        &[s.value.accel.x, s.value.accel.y, s.value.accel.z],
    );
    e.v("accelerometer_integral_dt", k.newest.imu_dt_us);
    e.k("accelerometer_clipping", 0u8);
    e.k("gyro_clipping", 0u8);
    e.k("accel_calibration_count", 0u8);
    e.k("gyro_calibration_count", 0u8);
}

fn vehicle_angular_velocity(t: u64, k: &Tick, _: &Vehicle, e: &mut Exp<'_>) {
    let r = k.tel.estimate.expect("an estimate").rate;
    e.v("timestamp", t);
    e.v("timestamp_sample", t_sample(t, k));
    e.va("xyz", &[r.x, r.y, r.z]);
    e.kn("xyz_derivative", 3, 0.0_f32);
}

fn vehicle_rates_setpoint(t: u64, k: &Tick, v: &Vehicle, e: &mut Exp<'_>) {
    let r = k.tel.rate_target;
    e.v("timestamp", t);
    e.v("roll", r.x);
    e.v("pitch", r.y);
    e.v("yaw", r.z);
    e.k("thrust_body[0]", 0.0_f32);
    e.k("thrust_body[1]", 0.0_f32);
    e.v(
        "thrust_body[2]",
        neg_ratio(k.tel.thrust_target_n, v.thrust_max_n),
    );
    e.k("reset_integral", false);
}

fn actuator_motors(t: u64, k: &Tick, _: &Vehicle, e: &mut Exp<'_>) {
    e.v("timestamp", t);
    e.v("timestamp_sample", t_sample(t, k));
    e.va("control", &k.motors);
    // Channels beyond the four motors are NaN, the tools' end marker.
    for i in 4..12 {
        e.k(&format!("control[{i}]"), NAN);
    }
    e.k("reversible_flags", 0u16);
}

fn vehicle_thrust_setpoint(t: u64, k: &Tick, v: &Vehicle, e: &mut Exp<'_>) {
    e.v("timestamp", t);
    e.v("timestamp_sample", t_sample(t, k));
    e.k("xyz[0]", 0.0_f32);
    e.k("xyz[1]", 0.0_f32);
    e.v(
        "xyz[2]",
        neg_ratio(k.tel.rate_command.thrust_n, v.thrust_max_n),
    );
}

fn vehicle_torque_setpoint(t: u64, k: &Tick, v: &Vehicle, e: &mut Exp<'_>) {
    let q = k.tel.rate_command.torque_nm;
    e.v("timestamp", t);
    e.v("timestamp_sample", t_sample(t, k));
    e.va(
        "xyz",
        &[
            q.x / v.torque_max_nm[0],
            q.y / v.torque_max_nm[1],
            q.z / v.torque_max_nm[2],
        ],
    );
}

fn vehicle_attitude(t: u64, k: &Tick, _: &Vehicle, e: &mut Exp<'_>) {
    let q = k.tel.estimate.expect("an estimate").attitude;
    e.v("timestamp", t);
    e.v("timestamp_sample", t_sample(t, k));
    e.va("q", &[q.w, q.x, q.y, q.z]);
    e.kn("delta_q_reset", 4, 0.0_f32);
    e.k("quat_reset_counter", 0u8);
}

fn vehicle_attitude_setpoint(t: u64, k: &Tick, v: &Vehicle, e: &mut Exp<'_>) {
    let q = k.tel.attitude_target;
    e.v("timestamp", t);
    e.k("yaw_sp_move_rate", 0.0_f32);
    e.va("q_d", &[q.w, q.x, q.y, q.z]);
    e.k("thrust_body[0]", 0.0_f32);
    e.k("thrust_body[1]", 0.0_f32);
    e.v(
        "thrust_body[2]",
        neg_ratio(k.tel.thrust_target_n, v.thrust_max_n),
    );
}

fn vehicle_local_position(t: u64, k: &Tick, _: &Vehicle, e: &mut Exp<'_>) {
    let est = k.tel.estimate.expect("an estimate");
    let valid = k.tel.valid;
    e.v("timestamp", t);
    e.v("timestamp_sample", t_sample(t, k));
    e.v("x", est.position.x);
    e.v("y", est.position.y);
    e.v("z", est.position.z);
    e.v("vx", est.velocity.x);
    e.v("vy", est.velocity.y);
    e.v("vz", est.velocity.z);
    e.v("z_deriv", est.velocity.z);
    e.v("heading", est.attitude.yaw());
    e.flag("xy_valid", valid.position);
    e.flag("z_valid", valid.altitude);
    e.flag("v_xy_valid", valid.velocity);
    e.flag("v_z_valid", valid.altitude || valid.velocity);
    e.flag("heading_good_for_control", valid.heading);
    e.flag("dead_reckoning", !k.status.estimator.gps_aiding);
    e.z(
        "ref_timestamp ref_lat ref_lon delta_xy*2 delta_z delta_vxy*2 delta_vz ax ay az \
         heading_var unaided_heading delta_heading tilt_var ref_alt dist_bottom \
         dist_bottom_var delta_dist_bottom eph epv evh evv vxy_max vz_max hagl_min \
         hagl_max_z hagl_max_xy xy_reset_counter z_reset_counter vxy_reset_counter \
         vz_reset_counter heading_reset_counter xy_global z_global dist_bottom_valid \
         dist_bottom_reset_counter dist_bottom_sensor_bitfield",
    );
}

fn vehicle_local_position_setpoint(t: u64, k: &Tick, v: &Vehicle, e: &mut Exp<'_>) {
    e.v("timestamp", t);
    let (pos, vel, yaw, thrust) = match &k.tel.setpoints {
        None => ([NAN; 3], [NAN; 3], NAN, [NAN; 3]),
        Some(s) => {
            let h_pos = s.horizontal == HorizontalForm::Position;
            let v_pos = s.vertical == VerticalForm::Position;
            let pos = [
                if h_pos { s.position.x } else { NAN },
                if h_pos { s.position.y } else { NAN },
                if v_pos { s.position.z } else { NAN },
            ];
            let vel = match s.velocity {
                None => [NAN; 3],
                Some(w) => {
                    let h = s.horizontal != HorizontalForm::Tilt;
                    let z = s.vertical != VerticalForm::Thrust;
                    [
                        if h { w.x } else { NAN },
                        if h { w.y } else { NAN },
                        if z { w.z } else { NAN },
                    ]
                }
            };
            let yaw = if s.yaw_form == YawForm::Heading {
                s.yaw
            } else {
                NAN
            };
            // The thrust target as a world vector: the target attitude
            // rotating (0, 0, -thrust / maximum).
            let th = k.tel.attitude_target.rotate(Vec3::new(
                0.0,
                0.0,
                neg_ratio(k.tel.thrust_target_n, v.thrust_max_n),
            ));
            (pos, vel, yaw, [th.x, th.y, th.z])
        }
    };
    for (name, x) in [("x", pos[0]), ("y", pos[1]), ("z", pos[2])] {
        e.nan_flag(name, x);
    }
    for (name, x) in [("vx", vel[0]), ("vy", vel[1]), ("vz", vel[2])] {
        e.nan_flag(name, x);
    }
    e.kn("acceleration", 3, NAN);
    e.va("thrust", &thrust);
    e.nan_flag("yaw", yaw);
    e.k("yawspeed", NAN);
}

fn manual_control_setpoint(t: u64, k: &Tick, _: &Vehicle, e: &mut Exp<'_>) {
    let rc = k.newest.rc.expect("a radio sample");
    let s = rc.value;
    e.v("timestamp", t);
    e.v("timestamp_sample", rc.t_sample.0);
    e.v("roll", s.roll);
    e.v("pitch", s.pitch);
    e.v("yaw", s.yaw);
    e.v("throttle", 2.0 * s.throttle - 1.0);
    e.v(
        "aux1",
        match s.mode_switch {
            0 => -1.0_f32,
            1 => 0.0,
            _ => 1.0,
        },
    );
    e.v("aux2", if s.arm_switch { 1.0_f32 } else { -1.0 });
    e.v("aux3", if s.kill_switch { 1.0_f32 } else { -1.0 });
    e.flag("valid", !s.failsafe && !s.frame_lost);
    e.track("arm switch", s.arm_switch);
    e.track("kill switch", s.kill_switch);
    e.track_value("mode switch", u64::from(s.mode_switch));
    e.k("flaps", NAN);
    e.k("aux4", NAN);
    e.k("aux5", NAN);
    e.k("aux6", NAN);
    e.k("data_source", 1u8);
    e.z("source_system_id buttons source_component_id sticks_moving");
}

fn vehicle_magnetometer(t: u64, k: &Tick, _: &Vehicle, e: &mut Exp<'_>) {
    let m = k.newest.mag.expect("a magnetometer sample");
    let f = m.value.field_ut;
    e.v("timestamp", t);
    e.v("timestamp_sample", m.t_sample.0);
    e.va("magnetometer_ga", &[f.x / 100.0, f.y / 100.0, f.z / 100.0]);
    e.z("device_id calibration_count");
}

fn vehicle_air_data(t: u64, k: &Tick, _: &Vehicle, e: &mut Exp<'_>) {
    let b = k.newest.baro.expect("a barometer sample");
    e.v("timestamp", t);
    e.v("timestamp_sample", b.t_sample.0);
    e.v("baro_alt_meter", b.value.altitude_m);
    e.z(
        "baro_device_id baro_pressure_pa ambient_temperature rho temperature_source \
         calibration_count",
    );
}

fn battery_status(t: u64, k: &Tick, v: &Vehicle, e: &mut Exp<'_>) {
    let b = k.newest.battery.expect("a battery sample").value;
    e.v("timestamp", t);
    e.v("voltage_v", b.voltage_v);
    e.v("current_a", b.current_a);
    for name in ["current_average_a", "discharged_mah", "remaining", "scale"] {
        e.k(name, -1.0_f32);
    }
    e.k("time_remaining_s", NAN);
    e.k("temperature", NAN);
    // The filtered per-cell voltage, once for each cell.
    let cell = k.tel.battery_cell_v;
    for i in 0..14 {
        let x = match cell {
            Some(c) if i < usize::from(v.cells) => c,
            _ => 0.0,
        };
        e.v(&format!("voltage_cell_v[{i}]"), x);
    }
    e.k("remaining_capacity_wh", NAN);
    e.v("nominal_voltage", v.nominal_v);
    e.v("internal_resistance_estimate", v.cell_resistance_ohm);
    e.k("connected", true);
    e.v("cell_count", v.cells);
    e.k("id", 1u8);
    e.track("low latch", k.tel.battery_low);
    e.track("critical latch", k.tel.battery_critical);
    let warning = if k.tel.battery_critical {
        2u8
    } else {
        u8::from(k.tel.battery_low)
    };
    e.code("warning", warning);
    e.z(
        "max_cell_voltage_delta full_charge_capacity_wh ocv_estimate ocv_estimate_filtered \
         volt_based_soc_estimate voltage_prediction prediction_error \
         estimation_covariance_norm capacity cycle_count average_time_to_empty \
         manufacture_date state_of_health max_error interface_error faults \
         over_discharge_count source priority is_powering_off is_required",
    );
}

fn vehicle_status(t: u64, k: &Tick, _: &Vehicle, e: &mut Exp<'_>) {
    let s = &k.status;
    let nav = px4_nav(s.nav, s.mode);
    e.v("timestamp", t);
    e.code("arming_state", if s.armed { 2 } else { 1 });
    e.code("nav_state", nav);
    e.code("nav_state_display", nav);
    e.code(
        "nav_state_user_intention",
        px4_nav(NavState::Disarmed, s.mode),
    );
    e.flag("failsafe", s.action != Action::None);
    e.flag("pre_flight_checks_pass", s.prearm.is_empty());
    e.k("vehicle_type", 1u8);
    e.k("system_type", 2u8);
    e.z(
        "armed_time takeoff_time nav_state_timestamp valid_nav_states_mask \
         can_set_nav_states_mask system_id latest_arming_reason latest_disarming_reason \
         executor_in_charge accepts_offboard_setpoints hil_state \
         failsafe_and_user_took_over failsafe_defer_state gcs_connection_lost \
         gcs_connection_lost_counter high_latency_data_link_lost is_vtol \
         is_vtol_tailsitter in_transition_mode in_transition_to_fw component_id \
         safety_button_available safety_off power_input_valid usb_connected \
         open_drone_id_system_present open_drone_id_system_healthy \
         parachute_system_present parachute_system_healthy \
         traffic_avoidance_system_present rc_calibration_in_progress calibration_enabled",
    );
}

fn failsafe_flags(t: u64, k: &Tick, _: &Vehicle, e: &mut Exp<'_>) {
    let s = &k.status;
    let r = |bit: Reasons| s.reasons.contains(bit);
    e.v("timestamp", t);
    e.flag("angular_velocity_invalid", !s.valid.attitude);
    e.flag("attitude_invalid", !s.valid.attitude);
    e.flag("local_altitude_invalid", !s.valid.altitude);
    e.flag("local_position_invalid", !s.valid.position);
    e.flag("local_position_invalid_relaxed", !s.valid.position);
    e.flag("local_velocity_invalid", !s.valid.velocity);
    e.flag(
        "offboard_control_signal_lost",
        r(Reasons::BRAIN_LOST) || r(Reasons::SETPOINT_STALE),
    );
    e.flag("home_position_invalid", !s.home_set);
    e.flag("manual_control_signal_lost", r(Reasons::RC_LOST));
    e.flag("fd_critical_failure", r(Reasons::LOSS_OF_CONTROL));
    e.flag("geofence_breached", r(Reasons::GEOFENCE));
    e.flag("gnss_lost", r(Reasons::GPS_LOST));
    let warning = if r(Reasons::BATTERY_CRITICAL) {
        2u8
    } else {
        u8::from(r(Reasons::BATTERY_LOW))
    };
    e.code("battery_warning", warning);
    e.z(
        "mode_req_angular_velocity mode_req_attitude mode_req_local_alt \
         mode_req_local_position mode_req_local_position_relaxed \
         mode_req_global_position mode_req_global_position_relaxed mode_req_mission \
         mode_req_offboard_signal mode_req_home_position \
         mode_req_wind_and_flight_time_compliance mode_req_prevent_arming \
         mode_req_manual_control mode_req_other mode_not_supported \
         global_position_invalid global_position_invalid_relaxed auto_mission_missing \
         gcs_connection_lost battery_low_remaining_time battery_unhealthy \
         fd_esc_arming_failure fd_imbalanced_prop fd_motor_failure fd_motor_stuck_failure \
         fd_alt_loss mission_failure vtol_fixed_wing_system_failure wind_limit_exceeded \
         flight_time_limit_exceeded position_accuracy_low navigator_failure \
         parachute_unhealthy traffic_avoidance_unhealthy remote_id_unhealthy",
    );
}

/// The bit numbers of the failsafe reasons (ADR-0005 §3, `oxide_status`).
const REASON_BITS: [(Reasons, u32); 11] = [
    (Reasons::RC_LOST, 0),
    (Reasons::NO_HUMAN_LINK, 1),
    (Reasons::BRAIN_LOST, 2),
    (Reasons::SETPOINT_STALE, 3),
    (Reasons::BATTERY_LOW, 4),
    (Reasons::BATTERY_CRITICAL, 5),
    (Reasons::GPS_LOST, 6),
    (Reasons::ESTIMATOR_UNHEALTHY, 7),
    (Reasons::IMU_FAILURE, 8),
    (Reasons::GEOFENCE, 9),
    (Reasons::LOSS_OF_CONTROL, 10),
];

/// The bit numbers of the failing pre-arm checks.
const PREARM_BITS: [(PrearmFlags, u32); 9] = [
    (PrearmFlags::KILL, 0),
    (PrearmFlags::THROTTLE, 1),
    (PrearmFlags::RC, 2),
    (PrearmFlags::NOT_STILL, 3),
    (PrearmFlags::ESTIMATOR, 4),
    (PrearmFlags::POSITION, 5),
    (PrearmFlags::BATTERY, 6),
    (PrearmFlags::CONFIG, 7),
    (PrearmFlags::LINK, 8),
];

fn oxide_status(t: u64, k: &Tick, v: &Vehicle, e: &mut Exp<'_>) {
    let s = &k.status;
    let mut reasons = 0u16;
    for (flag, bit) in REASON_BITS {
        if s.reasons.contains(flag) {
            reasons |= 1 << bit;
        }
    }
    let mut prearm = 0u16;
    for (flag, bit) in PREARM_BITS {
        if s.prearm.contains(flag) {
            prearm |= 1 << bit;
        }
    }
    let valid = u8::from(s.valid.attitude)
        | u8::from(s.valid.heading) << 1
        | u8::from(s.valid.altitude) << 2
        | u8::from(s.valid.velocity) << 3
        | u8::from(s.valid.position) << 4;
    e.v("timestamp", t);
    e.v("timing_faults", s.timing_faults);
    e.v("events_dropped", k.tel.events_dropped);
    e.bits("reasons", reasons, 11, 2);
    e.bits("prearm", prearm, 9, 2);
    e.code("mode", s.mode.code());
    e.code("nav", s.nav.code());
    e.code("action", s.action.code());
    e.v("estimator_source", v.source_code);
    e.bits("valid", u16::from(valid), 5, 1);
    e.flag("armed", s.armed);
    e.flag("killed", s.killed);
    e.flag("saturated", s.saturated);
    e.flag("no_estimate", s.no_estimate);
    e.flag("home_set", s.home_set);
    e.flag("params_unsaved", s.params_unsaved);
    e.flag("landed", k.tel.landed);
    e.flag("ground_contact", k.tel.ground_contact);
}

fn oxide_estimator(t: u64, k: &Tick, _: &Vehicle, e: &mut Exp<'_>) {
    let est = &k.status.estimator;
    let g = k.tel.gyro_filtered;
    e.v("timestamp", t);
    e.v("rejected", est.rejected);
    e.v("resets", est.resets);
    e.va("gyro_filtered", &[g.x, g.y, g.z]);
    e.flag("aligned", est.aligned);
    e.flag("gps_aiding", est.gps_aiding);
    e.flag("has_estimate", k.tel.estimate.is_some());
    // The heading check's record for the latest fix it judged.
    let f = &k.tel.heading_fit;
    e.v("fit_verdict", f.verdict);
    e.v("fit_seq", f.seq);
    e.v("fit_eps", f.eps);
    e.v("fit_sigma", f.sigma);
    e.v("fit_sa", f.sa);
    e.v("fit_kappa", f.kappa);
    e.v("fit_rho", f.rho);
    e.v("fit_stat", f.stat);
    e.v("fit_vel_ratio", f.vel_ratio);
    e.v("fit_t", f.t_us);
    e.v("fit_streak", f.streak);
    e.flag("fit_used", f.used);
    // The heading guard's state.
    let h = &k.tel.heading_guard;
    e.v("hdg_corrections", h.corrections);
    e.flag("hdg_contained", h.contained);
    e.v("hdg_shift", h.shift_rad);
    e.v("tilt_roll", h.tilt_roll_rad);
    e.v("tilt_pitch", h.tilt_pitch_rad);
    e.v("hdg_fine", h.fine_steps);
}

/// `oxide_event`: every event of every tick, in order, with the fixed codes
/// of ADR-0005 §3 written out here.
fn check_events(fl: &Flight, file: &UlogFile, cov: &mut Coverage) -> usize {
    let mut seen_at: BTreeMap<u64, usize> = BTreeMap::new();
    let mut n = 0;
    for r in file.records("oxide_event") {
        let t = r.u64("timestamp").expect("timestamp");
        let tick = fl
            .ticks
            .get(&t)
            .unwrap_or_else(|| panic!("{}: oxide_event at t={t}, no tick wrote", fl.ctx));
        let idx = seen_at.entry(t).or_insert(0);
        let ev = *tick
            .events
            .get(*idx)
            .unwrap_or_else(|| panic!("{}: more oxide_event records than events at t={t}", fl.ctx));
        *idx += 1;
        // (kind, code_a, code_b, mask, param, why, why_detail, report)
        let kind: u8;
        let (mut a, mut b, mut mask) = (0u8, 0u8, 0u16);
        let (mut key, mut index) = (0u32, 0u16);
        let (mut why, mut detail) = (0u8, 0u8);
        let mut rep = LoadReport::default();
        let param = |id: ParamId| (params::key_of(id).unwrap_or(0), id.0);
        match ev.kind {
            EventKind::Armed => kind = 0,
            EventKind::Disarmed(d) => {
                kind = 1;
                a = match d {
                    DisarmReason::Switch => 0,
                    DisarmReason::Kill => 1,
                    DisarmReason::Touchdown => 2,
                    DisarmReason::Terminated => 3,
                };
            }
            EventKind::ArmRefused(p) => {
                kind = 2;
                mask = p.0;
            }
            EventKind::NavChanged { from, to } => {
                kind = 3;
                (a, b) = (from.code(), to.code());
            }
            EventKind::ModeDegraded { requested, flown } => {
                kind = 4;
                (a, b) = (requested.code(), flown.code());
            }
            EventKind::ReasonRaised(x) => {
                kind = 5;
                mask = x.0;
            }
            EventKind::ReasonCleared(x) => {
                kind = 6;
                mask = x.0;
            }
            EventKind::ActionChanged { from, to } => {
                kind = 7;
                (a, b) = (from.code(), to.code());
            }
            EventKind::PilotOverride(x) => {
                kind = 8;
                mask = x.0;
            }
            EventKind::Touchdown => kind = 9,
            EventKind::HomeSet => kind = 10,
            EventKind::ParamChanged(id) => {
                kind = 11;
                (key, index) = param(id);
            }
            EventKind::ParamRefused { id, why: w } => {
                kind = 12;
                (key, index) = param(id);
                (why, detail) = (w.code(), w.detail_code());
            }
            EventKind::ParamLoadFault(x) => {
                kind = 13;
                rep = x;
            }
            EventKind::ParamLoadRecovered(x) => {
                kind = 14;
                rep = x;
            }
            EventKind::ParamFaultAcked => kind = 15,
            EventKind::HeadingCorrected { angle_mrad, count } => {
                kind = 16;
                mask = u16::from_ne_bytes(angle_mrad.to_ne_bytes());
                a = count;
            }
            EventKind::HeadingContained => kind = 17,
            EventKind::GpsSnapRepeated => kind = 18,
            EventKind::HeadingRefined { angle_mrad } => {
                kind = 19;
                mask = u16::from_ne_bytes(angle_mrad.to_ne_bytes());
            }
        }
        let mut e = Exp {
            topic: "oxide_event",
            cells: Vec::new(),
            cov,
        };
        e.v("timestamp", t);
        e.v("event_time", ev.t.0);
        e.v("param_key", key);
        e.v("param_index", index);
        e.v("mask", mask);
        e.v("applied", rep.applied);
        e.v("unknown", rep.unknown);
        e.v("dropped", rep.dropped);
        e.v("torn", rep.torn);
        e.code("kind", kind);
        e.v("code_a", a);
        e.v("code_b", b);
        e.v("why", why);
        e.v("why_detail", detail);
        e.v("invalid", rep.invalid);
        e.v("unreadable", rep.unreadable);
        let cells = e.cells;
        compare(file, &r, "oxide_event", &cells, n == 0, &fl.ctx, t);
        n += 1;
    }
    let events: usize = fl.ticks.values().map(|t| t.events.len()).sum();
    assert_eq!(n, events, "{}: an oxide_event record per event", fl.ctx);
    n
}

/// `oxide_logger`: the logger's counters at that moment, recomputed from the
/// file itself. The recorder reads them just before it writes the record.
fn check_logger(fl: &Flight, file: &UlogFile) -> usize {
    // The ring begins after the definitions: the end of the last subscription
    // message of the definitions section.
    let defs_end = file
        .subscriptions
        .iter()
        .take_while(|s| {
            s.offset
                < file
                    .items
                    .iter()
                    .find_map(|i| match i {
                        Item::Data(d) => Some(d.offset),
                        _ => None,
                    })
                    .unwrap()
        })
        .map(|s| s.offset + 3 + 3 + s.name.len())
        .max()
        .unwrap();
    let (mut written, mut syncs, mut dropouts) = (0u32, 0u32, 0u32);
    let mut high = 0u32;
    let mut n = 0;
    let mut prev_sync = false;
    for item in &file.items {
        match item {
            Item::Sync => {
                syncs += 1;
                prev_sync = true;
                continue;
            }
            Item::Dropout { .. } => dropouts += 1,
            Item::Data(d) if file.subscriptions[d.subscription].name == "oxide_logger" => {
                let r = file.view(d);
                let t = r.u64("timestamp").unwrap();
                assert!(
                    fl.ticks.contains_key(&t),
                    "{}: oxide_logger at t={t}",
                    fl.ctx
                );
                // A synchronisation message written just before this record
                // is not in the counters yet.
                let (want_syncs, sync_bytes) = if prev_sync {
                    (syncs - 1, 11)
                } else {
                    (syncs, 0)
                };
                assert_eq!(
                    r.u32("written"),
                    Some(written),
                    "{}: written at t={t}",
                    fl.ctx
                );
                assert_eq!(
                    r.u32("syncs"),
                    Some(want_syncs),
                    "{}: syncs at t={t}",
                    fl.ctx
                );
                assert_eq!(r.u32("dropouts"), Some(dropouts), "{}: dropouts", fl.ctx);
                assert_eq!(
                    r.u32("dropped_stream"),
                    Some(0),
                    "{}: dropped_stream",
                    fl.ctx
                );
                assert_eq!(r.u32("dropped_must"), Some(0), "{}: dropped_must", fl.ctx);
                let bytes = u64::try_from(d.offset - sync_bytes - defs_end).unwrap();
                assert_eq!(r.u64("bytes"), Some(bytes), "{}: bytes at t={t}", fl.ctx);
                // The most the ring held at once: more than one message, a
                // number that never goes down, and less than the ring.
                let hw = r.u32("high_water").unwrap();
                assert!(
                    hw >= high && hw > 0 && hw < 65536,
                    "{}: high_water {hw}",
                    fl.ctx
                );
                high = hw;
                n += 1;
            }
            _ => {}
        }
        if matches!(item, Item::Data(_) | Item::Log { .. } | Item::Param { .. }) {
            written += 1;
        }
        prev_sync = false;
    }
    n
}

const TOPICS: [(&str, Expect); 18] = [
    ("sensor_combined", sensor_combined),
    ("vehicle_angular_velocity", vehicle_angular_velocity),
    ("vehicle_rates_setpoint", vehicle_rates_setpoint),
    ("actuator_motors", actuator_motors),
    ("vehicle_thrust_setpoint", vehicle_thrust_setpoint),
    ("vehicle_torque_setpoint", vehicle_torque_setpoint),
    ("vehicle_attitude", vehicle_attitude),
    ("vehicle_attitude_setpoint", vehicle_attitude_setpoint),
    ("vehicle_local_position", vehicle_local_position),
    (
        "vehicle_local_position_setpoint",
        vehicle_local_position_setpoint,
    ),
    ("manual_control_setpoint", manual_control_setpoint),
    ("vehicle_magnetometer", vehicle_magnetometer),
    ("vehicle_air_data", vehicle_air_data),
    ("battery_status", battery_status),
    ("vehicle_status", vehicle_status),
    ("failsafe_flags", failsafe_flags),
    ("oxide_status", oxide_status),
    ("oxide_estimator", oxide_estimator),
];

/// Every record of the file against the flight, topic by topic; the number
/// of records of each topic.
fn check_flight(fl: &Flight, cov: &mut Coverage) -> BTreeMap<&'static str, usize> {
    let file = UlogFile::parse(&fl.bytes).expect("parses");
    let mut counts = BTreeMap::new();
    for (topic, expect) in TOPICS {
        counts.insert(topic, check_topic(fl, &file, topic, expect, cov));
    }
    counts.insert("oxide_event", check_events(fl, &file, cov));
    counts.insert("oxide_logger", check_logger(fl, &file));
    counts
}

/// Parameter changes on the ground: one accepted, one refused (out of
/// range), then the core is told the values are saved, so that
/// `params_unsaved` is seen both ways. At the start the stored-parameter
/// load reports unknown keys, so that the core holds a parameter fault: the
/// pilot's arming at 3 s is refused (`prearm` carries `CONFIG`) until an
/// operator acknowledges it at 3.4 s.
fn param_changes(sim: &mut Sim, now: Micros) {
    let set = |v: f32| ParamSet {
        id: ParamId::LIM_TILT,
        value: ParamValue::F32(v),
    };
    match now.0 {
        0 => sim.core_mut().report_param_load(LoadReport {
            applied: 3,
            unknown: 2,
            dropped: 1,
            invalid: false,
            unreadable: false,
            torn: 4,
        }),
        3_400_000 => sim.queue_param_ack(),
        300_000 => sim.queue_param_set(set(0.6)),
        500_000 => sim.queue_param_set(set(5.0)),
        700_000 => sim.core_mut().clear_unsaved(),
        _ => {}
    }
}

/// A declination the site does not have, set while the vehicle waits to be
/// armed: the heading guard corrects it (an event, the count and the shift in
/// `oxide_estimator`).
fn wrong_heading_corrected(sim: &mut Sim, now: Micros) {
    if now.0 == 100_000 {
        sim.queue_param_set(ParamSet {
            id: params::find("EKF_MAG_DECL").expect("a parameter"),
            value: ParamValue::F32(1.0),
        });
    }
}

/// A declination the site does not have and no correction budget, set
/// while the vehicle waits to be armed: the heading guard contains the
/// heading at the first disagreement (the latch, its event and its fields
/// of `oxide_estimator`).
fn wrong_heading_contained(sim: &mut Sim, now: Micros) {
    let set = |name: &str, value: ParamValue| ParamSet {
        id: params::find(name).expect("a parameter"),
        value,
    };
    match now.0 {
        100_000 => sim.queue_param_set(set("EKF_MAG_DECL", ParamValue::F32(1.0))),
        200_000 => sim.queue_param_set(set("EKF_HDG_NMAX", ParamValue::I32(0))),
        _ => {}
    }
}

/// A stored-parameter load that skipped torn records but understood the rest:
/// no fault, one event.
fn torn_load(sim: &mut Sim, now: Micros) {
    if now.0 == 0 {
        sim.core_mut().report_param_load(LoadReport {
            applied: 90,
            torn: 2,
            ..LoadReport::default()
        });
    }
}

/// The flights: `(scenario, profile, before each tick)`.
fn flown() -> Vec<(&'static str, Profile, &'static Script2)> {
    let none: &'static Script2 = &|_, _| {};
    let params: &'static Script2 = &param_changes;
    let torn: &'static Script2 = &torn_load;
    let mut v = Vec::new();
    for p in BOTH {
        for name in SCENARIOS {
            v.push((name, p, none));
        }
        v.push(("gust-hold", p, params));
        v.push(("battery-low", p, torn));
    }
    // A heading corrected (the default budget) and one contained, in flights
    // that carry the guard's events and fields.
    let corrected: &'static Script2 = &wrong_heading_corrected;
    let contained: &'static Script2 = &wrong_heading_contained;
    v.push(("gust-hold", F405_LEAN, corrected));
    v.push(("gust-hold", H743_FULL, contained));
    v
}

/// The scenarios flown on both profiles, and what each adds. All flags of
/// the topics must be seen both ways over them.
///
/// - `rc-loss-position`: the radio is lost, the vehicle returns and lands
///   (a touchdown and a disarm).
/// - `gust-hold`: position hold in a gust.
/// - `battery-critical`: the low-battery and then the critical-battery latch.
/// - `battery-low`: the low latch alone.
/// - `imu-loss`, `baro-loss`, `mag-loss`, `gps-loss`: a sensor stops, its
///   validity goes and its failsafe reason is raised.
/// - `pair-gps-loss+baro-loss`: both the height and the velocity are lost at
///   once (`v_z_valid` false).
/// - `rc-loss-override`: the pilot takes over from a failsafe.
/// - `geofence`, `brain-crash`, `setpoint-gap`, `no-human-link`: the other
///   failsafe reasons.
/// - `motor-loss`: a rotor fails (loss of control, saturation).
/// - `kill-in-rtl`: the kill switch.
/// - `rc-loss-on-ground`: a loss before take-off, with the failing pre-arm
///   checks.
const SCENARIOS: [&str; 17] = [
    "rc-loss-position",
    "gust-hold",
    "battery-critical",
    "imu-loss",
    "baro-loss",
    "mag-loss",
    "gps-loss",
    "geofence",
    "brain-crash",
    "setpoint-gap",
    "no-human-link",
    "motor-loss",
    "kill-in-rtl",
    "rc-loss-on-ground",
    "gps-loss-pilot",
    "pair-gps-loss+baro-loss",
    "rc-loss-override",
];

#[test]
fn the_reader_gives_the_values_the_simulator_had_at_the_same_ticks() {
    let runs = flown();
    let results: Vec<(String, BTreeMap<&str, usize>, Coverage)> = std::thread::scope(|s| {
        let handles: Vec<_> = runs
            .iter()
            .map(|(name, profile, script)| {
                s.spawn(move || {
                    let fl = fly_with(name, *profile, script);
                    // The reference quad (ADR-0005 §3): 32 N in all, and
                    // 1.272, 1.272 and 0.256 N·m about the three axes.
                    let v = &fl.vehicle;
                    assert_eq!(v.thrust_max_n, 32.0, "{}", fl.ctx);
                    for (got, want) in v.torque_max_nm.iter().zip([1.272, 1.272, 0.256]) {
                        assert!((got - want).abs() < 1e-5, "{}: torque {got}", fl.ctx);
                    }
                    assert_eq!(v.cells, 6, "{}", fl.ctx);
                    let mut cov = Coverage::default();
                    let counts = check_flight(&fl, &mut cov);
                    (fl.ctx, counts, cov)
                })
            })
            .collect();
        handles.into_iter().map(|h| h.join().unwrap()).collect()
    });

    // Every topic of the normal level is checked, none twice.
    let mut names: Vec<&str> = TOPICS.iter().map(|t| t.0).collect();
    names.extend(["oxide_event", "oxide_logger"]);
    let mut listed: Vec<&str> = NORMAL.iter().map(|t| t.name).collect();
    names.sort_unstable();
    listed.sort_unstable();
    assert_eq!(
        names, listed,
        "the topics checked are the topics of the normal level"
    );

    let mut cov = Coverage::default();
    for (ctx, counts, c) in &results {
        for (topic, n) in counts {
            assert!(*n > 0, "{ctx}: no record of {topic}");
        }
        println!("{ctx}: {counts:?}");
        for (k, v) in &c.flags {
            let e = cov.flags.entry(k.clone()).or_default();
            e[0] |= v[0];
            e[1] |= v[1];
        }
        for (k, v) in &c.values {
            cov.values.entry(k.clone()).or_default().extend(v);
        }
        for (t, m) in &c.constants {
            cov.constants.entry(t).or_default().extend(m.clone());
        }
    }
    for (topic, m) in &cov.constants {
        println!(
            "constants of {topic}: {}",
            m.keys().cloned().collect::<Vec<_>>().join(" ")
        );
    }
    for (k, v) in &cov.values {
        println!("values of {k}: {v:?}");
    }
    let never: Vec<String> = cov
        .flags
        .iter()
        .filter(|(_, v)| !(v[0] && v[1]))
        .map(|(k, v)| format!("{k} (seen false: {}, seen true: {})", v[0], v[1]))
        .collect();
    assert!(
        never.is_empty(),
        "flags that were not seen both ways in the flights:\n{}",
        never.join("\n")
    );
    println!("{} flags seen both ways", cov.flags.len());

    // The codes the flights must have shown, so that a code that is
    // swapped or never written cannot hide.
    let seen = |k: &str| cov.values.get(k).cloned().unwrap_or_default();
    let set = |v: &[u64]| {
        v.iter()
            .copied()
            .collect::<std::collections::BTreeSet<u64>>()
    };
    assert_eq!(seen("battery_status.warning"), set(&[0, 1, 2]));
    assert_eq!(seen("failsafe_flags.battery_warning"), set(&[0, 1, 2]));
    assert_eq!(seen("vehicle_status.arming_state"), set(&[1, 2]));
    assert_eq!(seen("manual_control_setpoint.mode switch"), set(&[0, 1, 2]));
    assert_eq!(
        seen("oxide_event.kind"),
        (0..=17).chain([19]).collect(),
        "every kind of event is in some flight (the heading guard's in the wrong-heading \
         flights, the fine step's kind 19 among them; kind 18, the snap-and-reject cycle, is in \
         none: no healthy flight has one, and its encoding is covered by the log crate's own \
         tests)"
    );
    // Every PX4 navigation state the core can show.
    assert_eq!(
        seen("vehicle_status.nav_state"),
        set(&[1, 2, 4, 5, 12, 13, 14, 15, 18])
    );
}

// ---- pyulog ----

/// The columns `pyulog` makes of a record: `(name, type)`, flattened, the
/// trailing padding left out.
fn columns(
    file: &UlogFile,
    layout: &Layout,
    prefix: &str,
    top: bool,
    out: &mut Vec<(String, String)>,
) {
    let trailing = if top {
        layout
            .fields
            .iter()
            .rev()
            .take_while(|f| f.is_padding())
            .count()
    } else {
        0
    };
    let last = layout.fields.len() - trailing;
    for f in &layout.fields[..last] {
        let n = f.array.unwrap_or(0);
        let names: Vec<String> = if n > 0 {
            (0..n).map(|i| format!("{prefix}{}[{i}]", f.name)).collect()
        } else {
            vec![format!("{prefix}{}", f.name)]
        };
        for name in names {
            match &f.ty {
                FieldType::Scalar(s) => {
                    let t = match s {
                        Scalar::I8 => "int8_t",
                        Scalar::U8 => "uint8_t",
                        Scalar::I16 => "int16_t",
                        Scalar::U16 => "uint16_t",
                        Scalar::I32 => "int32_t",
                        Scalar::U32 => "uint32_t",
                        Scalar::I64 => "int64_t",
                        Scalar::U64 => "uint64_t",
                        Scalar::F32 => "float",
                        Scalar::F64 => "double",
                        Scalar::Bool => "bool",
                        Scalar::Char => "char",
                    };
                    out.push((name, t.to_string()));
                }
                FieldType::Nested(inner) => columns(
                    file,
                    file.layout(inner).unwrap(),
                    &format!("{name}."),
                    false,
                    out,
                ),
            }
        }
    }
}

fn cell(r: &RecordView<'_>, name: &str, ty: &str) -> String {
    match ty {
        "float" => format!("0x{:08x}", r.f32(name).unwrap().to_bits()),
        "double" => format!("0x{:016x}", r.f64(name).unwrap().to_bits()),
        "uint8_t" => r.u8(name).unwrap().to_string(),
        "int8_t" => r.i8(name).unwrap().to_string(),
        "uint16_t" => r.u16(name).unwrap().to_string(),
        "int16_t" => r.i16(name).unwrap().to_string(),
        "uint32_t" => r.u32(name).unwrap().to_string(),
        "int32_t" => r.i32(name).unwrap().to_string(),
        "uint64_t" => r.u64(name).unwrap().to_string(),
        "int64_t" => r.i64(name).unwrap().to_string(),
        "bool" => i8::from(r.bool(name).unwrap()).to_string(),
        "char" => (r.char(name).unwrap() as i8).to_string(),
        other => panic!("{other}"),
    }
}

fn python() -> Option<String> {
    let Ok(py) = std::env::var("OXIDE_PYTHON") else {
        println!("NOTICE: OXIDE_PYTHON is not set: the pyulog checks are skipped");
        return None;
    };
    let ok = Command::new(&py)
        .args(["-c", "import pyulog"])
        .output()
        .is_ok_and(|o| o.status.success());
    if !ok {
        println!("NOTICE: {py} cannot import pyulog: the pyulog checks are skipped");
        return None;
    }
    Some(py)
}

fn run(py: &str, args: &[&str]) -> (bool, String, String) {
    let script = concat!(env!("CARGO_MANIFEST_DIR"), "/../../tools/ulog_check.py");
    let o = Command::new(py).arg(script).args(args).output().unwrap();
    (
        o.status.success(),
        String::from_utf8(o.stdout).unwrap(),
        String::from_utf8(o.stderr).unwrap(),
    )
}

#[test]
fn pyulog_reads_every_topic_of_a_flight_and_the_same_values() {
    let Some(py) = python() else { return };
    let expected: Vec<&str> = NORMAL.iter().map(|t| t.name).collect();
    let expected = expected.join(",");
    for profile in BOTH {
        for name in ["rc-loss-position", "gust-hold"] {
            let bytes = fly(name, profile).bytes;
            let path = std::env::temp_dir().join(format!(
                "oxide-sim-pyulog-{}-{name}-{}.ulg",
                std::process::id(),
                profile.name
            ));
            std::fs::write(&path, &bytes).unwrap();
            let p = path.to_str().unwrap();
            let ctx = format!("{name} on {}", profile.name);
            let file = UlogFile::parse(&bytes).unwrap();

            // No corruption, every topic present, add_roll_pitch_yaw works.
            let (ok, out, err) = run(&py, &[p, "--expect-topics", &expected]);
            assert!(ok, "{ctx}: ulog_check.py failed: {err}\n{out}");
            for topic in NORMAL.iter().map(|t| t.name) {
                assert!(
                    out.lines()
                        .any(|l| l.starts_with(&format!("topic {topic} 0 "))),
                    "{ctx}: no {topic} in\n{out}"
                );
            }

            for topic in NORMAL.iter().map(|t| t.name) {
                let (ok, csv, err) = run(&py, &[p, "--dump", topic]);
                assert!(ok, "{ctx}: {err}");
                let sub = file.subscriptions.iter().find(|s| s.name == topic).unwrap();
                let mut cols = Vec::new();
                columns(&file, &file.formats[sub.format].layout, "", true, &mut cols);
                let mut want = vec![
                    cols.iter()
                        .map(|(n, t)| format!("{n}:{t}"))
                        .collect::<Vec<_>>()
                        .join(","),
                ];
                for r in file.records(topic) {
                    want.push(
                        cols.iter()
                            .map(|(n, t)| cell(&r, n, t))
                            .collect::<Vec<_>>()
                            .join(","),
                    );
                }
                assert!(want.len() >= 2, "{ctx}: {topic} has no record");
                let got: Vec<&str> = csv.lines().collect();
                assert_eq!(got.len(), want.len(), "{ctx} {topic}: record count");
                for (i, (g, w)) in got.iter().zip(&want).enumerate() {
                    assert_eq!(g, w, "{ctx} {topic}: line {i}");
                }
            }
            let _ = std::fs::remove_file(path);
        }
    }
}

#[test]
fn ulog_check_without_expect_topics_accepts_a_topic_with_no_record() {
    let Some(py) = python() else { return };
    // A short flight: no parameter changes, and the EKF has not published yet.
    let scenario = fault_scenarios::by_name("rc-loss-position").expect("scenario");
    let mut sim = scenario.build(F405_LEAN, 0).expect("builds");
    let buf = SharedBuffer::new();
    sim.start_log(Level::Normal, &LogBuild::sim("hash"), Box::new(buf.clone()))
        .unwrap();
    for _ in 0..300 {
        sim.step();
    }
    sim.finish_log().unwrap();
    let path = std::env::temp_dir().join(format!("oxide-sim-short-{}.ulg", std::process::id()));
    std::fs::write(&path, buf.bytes()).unwrap();
    let p = path.to_str().unwrap();
    let (ok, out, err) = run(&py, &[p]);
    assert!(ok, "{err}\n{out}");
    let (ok, _, err) = run(&py, &[p, "--expect-topics", "no_such_topic"]);
    assert!(!ok);
    assert!(
        err.contains("expected topic no_such_topic is missing or has no records"),
        "{err}"
    );
    // 300 ticks: vehicle_attitude waits for the estimate (1 s).
    let (ok, _, err) = run(&py, &[p, "--expect-topics", "vehicle_attitude"]);
    assert!(!ok, "{err}");
    assert!(
        err.contains("expected topic vehicle_attitude is missing or has no records"),
        "{err}"
    );
    // A topic that has records passes.
    let (ok, _, err) = run(&py, &[p, "--expect-topics", "sensor_combined"]);
    assert!(ok, "{err}");
    let _ = std::fs::remove_file(path);
}
