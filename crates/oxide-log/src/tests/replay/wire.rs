// SPDX-License-Identifier: AGPL-3.0-or-later
//! Every type that crosses the boundary as a list of its fields.
//!
//! `Flat::flat` destructures a type with every field named and no `..`, and
//! `Flat::unflat` builds it with a struct literal that also names every field,
//! so a field added to a core type stops these from compiling. The mutation
//! walk in `mutation.rs` then proves each listed field is carried.

use super::harness::{T, one};
use crate::replay::{self, decode};
use oxide_core::io::{
    Action, BaroSample, BatterySample, BrainHeartbeat, EstimatorStatus, Event, EventKind,
    EventList, FlightMode, GcsHeartbeat, GpsSample, ImuSample, MagSample, NavState, Outputs,
    ParamSet, PrearmFlags, RcSample, Reasons, Stamped, StateSample, Status, TrajectorySetpoint,
    Validity,
};
use oxide_core::math::{Quat, Vec3};
use oxide_core::params::{self, LoadReport, ParamId, ParamValue};
use oxide_core::schedule::F405_LEAN;
use oxide_core::time::Micros;

/// One field: its bits, and two other values it may take.
#[derive(Clone)]
pub struct Fld {
    pub v: u64,
    pub alts: [u64; 2],
}

pub fn f32f(x: f32) -> Fld {
    let b = u64::from(x.to_bits());
    Fld {
        v: b,
        alts: [b ^ 1, b ^ 0x8000_0000],
    }
}
pub fn u64f(x: u64) -> Fld {
    Fld {
        v: x,
        alts: [x ^ 1, x ^ (1 << 63)],
    }
}
pub fn u32f(x: u32) -> Fld {
    u64f(u64::from(x)).narrow(32)
}
pub fn u16f(x: u16) -> Fld {
    u64f(u64::from(x)).narrow(16)
}
pub fn u8f(x: u8) -> Fld {
    u64f(u64::from(x)).narrow(8)
}
pub fn boolf(x: bool) -> Fld {
    let o = u64::from(!x);
    Fld {
        v: u64::from(x),
        alts: [o, o],
    }
}
/// A field with its own valid alternatives (a code, or a set of bits).
pub fn altf(x: u64, a: [u64; 2]) -> Fld {
    Fld { v: x, alts: a }
}

impl Fld {
    /// Keep the alternatives inside `bits` bits.
    fn narrow(mut self, bits: u32) -> Self {
        let m = (1u64 << bits) - 1;
        self.alts = [self.alts[0] & m, (self.v ^ (1 << (bits - 1))) & m];
        self
    }
}

pub trait Flat: Sized {
    fn flat(&self) -> Vec<Fld>;
    fn unflat(next: &mut dyn FnMut() -> u64) -> Self;
}

pub fn values(f: &[Fld]) -> Vec<u64> {
    f.iter().map(|x| x.v).collect()
}

/// Encodes with the replay encoder and decodes with the decoder.
pub trait Wire: Flat {
    fn through(&self) -> Self;
}

fn n32(next: &mut dyn FnMut() -> u64) -> f32 {
    f32::from_bits(next() as u32)
}

fn vec3(v: &Vec3<f32>) -> Vec<Fld> {
    let Vec3 { x, y, z } = v;
    vec![f32f(*x), f32f(*y), f32f(*z)]
}
fn unvec3(next: &mut dyn FnMut() -> u64) -> Vec3<f32> {
    Vec3 {
        x: n32(next),
        y: n32(next),
        z: n32(next),
    }
}

impl<T: Flat> Flat for Stamped<T> {
    fn flat(&self) -> Vec<Fld> {
        let Stamped {
            t_sample,
            t_arrival,
            value,
        } = self;
        let mut v = vec![u64f(t_sample.0), u64f(t_arrival.0)];
        v.extend(value.flat());
        v
    }
    fn unflat(next: &mut dyn FnMut() -> u64) -> Self {
        Stamped {
            t_sample: Micros(next()),
            t_arrival: Micros(next()),
            value: T::unflat(next),
        }
    }
}

impl Flat for ImuSample {
    fn flat(&self) -> Vec<Fld> {
        let ImuSample { gyro, accel } = self;
        [vec3(gyro), vec3(accel)].concat()
    }
    fn unflat(next: &mut dyn FnMut() -> u64) -> Self {
        ImuSample {
            gyro: unvec3(next),
            accel: unvec3(next),
        }
    }
}

impl Flat for StateSample {
    fn flat(&self) -> Vec<Fld> {
        let StateSample {
            position,
            velocity,
            attitude,
            rate,
        } = self;
        let Quat { w, x, y, z } = attitude;
        [
            vec3(position),
            vec3(velocity),
            vec![f32f(*w), f32f(*x), f32f(*y), f32f(*z)],
            vec3(rate),
        ]
        .concat()
    }
    fn unflat(next: &mut dyn FnMut() -> u64) -> Self {
        StateSample {
            position: unvec3(next),
            velocity: unvec3(next),
            attitude: Quat {
                w: n32(next),
                x: n32(next),
                y: n32(next),
                z: n32(next),
            },
            rate: unvec3(next),
        }
    }
}

impl Flat for RcSample {
    fn flat(&self) -> Vec<Fld> {
        let RcSample {
            roll,
            pitch,
            yaw,
            throttle,
            arm_switch,
            kill_switch,
            mode_switch,
            failsafe,
            frame_lost,
        } = self;
        vec![
            f32f(*roll),
            f32f(*pitch),
            f32f(*yaw),
            f32f(*throttle),
            boolf(*arm_switch),
            boolf(*kill_switch),
            u8f(*mode_switch),
            boolf(*failsafe),
            boolf(*frame_lost),
        ]
    }
    fn unflat(next: &mut dyn FnMut() -> u64) -> Self {
        RcSample {
            roll: n32(next),
            pitch: n32(next),
            yaw: n32(next),
            throttle: n32(next),
            arm_switch: next() != 0,
            kill_switch: next() != 0,
            mode_switch: next() as u8,
            failsafe: next() != 0,
            frame_lost: next() != 0,
        }
    }
}

impl Flat for BatterySample {
    fn flat(&self) -> Vec<Fld> {
        let BatterySample {
            voltage_v,
            current_a,
        } = self;
        vec![f32f(*voltage_v), f32f(*current_a)]
    }
    fn unflat(next: &mut dyn FnMut() -> u64) -> Self {
        BatterySample {
            voltage_v: n32(next),
            current_a: n32(next),
        }
    }
}

impl Flat for TrajectorySetpoint {
    fn flat(&self) -> Vec<Fld> {
        let TrajectorySetpoint {
            position,
            velocity,
            yaw,
        } = self;
        [vec3(position), vec3(velocity), vec![f32f(*yaw)]].concat()
    }
    fn unflat(next: &mut dyn FnMut() -> u64) -> Self {
        TrajectorySetpoint {
            position: unvec3(next),
            velocity: unvec3(next),
            yaw: n32(next),
        }
    }
}

impl Flat for GpsSample {
    fn flat(&self) -> Vec<Fld> {
        let GpsSample {
            position,
            velocity,
            h_acc_m,
            v_acc_m,
            s_acc_mps,
            fix_3d,
        } = self;
        [
            vec3(position),
            vec3(velocity),
            vec![
                f32f(*h_acc_m),
                f32f(*v_acc_m),
                f32f(*s_acc_mps),
                boolf(*fix_3d),
            ],
        ]
        .concat()
    }
    fn unflat(next: &mut dyn FnMut() -> u64) -> Self {
        GpsSample {
            position: unvec3(next),
            velocity: unvec3(next),
            h_acc_m: n32(next),
            v_acc_m: n32(next),
            s_acc_mps: n32(next),
            fix_3d: next() != 0,
        }
    }
}

impl Flat for BaroSample {
    fn flat(&self) -> Vec<Fld> {
        let BaroSample { altitude_m } = self;
        vec![f32f(*altitude_m)]
    }
    fn unflat(next: &mut dyn FnMut() -> u64) -> Self {
        BaroSample {
            altitude_m: n32(next),
        }
    }
}

impl Flat for MagSample {
    fn flat(&self) -> Vec<Fld> {
        let MagSample { field_ut } = self;
        vec3(field_ut)
    }
    fn unflat(next: &mut dyn FnMut() -> u64) -> Self {
        MagSample {
            field_ut: unvec3(next),
        }
    }
}

impl Flat for GcsHeartbeat {
    fn flat(&self) -> Vec<Fld> {
        let GcsHeartbeat = self;
        vec![]
    }
    fn unflat(_: &mut dyn FnMut() -> u64) -> Self {
        GcsHeartbeat
    }
}

impl Flat for BrainHeartbeat {
    fn flat(&self) -> Vec<Fld> {
        let BrainHeartbeat { progress } = self;
        vec![u32f(*progress)]
    }
    fn unflat(next: &mut dyn FnMut() -> u64) -> Self {
        BrainHeartbeat {
            progress: next() as u32,
        }
    }
}

impl Flat for ParamSet {
    fn flat(&self) -> Vec<Fld> {
        let ParamSet { id, value } = self;
        let n = params::COUNT as u64;
        let id = u64::from(id.0);
        let (kind, bits) = match value {
            ParamValue::F32(v) => (0u64, u64::from(v.to_bits())),
            ParamValue::I32(v) => (1u64, u64::from(*v as u32)),
        };
        vec![
            altf(id, [(id + 1) % n, (id + 2) % n]),
            altf(kind, [1 - kind, 1 - kind]),
            u32f(bits as u32),
        ]
    }
    fn unflat(next: &mut dyn FnMut() -> u64) -> Self {
        let id = ParamId(next() as u16);
        let kind = next();
        let bits = next() as u32;
        ParamSet {
            id,
            value: if kind == 0 {
                ParamValue::F32(f32::from_bits(bits))
            } else {
                ParamValue::I32(bits as i32)
            },
        }
    }
}

impl Flat for decode::TickRecord {
    fn flat(&self) -> Vec<Fld> {
        let decode::TickRecord {
            t,
            seq,
            present,
            param_ack,
            saved,
        } = self;
        vec![
            u64f(t.0),
            u64f(*seq),
            altf(
                u64::from(*present),
                [u64::from(*present ^ 1), u64::from(*present ^ (1 << 10))],
            ),
            boolf(*param_ack),
            boolf(*saved),
        ]
    }
    fn unflat(next: &mut dyn FnMut() -> u64) -> Self {
        decode::TickRecord {
            t: Micros(next()),
            seq: next(),
            present: next() as u16,
            param_ack: next() != 0,
            saved: next() != 0,
        }
    }
}

impl Flat for decode::Boot {
    fn flat(&self) -> Vec<Fld> {
        let decode::Boot {
            t,
            report,
            rotors,
            base_period_us,
            filter_period_ticks,
            task_period_ticks,
        } = self;
        let LoadReport {
            applied,
            unknown,
            dropped,
            invalid,
            unreadable,
            torn,
        } = report;
        let mut v = vec![
            u64f(t.0),
            u16f(*applied),
            u16f(*unknown),
            u16f(*dropped),
            boolf(*invalid),
            boolf(*unreadable),
            u16f(*torn),
            u8f(*rotors),
            u32f(*base_period_us),
            u32f(*filter_period_ticks),
        ];
        v.extend(task_period_ticks.iter().map(|x| u32f(*x)));
        v
    }
    fn unflat(next: &mut dyn FnMut() -> u64) -> Self {
        decode::Boot {
            t: Micros(next()),
            report: LoadReport {
                applied: next() as u16,
                unknown: next() as u16,
                dropped: next() as u16,
                invalid: next() != 0,
                unreadable: next() != 0,
                torn: next() as u16,
            },
            rotors: next() as u8,
            base_period_us: next() as u32,
            filter_period_ticks: next() as u32,
            task_period_ticks: [
                next() as u32,
                next() as u32,
                next() as u32,
                next() as u32,
                next() as u32,
            ],
        }
    }
}

fn code<T>(x: u8, a: [u8; 2], _: &T) -> Fld {
    altf(u64::from(x), [u64::from(a[0]), u64::from(a[1])])
}

impl Flat for decode::OutRecord<4> {
    fn flat(&self) -> Vec<Fld> {
        let decode::OutRecord {
            t,
            t_valid,
            motors,
            status,
            events,
        } = self;
        let Status {
            armed,
            killed,
            mode,
            saturated,
            timing_faults,
            no_estimate,
            estimator,
            nav,
            valid,
            reasons,
            action,
            prearm,
            home_set,
            params_unsaved,
        } = status;
        let EstimatorStatus {
            aligned,
            gps_aiding,
            rejected,
            resets,
            // Not in the replay record (the outputs cover them).
            heading_corrections: _,
            heading_contained: _,
            heading_shift_rad: _,
        } = estimator;
        let Validity {
            attitude,
            heading,
            altitude,
            velocity,
            position,
        } = valid;
        let mc = mode.code();
        let nc = nav.code();
        let ac = action.code();
        let mut v = vec![u64f(t.0), u64f(t_valid.0)];
        v.extend(motors.iter().map(|m| f32f(*m)));
        v.extend([
            boolf(*armed),
            boolf(*killed),
            code(mc, [(mc + 1) % 4, (mc + 2) % 4], mode),
            boolf(*saturated),
            u32f(*timing_faults),
            boolf(*no_estimate),
            boolf(*aligned),
            boolf(*gps_aiding),
            u32f(*rejected),
            u32f(*resets),
            code(nc, [(nc + 1) % 10, (nc + 2) % 10], nav),
            boolf(*attitude),
            boolf(*heading),
            boolf(*altitude),
            boolf(*velocity),
            boolf(*position),
            altf(
                u64::from(reasons.0),
                [u64::from(reasons.0 ^ 1), u64::from(reasons.0 ^ (1 << 10))],
            ),
            code(ac, [(ac + 1) % 6, (ac + 2) % 6], action),
            altf(
                u64::from(prearm.0),
                [u64::from(prearm.0 ^ 1), u64::from(prearm.0 ^ (1 << 8))],
            ),
            boolf(*home_set),
            boolf(*params_unsaved),
            altf(
                u64::from(*events),
                [u64::from(*events ^ 1), u64::from(*events ^ 2)],
            ),
        ]);
        v
    }
    fn unflat(next: &mut dyn FnMut() -> u64) -> Self {
        let t = Micros(next());
        let t_valid = Micros(next());
        let motors = [n32(next), n32(next), n32(next), n32(next)];
        let armed = next() != 0;
        let killed = next() != 0;
        let mode = FlightMode::from_code(next() as u8).unwrap();
        let saturated = next() != 0;
        let timing_faults = next() as u32;
        let no_estimate = next() != 0;
        let aligned = next() != 0;
        let gps_aiding = next() != 0;
        let rejected = next() as u32;
        let resets = next() as u32;
        let nav = NavState::from_code(next() as u8).unwrap();
        let valid = Validity {
            attitude: next() != 0,
            heading: next() != 0,
            altitude: next() != 0,
            velocity: next() != 0,
            position: next() != 0,
        };
        let reasons = Reasons(next() as u16);
        let action = Action::from_code(next() as u8).unwrap();
        let prearm = PrearmFlags(next() as u16);
        let home_set = next() != 0;
        let params_unsaved = next() != 0;
        let events = next() as u8;
        decode::OutRecord {
            t,
            t_valid,
            motors,
            status: Status {
                armed,
                killed,
                mode,
                saturated,
                timing_faults,
                no_estimate,
                estimator: EstimatorStatus {
                    aligned,
                    gps_aiding,
                    rejected,
                    resets,
                    ..EstimatorStatus::default()
                },
                nav,
                valid,
                reasons,
                action,
                prearm,
                home_set,
                params_unsaved,
            },
            events,
        }
    }
}

/// Rebuilds a value from its field values.
pub fn build<T: Flat>(v: &[u64]) -> T {
    let mut it = v.iter().copied();
    let mut next = || it.next().unwrap();
    T::unflat(&mut next)
}

/// Fields as values, for comparison.
pub fn vals<T: Flat>(x: &T) -> Vec<u64> {
    values(&x.flat())
}

fn first<'a>(f: &'a crate::reader::UlogFile, topic: &'a str) -> crate::reader::RecordView<'a> {
    f.records(topic).next().unwrap()
}

macro_rules! wire {
    ($ty:ty, $topic:ident, $fill:ident, $dec:ident) => {
        impl Wire for $ty {
            fn through(&self) -> Self {
                let f = one(&replay::$topic, |w| replay::$fill(w, T, self));
                let r = first(&f, replay::$topic.name);
                assert_eq!(decode::record_time(&r).unwrap(), T);
                decode::$dec(&r).unwrap()
            }
        }
    };
}

wire!(Stamped<ImuSample>, IN_IMU, fill_imu, decode_imu);
wire!(Stamped<StateSample>, IN_STATE, fill_state, decode_state);
wire!(Stamped<RcSample>, IN_RC, fill_rc, decode_rc);
wire!(
    Stamped<BatterySample>,
    IN_BATTERY,
    fill_battery,
    decode_battery
);
wire!(
    Stamped<TrajectorySetpoint>,
    IN_SETPOINT,
    fill_setpoint,
    decode_setpoint
);
wire!(Stamped<GpsSample>, IN_GPS, fill_gps, decode_gps);
wire!(Stamped<BaroSample>, IN_BARO, fill_baro, decode_baro);
wire!(Stamped<MagSample>, IN_MAG, fill_mag, decode_mag);
wire!(Stamped<GcsHeartbeat>, IN_GCS, fill_gcs, decode_gcs);
wire!(Stamped<BrainHeartbeat>, IN_BRAIN, fill_brain, decode_brain);

impl Wire for ParamSet {
    fn through(&self) -> Self {
        let f = one(&replay::IN_PARAM_SET, |w| {
            replay::fill_param_set(w, T, self)
        });
        decode::decode_param_set(&first(&f, "oxide_in_param_set")).unwrap()
    }
}

impl Wire for decode::TickRecord {
    fn through(&self) -> Self {
        // The tick record is written from `Inputs`: rebuild inputs with the
        // same presence bits (the values do not matter to the tick record).
        let mut inputs = super::samples::full_inputs();
        let drop = |bit: u8| self.present & (1 << bit) == 0;
        if drop(replay::bit::IMU) {
            inputs.imu = None;
        }
        if drop(replay::bit::STATE) {
            inputs.state = None;
        }
        if drop(replay::bit::RC) {
            inputs.rc = None;
        }
        if drop(replay::bit::BATTERY) {
            inputs.battery = None;
        }
        if drop(replay::bit::SETPOINT) {
            inputs.setpoint = None;
        }
        if drop(replay::bit::GPS) {
            inputs.gps = None;
        }
        if drop(replay::bit::BARO) {
            inputs.baro = None;
        }
        if drop(replay::bit::MAG) {
            inputs.mag = None;
        }
        if drop(replay::bit::GCS) {
            inputs.gcs_heartbeat = None;
        }
        if drop(replay::bit::BRAIN) {
            inputs.brain_heartbeat = None;
        }
        if drop(replay::bit::PARAM_SET) {
            inputs.param_set = None;
        }
        inputs.param_ack = self.param_ack;
        let f = one(&replay::IN_TICK, |w| {
            replay::fill_tick(w, self.t, self.seq, &inputs, self.saved);
        });
        decode::decode_tick(&first(&f, "oxide_in_tick")).unwrap()
    }
}

impl Wire for decode::Boot {
    fn through(&self) -> Self {
        let mut profile = F405_LEAN;
        profile.base_period_us = self.base_period_us;
        profile.filter_period_ticks = self.filter_period_ticks;
        for (t, p) in profile.tasks.iter_mut().zip(self.task_period_ticks) {
            t.period_ticks = p;
        }
        let f = one(&replay::BOOT, |w| {
            replay::fill_boot(w, self.t, &self.report, usize::from(self.rotors), &profile).unwrap();
        });
        decode::decode_boot(&first(&f, "oxide_boot")).unwrap()
    }
}

impl Wire for decode::OutRecord<4> {
    fn through(&self) -> Self {
        let mut events = EventList::new();
        for _ in 0..self.events {
            // More than four cannot be in a list: the record's count is
            // checked separately.
            events.push(Event {
                t: Micros(0),
                kind: EventKind::Armed,
            });
        }
        let out = Outputs::<4> {
            t_valid: self.t_valid,
            motors: self.motors,
            status: self.status,
            events,
        };
        let f = one(&replay::OUT, |w| {
            replay::fill_out(w, self.t, &out).unwrap();
        });
        decode::decode_out::<4>(&first(&f, "oxide_out")).unwrap()
    }
}
