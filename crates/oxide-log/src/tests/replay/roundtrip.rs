// SPDX-License-Identifier: AGPL-3.0-or-later
//! Step 2: encode, read with the reader, decode: the same bits come back.

use super::harness::*;
use super::samples::*;
use super::wire::*;
use crate::replay::{self, decode};
use oxide_core::io::{Outputs, Status};
use oxide_core::params::LoadReport;
use oxide_core::time::Micros;

fn same<W: Wire>(x: &W) {
    assert_eq!(vals(&x.through()), vals(x));
}

#[test]
fn every_input_comes_back_with_the_same_bits() {
    same(&imu());
    same(&state());
    same(&rc());
    same(&rc_other());
    same(&battery());
    same(&setpoint());
    same(&gps());
    same(&gps_nofix());
    same(&baro());
    same(&mag());
    same(&gcs());
    same(&brain());
}

#[test]
fn the_extreme_times_survive() {
    for (a, b) in [(0, 0), (u64::MAX, u64::MAX), (0, u64::MAX), (u64::MAX, 0)] {
        let mut s = imu();
        s.t_sample = Micros(a);
        s.t_arrival = Micros(b);
        same(&s);
    }
}

#[test]
fn the_float_corner_cases_survive_in_every_float_field() {
    let corner = [
        NAN_PAYLOAD,
        NEG_NAN,
        -0.0,
        0.0,
        f32::INFINITY,
        f32::NEG_INFINITY,
        SUBNORMAL,
        -SUBNORMAL,
        f32::MAX,
        f32::MIN,
        f32::MIN_POSITIVE,
        f32::EPSILON,
    ];
    for c in corner {
        let mut s = state();
        s.value.position = v3(c, c, c);
        s.value.velocity = v3(c, c, c);
        s.value.attitude = oxide_core::math::Quat {
            w: c,
            x: c,
            y: c,
            z: c,
        };
        s.value.rate = v3(c, c, c);
        same(&s);
        let mut g = gps();
        g.value.position = v3(c, c, c);
        g.value.h_acc_m = c;
        same(&g);
        let mut r = rc();
        r.value.roll = c;
        r.value.yaw = c;
        same(&r);
    }
}

#[test]
fn a_parameter_set_comes_back_by_key() {
    for ps in [set_f32(), set_i32(), set_wrong_kind()] {
        same(&ps);
    }
    // The key in the record is the parameter's stable key.
    let f = one(&replay::IN_PARAM_SET, |w| {
        replay::fill_param_set(w, T, &set_f32());
    });
    let r = f.records("oxide_in_param_set").next().unwrap();
    let id = oxide_core::params::find("RATE_ROLL_P").unwrap();
    assert_eq!(r.u32("key"), oxide_core::params::key_of(id));
    assert_eq!(r.u32("bits"), Some(NAN_PAYLOAD.to_bits()));
    assert_eq!(r.u8("kind"), Some(replay::KIND_F32));
    assert_eq!(r.bool("known"), Some(true));
}

#[test]
fn a_parameter_set_outside_the_table_is_exact_too() {
    let ps = set_unknown();
    let f = one(&replay::IN_PARAM_SET, |w| replay::fill_param_set(w, T, &ps));
    let r = f.records("oxide_in_param_set").next().unwrap();
    assert_eq!(r.bool("known"), Some(false));
    assert_eq!(r.u16("id"), Some(60_000));
    let back = decode::decode_param_set(&r).unwrap();
    assert_eq!(back.id, ps.id);
    assert_eq!(vals(&back), vals(&ps));
}

fn boots() -> Vec<decode::Boot> {
    let report = |a, u, d, i, r, t| LoadReport {
        applied: a,
        unknown: u,
        dropped: d,
        invalid: i,
        unreadable: r,
        torn: t,
    };
    vec![
        decode::Boot {
            t: Micros(0),
            report: LoadReport::default(),
            rotors: 4,
            base_period_us: 1000,
            filter_period_ticks: 4,
            task_period_ticks: [1, 20, 20, 4, 1],
        },
        decode::Boot {
            t: Micros(u64::MAX),
            report: report(u16::MAX, u16::MAX, u16::MAX, true, true, u16::MAX),
            rotors: 255,
            base_period_us: u32::MAX,
            filter_period_ticks: u32::MAX,
            task_period_ticks: [u32::MAX, 0, 1, 2, 3],
        },
    ]
}

#[test]
fn the_boot_record_comes_back_field_by_field() {
    for b in boots() {
        same(&b);
        assert_eq!(b.through(), b);
    }
}

#[test]
fn the_boot_record_carries_the_profile_it_was_given() {
    use oxide_core::schedule::{F405_LEAN, H743_FULL};
    for p in [F405_LEAN, H743_FULL] {
        let f = one(&replay::BOOT, |w| {
            replay::fill_boot(w, T, &LoadReport::default(), 4, &p).unwrap();
        });
        let b = decode::decode_boot(&f.records("oxide_boot").next().unwrap()).unwrap();
        assert_eq!(b.base_period_us, p.base_period_us);
        assert_eq!(b.filter_period_ticks, p.filter_period_ticks);
        let want = p.tasks.map(|t| t.period_ticks);
        assert_eq!(b.task_period_ticks, want);
        assert_eq!(b.rotors, 4);
        assert_eq!(b.t, T);
    }
}

#[test]
fn the_tick_record_comes_back() {
    for present in [0u16, 1, 0x07FF, 0x0555, 1 << 10] {
        for (ack, saved) in [(false, false), (true, false), (false, true), (true, true)] {
            let t = decode::TickRecord {
                t: Micros(u64::MAX),
                seq: u64::MAX - 3,
                present,
                param_ack: ack,
                saved,
            };
            assert_eq!(t.through(), t);
        }
    }
}

fn statuses() -> Vec<Status> {
    let mut v = vec![Status::default(), busy_status()];
    for mode in MODES {
        for nav in NAVS {
            for action in ACTIONS {
                v.push(Status {
                    mode,
                    nav,
                    action,
                    ..Status::default()
                });
            }
        }
    }
    v
}

#[test]
fn the_output_record_comes_back_for_every_enumeration_variant() {
    for status in statuses() {
        for events in 0..=3u8 {
            let o = decode::OutRecord::<4> {
                t: Micros(u64::MAX),
                t_valid: Micros(1),
                motors: [NAN_PAYLOAD, -0.0, f32::INFINITY, SUBNORMAL],
                status,
                events,
            };
            let back = o.through();
            assert_eq!(back.status, o.status);
            assert_eq!(back.events, o.events);
            assert_eq!(vals(&back), vals(&o));
        }
    }
}

#[test]
fn an_output_record_makes_outputs_with_the_events_given() {
    use oxide_core::io::{Event, EventKind, EventList};
    let o = decode::OutRecord::<4> {
        t: Micros(10),
        t_valid: Micros(11),
        motors: [0.1, 0.2, 0.3, 0.4],
        status: busy_status(),
        events: 1,
    };
    let mut ev = EventList::new();
    ev.push(Event {
        t: Micros(10),
        kind: EventKind::Touchdown,
    });
    let out = o.clone().into_outputs(ev);
    let want = Outputs::<4> {
        t_valid: Micros(11),
        motors: [0.1, 0.2, 0.3, 0.4],
        status: busy_status(),
        events: ev,
    };
    assert!(out.same_bits(&want));
}
