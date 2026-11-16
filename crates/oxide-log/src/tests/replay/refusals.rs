// SPDX-License-Identifier: AGPL-3.0-or-later
//! Step 2: what the decoders refuse, and how they say so.

use super::harness::*;
use super::samples::*;
use crate::replay::decode::DecodeErrorKind as K;
use crate::replay::{self, decode};
use oxide_core::io::Outputs;
use oxide_core::time::Micros;

fn out_record() -> Vec<u8> {
    record_bytes(&replay::OUT, |w| {
        let o = Outputs::<4> {
            motors: [0.1, 0.2, 0.3, 0.4],
            status: busy_status(),
            ..Outputs::off(Micros(9))
        };
        replay::fill_out(w, T, &o).unwrap();
    })
}

fn decode_out_with(edit: impl FnOnce(&mut Vec<u8>)) -> Result<(), decode::DecodeError> {
    let mut rec = out_record();
    edit(&mut rec);
    let f = one_raw(&replay::OUT, &rec);
    decode::decode_out::<4>(&f.records("oxide_out").next().unwrap()).map(|_| ())
}

// Offsets in oxide_out.
const REASONS: usize = 60;
const PREARM: usize = 62;
const MOTOR_COUNT: usize = 64;
const EVENTS: usize = 65;
const MODE: usize = 66;
const NAV: usize = 67;
const ACTION: usize = 68;
const ARMED: usize = 69;
const PARAMS_UNSAVED: usize = 81;

#[test]
fn the_untouched_output_record_decodes() {
    decode_out_with(|_| {}).unwrap();
}

#[test]
fn an_unknown_code_names_the_topic_and_the_field() {
    for (at, field, code) in [(MODE, "mode", 4u8), (NAV, "nav", 10), (ACTION, "action", 6)] {
        let e = decode_out_with(|r| r[at] = code).unwrap_err();
        assert_eq!(e.topic, "oxide_out");
        assert_eq!(e.field, field);
        assert_eq!(e.kind, K::UnknownCode(u32::from(code)));
        let text = e.to_string();
        assert!(text.contains("oxide_out") && text.contains(field), "{text}");
    }
}

#[test]
fn a_flag_byte_that_is_not_zero_or_one_is_refused() {
    let e = decode_out_with(|r| r[ARMED] = 2).unwrap_err();
    assert_eq!(
        (e.topic, e.field, e.kind),
        ("oxide_out", "armed", K::BadFlag(2))
    );
    let e = decode_out_with(|r| r[PARAMS_UNSAVED] = 255).unwrap_err();
    assert_eq!(e.field, "params_unsaved");
    assert_eq!(e.kind, K::BadFlag(255));
    // The reader's own `bool` would read both as true.
    let mut rec = record_bytes(&replay::IN_RC, |w| replay::fill_rc(w, T, &rc()));
    rec[40] = 7; // arm_switch
    let f = one_raw(&replay::IN_RC, &rec);
    let e = decode::decode_rc(&f.records("oxide_in_rc").next().unwrap()).unwrap_err();
    assert_eq!(
        (e.topic, e.field, e.kind),
        ("oxide_in_rc", "arm_switch", K::BadFlag(7))
    );
}

#[test]
fn bits_that_no_flag_has_are_refused() {
    let e = decode_out_with(|r| r[REASONS + 1] = 0x08).unwrap_err();
    assert_eq!((e.field, e.kind), ("reasons", K::UnknownBits(0x0800)));
    let e = decode_out_with(|r| r[PREARM + 1] = 0x02).unwrap_err();
    assert_eq!((e.field, e.kind), ("prearm", K::UnknownBits(0x0200)));
    // Reasons has 11 bits, so bit 10 is fine and bit 11 is not.
    decode_out_with(|r| {
        r[REASONS] = 0;
        r[REASONS + 1] = 0x04;
    })
    .unwrap();
}

#[test]
fn a_motor_count_above_eight_or_not_n_is_refused() {
    let e = decode_out_with(|r| r[MOTOR_COUNT] = 9).unwrap_err();
    assert_eq!((e.field, e.kind), ("motor_count", K::TooManyMotors(9)));
    let e = decode_out_with(|r| r[MOTOR_COUNT] = 5).unwrap_err();
    assert_eq!(
        (e.field, e.kind),
        (
            "motor_count",
            K::MotorCount {
                recorded: 5,
                expected: 4
            }
        )
    );
    let e = decode_out_with(|r| r[MOTOR_COUNT] = 0).unwrap_err();
    assert_eq!(
        e.kind,
        K::MotorCount {
            recorded: 0,
            expected: 4
        }
    );
}

#[test]
fn more_than_eight_rotors_cannot_be_decoded() {
    let f = one_raw(&replay::OUT, &out_record());
    let e = decode::decode_out::<9>(&f.records("oxide_out").next().unwrap()).unwrap_err();
    assert_eq!(e.field, "motor_count");
}

#[test]
fn an_event_count_above_the_most_a_tick_reports_is_refused() {
    let e = decode_out_with(|r| r[EVENTS] = 5).unwrap_err();
    assert_eq!((e.field, e.kind), ("events", K::TooManyEvents(5)));
}

#[test]
fn a_record_of_another_topic_is_refused() {
    let f = one(&replay::IN_BARO, |w| replay::fill_baro(w, T, &baro()));
    let e = decode::decode_imu(&f.records("oxide_in_baro").next().unwrap()).unwrap_err();
    assert_eq!((e.topic, e.kind), ("oxide_in_imu", K::WrongTopic));
}

#[test]
fn a_parameter_key_this_build_does_not_have_is_refused() {
    let mut rec = record_bytes(&replay::IN_PARAM_SET, |w| {
        replay::fill_param_set(w, T, &set_f32());
    });
    rec[8..12].copy_from_slice(&0xDEAD_BEEFu32.to_le_bytes());
    let f = one_raw(&replay::IN_PARAM_SET, &rec);
    let e = decode::decode_param_set(&f.records("oxide_in_param_set").next().unwrap()).unwrap_err();
    assert_eq!((e.field, e.kind), ("key", K::UnknownKey(0xDEAD_BEEF)));
}

#[test]
fn an_unknown_kind_or_an_id_inside_the_table_marked_unknown_is_refused() {
    let base = record_bytes(&replay::IN_PARAM_SET, |w| {
        replay::fill_param_set(w, T, &set_f32());
    });
    let dec = |rec: &[u8]| {
        let f = one_raw(&replay::IN_PARAM_SET, rec);
        decode::decode_param_set(&f.records("oxide_in_param_set").next().unwrap())
    };
    let mut r = base.clone();
    r[18] = 2; // kind
    let e = dec(&r).unwrap_err();
    assert_eq!((e.field, e.kind), ("kind", K::UnknownCode(2)));
    // known = false with an id that this build has.
    let mut r = base;
    r[19] = 0;
    r[16..18].copy_from_slice(&3u16.to_le_bytes());
    let e = dec(&r).unwrap_err();
    assert_eq!((e.field, e.kind), ("id", K::UnknownId(3)));
}

#[test]
fn a_vehicle_record_with_a_bad_name_is_refused() {
    let base = record_bytes(&replay::VEHICLE, |w| {
        replay::fill_vehicle_number(w, T, 0, "mass_kg", 0.65);
    });
    let dec = |rec: &[u8]| {
        let f = one_raw(&replay::VEHICLE, rec);
        decode::decode_vehicle(&f.records("oxide_vehicle").next().unwrap())
    };
    let v = dec(&base).unwrap();
    assert_eq!(
        (v.index, v.name.as_str(), v.value.to_bits()),
        (0, "mass_kg", 0.65f32.to_bits())
    );
    let mut r = base.clone();
    r[14] = 0xFF; // not UTF-8
    assert_eq!(dec(&r).unwrap_err().kind, K::BadName);
    let mut r = base;
    r[14 + 10] = b'x'; // text after the terminator
    assert_eq!(dec(&r).unwrap_err().kind, K::BadName);
}
