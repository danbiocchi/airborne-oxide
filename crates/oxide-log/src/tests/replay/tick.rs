// SPDX-License-Identifier: AGPL-3.0-or-later
//! Step 2: a tick record and the input records after it make an `Inputs`.

use super::harness::*;
use super::samples::*;
use crate::reader::UlogFile;
use crate::replay::decode::{self, InputsError};
use crate::replay::{self, bit};
use oxide_core::io::Inputs;
use oxide_core::time::Micros;

/// What the recorder writes for one tick, with the encoders.
fn write_group(b: &mut Builder, t: Micros, seq: u64, i: &Inputs, saved: bool) {
    b.push(&replay::IN_TICK, |w| replay::fill_tick(w, t, seq, i, saved));
    if let Some(s) = &i.imu {
        b.push(&replay::IN_IMU, |w| replay::fill_imu(w, t, s));
    }
    if let Some(s) = &i.state {
        b.push(&replay::IN_STATE, |w| replay::fill_state(w, t, s));
    }
    if let Some(s) = &i.rc {
        b.push(&replay::IN_RC, |w| replay::fill_rc(w, t, s));
    }
    if let Some(s) = &i.battery {
        b.push(&replay::IN_BATTERY, |w| replay::fill_battery(w, t, s));
    }
    if let Some(s) = &i.setpoint {
        b.push(&replay::IN_SETPOINT, |w| replay::fill_setpoint(w, t, s));
    }
    if let Some(s) = &i.gps {
        b.push(&replay::IN_GPS, |w| replay::fill_gps(w, t, s));
    }
    if let Some(s) = &i.baro {
        b.push(&replay::IN_BARO, |w| replay::fill_baro(w, t, s));
    }
    if let Some(s) = &i.mag {
        b.push(&replay::IN_MAG, |w| replay::fill_mag(w, t, s));
    }
    if let Some(s) = &i.gcs_heartbeat {
        b.push(&replay::IN_GCS, |w| replay::fill_gcs(w, t, s));
    }
    if let Some(s) = &i.brain_heartbeat {
        b.push(&replay::IN_BRAIN, |w| replay::fill_brain(w, t, s));
    }
    if let Some(s) = &i.param_set {
        b.push(&replay::IN_PARAM_SET, |w| replay::fill_param_set(w, t, s));
    }
}

/// Reads the first tick of `f`: the tick record, then every record after it.
fn read(f: &UlogFile) -> Result<(decode::TickRecord, Inputs), InputsError> {
    let mut all = f.items.iter().filter_map(|i| match i {
        crate::reader::Item::Data(r) => Some(f.view(r)),
        _ => None,
    });
    let tick = all.next().unwrap();
    decode::inputs_from_records(&tick, all)
}

fn subset(mask: u16) -> Inputs {
    let full = full_inputs();
    let on = |b: u8| mask & (1 << b) != 0;
    Inputs {
        imu: full.imu.filter(|_| on(bit::IMU)),
        state: full.state.filter(|_| on(bit::STATE)),
        rc: full.rc.filter(|_| on(bit::RC)),
        battery: full.battery.filter(|_| on(bit::BATTERY)),
        setpoint: full.setpoint.filter(|_| on(bit::SETPOINT)),
        gps: full.gps.filter(|_| on(bit::GPS)),
        baro: full.baro.filter(|_| on(bit::BARO)),
        mag: full.mag.filter(|_| on(bit::MAG)),
        gcs_heartbeat: full.gcs_heartbeat.filter(|_| on(bit::GCS)),
        brain_heartbeat: full.brain_heartbeat.filter(|_| on(bit::BRAIN)),
        param_set: full.param_set.filter(|_| on(bit::PARAM_SET)),
        param_ack: mask & 1 == 0,
    }
}

#[test]
fn an_inputs_with_everything_present_round_trips() {
    let i = full_inputs();
    let mut b = Builder::new();
    write_group(&mut b, T, 7, &i, true);
    let (tick, back) = read(&b.file()).unwrap();
    assert!(back.same_bits(&i));
    assert_eq!(
        tick,
        decode::TickRecord {
            t: T,
            seq: 7,
            present: bit::ALL,
            param_ack: true,
            saved: true
        }
    );
}

#[test]
fn an_inputs_with_nothing_present_round_trips() {
    let i = Inputs::default();
    let mut b = Builder::new();
    write_group(&mut b, Micros(0), 0, &i, false);
    let (tick, back) = read(&b.file()).unwrap();
    assert!(back.same_bits(&i));
    assert_eq!((tick.present, tick.seq, tick.saved), (0, 0, false));
}

#[test]
fn every_subset_of_the_inputs_round_trips() {
    for mask in 0..=bit::ALL {
        let i = subset(mask);
        let mut b = Builder::new();
        write_group(
            &mut b,
            Micros(u64::from(mask)),
            u64::from(mask),
            &i,
            mask % 3 == 0,
        );
        let (tick, back) = read(&b.file()).unwrap();
        assert!(back.same_bits(&i), "mask {mask:#x}");
        assert_eq!(tick.present, mask);
        assert_eq!(tick.saved, mask % 3 == 0);
    }
}

#[test]
fn the_param_set_of_a_wrong_kind_and_of_an_unknown_id_round_trip() {
    for ps in [set_wrong_kind(), set_unknown(), set_i32()] {
        let i = Inputs {
            param_set: Some(ps),
            ..Inputs::default()
        };
        let mut b = Builder::new();
        write_group(&mut b, T, 1, &i, false);
        let (_, back) = read(&b.file()).unwrap();
        assert!(back.same_bits(&i));
    }
}

#[test]
fn records_of_other_topics_between_ticks_are_left_alone() {
    let i = subset(bit::ALL & !(1 << bit::BARO));
    let mut b = Builder::new();
    write_group(&mut b, T, 3, &i, false);
    b.push(&replay::OUT, |w| {
        replay::fill_out(w, T, &oxide_core::io::Outputs::<4>::off(T)).unwrap();
    });
    let (_, back) = read(&b.file()).unwrap();
    assert!(back.same_bits(&i));
}

#[test]
fn an_input_record_whose_bit_is_not_set_is_refused() {
    let mut b = Builder::new();
    let none = Inputs::default();
    b.push(&replay::IN_TICK, |w| {
        replay::fill_tick(w, T, 0, &none, false)
    });
    b.push(&replay::IN_IMU, |w| replay::fill_imu(w, T, &imu()));
    assert_eq!(
        read(&b.file()).unwrap_err(),
        InputsError::NotPresent("oxide_in_imu")
    );
}

#[test]
fn a_set_bit_with_no_record_is_refused() {
    let with_imu = Inputs {
        imu: Some(imu()),
        gps: Some(gps()),
        ..Inputs::default()
    };
    let mut b = Builder::new();
    b.push(&replay::IN_TICK, |w| {
        replay::fill_tick(w, T, 0, &with_imu, false)
    });
    b.push(&replay::IN_IMU, |w| replay::fill_imu(w, T, &imu()));
    assert_eq!(
        read(&b.file()).unwrap_err(),
        InputsError::Missing("oxide_in_gps")
    );
}

#[test]
fn a_second_record_of_one_input_is_refused() {
    let with_imu = Inputs {
        imu: Some(imu()),
        ..Inputs::default()
    };
    let mut b = Builder::new();
    b.push(&replay::IN_TICK, |w| {
        replay::fill_tick(w, T, 0, &with_imu, false)
    });
    b.push(&replay::IN_IMU, |w| replay::fill_imu(w, T, &imu()));
    b.push(&replay::IN_IMU, |w| replay::fill_imu(w, T, &imu()));
    assert_eq!(
        read(&b.file()).unwrap_err(),
        InputsError::Duplicate("oxide_in_imu")
    );
}

#[test]
fn an_input_record_from_another_tick_is_refused() {
    let with_imu = Inputs {
        imu: Some(imu()),
        ..Inputs::default()
    };
    let mut b = Builder::new();
    b.push(&replay::IN_TICK, |w| {
        replay::fill_tick(w, T, 0, &with_imu, false)
    });
    b.push(&replay::IN_IMU, |w| {
        replay::fill_imu(w, Micros(T.0 + 1), &imu())
    });
    assert_eq!(
        read(&b.file()).unwrap_err(),
        InputsError::TimeMismatch {
            topic: "oxide_in_imu",
            tick: T.0,
            record: T.0 + 1
        }
    );
}

#[test]
fn a_bad_record_among_the_inputs_names_its_field() {
    let with_rc = Inputs {
        rc: Some(rc()),
        ..Inputs::default()
    };
    let mut bad = record_bytes(&replay::IN_RC, |w| replay::fill_rc(w, T, &rc()));
    bad[40] = 9;
    let mut b = Builder::new();
    b.push(&replay::IN_TICK, |w| {
        replay::fill_tick(w, T, 0, &with_rc, false)
    });
    b.push_raw(&replay::IN_RC, &bad);
    match read(&b.file()).unwrap_err() {
        InputsError::Decode(e) => assert_eq!((e.topic, e.field), ("oxide_in_rc", "arm_switch")),
        other => panic!("{other:?}"),
    }
}

#[test]
fn a_tick_record_with_a_bit_no_input_has_is_refused() {
    let mut rec = record_bytes(&replay::IN_TICK, |w| {
        replay::fill_tick(w, T, 0, &Inputs::default(), false);
    });
    rec[17] = 0x08; // bit 11
    let f = one_raw(&replay::IN_TICK, &rec);
    let e = decode::decode_tick(&f.records("oxide_in_tick").next().unwrap()).unwrap_err();
    assert_eq!(
        (e.field, e.kind),
        ("present", decode::DecodeErrorKind::UnknownBits(0x0800))
    );
}
