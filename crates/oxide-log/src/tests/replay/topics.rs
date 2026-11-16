// SPDX-License-Identifier: AGPL-3.0-or-later
//! Step 1: the topics and their encoders.

use super::samples::*;
use crate::format::assert_topic;
use crate::msg::{Error, Sink};
use crate::record::{self, RecordWriter, Topic};
use crate::replay::{self, TOPICS, bit};
use oxide_core::io::{Inputs, Outputs};
use oxide_core::params::LoadReport;
use oxide_core::schedule::{F405_LEAN, H743_FULL};
use oxide_core::time::Micros;

/// Writes one record of `topic` through `fill`; the record's whole message.
fn write(
    topic: &Topic,
    fill: impl FnOnce(&mut RecordWriter<'_, Vec<u8>>),
) -> Result<Vec<u8>, Error> {
    let mut v = Vec::new();
    record::data(&mut v, 7, topic, fill)?;
    Ok(v)
}

#[test]
fn every_topic_constant_passes_the_format_checker() {
    for t in &TOPICS {
        assert_topic(t);
    }
}

#[test]
fn the_topics_in_their_fixed_order_with_their_record_lengths() {
    let got: Vec<(&str, usize)> = TOPICS.iter().map(|t| (t.name, t.record_len)).collect();
    assert_eq!(
        got,
        [
            ("oxide_vehicle", 38),
            ("oxide_param_us", 16),
            ("oxide_boot", 47),
            ("oxide_in_tick", 20),
            ("oxide_in_imu", 48),
            ("oxide_in_state", 76),
            ("oxide_in_rc", 45),
            ("oxide_in_battery", 32),
            ("oxide_in_setpoint", 52),
            ("oxide_in_gps", 61),
            ("oxide_in_baro", 28),
            ("oxide_in_mag", 36),
            ("oxide_in_gcs", 24),
            ("oxide_in_brain", 28),
            ("oxide_in_param_set", 20),
            ("oxide_out", 82),
        ]
    );
    // The constants and the list are the same topics.
    assert_eq!(TOPICS[0], replay::VEHICLE);
    assert_eq!(TOPICS[1], replay::MICROS);
    assert_eq!(TOPICS[2], replay::BOOT);
    assert_eq!(TOPICS[3], replay::IN_TICK);
    assert_eq!(TOPICS[4], replay::IN_IMU);
    assert_eq!(TOPICS[5], replay::IN_STATE);
    assert_eq!(TOPICS[6], replay::IN_RC);
    assert_eq!(TOPICS[7], replay::IN_BATTERY);
    assert_eq!(TOPICS[8], replay::IN_SETPOINT);
    assert_eq!(TOPICS[9], replay::IN_GPS);
    assert_eq!(TOPICS[10], replay::IN_BARO);
    assert_eq!(TOPICS[11], replay::IN_MAG);
    assert_eq!(TOPICS[12], replay::IN_GCS);
    assert_eq!(TOPICS[13], replay::IN_BRAIN);
    assert_eq!(TOPICS[14], replay::IN_PARAM_SET);
    assert_eq!(TOPICS[15], replay::OUT);
}

#[test]
fn the_presence_bits_are_pinned() {
    assert_eq!(bit::IMU, 0);
    assert_eq!(bit::STATE, 1);
    assert_eq!(bit::RC, 2);
    assert_eq!(bit::BATTERY, 3);
    assert_eq!(bit::SETPOINT, 4);
    assert_eq!(bit::GPS, 5);
    assert_eq!(bit::BARO, 6);
    assert_eq!(bit::MAG, 7);
    assert_eq!(bit::GCS, 8);
    assert_eq!(bit::BRAIN, 9);
    assert_eq!(bit::PARAM_SET, 10);
    assert_eq!(bit::ALL, 0x07FF);
}

#[test]
fn the_param_kind_codes_are_pinned() {
    assert_eq!(replay::KIND_F32, 0);
    assert_eq!(replay::KIND_I32, 1);
}

#[test]
fn the_input_topic_of_each_bit() {
    let names = [
        "oxide_in_imu",
        "oxide_in_state",
        "oxide_in_rc",
        "oxide_in_battery",
        "oxide_in_setpoint",
        "oxide_in_gps",
        "oxide_in_baro",
        "oxide_in_mag",
        "oxide_in_gcs",
        "oxide_in_brain",
        "oxide_in_param_set",
    ];
    for (b, n) in names.iter().enumerate() {
        assert_eq!(replay::input_bit(n), Some(b as u8), "{n}");
    }
    assert_eq!(replay::input_bit("oxide_in_tick"), None);
    assert_eq!(replay::input_bit("oxide_out"), None);
    assert_eq!(replay::input_bit("nonsense"), None);
}

#[test]
fn present_mask_has_one_bit_per_input() {
    assert_eq!(replay::present_mask(&Inputs::default()), 0);
    assert_eq!(replay::present_mask(&full_inputs()), bit::ALL);
    let one = |f: fn(&mut Inputs)| {
        let mut i = Inputs::default();
        f(&mut i);
        replay::present_mask(&i)
    };
    assert_eq!(one(|i| i.imu = Some(imu())), 1 << bit::IMU);
    assert_eq!(one(|i| i.state = Some(state())), 1 << bit::STATE);
    assert_eq!(one(|i| i.rc = Some(rc())), 1 << bit::RC);
    assert_eq!(one(|i| i.battery = Some(battery())), 1 << bit::BATTERY);
    assert_eq!(one(|i| i.setpoint = Some(setpoint())), 1 << bit::SETPOINT);
    assert_eq!(one(|i| i.gps = Some(gps())), 1 << bit::GPS);
    assert_eq!(one(|i| i.baro = Some(baro())), 1 << bit::BARO);
    assert_eq!(one(|i| i.mag = Some(mag())), 1 << bit::MAG);
    assert_eq!(one(|i| i.gcs_heartbeat = Some(gcs())), 1 << bit::GCS);
    assert_eq!(one(|i| i.brain_heartbeat = Some(brain())), 1 << bit::BRAIN);
    assert_eq!(one(|i| i.param_set = Some(set_f32())), 1 << bit::PARAM_SET);
    // `param_ack` is a flag of the tick record, not an input record.
    assert_eq!(one(|i| i.param_ack = true), 0);
}

#[test]
fn every_encoder_writes_exactly_its_declared_length() {
    let t = Micros(42);
    let n = |topic: &Topic, r: Result<Vec<u8>, Error>| {
        let v = r.unwrap_or_else(|e| panic!("{}: {e:?}", topic.name));
        assert_eq!(v.len(), 3 + 2 + topic.record_len, "{}", topic.name);
    };
    n(
        &replay::VEHICLE,
        write(&replay::VEHICLE, |w| {
            replay::fill_vehicle_number(w, t, 3, "rotor15_torque_coeff_m", -0.0);
        }),
    );
    n(
        &replay::MICROS,
        write(&replay::MICROS, |w| {
            replay::fill_micros(w, t, "FS_RC_TIMEOUT", u32::MAX)
        }),
    );
    n(
        &replay::BOOT,
        write(&replay::BOOT, |w| {
            let _ = replay::fill_boot(w, t, &LoadReport::default(), 4, &F405_LEAN);
        }),
    );
    n(
        &replay::IN_TICK,
        write(&replay::IN_TICK, |w| {
            replay::fill_tick(w, t, 9, &full_inputs(), true);
        }),
    );
    n(
        &replay::IN_IMU,
        write(&replay::IN_IMU, |w| replay::fill_imu(w, t, &imu())),
    );
    n(
        &replay::IN_STATE,
        write(&replay::IN_STATE, |w| replay::fill_state(w, t, &state())),
    );
    n(
        &replay::IN_RC,
        write(&replay::IN_RC, |w| replay::fill_rc(w, t, &rc())),
    );
    n(
        &replay::IN_BATTERY,
        write(&replay::IN_BATTERY, |w| {
            replay::fill_battery(w, t, &battery())
        }),
    );
    n(
        &replay::IN_SETPOINT,
        write(&replay::IN_SETPOINT, |w| {
            replay::fill_setpoint(w, t, &setpoint())
        }),
    );
    n(
        &replay::IN_GPS,
        write(&replay::IN_GPS, |w| replay::fill_gps(w, t, &gps())),
    );
    n(
        &replay::IN_BARO,
        write(&replay::IN_BARO, |w| replay::fill_baro(w, t, &baro())),
    );
    n(
        &replay::IN_MAG,
        write(&replay::IN_MAG, |w| replay::fill_mag(w, t, &mag())),
    );
    n(
        &replay::IN_GCS,
        write(&replay::IN_GCS, |w| replay::fill_gcs(w, t, &gcs())),
    );
    n(
        &replay::IN_BRAIN,
        write(&replay::IN_BRAIN, |w| replay::fill_brain(w, t, &brain())),
    );
    for ps in [set_f32(), set_i32(), set_unknown(), set_wrong_kind()] {
        n(
            &replay::IN_PARAM_SET,
            write(&replay::IN_PARAM_SET, |w| replay::fill_param_set(w, t, &ps)),
        );
    }
    let out = Outputs::<4> {
        status: busy_status(),
        ..Outputs::off(Micros(u64::MAX))
    };
    n(
        &replay::OUT,
        write(&replay::OUT, |w| {
            let _ = replay::fill_out(w, t, &out);
        }),
    );
}

#[test]
fn the_encoders_write_the_exact_bits() {
    let t = Micros(0x0102_0304_0506_0708);
    let v = write(&replay::IN_IMU, |w| replay::fill_imu(w, t, &imu())).unwrap();
    let rec = &v[5..];
    assert_eq!(rec[..8], t.0.to_le_bytes(), "timestamp is the tick time");
    assert_eq!(rec[8..16], u64::MAX.to_le_bytes(), "t_sample");
    assert_eq!(
        rec[16..24],
        u64::MAX.wrapping_mul(3).wrapping_add(7).to_le_bytes(),
        "t_arrival"
    );
    assert_eq!(rec[24..28], 0x7FC0_1234u32.to_le_bytes(), "NaN payload");
    assert_eq!(rec[28..32], 0x8000_0000u32.to_le_bytes(), "negative zero");
    assert_eq!(rec[32..36], f32::INFINITY.to_bits().to_le_bytes());
    assert_eq!(rec[36..40], 1u32.to_le_bytes(), "subnormal");
}

#[test]
fn out_refuses_more_than_eight_motors_and_writes_nothing() {
    let out = Outputs::<9>::off(Micros(1));
    let mut v = Vec::new();
    let mut w = RecordWriter::new(&mut v, replay::OUT.record_len);
    let e = replay::fill_out(&mut w, Micros(1), &out).unwrap_err();
    assert_eq!((e.got, e.max), (9, 8));
    assert!(v.is_empty(), "nothing written for a refused record");
    // Eight is fine, and the unused entries are zero.
    let out8 = Outputs::<8> {
        motors: [0.5; 8],
        ..Outputs::off(Micros(1))
    };
    let v = write(&replay::OUT, |w| {
        replay::fill_out(w, Micros(1), &out8).unwrap();
    })
    .unwrap();
    assert_eq!(v.len(), 5 + 82);
}

#[test]
fn unused_motor_entries_are_zero_bits() {
    let out = Outputs::<4> {
        motors: [-0.0, 0.25, 0.5, f32::NAN],
        ..Outputs::off(Micros(5))
    };
    let v = write(&replay::OUT, |w| {
        replay::fill_out(w, Micros(0), &out).unwrap();
    })
    .unwrap();
    let rec = &v[5..];
    let m = |i: usize| u32::from_le_bytes(rec[16 + 4 * i..20 + 4 * i].try_into().unwrap());
    assert_eq!(m(0), 0x8000_0000);
    assert_eq!(m(1), 0.25f32.to_bits());
    assert_eq!(m(3), f32::NAN.to_bits());
    for i in 4..8 {
        assert_eq!(m(i), 0, "entry {i}");
    }
    assert_eq!(rec[64], 4, "motor_count");
}

#[test]
fn boot_refuses_a_rotor_count_over_255() {
    let mut v = Vec::new();
    let mut w = RecordWriter::new(&mut v, replay::BOOT.record_len);
    let e =
        replay::fill_boot(&mut w, Micros(0), &LoadReport::default(), 256, &H743_FULL).unwrap_err();
    assert_eq!((e.got, e.max), (256, 255));
    assert!(v.is_empty());
}

#[test]
fn vehicle_numbers_all_fit_their_name_field() {
    // Sixteen rotors is the most the configuration can name.
    let mut longest = 0;
    let mut count = 0;
    let cfg = oxide_core::config::VehicleConfig::<16>::from_vehicle_numbers({
        let mut names = Vec::new();
        quad16().vehicle_numbers(|n, v| names.push((n, v)));
        names
    })
    .unwrap();
    cfg.vehicle_numbers(|n, _| {
        longest = longest.max(n.len());
        count += 1;
    });
    assert_eq!(count, 8 + 5 * 16);
    assert!(longest <= replay::VEHICLE_NAME_LEN, "{longest}");
}

fn quad16() -> oxide_core::config::VehicleConfig<16> {
    let q = quad();
    let mut rotors = [q.rotors[0]; 16];
    for (i, r) in rotors.iter_mut().enumerate() {
        *r = q.rotors[i % 4];
    }
    oxide_core::config::VehicleConfig {
        mass_kg: q.mass_kg,
        inertia: q.inertia,
        rotors,
        thrust_expo: q.thrust_expo,
        nominal_voltage_v: q.nominal_voltage_v,
        gains: q.gains,
        limits: q.limits,
        estimator: q.estimator,
        failsafe: q.failsafe,
        battery_cells: q.battery_cells,
        battery_resistance_ohm: q.battery_resistance_ohm,
    }
}

#[test]
fn write_vehicle_writes_one_record_per_number() {
    let mut v = Vec::new();
    replay::write_vehicle(&mut v, 2, Micros(9), &quad()).unwrap();
    assert_eq!(v.len(), (8 + 5 * 4) * (5 + 38));
    let mut f = Vec::new();
    f.put(&v).unwrap();
    // Indices run 0.. in `vehicle_numbers` order, each record has msg id 2.
    let mut at = 0;
    let mut want = Vec::new();
    quad().vehicle_numbers(|n, x| want.push((n, x)));
    for (i, (n, x)) in want.iter().enumerate() {
        let rec = &v[at + 5..at + 5 + 38];
        assert_eq!(v[at + 3..at + 5], 2u16.to_le_bytes());
        assert_eq!(rec[..8], 9u64.to_le_bytes());
        assert_eq!(rec[8..12], x.to_bits().to_le_bytes());
        assert_eq!(rec[12..14], (i as u16).to_le_bytes());
        let name = &rec[14..38];
        let end = name.iter().position(|b| *b == 0).unwrap_or(24);
        assert_eq!(std::str::from_utf8(&name[..end]).unwrap(), *n);
        at += 43;
    }
}
