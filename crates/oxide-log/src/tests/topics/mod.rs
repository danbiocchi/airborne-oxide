// SPDX-License-Identifier: AGPL-3.0-or-later
//! Steps 1 and 2: the normal level's topics, their encoders and the scales.

mod oxide;
mod px4;

use crate::format::check_topic;
use crate::msg;
use crate::reader::{RecordView, UlogFile};
use crate::record::{self, RecordWriter, Topic};
use crate::topics::{self, NORMAL};
use oxide_core::config::{
    EstimatorConfig, FailsafeParams, Gains, Limits, Rotor, Spin, VehicleConfig,
};
use oxide_core::math::Vec3;

/// The reference quad of `airframes/quad-x-5in.toml` as the core is given it.
pub(crate) fn quad() -> VehicleConfig<4> {
    let a = 0.0795;
    let r = |x: f32, y: f32, spin| Rotor {
        x,
        y,
        spin,
        max_thrust_n: 8.0,
        torque_coeff_m: 0.016,
    };
    VehicleConfig {
        mass_kg: 0.65,
        inertia: Vec3::new(1.6e-3, 1.6e-3, 2.8e-3),
        rotors: [
            r(a, a, Spin::Ccw),
            r(-a, -a, Spin::Ccw),
            r(a, -a, Spin::Cw),
            r(-a, a, Spin::Cw),
        ],
        thrust_expo: 1.0,
        nominal_voltage_v: 22.2,
        gains: Gains::default(),
        limits: Limits::default(),
        estimator: EstimatorConfig::default(),
        failsafe: FailsafeParams::default(),
        battery_cells: 6,
        battery_resistance_ohm: 0.025,
    }
}

/// A log with one topic and one record of it, parsed. A record of the wrong
/// length makes this panic: the writer refuses it.
pub(crate) fn one_record(
    topic: &Topic,
    fill: impl FnOnce(&mut RecordWriter<'_, Vec<u8>>),
) -> UlogFile {
    let mut f = Vec::new();
    msg::file_header(&mut f, 0).unwrap();
    msg::flag_bits(&mut f, [0; 8], [0; 8]).unwrap();
    topic.write_format(&mut f).unwrap();
    topic.add_logged(&mut f, 0, 0).unwrap();
    record::data(&mut f, 0, topic, fill).unwrap();
    UlogFile::parse(&f).unwrap()
}

/// A float field, which must exist.
pub(crate) fn val(r: &RecordView<'_>, path: &str) -> f32 {
    r.f32(path)
        .unwrap_or_else(|| panic!("no float field {path}"))
}

/// An integer or boolean field of any width, which must exist.
pub(crate) fn int(r: &RecordView<'_>, path: &str) -> i64 {
    r.u64(path)
        .map(|v| v as i64)
        .or_else(|| r.u32(path).map(i64::from))
        .or_else(|| r.u16(path).map(i64::from))
        .or_else(|| r.u8(path).map(i64::from))
        .or_else(|| r.i32(path).map(i64::from))
        .or_else(|| r.bool(path).map(i64::from))
        .unwrap_or_else(|| panic!("no integer field {path}"))
}

/// Every float named equals its value (NaN equals NaN) within 1e-6, relative
/// above 1.
pub(crate) fn floats(r: &RecordView<'_>, want: &[(&str, f32)]) {
    for (path, w) in want {
        let g = val(r, path);
        if w.is_nan() {
            assert!(g.is_nan(), "{path}: {g} but NaN wanted");
        } else {
            let tol = 1e-6 * w.abs().max(1.0);
            assert!((g - w).abs() <= tol, "{path}: {g} but {w} wanted");
        }
    }
}

/// Every integer or boolean named equals its value.
pub(crate) fn ints(r: &RecordView<'_>, want: &[(&str, i64)]) {
    for (path, w) in want {
        assert_eq!(int(r, path), *w, "{path}");
    }
}

/// One data message of every normal topic, in list order, with values that
/// differ from zero. Message id `i` is the position in [`NORMAL`].
pub(crate) fn sample_records() -> Vec<Vec<u8>> {
    px4::all_samples()
        .into_iter()
        .chain(oxide::all_samples())
        .collect()
}

#[test]
fn every_topic_constant_passes_the_format_checker() {
    for t in NORMAL {
        let l = check_topic(t).unwrap_or_else(|e| panic!("{e}"));
        assert_eq!(l.size_no_trailing_padding, t.record_len, "{}", t.name);
    }
}

#[test]
fn the_list_has_the_topics_of_the_adr_in_a_fixed_order() {
    let names: Vec<&str> = NORMAL.iter().map(|t| t.name).collect();
    assert_eq!(
        names,
        [
            "sensor_combined",
            "vehicle_angular_velocity",
            "vehicle_rates_setpoint",
            "actuator_motors",
            "vehicle_thrust_setpoint",
            "vehicle_torque_setpoint",
            "vehicle_attitude",
            "vehicle_attitude_setpoint",
            "vehicle_local_position",
            "vehicle_local_position_setpoint",
            "manual_control_setpoint",
            "vehicle_magnetometer",
            "vehicle_air_data",
            "battery_status",
            "vehicle_status",
            "failsafe_flags",
            "oxide_status",
            "oxide_estimator",
            "oxide_event",
            "oxide_logger",
        ]
    );
    // A topic's id is its position; the named ids say the same.
    for (name, id) in topics::IDS.iter().copied() {
        assert_eq!(NORMAL[usize::from(id)].name, name);
    }
    assert_eq!(topics::IDS.len(), NORMAL.len());
}

#[test]
fn record_lengths_are_pinned() {
    let lens: Vec<(&str, usize)> = NORMAL.iter().map(|t| (t.name, t.record_len)).collect();
    assert_eq!(
        lens,
        [
            ("sensor_combined", 48),
            ("vehicle_angular_velocity", 40),
            ("vehicle_rates_setpoint", 33),
            ("actuator_motors", 66),
            ("vehicle_thrust_setpoint", 28),
            ("vehicle_torque_setpoint", 28),
            ("vehicle_attitude", 49),
            ("vehicle_attitude_setpoint", 40),
            ("vehicle_local_position", 192),
            ("vehicle_local_position_setpoint", 64),
            ("manual_control_setpoint", 70),
            ("vehicle_magnetometer", 33),
            ("vehicle_air_data", 38),
            ("battery_status", 166),
            ("vehicle_status", 78),
            ("failsafe_flags", 101),
            ("oxide_status", 33),
            ("oxide_estimator", 88),
            ("oxide_event", 39),
            ("oxide_logger", 40),
        ]
    );
}

#[test]
fn every_encoder_writes_exactly_its_declared_length() {
    let samples = sample_records();
    assert_eq!(samples.len(), NORMAL.len());
    for (t, bytes) in NORMAL.iter().zip(&samples) {
        // 3-byte message header, 2-byte message id, then the record.
        assert_eq!(bytes.len(), 5 + t.record_len, "{}", t.name);
        assert_eq!(bytes[2], b'D');
    }
}

#[test]
fn nested_formats_are_listed_and_there_are_none_for_these_topics() {
    // No topic of the normal level nests another type; the list exists for
    // the ones that will (`vehicle_gnss`).
    assert!(topics::NESTED.is_empty());
}

mod scales {
    use super::*;
    use topics::{Scales, torque_limits};

    fn rotor(x: f32, y: f32, spin: Spin, t: f32, k: f32) -> Rotor {
        Rotor {
            x,
            y,
            spin,
            max_thrust_n: t,
            torque_coeff_m: k,
        }
    }

    #[test]
    fn the_reference_quad_by_hand() {
        // Roll: the two rotors on the side that pushes the right side down
        // (y < 0) at 8 N, the other two off: 2 · 0.0795 · 8 = 1.272 N·m.
        // Pitch likewise. Yaw: the two rotors of one spin at 8 N, the others
        // off: 2 · 0.016 · 8 = 0.256 N·m. Thrust: 4 · 8 = 32 N.
        let s = Scales::new(&quad());
        assert_eq!(s.thrust_max_n, 32.0);
        for (got, want) in [
            (s.torque_max_nm.x, 1.272),
            (s.torque_max_nm.y, 1.272),
            (s.torque_max_nm.z, 0.256),
        ] {
            assert!((got - want).abs() < 1e-6, "{got} {want}");
        }
        assert_eq!(s.battery_cells, 6);
        assert_eq!(s.nominal_voltage_v, 22.2);
        assert!((s.cell_resistance_ohm - 0.025 / 6.0).abs() < 1e-9);
    }

    #[test]
    fn an_odd_layout_by_hand() {
        // Roll coefficients -y: (-0.2, -0.2, +0.4): up 0.4 · 10 = 4, down
        // 0.4 · 10 = 4. Pitch x: (0.1, -0.1, 0): 1 and 1. Yaw ±k with spins
        // (+, -, +): up 0.02 · 10 · 2 = 0.4, down 0.2: the larger, 0.4.
        let rotors = [
            rotor(0.1, 0.2, Spin::Ccw, 10.0, 0.02),
            rotor(-0.1, 0.2, Spin::Cw, 10.0, 0.02),
            rotor(0.0, -0.4, Spin::Ccw, 10.0, 0.02),
        ];
        let t = torque_limits(&rotors);
        for (got, want) in [(t.x, 4.0), (t.y, 1.0), (t.z, 0.4)] {
            assert!((got - want).abs() < 1e-5, "{got} {want}");
        }
    }

    #[test]
    fn no_rotors_or_bad_numbers_give_zero_not_nan() {
        let t = torque_limits::<0>(&[]);
        assert_eq!((t.x, t.y, t.z), (0.0, 0.0, 0.0));
        let bad = [rotor(f32::NAN, 0.1, Spin::Ccw, 5.0, f32::INFINITY)];
        let t = torque_limits(&bad);
        assert!(t.x.is_finite() && t.y.is_finite() && t.z.is_finite());
    }
}
