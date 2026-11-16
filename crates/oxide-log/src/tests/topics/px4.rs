// SPDX-License-Identifier: AGPL-3.0-or-later
//! One table test per PX4-named topic: a hand-made sample goes in, literal
//! values are read back with the reader.

use super::{floats, int, ints, one_record, quad, val};
use crate::record::{self, RecordWriter};
use crate::topics::{self, NORMAL, Scales};
use oxide_core::control::{HorizontalForm, VerticalForm, YawForm};
use oxide_core::flight::Setpoints;
use oxide_core::io::{
    Action, BaroSample, BatterySample, FlightMode, ImuSample, MagSample, NavState, PrearmFlags,
    RcSample, Reasons, Stamped, StateSample, Status, Validity,
};
use oxide_core::math::{Quat, Vec3};
use oxide_core::time::Micros;

const NAN: f32 = f32::NAN;

fn scales() -> Scales {
    Scales::new(&quad())
}

fn topic(name: &str) -> &'static crate::record::Topic {
    NORMAL.iter().find(|t| t.name == name).unwrap()
}

fn v3(x: f32, y: f32, z: f32) -> Vec3<f32> {
    Vec3::new(x, y, z)
}

fn q(w: f32, x: f32, y: f32, z: f32) -> Quat<f32> {
    Quat { w, x, y, z }
}

/// Every PX4-named topic once, in list order, in a plausible state.
pub(super) fn all_samples() -> Vec<Vec<u8>> {
    let sc = scales();
    let now = Micros(2_000_000);
    let ts = Micros(1_999_000);
    let est = StateSample {
        position: v3(1.0, 2.0, -3.0),
        velocity: v3(0.5, -0.25, 1.5),
        attitude: q(0.9238795, 0.3826834, 0.0, 0.0),
        rate: v3(0.1, -0.2, 0.3),
    };
    let status = Status {
        armed: true,
        mode: FlightMode::Position,
        nav: NavState::Position,
        valid: Validity::ALL,
        ..Status::default()
    };
    let sp = Setpoints {
        position: v3(1.0, 2.0, -3.0),
        velocity: Some(v3(0.1, 0.2, 0.3)),
        yaw: 0.5,
        horizontal: HorizontalForm::Position,
        vertical: VerticalForm::Position,
        yaw_form: YawForm::Heading,
    };
    let mut out = Vec::new();
    let mut put = |name: &str, fill: &mut dyn FnMut(&mut RecordWriter<'_, Vec<u8>>)| {
        let id = topics::IDS.iter().find(|(n, _)| *n == name).unwrap().1;
        let mut v = Vec::new();
        record::data(&mut v, id, topic(name), |w| fill(w)).unwrap();
        out.push(v);
    };
    let imu = Stamped::ideal(
        ts,
        ImuSample {
            gyro: v3(0.5, -1.5, 2.25),
            accel: v3(0.125, -0.25, -9.75),
        },
    );
    put("sensor_combined", &mut |w| {
        topics::sensor_combined(w, now, &imu, 1000)
    });
    put("vehicle_angular_velocity", &mut |w| {
        topics::vehicle_angular_velocity(w, now, ts, est.rate)
    });
    put("vehicle_rates_setpoint", &mut |w| {
        topics::vehicle_rates_setpoint(w, now, v3(1.0, -2.0, 0.5), 16.0, &sc)
    });
    put("actuator_motors", &mut |w| {
        topics::actuator_motors(w, now, ts, &[0.25, 0.5, 0.75, 1.0])
    });
    put("vehicle_thrust_setpoint", &mut |w| {
        topics::vehicle_thrust_setpoint(w, now, ts, 8.0, &sc)
    });
    put("vehicle_torque_setpoint", &mut |w| {
        topics::vehicle_torque_setpoint(w, now, ts, v3(0.1, -0.2, 0.05), &sc)
    });
    put("vehicle_attitude", &mut |w| {
        topics::vehicle_attitude(w, now, ts, est.attitude)
    });
    put("vehicle_attitude_setpoint", &mut |w| {
        topics::vehicle_attitude_setpoint(w, now, est.attitude, 24.0, &sc)
    });
    put("vehicle_local_position", &mut |w| {
        topics::vehicle_local_position(w, now, ts, &est, &Validity::ALL, true)
    });
    put("vehicle_local_position_setpoint", &mut |w| {
        topics::vehicle_local_position_setpoint(w, now, Some(&sp), est.attitude, 16.0, &sc)
    });
    let rc = Stamped::ideal(
        ts,
        RcSample {
            roll: 0.5,
            throttle: 0.75,
            arm_switch: true,
            mode_switch: 1,
            ..RcSample::default()
        },
    );
    put("manual_control_setpoint", &mut |w| {
        topics::manual_control_setpoint(w, now, &rc)
    });
    let mag = Stamped::ideal(
        ts,
        MagSample {
            field_ut: v3(25.0, -5.0, 43.3),
        },
    );
    put("vehicle_magnetometer", &mut |w| {
        topics::vehicle_magnetometer(w, now, &mag)
    });
    let baro = Stamped::ideal(ts, BaroSample { altitude_m: 12.5 });
    put("vehicle_air_data", &mut |w| {
        topics::vehicle_air_data(w, now, &baro)
    });
    let bat = BatterySample {
        voltage_v: 22.2,
        current_a: 15.5,
    };
    put("battery_status", &mut |w| {
        topics::battery_status(w, now, &bat, false, false, &sc)
    });
    put("vehicle_status", &mut |w| {
        topics::vehicle_status(w, now, &status)
    });
    put("failsafe_flags", &mut |w| {
        topics::failsafe_flags(w, now, &status)
    });
    out
}

#[test]
fn sensor_combined() {
    let imu = Stamped {
        t_sample: Micros(1_000_000),
        t_arrival: Micros(1_000_004),
        value: ImuSample {
            gyro: v3(0.5, -1.5, 2.25),
            accel: v3(0.125, -0.25, -9.75),
        },
    };
    let f = one_record(topic("sensor_combined"), |w| {
        topics::sensor_combined(w, Micros(1_000_010), &imu, 1000)
    });
    let r = f.records("sensor_combined").next().unwrap();
    floats(
        &r,
        &[
            ("gyro_rad[0]", 0.5),
            ("gyro_rad[1]", -1.5),
            ("gyro_rad[2]", 2.25),
            ("accelerometer_m_s2[0]", 0.125),
            ("accelerometer_m_s2[1]", -0.25),
            ("accelerometer_m_s2[2]", -9.75),
        ],
    );
    assert_eq!(r.u64("timestamp"), Some(1_000_010));
    assert_eq!(r.i32("accelerometer_timestamp_relative"), Some(-10));
    ints(
        &r,
        &[
            ("gyro_integral_dt", 1000),
            ("accelerometer_integral_dt", 1000),
            ("accelerometer_clipping", 0),
            ("gyro_clipping", 0),
            ("accel_calibration_count", 0),
            ("gyro_calibration_count", 0),
        ],
    );
}

#[test]
fn sensor_combined_marks_a_sample_time_it_cannot_express_as_invalid() {
    // PX4's RELATIVE_TIMESTAMP_INVALID is 0x7fffffff.
    let far = |t_sample: u64, now: u64| {
        let imu = Stamped::ideal(Micros(t_sample), ImuSample::default());
        let f = one_record(topic("sensor_combined"), |w| {
            topics::sensor_combined(w, Micros(now), &imu, 0)
        });
        f.records("sensor_combined")
            .next()
            .unwrap()
            .i32("accelerometer_timestamp_relative")
            .unwrap()
    };
    assert_eq!(far(0, 10_000_000_000), i32::MAX);
    assert_eq!(far(10_000_000_000, 0), i32::MAX);
    assert_eq!(far(5, 5), 0);
    assert_eq!(far(0, 2_147_483_647), -2_147_483_647);
    assert_eq!(far(2_147_483_646, 0), 2_147_483_646);
    assert_eq!(
        far(2_147_483_647, 0),
        i32::MAX,
        "the marker's own value is not a time"
    );
}

#[test]
fn vehicle_angular_velocity() {
    let f = one_record(topic("vehicle_angular_velocity"), |w| {
        topics::vehicle_angular_velocity(w, Micros(5_000), Micros(4_000), v3(0.1, -0.2, 0.3))
    });
    let r = f.records("vehicle_angular_velocity").next().unwrap();
    assert_eq!(r.u64("timestamp"), Some(5_000));
    assert_eq!(r.u64("timestamp_sample"), Some(4_000));
    floats(
        &r,
        &[
            ("xyz[0]", 0.1),
            ("xyz[1]", -0.2),
            ("xyz[2]", 0.3),
            ("xyz_derivative[0]", 0.0),
            ("xyz_derivative[1]", 0.0),
            ("xyz_derivative[2]", 0.0),
        ],
    );
}

#[test]
fn vehicle_rates_setpoint_thrust_is_normalised_and_negative_up() {
    // 16 N of a 32 N total: half, on the negative body z axis.
    let f = one_record(topic("vehicle_rates_setpoint"), |w| {
        topics::vehicle_rates_setpoint(w, Micros(7_000), v3(1.0, -2.0, 0.5), 16.0, &scales())
    });
    let r = f.records("vehicle_rates_setpoint").next().unwrap();
    assert_eq!(r.u64("timestamp"), Some(7_000));
    floats(
        &r,
        &[
            ("roll", 1.0),
            ("pitch", -2.0),
            ("yaw", 0.5),
            ("thrust_body[0]", 0.0),
            ("thrust_body[1]", 0.0),
            ("thrust_body[2]", -0.5),
        ],
    );
    assert_eq!(r.bool("reset_integral"), Some(false));
    assert!(
        r.field_bytes("_padding0").is_none(),
        "trailing padding is not in a record"
    );
}

#[test]
fn actuator_motors_unused_channels_are_nan() {
    let f = one_record(topic("actuator_motors"), |w| {
        topics::actuator_motors(w, Micros(9_000), Micros(8_000), &[0.25, 0.5, 0.75, 1.0])
    });
    let r = f.records("actuator_motors").next().unwrap();
    assert_eq!(r.u64("timestamp"), Some(9_000));
    assert_eq!(r.u64("timestamp_sample"), Some(8_000));
    let mut want = vec![
        ("control[0]", 0.25),
        ("control[1]", 0.5),
        ("control[2]", 0.75),
        ("control[3]", 1.0),
    ];
    let names: Vec<String> = (4..12).map(|i| format!("control[{i}]")).collect();
    want.extend(names.iter().map(|n| (n.as_str(), NAN)));
    floats(&r, &want);
    assert_eq!(r.u16("reversible_flags"), Some(0));
}

#[test]
fn actuator_motors_beyond_twelve_are_cut_not_overrun() {
    let m: Vec<f32> = (0..14).map(|i| i as f32 / 16.0).collect();
    let f = one_record(topic("actuator_motors"), |w| {
        topics::actuator_motors(w, Micros(1), Micros(1), &m)
    });
    let r = f.records("actuator_motors").next().unwrap();
    assert_eq!(val(&r, "control[11]"), 11.0 / 16.0);
}

#[test]
fn thrust_and_torque_setpoints_are_normalised() {
    let f = one_record(topic("vehicle_thrust_setpoint"), |w| {
        topics::vehicle_thrust_setpoint(w, Micros(3_000), Micros(2_000), 8.0, &scales())
    });
    let r = f.records("vehicle_thrust_setpoint").next().unwrap();
    assert_eq!(r.u64("timestamp"), Some(3_000));
    assert_eq!(r.u64("timestamp_sample"), Some(2_000));
    floats(&r, &[("xyz[0]", 0.0), ("xyz[1]", 0.0), ("xyz[2]", -0.25)]);

    // Half of each axis' limit, and a full negative one.
    let s = scales();
    let t = v3(
        0.5 * s.torque_max_nm.x,
        -s.torque_max_nm.y,
        0.25 * s.torque_max_nm.z,
    );
    let f = one_record(topic("vehicle_torque_setpoint"), |w| {
        topics::vehicle_torque_setpoint(w, Micros(3_000), Micros(2_000), t, &s)
    });
    let r = f.records("vehicle_torque_setpoint").next().unwrap();
    floats(&r, &[("xyz[0]", 0.5), ("xyz[1]", -1.0), ("xyz[2]", 0.25)]);
}

#[test]
fn a_zero_scale_writes_zero_never_nan_or_infinity() {
    let zero = Scales {
        thrust_max_n: 0.0,
        torque_max_nm: Vec3::new(0.0, 0.0, 0.0),
        ..scales()
    };
    let f = one_record(topic("vehicle_thrust_setpoint"), |w| {
        topics::vehicle_thrust_setpoint(w, Micros(1), Micros(1), 8.0, &zero)
    });
    let r = f.records("vehicle_thrust_setpoint").next().unwrap();
    for i in 0..3 {
        assert_eq!(val(&r, &format!("xyz[{i}]")).to_bits(), 0, "positive zero");
    }
    let f = one_record(topic("vehicle_torque_setpoint"), |w| {
        topics::vehicle_torque_setpoint(w, Micros(1), Micros(1), v3(1.0, -1.0, 1.0), &zero)
    });
    let r = f.records("vehicle_torque_setpoint").next().unwrap();
    for i in 0..3 {
        assert_eq!(val(&r, &format!("xyz[{i}]")).to_bits(), 0);
    }
    let f = one_record(topic("vehicle_attitude_setpoint"), |w| {
        topics::vehicle_attitude_setpoint(w, Micros(1), Quat::default(), 8.0, &zero)
    });
    let r = f.records("vehicle_attitude_setpoint").next().unwrap();
    assert_eq!(val(&r, "thrust_body[2]").to_bits(), 0);
}

#[test]
fn zero_thrust_is_positive_zero() {
    let f = one_record(topic("vehicle_thrust_setpoint"), |w| {
        topics::vehicle_thrust_setpoint(w, Micros(1), Micros(1), 0.0, &scales())
    });
    let r = f.records("vehicle_thrust_setpoint").next().unwrap();
    assert_eq!(val(&r, "xyz[2]").to_bits(), 0);
}

#[test]
fn vehicle_attitude_is_w_first_body_to_world() {
    let f = one_record(topic("vehicle_attitude"), |w| {
        topics::vehicle_attitude(w, Micros(6_000), Micros(5_000), q(0.5, -0.25, 0.75, -1.0))
    });
    let r = f.records("vehicle_attitude").next().unwrap();
    assert_eq!(r.u64("timestamp"), Some(6_000));
    assert_eq!(r.u64("timestamp_sample"), Some(5_000));
    floats(
        &r,
        &[
            ("q[0]", 0.5),
            ("q[1]", -0.25),
            ("q[2]", 0.75),
            ("q[3]", -1.0),
            ("delta_q_reset[0]", 0.0),
            ("delta_q_reset[3]", 0.0),
        ],
    );
    assert_eq!(r.u8("quat_reset_counter"), Some(0));
    assert_eq!(r.raw().len(), 49);
}

#[test]
fn vehicle_attitude_setpoint() {
    // 24 of 32 N: three quarters.
    let f = one_record(topic("vehicle_attitude_setpoint"), |w| {
        topics::vehicle_attitude_setpoint(w, Micros(4_000), q(0.5, 0.5, 0.5, 0.5), 24.0, &scales())
    });
    let r = f.records("vehicle_attitude_setpoint").next().unwrap();
    assert_eq!(r.u64("timestamp"), Some(4_000));
    floats(
        &r,
        &[
            ("yaw_sp_move_rate", 0.0),
            ("q_d[0]", 0.5),
            ("q_d[1]", 0.5),
            ("q_d[2]", 0.5),
            ("q_d[3]", 0.5),
            ("thrust_body[0]", 0.0),
            ("thrust_body[1]", 0.0),
            ("thrust_body[2]", -0.75),
        ],
    );
}

fn local_position(valid: Validity, gps_aiding: bool) -> crate::reader::UlogFile {
    // Yaw 90 degrees: heading pi/2.
    let h = core::f32::consts::FRAC_1_SQRT_2;
    let est = StateSample {
        position: v3(1.0, 2.0, -3.0),
        velocity: v3(0.5, -0.25, 1.5),
        attitude: q(h, 0.0, 0.0, h),
        rate: Vec3::default(),
    };
    one_record(topic("vehicle_local_position"), |w| {
        topics::vehicle_local_position(w, Micros(8_000), Micros(7_000), &est, &valid, gps_aiding)
    })
}

#[test]
fn vehicle_local_position_values_and_validity() {
    let valid = Validity {
        attitude: true,
        heading: true,
        altitude: true,
        velocity: false,
        position: true,
    };
    let f = local_position(valid, false);
    let r = f.records("vehicle_local_position").next().unwrap();
    assert_eq!(r.u64("timestamp"), Some(8_000));
    assert_eq!(r.u64("timestamp_sample"), Some(7_000));
    floats(
        &r,
        &[
            ("x", 1.0),
            ("y", 2.0),
            ("z", -3.0),
            ("vx", 0.5),
            ("vy", -0.25),
            ("vz", 1.5),
            ("z_deriv", 1.5),
            ("heading", core::f32::consts::FRAC_PI_2),
            ("ax", 0.0),
            ("eph", 0.0),
            ("ref_alt", 0.0),
        ],
    );
    assert_eq!(r.f64("ref_lat"), Some(0.0));
    ints(
        &r,
        &[
            ("xy_valid", 1),
            ("z_valid", 1),
            ("v_xy_valid", 0),
            ("v_z_valid", 1),
            ("heading_good_for_control", 1),
            ("xy_global", 0),
            ("z_global", 0),
            ("dist_bottom_valid", 0),
            ("dead_reckoning", 1),
            ("xy_reset_counter", 0),
        ],
    );
    assert_eq!(r.raw().len(), 192);
}

#[test]
fn vehicle_local_position_vertical_speed_validity_is_inferred() {
    // No flag for vertical speed: valid when either height or velocity is.
    let only = |altitude, velocity| {
        let f = local_position(
            Validity {
                altitude,
                velocity,
                ..Validity::NONE
            },
            true,
        );
        let r = f.records("vehicle_local_position").next().unwrap();
        (int(&r, "v_z_valid"), int(&r, "dead_reckoning"))
    };
    assert_eq!(only(false, false), (0, 0));
    assert_eq!(only(true, false), (1, 0));
    assert_eq!(only(false, true), (1, 0));
}

fn setpoint_record(sp: Option<&Setpoints>, q_d: Quat<f32>, thrust: f32) -> crate::reader::UlogFile {
    one_record(topic("vehicle_local_position_setpoint"), |w| {
        topics::vehicle_local_position_setpoint(w, Micros(11_000), sp, q_d, thrust, &scales())
    })
}

fn sp(h: HorizontalForm, v: VerticalForm, y: YawForm, vel: Option<Vec3<f32>>) -> Setpoints {
    Setpoints {
        position: v3(1.0, 2.0, -3.0),
        velocity: vel,
        yaw: 0.5,
        horizontal: h,
        vertical: v,
        yaw_form: y,
    }
}

#[test]
fn vehicle_local_position_setpoint_nan_means_not_controlled() {
    let all = sp(
        HorizontalForm::Position,
        VerticalForm::Position,
        YawForm::Heading,
        Some(v3(0.1, 0.2, 0.3)),
    );
    let f = setpoint_record(Some(&all), Quat::default(), 16.0);
    let r = f.records("vehicle_local_position_setpoint").next().unwrap();
    assert_eq!(r.u64("timestamp"), Some(11_000));
    floats(
        &r,
        &[
            ("x", 1.0),
            ("y", 2.0),
            ("z", -3.0),
            ("vx", 0.1),
            ("vy", 0.2),
            ("vz", 0.3),
            ("yaw", 0.5),
            ("acceleration[0]", NAN),
            ("acceleration[2]", NAN),
            ("yawspeed", NAN),
            ("thrust[0]", 0.0),
            ("thrust[1]", 0.0),
            ("thrust[2]", -0.5),
        ],
    );

    // Velocity forms: no position, velocity yes; free yaw: no yaw.
    let vel = sp(
        HorizontalForm::Velocity,
        VerticalForm::Velocity,
        YawForm::Free,
        Some(v3(0.1, 0.2, 0.3)),
    );
    let f = setpoint_record(Some(&vel), Quat::default(), 16.0);
    let r = f.records("vehicle_local_position_setpoint").next().unwrap();
    floats(
        &r,
        &[
            ("x", NAN),
            ("y", NAN),
            ("z", NAN),
            ("vx", 0.1),
            ("vy", 0.2),
            ("vz", 0.3),
            ("yaw", NAN),
        ],
    );

    // Tilt and thrust: nothing is controlled but the attitude.
    let tilt = sp(
        HorizontalForm::Tilt,
        VerticalForm::Thrust,
        YawForm::Heading,
        None,
    );
    let f = setpoint_record(Some(&tilt), Quat::default(), 16.0);
    let r = f.records("vehicle_local_position_setpoint").next().unwrap();
    floats(
        &r,
        &[
            ("x", NAN),
            ("y", NAN),
            ("z", NAN),
            ("vx", NAN),
            ("vy", NAN),
            ("vz", NAN),
            ("yaw", 0.5),
            ("thrust[2]", -0.5),
        ],
    );

    // Mixed: horizontal position, vertical speed.
    let mixed = sp(
        HorizontalForm::Position,
        VerticalForm::Velocity,
        YawForm::Heading,
        Some(v3(0.1, 0.2, 0.3)),
    );
    let f = setpoint_record(Some(&mixed), Quat::default(), 16.0);
    let r = f.records("vehicle_local_position_setpoint").next().unwrap();
    floats(&r, &[("x", 1.0), ("y", 2.0), ("z", NAN), ("vz", 0.3)]);
}

#[test]
fn vehicle_local_position_setpoint_thrust_is_a_ned_vector() {
    // The target rolled 90 degrees about north: body down is east, so the
    // thrust (negative body z, half of the total) points east.
    let h = core::f32::consts::FRAC_1_SQRT_2;
    let all = sp(
        HorizontalForm::Position,
        VerticalForm::Position,
        YawForm::Heading,
        None,
    );
    let f = setpoint_record(Some(&all), q(h, h, 0.0, 0.0), 16.0);
    let r = f.records("vehicle_local_position_setpoint").next().unwrap();
    floats(
        &r,
        &[("thrust[0]", 0.0), ("thrust[1]", 0.5), ("thrust[2]", 0.0)],
    );
}

#[test]
fn vehicle_local_position_setpoint_without_guidance_is_all_nan() {
    let f = setpoint_record(None, Quat::default(), 16.0);
    let r = f.records("vehicle_local_position_setpoint").next().unwrap();
    for p in [
        "x",
        "y",
        "z",
        "vx",
        "vy",
        "vz",
        "yaw",
        "yawspeed",
        "thrust[0]",
        "thrust[2]",
        "acceleration[1]",
    ] {
        assert!(val(&r, p).is_nan(), "{p}");
    }
}

#[test]
fn manual_control_setpoint_throttle_is_zero_centred() {
    let rc = |rc: RcSample| {
        let s = Stamped {
            t_sample: Micros(90_000),
            t_arrival: Micros(90_500),
            value: rc,
        };
        one_record(topic("manual_control_setpoint"), |w| {
            topics::manual_control_setpoint(w, Micros(100_000), &s)
        })
    };
    let f = rc(RcSample {
        roll: 0.5,
        pitch: -0.25,
        yaw: 1.0,
        throttle: 0.75,
        arm_switch: true,
        kill_switch: false,
        mode_switch: 2,
        failsafe: false,
        frame_lost: false,
    });
    let r = f.records("manual_control_setpoint").next().unwrap();
    assert_eq!(r.u64("timestamp"), Some(100_000));
    assert_eq!(r.u64("timestamp_sample"), Some(90_000));
    floats(
        &r,
        &[
            ("roll", 0.5),
            ("pitch", -0.25),
            ("yaw", 1.0),
            ("throttle", 0.5),
            ("aux1", 1.0),
            ("aux2", 1.0),
            ("aux3", -1.0),
            ("aux4", NAN),
            ("aux5", NAN),
            ("aux6", NAN),
            ("flaps", NAN),
        ],
    );
    ints(
        &r,
        &[
            ("valid", 1),
            ("data_source", 1),
            ("sticks_moving", 0),
            ("buttons", 0),
            ("source_system_id", 0),
        ],
    );

    // Throttle ends, the mode switch positions (0 low, 1 middle, anything
    // higher high), the kill switch.
    let cases = [
        (0.0_f32, 0_u8, true, -1.0_f32, 1.0_f32),
        (1.0, 1, false, 0.0, -1.0),
        (0.5, 7, false, 1.0, -1.0),
    ];
    for (t, m, kill, aux1, aux3) in cases {
        let f = rc(RcSample {
            throttle: t,
            mode_switch: m,
            kill_switch: kill,
            ..RcSample::default()
        });
        let r = f.records("manual_control_setpoint").next().unwrap();
        floats(
            &r,
            &[("throttle", 2.0 * t - 1.0), ("aux1", aux1), ("aux3", aux3)],
        );
    }
    let f = rc(RcSample {
        failsafe: true,
        ..RcSample::default()
    });
    assert_eq!(
        int(
            &f.records("manual_control_setpoint").next().unwrap(),
            "valid"
        ),
        0
    );
    let f = rc(RcSample {
        frame_lost: true,
        ..RcSample::default()
    });
    assert_eq!(
        int(
            &f.records("manual_control_setpoint").next().unwrap(),
            "valid"
        ),
        0
    );
}

#[test]
fn vehicle_magnetometer_is_in_gauss() {
    let s = Stamped {
        t_sample: Micros(40_000),
        t_arrival: Micros(41_000),
        value: MagSample {
            field_ut: v3(25.0, -5.0, 43.3),
        },
    };
    let f = one_record(topic("vehicle_magnetometer"), |w| {
        topics::vehicle_magnetometer(w, Micros(50_000), &s)
    });
    let r = f.records("vehicle_magnetometer").next().unwrap();
    assert_eq!(r.u64("timestamp"), Some(50_000));
    assert_eq!(r.u64("timestamp_sample"), Some(40_000));
    floats(
        &r,
        &[
            ("magnetometer_ga[0]", 0.25),
            ("magnetometer_ga[1]", -0.05),
            ("magnetometer_ga[2]", 0.433),
        ],
    );
    ints(&r, &[("device_id", 0), ("calibration_count", 0)]);
}

#[test]
fn vehicle_air_data_carries_the_altitude_only() {
    let s = Stamped {
        t_sample: Micros(40_000),
        t_arrival: Micros(41_000),
        value: BaroSample { altitude_m: 12.5 },
    };
    let f = one_record(topic("vehicle_air_data"), |w| {
        topics::vehicle_air_data(w, Micros(50_000), &s)
    });
    let r = f.records("vehicle_air_data").next().unwrap();
    assert_eq!(r.u64("timestamp"), Some(50_000));
    assert_eq!(r.u64("timestamp_sample"), Some(40_000));
    floats(
        &r,
        &[
            ("baro_alt_meter", 12.5),
            ("baro_pressure_pa", 0.0),
            ("ambient_temperature", 0.0),
            ("rho", 0.0),
        ],
    );
}

fn battery(low: bool, critical: bool) -> crate::reader::UlogFile {
    let s = BatterySample {
        voltage_v: 22.2,
        current_a: 15.5,
    };
    one_record(topic("battery_status"), |w| {
        topics::battery_status(w, Micros(60_000), &s, low, critical, &scales())
    })
}

#[test]
fn battery_status_values_and_pxs_invalid_markers() {
    let f = battery(false, false);
    let r = f.records("battery_status").next().unwrap();
    assert_eq!(r.u64("timestamp"), Some(60_000));
    floats(
        &r,
        &[
            ("voltage_v", 22.2),
            ("current_a", 15.5),
            ("current_average_a", -1.0),
            ("discharged_mah", -1.0),
            ("remaining", -1.0),
            ("scale", -1.0),
            ("time_remaining_s", NAN),
            ("temperature", NAN),
            ("voltage_cell_v[0]", 0.0),
            ("voltage_cell_v[13]", 0.0),
            ("remaining_capacity_wh", NAN),
            ("nominal_voltage", 22.2),
            ("internal_resistance_estimate", 0.025 / 6.0),
        ],
    );
    ints(
        &r,
        &[
            ("connected", 1),
            ("cell_count", 6),
            ("id", 1),
            ("source", 0),
            ("warning", 0),
            ("capacity", 0),
        ],
    );
    assert_eq!(r.raw().len(), 166);
}

#[test]
fn battery_status_warning_follows_the_latches() {
    let w = |low, critical| {
        int(
            &battery(low, critical)
                .records("battery_status")
                .next()
                .unwrap(),
            "warning",
        )
    };
    assert_eq!(w(false, false), 0);
    assert_eq!(w(true, false), 1);
    assert_eq!(w(false, true), 2);
    assert_eq!(w(true, true), 2);
}

fn status_record(s: &Status) -> crate::reader::UlogFile {
    one_record(topic("vehicle_status"), |w| {
        topics::vehicle_status(w, Micros(70_000), s)
    })
}

#[test]
fn vehicle_status_uses_px4_numbers() {
    let s = Status {
        armed: true,
        mode: FlightMode::Position,
        nav: NavState::Hold,
        action: Action::Hold,
        ..Status::default()
    };
    let f = status_record(&s);
    let r = f.records("vehicle_status").next().unwrap();
    assert_eq!(r.u64("timestamp"), Some(70_000));
    ints(
        &r,
        &[
            ("arming_state", 2),
            ("nav_state", 4),
            ("nav_state_user_intention", 2),
            ("nav_state_display", 4),
            ("failsafe", 1),
            ("failsafe_and_user_took_over", 0),
            ("vehicle_type", 1),
            ("system_type", 2),
            ("is_vtol", 0),
            ("hil_state", 0),
            ("pre_flight_checks_pass", 1),
            ("armed_time", 0),
            ("takeoff_time", 0),
            ("nav_state_timestamp", 0),
            ("gcs_connection_lost", 0),
        ],
    );
    assert_eq!(r.raw().len(), 78);

    // Every nav state with its PX4 number (armed, mode irrelevant).
    let table = [
        (NavState::Stabilized, 15),
        (NavState::Altitude, 1),
        (NavState::Position, 2),
        (NavState::Offboard, 14),
        (NavState::Hold, 4),
        (NavState::Rtl, 5),
        (NavState::Land, 18),
        (NavState::LandNow, 12),
        (NavState::Terminated, 13),
    ];
    for (nav, px4) in table {
        let f = status_record(&Status {
            armed: true,
            nav,
            mode: FlightMode::Altitude,
            ..Status::default()
        });
        let r = f.records("vehicle_status").next().unwrap();
        ints(&r, &[("nav_state", px4), ("nav_state_user_intention", 1)]);
    }
}

#[test]
fn vehicle_status_disarmed_shows_the_pilots_mode() {
    let s = Status {
        armed: false,
        nav: NavState::Disarmed,
        mode: FlightMode::Altitude,
        prearm: PrearmFlags::THROTTLE,
        ..Status::default()
    };
    let f = status_record(&s);
    let r = f.records("vehicle_status").next().unwrap();
    ints(
        &r,
        &[
            ("arming_state", 1),
            ("nav_state", 1),
            ("nav_state_user_intention", 1),
            ("failsafe", 0),
            ("pre_flight_checks_pass", 0),
        ],
    );
}

#[test]
fn failsafe_flags_map_what_has_a_counterpart() {
    let s = Status {
        valid: Validity {
            attitude: true,
            altitude: true,
            ..Validity::NONE
        },
        reasons: Reasons::RC_LOST
            .union(Reasons::BATTERY_LOW)
            .union(Reasons::GPS_LOST)
            .union(Reasons::LOSS_OF_CONTROL)
            .union(Reasons::BRAIN_LOST),
        home_set: false,
        ..Status::default()
    };
    let f = one_record(topic("failsafe_flags"), |w| {
        topics::failsafe_flags(w, Micros(80_000), &s)
    });
    let r = f.records("failsafe_flags").next().unwrap();
    assert_eq!(r.u64("timestamp"), Some(80_000));
    ints(
        &r,
        &[
            ("angular_velocity_invalid", 0),
            ("attitude_invalid", 0),
            ("local_altitude_invalid", 0),
            ("local_position_invalid", 1),
            ("local_position_invalid_relaxed", 1),
            ("local_velocity_invalid", 1),
            ("home_position_invalid", 1),
            ("manual_control_signal_lost", 1),
            ("offboard_control_signal_lost", 1),
            ("battery_warning", 1),
            ("fd_critical_failure", 1),
            ("gnss_lost", 1),
            ("geofence_breached", 0),
            ("gcs_connection_lost", 0),
            ("mode_req_attitude", 0),
            ("mode_not_supported", 0),
        ],
    );
    assert_eq!(r.raw().len(), 101);

    // The other reasons and a critical battery.
    let s = Status {
        valid: Validity::ALL,
        home_set: true,
        reasons: Reasons::BATTERY_CRITICAL
            .union(Reasons::BATTERY_LOW)
            .union(Reasons::SETPOINT_STALE)
            .union(Reasons::GEOFENCE),
        ..Status::default()
    };
    let f = one_record(topic("failsafe_flags"), |w| {
        topics::failsafe_flags(w, Micros(80_000), &s)
    });
    let r = f.records("failsafe_flags").next().unwrap();
    ints(
        &r,
        &[
            ("angular_velocity_invalid", 0),
            ("local_position_invalid", 0),
            ("home_position_invalid", 0),
            ("manual_control_signal_lost", 0),
            ("offboard_control_signal_lost", 1),
            ("battery_warning", 2),
            ("geofence_breached", 1),
            ("fd_critical_failure", 0),
            ("gnss_lost", 0),
        ],
    );
    // Nothing valid: attitude and rates invalid.
    let f = one_record(topic("failsafe_flags"), |w| {
        topics::failsafe_flags(w, Micros(1), &Status::default())
    });
    let r = f.records("failsafe_flags").next().unwrap();
    ints(
        &r,
        &[("angular_velocity_invalid", 1), ("attitude_invalid", 1)],
    );
}
