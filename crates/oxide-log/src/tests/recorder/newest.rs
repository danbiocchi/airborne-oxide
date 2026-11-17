// SPDX-License-Identifier: AGPL-3.0-or-later
//! The newest sample feeds each input-driven topic, the newest IMU time
//! stamps the estimate and the commands, and the configuration and status
//! reach the records (ADR-0005 §3).

use super::{inputs_at, started, telemetry};
use crate::header::Level;
use crate::reader::UlogFile;
use oxide_core::config::EstimatorSource;
use oxide_core::io::{
    BaroSample, BatterySample, ImuSample, Inputs, MagSample, Outputs, RcSample, Stamped,
};
use oxide_core::math::Vec3;
use oxide_core::time::Micros;

/// Samples arrive on their own schedules and change every time; the IMU's
/// `t_sample` is 300 us before the tick that delivers it.
const IMU_EVERY: u64 = 3_000;
const SLOW_EVERY: u64 = 7_000;
const IMU_LAG: u64 = 300;

/// The value a sample delivered at `t` carries, exact in `f32`.
fn val(t: u64) -> f32 {
    (t / 1_000) as f32
}

fn inputs(t: u64) -> Inputs {
    let mut i = Inputs::default();
    if t.is_multiple_of(IMU_EVERY) {
        i.imu = Some(Stamped::ideal(
            Micros(t.saturating_sub(IMU_LAG)),
            ImuSample {
                gyro: Vec3::new(val(t), 0.0, 0.0),
                accel: Vec3::new(0.0, 0.0, -9.81),
            },
        ));
    }
    if t.is_multiple_of(SLOW_EVERY) {
        let m = Micros(t);
        i.rc = Some(Stamped::ideal(
            m,
            RcSample {
                throttle: val(t) / 1000.0,
                ..RcSample::default()
            },
        ));
        i.mag = Some(Stamped::ideal(
            m,
            MagSample {
                field_ut: Vec3::new(val(t), 0.0, 40.0),
            },
        ));
        i.baro = Some(Stamped::ideal(m, BaroSample { altitude_m: val(t) }));
        i.battery = Some(Stamped::ideal(
            m,
            BatterySample {
                voltage_v: 20.0 + val(t),
                current_a: 10.0,
            },
        ));
    }
    i
}

fn newest(t: u64, every: u64) -> u64 {
    t / every * every
}

#[test]
fn input_topics_carry_the_newest_sample_and_the_estimate_the_newest_imu_time() {
    let mut file = Vec::new();
    let (mut rec, cfg) = started(&mut file, Level::Normal);
    let mut t = 0;
    while t < 400_000 {
        rec.record(
            Micros(t),
            &inputs(t),
            &Outputs::<4>::off(Micros(t + 1000)),
            &telemetry(),
            &cfg,
        );
        rec.drain(&mut file);
        t += 1_000;
    }
    let f = UlogFile::parse(&file).unwrap();
    let ts = |r: &crate::reader::RecordView<'_>| r.u64("timestamp").unwrap();
    let imu_t = |now: u64| newest(now, IMU_EVERY).saturating_sub(IMU_LAG);

    let mut n = 0;
    for r in f.records("sensor_combined") {
        let now = ts(&r);
        assert_eq!(
            r.f32("gyro_rad[0]"),
            Some(val(newest(now, IMU_EVERY))),
            "{now}"
        );
        assert_eq!(
            r.i32("accelerometer_timestamp_relative"),
            Some((imu_t(now) as i64 - now as i64) as i32),
            "{now}"
        );
        n += 1;
    }
    assert!(n > 30);
    // Some record is not on an arrival tick, so a held sample is tested.
    for topic in [
        "vehicle_angular_velocity",
        "vehicle_attitude",
        "actuator_motors",
        "vehicle_thrust_setpoint",
        "vehicle_torque_setpoint",
        "vehicle_local_position",
    ] {
        let mut n = 0;
        for r in f.records(topic) {
            let now = ts(&r);
            assert_eq!(r.u64("timestamp_sample"), Some(imu_t(now)), "{topic} {now}");
            n += 1;
        }
        assert!(n > 5, "{topic}");
    }
    for r in f.records("manual_control_setpoint") {
        let now = ts(&r);
        let s = newest(now, SLOW_EVERY);
        assert_eq!(r.u64("timestamp_sample"), Some(s), "rc {now}");
        assert_eq!(
            r.f32("throttle"),
            Some(2.0 * (val(s) / 1000.0) - 1.0),
            "rc {now}"
        );
    }
    for r in f.records("vehicle_magnetometer") {
        let now = ts(&r);
        let s = newest(now, SLOW_EVERY);
        assert_eq!(r.u64("timestamp_sample"), Some(s), "mag {now}");
        assert_eq!(
            r.f32("magnetometer_ga[0]"),
            Some(val(s) / 100.0),
            "mag {now}"
        );
    }
    for r in f.records("vehicle_air_data") {
        let now = ts(&r);
        let s = newest(now, SLOW_EVERY);
        assert_eq!(r.u64("timestamp_sample"), Some(s), "baro {now}");
        assert_eq!(r.f32("baro_alt_meter"), Some(val(s)), "baro {now}");
    }
    for r in f.records("battery_status") {
        let now = ts(&r);
        let s = newest(now, SLOW_EVERY);
        assert_eq!(r.f32("voltage_v"), Some(20.0 + val(s)), "battery {now}");
    }
}

#[test]
fn the_estimator_source_and_gps_aiding_reach_the_records() {
    let mut file = Vec::new();
    let (mut rec, mut cfg) = started(&mut file, Level::Normal);
    cfg.estimator.source = EstimatorSource::Ekf;
    let mut out = Outputs::<4>::off(Micros(1000));
    out.status.estimator.gps_aiding = true;
    rec.record(Micros(0), &inputs_at(0), &out, &telemetry(), &cfg);
    rec.drain(&mut file);
    let f = UlogFile::parse(&file).unwrap();
    let s = f.records("oxide_status").next().unwrap();
    assert_eq!(s.u8("estimator_source"), Some(EstimatorSource::Ekf.code()));
    let p = f.records("vehicle_local_position").next().unwrap();
    assert_eq!(p.bool("dead_reckoning"), Some(false));
    let e = f.records("oxide_estimator").next().unwrap();
    assert_eq!(e.bool("gps_aiding"), Some(true));

    // And the other way, with the default configuration and status.
    let mut file = Vec::new();
    let (mut rec, cfg) = started(&mut file, Level::Normal);
    let out = Outputs::<4>::off(Micros(1000));
    rec.record(Micros(0), &inputs_at(0), &out, &telemetry(), &cfg);
    rec.drain(&mut file);
    let f = UlogFile::parse(&file).unwrap();
    let s = f.records("oxide_status").next().unwrap();
    assert_eq!(
        s.u8("estimator_source"),
        Some(EstimatorSource::External.code())
    );
    let p = f.records("vehicle_local_position").next().unwrap();
    assert_eq!(p.bool("dead_reckoning"), Some(true));
}
