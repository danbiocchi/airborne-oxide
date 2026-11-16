// SPDX-License-Identifier: AGPL-3.0-or-later
//! A topic made from an input sample is written when it is due and a sample
//! has arrived since its last record (ADR-0005 §3): a sensor that stops
//! leaves a gap, as in PX4.

use super::rates::{run, times};
use oxide_core::io::Inputs;

/// The five topics made from an input sample, with the period of each.
const SENSOR_TOPICS: [(&str, u64); 5] = [
    ("sensor_combined", 10_000),
    ("manual_control_setpoint", 100_000),
    ("vehicle_magnetometer", 100_000),
    ("vehicle_air_data", 100_000),
    ("battery_status", 200_000),
];

/// Takes every input away from `t` on.
fn silence(t: &mut super::rates::Tick) {
    t.inputs = Inputs::default();
}

#[test]
fn a_sample_on_every_tick_gives_each_topic_its_rate() {
    let (f, _, _) = run(1000, 2_000_000, |_| {});
    for (name, period) in SENSOR_TOPICS {
        let ts = times(&f, name);
        let want: Vec<u64> = (0..2_000_000 / period).map(|k| k * period).collect();
        assert_eq!(ts, want, "{name}");
    }
}

#[test]
fn with_no_sample_for_three_seconds_a_topic_writes_nothing_then_one_record() {
    // Every input stops after 1 s and returns at 4 s.
    let (f, _, _) = run(1000, 5_000_000, |t| {
        if (1_000_000..4_000_000).contains(&t.now) {
            silence(t);
        }
    });
    for (name, period) in SENSOR_TOPICS {
        let ts = times(&f, name);
        // The last record before the gap is the grid point at 1 s: a sample
        // arrived (at 0.999 s) since the one before it. Then nothing.
        let quiet: Vec<u64> = ts
            .iter()
            .copied()
            .filter(|t| (1_000_001..4_000_000).contains(t))
            .collect();
        assert!(quiet.is_empty(), "{name}: {quiet:?}");
        assert!(ts.contains(&1_000_000), "{name}");
        let after: Vec<u64> = ts.iter().copied().filter(|t| *t > 1_000_000).collect();
        // The first sample is written at once, one record and no burst, and
        // the grid goes on from there on its own phase.
        assert_eq!(after[0], 4_000_000, "{name}: {after:?}");
        assert_eq!(after[1], 4_000_000 + period, "{name}: {after:?}");
        assert!(after.windows(2).all(|w| w[1] - w[0] == period), "{name}");
        assert_eq!(after.len() as u64, 1_000_000 / period, "{name}");
    }
}

#[test]
fn a_sensor_slower_than_the_topic_gives_one_record_per_sample() {
    // The IMU delivers every 25 ms (the topic is 100 Hz); the other inputs
    // every 400 ms (the topics are 5 and 10 Hz).
    let (f, _, _) = run(1000, 2_000_000, |t| {
        let imu = t.inputs.imu.take();
        let rest = std::mem::take(&mut t.inputs);
        if t.now % 25_000 == 0 {
            t.inputs.imu = imu;
        }
        if t.now % 400_000 == 0 {
            t.inputs = Inputs {
                imu: t.inputs.imu,
                ..rest
            };
        }
    });
    let want_imu: Vec<u64> = (0..80).map(|k| k * 25_000).collect();
    assert_eq!(times(&f, "sensor_combined"), want_imu);
    let slow: Vec<u64> = (0..5).map(|k| k * 400_000).collect();
    for name in [
        "manual_control_setpoint",
        "vehicle_magnetometer",
        "vehicle_air_data",
        "battery_status",
    ] {
        assert_eq!(times(&f, name), slow, "{name}");
    }
}

#[test]
fn a_topic_whose_input_alone_stops_is_the_only_one_that_stops() {
    let (f, _, _) = run(1000, 3_000_000, |t| {
        if t.now >= 1_000_000 {
            t.inputs.baro = None;
        }
    });
    // Ten on the grid before 1 s, and one at 1 s: the sample of 0.999 s.
    assert_eq!(times(&f, "vehicle_air_data").len(), 11);
    assert_eq!(times(&f, "vehicle_magnetometer").len(), 30);
    assert_eq!(times(&f, "sensor_combined").len(), 300);
    // Topics made from the estimate and the commands do not depend on it.
    assert_eq!(times(&f, "vehicle_attitude").len(), 150);
    assert_eq!(times(&f, "actuator_motors").len(), 300);
}
