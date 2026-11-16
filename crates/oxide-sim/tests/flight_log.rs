// SPDX-License-Identifier: AGPL-3.0-or-later
//! The simulator writes logs (ADR-0005, task S1): the hook in `Simulation`,
//! what a whole flight's file holds, and that logging changes nothing.

use oxide_core::schedule::{F405_LEAN, H743_FULL, Profile};
use oxide_log::reader::{Item, UlogFile};
use oxide_sim::fault_scenarios;
use oxide_sim::flight_log::{Level, LogBuild, LogError, SharedBuffer};
use oxide_sim::scenario::Script;
use oxide_sim::{Airframe, SimOptions, Simulation};

const BOTH: [Profile; 2] = [F405_LEAN, H743_FULL];

fn build() -> LogBuild {
    LogBuild::sim("test-hash")
}

/// What a flight gave: the trace hash, and the file when a log was asked for.
struct Flight {
    hash: u64,
    file: Option<Vec<u8>>,
    dropped: u32,
    dropouts: u32,
    length_us: u64,
}

fn fly(name: &str, profile: Profile, logged: bool) -> Flight {
    let scenario = fault_scenarios::by_name(name).expect("scenario");
    let mut sim = scenario.build(profile, 0).expect("builds");
    let buf = SharedBuffer::new();
    if logged {
        sim.start_log(Level::Normal, &build(), Box::new(buf.clone()))
            .expect("log starts");
    }
    sim.run_until(scenario.end(), |_| {});
    let (dropped, dropouts) = sim
        .log_stats()
        .map_or((0, 0), |s| (s.dropped_stream + s.dropped_must, s.dropouts));
    if logged {
        sim.finish_log().expect("finishes");
    }
    Flight {
        hash: sim.trace_hash(),
        file: logged.then(|| buf.bytes()),
        dropped,
        dropouts,
        length_us: scenario.end().0,
    }
}

#[test]
fn a_flight_with_a_log_has_the_trace_hash_of_one_without() {
    for name in ["rc-loss-position", "gust-hold", "motor-loss"] {
        for profile in BOTH {
            let with = fly(name, profile, true);
            let without = fly(name, profile, false);
            assert_eq!(with.hash, without.hash, "{name} on {}", profile.name);
            assert!(with.file.is_some_and(|f| f.len() > 1000));
        }
    }
}

#[test]
fn nothing_drops_in_a_whole_flight_on_either_profile() {
    for profile in BOTH {
        for name in ["rc-loss-position", "gust-hold"] {
            let f = fly(name, profile, true);
            assert_eq!(f.dropped, 0, "{name} on {}", profile.name);
            assert_eq!(f.dropouts, 0, "{name} on {}", profile.name);
        }
    }
}

/// The rate of a streamed topic in Hz.
const RATES: [(&str, u64); 16] = [
    ("sensor_combined", 100),
    ("vehicle_angular_velocity", 100),
    ("vehicle_rates_setpoint", 100),
    ("actuator_motors", 100),
    ("vehicle_thrust_setpoint", 100),
    ("vehicle_torque_setpoint", 100),
    ("vehicle_attitude", 50),
    ("vehicle_attitude_setpoint", 50),
    ("vehicle_local_position", 20),
    ("vehicle_local_position_setpoint", 20),
    ("manual_control_setpoint", 10),
    ("vehicle_magnetometer", 10),
    ("vehicle_air_data", 10),
    ("battery_status", 5),
    ("oxide_estimator", 5),
    ("oxide_logger", 1),
];

#[test]
fn the_log_of_a_whole_flight_parses_and_each_topic_has_its_rate() {
    for profile in BOTH {
        let f = fly("rc-loss-position", profile, true);
        let bytes = f.file.expect("file");
        let file = UlogFile::parse(&bytes).expect("the reader parses it");
        assert!(file.truncated.is_none());
        assert_eq!(file.info_str("ver_sw"), Some("test-hash"));
        assert_eq!(file.info_str("ver_hw"), Some("SIM"));
        assert_eq!(file.info_str("oxide_profile"), Some(profile.name));
        assert_eq!(file.info_str("oxide_log_level"), Some("normal"));
        for (topic, hz) in RATES {
            // The estimate is published from the end of the EKF's alignment;
            // what is built from it starts then (1 s), the rest at the start.
            let from_estimate = matches!(
                topic,
                "vehicle_angular_velocity" | "vehicle_attitude" | "vehicle_local_position"
            );
            let first = file
                .records(topic)
                .next()
                .and_then(|r| r.u64("timestamp"))
                .expect("a first record");
            assert!(
                if from_estimate {
                    first <= 2_000_000
                } else {
                    first == 0
                },
                "{topic} starts at {first}"
            );
            let n = file.records(topic).count() as u64;
            let want = (f.length_us - first) * hz / 1_000_000;
            assert!(
                n.abs_diff(want) <= 1,
                "{topic} on {}: {n} records, {want} expected",
                profile.name
            );
        }
    }
}

/// The logged strings of a file, in order.
fn story(file: &UlogFile) -> Vec<String> {
    file.items
        .iter()
        .filter_map(|i| match i {
            Item::Log { text, .. } => Some(text.clone()),
            _ => None,
        })
        .collect()
}

#[test]
fn the_events_of_rc_loss_position_come_in_the_order_of_the_flight() {
    let f = fly("rc-loss-position", F405_LEAN, true);
    let bytes = f.file.expect("file");
    let file = UlogFile::parse(&bytes).expect("parses");
    let lines = story(&file);
    let at = |needle: &str| {
        lines
            .iter()
            .position(|l| l.contains(needle))
            .unwrap_or_else(|| panic!("no `{needle}` in {lines:#?}"))
    };
    let armed = at("Armed");
    let raised = at("Reason raised: RC_LOST");
    let action = at("Action: ");
    let landed = at("Touchdown");
    let disarmed = at("Disarmed: touchdown");
    assert!(
        armed < raised && raised < action && action < landed && landed < disarmed,
        "{lines:#?}"
    );
    // The same story in the status records: armed, then a failsafe action in
    // force, then disarmed.
    let states: Vec<u8> = file
        .records("vehicle_status")
        .map(|r| r.u8("arming_state").expect("arming_state"))
        .collect();
    let first_armed = states.iter().position(|s| *s == 2).expect("armed");
    let last_disarmed = states.iter().rposition(|s| *s == 1).expect("disarmed");
    assert!(first_armed < last_disarmed, "{states:?}");
    let in_force: Vec<bool> = file
        .records("vehicle_status")
        .map(|r| r.bool("failsafe").expect("failsafe"))
        .collect();
    let first_failsafe = in_force.iter().position(|b| *b).expect("a failsafe");
    assert!(first_armed < first_failsafe && first_failsafe < last_disarmed);
    // The data records of the events say the same, in the same order.
    let kinds: Vec<u8> = file
        .records("oxide_event")
        .map(|r| r.u8("kind").expect("kind"))
        .collect();
    assert_eq!(kinds.len(), lines.len(), "one record per logged string");
}

#[test]
fn a_log_cannot_start_after_the_first_tick_or_twice() {
    let scenario = fault_scenarios::by_name("hover-step").expect("scenario");
    let mut sim = scenario.build(F405_LEAN, 0).expect("builds");
    sim.step();
    let e = sim
        .start_log(Level::Normal, &build(), Box::new(SharedBuffer::new()))
        .expect_err("after the first tick");
    assert!(matches!(e, LogError::AfterFirstTick), "{e}");
    assert!(sim.log_stats().is_none());

    let mut sim = scenario.build(F405_LEAN, 0).expect("builds");
    sim.start_log(Level::Normal, &build(), Box::new(SharedBuffer::new()))
        .expect("first");
    let e = sim
        .start_log(Level::Normal, &build(), Box::new(SharedBuffer::new()))
        .expect_err("twice");
    assert!(matches!(e, LogError::AlreadyStarted), "{e}");
    // No log, no finish.
    let mut plain = scenario.build(F405_LEAN, 0).expect("builds");
    assert!(matches!(plain.finish_log(), Err(LogError::NotStarted)));
}

#[test]
fn the_replay_level_starts_and_its_header_says_so() {
    let scenario = fault_scenarios::by_name("hover-step").expect("scenario");
    let mut sim = scenario.build(F405_LEAN, 0).expect("builds");
    let buf = SharedBuffer::new();
    sim.start_log(Level::Replay, &build(), Box::new(buf.clone()))
        .expect("replay level starts");
    sim.finish_log().expect("finishes");
    let file = UlogFile::parse(&buf.bytes()).expect("parses");
    assert_eq!(file.info_str("oxide_log_level"), Some("replay"));
    assert_eq!(file.info_str("oxide_profile"), Some("f405-lean"));
    // The start records of ADR-0005 §5 are in the data section already.
    assert_eq!(file.records("oxide_boot").count(), 1);
    assert_eq!(file.records("oxide_param_us").count(), 10);
    assert!(file.records("oxide_vehicle").count() > 20);
    assert_eq!(file.records("oxide_in_tick").count(), 0);
}

#[test]
fn a_configuration_that_could_never_replay_is_refused_at_start_and_names_the_parameter() {
    use oxide_core::params::LoadReport;
    use oxide_sim::scenario::Script;
    let airframe = Airframe::reference_quad().expect("airframe");
    let boot = |cfg: &mut oxide_core::config::VehicleConfig<4>| {
        // 16,000,001 us is not the f32 number of seconds it reads back as.
        cfg.failsafe.rc_timeout_us = 16_000_001;
        LoadReport::default()
    };
    let mut sim = Simulation::<Script, 4>::with_boot(
        &airframe,
        F405_LEAN,
        Script::default(),
        SimOptions::default(),
        boot,
    )
    .expect("builds");
    let buf = SharedBuffer::new();
    let e = sim
        .start_log(Level::Replay, &build(), Box::new(buf.clone()))
        .expect_err("not replayable");
    assert!(matches!(e, LogError::NotReplayable(_)), "{e}");
    let text = e.to_string();
    assert!(text.contains("FS_RC_TIMEOUT"), "{text}");
    assert!(
        text.starts_with("cannot log at the replay level: "),
        "{text}"
    );
    // Nothing was written, and the normal level is fine with it.
    assert!(buf.bytes().is_empty());
    sim.start_log(Level::Normal, &build(), Box::new(SharedBuffer::new()))
        .expect("normal level");
}

/// A writer that fails after `limit` bytes.
struct Failing {
    limit: usize,
    taken: usize,
}

impl std::io::Write for Failing {
    fn write(&mut self, b: &[u8]) -> std::io::Result<usize> {
        if self.taken + b.len() > self.limit {
            return Err(std::io::Error::other("disk full"));
        }
        self.taken += b.len();
        Ok(b.len())
    }
    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}

#[test]
fn an_io_error_neither_panics_nor_changes_the_flight() {
    let scenario = fault_scenarios::by_name("motor-loss").expect("scenario");
    let plain = {
        let mut sim = scenario.build(F405_LEAN, 0).expect("builds");
        sim.run_until(scenario.end(), |_| {});
        sim.trace_hash()
    };
    let header_len = {
        let mut sim = scenario.build(F405_LEAN, 0).expect("builds");
        let buf = SharedBuffer::new();
        sim.start_log(Level::Normal, &build(), Box::new(buf.clone()))
            .expect("starts");
        buf.bytes().len()
    };
    for limit in [0, header_len - 1, header_len + 5_000, 200_000] {
        let mut sim = scenario.build(F405_LEAN, 0).expect("builds");
        let started = sim.start_log(
            Level::Normal,
            &build(),
            Box::new(Failing { limit, taken: 0 }),
        );
        if limit < header_len {
            assert!(matches!(started, Err(LogError::Io(_))), "{started:?}");
        } else {
            started.expect("the header fits");
        }
        sim.run_until(scenario.end(), |_| {});
        assert_eq!(sim.trace_hash(), plain, "limit {limit}");
        if limit >= header_len {
            match sim.finish_log() {
                Err(LogError::Io(e)) => assert_eq!(e.to_string(), "disk full"),
                other => panic!("limit {limit}: {:?}", other.map(|_| ())),
            }
        }
    }
}

#[test]
fn the_simulation_keeps_the_load_report_of_its_boot() {
    use oxide_core::params::LoadReport;
    let airframe = Airframe::reference_quad().expect("airframe");
    let report = LoadReport {
        applied: 3,
        unknown: 1,
        dropped: 2,
        invalid: false,
        unreadable: false,
        ..LoadReport::default()
    };
    let sim = Simulation::<Script, 4>::with_boot(
        &airframe,
        F405_LEAN,
        Script::default(),
        SimOptions::default(),
        |_| report,
    )
    .expect("builds");
    assert_eq!(sim.load_report(), report);
    let plain =
        Simulation::<Script, 4>::new(&airframe, F405_LEAN, Script::default()).expect("builds");
    assert_eq!(plain.load_report(), LoadReport::default());
}
