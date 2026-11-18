// SPDX-License-Identifier: AGPL-3.0-or-later
//! ArduCopter's tuning procedure on the real simulator program (ADR-0007
//! section 2, `oxide_bridge::tune`).
//!
//! Every test here needs the binary and is ignored by default: set
//! `OXIDE_ARDUCOPTER` to its path and run with `--ignored` (and `--release`).
//! The numbers below were measured on V4.7.2 (SHA-256 `caed35e4...`, seed 0).

use std::path::{Path, PathBuf};

use oxide_bridge::parm;
use oxide_bridge::sitl::PINNED;
use oxide_bridge::tune::{
    self, AUTOTUNE_S, FlightPlan, FlightResult, GAIN_NAMES, HOVER_S, TuneConfig, TuneError,
};

#[path = "support/pool.rs"]
mod pool;
use pool::Instance;

fn binary() -> PathBuf {
    match std::env::var_os("OXIDE_ARDUCOPTER") {
        Some(p) if !p.is_empty() => PathBuf::from(p),
        _ => panic!(
            "OXIDE_ARDUCOPTER is not set: set it to the path of the ArduCopter SITL V4.7.2 \
             binary (the one with SHA-256 {}) to run these tests",
            PINNED.sha256
        ),
    }
}

/// A directory that is removed when it goes out of scope.
struct TempDir(PathBuf);

impl TempDir {
    fn new(tag: &str) -> Self {
        let p = std::env::temp_dir().join(format!(
            "oxide-arducopter-tune-{}-{tag}",
            std::process::id()
        ));
        let _ = std::fs::remove_dir_all(&p);
        std::fs::create_dir_all(&p).expect("temporary directory");
        Self(p)
    }
    fn path(&self) -> &Path {
        &self.0
    }
}

impl Drop for TempDir {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

fn plan(root: &Path, instance: u8, seed: u64, stop_after_hover: bool) -> FlightPlan {
    FlightPlan {
        binary: binary(),
        root: root.to_path_buf(),
        flight_name: "tune-test".into(),
        instance,
        seed,
        files: tune::tuning_files(),
        extra: Vec::new(),
        stop_on_peak: false,
        stop_after_hover,
        autotune_s: AUTOTUNE_S,
        // The tuner is let run to its end where a test says so.
        enforce_class: stop_after_hover,
    }
}

fn text_at(f: &FlightResult, needle: &str) -> Option<f64> {
    tune::texts(&f.audit)
        .into_iter()
        .find(|(_, t)| t.contains(needle))
        .map(|(t, _)| t)
}

/// The hover, the learned thrust and the spectrum (stages 1 and 2): the
/// times of the script, what the gyro samples sent say, the thrust the
/// program learned, and a check that the program took every name and value
/// of `setup.parm`.
#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn the_hover_learns_the_thrust_and_the_spectrum_has_no_peak() {
    let lease = Instance::lease();
    let root = TempDir::new("hover");
    let mut p = plan(root.path(), lease.0, 0, true);
    p.enforce_class = false;
    let f = tune::fly_flight(&p).unwrap_or_else(|e| panic!("{e}"));
    assert_eq!(f.failure, None);
    let m = f.marks;
    println!(
        "MEASURED hover: arm {} us, throttle up {:?}, hover {:?}, spectrum {:?}, end {:?}; \
         stats {:?}; trace hash {:016x}",
        m.arm, m.climb, m.hover, m.spectrum, m.end, f.stats, f.trace_hash
    );
    // The script's own times (flight microseconds): the arm switch at
    // 13 s, the stick up at 16 s, the hover from the true height 9.5 m, the
    // spectrum 60 s later.
    assert_eq!(m.arm, 13_000_000);
    assert_eq!(m.climb, Some(16_000_000));
    assert_eq!(m.hover, Some(24_759_500));
    assert_eq!(m.spectrum, Some(24_759_500 + HOVER_S * 1_000_000));
    // The program armed 0.22 s after the switch and took the mode command.
    assert_eq!(text_at(&f, "Arming motors").map(|t| t.round()), Some(13.0));
    assert!(f.audit.mode_changes.iter().any(|(_, mode)| *mode == 2));
    // The spectrum of the 30 s of sent gyro samples: 30 000 samples, 57
    // segments, no line near ten times the band's median power.
    let (s, v) = f.spectrum.as_ref().expect("a spectrum");
    assert_eq!(s.samples, 30_000);
    assert_eq!(s.segments, 57);
    assert!(v.peak.is_none(), "{v:?}");
    let ratios: Vec<f64> = v.lines.iter().map(|l| l.ratio).collect();
    println!(
        "MEASURED spectrum: lines {:?}",
        v.lines
            .iter()
            .map(|l| (l.axis, l.freq_hz, l.ratio))
            .collect::<Vec<_>>()
    );
    assert!(ratios.iter().all(|r| (1.0..2.0).contains(r)), "{ratios:?}");
    // What the program learned in 60 s of hover (read at the end of it).
    let (before, _) = tune::read_back(&f);
    let learned = tune::echoes(
        &f.audit,
        tune::program_us(m.spectrum.expect("spectrum")),
        tune::program_us(m.spectrum.expect("spectrum") + 1_000_000),
    );
    let thrust = f64::from(learned["MOT_THST_HOVER"]);
    println!("MEASURED learned hover thrust {thrust}");
    assert!((thrust - 0.1495).abs() < 0.002, "{thrust}");
    // Every name of setup.parm was taken, with its value; the defaults the
    // report names are the program's.
    for (name, value) in parm::parse(parm::SETUP).expect("setup.parm") {
        let got = f64::from(before[&name]);
        assert!(
            (got - value).abs() < 1e-5,
            "{name}: asked {value}, read {got}"
        );
    }
    assert_eq!(before["AUTOTUNE_MIN_D"], 0.0005);
    assert_eq!(before["AUTOTUNE_AXES"], 7.0);
    // The program's default aggressiveness is 0.075, not the 0.1 the
    // specification calls the default.
    assert_eq!(before["AUTOTUNE_AGGR"], 0.075);
    assert_eq!(before["MOT_HOVER_LEARN"], 2.0);
    for (name, value) in [
        ("ATC_RAT_RLL_P", 0.05),
        ("ATC_RAT_YAW_P", 0.065),
        ("ATC_ANG_RLL_P", 4.5),
        ("ATC_ACC_R_MAX", 1100.0),
        ("ATC_ACC_Y_MAX", 270.0),
        ("ATC_RAT_YAW_FLTD", 20.0),
    ] {
        assert!((f64::from(before[name]) - value).abs() < 1e-5, "{name}");
    }
    // The hover on the starting gains, calm air, ALT_HOLD: tilt and
    // position over the last 20 s, and how far it went in the 60 s.
    assert!(
        (f.stats.hover_tilt_rms_deg - 0.2226).abs() < 0.005,
        "{:?}",
        f.stats
    );
    assert!(
        (f.stats.hover_pos_rms_m - 1.482).abs() < 0.02,
        "{:?}",
        f.stats
    );
    assert!(
        (f.stats.max_horizontal_m - 10.38).abs() < 0.05,
        "{:?}",
        f.stats
    );
}

/// The hover flight is a flight that repeats: the same trace hash on two
/// instances, and the same spectrum.
#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn the_tuning_flight_repeats_bit_for_bit_on_two_instances() {
    let a = Instance::lease();
    let b = Instance::lease();
    let (ra, rb) = {
        let (da, db) = (TempDir::new("rep-a"), TempDir::new("rep-b"));
        let (pa, pb) = (plan(da.path(), a.0, 0, true), plan(db.path(), b.0, 0, true));
        std::thread::scope(|s| {
            let ha = s.spawn(|| tune::fly_flight(&pa));
            let hb = s.spawn(|| tune::fly_flight(&pb));
            (
                ha.join().expect("thread").unwrap_or_else(|e| panic!("{e}")),
                hb.join().expect("thread").unwrap_or_else(|e| panic!("{e}")),
            )
        })
    };
    assert_eq!(ra.trace_hash, rb.trace_hash);
    assert_eq!(ra.audit.packet_hash, rb.audit.packet_hash);
    assert_eq!(ra.audit.reply_hash, rb.audit.reply_hash);
    assert_eq!(ra.spectrum, rb.spectrum);
    assert_eq!(tune::read_back(&ra), tune::read_back(&rb));
}

/// The tuner on our IMU model's noise with the default `AUTOTUNE_MIN_D`
/// (stages 3 and 4): every message, the time of each axis, the landing and
/// the saved gains, and the read-back, with the box not enforced. Flown twice
/// on two instances: the read-back is the same.
#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn the_tuner_runs_to_its_messages_and_saves_its_gains_on_landing() {
    let a = Instance::lease();
    let b = Instance::lease();
    let (da, db) = (TempDir::new("tune-a"), TempDir::new("tune-b"));
    let (pa, pb) = (
        plan(da.path(), a.0, 0, false),
        plan(db.path(), b.0, 0, false),
    );
    let (ra, rb) = std::thread::scope(|s| {
        let ha = s.spawn(|| tune::fly_flight(&pa));
        let hb = s.spawn(|| tune::fly_flight(&pb));
        (
            ha.join().expect("thread").unwrap_or_else(|e| panic!("{e}")),
            hb.join().expect("thread").unwrap_or_else(|e| panic!("{e}")),
        )
    });
    assert_eq!(ra.failure, None);
    assert_eq!(ra.trace_hash, rb.trace_hash, "the flight repeats");
    let (before, after) = tune::read_back(&ra);
    assert_eq!(tune::read_back(&rb).1, after, "the read-back repeats");
    // The messages of the tuner and their times (flight time, s).
    let at = |needle: &str| text_at(&ra, needle).unwrap_or_else(|| panic!("no text {needle:?}"));
    let started = at("AutoTune: Started");
    let roll = at("AutoTune: Roll complete");
    let pitch = at("AutoTune: Pitch complete");
    let yaw = at("AutoTune: Yaw(E) complete");
    let success = at("AutoTune: Success");
    let saved = at("AutoTune: Saved gains for Roll Pitch Yaw(E)");
    println!(
        "MEASURED autotune: started {started:.3}, roll {roll:.3}, pitch {pitch:.3}, yaw {yaw:.3}, \
         success {success:.3}, saved {saved:.3}; marks {:?}; stats {:?}",
        ra.marks, ra.stats
    );
    assert!((started - 85.762).abs() < 0.01, "{started}");
    assert!((roll - 185.165).abs() < 0.01, "{roll}");
    assert!((pitch - 277.137).abs() < 0.01, "{pitch}");
    assert!((yaw - 524.564).abs() < 0.01, "{yaw}");
    assert_eq!(success, yaw, "the success message follows the last axis");
    // Landed and disarmed by itself 7.2 s after the throttle went low, and
    // the gains saved at that moment.
    assert_eq!(saved, at("Disarming motors"));
    let low = ra.marks.throttle_low.expect("throttle low") as f64 * 1e-6;
    assert!((saved - low - 7.2).abs() < 0.3, "{saved} {low}");
    println!(
        "MEASURED pack {:?}, pack before landing {:?}, stats {:?}",
        ra.pack, ra.stats.pack_before_landing_v, ra.stats
    );
    assert_eq!(
        tune::check(&tune::texts(&ra.audit), &after, None, 0.0).expect_err("no pack"),
        "the pack voltage before the landing was not measured"
    );
    // The gains after the save, as the program reported them.
    println!("MEASURED gains after: {after:?}");
    let g = |n: &str| f64::from(after[n]);
    assert!((g("ATC_RAT_RLL_P") - 0.054_715_484).abs() < 1e-7);
    assert!((g("ATC_RAT_PIT_P") - 0.054_852_616).abs() < 1e-7);
    assert!((g("ATC_RAT_YAW_P") - 0.048_323_74).abs() < 1e-7);
    assert!((g("ATC_ANG_RLL_P") - 25.442_236).abs() < 1e-4);
    assert!((g("ATC_ANG_PIT_P") - 27.75).abs() < 1e-4);
    assert!((g("ATC_ANG_YAW_P") - 1.957_734_2).abs() < 1e-5);
    assert!((g("ATC_ACC_R_MAX") - 4874.6885).abs() < 0.01);
    assert!((g("ATC_ACC_P_MAX") - 5008.833).abs() < 0.01);
    assert!((g("ATC_ACC_Y_MAX") - 206.292_11).abs() < 0.01);
    assert_eq!(g("ATC_RAT_YAW_FLTE"), 5.0);
    // The thrust the program learned in the hover and kept: with the wait
    // of 480 s the vehicle lands on a pack with charge left (it was 0.1824,
    // an artefact of landing on an empty pack, after 900 s).
    assert!((g("MOT_THST_HOVER") - 0.149_965_4).abs() < 1e-6);
    // The gains the tuner started from are the starting gains.
    assert_eq!(before["ATC_RAT_RLL_P"], 0.05);
    assert_eq!(GAIN_NAMES.len(), after.len());
    // The tuner needed 438.8 s of its 480 s (started 85.762, success 524.564)
    // and the throttle went low at 565.760 s.
    assert!(yaw - started < AUTOTUNE_S as f64, "{}", yaw - started);
    assert_eq!(ra.marks.throttle_low, Some(565_759_500));
    // The pack, with the class not enforced: the voltage under load over the
    // last second before the throttle went low is above the gate's
    // low-battery threshold, and the state of charge at each mark.
    let pack = ra.stats.pack_before_landing_v.expect("pack measured");
    assert!((pack - 21.6549).abs() < 0.001, "{pack}");
    assert!(pack > tune::low_battery_volt());
    assert_eq!(
        tune::check(
            &tune::texts(&ra.audit),
            &after,
            Some(pack),
            tune::low_battery_volt()
        ),
        Ok(())
    );
    let soc = |name: &str| {
        ra.pack
            .iter()
            .find(|(n, _)| *n == name)
            .expect(name)
            .1
            .state_of_charge
    };
    assert!((soc("hover reached") - 0.9876).abs() < 0.001);
    assert!((soc("AUTOTUNE start") - 0.8991).abs() < 0.001);
    assert!((soc("throttle low") - 0.1454).abs() < 0.001);
    assert!((soc("disarmed") - 0.1377).abs() < 0.001);
    // The class: the vehicle stays inside the limits of the gate's class
    // (furthest 26.78 m out, highest 10.65 m, tilt up to 15.6 deg) and lands
    // at 2.481 m/s, which the gate's own rule calls a crash (more than
    // 2.0 m/s): the throttle stick low in AUTOTUNE descends at the program's
    // PILOT_SPEED_DN, 2.5 m/s.
    assert!(
        (ra.stats.furthest_from_home_m - 26.78).abs() < 0.05,
        "{:?}",
        ra.stats
    );
    assert!((ra.stats.highest_m - 10.65).abs() < 0.05, "{:?}", ra.stats);
    assert!(
        (ra.stats.largest_tilt_deg - 15.58).abs() < 0.05,
        "{:?}",
        ra.stats
    );
    let touchdown = ra.stats.touchdown_speed_mps.expect("it came down");
    assert!((touchdown - 2.4809).abs() < 0.001, "{touchdown}");
    assert_eq!(ra.stats.class, Some(oxide_sim::outcome::FlightClass::Crash));
    let exit = ra.stats.class_exit_s.expect("not safe at the touchdown");
    assert!((exit - 570.8135).abs() < 1e-3, "{exit}");
}

/// The procedure as the command runs it, on the specification as it stands
/// (ADR-0007 section 2): the tuner succeeds in 438.8 s of its 480 s and the
/// pack is at 21.65 V before the landing, but the landing itself is a crash by
/// the gate's class (touchdown at 2.481 m/s, more than 2.0 m/s). The flight is
/// stopped at the touchdown, the report says what happened, and no
/// `tuned-gains.parm` is written.
#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn the_procedure_fails_its_class_check_at_the_touchdown_and_writes_no_tuned_file() {
    let lease = Instance::lease();
    let out = TempDir::new("proc");
    let err = tune::tune(&TuneConfig {
        binary: binary(),
        out: out.path().to_path_buf(),
        instance: lease.0,
        seed: 0,
    })
    .expect_err("the touchdown is a crash by the gate's class");
    let TuneError::Failed(why) = &err else {
        panic!("{err}");
    };
    assert!(
        why.contains("class crash at flight time 570.813 s, 26.78 m from where it armed"),
        "{why}"
    );
    assert!(why.contains("touchdown speed Some(2.48"), "{why}");
    assert!(!out.path().join("tuned-gains.parm").exists());
    let report = std::fs::read_to_string(out.path().join("report.txt")).expect("report");
    assert!(report.contains("RESULT: FAILED"), "{report}");
    assert!(report.contains("AutoTune: Started"));
    assert!(report.contains("AutoTune: Success"));
    assert!(report.contains("as read at the moment the flight was stopped"));
    assert!(report.contains(
        "pack voltage under load, mean of the last second before the throttle went low: 21.655 V"
    ));
    assert!(report.contains("AUTOTUNE_AGGR 0.075, AUTOTUNE_AXES 7, AUTOTUNE_MIN_D 0.0005"));
    let spectrum = std::fs::read_to_string(out.path().join("spectrum.csv")).expect("spectrum");
    assert_eq!(spectrum.lines().count(), 1 + 513);
    assert!(spectrum.starts_with("freq_hz,gyro_x_rad_s"));
}
