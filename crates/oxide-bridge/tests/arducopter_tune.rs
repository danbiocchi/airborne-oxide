// SPDX-License-Identifier: AGPL-3.0-or-later
//! ArduCopter's tuning procedure on the real simulator program (ADR-0007
//! section 2 as changed by ADR-0009, `oxide_bridge::tune`): two tuning flights
//! from POSHOLD, roll and pitch and then yaw, the notch decided once.
//!
//! Every test here needs the binary and is ignored by default: set
//! `OXIDE_ARDUCOPTER` to its path and run with `--ignored` (and `--release`).
//! The numbers below were measured on V4.7.2 (SHA-256 `caed35e4...`, seed 0)
//! with the IMU model of ADR-0008. One more test flies the procedure on
//! tuning seeds 0 to 19 and is run with `OXIDE_TUNE_TWENTY=1`.

use std::path::{Path, PathBuf};

use oxide_bridge::arducopter::{
    ArduCopter, ArduCopterConfig, BrainLink, PREROLL_STEPS, bridged_options,
};
use oxide_bridge::mav::MavEvent;
use oxide_bridge::parm;
use oxide_bridge::runner::committed_params;
use oxide_bridge::sitl::PINNED;
use oxide_bridge::tune::{
    self, AUTOTUNE_S, FlightPlan, FlightResult, GAIN_NAMES, HOVER_S, NOTCH_NAMES, Part, TuneConfig,
};
use oxide_sim::scenario::NoPilot;
use oxide_sim::{Airframe, SimOptions, Simulation};

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

/// Set (to anything but empty) to fly the procedure on tuning seeds 0 to 19:
/// it takes a long time.
const TWENTY: &str = "OXIDE_TUNE_TWENTY";

/// The least number of the twenty tuning seeds the procedure must be accepted
/// on (ADR-0009).
const TWENTY_LEAST: usize = 18;

/// The tuning seed whose first hover shows a peak (seed 9 in the diagnosis the
/// procedure is built on; found again with this code).
const NOTCH_SEED: u64 = 9;

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
        read_notch: false,
        autotune_s: AUTOTUNE_S,
        // The tuner is let run to its end where a test says so.
        enforce_class: stop_after_hover,
    }
}

fn config(root: &Path, instance: u8, seed: u64) -> TuneConfig {
    TuneConfig {
        binary: binary(),
        out: root.to_path_buf(),
        instance,
        seed,
    }
}

fn text_at(f: &FlightResult, needle: &str) -> Option<f64> {
    tune::texts(&f.audit)
        .into_iter()
        .find(|(_, t)| t.contains(needle))
        .map(|(t, _)| t)
}

/// Flies two plans at once, on two instances.
fn fly_pair(a: &FlightPlan, b: &FlightPlan) -> (FlightResult, FlightResult) {
    std::thread::scope(|s| {
        let ha = s.spawn(|| tune::fly_flight(a));
        let hb = s.spawn(|| tune::fly_flight(b));
        (
            ha.join().expect("thread").unwrap_or_else(|e| panic!("{e}")),
            hb.join().expect("thread").unwrap_or_else(|e| panic!("{e}")),
        )
    })
}

/// The hover from POSHOLD, the learned thrust and the spectrum (stages 1 and
/// 2): the times of the script, what the gyro samples sent say, the thrust the
/// program learned, and a check that the program took every name and value of
/// `setup.parm`.
#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn the_hover_from_poshold_learns_the_thrust_and_the_spectrum_has_no_peak() {
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
    assert_eq!(m.hover, Some(24_759_000));
    assert_eq!(m.spectrum, Some(24_759_000 + HOVER_S * 1_000_000));
    // The program armed 0.22 s after the switch and took the mode command:
    // POSHOLD, mode 16, and no ALT_HOLD (mode 2) before the tuner's.
    assert_eq!(text_at(&f, "Arming motors").map(|t| t.round()), Some(13.0));
    assert!(f.audit.mode_changes.iter().any(|(_, mode)| *mode == 16));
    assert!(f.audit.mode_changes.iter().all(|(_, mode)| *mode != 2));
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
    assert!(
        ratios.iter().all(|r| (1.0..tune::PEAK_RATIO).contains(r)),
        "{ratios:?}"
    );
    // The largest line of each axis (x, y, z): its frequency and its power
    // over the band's median, 2.0 to 2.5 times: no line near 10.
    for (l, (hz, ratio)) in v.lines.iter().zip([
        (68.359_375, 2.4435),
        (41.015_625, 2.4510),
        (100.585_937_5, 2.0337),
    ]) {
        assert_eq!(l.freq_hz, hz, "{l:?}");
        assert!((l.ratio - ratio).abs() < 0.001, "{l:?}");
    }
    // What the program learned in 60 s of hover (read at the end of it).
    let (before, _) = tune::read_back(&f);
    let learned = tune::echoes(
        &f.audit,
        tune::program_us(m.spectrum.expect("spectrum")),
        tune::program_us(m.spectrum.expect("spectrum") + 1_000_000),
    );
    let thrust = f64::from(learned["MOT_THST_HOVER"]);
    println!("MEASURED learned hover thrust {thrust}");
    assert!((thrust - 0.149_551_6).abs() < 1e-6, "{thrust}");
    // The hover from POSHOLD in calm air: tilt and position over the last
    // 20 s, how far it went in the 60 s, and the class.
    assert!(
        (f.stats.hover_tilt_rms_deg - 1.3395).abs() < 0.005,
        "{:?}",
        f.stats
    );
    assert!(
        (f.stats.hover_pos_rms_m - 0.1587).abs() < 0.002,
        "{:?}",
        f.stats
    );
    assert!(
        (f.stats.max_horizontal_m - 0.364).abs() < 0.01,
        "{:?}",
        f.stats
    );
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
        fly_pair(&pa, &pb)
    };
    assert_eq!(ra.trace_hash, rb.trace_hash);
    assert_eq!(ra.audit.packet_hash, rb.audit.packet_hash);
    assert_eq!(ra.audit.reply_hash, rb.audit.reply_hash);
    assert_eq!(ra.spectrum, rb.spectrum);
    assert_eq!(tune::read_back(&ra), tune::read_back(&rb));
}

/// What one tuning flight measured, pinned (seed 0, ADR-0008's IMU).
struct Pins {
    tag: &'static str,
    /// The tuner's start message, its axes' completion messages and the
    /// success message, flight time s.
    started: f64,
    complete: &'static [(&'static str, f64)],
    success: f64,
    /// The descent began (flight microseconds), the time from it to the
    /// ground and from the ground to the save at the disarm, s.
    descent_us: u64,
    down_s: f64,
    saved_after_ground_s: f64,
    /// The pack under load before the descent, V, and the state of charge at
    /// the descent.
    pack_v: f64,
    soc_descent: f64,
    /// The gate's own judgement: furthest from where it armed, highest, tilt
    /// up to; the touchdown; the descent from, and its greatest speed.
    furthest_m: f64,
    highest_m: f64,
    tilt_deg: f64,
    touchdown_mps: f64,
    descent_from_m: f64,
    descent_top_mps: f64,
}

/// Prints what a tuning flight left and holds it to `want`.
fn check_flight_against(f: &FlightResult, want: &Pins) {
    let tag = want.tag;
    let (_, after) = tune::read_back(f);
    let at =
        |needle: &str| text_at(f, needle).unwrap_or_else(|| panic!("{tag}: no text {needle:?}"));
    println!(
        "MEASURED {tag} autotune: started {:.3}, complete {:?}, success {:.3}, saved {:.3}; \
         marks {:?}; stats {:?}",
        at("AutoTune: Started"),
        want.complete
            .iter()
            .map(|(a, _)| (*a, at(&format!("AutoTune: {a} complete"))))
            .collect::<Vec<_>>(),
        at("AutoTune: Success"),
        at("AutoTune: Saved gains for"),
        f.marks,
        f.stats
    );
    println!(
        "MEASURED {tag} pack {:?}, pack before landing {:?}",
        f.pack, f.stats.pack_before_landing_v
    );
    println!("MEASURED {tag} gains after: {after:?}");
    assert!(
        (at("AutoTune: Started") - want.started).abs() < 0.01,
        "{tag}"
    );
    for (axis, time) in want.complete {
        let got = at(&format!("AutoTune: {axis} complete"));
        assert!((got - time).abs() < 0.01, "{tag}: {axis} complete at {got}");
    }
    let success = at("AutoTune: Success");
    assert_eq!(
        Some(success),
        want.complete
            .last()
            .map(|(a, _)| at(&format!("AutoTune: {a} complete"))),
        "{tag}: the success message follows the last axis"
    );
    assert!((success - want.success).abs() < 0.01, "{tag}: {success}");
    assert!(
        success - want.started < AUTOTUNE_S as f64,
        "{tag}: the tuner finished inside its window"
    );
    // The descent with the stick at 0.30, the landing and the save at the
    // disarm, inside AUTOTUNE.
    let saved = at("AutoTune: Saved gains for");
    assert_eq!(saved, at("Disarming motors"), "{tag}");
    assert_eq!(f.marks.descent, Some(want.descent_us), "{tag}");
    let ground = f.marks.throttle_low.expect("on the ground") as f64 * 1e-6;
    let descent = want.descent_us as f64 * 1e-6;
    assert!(
        (ground - descent - want.down_s).abs() < 0.005,
        "{tag}: {ground} {descent}"
    );
    assert!(
        (saved - ground - want.saved_after_ground_s).abs() < 0.01,
        "{tag}: {saved} {ground}"
    );
    // The pack and the state of charge at each mark.
    let pack = f.stats.pack_before_landing_v.expect("pack measured");
    assert!((pack - want.pack_v).abs() < 0.001, "{tag}: {pack}");
    assert!(pack > tune::low_battery_volt(), "{tag}");
    let soc = |name: &str| {
        f.pack
            .iter()
            .find(|(n, _)| *n == name)
            .expect(name)
            .1
            .state_of_charge
    };
    assert!((soc("hover reached") - 0.9876).abs() < 0.001, "{tag}");
    assert!((soc("AUTOTUNE start") - 0.8990).abs() < 0.001, "{tag}");
    assert!(
        (soc("descent start") - want.soc_descent).abs() < 0.001,
        "{tag}"
    );
    // The class: safe, far inside the gate's limits, a slow touchdown.
    let st = &f.stats;
    assert_eq!(
        st.class,
        Some(oxide_sim::outcome::FlightClass::Safe),
        "{tag}"
    );
    assert_eq!(st.class_exit_s, None, "{tag}");
    assert!(
        (st.furthest_from_home_m - want.furthest_m).abs() < 0.05,
        "{tag}: {st:?}"
    );
    assert!(
        (st.highest_m - want.highest_m).abs() < 0.05,
        "{tag}: {st:?}"
    );
    assert!(
        (st.largest_tilt_deg - want.tilt_deg).abs() < 0.05,
        "{tag}: {st:?}"
    );
    let touchdown = st.touchdown_speed_mps.expect("it came down");
    assert!(
        (touchdown - want.touchdown_mps).abs() < 0.001,
        "{tag}: {touchdown}"
    );
    let from = st.descent_from_m.expect("descent");
    assert!((from - want.descent_from_m).abs() < 0.01, "{tag}: {from}");
    let top = st.descent_max_down_mps.expect("descent speed");
    assert!((top - want.descent_top_mps).abs() < 0.002, "{tag}: {top}");
    assert!(top < oxide_sim::outcome::CRASH_TOUCHDOWN_MPS, "{tag}");
}

/// The tuner on our IMU model's noise with the default `AUTOTUNE_MIN_D`
/// (stages 3 and 4), in both flights of the procedure: the roll-and-pitch
/// flight from the starting gains and the yaw flight on the gains the first
/// saved. Each runs to its messages, lands and saves its gains; each is flown
/// on two instances and reads back the same.
#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn the_tuner_runs_to_its_messages_and_saves_its_gains_on_landing_in_both_flights() {
    let a = Instance::lease();
    let b = Instance::lease();
    let (da, db) = (TempDir::new("tune-a"), TempDir::new("tune-b"));
    let (ca, cb) = (config(da.path(), a.0, 0), config(db.path(), b.0, 0));
    // Flight 1: roll and pitch.
    let (pa, pb) = (
        tune::flight_plan(&ca, 1, Part::RollPitch, None),
        tune::flight_plan(&cb, 1, Part::RollPitch, None),
    );
    let (ra, rb) = fly_pair(&pa, &pb);
    assert_eq!(ra.failure, None);
    assert_eq!(ra.trace_hash, rb.trace_hash, "the flight repeats");
    let (before_rp, after_rp) = tune::read_back(&ra);
    assert_eq!(tune::read_back(&rb).1, after_rp, "the read-back repeats");
    assert_eq!(before_rp["AUTOTUNE_AXES"], 3.0);
    assert_eq!(
        before_rp["ATC_RAT_RLL_P"], 0.05,
        "it starts from the starting gains"
    );
    assert!(ra.spectrum.as_ref().is_some_and(|(_, v)| v.peak.is_none()));
    check_flight_against(
        &ra,
        &Pins {
            tag: "roll-and-pitch",
            started: 85.762,
            complete: &[("Roll", 177.587), ("Pitch", 269.712)],
            success: 269.712,
            descent_us: 565_759_000,
            down_s: 16.047,
            saved_after_ground_s: 1.841,
            pack_v: 21.6601,
            soc_descent: 0.1462,
            furthest_m: 2.766,
            highest_m: 10.648,
            tilt_deg: 15.565,
            touchdown_mps: 0.6866,
            descent_from_m: 10.432,
            descent_top_mps: 0.7164,
        },
    );
    // The gains the roll-and-pitch flight saved: the roll, pitch and angle
    // gains and the acceleration limits it tunes; yaw as it started.
    let g = |after: &std::collections::BTreeMap<String, f32>, n: &str| f64::from(after[n]);
    assert!((g(&after_rp, "ATC_RAT_RLL_P") - 0.054_852_62).abs() < 1e-7);
    assert!((g(&after_rp, "ATC_RAT_PIT_P") - 0.057_739_586).abs() < 1e-7);
    assert!((g(&after_rp, "ATC_ANG_RLL_P") - 27.75).abs() < 1e-4);
    assert!((g(&after_rp, "ATC_ANG_PIT_P") - 27.75).abs() < 1e-4);
    assert!((g(&after_rp, "ATC_ACC_R_MAX") - 4_976.311_5).abs() < 0.01);
    assert!((g(&after_rp, "ATC_ACC_P_MAX") - 5112.084).abs() < 0.01);
    assert_eq!(after_rp["ATC_RAT_YAW_P"], 0.065);
    assert_eq!(after_rp["ATC_ANG_YAW_P"], 4.5);
    assert_eq!(after_rp["ATC_RAT_YAW_FLTE"], 2.0);
    // The thrust the program learned in the hover and kept.
    assert!((g(&after_rp, "MOT_THST_HOVER") - 0.149_997_55).abs() < 1e-6);
    assert_eq!(GAIN_NAMES.len(), after_rp.len());
    assert_eq!(
        tune::check_flight(
            &tune::texts(&ra.audit),
            &before_rp,
            &after_rp,
            ra.stats.pack_before_landing_v,
            tune::low_battery_volt(),
            Part::RollPitch.axes()
        ),
        Ok(())
    );
    assert!(
        text_at(&ra, "AutoTune: Yaw").is_none(),
        "the roll-and-pitch flight does not tune yaw"
    );
    assert!(
        ra.audit.mode_changes.iter().all(|(_, m)| *m != 9),
        "no LAND mode: the vehicle stayed in AUTOTUNE to the disarm"
    );
    // The file the yaw flight loads: what the first flight read back.
    let file = da.path().join("tuned-gains-1.parm");
    std::fs::write(
        &file,
        tune::roll_pitch_file(&after_rp, &ra.audit.binary_sha256, 0, ra.sim_s),
    )
    .expect("tuned-gains-1.parm");
    // Flight 2: yaw, on both instances, from the same file.
    let yaw = Part::Yaw {
        roll_pitch_file: &file,
    };
    let (pa, pb) = (
        tune::flight_plan(&ca, 2, yaw, None),
        tune::flight_plan(&cb, 2, yaw, None),
    );
    let (ya, yb) = fly_pair(&pa, &pb);
    assert_eq!(ya.failure, None);
    assert_eq!(ya.trace_hash, yb.trace_hash, "the flight repeats");
    let (before_yaw, after_yaw) = tune::read_back(&ya);
    assert_eq!(tune::read_back(&yb).1, after_yaw, "the read-back repeats");
    assert_eq!(before_yaw["AUTOTUNE_AXES"], 4.0);
    // It starts from the gains the first flight saved, not the starting gains.
    for name in [
        "ATC_RAT_RLL_P",
        "ATC_RAT_PIT_P",
        "ATC_ANG_RLL_P",
        "ATC_ACC_R_MAX",
        "ATC_ACC_P_MAX",
    ] {
        assert_eq!(before_yaw[name], after_rp[name], "{name}");
    }
    assert_ne!(before_yaw["ATC_RAT_RLL_P"], 0.05);
    check_flight_against(
        &ya,
        &Pins {
            tag: "yaw",
            started: 85.762,
            complete: &[("Yaw(E)", 431.439)],
            success: 431.439,
            descent_us: 565_759_500,
            down_s: 16.048,
            saved_after_ground_s: 1.8395,
            pack_v: 21.6776,
            soc_descent: 0.1490,
            furthest_m: 1.5715,
            highest_m: 10.630,
            tilt_deg: 3.118,
            touchdown_mps: 0.6864,
            descent_from_m: 10.4323,
            descent_top_mps: 0.7165,
        },
    );
    // The yaw gains the yaw flight saved, and the yaw error filter the tuner
    // moves from 2 to 5.
    assert!((g(&after_yaw, "ATC_RAT_YAW_P") - 0.050_802_436).abs() < 1e-7);
    assert!((g(&after_yaw, "ATC_ANG_YAW_P") - 2.060_773).abs() < 1e-5);
    assert!((g(&after_yaw, "ATC_ACC_Y_MAX") - 219.504_04).abs() < 0.01);
    assert_eq!(after_yaw["ATC_RAT_YAW_FLTE"], 5.0);
    assert!((g(&after_yaw, "MOT_THST_HOVER") - 0.149_967_63).abs() < 1e-6);
    assert_eq!(
        tune::check_flight(
            &tune::texts(&ya.audit),
            &before_yaw,
            &after_yaw,
            ya.stats.pack_before_landing_v,
            tune::low_battery_volt(),
            yaw.axes()
        ),
        Ok(())
    );
    assert!(
        text_at(&ya, "AutoTune: Roll").is_none() && text_at(&ya, "AutoTune: Pitch").is_none(),
        "the yaw flight does not tune roll or pitch"
    );
    // The yaw flight changed nothing but yaw.
    assert_eq!(tune::check_carried(&after_rp, &after_yaw), Ok(()));
    assert_ne!(after_rp["ATC_RAT_YAW_P"], after_yaw["ATC_RAT_YAW_P"]);
    assert!(
        ya.audit.mode_changes.iter().all(|(_, m)| *m != 9),
        "no LAND mode"
    );
    println!(
        "MEASURED tuned values: roll P {}, pitch P {}, yaw P {}, hover thrust {}",
        after_yaw["ATC_RAT_RLL_P"],
        after_yaw["ATC_RAT_PIT_P"],
        after_yaw["ATC_RAT_YAW_P"],
        after_yaw["MOT_THST_HOVER"]
    );
}

/// The procedure as the command runs it, on two instances: every check holds
/// in both flights and the file it writes is the committed
/// `tuned-gains.parm`, byte for byte (the header holds nothing that differs
/// between runs), the same on both.
#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn a_fresh_run_of_the_procedure_writes_the_committed_tuned_file_byte_for_byte_on_two_instances() {
    let (a, b) = (Instance::lease(), Instance::lease());
    let (da, db) = (TempDir::new("proc-a"), TempDir::new("proc-b"));
    let run = |dir: &TempDir, instance: u8| tune::tune(&config(dir.path(), instance, 0));
    let (oa, ob) = std::thread::scope(|s| {
        let ha = s.spawn(|| run(&da, a.0));
        let hb = s.spawn(|| run(&db, b.0));
        (ha.join().expect("thread"), hb.join().expect("thread"))
    });
    let oa = oa.unwrap_or_else(|e| panic!("{e}"));
    let ob = ob.unwrap_or_else(|e| panic!("{e}"));
    assert_eq!(
        (oa.flights.len(), ob.flights.len()),
        (2, 2),
        "no peak: two flights"
    );
    assert_eq!(oa.values_written(), 29);
    let committed = std::fs::read_to_string(parm::tuned_gains_path()).expect("committed file");
    assert_eq!(committed, parm::TUNED_GAINS);
    assert!(
        !committed.contains("INS_HNTCH"),
        "seed 0 shows no peak: no notch"
    );
    for d in [&da, &db] {
        let fresh = std::fs::read_to_string(d.path().join("tuned-gains.parm")).expect("written");
        assert_eq!(fresh, committed, "a fresh run writes the committed file");
        let report = std::fs::read_to_string(d.path().join("report.txt")).expect("report");
        assert!(
            report.contains("RESULT: the tuner ran to its success and saved-gains messages"),
            "{report}"
        );
        assert_eq!(
            report.matches("class of the flight: safe").count(),
            2,
            "{report}"
        );
        assert_eq!(
            report
                .matches(
                    "pack voltage under load, mean of the last second before the descent began"
                )
                .count(),
            2
        );
        assert!(report.contains("AUTOTUNE_AGGR 0.075, AUTOTUNE_AXES 3, AUTOTUNE_MIN_D 0.0005"));
        assert!(report.contains("AutoTune: Saved gains for Roll Pitch"));
        assert!(report.contains("AutoTune: Saved gains for Yaw(E)"));
        assert!(report.contains("notch: none"));
        assert!(report.contains("POSHOLD command 2.000 s"));
        let spectrum = std::fs::read_to_string(d.path().join("spectrum.csv")).expect("spectrum");
        assert_eq!(spectrum.lines().count(), 1 + 513);
        assert!(d.path().join("tuned-gains-1.parm").exists());
    }
    // Both instances flew the same flights: every trace hash line, whole.
    let hashes = |d: &TempDir| -> Vec<String> {
        std::fs::read_to_string(d.path().join("report.txt"))
            .expect("report")
            .lines()
            .filter(|l| l.contains("trace hash"))
            .map(|l| l.split(',').next().unwrap_or("").trim().to_string())
            .collect()
    };
    let (ha, hb) = (hashes(&da), hashes(&db));
    assert_eq!(ha.len(), 2);
    assert!(
        ha.iter().all(|l| l.len() > "trace hash ".len() + 15),
        "{ha:?}"
    );
    assert_eq!(ha, hb);
    assert_eq!(
        std::fs::read_to_string(da.path().join("tuned-gains-1.parm")).expect("file 1"),
        std::fs::read_to_string(db.path().join("tuned-gains-1.parm")).expect("file 1")
    );
}

/// A seed whose first hover shows a peak: three flights, the notch decided by
/// the first hover, flown in the roll-and-pitch and the yaw flights, and the
/// tuned file ends with the six notch lines, read back.
#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn a_seed_whose_first_hover_shows_a_peak_flies_three_flights_and_its_file_ends_with_the_notch() {
    let lease = Instance::lease();
    let root = TempDir::new("peak");
    let o = tune::tune(&config(root.path(), lease.0, NOTCH_SEED)).unwrap_or_else(|e| panic!("{e}"));
    println!("MEASURED notch {:?}; read back {:?}", o.notch, o.notch_read);
    assert_eq!(o.flights.len(), 3);
    // Flight 1 landed after its hover on the peak; flights 2 and 3 tuned.
    let (f1, f2, f3) = (&o.flights[0], &o.flights[1], &o.flights[2]);
    assert!(f1.marks.autotune.is_none() && f1.marks.descent.is_some());
    assert!(f2.marks.autotune.is_some() && f3.marks.autotune.is_some());
    let n = o.notch.as_ref().expect("a notch");
    assert_eq!(n.flight, 1);
    let peak = f1
        .spectrum
        .as_ref()
        .and_then(|(_, v)| v.peak)
        .expect("flight 1 shows a peak");
    assert_eq!(peak, n.line);
    assert!(peak.ratio >= tune::PEAK_RATIO);
    // The line of the rotors: 222.66 Hz on the x axis, 19.3 times the band's
    // median power; the hover thrust the flight learned is the notch's
    // reference.
    assert_eq!((peak.axis, peak.freq_hz), (0, 222.656_25));
    assert!((peak.ratio - 19.258).abs() < 0.01, "{peak:?}");
    assert!((n.reference - 0.149_507_88).abs() < 1e-7, "{}", n.reference);
    // The notch is in the flights that tune, and the roll-and-pitch flight
    // has the learned thrust.
    let has = |f: &FlightResult, name: &str, value: f64| {
        f.plan.extra.iter().any(|(n, v)| n == name && *v == value)
    };
    for f in [f2, f3] {
        for (name, value) in &n.params {
            assert!(has(f, name, *value), "{name} in {}", f.plan.flight_name);
        }
    }
    assert!(has(f2, "MOT_THST_HOVER", n.reference));
    assert!(f3.plan.extra.iter().all(|(n, _)| n != "MOT_THST_HOVER"));
    // Later hovers' spectra are reported, and decide nothing.
    assert!(f2.spectrum.is_some() && f3.spectrum.is_some());
    // The file: 29 gains and the six notch lines after them, read back.
    let file = std::fs::read_to_string(root.path().join("tuned-gains.parm")).expect("tuned file");
    assert_eq!(file, o.tuned);
    let values: Vec<&str> = file.lines().filter(|l| !l.starts_with('#')).collect();
    assert_eq!(values.len(), 35);
    let tail: Vec<&str> = values[29..]
        .iter()
        .map(|l| l.split(' ').next().expect("name"))
        .collect();
    assert_eq!(tail, NOTCH_NAMES);
    assert_eq!(values[29], "INS_HNTCH_ENABLE 1");
    assert_eq!(values[30], "INS_HNTCH_MODE 1");
    assert_eq!(values[31], "INS_HNTCH_FREQ 222.65625");
    assert_eq!(values[32], "INS_HNTCH_BW 111.328125");
    assert_eq!(values[33], "INS_HNTCH_REF 0.14950788");
    assert_eq!(values[34], "INS_HNTCH_ATT 40");
    for (name, value) in parm::parse(&file).expect("parses") {
        if let Some((_, set)) = n.params.iter().find(|(m, _)| *m == name) {
            assert_eq!(value as f32, *set as f32, "{name} read back as it was set");
        }
    }
    let report = std::fs::read_to_string(root.path().join("report.txt")).expect("report");
    assert!(
        report.contains("notch: decided by the hover of flight 1: PEAK at"),
        "{report}"
    );
}

/// A flight that loads a file with the six notch lines has the notch set in the
/// program. It is started the way a gate flight is: the committed list of
/// parameter files with that file as the tuned one, and the committed extra
/// parameters. A gate flight sends no parameter read, so the test reads the six
/// itself.
#[test]
#[ignore = "needs the ArduCopter SITL binary: set OXIDE_ARDUCOPTER"]
fn a_flight_that_loads_a_tuned_file_with_notch_lines_has_the_notch_set_in_the_program() {
    let lease = Instance::lease();
    let notch = [
        ("INS_HNTCH_ENABLE", 1.0_f32),
        ("INS_HNTCH_MODE", 1.0),
        ("INS_HNTCH_FREQ", 222.65625),
        ("INS_HNTCH_BW", 111.328125),
        ("INS_HNTCH_REF", 0.149_507_88),
        ("INS_HNTCH_ATT", 40.0),
    ];
    assert_eq!(notch.map(|(n, _)| n), NOTCH_NAMES);
    for (tag, with_notch) in [("without", false), ("with", true)] {
        let root = TempDir::new(&format!("loads-{tag}"));
        // The committed gains without any notch line of their own, and the six
        // lines after them when the flight is to have them.
        let mut text: String = parm::TUNED_GAINS
            .lines()
            .filter(|l| !l.starts_with("INS_HNTCH_"))
            .map(|l| format!("{l}\n"))
            .collect();
        if with_notch {
            for (name, value) in notch {
                text.push_str(&format!("{name} {value}\n"));
            }
        }
        let tuned = root.path().join("tuned-gains.parm");
        std::fs::write(&tuned, text).expect("the tuned file");
        let airframe = Airframe::reference_quad().expect("airframe");
        let (mut files, extras) = committed_params(&airframe);
        let at = files
            .iter()
            .position(|p| p.file_name().is_some_and(|n| n == "tuned-gains.parm"))
            .expect("the committed list has the tuned file");
        files[at] = tuned;
        let mut c = ArduCopterConfig::new(binary(), root.path().join("flight"), lease.0);
        c.flight_name = Some("sitl".into());
        c.param_files = files;
        c.extra_params = extras;
        c.brain = BrainLink::None;
        let options = bridged_options(SimOptions::realistic(0), &mut c);
        let ap = ArduCopter::start(c).unwrap_or_else(|e| panic!("{e}"));
        let mut sim =
            Simulation::with_autopilot(&airframe, ap, NoPilot, options).expect("the simulation");
        sim.preroll(PREROLL_STEPS).unwrap_or_else(|e| panic!("{e}"));
        for name in NOTCH_NAMES {
            sim.autopilot()
                .instrument()
                .param_request(name)
                .expect("param_request");
        }
        for _ in 0..400 {
            sim.try_step().unwrap_or_else(|e| panic!("{e}"));
        }
        let audit = sim
            .into_autopilot()
            .finish()
            .unwrap_or_else(|e| panic!("{e}"));
        let read = |name: &str| -> f32 {
            audit
                .heard
                .iter()
                .find_map(|h| match &h.event {
                    MavEvent::ParamValue { name: n, value, .. } if n == name => Some(*value),
                    _ => None,
                })
                .unwrap_or_else(|| panic!("{tag}: no value for {name} was read"))
        };
        println!(
            "MEASURED {tag} the lines: the program has {:?}",
            NOTCH_NAMES
                .iter()
                .map(|n| (*n, read(n)))
                .collect::<Vec<_>>()
        );
        if with_notch {
            for (name, value) in notch {
                assert_eq!(read(name), value, "{name}");
            }
        } else {
            // The list this flight was started with is a gate flight's: the
            // effective file the program was given has the hash the gate
            // expects of a flight of a plain scenario.
            assert_eq!(
                Some(audit.params_sha256.as_str()),
                oxide_bridge::gate::files::expected_params_sha256("gust-hold")
                    .ok()
                    .as_deref(),
                "the effective parameter file of a gate flight"
            );
            assert_eq!(
                read("INS_HNTCH_ENABLE"),
                0.0,
                "the notch is off without the lines"
            );
        }
    }
}

/// The procedure on tuning seeds 0 to 19, three flights at a time: accepted on
/// at least 18. Prints the count and each failure's reason. Behind
/// `OXIDE_TUNE_TWENTY=1`: it takes long.
#[test]
#[ignore = "needs the ArduCopter SITL binary and takes long: set OXIDE_ARDUCOPTER and OXIDE_TUNE_TWENTY=1"]
fn the_procedure_is_accepted_on_at_least_18_of_the_tuning_seeds_0_to_19() {
    if !std::env::var_os(TWENTY).is_some_and(|v| !v.is_empty()) {
        eprintln!("{TWENTY} is not set: the twenty seeds are not flown");
        return;
    }
    use std::sync::Mutex;
    use std::sync::atomic::{AtomicU64, Ordering};
    const SEEDS: u64 = 20;
    const AT_ONCE: usize = 3;
    let next = AtomicU64::new(0);
    let results: Mutex<Vec<(u64, Result<usize, String>)>> = Mutex::new(Vec::new());
    std::thread::scope(|s| {
        for _ in 0..AT_ONCE {
            s.spawn(|| {
                loop {
                    let seed = next.fetch_add(1, Ordering::SeqCst);
                    if seed >= SEEDS {
                        break;
                    }
                    let lease = Instance::lease();
                    let dir = TempDir::new(&format!("twenty-{seed}"));
                    let r = tune::tune(&config(dir.path(), lease.0, seed))
                        .map(|o| o.flights.len())
                        .map_err(|e| e.to_string());
                    results.lock().expect("results").push((seed, r));
                }
            });
        }
    });
    let mut results = results.into_inner().expect("results");
    results.sort_by_key(|(s, _)| *s);
    let accepted = results.iter().filter(|(_, r)| r.is_ok()).count();
    println!("MEASURED the procedure was accepted on {accepted} of {SEEDS} tuning seeds");
    for (seed, r) in &results {
        match r {
            Ok(n) => println!("MEASURED seed {seed}: accepted, {n} flights"),
            Err(e) => println!("MEASURED seed {seed}: FAILED: {e}"),
        }
    }
    assert!(
        accepted >= TWENTY_LEAST,
        "accepted on {accepted} of {SEEDS}, at least {TWENTY_LEAST} are needed"
    );
}
