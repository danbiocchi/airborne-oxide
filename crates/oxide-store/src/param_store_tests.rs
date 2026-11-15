// SPDX-License-Identifier: AGPL-3.0-or-later
//! Tests of `Store::unmounted`/`remount` and of `ParamStore` (ADR-0004 §6).

use std::cell::Cell;

use crate::flash::{Flash, FlashError};
use crate::sim::SimFlash;
use crate::store::{Error, MountReport, Store};

/// A flash that fails every read while `failing` is set.
struct Flaky<F: Flash> {
    inner: F,
    failing: Cell<bool>,
}

impl<F: Flash> Flaky<F> {
    fn new(inner: F) -> Self {
        Self {
            inner,
            failing: Cell::new(false),
        }
    }
}

impl<F: Flash> Flash for Flaky<F> {
    const SECTOR_SIZE: usize = F::SECTOR_SIZE;
    fn read(&self, s: usize, o: usize, b: &mut [u8]) -> Result<(), FlashError> {
        if self.failing.get() {
            Err(FlashError::Device)
        } else {
            self.inner.read(s, o, b)
        }
    }
    fn program(&mut self, s: usize, o: usize, d: &[u8]) -> Result<(), FlashError> {
        self.inner.program(s, o, d)
    }
    fn erase(&mut self, s: usize) -> Result<(), FlashError> {
        self.inner.erase(s)
    }
}

#[test]
fn an_unmounted_store_holds_nothing_and_refuses_writes() {
    let mut s = Store::unmounted(SimFlash::<88>::new());
    assert_eq!(s.get(1), Ok(None));
    assert_eq!(s.iter().count(), 0);
    assert_eq!(s.set(1, 2), Err(Error::Corrupt));
}

#[test]
fn a_failed_remount_keeps_the_flash_and_a_later_one_works() {
    let mut s = Store::unmounted(Flaky::new(SimFlash::<88>::new()));
    s.flash_mut().failing.set(true);
    assert_eq!(s.remount(), Err(Error::Flash(FlashError::Device)));
    assert_eq!(s.set(1, 2), Err(Error::Corrupt), "not usable until mounted");
    s.flash_mut().failing.set(false);
    assert_eq!(s.remount(), Ok(MountReport::Fresh));
    s.set(1, 2).unwrap();
    assert_eq!(s.get(1), Ok(Some(2)));
    // And again on a populated store: the in-memory state is rebuilt.
    assert_eq!(s.remount(), Ok(MountReport::Clean));
    assert_eq!(s.get(1), Ok(Some(2)));
}

// ---- ParamStore ----

use oxide_core::Core;
use oxide_core::config::{
    EstimatorConfig, FailsafeParams, Gains, Limits, Rotor, Spin, VehicleConfig,
};
use oxide_core::io::{
    BatterySample, EventKind, Inputs, Outputs, ParamSet, PrearmFlags, RcSample, Stamped,
    StateSample,
};
use oxide_core::math::Vec3;
use oxide_core::params::{self, LoadReport, ParamId, ParamValue};
use oxide_core::schedule::{F405_LEAN, Profile};
use oxide_core::time::Micros;

use crate::param_store::{ParamStore, Serviced};
use crate::sim::BitCut;
use crate::store::Replaced;

const PROFILE: Profile = F405_LEAN;
/// Eight record slots per sector: (112 - 16) / 12.
type Flash8 = SimFlash<112>;

/// The firmware's own values: a quad as the core's own tests build it.
fn quad() -> VehicleConfig<4> {
    let a = 0.08;
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

fn rc(arm: bool) -> RcSample {
    RcSample {
        arm_switch: arm,
        mode_switch: 1,
        ..RcSample::default()
    }
}

fn setf(id: ParamId, v: f32) -> ParamSet {
    ParamSet {
        id,
        value: ParamValue::F32(v),
    }
}

/// A core on the external-state rig, driven one tick at a time.
struct Rig {
    core: Core<4>,
    k: u64,
    /// Every event the core has reported.
    events: Vec<EventKind>,
}

impl Rig {
    fn new(cfg: VehicleConfig<4>, report: LoadReport) -> Self {
        let mut core = Core::new(cfg, PROFILE).unwrap();
        core.report_param_load(report);
        Self {
            core,
            k: 0,
            events: Vec::new(),
        }
    }

    fn tick(&mut self, rc_sample: RcSample, set: Option<ParamSet>, ack: bool) -> Outputs<4> {
        let t = Micros(self.k * 1000);
        self.k += 1;
        let inputs = Inputs {
            state: Some(Stamped::ideal(t, StateSample::default())),
            rc: Some(Stamped::ideal(t, rc_sample)),
            battery: Some(Stamped::ideal(
                t,
                BatterySample {
                    voltage_v: 23.0,
                    current_a: 0.0,
                },
            )),
            param_set: set,
            param_ack: ack,
            ..Inputs::default()
        };
        let out = self.core.tick(t, &inputs);
        self.events.extend(out.events.iter().map(|e| e.kind));
        out
    }

    fn ticks(&mut self, n: u64, rc_sample: RcSample) -> Outputs<4> {
        let mut out = self.tick(rc_sample, None, false);
        for _ in 1..n {
            out = self.tick(rc_sample, None, false);
        }
        out
    }

    /// Settles and tries to arm; whether the vehicle armed.
    fn try_arm(&mut self) -> (bool, PrearmFlags) {
        self.ticks(1100, rc(false));
        let out = self.ticks(100, rc(true));
        (out.status.armed, out.status.prearm)
    }

    fn disarm(&mut self) {
        let out = self.ticks(200, rc(false));
        assert!(!out.status.armed && !self.core.is_armed());
    }

    fn set(&mut self, id: ParamId, v: f32) {
        self.tick(rc(false), Some(setf(id, v)), false);
    }

    fn ack(&mut self) {
        self.tick(rc(false), None, true);
    }
}

fn boot_on(flash: Flash8) -> (ParamStore<Flash8, 4>, VehicleConfig<4>, LoadReport) {
    let mut cfg = quad();
    let (ps, report) = ParamStore::boot(flash, &mut cfg, &PROFILE);
    (ps, cfg, report)
}

/// The activity counters of a flash: nothing here moves unless something is
/// programmed or erased.
fn activity(f: &Flash8) -> (u64, u64, u32) {
    (f.events(), f.program_calls(), f.total_erases())
}

#[test]
fn a_blank_flash_boots_clean_and_a_saved_parameter_comes_back() {
    let (mut ps, cfg, report) = boot_on(Flash8::new());
    assert!(report.clean() && report.applied == 0, "{report:?}");
    assert!(cfg == quad(), "a blank store leaves the firmware's values");
    let mut rig = Rig::new(cfg, report);
    assert_eq!(ps.service(&mut rig.core), Ok(Serviced::Idle));

    rig.set(ParamId::LIM_TILT, 0.3);
    assert!(rig.core.params_unsaved());
    assert_eq!(
        ps.service(&mut rig.core),
        Ok(Serviced::Saved(Replaced::Appended))
    );
    assert!(!rig.core.params_unsaved(), "the core was told it is saved");
    assert_eq!(ps.service(&mut rig.core), Ok(Serviced::Idle));

    let (ps2, cfg2, report2) = boot_on(ps.into_flash());
    assert!(report2.clean() && report2.applied == 1, "{report2:?}");
    assert_eq!(cfg2.limits.max_tilt_rad, 0.3);
    let mut want = quad();
    want.limits.max_tilt_rad = 0.3;
    assert!(cfg2 == want);
    assert_eq!(ps2.torn_records(), 0);
}

#[test]
fn the_pair_that_is_valid_only_together_is_invalid_one_at_a_time() {
    // The premise of the cut test below: either record alone, loaded onto the
    // firmware's values, breaks a rule.
    let crit = (params::key("BAT_CRIT_V"), 3.6_f32.to_bits());
    let low = (params::key("BAT_LOW_V"), 3.8_f32.to_bits());
    for (what, one) in [("crit only", crit), ("low only", low)] {
        let mut cfg = quad();
        let r = params::load(&mut cfg, &PROFILE, [one]);
        if what == "crit only" {
            assert!(
                r.invalid && r.applied == 0 && cfg == quad(),
                "{what}: {r:?}"
            );
        } else {
            // Raising the low threshold alone is a valid configuration.
            assert!(r.clean() && r.applied == 1, "{what}: {r:?}");
        }
    }
    let mut cfg = quad();
    let r = params::load(&mut cfg, &PROFILE, [crit, low]);
    assert!(r.clean() && r.applied == 2, "{r:?}");
    assert_eq!(cfg.failsafe.battery_critical_v, 3.6);
    assert_eq!(cfg.failsafe.battery_low_v, 3.8);
}

/// A flash that logs, for every program call, how many bits it clears.
struct Recorder {
    inner: Flash8,
    calls: Vec<usize>,
}

impl Flash for Recorder {
    const SECTOR_SIZE: usize = Flash8::SECTOR_SIZE;
    fn read(&self, s: usize, o: usize, b: &mut [u8]) -> Result<(), FlashError> {
        self.inner.read(s, o, b)
    }
    fn program(&mut self, s: usize, o: usize, d: &[u8]) -> Result<(), FlashError> {
        self.calls
            .push(self.inner.bits_cleared_by(s, o, d).unwrap());
        self.inner.program(s, o, d)
    }
    fn erase(&mut self, s: usize) -> Result<(), FlashError> {
        self.inner.erase(s)
    }
}

/// A flash holding one earlier saved parameter (LIM_TILT = 0.3), and the
/// firmware configuration with it applied.
fn flash_with_one_saved() -> (Flash8, VehicleConfig<4>) {
    let (mut ps, cfg, report) = boot_on(Flash8::new());
    let mut rig = Rig::new(cfg, report);
    rig.set(ParamId::LIM_TILT, 0.3);
    assert!(matches!(ps.service(&mut rig.core), Ok(Serviced::Saved(_))));
    (ps.into_flash(), *rig.core.config())
}

#[derive(Clone, Copy, Debug)]
enum Cut {
    Bytes(u64),
    Call(u64, BitCut),
}

/// Runs the save of the pair (set in the one order the core accepts), with an
/// optional cut. Returns the flash and what the save returned.
fn save_pair(
    start: &Flash8,
    cut: Option<Cut>,
) -> (ParamStore<Flash8, 4>, Rig, Result<Serviced, crate::Error>) {
    let (mut ps, cfg, report) = boot_on(start.clone());
    assert!(report.clean());
    let mut rig = Rig::new(cfg, report);
    rig.set(ParamId::BAT_LOW_V, 3.8);
    rig.set(ParamId::BAT_CRIT_V, 3.6);
    assert_eq!(rig.core.config().failsafe.battery_critical_v, 3.6);
    match cut {
        None => {}
        Some(Cut::Bytes(n)) => ps.flash_mut().cut_power_after(n, n),
        Some(Cut::Call(c, how)) => ps.flash_mut().cut_program_call(c, how),
    }
    let r = ps.service(&mut rig.core);
    (ps, rig, r)
}

#[test]
fn a_pair_valid_only_together_is_never_half_loaded_whatever_cut_interrupts_the_save() {
    let started = std::time::Instant::now();
    let (start, old) = flash_with_one_saved();
    let mut new = old;
    new.failsafe.battery_low_v = 3.8;
    new.failsafe.battery_critical_v = 3.6;

    // The uncut save, with its program calls and their bit counts.
    let (events, calls) = {
        let rec = Recorder {
            inner: start.clone(),
            calls: Vec::new(),
        };
        let mut cfg = quad();
        let (mut ps, report) = ParamStore::boot(rec, &mut cfg, &PROFILE);
        assert!(report.clean());
        let mut rig = Rig::new(cfg, report);
        rig.set(ParamId::BAT_LOW_V, 3.8);
        rig.set(ParamId::BAT_CRIT_V, 3.6);
        let e0 = ps.flash().inner.events();
        ps.flash_mut().calls.clear();
        assert_eq!(
            ps.service(&mut rig.core),
            Ok(Serviced::Saved(Replaced::Rewritten))
        );
        let rec = ps.into_flash();
        (rec.inner.events() - e0, rec.calls)
    };
    assert!(
        calls.len() >= 8,
        "an erase-and-rewrite has many calls: {calls:?}"
    );

    let mut cuts = Vec::new();
    for n in 0..events {
        cuts.push(Cut::Bytes(n));
    }
    for (c, &bits) in calls.iter().enumerate() {
        let c = c as u64;
        for seed in 0..8 {
            cuts.push(Cut::Call(c, BitCut::Seeded(seed)));
        }
        for k in 0..bits {
            cuts.push(Cut::Call(c, BitCut::AllBut(k)));
            cuts.push(Cut::Call(c, BitCut::Only(k)));
        }
    }

    let (mut got_old, mut got_new) = (0, 0);
    for cut in &cuts {
        let (mut ps, mut rig, r) = save_pair(&start, Some(*cut));
        assert!(r.is_err(), "{cut:?}: a cut inside the save must fail it");
        assert!(ps.flash().is_dead(), "{cut:?}");
        // Power returns; boot from what is on the flash.
        let mut flash = ps.flash().clone();
        flash.restore_power();
        let (_, cfg2, report2) = boot_on(flash);
        assert!(
            report2.clean() && !report2.unreadable,
            "{cut:?}: the report must be clean: {report2:?}"
        );
        if cfg2 == old {
            got_old += 1;
        } else if cfg2 == new {
            got_new += 1;
        } else {
            panic!("{cut:?}: neither the old nor the new configuration: {cfg2:?}");
        }
        // The same running firmware tries again (the core is still unsaved):
        // it remounts, finishes or finds the save done, and the result is the
        // new configuration entirely.
        {
            ps.flash_mut().restore_power();
            assert!(rig.core.params_unsaved());
            assert!(
                matches!(ps.service(&mut rig.core), Ok(Serviced::Saved(_))),
                "{cut:?}: the retry must save"
            );
            let (_, cfg3, report3) = boot_on(ps.into_flash());
            assert!(report3.clean() && cfg3 == new, "{cut:?}: after the retry");
        }
    }
    assert!(got_old > 0 && got_new > 0, "old {got_old}, new {got_new}");
    println!(
        "pair save: {} cut points ({} byte cuts, {} inside {} program calls), \
         old {got_old}, new {got_new}, {:?}",
        cuts.len(),
        events,
        cuts.len() as u64 - events,
        calls.len(),
        started.elapsed()
    );
}

#[test]
fn while_armed_service_never_touches_the_flash_and_after_disarming_it_saves() {
    let (mut ps, cfg, report) = boot_on(Flash8::new());
    let mut rig = Rig::new(cfg, report);
    let (armed, prearm) = rig.try_arm();
    assert!(armed, "{prearm:?}");
    let before = activity(ps.flash());
    for (i, v) in [0.25, 0.3, 0.35].into_iter().enumerate() {
        rig.set(ParamId::LIM_TILT, v);
        assert!(rig.core.params_unsaved());
        assert_eq!(ps.service(&mut rig.core), Ok(Serviced::Idle), "{i}");
        assert_eq!(activity(ps.flash()), before, "nothing programmed or erased");
        assert!(rig.core.params_unsaved(), "still unsaved");
    }
    rig.disarm();
    assert_eq!(
        ps.service(&mut rig.core),
        Ok(Serviced::Saved(Replaced::Appended))
    );
    assert!(activity(ps.flash()) != before);
    let (_, cfg2, report2) = boot_on(ps.into_flash());
    assert!(report2.clean() && cfg2.limits.max_tilt_rad == 0.35);
}

/// Damages both headers of a flash that holds records.
fn corrupt_flash() -> Flash8 {
    let (mut f, _) = flash_with_one_saved();
    for sector in 0..2 {
        let b = f.bytes_mut(sector).unwrap();
        b[0] &= 0x5A;
    }
    f
}

#[test]
fn a_corrupt_flash_boots_unreadable_refuses_arming_and_is_formatted_only_after_the_acknowledgement()
{
    let (mut ps, cfg, report) = boot_on(corrupt_flash());
    assert!(report.unreadable && !report.clean(), "{report:?}");
    assert!(cfg == quad(), "the firmware's own values");
    let mut rig = Rig::new(cfg, report);
    let (armed, prearm) = rig.try_arm();
    assert!(!armed);
    assert_eq!(prearm, PrearmFlags::CONFIG);
    assert!(rig.core.params_fault());
    // A change is applied but stays unsaved, and nothing is written.
    rig.set(ParamId::LIM_TILT, 0.3);
    assert!(rig.core.params_unsaved());
    let before = activity(ps.flash());
    for _ in 0..3 {
        assert_eq!(ps.service(&mut rig.core), Ok(Serviced::Idle));
    }
    assert_eq!(activity(ps.flash()), before);

    let faults = |r: &Rig| {
        r.events
            .iter()
            .filter(|e| matches!(e, EventKind::ParamLoadFault(l) if *l == report))
            .count()
    };
    assert_eq!(faults(&rig), 1, "one fault event, with the report");
    rig.ack();
    assert!(!rig.core.params_fault() && rig.core.params_unsaved());
    assert!(rig.events.contains(&EventKind::ParamFaultAcked));
    assert_eq!(
        ps.service(&mut rig.core),
        Ok(Serviced::Saved(Replaced::Appended)),
        "formatted, then the value in use written"
    );
    assert!(ps.flash().total_erases() > 0);
    assert_eq!(ps.service(&mut rig.core), Ok(Serviced::Idle));

    let (_, cfg2, report2) = boot_on(ps.into_flash());
    assert!(report2.clean() && report2.applied == 1, "{report2:?}");
    assert_eq!(cfg2.limits.max_tilt_rad, 0.3);
    let mut rig2 = Rig::new(cfg2, report2);
    assert!(rig2.try_arm().0, "a clean boot arms");
}

/// Writes raw records beside what a save of LIM_TILT = 0.3 left.
fn flash_with_raw(extra: &[(u32, u32)]) -> Flash8 {
    let mut f = Flash8::new();
    {
        let (mut s, _) = Store::mount(&mut f).unwrap();
        s.set(params::key("LIM_TILT"), 0.3_f32.to_bits()).unwrap();
        for &(k, v) in extra {
            s.set(k, v).unwrap();
        }
    }
    f
}

fn a_value_or_key_this_firmware_does_not_understand(extra: (u32, u32), what: &str) {
    let (mut ps, cfg, report) = boot_on(flash_with_raw(&[extra]));
    assert!(!report.clean() && !report.unreadable, "{what}: {report:?}");
    assert_eq!(report.applied, 1, "{what}: the others load");
    assert_eq!(cfg.limits.max_tilt_rad, 0.3, "{what}");
    let mut rig = Rig::new(cfg, report);
    let (armed, prearm) = rig.try_arm();
    assert!(!armed && prearm == PrearmFlags::CONFIG, "{what}");
    // Force something unsaved: the flash is still untouched.
    rig.set(ParamId::LIM_TILT, 0.3);
    let before = activity(ps.flash());
    assert_eq!(ps.service(&mut rig.core), Ok(Serviced::Idle), "{what}");
    assert_eq!(
        activity(ps.flash()),
        before,
        "{what}: untouched until the ack"
    );
    let mut peek = ps.flash().clone();
    let (s, _) = Store::mount(&mut peek).unwrap();
    assert_eq!(s.get(extra.0), Ok(Some(extra.1)), "{what}: still there");

    rig.ack();
    assert_eq!(
        ps.service(&mut rig.core),
        Ok(Serviced::Saved(Replaced::Appended)),
        "{what}: one tombstone"
    );
    let mut flash = ps.into_flash();
    {
        let (s, _) = Store::mount(&mut flash).unwrap();
        assert_eq!(s.get(extra.0), Ok(None), "{what}: gone from the store");
        assert_eq!(s.get(params::key("LIM_TILT")), Ok(Some(0.3_f32.to_bits())));
    }
    let (_, cfg2, report2) = boot_on(flash);
    assert!(
        report2.clean() && report2.applied == 1,
        "{what}: {report2:?}"
    );
    assert_eq!(cfg2.limits.max_tilt_rad, 0.3);
}

#[test]
fn a_stored_value_out_of_range_is_dropped_held_and_removed_after_the_acknowledgement() {
    let r = report_for_range();
    assert_eq!(r, (1, 0), "dropped 1, unknown 0");
    a_value_or_key_this_firmware_does_not_understand(
        (params::key("LIM_TILT_STAB"), 99.0_f32.to_bits()),
        "out of range",
    );
}

/// What boot reports for the out-of-range record: (dropped, unknown).
fn report_for_range() -> (u16, u16) {
    let (_, _, r) = boot_on(flash_with_raw(&[(
        params::key("LIM_TILT_STAB"),
        99.0_f32.to_bits(),
    )]));
    (r.dropped, r.unknown)
}

#[test]
fn a_stored_key_no_parameter_has_is_counted_held_and_removed_after_the_acknowledgement() {
    let key = 0x0BAD_CAFE;
    assert!(params::find_key(key).is_none());
    let (_, _, r) = boot_on(flash_with_raw(&[(key, 7)]));
    assert_eq!((r.unknown, r.dropped), (1, 0));
    a_value_or_key_this_firmware_does_not_understand((key, 7), "unknown key");
}

/// A flash whose reads fail from the `n`-th read on (counted from boot).
struct FailsFrom {
    inner: Flash8,
    reads: Cell<u64>,
    from: Cell<Option<u64>>,
}

impl Flash for FailsFrom {
    const SECTOR_SIZE: usize = Flash8::SECTOR_SIZE;
    fn read(&self, s: usize, o: usize, b: &mut [u8]) -> Result<(), FlashError> {
        let n = self.reads.get();
        self.reads.set(n + 1);
        match self.from.get() {
            Some(f) if n >= f => Err(FlashError::Device),
            _ => self.inner.read(s, o, b),
        }
    }
    fn program(&mut self, s: usize, o: usize, d: &[u8]) -> Result<(), FlashError> {
        self.inner.program(s, o, d)
    }
    fn erase(&mut self, s: usize) -> Result<(), FlashError> {
        self.inner.erase(s)
    }
}

#[test]
fn a_flash_that_fails_a_read_at_boot_is_unreadable_with_the_firmwares_values_at_any_read() {
    let (start, saved) = flash_with_one_saved();
    // How many reads an unfailing boot makes.
    let total = {
        let f = FailsFrom {
            inner: start.clone(),
            reads: Cell::new(0),
            from: Cell::new(None),
        };
        let mut cfg = quad();
        let (ps, r) = ParamStore::boot(f, &mut cfg, &PROFILE);
        assert!(r.clean() && cfg == saved);
        ps.flash().reads.get()
    };
    assert!(total > 10);
    for from in 0..=total {
        let f = FailsFrom {
            inner: start.clone(),
            reads: Cell::new(0),
            from: Cell::new(Some(from)),
        };
        let mut cfg = quad();
        let (mut ps, report) = ParamStore::boot(f, &mut cfg, &PROFILE);
        if from < total {
            assert!(report.unreadable, "from {from}: {report:?}");
            assert!(cfg == quad(), "from {from}: no half-applied values");
            // The device heals; the operator acknowledges; the store is
            // rewritten from the values in use.
            ps.flash_mut().from.set(None);
            let mut rig = Rig::new(cfg, report);
            let before = activity(&ps.flash().inner);
            assert_eq!(ps.service(&mut rig.core), Ok(Serviced::Idle));
            rig.ack();
            assert!(matches!(ps.service(&mut rig.core), Ok(Serviced::Saved(_))));
            assert!(activity(&ps.flash().inner) != before || from > 0);
            let (_, cfg2, report2) = boot_on(ps.into_flash().inner);
            assert!(report2.clean(), "from {from}: {report2:?}");
            assert!(cfg2 == quad(), "rewritten from the values in use");
        } else {
            assert!(report.clean() && cfg == saved, "no failure reached");
        }
    }
}

#[test]
fn the_store_keeps_the_firmwares_own_values_so_a_save_cannot_be_given_the_wrong_ones() {
    // Two values stored; boot loads them. The store still knows the
    // firmware's own configuration, not the loaded one.
    let (mut ps, cfg, report) = boot_on(flash_with_raw(&[(
        params::key("BAT_LOW_V"),
        3.8_f32.to_bits(),
    )]));
    assert!(report.clean() && report.applied == 2, "{report:?}");
    assert!(*ps.baseline() == quad());
    assert!(cfg != quad());
    // Another parameter changes; the save keeps all three.
    let mut rig = Rig::new(cfg, report);
    rig.set(ParamId::LIM_RATE_ROLL, 3.0);
    assert!(matches!(ps.service(&mut rig.core), Ok(Serviced::Saved(_))));
    let (_, cfg2, report2) = boot_on(ps.into_flash());
    assert!(report2.clean() && report2.applied == 3, "{report2:?}");
    assert!(cfg2 == *rig.core.config());
    assert_eq!(cfg2.limits.max_tilt_rad, 0.3);
    assert_eq!(cfg2.failsafe.battery_low_v, 3.8);
}

#[test]
fn a_record_damaged_in_the_store_is_reported_by_boot_as_torn_and_the_vehicle_still_arms() {
    // Two saves of one parameter: 0.3, then 0.5 (two appended records).
    let (mut ps, cfg, report) = boot_on(Flash8::new());
    let mut rig = Rig::new(cfg, report);
    for v in [0.3, 0.5] {
        rig.set(ParamId::LIM_TILT, v);
        assert_eq!(
            ps.service(&mut rig.core),
            Ok(Serviced::Saved(Replaced::Appended))
        );
    }
    let mut flash = ps.into_flash();
    // One bit of the newer record's value (slot 1: header 16 + 12, value at +4).
    flash.bytes_mut(0).unwrap()[16 + 12 + 4] ^= 0x01;

    let (ps2, cfg2, report2) = boot_on(flash);
    assert_eq!(cfg2.limits.max_tilt_rad, 0.3, "the older value");
    assert!(report2.clean() && report2.applied == 1, "{report2:?}");
    assert_eq!(report2.torn, 1, "{report2:?}");
    assert_eq!(ps2.torn_records(), 1);

    let mut rig2 = Rig::new(cfg2, report2);
    let (armed, prearm) = rig2.try_arm();
    assert!(armed, "{prearm:?}");
    assert!(!rig2.core.params_fault());
    let recovered: Vec<_> = rig2
        .events
        .iter()
        .filter(|e| matches!(e, EventKind::ParamLoadRecovered(r) if *r == report2))
        .collect();
    assert_eq!(recovered.len(), 1, "{:?}", rig2.events);
}

#[test]
fn a_clean_store_reports_no_torn_records() {
    let (flash, _) = flash_with_one_saved();
    let (_, _, report) = boot_on(flash);
    assert!(report.clean() && report.torn == 0, "{report:?}");
}

/// A flash whose programs fail while `fail_programs` is nonzero (each failure
/// uses one up; `u32::MAX` never runs out).
struct BadCells {
    inner: Flash8,
    fail_programs: Cell<u32>,
}

impl Flash for BadCells {
    const SECTOR_SIZE: usize = Flash8::SECTOR_SIZE;
    fn read(&self, s: usize, o: usize, b: &mut [u8]) -> Result<(), FlashError> {
        self.inner.read(s, o, b)
    }
    fn program(&mut self, s: usize, o: usize, d: &[u8]) -> Result<(), FlashError> {
        let n = self.fail_programs.get();
        if n > 0 {
            if n != u32::MAX {
                self.fail_programs.set(n - 1);
            }
            return Err(FlashError::Device);
        }
        self.inner.program(s, o, d)
    }
    fn erase(&mut self, s: usize) -> Result<(), FlashError> {
        self.inner.erase(s)
    }
}

fn bad_cells(fail: u32) -> (ParamStore<BadCells, 4>, Rig) {
    let flash = BadCells {
        inner: Flash8::new(),
        fail_programs: Cell::new(0),
    };
    let mut cfg = quad();
    let (ps, report) = ParamStore::boot(flash, &mut cfg, &PROFILE);
    let mut rig = Rig::new(cfg, report);
    // Two changes: the save is a rewrite (one erase per attempt).
    rig.set(ParamId::LIM_TILT, 0.3);
    rig.set(ParamId::LIM_RATE_ROLL, 3.0);
    ps.flash().fail_programs.set(fail);
    (ps, rig)
}

#[test]
fn a_save_that_keeps_failing_erases_three_times_and_then_gives_up_until_the_next_boot() {
    let (mut ps, mut rig) = bad_cells(u32::MAX);
    assert!(!ps.gave_up());
    for call in 0..3 {
        assert!(
            matches!(ps.service(&mut rig.core), Err(Error::Flash(_))),
            "call {call}"
        );
        assert_eq!(ps.flash().inner.total_erases(), call + 1, "call {call}");
    }
    assert!(ps.gave_up());
    let before = activity(&ps.flash().inner);
    let reads = ps.flash().inner.reads();
    for call in 0..20 {
        assert_eq!(ps.service(&mut rig.core), Err(Error::GaveUp), "call {call}");
    }
    assert_eq!(activity(&ps.flash().inner), before, "no write");
    assert_eq!(ps.flash().inner.reads(), reads, "not even a read");
    assert_eq!(ps.flash().inner.total_erases(), 3);
    assert!(rig.core.params_unsaved(), "still unsaved");

    // The next boot starts again (the flash has healed).
    let flash = ps.into_flash();
    flash.fail_programs.set(0);
    let mut cfg = quad();
    let (mut ps2, report) = ParamStore::boot(flash, &mut cfg, &PROFILE);
    assert!(!ps2.gave_up());
    let mut rig2 = Rig::new(cfg, report);
    rig2.set(ParamId::LIM_TILT, 0.3);
    assert!(matches!(
        ps2.service(&mut rig2.core),
        Ok(Serviced::Saved(_))
    ));
}

#[test]
fn a_save_that_fails_once_and_then_works_saves_on_the_second_call_and_the_count_resets() {
    let (mut ps, mut rig) = bad_cells(1);
    assert!(matches!(ps.service(&mut rig.core), Err(Error::Flash(_))));
    assert!(!ps.gave_up());
    assert!(matches!(
        ps.service(&mut rig.core),
        Ok(Serviced::Saved(Replaced::Rewritten))
    ));
    assert!(!rig.core.params_unsaved());
    // Two failures, a success, two more failures: never three in a row.
    for round in 0..3 {
        rig.set(ParamId::LIM_TILT, 0.2 + 0.05 * round as f32);
        rig.set(ParamId::LIM_RATE_ROLL, 3.0 + 0.1 * round as f32);
        ps.flash().fail_programs.set(u32::MAX);
        for _ in 0..2 {
            assert!(matches!(ps.service(&mut rig.core), Err(Error::Flash(_))));
        }
        ps.flash().fail_programs.set(0);
        assert!(matches!(ps.service(&mut rig.core), Ok(Serviced::Saved(_))));
        assert!(!ps.gave_up(), "round {round}");
    }
}
