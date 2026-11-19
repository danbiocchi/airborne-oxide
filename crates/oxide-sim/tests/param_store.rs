// SPDX-License-Identifier: AGPL-3.0-or-later
//! Parameters on flash, end to end (ADR-0004 §6): a simulated flight boots
//! from a `ParamStore` on a simulated flash, a parameter is changed in flight,
//! the store is written only after the disarm, and a second boot reads it
//! back. A power cut during the save leaves the old or the new values and a
//! vehicle that arms.

use oxide_core::config::VehicleConfig;
use oxide_core::io::{EventKind, ParamSet};
use oxide_core::params::{LoadReport, ParamId, ParamValue};
use oxide_core::schedule::{F405_LEAN, Profile};
use oxide_sim::scenario::Script;
use oxide_sim::{Airframe, SimOptions, Simulation};
use oxide_store::sim::{BitCut, SimFlash};
use oxide_store::{ParamStore, Replaced, Serviced};

const PROFILE: Profile = F405_LEAN;
type Flash = SimFlash<1024>;
type Sim = Simulation<Script, 4>;

/// Arm at 2 s, climb, hover at about 4 m, land (on the ground by 11 s),
/// disarm at 13 s, run on to 16 s.
fn script() -> Script {
    Script {
        arm_at_s: 2.0,
        disarm_at_s: Some(13.0),
        mode: vec![(0.0, 1)],
        sticks: vec![
            (0.0, [0.0, 0.0, 0.0, 0.0]),
            (2.5, [0.0, 0.0, 0.0, 1.0]),
            (4.0, [0.0, 0.0, 0.0, 0.5]),
            (8.0, [0.0, 0.0, 0.0, 0.0]),
        ],
        duration_s: 16.0,
        ..Script::default()
    }
}

/// What a boot gave: the simulation, its store, the firmware's own
/// configuration (the baseline for saving) and the load report.
struct Booted {
    sim: Sim,
    store: ParamStore<Flash, 4>,
    baseline: VehicleConfig<4>,
    report: LoadReport,
}

fn boot(flash: Flash) -> Booted {
    let airframe = Airframe::reference_quad().expect("reference airframe");
    let mut flash = Some(flash);
    let mut store = None;
    let mut report = LoadReport::default();
    let sim = Sim::with_boot(&airframe, PROFILE, script(), SimOptions::default(), |cfg| {
        let (s, r) = ParamStore::boot(flash.take().expect("once"), cfg, &PROFILE);
        store = Some(s);
        report = r;
        r
    })
    .expect("simulation");
    let store = store.expect("booted");
    let baseline = *store.baseline();
    Booted {
        sim,
        store,
        baseline,
        report,
    }
}

fn activity(f: &Flash) -> (u64, u64, u32) {
    (f.events(), f.program_calls(), f.total_erases())
}

fn live(id: ParamId, v: f32) -> ParamSet {
    ParamSet {
        id,
        value: ParamValue::F32(v),
    }
}

/// How to interrupt the save.
#[derive(Clone, Copy, Debug)]
enum Cut {
    Bytes(u64),
    Call(u64, BitCut),
}

/// What a flight showed.
#[derive(Debug, Default)]
struct Flight {
    /// Ticks on which the core was armed (first, last).
    armed: Option<(u64, u64)>,
    /// The tick on which a save finished, and what it did.
    saved: Option<(u64, Replaced)>,
    /// The tick on which a save failed (a cut).
    failed: Option<u64>,
    /// `ParamChanged` events seen.
    changed: usize,
    /// Flash events and program calls the save took.
    save_cost: Option<(u64, u64)>,
}

/// Flies `b` to the end of its script. `changes` are `(time in s, change)`,
/// each put into the next tick's inputs once. `service` is called after every
/// tick; the flash is asserted untouched while the core is armed.
fn fly(b: &mut Booted, changes: &[(f64, ParamSet)], cut: Option<Cut>) -> Flight {
    let mut f = Flight::default();
    let mut next_change = 0;
    let mut at_arming = None;
    let mut cut_armed = cut.is_none();
    let mut k = 0_u64;
    while b.sim.time().as_secs_f64() < 16.0 {
        if let Some((t, ps)) = changes.get(next_change)
            && b.sim.time().as_secs_f64() >= *t
        {
            b.sim.queue_param_set(*ps);
            next_change += 1;
        }
        let rec = b.sim.step();
        f.changed += rec
            .core
            .as_ref()
            .expect("core part")
            .events
            .iter()
            .filter(|e| matches!(e.kind, EventKind::ParamChanged(_)))
            .count();
        let armed = b.sim.core().is_armed();
        if armed {
            let first = f.armed.map_or(k, |(a, _)| a);
            f.armed = Some((first, k));
            let seen = *at_arming.get_or_insert_with(|| activity(b.store.flash()));
            assert_eq!(
                activity(b.store.flash()),
                seen,
                "tick {k}: the flash was touched while armed"
            );
        }
        // Arm the cut once the vehicle is on the ground and disarmed, with
        // something to save.
        if !cut_armed && f.armed.is_some() && !armed && b.sim.core().params_unsaved() {
            match cut {
                Some(Cut::Bytes(n)) => b.store.flash_mut().cut_power_after(n, n ^ 0x5eed),
                Some(Cut::Call(c, how)) => b.store.flash_mut().cut_program_call(c, how),
                None => {}
            }
            cut_armed = true;
        }
        let before = activity(b.store.flash());
        let r = b.store.service(b.sim.core_mut());
        match r {
            Ok(Serviced::Idle) => {
                assert_eq!(activity(b.store.flash()), before, "tick {k}: idle writes");
            }
            Ok(Serviced::Saved(how)) => {
                assert!(!armed, "tick {k}: saved while armed");
                f.saved = Some((k, how));
                let after = activity(b.store.flash());
                f.save_cost = Some((after.0 - before.0, after.1 - before.1));
            }
            Err(_) => {
                assert!(cut.is_some(), "tick {k}: an unexpected store error {r:?}");
                f.failed = Some(k);
                break;
            }
        }
        k += 1;
    }
    f
}

/// Boots a new simulation from `flash` and runs it until it arms (or 6 s).
fn arms_after_boot(flash: Flash) -> (VehicleConfig<4>, LoadReport) {
    let mut b = boot(flash);
    assert!(!b.sim.core().params_fault(), "{:?}", b.report);
    let cfg = *b.sim.core().config();
    let mut armed = false;
    while b.sim.time().as_secs_f64() < 6.0 {
        let r = b.sim.step();
        armed |= r.status().expect("core part").armed;
    }
    assert!(armed, "the vehicle arms after this boot");
    (cfg, b.report)
}

#[test]
fn a_parameter_changed_in_flight_is_saved_only_after_the_disarm_and_a_second_boot_has_it() {
    let mut a = boot(Flash::new());
    assert!(a.report.clean(), "{:?}", a.report);
    assert!(*a.sim.core().config() == a.baseline);
    let f = fly(&mut a, &[(8.0, live(ParamId::LIM_TILT, 0.5))], None);

    assert_eq!(f.changed, 1, "one ParamChanged event");
    let (first, last) = f.armed.expect("the vehicle armed");
    assert!(
        first < last && last > 10_000,
        "armed over a flight: {first}..{last}"
    );
    let (saved_at, how) = f.saved.expect("the change was saved");
    assert!(
        saved_at > last,
        "saved at {saved_at}, after the last armed tick {last}"
    );
    assert_eq!(how, Replaced::Appended);
    assert!(!a.sim.core().params_unsaved());
    assert_eq!(a.store.flash().total_erases(), 0, "one appended record");

    let (cfg, report) = arms_after_boot(a.store.into_flash());
    assert!(report.clean() && report.applied == 1, "{report:?}");
    assert_eq!(cfg.limits.max_tilt_rad, 0.5);
    let mut want = a.baseline;
    want.limits.max_tilt_rad = 0.5;
    assert!(cfg == want, "everything else is the firmware's own");
}

#[test]
fn a_cut_during_the_save_of_a_pair_leaves_the_old_or_the_new_values_and_a_vehicle_that_arms() {
    let started = std::time::Instant::now();
    // The pair is valid only together; set in the one order a run-time change
    // accepts, on consecutive ticks.
    let pair = [
        (8.0, live(ParamId::BAT_LOW_V, 3.8)),
        (8.002, live(ParamId::BAT_CRIT_V, 3.6)),
    ];
    // An uncut flight measures the save.
    let mut u = boot(Flash::new());
    let f = fly(&mut u, &pair, None);
    assert_eq!(f.saved.map(|s| s.1), Some(Replaced::Rewritten));
    let (events, calls) = f.save_cost.expect("a save");
    assert!(events >= 40 && calls >= 7, "{events} events, {calls} calls");
    let old = u.baseline;
    let mut new = old;
    new.failsafe.battery_low_v = 3.8;
    new.failsafe.battery_critical_v = 3.6;
    let (cfg, report) = arms_after_boot(u.store.into_flash());
    assert!(report.clean() && cfg == new);

    let (mut got_old, mut got_new) = (0, 0);
    let seeds = 96_u64;
    for seed in 0..seeds {
        let call = seed % calls;
        let cut = match seed % 3 {
            0 => Cut::Bytes(seed.wrapping_mul(7919) % events),
            1 => Cut::Call(call, BitCut::Seeded(seed)),
            _ if seed % 2 == 0 => Cut::Call(call, BitCut::AllBut(seed as usize % 9)),
            _ => Cut::Call(call, BitCut::Only(seed as usize % 9)),
        };
        let mut a = boot(Flash::new());
        let f = fly(&mut a, &pair, Some(cut));
        assert!(
            f.failed.is_some() && f.saved.is_none(),
            "{cut:?}: the cut must fail the save"
        );
        let mut flash = a.store.into_flash();
        flash.restore_power();
        let (cfg, report) = arms_after_boot(flash);
        assert!(report.clean(), "{cut:?}: {report:?}");
        if cfg == old {
            got_old += 1;
        } else if cfg == new {
            got_new += 1;
        } else {
            panic!("{cut:?}: neither the old nor the new values: {cfg:?}");
        }
    }
    assert!(got_old > 0, "some cut leaves the old values");
    println!(
        "{seeds} seeded cuts of a save of {events} events in {calls} program calls: \
         old {got_old}, new {got_new}, {:?}",
        started.elapsed()
    );
}
