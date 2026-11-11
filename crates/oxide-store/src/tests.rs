// SPDX-License-Identifier: AGPL-3.0-or-later
//! Store tests: behaviour, exhaustive power cuts, random sequences with cuts.

use std::collections::BTreeMap;
use std::time::Instant;

use crate::flash::{Flash, FlashError};
use crate::sim::SimFlash;
use crate::store::{Error, MountReport, Store};

/// Six record slots per sector: (88 - 16) / 12.
const SMALL: usize = 16 + 12 * 6;
type Small = SimFlash<SMALL>;
type Model = BTreeMap<u32, u32>;

#[derive(Clone, Copy, Debug)]
enum Op {
    Set(u32, u32),
    Remove(u32),
}

impl Op {
    fn apply(self, m: &mut Model) {
        match self {
            Self::Set(k, v) => {
                m.insert(k, v);
            }
            Self::Remove(k) => {
                m.remove(&k);
            }
        }
    }

    fn run<F: Flash>(self, s: &mut Store<F>) -> Result<(), Error> {
        match self {
            Self::Set(k, v) => s.set(k, v),
            Self::Remove(k) => s.remove(k),
        }
    }
}

fn contents<F: Flash>(s: &Store<F>) -> Model {
    s.iter().collect()
}

/// `get` and `iter` agree with each other and with `model`.
fn assert_matches<F: Flash>(s: &Store<F>, model: &Model, ctx: &str) {
    assert_eq!(&contents(s), model, "iter differs: {ctx}");
    for (&k, &v) in model {
        assert_eq!(s.get(k), Some(v), "get({k}): {ctx}");
    }
}

fn fresh() -> Small {
    Small::new()
}

#[test]
fn a_set_then_a_remount_reads_back() {
    let mut f = fresh();
    let (mut s, r) = Store::mount(&mut f).unwrap();
    assert_eq!(r, MountReport::Fresh);
    assert_eq!(s.get(1), None);
    s.set(1, 10).unwrap();
    s.set(2, 20).unwrap();
    s.set(u32::MAX, u32::MAX).unwrap();
    let (s, r) = Store::mount(&mut f).unwrap();
    assert_eq!(r, MountReport::Clean);
    assert_eq!(s.get(1), Some(10));
    assert_eq!(s.get(2), Some(20));
    assert_eq!(s.get(u32::MAX), Some(u32::MAX));
    assert_eq!(s.get(3), None);
}

#[test]
fn the_newest_value_wins() {
    let mut f = fresh();
    let (mut s, _) = Store::mount(&mut f).unwrap();
    s.set(1, 1).unwrap();
    s.set(1, 2).unwrap();
    s.set(1, 3).unwrap();
    assert_eq!(s.get(1), Some(3));
    assert_eq!(contents(&s), Model::from([(1, 3)]));
    let (s, _) = Store::mount(&mut f).unwrap();
    assert_eq!(s.get(1), Some(3));
}

#[test]
fn remove_hides_the_key_and_a_later_set_revives_it() {
    let mut f = fresh();
    let (mut s, _) = Store::mount(&mut f).unwrap();
    s.set(1, 5).unwrap();
    s.set(2, 6).unwrap();
    s.remove(1).unwrap();
    assert_eq!(s.get(1), None);
    assert_eq!(contents(&s), Model::from([(2, 6)]));
    let (mut s, r) = Store::mount(&mut f).unwrap();
    assert_eq!(r, MountReport::Clean);
    assert_eq!(s.get(1), None);
    s.set(1, 0).unwrap();
    assert_eq!(s.get(1), Some(0));
}

#[test]
fn removing_an_absent_key_writes_nothing() {
    let mut f = fresh();
    let (mut s, _) = Store::mount(&mut f).unwrap();
    s.set(1, 1).unwrap();
    s.remove(1).unwrap();
    s.remove(1).unwrap();
    s.remove(99).unwrap();
    assert_eq!(f.events(), 16 /* header at mount */ + 12 + 12);
}

#[test]
fn a_full_sector_compacts_and_keeps_every_live_key() {
    let mut f = fresh();
    let (mut s, _) = Store::mount(&mut f).unwrap();
    for k in 1..=3 {
        s.set(k, k * 10).unwrap();
    }
    for i in 0..3 {
        s.set(1, 100 + i).unwrap(); // sixth record fills the sector
    }
    assert_eq!(s.erase_count(), 0);
    s.set(4, 40).unwrap(); // seventh record: compacts
    assert_eq!(s.erase_count(), 1);
    let want = Model::from([(1, 102), (2, 20), (3, 30), (4, 40)]);
    assert_matches(&s, &want, "after compaction");
    let (s, r) = Store::mount(&mut f).unwrap();
    assert_eq!(r, MountReport::Clean);
    assert_matches(&s, &want, "after remount");
}

#[test]
fn erase_count_rises_by_one_per_compaction() {
    let mut f = fresh();
    let (mut s, _) = Store::mount(&mut f).unwrap();
    let (mut used, mut compactions) = (0_usize, 0_u32);
    for i in 0..40 {
        s.set(7, i).unwrap();
        if used == 6 {
            compactions += 1;
            used = 1;
        } else {
            used += 1;
        }
        assert_eq!(s.erase_count(), compactions, "after set {i}");
    }
    assert_eq!(compactions, 6);
    assert_eq!(f.total_erases(), 6);
    let (s, _) = Store::mount(&mut f).unwrap();
    assert_eq!(s.erase_count(), 6);
    assert_eq!(s.get(7), Some(39));
}

#[test]
fn more_live_keys_than_slots_is_full_and_changes_nothing() {
    let mut f = fresh();
    let (mut s, _) = Store::mount(&mut f).unwrap();
    for k in 1..=6 {
        s.set(k, k).unwrap();
    }
    assert_eq!(s.set(7, 7), Err(Error::Full));
    // Replacing an existing key of a full sector still works, via compaction.
    s.set(3, 33).unwrap();
    assert_eq!(s.get(3), Some(33));
    assert_eq!(s.get(7), None);
    s.remove(6).unwrap();
    s.set(7, 7).unwrap();
    assert_eq!(
        contents(&s),
        Model::from([(1, 1), (2, 2), (3, 33), (4, 4), (5, 5), (7, 7)])
    );
}

#[test]
fn removing_from_a_full_sector_compacts_the_key_away() {
    let mut f = fresh();
    let (mut s, _) = Store::mount(&mut f).unwrap();
    for i in 0..5 {
        s.set(1, i).unwrap();
    }
    s.set(2, 2).unwrap(); // full
    s.remove(1).unwrap();
    assert_eq!(s.erase_count(), 1);
    assert_eq!(contents(&s), Model::from([(2, 2)]));
}

#[test]
fn a_torn_record_is_skipped_and_never_reused() {
    let mut f = fresh();
    let (mut s, _) = Store::mount(&mut f).unwrap();
    s.set(1, 1).unwrap();
    f.cut_power_after(5, 0);
    let (mut s, _) = Store::mount(&mut f).unwrap();
    assert_eq!(s.set(2, 2), Err(Error::Flash(FlashError::PowerLoss)));
    assert_eq!(s.set(3, 3), Err(Error::NeedsRemount));
    f.restore_power();
    let (mut s, r) = Store::mount(&mut f).unwrap();
    assert_eq!(r, MountReport::Recovered);
    assert_eq!(s.torn_records(), 1);
    assert_eq!(s.get(2), None);
    s.set(3, 3).unwrap();
    assert_eq!(contents(&s), Model::from([(1, 1), (3, 3)]));
    // The torn slot is still there and still counted; the new record is after it.
    let (s, r) = Store::mount(&mut f).unwrap();
    assert_eq!((r, s.torn_records()), (MountReport::Recovered, 1));
    assert_eq!(s.get(3), Some(3));
}

#[test]
fn both_sectors_damaged_is_corrupt_and_format_recovers() {
    let mut f = fresh();
    {
        let (mut s, _) = Store::mount(&mut f).unwrap();
        for i in 0..9 {
            s.set(1, i).unwrap(); // leaves both sectors written
        }
    }
    for sector in 0..2 {
        f.bytes_mut(sector)
            .unwrap()
            .iter_mut()
            .for_each(|b| *b &= 0x5A);
    }
    let (mut s, r) = Store::mount(&mut f).unwrap();
    assert_eq!(r, MountReport::Corrupt);
    assert_eq!(s.get(1), None);
    assert_eq!(contents(&s), Model::new());
    assert_eq!(s.set(1, 1), Err(Error::Corrupt));
    assert_eq!(s.remove(1), Err(Error::Corrupt));
    s.format().unwrap();
    s.set(1, 42).unwrap();
    assert_eq!(s.get(1), Some(42));
    let before = s.erase_count();
    assert!(before >= 1, "format accounts for the erases it made");
    let (s, r) = Store::mount(&mut f).unwrap();
    assert_eq!(r, MountReport::Clean);
    assert_eq!(s.get(1), Some(42));
    assert_eq!(s.erase_count(), before);
}

#[test]
fn one_damaged_sector_is_recovered_from_the_other() {
    let mut f = fresh();
    {
        let (mut s, _) = Store::mount(&mut f).unwrap();
        for i in 0..8 {
            s.set(1, i).unwrap(); // active is now sector 1, sector 0 older
        }
        s.set(2, 2).unwrap();
    }
    f.bytes_mut(0).unwrap().fill(0x00);
    let (s, r) = Store::mount(&mut f).unwrap();
    assert_eq!(r, MountReport::Recovered);
    assert_eq!(contents(&s), Model::from([(1, 7), (2, 2)]));
}

#[test]
fn a_store_that_is_not_blank_but_has_no_valid_sector_is_not_formatted_by_mount() {
    let mut f = fresh();
    f.bytes_mut(0).unwrap()[60] = 0x00;
    let (_, r) = Store::mount(&mut f).unwrap();
    assert_eq!(r, MountReport::Corrupt);
    assert_eq!(f.bytes(0).unwrap()[60], 0x00);
    assert_eq!(f.total_erases(), 0);
}

#[test]
fn a_flash_read_error_is_an_error_not_a_panic() {
    struct Broken;
    impl Flash for Broken {
        const SECTOR_SIZE: usize = 64;
        fn read(&self, _: usize, _: usize, _: &mut [u8]) -> Result<(), FlashError> {
            Err(FlashError::Device)
        }
        fn program(&mut self, _: usize, _: usize, _: &[u8]) -> Result<(), FlashError> {
            Err(FlashError::Device)
        }
        fn erase(&mut self, _: usize) -> Result<(), FlashError> {
            Err(FlashError::Device)
        }
    }
    assert!(matches!(
        Store::mount(Broken),
        Err(Error::Flash(FlashError::Device))
    ));
}

#[test]
fn the_real_sector_size_holds_1364_records() {
    type Big = SimFlash<16384>;
    assert_eq!(Store::<Big>::SLOTS, 1364);
    let mut f = Big::new();
    let (mut s, _) = Store::mount(&mut f).unwrap();
    for k in 0..1364_u32 {
        s.set(k, !k).unwrap();
    }
    assert_eq!(s.erase_count(), 0);
    assert_eq!(s.set(5000, 1), Err(Error::Full));
    s.set(1000, 7).unwrap(); // full sector of live keys: compacts
    assert_eq!(s.erase_count(), 1);
    assert_eq!(s.get(1000), Some(7));
    assert_eq!(s.get(1363), Some(!1363));
    assert_eq!(s.iter().count(), 1364);
}

#[test]
fn worst_case_cost_of_a_16k_compaction() {
    type Big = SimFlash<16384>;
    let mut f = Big::new();
    let (mut s, _) = Store::mount(&mut f).unwrap();
    for k in 0..1364_u32 {
        s.set(k, k).unwrap();
    }
    let (mut s, _) = Store::mount(&mut f).unwrap();
    let t = Instant::now();
    s.set(1, 99).unwrap(); // every record live: the worst case
    let dt = t.elapsed();
    let reads = f.reads();
    eprintln!(
        "16 KB worst-case compaction: {} flash reads in this test (incl. mount), {dt:?} on the host",
        reads
    );
    // About n(n-1) key reads for the two passes, plus 3 per record.
    assert!(reads < 4_000_000, "{reads}");
}

// ---------------------------------------------------------------- power cuts

/// The fixed script: forces two compactions on a 6-slot sector, and removes.
/// It ends with the sector full, so the follow-up starts with a compaction.
const SCRIPT: [Op; 13] = [
    Op::Set(1, 10),
    Op::Set(2, 20),
    Op::Set(3, 30),
    Op::Set(1, 11),
    Op::Set(2, 21),
    Op::Set(1, 12),
    Op::Set(3, 31), // seventh record: compaction 1
    Op::Remove(2),
    Op::Set(1, 13),
    Op::Set(u32::MAX, 7),
    Op::Set(4, 40),
    Op::Set(1, 14), // full again: compaction 2
    Op::Remove(4),  // the sixth record: the sector is full again
];

/// Used after a first cut to prove the store carries on (and survives a second).
const FOLLOW_UP: [Op; 9] = [
    Op::Set(1, 100),
    Op::Set(2, 200),
    Op::Set(3, 300),
    Op::Set(4, 400),
    Op::Remove(1),
    Op::Set(5, 500),
    Op::Set(2, 201),
    Op::Set(1, 101),
    Op::Set(3, 301),
];

const UNIVERSE: [u32; 7] = [1, 2, 3, 4, 5, u32::MAX, KEY_CRC_ZERO];

/// Records what each flash event was, to know where erases are.
struct Recorder<'a> {
    flash: &'a mut Small,
    log: Vec<bool>,
}

impl Flash for Recorder<'_> {
    const SECTOR_SIZE: usize = SMALL;
    fn read(&self, s: usize, o: usize, b: &mut [u8]) -> Result<(), FlashError> {
        self.flash.read(s, o, b)
    }
    fn program(&mut self, s: usize, o: usize, d: &[u8]) -> Result<(), FlashError> {
        self.flash.program(s, o, d)?;
        self.log.extend(d.iter().map(|_| false));
        Ok(())
    }
    fn erase(&mut self, s: usize) -> Result<(), FlashError> {
        self.flash.erase(s)?;
        self.log.push(true);
        Ok(())
    }
}

/// Events of `ops` run on a copy of `flash`, in order: `true` for an erase.
fn event_kinds(flash: &Small, ops: &[Op]) -> Vec<bool> {
    let mut copy = flash.clone();
    let mut rec = Recorder {
        flash: &mut copy,
        log: Vec::new(),
    };
    let (mut s, _) = Store::mount(&mut rec).unwrap();
    for op in ops {
        op.run(&mut s).unwrap();
    }
    rec.log
}

/// Runs `ops` against a store on `flash` with the power cut at event `k` of
/// the run. Returns the index of the interrupted operation, if any.
fn run_with_cut(flash: &mut Small, ops: &[Op], cut: Option<(u64, u64)>) -> Option<usize> {
    let (mut s, _) = Store::mount(&mut *flash).unwrap();
    if let Some((k, seed)) = cut {
        s.flash_mut().cut_power_after(k, seed);
    }
    for (i, op) in ops.iter().enumerate() {
        match op.run(&mut s) {
            Ok(()) => {}
            Err(Error::Flash(FlashError::PowerLoss)) => return Some(i),
            Err(e) => panic!("unexpected {e:?} at op {i} ({op:?})"),
        }
    }
    None
}

/// States before and after each op of `ops` starting from `start`.
fn models(start: &Model, ops: &[Op]) -> Vec<Model> {
    let mut v = vec![start.clone()];
    for op in ops {
        let mut m = v.last().unwrap().clone();
        op.apply(&mut m);
        v.push(m);
    }
    v
}

/// After a cut during op `j`: every key is as before op `j` or as after it,
/// every other key is untouched, and `iter` agrees with `get`.
fn assert_old_or_new(s: &Store<&mut Small>, before: &Model, after: &Model, ctx: &str) {
    for &k in &UNIVERSE {
        let got = s.get(k);
        let (b, a) = (before.get(&k).copied(), after.get(&k).copied());
        assert!(
            got == b || got == a,
            "key {k}: got {got:?}, before {b:?}, after {a:?}: {ctx}"
        );
    }
    let listed = contents(s);
    for &k in &UNIVERSE {
        assert_eq!(
            listed.get(&k).copied(),
            s.get(k),
            "iter vs get for {k}: {ctx}"
        );
    }
    assert!(
        listed.keys().all(|k| UNIVERSE.contains(k)),
        "stray key: {ctx}"
    );
}

const ERASE_SEEDS: u64 = 16;
const FOLLOW_UP_ERASE_SEEDS: u64 = 4;

#[test]
fn power_cut_at_every_point_of_every_operation() {
    let t = Instant::now();
    // Dry run of the script on a formatted flash.
    let mut base = fresh();
    let _ = Store::mount(&mut base).unwrap();
    let kinds = event_kinds(&base, &SCRIPT);
    let programmed = kinds.iter().filter(|e| !**e).count();
    let erases = kinds.len() - programmed;
    assert_eq!(erases, 2, "script must force exactly two compactions");
    let expected = models(&Model::new(), &SCRIPT);

    let (mut first_cuts, mut second_cuts) = (0_u64, 0_u64);
    let (mut recovered, mut clean) = (0_u64, 0_u64);
    for (k, &is_erase) in kinds.iter().enumerate() {
        for seed in 0..(if is_erase { ERASE_SEEDS } else { 1 }) {
            first_cuts += 1;
            let ctx = format!("first cut at event {k} seed {seed}");
            let mut flash = base.clone();
            let j = run_with_cut(&mut flash, &SCRIPT, Some((k as u64, seed)))
                .unwrap_or_else(|| panic!("no cut happened: {ctx}"));
            assert!(flash.is_dead(), "{ctx}");
            flash.restore_power();

            let (s, report) =
                Store::mount(&mut flash).unwrap_or_else(|e| panic!("mount: {e} ({ctx})"));
            assert_ne!(report, MountReport::Corrupt, "{ctx}");
            if report == MountReport::Recovered {
                recovered += 1
            } else {
                clean += 1
            }
            let (before, after) = (expected.get(j).unwrap(), expected.get(j + 1).unwrap());
            assert_old_or_new(&s, before, after, &ctx);
            let seen: Model = contents(&s);

            // The store carries on, and survives a second cut anywhere in the
            // follow-up.
            let kinds2 = event_kinds(&flash, &FOLLOW_UP);
            let expected2 = models(&seen, &FOLLOW_UP);
            for (k2, &is_erase2) in kinds2.iter().enumerate() {
                for seed2 in 0..(if is_erase2 { FOLLOW_UP_ERASE_SEEDS } else { 1 }) {
                    second_cuts += 1;
                    let ctx2 = format!("{ctx}; second cut at event {k2} seed {seed2}");
                    let mut f2 = flash.clone();
                    let j2 = run_with_cut(&mut f2, &FOLLOW_UP, Some((k2 as u64, seed2)))
                        .unwrap_or_else(|| panic!("no second cut: {ctx2}"));
                    f2.restore_power();
                    let (mut s2, r2) =
                        Store::mount(&mut f2).unwrap_or_else(|e| panic!("mount: {e} ({ctx2})"));
                    assert_ne!(r2, MountReport::Corrupt, "{ctx2}");
                    assert_old_or_new(
                        &s2,
                        expected2.get(j2).unwrap(),
                        expected2.get(j2 + 1).unwrap(),
                        &ctx2,
                    );
                    // Still writable.
                    let seen2 = contents(&s2);
                    let k = seen2.keys().next().copied().unwrap_or(1);
                    s2.set(k, 4242)
                        .unwrap_or_else(|e| panic!("set after second cut: {e} ({ctx2})"));
                    assert_eq!(s2.get(k), Some(4242), "{ctx2}");
                    for (&other, &v) in seen2.iter().filter(|(o, _)| **o != k) {
                        assert_eq!(s2.get(other), Some(v), "{ctx2}");
                    }
                }
            }
            // Finish the follow-up uncut: everything lands.
            let (mut s, _) = Store::mount(&mut flash).unwrap();
            for op in FOLLOW_UP {
                op.run(&mut s)
                    .unwrap_or_else(|e| panic!("follow-up: {e} ({ctx})"));
            }
            assert_matches(&s, expected2.last().unwrap(), &ctx);
        }
    }
    eprintln!(
        "script: {} events ({programmed} programmed bytes + {erases} erases); first cuts {first_cuts} \
         ({ERASE_SEEDS} seeds per erase), second cuts {second_cuts}; mounts clean {clean} recovered {recovered}; {:?}",
        kinds.len(),
        t.elapsed()
    );
    assert!(first_cuts >= programmed as u64 + 2 * ERASE_SEEDS);
    assert!(
        recovered > 0,
        "some cuts must leave something to recover from"
    );
}

#[test]
fn an_uncut_script_ends_in_the_expected_state() {
    let mut f = fresh();
    let (mut s, _) = Store::mount(&mut f).unwrap();
    let mut m = Model::new();
    for op in SCRIPT {
        op.run(&mut s).unwrap();
        op.apply(&mut m);
    }
    assert_eq!(s.erase_count(), 2);
    assert_matches(&s, &m, "end of script");
}

#[test]
fn power_cut_while_initialising_a_blank_flash() {
    let mut f = fresh();
    let _ = Store::mount(&mut f).unwrap();
    let events = f.events();
    assert_eq!(events, 16);
    for k in 0..events {
        let mut f = fresh();
        f.cut_power_after(k, 0);
        assert!(Store::mount(&mut f).is_err());
        f.restore_power();
        let (mut s, r) = Store::mount(&mut f).unwrap();
        // Nothing was ever acknowledged, so either outcome loses nothing.
        assert!(
            matches!(r, MountReport::Fresh | MountReport::Corrupt),
            "cut {k}: {r:?}"
        );
        if r == MountReport::Corrupt {
            s.format().unwrap();
        }
        s.set(1, 1).unwrap();
        assert_eq!(s.get(1), Some(1));
    }
}

#[test]
fn power_cut_during_format_leaves_a_mountable_flash_and_a_second_format_finishes() {
    let mut base = fresh();
    {
        let (mut s, _) = Store::mount(&mut base).unwrap();
        for op in SCRIPT {
            op.run(&mut s).unwrap();
        }
    }
    let span = {
        let mut f = base.clone();
        let start = f.events();
        let (mut s, _) = Store::mount(&mut f).unwrap();
        s.format().unwrap();
        f.events() - start
    };
    assert!(span > 0);
    for k in 0..span {
        for seed in 0..4 {
            let mut f = base.clone();
            {
                let (mut s, _) = Store::mount(&mut f).unwrap();
                s.flash_mut().cut_power_after(k, seed);
                assert!(s.format().is_err());
            }
            f.restore_power();
            let (mut s, _) = Store::mount(&mut f).unwrap();
            s.format().unwrap();
            assert_eq!(contents(&s), Model::new());
            s.set(9, 9).unwrap();
            let (s, r) = Store::mount(&mut f).unwrap();
            assert_eq!(r, MountReport::Clean, "cut {k} seed {seed}");
            assert_eq!(s.get(9), Some(9));
        }
    }
}

// ------------------------------------------------------------ random testing

struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        x
    }

    fn below(&mut self, n: u64) -> u64 {
        self.next() % n
    }
}

fn random_sequence(seed: u64, ops: usize) -> (u64, u64) {
    const RANDOM_SIZE: usize = 16 + 12 * 12;
    let mut flash = SimFlash::<RANDOM_SIZE>::new();
    let mut rng = Rng(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1);
    let mut model = Model::new();
    let (mut cuts, mut compactions_seen) = (0_u64, 0_u64);
    let mut last_erase_count = 0;
    let _ = Store::mount(&mut flash).unwrap();
    for n in 0..ops {
        let key = rng.below(9) as u32; // up to 9 live keys in 12 slots
        let op = if rng.below(4) == 0 {
            Op::Remove(key)
        } else {
            Op::Set(key, rng.next() as u32)
        };
        let cut = rng.below(6) == 0;
        let (mut s, _) = Store::mount(&mut flash).unwrap();
        if cut {
            s.flash_mut().cut_power_after(rng.below(110), rng.next());
        }
        let ctx = format!("seed {seed} op {n} {op:?} cut {cut}");
        match op.run(&mut s) {
            Ok(()) => {
                op.apply(&mut model);
                assert_matches(&s, &model, &ctx);
                if s.erase_count() > last_erase_count {
                    compactions_seen += 1;
                }
                last_erase_count = s.erase_count();
                flash.restore_power();
            }
            Err(Error::Flash(FlashError::PowerLoss)) => {
                cuts += 1;
                flash.restore_power();
                let (s, report) = Store::mount(&mut flash).unwrap();
                assert_ne!(report, MountReport::Corrupt, "{ctx}");
                let mut after = model.clone();
                op.apply(&mut after);
                // The interrupted key is old or new; everything else is as it was.
                let got = s.get(key);
                assert!(
                    got == model.get(&key).copied() || got == after.get(&key).copied(),
                    "{ctx}: {got:?}"
                );
                let listed = contents(&s);
                let mut expect = model.clone();
                match got {
                    Some(v) => expect.insert(key, v),
                    None => expect.remove(&key),
                };
                assert_eq!(listed, expect, "{ctx}");
                assert!(s.erase_count() >= last_erase_count, "{ctx}");
                last_erase_count = s.erase_count();
                model = expect;
            }
            Err(e) => panic!("{ctx}: unexpected {e:?}"),
        }
    }
    let (s, _) = Store::mount(&mut flash).unwrap();
    assert_matches(&s, &model, "end");
    (cuts, compactions_seen)
}

#[test]
fn random_sequences_with_random_cuts_match_a_model() {
    let t = Instant::now();
    let (mut ops, mut cuts, mut comp) = (0, 0, 0);
    for seed in 1..=4 {
        let (c, k) = random_sequence(seed, 1500);
        ops += 1500;
        cuts += c;
        comp += k;
    }
    eprintln!(
        "random: {ops} operations, {cuts} power cuts, {comp} compactions observed uncut; {:?}",
        t.elapsed()
    );
    assert!(cuts > 100 && comp > 50, "cuts {cuts} compactions {comp}");
}

// -------------------------------------------- keys whose CRC aliases an erased field

/// `crc32(key, 0) == 0xFFFF_FFFF`: a half-written remove has an erased CRC field.
const KEY_CRC_ONES: u32 = 0xFFFF_FFFF;
/// `crc32(key, 0) == 0`: a half-written set of value 0 has an erased CRC field.
const KEY_CRC_ZERO: u32 = 0x101B_2F50;

#[test]
fn keys_whose_crc_aliases_an_erased_field_are_safe_at_every_cut() {
    let script = [
        Op::Set(KEY_CRC_ONES, 5),
        Op::Set(KEY_CRC_ZERO, 7),
        Op::Remove(KEY_CRC_ONES),
        Op::Set(KEY_CRC_ZERO, 0),
        Op::Set(KEY_CRC_ONES, 0),
        Op::Remove(KEY_CRC_ZERO),
        Op::Set(KEY_CRC_ZERO, 0),
    ];
    let mut base = fresh();
    let _ = Store::mount(&mut base).unwrap();
    let kinds = event_kinds(&base, &script);
    let expected = models(&Model::new(), &script);
    for (k, _) in kinds.iter().enumerate() {
        let ctx = format!("cut at event {k}");
        let mut flash = base.clone();
        let j = run_with_cut(&mut flash, &script, Some((k as u64, 0))).expect(&ctx);
        flash.restore_power();
        let (s, report) = Store::mount(&mut flash).unwrap();
        assert_ne!(report, MountReport::Corrupt, "{ctx}");
        assert_old_or_new(&s, &expected[j], &expected[j + 1], &ctx);
    }
}

#[test]
fn the_aliasing_keys_really_alias() {
    use crate::layout::record_crc;
    assert_eq!(record_crc(KEY_CRC_ZERO, 0), 0);
    assert_eq!(record_crc(KEY_CRC_ONES, 0), u32::MAX);
}

#[test]
fn mount_picks_the_newer_sector_across_a_sequence_wrap() {
    use crate::layout::{STATE_VALID, header_body, record_bytes};
    let mut f = fresh();
    for (sector, seq, value) in [(0, u32::MAX, 100), (1, 0, 200)] {
        let b = f.bytes_mut(sector).unwrap();
        b[..12].copy_from_slice(&header_body(seq));
        b[12..16].copy_from_slice(&STATE_VALID.to_le_bytes());
        b[16..28].copy_from_slice(&record_bytes(1, value, false));
    }
    let (s, _) = Store::mount(&mut f).unwrap();
    assert_eq!(s.get(1), Some(200));
    assert_eq!(s.erase_count(), 0);
}
