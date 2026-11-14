// SPDX-License-Identifier: AGPL-3.0-or-later
//! Store tests: behaviour, exhaustive power cuts, random sequences with cuts.

use std::collections::BTreeMap;
use std::time::Instant;

use crate::flash::{Flash, FlashError};
use crate::sim::{BitCut, SimFlash};
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
    s.iter().map(|r| r.unwrap()).collect()
}

/// `get` and `iter` agree with each other and with `model`.
fn assert_matches<F: Flash>(s: &Store<F>, model: &Model, ctx: &str) {
    assert_eq!(&contents(s), model, "iter differs: {ctx}");
    for (&k, &v) in model {
        assert_eq!(s.get(k).unwrap(), Some(v), "get({k}): {ctx}");
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
    assert_eq!(s.get(1).unwrap(), None);
    s.set(1, 10).unwrap();
    s.set(2, 20).unwrap();
    s.set(u32::MAX, u32::MAX).unwrap();
    let (s, r) = Store::mount(&mut f).unwrap();
    assert_eq!(r, MountReport::Clean);
    assert_eq!(s.get(1).unwrap(), Some(10));
    assert_eq!(s.get(2).unwrap(), Some(20));
    assert_eq!(s.get(u32::MAX).unwrap(), Some(u32::MAX));
    assert_eq!(s.get(3).unwrap(), None);
}

#[test]
fn the_newest_value_wins() {
    let mut f = fresh();
    let (mut s, _) = Store::mount(&mut f).unwrap();
    s.set(1, 1).unwrap();
    s.set(1, 2).unwrap();
    s.set(1, 3).unwrap();
    assert_eq!(s.get(1).unwrap(), Some(3));
    assert_eq!(contents(&s), Model::from([(1, 3)]));
    let (s, _) = Store::mount(&mut f).unwrap();
    assert_eq!(s.get(1).unwrap(), Some(3));
}

#[test]
fn remove_hides_the_key_and_a_later_set_revives_it() {
    let mut f = fresh();
    let (mut s, _) = Store::mount(&mut f).unwrap();
    s.set(1, 5).unwrap();
    s.set(2, 6).unwrap();
    s.remove(1).unwrap();
    assert_eq!(s.get(1).unwrap(), None);
    assert_eq!(contents(&s), Model::from([(2, 6)]));
    let (mut s, r) = Store::mount(&mut f).unwrap();
    assert_eq!(r, MountReport::Clean);
    assert_eq!(s.get(1).unwrap(), None);
    s.set(1, 0).unwrap();
    assert_eq!(s.get(1).unwrap(), Some(0));
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
    assert_eq!(s.get(7).unwrap(), Some(39));
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
    assert_eq!(s.get(3).unwrap(), Some(33));
    assert_eq!(s.get(7).unwrap(), None);
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
    assert_eq!(s.get(2).unwrap(), None);
    s.set(3, 3).unwrap();
    assert_eq!(contents(&s), Model::from([(1, 1), (3, 3)]));
    // The torn slot is still there and still counted; the new record is after it.
    let (s, r) = Store::mount(&mut f).unwrap();
    assert_eq!((r, s.torn_records()), (MountReport::Recovered, 1));
    assert_eq!(s.get(3).unwrap(), Some(3));
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
    assert_eq!(s.get(1).unwrap(), None);
    assert_eq!(contents(&s), Model::new());
    assert_eq!(s.set(1, 1), Err(Error::Corrupt));
    assert_eq!(s.remove(1), Err(Error::Corrupt));
    s.format().unwrap();
    s.set(1, 42).unwrap();
    assert_eq!(s.get(1).unwrap(), Some(42));
    let before = s.erase_count();
    assert!(before >= 1, "format accounts for the erases it made");
    let (s, r) = Store::mount(&mut f).unwrap();
    assert_eq!(r, MountReport::Clean);
    assert_eq!(s.get(1).unwrap(), Some(42));
    assert_eq!(s.erase_count(), before);
}

/// A store that has compacted once: contents {1: 7, 2: 2}, the active sector is
/// the one with sequence 1, the other holds the previous generation.
fn after_one_compaction() -> (Small, usize) {
    let mut f = fresh();
    let (mut s, _) = Store::mount(&mut f).unwrap();
    for i in 0..6 {
        s.set(1, i).unwrap();
    }
    s.set(2, 2).unwrap(); // compacts
    s.set(1, 7).unwrap();
    assert_eq!(s.erase_count(), 1);
    (f, 1)
}

#[test]
fn a_damaged_active_header_is_corrupt_and_never_a_silent_rollback_to_old_values() {
    let (mut f, active) = after_one_compaction();
    // Clear a bit of the active sector's header body (its sequence number).
    f.bytes_mut(active).unwrap()[4] &= !1;
    let (s, r) = Store::mount(&mut f).unwrap();
    assert_eq!(r, MountReport::Corrupt);
    assert_eq!(s.get(1).unwrap(), None);
    assert_eq!(contents(&s), Model::new());
}

#[test]
fn after_a_compaction_the_old_sector_is_retired_and_a_remount_is_clean() {
    use crate::layout::{Header, STATE_RETIRED, decode_header};
    let (mut f, active) = after_one_compaction();
    let header = |f: &Small, sector: usize| {
        let b = f.bytes(sector).unwrap();
        let w = |o: usize| u32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]]);
        decode_header([w(0), w(4), w(8), w(12)])
    };
    assert_eq!(header(&f, active), Header::Valid(1));
    assert_eq!(header(&f, active ^ 1), Header::Retired(0));
    assert_eq!(
        &f.bytes(active ^ 1).unwrap()[12..16],
        &STATE_RETIRED.to_le_bytes()
    );
    let (s, r) = Store::mount(&mut f).unwrap();
    assert_eq!(r, MountReport::Clean);
    assert_eq!(contents(&s), Model::from([(1, 7), (2, 2)]));
    assert_eq!(s.erase_count(), 1);
}

#[test]
fn both_sectors_valid_mounts_the_higher_sequence_and_reports_recovered() {
    use crate::layout::{STATE_VALID, header_body, record_bytes};
    for (seq0, seq1, want) in [(5, 6, 1), (6, 5, 0), (u32::MAX, 0, 1)] {
        let mut f = fresh();
        for (sector, seq) in [(0, seq0), (1, seq1)] {
            let b = f.bytes_mut(sector).unwrap();
            b[..12].copy_from_slice(&header_body(seq));
            b[12..16].copy_from_slice(&STATE_VALID.to_le_bytes());
            b[16..28].copy_from_slice(&record_bytes(1, 100 + sector as u32, false));
        }
        let (s, r) = Store::mount(&mut f).unwrap();
        assert_eq!(r, MountReport::Recovered, "{seq0} {seq1}");
        assert_eq!(s.get(1).unwrap(), Some(100 + want), "{seq0} {seq1}");
        assert_eq!(f.total_erases(), 0, "mount erases nothing here");
    }
}

#[test]
fn a_valid_sector_beside_a_writing_or_damaged_one_is_recovered_and_beside_a_blank_or_retired_one_clean()
 {
    use crate::layout::{STATE_RETIRED, STATE_VALID, header_body, record_bytes};
    let valid = |f: &mut Small, sector: usize| {
        let b = f.bytes_mut(sector).unwrap();
        b[..12].copy_from_slice(&header_body(3));
        b[12..16].copy_from_slice(&STATE_VALID.to_le_bytes());
        b[16..28].copy_from_slice(&record_bytes(1, 9, false));
    };
    let other = |f: &mut Small, how: &str| {
        let b = f.bytes_mut(1).unwrap();
        match how {
            "blank" => {}
            "retired" => {
                b[..12].copy_from_slice(&header_body(2));
                b[12..16].copy_from_slice(&STATE_RETIRED.to_le_bytes());
                b[16..28].copy_from_slice(&record_bytes(1, 8, false));
            }
            "writing" => {
                b[..12].copy_from_slice(&header_body(4));
                b[16..28].copy_from_slice(&record_bytes(1, 7, false));
            }
            "bad" => b[..16].fill(0x12 & 0xF0),
            _ => unreachable!(),
        }
    };
    for (how, want) in [
        ("blank", MountReport::Clean),
        ("retired", MountReport::Clean),
        ("writing", MountReport::Recovered),
        ("bad", MountReport::Recovered),
    ] {
        let mut f = fresh();
        valid(&mut f, 0);
        other(&mut f, how);
        let (s, r) = Store::mount(&mut f).unwrap();
        assert_eq!(r, want, "{how}");
        assert_eq!(s.get(1).unwrap(), Some(9), "{how}");
        assert_eq!(f.total_erases(), 0, "{how}: mount writes nothing");
        assert_eq!(f.events(), 0, "{how}: mount writes nothing");
    }
}

#[test]
fn no_valid_sector_and_nothing_stored_is_fresh_whatever_the_headers_hold() {
    use crate::layout::{header_body, record_bytes};
    // Header bodies in any state, in either sector, but no record bytes.
    let mut f = fresh();
    f.bytes_mut(0).unwrap()[..12].copy_from_slice(&header_body(5));
    f.bytes_mut(1).unwrap()[..16].fill(0x0F);
    let (mut s, r) = Store::mount(&mut f).unwrap();
    assert_eq!(r, MountReport::Fresh);
    assert_eq!(contents(&s), Model::new());
    s.set(1, 1).unwrap();
    assert_eq!(f.total_erases(), 2, "both dirty sectors were erased");
    let (s, r) = Store::mount(&mut f).unwrap();
    assert_eq!((r, s.get(1).unwrap()), (MountReport::Clean, Some(1)));
    // One byte of record area written: anything else is damage.
    let mut f = fresh();
    f.bytes_mut(0).unwrap()[..12].copy_from_slice(&header_body(5));
    f.bytes_mut(0).unwrap()[16..28].copy_from_slice(&record_bytes(1, 2, false));
    let (_, r) = Store::mount(&mut f).unwrap();
    assert_eq!(r, MountReport::Corrupt);
    assert_eq!(f.events(), 0);
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

/// A flash that fails reads on demand: every read from the `n`-th on (counting
/// from when `fail_from` is set) returns a device error.
struct FlakyReads<F: Flash> {
    inner: F,
    reads: std::cell::Cell<u64>,
    fail_from: std::cell::Cell<Option<u64>>,
}

impl<F: Flash> FlakyReads<F> {
    fn new(inner: F) -> Self {
        Self {
            inner,
            reads: std::cell::Cell::new(0),
            fail_from: std::cell::Cell::new(None),
        }
    }

    fn fail_after(&self, n: u64) {
        self.fail_from.set(Some(self.reads.get() + n));
    }

    fn heal(&self) {
        self.fail_from.set(None);
    }
}

impl<F: Flash> Flash for FlakyReads<F> {
    const SECTOR_SIZE: usize = F::SECTOR_SIZE;
    fn read(&self, s: usize, o: usize, b: &mut [u8]) -> Result<(), FlashError> {
        let n = self.reads.get();
        self.reads.set(n + 1);
        match self.fail_from.get() {
            Some(from) if n >= from => Err(FlashError::Device),
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
fn a_flash_read_error_is_never_reported_as_an_absent_key() {
    let mut f = FlakyReads::new(fresh());
    let (mut s, _) = Store::mount(&mut f).unwrap();
    s.set(1, 10).unwrap();
    s.set(2, 20).unwrap();
    s.set(1, 11).unwrap();
    assert_eq!(s.get(1), Ok(Some(11)));
    assert_eq!(s.get(9), Ok(None));
    // From the first read on, every read fails.
    s.flash_mut().fail_after(0);
    let dev = Err(Error::Flash(FlashError::Device));
    assert_eq!(s.get(1), dev);
    assert_eq!(
        s.get(9),
        dev,
        "an absent key is not told apart from a failure"
    );
    // A failure part-way through the search is a failure too.
    s.flash_mut().heal();
    // Looking up key 2 makes 5 reads: the newest key, the older key, then the
    // three words of the record.
    for n in 0..5 {
        s.flash_mut().fail_after(n);
        assert_eq!(s.get(2), dev, "fails at read {n}");
        s.flash_mut().heal();
        assert_eq!(s.get(2), Ok(Some(20)));
    }
}

#[test]
fn iter_yields_the_read_error_and_then_ends() {
    let mut f = FlakyReads::new(fresh());
    let (mut s, _) = Store::mount(&mut f).unwrap();
    s.set(1, 10).unwrap();
    s.set(2, 20).unwrap();
    s.set(3, 30).unwrap();
    let all: Vec<_> = s.iter().collect();
    assert_eq!(all, vec![Ok((1, 10)), Ok((2, 20)), Ok((3, 30))]);
    for n in 0..40 {
        s.flash_mut().fail_after(n);
        let got: Vec<_> = s.iter().collect();
        s.flash_mut().heal();
        // Some prefix of the good items, then at most one error, then nothing.
        let errors = got.iter().filter(|r| r.is_err()).count();
        assert!(errors <= 1, "read {n}: {got:?}");
        if errors == 1 {
            assert_eq!(
                got.last(),
                Some(&Err(Error::Flash(FlashError::Device))),
                "{got:?}"
            );
            assert!(got.len() <= 4, "{got:?}");
            let items = got.len() - 1;
            assert_eq!(&got[..items], &all[..items], "read {n}: {got:?}");
        } else {
            assert_eq!(got, all, "read {n}");
        }
        // Once an iterator has yielded an error it stays finished.
        s.flash_mut().fail_after(n);
        {
            let mut it = s.iter();
            let mut saw_error = false;
            for _ in 0..10 {
                match it.next() {
                    Some(Err(_)) => saw_error = true,
                    Some(Ok(_)) => assert!(!saw_error, "an item after an error"),
                    None => break,
                }
            }
            if saw_error {
                // The reads still fail, so a live iterator would yield another
                // error.
                assert_eq!(it.next(), None, "ends after the error");
            }
        }
        s.flash_mut().heal();
    }
    // At least some read position must have produced an error mid-iteration.
    s.flash_mut().fail_after(3);
    assert!(s.iter().any(|r| r.is_err()));
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
    assert_eq!(s.get(1000).unwrap(), Some(7));
    assert_eq!(s.get(1363).unwrap(), Some(!1363));
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

/// One way to cut the power during a run.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Cut {
    /// After this many flash events (programmed bytes and erases) from the
    /// start of the run; the seed shapes an interrupted erase.
    Byte(u64, u64),
    /// Inside the program call with this number (from the start of the run),
    /// leaving this subset of the bits the call clears.
    Bits(u64, BitCut),
}

impl Cut {
    fn arm<const N: usize>(self, f: &mut SimFlash<N>) {
        match self {
            Self::Byte(k, seed) => f.cut_power_after(k, seed),
            Self::Bits(call, how) => f.cut_program_call(call, how),
        }
    }

    fn is_bits(self) -> bool {
        matches!(self, Self::Bits(..))
    }
}

/// What a run does to the flash: whether each event is an erase, and how many
/// bits each program call clears.
struct Plan {
    is_erase: Vec<bool>,
    call_bits: Vec<usize>,
}

impl Plan {
    fn programmed_bytes(&self) -> usize {
        self.is_erase.iter().filter(|e| !**e).count()
    }

    fn erases(&self) -> usize {
        self.is_erase.len() - self.programmed_bytes()
    }

    /// Byte-prefix cuts: at every event (an erase with `erase_seeds` seeds).
    fn byte_cuts(&self, erase_seeds: u64) -> Vec<Cut> {
        let mut v = Vec::new();
        for (k, &erase) in self.is_erase.iter().enumerate() {
            for seed in 0..(if erase { erase_seeds } else { 1 }) {
                v.push(Cut::Byte(k as u64, seed));
            }
        }
        v
    }

    /// Cuts inside every program call: `seeds` seeded subsets, all but bit
    /// `k` and only bit `k`, for every bit `k` the call clears.
    fn bit_cuts(&self, seeds: u64) -> Vec<Cut> {
        let mut v = Vec::new();
        for (call, &bits) in self.call_bits.iter().enumerate() {
            let call = call as u64;
            v.extend((0..seeds).map(|s| Cut::Bits(call, BitCut::Seeded(s))));
            v.extend((0..bits).map(|k| Cut::Bits(call, BitCut::AllBut(k))));
            v.extend((0..bits).map(|k| Cut::Bits(call, BitCut::Only(k))));
        }
        v
    }

    /// Only the seeded subsets of every program call.
    fn seeded_cuts(&self, seeds: u64) -> Vec<Cut> {
        let mut v = Vec::new();
        for call in 0..self.call_bits.len() as u64 {
            v.extend((0..seeds).map(|s| Cut::Bits(call, BitCut::Seeded(s))));
        }
        v
    }
}

/// Records what each flash event and program call was.
struct Recorder<'a> {
    flash: &'a mut Small,
    plan: Plan,
}

impl Flash for Recorder<'_> {
    const SECTOR_SIZE: usize = SMALL;
    fn read(&self, s: usize, o: usize, b: &mut [u8]) -> Result<(), FlashError> {
        self.flash.read(s, o, b)
    }
    fn program(&mut self, s: usize, o: usize, d: &[u8]) -> Result<(), FlashError> {
        let bits = self.flash.bits_cleared_by(s, o, d).unwrap_or(0);
        self.flash.program(s, o, d)?;
        self.plan.call_bits.push(bits);
        self.plan.is_erase.extend(d.iter().map(|_| false));
        Ok(())
    }
    fn erase(&mut self, s: usize) -> Result<(), FlashError> {
        self.flash.erase(s)?;
        self.plan.is_erase.push(true);
        Ok(())
    }
}

/// The plan of what `run` does to a copy of `flash` after mounting it.
fn plan_of(flash: &Small, run: impl FnOnce(&mut Store<&mut Recorder<'_>>)) -> Plan {
    let mut copy = flash.clone();
    let mut rec = Recorder {
        flash: &mut copy,
        plan: Plan {
            is_erase: Vec::new(),
            call_bits: Vec::new(),
        },
    };
    let (mut s, _) = Store::mount(&mut rec).unwrap();
    run(&mut s);
    rec.plan
}

/// The plan of `ops` run on a copy of `flash`.
fn plan_ops(flash: &Small, ops: &[Op]) -> Plan {
    plan_of(flash, |s| {
        for op in ops {
            op.run(s).unwrap();
        }
    })
}

/// Runs `ops` against a store on `flash` with the power cut as given, counted
/// from the start of the run. Returns the index of the interrupted operation,
/// if any.
fn run_with_cut(flash: &mut Small, ops: &[Op], cut: Option<Cut>) -> Option<usize> {
    let (mut s, _) = Store::mount(&mut *flash).unwrap();
    if let Some(cut) = cut {
        cut.arm(s.flash_mut());
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
        let got = s.get(k).unwrap();
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
            s.get(k).unwrap(),
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
/// Seeded subsets per program call, first cut.
const CALL_SEEDS: u64 = 8;
/// Seeded subsets per program call, second cut.
const FOLLOW_UP_CALL_SEEDS: u64 = 2;
/// Second cuts after a first cut inside a call: every this-many-th of them,
/// shifted by the first cut's number, so that the positions of all of them are
/// covered across the first cuts without multiplying the run time.
const SECOND_STRIDE: usize = 4;

#[test]
fn power_cut_at_every_point_of_every_operation() {
    let t = Instant::now();
    // Dry run of the script on a formatted flash.
    let mut base = fresh();
    let _ = Store::mount(&mut base).unwrap();
    let plan = plan_ops(&base, &SCRIPT);
    let (programmed, erases) = (plan.programmed_bytes(), plan.erases());
    assert_eq!(erases, 2, "script must force exactly two compactions");
    let expected = models(&Model::new(), &SCRIPT);

    let byte_cuts = plan.byte_cuts(ERASE_SEEDS);
    let bit_cuts = plan.bit_cuts(CALL_SEEDS);
    let first: Vec<Cut> = byte_cuts.iter().chain(&bit_cuts).copied().collect();
    let (mut second_byte, mut second_bits) = (0_u64, 0_u64);
    let (mut recovered, mut clean) = (0_u64, 0_u64);
    for (n, &cut) in first.iter().enumerate() {
        let ctx = format!("first cut {cut:?}");
        let mut flash = base.clone();
        let j = run_with_cut(&mut flash, &SCRIPT, Some(cut))
            .unwrap_or_else(|| panic!("no cut happened: {ctx}"));
        assert!(flash.is_dead(), "{ctx}");
        flash.restore_power();

        let (s, report) = Store::mount(&mut flash).unwrap_or_else(|e| panic!("mount: {e} ({ctx})"));
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
        let plan2 = plan_ops(&flash, &FOLLOW_UP);
        let expected2 = models(&seen, &FOLLOW_UP);
        let mut second: Vec<Cut> = plan2.byte_cuts(FOLLOW_UP_ERASE_SEEDS);
        second.extend(plan2.seeded_cuts(FOLLOW_UP_CALL_SEEDS));
        for (i, &cut2) in second.iter().enumerate() {
            if cut.is_bits() && (i + n) % SECOND_STRIDE != 0 {
                continue;
            }
            if cut2.is_bits() {
                second_bits += 1;
            } else {
                second_byte += 1;
            }
            let ctx2 = format!("{ctx}; second cut {cut2:?}");
            let mut f2 = flash.clone();
            let j2 = run_with_cut(&mut f2, &FOLLOW_UP, Some(cut2))
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
            assert_eq!(s2.get(k).unwrap(), Some(4242), "{ctx2}");
            for (&other, &v) in seen2.iter().filter(|(o, _)| **o != k) {
                assert_eq!(s2.get(other).unwrap(), Some(v), "{ctx2}");
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
    eprintln!(
        "script: {} events ({programmed} programmed bytes + {erases} erases) in {} program calls ({} bits); \
         first cuts {} ({} byte-prefix incl. {ERASE_SEEDS} seeds per erase, {} inside a call: {CALL_SEEDS} seeded + all-but-bit + only-bit); \
         second cuts {} byte-prefix + {} inside a call; mounts clean {clean} recovered {recovered}; {:?}",
        plan.is_erase.len(),
        plan.call_bits.len(),
        plan.call_bits.iter().sum::<usize>(),
        first.len(),
        byte_cuts.len(),
        bit_cuts.len(),
        second_byte,
        second_bits,
        t.elapsed()
    );
    assert!(byte_cuts.len() >= programmed + 2 * ERASE_SEEDS as usize);
    assert!(
        bit_cuts.len()
            >= plan.call_bits.len() * (CALL_SEEDS as usize)
                + 2 * plan.call_bits.iter().sum::<usize>()
    );
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
    let calls = f.program_calls();
    assert_eq!(calls, 2);
    let check = |mut f: Small, ctx: &str| {
        f.restore_power();
        let (mut s, r) = Store::mount(&mut f).unwrap();
        // Nothing was ever stored, so starting empty loses nothing; and a
        // flash that holds only a cut-short header is never "corrupt".
        assert!(
            matches!(r, MountReport::Fresh | MountReport::Clean),
            "{ctx}: {r:?}"
        );
        assert_eq!(contents(&s), Model::new(), "{ctx}");
        s.set(1, 1).unwrap();
        assert_eq!(s.get(1).unwrap(), Some(1), "{ctx}");
    };
    for k in 0..events {
        let mut f = fresh();
        f.cut_power_after(k, 0);
        assert!(Store::mount(&mut f).is_err());
        check(f, &format!("byte cut {k}"));
    }
    for call in 0..calls {
        // Bits of this call: the header body (call 0), then the state word.
        let mut probe = fresh();
        let body = crate::layout::header_body(0);
        let bits = if call == 0 {
            probe.bits_cleared_by(0, 0, &body).unwrap()
        } else {
            probe.program(0, 0, &body).unwrap();
            let state = crate::layout::STATE_VALID.to_le_bytes();
            probe.bits_cleared_by(0, 12, &state).unwrap()
        };
        let mut cuts = vec![];
        cuts.extend((0..8).map(BitCut::Seeded));
        cuts.extend((0..bits).map(BitCut::AllBut));
        cuts.extend((0..bits).map(BitCut::Only));
        for how in cuts {
            let mut f = fresh();
            f.cut_program_call(call, how);
            assert!(Store::mount(&mut f).is_err());
            check(f, &format!("call {call} {how:?}"));
        }
    }
}

/// After a failed mount of a fresh flash, a second mount that is itself cut
/// anywhere still ends in a mountable store.
#[test]
fn a_cut_during_the_recovery_of_an_interrupted_initialisation_is_recovered_too() {
    for k in 0..16 {
        let mut f = fresh();
        f.cut_power_after(k, 0);
        assert!(Store::mount(&mut f).is_err());
        f.restore_power();
        // The recovery of a header left half written erases sector 0.
        let mut g = f.clone();
        let _ = Store::mount(&mut g).unwrap();
        for k2 in 0..(g.events() - f.events()) {
            for seed in 0..4 {
                let mut h = f.clone();
                h.cut_power_after(k2, seed);
                let _ = Store::mount(&mut h);
                h.restore_power();
                let (_, r) = Store::mount(&mut h).unwrap();
                assert!(
                    matches!(r, MountReport::Fresh | MountReport::Clean),
                    "cut {k} then {k2} seed {seed}: {r:?}"
                );
            }
        }
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
    let plan = plan_of(&base, |s| s.format().unwrap());
    assert!(plan.erases() >= 1 && plan.programmed_bytes() > 0);
    let mut cuts = plan.byte_cuts(4);
    cuts.extend(plan.bit_cuts(CALL_SEEDS));
    for cut in &cuts {
        let mut f = base.clone();
        {
            let (mut s, _) = Store::mount(&mut f).unwrap();
            cut.arm(s.flash_mut());
            assert!(s.format().is_err(), "{cut:?}");
        }
        f.restore_power();
        // Whatever is left is mountable (nothing in it may be trusted to be
        // the old content), and a second format finishes the job.
        let (mut s, r) = Store::mount(&mut f).unwrap();
        assert!(
            matches!(
                r,
                MountReport::Fresh
                    | MountReport::Clean
                    | MountReport::Corrupt
                    | MountReport::Recovered
            ),
            "{cut:?}: {r:?}"
        );
        // An interrupted format of a populated store leaves an empty store,
        // the old one (if the cut came before the erase of its sector), or
        // wreckage that mount calls corrupt: it never invents values.
        let listed = contents(&s);
        assert!(
            listed.is_empty() || listed == models(&Model::new(), &SCRIPT).last().cloned().unwrap(),
            "{cut:?}: {listed:?}"
        );
        s.format().unwrap();
        assert_eq!(contents(&s), Model::new());
        s.set(9, 9).unwrap();
        let (s, r) = Store::mount(&mut f).unwrap();
        assert_eq!(r, MountReport::Clean, "{cut:?}");
        assert_eq!(s.get(9).unwrap(), Some(9));
    }
    eprintln!("format: {} cut points", cuts.len());
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

fn random_sequence(seed: u64, ops: usize) -> (u64, u64, u64) {
    const RANDOM_SIZE: usize = 16 + 12 * 12;
    let mut flash = SimFlash::<RANDOM_SIZE>::new();
    let mut rng = Rng(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1);
    let mut model = Model::new();
    let (mut byte_cuts, mut bit_cuts, mut compactions_seen) = (0_u64, 0_u64, 0_u64);
    let mut last_erase_count = 0;
    let _ = Store::mount(&mut flash).unwrap();
    for n in 0..ops {
        let key = rng.below(9) as u32; // up to 9 live keys in 12 slots
        let op = if rng.below(4) == 0 {
            Op::Remove(key)
        } else {
            Op::Set(key, rng.next() as u32)
        };
        let cut = (rng.below(6) == 0).then(|| {
            if rng.below(2) == 0 {
                Cut::Byte(rng.below(110), rng.next())
            } else {
                // Mostly an early call (an append is two), sometimes deep into a
                // compaction; any subset of its bits.
                let call = if rng.below(4) == 0 {
                    rng.below(40)
                } else {
                    rng.below(4)
                };
                let how = match rng.below(3) {
                    0 => BitCut::Seeded(rng.next()),
                    1 => BitCut::AllBut(rng.below(70) as usize),
                    _ => BitCut::Only(rng.below(70) as usize),
                };
                Cut::Bits(call, how)
            }
        });
        let (mut s, _) = Store::mount(&mut flash).unwrap();
        if let Some(c) = cut {
            c.arm(s.flash_mut());
        }
        let ctx = format!("seed {seed} op {n} {op:?} cut {cut:?}");
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
                if cut.is_some_and(Cut::is_bits) {
                    bit_cuts += 1;
                } else {
                    byte_cuts += 1;
                }
                flash.restore_power();
                let (s, report) = Store::mount(&mut flash).unwrap();
                assert_ne!(report, MountReport::Corrupt, "{ctx}");
                let mut after = model.clone();
                op.apply(&mut after);
                // The interrupted key is old or new; everything else is as it was.
                let got = s.get(key).unwrap();
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
    (byte_cuts, bit_cuts, compactions_seen)
}

#[test]
fn random_sequences_with_random_cuts_match_a_model() {
    let t = Instant::now();
    let (mut ops, mut byte_cuts, mut bit_cuts, mut comp) = (0, 0, 0, 0);
    for seed in 1..=8 {
        let (b, c, k) = random_sequence(seed, 1500);
        ops += 1500;
        byte_cuts += b;
        bit_cuts += c;
        comp += k;
    }
    eprintln!(
        "random: {ops} operations, {byte_cuts} byte-prefix and {bit_cuts} inside-a-call power cuts, \
         {comp} compactions observed uncut; {:?}",
        t.elapsed()
    );
    assert!(
        byte_cuts > 100 && bit_cuts > 100 && comp > 50,
        "{byte_cuts} {bit_cuts} {comp}"
    );
}

// -------------------------------------------- keys whose CRC aliases an erased field

/// `crc32(key, 0) == 0xFFFF_FFFF`: before the one-hot kind, a half-written
/// remove of this key had an erased CRC field. Kept: the CRC still aliases.
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
    let plan = plan_ops(&base, &script);
    let expected = models(&Model::new(), &script);
    let mut cuts = plan.byte_cuts(1);
    cuts.extend(plan.bit_cuts(CALL_SEEDS));
    for cut in &cuts {
        let ctx = format!("{cut:?}");
        let mut flash = base.clone();
        let j = run_with_cut(&mut flash, &script, Some(*cut)).expect(&ctx);
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
    // With the one-hot kind in the top two bits the check words of these keys
    // are still not an erased word, and a set and a tombstone differ in both
    // kind bits.
    use crate::layout::record_check;
    assert_eq!(record_check(KEY_CRC_ONES, 0, false), 0x7FFF_FFFF);
    assert_eq!(record_check(KEY_CRC_ONES, 0, true), 0xBFFF_FFFF);
    assert_eq!(record_check(KEY_CRC_ZERO, 0, false), 0x4000_0000);
    assert_eq!(record_check(KEY_CRC_ZERO, 0, true), 0x8000_0000);
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
    assert_eq!(s.get(1).unwrap(), Some(200));
    assert_eq!(s.erase_count(), 0);
}
