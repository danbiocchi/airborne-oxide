// SPDX-License-Identifier: AGPL-3.0-or-later
//! The store: an append-only log of records in one sector at a time, compacted
//! into the other sector when full.
//!
//! # Sectors and what a power cut can do
//!
//! Exactly one sector is live: its header is `Valid` (see [`crate::layout`]
//! for the header, the record format and why an interrupted write never reads
//! as a record; [`crate::flash::Flash`] for the contract the flash must keep).
//! New records are appended to it. When it is full, or for
//! [`Store::replace_all`], a fresh sector is built in the other one:
//!
//! 1. erase it;
//! 2. program the header body (magic, version, sequence + 1, CRC);
//! 3. program the records, each in two calls;
//! 4. program its state word to *valid*: from this call on the change exists
//!    (a cut inside the call leaves a state word that is not `A55A3CC3`, which
//!    reads as retired, so the old sector is still the live one);
//! 5. program the old sector's state word to `0` (*retired*).
//!
//! Step 5 matters: without it a stale sector keeps a valid header, and if the
//! newer sector's header were ever damaged the store would silently roll back
//! to old values. Now a damaged header of the live sector leaves no valid
//! sector, and the store says [`MountReport::Corrupt`].
//!
//! # Mount
//!
//! | Sectors | Meaning | Active | Report |
//! |---|---|---|---|
//! | `Valid` + `Blank` or `Retired` | normal | the valid one | `Clean` (`Recovered` if it holds torn records) |
//! | `Valid` + `Valid` | cut between steps 4 and 5 | higher sequence (wrapping compare); the older sector is then retired (step 5, one 4-byte program call) | `Recovered` |
//! | `Valid` + `Valid`, equal sequences | not a power cut: damage | none | `Corrupt` |
//! | `Valid` + `Writing` or `Bad` | cut during an erase or the copy | the valid one | `Recovered` |
//! | no `Valid`, and no byte outside the two 16-byte headers differs from `0xFF` | a first initialisation or a format was cut; nothing was ever stored | erase what is not blank, write a header on sector 0 | `Fresh` |
//! | anything else | damage | none | `Corrupt` |
//!
//! Mount writes to flash in two rows only: `Fresh`, and `Valid` + `Valid`,
//! where it programs the older sector's state word to retired (as step 5 would
//! have), so that a later damage to the live header is `Corrupt` and not a
//! return to old values. A failure of that one call is not a mount failure
//! (the store is usable; the next mount tries again), and a power cut inside
//! it leaves a state word that is not valid, so the sector reads as retired. A valid sector beside a
//! `Retired` one is reported `Clean` even when the retired sector is really a
//! half-written new one whose state word was cut short (the two look alike and
//! are treated alike: not used, erased by the next compaction).
//!
//! # Cost
//!
//! With `S` = `SECTOR_SIZE` the sector holds `n = (S - 16) / 12` records
//! (1364 for 16 KB). `get` reads at most `n` keys and verifies the CRC of
//! matching ones. Finding which records are live (`iter`, compaction) asks, for
//! each record, whether a later one has the same key: at most `n(n-1)/2` four
//! byte key reads (about 0.93 million for 16 KB; a CRC is computed only for a
//! later record with the same key). A compaction does that once to count and
//! once to copy, plus one sector erase and at most `n` records, each now two
//! program calls (8 and 4 bytes, the same bytes as before), plus the state
//! word and the retiring of the old sector (two more 4-byte calls). No
//! allocation; the stack use is a few dozen bytes.
//!
//! [`Store::replace_all`] with `w` wanted pairs and `u` records in use costs
//! up to `w` lookups (each at most `u` key reads) to find a first and a second
//! difference, and, if there is no second, one pass over the live records
//! (`u(u-1)/2` key reads, as `iter`) with `w` comparisons per live key. It
//! stops at the second difference. A rewrite then does one erase and `w`
//! records, with no scan of the old sector at all: it is cheaper than a
//! compaction, which has to find the live records.
//!
//! Every `set` of a full sector compacts, so a sector full of distinct live
//! keys erases on every write; the caller keeps the live key count well below
//! `n` (a parameter table has about 100). Writing a changed table with
//! `replace_all` costs at most one erase however many keys changed.

use crate::flash::{Flash, FlashError};
use crate::layout::{
    CHECK_OFFSET, HEADER_SIZE, Header, RECORD_SIZE, RECORDS_OFFSET, STATE_OFFSET, STATE_RETIRED,
    STATE_VALID, Slot, decode_header, decode_slot, header_body, pack, record_parts,
};

/// Why a store operation failed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// The flash failed. After this the store must be mounted again.
    Flash(FlashError),
    /// The store was mounted [`MountReport::Corrupt`] and not yet formatted.
    Corrupt,
    /// More live keys than one sector holds.
    Full,
    /// An earlier flash error left the in-memory state unreliable; mount again.
    NeedsRemount,
}

impl From<FlashError> for Error {
    fn from(e: FlashError) -> Self {
        Self::Flash(e)
    }
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Flash(e) => write!(f, "flash error: {e:?}"),
            Self::Corrupt => f.write_str("store is corrupt: format it first"),
            Self::Full => f.write_str("more live keys than one sector holds"),
            Self::NeedsRemount => f.write_str("an earlier flash error: mount again"),
        }
    }
}

#[cfg(feature = "std")]
impl std::error::Error for Error {}

/// What [`Store::replace_all`] did.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Replaced {
    /// The store already held exactly the wanted pairs; nothing was written.
    Unchanged,
    /// One record (a set or a tombstone) was appended.
    Appended,
    /// A fresh sector holding exactly the wanted pairs was written (one erase).
    Rewritten,
}

/// What [`Store::mount`] found (the table is in the module docs).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MountReport {
    /// A valid sector, beside a blank or retired one, and no torn records.
    Clean,
    /// No valid sector and nothing but (perhaps cut-short) headers on the
    /// flash: it was blank, or a first initialisation or a `format` was
    /// interrupted. Nothing was ever stored, so nothing is lost. The store was
    /// initialised empty.
    Fresh,
    /// A valid sector, but torn records, a second valid sector (a compaction
    /// was cut before it retired the old one; the newer was taken), or a
    /// half-written or damaged sector were found. No acknowledged write was
    /// lost.
    Recovered,
    /// No valid sector, and something other than headers is on the flash: the
    /// store holds nothing it can trust and refuses writes until
    /// [`Store::format`]. In particular a damaged header of the live sector
    /// lands here, however intact the retired sector beside it is.
    Corrupt,
}

/// The store.
pub struct Store<F: Flash> {
    flash: F,
    /// Sector holding the live log, or `None` when corrupt.
    active: Option<usize>,
    /// Sequence number of the active sector.
    sequence: u32,
    /// Slots from the start up to the last non-free one.
    used: usize,
    torn: usize,
    faulted: bool,
}

impl<F: Flash> Store<F> {
    /// Record slots per sector.
    pub const SLOTS: usize = {
        assert!(F::SECTOR_SIZE >= HEADER_SIZE + RECORD_SIZE);
        (F::SECTOR_SIZE - HEADER_SIZE) / RECORD_SIZE
    };

    /// Opens the store. Writes only to initialise a flash that holds nothing
    /// but headers (`MountReport::Fresh`) and to retire the older of two valid
    /// sectors. See [`MountReport`] and the table in
    /// the module docs.
    ///
    /// # Errors
    /// A flash read or program failure.
    pub fn mount(flash: F) -> Result<(Self, MountReport), Error> {
        let mut s = Self::unmounted(flash);
        let report = s.remount()?;
        Ok((s, report))
    }

    /// A store over `flash` that has not been mounted: it holds nothing and
    /// refuses writes ([`Error::Corrupt`]) until [`Store::remount`] succeeds or
    /// [`Store::format`] is called. Unlike [`Store::mount`] it cannot fail, so
    /// the flash is never lost to a failed mount.
    #[must_use]
    pub fn unmounted(flash: F) -> Self {
        Self {
            flash,
            active: None,
            sequence: 0,
            used: 0,
            torn: 0,
            faulted: false,
        }
    }

    /// Mounts again, from what the flash holds now: the same as
    /// [`Store::mount`] on a store whose in-memory state is rebuilt from
    /// scratch. The way back after a flash error (`Error::NeedsRemount`). On an
    /// error the store is left unmounted (as after [`Store::unmounted`]).
    ///
    /// # Errors
    /// A flash read or program failure.
    pub fn remount(&mut self) -> Result<MountReport, Error> {
        self.active = None;
        self.sequence = 0;
        self.used = 0;
        self.torn = 0;
        self.faulted = false;
        let r = self.mount_inner();
        if r.is_err() {
            self.active = None;
        }
        r.map_err(Error::from)
    }

    fn mount_inner(&mut self) -> Result<MountReport, FlashError> {
        let h = [self.read_header(0)?, self.read_header(1)?];
        let best = match (h[0], h[1]) {
            // Equal sequences cannot come from a power cut (each compaction
            // adds one): it is damage or tampering, and which sector is right
            // is unknowable.
            (Header::Valid(a), Header::Valid(b)) if a == b => return Ok(MountReport::Corrupt),
            (Header::Valid(a), Header::Valid(b)) => Some(usize::from(b.wrapping_sub(a) as i32 > 0)),
            (Header::Valid(_), _) => Some(0),
            (_, Header::Valid(_)) => Some(1),
            _ => None,
        };
        if let Some(a) = best {
            let seq = match h.get(a) {
                Some(Header::Valid(q)) => *q,
                _ => 0,
            };
            self.active = Some(a);
            self.sequence = seq;
            self.scan_active()?;
            // Both valid: a compaction was cut between marking the new sector
            // valid and retiring the old one. A sector being written or
            // damaged: a compaction or an erase was cut.
            let other_ignored = matches!(
                h.get(a ^ 1),
                Some(Header::Valid(_) | Header::Writing(_) | Header::Bad)
            );
            if matches!(h.get(a ^ 1), Some(Header::Valid(_))) {
                // Retire the stale sector now (the call a compaction ends
                // with), so that damage to the live header later is `Corrupt`
                // and not a silent return to the old values. If this fails
                // the store is still usable, and the next mount tries again.
                let _ = self
                    .flash
                    .program(a ^ 1, STATE_OFFSET, &pack::<4>(&[STATE_RETIRED]));
            }
            let report = if self.torn > 0 || other_ignored {
                MountReport::Recovered
            } else {
                MountReport::Clean
            };
            return Ok(report);
        }
        // No valid sector. If nothing but (perhaps damaged) headers was ever
        // written, no value was ever stored: a first initialisation or a
        // format was interrupted, and starting empty loses nothing.
        if self.is_blank_from(0, HEADER_SIZE)? && self.is_blank_from(1, HEADER_SIZE)? {
            for sector in 0..2 {
                if !self.is_blank_from(sector, 0)? {
                    self.flash.erase(sector)?;
                }
            }
            self.write_header(0, 0)?;
            self.active = Some(0);
            return Ok(MountReport::Fresh);
        }
        Ok(MountReport::Corrupt)
    }

    /// Read access to the flash.
    #[must_use]
    pub fn flash(&self) -> &F {
        &self.flash
    }

    /// Access to the flash, for tests that arm a power cut.
    pub fn flash_mut(&mut self) -> &mut F {
        &mut self.flash
    }

    /// Gives the flash back.
    pub fn into_flash(self) -> F {
        self.flash
    }

    /// Torn records found in the active sector at mount (they are skipped).
    #[must_use]
    pub fn torn_records(&self) -> usize {
        self.torn
    }

    /// The active sector's sequence number, which is also the count of sector
    /// erases the store has made (and `format` accounts for) in its life.
    #[must_use]
    pub fn erase_count(&self) -> u32 {
        self.sequence
    }

    /// The newest value for `key`: `Ok(None)` when the key is absent (or was
    /// removed, or the store is [`MountReport::Corrupt`] and so empty).
    ///
    /// # Errors
    /// A flash read failure. It is never reported as "absent".
    pub fn get(&self, key: u32) -> Result<Option<u32>, Error> {
        self.lookup(key).map_err(Error::from)
    }

    /// Sets `key` to `value`, compacting first when the sector is full.
    ///
    /// # Errors
    /// [`Error::Full`] when more live keys than a sector holds; flash errors.
    pub fn set(&mut self, key: u32, value: u32) -> Result<(), Error> {
        self.ready()?;
        let r = self.write(key, Some(value));
        self.note(r)
    }

    /// Removes `key` (a tombstone record). Removing an absent key writes nothing.
    ///
    /// # Errors
    /// Flash errors.
    pub fn remove(&mut self, key: u32) -> Result<(), Error> {
        self.ready()?;
        let r = match self.lookup(key) {
            Ok(None) => Ok(()),
            Ok(Some(_)) => self.write(key, None),
            Err(e) => Err(e.into()),
        };
        self.note(r)
    }

    /// The live keys and values, in the order their newest record was written.
    /// A flash read failure is yielded as an `Err` item and ends the iteration.
    #[must_use]
    pub fn iter(&self) -> Iter<'_, F> {
        Iter {
            store: self,
            next: 0,
            done: false,
        }
    }

    /// Erases whatever is on the flash and starts an empty store. Destructive;
    /// the way out of [`MountReport::Corrupt`]. Unlike `set`, an interruption
    /// is not all-or-nothing: mount again (it reports `Fresh`, `Clean` or
    /// `Corrupt`) and format again.
    ///
    /// # Errors
    /// Flash errors.
    pub fn format(&mut self) -> Result<(), Error> {
        let r = self.format_inner();
        if r.is_err() {
            self.faulted = true;
        }
        r.map_err(Error::from)
    }

    fn format_inner(&mut self) -> Result<(), FlashError> {
        let mut highest: Option<u32> = None;
        for sector in 0..2 {
            if let Header::Valid(q) | Header::Writing(q) | Header::Retired(q) =
                self.read_header(sector)?
            {
                highest =
                    Some(highest.map_or(q, |h| if q.wrapping_sub(h) as i32 > 0 { q } else { h }));
            }
        }
        // Erase the older sector first, so an interruption leaves the newer.
        let first = self.active.map_or(0, |a| a ^ 1);
        let mut erased = 0_u32;
        for sector in [first, first ^ 1] {
            if !self.is_blank(sector)? {
                self.flash.erase(sector)?;
                erased = erased.wrapping_add(1);
            }
        }
        let seq = highest.unwrap_or(0).wrapping_add(erased);
        self.write_header(0, seq)?;
        self.active = Some(0);
        self.sequence = seq;
        self.used = 0;
        self.torn = 0;
        self.faulted = false;
        Ok(())
    }

    fn ready(&self) -> Result<(), Error> {
        if self.faulted {
            Err(Error::NeedsRemount)
        } else if self.active.is_none() {
            Err(Error::Corrupt)
        } else {
            Ok(())
        }
    }

    /// Marks the store unreliable after a flash error.
    fn note<T>(&mut self, r: Result<T, Error>) -> Result<T, Error> {
        if matches!(r, Err(Error::Flash(_))) {
            self.faulted = true;
        }
        r
    }

    fn sector(&self) -> Result<usize, FlashError> {
        self.active.ok_or(FlashError::Device)
    }

    fn read_u32(&self, sector: usize, offset: usize) -> Result<u32, FlashError> {
        let mut b = [0_u8; 4];
        self.flash.read(sector, offset, &mut b)?;
        Ok(u32::from_le_bytes(b))
    }

    fn read_header(&self, sector: usize) -> Result<Header, FlashError> {
        Ok(decode_header([
            self.read_u32(sector, 0)?,
            self.read_u32(sector, 4)?,
            self.read_u32(sector, 8)?,
            self.read_u32(sector, STATE_OFFSET)?,
        ]))
    }

    /// Writes a finished header: body first, state word last.
    fn write_header(&mut self, sector: usize, sequence: u32) -> Result<(), FlashError> {
        self.flash.program(sector, 0, &header_body(sequence))?;
        self.flash
            .program(sector, STATE_OFFSET, &STATE_VALID.to_le_bytes())
    }

    fn is_blank(&self, sector: usize) -> Result<bool, FlashError> {
        self.is_blank_from(sector, 0)
    }

    /// Whether every byte of `sector` from `start` on is erased.
    fn is_blank_from(&self, sector: usize, start: usize) -> Result<bool, FlashError> {
        let mut buf = [0_u8; 16];
        let mut offset = start;
        while offset < F::SECTOR_SIZE {
            let n = buf.len().min(F::SECTOR_SIZE - offset);
            let part = buf.get_mut(..n).ok_or(FlashError::OutOfBounds)?;
            self.flash.read(sector, offset, part)?;
            if part.iter().any(|&b| b != 0xFF) {
                return Ok(false);
            }
            offset += n;
        }
        Ok(true)
    }

    fn slot_offset(slot: usize) -> usize {
        RECORDS_OFFSET + slot * RECORD_SIZE
    }

    fn key_at(&self, sector: usize, slot: usize) -> Result<u32, FlashError> {
        self.read_u32(sector, Self::slot_offset(slot))
    }

    fn read_slot(&self, sector: usize, slot: usize) -> Result<Slot, FlashError> {
        let o = Self::slot_offset(slot);
        Ok(decode_slot(
            self.read_u32(sector, o)?,
            self.read_u32(sector, o + 4)?,
            self.read_u32(sector, o + 8)?,
        ))
    }

    /// Finds `used` and `torn` of the active sector.
    fn scan_active(&mut self) -> Result<(), FlashError> {
        let sector = self.sector()?;
        self.used = 0;
        self.torn = 0;
        for slot in 0..Self::SLOTS {
            match self.read_slot(sector, slot)? {
                Slot::Free => {}
                s => {
                    self.used = slot + 1;
                    if s == Slot::Torn {
                        self.torn += 1;
                    }
                }
            }
        }
        Ok(())
    }

    /// Whether a record after `slot` (a valid set or tombstone) has `key`.
    fn superseded(&self, sector: usize, slot: usize, key: u32) -> Result<bool, FlashError> {
        for later in slot + 1..self.used {
            if self.key_at(sector, later)? == key
                && matches!(
                    self.read_slot(sector, later)?,
                    Slot::Set(..) | Slot::Tomb(_)
                )
            {
                return Ok(true);
            }
        }
        Ok(false)
    }

    fn lookup(&self, key: u32) -> Result<Option<u32>, FlashError> {
        let Some(sector) = self.active else {
            return Ok(None);
        };
        for slot in (0..self.used).rev() {
            if self.key_at(sector, slot)? == key {
                match self.read_slot(sector, slot)? {
                    Slot::Set(_, v) => return Ok(Some(v)),
                    Slot::Tomb(_) => return Ok(None),
                    Slot::Free | Slot::Torn => {}
                }
            }
        }
        Ok(None)
    }

    /// Programs a record in two calls: key and value, then the check word. A
    /// power cut inside either can never leave something that reads as a
    /// record (see [`crate::layout`]).
    fn program_record(
        &mut self,
        sector: usize,
        slot: usize,
        key: u32,
        value: Option<u32>,
    ) -> Result<(), FlashError> {
        let (kv, check) = record_parts(key, value.unwrap_or(0), value.is_none());
        let at = Self::slot_offset(slot);
        self.flash.program(sector, at, &kv)?;
        self.flash.program(sector, at + CHECK_OFFSET, &check)
    }

    /// Appends a set (`Some`) or a tombstone (`None`), compacting if full.
    fn write(&mut self, key: u32, value: Option<u32>) -> Result<(), Error> {
        if self.used < Self::SLOTS {
            let sector = self.sector()?;
            self.program_record(sector, self.used, key, value)?;
            self.used += 1;
            Ok(())
        } else {
            self.compact(key, value)
        }
    }

    /// Copies the live records, with the pending change applied, to the other
    /// sector. The change and the copy become visible together when the new
    /// sector's state word is programmed.
    fn compact(&mut self, key: u32, value: Option<u32>) -> Result<(), Error> {
        let old = self.sector()?;
        let new = old ^ 1;
        let mut live = usize::from(value.is_some());
        for slot in 0..self.used {
            if let Slot::Set(k, _) = self.read_slot(old, slot)?
                && k != key
                && !self.superseded(old, slot, k)?
            {
                live += 1;
            }
        }
        if live > Self::SLOTS {
            return Err(Error::Full);
        }
        let sequence = self.sequence.wrapping_add(1);
        self.begin_sector(new, sequence)?;
        let mut out = 0;
        for slot in 0..self.used {
            if let Slot::Set(k, v) = self.read_slot(old, slot)?
                && k != key
                && !self.superseded(old, slot, k)?
            {
                self.program_record(new, out, k, Some(v))?;
                out += 1;
            }
        }
        if let Some(v) = value {
            self.program_record(new, out, key, Some(v))?;
            out += 1;
        }
        self.finish_sector(old, new, sequence, out)?;
        Ok(())
    }

    /// Erases `new` and programs the body of its header (not yet valid).
    fn begin_sector(&mut self, new: usize, sequence: u32) -> Result<(), FlashError> {
        self.flash.erase(new)?;
        self.flash.program(new, 0, &header_body(sequence))
    }

    /// Makes `new` (with `out` records) the live sector by programming its state
    /// word, then retires `old` so that it can never be mistaken for the live
    /// one if `new` is damaged later.
    fn finish_sector(
        &mut self,
        old: usize,
        new: usize,
        sequence: u32,
        out: usize,
    ) -> Result<(), FlashError> {
        self.flash
            .program(new, STATE_OFFSET, &pack::<4>(&[STATE_VALID]))?;
        self.active = Some(new);
        self.sequence = sequence;
        self.used = out;
        self.flash
            .program(old, STATE_OFFSET, &pack::<4>(&[STATE_RETIRED]))
    }

    /// Makes the store hold exactly `wanted` (distinct keys): every pair in it
    /// reads back, every other key is gone. All or nothing under power loss.
    ///
    /// The differences are counted: pairs of `wanted` that are absent or hold
    /// another value, plus live keys not in `wanted`. With none nothing is
    /// written ([`Replaced::Unchanged`]); with exactly one a single record is
    /// appended, a set or a tombstone, compacting if the sector is full as
    /// any append may ([`Replaced::Appended`]); with two or more a fresh sector
    /// holding exactly `wanted` is written, in the way a compaction writes one
    /// (erase the other sector, header body with the next sequence, the
    /// records, the state word valid, then retire the old sector), so the
    /// change becomes visible in one program call ([`Replaced::Rewritten`]).
    ///
    /// The result of repeating a key in `wanted` is unspecified (but nothing
    /// panics and the store stays consistent).
    ///
    /// # Errors
    /// [`Error::Full`] when `wanted` has more pairs than a sector has slots
    /// (before anything is erased); [`Error::Corrupt`] on a store mounted
    /// [`MountReport::Corrupt`] (format it first); flash errors.
    pub fn replace_all(
        &mut self,
        wanted: impl Iterator<Item = (u32, u32)> + Clone,
    ) -> Result<Replaced, Error> {
        self.ready()?;
        let r = self.replace_all_inner(wanted);
        self.note(r)
    }

    fn replace_all_inner(
        &mut self,
        wanted: impl Iterator<Item = (u32, u32)> + Clone,
    ) -> Result<Replaced, Error> {
        if wanted.clone().count() > Self::SLOTS {
            return Err(Error::Full);
        }
        // The first difference, and whether there is a second.
        let mut first: Option<(u32, Option<u32>)> = None;
        let mut several = false;
        for (k, v) in wanted.clone() {
            if self.lookup(k)? != Some(v) {
                several = first.is_some();
                first.get_or_insert((k, Some(v)));
                if several {
                    break;
                }
            }
        }
        if !several {
            for item in self.iter() {
                let (k, _) = item?;
                if !wanted.clone().any(|(w, _)| w == k) {
                    several = first.is_some();
                    first.get_or_insert((k, None));
                    if several {
                        break;
                    }
                }
            }
        }
        match (first, several) {
            (None, _) => Ok(Replaced::Unchanged),
            (Some((key, value)), false) => {
                self.write(key, value)?;
                Ok(Replaced::Appended)
            }
            (Some(_), true) => {
                let old = self.sector()?;
                let new = old ^ 1;
                let sequence = self.sequence.wrapping_add(1);
                self.begin_sector(new, sequence)?;
                let mut out = 0;
                for (k, v) in wanted {
                    self.program_record(new, out, k, Some(v))?;
                    out += 1;
                }
                self.finish_sector(old, new, sequence, out)?;
                Ok(Replaced::Rewritten)
            }
        }
    }
}

/// Iterator over live keys and values; see [`Store::iter`].
pub struct Iter<'a, F: Flash> {
    store: &'a Store<F>,
    next: usize,
    /// Set after an error has been yielded.
    done: bool,
}

impl<F: Flash> Iter<'_, F> {
    /// Whether the record in `slot` is a set that nothing later replaces.
    fn live(&self, sector: usize, slot: usize) -> Result<Option<(u32, u32)>, FlashError> {
        let Slot::Set(k, v) = self.store.read_slot(sector, slot)? else {
            return Ok(None);
        };
        Ok((!self.store.superseded(sector, slot, k)?).then_some((k, v)))
    }
}

impl<F: Flash> Iterator for Iter<'_, F> {
    type Item = Result<(u32, u32), Error>;

    fn next(&mut self) -> Option<Self::Item> {
        let sector = self.store.active?;
        while !self.done && self.next < self.store.used {
            let slot = self.next;
            self.next += 1;
            match self.live(sector, slot) {
                Ok(Some(kv)) => return Some(Ok(kv)),
                Ok(None) => {}
                Err(e) => {
                    self.done = true;
                    return Some(Err(e.into()));
                }
            }
        }
        None
    }
}
