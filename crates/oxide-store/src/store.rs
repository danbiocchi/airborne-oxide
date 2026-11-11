// SPDX-License-Identifier: AGPL-3.0-or-later
//! The store: an append-only log of records in one sector at a time, compacted
//! into the other sector when full.
//!
//! # Cost
//!
//! With `S` = `SECTOR_SIZE` the sector holds `n = (S - 16) / 12` records
//! (1364 for 16 KB). `get` reads at most `n` keys and verifies the CRC of
//! matching ones. Finding which records are live (`iter`, compaction) asks, for
//! each record, whether a later one has the same key: at most `n(n-1)/2` four
//! byte key reads (about 0.93 million for 16 KB; a CRC is computed only for a
//! later record with the same key). A compaction does that once to count and
//! once to copy, plus one sector erase and at most `n` record programs. No
//! allocation; the stack use is a few dozen bytes.
//!
//! Every `set` of a full sector compacts, so a sector full of distinct live
//! keys erases on every write; the caller keeps the live key count well below
//! `n` (a parameter table has about 100).

use crate::flash::{Flash, FlashError};
use crate::layout::{
    HEADER_SIZE, Header, RECORD_SIZE, RECORDS_OFFSET, STATE_OFFSET, STATE_VALID, Slot,
    decode_header, decode_slot, header_body, pack, record_bytes,
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

/// What [`Store::mount`] found.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MountReport {
    /// A valid sector and nothing to ignore.
    Clean,
    /// Both sectors were erased; the store was initialised empty.
    Fresh,
    /// A valid sector, but a half-written or damaged sector or torn records
    /// were ignored. No acknowledged write was lost.
    Recovered,
    /// No valid sector, but the flash is not blank. The store is empty and
    /// refuses writes until [`Store::format`].
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

    /// Opens the store, repairing nothing on flash except initialising a blank
    /// device. See [`MountReport`].
    ///
    /// # Errors
    /// A flash read or program failure.
    pub fn mount(flash: F) -> Result<(Self, MountReport), Error> {
        let mut s = Self {
            flash,
            active: None,
            sequence: 0,
            used: 0,
            torn: 0,
            faulted: false,
        };
        let h = [s.read_header(0)?, s.read_header(1)?];
        let best = match (h[0], h[1]) {
            (Header::Valid(a), Header::Valid(b)) => Some(usize::from(b > a)),
            (Header::Valid(_), _) => Some(0),
            (_, Header::Valid(_)) => Some(1),
            _ => None,
        };
        if let Some(a) = best {
            let seq = match h.get(a) {
                Some(Header::Valid(q)) => *q,
                _ => 0,
            };
            s.active = Some(a);
            s.sequence = seq;
            s.scan_active()?;
            let other_ignored = matches!(h.get(a ^ 1), Some(Header::Writing(_) | Header::Bad));
            let report = if s.torn > 0 || other_ignored {
                MountReport::Recovered
            } else {
                MountReport::Clean
            };
            return Ok((s, report));
        }
        if s.is_blank(0)? && s.is_blank(1)? {
            s.write_header(0, 0)?;
            s.active = Some(0);
            return Ok((s, MountReport::Fresh));
        }
        Ok((s, MountReport::Corrupt))
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

    /// The newest value for `key`. A flash read failure reads as `None`.
    #[must_use]
    pub fn get(&self, key: u32) -> Option<u32> {
        self.lookup(key).ok().flatten()
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
    #[must_use]
    pub fn iter(&self) -> Iter<'_, F> {
        Iter {
            store: self,
            next: 0,
        }
    }

    /// Erases whatever is on the flash and starts an empty store. Destructive;
    /// the way out of [`MountReport::Corrupt`]. Unlike `set`, an interruption
    /// is not all-or-nothing: mount again and format again.
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
            if let Header::Valid(q) | Header::Writing(q) = self.read_header(sector)? {
                highest = Some(highest.map_or(q, |h| h.max(q)));
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
    fn note(&mut self, r: Result<(), Error>) -> Result<(), Error> {
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
        let mut buf = [0_u8; 16];
        let mut offset = 0;
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

    /// Appends a set (`Some`) or a tombstone (`None`), compacting if full.
    fn write(&mut self, key: u32, value: Option<u32>) -> Result<(), Error> {
        if self.used < Self::SLOTS {
            let sector = self.sector()?;
            let bytes = record_bytes(key, value.unwrap_or(0), value.is_none());
            self.flash
                .program(sector, Self::slot_offset(self.used), &bytes)?;
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
        self.flash.erase(new)?;
        self.flash.program(new, 0, &header_body(sequence))?;
        let mut out = 0;
        for slot in 0..self.used {
            if let Slot::Set(k, v) = self.read_slot(old, slot)?
                && k != key
                && !self.superseded(old, slot, k)?
            {
                self.flash
                    .program(new, Self::slot_offset(out), &record_bytes(k, v, false))?;
                out += 1;
            }
        }
        if let Some(v) = value {
            self.flash
                .program(new, Self::slot_offset(out), &record_bytes(key, v, false))?;
            out += 1;
        }
        self.flash
            .program(new, STATE_OFFSET, &pack::<4>(&[STATE_VALID]))?;
        self.active = Some(new);
        self.sequence = sequence;
        self.used = out;
        Ok(())
    }
}

/// Iterator over live keys and values; see [`Store::iter`].
pub struct Iter<'a, F: Flash> {
    store: &'a Store<F>,
    next: usize,
}

impl<F: Flash> Iterator for Iter<'_, F> {
    type Item = (u32, u32);

    fn next(&mut self) -> Option<(u32, u32)> {
        let sector = self.store.active?;
        while self.next < self.store.used {
            let slot = self.next;
            self.next += 1;
            if let Ok(Slot::Set(k, v)) = self.store.read_slot(sector, slot)
                && self.store.superseded(sector, slot, k) == Ok(false)
            {
                return Some((k, v));
            }
        }
        None
    }
}
