// SPDX-License-Identifier: AGPL-3.0-or-later
//! The flash interface the store is written against.

/// Why a flash operation failed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FlashError {
    /// Sector, offset or length outside the device.
    OutOfBounds,
    /// A program asked for a 1 bit where the flash holds a 0 (only the
    /// simulated flash reports this; real parts silently AND). The store
    /// never does this.
    BitSet,
    /// The power failed during the operation (simulated flash only).
    PowerLoss,
    /// The device reported a failure.
    Device,
}

/// Two sectors of NOR flash, each [`Flash::SECTOR_SIZE`] bytes.
///
/// Erased bytes read `0xFF`. `program` can only clear bits: the stored byte
/// becomes `old & new`. `erase` sets every byte of a sector to `0xFF`.
///
/// # The contract the store relies on
///
/// * A `program` call that returned `Ok` is complete.
/// * A `program` call interrupted by power loss may have cleared **any subset**
///   of the bits it was asked to clear, in any byte of that call, and **none
///   outside it**. (Not only a prefix of the bytes: a real part programs a
///   word at a time and can stop inside one.)
/// * An interrupted `erase` leaves each byte of the sector unchanged, `0xFF`,
///   or anything between (bits are only ever set, never cleared).
/// * Calls take effect in the order the store makes them: the store never
///   starts a call before the previous one has returned `Ok`. One exception:
///   when mount retires the older of two valid sectors and that call fails,
///   the store carries on with the newer sector. The failed call touched only
///   the state word of the sector that is no longer used, and a state word
///   left half-programmed never reads as valid.
/// * A read returns what is stored, or an error; it never changes the flash.
///
/// # Which flash this fits
///
/// This is the contract of plain NOR flash such as the STM32F405's: a program
/// only clears bits, may be repeated on the same bytes (the state word is
/// programmed after the header body beside it, and again to retire a sector),
/// and an interrupted program leaves a subset of its bits. It is **not** the
/// STM32H743's: that flash programs whole 32-byte words with an
/// error-correcting code, a word can be programmed only once per erase, and an
/// interrupted word reads as an uncorrectable error. The two-call record, the
/// state word programmed later and the retire call all break that. An H7
/// layout (one write per 32-byte word, a torn word treated as a read error
/// that is skipped) comes with H743 bring-up. The calls the store makes, with
/// their offsets and lengths, are [`crate::layout::PROGRAM_CALLS`] (every one
/// is 4-byte aligned and 4, 8 or 12 bytes long; a test checks that no call
/// falls outside the list).
///
/// The store is written so that nothing else is needed (see
/// [`crate::layout`] for why an interrupted write can never read as a valid
/// record, and [`crate::store`] for the mount rules).
pub trait Flash {
    /// Bytes per sector. At least 28 (a header and one record).
    const SECTOR_SIZE: usize;

    /// Reads `buf.len()` bytes of `sector` starting at `offset`.
    fn read(&self, sector: usize, offset: usize, buf: &mut [u8]) -> Result<(), FlashError>;

    /// Programs `data` into `sector` at `offset`, clearing bits only.
    fn program(&mut self, sector: usize, offset: usize, data: &[u8]) -> Result<(), FlashError>;

    /// Erases `sector`.
    fn erase(&mut self, sector: usize) -> Result<(), FlashError>;
}

impl<F: Flash> Flash for &mut F {
    const SECTOR_SIZE: usize = F::SECTOR_SIZE;

    fn read(&self, sector: usize, offset: usize, buf: &mut [u8]) -> Result<(), FlashError> {
        (**self).read(sector, offset, buf)
    }

    fn program(&mut self, sector: usize, offset: usize, data: &[u8]) -> Result<(), FlashError> {
        (**self).program(sector, offset, data)
    }

    fn erase(&mut self, sector: usize) -> Result<(), FlashError> {
        (**self).erase(sector)
    }
}
