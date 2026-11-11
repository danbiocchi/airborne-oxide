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
/// becomes `old & new`. `erase` sets every byte of a sector to `0xFF`. After
/// an interrupted `program` or `erase` the contents are unspecified beyond
/// that: bits may only have moved in the direction of the operation.
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
