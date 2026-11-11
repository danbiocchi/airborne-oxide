// SPDX-License-Identifier: AGPL-3.0-or-later
//! A simulated NOR flash for host tests: enforces the programming rules,
//! counts erases and reads, and can lose power at a chosen point.
//!
//! Power is cut by *event number*: every programmed byte is one event and every
//! erase is one event. A cut at event `n` lets events `0..n` complete. If event
//! `n` is a programmed byte, that byte and the rest of the operation are not
//! written. If it is an erase, the sector is left part-way erased: each byte
//! independently keeps its old value, becomes `0xFF`, or has a random subset of
//! its cleared bits set (never a bit cleared), drawn from a generator seeded
//! by the cut. After the cut every operation fails with
//! [`FlashError::PowerLoss`] until [`SimFlash::restore_power`].

use core::cell::Cell;

use crate::flash::{Flash, FlashError};

/// xorshift64 generator (Marsaglia), enough to scramble an interrupted erase.
#[derive(Clone, Copy, Debug)]
struct Xorshift(u64);

impl Xorshift {
    fn new(seed: u64) -> Self {
        // A zero state would stay zero.
        Self(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) ^ 0xD1B5_4A32_D192_ED03 | 1)
    }

    fn next(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        x
    }
}

/// Two sectors of `N` bytes each.
#[derive(Clone, Debug)]
pub struct SimFlash<const N: usize> {
    data: [[u8; N]; 2],
    erases: [u32; 2],
    events: u64,
    reads: Cell<u64>,
    cut: Option<(u64, u64)>,
    dead: bool,
}

impl<const N: usize> Default for SimFlash<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> SimFlash<N> {
    /// Fully erased flash.
    #[must_use]
    pub fn new() -> Self {
        Self {
            data: [[0xFF; N]; 2],
            erases: [0; 2],
            events: 0,
            reads: Cell::new(0),
            cut: None,
            dead: false,
        }
    }

    /// Events (programmed bytes plus erases) so far.
    #[must_use]
    pub fn events(&self) -> u64 {
        self.events
    }

    /// Calls to `read` so far.
    #[must_use]
    pub fn reads(&self) -> u64 {
        self.reads.get()
    }

    /// Erases of `sector` so far (an interrupted erase counts).
    #[must_use]
    pub fn erase_count(&self, sector: usize) -> u32 {
        self.erases.get(sector).copied().unwrap_or(0)
    }

    /// Erases of both sectors.
    #[must_use]
    pub fn total_erases(&self) -> u32 {
        self.erases.iter().sum()
    }

    /// Cuts the power at the event `after_events` events from now. `seed`
    /// shapes an interrupted erase.
    pub fn cut_power_after(&mut self, after_events: u64, seed: u64) {
        self.cut = Some((self.events.saturating_add(after_events), seed));
    }

    /// Whether the cut has happened.
    #[must_use]
    pub fn is_dead(&self) -> bool {
        self.dead
    }

    /// Powers up again and clears any pending cut. The contents stay.
    pub fn restore_power(&mut self) {
        self.dead = false;
        self.cut = None;
    }

    /// Direct view of a sector, for tests that damage it.
    #[must_use]
    pub fn bytes(&self, sector: usize) -> Option<&[u8; N]> {
        self.data.get(sector)
    }

    /// Direct mutable view of a sector, for tests that damage it.
    pub fn bytes_mut(&mut self, sector: usize) -> Option<&mut [u8; N]> {
        self.data.get_mut(sector)
    }

    fn cut_now(&self) -> Option<u64> {
        match self.cut {
            Some((at, seed)) if at == self.events => Some(seed),
            _ => None,
        }
    }
}

impl<const N: usize> Flash for SimFlash<N> {
    const SECTOR_SIZE: usize = N;

    fn read(&self, sector: usize, offset: usize, buf: &mut [u8]) -> Result<(), FlashError> {
        let end = offset
            .checked_add(buf.len())
            .ok_or(FlashError::OutOfBounds)?;
        let src = self
            .data
            .get(sector)
            .and_then(|s| s.get(offset..end))
            .ok_or(FlashError::OutOfBounds)?;
        buf.copy_from_slice(src);
        self.reads.set(self.reads.get() + 1);
        Ok(())
    }

    fn program(&mut self, sector: usize, offset: usize, data: &[u8]) -> Result<(), FlashError> {
        if self.dead {
            return Err(FlashError::PowerLoss);
        }
        let end = offset
            .checked_add(data.len())
            .ok_or(FlashError::OutOfBounds)?;
        let dst = self
            .data
            .get(sector)
            .and_then(|s| s.get(offset..end))
            .ok_or(FlashError::OutOfBounds)?;
        if dst.iter().zip(data).any(|(&old, &new)| new & !old != 0) {
            return Err(FlashError::BitSet);
        }
        for (i, &new) in data.iter().enumerate() {
            if self.cut_now().is_some() {
                self.dead = true;
                return Err(FlashError::PowerLoss);
            }
            let byte = self
                .data
                .get_mut(sector)
                .and_then(|s| s.get_mut(offset + i))
                .ok_or(FlashError::OutOfBounds)?;
            *byte &= new;
            self.events += 1;
        }
        Ok(())
    }

    fn erase(&mut self, sector: usize) -> Result<(), FlashError> {
        if self.dead {
            return Err(FlashError::PowerLoss);
        }
        let cut = self.cut_now();
        let s = self.data.get_mut(sector).ok_or(FlashError::OutOfBounds)?;
        if let Some(seed) = cut {
            let mut rng = Xorshift::new(seed ^ self.events.rotate_left(32));
            for b in s.iter_mut() {
                let r = rng.next();
                // Only ever set bits.
                let set = match r % 3 {
                    0 => 0x00,
                    1 => 0xFF,
                    _ => (r >> 8) as u8,
                };
                *b |= set;
            }
            self.dead = true;
        } else {
            s.fill(0xFF);
        }
        if let Some(n) = self.erases.get_mut(sector) {
            *n += 1;
        }
        self.events += 1;
        if cut.is_some() {
            Err(FlashError::PowerLoss)
        } else {
            Ok(())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn programming_only_clears_bits() {
        let mut f = SimFlash::<32>::new();
        f.program(0, 0, &[0xF0, 0xFF]).unwrap();
        f.program(0, 0, &[0xC0]).unwrap();
        let mut b = [0; 2];
        f.read(0, 0, &mut b).unwrap();
        assert_eq!(b, [0xC0, 0xFF]);
    }

    #[test]
    fn setting_a_cleared_bit_is_an_error_and_writes_nothing() {
        let mut f = SimFlash::<32>::new();
        f.program(0, 0, &[0x0F]).unwrap();
        assert_eq!(f.program(0, 0, &[0xAA, 0x00]), Err(FlashError::BitSet));
        let mut b = [0; 2];
        f.read(0, 0, &mut b).unwrap();
        assert_eq!(b, [0x0F, 0xFF]);
    }

    #[test]
    fn erase_restores_ff_and_counts() {
        let mut f = SimFlash::<32>::new();
        f.program(1, 3, &[0]).unwrap();
        f.erase(1).unwrap();
        assert_eq!(f.bytes(1).unwrap(), &[0xFF; 32]);
        assert_eq!(
            (f.erase_count(0), f.erase_count(1), f.total_erases()),
            (0, 1, 1)
        );
    }

    #[test]
    fn out_of_range_is_refused() {
        let mut f = SimFlash::<32>::new();
        assert_eq!(f.program(2, 0, &[0]), Err(FlashError::OutOfBounds));
        assert_eq!(f.program(0, 31, &[0, 0]), Err(FlashError::OutOfBounds));
        assert_eq!(f.erase(2), Err(FlashError::OutOfBounds));
        assert_eq!(
            f.read(0, usize::MAX, &mut [0]),
            Err(FlashError::OutOfBounds)
        );
    }

    #[test]
    fn power_cut_stops_after_exactly_n_bytes() {
        for n in 0..6 {
            let mut f = SimFlash::<32>::new();
            f.cut_power_after(n, 0);
            assert_eq!(f.program(0, 0, &[0; 6]), Err(FlashError::PowerLoss));
            assert!(f.is_dead());
            assert_eq!(f.program(0, 8, &[0]), Err(FlashError::PowerLoss));
            let mut b = [0xAA; 8];
            f.read(0, 0, &mut b).unwrap();
            let zeros = b.iter().take_while(|&&x| x == 0).count();
            assert_eq!(zeros as u64, n);
            assert!(b.iter().skip(n as usize).all(|&x| x == 0xFF));
            f.restore_power();
            f.program(0, 8, &[0]).unwrap();
        }
    }

    #[test]
    fn interrupted_erase_only_sets_bits_and_depends_on_the_seed() {
        let mut images = Vec::new();
        for seed in 0..8 {
            let mut f = SimFlash::<64>::new();
            f.program(0, 0, &[0x12; 64]).unwrap();
            f.cut_power_after(0, seed);
            assert_eq!(f.erase(0), Err(FlashError::PowerLoss));
            let img = *f.bytes(0).unwrap();
            assert!(img.iter().all(|&b| b & 0x12 == 0x12), "a bit was cleared");
            assert!(img.iter().any(|&b| b != 0xFF) && img.contains(&0xFF));
            images.push(img);
        }
        images.sort();
        images.dedup();
        assert!(images.len() > 4);
    }
}
