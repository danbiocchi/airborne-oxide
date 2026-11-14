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
//!
//! A cut can also fall *inside* a program call, which is what real flash does
//! (the flash contract in [`crate::flash`]): [`SimFlash::cut_program_call`]
//! names a program call by number and leaves any subset of the bits it was
//! asked to clear cleared, chosen by a [`BitCut`]. The bits a call clears are
//! numbered in order, byte by byte and from bit 0 of each byte up, counting
//! only bits that are 1 in the flash and 0 in the data;
//! [`SimFlash::bits_cleared_by`] gives how many there are, so that a test can
//! enumerate them.

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

/// Which bits of an interrupted program call got cleared.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BitCut {
    /// Each bit the call would clear is cleared or not by a generator seeded
    /// with this value.
    Seeded(u64),
    /// Every bit the call would clear except bit `k` (counted as described in
    /// the module docs).
    AllBut(usize),
    /// Only bit `k` of those the call would clear.
    Only(usize),
}

/// Two sectors of `N` bytes each.
#[derive(Clone, Debug)]
pub struct SimFlash<const N: usize> {
    data: [[u8; N]; 2],
    erases: [u32; 2],
    events: u64,
    reads: Cell<u64>,
    cut: Option<(u64, u64)>,
    /// Program calls started (and not refused for bounds or a bit set).
    calls: u64,
    /// The program call (by absolute number) to interrupt, and how.
    call_cut: Option<(u64, BitCut)>,
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
            calls: 0,
            call_cut: None,
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

    /// Program calls made so far (a call refused for bounds or for setting a
    /// bit does not count; an interrupted one does).
    #[must_use]
    pub fn program_calls(&self) -> u64 {
        self.calls
    }

    /// Interrupts the program call that is `after_calls` calls from now (0 =
    /// the next one), leaving the subset of its bit-clears that `how` names.
    pub fn cut_program_call(&mut self, after_calls: u64, how: BitCut) {
        self.call_cut = Some((self.calls.saturating_add(after_calls), how));
    }

    /// How many bits `program(sector, offset, data)` would clear (bits that
    /// are 1 in the flash and 0 in `data`); `None` when out of bounds.
    #[must_use]
    pub fn bits_cleared_by(&self, sector: usize, offset: usize, data: &[u8]) -> Option<usize> {
        let end = offset.checked_add(data.len())?;
        let dst = self.data.get(sector)?.get(offset..end)?;
        Some(
            dst.iter()
                .zip(data)
                .map(|(&old, &new)| (old & !new).count_ones() as usize)
                .sum(),
        )
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
        self.call_cut = None;
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

    /// Clears the subset of the call's bits that `how` names, then dies.
    fn interrupt_call(
        &mut self,
        sector: usize,
        offset: usize,
        data: &[u8],
        how: BitCut,
        call: u64,
    ) {
        let mut rng = Xorshift::new(match how {
            BitCut::Seeded(seed) => seed ^ call.rotate_left(32),
            _ => 0,
        });
        let mut index = 0_usize;
        if let Some(dst) = self.data.get_mut(sector) {
            for (i, &new) in data.iter().enumerate() {
                let Some(byte) = dst.get_mut(offset + i) else {
                    break;
                };
                for bit in 0..8 {
                    let mask = 1_u8 << bit;
                    if *byte & mask == 0 || new & mask != 0 {
                        continue; // not a bit this call clears
                    }
                    let clear = match how {
                        BitCut::Seeded(_) => rng.next() >> 33 & 1 == 1,
                        BitCut::AllBut(k) => index != k,
                        BitCut::Only(k) => index == k,
                    };
                    if clear {
                        *byte &= !mask;
                    }
                    index += 1;
                }
            }
        }
        self.events += data.len() as u64;
        self.dead = true;
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
        let call = self.calls;
        self.calls += 1;
        if let Some((n, how)) = self.call_cut
            && n == call
        {
            self.interrupt_call(sector, offset, data, how, call);
            return Err(FlashError::PowerLoss);
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

    /// Reads `n` bytes of sector 0.
    fn image<const N: usize>(f: &SimFlash<N>) -> [u8; N] {
        *f.bytes(0).unwrap()
    }

    #[test]
    fn bits_cleared_by_counts_the_ones_that_become_zeros() {
        let mut f = SimFlash::<32>::new();
        assert_eq!(f.bits_cleared_by(0, 0, &[0x00, 0xFF, 0x0F]), Some(8 + 4));
        f.program(0, 0, &[0xF0]).unwrap();
        // Bits already clear do not count; bits the data would set are ignored
        // here (the program itself refuses them).
        assert_eq!(f.bits_cleared_by(0, 0, &[0x00]), Some(4));
        assert_eq!(f.bits_cleared_by(0, 31, &[0, 0]), None);
        assert_eq!(f.bits_cleared_by(2, 0, &[0]), None);
    }

    /// Which bits a cut call left cleared: the ones of `old` that are now 0.
    fn cleared_bits(before: &[u8], after: &[u8]) -> Vec<usize> {
        let mut v = Vec::new();
        for (i, (b, a)) in before.iter().zip(after).enumerate() {
            for bit in 0..8 {
                assert!(a & !b == 0, "a bit was set");
                if (b >> bit) & 1 == 1 && (a >> bit) & 1 == 0 {
                    v.push(i * 8 + bit);
                }
            }
        }
        v
    }

    #[test]
    fn all_but_bit_k_clears_every_bit_of_the_call_except_that_one() {
        let data = [0x3C, 0x00, 0xA5, 0xFF];
        let probe = SimFlash::<32>::new();
        let n = probe.bits_cleared_by(0, 4, &data).unwrap();
        assert_eq!(n, 4 + 8 + 4);
        let want_all: Vec<usize> = (0..data.len() * 8)
            .filter(|i| (data[i / 8] >> (i % 8)) & 1 == 0)
            .collect();
        for k in 0..n {
            let mut f = SimFlash::<32>::new();
            let before = image(&f);
            f.cut_program_call(0, BitCut::AllBut(k));
            assert_eq!(f.program(0, 4, &data), Err(FlashError::PowerLoss));
            assert!(f.is_dead());
            let after = image(&f);
            assert_eq!(&before[..4], &after[..4], "outside the call");
            assert_eq!(&before[8..], &after[8..], "outside the call");
            let cleared = cleared_bits(&before[4..8], &after[4..8]);
            assert_eq!(cleared.len(), n - 1, "k={k}");
            assert!(cleared.iter().all(|c| want_all.contains(c)));
            // Exactly one of the wanted bits is missing; it is the k-th.
            let missing: Vec<_> = want_all.iter().filter(|w| !cleared.contains(w)).collect();
            assert_eq!(missing, vec![&want_all[k]], "k={k}");
            // Dead until power is restored; then the rest can be programmed.
            assert_eq!(f.program(0, 12, &[0]), Err(FlashError::PowerLoss));
            assert_eq!(f.erase(0), Err(FlashError::PowerLoss));
            f.restore_power();
            f.program(0, 12, &[0]).unwrap();
        }
    }

    #[test]
    fn only_bit_k_clears_just_that_one() {
        let data = [0x3C, 0x00, 0xA5, 0xFF];
        let n = SimFlash::<32>::new().bits_cleared_by(0, 4, &data).unwrap();
        let want_all: Vec<usize> = (0..data.len() * 8)
            .filter(|i| (data[i / 8] >> (i % 8)) & 1 == 0)
            .collect();
        assert_eq!(want_all.len(), n);
        for (k, &bit) in want_all.iter().enumerate() {
            let mut f = SimFlash::<32>::new();
            let before = image(&f);
            f.cut_program_call(0, BitCut::Only(k));
            assert_eq!(f.program(0, 4, &data), Err(FlashError::PowerLoss));
            assert!(f.is_dead());
            let after = image(&f);
            assert_eq!(&before[..4], &after[..4]);
            assert_eq!(&before[8..], &after[8..]);
            assert_eq!(cleared_bits(&before[4..8], &after[4..8]), vec![bit]);
            assert_eq!(f.program(0, 12, &[0]), Err(FlashError::PowerLoss));
            f.restore_power();
        }
        // A bit index past the end clears nothing, and the flash still dies.
        let mut f = SimFlash::<32>::new();
        f.cut_program_call(0, BitCut::Only(n));
        assert_eq!(f.program(0, 4, &data), Err(FlashError::PowerLoss));
        assert_eq!(image(&f), image(&SimFlash::<32>::new()));
        assert!(f.is_dead());
    }

    #[test]
    fn a_seeded_subset_clears_only_bits_of_the_call_and_depends_on_the_seed() {
        let data = [0x00; 8];
        let mut images = Vec::new();
        for seed in 0..16 {
            let mut f = SimFlash::<32>::new();
            f.cut_program_call(0, BitCut::Seeded(seed));
            assert_eq!(f.program(0, 8, &data), Err(FlashError::PowerLoss));
            assert!(f.is_dead());
            let img = image(&f);
            assert_eq!(&img[..8], &[0xFF; 8], "outside the call");
            assert_eq!(&img[16..], &[0xFF; 16], "outside the call");
            let cleared = cleared_bits(&[0xFF; 8], &img[8..16]);
            assert!(!cleared.is_empty() && cleared.len() < 64, "{cleared:?}");
            images.push(img);
            // The same seed gives the same subset.
            let mut g = SimFlash::<32>::new();
            g.cut_program_call(0, BitCut::Seeded(seed));
            let _ = g.program(0, 8, &data);
            assert_eq!(image(&g), img);
        }
        images.sort();
        images.dedup();
        assert!(images.len() > 8);
    }

    #[test]
    fn the_cut_names_the_call_by_number_and_earlier_calls_complete() {
        let mut f = SimFlash::<32>::new();
        f.program(0, 0, &[0x00]).unwrap();
        assert_eq!(f.program_calls(), 1);
        f.cut_program_call(2, BitCut::AllBut(0));
        f.program(0, 1, &[0x00]).unwrap();
        f.program(0, 2, &[0x00]).unwrap();
        assert!(!f.is_dead());
        assert_eq!(f.program(0, 3, &[0x01, 0x00]), Err(FlashError::PowerLoss));
        // Every bit but the first of the call (bit 1 of byte 3): the call wants
        // 0xFE there, and bit 0 was never to be cleared.
        assert_eq!(f.bytes(0).unwrap()[3], 0x03);
        assert_eq!(f.bytes(0).unwrap()[4], 0x00);
        assert!(f.is_dead());
        f.restore_power();
        f.program(0, 5, &[0]).unwrap();
    }

    #[test]
    fn a_refused_call_is_not_counted_and_a_cut_call_refuses_bit_sets_first() {
        let mut f = SimFlash::<32>::new();
        f.program(0, 0, &[0x0F]).unwrap();
        assert_eq!(f.program(0, 0, &[0xFF]), Err(FlashError::BitSet));
        assert_eq!(f.program(5, 0, &[0]), Err(FlashError::OutOfBounds));
        assert_eq!(f.program_calls(), 1);
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
