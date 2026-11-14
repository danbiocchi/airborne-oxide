// SPDX-License-Identifier: AGPL-3.0-or-later
//! On-flash layout. All integers are little-endian.
//!
//! ```text
//! sector header (16 bytes)            record (12 bytes, two program calls)
//!   0  u16 magic   0x534F               0  u32 key        } call 1
//!   2  u16 version 1                    4  u32 value      }
//!   4  u32 sequence                     8  u32 check word   call 2
//!   8  u32 CRC-32 of bytes 0..8
//!  12  u32 state: FFFFFFFF writing, A55A3CC3 valid, 00000000 retired
//! ```
//!
//! A header is *intact* when its first 12 bytes are the magic and version, a
//! sequence number and the CRC-32 of those 8 bytes. An intact header is
//! `Valid` with state `A55A3CC3`, `Writing` with state `FFFFFFFF`, and
//! `Retired` with any other state; a header that is all `0xFF` is `Blank`;
//! anything else is `Bad`.
//!
//! A record slot that reads all `0xFF` is free. A record whose check word
//! fails is torn: skipped and never reused.
//!
//! # The check word
//!
//! The check word of a record (bytes 8..12) is `crc30 | kind << 30`, where
//! `crc30` is the low 30 bits of the CRC-32 of the key and value, and `kind`
//! is two bits, **one-hot**: `0b01` for a set and `0b10` for a tombstone (a
//! removed key, stored with value 0). `0b11` (what erased flash reads) and
//! `0b00` are not valid kinds, so a check word that is erased, or that
//! decodes to either, is never a record.
//!
//! # Why an interrupted write can never read as a record
//!
//! The flash contract ([`crate::flash::Flash`]) is that a program call
//! interrupted by power loss leaves any subset of the bits it was asked to
//! clear cleared, in any byte of that call, and nothing outside it. A record
//! is written in two calls, key and value first, check word second, and the
//! second is not started before the first has returned `Ok`. Two cases:
//!
//! * **The key-and-value call is interrupted.** The check call never ran, so
//!   the check word is still erased: kind `0b11`, which no record has. The
//!   slot is torn whatever state the key and value are in (or free, if
//!   nothing at all was cleared).
//! * **The check call is interrupted.** Key and value are final. The check
//!   word is the erased word with some of the zeros of the complete word `C`
//!   cleared, so it still has a 1 everywhere `C` has a 1. It is a record only
//!   if it equals the complete check word of this key and value, either `C`
//!   (every bit was cleared: the call was not interrupted after all) or `C'`,
//!   the check word of the other kind. A set has bit 31 clear and bit 30 set;
//!   a tombstone has bit 30 clear and bit 31 set. So `C` always has a 1 at a
//!   kind bit where `C'` has a 0, and the interrupted word keeps that 1: it
//!   cannot equal `C'`. With the old encoding (a tombstone flag in bit 30 and
//!   bit 31 always 0) a set of value 0 cut short could become the tombstone
//!   of the same key; the one-hot kind rules that out structurally, not by
//!   luck of the CRC.
//!
//! The CRC is then only a defence against corruption (a bit that flips in
//! storage, which a 30-bit check misses with probability 2^-30), not what
//! makes power loss safe. The tests enumerate every partial state of the
//! check word of several records (`layout::tests`).

use crate::crc::crc32;

/// Bytes in a sector header.
pub const HEADER_SIZE: usize = 16;
/// Bytes in a record.
pub const RECORD_SIZE: usize = 12;
/// Offset of the state word in the header.
pub const STATE_OFFSET: usize = 12;
/// Offset of the first record.
pub const RECORDS_OFFSET: usize = HEADER_SIZE;

/// First header word: magic in the low half, format version in the high half.
pub const WORD0: u32 = (1 << 16) | 0x534F;
/// State word of a finished sector. No byte is 0xFF, so a state word that was
/// only partly programmed can never read as this value.
pub const STATE_VALID: u32 = 0xA55A_3CC3;
/// State word of a sector still being written (erased flash).
pub const STATE_WRITING: u32 = 0xFFFF_FFFF;
/// State word of a sector whose contents have been superseded: programmed over
/// [`STATE_VALID`] once the sector that replaces it is valid. Any other state
/// word of a sector with an intact header body also reads as retired (it is a
/// retirement or a valid-marking that was cut short; the sector is not used).
pub const STATE_RETIRED: u32 = 0x0000_0000;

/// Packs little-endian words into bytes.
#[must_use]
pub fn pack<const N: usize>(words: &[u32]) -> [u8; N] {
    let mut out = [0xFF_u8; N];
    let bytes = words.iter().flat_map(|w| w.to_le_bytes());
    for (d, s) in out.iter_mut().zip(bytes) {
        *d = s;
    }
    out
}

/// The first 12 bytes of a header (magic, version, sequence, CRC); the state
/// word is programmed separately, last.
#[must_use]
pub fn header_body(sequence: u32) -> [u8; 12] {
    pack(&[WORD0, sequence, crc32(&pack::<8>(&[WORD0, sequence]))])
}

/// CRC-32 of a record's key and value.
#[must_use]
pub fn record_crc(key: u32, value: u32) -> u32 {
    crc32(&pack::<8>(&[key, value]))
}

/// Kind bits of a set record's check word.
pub const KIND_SET: u32 = 0b01;
/// Kind bits of a tombstone's check word.
pub const KIND_TOMB: u32 = 0b10;
/// Offset of the check word in a record: a record is programmed in two calls,
/// bytes `0..CHECK_OFFSET` (key and value) and then the check word.
pub const CHECK_OFFSET: usize = 8;

/// Check word of a record: 30 CRC bits and the one-hot kind in bits 30..32.
#[must_use]
pub fn record_check(key: u32, value: u32, tomb: bool) -> u32 {
    let kind = if tomb { KIND_TOMB } else { KIND_SET };
    (record_crc(key, value) & 0x3FFF_FFFF) | (kind << 30)
}

/// The two parts of a record, programmed in this order: key and value, then
/// the check word. A tombstone (`tomb`) ignores `value`.
#[must_use]
pub fn record_parts(key: u32, value: u32, tomb: bool) -> ([u8; CHECK_OFFSET], [u8; 4]) {
    let value = if tomb { 0 } else { value };
    (pack(&[key, value]), pack(&[record_check(key, value, tomb)]))
}

/// The 12 bytes of a record; a tombstone (`tomb`) ignores `value`.
#[must_use]
pub fn record_bytes(key: u32, value: u32, tomb: bool) -> [u8; RECORD_SIZE] {
    if tomb {
        pack(&[key, 0, record_check(key, 0, true)])
    } else {
        pack(&[key, value, record_check(key, value, false)])
    }
}

/// What a record slot holds.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Slot {
    /// All `0xFF`.
    Free,
    /// Written but failing its CRC.
    Torn,
    /// A key and its value.
    Set(u32, u32),
    /// A removed key.
    Tomb(u32),
}

/// Classifies the three words of a record slot.
#[must_use]
pub fn decode_slot(key: u32, value: u32, crc: u32) -> Slot {
    if key == u32::MAX && value == u32::MAX && crc == u32::MAX {
        return Slot::Free;
    }
    // Only the one-hot kinds are records; 0b11 is what erased flash reads.
    match crc >> 30 {
        KIND_SET if crc == record_check(key, value, false) => Slot::Set(key, value),
        KIND_TOMB if value == 0 && crc == record_check(key, 0, true) => Slot::Tomb(key),
        _ => Slot::Torn,
    }
}

/// What a sector header holds.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Header {
    /// All 16 bytes erased.
    Blank,
    /// Intact, state word still erased: a sector being written.
    Writing(u32),
    /// Intact and finished, with its sequence number.
    Valid(u32),
    /// Intact, but its state word is neither erased nor valid: the sector has
    /// been replaced (see [`STATE_RETIRED`]), or the program of its valid
    /// state word was cut short. Its sequence number.
    Retired(u32),
    /// Body damaged, or not a header of this format.
    Bad,
}

/// Classifies the four words of a sector header.
#[must_use]
pub fn decode_header(words: [u32; 4]) -> Header {
    let [w0, seq, crc, state] = words;
    if words.iter().all(|&w| w == u32::MAX) {
        return Header::Blank;
    }
    if w0 != WORD0 || crc != crc32(&pack::<8>(&[w0, seq])) {
        return Header::Bad;
    }
    match state {
        STATE_WRITING => Header::Writing(seq),
        STATE_VALID => Header::Valid(seq),
        _ => Header::Retired(seq),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn words(bytes: &[u8]) -> Vec<u32> {
        bytes
            .chunks_exact(4)
            .map(|c| u32::from_le_bytes([c[0], c[1], c[2], c[3]]))
            .collect()
    }

    #[test]
    fn state_word_has_no_erased_byte() {
        assert!(STATE_VALID.to_le_bytes().iter().all(|&b| b != 0xFF));
    }

    #[test]
    fn an_all_ones_record_is_not_a_real_record() {
        // Otherwise a real record would read as a free slot.
        assert_ne!(record_check(u32::MAX, u32::MAX, false), u32::MAX);
        assert_ne!(record_check(u32::MAX, 0, true), u32::MAX);
    }

    #[test]
    fn a_check_word_has_a_one_hot_kind_and_never_reads_as_erased() {
        for (k, v) in [(0xFFFF_FFFF, 0), (0x101B_2F50, 0), (1, 2), (0, 0)] {
            assert_eq!(record_check(k, v, false) >> 30, 0b01, "set");
            assert_eq!(record_check(k, v, true) >> 30, 0b10, "tombstone");
            assert_ne!(record_check(k, v, false), u32::MAX);
            assert_ne!(record_check(k, v, true), u32::MAX);
        }
    }

    #[test]
    fn a_check_word_with_kind_00_or_11_is_never_a_record() {
        for (k, v) in [(1, 2), (0, 0), (0x101B_2F50, 0), (0xFFFF_FFFF, 0xFFFF_FFFF)] {
            let crc30 = record_crc(k, v) & 0x3FFF_FFFF;
            for kind in [0b00, 0b11] {
                assert_eq!(decode_slot(k, v, crc30 | kind << 30), Slot::Torn);
            }
            let crc30 = record_crc(k, 0) & 0x3FFF_FFFF;
            for kind in [0b00, 0b11] {
                assert_eq!(decode_slot(k, 0, crc30 | kind << 30), Slot::Torn);
            }
        }
    }

    #[test]
    fn records_round_trip() {
        for (k, v) in [
            (0, 0),
            (1, 2),
            (u32::MAX, u32::MAX),
            (7, 0),
            (0xDEAD, 0xBEEF),
        ] {
            let w = words(&record_bytes(k, v, false));
            assert_eq!(decode_slot(w[0], w[1], w[2]), Slot::Set(k, v));
            let w = words(&record_bytes(k, v, true));
            assert_eq!(decode_slot(w[0], w[1], w[2]), Slot::Tomb(k));
        }
    }

    #[test]
    fn every_proper_prefix_of_a_record_is_torn_or_free() {
        for tomb in [false, true] {
            for (k, v) in [
                (1, 2),
                (0xABCD_1234, 0x0F0F_F0F0),
                (5, 0),
                (0xFFFF_FFFF, 0),
                (0x101B_2F50, 0),
            ] {
                let full = record_bytes(k, v, tomb);
                for cut in 0..RECORD_SIZE {
                    let mut b = [0xFF_u8; RECORD_SIZE];
                    b[..cut].copy_from_slice(&full[..cut]);
                    let w = words(&b);
                    let s = decode_slot(w[0], w[1], w[2]);
                    if b == full {
                        continue; // the missing bytes were 0xFF anyway
                    }
                    assert!(
                        matches!(s, Slot::Free | Slot::Torn),
                        "tomb={tomb} k={k} v={v} cut={cut}: {s:?}"
                    );
                }
            }
        }
    }

    #[test]
    fn headers_decode() {
        let body = header_body(7);
        let w = words(&body);
        assert_eq!(
            decode_header([w[0], w[1], w[2], STATE_WRITING]),
            Header::Writing(7)
        );
        assert_eq!(
            decode_header([w[0], w[1], w[2], STATE_VALID]),
            Header::Valid(7)
        );
        assert_eq!(
            decode_header([w[0], w[1], w[2], STATE_RETIRED]),
            Header::Retired(7)
        );
        // A state word cut short on its way to valid, or on its way to retired.
        assert_eq!(
            decode_header([w[0], w[1], w[2], 0xFFFF_00FF]),
            Header::Retired(7)
        );
        assert_eq!(
            decode_header([w[0], w[1], w[2], STATE_VALID & !1]),
            Header::Retired(7)
        );
        assert_eq!(
            decode_header([w[0] ^ 1, w[1], w[2], STATE_RETIRED]),
            Header::Bad
        );
        assert_eq!(
            decode_header([w[0], w[1] ^ 1, w[2], STATE_VALID]),
            Header::Bad
        );
        assert_eq!(decode_header([u32::MAX; 4]), Header::Blank);
        assert_eq!(decode_header([0; 4]), Header::Bad);
    }

    // ------------------------------------------------ interrupted programming

    /// xorshift64, enough to pick bit subsets in tests.
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
    }

    fn rng(seed: u64) -> Rng {
        Rng(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1)
    }

    /// Key, value and check word of a record as the three words of its slot.
    fn slot_words(key: u32, value: u32, tomb: bool) -> [u32; 3] {
        let w = words(&record_bytes(key, value, tomb));
        [w[0], w[1], w[2]]
    }

    /// What a slot may read as after an interrupted write: never a record.
    fn assert_not_a_record(w: [u32; 3], ctx: &str) {
        let got = decode_slot(w[0], w[1], w[2]);
        let want = if w == [u32::MAX; 3] {
            Slot::Free
        } else {
            Slot::Torn
        };
        assert_eq!(got, want, "{ctx}");
    }

    /// Records for the tests below: the awkward keys and values, tombstones,
    /// and enough searched keys that at least three have a check word with
    /// at most 16 zero bits (so that the exhaustive form really runs).
    fn test_records() -> Vec<(u32, u32, bool)> {
        let mut v = Vec::new();
        for (k, val) in [
            (0xFFFF_FFFF, 0xFFFF_FFFF),
            (0xFFFF_FFFF, 0),
            (0x101B_2F50, 0),
            (0x101B_2F50, 9),
            (0, 0),
            (0, 7),
            (1, 2),
            (0xABCD_1234, 0x0F0F_F0F0),
        ] {
            v.push((k, val, false));
            v.push((k, 0, true));
        }
        let few_zeros = |v: &[(u32, u32, bool)]| {
            v.iter()
                .filter(|&&(k, val, t)| (!slot_words(k, val, t)[2]).count_ones() <= 16)
                .count()
        };
        let mut key = 100_u32;
        while few_zeros(&v) < 6 {
            v.push((key, 0, false));
            v.push((key, 0, true));
            key += 1;
        }
        assert!(few_zeros(&v) >= 3);
        v
    }

    /// Words with the bits `cleared` taken out of an erased word.
    fn erased_with(cleared: u32) -> u32 {
        !cleared
    }

    #[test]
    fn an_interrupted_check_call_never_reads_as_a_record() {
        let (mut exhaustive, mut sampled) = (0_usize, 0_usize);
        for (key, value, tomb) in test_records() {
            let [k, v, c] = slot_words(key, value, tomb);
            let z = !c; // the bits the check call clears
            let ctx = format!("key {key:#x} value {value:#x} tomb {tomb}");
            let bits: Vec<u32> = (0..32)
                .filter(|b| z >> b & 1 == 1)
                .map(|b| 1 << b)
                .collect();
            if bits.len() <= 16 {
                exhaustive += 1;
                // Every proper subset of Z got cleared: every partial state.
                for m in 0..(1_u32 << bits.len()) - 1 {
                    let cleared = bits
                        .iter()
                        .enumerate()
                        .filter(|&(i, _)| m >> i & 1 == 1)
                        .fold(0, |a, (_, b)| a | b);
                    assert_ne!(cleared, z);
                    assert_not_a_record([k, v, erased_with(cleared)], &format!("{ctx} m {m}"));
                }
            } else {
                sampled += 1;
                for &b in &bits {
                    assert_not_a_record(
                        [k, v, erased_with(z & !b)],
                        &format!("{ctx} missing {b:#x}"),
                    );
                    assert_not_a_record([k, v, erased_with(b)], &format!("{ctx} only {b:#x}"));
                }
                let mut r = rng(u64::from(key) << 1 | u64::from(tomb));
                for n in 0..10_000 {
                    let cleared = (r.next() as u32) & z;
                    if cleared == z {
                        continue;
                    }
                    assert_not_a_record([k, v, erased_with(cleared)], &format!("{ctx} seeded {n}"));
                }
            }
        }
        assert!(
            exhaustive >= 3,
            "only {exhaustive} records enumerated fully"
        );
        eprintln!("check call: {exhaustive} records enumerated fully, {sampled} sampled");
    }

    #[test]
    fn an_interrupted_key_and_value_call_never_reads_as_a_record() {
        for (key, value, tomb) in test_records() {
            let [k, v, _] = slot_words(key, value, tomb);
            // The check word is still erased: its call has not started.
            let kv = u64::from(v) << 32 | u64::from(k);
            let z = !kv; // the bits the key and value call clears
            let ctx = format!("key {key:#x} value {value:#x} tomb {tomb}");
            let state = |cleared: u64| {
                let w = !cleared;
                [w as u32, (w >> 32) as u32, u32::MAX]
            };
            for b in (0..64).filter(|b| z >> b & 1 == 1).map(|b| 1_u64 << b) {
                assert_not_a_record(state(z & !b), &format!("{ctx} missing {b:#x}"));
                assert_not_a_record(state(b), &format!("{ctx} only {b:#x}"));
            }
            let mut r = rng(u64::from(key) << 1 | u64::from(tomb) | 1 << 40);
            for n in 0..1_000 {
                assert_not_a_record(state(r.next() & z), &format!("{ctx} seeded {n}"));
            }
            // And the whole call done, check not yet started: still no record.
            assert_not_a_record(state(z), &ctx);
        }
    }

    #[test]
    fn a_set_of_value_zero_cut_short_is_never_a_tombstone() {
        // The case a plain flag bit gets wrong: a set of value 0 and the
        // tombstone of the same key differ only in the kind, so a half-written
        // check word must not be able to turn one into the other.
        for key in [0, 1, 0x101B_2F50, 0xFFFF_FFFF] {
            let set = record_check(key, 0, false);
            let tomb = record_check(key, 0, true);
            // No state reachable from erased by clearing bits of `set` equals
            // `tomb` (and the reverse): each has a 0 where the other has a 1.
            assert_ne!(!set & tomb, 0);
            assert_ne!(!tomb & set, 0);
        }
    }
}
