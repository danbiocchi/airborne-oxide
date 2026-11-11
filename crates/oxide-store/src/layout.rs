// SPDX-License-Identifier: AGPL-3.0-or-later
//! On-flash layout. All integers are little-endian.
//!
//! ```text
//! sector header (16 bytes)            record (12 bytes)
//!   0  u16 magic   0x534F               0  u32 key
//!   2  u16 version 1                    4  u32 value
//!   4  u32 sequence                     8  u32 CRC-32 of bytes 0..8
//!   8  u32 CRC-32 of bytes 0..8
//!  12  u32 state: FFFFFFFF writing, A55A3CC3 valid
//! ```
//!
//! A record slot that reads all `0xFF` is free. A record whose check word
//! fails is torn: skipped and never reused.
//!
//! The check word of a record (bytes 8..12) is `crc30 | kind << 30`, where
//! `crc30` is the low 30 bits of the CRC-32 of the key and value, and `kind`
//! is 0 for a set and 1 for a tombstone (a removed key, stored with value 0).
//! Bit 31 is always 0, so a check word is never `0xFFFF_FFFF`: a record whose
//! key and value are written but whose check word is not (it is programmed
//! last, so its top byte is still `0xFF`) can never read as a valid record of
//! either kind. A 30-bit check misses a random corruption with probability
//! 2^-30.

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

/// Check word of a record: 30 CRC bits, the tombstone flag in bit 30, bit 31
/// clear.
#[must_use]
pub fn record_check(key: u32, value: u32, tomb: bool) -> u32 {
    (record_crc(key, value) & 0x3FFF_FFFF) | (u32::from(tomb) << 30)
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
    if crc == record_check(key, value, false) {
        Slot::Set(key, value)
    } else if value == 0 && crc == record_check(key, 0, true) {
        Slot::Tomb(key)
    } else {
        Slot::Torn
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
    /// Anything else.
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
        _ => Header::Bad,
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
    fn a_check_word_never_reads_as_erased() {
        for (k, v) in [(0xFFFF_FFFF, 0), (0x101B_2F50, 0), (1, 2)] {
            assert_eq!(record_check(k, v, false) >> 31, 0);
            assert_eq!(record_check(k, v, true) >> 31, 0);
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
        assert_eq!(decode_header([w[0], w[1], w[2], 0xFFFF_00FF]), Header::Bad);
        assert_eq!(
            decode_header([w[0], w[1] ^ 1, w[2], STATE_VALID]),
            Header::Bad
        );
        assert_eq!(decode_header([u32::MAX; 4]), Header::Blank);
        assert_eq!(decode_header([0; 4]), Header::Bad);
    }
}
