// SPDX-License-Identifier: AGPL-3.0-or-later
//! CRC-32 (IEEE 802.3: reflected, polynomial 0xEDB88320, init and final XOR
//! 0xFFFFFFFF), computed bit by bit with no table.

/// CRC-32 of `data`.
#[must_use]
pub fn crc32(data: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFF_u32;
    for &byte in data {
        crc ^= u32::from(byte);
        for _ in 0..8 {
            let lsb = crc & 1;
            crc >>= 1;
            if lsb != 0 {
                crc ^= 0xEDB8_8320;
            }
        }
    }
    !crc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn matches_the_standard_check_value() {
        assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
        assert_eq!(crc32(b""), 0);
    }
}
