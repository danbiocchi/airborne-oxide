// SPDX-License-Identifier: AGPL-3.0-or-later
//! Trace hash for the determinism check (DESIGN §5.2 item 6): FNV-1a over
//! the exact bit patterns of the state at every step. Stable across Rust
//! versions and platforms, unlike `std`'s default hasher.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TraceHash(u64);

impl Default for TraceHash {
    fn default() -> Self {
        Self(0xcbf2_9ce4_8422_2325)
    }
}

impl TraceHash {
    const PRIME: u64 = 0x0000_0100_0000_01b3;

    pub fn write_bytes(&mut self, bytes: &[u8]) {
        for b in bytes {
            self.0 ^= u64::from(*b);
            self.0 = self.0.wrapping_mul(Self::PRIME);
        }
    }
    pub fn write_u64(&mut self, v: u64) {
        self.write_bytes(&v.to_le_bytes());
    }
    pub fn write_f64(&mut self, v: f64) {
        self.write_u64(v.to_bits());
    }
    pub fn write_f32(&mut self, v: f32) {
        self.write_bytes(&v.to_bits().to_le_bytes());
    }
    pub fn value(&self) -> u64 {
        self.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn matches_fnv1a_reference() {
        // FNV-1a 64 of "a" is 0xaf63dc4c8601ec8c.
        let mut h = TraceHash::default();
        h.write_bytes(b"a");
        assert_eq!(h.value(), 0xaf63_dc4c_8601_ec8c);
    }
}
