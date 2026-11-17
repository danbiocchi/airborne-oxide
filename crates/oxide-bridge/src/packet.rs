// SPDX-License-Identifier: AGPL-3.0-or-later
//! The motor packet: one UDP datagram per frame from the autopilot's
//! simulator build, little-endian, no padding.
//!
//! `u16 magic`, `u16 frame_rate`, `u32 frame_count`, then `u16 pwm[16]` when
//! the magic is 18458 (40 bytes) or `u16 pwm[32]` when it is 29569 (72
//! bytes). Pulse widths are microseconds; a disarmed motor output is 1000 and
//! an unused channel 0. `frame_rate` is the simulator's echo of our own rate
//! and is never used as the step.

use core::fmt;

/// Magic of the packet with 16 channels.
pub const MAGIC_16: u16 = 18458;
/// Magic of the packet with 32 channels.
pub const MAGIC_32: u16 = 29569;
/// Length of the header, bytes.
pub const HEADER_LEN: usize = 8;
/// Length of the packet with 16 channels, bytes.
pub const LEN_16: usize = HEADER_LEN + 2 * 16;
/// Length of the packet with 32 channels, bytes.
pub const LEN_32: usize = HEADER_LEN + 2 * 32;

/// A parsed motor packet.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ServoPacket {
    /// The simulator's echo of our frame rate, Hz. Informational only.
    pub frame_rate: u16,
    /// Counts the frames; a repeat of the same count is a resend.
    pub frame_count: u32,
    /// Pulse widths, microseconds; only the first `channels` are meaningful,
    /// the rest are 0.
    pub pwm: [u16; 32],
    /// 16 or 32.
    pub channels: usize,
}

/// Why a datagram is not a motor packet.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PacketError {
    /// Fewer than two bytes: there is no magic to read.
    TooShort { len: usize },
    /// The magic is neither of the two known values.
    UnknownMagic { magic: u16, len: usize },
    /// The length is not the one this magic has.
    WrongLength {
        magic: u16,
        expected: usize,
        found: usize,
    },
}

impl fmt::Display for PacketError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match *self {
            Self::TooShort { len } => {
                write!(
                    f,
                    "a datagram of {len} byte(s) is too short to hold a magic"
                )
            }
            Self::UnknownMagic { magic, len } => write!(
                f,
                "unknown packet magic {magic} (known: {MAGIC_16} and {MAGIC_32}) in a datagram of {len} bytes"
            ),
            Self::WrongLength {
                magic,
                expected,
                found,
            } => write!(
                f,
                "a packet with magic {magic} is {expected} bytes long, this one has {found}"
            ),
        }
    }
}

impl std::error::Error for PacketError {}

impl ServoPacket {
    /// Parses one datagram.
    pub fn parse(bytes: &[u8]) -> Result<Self, PacketError> {
        let len = bytes.len();
        let magic = match bytes.first_chunk::<2>() {
            Some(m) => u16::from_le_bytes(*m),
            None => return Err(PacketError::TooShort { len }),
        };
        let channels = match magic {
            MAGIC_16 => 16,
            MAGIC_32 => 32,
            _ => return Err(PacketError::UnknownMagic { magic, len }),
        };
        let expected = HEADER_LEN + 2 * channels;
        if len != expected {
            return Err(PacketError::WrongLength {
                magic,
                expected,
                found: len,
            });
        }
        let (head, body) = bytes.split_at(HEADER_LEN);
        let mut frame_rate = [0u8; 2];
        let mut frame_count = [0u8; 4];
        for (d, s) in frame_rate.iter_mut().zip(head.iter().skip(2)) {
            *d = *s;
        }
        for (d, s) in frame_count.iter_mut().zip(head.iter().skip(4)) {
            *d = *s;
        }
        let mut pwm = [0u16; 32];
        for (d, s) in pwm.iter_mut().zip(body.chunks_exact(2)) {
            *d = u16::from_le_bytes([
                s.first().copied().unwrap_or(0),
                s.get(1).copied().unwrap_or(0),
            ]);
        }
        Ok(Self {
            frame_rate: u16::from_le_bytes(frame_rate),
            frame_count: u32::from_le_bytes(frame_count),
            pwm,
            channels,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A packet of the short kind, built by hand, byte by byte.
    fn short_bytes() -> Vec<u8> {
        let mut b = vec![
            0x1A, 0x48, // magic 18458 = 0x481A
            0xE8, 0x03, // frame_rate 1000
            0x78, 0x56, 0x34, 0x12, // frame_count 0x12345678
        ];
        for i in 0..16u16 {
            b.extend_from_slice(&(1000 + i).to_le_bytes());
        }
        b
    }

    #[test]
    fn a_packet_of_16_channels_parses_from_literal_bytes() {
        let b = short_bytes();
        assert_eq!(b.len(), 40);
        let p = ServoPacket::parse(&b).expect("parses");
        assert_eq!(p.frame_rate, 1000);
        assert_eq!(p.frame_count, 0x1234_5678);
        assert_eq!(p.channels, 16);
        for i in 0..16 {
            assert_eq!(p.pwm[i], 1000 + i as u16);
        }
        assert!(p.pwm[16..].iter().all(|&v| v == 0), "the rest is zero");
    }

    #[test]
    fn a_packet_of_32_channels_parses_from_literal_bytes() {
        let mut b = vec![
            0x81, 0x73, // magic 29569 = 0x7381
            0x20, 0x4E, // frame_rate 20000
            0x01, 0x00, 0x00, 0x00, // frame_count 1
        ];
        for i in 0..32u16 {
            b.extend_from_slice(&(2000 - i).to_le_bytes());
        }
        assert_eq!(b.len(), 72);
        let p = ServoPacket::parse(&b).expect("parses");
        assert_eq!(p.frame_rate, 20_000);
        assert_eq!(p.frame_count, 1);
        assert_eq!(p.channels, 32);
        for i in 0..32 {
            assert_eq!(p.pwm[i], 2000 - i as u16);
        }
    }

    #[test]
    fn every_wrong_length_from_0_to_80_is_refused_by_name() {
        for magic in [MAGIC_16, MAGIC_32] {
            let right = if magic == MAGIC_16 { 40 } else { 72 };
            for len in 0..=80usize {
                let mut b = vec![0u8; len];
                if len >= 2 {
                    b[..2].copy_from_slice(&magic.to_le_bytes());
                }
                let r = ServoPacket::parse(&b);
                if len == right {
                    assert!(r.is_ok(), "magic {magic}, length {len}");
                } else if len < 2 {
                    assert_eq!(r, Err(PacketError::TooShort { len }), "length {len}");
                } else {
                    assert_eq!(
                        r,
                        Err(PacketError::WrongLength {
                            magic,
                            expected: right,
                            found: len
                        }),
                        "magic {magic}, length {len}"
                    );
                }
            }
        }
    }

    #[test]
    fn an_unknown_magic_is_refused_with_its_value() {
        let mut b = short_bytes();
        b[0] = 0x34;
        b[1] = 0x12;
        let e = ServoPacket::parse(&b).expect_err("refused");
        assert_eq!(
            e,
            PacketError::UnknownMagic {
                magic: 0x1234,
                len: 40
            }
        );
        assert!(e.to_string().contains("4660"), "{e}");
        // The unknown magic wins over a wrong length.
        assert!(matches!(
            ServoPacket::parse(&[0x34, 0x12, 0, 0, 0]),
            Err(PacketError::UnknownMagic {
                magic: 0x1234,
                len: 5
            })
        ));
    }

    #[test]
    fn the_messages_say_what_is_wrong() {
        let e = ServoPacket::parse(&[0x1A, 0x48, 0, 0]).expect_err("short");
        assert!(
            e.to_string().contains("18458") && e.to_string().contains("40"),
            "{e}"
        );
        let e = ServoPacket::parse(&[1]).expect_err("short");
        assert!(e.to_string().contains("1 byte"), "{e}");
    }
}
