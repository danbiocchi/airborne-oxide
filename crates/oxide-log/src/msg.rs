// SPDX-License-Identifier: AGPL-3.0-or-later
//! ULog message encoders (little endian, no allocation).
//!
//! Every encoder writes one whole message (3-byte header, then the payload)
//! into a [`Sink`]. If the sink refuses part of it, the error is returned and
//! the sink may hold a partial message: a sink that must stay whole (the
//! [`Logger`](crate::logger::Logger) ring) rolls back by itself. The layouts
//! are those of the ULog specification as `pyulog` reads them; see
//! `docs/adr/0005-logging-and-replay.md` §2.

/// The 7 magic bytes that start a ULog file (`"ULog"` then `01 12 35`).
pub const MAGIC: [u8; 7] = [0x55, 0x4C, 0x6F, 0x67, 0x01, 0x12, 0x35];
/// The file format version PX4 writes.
pub const VERSION: u8 = 1;
/// The fixed 8-byte payload of a synchronisation message.
pub const SYNC_MAGIC: [u8; 8] = [0x2F, 0x73, 0x13, 0x20, 0x25, 0x0C, 0xBB, 0x12];
/// Bytes in the file header.
pub const FILE_HEADER_LEN: usize = 16;
/// Bytes in a message header (`u16` payload size, `u8` type).
pub const MSG_HEADER_LEN: usize = 3;
/// Largest payload a message can carry (`u16` size field).
pub const MAX_PAYLOAD: usize = u16::MAX as usize;
/// Largest `key_len` (`u8`).
pub const MAX_KEY_LEN: usize = u8::MAX as usize;

/// Message type letters.
pub mod kind {
    /// Flag bits.
    pub const FLAG_BITS: u8 = b'B';
    /// Format definition.
    pub const FORMAT: u8 = b'F';
    /// Information.
    pub const INFO: u8 = b'I';
    /// Multi information.
    pub const MULTI_INFO: u8 = b'M';
    /// Parameter.
    pub const PARAMETER: u8 = b'P';
    /// Default parameter.
    pub const DEFAULT_PARAMETER: u8 = b'Q';
    /// Subscription (add logged message).
    pub const ADD_LOGGED: u8 = b'A';
    /// Unsubscription.
    pub const REMOVE_LOGGED: u8 = b'R';
    /// Logged data.
    pub const DATA: u8 = b'D';
    /// Logged string.
    pub const LOGGED_STRING: u8 = b'L';
    /// Tagged logged string.
    pub const TAGGED_STRING: u8 = b'C';
    /// Synchronisation.
    pub const SYNC: u8 = b'S';
    /// Dropout.
    pub const DROPOUT: u8 = b'O';
}

/// Why an encoder or a record writer refused.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// The sink has no room for the bytes.
    Full,
    /// A payload over 65,535 bytes, or a key over 255.
    TooLong,
    /// A name with a character outside `a-zA-Z0-9_-/`, or an empty one.
    BadName,
    /// A log level outside 0..=7.
    BadLevel,
    /// A record whose bytes differ from the length its topic declares.
    RecordLength {
        /// Declared length.
        expected: usize,
        /// Bytes written (or that would have been, when too many).
        got: usize,
    },
}

/// Somewhere bytes go: a slice, a `Vec`, the logger's ring.
pub trait Sink {
    /// Appends all of `bytes` or fails with [`Error::Full`].
    fn put(&mut self, bytes: &[u8]) -> Result<(), Error>;
}

/// A [`Sink`] over a caller's slice.
#[derive(Debug)]
pub struct SliceSink<'a> {
    buf: &'a mut [u8],
    len: usize,
}

impl<'a> SliceSink<'a> {
    /// Starts empty over `buf`.
    pub fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, len: 0 }
    }

    /// The bytes written so far.
    pub fn written(&self) -> &[u8] {
        self.buf.get(..self.len).unwrap_or(&[])
    }
}

impl Sink for SliceSink<'_> {
    fn put(&mut self, bytes: &[u8]) -> Result<(), Error> {
        let end = self.len.checked_add(bytes.len()).ok_or(Error::Full)?;
        let dst = self.buf.get_mut(self.len..end).ok_or(Error::Full)?;
        dst.copy_from_slice(bytes);
        self.len = end;
        Ok(())
    }
}

#[cfg(any(test, feature = "std"))]
impl Sink for std::vec::Vec<u8> {
    fn put(&mut self, bytes: &[u8]) -> Result<(), Error> {
        self.extend_from_slice(bytes);
        Ok(())
    }
}

/// Writes the 3-byte message header for a payload of `len` bytes.
fn header<S: Sink + ?Sized>(sink: &mut S, len: usize, ty: u8) -> Result<(), Error> {
    let n = u16::try_from(len).map_err(|_| Error::TooLong)?;
    let b = n.to_le_bytes();
    sink.put(&[b[0], b[1], ty])
}

fn valid_name(name: &str) -> bool {
    !name.is_empty()
        && name
            .bytes()
            .all(|c| c.is_ascii_alphanumeric() || c == b'_' || c == b'-' || c == b'/')
}

/// The 16-byte file header: magic, version 1, start time in microseconds.
pub fn file_header<S: Sink + ?Sized>(sink: &mut S, start_us: u64) -> Result<(), Error> {
    sink.put(&MAGIC)?;
    sink.put(&[VERSION])?;
    sink.put(&start_us.to_le_bytes())
}

/// Flag bits `B`: 40-byte payload, appended-data offsets zero.
pub fn flag_bits<S: Sink + ?Sized>(
    sink: &mut S,
    compat: [u8; 8],
    incompat: [u8; 8],
) -> Result<(), Error> {
    header(sink, 40, kind::FLAG_BITS)?;
    sink.put(&compat)?;
    sink.put(&incompat)?;
    sink.put(&[0u8; 24])
}

/// Format `F`: the payload is `name:fields`, where `fields` is
/// `"type name;type name;..."` with its closing `;`.
pub fn format<S: Sink + ?Sized>(sink: &mut S, name: &str, fields: &str) -> Result<(), Error> {
    if !valid_name(name) {
        return Err(Error::BadName);
    }
    header(sink, name.len() + 1 + fields.len(), kind::FORMAT)?;
    sink.put(name.as_bytes())?;
    sink.put(b":")?;
    sink.put(fields.as_bytes())
}

/// The value of an information message.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum InfoValue<'a> {
    /// `char[N]`, without terminator.
    Str(&'a str),
    /// `uint32_t`.
    U32(u32),
    /// `uint64_t`.
    U64(u64),
    /// `int32_t`.
    I32(i32),
    /// `float`.
    F32(f32),
}

/// A message key: `"<type> <name>"`, built on the stack.
struct Key {
    buf: [u8; MAX_KEY_LEN],
    len: usize,
}

impl Key {
    fn new() -> Self {
        Self {
            buf: [0; MAX_KEY_LEN],
            len: 0,
        }
    }

    fn push(&mut self, bytes: &[u8]) -> Result<(), Error> {
        let end = self.len + bytes.len();
        self.buf
            .get_mut(self.len..end)
            .ok_or(Error::TooLong)?
            .copy_from_slice(bytes);
        self.len = end;
        Ok(())
    }

    fn push_decimal(&mut self, mut n: usize) -> Result<(), Error> {
        let mut digits = [0u8; 20];
        let mut i = digits.len();
        loop {
            i -= 1;
            if let Some(d) = digits.get_mut(i) {
                *d = b'0' + (n % 10) as u8;
            }
            n /= 10;
            if n == 0 {
                break;
            }
        }
        self.push(digits.get(i..).unwrap_or(&[]))
    }

    fn bytes(&self) -> &[u8] {
        self.buf.get(..self.len).unwrap_or(&[])
    }
}

/// One `key_len, key, value` body, after a message header and any leading byte.
fn keyed<S: Sink + ?Sized>(sink: &mut S, ty: u8, key: &Key, value: &[u8]) -> Result<(), Error> {
    header(sink, 1 + key.len + value.len(), ty)?;
    sink.put(&[key.len as u8])?;
    sink.put(key.bytes())?;
    sink.put(value)
}

/// Information `I`: key `"<type> <name>"` then the raw value.
pub fn info<S: Sink + ?Sized>(sink: &mut S, name: &str, value: InfoValue<'_>) -> Result<(), Error> {
    if !valid_name(name) {
        return Err(Error::BadName);
    }
    let mut key = Key::new();
    let mut scratch = [0u8; 8];
    let raw: &[u8] = match value {
        InfoValue::Str(s) => {
            key.push(b"char[")?;
            key.push_decimal(s.len())?;
            key.push(b"]")?;
            s.as_bytes()
        }
        InfoValue::U32(v) => {
            key.push(b"uint32_t")?;
            scratch
                .get_mut(..4)
                .unwrap_or(&mut [])
                .copy_from_slice(&v.to_le_bytes());
            scratch.get(..4).unwrap_or(&[])
        }
        InfoValue::U64(v) => {
            key.push(b"uint64_t")?;
            scratch = v.to_le_bytes();
            &scratch
        }
        InfoValue::I32(v) => {
            key.push(b"int32_t")?;
            scratch
                .get_mut(..4)
                .unwrap_or(&mut [])
                .copy_from_slice(&v.to_le_bytes());
            scratch.get(..4).unwrap_or(&[])
        }
        InfoValue::F32(v) => {
            key.push(b"float")?;
            scratch
                .get_mut(..4)
                .unwrap_or(&mut [])
                .copy_from_slice(&v.to_le_bytes());
            scratch.get(..4).unwrap_or(&[])
        }
    };
    key.push(b" ")?;
    key.push(name.as_bytes())?;
    keyed(sink, kind::INFO, &key, raw)
}

/// The value of a parameter message (ULog allows only these two types).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ParamValue {
    /// `int32_t`.
    I32(i32),
    /// `float`.
    F32(f32),
}

/// Parameter `P`: valid in the definitions (initial value) and in the data
/// section (the value changed).
pub fn parameter<S: Sink + ?Sized>(
    sink: &mut S,
    name: &str,
    value: ParamValue,
) -> Result<(), Error> {
    if !valid_name(name) {
        return Err(Error::BadName);
    }
    let mut key = Key::new();
    let raw = match value {
        ParamValue::I32(v) => {
            key.push(b"int32_t ")?;
            v.to_le_bytes()
        }
        ParamValue::F32(v) => {
            key.push(b"float ")?;
            v.to_le_bytes()
        }
    };
    key.push(name.as_bytes())?;
    keyed(sink, kind::PARAMETER, &key, &raw)
}

/// Add logged message `A` (a subscription): instance, message id, topic name.
pub fn add_logged<S: Sink + ?Sized>(
    sink: &mut S,
    multi_id: u8,
    msg_id: u16,
    name: &str,
) -> Result<(), Error> {
    if !valid_name(name) {
        return Err(Error::BadName);
    }
    header(sink, 3 + name.len(), kind::ADD_LOGGED)?;
    sink.put(&[multi_id])?;
    sink.put(&msg_id.to_le_bytes())?;
    sink.put(name.as_bytes())
}

/// Data `D` from ready bytes: `msg_id`, then `record` (already without
/// trailing padding). Use [`crate::record::data`] to fill a record field by
/// field and check its length.
pub fn data_raw<S: Sink + ?Sized>(sink: &mut S, msg_id: u16, record: &[u8]) -> Result<(), Error> {
    header(sink, 2 + record.len(), kind::DATA)?;
    sink.put(&msg_id.to_le_bytes())?;
    sink.put(record)
}

/// Logged string `L`: `level` 0..=7 (written as the ASCII digit), time in
/// microseconds, text without terminator.
pub fn logged_string<S: Sink + ?Sized>(
    sink: &mut S,
    level: u8,
    time_us: u64,
    text: &str,
) -> Result<(), Error> {
    if level > 7 {
        return Err(Error::BadLevel);
    }
    header(sink, 9 + text.len(), kind::LOGGED_STRING)?;
    sink.put(&[b'0' + level])?;
    sink.put(&time_us.to_le_bytes())?;
    sink.put(text.as_bytes())
}

/// Synchronisation `S`: the fixed 8-byte magic.
pub fn sync<S: Sink + ?Sized>(sink: &mut S) -> Result<(), Error> {
    header(sink, SYNC_MAGIC.len(), kind::SYNC)?;
    sink.put(&SYNC_MAGIC)
}

/// Dropout `O`: the length of the gap in milliseconds.
pub fn dropout<S: Sink + ?Sized>(sink: &mut S, duration_ms: u16) -> Result<(), Error> {
    header(sink, 2, kind::DROPOUT)?;
    sink.put(&duration_ms.to_le_bytes())
}
