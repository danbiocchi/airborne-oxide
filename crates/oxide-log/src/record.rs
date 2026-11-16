// SPDX-License-Identifier: AGPL-3.0-or-later
//! Topics and the record writer.
//!
//! A [`Topic`] is a constant: the name, the ULog format string written by hand
//! in PX4's field order, and the length of a data record. A data record is the
//! format's size **without trailing padding** (`pyulog` rejects a record that
//! includes it; ADR-0005 §2). [`RecordWriter`] fills a record field by field
//! and refuses a record of any other length. With the `std` feature,
//! [`crate::format::check_topic`] checks the constant against its own string.

use crate::msg::{self, Error, Sink};

/// A logged topic.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Topic {
    /// Topic name (`vehicle_attitude`).
    pub name: &'static str,
    /// The fields in ULog format: `"uint64_t timestamp;float[4] q;..."`, with
    /// the closing `;` and any `_padding` fields, in PX4's order.
    pub format: &'static str,
    /// Bytes in a data record: the size of `format` without trailing padding.
    pub record_len: usize,
}

impl Topic {
    /// Writes this topic's format message.
    pub fn write_format<S: Sink + ?Sized>(&self, sink: &mut S) -> Result<(), Error> {
        msg::format(sink, self.name, self.format)
    }

    /// Writes the subscription of this topic as instance `multi_id` under `msg_id`.
    pub fn add_logged<S: Sink + ?Sized>(
        &self,
        sink: &mut S,
        multi_id: u8,
        msg_id: u16,
    ) -> Result<(), Error> {
        msg::add_logged(sink, multi_id, msg_id, self.name)
    }
}

/// Writes a data message for `topic`: the header and `msg_id`, then the record
/// that `fill` writes. Returns [`Error::RecordLength`] if `fill` did not write
/// exactly `topic.record_len` bytes (a short record leaves a torn message in
/// the sink: only a sink that rolls back, such as the logger's, may be used).
pub fn data<S: Sink + ?Sized>(
    sink: &mut S,
    msg_id: u16,
    topic: &Topic,
    fill: impl FnOnce(&mut RecordWriter<'_, S>),
) -> Result<(), Error> {
    let payload = topic.record_len.checked_add(2).ok_or(Error::TooLong)?;
    let n = u16::try_from(payload).map_err(|_| Error::TooLong)?;
    let b = n.to_le_bytes();
    sink.put(&[b[0], b[1], msg::kind::DATA])?;
    sink.put(&msg_id.to_le_bytes())?;
    let mut w = RecordWriter::new(sink, topic.record_len);
    fill(&mut w);
    w.finish()
}

/// Fills one record. Each call appends a field; the first error (a full sink,
/// too many bytes) is kept and later calls do nothing. [`finish`](Self::finish)
/// reports it, or a record shorter than declared.
pub struct RecordWriter<'a, S: Sink + ?Sized> {
    sink: &'a mut S,
    expected: usize,
    written: usize,
    error: Option<Error>,
}

impl<'a, S: Sink + ?Sized> RecordWriter<'a, S> {
    /// A writer for a record of `expected` bytes.
    pub fn new(sink: &'a mut S, expected: usize) -> Self {
        Self {
            sink,
            expected,
            written: 0,
            error: None,
        }
    }

    /// Bytes written so far.
    pub fn len(&self) -> usize {
        self.written
    }

    /// True before the first byte.
    pub fn is_empty(&self) -> bool {
        self.written == 0
    }

    fn put(&mut self, bytes: &[u8]) -> &mut Self {
        if self.error.is_some() {
            return self;
        }
        let total = self.written + bytes.len();
        if total > self.expected {
            self.error = Some(Error::RecordLength {
                expected: self.expected,
                got: total,
            });
        } else if let Err(e) = self.sink.put(bytes) {
            self.error = Some(e);
        } else {
            self.written = total;
        }
        self
    }

    /// `uint8_t`.
    pub fn u8(&mut self, v: u8) -> &mut Self {
        self.put(&[v])
    }
    /// `bool` (one byte, 0 or 1).
    pub fn bool(&mut self, v: bool) -> &mut Self {
        self.put(&[u8::from(v)])
    }
    /// `int8_t`.
    pub fn i8(&mut self, v: i8) -> &mut Self {
        self.put(&v.to_le_bytes())
    }
    /// `uint16_t`.
    pub fn u16(&mut self, v: u16) -> &mut Self {
        self.put(&v.to_le_bytes())
    }
    /// `int16_t`.
    pub fn i16(&mut self, v: i16) -> &mut Self {
        self.put(&v.to_le_bytes())
    }
    /// `uint32_t`.
    pub fn u32(&mut self, v: u32) -> &mut Self {
        self.put(&v.to_le_bytes())
    }
    /// `int32_t`.
    pub fn i32(&mut self, v: i32) -> &mut Self {
        self.put(&v.to_le_bytes())
    }
    /// `uint64_t`.
    pub fn u64(&mut self, v: u64) -> &mut Self {
        self.put(&v.to_le_bytes())
    }
    /// `int64_t`.
    pub fn i64(&mut self, v: i64) -> &mut Self {
        self.put(&v.to_le_bytes())
    }
    /// `float`, as its exact bits (a NaN payload and a negative zero survive).
    pub fn f32(&mut self, v: f32) -> &mut Self {
        self.put(&v.to_bits().to_le_bytes())
    }
    /// `double`, as its exact bits.
    pub fn f64(&mut self, v: f64) -> &mut Self {
        self.put(&v.to_bits().to_le_bytes())
    }
    /// `float[N]`: the elements in order.
    pub fn f32_array(&mut self, v: &[f32]) -> &mut Self {
        for x in v {
            self.f32(*x);
        }
        self
    }
    /// Raw bytes (a `uint8_t[N]` or any pre-encoded field).
    pub fn bytes(&mut self, v: &[u8]) -> &mut Self {
        self.put(v)
    }
    /// `char[n]` from `s`: zero-filled, and cut to `n` bytes if longer.
    pub fn char_array(&mut self, n: usize, s: &str) -> &mut Self {
        let text = s.as_bytes();
        let take = text.len().min(n);
        self.put(text.get(..take).unwrap_or(&[]));
        self.pad(n - take)
    }
    /// `n` zero bytes of padding that is not trailing (the trailing kind is
    /// never written).
    pub fn pad(&mut self, n: usize) -> &mut Self {
        let zeros = [0u8; 16];
        let mut left = n;
        while left > 0 && self.error.is_none() {
            let k = left.min(zeros.len());
            self.put(zeros.get(..k).unwrap_or(&[]));
            left -= k;
        }
        self
    }

    /// The first error, or a length error if fewer than the declared bytes
    /// were written.
    pub fn finish(&self) -> Result<(), Error> {
        match self.error {
            Some(e) => Err(e),
            None if self.written != self.expected => Err(Error::RecordLength {
                expected: self.expected,
                got: self.written,
            }),
            None => Ok(()),
        }
    }
}
