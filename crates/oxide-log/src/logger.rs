// SPDX-License-Identifier: AGPL-3.0-or-later
//! The logger: a ring of whole messages that never blocks (ADR-0005 §7).
//!
//! [`Logger::write`] builds one message directly in the ring's free space and
//! commits it only if it is complete and fits; otherwise nothing of it stays,
//! it is counted as dropped, and the time of the first drop of the run is
//! remembered. The next message that fits is preceded by a dropout message `O`
//! with the length of the gap, and by a synchronisation message `S` when 500 ms
//! have passed since the last one: all in the same all-or-nothing step.
//! [`Logger::drain`] hands the oldest bytes to storage, which may take only
//! part of them. Bounded work per call, no recursion, no allocation.

use crate::msg::{self, Error, Sink};

/// Microseconds between synchronisation messages (PX4's 500 ms).
pub const SYNC_INTERVAL_US: u64 = 500_000;

/// What a message is for, which decides who may use the reserve.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Class {
    /// Streamed samples: stop short of the reserve.
    Stream,
    /// Events, parameters, status, definitions: may use the whole ring.
    Must,
}

/// Counters, all since [`Logger::new`].
///
/// The `u32` counters (`written`, `dropped_stream`, `dropped_must`,
/// `dropouts`, `syncs`) stop at their maximum: they do not wrap, so after
/// that a reader sees a constant, and "written + dropped = attempted" no
/// longer holds. At the message rates of the simulator's flights that takes
/// about 63 days at the normal level (785 messages a second), about 12 days
/// at the replay level on the F405 profile (4096 a second) and about 3.8
/// days at the replay level on the H743 profile (13.1 thousand a second).
/// A stalled sink counts failed attempts faster still (2.6 thousand a second
/// in a 20 s stall with a 4 kB ring: about 19 days). `bytes` is a `u64`.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Stats {
    /// Messages committed by [`Logger::write`] (dropouts and syncs the logger
    /// adds are not counted here).
    pub written: u32,
    /// `Stream` messages dropped (too big, or into the reserve, or the builder failed).
    pub dropped_stream: u32,
    /// `Must` write attempts that failed. This counts attempts, not distinct
    /// lost messages: the recorder tries a status record again on the next
    /// tick when it did not fit, and each failed try counts, and an event's
    /// text and its data record count separately.
    pub dropped_must: u32,
    /// Bytes committed to the ring, prefixes included.
    pub bytes: u64,
    /// The most bytes the ring has held at once.
    pub high_water: usize,
    /// Dropout messages written.
    pub dropouts: u32,
    /// Synchronisation messages written.
    pub syncs: u32,
}

/// A fixed ring of `CAP` bytes holding whole ULog messages.
#[derive(Debug)]
pub struct Logger<const CAP: usize> {
    buf: [u8; CAP],
    head: usize,
    len: usize,
    reserve: usize,
    started: bool,
    last_sync_us: u64,
    gap_start_us: Option<u64>,
    stats: Stats,
}

/// The window of free ring space one message is built in. It is a [`Sink`]:
/// bytes that do not fit fail with [`Error::Full`] and the message is dropped.
#[derive(Debug)]
pub struct MessageWriter<'a> {
    ring: &'a mut [u8],
    start: usize,
    limit: usize,
    pos: usize,
    overflow: bool,
}

impl MessageWriter<'_> {
    /// Bytes written into this message so far.
    pub fn len(&self) -> usize {
        self.pos
    }

    /// True before the first byte.
    pub fn is_empty(&self) -> bool {
        self.pos == 0
    }
}

impl Sink for MessageWriter<'_> {
    fn put(&mut self, bytes: &[u8]) -> Result<(), Error> {
        let end = self.pos.checked_add(bytes.len()).ok_or(Error::Full)?;
        if self.overflow || end > self.limit {
            self.overflow = true;
            return Err(Error::Full);
        }
        let cap = self.ring.len();
        let mut rest = bytes;
        let mut at = self
            .start
            .checked_add(self.pos)
            .and_then(|p| p.checked_rem(cap))
            .ok_or(Error::Full)?;
        while !rest.is_empty() {
            let n = rest.len().min(cap - at);
            let (now, later) = rest.split_at(n);
            let dst = self.ring.get_mut(at..at + n).ok_or(Error::Full)?;
            dst.copy_from_slice(now);
            rest = later;
            at = 0;
        }
        self.pos = end;
        Ok(())
    }
}

impl<const CAP: usize> Logger<CAP> {
    /// An empty ring. `reserve` bytes are kept for [`Class::Must`] messages.
    pub const fn new(reserve: usize) -> Self {
        Self {
            buf: [0; CAP],
            head: 0,
            len: 0,
            reserve,
            started: false,
            last_sync_us: 0,
            gap_start_us: None,
            stats: Stats {
                written: 0,
                dropped_stream: 0,
                dropped_must: 0,
                bytes: 0,
                high_water: 0,
                dropouts: 0,
                syncs: 0,
            },
        }
    }

    /// Bytes waiting to be drained.
    pub fn len(&self) -> usize {
        self.len
    }

    /// True when nothing waits.
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// The counters.
    pub fn stats(&self) -> Stats {
        self.stats
    }

    /// Builds one message in place. It is in the ring whole, or not at all;
    /// the result says which. `now_us` is the log clock: it dates gaps and
    /// decides when a synchronisation message is due (the first call starts
    /// that clock). A message of up to `CAP` bytes is accepted for
    /// [`Class::Must`] (`CAP - reserve` for [`Class::Stream`]), counting the
    /// dropout and synchronisation messages put in front of it. `build` must
    /// write something.
    pub fn write(
        &mut self,
        class: Class,
        now_us: u64,
        build: impl FnOnce(&mut MessageWriter<'_>) -> Result<(), Error>,
    ) -> bool {
        if !self.started {
            self.started = true;
            self.last_sync_us = now_us;
        }
        let free = CAP - self.len;
        let limit = match class {
            Class::Must => free,
            Class::Stream => free.saturating_sub(self.reserve),
        };
        let phys = self
            .head
            .checked_add(self.len)
            .and_then(|p| p.checked_rem(CAP))
            .unwrap_or(0);
        let mut w = MessageWriter {
            ring: &mut self.buf,
            start: phys,
            limit,
            pos: 0,
            overflow: false,
        };
        let mut ok = true;
        let mut wrote_dropout = false;
        if let Some(gap) = self.gap_start_us {
            let ms = (now_us.saturating_sub(gap) / 1_000).clamp(1, u64::from(u16::MAX));
            ok = msg::dropout(&mut w, ms as u16).is_ok();
            wrote_dropout = ok;
        }
        let mut wrote_sync = false;
        // A log clock that stepped back by more than the interval would
        // otherwise wait for the old time to come round again.
        let sync_due = now_us.saturating_sub(self.last_sync_us) >= SYNC_INTERVAL_US
            || self.last_sync_us > now_us.saturating_add(SYNC_INTERVAL_US);
        if ok && sync_due {
            ok = msg::sync(&mut w).is_ok();
            wrote_sync = ok;
        }
        let prefix = w.pos;
        ok = ok && build(&mut w).is_ok() && !w.overflow && w.pos > prefix;
        let used = w.pos;
        if ok {
            self.len += used;
            self.stats.written = self.stats.written.saturating_add(1);
            self.stats.bytes += used as u64;
            self.stats.high_water = self.stats.high_water.max(self.len);
            if wrote_dropout {
                self.gap_start_us = None;
                self.stats.dropouts = self.stats.dropouts.saturating_add(1);
            }
            if wrote_sync {
                self.last_sync_us = now_us;
                self.stats.syncs = self.stats.syncs.saturating_add(1);
            }
        } else {
            match class {
                Class::Stream => {
                    self.stats.dropped_stream = self.stats.dropped_stream.saturating_add(1);
                }
                Class::Must => self.stats.dropped_must = self.stats.dropped_must.saturating_add(1),
            }
            self.gap_start_us.get_or_insert(now_us);
        }
        ok
    }

    /// Hands the oldest bytes to `sink`, at most twice (the ring may wrap),
    /// and returns how many it took in all. `sink` returns how many bytes of
    /// its argument it took; it may take part of a message, and the next call
    /// continues where it stopped.
    pub fn drain(&mut self, mut sink: impl FnMut(&[u8]) -> usize) -> usize {
        let mut total = 0;
        for _ in 0..2 {
            if self.len == 0 {
                break;
            }
            let n = (CAP - self.head).min(self.len);
            let chunk = self.buf.get(self.head..self.head + n).unwrap_or(&[]);
            let took = sink(chunk).min(n);
            self.head = (self.head + took) % CAP.max(1);
            self.len -= took;
            total += took;
            if took < n {
                break;
            }
        }
        total
    }
}
