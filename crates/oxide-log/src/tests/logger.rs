// SPDX-License-Identifier: AGPL-3.0-or-later
//! Step 3: the logger ring.

use crate::logger::{Class, Logger};
use crate::msg::{self, Error, SYNC_MAGIC};
use proptest::prelude::*;

/// A logged string of exactly `n` bytes on the wire (3 header + 9 + text).
fn put_len(l: &mut Logger<256>, class: Class, t: u64, n: usize) -> bool {
    l.write(class, t, |w| {
        msg::logged_string(w, 6, t, &"x".repeat(n - 12))
    })
}

fn drain_all<const C: usize>(l: &mut Logger<C>) -> Vec<u8> {
    let mut out = Vec::new();
    l.drain(|b| {
        out.extend_from_slice(b);
        b.len()
    });
    out
}

/// Splits a drained stream into `(type, payload)`, panicking on a torn message.
pub fn messages(stream: &[u8]) -> Vec<(u8, &[u8])> {
    let mut out = Vec::new();
    let mut at = 0;
    while at < stream.len() {
        assert!(at + 3 <= stream.len(), "torn header at {at}");
        let n = u16::from_le_bytes([stream[at], stream[at + 1]]) as usize;
        assert!(at + 3 + n <= stream.len(), "torn message at {at}");
        out.push((stream[at + 2], &stream[at + 3..at + 3 + n]));
        at += 3 + n;
    }
    out
}

#[test]
fn a_message_larger_than_the_ring_is_dropped_and_counted_and_leaves_nothing() {
    let mut l = Logger::<256>::new(0);
    assert!(!put_len(&mut l, Class::Must, 0, 257));
    let s = l.stats();
    assert_eq!(
        (s.written, s.dropped_must, s.dropped_stream, s.bytes),
        (0, 1, 0, 0)
    );
    assert!(drain_all(&mut l).is_empty());
    assert_eq!(l.len(), 0);
}

#[test]
fn the_largest_message_is_the_whole_ring_for_must_and_less_the_reserve_for_stream() {
    // Time 0 and nothing dropped: no sync and no dropout in front.
    let mut l = Logger::<256>::new(0);
    assert!(put_len(&mut l, Class::Must, 0, 256));
    assert_eq!(l.stats().high_water, 256);
    assert_eq!(drain_all(&mut l).len(), 256);
    // The same size with a reserve of 16: Stream stops at 240.
    let mut l = Logger::<256>::new(16);
    assert!(put_len(&mut l, Class::Stream, 0, 240));
    let mut l = Logger::<256>::new(16);
    assert!(!put_len(&mut l, Class::Stream, 0, 241));
    assert_eq!(l.stats().dropped_stream, 1);
}

#[test]
fn the_reserve_keeps_room_for_a_must_message_when_stream_has_filled_the_rest() {
    let mut l = Logger::<256>::new(100);
    let mut n = 0;
    while put_len(&mut l, Class::Stream, 10, 20) {
        n += 1;
    }
    // 156 bytes may be used by Stream: seven messages of 20 bytes.
    assert_eq!(n, 7);
    assert_eq!(l.len(), 140);
    // Stream is refused; a Must message of 100 bytes still fits. The drops so
    // far owe a dropout (5 bytes) in front of it, so it takes 105.
    assert!(!put_len(&mut l, Class::Stream, 11, 20));
    assert!(put_len(&mut l, Class::Must, 12, 100));
    assert_eq!(l.len(), 245);
    // 11 bytes are left: not even the smallest logged string (12) fits.
    assert!(!put_len(&mut l, Class::Must, 13, 12));
    let s = l.stats();
    assert_eq!((s.written, s.dropped_stream, s.dropped_must), (7 + 1, 2, 1));
}

#[test]
fn after_drops_the_next_message_is_preceded_by_exactly_one_dropout_with_the_gap() {
    let mut l = Logger::<256>::new(0);
    assert!(put_len(&mut l, Class::Must, 100, 200));
    // Full: two drops, the first at 1_000_000 us.
    assert!(!put_len(&mut l, Class::Stream, 1_000_000, 100));
    assert!(!put_len(&mut l, Class::Stream, 1_200_000, 100));
    assert_eq!(drain_all(&mut l).len(), 200);
    assert!(put_len(&mut l, Class::Stream, 1_350_000, 30));
    assert!(put_len(&mut l, Class::Stream, 1_360_000, 30));
    let out = drain_all(&mut l);
    let m = messages(&out);
    let types: Vec<u8> = m.iter().map(|x| x.0).collect();
    // A sync is due as well (more than 500 ms since the start at 100 us).
    assert_eq!(types, b"OSLL");
    assert_eq!(m[0].1, 350u16.to_le_bytes());
    assert_eq!(l.stats().dropouts, 1);
    // A new run of drops gives a new dropout later, with its own duration.
    assert!(put_len(&mut l, Class::Stream, 1_361_000, 200));
    assert!(!put_len(&mut l, Class::Stream, 1_362_000, 100));
    drain_all(&mut l);
    assert!(put_len(&mut l, Class::Stream, 1_362_900, 20));
    let out = drain_all(&mut l);
    assert_eq!(
        messages(&out)[0],
        (b'O', &1u16.to_le_bytes()[..]),
        "at least 1 ms"
    );
    assert_eq!(l.stats().dropouts, 2);
}

#[test]
fn the_dropout_duration_saturates_at_65535_ms() {
    let mut l = Logger::<256>::new(0);
    assert!(!put_len(&mut l, Class::Stream, 1_000, 300));
    assert!(put_len(&mut l, Class::Stream, 1_000 + 100_000_000, 20));
    let out = drain_all(&mut l);
    let m = messages(&out);
    assert_eq!(m[0].0, b'O');
    assert_eq!(m[0].1, 65_535u16.to_le_bytes());
}

#[test]
fn a_dropout_and_its_message_are_written_together_or_not_at_all() {
    let mut l = Logger::<64>::new(0);
    assert!(l.write(Class::Must, 0, |w| msg::logged_string(
        w,
        6,
        0,
        &"x".repeat(40)
    ))); // 52 bytes
    // A drop (60 bytes cannot fit with 52 used).
    assert!(!l.write(Class::Stream, 10, |w| msg::logged_string(
        w,
        6,
        0,
        &"y".repeat(48)
    )));
    // 12 bytes are free: the message alone (12) would fit, the dropout (5) in front of it does not.
    assert!(!l.write(Class::Stream, 20, |w| msg::logged_string(w, 6, 0, "")));
    assert_eq!(l.len(), 52, "nothing of the refused message stayed");
    assert_eq!(l.stats().dropouts, 0);
    drain_all(&mut l);
    // Room again: the pending dropout comes out in front, and counts the gap from the first drop.
    assert!(l.write(Class::Stream, 5_010, |w| msg::logged_string(w, 6, 0, "ok")));
    let out = drain_all(&mut l);
    let m = messages(&out);
    assert_eq!(m[0], (b'O', &5u16.to_le_bytes()[..]));
}

#[test]
fn a_builder_that_fails_leaves_nothing_and_is_counted_dropped() {
    let mut l = Logger::<256>::new(0);
    assert!(!l.write(Class::Stream, 0, |w| {
        msg::logged_string(w, 6, 0, "half")?;
        Err(Error::TooLong)
    }));
    assert_eq!(l.len(), 0);
    assert!(drain_all(&mut l).is_empty());
    assert_eq!(l.stats().dropped_stream, 1);
    // A builder that writes nothing is not a message either.
    assert!(!l.write(Class::Stream, 1, |_| Ok(())));
    assert_eq!(l.stats().written, 0);
}

#[test]
fn syncs_come_every_500_ms_and_precede_the_message() {
    let mut l = Logger::<256>::new(0);
    let times = [0u64, 499_999, 500_000, 999_999, 1_000_000, 11_000_000];
    for t in times {
        assert!(put_len(&mut l, Class::Stream, t, 20));
        let out = drain_all(&mut l);
        let m = messages(&out);
        let want_sync = matches!(t, 500_000 | 1_000_000 | 11_000_000);
        assert_eq!(m.len(), if want_sync { 2 } else { 1 }, "t={t}");
        if want_sync {
            assert_eq!(m[0], (b'S', &SYNC_MAGIC[..]));
        }
        assert_eq!(m.last().unwrap().0, b'L');
    }
    assert_eq!(l.stats().syncs, 3);
}

#[test]
fn drain_may_stop_inside_a_message_and_resume_where_it_stopped() {
    let mut l = Logger::<64>::new(0);
    let mut all = Vec::new();
    let mut taken = Vec::new();
    for i in 0..40u64 {
        let t = i * 10_000;
        if l.write(Class::Stream, t, |w| {
            msg::logged_string(w, 6, t, "abcdefgh")
        }) {
            // the same message, as the encoder writes it alone
            let mut v = Vec::new();
            msg::logged_string(&mut v, 6, t, "abcdefgh").unwrap();
            all.extend_from_slice(&v);
        }
        // Take 7 bytes at a time: the ring wraps around many times.
        l.drain(|b| {
            let k = b.len().min(7);
            taken.extend_from_slice(&b[..k]);
            k
        });
    }
    drain_all(&mut l).iter().for_each(|b| taken.push(*b));
    // Syncs are in `taken` as well: strip them to compare with what was written.
    let m = messages(&taken);
    let mut rebuilt = Vec::new();
    for (ty, p) in m {
        if ty == b'L' {
            rebuilt.extend_from_slice(&(p.len() as u16).to_le_bytes());
            rebuilt.push(ty);
            rebuilt.extend_from_slice(p);
        }
    }
    assert_eq!(rebuilt, all);
    assert!(l.stats().high_water <= 64);
}

#[test]
fn a_zero_sized_ring_drops_everything_without_failing() {
    let mut l = Logger::<0>::new(0);
    assert!(!l.write(Class::Must, 0, |w| msg::sync(w)));
    assert_eq!(l.stats().dropped_must, 1);
    assert_eq!(l.drain(|_| 0), 0);
}

#[derive(Debug, Clone)]
struct Op {
    must: bool,
    dt_us: u64,
    kind: u8,
    size: usize,
    drain: Option<usize>,
}

fn op() -> impl Strategy<Value = Op> {
    (
        any::<bool>(),
        prop_oneof![0u64..2_000, 0u64..700_000, 0u64..90_000_000],
        0u8..3,
        prop_oneof![0usize..40, 0usize..300, 0usize..2_000],
        prop_oneof![
            Just(None),
            (0usize..64).prop_map(Some),
            (0usize..1_000).prop_map(Some)
        ],
    )
        .prop_map(|(must, dt_us, kind, size, drain)| Op {
            must,
            dt_us,
            kind,
            size,
            drain,
        })
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(400))]

    /// The drained bytes are always whole messages; the count of logged
    /// strings and data messages equals `stats().written`; and written plus
    /// dropped equals the attempts.
    #[test]
    fn the_stream_is_whole_messages_and_the_counts_add_up(
        ops in proptest::collection::vec(op(), 1..120),
        reserve in 0usize..1_200,
    ) {
        let mut l = Logger::<1024>::new(reserve);
        let mut stream = Vec::new();
        let mut t = 0u64;
        let mut attempts = 0u32;
        let mut accepted = 0u32;
        for o in &ops {
            t += o.dt_us;
            let class = if o.must { Class::Must } else { Class::Stream };
            attempts += 1;
            let ok = l.write(class, t, |w| match o.kind {
                0 => msg::logged_string(w, 6, t, &"x".repeat(o.size)),
                1 => msg::data_raw(w, 0, &vec![0xA5; o.size]),
                _ => msg::parameter(w, "P", msg::ParamValue::I32(o.size as i32)),
            });
            // An empty logged string still has 12 bytes: every kind writes something.
            accepted += u32::from(ok);
            if let Some(n) = o.drain {
                l.drain(|b| { let k = b.len().min(n); stream.extend_from_slice(&b[..k]); k });
            }
            prop_assert!(l.len() <= 1024);
        }
        while !l.is_empty() {
            l.drain(|b| { stream.extend_from_slice(b); b.len() });
        }
        let s = l.stats();
        prop_assert_eq!(s.written, accepted);
        prop_assert_eq!(s.written + s.dropped_stream + s.dropped_must, attempts);
        prop_assert!(s.high_water <= 1024);
        let mut counted = 0u32;
        let mut at = 0;
        while at < stream.len() {
            prop_assert!(at + 3 <= stream.len());
            let n = u16::from_le_bytes([stream[at], stream[at + 1]]) as usize;
            prop_assert!(at + 3 + n <= stream.len());
            let ty = stream[at + 2];
            let p = &stream[at + 3..at + 3 + n];
            match ty {
                b'L' | b'D' | b'P' => counted += 1,
                b'S' => prop_assert_eq!(p, &SYNC_MAGIC[..]),
                b'O' => {
                    prop_assert_eq!(p.len(), 2);
                    prop_assert!(u16::from_le_bytes([p[0], p[1]]) >= 1);
                }
                _ => prop_assert!(false, "unexpected type {}", ty),
            }
            at += 3 + n;
        }
        prop_assert_eq!(counted, s.written);
        prop_assert_eq!(stream.len() as u64, s.bytes);
    }
}
