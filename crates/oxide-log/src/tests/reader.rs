// SPDX-License-Identifier: AGPL-3.0-or-later
//! Step 4: the reader (std).

use crate::logger::{Class, Logger};
use crate::msg::{self, InfoValue, ParamValue};
use crate::reader::{Item, ReadErrorKind, UlogFile};
use crate::record::{self, Topic};
use proptest::prelude::*;

use super::msg::{EXT, MIN, split};

const PAD: &[u8] = include_bytes!("../../tests/fixtures/pad.ulg");

fn gyro(k: usize) -> [f32; 3] {
    let k = (k + 1) as f64;
    [(0.1 * k) as f32, (0.2 * k) as f32, (0.3 * k) as f32]
}

#[test]
fn min_ulg_parses_to_what_the_probe_wrote() {
    let f = UlogFile::parse(MIN).unwrap();
    assert_eq!(f.start_us, 1_000_000);
    assert_eq!(f.compat_flags, [0; 8]);
    assert_eq!(f.incompat_flags, [0; 8]);
    assert_eq!(f.appended_offsets, [0; 3]);
    assert_eq!(f.formats.len(), 1);
    assert_eq!(f.formats[0].name, "oxide_probe");
    assert_eq!(f.formats[0].layout.size, 24);
    assert_eq!(f.formats[0].layout.size_no_trailing_padding, 20);
    assert_eq!(f.info_str("sys_name"), Some("OxideTest"));
    assert_eq!(f.info_str("ver_sw"), Some("abc1234"));
    assert_eq!(
        f.info("ver_sw_release").unwrap().as_u32(),
        Some(0x0001_00FF)
    );
    assert_eq!(f.info("time_ref_utc").unwrap().as_i32(), Some(0));
    assert_eq!(f.initial_params.len(), 2);
    assert_eq!(f.initial_params[0].name, "OXIDE_PROBE_ID");
    assert_eq!(f.initial_params[0].value, ParamValue::I32(42));
    assert_eq!(f.initial_params[1].value, ParamValue::F32(0.5));
    assert_eq!(f.subscriptions.len(), 1);
    assert_eq!(
        (f.subscriptions[0].multi_id, f.subscriptions[0].msg_id),
        (0, 0)
    );
    assert_eq!(f.subscriptions[0].name, "oxide_probe");
    assert_eq!(f.items.len(), 4);
    assert!(f.truncated.is_none());

    let recs: Vec<_> = f.records("oxide_probe").collect();
    assert_eq!(recs.len(), 3);
    for (k, r) in recs.iter().enumerate() {
        assert_eq!(r.u64("timestamp"), Some(2_000_000 + 1000 * k as u64));
        let g = gyro(k);
        for (i, want) in g.iter().enumerate() {
            assert_eq!(
                r.f32(&format!("gyro_rad[{i}]")).unwrap().to_bits(),
                want.to_bits()
            );
        }
        assert_eq!(r.raw().len(), 20);
        // Wrong type, missing field, index out of range, trailing padding: no value.
        assert_eq!(r.u32("timestamp"), None);
        assert_eq!(r.f32("nothing"), None);
        assert_eq!(r.f32("gyro_rad[3]"), None);
        assert_eq!(r.f32("gyro_rad"), None);
        assert_eq!(r.u8("_padding0[0]"), None);
        assert_eq!(r.field_bytes("gyro_rad").unwrap().len(), 12);
    }
    match &f.items[3] {
        Item::Log {
            level,
            tag,
            time_us,
            text,
        } => {
            assert_eq!(
                (*level, *tag, *time_us, text.as_str()),
                (6, None, 2_002_500, "hello oxide")
            );
        }
        other => panic!("{other:?}"),
    }
}

#[test]
fn ext_ulg_parses_with_every_kind_of_item_in_file_order() {
    let f = UlogFile::parse(EXT).unwrap();
    assert_eq!(f.compat_flags[0], 1);
    assert_eq!(f.default_params.len(), 1);
    assert_eq!(f.default_params[0].default_types, 0b11);
    assert_eq!(f.default_params[0].name, "OXIDE_PROBE_GAIN");
    assert_eq!(f.default_params[0].value, ParamValue::F32(0.25));
    let multi = f.info_multi.iter().find(|e| e.name == "boot_text").unwrap();
    assert_eq!(multi.value, b"abcd");
    assert!(!multi.continued);
    let kinds: Vec<&str> = f
        .items
        .iter()
        .map(|i| match i {
            Item::Data(_) => "D",
            Item::Sync => "S",
            Item::Dropout { .. } => "O",
            Item::Log { tag: None, .. } => "L",
            Item::Log { tag: Some(_), .. } => "C",
            Item::Param { .. } => "P",
            other => panic!("{other:?}"),
        })
        .collect();
    assert_eq!(kinds, ["D", "S", "D", "O", "D", "L", "C", "P"]);
    assert!(matches!(f.items[3], Item::Dropout { ms: 17 }));
    match &f.items[6] {
        Item::Log {
            level,
            tag,
            time_us,
            text,
        } => {
            assert_eq!(
                (*level, *tag, *time_us, text.as_str()),
                (4, Some(7), 2_002_600, "tagged warn")
            );
        }
        other => panic!("{other:?}"),
    }
    match &f.items[7] {
        Item::Param { name, value } => {
            assert_eq!(
                (name.as_str(), *value),
                ("OXIDE_PROBE_GAIN", ParamValue::F32(0.75))
            );
        }
        other => panic!("{other:?}"),
    }
}

#[test]
fn pad_ulg_is_refused_as_pyulog_refuses_it() {
    // pad.ulg is min.ulg with 4 bytes of trailing padding in every D.
    let m = split(MIN);
    let first_d: usize = 16 + m[..9].iter().map(|x| x.len()).sum::<usize>();
    let e = UlogFile::parse(PAD).unwrap_err();
    assert_eq!(e.offset, first_d);
    assert_eq!(
        e.kind,
        ReadErrorKind::RecordLength {
            topic: "oxide_probe".into(),
            expected: 20,
            got: 24
        }
    );
    assert!(e.to_string().contains(&first_d.to_string()), "{e}");
}

fn definitions(out: &mut Vec<u8>, topics: &[&Topic]) {
    msg::file_header(out, 5).unwrap();
    msg::flag_bits(out, [0; 8], [0; 8]).unwrap();
    for t in topics {
        t.write_format(out).unwrap();
    }
}

const SMALL: Topic = Topic {
    name: "small",
    format: "uint64_t timestamp;uint8_t[8] data;",
    record_len: 16,
};

fn with_sub(topics: &[&Topic]) -> Vec<u8> {
    let mut v = Vec::new();
    definitions(&mut v, topics);
    for (i, t) in topics.iter().enumerate() {
        t.add_logged(&mut v, 0, i as u16).unwrap();
    }
    v
}

fn put_small(v: &mut Vec<u8>, id: u16, t: u64) {
    record::data(v, id, &SMALL, |w| {
        w.u64(t).bytes(&[1; 8]);
    })
    .unwrap();
}

#[test]
fn the_error_cases_say_where() {
    // Bad magic and too short.
    assert_eq!(
        UlogFile::parse(b"short").unwrap_err().kind,
        ReadErrorKind::TooShort
    );
    let mut bad = MIN.to_vec();
    bad[3] = b'X';
    let e = UlogFile::parse(&bad).unwrap_err();
    assert_eq!((e.offset, e.kind), (0, ReadErrorKind::BadMagic));
    // An incompatible flag the reader does not know.
    let mut bad = MIN.to_vec();
    bad[16 + 3 + 8] = 2;
    let e = UlogFile::parse(&bad).unwrap_err();
    assert_eq!(e.offset, 16);
    assert!(matches!(e.kind, ReadErrorKind::UnknownIncompatFlags(_)));
    // Unknown message id.
    let mut v = with_sub(&[&SMALL]);
    let at = v.len();
    put_small(&mut v, 9, 1);
    let e = UlogFile::parse(&v).unwrap_err();
    assert_eq!((e.offset, e.kind), (at, ReadErrorKind::UnknownMessageId(9)));
    // A record shorter than the format, then longer.
    let mut v = with_sub(&[&SMALL]);
    let at = v.len();
    msg::data_raw(&mut v, 0, &[0; 15]).unwrap();
    let e = UlogFile::parse(&v).unwrap_err();
    assert_eq!(e.offset, at);
    assert!(matches!(
        e.kind,
        ReadErrorKind::RecordLength {
            expected: 16,
            got: 15,
            ..
        }
    ));
    // A definition after the first subscription.
    let mut v = with_sub(&[&SMALL]);
    let at = v.len();
    msg::format(&mut v, "late", "uint64_t timestamp;").unwrap();
    let e = UlogFile::parse(&v).unwrap_err();
    assert_eq!(
        (e.offset, e.kind),
        (at, ReadErrorKind::DefinitionAfterSubscription(b'F'))
    );
    // A subscription of a format never defined, and a duplicate message id.
    let mut v = Vec::new();
    definitions(&mut v, &[&SMALL]);
    let at = v.len();
    msg::add_logged(&mut v, 0, 0, "nothere").unwrap();
    let e = UlogFile::parse(&v).unwrap_err();
    assert_eq!(
        (e.offset, e.kind),
        (at, ReadErrorKind::UnknownTopic("nothere".into()))
    );
    let mut v = with_sub(&[&SMALL]);
    let at = v.len();
    SMALL.add_logged(&mut v, 1, 0).unwrap();
    let e = UlogFile::parse(&v).unwrap_err();
    assert_eq!(
        (e.offset, e.kind),
        (at, ReadErrorKind::DuplicateMessageId(0))
    );
    // A sync with the wrong magic.
    let mut v = with_sub(&[&SMALL]);
    let at = v.len();
    msg::data_raw(&mut v, 0, &[0; 16]).unwrap();
    v.truncate(at);
    v.extend_from_slice(&[8, 0, b'S', 1, 2, 3, 4, 5, 6, 7, 8]);
    let e = UlogFile::parse(&v).unwrap_err();
    assert_eq!((e.offset, e.kind), (at, ReadErrorKind::BadSync));
    // A format that does not parse reports the offset of its message.
    let mut v = Vec::new();
    msg::file_header(&mut v, 0).unwrap();
    msg::flag_bits(&mut v, [0; 8], [0; 8]).unwrap();
    let at = v.len();
    msg::format(&mut v, "broken", "uint64_t timestamp;mystery x;").unwrap();
    msg::add_logged(&mut v, 0, 0, "broken").unwrap();
    let e = UlogFile::parse(&v).unwrap_err();
    assert_eq!(e.offset, at);
    assert!(matches!(e.kind, ReadErrorKind::BadFormat(_)));
}

#[test]
fn a_file_cut_by_power_loss_keeps_everything_before_the_cut() {
    let m = split(MIN);
    let last_start = MIN.len() - m[12].len();
    // Cut inside the last message (the logged string), and inside its header.
    for cut in [MIN.len() - 1, MIN.len() - 5, last_start + 2, last_start + 1] {
        let f = UlogFile::parse(&MIN[..cut]).unwrap();
        let t = f.truncated.expect("reported as truncated");
        assert_eq!(t.offset, last_start, "cut at {cut}");
        assert_eq!(
            f.items.len(),
            3,
            "the three records before it are available"
        );
    }
    // Cut exactly between messages: not truncated, one message fewer.
    let f = UlogFile::parse(&MIN[..last_start]).unwrap();
    assert!(f.truncated.is_none());
    assert_eq!(f.items.len(), 3);
    // Cut inside a definition: an error with the offset.
    let cut = 16 + m[0].len() + 10;
    let e = UlogFile::parse(&MIN[..cut]).unwrap_err();
    assert_eq!(e.offset, 16 + m[0].len());
    assert!(matches!(e.kind, ReadErrorKind::Truncated { .. }));
}

const NEST: Topic = Topic {
    name: "nest",
    format: "uint64_t timestamp;double d;inner[2] items;int16_t i16;char[4] name;uint8_t[2] _padding0;int32_t after;uint8_t[4] _padding1;",
    record_len: 8 + 8 + 16 + 2 + 4 + 2 + 4,
};

#[test]
fn nested_types_arrays_and_padding_in_the_middle_read_by_name() {
    let mut v = Vec::new();
    msg::file_header(&mut v, 0).unwrap();
    msg::flag_bits(&mut v, [0; 8], [0; 8]).unwrap();
    msg::format(&mut v, "inner", "uint32_t a;float[1] b;").unwrap();
    NEST.write_format(&mut v).unwrap();
    NEST.add_logged(&mut v, 0, 7).unwrap();
    record::data(&mut v, 7, &NEST, |w| {
        w.u64(99).f64(-1.25);
        for k in 0..2u32 {
            w.u32(10 + k).f32(0.5 + k as f32);
        }
        w.i16(-7).char_array(4, "ab").pad(2).i32(-9);
    })
    .unwrap();
    let f = UlogFile::parse(&v).unwrap();
    let r = f.records("nest").next().unwrap();
    assert_eq!(r.u64("timestamp"), Some(99));
    assert_eq!(r.f64("d"), Some(-1.25));
    assert_eq!(r.u32("items[0].a"), Some(10));
    assert_eq!(r.u32("items[1].a"), Some(11));
    assert_eq!(r.f32("items[1].b[0]"), Some(1.5));
    assert_eq!(r.i16("i16"), Some(-7));
    assert_eq!(r.field_bytes("name"), Some(&b"ab\0\0"[..]));
    assert_eq!(r.char("name[1]"), Some(b'b'));
    assert_eq!(r.i32("after"), Some(-9));
    assert_eq!(r.u8("_padding0[1]"), Some(0));
    assert_eq!(
        r.u8("_padding1[0]"),
        None,
        "the trailing padding is not in the record"
    );
    assert_eq!(r.raw().len(), NEST.record_len);
}

#[test]
fn our_own_writer_round_trips_in_order() {
    let mut v = Vec::new();
    msg::file_header(&mut v, 42).unwrap();
    msg::flag_bits(&mut v, [1, 0, 0, 0, 0, 0, 0, 0], [0; 8]).unwrap();
    SMALL.write_format(&mut v).unwrap();
    msg::info(&mut v, "sys_name", InfoValue::Str("Oxide")).unwrap();
    msg::info(
        &mut v,
        "oxide_config_hash",
        InfoValue::U64(0xDEAD_BEEF_0123_4567),
    )
    .unwrap();
    msg::parameter(&mut v, "A", ParamValue::F32(f32::from_bits(0x7FC0_0001))).unwrap();
    SMALL.add_logged(&mut v, 0, 0).unwrap();
    put_small(&mut v, 0, 10);
    msg::logged_string(&mut v, 3, 11, "err").unwrap();
    msg::parameter(&mut v, "A", ParamValue::F32(-0.0)).unwrap();
    msg::dropout(&mut v, 5).unwrap();
    msg::sync(&mut v).unwrap();
    put_small(&mut v, 0, 12);
    let f = UlogFile::parse(&v).unwrap();
    assert_eq!(f.start_us, 42);
    assert_eq!(f.compat_flags[0], 1);
    assert_eq!(
        f.info("oxide_config_hash").unwrap().as_u64(),
        Some(0xDEAD_BEEF_0123_4567)
    );
    match f.initial_params[0].value {
        ParamValue::F32(x) => assert_eq!(x.to_bits(), 0x7FC0_0001),
        v => panic!("{v:?}"),
    }
    let order: Vec<char> = f
        .items
        .iter()
        .map(|i| match i {
            Item::Data(_) => 'D',
            Item::Log { .. } => 'L',
            Item::Param { .. } => 'P',
            Item::Dropout { .. } => 'O',
            Item::Sync => 'S',
            o => panic!("{o:?}"),
        })
        .collect();
    assert_eq!(order, ['D', 'L', 'P', 'O', 'S', 'D']);
    match &f.items[2] {
        Item::Param {
            value: ParamValue::F32(x),
            ..
        } => assert_eq!(x.to_bits(), (-0.0f32).to_bits()),
        o => panic!("{o:?}"),
    }
    let ts: Vec<u64> = f
        .records("small")
        .map(|r| r.u64("timestamp").unwrap())
        .collect();
    assert_eq!(ts, [10, 12]);
}

const T1: Topic = Topic {
    name: "t1",
    format: "uint64_t timestamp;uint8_t[1] data;",
    record_len: 9,
};
const T2: Topic = Topic {
    name: "t2",
    format: "uint64_t timestamp;uint8_t[40] data;",
    record_len: 48,
};
const T3: Topic = Topic {
    name: "t3",
    format: "uint64_t timestamp;uint8_t[300] data;",
    record_len: 308,
};

proptest! {
    #![proptest_config(ProptestConfig::with_cases(300))]

    /// Whatever the pattern of writes and drains, the drained bytes appended to
    /// a valid header and definitions parse with the reader as whole messages,
    /// the reader's count of data and logged-string messages equals
    /// `stats().written`, and written plus dropped equals the attempts.
    #[test]
    fn the_logger_stream_always_parses_with_the_reader(
        ops in proptest::collection::vec(
            (any::<bool>(), 0u64..800_000, 0u8..4, 0usize..400, prop_oneof![Just(None), (0usize..50).prop_map(Some)]),
            1..100),
        reserve in 0usize..600,
    ) {
        let topics = [&T1, &T2, &T3];
        let mut file = with_sub(&topics);
        let defs_len = file.len();
        let mut l = Logger::<512>::new(reserve);
        let mut t = 0u64;
        let mut attempts = 0u32;
        for (must, dt, kind, size, drain) in ops {
            t += dt;
            attempts += 1;
            let class = if must { Class::Must } else { Class::Stream };
            l.write(class, t, |w| match kind {
                0 => record::data(w, 0, &T1, |r| { r.u64(t).u8(1); }),
                1 => record::data(w, 1, &T2, |r| { r.u64(t).bytes(&[2; 40]); }),
                2 => record::data(w, 2, &T3, |r| { r.u64(t).bytes(&[3; 300]); }),
                _ => msg::logged_string(w, 6, t, &"x".repeat(size)),
            });
            if let Some(n) = drain {
                l.drain(|b| { let k = b.len().min(n); file.extend_from_slice(&b[..k]); k });
            }
        }
        while !l.is_empty() {
            l.drain(|b| { file.extend_from_slice(b); b.len() });
        }
        let f = UlogFile::parse(&file).map_err(|e| TestCaseError::fail(e.to_string()))?;
        prop_assert!(f.truncated.is_none());
        let s = l.stats();
        let counted = f.items.iter().filter(|i| matches!(i, Item::Data(_) | Item::Log { .. })).count();
        prop_assert_eq!(counted as u32, s.written);
        prop_assert_eq!(s.written + s.dropped_stream + s.dropped_must, attempts);
        prop_assert_eq!((file.len() - defs_len) as u64, s.bytes);
        let dropouts = f.items.iter().filter(|i| matches!(i, Item::Dropout { .. })).count();
        prop_assert_eq!(dropouts as u32, s.dropouts);
        // Data timestamps never go backwards within a topic.
        for name in ["t1", "t2", "t3"] {
            let ts: Vec<u64> = f.records(name).map(|r| r.u64("timestamp").unwrap()).collect();
            prop_assert!(ts.windows(2).all(|w| w[0] <= w[1]));
        }
    }
}
