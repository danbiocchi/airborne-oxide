// SPDX-License-Identifier: AGPL-3.0-or-later
//! Step 2: topic constants and the record writer.

use crate::msg::{self, Error, SliceSink};
use crate::record::{self, Topic};

use super::msg::{MIN, split};

pub const PROBE: Topic = Topic {
    name: "oxide_probe",
    format: "uint64_t timestamp;float[3] gyro_rad;uint8_t[4] _padding0;",
    record_len: 20,
};

fn write(
    topic: &Topic,
    f: impl FnOnce(&mut record::RecordWriter<'_, SliceSink<'_>>),
) -> (Result<(), Error>, Vec<u8>) {
    let mut buf = [0u8; 512];
    let mut s = SliceSink::new(&mut buf);
    let r = record::data(&mut s, 3, topic, f);
    (r, s.written().to_vec())
}

#[test]
fn a_record_filled_field_by_field_equals_the_probe_message() {
    let m = split(MIN);
    let (r, got) = write(&PROBE, |w| {
        w.u64(2_000_000)
            .f32((0.1f64 * 1.0) as f32)
            .f32((0.2f64 * 1.0) as f32)
            .f32((0.3f64 * 1.0) as f32);
    });
    assert_eq!(r, Ok(()));
    // The probe's message is for msg_id 0: compare everything but that id.
    assert_eq!(got[..3], m[9][..3]);
    assert_eq!(got[3..5], 3u16.to_le_bytes());
    assert_eq!(got[5..], m[9][5..]);
}

#[test]
fn f32_array_writes_each_element_in_order() {
    let (r, got) = write(&PROBE, |w| {
        w.u64(7).f32_array(&[1.0, 2.0, 3.0]);
    });
    assert_eq!(r, Ok(()));
    assert_eq!(
        &got[5 + 8..],
        [1.0f32, 2.0, 3.0]
            .iter()
            .flat_map(|f| f.to_le_bytes())
            .collect::<Vec<_>>()
    );
}

#[test]
fn too_few_and_too_many_bytes_are_errors() {
    let (r, _) = write(&PROBE, |w| {
        w.u64(1).f32(1.0);
    });
    assert_eq!(
        r,
        Err(Error::RecordLength {
            expected: 20,
            got: 12
        })
    );
    let (r, got) = write(&PROBE, |w| {
        w.u64(1).f32_array(&[1.0, 2.0, 3.0]).u8(9);
    });
    assert_eq!(
        r,
        Err(Error::RecordLength {
            expected: 20,
            got: 21
        })
    );
    // Nothing past the declared length reaches the sink.
    assert_eq!(got.len(), 5 + 20);
}

#[test]
fn every_scalar_type_is_little_endian_and_sized() {
    const ALL: Topic = Topic {
        name: "all",
        format: "uint64_t timestamp;",
        record_len: 8 + 1 + 1 + 1 + 2 + 2 + 4 + 4 + 8 + 8 + 4 + 8 + 3 + 5 + 2,
    };
    let (r, got) = write(&ALL, |w| {
        w.u64(0x0102_0304_0506_0708)
            .u8(0xAB)
            .bool(true)
            .i8(-2)
            .u16(0x1234)
            .i16(-3)
            .u32(0xDEAD_BEEF)
            .i32(-4)
            .u64(5)
            .i64(-6)
            .f32(1.5)
            .f64(-2.5)
            .pad(3)
            .bytes(&[1, 2, 3, 4, 5])
            .char_array(2, "hi");
    });
    assert_eq!(r, Ok(()));
    let rec = &got[5..];
    assert_eq!(rec[..8], [8, 7, 6, 5, 4, 3, 2, 1]);
    assert_eq!(rec[8..11], [0xAB, 1, 0xFE]);
    assert_eq!(rec[11..13], [0x34, 0x12]);
    assert_eq!(rec[13..15], (-3i16).to_le_bytes());
    assert_eq!(rec[15..19], 0xDEAD_BEEFu32.to_le_bytes());
    assert_eq!(rec[19..23], (-4i32).to_le_bytes());
    assert_eq!(rec[23..31], 5u64.to_le_bytes());
    assert_eq!(rec[31..39], (-6i64).to_le_bytes());
    assert_eq!(rec[39..43], 1.5f32.to_le_bytes());
    assert_eq!(rec[43..51], (-2.5f64).to_le_bytes());
    assert_eq!(rec[51..54], [0, 0, 0]);
    assert_eq!(rec[54..59], [1, 2, 3, 4, 5]);
    assert_eq!(rec[59..61], *b"hi");
}

#[test]
fn char_arrays_are_zero_filled_and_cut_to_length() {
    const T: Topic = Topic {
        name: "s",
        format: "uint64_t timestamp;",
        record_len: 8 + 6 + 3,
    };
    let (r, got) = write(&T, |w| {
        w.u64(0).char_array(6, "abc").char_array(3, "abcdef");
    });
    assert_eq!(r, Ok(()));
    assert_eq!(&got[5 + 8..], b"abc\0\0\0abc");
}

#[test]
fn nan_payloads_and_negative_zero_survive() {
    const T: Topic = Topic {
        name: "n",
        format: "uint64_t timestamp;",
        record_len: 8 + 4 + 4 + 8 + 8,
    };
    let nan32 = f32::from_bits(0x7FC0_1234);
    let snan32 = f32::from_bits(0x7F80_0001);
    let nan64 = f64::from_bits(0x7FF8_0000_DEAD_BEEF);
    let (r, got) = write(&T, |w| {
        w.u64(0).f32(nan32).f32(-0.0).f64(nan64).f64(-0.0);
    });
    assert_eq!(r, Ok(()));
    let rec = &got[5 + 8..];
    assert_eq!(rec[..4], 0x7FC0_1234u32.to_le_bytes());
    assert_eq!(rec[4..8], 0x8000_0000u32.to_le_bytes());
    assert_eq!(rec[8..16], 0x7FF8_0000_DEAD_BEEFu64.to_le_bytes());
    assert_eq!(rec[16..24], 0x8000_0000_0000_0000u64.to_le_bytes());
    // A signalling NaN is not quietened on the way either.
    let (_, got) = write(&T, |w| {
        w.u64(0).f32(snan32).f32(0.0).f64(0.0).f64(0.0);
    });
    assert_eq!(got[5 + 8..5 + 12], 0x7F80_0001u32.to_le_bytes());
}

#[test]
fn a_full_sink_is_reported_and_the_record_writer_stops() {
    let mut buf = [0u8; 10];
    let mut s = SliceSink::new(&mut buf);
    let r = record::data(&mut s, 0, &PROBE, |w| {
        w.u64(1).f32_array(&[0.0; 3]);
    });
    assert_eq!(r, Err(Error::Full));
}

#[test]
fn a_topic_writes_its_own_format_and_subscription() {
    let m = split(MIN);
    let mut buf = [0u8; 256];
    let mut s = SliceSink::new(&mut buf);
    PROBE.write_format(&mut s).unwrap();
    PROBE.add_logged(&mut s, 0, 0).unwrap();
    let mut want = m[1].to_vec();
    want.extend_from_slice(m[8]);
    assert_eq!(s.written(), want);
    let _ = msg::MAX_PAYLOAD;
}
