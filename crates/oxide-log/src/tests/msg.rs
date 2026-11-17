// SPDX-License-Identifier: AGPL-3.0-or-later
//! Step 1: every encoder against the bytes of the probe files (`min.ulg`,
//! `ext.ulg`, written by `write_min_ulog.py`, a struct-only writer that shares
//! no code with this crate).

use crate::msg::{self, InfoValue, ParamValue, SliceSink};

pub const MIN: &[u8] = include_bytes!("../../tests/fixtures/min.ulg");
pub const EXT: &[u8] = include_bytes!("../../tests/fixtures/ext.ulg");

/// Splits a ULog byte string (after its 16-byte header) into whole messages.
pub fn split(file: &[u8]) -> Vec<&[u8]> {
    let mut out = Vec::new();
    let mut at = 16;
    while at < file.len() {
        let n = u16::from_le_bytes([file[at], file[at + 1]]) as usize;
        out.push(&file[at..at + 3 + n]);
        at += 3 + n;
    }
    out
}

fn enc(f: impl FnOnce(&mut SliceSink<'_>) -> Result<(), msg::Error>) -> Vec<u8> {
    let mut buf = [0u8; 2048];
    let mut s = SliceSink::new(&mut buf);
    f(&mut s).unwrap();
    s.written().to_vec()
}

#[test]
fn file_header_is_the_probe_header() {
    assert_eq!(enc(|s| msg::file_header(s, 1_000_000)), &MIN[..16]);
}

#[test]
fn flag_bits_match_the_probe() {
    let m = split(MIN);
    assert_eq!(enc(|s| msg::flag_bits(s, [0; 8], [0; 8])), m[0]);
    assert_eq!(m[0].len(), 43);
    // ext.ulg sets compat bit 0 (default parameters).
    let e = split(EXT);
    let mut compat = [0u8; 8];
    compat[0] = 1;
    assert_eq!(enc(|s| msg::flag_bits(s, compat, [0; 8])), e[0]);
}

#[test]
fn format_matches_the_probe() {
    let m = split(MIN);
    let got = enc(|s| {
        msg::format(
            s,
            "oxide_probe",
            "uint64_t timestamp;float[3] gyro_rad;uint8_t[4] _padding0;",
        )
    });
    assert_eq!(got, m[1]);
}

#[test]
fn information_messages_match_the_probe() {
    let m = split(MIN);
    assert_eq!(
        enc(|s| msg::info(s, "sys_name", InfoValue::Str("OxideTest"))),
        m[2]
    );
    assert_eq!(
        enc(|s| msg::info(s, "ver_sw", InfoValue::Str("abc1234"))),
        m[3]
    );
    assert_eq!(
        enc(|s| msg::info(s, "ver_sw_release", InfoValue::U32(0x0001_00FF))),
        m[4]
    );
    assert_eq!(
        enc(|s| msg::info(s, "time_ref_utc", InfoValue::I32(0))),
        m[5]
    );
}

#[test]
fn information_value_types_have_the_right_keys_and_widths() {
    // u64 and f32 are not in the probe; the layout is the same as for u32.
    let b = enc(|s| msg::info(s, "t", InfoValue::U64(0x0102_0304_0506_0708)));
    let key = b"uint64_t t";
    let mut want = vec![(1 + key.len() + 8) as u8, 0, b'I', key.len() as u8];
    want.extend_from_slice(key);
    want.extend_from_slice(&0x0102_0304_0506_0708u64.to_le_bytes());
    assert_eq!(b, want);
    let b = enc(|s| msg::info(s, "g", InfoValue::F32(0.5)));
    let key = b"float g";
    let mut want = vec![(1 + key.len() + 4) as u8, 0, b'I', key.len() as u8];
    want.extend_from_slice(key);
    want.extend_from_slice(&0.5f32.to_le_bytes());
    assert_eq!(b, want);
    // A string key carries its length, also past one digit.
    let long = "x".repeat(120);
    let b = enc(|s| msg::info(s, "k", InfoValue::Str(&long)));
    let key = b"char[120] k";
    assert_eq!(&b[3..4], &[key.len() as u8]);
    assert_eq!(&b[4..4 + key.len()], key);
    assert_eq!(b.len(), 3 + 1 + key.len() + 120);
}

#[test]
fn parameters_match_the_probe_in_definitions_and_data() {
    let m = split(MIN);
    assert_eq!(
        enc(|s| msg::parameter(s, "OXIDE_PROBE_ID", ParamValue::I32(42))),
        m[6]
    );
    assert_eq!(
        enc(|s| msg::parameter(s, "OXIDE_PROBE_GAIN", ParamValue::F32(0.5))),
        m[7]
    );
    // The same bytes serve as a change in the data section (ext.ulg, last message).
    let e = split(EXT);
    assert_eq!(
        enc(|s| msg::parameter(s, "OXIDE_PROBE_GAIN", ParamValue::F32(0.75))),
        *e.last().unwrap()
    );
}

#[test]
fn add_logged_matches_the_probe() {
    let m = split(MIN);
    assert_eq!(enc(|s| msg::add_logged(s, 0, 0, "oxide_probe")), m[8]);
}

#[test]
fn data_header_is_size_type_and_message_id() {
    let m = split(MIN);
    let got = enc(|s| msg::data_raw(s, 0, &m[9][5..]));
    assert_eq!(got, m[9]);
}

#[test]
fn logged_string_matches_the_probe() {
    let m = split(MIN);
    assert_eq!(
        enc(|s| msg::logged_string(s, 6, 2_002_500, "hello oxide")),
        m[12]
    );
    assert_eq!(m[12][3], b'6');
}

#[test]
fn sync_and_dropout_match_the_probe() {
    let e = split(EXT);
    // ext.ulg order: B F I I I I P P Q M A D S D O D L C P
    assert_eq!(enc(|s| msg::sync(s)), e[12]);
    assert_eq!(enc(|s| msg::dropout(s, 17)), e[14]);
    assert_eq!(
        e[12],
        [8, 0, b'S', 0x2F, 0x73, 0x13, 0x20, 0x25, 0x0C, 0xBB, 0x12]
    );
    assert_eq!(e[14], [2, 0, b'O', 17, 0]);
}

#[test]
fn messages_that_cannot_be_represented_are_refused() {
    let mut buf = [0u8; 16];
    // Too small a sink.
    assert_eq!(
        msg::sync(&mut SliceSink::new(&mut buf[..4])),
        Err(msg::Error::Full)
    );
    let mut big = [0u8; 70_000];
    let long = "y".repeat(65_540);
    assert_eq!(
        msg::logged_string(&mut SliceSink::new(&mut big), 6, 0, &long),
        Err(msg::Error::TooLong)
    );
    let mut buf = [0u8; 300];
    // Level is 0..=7.
    assert_eq!(
        msg::logged_string(&mut SliceSink::new(&mut buf), 8, 0, "x"),
        Err(msg::Error::BadLevel)
    );
    // A parameter name outside a-zA-Z0-9_-/ is refused.
    assert_eq!(
        msg::parameter(
            &mut SliceSink::new(&mut buf),
            "bad name",
            ParamValue::I32(0)
        ),
        Err(msg::Error::BadName)
    );
    // key_len is a u8.
    let name = "n".repeat(250);
    assert_eq!(
        msg::info(&mut SliceSink::new(&mut buf), &name, InfoValue::U32(0)),
        Err(msg::Error::TooLong)
    );
}

/// What `write_min_ulog.py` writes, written with our encoders: byte identical.
#[test]
fn a_whole_file_is_byte_identical_to_min_ulg() {
    let mut buf = [0u8; 1024];
    let mut s = SliceSink::new(&mut buf);
    msg::file_header(&mut s, 1_000_000).unwrap();
    msg::flag_bits(&mut s, [0; 8], [0; 8]).unwrap();
    msg::format(
        &mut s,
        "oxide_probe",
        "uint64_t timestamp;float[3] gyro_rad;uint8_t[4] _padding0;",
    )
    .unwrap();
    msg::info(&mut s, "sys_name", InfoValue::Str("OxideTest")).unwrap();
    msg::info(&mut s, "ver_sw", InfoValue::Str("abc1234")).unwrap();
    msg::info(&mut s, "ver_sw_release", InfoValue::U32(0x0001_00FF)).unwrap();
    msg::info(&mut s, "time_ref_utc", InfoValue::I32(0)).unwrap();
    msg::parameter(&mut s, "OXIDE_PROBE_ID", ParamValue::I32(42)).unwrap();
    msg::parameter(&mut s, "OXIDE_PROBE_GAIN", ParamValue::F32(0.5)).unwrap();
    msg::add_logged(&mut s, 0, 0, "oxide_probe").unwrap();
    for (i, t) in [2_000_000u64, 2_001_000, 2_002_000].into_iter().enumerate() {
        let k = (i + 1) as f64;
        let mut rec = [0u8; 20];
        rec[..8].copy_from_slice(&t.to_le_bytes());
        rec[8..12].copy_from_slice(&((0.1 * k) as f32).to_le_bytes());
        rec[12..16].copy_from_slice(&((0.2 * k) as f32).to_le_bytes());
        rec[16..20].copy_from_slice(&((0.3 * k) as f32).to_le_bytes());
        msg::data_raw(&mut s, 0, &rec).unwrap();
    }
    msg::logged_string(&mut s, 6, 2_002_500, "hello oxide").unwrap();
    assert_eq!(s.written(), MIN);
}
