// SPDX-License-Identifier: AGPL-3.0-or-later
//! Step 5: what `pyulog` reads from a file written here equals what the Rust
//! reader reads, bit for bit. Skipped, with a printed notice, unless the
//! environment variable `OXIDE_PYTHON` names a Python that has `pyulog`.

use crate::format::{FieldType, Layout};
use crate::logger::{Class, Logger};
use crate::msg::{self, InfoValue, ParamValue};
use crate::reader::{Item, RecordView, UlogFile};
use crate::record::{self, Topic};
use std::process::Command;

const RICH: Topic = Topic {
    name: "oxide_rich",
    format: "uint64_t timestamp;double d;float[3] v;uint8_t[4] _padding0;uint64_t big;int32_t i;\
             uint16_t u;int8_t s;uint8_t a;bool flag;char[3] tag;uint8_t[4] _padding1;",
    record_len: 52,
};
const SUB: Topic = Topic {
    name: "oxide_sub",
    format: "uint32_t a;float[2] b;",
    record_len: 12,
};
const OUTER: Topic = Topic {
    name: "oxide_outer",
    format: "uint64_t timestamp;oxide_sub[2] items;uint8_t flag;uint8_t[7] _padding0;",
    record_len: 8 + 24 + 1,
};

fn put_rich(w: &mut impl msg::Sink, id: u16, t: u64, k: u32) -> Result<(), msg::Error> {
    record::data(w, id, &RICH, |r| {
        r.u64(t)
            .f64(f64::from_bits(0x7FF8_0000_DEAD_BEEF ^ u64::from(k)))
            .f32_array(&[f32::from_bits(0x7FC0_1234 + k), -0.0, k as f32 * 0.1])
            .pad(4)
            .u64(u64::MAX - u64::from(k))
            .i32(-(k as i32))
            .u16(65_000)
            .i8(-5)
            .u8(250)
            .bool(k % 2 == 0)
            .char_array(3, "ab");
    })
}

fn put_outer(w: &mut impl msg::Sink, id: u16, t: u64, k: u32) -> Result<(), msg::Error> {
    record::data(w, id, &OUTER, |r| {
        r.u64(t);
        for j in 0..2u32 {
            r.u32(k * 10 + j).f32_array(&[1.5 + j as f32, -2.25]);
        }
        r.u8(7);
    })
}

/// A file with two topics (padding inside and trailing, arrays, nested types),
/// a parameter change, a logged string, a dropout and a sync. The data section
/// goes through the logger so the dropout and sync are its own.
fn build_file() -> Vec<u8> {
    let mut f = Vec::new();
    msg::file_header(&mut f, 0).unwrap();
    msg::flag_bits(&mut f, [0; 8], [0; 8]).unwrap();
    SUB.write_format(&mut f).unwrap();
    RICH.write_format(&mut f).unwrap();
    OUTER.write_format(&mut f).unwrap();
    msg::info(&mut f, "sys_name", InfoValue::Str("Oxide")).unwrap();
    msg::info(
        &mut f,
        "oxide_config_hash",
        InfoValue::U64(0x0123_4567_89AB_CDEF),
    )
    .unwrap();
    msg::info(&mut f, "ver_sw_release", InfoValue::U32(0x0001_0000)).unwrap();
    msg::parameter(&mut f, "OX_I", ParamValue::I32(-42)).unwrap();
    msg::parameter(&mut f, "OX_F", ParamValue::F32(0.5)).unwrap();
    RICH.add_logged(&mut f, 0, 0).unwrap();
    OUTER.add_logged(&mut f, 0, 1).unwrap();

    let mut l = Logger::<300>::new(0);
    let drain = |l: &mut Logger<300>, f: &mut Vec<u8>| {
        l.drain(|b| {
            f.extend_from_slice(b);
            b.len()
        });
    };
    for k in 0..5u32 {
        let t = 10_000 + 10_000 * u64::from(k);
        assert!(l.write(Class::Stream, t, |w| put_rich(w, 0, t, k)));
    }
    assert!(
        !l.write(Class::Stream, 60_000, |w| put_rich(w, 0, 60_000, 5)),
        "the ring is full"
    );
    drain(&mut l, &mut f);
    // 640 ms later: a dropout of 640 ms and a sync in front of this one.
    assert!(l.write(Class::Stream, 700_000, |w| put_rich(w, 0, 700_000, 6)));
    assert!(l.write(Class::Must, 700_100, |w| msg::parameter(
        w,
        "OX_F",
        ParamValue::F32(0.75)
    )));
    assert!(l.write(Class::Must, 700_200, |w| msg::logged_string(
        w,
        4,
        700_200,
        "low battery"
    )));
    assert!(l.write(Class::Stream, 700_300, |w| put_outer(w, 1, 700_300, 1)));
    assert!(l.write(Class::Stream, 700_400, |w| put_outer(w, 1, 700_400, 2)));
    drain(&mut l, &mut f);
    assert!(l.write(Class::Stream, 710_000, |w| put_rich(w, 0, 710_000, 7)));
    drain(&mut l, &mut f);
    assert_eq!((l.stats().dropouts, l.stats().syncs), (1, 1));
    f
}

/// The columns `pyulog` makes of a record: `(name, type)`, flattened.
fn columns(
    file: &UlogFile,
    layout: &Layout,
    prefix: &str,
    top: bool,
    out: &mut Vec<(String, String)>,
) {
    let last = if top {
        layout.fields.len()
            - layout
                .fields
                .iter()
                .rev()
                .take_while(|f| f.is_padding())
                .count()
    } else {
        layout.fields.len()
    };
    for f in &layout.fields[..last] {
        let n = f.array.unwrap_or(0);
        let names: Vec<String> = if n > 0 {
            (0..n).map(|i| format!("{prefix}{}[{i}]", f.name)).collect()
        } else {
            vec![format!("{prefix}{}", f.name)]
        };
        for name in names {
            match &f.ty {
                FieldType::Scalar(s) => {
                    let t = match s {
                        crate::format::Scalar::I8 => "int8_t",
                        crate::format::Scalar::U8 => "uint8_t",
                        crate::format::Scalar::I16 => "int16_t",
                        crate::format::Scalar::U16 => "uint16_t",
                        crate::format::Scalar::I32 => "int32_t",
                        crate::format::Scalar::U32 => "uint32_t",
                        crate::format::Scalar::I64 => "int64_t",
                        crate::format::Scalar::U64 => "uint64_t",
                        crate::format::Scalar::F32 => "float",
                        crate::format::Scalar::F64 => "double",
                        crate::format::Scalar::Bool => "bool",
                        crate::format::Scalar::Char => "char",
                    };
                    out.push((name, t.to_string()));
                }
                FieldType::Nested(inner) => {
                    columns(
                        file,
                        file.layout(inner).unwrap(),
                        &format!("{name}."),
                        false,
                        out,
                    );
                }
            }
        }
    }
}

fn cell(r: &RecordView<'_>, name: &str, ty: &str) -> String {
    match ty {
        "float" => format!("0x{:08x}", r.f32(name).unwrap().to_bits()),
        "double" => format!("0x{:016x}", r.f64(name).unwrap().to_bits()),
        "uint8_t" => r.u8(name).unwrap().to_string(),
        "int8_t" => r.i8(name).unwrap().to_string(),
        "uint16_t" => r.u16(name).unwrap().to_string(),
        "int16_t" => r.i16(name).unwrap().to_string(),
        "uint32_t" => r.u32(name).unwrap().to_string(),
        "int32_t" => r.i32(name).unwrap().to_string(),
        "uint64_t" => r.u64(name).unwrap().to_string(),
        "int64_t" => r.i64(name).unwrap().to_string(),
        "bool" => i8::from(r.bool(name).unwrap()).to_string(),
        "char" => (r.char(name).unwrap() as i8).to_string(),
        other => panic!("{other}"),
    }
}

fn python() -> Option<String> {
    let Ok(py) = std::env::var("OXIDE_PYTHON") else {
        println!("NOTICE: OXIDE_PYTHON is not set: the pyulog cross-check is skipped");
        return None;
    };
    let ok = Command::new(&py)
        .args(["-c", "import pyulog"])
        .output()
        .is_ok_and(|o| o.status.success());
    if !ok {
        println!("NOTICE: {py} cannot import pyulog: the pyulog cross-check is skipped");
        return None;
    }
    Some(py)
}

fn run(py: &str, args: &[&str]) -> (bool, String, String) {
    let script = concat!(env!("CARGO_MANIFEST_DIR"), "/../../tools/ulog_check.py");
    let o = Command::new(py).arg(script).args(args).output().unwrap();
    (
        o.status.success(),
        String::from_utf8(o.stdout).unwrap(),
        String::from_utf8(o.stderr).unwrap(),
    )
}

#[test]
fn pyulog_reads_the_same_values_bit_for_bit() {
    let Some(py) = python() else { return };
    let bytes = build_file();
    let path = std::env::temp_dir().join(format!("oxide-log-xcheck-{}.ulg", std::process::id()));
    std::fs::write(&path, &bytes).unwrap();
    let p = path.to_str().unwrap();
    let file = UlogFile::parse(&bytes).unwrap();

    let (ok, out, err) = run(&py, &[p]);
    assert!(ok, "ulog_check.py failed: {err}\n{out}");
    assert!(out.contains("topic oxide_rich 0 7\n"), "{out}");
    assert!(out.contains("topic oxide_outer 0 2\n"), "{out}");

    for name in ["oxide_rich", "oxide_outer"] {
        let (ok, csv, err) = run(&py, &[p, "--dump", name]);
        assert!(ok, "{err}");
        let sub = file.subscriptions.iter().find(|s| s.name == name).unwrap();
        let mut cols = Vec::new();
        columns(&file, &file.formats[sub.format].layout, "", true, &mut cols);
        let mut want = vec![
            cols.iter()
                .map(|(n, t)| format!("{n}:{t}"))
                .collect::<Vec<_>>()
                .join(","),
        ];
        for r in file.records(name) {
            want.push(
                cols.iter()
                    .map(|(n, t)| cell(&r, n, t))
                    .collect::<Vec<_>>()
                    .join(","),
            );
        }
        assert_eq!(csv.lines().collect::<Vec<_>>(), want, "topic {name}");
    }

    // What the writer put in: spot-check the CSV's interesting cells.
    let (_, csv, _) = run(&py, &[p, "--dump", "oxide_rich"]);
    let row0: Vec<&str> = csv.lines().nth(1).unwrap().split(',').collect();
    assert_eq!(row0[0], "10000");
    assert_eq!(
        row0[1],
        format!("0x{:016x}", 0x7FF8_0000_DEAD_BEEFu64),
        "NaN payload of a double"
    );
    assert_eq!(row0[2], "0x7fc01234", "NaN payload of a float");
    assert_eq!(row0[3], "0x80000000", "negative zero");

    let (ok, meta, err) = run(&py, &[p, "--meta"]);
    assert!(ok, "{err}");
    let lines: Vec<&str> = meta.lines().collect();
    assert!(lines.contains(&"start 0"), "{meta}");
    assert!(lines.contains(&"param OX_I -42"));
    assert!(lines.contains(&"param OX_F 0x3f000000"));
    assert!(lines.contains(&"changed 700000 OX_F 0x3f400000"), "{meta}");
    assert!(lines.contains(&"log 700200 WARNING low battery"));
    assert!(lines.contains(&"dropout 50000 640"), "{meta}");
    assert!(lines.contains(&"sync 1"));
    assert!(lines.contains(&"info sys_name Oxide"));
    assert!(lines.contains(&"info oxide_config_hash 81985529216486895"));

    // The Rust reader agrees on the non-record items.
    let kinds: Vec<&str> = file
        .items
        .iter()
        .map(|i| match i {
            Item::Data(_) => "D",
            Item::Param { .. } => "P",
            Item::Log { .. } => "L",
            Item::Dropout { .. } => "O",
            Item::Sync => "S",
            _ => "?",
        })
        .collect();
    assert_eq!(kinds.join(""), "DDDDDOSDPLDDD");
    let _ = std::fs::remove_file(path);
}

#[test]
fn pyulog_refuses_trailing_padding_in_a_record_and_so_do_we() {
    let Some(py) = python() else { return };
    let path = std::env::temp_dir().join(format!("oxide-log-pad-{}.ulg", std::process::id()));
    std::fs::write(&path, include_bytes!("../../tests/fixtures/pad.ulg")).unwrap();
    let (ok, _, err) = run(&py, &[path.to_str().unwrap()]);
    assert!(!ok);
    assert!(err.contains("corruption"), "{err}");
    let _ = std::fs::remove_file(path);
}
