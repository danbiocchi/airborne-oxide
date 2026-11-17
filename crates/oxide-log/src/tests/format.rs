// SPDX-License-Identifier: AGPL-3.0-or-later
//! Step 2 (std): the format-string parser and the topic checker.

use crate::format::{self, FormatError, Scalar};
use crate::record::Topic;

#[test]
fn the_probe_format_has_padded_and_unpadded_sizes() {
    let l = format::layout(
        "oxide_probe",
        "uint64_t timestamp;float[3] gyro_rad;uint8_t[4] _padding0;",
    )
    .unwrap();
    assert_eq!(l.size, 24);
    assert_eq!(l.size_no_trailing_padding, 20);
    assert_eq!(l.fields.len(), 3);
    assert_eq!(l.fields[1].offset, 8);
    assert_eq!(l.fields[1].array, Some(3));
    assert_eq!(l.fields[1].size, 12);
    assert_eq!(l.fields[2].offset, 20);
}

#[test]
fn padding_in_the_middle_is_part_of_the_record() {
    // pyulog's probe case "mid padding present in D".
    let l = format::layout(
        "t",
        "uint64_t timestamp;uint8_t a;uint8_t[3] _padding0;uint32_t b;",
    )
    .unwrap();
    assert_eq!(l.size, 16);
    assert_eq!(l.size_no_trailing_padding, 16);
    let l = format::layout(
        "t",
        "uint64_t timestamp;uint32_t b;uint8_t[2] _padding0;uint8_t[2] _padding1;",
    )
    .unwrap();
    assert_eq!(l.size, 16);
    assert_eq!(
        l.size_no_trailing_padding, 12,
        "a run of trailing padding fields is all dropped"
    );
}

#[test]
fn every_scalar_type_has_its_size() {
    let want = [
        ("int8_t", 1, Scalar::I8),
        ("uint8_t", 1, Scalar::U8),
        ("int16_t", 2, Scalar::I16),
        ("uint16_t", 2, Scalar::U16),
        ("int32_t", 4, Scalar::I32),
        ("uint32_t", 4, Scalar::U32),
        ("int64_t", 8, Scalar::I64),
        ("uint64_t", 8, Scalar::U64),
        ("float", 4, Scalar::F32),
        ("double", 8, Scalar::F64),
        ("bool", 1, Scalar::Bool),
        ("char", 1, Scalar::Char),
    ];
    for (name, size, sc) in want {
        let l = format::layout("t", &format!("{name} x;")).unwrap();
        assert_eq!(l.size, size, "{name}");
        assert_eq!(l.fields[0].scalar(), Some(sc), "{name}");
    }
}

#[test]
fn syntax_errors_are_named() {
    let bad = |s: &str| format::layout("t", s).unwrap_err();
    assert!(matches!(bad(""), FormatError::NoFields));
    assert!(matches!(bad("uint64_t;"), FormatError::BadField { .. }));
    assert!(matches!(bad("uint64_t  a;"), FormatError::BadField { .. }));
    assert!(matches!(
        bad("uint64_t a;;uint8_t b;"),
        FormatError::BadField { .. }
    ));
    assert!(matches!(bad("float[0] a;"), FormatError::BadArray { .. }));
    assert!(matches!(
        bad("float[70000] a;"),
        FormatError::BadArray { .. }
    ));
    assert!(matches!(bad("float[3 a;"), FormatError::BadArray { .. }));
    assert!(matches!(bad("float a-b;"), FormatError::BadName { .. }));
    assert!(matches!(bad("other a;"), FormatError::UnknownType { .. }));
    assert!(matches!(
        bad("float a;float a;"),
        FormatError::DuplicateField { .. }
    ));
}

#[test]
fn nested_types_resolve_in_any_order_and_cycles_are_refused() {
    let defs = [
        ("outer", "uint64_t timestamp;inner[2] items;uint8_t tail;"),
        ("inner", "uint32_t a;uint8_t[4] _padding0;"),
    ];
    let m = format::resolve(&defs).unwrap();
    let outer = &m["outer"];
    assert_eq!(m["inner"].size, 8);
    assert_eq!(m["inner"].size_no_trailing_padding, 4);
    assert_eq!(outer.size, 8 + 16 + 1);
    assert_eq!(outer.fields[1].size, 16);
    assert_eq!(outer.fields[2].offset, 24);
    // Nested padding is kept: only top-level trailing padding is dropped.
    let defs = [
        ("o", "uint64_t timestamp;inner i;"),
        ("inner", "uint32_t a;uint8_t[4] _padding0;"),
    ];
    assert_eq!(
        format::resolve(&defs).unwrap()["o"].size_no_trailing_padding,
        16
    );

    let cyc = [("a", "b x;"), ("b", "a y;")];
    assert!(matches!(
        format::resolve(&cyc),
        Err((_, FormatError::Cycle { .. }))
    ));
    let missing = [("a", "nothere x;")];
    assert!(matches!(
        format::resolve(&missing),
        Err((0, FormatError::UnknownType { .. }))
    ));
}

fn topic(name: &'static str, format: &'static str, record_len: usize) -> Topic {
    Topic {
        name,
        format,
        record_len,
    }
}

#[test]
fn the_topic_checker_accepts_a_px4_style_topic() {
    let t = topic(
        "oxide_probe",
        "uint64_t timestamp;float[3] gyro_rad;uint8_t[4] _padding0;",
        20,
    );
    format::check_topic(&t).unwrap();
    format::assert_topic(&t);
}

#[test]
fn the_topic_checker_names_each_rule() {
    let err = |t: Topic| format::check_topic(&t).unwrap_err();
    // Declared length differs from the string.
    assert!(
        err(topic(
            "t",
            "uint64_t timestamp;float x;uint8_t[4] _padding0;",
            16
        ))
        .contains("record_len")
    );
    // The unpadded length counts padding in the middle but not at the end.
    format::check_topic(&topic(
        "t",
        "uint64_t timestamp;uint8_t a;uint8_t[3] _padding0;uint32_t b;",
        16,
    ))
    .unwrap();
    // timestamp must be the first field and a uint64_t.
    assert!(
        err(topic(
            "t",
            "float x;uint64_t timestamp;uint8_t[4] _padding0;",
            12
        ))
        .contains("timestamp")
    );
    assert!(err(topic("t", "uint32_t timestamp;uint32_t x;", 8)).contains("timestamp"));
    // Padding fields are uint8_t arrays.
    assert!(err(topic("t", "uint64_t timestamp;float _padding0;", 8)).contains("padding"));
    // Natural alignment and a size that is a multiple of 8 (PX4's generator).
    assert!(
        err(topic(
            "t",
            "uint64_t timestamp;uint8_t a;uint32_t b;uint8_t[3] _padding0;",
            13
        ))
        .contains("align")
    );
    assert!(err(topic("t", "uint64_t timestamp;uint32_t b;", 12)).contains("multiple of 8"));
    // The name must be valid.
    assert!(err(topic("bad name", "uint64_t timestamp;", 8)).contains("name"));
}
