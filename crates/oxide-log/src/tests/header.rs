// SPDX-License-Identifier: AGPL-3.0-or-later
//! Step 3: the definitions section.

use super::topics::{quad, sample_records};
use crate::header::{BuildInfo, Header, Level, NestedFormat, write_definitions};
use crate::msg::{Error, ParamValue, SliceSink};
use crate::reader::UlogFile;
use crate::record::Topic;
use crate::topics::{NESTED, NORMAL};
use oxide_core::config::VehicleConfig;
use oxide_core::params::{self, ParamId};
use oxide_core::schedule::F405_LEAN;
use std::process::Command;

const BUILD: BuildInfo<'static> = BuildInfo {
    git_hash: "3f2a9c1d0e5b7a8c9d0e1f2a3b4c5d6e7f8a9b0c",
    board: "SIM",
};

fn header_with(
    cfg: &VehicleConfig<4>,
    level: Level,
    nested: &[NestedFormat],
    topics: &[Topic],
) -> Vec<u8> {
    let h = Header {
        start_us: 1_234_567,
        build: BUILD,
        profile: "f405-lean",
        level,
        nested,
    };
    let mut out = Vec::new();
    write_definitions(&mut out, &h, cfg, topics).unwrap();
    out
}

fn normal(cfg: &VehicleConfig<4>) -> Vec<u8> {
    header_with(cfg, Level::Normal, NESTED, NORMAL)
}

#[test]
fn the_output_parses_and_says_what_it_was_given() {
    let cfg = quad();
    let f = UlogFile::parse(&normal(&cfg)).unwrap();
    assert_eq!(f.start_us, 1_234_567);
    assert_eq!(f.version, 1);
    assert_eq!(f.compat_flags, [0; 8]);
    assert_eq!(f.incompat_flags, [0; 8]);

    // Formats: the topics, in list order, as declared.
    let formats: Vec<(&str, &str)> = f
        .formats
        .iter()
        .map(|d| (d.name.as_str(), d.fields.as_str()))
        .collect();
    let want: Vec<(&str, &str)> = NORMAL.iter().map(|t| (t.name, t.format)).collect();
    assert_eq!(formats, want);

    // Subscriptions: one per topic, id = position, instance 0.
    assert_eq!(f.subscriptions.len(), NORMAL.len());
    for (i, (s, t)) in f.subscriptions.iter().zip(NORMAL).enumerate() {
        assert_eq!(
            (s.name.as_str(), s.msg_id, s.multi_id),
            (t.name, i as u16, 0)
        );
        assert_eq!(f.formats[s.format].name, t.name);
    }

    // Information.
    assert_eq!(f.info_str("sys_name"), Some("Oxide"));
    assert_eq!(f.info_str("ver_sw"), Some(BUILD.git_hash));
    assert_eq!(f.info_str("ver_hw"), Some("SIM"));
    assert_eq!(f.info_str("oxide_profile"), Some("f405-lean"));
    assert_eq!(f.info_str("oxide_log_level"), Some("normal"));
    let hash = f.info("oxide_config_hash").unwrap();
    assert_eq!(hash.ty, "uint64_t");
    assert_eq!(hash.as_u64(), Some(cfg.config_hash()));
    assert_eq!(f.info.len(), 6);

    // Nothing but definitions: no data, no change.
    assert!(f.items.is_empty());
    assert!(f.default_params.is_empty());
}

#[test]
fn all_the_parameters_are_there_with_their_exact_values() {
    let mut cfg = quad();
    // Move a few off their defaults, including one held in microseconds and
    // an integer.
    for (name, v) in [
        ("RATE_ROLL_P", ParamValue::F32(0.123_456_79)),
        ("EKF_SOURCE", ParamValue::I32(1)),
    ] {
        let id = params::find(name).unwrap();
        let v = match v {
            ParamValue::F32(x) => params::ParamValue::F32(x),
            ParamValue::I32(x) => params::ParamValue::I32(x),
        };
        params::set(&mut cfg, &F405_LEAN, id, v).unwrap();
    }
    let f = UlogFile::parse(&normal(&cfg)).unwrap();
    assert_eq!(params::COUNT, 98);
    assert_eq!(f.initial_params.len(), 98);
    for (i, (p, def)) in f.initial_params.iter().zip(params::PARAMS).enumerate() {
        assert_eq!(p.name, def.name, "table order");
        let want = params::get(&cfg, ParamId(i as u16)).unwrap();
        match (p.value, want) {
            (ParamValue::F32(a), params::ParamValue::F32(b)) => {
                assert_eq!(a.to_bits(), b.to_bits(), "{}", def.name)
            }
            (ParamValue::I32(a), params::ParamValue::I32(b)) => assert_eq!(a, b, "{}", def.name),
            (got, want) => panic!("{}: {got:?} but {want:?}", def.name),
        }
    }
    let value = |n: &str| f.initial_params.iter().find(|p| p.name == n).unwrap().value;
    assert_eq!(value("RATE_ROLL_P"), ParamValue::F32(0.123_456_79));
    assert_eq!(value("EKF_SOURCE"), ParamValue::I32(1));
}

#[test]
fn two_configurations_one_parameter_apart_differ_in_that_p_and_the_hash() {
    let a = quad();
    let mut b = quad();
    let id = params::find("ATT_ROLL_P").unwrap();
    let old = match params::get(&a, id).unwrap() {
        params::ParamValue::F32(x) => x,
        other => panic!("{other:?}"),
    };
    params::set(&mut b, &F405_LEAN, id, params::ParamValue::F32(old + 0.5)).unwrap();
    let (fa, fb) = (
        UlogFile::parse(&normal(&a)).unwrap(),
        UlogFile::parse(&normal(&b)).unwrap(),
    );
    let differing: Vec<&str> = fa
        .initial_params
        .iter()
        .zip(&fb.initial_params)
        .filter(|(x, y)| x != y)
        .map(|(x, _)| x.name.as_str())
        .collect();
    assert_eq!(differing, ["ATT_ROLL_P"]);
    assert_ne!(
        fa.info("oxide_config_hash").unwrap().as_u64(),
        fb.info("oxide_config_hash").unwrap().as_u64()
    );
    // And nothing else differs: same formats, same other information.
    assert_eq!(fa.formats, fb.formats);
    assert_eq!(fa.subscriptions, fb.subscriptions);
    for k in [
        "sys_name",
        "ver_sw",
        "ver_hw",
        "oxide_profile",
        "oxide_log_level",
    ] {
        assert_eq!(fa.info(k), fb.info(k), "{k}");
    }
}

#[test]
fn the_vehicle_description_is_in_the_hash_though_it_is_not_a_parameter() {
    let a = quad();
    let mut b = quad();
    b.mass_kg += 0.01;
    let ha = UlogFile::parse(&normal(&a)).unwrap();
    let hb = UlogFile::parse(&normal(&b)).unwrap();
    assert_eq!(ha.initial_params, hb.initial_params);
    assert_ne!(
        ha.info("oxide_config_hash").unwrap().as_u64(),
        hb.info("oxide_config_hash").unwrap().as_u64()
    );
}

#[test]
fn the_replay_level_says_so_and_takes_a_longer_list() {
    const EXTRA: Topic = Topic {
        name: "oxide_extra",
        format: "uint64_t timestamp;uint32_t a;uint8_t[4] _padding0;",
        record_len: 12,
    };
    let mut list: Vec<Topic> = NORMAL.to_vec();
    list.push(EXTRA);
    let bytes = header_with(&quad(), Level::Replay, NESTED, &list);
    let f = UlogFile::parse(&bytes).unwrap();
    assert_eq!(f.info_str("oxide_log_level"), Some("replay"));
    assert_eq!(f.subscriptions.len(), NORMAL.len() + 1);
    let last = f.subscriptions.last().unwrap();
    assert_eq!(
        (last.name.as_str(), last.msg_id),
        ("oxide_extra", NORMAL.len() as u16)
    );
}

#[test]
fn nested_formats_come_before_the_topics_that_use_them() {
    const SUB: NestedFormat = NestedFormat {
        name: "oxide_sub",
        fields: "uint32_t a;float[2] b;",
    };
    const OUTER: Topic = Topic {
        name: "oxide_outer",
        format: "uint64_t timestamp;oxide_sub[2] items;uint8_t flag;uint8_t[7] _padding0;",
        record_len: 8 + 24 + 1,
    };
    let bytes = header_with(&quad(), Level::Normal, &[SUB], &[OUTER]);
    let f = UlogFile::parse(&bytes).unwrap();
    let names: Vec<&str> = f.formats.iter().map(|d| d.name.as_str()).collect();
    assert_eq!(names, ["oxide_sub", "oxide_outer"]);
    assert_eq!(f.subscriptions.len(), 1);
    assert_eq!(f.subscriptions[0].name, "oxide_outer");
}

#[test]
fn the_order_is_flags_formats_information_parameters_subscriptions() {
    let bytes = normal(&quad());
    // Message types in file order, after the 16-byte file header.
    let mut kinds = String::new();
    let mut at = 16;
    while at + 3 <= bytes.len() {
        let len = usize::from(u16::from_le_bytes([bytes[at], bytes[at + 1]]));
        let k = bytes[at + 2] as char;
        if !kinds.ends_with(k) {
            kinds.push(k);
        }
        at += 3 + len;
    }
    assert_eq!(at, bytes.len(), "whole messages");
    assert_eq!(kinds, "BFIPA");
}

#[test]
fn an_empty_name_is_not_left_empty() {
    let h = Header {
        start_us: 0,
        build: BuildInfo {
            git_hash: "",
            board: "",
        },
        profile: "",
        level: Level::Normal,
        nested: NESTED,
    };
    let mut out = Vec::new();
    write_definitions(&mut out, &h, &quad(), NORMAL).unwrap();
    let f = UlogFile::parse(&out).unwrap();
    for k in ["ver_sw", "ver_hw", "oxide_profile"] {
        assert_eq!(f.info_str(k), Some("unknown"), "{k}");
    }
}

#[test]
fn a_sink_that_is_too_small_gives_an_error_not_a_panic() {
    let h = Header {
        start_us: 0,
        build: BUILD,
        profile: "p",
        level: Level::Normal,
        nested: NESTED,
    };
    let mut whole = Vec::new();
    write_definitions(&mut whole, &h, &quad(), NORMAL).unwrap();
    let full = whole.len();
    for cap in [0, 10, 100, full / 2, full - 1] {
        let mut buf = vec![0u8; cap];
        let mut s = SliceSink::new(&mut buf);
        assert_eq!(
            write_definitions(&mut s, &h, &quad(), NORMAL),
            Err(Error::Full),
            "{cap}"
        );
    }
    let mut buf = vec![0u8; full + 16];
    let mut s = SliceSink::new(&mut buf);
    write_definitions(&mut s, &h, &quad(), NORMAL).unwrap();
}

#[test]
fn a_topic_name_that_is_not_a_name_is_refused() {
    let bad = Topic {
        name: "has space",
        format: "uint64_t timestamp;",
        record_len: 8,
    };
    let h = Header {
        start_us: 0,
        build: BUILD,
        profile: "p",
        level: Level::Normal,
        nested: NESTED,
    };
    let mut out = Vec::new();
    assert_eq!(
        write_definitions(&mut out, &h, &quad(), &[bad]),
        Err(Error::BadName)
    );
}

// ---- pyulog ----

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
fn pyulog_reads_a_header_and_one_record_of_each_topic_without_corruption() {
    let Some(py) = python() else { return };
    let cfg = quad();
    let mut bytes = normal(&cfg);
    for r in sample_records() {
        bytes.extend_from_slice(&r);
    }
    let path = std::env::temp_dir().join(format!("oxide-log-header-{}.ulg", std::process::id()));
    std::fs::write(&path, &bytes).unwrap();
    let p = path.to_str().unwrap();

    let (ok, out, err) = run(&py, &[p]);
    assert!(ok, "ulog_check.py failed: {err}\n{out}");
    for t in NORMAL {
        assert!(
            out.contains(&format!("topic {} 0 1\n", t.name)),
            "{}: {out}",
            t.name
        );
    }

    let (ok, meta, err) = run(&py, &[p, "--meta"]);
    assert!(ok, "{err}");
    let lines: Vec<&str> = meta.lines().collect();
    assert!(lines.contains(&"start 1234567"), "{meta}");
    assert!(lines.contains(&"info sys_name Oxide"));
    assert!(lines.contains(&format!("info ver_sw {}", BUILD.git_hash).as_str()));
    assert!(lines.contains(&"info ver_hw SIM"));
    assert!(lines.contains(&"info oxide_profile f405-lean"));
    assert!(lines.contains(&"info oxide_log_level normal"));
    assert!(lines.contains(&format!("info oxide_config_hash {}", cfg.config_hash()).as_str()));
    let params_seen = lines.iter().filter(|l| l.starts_with("param ")).count();
    assert_eq!(params_seen, 98);
    for (i, def) in params::PARAMS.iter().enumerate() {
        let text = match params::get(&cfg, ParamId(i as u16)).unwrap() {
            params::ParamValue::F32(x) => format!("0x{:08x}", x.to_bits()),
            params::ParamValue::I32(x) => x.to_string(),
        };
        assert!(
            lines.contains(&format!("param {} {text}", def.name).as_str()),
            "{}",
            def.name
        );
    }
    let _ = std::fs::remove_file(path);
}
