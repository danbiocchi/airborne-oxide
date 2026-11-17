// SPDX-License-Identifier: AGPL-3.0-or-later
//! Step 3: `pyulog` reads a file of replay records without corruption, and
//! the values it reads equal ours. Skipped, with a printed notice, unless the
//! environment variable `OXIDE_PYTHON` names a Python that has `pyulog`.

use super::harness::*;
use super::samples::*;
use crate::reader::UlogFile;
use crate::replay::{self, TOPICS};
use oxide_core::io::Outputs;
use oxide_core::params::LoadReport;
use oxide_core::schedule::H743_FULL;
use oxide_core::time::Micros;
use std::process::Command;

/// A header, then one record of every replay topic and nothing else.
fn one_of_everything() -> Vec<u8> {
    let t = T;
    let full = full_inputs();
    let mut b = Builder::new();
    b.info_u64("oxide_config_hash", quad().config_hash());
    b.push(&replay::VEHICLE, |w| {
        replay::fill_vehicle_number(w, t, 0, "mass_kg", 0.65);
    });
    b.push(&replay::MICROS, |w| {
        replay::fill_micros(w, t, "EKF_MAG_TMO", 16_000_001)
    });
    b.push(&replay::BOOT, |w| {
        replay::fill_boot(w, t, &LoadReport::default(), 4, &H743_FULL).unwrap();
    });
    b.push(&replay::IN_TICK, |w| {
        replay::fill_tick(w, t, 0, &full, true)
    });
    b.push(&replay::IN_IMU, |w| replay::fill_imu(w, t, &imu()));
    b.push(&replay::IN_STATE, |w| replay::fill_state(w, t, &state()));
    b.push(&replay::IN_RC, |w| replay::fill_rc(w, t, &rc()));
    b.push(&replay::IN_BATTERY, |w| {
        replay::fill_battery(w, t, &battery())
    });
    b.push(&replay::IN_SETPOINT, |w| {
        replay::fill_setpoint(w, t, &setpoint())
    });
    b.push(&replay::IN_GPS, |w| replay::fill_gps(w, t, &gps()));
    b.push(&replay::IN_BARO, |w| replay::fill_baro(w, t, &baro()));
    b.push(&replay::IN_MAG, |w| replay::fill_mag(w, t, &mag()));
    b.push(&replay::IN_GCS, |w| replay::fill_gcs(w, t, &gcs()));
    b.push(&replay::IN_BRAIN, |w| replay::fill_brain(w, t, &brain()));
    b.push(&replay::IN_PARAM_SET, |w| {
        replay::fill_param_set(w, t, &set_f32())
    });
    let out = Outputs::<4> {
        motors: [NAN_PAYLOAD, -0.0, 0.5, f32::INFINITY],
        status: busy_status(),
        ..Outputs::off(Micros(u64::MAX))
    };
    b.push(&replay::OUT, |w| replay::fill_out(w, t, &out).unwrap());
    b.bytes()
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

/// One cell of `ulog_check.py --dump`, as it prints it, from our reader.
fn cell(file: &UlogFile, topic: &str, col: &str) -> String {
    let (name, ty) = col.split_once(':').unwrap();
    let r = file.records(topic).next().unwrap();
    match ty {
        "float" => format!("0x{:08x}", r.f32(name).unwrap().to_bits()),
        "uint8_t" => r.u8(name).unwrap().to_string(),
        "uint16_t" => r.u16(name).unwrap().to_string(),
        "uint32_t" => r.u32(name).unwrap().to_string(),
        "uint64_t" => r.u64(name).unwrap().to_string(),
        "bool" => i8::from(r.bool(name).unwrap()).to_string(),
        "char" => (r.char(name).unwrap() as i8).to_string(),
        other => panic!("{other}"),
    }
}

#[test]
fn pyulog_reads_every_replay_record_without_corruption_and_with_our_values() {
    let Some(py) = python() else { return };
    let bytes = one_of_everything();
    let path = std::env::temp_dir().join(format!("oxide-log-replay-{}.ulg", std::process::id()));
    std::fs::write(&path, &bytes).unwrap();
    let p = path.to_str().unwrap();
    let file = UlogFile::parse(&bytes).unwrap();

    let (ok, out, err) = run(&py, &[p]);
    assert!(ok, "ulog_check.py failed: {err}\n{out}");
    for t in &TOPICS {
        assert!(
            out.contains(&format!("topic {} 0 1\n", t.name)),
            "{}: {out}",
            t.name
        );
    }

    for t in &TOPICS {
        let (ok, csv, err) = run(&py, &[p, "--dump", t.name]);
        assert!(ok, "{err}");
        let mut lines = csv.lines();
        let header = lines.next().unwrap();
        let row: Vec<&str> = lines.next().unwrap().split(',').collect();
        assert!(lines.next().is_none());
        let cols: Vec<&str> = header.split(',').collect();
        assert_eq!(cols.len(), row.len());
        for (col, got) in cols.iter().zip(&row) {
            assert_eq!(*got, cell(&file, t.name, col), "{} {col}", t.name);
        }
        // No trailing padding in what pyulog sees; the first column is the tick time.
        assert!(cols[0] == "timestamp:uint64_t", "{header}");
        assert_eq!(row[0], T.0.to_string());
    }

    // The interesting bits are what we put in.
    let (_, csv, _) = run(&py, &[p, "--dump", "oxide_in_imu"]);
    let row: Vec<&str> = csv.lines().nth(1).unwrap().split(',').collect();
    assert_eq!(row[3], "0x7fc01234", "NaN payload survives pyulog");
    assert_eq!(row[4], "0x80000000", "negative zero survives pyulog");
    let (_, meta, err) = run(&py, &[p, "--meta"]);
    assert!(meta.contains("info oxide_config_hash"), "{meta}{err}");
    assert!(!meta.contains("dropout"), "{meta}");
    let _ = std::fs::remove_file(path);
}

#[test]
fn the_bytes_a_tick_costs() {
    // A message is 3 bytes of header, 2 of message id, then the record.
    let msg = |t: &crate::record::Topic| 5 + t.record_len;
    let tick_only = msg(&replay::IN_TICK) + msg(&replay::OUT);
    let with_imu = tick_only + msg(&replay::IN_IMU);
    let every = tick_only
        + [
            &replay::IN_IMU,
            &replay::IN_STATE,
            &replay::IN_RC,
            &replay::IN_BATTERY,
            &replay::IN_SETPOINT,
            &replay::IN_GPS,
            &replay::IN_BARO,
            &replay::IN_MAG,
            &replay::IN_GCS,
            &replay::IN_BRAIN,
            &replay::IN_PARAM_SET,
        ]
        .iter()
        .map(|t| msg(t))
        .sum::<usize>();
    assert_eq!((tick_only, with_imu, every), (112, 165, 617));
    // And the encoders write exactly that many bytes.
    let mut v = Vec::new();
    let full = full_inputs();
    crate::record::data(&mut v, 0, &replay::IN_TICK, |w| {
        replay::fill_tick(w, T, 0, &full, false);
    })
    .unwrap();
    assert_eq!(v.len(), msg(&replay::IN_TICK));
}
