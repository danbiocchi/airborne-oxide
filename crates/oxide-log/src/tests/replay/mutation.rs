// SPDX-License-Identifier: AGPL-3.0-or-later
//! A walk by mutation: change each field of each type on its own, send it
//! through the encoder and the decoder, and require that exactly that change
//! comes back. A field the encoder or the decoder leaves out fails here; a
//! field added to a core type fails to compile in `wire.rs` (`flat` and
//! `unflat` name every field and use no `..`), and a field added to a
//! topic's `Flat` list without the encoder fails here.

use super::samples::*;
use super::wire::*;
use crate::replay::decode;
use oxide_core::time::Micros;

/// For every field and each of its two alternatives: the mutated value goes
/// through and comes back as the mutation, which is not the original.
fn walk<W: Wire>(name: &str, originals: &[W]) -> usize {
    let mut checked = 0;
    for orig in originals {
        let flat = orig.flat();
        let base = vals(orig);
        assert_eq!(vals(&orig.through()), base, "{name}: unchanged value");
        for (k, f) in flat.iter().enumerate() {
            for alt in f.alts {
                if alt == f.v {
                    continue;
                }
                let mut v = base.clone();
                v[k] = alt;
                let mutated: W = build(&v);
                let back = vals(&mutated.through());
                assert_eq!(
                    back, v,
                    "{name}: field {k} changed to {alt:#x} came back wrong"
                );
                assert_ne!(back, base, "{name}: field {k} is not recorded");
                checked += 1;
            }
        }
    }
    checked
}

#[test]
fn every_field_of_every_input_is_recorded() {
    walk("imu", &[imu()]);
    walk("state", &[state()]);
    walk("rc", &[rc(), rc_other()]);
    walk("battery", &[battery()]);
    walk("setpoint", &[setpoint()]);
    walk("gps", &[gps(), gps_nofix()]);
    walk("baro", &[baro()]);
    walk("mag", &[mag()]);
    walk("gcs", &[gcs()]);
    walk("brain", &[brain()]);
    walk("param_set", &[set_f32(), set_i32()]);
}

#[test]
fn the_walk_counts_the_fields_it_visited() {
    // Two stamps and 6 floats for the IMU, each with two alternatives.
    assert_eq!(walk("imu", &[imu()]), (2 + 6) * 2);
    // Two stamps and 13 floats.
    assert_eq!(walk("state", &[state()]), (2 + 13) * 2);
    // Two stamps, 4 floats, 2 switches, the mode switch, 2 flags.
    assert_eq!(walk("rc", &[rc()]), (2 + 4 + 2 + 1 + 2) * 2);
    // A heartbeat has only its two stamps.
    assert_eq!(walk("gcs", &[gcs()]), 2 * 2);
}

#[test]
fn every_field_of_the_tick_boot_and_output_records_is_recorded() {
    walk(
        "tick",
        &[decode::TickRecord {
            t: Micros(12),
            seq: 99,
            present: 0x0123,
            param_ack: false,
            saved: true,
        }],
    );
    walk(
        "boot",
        &[decode::Boot {
            t: Micros(5),
            report: oxide_core::params::LoadReport {
                applied: 90,
                unknown: 1,
                dropped: 2,
                invalid: false,
                unreadable: true,
                torn: 3,
            },
            rotors: 4,
            base_period_us: 1000,
            filter_period_ticks: 4,
            task_period_ticks: [1, 20, 20, 4, 1],
        }],
    );
    let mut outs = Vec::new();
    for status in [oxide_core::io::Status::default(), busy_status()] {
        for events in [0u8, 1, 2, 3] {
            outs.push(decode::OutRecord::<4> {
                t: Micros(1),
                t_valid: Micros(2),
                motors: [0.1, 0.2, 0.3, 0.4],
                status,
                events,
            });
        }
    }
    walk("out", &outs);
}
