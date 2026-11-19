// SPDX-License-Identifier: AGPL-3.0-or-later
//! The report and its two figures for a small table, byte for byte. The
//! golden files are in `tests/gate_report/`; to see a difference, compare the
//! file written under the name in the failure message.

#[path = "support/gate_fixture.rs"]
mod fx;

use std::path::PathBuf;

use fx::{AC, F405, Fixture};
use oxide_bridge::gate::report::{cumulative_svg, markdown, scatter_svg, write};
use oxide_bridge::gate::verdict::{Evaluation, Verdict, verdict_at};

fn golden(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/gate_report")
        .join(name)
}

fn small() -> Evaluation {
    let mut f = Fixture::small("report");
    // Make the tables worth a picture: varied errors, one crash of ArduCopter,
    // one flight of ours lost to it, a motor that never stops.
    for i in 0..20 {
        f.row("gust-hold", F405, i).set(
            "rms_h_m",
            &format!("{}", 0.14 + f64::from(i as u32) * 0.002),
        );
        f.row("gust-hold", AC, i).set(
            "rms_h_m",
            &format!("{}", 0.15 + f64::from((i * 7 % 20) as u32) * 0.0025),
        );
    }
    f.row("gust-hold", AC, 4).set("class", "crash");
    f.row("gps-loss", AC, 0).set("class", "fly-away");
    f.row("motor-loss", AC, 1).set("last_motor_on_s", "59.995");
    // What the report holds and decides nothing with: a landing 5 m away
    // (3 north, 4 east), one 10 m away (6 south, 8 west), one 1 m away with no
    // touchdown, one never disarmed.
    for (i, (n, e, tilt, touch, disarmed)) in [
        ("3", "4", "10", "0.5", "12.5"),
        ("-6", "-8", "20", "1.5", "14"),
        ("0", "-1", "35.25", "", ""),
    ]
    .into_iter()
    .enumerate()
    {
        let r = f.row("gps-loss", F405, i as u64);
        r.set("end_n_m", n);
        r.set("end_e_m", e);
        r.set("max_tilt_deg", tilt);
        r.set("touchdown_mps", touch);
        r.set("disarmed_at_s", disarmed);
    }
    f.write();
    let v = verdict_at(&f.dir, Some(&f.info.commit));
    f.remove();
    let Verdict::Evaluated(mut e) = v else {
        panic!("the tables should be complete");
    };
    // The golden files do not hold the hashes of the repository's own files
    // (which change when a scenario or a parameter file does): the report
    // prints gate.txt as it is, so the hashes are fixed here.
    e.info.commit = "0123456789abcdef0123456789abcdef01234567".into();
    e.info.params = vec![("base.parm".into(), "aa".into())];
    e.info.binary_sha256 = Some("b1".repeat(32));
    for (name, sha) in &mut e.info.scenario_files {
        *sha = format!("{:064x}", oxide_sim::rng::run_seed(name, 99));
    }
    *e
}

fn check(name: &str, got: &str) {
    let path = golden(name);
    // A checkout that turned line ends into CR LF is still the same text.
    let want = std::fs::read_to_string(&path)
        .unwrap_or_default()
        .replace("\r\n", "\n");
    if want != got {
        let new = std::env::temp_dir().join(format!("oxide-gate-report-{name}"));
        std::fs::write(&new, got).expect("write");
        panic!(
            "{name} differs from {}: the new text is in {}",
            path.display(),
            new.display()
        );
    }
}

#[test]
fn the_markdown_report_of_a_small_table_is_this_text_byte_for_byte() {
    check("report.md", &markdown(&small(), "report"));
}

#[test]
fn the_cumulative_curves_are_this_svg_byte_for_byte() {
    check("report-cumulative.svg", &cumulative_svg(&small()));
}

#[test]
fn the_scatter_plot_is_this_svg_byte_for_byte() {
    check("report-scatter.svg", &scatter_svg(&small()));
}

#[test]
fn the_same_tables_twice_give_the_same_bytes() {
    let (a, b) = (small(), small());
    assert_eq!(markdown(&a, "r"), markdown(&b, "r"));
    assert_eq!(cumulative_svg(&a), cumulative_svg(&b));
    assert_eq!(scatter_svg(&a), scatter_svg(&b));
}

#[test]
fn the_report_says_the_verdict_each_criterion_and_what_the_record_asks_for() {
    let md = markdown(&small(), "report");
    for needle in [
        "**Verdict: NOT PASSED**",
        "### A1: HOLDS",
        "### A2: HOLDS",
        "### B1: HOLDS",
        "### B2: HOLDS",
        "### B3: HOLDS",
        "### Repeats: HOLDS",
        "| core-f405 |",
        "| arducopter |",
        "| core-h743 |",
        "| core-f405-truth |",
        "95th percentile",
        "Lower `rms_h_m`",
        "| gps-loss | on the outcome |",
        "| hover-step | not scored |",
        "| motor-loss | on the time to motors off |",
        "## Flights flown twice",
        "```text\noxide-bridge gate 1\n",
        "(report-cumulative.svg)",
        "(report-scatter.svg)",
    ] {
        assert!(md.contains(needle), "{needle:?} is not in\n{md}");
    }
}

#[test]
fn the_figures_are_plain_svg_text_with_the_four_curves_and_the_diagonal() {
    let e = small();
    let c = cumulative_svg(&e);
    assert!(c.starts_with("<svg ") && c.ends_with("</svg>\n"));
    assert_eq!(c.matches("<polyline").count(), 4);
    let s = scatter_svg(&e);
    // 20 indices; ArduCopter's flight of index 4 is a crash and is not drawn.
    assert_eq!(s.matches("<circle").count(), 19);
    assert!(c.contains("arducopter (19 safe of 20)"));
    assert!(c.contains("core-f405 (20 flights)"));
    assert!(s.contains("stroke-dasharray"));
    assert!(c.is_ascii() && s.is_ascii());
}

#[test]
fn write_makes_the_report_and_two_figures_beside_it() {
    let dir = std::env::temp_dir().join(format!("oxide-gate-report-out-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    let paths = write(&small(), &dir.join("gate-report.md")).expect("writes");
    assert_eq!(paths.len(), 3);
    assert!(dir.join("gate-report.md").is_file());
    assert!(dir.join("gate-report-cumulative.svg").is_file());
    assert!(dir.join("gate-report-scatter.svg").is_file());
    let md = std::fs::read_to_string(dir.join("gate-report.md")).expect("md");
    assert!(md.contains("(gate-report-cumulative.svg)"));
    let _ = std::fs::remove_dir_all(&dir);
}

// ---- what the report holds for each scenario and decides nothing with -----

use oxide_bridge::gate::verdict::{Class, Flight, Judged, reported_of};

/// A flight with the cells the table of "reported" reads.
fn flight(
    tilt: Option<f64>,
    touchdown: Option<f64>,
    disarmed: Option<f64>,
    end: (f64, f64),
) -> Flight {
    Flight {
        scenario: "s".into(),
        autopilot: "a".into(),
        index: 0,
        class: Class::Crash,
        judged: Judged::Fail,
        rms_h_m: None,
        rms_v_m: None,
        max_tilt_deg: tilt,
        last_motor_on_s: None,
        touchdown_mps: touchdown,
        disarmed_at_s: disarmed,
        end_n_m: Some(end.0),
        end_e_m: Some(end.1),
        repeats: None,
        trace_hash: String::new(),
        packet_hash: String::new(),
        reply_hash: String::new(),
    }
}

#[test]
fn the_reported_numbers_of_an_odd_count_are_the_middle_value_and_the_largest() {
    // Distances 5, 13 (negative north), 10 (negative east): median 10, worst 13.
    // Touchdown speeds 1, 2, 6 and a flight with none: median 2, worst 6, 3 of 4.
    // Tilts 30, 5, 12: median 12, worst 30. One flight never disarmed.
    let flights = [
        flight(Some(30.0), Some(1.0), Some(20.0), (3.0, 4.0)),
        flight(Some(5.0), Some(2.0), Some(10.0), (-5.0, 12.0)),
        flight(Some(12.0), Some(6.0), None, (0.0, -10.0)),
        flight(Some(5.0), None, Some(15.0), (0.0, 0.0)),
    ];
    let r = reported_of("s", "a", &flights[..3]);
    assert_eq!(r.flights, 3);
    assert_eq!(
        (r.touched_down, r.touchdown_median, r.touchdown_worst),
        (3, Some(2.0), Some(6.0))
    );
    assert_eq!(
        (r.distance_median, r.distance_worst),
        (Some(10.0), Some(13.0))
    );
    assert_eq!((r.tilt_median, r.tilt_worst), (Some(12.0), Some(30.0)));
    assert_eq!(
        (r.disarmed, r.disarmed_median, r.disarmed_latest),
        (2, Some(15.0), Some(20.0))
    );
    // All four: a row without a touchdown is not a zero speed, and the even
    // counts take the mean of the two middle values.
    let r = reported_of("s", "a", &flights);
    assert_eq!(r.flights, 4);
    assert_eq!(
        (r.touched_down, r.touchdown_median, r.touchdown_worst),
        (3, Some(2.0), Some(6.0))
    );
    assert_eq!(
        (r.distance_median, r.distance_worst),
        (Some(7.5), Some(13.0))
    );
    assert_eq!((r.tilt_median, r.tilt_worst), (Some(8.5), Some(30.0)));
    assert_eq!(
        (r.disarmed, r.disarmed_median, r.disarmed_latest),
        (3, Some(15.0), Some(20.0))
    );
}

#[test]
fn a_table_with_no_value_reports_none() {
    let r = reported_of("s", "a", &[]);
    assert_eq!((r.flights, r.touched_down, r.disarmed), (0, 0, 0));
    assert_eq!(r.touchdown_median, None);
    assert_eq!(r.distance_worst, None);
    assert_eq!(r.tilt_median, None);
    assert_eq!(r.disarmed_latest, None);
    let mut f = flight(None, None, None, (1.0, 1.0));
    f.end_e_m = None;
    let r = reported_of("s", "a", &[f]);
    assert_eq!(r.flights, 1);
    assert_eq!(r.distance_median, None, "a distance needs both coordinates");
}

#[test]
fn the_report_has_a_row_for_every_scenario_but_the_gust_one_and_each_core_and_arducopter() {
    let e = small();
    let rows: Vec<(&str, &str)> = e
        .reported
        .iter()
        .map(|r| (r.scenario.as_str(), r.autopilot.as_str()))
        .collect();
    let who = ["core-f405", "core-h743", "arducopter"];
    let mut want = Vec::new();
    for s in ["gps-loss", "motor-loss", "hover-step"] {
        for a in who {
            want.push((s, a));
        }
    }
    assert_eq!(rows, want);
    let md = markdown(&e, "r");
    let at = |h: &str| {
        md.find(h)
            .unwrap_or_else(|| panic!("{h} is not in the report"))
    };
    assert!(
        at("## Time from the failure to motors off")
            < at("## Reported for each scenario, deciding nothing")
    );
    assert!(at("## Reported for each scenario, deciding nothing") < at("## Flights flown twice"));
    assert!(md.contains("| gps-loss | core-f405 | 3 | 2 | 1.0000 | 1.5000 | 5.0000 | 10.0000 | 20.0000 | 35.2500 | 2 | 13.2500 | 14.0000 |"));
}
