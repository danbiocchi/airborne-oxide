// SPDX-License-Identifier: AGPL-3.0-or-later
//! The report and its two figures for a small table, byte for byte. The
//! golden files are in `tests/gate_report/`; to see a difference, compare the
//! file written under the name in the failure message.

#[path = "support/gate_fixture.rs"]
mod fx;

use std::path::PathBuf;

use fx::{AC, F405, Fixture};
use oxide_bridge::gate::report::{cumulative_svg, markdown, scatter_svg, write};
use oxide_bridge::gate::verdict::{Evaluation, Verdict, verdict};

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
    f.write();
    let v = verdict(&f.dir);
    f.remove();
    match v {
        Verdict::Evaluated(e) => *e,
        Verdict::Incomplete(i) => panic!("{i:?}"),
    }
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
    assert_eq!(s.matches("<circle").count(), 20);
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
