// SPDX-License-Identifier: AGPL-3.0-or-later
//! The report of gate 1 (ADR-0007 section 8) from an [`Evaluation`]: a
//! Markdown file and two SVG files written by this program, no external tool.
//! The same tables give the same bytes: nothing here reads the clock or the
//! machine.

use std::fmt::Write as _;
use std::path::{Path, PathBuf};

use crate::gate::plan::Scoring;
use crate::gate::stats::median;
use crate::gate::verdict::{Check, Classes, Evaluation};

/// The suffix of the cumulative curves' file, added to the report's stem.
pub const CUMULATIVE_SUFFIX: &str = "-cumulative.svg";
/// The suffix of the scatter plot's file.
pub const SCATTER_SUFFIX: &str = "-scatter.svg";

fn fixed(x: Option<f64>) -> String {
    x.map_or("-".to_string(), |v| format!("{v:.4}"))
}

fn criterion(s: &mut String, title: &str, c: &Check) {
    let _ = writeln!(s, "### {title}: {}\n", c.status.word());
    for l in &c.lines {
        let _ = writeln!(s, "- {l}");
    }
    if c.lines.is_empty() {
        let _ = writeln!(s, "- (nothing to judge)");
    }
    for f in &c.failing {
        let _ = writeln!(s, "- **fails:** {f}");
    }
    let _ = writeln!(s);
}

fn classes(c: &Classes) -> String {
    format!("{} / {} / {}", c.safe, c.crash, c.fly_away)
}

/// The Markdown report. `stem` is the report's file stem, which names the two
/// figures beside it.
pub fn markdown(e: &Evaluation, stem: &str) -> String {
    let mut s = String::new();
    let _ = writeln!(s, "# Gate 1 report\n");
    let _ = writeln!(
        s,
        "**Verdict: {}**\n",
        if e.passed { "PASSED" } else { "NOT PASSED" }
    );
    if !e.trial_reasons.is_empty() {
        let _ = writeln!(
            s,
            "This is not the gate's plan, so it is not passed whatever the numbers say:\n"
        );
        for r in &e.trial_reasons {
            let _ = writeln!(s, "- {r}");
        }
        let _ = writeln!(s);
    }
    let _ = writeln!(
        s,
        "Commit `{}`, {}.\n",
        e.info.commit,
        if e.info.tree_dirty {
            "dirty tree"
        } else {
            "clean tree"
        }
    );
    let _ = writeln!(s, "## The criteria\n");
    criterion(&mut s, "A1", &e.a1);
    criterion(&mut s, "A2", &e.a2);
    criterion(&mut s, "B1", &e.b1);
    criterion(&mut s, "B2", &e.b2);
    criterion(&mut s, "B3", &e.b3);
    criterion(&mut s, "Repeats", &e.repeats);

    let _ = writeln!(s, "## Criterion A: holding position in gusts\n");
    let _ = writeln!(
        s,
        "RMS horizontal error from the hold point, m. ArduCopter's statistics are over its safe \
         flights only; the others are over all their flights.\n"
    );
    let _ = writeln!(
        s,
        "| autopilot | flights | not safe | counted | median | 95th percentile | mean | worst | \
         vertical RMS median | vertical RMS worst | tilt median (deg) | tilt worst (deg) |"
    );
    let _ = writeln!(s, "|---|---|---|---|---|---|---|---|---|---|---|---|");
    for g in &e.gust {
        let _ = writeln!(
            s,
            "| {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} |",
            g.autopilot,
            g.flights,
            g.unsafe_flights,
            g.counted,
            fixed(g.median),
            fixed(g.p95),
            fixed(g.mean),
            fixed(g.worst),
            fixed(g.v_median),
            fixed(g.v_worst),
            fixed(g.tilt_median),
            fixed(g.tilt_worst)
        );
    }
    let _ = writeln!(
        s,
        "\nLower `rms_h_m`: core-f405 in {} indices, ArduCopter in {}, equal in {}.\n",
        e.wins.ours, e.wins.arducopter, e.wins.equal
    );
    let _ = writeln!(
        s,
        "![cumulative curves of rms_h_m]({stem}{CUMULATIVE_SUFFIX})\n"
    );
    let _ = writeln!(
        s,
        "![rms_h_m of each index, ArduCopter across, core-f405 up]({stem}{SCATTER_SUFFIX})\n"
    );

    let _ = writeln!(s, "## Criterion B: failures\n");
    let _ = writeln!(
        s,
        "Classes are safe / crash / fly-away. \"Passed\" counts the rows whose verdict column \
         says pass.\n"
    );
    let _ = writeln!(
        s,
        "| scenario | scored | core-f405 | core-f405 passed | core-h743 | core-h743 passed | \
         ArduCopter |"
    );
    let _ = writeln!(s, "|---|---|---|---|---|---|---|");
    for r in &e.scenarios {
        let how = match r.scoring {
            Scoring::Outcome => "on the outcome",
            Scoring::MotorsOff => "on the time to motors off",
            Scoring::NotScored => "not scored",
            Scoring::Gust => "gust",
        };
        let _ = writeln!(
            s,
            "| {} | {how} | {} | {} of {} | {} | {} of {} | {} |",
            r.scenario,
            classes(&r.core_f405),
            r.core_f405.passed,
            r.core_f405.flights,
            classes(&r.core_h743),
            r.core_h743.passed,
            r.core_h743.flights,
            classes(&r.arducopter)
        );
    }

    let _ = writeln!(s, "\n## Time from the failure to motors off\n");
    let _ = writeln!(
        s,
        "| scenario | failure at (s) | core-f405 median (s) | core-h743 median (s) | ArduCopter \
         median (s) |"
    );
    let _ = writeln!(s, "|---|---|---|---|---|");
    for m in &e.motors {
        let _ = writeln!(
            s,
            "| {} | {} | {} | {} | {} |",
            m.scenario,
            m.failure_start_s,
            fixed(median(&m.core_f405)),
            fixed(median(&m.core_h743)),
            fixed(median(&m.arducopter))
        );
    }

    let _ = writeln!(s, "\n## Flights flown twice\n");
    let _ = writeln!(
        s,
        "| scenario | index | repeats | hashes as in the main table |"
    );
    let _ = writeln!(s, "|---|---|---|---|");
    for t in &e.twice {
        let _ = writeln!(
            s,
            "| {} | {} | {} | {} |",
            t.scenario,
            t.index,
            match t.repeats {
                Some(true) => "yes",
                Some(false) => "no",
                None => "empty",
            },
            if t.hashes_equal { "yes" } else { "no" }
        );
    }

    let _ = writeln!(s, "\n## gate.txt\n\n```text\n{}```", e.info.to_text());
    s
}

/// The colour of an autopilot's curve.
fn colour(autopilot: &str) -> &'static str {
    match autopilot {
        "arducopter" => "#d95f02",
        "core-f405" => "#1b9e77",
        "core-h743" => "#7570b3",
        _ => "#666666",
    }
}

const W: f64 = 640.0;
const H: f64 = 480.0;
const LEFT: f64 = 64.0;
const RIGHT: f64 = 24.0;
const TOP: f64 = 28.0;
const BOTTOM: f64 = 56.0;

/// A plot area from 0 to `xmax` and 0 to `ymax`.
struct Frame {
    xmax: f64,
    ymax: f64,
}

impl Frame {
    fn x(&self, v: f64) -> f64 {
        LEFT + v / self.xmax * (W - LEFT - RIGHT)
    }

    fn y(&self, v: f64) -> f64 {
        H - BOTTOM - v / self.ymax * (H - TOP - BOTTOM)
    }

    fn open(&self, title: &str, xlabel: &str, ylabel: &str) -> String {
        let mut s = String::new();
        let _ = writeln!(
            s,
            "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"{W:.0}\" height=\"{H:.0}\" \
             viewBox=\"0 0 {W:.0} {H:.0}\" font-family=\"sans-serif\" font-size=\"12\">"
        );
        let _ = writeln!(
            s,
            "<rect width=\"{W:.0}\" height=\"{H:.0}\" fill=\"#ffffff\"/>"
        );
        let _ = writeln!(
            s,
            "<text x=\"{:.2}\" y=\"18\" text-anchor=\"middle\" fill=\"#222222\">{title}</text>",
            W / 2.0
        );
        for k in 0..=5 {
            let fx = self.xmax * f64::from(k) / 5.0;
            let fy = self.ymax * f64::from(k) / 5.0;
            let _ = writeln!(
                s,
                "<line x1=\"{:.2}\" y1=\"{:.2}\" x2=\"{:.2}\" y2=\"{:.2}\" stroke=\"#dddddd\"/>",
                self.x(fx),
                self.y(0.0),
                self.x(fx),
                self.y(self.ymax)
            );
            let _ = writeln!(
                s,
                "<line x1=\"{:.2}\" y1=\"{:.2}\" x2=\"{:.2}\" y2=\"{:.2}\" stroke=\"#dddddd\"/>",
                self.x(0.0),
                self.y(fy),
                self.x(self.xmax),
                self.y(fy)
            );
            let _ = writeln!(
                s,
                "<text x=\"{:.2}\" y=\"{:.2}\" text-anchor=\"middle\" fill=\"#444444\">{fx:.3}</text>",
                self.x(fx),
                H - BOTTOM + 16.0
            );
            let _ = writeln!(
                s,
                "<text x=\"{:.2}\" y=\"{:.2}\" text-anchor=\"end\" fill=\"#444444\">{fy:.3}</text>",
                LEFT - 6.0,
                self.y(fy) + 4.0
            );
        }
        let _ = writeln!(
            s,
            "<text x=\"{:.2}\" y=\"{:.2}\" text-anchor=\"middle\" fill=\"#222222\">{xlabel}</text>",
            (LEFT + W - RIGHT) / 2.0,
            H - 14.0
        );
        let _ = writeln!(
            s,
            "<text x=\"14\" y=\"{:.2}\" text-anchor=\"middle\" fill=\"#222222\" \
             transform=\"rotate(-90 14 {:.2})\">{ylabel}</text>",
            (TOP + H - BOTTOM) / 2.0,
            (TOP + H - BOTTOM) / 2.0
        );
        s
    }
}

/// The largest of the values, or 1 when there are none.
fn largest<'a>(values: impl Iterator<Item = &'a f64>) -> f64 {
    let m = values.fold(0.0_f64, |a, b| a.max(*b));
    if m > 0.0 { m * 1.05 } else { 1.0 }
}

/// The cumulative curves of `rms_h_m`, one for each autopilot: the fraction of
/// its flights at or below the value.
pub fn cumulative_svg(e: &Evaluation) -> String {
    let frame = Frame {
        xmax: largest(e.curves.iter().flat_map(|(_, v)| v.iter())),
        ymax: 1.0,
    };
    let mut s = frame.open(
        "Gust hold: cumulative distribution of rms_h_m",
        "rms_h_m (m)",
        "fraction of flights",
    );
    for (row, (who, values)) in e.curves.iter().enumerate() {
        let mut sorted = values.clone();
        sorted.sort_by(f64::total_cmp);
        let n = sorted.len().max(1) as f64;
        let points: Vec<String> = sorted
            .iter()
            .enumerate()
            .map(|(k, v)| format!("{:.2},{:.2}", frame.x(*v), frame.y((k + 1) as f64 / n)))
            .collect();
        let _ = writeln!(
            s,
            "<polyline fill=\"none\" stroke=\"{}\" stroke-width=\"1.5\" points=\"{}\"/>",
            colour(who),
            points.join(" ")
        );
        let ly = TOP + 14.0 + 16.0 * row as f64;
        let _ = writeln!(
            s,
            "<line x1=\"{:.2}\" y1=\"{ly:.2}\" x2=\"{:.2}\" y2=\"{ly:.2}\" stroke=\"{}\" \
             stroke-width=\"2\"/>",
            W - RIGHT - 150.0,
            W - RIGHT - 126.0,
            colour(who)
        );
        let _ = writeln!(
            s,
            "<text x=\"{:.2}\" y=\"{:.2}\" fill=\"#222222\">{who} ({} flights)</text>",
            W - RIGHT - 120.0,
            ly + 4.0,
            values.len()
        );
    }
    s.push_str("</svg>\n");
    s
}

/// One point for each index: ArduCopter's `rms_h_m` across and `core-f405`'s
/// up, with the diagonal. A point below the diagonal is an index our core won.
pub fn scatter_svg(e: &Evaluation) -> String {
    let m = largest(e.paired.iter().flat_map(|(a, b)| [a, b]));
    let frame = Frame { xmax: m, ymax: m };
    let mut s = frame.open(
        "Gust hold: one point for each index",
        "ArduCopter rms_h_m (m)",
        "core-f405 rms_h_m (m)",
    );
    let _ = writeln!(
        s,
        "<line x1=\"{:.2}\" y1=\"{:.2}\" x2=\"{:.2}\" y2=\"{:.2}\" stroke=\"#444444\" \
         stroke-dasharray=\"4 3\"/>",
        frame.x(0.0),
        frame.y(0.0),
        frame.x(m),
        frame.y(m)
    );
    for (ac, ours) in &e.paired {
        let _ = writeln!(
            s,
            "<circle cx=\"{:.2}\" cy=\"{:.2}\" r=\"2\" fill=\"{}\" fill-opacity=\"0.6\"/>",
            frame.x(*ac),
            frame.y(*ours),
            colour("core-f405")
        );
    }
    s.push_str("</svg>\n");
    s
}

/// Writes the report at `path` and the two figures beside it; returns the
/// three paths.
pub fn write(e: &Evaluation, path: &Path) -> std::io::Result<Vec<PathBuf>> {
    let stem = path.file_stem().map_or_else(
        || "report".to_string(),
        |s| s.to_string_lossy().into_owned(),
    );
    let dir = path.parent().map(Path::to_path_buf).unwrap_or_default();
    let cumulative = dir.join(format!("{stem}{CUMULATIVE_SUFFIX}"));
    let scatter = dir.join(format!("{stem}{SCATTER_SUFFIX}"));
    if !dir.as_os_str().is_empty() {
        std::fs::create_dir_all(&dir)?;
    }
    std::fs::write(path, markdown(e, &stem))?;
    std::fs::write(&cumulative, cumulative_svg(e))?;
    std::fs::write(&scatter, scatter_svg(e))?;
    Ok(vec![path.to_path_buf(), cumulative, scatter])
}
