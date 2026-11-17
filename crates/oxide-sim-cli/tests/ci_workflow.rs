// SPDX-License-Identifier: AGPL-3.0-or-later
//! The CI workflow file must be valid for GitHub, which no tool here checks.

/// GitHub allows the `runner` context only in a step (`steps[*].env`, `run`,
/// ...), not in a job's own `env:`. A workflow that does is rejected whole.
#[test]
fn runner_context_is_not_used_in_a_job_level_env() {
    let text = include_str!("../../../.github/workflows/ci.yml");
    let lines: Vec<&str> = text.lines().collect();
    let indent = |l: &str| l.len() - l.trim_start().len();
    for (n, line) in lines.iter().enumerate() {
        if !line.contains("${{") || !line.contains("runner.") {
            continue;
        }
        // The nearest earlier line indented less than this one is its parent.
        let parent = lines[..n]
            .iter()
            .rev()
            .find(|l| !l.trim().is_empty() && indent(l) < indent(line))
            .expect("a parent key");
        let job_level = indent(parent) == 4 && parent.trim() == "env:";
        assert!(
            !job_level,
            "line {}: `runner` context in a job-level env: {line}",
            n + 1
        );
    }
}
