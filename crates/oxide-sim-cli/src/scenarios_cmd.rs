// SPDX-License-Identifier: AGPL-3.0-or-later
//! `oxide-sim scenarios DIR [--seeds N] [--profile f405|h743|both]
//! [--threads T]`: flies every scenario file of a directory (ADR-0004 §7).

use std::path::Path;
use std::time::Instant;

use oxide_sim::montecarlo::run_indexed;
use oxide_sim::scenario_file::{self, RunProfile, ScenarioFile};

/// What to fly, from the command line.
#[derive(Debug, PartialEq)]
pub struct Options {
    pub dir: String,
    /// Runs per scenario and profile; `None` = each file's `[run] seeds`.
    pub seeds: Option<u64>,
    /// Profiles; `None` = each file's `[run] profiles`.
    pub profiles: Option<Vec<RunProfile>>,
    /// Worker threads, 0 = all cores.
    pub threads: usize,
}

pub fn parse(mut it: impl Iterator<Item = String>) -> Result<Options, String> {
    let dir = it
        .next()
        .ok_or("scenarios needs the directory of the scenario files")?;
    if dir.starts_with("--") {
        return Err(format!("scenarios needs a directory first, got {dir}"));
    }
    let mut o = Options {
        dir,
        seeds: None,
        profiles: None,
        threads: 0,
    };
    while let Some(flag) = it.next() {
        let mut value = || it.next().ok_or(format!("{flag} needs a value"));
        match flag.as_str() {
            "--seeds" => {
                let v = value()?;
                let n: u64 = v
                    .parse()
                    .map_err(|_| format!("--seeds needs a positive integer, got {v}"))?;
                if n == 0 {
                    return Err("--seeds must be at least 1".into());
                }
                o.seeds = Some(n);
            }
            "--profile" => {
                o.profiles = Some(match value()?.as_str() {
                    "f405" => vec![RunProfile::F405],
                    "h743" => vec![RunProfile::H743],
                    "both" => vec![RunProfile::F405, RunProfile::H743],
                    other => return Err(format!("unknown profile {other} (f405 | h743 | both)")),
                });
            }
            "--threads" => {
                let v = value()?;
                o.threads = v
                    .parse()
                    .map_err(|_| format!("--threads needs a non-negative integer, got {v}"))?;
            }
            other => return Err(format!("unknown option {other}")),
        }
    }
    Ok(o)
}

/// Every `*.toml` of `dir`, loaded, in file-name order.
pub fn load_dir(dir: &Path) -> Result<Vec<ScenarioFile>, String> {
    let entries = std::fs::read_dir(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
    let mut paths = Vec::new();
    for entry in entries {
        let path = entry.map_err(|e| format!("{}: {e}", dir.display()))?.path();
        if path.is_file() && path.extension().is_some_and(|x| x == "toml") {
            paths.push(path);
        }
    }
    paths.sort();
    if paths.is_empty() {
        return Err(format!("{}: no *.toml scenario files", dir.display()));
    }
    paths
        .iter()
        .map(|p| scenario_file::load_path(p).map_err(|e| e.to_string()))
        .collect()
}

/// One flight to fly: file, profile, run index.
struct Job {
    file: usize,
    profile: RunProfile,
    index: u64,
}

/// The first reason a run failed, or `None` when it passed. A run that
/// cannot be set up fails.
fn first_failure(file: &ScenarioFile, job: &Job) -> Option<String> {
    match file.fly_checked(job.profile.profile(), job.index, &[]) {
        Ok(v) => v.violations.into_iter().next(),
        Err(e) => Some(format!("cannot run: {e}")),
    }
}

/// Flies the directory; true when every run passed.
pub fn run(args: impl Iterator<Item = String>) -> Result<bool, Box<dyn std::error::Error>> {
    let o = parse(args)?;
    let files = load_dir(Path::new(&o.dir))?;
    // One flight per (file, profile, index), in a fixed order, flown in
    // parallel; the lines come out in the same order whatever the schedule.
    let mut jobs = Vec::new();
    let mut groups = Vec::new();
    for (file, f) in files.iter().enumerate() {
        let profiles = o.profiles.as_deref().unwrap_or(&f.run.profiles);
        let seeds = o.seeds.unwrap_or(f.run.seeds);
        for &profile in profiles {
            groups.push((file, profile, jobs.len(), seeds as usize));
            for index in 0..seeds {
                jobs.push(Job {
                    file,
                    profile,
                    index,
                });
            }
        }
    }
    let start = Instant::now();
    let results = run_indexed(jobs.len(), o.threads, |j| {
        let job = &jobs[j];
        first_failure(&files[job.file], job)
    });
    let wall_s = start.elapsed().as_secs_f64();
    let (mut total, mut failed) = (0_usize, 0_usize);
    for (file, profile, first, n) in groups {
        let name = &files[file].name;
        let runs = results.get(first..first + n).unwrap_or(&[]);
        let bad = runs.iter().filter(|r| r.is_some()).count();
        total += n;
        failed += bad;
        println!("{name:<36} {:<5} passed {}/{n}", profile.name(), n - bad);
        if let Some((i, reason)) = runs
            .iter()
            .enumerate()
            .find_map(|(i, r)| r.as_ref().map(|r| (i, r)))
        {
            println!("    first failure, run {i}: {reason}");
        }
    }
    println!(
        "{} files, {total} flights, {} passed, {failed} failed in {wall_s:.1} s",
        files.len(),
        total - failed
    );
    Ok(failed == 0)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn args(a: &[&str]) -> impl Iterator<Item = String> {
        a.iter()
            .map(|s| (*s).to_string())
            .collect::<Vec<_>>()
            .into_iter()
    }

    #[test]
    fn options_parse() {
        let o = parse(args(&[
            "d",
            "--seeds",
            "3",
            "--profile",
            "both",
            "--threads",
            "2",
        ]))
        .expect("valid");
        assert_eq!(o.dir, "d");
        assert_eq!(o.seeds, Some(3));
        assert_eq!(o.profiles, Some(vec![RunProfile::F405, RunProfile::H743]));
        assert_eq!(o.threads, 2);
        let o = parse(args(&["d"])).expect("valid");
        assert_eq!((o.seeds, o.profiles, o.threads), (None, None, 0));
        assert_eq!(
            parse(args(&["d", "--profile", "h743"]))
                .expect("valid")
                .profiles,
            Some(vec![RunProfile::H743])
        );
    }

    #[test]
    fn bad_options_are_refused() {
        for bad in [
            &[][..],
            &["--seeds", "3"],
            &["d", "--seeds", "0"],
            &["d", "--seeds", "x"],
            &["d", "--profile", "f777"],
            &["d", "--threads"],
            &["d", "--bogus"],
            &["d", "--seed", "3"],
        ] {
            assert!(parse(args(bad)).is_err(), "{bad:?}");
        }
    }

    fn temp_dir(tag: &str) -> std::path::PathBuf {
        let dir =
            std::env::temp_dir().join(format!("oxide-scenarios-{}-{tag}", std::process::id()));
        std::fs::remove_dir_all(&dir).ok();
        std::fs::create_dir_all(&dir).expect("temp dir");
        dir
    }

    const PASSING: &str = "name = \"calm\"\nduration_s = 2.0\n";
    /// Needs a height the flight (which never arms) cannot reach.
    const FAILING: &str = "name = \"never\"\nduration_s = 2.0\n[expect]\nmin_height_m = 4.0\n[run]\nseeds = 2\nprofiles = [\"f405\"]\n";

    #[test]
    fn a_directory_with_a_failing_scenario_fails() {
        let dir = temp_dir("fail");
        std::fs::write(dir.join("calm.toml"), PASSING).expect("write");
        std::fs::write(dir.join("never.toml"), FAILING).expect("write");
        std::fs::write(dir.join("notes.txt"), "not a scenario").expect("write");
        let d = dir.to_string_lossy().into_owned();
        assert!(!run(args(&[&d, "--threads", "2"])).expect("runs"));
        // Only the passing file, with the seeds and profiles given.
        std::fs::remove_file(dir.join("never.toml")).expect("remove");
        assert!(run(args(&[&d, "--seeds", "2", "--profile", "both"])).expect("runs"));
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn a_bad_directory_or_file_is_an_error_naming_it() {
        let dir = temp_dir("bad");
        let d = dir.to_string_lossy().into_owned();
        let e = run(args(&[&d])).expect_err("empty directory");
        assert!(e.to_string().contains("no *.toml"), "{e}");
        std::fs::write(
            dir.join("bad.toml"),
            "name = \"x\"\nduration_s = 2.0\ncolour = 1\n",
        )
        .expect("write");
        let e = run(args(&[&d])).expect_err("bad file");
        assert!(
            e.to_string().contains("bad.toml") && e.to_string().contains("colour"),
            "{e}"
        );
        let e = run(args(&["/no/such/dir"])).expect_err("missing");
        assert!(e.to_string().contains("/no/such/dir"), "{e}");
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn the_repository_scenarios_all_pass_on_one_seed() {
        let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scenarios");
        let d = dir.to_string_lossy().into_owned();
        assert!(run(args(&[&d, "--seeds", "1", "--profile", "f405"])).expect("runs"));
    }
}
