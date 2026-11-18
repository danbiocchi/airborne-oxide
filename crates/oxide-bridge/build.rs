// SPDX-License-Identifier: AGPL-3.0-or-later
//! Puts the git hash of the build into `OXIDE_GIT_HASH` (the `ver_sw` of every
//! log the binary writes): `git rev-parse HEAD`, with `-dirty` appended when
//! `git status --porcelain --untracked-files=no` lists anything (a change to a
//! tracked file; a stray untracked file does not make a build no commit's, as
//! it does not for `gate fly`), and `unknown` when git is not
//! installed or this is not a repository. It never fails the build.
//!
//! The script reruns when what it read can have moved: `HEAD` (a checkout),
//! the ref `HEAD` names (a commit on the branch), `packed-refs`, the index
//! (staging) and the reflog `logs/HEAD`, which every commit appends to: with
//! the branch only in `packed-refs`, a commit creates a loose ref file that did
//! not exist when the script ran and so could not be named. Edits to files that
//! are not yet staged, and the cleaning of a dirty tree, are seen the next time
//! the script runs for another reason, not as they are made: `-dirty` is the
//! state at the last run. A path that does not exist is never named, because
//! cargo would then rerun on every build.

use std::path::{Path, PathBuf};
use std::process::Command;

/// The trimmed stdout of `git args...` run in `dir`, when it succeeds.
fn git(dir: &Path, args: &[&str]) -> Option<String> {
    let out = Command::new("git")
        .current_dir(dir)
        .args(args)
        .output()
        .ok()?;
    if !out.status.success() {
        return None;
    }
    let text = String::from_utf8(out.stdout).ok()?;
    Some(text.trim().to_string())
}

/// The file `git rev-parse --git-path name` names, when it exists (relative to
/// `dir`, as git prints it).
fn git_path(dir: &Path, name: &str) -> Option<PathBuf> {
    let path = PathBuf::from(git(dir, &["rev-parse", "--git-path", name])?);
    dir.join(&path).exists().then_some(path)
}

/// The files cargo is told to watch for the repository at `dir`: the existing
/// ones among the list in the module comment. Empty outside a repository.
pub fn watched(dir: &Path) -> Vec<PathBuf> {
    let mut names = vec!["HEAD".to_string()];
    if let Some(r) = git(dir, &["symbolic-ref", "-q", "HEAD"]) {
        names.push(r);
    }
    names.extend(["packed-refs", "index", "logs/HEAD"].map(String::from));
    names.iter().filter_map(|n| git_path(dir, n)).collect()
}

/// `git rev-parse HEAD` of `dir`, `-dirty` when the tree has changes.
pub fn hash_of(dir: &Path) -> String {
    match git(dir, &["rev-parse", "HEAD"]) {
        Some(h) if !h.is_empty() => {
            let dirty = git(dir, &["status", "--porcelain", "--untracked-files=no"])
                .is_some_and(|s| !s.is_empty());
            if dirty { format!("{h}-dirty") } else { h }
        }
        _ => "unknown".to_string(),
    }
}

#[allow(dead_code)] // the tests include this file and do not call `main`
fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    for p in watched(Path::new(".")) {
        println!("cargo:rerun-if-changed={}", p.display());
    }
    println!("cargo:rustc-env=OXIDE_GIT_HASH={}", hash_of(Path::new(".")));
}
