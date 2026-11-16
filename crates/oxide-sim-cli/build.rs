// SPDX-License-Identifier: AGPL-3.0-or-later
//! Puts the git hash of the build into `OXIDE_GIT_HASH` (the `ver_sw` of every
//! log the binary writes): `git rev-parse HEAD`, with `-dirty` appended when
//! `git status --porcelain` lists anything, and `unknown` when git is not
//! installed or this is not a repository. It never fails the build.
//!
//! The script reruns when what it read can have moved: `HEAD` (a checkout),
//! the ref `HEAD` names (a commit on the branch), `packed-refs` and the index
//! (staging). Edits to files that are not yet staged are seen the next time the
//! script runs for another reason, not as they are made. A path that does not
//! exist is never named, because cargo would then rerun on every build.

use std::path::Path;
use std::process::Command;

/// The trimmed stdout of `git args...` when it succeeds.
fn git(args: &[&str]) -> Option<String> {
    let out = Command::new("git").args(args).output().ok()?;
    if !out.status.success() {
        return None;
    }
    let text = String::from_utf8(out.stdout).ok()?;
    Some(text.trim().to_string())
}

/// Tells cargo to watch the file `git rev-parse --git-path name` names.
fn watch(name: &str) {
    if let Some(path) = git(&["rev-parse", "--git-path", name])
        && Path::new(&path).exists()
    {
        println!("cargo:rerun-if-changed={path}");
    }
}

fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    let hash = match git(&["rev-parse", "HEAD"]) {
        Some(h) if !h.is_empty() => {
            watch("HEAD");
            if let Some(r) = git(&["symbolic-ref", "-q", "HEAD"]) {
                watch(&r);
            }
            watch("packed-refs");
            watch("index");
            let dirty = git(&["status", "--porcelain"]).is_some_and(|s| !s.is_empty());
            if dirty { format!("{h}-dirty") } else { h }
        }
        _ => "unknown".to_string(),
    };
    println!("cargo:rustc-env=OXIDE_GIT_HASH={hash}");
}
