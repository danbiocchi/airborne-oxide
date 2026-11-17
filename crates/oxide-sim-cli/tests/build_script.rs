// SPDX-License-Identifier: AGPL-3.0-or-later
//! `build.rs` must rerun whenever a commit moves `HEAD`, whatever way the
//! branch ref is stored.

#[path = "../build.rs"]
mod build_script;

use std::path::{Path, PathBuf};
use std::process::Command;

fn git(dir: &Path, args: &[&str]) {
    let ok = Command::new("git")
        .current_dir(dir)
        .args(["-c", "user.name=t", "-c", "user.email=t@example.com"])
        .args(args)
        .output()
        .is_ok_and(|o| o.status.success());
    assert!(ok, "git {args:?} failed");
}

fn temp_repo(tag: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("oxide-build-rs-{tag}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    git(&dir, &["init", "-q"]);
    git(&dir, &["commit", "-q", "--allow-empty", "-m", "a"]);
    dir
}

/// With the branch only in `packed-refs`, the next commit creates a loose ref
/// file that did not exist when the script ran. The commit appends to
/// `logs/HEAD`, so that file must be watched.
#[test]
fn a_commit_is_seen_when_the_branch_ref_is_packed() {
    let dir = temp_repo("packed");
    git(&dir, &["pack-refs", "--all"]);
    let watched = build_script::watched(&dir);
    let reflog = dir.join(".git/logs/HEAD");
    assert!(reflog.exists(), "the test repository has a reflog");
    let names: Vec<String> = watched
        .iter()
        .map(|p| p.to_string_lossy().replace('\\', "/"))
        .collect();
    assert!(
        names.iter().any(|n| n.ends_with("logs/HEAD")),
        "watched: {names:?}"
    );
    let _ = std::fs::remove_dir_all(dir);
}

#[test]
fn the_hash_is_the_commit_and_dirty_is_marked() {
    let dir = temp_repo("hash");
    let h = build_script::hash_of(&dir);
    assert_eq!(h.len(), 40, "{h}");
    std::fs::write(dir.join("f"), "x").unwrap();
    assert_eq!(build_script::hash_of(&dir), format!("{h}-dirty"));
    let _ = std::fs::remove_dir_all(dir);
}

/// Without git, or outside a repository, the hash is `unknown`, nothing is
/// watched, and the build is not failed.
#[test]
fn outside_a_repository_the_hash_is_unknown_and_nothing_is_watched() {
    let dir = std::env::temp_dir().join(format!("oxide-build-rs-none-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    // Not a repository, even if a parent directory were one.
    let ok = !Command::new("git")
        .current_dir(&dir)
        .env("GIT_CEILING_DIRECTORIES", &dir)
        .args(["rev-parse", "--git-dir"])
        .output()
        .is_ok_and(|o| o.status.success());
    assert!(ok, "the test directory is inside a repository");
    assert_eq!(build_script::hash_of(&dir), "unknown");
    assert!(build_script::watched(&dir).is_empty());
    // A repository without a commit has no hash either.
    git(&dir, &["init", "-q"]);
    assert_eq!(build_script::hash_of(&dir), "unknown");
    let _ = std::fs::remove_dir_all(dir);
}
