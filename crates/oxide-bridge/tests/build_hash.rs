// SPDX-License-Identifier: AGPL-3.0-or-later
//! The commit a build carries (`OXIDE_GIT_HASH`): a stray untracked file does
//! not make it `-dirty`, as it does not for `gate fly`; a change to a tracked
//! file does. A program built at no commit gives no gate verdict, so this
//! decides whether a program may.

#[path = "../build.rs"]
mod build;

use std::path::{Path, PathBuf};
use std::process::Command;

fn git(dir: &Path, args: &[&str]) {
    let st = Command::new("git")
        .current_dir(dir)
        .args(["-c", "user.name=t", "-c", "user.email=t@example.com"])
        .args(["-c", "commit.gpgsign=false"])
        .args(args)
        .status()
        .expect("git");
    assert!(st.success(), "git {args:?}");
}

fn repo(tag: &str) -> PathBuf {
    let d = std::env::temp_dir().join(format!("oxide-build-hash-{tag}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&d);
    std::fs::create_dir_all(&d).expect("dir");
    git(&d, &["init", "-q"]);
    std::fs::write(d.join("a.txt"), "one\n").expect("write");
    git(&d, &["add", "a.txt"]);
    git(&d, &["commit", "-q", "-m", "first"]);
    d
}

#[test]
fn an_untracked_file_does_not_make_a_build_dirty_and_a_change_to_a_tracked_file_does() {
    let d = repo("dirty");
    let clean = build::hash_of(&d);
    assert_eq!(clean.len(), 40, "{clean}");
    std::fs::write(d.join("note.txt"), "x").expect("write");
    assert_eq!(build::hash_of(&d), clean, "an untracked file");
    std::fs::write(d.join("a.txt"), "two\n").expect("write");
    assert_eq!(build::hash_of(&d), format!("{clean}-dirty"));
    let _ = std::fs::remove_dir_all(&d);
}

#[test]
fn a_directory_that_is_no_repository_is_unknown() {
    let d = std::env::temp_dir().join(format!("oxide-build-hash-none-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&d);
    std::fs::create_dir_all(&d).expect("dir");
    assert_eq!(build::hash_of(&d), "unknown");
    let _ = std::fs::remove_dir_all(&d);
}
