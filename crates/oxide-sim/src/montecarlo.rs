// SPDX-License-Identifier: AGPL-3.0-or-later
//! Parallel batch runs. Each run is independent and single-threaded; only
//! the scheduling of runs over OS threads varies, and results are returned in
//! index order, so the output never depends on it (DESIGN §5.2 item 1).

use std::sync::Mutex;
use std::sync::atomic::{AtomicUsize, Ordering};

/// Runs `f(i)` for `i` in `0..n` on up to `threads` OS threads (0 = all
/// available cores). Results come back in index order whatever the
/// scheduling. A panic in `f` propagates.
pub fn run_indexed<R: Send>(n: usize, threads: usize, f: impl Fn(usize) -> R + Sync) -> Vec<R> {
    if n == 0 {
        return Vec::new();
    }
    let available = std::thread::available_parallelism().map_or(1, usize::from);
    let workers = if threads == 0 { available } else { threads }.clamp(1, n);
    let next = AtomicUsize::new(0);
    let results: Mutex<Vec<(usize, R)>> = Mutex::new(Vec::with_capacity(n));
    std::thread::scope(|scope| {
        for _ in 0..workers {
            scope.spawn(|| {
                loop {
                    let i = next.fetch_add(1, Ordering::Relaxed);
                    if i >= n {
                        break;
                    }
                    let r = f(i);
                    // A poisoned lock means another worker panicked; the
                    // scope re-raises that panic, so just keep going.
                    results
                        .lock()
                        .unwrap_or_else(std::sync::PoisonError::into_inner)
                        .push((i, r));
                }
            });
        }
    });
    let mut out = results
        .into_inner()
        .unwrap_or_else(std::sync::PoisonError::into_inner);
    out.sort_by_key(|(i, _)| *i);
    out.into_iter().map(|(_, r)| r).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn order_is_preserved_for_any_thread_count() {
        for threads in [1, 3, 0] {
            let out = run_indexed(100, threads, |i| {
                // Uneven work so completion order differs from index order.
                let mut x = 0u64;
                for k in 0..((100 - i) * 50) as u64 {
                    x = x.wrapping_add(k);
                }
                (i, x)
            });
            assert_eq!(out.len(), 100);
            for (i, (j, _)) in out.iter().enumerate() {
                assert_eq!(i, *j, "threads = {threads}");
            }
        }
    }

    #[test]
    fn empty_batch() {
        let out: Vec<usize> = run_indexed(0, 4, |i| i);
        assert!(out.is_empty());
    }

    #[test]
    fn more_threads_than_items() {
        assert_eq!(run_indexed(2, 16, |i| i * 2), vec![0, 2]);
    }
}
