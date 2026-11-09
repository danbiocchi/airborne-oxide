// SPDX-License-Identifier: AGPL-3.0-or-later
//! Airborne Oxide flight core.
//!
//! `no_std`, no `alloc`, no global state (DESIGN §4). Board firmware and the
//! simulator both drive it through [`Core::tick`], using the types in [`io`].

#![cfg_attr(not(test), no_std)]
#![deny(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::indexing_slicing
)]
#![cfg_attr(
    test,
    allow(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::panic,
        clippy::indexing_slicing
    )
)]

pub mod allocation;
pub mod config;
pub mod control;
pub mod estimator;
pub mod filters;
pub mod flight;
pub mod io;
pub mod linalg;
pub mod math;
pub mod schedule;
pub mod time;

pub use flight::Core;

/// Standard gravity, m/s².
pub const GRAVITY: f32 = 9.806_65;
