// SPDX-License-Identifier: AGPL-3.0-or-later
//! Airborne Oxide parameter store.
//!
//! A log-structured store of 32-bit values under 32-bit keys over two flash
//! sectors, safe against power loss at any byte (ADR-0004 §6). `no_std`, no
//! allocation, no dependencies. The crate knows nothing about parameters.

#![cfg_attr(not(any(test, feature = "std")), no_std)]
#![forbid(unsafe_code)]
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

pub mod crc;
pub mod flash;
pub mod layout;
pub mod store;

pub use flash::{Flash, FlashError};
pub use store::{Error, Iter, MountReport, Store};
