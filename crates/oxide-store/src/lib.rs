// SPDX-License-Identifier: AGPL-3.0-or-later
//! Airborne Oxide parameter store.
//!
//! A log-structured store of 32-bit values under 32-bit keys over two flash
//! sectors, safe against power loss anywhere in a program or erase call
//! (ADR-0004 §6): the flash contract is in [`flash::Flash`], the record format
//! and the argument that a torn record never reads as valid in [`layout`], the
//! mount rules and costs in [`store`]. [`Store::replace_all`] makes the store
//! hold exactly a given set of pairs, all or nothing. `no_std`, no allocation.
//! The store itself knows nothing about parameters; [`param_store`] is the one
//! module that does (it depends on `oxide-core`): the policy of ADR-0004 §6 for
//! loading parameters at boot and saving them while disarmed.

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
pub mod param_store;
#[cfg(any(test, feature = "std"))]
pub mod sim;
pub mod store;

pub use flash::{Flash, FlashError};
pub use param_store::{ParamStore, Serviced};
pub use store::{Error, Iter, MountReport, Replaced, Store};

#[cfg(test)]
mod param_store_tests;
#[cfg(test)]
mod tests;
