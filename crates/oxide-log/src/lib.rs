// SPDX-License-Identifier: AGPL-3.0-or-later
//! Airborne Oxide log format layer (ADR-0005 §1): the ULog file format and
//! nothing about flight.
//!
//! - [`msg`]: encoders for every ULog message a writer needs, into a [`Sink`].
//! - [`record`]: topic descriptions and the field-by-field [`RecordWriter`].
//! - [`logger`]: a ring of whole messages that drops and counts instead of
//!   blocking (ADR-0005 §7).
//! - `format` and `reader` (feature `std`): the format-string parser and a
//!   strict ULog reader.
//!
//! `no_std`, no allocation, no `unsafe`, except for the `std` feature.

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

#[cfg(any(test, feature = "std"))]
pub mod format;
pub mod logger;
pub mod msg;
#[cfg(any(test, feature = "std"))]
pub mod reader;
pub mod record;

pub use msg::{Error, Sink};

#[cfg(test)]
mod tests;
