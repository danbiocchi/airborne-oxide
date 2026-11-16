// SPDX-License-Identifier: AGPL-3.0-or-later
//! Airborne Oxide logs (ADR-0005), in two layers.
//!
//! The format layer knows the ULog file format and nothing about flight:
//!
//! - [`msg`]: encoders for every ULog message a writer needs, into a [`Sink`].
//! - [`record`]: topic descriptions and the field-by-field [`RecordWriter`].
//! - [`logger`]: a ring of whole messages that drops and counts instead of
//!   blocking (ADR-0005 §7).
//! - `format` and `reader` (feature `std`): the format-string parser and a
//!   strict ULog reader.
//!
//! The topic layer knows the flight core's types and maps them to topics. It
//! is the only part that depends on `oxide-core`:
//!
//! - [`topics`]: the normal level's topics (ADR-0005 §3) and their encoders.
//! - [`header`]: the definitions section of a log.
//! - [`recorder`]: what is written after each tick, and when.
//! - [`replay`]: the replay level's records of the core's boundary (ADR-0005 §5).
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
pub mod header;
pub mod logger;
pub mod msg;
#[cfg(any(test, feature = "std"))]
pub mod reader;
pub mod record;
pub mod recorder;
pub mod replay;
pub mod topics;

pub use msg::{Error, Sink};

#[cfg(test)]
mod tests;
