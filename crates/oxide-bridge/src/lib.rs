// SPDX-License-Identifier: AGPL-3.0-or-later
//! The bridge that flies another autopilot's simulator build on our physics
//! (ADR-0006): the JSON exchange, the simulator program as a checked child
//! process, and the MAVLink client for it.
//!
//! - [`packet`]: the motor packet the simulator sends (binary, little-endian).
//! - [`reply`]: the line of text we answer with.
//! - [`motors`]: pulse widths to normalised motor commands.
//! - [`link`]: the UDP socket and the lockstep rule.
//! - [`sha256`]: for checking the simulator binary.
//! - [`sitl`]: starts and stops the simulator program (ArduCopter SITL) in its
//!   own new directory, and never leaves it running.
//! - [`mav`]: the MAVLink client: typed events out, small sends in, never
//!   blocking on the program (which only moves while we exchange).
//! - [`bridge`]: [`bridge::JsonBridge`], an `oxide_sim::Autopilot<4>`.

pub mod bridge;
pub mod link;
pub mod mav;
pub mod motors;
pub mod packet;
pub mod reply;
pub mod sha256;
pub mod sitl;
