// SPDX-License-Identifier: AGPL-3.0-or-later
//! The bridge that flies another autopilot's simulator build on our physics
//! (ADR-0006). So far: the JSON exchange only, the half that needs neither the
//! simulator program nor MAVLink.
//!
//! - [`packet`]: the motor packet the simulator sends (binary, little-endian).
//! - [`reply`]: the line of text we answer with.
//! - [`motors`]: pulse widths to normalised motor commands.
//! - [`link`]: the UDP socket and the lockstep rule.
//! - [`sha256`]: for checking the simulator binary.
//! - [`bridge`]: [`bridge::JsonBridge`], an `oxide_sim::Autopilot<4>`.

pub mod bridge;
pub mod link;
pub mod motors;
pub mod packet;
pub mod reply;
pub mod sha256;
