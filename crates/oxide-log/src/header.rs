// SPDX-License-Identifier: AGPL-3.0-or-later
//! The definitions section of a log (ADR-0005 §2).
//!
//! [`write_definitions`] writes straight to a [`Sink`], not through the
//! logger's ring: the file header, the flag bits, every format (nested
//! formats first, then the topics), the information messages, all the
//! parameters, and then one subscription per topic. A topic's message id is
//! its position in the list the caller passes.
//!
//! The git hash, the board and the profile's name come from the caller
//! ([`BuildInfo`]); nothing here reads the environment.

use crate::msg::{self, Error, InfoValue, ParamValue, Sink};
use crate::record::Topic;
use oxide_core::config::VehicleConfig;
use oxide_core::params::{self, ParamId};

/// A format that topics nest: a name and its fields, as in a format message.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NestedFormat {
    /// Format name.
    pub name: &'static str,
    /// The fields, `"type name;..."` with the closing `;`.
    pub fields: &'static str,
}

/// How much a log records (ADR-0005 §3 and §5).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Level {
    /// A flight any ULog tool can show.
    Normal,
    /// The normal level plus an exact record of the core's boundary.
    Replay,
}

impl Level {
    /// The text of `oxide_log_level`.
    pub const fn as_str(self) -> &'static str {
        match self {
            Level::Normal => "normal",
            Level::Replay => "replay",
        }
    }
}

/// Who built this and what it runs on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BuildInfo<'a> {
    /// The git hash of the software (`ver_sw`).
    pub git_hash: &'a str,
    /// The board (`ver_hw`): `SIM` in the simulator.
    pub board: &'a str,
}

/// What the definitions say besides the topics and the parameters.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Header<'a> {
    /// The start time in the file header, µs.
    pub start_us: u64,
    /// `ver_sw` and `ver_hw`.
    pub build: BuildInfo<'a>,
    /// The scheduling profile's name (`oxide_profile`).
    pub profile: &'a str,
    /// `oxide_log_level`.
    pub level: Level,
    /// Formats the topics nest, written before the topics.
    pub nested: &'a [NestedFormat],
}

/// An information value that is never empty: `char[0]` is not something every
/// reader takes, and an empty name tells nothing.
fn text(s: &str) -> &str {
    if s.is_empty() { "unknown" } else { s }
}

/// Writes the definitions section: the file header, the flag bits, the
/// nested formats and the topics' formats, the information messages
/// (`sys_name` `Oxide`, `ver_sw`, `ver_hw`, `oxide_profile`,
/// `oxide_config_hash` as `uint64_t`, `oxide_log_level`), all the parameters
/// of the configuration with their exact values (`float` or `int32_t` by the
/// parameter's own type, in table order, as `params::get` reads them), and one
/// subscription per topic with message id = position in `topics`.
///
/// An empty git hash, board or profile name is written as `unknown`. If the
/// sink refuses, the error is returned and the sink holds a partial section.
pub fn write_definitions<S: Sink + ?Sized, const N: usize>(
    sink: &mut S,
    h: &Header<'_>,
    cfg: &VehicleConfig<N>,
    topics: &[Topic],
) -> Result<(), Error> {
    msg::file_header(sink, h.start_us)?;
    msg::flag_bits(sink, [0; 8], [0; 8])?;
    for n in h.nested {
        msg::format(sink, n.name, n.fields)?;
    }
    for t in topics {
        t.write_format(sink)?;
    }
    msg::info(sink, "sys_name", InfoValue::Str("Oxide"))?;
    msg::info(sink, "ver_sw", InfoValue::Str(text(h.build.git_hash)))?;
    msg::info(sink, "ver_hw", InfoValue::Str(text(h.build.board)))?;
    msg::info(sink, "oxide_profile", InfoValue::Str(text(h.profile)))?;
    msg::info(sink, "oxide_config_hash", InfoValue::U64(cfg.config_hash()))?;
    msg::info(sink, "oxide_log_level", InfoValue::Str(h.level.as_str()))?;
    for (i, def) in params::PARAMS.iter().enumerate() {
        let Some(id) = u16::try_from(i).ok().map(ParamId) else {
            break;
        };
        let value = match params::get(cfg, id) {
            Some(params::ParamValue::F32(v)) => ParamValue::F32(v),
            Some(params::ParamValue::I32(v)) => ParamValue::I32(v),
            None => continue,
        };
        msg::parameter(sink, def.name, value)?;
    }
    for (i, t) in topics.iter().enumerate() {
        let id = u16::try_from(i).map_err(|_| Error::TooLong)?;
        t.add_logged(sink, 0, id)?;
    }
    Ok(())
}
