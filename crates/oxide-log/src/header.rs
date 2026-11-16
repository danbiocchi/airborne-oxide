// SPDX-License-Identifier: AGPL-3.0-or-later
//! The definitions section of a log (ADR-0005 §2).

/// A format that topics nest: a name and its fields, as in a format message.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NestedFormat {
    /// Format name.
    pub name: &'static str,
    /// The fields, `"type name;..."` with the closing `;`.
    pub fields: &'static str,
}
