// SPDX-License-Identifier: AGPL-3.0-or-later
//! ULog format strings: parsing, offsets, sizes, padding rules (`std` only).
//!
//! The grammar and the padding rules are those of `inv-ulog.md` §2: fields are
//! `type name;` or `type[N] name;`, a type is a basic type or another format's
//! name, offsets are packed (ULog has no implicit alignment), and a trailing
//! run of `_padding*` fields is part of the format but not of a data record.

use crate::record::Topic;
use std::collections::BTreeMap;
use std::fmt;
use std::string::{String, ToString};
use std::vec::Vec;

/// A basic ULog type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Scalar {
    /// `int8_t`
    I8,
    /// `uint8_t`
    U8,
    /// `int16_t`
    I16,
    /// `uint16_t`
    U16,
    /// `int32_t`
    I32,
    /// `uint32_t`
    U32,
    /// `int64_t`
    I64,
    /// `uint64_t`
    U64,
    /// `float`
    F32,
    /// `double`
    F64,
    /// `bool`
    Bool,
    /// `char`
    Char,
}

impl Scalar {
    /// Parses a basic type name.
    pub fn parse(s: &str) -> Option<Self> {
        Some(match s {
            "int8_t" => Self::I8,
            "uint8_t" => Self::U8,
            "int16_t" => Self::I16,
            "uint16_t" => Self::U16,
            "int32_t" => Self::I32,
            "uint32_t" => Self::U32,
            "int64_t" => Self::I64,
            "uint64_t" => Self::U64,
            "float" => Self::F32,
            "double" => Self::F64,
            "bool" => Self::Bool,
            "char" => Self::Char,
            _ => return None,
        })
    }

    /// Size in bytes.
    pub fn size(self) -> usize {
        match self {
            Self::I8 | Self::U8 | Self::Bool | Self::Char => 1,
            Self::I16 | Self::U16 => 2,
            Self::I32 | Self::U32 | Self::F32 => 4,
            Self::I64 | Self::U64 | Self::F64 => 8,
        }
    }
}

/// The type of a field: basic, or the name of another format.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FieldType {
    /// A basic type.
    Scalar(Scalar),
    /// A nested format.
    Nested(String),
}

/// One field of a format, with its place in the record.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Field {
    /// Field name.
    pub name: String,
    /// Element type.
    pub ty: FieldType,
    /// `Some(n)` for `type[n] name`.
    pub array: Option<usize>,
    /// Byte offset from the start of the record.
    pub offset: usize,
    /// Size of one element (a nested element is the nested format's padded size).
    pub elem_size: usize,
    /// Total size (`elem_size` times the array length).
    pub size: usize,
}

impl Field {
    /// The basic type, if it is one.
    pub fn scalar(&self) -> Option<Scalar> {
        match self.ty {
            FieldType::Scalar(s) => Some(s),
            FieldType::Nested(_) => None,
        }
    }

    /// True for a `_padding*` field.
    pub fn is_padding(&self) -> bool {
        self.name.starts_with("_padding")
    }
}

/// A format with offsets and sizes.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Layout {
    /// Format name.
    pub name: String,
    /// Fields in order.
    pub fields: Vec<Field>,
    /// Size of all fields, padding included (the size when nested).
    pub size: usize,
    /// Size without the trailing run of `_padding*` fields (a data record's length).
    pub size_no_trailing_padding: usize,
}

/// Why a format string was refused.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FormatError {
    /// No field at all.
    NoFields,
    /// A field that is not `type name` (also: empty field, two spaces).
    BadField {
        /// The text of the field.
        text: String,
    },
    /// A bad array suffix (size 0, over 65,535, or no closing bracket).
    BadArray {
        /// The text of the type.
        text: String,
    },
    /// A name with a character outside the allowed set.
    BadName {
        /// The name.
        name: String,
    },
    /// A type that is neither basic nor a known format.
    UnknownType {
        /// The type name.
        name: String,
    },
    /// Two fields with one name.
    DuplicateField {
        /// The name.
        name: String,
    },
    /// A layout larger than any message can carry.
    TooLarge {
        /// The format.
        name: String,
    },
    /// A format that contains itself.
    Cycle {
        /// A format on the cycle.
        name: String,
    },
}

impl fmt::Display for FormatError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NoFields => write!(f, "format has no fields"),
            Self::BadField { text } => write!(f, "bad field {text:?}"),
            Self::BadArray { text } => write!(f, "bad array type {text:?}"),
            Self::BadName { name } => write!(f, "bad name {name:?}"),
            Self::UnknownType { name } => write!(f, "unknown type {name:?}"),
            Self::DuplicateField { name } => write!(f, "duplicate field {name:?}"),
            Self::TooLarge { name } => write!(f, "format {name:?} is larger than 65535 bytes"),
            Self::Cycle { name } => write!(f, "format {name:?} contains itself"),
        }
    }
}

impl std::error::Error for FormatError {}

/// One field before types are resolved.
struct RawField {
    ty: String,
    array: Option<usize>,
    name: String,
}

fn name_ok(s: &str, slash_dash: bool) -> bool {
    !s.is_empty()
        && s.bytes().all(|c| {
            c.is_ascii_alphanumeric() || c == b'_' || (slash_dash && (c == b'-' || c == b'/'))
        })
}

fn parse_fields(fields: &str) -> Result<Vec<RawField>, FormatError> {
    let mut out = Vec::new();
    let mut pieces: Vec<&str> = fields.split(';').collect();
    // The closing ';' leaves one empty piece; PX4 always writes it, pyulog does not need it.
    if pieces.last() == Some(&"") {
        pieces.pop();
    }
    for text in pieces {
        let bad = || FormatError::BadField {
            text: text.to_string(),
        };
        let (ty, name) = text.split_once(' ').ok_or_else(bad)?;
        if ty.is_empty() || name.contains(' ') {
            return Err(bad());
        }
        let (base, array) = match ty.split_once('[') {
            None => (ty, None),
            Some((base, rest)) => {
                let n = rest
                    .strip_suffix(']')
                    .and_then(|d| d.parse::<usize>().ok())
                    .filter(|n| (1..=usize::from(u16::MAX)).contains(n));
                match n {
                    Some(n) => (base, Some(n)),
                    None => {
                        return Err(FormatError::BadArray {
                            text: ty.to_string(),
                        });
                    }
                }
            }
        };
        if !name_ok(name, false) {
            return Err(FormatError::BadName {
                name: name.to_string(),
            });
        }
        if !name_ok(base, true) {
            return Err(FormatError::BadName {
                name: base.to_string(),
            });
        }
        out.push(RawField {
            ty: base.to_string(),
            array,
            name: name.to_string(),
        });
    }
    if out.is_empty() {
        return Err(FormatError::NoFields);
    }
    Ok(out)
}

/// The largest layout, in bytes: a message carries at most 65,535.
const MAX_LAYOUT: usize = u16::MAX as usize;

fn build(
    name: &str,
    fields: &str,
    nested: &dyn Fn(&str) -> Option<usize>,
) -> Result<Layout, FormatError> {
    let raw = parse_fields(fields)?;
    let mut out: Vec<Field> = Vec::with_capacity(raw.len());
    let mut offset = 0usize;
    for r in raw {
        if out.iter().any(|f| f.name == r.name) {
            return Err(FormatError::DuplicateField { name: r.name });
        }
        let (ty, elem_size) = match Scalar::parse(&r.ty) {
            Some(s) => (FieldType::Scalar(s), s.size()),
            None => match nested(&r.ty) {
                Some(size) => (FieldType::Nested(r.ty.clone()), size),
                None => return Err(FormatError::UnknownType { name: r.ty }),
            },
        };
        let too_large = || FormatError::TooLarge {
            name: name.to_string(),
        };
        let size = elem_size
            .checked_mul(r.array.unwrap_or(1))
            .filter(|s| *s <= MAX_LAYOUT)
            .ok_or_else(too_large)?;
        out.push(Field {
            name: r.name,
            ty,
            array: r.array,
            offset,
            elem_size,
            size,
        });
        offset = offset
            .checked_add(size)
            .filter(|o| *o <= MAX_LAYOUT)
            .ok_or_else(too_large)?;
    }
    let trailing: usize = out
        .iter()
        .rev()
        .take_while(|f| f.is_padding())
        .map(|f| f.size)
        .sum();
    Ok(Layout {
        name: name.to_string(),
        fields: out,
        size: offset,
        size_no_trailing_padding: offset.saturating_sub(trailing),
    })
}

/// Parses one format that uses basic types only.
pub fn layout(name: &str, fields: &str) -> Result<Layout, FormatError> {
    build(name, fields, &|_| None)
}

/// Resolves a set of `(name, fields)` definitions, nested types included, in
/// any order. On error, returns the index of the definition at fault.
pub fn resolve(defs: &[(&str, &str)]) -> Result<BTreeMap<String, Layout>, (usize, FormatError)> {
    struct Frame {
        i: usize,
        fields: Vec<RawField>,
        next: usize,
    }
    let mut by_name: BTreeMap<&str, usize> = BTreeMap::new();
    for (i, (n, _)) in defs.iter().enumerate() {
        by_name.entry(*n).or_insert(i);
    }
    // 0 = todo, 1 = on the stack, 2 = done. Iterative: the depth of the
    // nesting is the file's choice, not ours.
    let mut state = std::vec![0u8; defs.len()];
    let mut done: BTreeMap<String, Layout> = BTreeMap::new();
    let mut stack: Vec<Frame> = Vec::new();
    for root in 0..defs.len() {
        if state.get(root) != Some(&0) {
            continue;
        }
        let &(_, fields) = defs.get(root).ok_or((root, FormatError::NoFields))?;
        let fields = parse_fields(fields).map_err(|e| (root, e))?;
        if let Some(s) = state.get_mut(root) {
            *s = 1;
        }
        stack.push(Frame {
            i: root,
            fields,
            next: 0,
        });
        while let Some(top) = stack.last_mut() {
            let i = top.i;
            let &(name, fields) = defs.get(i).ok_or((i, FormatError::NoFields))?;
            let Some(r) = top.fields.get(top.next) else {
                let l =
                    build(name, fields, &|t| done.get(t).map(|l| l.size)).map_err(|e| (i, e))?;
                done.insert(name.to_string(), l);
                if let Some(s) = state.get_mut(i) {
                    *s = 2;
                }
                stack.pop();
                continue;
            };
            top.next += 1;
            if Scalar::parse(&r.ty).is_some() {
                continue;
            }
            let Some(&j) = by_name.get(r.ty.as_str()) else {
                return Err((i, FormatError::UnknownType { name: r.ty.clone() }));
            };
            match state.get(j) {
                Some(1) => {
                    let n = defs.get(j).map_or("", |d| d.0);
                    return Err((
                        j,
                        FormatError::Cycle {
                            name: n.to_string(),
                        },
                    ));
                }
                Some(0) => {
                    let &(_, jf) = defs.get(j).ok_or((j, FormatError::NoFields))?;
                    let jfields = parse_fields(jf).map_err(|e| (j, e))?;
                    if let Some(s) = state.get_mut(j) {
                        *s = 1;
                    }
                    stack.push(Frame {
                        i: j,
                        fields: jfields,
                        next: 0,
                    });
                }
                _ => {}
            }
        }
    }
    Ok(done)
}

/// Checks a [`Topic`] constant against its own format string: it parses
/// (basic types only), the name is valid, `timestamp` is the first field and a
/// `uint64_t`, field names are unique, `_padding*` fields are `uint8_t`
/// arrays, every basic field sits at an offset that is a multiple of its size
/// (as in PX4's generated structs), the padded size is a multiple of 8, and
/// `record_len` equals the size without trailing padding. Returns the layout,
/// or a message naming the first rule broken.
pub fn check_topic(t: &Topic) -> Result<Layout, String> {
    if !name_ok(t.name, true) {
        return Err(std::format!("topic name {:?} is not a valid name", t.name));
    }
    let l = layout(t.name, t.format).map_err(|e| std::format!("{}: {e}", t.name))?;
    let first = l.fields.first();
    if !matches!(first, Some(f) if f.name == "timestamp"
        && f.array.is_none()
        && f.scalar() == Some(Scalar::U64))
    {
        return Err(std::format!(
            "{}: the first field must be `uint64_t timestamp`",
            t.name
        ));
    }
    for f in &l.fields {
        if f.is_padding() {
            let digits = f.name.trim_start_matches("_padding");
            if f.scalar() != Some(Scalar::U8)
                || f.array.is_none()
                || !digits.bytes().all(|c| c.is_ascii_digit())
            {
                return Err(std::format!(
                    "{}: padding field {} must be a `uint8_t[N] _padding<digits>`",
                    t.name,
                    f.name
                ));
            }
        } else {
            let align = match f.scalar() {
                Some(s) => s.size(),
                None => 8,
            };
            if f.offset % align != 0 {
                return Err(std::format!(
                    "{}: field {} at offset {} is not aligned to {align}",
                    t.name,
                    f.name,
                    f.offset
                ));
            }
        }
    }
    if l.size % 8 != 0 {
        return Err(std::format!(
            "{}: padded size {} is not a multiple of 8",
            t.name,
            l.size
        ));
    }
    if l.size_no_trailing_padding != t.record_len {
        return Err(std::format!(
            "{}: record_len {} but the format is {} bytes without trailing padding ({} padded)",
            t.name,
            t.record_len,
            l.size_no_trailing_padding,
            l.size
        ));
    }
    Ok(l)
}

/// [`check_topic`] as an assertion, for the tests of a topic table.
#[allow(clippy::panic)]
pub fn assert_topic(t: &Topic) {
    if let Err(e) = check_topic(t) {
        panic!("topic constant is inconsistent: {e}");
    }
}
