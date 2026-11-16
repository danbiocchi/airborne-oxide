// SPDX-License-Identifier: AGPL-3.0-or-later
//! A strict ULog reader (`std` only).
//!
//! Strict in the way `pyulog` is: an unknown message id, a data record whose
//! length is not the format's size without trailing padding, a definition after
//! the first subscription, or a truncated definition is an error that carries
//! the byte offset. A truncated **last** message in the data section (a file
//! cut by power loss) is not an error: everything before it is returned and the
//! cut is reported in [`UlogFile::truncated`]. Every item of the data section is
//! kept, in file order, because replay needs the order.

use crate::format::{self, FormatError, Layout, Scalar};
use crate::msg::ParamValue;
use crate::msg::{SYNC_MAGIC, kind};
use std::collections::BTreeMap;
use std::fmt;
use std::string::{String, ToString};
use std::vec::Vec;

/// Why parsing stopped, and where.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReadError {
    /// Byte offset in the file of the message at fault (0 for the file header).
    pub offset: usize,
    /// What was wrong.
    pub kind: ReadErrorKind,
}

/// The ways a file can be wrong.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ReadErrorKind {
    /// Fewer than 16 bytes.
    TooShort,
    /// The first 7 bytes are not the ULog magic.
    BadMagic,
    /// An incompatibility flag this reader does not know (the file must not be parsed).
    UnknownIncompatFlags([u8; 8]),
    /// A message cut short inside the definitions section.
    Truncated {
        /// Bytes the message needs (header included).
        needed: usize,
        /// Bytes the file has from its start.
        available: usize,
    },
    /// A message whose payload does not have the layout of its type.
    BadMessage {
        /// The message type letter.
        msg_type: u8,
        /// What is wrong.
        why: &'static str,
    },
    /// A flag-bits message that is not the first message.
    FlagBitsNotFirst,
    /// A message type that cannot be in the definitions section.
    UnexpectedInDefinitions(u8),
    /// A format, flag-bits or other definition after the first subscription.
    DefinitionAfterSubscription(u8),
    /// A format string that does not parse.
    BadFormat(FormatError),
    /// A subscription to a topic with no format.
    UnknownTopic(String),
    /// A message id subscribed twice.
    DuplicateMessageId(u16),
    /// A data message for a message id that was never subscribed.
    UnknownMessageId(u16),
    /// A data record of the wrong length.
    RecordLength {
        /// The topic.
        topic: String,
        /// The format's size without trailing padding.
        expected: usize,
        /// The record's bytes.
        got: usize,
    },
    /// A synchronisation message with the wrong magic.
    BadSync,
}

impl fmt::Display for ReadError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "at byte {}: ", self.offset)?;
        match &self.kind {
            ReadErrorKind::TooShort => write!(f, "file shorter than its 16-byte header"),
            ReadErrorKind::BadMagic => write!(f, "not a ULog file (bad magic)"),
            ReadErrorKind::UnknownIncompatFlags(b) => {
                write!(f, "unknown incompatibility flags {b:02x?}")
            }
            ReadErrorKind::Truncated { needed, available } => {
                write!(
                    f,
                    "message cut short in the definitions ({available} of {needed} bytes)"
                )
            }
            ReadErrorKind::BadMessage { msg_type, why } => {
                write!(f, "bad {:?} message: {why}", char::from(*msg_type))
            }
            ReadErrorKind::FlagBitsNotFirst => write!(f, "flag bits are not the first message"),
            ReadErrorKind::UnexpectedInDefinitions(t) => {
                write!(f, "{:?} message in the definitions section", char::from(*t))
            }
            ReadErrorKind::DefinitionAfterSubscription(t) => {
                write!(
                    f,
                    "{:?} message after the first subscription",
                    char::from(*t)
                )
            }
            ReadErrorKind::BadFormat(e) => write!(f, "bad format: {e}"),
            ReadErrorKind::UnknownTopic(n) => write!(f, "subscription to unknown topic {n:?}"),
            ReadErrorKind::DuplicateMessageId(i) => write!(f, "message id {i} subscribed twice"),
            ReadErrorKind::UnknownMessageId(i) => write!(f, "data for unknown message id {i}"),
            ReadErrorKind::RecordLength {
                topic,
                expected,
                got,
            } => {
                write!(
                    f,
                    "record of {topic:?} has {got} bytes, the format needs {expected}"
                )
            }
            ReadErrorKind::BadSync => write!(f, "synchronisation message with the wrong magic"),
        }
    }
}

impl std::error::Error for ReadError {}

/// A format message with its resolved layout.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FormatDef {
    /// Format (topic) name.
    pub name: String,
    /// The fields string as written.
    pub fields: String,
    /// Offsets and sizes.
    pub layout: Layout,
    /// File offset of the message.
    pub offset: usize,
}

/// An information message (`I`, or `M` with `continued`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct InfoEntry {
    /// The type as written: `char[9]`, `uint32_t`, ...
    pub ty: String,
    /// The key name.
    pub name: String,
    /// The value bytes.
    pub value: Vec<u8>,
    /// For `M`: continues the previous message with the same key.
    pub continued: bool,
}

impl InfoEntry {
    /// The value as text, for a `char[N]` entry.
    pub fn as_str(&self) -> Option<&str> {
        if self.ty.starts_with("char[") {
            core::str::from_utf8(&self.value).ok()
        } else {
            None
        }
    }

    fn fixed<const N: usize>(&self, ty: &str) -> Option<[u8; N]> {
        if self.ty == ty {
            self.value.as_slice().try_into().ok()
        } else {
            None
        }
    }

    /// `uint32_t` value.
    pub fn as_u32(&self) -> Option<u32> {
        self.fixed("uint32_t").map(u32::from_le_bytes)
    }
    /// `int32_t` value.
    pub fn as_i32(&self) -> Option<i32> {
        self.fixed("int32_t").map(i32::from_le_bytes)
    }
    /// `uint64_t` value.
    pub fn as_u64(&self) -> Option<u64> {
        self.fixed("uint64_t").map(u64::from_le_bytes)
    }
    /// `float` value.
    pub fn as_f32(&self) -> Option<f32> {
        self.fixed("float")
            .map(|b| f32::from_bits(u32::from_le_bytes(b)))
    }
}

/// A parameter in the definitions.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Param<'a> {
    /// Name.
    pub name: &'a str,
    /// Value.
    pub value: ParamValue,
}

/// An initial parameter, owned.
#[derive(Debug, Clone, PartialEq)]
pub struct ParamEntry {
    /// Name.
    pub name: String,
    /// Value.
    pub value: ParamValue,
}

/// A default parameter (`Q`).
#[derive(Debug, Clone, PartialEq)]
pub struct DefaultParam {
    /// Bit 0: system default; bit 1: default for this configuration.
    pub default_types: u8,
    /// Name.
    pub name: String,
    /// Value.
    pub value: ParamValue,
}

/// A subscription (`A`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Subscription {
    /// Instance.
    pub multi_id: u8,
    /// Message id the data messages use.
    pub msg_id: u16,
    /// Topic name.
    pub name: String,
    /// Index into [`UlogFile::formats`].
    pub format: usize,
    /// File offset of the message.
    pub offset: usize,
}

/// One data message.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DataRecord {
    /// Index into [`UlogFile::subscriptions`].
    pub subscription: usize,
    /// File offset of the message.
    pub offset: usize,
    /// The record bytes, without the message id and without trailing padding.
    pub bytes: Vec<u8>,
}

/// One item of the data section.
#[derive(Debug, Clone, PartialEq)]
pub enum Item {
    /// A data record.
    Data(DataRecord),
    /// A subscription made after the first data (index into subscriptions).
    Subscribe(usize),
    /// An information message in the data section.
    Info(InfoEntry),
    /// A parameter change.
    Param {
        /// Name.
        name: String,
        /// New value.
        value: ParamValue,
    },
    /// A logged string (`L`), or a tagged one (`C`, with `tag`).
    Log {
        /// Level 0..=7 (the file has the ASCII digit).
        level: u8,
        /// Tag of a `C` message.
        tag: Option<u16>,
        /// Time in microseconds.
        time_us: u64,
        /// The text.
        text: String,
    },
    /// A dropout of this many milliseconds.
    Dropout {
        /// Duration.
        ms: u16,
    },
    /// A synchronisation message.
    Sync,
    /// A message of a type this reader skips (`Q`, `M`, `R`, unknown letters).
    Unknown {
        /// The type letter.
        msg_type: u8,
        /// Payload length.
        len: usize,
    },
}

/// A message cut by the end of the file.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Truncated {
    /// File offset where the message starts.
    pub offset: usize,
    /// Bytes the message needs (header included); 3 if even the header is cut.
    pub needed: usize,
    /// Bytes the file has from `offset`.
    pub available: usize,
}

/// A parsed ULog file.
#[derive(Debug, Clone, PartialEq)]
pub struct UlogFile {
    /// Start time from the file header, microseconds.
    pub start_us: u64,
    /// File format version byte.
    pub version: u8,
    /// Compatibility flags (zero if no flag-bits message).
    pub compat_flags: [u8; 8],
    /// Incompatibility flags.
    pub incompat_flags: [u8; 8],
    /// Appended-data offsets.
    pub appended_offsets: [u64; 3],
    /// Every format, in file order.
    pub formats: Vec<FormatDef>,
    /// Information messages of the definitions (`I`).
    pub info: Vec<InfoEntry>,
    /// Multi information messages of the definitions (`M`).
    pub info_multi: Vec<InfoEntry>,
    /// Initial parameters.
    pub initial_params: Vec<ParamEntry>,
    /// Default parameters.
    pub default_params: Vec<DefaultParam>,
    /// Subscriptions, in file order (those of the definitions section first).
    pub subscriptions: Vec<Subscription>,
    /// Every item of the data section, in file order.
    pub items: Vec<Item>,
    /// Set when the last message is cut by the end of the file.
    pub truncated: Option<Truncated>,
}

struct Cursor {
    at: usize,
}

fn u16_at(b: &[u8], at: usize) -> Option<u16> {
    Some(u16::from_le_bytes(b.get(at..at + 2)?.try_into().ok()?))
}

fn u64_at(b: &[u8], at: usize) -> Option<u64> {
    Some(u64::from_le_bytes(b.get(at..at + 8)?.try_into().ok()?))
}

fn bad(msg_type: u8, why: &'static str) -> ReadErrorKind {
    ReadErrorKind::BadMessage { msg_type, why }
}

/// Splits `key_len, key, value` into an entry.
fn parse_keyed(msg_type: u8, p: &[u8], continued: bool) -> Result<InfoEntry, ReadErrorKind> {
    let key_len = usize::from(*p.first().ok_or(bad(msg_type, "empty payload"))?);
    let key = p
        .get(1..1 + key_len)
        .ok_or(bad(msg_type, "key longer than the message"))?;
    let value = p.get(1 + key_len..).unwrap_or(&[]);
    let key = core::str::from_utf8(key).map_err(|_| bad(msg_type, "key is not text"))?;
    let (ty, name) = key
        .split_once(' ')
        .ok_or(bad(msg_type, "key is not `type name`"))?;
    Ok(InfoEntry {
        ty: ty.to_string(),
        name: name.to_string(),
        value: value.to_vec(),
        continued,
    })
}

fn param_of(msg_type: u8, e: &InfoEntry) -> Result<ParamValue, ReadErrorKind> {
    let four: [u8; 4] = e
        .value
        .as_slice()
        .try_into()
        .map_err(|_| bad(msg_type, "parameter value is not 4 bytes"))?;
    match e.ty.as_str() {
        "int32_t" => Ok(ParamValue::I32(i32::from_le_bytes(four))),
        "float" => Ok(ParamValue::F32(f32::from_bits(u32::from_le_bytes(four)))),
        _ => Err(bad(msg_type, "parameter type is neither int32_t nor float")),
    }
}

impl UlogFile {
    /// Parses a whole file.
    pub fn parse(bytes: &[u8]) -> Result<Self, ReadError> {
        let err = |offset: usize, kind: ReadErrorKind| ReadError { offset, kind };
        if bytes.len() < crate::msg::FILE_HEADER_LEN {
            return Err(err(0, ReadErrorKind::TooShort));
        }
        if bytes.get(..7) != Some(&crate::msg::MAGIC[..]) {
            return Err(err(0, ReadErrorKind::BadMagic));
        }
        let mut file = UlogFile {
            start_us: u64_at(bytes, 8).unwrap_or(0),
            version: bytes.get(7).copied().unwrap_or(0),
            compat_flags: [0; 8],
            incompat_flags: [0; 8],
            appended_offsets: [0; 3],
            formats: Vec::new(),
            info: Vec::new(),
            info_multi: Vec::new(),
            initial_params: Vec::new(),
            default_params: Vec::new(),
            subscriptions: Vec::new(),
            items: Vec::new(),
            truncated: None,
        };
        let mut pending: Vec<(String, String, usize)> = Vec::new();
        let mut pending_names: std::collections::BTreeSet<String> = Default::default();
        let mut format_index: BTreeMap<String, usize> = BTreeMap::new();
        let mut by_id: BTreeMap<u16, usize> = BTreeMap::new();
        let mut in_data = false;
        let mut cur = Cursor {
            at: crate::msg::FILE_HEADER_LEN,
        };

        while cur.at < bytes.len() {
            let at = cur.at;
            let avail = bytes.len() - at;
            let truncated = |needed: usize| Truncated {
                offset: at,
                needed,
                available: avail,
            };
            let (size, ty) = match (u16_at(bytes, at), bytes.get(at + 2)) {
                // As pyulog: a size-0, type-0 message in the data is corruption
                // (a zero-filled tail); keep everything before it.
                (Some(0), Some(0)) if in_data => {
                    file.truncated = Some(truncated(3));
                    break;
                }
                (Some(s), Some(t)) => (usize::from(s), *t),
                _ => {
                    return if in_data {
                        file.truncated = Some(truncated(3));
                        break;
                    } else {
                        Err(err(
                            at,
                            ReadErrorKind::Truncated {
                                needed: 3,
                                available: avail,
                            },
                        ))
                    };
                }
            };
            let Some(p) = bytes.get(at + 3..at + 3 + size) else {
                return if in_data {
                    file.truncated = Some(truncated(3 + size));
                    break;
                } else {
                    Err(err(
                        at,
                        ReadErrorKind::Truncated {
                            needed: 3 + size,
                            available: avail,
                        },
                    ))
                };
            };
            cur.at = at + 3 + size;

            // The first A, L or C message ends the definitions.
            if !in_data
                && matches!(
                    ty,
                    kind::ADD_LOGGED | kind::LOGGED_STRING | kind::TAGGED_STRING
                )
            {
                in_data = true;
                end_definitions(&mut file, &pending).map_err(|(off, k)| err(off, k))?;
            }

            let r: Result<(), ReadErrorKind> = (|| {
                match ty {
                    kind::FLAG_BITS => {
                        if in_data {
                            return Err(ReadErrorKind::DefinitionAfterSubscription(ty));
                        }
                        if at != crate::msg::FILE_HEADER_LEN {
                            return Err(ReadErrorKind::FlagBitsNotFirst);
                        }
                        if p.len() < 40 {
                            return Err(bad(ty, "shorter than 40 bytes"));
                        }
                        file.compat_flags
                            .copy_from_slice(p.get(0..8).unwrap_or(&[0; 8]));
                        file.incompat_flags
                            .copy_from_slice(p.get(8..16).unwrap_or(&[0; 8]));
                        for (i, o) in file.appended_offsets.iter_mut().enumerate() {
                            *o = u64_at(p, 16 + 8 * i).unwrap_or(0);
                        }
                        if file.incompat_flags != [0; 8] {
                            return Err(ReadErrorKind::UnknownIncompatFlags(file.incompat_flags));
                        }
                    }
                    kind::FORMAT => {
                        if in_data {
                            return Err(ReadErrorKind::DefinitionAfterSubscription(ty));
                        }
                        let text = core::str::from_utf8(p).map_err(|_| bad(ty, "not text"))?;
                        let (name, fields) = text
                            .split_once(':')
                            .ok_or(bad(ty, "no ':' after the name"))?;
                        if !pending_names.insert(name.to_string()) {
                            return Err(bad(ty, "format defined twice"));
                        }
                        pending.push((name.to_string(), fields.to_string(), at));
                    }
                    kind::INFO | kind::MULTI_INFO | kind::PARAMETER | kind::DEFAULT_PARAMETER => {
                        let e = match ty {
                            kind::MULTI_INFO => {
                                let c = *p.first().ok_or(bad(ty, "empty payload"))?;
                                parse_keyed(ty, p.get(1..).unwrap_or(&[]), c != 0)?
                            }
                            kind::DEFAULT_PARAMETER => {
                                parse_keyed(ty, p.get(1..).unwrap_or(&[]), false)?
                            }
                            _ => parse_keyed(ty, p, false)?,
                        };
                        match (ty, in_data) {
                            (kind::INFO, false) => file.info.push(e),
                            (kind::INFO, true) => file.items.push(Item::Info(e)),
                            (kind::MULTI_INFO, false) => file.info_multi.push(e),
                            (kind::PARAMETER, false) => {
                                let value = param_of(ty, &e)?;
                                file.initial_params.push(ParamEntry {
                                    name: e.name,
                                    value,
                                });
                            }
                            (kind::PARAMETER, true) => {
                                let value = param_of(ty, &e)?;
                                file.items.push(Item::Param {
                                    name: e.name,
                                    value,
                                });
                            }
                            (kind::DEFAULT_PARAMETER, false) => {
                                let value = param_of(ty, &e)?;
                                let default_types = *p.first().unwrap_or(&0);
                                file.default_params.push(DefaultParam {
                                    default_types,
                                    name: e.name,
                                    value,
                                });
                            }
                            _ => file.items.push(Item::Unknown {
                                msg_type: ty,
                                len: p.len(),
                            }),
                        }
                    }
                    kind::ADD_LOGGED => {
                        if p.len() < 3 {
                            return Err(bad(ty, "shorter than 3 bytes"));
                        }
                        let multi_id = p.first().copied().unwrap_or(0);
                        let msg_id = u16_at(p, 1).unwrap_or(0);
                        let name = core::str::from_utf8(p.get(3..).unwrap_or(&[]))
                            .map_err(|_| bad(ty, "name is not text"))?;
                        if format_index.is_empty() {
                            for (k, f) in file.formats.iter().enumerate() {
                                format_index.insert(f.name.clone(), k);
                            }
                        }
                        let format = format_index
                            .get(name)
                            .copied()
                            .ok_or_else(|| ReadErrorKind::UnknownTopic(name.to_string()))?;
                        if by_id.contains_key(&msg_id) {
                            return Err(ReadErrorKind::DuplicateMessageId(msg_id));
                        }
                        by_id.insert(msg_id, file.subscriptions.len());
                        file.subscriptions.push(Subscription {
                            multi_id,
                            msg_id,
                            name: name.to_string(),
                            format,
                            offset: at,
                        });
                        // Subscriptions before any other item are the file's own list;
                        // later ones are items, to keep the order.
                        if !file.items.is_empty() {
                            file.items
                                .push(Item::Subscribe(file.subscriptions.len() - 1));
                        }
                    }
                    kind::DATA => {
                        if !in_data {
                            return Err(ReadErrorKind::UnexpectedInDefinitions(ty));
                        }
                        let id = u16_at(p, 0).ok_or(bad(ty, "shorter than 2 bytes"))?;
                        let &sub = by_id.get(&id).ok_or(ReadErrorKind::UnknownMessageId(id))?;
                        let s = file
                            .subscriptions
                            .get(sub)
                            .ok_or(ReadErrorKind::UnknownMessageId(id))?;
                        let layout = &file
                            .formats
                            .get(s.format)
                            .ok_or(ReadErrorKind::UnknownMessageId(id))?
                            .layout;
                        let rec = p.get(2..).unwrap_or(&[]);
                        if rec.len() != layout.size_no_trailing_padding {
                            return Err(ReadErrorKind::RecordLength {
                                topic: s.name.clone(),
                                expected: layout.size_no_trailing_padding,
                                got: rec.len(),
                            });
                        }
                        file.items.push(Item::Data(DataRecord {
                            subscription: sub,
                            offset: at,
                            bytes: rec.to_vec(),
                        }));
                    }
                    kind::LOGGED_STRING | kind::TAGGED_STRING => {
                        let tagged = ty == kind::TAGGED_STRING;
                        let head = if tagged { 11 } else { 9 };
                        if p.len() < head {
                            return Err(bad(ty, "shorter than its fixed part"));
                        }
                        let digit = p.first().copied().unwrap_or(0);
                        if !(b'0'..=b'7').contains(&digit) {
                            return Err(bad(ty, "level is not an ASCII digit 0..7"));
                        }
                        let (tag, t_at) = if tagged { (u16_at(p, 1), 3) } else { (None, 1) };
                        file.items.push(Item::Log {
                            level: digit - b'0',
                            tag,
                            time_us: u64_at(p, t_at).unwrap_or(0),
                            text: String::from_utf8_lossy(p.get(head..).unwrap_or(&[]))
                                .into_owned(),
                        });
                    }
                    kind::SYNC => {
                        if p != SYNC_MAGIC {
                            return Err(ReadErrorKind::BadSync);
                        }
                        if in_data {
                            file.items.push(Item::Sync);
                        }
                    }
                    kind::DROPOUT => {
                        if p.len() != 2 {
                            return Err(bad(ty, "payload is not 2 bytes"));
                        }
                        if in_data {
                            file.items.push(Item::Dropout {
                                ms: u16_at(p, 0).unwrap_or(0),
                            });
                        }
                    }
                    kind::REMOVE_LOGGED => {
                        if !in_data {
                            return Err(ReadErrorKind::UnexpectedInDefinitions(ty));
                        }
                        file.items.push(Item::Unknown {
                            msg_type: ty,
                            len: p.len(),
                        });
                    }
                    other => {
                        if in_data {
                            file.items.push(Item::Unknown {
                                msg_type: other,
                                len: p.len(),
                            });
                        }
                    }
                }
                Ok(())
            })();
            r.map_err(|k| err(at, k))?;
        }
        if !in_data {
            end_definitions(&mut file, &pending).map_err(|(off, k)| err(off, k))?;
        }
        Ok(file)
    }
}

fn end_definitions(
    file: &mut UlogFile,
    pending: &[(String, String, usize)],
) -> Result<(), (usize, ReadErrorKind)> {
    let defs: Vec<(&str, &str)> = pending
        .iter()
        .map(|(n, f, _)| (n.as_str(), f.as_str()))
        .collect();
    let mut layouts = format::resolve(&defs).map_err(|(i, e)| {
        let off = pending.get(i).map_or(0, |p| p.2);
        (off, ReadErrorKind::BadFormat(e))
    })?;
    for (name, fields, offset) in pending {
        if let Some(layout) = layouts.remove(name) {
            file.formats.push(FormatDef {
                name: name.clone(),
                fields: fields.clone(),
                layout,
                offset: *offset,
            });
        }
    }
    Ok(())
}

/// Where a field is in a record.
struct Region {
    scalar: Option<Scalar>,
    offset: usize,
    len: usize,
}

/// One data record with access by field name.
#[derive(Debug, Clone, Copy)]
pub struct RecordView<'a> {
    file: &'a UlogFile,
    rec: &'a DataRecord,
}

impl UlogFile {
    /// The information value with this key name.
    pub fn info(&self, name: &str) -> Option<&InfoEntry> {
        self.info.iter().find(|e| e.name == name)
    }

    /// The text of a `char[N]` information value.
    pub fn info_str(&self, name: &str) -> Option<&str> {
        self.info(name).and_then(InfoEntry::as_str)
    }

    /// The layout of the format called `name`.
    pub fn layout(&self, name: &str) -> Option<&Layout> {
        self.formats
            .iter()
            .find(|f| f.name == name)
            .map(|f| &f.layout)
    }

    /// Access to a data record by field name.
    pub fn view<'a>(&'a self, rec: &'a DataRecord) -> RecordView<'a> {
        RecordView { file: self, rec }
    }

    /// Every data record of the topic called `topic` (all instances), in file order.
    pub fn records<'a>(&'a self, topic: &'a str) -> impl Iterator<Item = RecordView<'a>> + 'a {
        self.items.iter().filter_map(move |i| match i {
            Item::Data(r)
                if self
                    .subscriptions
                    .get(r.subscription)
                    .is_some_and(|s| s.name == topic) =>
            {
                Some(self.view(r))
            }
            _ => None,
        })
    }
}

fn parse_segment(seg: &str) -> Option<(&str, Option<usize>)> {
    match seg.split_once('[') {
        None => Some((seg, None)),
        Some((name, rest)) => {
            let idx = rest.strip_suffix(']')?.parse::<usize>().ok()?;
            Some((name, Some(idx)))
        }
    }
}

macro_rules! scalar_accessors {
    ($($(#[$m:meta])* $name:ident, $t:ty, $variant:ident;)*) => {$(
        $(#[$m])*
        pub fn $name(&self, path: &str) -> Option<$t> {
            let b = self.scalar_bytes(path, Scalar::$variant)?;
            Some(<$t>::from_le_bytes(b.try_into().ok()?))
        }
    )*};
}

impl<'a> RecordView<'a> {
    /// The topic name.
    pub fn topic(&self) -> &'a str {
        self.file
            .subscriptions
            .get(self.rec.subscription)
            .map_or("", |s| s.name.as_str())
    }

    /// The record bytes (without trailing padding).
    pub fn raw(&self) -> &'a [u8] {
        &self.rec.bytes
    }

    /// File offset of the message.
    pub fn offset(&self) -> usize {
        self.rec.offset
    }

    fn locate(&self, path: &str) -> Option<Region> {
        let sub = self.file.subscriptions.get(self.rec.subscription)?;
        let mut layout = &self.file.formats.get(sub.format)?.layout;
        let mut base = 0usize;
        let mut segs = path.split('.').peekable();
        while let Some(seg) = segs.next() {
            let (name, idx) = parse_segment(seg)?;
            let f = layout.fields.iter().find(|f| f.name == name)?;
            let last = segs.peek().is_none();
            let (offset, len) = match (f.array, idx) {
                (None, None) => (f.offset, f.size),
                (Some(n), Some(i)) if i < n => (f.offset + i * f.elem_size, f.elem_size),
                (Some(_), None) if last => {
                    return Some(Region {
                        scalar: None,
                        offset: base + f.offset,
                        len: f.size,
                    });
                }
                _ => return None,
            };
            match &f.ty {
                format::FieldType::Scalar(sc) if last => {
                    return Some(Region {
                        scalar: Some(*sc),
                        offset: base + offset,
                        len,
                    });
                }
                format::FieldType::Nested(n) if !last => {
                    layout = self.file.layout(n)?;
                    base += offset;
                }
                format::FieldType::Nested(_) => {
                    return Some(Region {
                        scalar: None,
                        offset: base + offset,
                        len,
                    });
                }
                format::FieldType::Scalar(_) => return None,
            }
        }
        None
    }

    fn scalar_bytes(&self, path: &str, want: Scalar) -> Option<&'a [u8]> {
        let r = self.locate(path)?;
        if r.scalar != Some(want) {
            return None;
        }
        self.rec.bytes.get(r.offset..r.offset + r.len)
    }

    /// The bytes of a field, a whole array (`name`), an element (`name[i]`), or a nested value.
    /// `None` if the field is unknown or lies in the trailing padding.
    pub fn field_bytes(&self, path: &str) -> Option<&'a [u8]> {
        let r = self.locate(path)?;
        self.rec.bytes.get(r.offset..r.offset + r.len)
    }

    scalar_accessors! {
        /// `uint8_t` field.
        u8, u8, U8;
        /// `int8_t` field.
        i8, i8, I8;
        /// `uint16_t` field.
        u16, u16, U16;
        /// `int16_t` field.
        i16, i16, I16;
        /// `uint32_t` field.
        u32, u32, U32;
        /// `int32_t` field.
        i32, i32, I32;
        /// `uint64_t` field.
        u64, u64, U64;
        /// `int64_t` field.
        i64, i64, I64;
    }

    /// `float` field.
    pub fn f32(&self, path: &str) -> Option<f32> {
        let b = self.scalar_bytes(path, Scalar::F32)?;
        Some(f32::from_bits(u32::from_le_bytes(b.try_into().ok()?)))
    }

    /// `double` field.
    pub fn f64(&self, path: &str) -> Option<f64> {
        let b = self.scalar_bytes(path, Scalar::F64)?;
        Some(f64::from_bits(u64::from_le_bytes(b.try_into().ok()?)))
    }

    /// `bool` field.
    pub fn bool(&self, path: &str) -> Option<bool> {
        self.scalar_bytes(path, Scalar::Bool)?
            .first()
            .map(|b| *b != 0)
    }

    /// One `char` of a field.
    pub fn char(&self, path: &str) -> Option<u8> {
        self.scalar_bytes(path, Scalar::Char)?.first().copied()
    }
}
