// SPDX-License-Identifier: AGPL-3.0-or-later
//! Builds ULog files that hold replay records, and reads them back.

use crate::msg::{self, InfoValue, ParamValue as MsgParam};
use crate::reader::UlogFile;
use crate::record::{self, RecordWriter, Topic};
use crate::replay::TOPICS;

/// The tick time every single-record test writes (and expects back).
pub const T: oxide_core::time::Micros = oxide_core::time::Micros(0x1122_3344_5566_7788);

/// A file under construction: definitions for every replay topic, then data.
#[derive(Default)]
pub struct Builder {
    infos: Vec<(String, u64)>,
    params: Vec<(String, MsgParam)>,
    data: Vec<u8>,
}

pub fn id_of(topic: &Topic) -> u16 {
    TOPICS.iter().position(|t| t == topic).unwrap() as u16
}

/// The bytes of one record (no message header, no id).
pub fn record_bytes(topic: &Topic, fill: impl FnOnce(&mut RecordWriter<'_, Vec<u8>>)) -> Vec<u8> {
    let mut v = Vec::new();
    record::data(&mut v, 0, topic, fill).unwrap();
    v[5..].to_vec()
}

impl Builder {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn info_u64(&mut self, name: &str, v: u64) -> &mut Self {
        self.infos.push((name.to_string(), v));
        self
    }

    pub fn param(&mut self, name: &str, v: MsgParam) -> &mut Self {
        self.params.push((name.to_string(), v));
        self
    }

    pub fn push(
        &mut self,
        topic: &Topic,
        fill: impl FnOnce(&mut RecordWriter<'_, Vec<u8>>),
    ) -> &mut Self {
        record::data(&mut self.data, id_of(topic), topic, fill).unwrap();
        self
    }

    pub fn push_raw(&mut self, topic: &Topic, record: &[u8]) -> &mut Self {
        msg::data_raw(&mut self.data, id_of(topic), record).unwrap();
        self
    }

    pub fn bytes(&self) -> Vec<u8> {
        let mut f = Vec::new();
        msg::file_header(&mut f, 0).unwrap();
        msg::flag_bits(&mut f, [0; 8], [0; 8]).unwrap();
        for t in &TOPICS {
            t.write_format(&mut f).unwrap();
        }
        msg::info(&mut f, "sys_name", InfoValue::Str("Oxide")).unwrap();
        for (n, v) in &self.infos {
            msg::info(&mut f, n, InfoValue::U64(*v)).unwrap();
        }
        for (n, v) in &self.params {
            msg::parameter(&mut f, n, *v).unwrap();
        }
        for (i, t) in TOPICS.iter().enumerate() {
            t.add_logged(&mut f, 0, i as u16).unwrap();
        }
        f.extend_from_slice(&self.data);
        f
    }

    pub fn file(&self) -> UlogFile {
        UlogFile::parse(&self.bytes()).unwrap()
    }
}

/// A file with one record of `topic`.
pub fn one(topic: &Topic, fill: impl FnOnce(&mut RecordWriter<'_, Vec<u8>>)) -> UlogFile {
    Builder::new().push(topic, fill).file()
}

/// A file with one record of `topic` given as bytes.
pub fn one_raw(topic: &Topic, record: &[u8]) -> UlogFile {
    Builder::new().push_raw(topic, record).file()
}
