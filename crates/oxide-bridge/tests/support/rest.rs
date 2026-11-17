// SPDX-License-Identifier: AGPL-3.0-or-later
//! Drives the exchange with the vehicle at rest, for the tests that run the
//! real simulator program: position 0, level, zero rates, specific force
//! `(0, 0, -9.80665)` in the body frame, a full battery, no radio. The time
//! stamp of exchange `k` is `timestamp_ns(500, first_k + k)`; `first_k` is 1
//! unless a test says otherwise (a first stamp of zero works, but the run is
//! then not repeatable: see `a_first_time_stamp_of_zero_...`). The client is
//! polled once after every exchange. Our own heartbeat goes out at 1 Hz of
//! simulated time.

#![allow(dead_code)]

use std::fmt;
use std::time::Duration;

use oxide_bridge::link::{Link, LinkError};
use oxide_bridge::mav::{MavClient, MavError, MavEvent};
use oxide_bridge::packet::ServoPacket;
use oxide_bridge::reply::{BatteryReading, Reply, ReplyError, timestamp_ns};

/// The step of a bridged flight, microseconds.
pub const STEP_US: u64 = 500;

/// An event and the number of exchanges done when it was read, which is the
/// simulated time of the read in steps of [`STEP_US`].
#[derive(Clone, Debug, PartialEq)]
pub struct Timed {
    pub exchange: u64,
    pub event: MavEvent,
}

#[derive(Debug)]
pub enum RestError {
    Link(LinkError),
    Mav(MavError),
    Reply(ReplyError),
}

impl fmt::Display for RestError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Link(e) => write!(f, "exchange: {e}"),
            Self::Mav(e) => write!(f, "mavlink: {e}"),
            Self::Reply(e) => write!(f, "reply: {e}"),
        }
    }
}

impl std::error::Error for RestError {}

pub struct Rest {
    pub link: Link,
    pub client: MavClient,
    /// Exchanges done.
    pub exchanges: u64,
    /// The step number of the first time stamp.
    pub first_k: u64,
    /// The frame count of the program's first packet.
    pub first_frame: u32,
    pub reply_timeout: Duration,
    /// Send our own heartbeat at 1 Hz of simulated time, from exchange 0
    /// (what a ground station does; the program sends no status text to a
    /// link it has not heard).
    pub gcs_heartbeat: bool,
    pub events: Vec<Timed>,
    pub packets: Vec<ServoPacket>,
    line: String,
}

impl Rest {
    /// Waits for the program's first packet (it sends it once a client is
    /// connected to its serial 0, which `client` is).
    pub fn new(
        mut link: Link,
        client: MavClient,
        first_k: u64,
        reply_timeout: Duration,
        first_packet_timeout: Duration,
    ) -> Result<Self, RestError> {
        link.wait_for_first_packet(first_packet_timeout)
            .map_err(RestError::Link)?;
        let first_frame = link.last_frame_count().unwrap_or(0);
        Ok(Self {
            link,
            client,
            exchanges: 0,
            first_k,
            first_frame,
            reply_timeout,
            gcs_heartbeat: true,
            events: Vec::new(),
            packets: Vec::new(),
            line: String::new(),
        })
    }

    /// One exchange, then one poll of the client.
    pub fn step(&mut self) -> Result<(), RestError> {
        if self.gcs_heartbeat && self.exchanges.is_multiple_of(1_000_000 / STEP_US) {
            self.client.heartbeat().map_err(RestError::Mav)?;
        }
        let reply = Reply {
            timestamp_ns: timestamp_ns(STEP_US, self.first_k + self.exchanges),
            gyro: [0.0; 3],
            accel_body: [0.0, 0.0, -9.80665],
            position: [0.0; 3],
            velocity: [0.0; 3],
            quaternion: [1.0, 0.0, 0.0, 0.0],
            battery: Some(BatteryReading {
                voltage: 16.8,
                current: 0.0,
            }),
            rc: None,
        };
        self.line.clear();
        reply.write(&mut self.line).map_err(RestError::Reply)?;
        let packet = self
            .link
            .exchange(&self.line, self.reply_timeout)
            .map_err(RestError::Link)?;
        self.packets.push(packet);
        self.exchanges += 1;
        let mut got = Vec::new();
        self.client.poll(&mut got).map_err(RestError::Mav)?;
        let exchange = self.exchanges;
        self.events
            .extend(got.into_iter().map(|event| Timed { exchange, event }));
        Ok(())
    }

    /// `n` exchanges.
    pub fn steps(&mut self, n: u64) -> Result<(), RestError> {
        for _ in 0..n {
            self.step()?;
        }
        Ok(())
    }

    /// Simulated seconds, as a whole number of steps.
    pub fn run_s(&mut self, seconds: f64) -> Result<(), RestError> {
        let n = (seconds * 1.0e6 / STEP_US as f64).round() as u64;
        self.steps(n)
    }

    /// Simulated time of `exchange`, seconds.
    pub fn seconds(exchange: u64) -> f64 {
        exchange as f64 * STEP_US as f64 * 1.0e-6
    }

    /// The number of exchanges done when the first event satisfying `want`
    /// was read, from exchange `from` on.
    pub fn first_at(&self, from: u64, want: impl Fn(&MavEvent) -> bool) -> Option<u64> {
        self.events
            .iter()
            .find(|t| t.exchange >= from && want(&t.event))
            .map(|t| t.exchange)
    }

    /// The events of the half-open exchange range `[from, to)` that satisfy
    /// `want`.
    pub fn count(&self, from: u64, to: u64, want: impl Fn(&MavEvent) -> bool) -> usize {
        self.events
            .iter()
            .filter(|t| t.exchange >= from && t.exchange < to && want(&t.event))
            .count()
    }
}
