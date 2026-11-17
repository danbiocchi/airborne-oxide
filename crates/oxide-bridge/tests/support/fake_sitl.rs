// SPDX-License-Identifier: AGPL-3.0-or-later
//! A stand-in for the autopilot's simulator build, for the tests only. It is
//! written from the stated facts of the interface (ADR-0006, the part's
//! specification) and speaks the simulator's side of the protocol: it sends a
//! motor packet, waits for the reply line (and sends the packet again after a
//! second without one, with the same frame count), reads the few numbers it
//! needs from the line by key, and computes the next pulses.
//!
//! Its "flight controller" is a few lines: arm at a given time, then one
//! collective thrust from a PD on height, the same pulse on all four outputs.

use std::net::{SocketAddr, UdpSocket};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

/// What the stand-in does.
#[derive(Clone, Debug)]
pub struct Options {
    /// The bridge's port on 127.0.0.1.
    pub bridge_port: u16,
    /// The height the controller holds, m.
    pub target_height_m: f64,
    /// The time from which the motors are armed, s (simulated time, from the
    /// reply's time stamp).
    pub arm_at_s: f64,
    /// The frame count of the first packet.
    pub first_frame: u32,
    /// Send the same packet again after this long without a reply.
    pub resend_after: Duration,
    /// Treat the reply to this frame as lost: record it, then send the same
    /// frame again at once, as the simulator does a second later.
    pub lose_reply_at: Option<u32>,
    /// After the reply to this frame, send the frame number two further on.
    pub skip_after: Option<u32>,
    /// After the reply to this frame, send nothing more.
    pub stop_after: Option<u32>,
    /// Ignore a reply that does not start with a newline.
    pub require_leading_newline: bool,
}

impl Options {
    pub fn new(bridge_port: u16) -> Self {
        Self {
            bridge_port,
            target_height_m: 2.0,
            arm_at_s: 0.5,
            first_frame: 1,
            resend_after: Duration::from_secs(1),
            lose_reply_at: None,
            skip_after: None,
            stop_after: None,
            require_leading_newline: true,
        }
    }
}

/// What the stand-in saw, once it is stopped.
#[derive(Clone, Debug, Default)]
pub struct Report {
    /// Every reply it accepted, in order (a "lost" one included).
    pub replies: Vec<String>,
    /// Replies it ignored for lacking the leading newline.
    pub ignored: u32,
    /// Packets it sent, resends included.
    pub packets: u32,
    /// How many of those were resends after a silence.
    pub resends: u32,
}

/// A running stand-in.
pub struct FakeSitl {
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<Report>>,
}

impl FakeSitl {
    pub fn spawn(options: Options) -> Self {
        let socket = UdpSocket::bind("127.0.0.1:0").expect("the stand-in binds a socket");
        socket
            .set_read_timeout(Some(Duration::from_millis(10)))
            .expect("read time-out");
        let stop = Arc::new(AtomicBool::new(false));
        let flag = Arc::clone(&stop);
        let thread = std::thread::spawn(move || run(&socket, &options, &flag));
        Self {
            stop,
            thread: Some(thread),
        }
    }

    /// Stops the thread and returns what it saw.
    pub fn finish(mut self) -> Report {
        self.stop.store(true, Ordering::SeqCst);
        match self.thread.take() {
            Some(t) => t.join().expect("the stand-in thread did not panic"),
            None => Report::default(),
        }
    }
}

impl Drop for FakeSitl {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::SeqCst);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

fn packet(frame: u32, pwm: &[u16; 4]) -> Vec<u8> {
    let mut b = Vec::with_capacity(40);
    b.extend_from_slice(&18458u16.to_le_bytes());
    b.extend_from_slice(&2000u16.to_le_bytes());
    b.extend_from_slice(&frame.to_le_bytes());
    for k in 0..16 {
        let v = pwm.get(k).copied().unwrap_or(0);
        b.extend_from_slice(&v.to_le_bytes());
    }
    b
}

/// The number at `index` in the list that follows `key` (a substring scan,
/// as the real thing reads its keys), e.g. `"position":[` and 2.
fn list_number(text: &str, key: &str, index: usize) -> Option<f64> {
    let start = text.find(key)? + key.len();
    let rest = text.get(start..)?;
    let end = rest.find(']')?;
    rest.get(..end)?.split(',').nth(index)?.trim().parse().ok()
}

fn scalar_number(text: &str, key: &str) -> Option<f64> {
    let start = text.find(key)? + key.len();
    let rest = text.get(start..)?;
    let end = rest.find(',')?;
    rest.get(..end)?.parse().ok()
}

/// The collective pulse for the state in `text`.
fn controller(text: &str, o: &Options) -> [u16; 4] {
    let t = scalar_number(text, "\"timestamp\":").unwrap_or(0.0);
    let height = -list_number(text, "\"position\":[", 2).unwrap_or(0.0);
    let climb = -list_number(text, "\"velocity\":[", 2).unwrap_or(0.0);
    if t < o.arm_at_s {
        return [1000; 4];
    }
    let u = (HOVER + KP * (o.target_height_m - height) - KD * climb).clamp(0.0, 1.0);
    // The inverse of the motor map with spin 0.15 to 0.95.
    let a = 0.15 + u * 0.8;
    let pwm = (1000.0 + a * 1000.0 + 0.5) as u16;
    [pwm; 4]
}

const HOVER: f64 = 0.40;
const KP: f64 = 0.36;
const KD: f64 = 0.18;

fn run(socket: &UdpSocket, o: &Options, stop: &AtomicBool) -> Report {
    let to = SocketAddr::from(([127, 0, 0, 1], o.bridge_port));
    let mut report = Report::default();
    let mut frame = o.first_frame;
    let mut pwm = [1000u16; 4];
    let mut halted = false;
    let mut lost_done = false;
    let mut buf = [0u8; 4096];

    let _ = socket.send_to(&packet(frame, &pwm), to);
    report.packets += 1;
    let mut last_sent = Instant::now();

    while !stop.load(Ordering::SeqCst) {
        match socket.recv_from(&mut buf) {
            Ok((n, _)) => {
                let text = String::from_utf8_lossy(buf.get(..n).unwrap_or(&[])).into_owned();
                if o.require_leading_newline && !text.starts_with('\n') {
                    report.ignored += 1;
                    continue;
                }
                report.replies.push(text.clone());
                if o.lose_reply_at == Some(frame) && !lost_done {
                    lost_done = true;
                    let _ = socket.send_to(&packet(frame, &pwm), to);
                    report.packets += 1;
                    last_sent = Instant::now();
                    continue;
                }
                pwm = controller(&text, o);
                if o.stop_after == Some(frame) {
                    halted = true;
                    continue;
                }
                frame = frame.wrapping_add(if o.skip_after == Some(frame) { 2 } else { 1 });
                let _ = socket.send_to(&packet(frame, &pwm), to);
                report.packets += 1;
                last_sent = Instant::now();
            }
            Err(_) => {
                // Nothing yet (or the peer is gone): after a silence, send the
                // same packet again with the same frame count.
                if !halted && last_sent.elapsed() >= o.resend_after {
                    let _ = socket.send_to(&packet(frame, &pwm), to);
                    report.packets += 1;
                    report.resends += 1;
                    last_sent = Instant::now();
                }
            }
        }
    }
    report
}
