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
    /// Wait this long before the first packet (a slow start-up).
    pub start_delay: Duration,
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
            start_delay: Duration::ZERO,
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
    /// Replies that broke the wire format ([`check_line`]); each is recorded
    /// in `replies` too, and answered, so that the first one is in the report
    /// and the flight goes on.
    pub rejected: u32,
    /// The first reason a reply was rejected.
    pub first_rejection: Option<String>,
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
    number(rest.get(..end)?.split(',').nth(index)?)
}

/// A number as the wire format writes it: an optional minus, digits, a point
/// and digits. No exponent, no space, no `NaN`, no `inf`, no plus sign.
fn number(item: &str) -> Option<f64> {
    let body = item.strip_prefix('-').unwrap_or(item);
    let (int, frac) = body.split_once('.')?;
    let digits = |t: &str| !t.is_empty() && t.bytes().all(|b| b.is_ascii_digit());
    (digits(int) && digits(frac)).then(|| item.parse().ok())?
}

fn scalar_number(text: &str, key: &str) -> Option<f64> {
    let start = text.find(key)? + key.len();
    let rest = text.get(start..)?;
    let end = rest.find(',')?;
    number(rest.get(..end)?)
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
    let mut last_stamp: Option<u64> = None;

    std::thread::sleep(o.start_delay);
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
                if let Err(why) = check_line(&text, &mut last_stamp) {
                    report.rejected += 1;
                    report.first_rejection.get_or_insert(why);
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

/// A strict reading of one reply line, as the real simulator build would
/// need it: the leading and the trailing newline, every key present, in this
/// order, with no space after a colon or anywhere, every number a fixed-point
/// decimal with the stated number of fractional digits (no exponent, no
/// `NaN`, no `inf`), the radio values integers, and the time stamp growing.
/// Returns the first thing that is wrong.
pub fn check_line(text: &str, last_stamp: &mut Option<u64>) -> Result<(), String> {
    let body = text
        .strip_prefix('\n')
        .ok_or("no leading newline")?
        .strip_suffix("}\n")
        .ok_or("no trailing `}` and newline")?;
    if body.contains(|c: char| c.is_whitespace()) {
        return Err("whitespace inside the line".into());
    }
    let mut rest = body;
    let mut eat = |what: &str| -> Result<(), String> {
        rest = rest
            .strip_prefix(what)
            .ok_or_else(|| format!("expected `{what}` at `{}`", &rest[..rest.len().min(30)]))?;
        Ok(())
    };
    // The time stamp: whole seconds, a point and nine digits.
    eat("{\"timestamp\":")?;
    let end = rest.find(',').ok_or("no comma after the timestamp")?;
    let stamp = &rest[..end];
    let (secs, nanos) = stamp.split_once('.').ok_or("timestamp without a point")?;
    if nanos.len() != 9 || !nanos.bytes().all(|b| b.is_ascii_digit()) {
        return Err(format!(
            "timestamp `{stamp}` has not nine fractional digits"
        ));
    }
    let ns = secs
        .parse::<u64>()
        .ok()
        .and_then(|s| s.checked_mul(1_000_000_000))
        .and_then(|s| s.checked_add(nanos.parse::<u64>().ok()?))
        .ok_or_else(|| format!("timestamp `{stamp}` is not a number"))?;
    if last_stamp.is_some_and(|last| ns <= last) {
        return Err(format!("timestamp `{stamp}` does not increase"));
    }
    *last_stamp = Some(ns);
    let mut rest = &rest[end..];
    let lists: [(&str, usize, usize); 5] = [
        (",\"imu\":{\"gyro\":[", 3, 9),
        ("],\"accel_body\":[", 3, 9),
        ("]},\"position\":[", 3, 9),
        ("],\"quaternion\":[", 4, 12),
        ("],\"velocity\":[", 3, 9),
    ];
    for (key, count, digits) in lists {
        rest = rest
            .strip_prefix(key)
            .ok_or_else(|| format!("expected `{key}` at `{}`", &rest[..rest.len().min(30)]))?;
        let end = rest.find(']').ok_or("an unclosed list")?;
        let items: Vec<&str> = rest[..end].split(',').collect();
        if items.len() != count {
            return Err(format!("`{key}` has {} values, not {count}", items.len()));
        }
        for item in items {
            number(item).ok_or_else(|| format!("`{item}` is not a fixed-point number"))?;
            if item.split_once('.').map(|(_, f)| f.len()) != Some(digits) {
                return Err(format!("`{item}` has not {digits} fractional digits"));
            }
        }
        rest = &rest[end..];
    }
    rest = rest
        .strip_prefix(']')
        .ok_or("the velocity list is not closed")?;
    if let Some(battery) = rest.strip_prefix(",\"battery\":{\"voltage\":") {
        let end = battery.find(",\"current\":").ok_or("no battery current")?;
        number(&battery[..end]).ok_or("the battery voltage is not a fixed-point number")?;
        let cur = &battery[end + ",\"current\":".len()..];
        let end = cur.find('}').ok_or("an unclosed battery")?;
        number(&cur[..end]).ok_or("the battery current is not a fixed-point number")?;
        rest = &cur[end + 1..];
    }
    if let Some(rc) = rest.strip_prefix(",\"rc\":{") {
        let rc = rc.strip_suffix('}').ok_or("an unclosed rc")?;
        let items: Vec<&str> = rc.split(',').collect();
        if items.len() != 8 {
            return Err(format!("rc has {} channels, not 8", items.len()));
        }
        for (i, item) in items.iter().enumerate() {
            let want = format!("\"rc_{}\":", i + 1);
            let v = item
                .strip_prefix(want.as_str())
                .ok_or_else(|| format!("expected `{want}` in `{item}`"))?;
            if v.is_empty() || !v.bytes().all(|b| b.is_ascii_digit()) {
                return Err(format!("rc value `{v}` is not an integer"));
            }
        }
        rest = "";
    }
    if !rest.is_empty() {
        return Err(format!(
            "unexpected `{}` before the end",
            &rest[..rest.len().min(30)]
        ));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    const GOOD: &str = concat!(
        "\n{\"timestamp\":0.000500001,",
        "\"imu\":{\"gyro\":[0.100000000,-0.200000000,0.300000000],",
        "\"accel_body\":[0.000000000,0.000000000,-9.806650000]},",
        "\"position\":[1.500000000,-2.250000000,-3.000000000],",
        "\"quaternion\":[1.000000000000,0.000000000000,0.000000000000,0.000000000000],",
        "\"velocity\":[0.500000000,0.000000000,-0.125000000],",
        "\"battery\":{\"voltage\":16.800000000,\"current\":12.500000000},",
        "\"rc\":{\"rc_1\":1500,\"rc_2\":1500,\"rc_3\":1000,\"rc_4\":1500,",
        "\"rc_5\":1000,\"rc_6\":1000,\"rc_7\":1000,\"rc_8\":1000}}\n"
    );

    fn verdict(text: &str) -> Result<(), String> {
        check_line(text, &mut None)
    }

    #[test]
    fn the_checker_accepts_a_good_line_with_and_without_the_optional_sections() {
        verdict(GOOD).expect("good");
        let no_rc = GOOD.replace(
            ",\"rc\":{\"rc_1\":1500,\"rc_2\":1500,\"rc_3\":1000,\"rc_4\":1500,\
             \"rc_5\":1000,\"rc_6\":1000,\"rc_7\":1000,\"rc_8\":1000}",
            "",
        );
        assert!(!no_rc.contains("rc_1"));
        verdict(&no_rc).expect("no rc");
        let bare = no_rc.replace(
            ",\"battery\":{\"voltage\":16.800000000,\"current\":12.500000000}",
            "",
        );
        assert!(!bare.contains("battery"));
        verdict(&bare).expect("neither");
    }

    #[test]
    fn the_checker_refuses_each_way_the_wire_format_can_be_broken() {
        let bad: Vec<(&str, String)> = vec![
            ("no leading newline", GOOD[1..].to_string()),
            ("no trailing newline", GOOD[..GOOD.len() - 1].to_string()),
            (
                "a space after a colon",
                GOOD.replace("\"timestamp\":", "\"timestamp\": "),
            ),
            (
                "a space after a comma",
                GOOD.replace("-0.200000000,", "-0.200000000, "),
            ),
            ("an exponent", GOOD.replace("-2.250000000", "-2.25e0")),
            (
                "an exponent form of a small number",
                GOOD.replace("0.100000000", "1e-1"),
            ),
            ("NaN", GOOD.replace("0.300000000", "NaN")),
            ("inf", GOOD.replace("-3.000000000", "inf")),
            ("a plus sign", GOOD.replace("1.500000000", "+1.500000000")),
            ("too few digits", GOOD.replace("1.500000000,", "1.5,")),
            (
                "too few quaternion digits",
                GOOD.replace("1.000000000000,", "1.000000000,"),
            ),
            ("a missing imu", GOOD.replace("\"imu\"", "\"imx\"")),
            (
                "a missing position",
                GOOD.replace("\"position\"", "\"pos\""),
            ),
            (
                "a missing quaternion",
                GOOD.replace("\"quaternion\"", "\"q\""),
            ),
            (
                "a missing velocity",
                GOOD.replace("\"velocity\"", "\"vel\""),
            ),
            (
                "a missing value",
                GOOD.replace(
                    "[0.500000000,0.000000000,-0.125000000]",
                    "[0.500000000,0.000000000]",
                ),
            ),
            (
                "a float rc value",
                GOOD.replace("\"rc_3\":1000", "\"rc_3\":1000.0"),
            ),
            (
                "rc out of order",
                GOOD.replace("\"rc_1\":1500,\"rc_2\":1500", "\"rc_2\":1500,\"rc_1\":1500"),
            ),
            (
                "keys out of order",
                GOOD.replace("\"position\"", "\"velocity2\""),
            ),
            ("a stray key", GOOD.replace("}}\n", "},\"x\":1}\n")),
        ];
        for (what, text) in &bad {
            assert!(verdict(text).is_err(), "{what} was accepted: {text:?}");
        }
    }

    #[test]
    fn the_checker_wants_the_time_stamp_to_grow() {
        let mut last = None;
        check_line(GOOD, &mut last).expect("first");
        assert!(check_line(GOOD, &mut last).is_err(), "the same stamp twice");
        let later = GOOD.replace("0.000500001", "0.001000002");
        check_line(&later, &mut last).expect("later");
        assert!(check_line(GOOD, &mut last).is_err(), "a step back");
    }

    #[test]
    fn the_real_writer_passes_the_checker() {
        use oxide_bridge::reply::{BatteryReading, Reply};
        let mut r = Reply {
            timestamp_ns: 500_001,
            gyro: [1e-12, -3.5e7, 0.25],
            accel_body: [0.0, 0.0, -9.80665],
            position: [1e9, -1e-9, 0.0],
            velocity: [0.5, 0.0, -0.125],
            quaternion: [0.5, 0.5, 0.5, -0.5],
            battery: Some(BatteryReading {
                voltage: 16.8,
                current: 1e-7,
            }),
            rc: Some([1000, 1500, 2000, 1500, 1000, 1000, 1000, 1000]),
        };
        let mut s = String::new();
        r.write(&mut s).expect("finite");
        verdict(&s).expect("with everything");
        r.battery = None;
        r.rc = None;
        let mut s = String::new();
        r.write(&mut s).expect("finite");
        verdict(&s).expect("with neither");
    }
}
