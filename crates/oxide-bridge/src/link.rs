// SPDX-License-Identifier: AGPL-3.0-or-later
//! The UDP link and the lockstep rule (ADR-0006 §3).
//!
//! The simulator build sends one motor packet per frame and waits for our
//! reply before it sends the next. If it hears nothing for a second it sends
//! the same packet again with the same frame count. [`Link::exchange`] sends
//! the reply for the packet it holds and returns the next one:
//!
//! - a packet with the same frame count as the last is a resend: the stored
//!   reply is sent again and the wait goes on, so that the caller never
//!   advances its physics on it;
//! - a packet whose count is not the last plus one (it skipped, or went back)
//!   is [`LinkError::FrameCount`];
//! - the wall clock is used for the time-out only, and for asking a watcher
//!   every 100 ms whether the simulator is still there
//!   ([`Link::exchange_watched`]): a simulator that has ended is then noticed
//!   at once and not when the time-out runs out.
//!
//! The socket is bound to `127.0.0.1`. The peer is learned from each datagram
//! that parses as a motor packet, and the reply goes to the address and port of
//! the last one. A datagram that does not parse (a stranger writing to the
//! port, or rubbish) ends the exchange with an error and does not change the
//! peer.

use core::fmt;
use std::io;
use std::net::{SocketAddr, UdpSocket};
use std::time::{Duration, Instant};

use crate::packet::{PacketError, ServoPacket};

/// How often a waiting exchange asks its watcher whether the peer is alive.
const WATCH_EVERY: Duration = Duration::from_millis(100);

/// Called while an exchange waits: `false` says the peer has gone.
pub type Watch<'a, 'b> = &'a mut (dyn FnMut() -> bool + 'b);

/// The longest datagram taken in, bytes. A motor packet is at most 72.
const MAX_DATAGRAM: usize = 2048;

/// Why an exchange failed.
#[derive(Debug)]
pub enum LinkError {
    /// The socket failed.
    Io(io::Error),
    /// The simulator's first packet did not arrive in time.
    FirstPacketTimeout { waited: Duration },
    /// No packet arrived in time. `frame` is the count it waited for.
    Timeout { frame: u32, waited: Duration },
    /// The frame count skipped or went back.
    FrameCount { expected: u32, found: u32 },
    /// A datagram that is not a motor packet.
    Packet(PacketError),
    /// A datagram longer than the largest one taken in (the rest is lost).
    TooLong { limit: usize },
    /// The watcher said the simulator process is gone, `waited` after the
    /// reply went out.
    PeerGone { waited: Duration },
}

impl fmt::Display for LinkError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io(e) => write!(f, "the socket failed: {e}"),
            Self::FirstPacketTimeout { waited } => write!(
                f,
                "no first packet from the simulator within {} ms",
                waited.as_millis()
            ),
            Self::Timeout { frame, waited } => write!(
                f,
                "no packet for frame {frame} within {} ms",
                waited.as_millis()
            ),
            Self::FrameCount { expected, found } => write!(
                f,
                "frame count out of order: expected {expected}, found {found}"
            ),
            Self::Packet(e) => write!(f, "{e}"),
            Self::PeerGone { waited } => write!(
                f,
                "the simulator process is gone: nothing came for {} ms",
                waited.as_millis()
            ),
            Self::TooLong { limit } => {
                write!(
                    f,
                    "a datagram longer than {limit} bytes, not a motor packet"
                )
            }
        }
    }
}

impl std::error::Error for LinkError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Io(e) => Some(e),
            Self::Packet(e) => Some(e),
            _ => None,
        }
    }
}

impl From<io::Error> for LinkError {
    fn from(e: io::Error) -> Self {
        Self::Io(e)
    }
}

impl From<PacketError> for LinkError {
    fn from(e: PacketError) -> Self {
        Self::Packet(e)
    }
}

/// The socket, the peer and the frame-count state.
#[derive(Debug)]
pub struct Link {
    socket: UdpSocket,
    peer: Option<SocketAddr>,
    last_count: Option<u32>,
    last_reply: String,
}

impl Link {
    /// Binds `127.0.0.1:port`; port 0 lets the system choose
    /// ([`Link::local_port`] tells which).
    pub fn bind(port: u16) -> io::Result<Self> {
        let socket = UdpSocket::bind(("127.0.0.1", port))?;
        Ok(Self {
            socket,
            peer: None,
            last_count: None,
            last_reply: String::new(),
        })
    }

    pub fn local_port(&self) -> u16 {
        self.socket.local_addr().map_or(0, |a| a.port())
    }

    /// The simulator's first packet has arrived.
    pub fn has_peer(&self) -> bool {
        self.peer.is_some()
    }

    /// The frame count of the packet the last exchange returned (the first
    /// packet's, before any).
    pub fn last_frame_count(&self) -> Option<u32> {
        self.last_count
    }

    /// Sends `reply` for the packet held and returns the next packet. The
    /// first call waits for the simulator's first packet before it replies.
    pub fn exchange(&mut self, reply: &str, timeout: Duration) -> Result<ServoPacket, LinkError> {
        self.exchange_watched(reply, timeout, None)
    }

    /// [`Link::exchange`] that asks `watch` every 100 ms while it waits and
    /// ends with [`LinkError::PeerGone`] when it says `false`. A packet that
    /// has arrived is taken before the watcher is asked.
    pub fn exchange_watched(
        &mut self,
        reply: &str,
        timeout: Duration,
        mut watch: Option<Watch<'_, '_>>,
    ) -> Result<ServoPacket, LinkError> {
        if self.peer.is_none() {
            self.wait_for_first_packet_watched(timeout, watch.as_deref_mut())?;
        }
        let deadline = Instant::now() + timeout;
        self.send(reply)?;
        let last = self.last_count.unwrap_or(0);
        let expected = last.wrapping_add(1);
        loop {
            let Some(p) = self.receive(deadline, watch.as_deref_mut())? else {
                return Err(LinkError::Timeout {
                    frame: expected,
                    waited: timeout,
                });
            };
            if p.frame_count == last {
                // Our reply was lost or late: answer again, take no step.
                self.resend()?;
            } else if p.frame_count == expected {
                self.last_count = Some(expected);
                return Ok(p);
            } else {
                return Err(LinkError::FrameCount {
                    expected,
                    found: p.frame_count,
                });
            }
        }
    }

    /// Waits for the simulator's first packet (whatever its count) and
    /// learns the peer from it. A caller that wants a longer wait for the
    /// first packet than for the others calls this before the first
    /// [`Link::exchange`].
    pub fn wait_for_first_packet(&mut self, timeout: Duration) -> Result<(), LinkError> {
        self.wait_for_first_packet_watched(timeout, None)
    }

    /// [`Link::wait_for_first_packet`] with a watcher (see
    /// [`Link::exchange_watched`]).
    pub fn wait_for_first_packet_watched(
        &mut self,
        timeout: Duration,
        watch: Option<Watch<'_, '_>>,
    ) -> Result<(), LinkError> {
        let Some(p) = self.receive(Instant::now() + timeout, watch)? else {
            return Err(LinkError::FirstPacketTimeout { waited: timeout });
        };
        self.last_count = Some(p.frame_count);
        Ok(())
    }

    /// Sends a reply to the peer and keeps it for a resend.
    fn send(&mut self, reply: &str) -> Result<(), LinkError> {
        self.last_reply.clear();
        self.last_reply.push_str(reply);
        self.resend()
    }

    fn resend(&self) -> Result<(), LinkError> {
        if let Some(peer) = self.peer {
            self.socket.send_to(self.last_reply.as_bytes(), peer)?;
        }
        Ok(())
    }

    /// The next datagram parsed as a packet, or `None` at the deadline. The
    /// peer becomes the sender of that datagram.
    fn receive(
        &mut self,
        deadline: Instant,
        mut watch: Option<Watch<'_, '_>>,
    ) -> Result<Option<ServoPacket>, LinkError> {
        // One byte more than the limit, to tell a datagram that was cut off.
        let mut buf = [0u8; MAX_DATAGRAM + 1];
        let started = Instant::now();
        let mut next_watch = started + WATCH_EVERY;
        loop {
            let now = Instant::now();
            if now >= deadline {
                return Ok(None);
            }
            if let Some(w) = watch.as_mut()
                && now >= next_watch
            {
                next_watch = now + WATCH_EVERY;
                if !w() {
                    return Err(LinkError::PeerGone {
                        waited: now - started,
                    });
                }
            }
            // A zero time-out would mean "wait for ever" to the socket.
            let mut left = (deadline - now).max(Duration::from_millis(1));
            if watch.is_some() {
                left = left.min(WATCH_EVERY);
            }
            self.socket.set_read_timeout(Some(left))?;
            match self.socket.recv_from(&mut buf) {
                Ok((n, from)) => {
                    if n > MAX_DATAGRAM {
                        return Err(LinkError::TooLong {
                            limit: MAX_DATAGRAM,
                        });
                    }
                    let packet = ServoPacket::parse(buf.get(..n).unwrap_or(&[]))?;
                    self.peer = Some(from);
                    return Ok(Some(packet));
                }
                // Windows reports a datagram longer than the buffer as an
                // error, where other systems cut it off and return the
                // buffer's length (the check above).
                Err(e) if is_cut_off(&e) => {
                    return Err(LinkError::TooLong {
                        limit: MAX_DATAGRAM,
                    });
                }
                Err(e) => match e.kind() {
                    // The socket's time-out can run out a little before our
                    // deadline on some systems: the deadline check at the top
                    // of the loop decides whether the wait is over.
                    io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut => {}
                    // Windows reports a send to a closed port on the next
                    // receive; the wait goes on. (Not exercised on Linux, and
                    // not yet run on Windows.)
                    io::ErrorKind::ConnectionReset | io::ErrorKind::Interrupted => {}
                    _ => return Err(LinkError::Io(e)),
                },
            }
        }
    }
}

/// Windows' error code for "the datagram was larger than the buffer"
/// (`WSAEMSGSIZE`).
const WINDOWS_MESSAGE_TOO_LONG: i32 = 10040;

/// The receive failed because the datagram did not fit the buffer. Only
/// Windows reports this as an error.
fn is_cut_off(e: &io::Error) -> bool {
    cfg!(windows) && e.raw_os_error() == Some(WINDOWS_MESSAGE_TOO_LONG)
}

#[cfg(test)]
mod tests {
    use super::*;

    const WAIT: Duration = Duration::from_secs(5);

    #[test]
    fn only_windows_reports_a_cut_off_datagram_as_an_error() {
        let too_long = io::Error::from_raw_os_error(WINDOWS_MESSAGE_TOO_LONG);
        assert_eq!(is_cut_off(&too_long), cfg!(windows));
        assert!(!is_cut_off(&io::Error::from(io::ErrorKind::TimedOut)));
        assert!(!is_cut_off(&io::Error::from_raw_os_error(1)));
    }

    /// A motor packet of 16 channels with the given count.
    fn packet(count: u32) -> Vec<u8> {
        let mut b = Vec::new();
        b.extend_from_slice(&18458u16.to_le_bytes());
        b.extend_from_slice(&2000u16.to_le_bytes());
        b.extend_from_slice(&count.to_le_bytes());
        for i in 0..16u16 {
            b.extend_from_slice(&(1000 + i).to_le_bytes());
        }
        b
    }

    /// The simulator's side: a plain socket aimed at the link.
    struct Peer {
        socket: UdpSocket,
        to: SocketAddr,
    }

    impl Peer {
        fn new(link: &Link) -> Self {
            let socket = UdpSocket::bind("127.0.0.1:0").expect("bind");
            socket.set_read_timeout(Some(WAIT)).expect("timeout");
            Self {
                socket,
                to: SocketAddr::from(([127, 0, 0, 1], link.local_port())),
            }
        }
        fn send(&self, count: u32) {
            self.socket.send_to(&packet(count), self.to).expect("send");
        }
        fn recv(&self) -> String {
            let mut buf = [0u8; 2048];
            let n = self.socket.recv(&mut buf).expect("a reply");
            String::from_utf8(buf[..n].to_vec()).expect("text")
        }
        fn nothing_more(&self) {
            self.socket
                .set_read_timeout(Some(Duration::from_millis(60)))
                .expect("timeout");
            let mut buf = [0u8; 2048];
            assert!(self.socket.recv(&mut buf).is_err(), "nothing more was sent");
            self.socket.set_read_timeout(Some(WAIT)).expect("timeout");
        }
    }

    #[test]
    fn the_first_call_learns_the_peer_replies_to_it_and_returns_the_next_packet() {
        let mut link = Link::bind(0).expect("bind");
        assert_ne!(link.local_port(), 0);
        assert!(!link.has_peer());
        let peer = Peer::new(&link);
        peer.send(5);
        peer.send(6);
        let p = link.exchange("r5", WAIT).expect("exchange");
        assert_eq!(p.frame_count, 6);
        assert_eq!(p.pwm[3], 1003);
        assert!(link.has_peer());
        assert_eq!(link.last_frame_count(), Some(6));
        assert_eq!(peer.recv(), "r5", "the reply to the first packet");
        peer.nothing_more();
    }

    #[test]
    fn a_watcher_that_says_the_peer_is_gone_ends_the_wait_at_once() {
        let mut link = Link::bind(0).expect("bind");
        let peer = Peer::new(&link);
        peer.send(5);
        let mut calls = 0;
        let mut watch = || {
            calls += 1;
            calls < 3
        };
        let started = Instant::now();
        let e = link
            .exchange_watched("r", Duration::from_secs(20), Some(&mut watch))
            .expect_err("the peer is gone");
        let waited = started.elapsed();
        assert!(matches!(e, LinkError::PeerGone { .. }), "{e}");
        assert!(e.to_string().contains("gone"), "{e}");
        assert!(
            waited >= Duration::from_millis(200) && waited < Duration::from_secs(2),
            "{waited:?}"
        );
    }

    #[test]
    fn a_packet_that_has_come_is_taken_before_the_watcher_is_asked() {
        let mut link = Link::bind(0).expect("bind");
        let peer = Peer::new(&link);
        peer.send(5);
        peer.send(6);
        let mut watch = || false;
        let p = link
            .exchange_watched("r", WAIT, Some(&mut watch))
            .expect("the packet was there");
        assert_eq!(p.frame_count, 6);
    }

    #[test]
    fn a_watcher_that_says_alive_changes_nothing_about_the_time_out() {
        let mut link = Link::bind(0).expect("bind");
        let peer = Peer::new(&link);
        peer.send(5);
        let mut watch = || true;
        let e = link
            .exchange_watched("r", Duration::from_millis(350), Some(&mut watch))
            .expect_err("nothing comes");
        assert!(matches!(e, LinkError::Timeout { .. }), "{e}");
    }

    #[test]
    fn a_resent_packet_gets_the_stored_reply_again_and_no_step() {
        let mut link = Link::bind(0).expect("bind");
        let peer = Peer::new(&link);
        peer.send(5);
        peer.send(5);
        peer.send(5);
        peer.send(6);
        let p = link.exchange("r5", WAIT).expect("exchange");
        assert_eq!(p.frame_count, 6, "the repeats were not returned");
        for _ in 0..3 {
            assert_eq!(peer.recv(), "r5", "the same reply again");
        }
        peer.nothing_more();
        // The next frame: its own reply, and a late repeat of 6 gets that one.
        peer.send(6);
        peer.send(7);
        let p = link.exchange("r6", WAIT).expect("exchange");
        assert_eq!(p.frame_count, 7);
        assert_eq!(peer.recv(), "r6");
        assert_eq!(peer.recv(), "r6", "a repeat of 6 gets the reply to 6");
        peer.nothing_more();
    }

    #[test]
    fn a_skip_or_a_step_back_is_a_frame_count_error_naming_both_numbers() {
        let mut link = Link::bind(0).expect("bind");
        let peer = Peer::new(&link);
        peer.send(5);
        peer.send(7);
        match link.exchange("r5", WAIT) {
            Err(LinkError::FrameCount { expected, found }) => {
                assert_eq!((expected, found), (6, 7));
            }
            other => panic!("{other:?}"),
        }
        let e = LinkError::FrameCount {
            expected: 6,
            found: 7,
        };
        assert!(
            e.to_string().contains('6') && e.to_string().contains('7'),
            "{e}"
        );

        let mut link = Link::bind(0).expect("bind");
        let peer = Peer::new(&link);
        peer.send(5);
        peer.send(6);
        link.exchange("r5", WAIT).expect("ok");
        peer.send(4);
        match link.exchange("r6", WAIT) {
            Err(LinkError::FrameCount { expected, found }) => {
                assert_eq!((expected, found), (7, 4));
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn the_count_wraps_at_the_top_of_u32() {
        let mut link = Link::bind(0).expect("bind");
        let peer = Peer::new(&link);
        peer.send(u32::MAX);
        peer.send(0);
        let p = link.exchange("r", WAIT).expect("wraps");
        assert_eq!(p.frame_count, 0);
    }

    #[test]
    fn nothing_in_time_is_a_timeout_that_names_the_frame() {
        let mut link = Link::bind(0).expect("bind");
        let t0 = Instant::now();
        let e = link
            .exchange("r", Duration::from_millis(100))
            .expect_err("no first packet");
        assert!(matches!(e, LinkError::FirstPacketTimeout { .. }), "{e}");
        // A socket time-out may expire a little early on some systems (the
        // loop then waits on); the lower bound leaves room for that.
        assert!(t0.elapsed() >= Duration::from_millis(50));
        assert!(!link.has_peer());

        let peer = Peer::new(&link);
        peer.send(41);
        let t0 = Instant::now();
        let e = link
            .exchange("r41", Duration::from_millis(150))
            .expect_err("no next packet");
        match e {
            LinkError::Timeout { frame, .. } => assert_eq!(frame, 42),
            other => panic!("{other:?}"),
        }
        let took = t0.elapsed();
        assert!(
            took >= Duration::from_millis(100) && took < Duration::from_secs(3),
            "{took:?}"
        );
        assert_eq!(peer.recv(), "r41", "the reply was sent before the wait");
    }

    #[test]
    fn repeats_that_keep_coming_do_not_extend_the_wait() {
        // Repeats arrive every 20 ms for a second, the next frame never does:
        // the time-out still ends the wait, counted from the start.
        let mut link = Link::bind(0).expect("bind");
        let peer = Peer::new(&link);
        peer.send(9);
        let to = peer.to;
        let sender = std::thread::spawn(move || {
            let s = UdpSocket::bind("127.0.0.1:0").expect("bind");
            for _ in 0..50 {
                let _ = s.send_to(&packet(9), to);
                std::thread::sleep(Duration::from_millis(20));
            }
        });
        let t0 = Instant::now();
        let e = link
            .exchange("r", Duration::from_millis(200))
            .expect_err("times out");
        assert!(matches!(e, LinkError::Timeout { frame: 10, .. }), "{e}");
        assert!(
            t0.elapsed() < Duration::from_millis(800),
            "{:?}",
            t0.elapsed()
        );
        sender.join().expect("thread");
    }

    #[test]
    fn a_datagram_that_is_not_a_packet_is_a_packet_error() {
        let mut link = Link::bind(0).expect("bind");
        let peer = Peer::new(&link);
        peer.send(1);
        peer.socket.send_to(&[1, 2, 3], peer.to).expect("send");
        let e = link.exchange("r", WAIT).expect_err("garbage");
        assert!(
            matches!(
                e,
                LinkError::Packet(PacketError::TooShort { .. })
                    | LinkError::Packet(PacketError::UnknownMagic { .. })
            ),
            "{e}"
        );
    }

    #[test]
    fn the_reply_goes_to_the_address_the_last_datagram_came_from() {
        let mut link = Link::bind(0).expect("bind");
        let first = Peer::new(&link);
        first.send(1);
        first.send(2);
        link.exchange("a", WAIT).expect("ok");
        assert_eq!(first.recv(), "a");
        // The simulator now speaks from another port: the reply to the
        // packet it sent from there goes there.
        let second = Peer::new(&link);
        second.send(3);
        link.exchange("b", WAIT).expect("ok");
        assert_eq!(first.recv(), "b", "sent before the new address was known");
        second.send(4);
        link.exchange("c", WAIT).expect("ok");
        assert_eq!(second.recv(), "c");
        first.nothing_more();
    }

    #[test]
    fn a_datagram_that_is_not_a_packet_does_not_change_the_peer() {
        let mut link = Link::bind(0).expect("bind");
        let a = Peer::new(&link);
        a.send(1);
        a.send(2);
        link.exchange("r1", WAIT).expect("ok");
        let a_addr = a.socket.local_addr().expect("addr");
        assert_eq!(link.peer, Some(a_addr));
        // A stranger writes rubbish to the port: the exchange fails, and the
        // stranger is not where the next reply goes.
        let b = Peer::new(&link);
        b.socket.send_to(&[1, 2, 3], b.to).expect("send");
        let e = link.exchange("r2", WAIT).expect_err("rubbish");
        assert!(matches!(e, LinkError::Packet(_)), "{e}");
        assert_eq!(link.peer, Some(a_addr), "the peer is still the simulator");
        b.socket.send_to(&packet(3), b.to).expect("send");
        let e = link.exchange("r2", WAIT).expect("a real packet");
        assert_eq!(e.frame_count, 3);
        assert_eq!(link.peer, b.socket.local_addr().ok(), "a packet moves it");
    }

    #[test]
    fn a_datagram_longer_than_the_buffer_says_it_was_too_long() {
        let mut link = Link::bind(0).expect("bind");
        let peer = Peer::new(&link);
        peer.send(1);
        peer.socket
            .send_to(&vec![0u8; 3000], peer.to)
            .expect("send");
        let e = link.exchange("r", WAIT).expect_err("too long");
        assert!(matches!(e, LinkError::TooLong { .. }), "{e:?}");
        assert!(e.to_string().contains("longer than"), "{e}");
        // A datagram of exactly the largest size is still just a wrong length.
        let mut link = Link::bind(0).expect("bind");
        let peer = Peer::new(&link);
        peer.send(1);
        peer.socket
            .send_to(&vec![0u8; MAX_DATAGRAM], peer.to)
            .expect("send");
        let e = link.exchange("r", WAIT).expect_err("unknown magic");
        assert!(matches!(e, LinkError::Packet(_)), "{e:?}");
    }
}
