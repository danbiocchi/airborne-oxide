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
//! - the wall clock is used for the time-out only.
//!
//! The socket is bound to `127.0.0.1`. The peer is learned from each datagram,
//! and the reply goes to the address and port of the last one.

use core::fmt;
use std::io;
use std::net::{SocketAddr, UdpSocket};
use std::time::{Duration, Instant};

use crate::packet::{PacketError, ServoPacket};

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
        if self.peer.is_none() {
            self.wait_for_first_packet(timeout)?;
        }
        let deadline = Instant::now() + timeout;
        self.send(reply)?;
        let last = self.last_count.unwrap_or(0);
        let expected = last.wrapping_add(1);
        loop {
            let Some(p) = self.receive(deadline)? else {
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
        let Some(p) = self.receive(Instant::now() + timeout)? else {
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
    fn receive(&mut self, deadline: Instant) -> Result<Option<ServoPacket>, LinkError> {
        let mut buf = [0u8; 2048];
        loop {
            let now = Instant::now();
            if now >= deadline {
                return Ok(None);
            }
            // A zero time-out would mean "wait for ever" to the socket.
            let left = (deadline - now).max(Duration::from_millis(1));
            self.socket.set_read_timeout(Some(left))?;
            match self.socket.recv_from(&mut buf) {
                Ok((n, from)) => {
                    self.peer = Some(from);
                    return Ok(Some(ServoPacket::parse(buf.get(..n).unwrap_or(&[]))?));
                }
                Err(e) => match e.kind() {
                    io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut => return Ok(None),
                    // Windows reports a send to a closed port on the next
                    // receive; the wait goes on.
                    io::ErrorKind::ConnectionReset | io::ErrorKind::Interrupted => {}
                    _ => return Err(LinkError::Io(e)),
                },
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const WAIT: Duration = Duration::from_secs(5);

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
        assert!(t0.elapsed() >= Duration::from_millis(90));
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
            took >= Duration::from_millis(140) && took < Duration::from_secs(3),
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
}
