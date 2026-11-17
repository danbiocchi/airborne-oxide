// SPDX-License-Identifier: AGPL-3.0-or-later
//! The autopilot that flies the vehicle through the JSON exchange
//! (ADR-0006 §3 to §5).
//!
//! [`JsonBridge`] implements `oxide_sim::Autopilot<4>`. Per tick it writes the
//! vehicle's state as one reply line, sends it, waits for the simulator
//! build's next motor packet, and returns that packet's pulses as motor
//! commands valid at once (`t_valid = t`: the packet that answers the state at
//! `t` drives the physics from `t`).
//!
//! The reply is filled from the physics truth: position, velocity and
//! attitude as they are, the body rate as `gyro`, and the specific force
//! rotated into the body frame as `accel_body`, which is the ideal IMU sample
//! of `oxide_sim::sensors::imu` (our IMU model's samples, the GPS error, the
//! pre-roll, arming and the modes come later, in the part that runs the real
//! program).
//!
//! The time stamp of the reply of exchange `n` (from 0) is one step times
//! `n + 1`; see [`Exchange`], which both autopilots of this crate use.
//!
//! `armed` is "any of the four pulses is above 1000". The pulses are lockstep
//! data and repeat bit for bit; the autopilot's own armed flag arrives over a
//! channel whose timing depends on the wall clock and is audit data only.

use core::fmt;
use std::time::Duration;

use oxide_core::io::{RcSample, Stamped};
use oxide_core::math::Vec3;
use oxide_core::time::Micros;
use oxide_sim::autopilot::{Autopilot, AutopilotInput, AutopilotOutput};
use oxide_sim::hash::TraceHash;
use oxide_sim::scenario::COMMAND_PERIOD_US;
use oxide_sim::vehicle::BodyState;

use crate::link::{Link, LinkError};
use crate::motors::{DEFAULT_SPIN_MAX, DEFAULT_SPIN_MIN, MotorError, MotorMap};
use crate::packet::ServoPacket;
use crate::reply::{BatteryReading, Reply, ReplyError, timestamp_ns};

/// How the bridge is set up.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BridgeConfig {
    /// The simulation step and tick period, µs.
    pub step_us: u64,
    /// The autopilot's `MOT_SPIN_MIN`, 0 to 1.
    pub spin_min: f32,
    /// The autopilot's `MOT_SPIN_MAX`, 0 to 1.
    pub spin_max: f32,
    /// How long to wait for each next packet.
    pub reply_timeout: Duration,
    /// How long to wait for the first packet (the program's start-up).
    pub first_packet_timeout: Duration,
    /// Send the `battery` section.
    pub send_battery: bool,
    /// Send the `rc` section when the radio is there.
    pub send_rc: bool,
}

impl BridgeConfig {
    /// 500 µs is the step of a bridged flight; this takes it as an argument
    /// so that a test can use another.
    pub fn new(step_us: u64) -> Self {
        Self {
            step_us,
            spin_min: DEFAULT_SPIN_MIN,
            spin_max: DEFAULT_SPIN_MAX,
            reply_timeout: Duration::from_secs(10),
            first_packet_timeout: Duration::from_secs(60),
            send_battery: true,
            send_rc: true,
        }
    }
}

/// The configuration is not usable.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ConfigError {
    /// The step is zero.
    Step,
    /// The spin range is not usable.
    Motors(MotorError),
    /// A time-out is zero.
    Timeout,
}

impl fmt::Display for ConfigError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Step => write!(f, "the step must not be zero"),
            Self::Motors(e) => write!(f, "{e}"),
            Self::Timeout => write!(f, "the time-outs must not be zero"),
        }
    }
}

impl std::error::Error for ConfigError {}

/// Why a tick failed. Each says at which step.
#[derive(Debug)]
pub enum BridgeError {
    /// The exchange failed: a time-out, a frame count out of order, a bad
    /// datagram, the socket. `step` counts the exchanges done before it.
    Link { step: u64, source: LinkError },
    /// The state to send is not a finite number; nothing was sent.
    Reply { step: u64, source: ReplyError },
}

impl fmt::Display for BridgeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Link { step, source } => write!(f, "at step {step}: {source}"),
            Self::Reply { step, source } => write!(f, "at step {step}: {source}"),
        }
    }
}

impl std::error::Error for BridgeError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Link { source, .. } => Some(source),
            Self::Reply { source, .. } => Some(source),
        }
    }
}

/// Our radio sample as the eight channels of the reply, in microseconds:
/// roll, pitch, throttle, yaw (sticks −1 to 1 to 1000 to 2000 with 1500 at
/// the centre, throttle 0 to 1 to 1000 to 2000), then channel 5 the mode
/// switch (positions 0, 1, 2 to 1000, 1500, 2000), 6 the arm switch, 7 the
/// kill switch (off, on to 1000, 2000), and 8 unused (1000). `None` when a
/// stick is not a number.
pub fn rc_pulses(rc: &RcSample) -> Option<[u16; 8]> {
    let stick = |x: f32| -> Option<u16> {
        x.is_finite()
            .then(|| (1500.0 + 500.0 * f64::from(x.clamp(-1.0, 1.0)) + 0.5) as u16)
    };
    let throttle = |x: f32| -> Option<u16> {
        x.is_finite()
            .then(|| (1000.0 + 1000.0 * f64::from(x.clamp(0.0, 1.0)) + 0.5) as u16)
    };
    let switch = |on: bool| if on { 2000 } else { 1000 };
    Some([
        stick(rc.roll)?,
        stick(rc.pitch)?,
        throttle(rc.throttle)?,
        stick(rc.yaw)?,
        match rc.mode_switch {
            0 => 1000,
            1 => 1500,
            _ => 2000,
        },
        switch(rc.arm_switch),
        switch(rc.kill_switch),
        1000,
    ])
}

/// What a reply holds besides the time stamp and the radio, from the body
/// the simulator shows: the truth as it is, the specific force rotated into
/// the body frame (the ideal IMU sample of `oxide_sim::sensors::imu`), the
/// battery. The one place the reply's fields are filled; a caller that has
/// its own IMU sample or a position error overwrites those fields.
pub fn reply_of_body<const N: usize>(
    timestamp_ns: u64,
    body: &BodyState<N>,
    specific_force_world: Vec3<f64>,
    battery: (f64, f64),
    send_battery: bool,
    rc: Option<[u16; 8]>,
) -> Reply {
    let accel = body.attitude.inverse_rotate(specific_force_world);
    Reply {
        timestamp_ns,
        gyro: [body.rate.x, body.rate.y, body.rate.z],
        accel_body: [accel.x, accel.y, accel.z],
        position: [body.position.x, body.position.y, body.position.z],
        velocity: [body.velocity.x, body.velocity.y, body.velocity.z],
        quaternion: [
            body.attitude.w,
            body.attitude.x,
            body.attitude.y,
            body.attitude.z,
        ],
        battery: send_battery.then_some(BatteryReading {
            voltage: battery.0,
            current: battery.1,
        }),
        rc,
    }
}

/// Fills the reply from one tick's input: the truth, as the ideal IMU sample
/// forms it (see [`reply_of_body`]).
pub fn reply_from_input<const N: usize>(
    timestamp_ns: u64,
    input: &AutopilotInput<'_, N>,
    battery: bool,
    rc: Option<[u16; 8]>,
) -> Reply {
    reply_of_body(
        timestamp_ns,
        input.body,
        input.specific_force_world,
        (input.battery_v, input.battery_a),
        battery,
        rc,
    )
}

/// The radio as the reply sends it: the last valid frame while it is no
/// older than the radio's period. A frame with failsafe or a lost frame ends
/// it; so does a silent radio.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RcLatch {
    /// The last valid radio frame and the tick it arrived at.
    last: Option<(Micros, [u16; 8])>,
}

impl RcLatch {
    pub fn new() -> Self {
        Self::default()
    }

    /// The channels to send at `t`, given the frame that arrived at this tick
    /// (if any).
    pub fn at(&mut self, t: Micros, frame: &Option<Stamped<RcSample>>) -> Option<[u16; 8]> {
        if let Some(frame) = frame {
            self.last = if frame.value.is_valid() {
                rc_pulses(&frame.value).map(|p| (t, p))
            } else {
                None
            };
        }
        let (at, pulses) = self.last?;
        (t.since(at) <= COMMAND_PERIOD_US).then_some(pulses)
    }
}

/// The exchange with the simulator build, once: the socket, the motor map,
/// the reply's time stamp and the hashes that prove a flight repeats. Both
/// autopilots of this crate send their replies through it, so that there is
/// one implementation of the stamps, the line and the wait.
///
/// The time stamp of exchange `n` (counting every reply sent, from 0) is
/// `timestamp_ns(step, n + 1)`: the first reply carries one step, not zero.
/// A first stamp of zero is accepted by the program, but the run then does
/// not repeat (measured on ArduCopter V4.7.2).
#[derive(Debug)]
pub struct Exchange {
    link: Link,
    config: BridgeConfig,
    motors: MotorMap,
    line: String,
    frames: u64,
    reply_hash: TraceHash,
    packet_hash: TraceHash,
}

impl Exchange {
    pub fn new(link: Link, config: BridgeConfig) -> Result<Self, ConfigError> {
        if config.step_us == 0 {
            return Err(ConfigError::Step);
        }
        if config.reply_timeout.is_zero() || config.first_packet_timeout.is_zero() {
            return Err(ConfigError::Timeout);
        }
        let motors =
            MotorMap::new(config.spin_min, config.spin_max).map_err(ConfigError::Motors)?;
        Ok(Self {
            link,
            config,
            motors,
            line: String::with_capacity(512),
            frames: 0,
            reply_hash: TraceHash::default(),
            packet_hash: TraceHash::default(),
        })
    }

    pub fn config(&self) -> &BridgeConfig {
        &self.config
    }
    pub fn link(&self) -> &Link {
        &self.link
    }
    /// Replies sent so far (one per successful exchange): the number of the
    /// next exchange.
    pub fn frames(&self) -> u64 {
        self.frames
    }
    /// The time stamp of the next reply, ns.
    pub fn next_timestamp_ns(&self) -> u64 {
        timestamp_ns(self.config.step_us, self.frames.saturating_add(1))
    }
    /// FNV-1a over the bytes of every reply line sent.
    pub fn reply_hash(&self) -> u64 {
        self.reply_hash.value()
    }
    /// FNV-1a over every motor packet received (frame count and pulses).
    pub fn packet_hash(&self) -> u64 {
        self.packet_hash.value()
    }

    /// Stamps `reply` with the next time stamp, sends it and returns the
    /// simulator build's next motor packet. The battery and radio sections
    /// are dropped when the configuration says so. If the reply cannot be
    /// written, or the exchange fails, nothing is counted.
    pub fn exchange(&mut self, mut reply: Reply) -> Result<ServoPacket, BridgeError> {
        let step = self.frames;
        reply.timestamp_ns = self.next_timestamp_ns();
        if !self.config.send_battery {
            reply.battery = None;
        }
        if !self.config.send_rc {
            reply.rc = None;
        }
        self.line.clear();
        reply
            .write(&mut self.line)
            .map_err(|source| BridgeError::Reply { step, source })?;
        // The first packet of the program may take a while to come.
        if !self.link.has_peer() {
            self.link
                .wait_for_first_packet(self.config.first_packet_timeout)
                .map_err(|source| BridgeError::Link { step, source })?;
        }
        let packet = self
            .link
            .exchange(&self.line, self.config.reply_timeout)
            .map_err(|source| BridgeError::Link { step, source })?;
        self.frames += 1;
        self.reply_hash.write_bytes(self.line.as_bytes());
        self.packet_hash.write_u64(u64::from(packet.frame_count));
        for p in packet.pwm.iter().take(packet.channels) {
            self.packet_hash.write_bytes(&p.to_le_bytes());
        }
        Ok(packet)
    }

    /// The first four pulses of a packet.
    pub fn pulses(packet: &ServoPacket) -> [u16; 4] {
        let mut out = [0; 4];
        for (o, &p) in out.iter_mut().zip(packet.pwm.iter()) {
            *o = p;
        }
        out
    }

    /// The motor commands for rotors 0 to 3 from a packet.
    pub fn rotors(&self, packet: &ServoPacket) -> [f32; 4] {
        self.motors.rotors(&Self::pulses(packet))
    }
}

/// `armed` is "any of the four pulses is above 1000".
pub fn armed_by_pulses(pulses: &[u16; 4]) -> bool {
    pulses.iter().any(|&p| p > 1000)
}

/// The JSON bridge as the simulator's autopilot.
#[derive(Debug)]
pub struct JsonBridge {
    ex: Exchange,
    rc: RcLatch,
}

impl JsonBridge {
    pub fn new(link: Link, config: BridgeConfig) -> Result<Self, ConfigError> {
        Ok(Self {
            ex: Exchange::new(link, config)?,
            rc: RcLatch::new(),
        })
    }

    pub fn config(&self) -> &BridgeConfig {
        self.ex.config()
    }
    pub fn link(&self) -> &Link {
        self.ex.link()
    }
    /// Replies sent so far (one per successful tick).
    pub fn frames(&self) -> u64 {
        self.ex.frames()
    }
    pub fn exchange(&self) -> &Exchange {
        &self.ex
    }

    /// The radio channels to send at `t` (see [`RcLatch`]).
    fn rc_at(&mut self, t: Micros, input: &AutopilotInput<'_, 4>) -> Option<[u16; 8]> {
        self.rc.at(t, &input.rc)
    }
}

impl Autopilot<4> for JsonBridge {
    type Error = BridgeError;

    fn period_us(&self) -> u64 {
        self.ex.config().step_us
    }

    fn tick(
        &mut self,
        t: Micros,
        input: &AutopilotInput<'_, 4>,
    ) -> Result<AutopilotOutput<4>, BridgeError> {
        let rc = self.rc_at(t, input);
        let reply = reply_from_input(0, input, true, rc);
        let packet = self.ex.exchange(reply)?;
        Ok(AutopilotOutput {
            t_valid: t,
            motors: self.ex.rotors(&packet),
            armed: armed_by_pulses(&Exchange::pulses(&packet)),
            core: None,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxide_core::io::Stamped;
    use oxide_core::math::{Quat, Vec3};
    use oxide_sim::faults::ActiveFaults;
    use oxide_sim::sensors::{self, SensorFrame};
    use oxide_sim::vehicle::BodyState;

    fn sticks(roll: f32, pitch: f32, throttle: f32, yaw: f32) -> RcSample {
        RcSample {
            roll,
            pitch,
            yaw,
            throttle,
            ..RcSample::default()
        }
    }

    #[test]
    fn sticks_map_to_1000_1500_2000_and_the_switches_to_their_channels() {
        let p = rc_pulses(&sticks(0.0, 0.0, 0.0, 0.0)).expect("pulses");
        assert_eq!(p, [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000]);
        let p = rc_pulses(&sticks(-1.0, 1.0, 1.0, -1.0)).expect("pulses");
        assert_eq!(&p[..4], &[1000, 2000, 2000, 1000]);
        let p = rc_pulses(&sticks(0.5, -0.25, 0.6, 0.25)).expect("pulses");
        assert_eq!(&p[..4], &[1750, 1375, 1600, 1625]);
        // Out of range sticks are held at the ends.
        let p = rc_pulses(&sticks(9.0, -9.0, 9.0, 0.0)).expect("pulses");
        assert_eq!(&p[..4], &[2000, 1000, 2000, 1500]);
        let p = rc_pulses(&sticks(0.0, 0.0, -1.0, 0.0)).expect("pulses");
        assert_eq!(p[2], 1000);
        // Switches: mode 0, 1, 2; arm; kill.
        for (mode, want) in [(0u8, 1000u16), (1, 1500), (2, 2000)] {
            let rc = RcSample {
                mode_switch: mode,
                ..RcSample::default()
            };
            assert_eq!(rc_pulses(&rc).expect("pulses")[4], want, "mode {mode}");
        }
        let rc = RcSample {
            arm_switch: true,
            kill_switch: true,
            ..RcSample::default()
        };
        let p = rc_pulses(&rc).expect("pulses");
        assert_eq!((p[5], p[6], p[7]), (2000, 2000, 1000));
        // A stick that is not a number sends no radio.
        assert_eq!(rc_pulses(&sticks(f32::NAN, 0.0, 0.0, 0.0)), None);
        assert_eq!(rc_pulses(&sticks(0.0, 0.0, f32::INFINITY, 0.0)), None);
    }

    fn input<'a>(
        body: &'a BodyState<4>,
        force: Vec3<f64>,
        frame: &'a SensorFrame,
        rc: Option<Stamped<RcSample>>,
    ) -> AutopilotInput<'a, 4> {
        AutopilotInput {
            body,
            specific_force_world: force,
            battery_v: 24.5,
            battery_a: 3.25,
            wind: Vec3::zero(),
            sensors: frame,
            rc,
            setpoint: None,
            gcs_heartbeat: None,
            brain_heartbeat: None,
            param_set: None,
            param_ack: false,
            faults: ActiveFaults::default(),
            rotor_scale: [1.0; 4],
        }
    }

    #[test]
    fn the_reply_carries_the_truth_and_the_ideal_imu_sample() {
        let frame = SensorFrame::default();
        let body = BodyState::<4> {
            position: Vec3::new(1.0, 2.0, -3.0),
            velocity: Vec3::new(0.5, -0.5, 0.25),
            rate: Vec3::new(0.1, 0.2, 0.3),
            attitude: Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), 0.7),
            ..BodyState::default()
        };
        let force = Vec3::new(0.3, -0.2, -9.80665);
        let r = reply_from_input(77, &input(&body, force, &frame, None), true, None);
        assert_eq!(r.timestamp_ns, 77);
        assert_eq!(r.position, [1.0, 2.0, -3.0]);
        assert_eq!(r.velocity, [0.5, -0.5, 0.25]);
        assert_eq!(r.gyro, [0.1, 0.2, 0.3]);
        let q = body.attitude;
        assert_eq!(r.quaternion, [q.w, q.x, q.y, q.z]);
        // The same definition as the ideal IMU sample, not a copy of it.
        let imu = sensors::imu(Micros(0), &body, force).value;
        for (got, want) in r
            .accel_body
            .iter()
            .zip([imu.accel.x, imu.accel.y, imu.accel.z])
        {
            assert!((got - f64::from(want)).abs() < 1e-5, "{got} {want}");
        }
        let b = r.battery.expect("battery");
        assert_eq!((b.voltage, b.current), (24.5, 3.25));
        assert!(r.rc.is_none());
        // At rest the specific force reads minus gravity on the body z axis.
        let rest = BodyState::<4>::default();
        let r = reply_from_input(
            0,
            &input(&rest, Vec3::new(0.0, 0.0, -9.80665), &frame, None),
            false,
            None,
        );
        assert_eq!(r.accel_body, [0.0, 0.0, -9.80665]);
        assert!(r.battery.is_none(), "the battery section is optional");
    }

    fn none<'a>(body: &'a BodyState<4>, frame: &'a SensorFrame) -> AutopilotInput<'a, 4> {
        input(body, Vec3::zero(), frame, None)
    }

    fn bridge(config: BridgeConfig) -> JsonBridge {
        JsonBridge::new(Link::bind(0).expect("bind"), config).expect("config")
    }

    fn at(t: u64, rc: RcSample) -> Option<Stamped<RcSample>> {
        Some(Stamped::ideal(Micros(t), rc))
    }

    #[test]
    fn the_radio_stays_for_one_period_and_failsafe_or_a_lost_frame_ends_it() {
        let mut b = bridge(BridgeConfig::new(500));
        let frame = SensorFrame::default();
        let body = BodyState::<4>::default();
        let good = sticks(0.0, 0.0, 0.5, 0.0);
        // Nothing yet.
        assert_eq!(b.rc_at(Micros(0), &none(&body, &frame)), None);
        // A frame arrives; it holds for the radio's period, and no longer.
        let i = input(&body, Vec3::zero(), &frame, at(1_000_000, good));
        let want = rc_pulses(&good);
        assert_eq!(b.rc_at(Micros(1_000_000), &i), want);
        assert_eq!(b.rc_at(Micros(1_010_000), &none(&body, &frame)), want);
        assert_eq!(b.rc_at(Micros(1_020_000), &none(&body, &frame)), want);
        assert_eq!(b.rc_at(Micros(1_020_500), &none(&body, &frame)), None);
        // A new frame renews it; a failsafe frame ends it at once.
        let i = input(&body, Vec3::zero(), &frame, at(1_040_000, good));
        assert_eq!(b.rc_at(Micros(1_040_000), &i), want);
        let failsafe = RcSample {
            failsafe: true,
            ..good
        };
        let i = input(&body, Vec3::zero(), &frame, at(1_060_000, failsafe));
        assert_eq!(b.rc_at(Micros(1_060_000), &i), None);
        assert_eq!(b.rc_at(Micros(1_060_500), &none(&body, &frame)), None);
        let lost = RcSample {
            frame_lost: true,
            ..good
        };
        let i = input(&body, Vec3::zero(), &frame, at(1_080_000, good));
        assert_eq!(b.rc_at(Micros(1_080_000), &i), want);
        let i = input(&body, Vec3::zero(), &frame, at(1_100_000, lost));
        assert_eq!(b.rc_at(Micros(1_100_000), &i), None);
    }

    #[test]
    fn a_configuration_that_cannot_work_is_refused() {
        let new = |c: BridgeConfig| JsonBridge::new(Link::bind(0).expect("bind"), c);
        assert!(new(BridgeConfig::new(500)).is_ok());
        assert_eq!(new(BridgeConfig::new(0)).err(), Some(ConfigError::Step));
        let mut c = BridgeConfig::new(500);
        c.spin_min = 0.9;
        c.spin_max = 0.9;
        assert!(matches!(new(c).err(), Some(ConfigError::Motors(_))));
        let mut c = BridgeConfig::new(500);
        c.reply_timeout = Duration::ZERO;
        assert_eq!(new(c).err(), Some(ConfigError::Timeout));
        let mut c = BridgeConfig::new(500);
        c.first_packet_timeout = Duration::ZERO;
        assert_eq!(new(c).err(), Some(ConfigError::Timeout));
        assert_eq!(bridge(BridgeConfig::new(500)).period_us(), 500);
    }

    #[test]
    fn the_errors_say_which_step_and_what() {
        let e = BridgeError::Link {
            step: 12,
            source: LinkError::FrameCount {
                expected: 13,
                found: 15,
            },
        };
        let s = e.to_string();
        assert!(
            s.contains("step 12") && s.contains("13") && s.contains("15"),
            "{s}"
        );
        let e = BridgeError::Reply {
            step: 3,
            source: ReplyError::NonFinite { field: "gyro" },
        };
        let s = e.to_string();
        assert!(s.contains("step 3") && s.contains("gyro"), "{s}");
        assert!(std::error::Error::source(&e).is_some());
    }

    /// A motor packet of 16 channels with these first four pulses.
    fn packet(count: u32, pwm: [u16; 4]) -> Vec<u8> {
        let mut b = Vec::new();
        b.extend_from_slice(&18458u16.to_le_bytes());
        b.extend_from_slice(&2000u16.to_le_bytes());
        b.extend_from_slice(&count.to_le_bytes());
        for k in 0..16 {
            b.extend_from_slice(&pwm.get(k).copied().unwrap_or(0).to_le_bytes());
        }
        b
    }

    #[test]
    fn the_commands_act_at_the_tick_that_asked_and_armed_means_a_pulse_above_1000() {
        let mut config = BridgeConfig::new(500);
        config.reply_timeout = Duration::from_secs(5);
        config.spin_min = 0.0;
        config.spin_max = 1.0;
        let mut b = bridge(config);
        let to = std::net::SocketAddr::from(([127, 0, 0, 1], b.link().local_port()));
        let peer = std::net::UdpSocket::bind("127.0.0.1:0").expect("bind");
        // The first packet, then the answers to the replies of three ticks.
        for (count, pwm) in [
            (1, [1000; 4]),
            (2, [1000; 4]),
            (3, [1000, 1000, 1000, 1001]),
            (4, [1250, 1500, 1750, 2000]),
        ] {
            peer.send_to(&packet(count, pwm), to).expect("send");
        }
        let frame = SensorFrame::default();
        let body = BodyState::<4>::default();
        let i = none(&body, &frame);
        let o = b.tick(Micros(1_500), &i).expect("tick");
        assert_eq!(o.t_valid, Micros(1_500), "valid at the tick's own time");
        assert_eq!(o.motors, [0.0; 4]);
        assert!(!o.armed, "all four at 1000");
        let o = b.tick(Micros(2_000), &i).expect("tick");
        assert_eq!(o.t_valid, Micros(2_000));
        assert!(o.armed, "one pulse at 1001 is armed");
        let o = b.tick(Micros(2_500), &i).expect("tick");
        assert_eq!(o.t_valid, Micros(2_500));
        assert_eq!(o.motors, [0.25, 0.5, 0.75, 1.0], "output k drives rotor k");
        assert!(o.core.is_none());
        assert_eq!(b.frames(), 3);
    }

    fn stamp_of(datagram: &[u8]) -> String {
        let text = String::from_utf8_lossy(datagram).into_owned();
        let key = "\"timestamp\":";
        let start = text.find(key).expect("a time stamp") + key.len();
        let end = text[start..].find(',').expect("a comma") + start;
        text[start..end].to_string()
    }

    #[test]
    fn the_first_reply_carries_one_step_and_exchange_n_carries_n_plus_one_steps() {
        let mut config = BridgeConfig::new(500);
        config.reply_timeout = Duration::from_secs(5);
        let mut ex = Exchange::new(Link::bind(0).expect("bind"), config).expect("config");
        let to = std::net::SocketAddr::from(([127, 0, 0, 1], ex.link().local_port()));
        let peer = std::net::UdpSocket::bind("127.0.0.1:0").expect("bind");
        peer.set_read_timeout(Some(Duration::from_secs(5)))
            .expect("timeout");
        for count in 1..=4 {
            peer.send_to(&packet(count, [1000; 4]), to).expect("send");
        }
        assert_eq!(
            ex.next_timestamp_ns(),
            500_001,
            "the next stamp is one step"
        );
        let frame = SensorFrame::default();
        let body = BodyState::<4>::default();
        let i = none(&body, &frame);
        for n in 0..3u64 {
            assert_eq!(ex.frames(), n);
            ex.exchange(reply_from_input(0, &i, true, None))
                .expect("exchange");
        }
        let mut buf = [0u8; 2048];
        for (n, want) in ["0.000500001", "0.001000002", "0.001500003"]
            .iter()
            .enumerate()
        {
            let len = peer.recv(&mut buf).expect("a reply");
            assert_eq!(&stamp_of(&buf[..len]), want, "exchange {n}");
        }
        assert_eq!(ex.frames(), 3);
        // The hashes saw three replies and three packets.
        assert_ne!(ex.reply_hash(), TraceHash::default().value());
        assert_ne!(ex.packet_hash(), TraceHash::default().value());
    }

    #[test]
    fn the_radio_latch_is_the_bridges_rule_on_its_own() {
        let mut l = RcLatch::new();
        let good = sticks(0.0, 0.0, 0.5, 0.0);
        assert_eq!(l.at(Micros(0), &None), None);
        let want = rc_pulses(&good);
        assert_eq!(l.at(Micros(20_000), &at(20_000, good)), want);
        assert_eq!(l.at(Micros(40_000), &None), want);
        assert_eq!(l.at(Micros(40_500), &None), None);
    }
}
