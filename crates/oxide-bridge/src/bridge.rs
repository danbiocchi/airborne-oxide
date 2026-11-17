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
//! `armed` is "any of the four pulses is above 1000". The pulses are lockstep
//! data and repeat bit for bit; the autopilot's own armed flag arrives over a
//! channel whose timing depends on the wall clock and is audit data only.

use core::fmt;
use std::time::Duration;

use oxide_core::io::RcSample;
use oxide_core::time::Micros;
use oxide_sim::autopilot::{Autopilot, AutopilotInput, AutopilotOutput};
use oxide_sim::scenario::COMMAND_PERIOD_US;

use crate::link::{Link, LinkError};
use crate::motors::{DEFAULT_SPIN_MAX, DEFAULT_SPIN_MIN, MotorError, MotorMap};
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
    /// datagram, the socket.
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

/// Fills the reply from one tick's input: the truth, as the ideal IMU sample
/// forms it. This is the one small function that a later part replaces or
/// extends (our IMU model's samples, a GPS error).
pub fn reply_from_input<const N: usize>(
    timestamp_ns: u64,
    input: &AutopilotInput<'_, N>,
    battery: bool,
    rc: Option<[u16; 8]>,
) -> Reply {
    let b = input.body;
    let accel = b.attitude.inverse_rotate(input.specific_force_world);
    Reply {
        timestamp_ns,
        gyro: [b.rate.x, b.rate.y, b.rate.z],
        accel_body: [accel.x, accel.y, accel.z],
        position: [b.position.x, b.position.y, b.position.z],
        velocity: [b.velocity.x, b.velocity.y, b.velocity.z],
        quaternion: [b.attitude.w, b.attitude.x, b.attitude.y, b.attitude.z],
        battery: battery.then_some(BatteryReading {
            voltage: input.battery_v,
            current: input.battery_a,
        }),
        rc,
    }
}

/// The JSON bridge as the simulator's autopilot.
#[derive(Debug)]
pub struct JsonBridge {
    link: Link,
    config: BridgeConfig,
    motors: MotorMap,
    line: String,
    /// The last valid radio frame and the tick it arrived at.
    last_rc: Option<(Micros, [u16; 8])>,
    frames: u64,
}

impl JsonBridge {
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
            last_rc: None,
            frames: 0,
        })
    }

    pub fn config(&self) -> &BridgeConfig {
        &self.config
    }
    pub fn link(&self) -> &Link {
        &self.link
    }
    /// Replies sent so far (one per successful tick).
    pub fn frames(&self) -> u64 {
        self.frames
    }

    /// The radio channels to send at `t`: the last valid frame while it is no
    /// older than the radio's period. A frame with failsafe or a lost frame
    /// ends it; so does a silent radio.
    fn rc_at(&mut self, t: Micros, input: &AutopilotInput<'_, 4>) -> Option<[u16; 8]> {
        if let Some(frame) = &input.rc {
            self.last_rc = if frame.value.is_valid() {
                rc_pulses(&frame.value).map(|p| (t, p))
            } else {
                None
            };
        }
        let (at, pulses) = self.last_rc?;
        (t.since(at) <= COMMAND_PERIOD_US).then_some(pulses)
    }
}

impl Autopilot<4> for JsonBridge {
    type Error = BridgeError;

    fn period_us(&self) -> u64 {
        self.config.step_us
    }

    fn tick(
        &mut self,
        t: Micros,
        input: &AutopilotInput<'_, 4>,
    ) -> Result<AutopilotOutput<4>, BridgeError> {
        let step = t.0 / self.config.step_us;
        let rc = if self.config.send_rc {
            self.rc_at(t, input)
        } else {
            None
        };
        let reply = reply_from_input(
            timestamp_ns(self.config.step_us, step),
            input,
            self.config.send_battery,
            rc,
        );
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
        let pulses = packet.pwm.get(..4).unwrap_or(&[]);
        Ok(AutopilotOutput {
            t_valid: t,
            motors: self.motors.rotors(pulses),
            armed: pulses.iter().any(|&p| p > 1000),
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
}
