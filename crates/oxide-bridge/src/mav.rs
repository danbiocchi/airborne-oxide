// SPDX-License-Identifier: AGPL-3.0-or-later
//! The MAVLink client for the simulator program (ADR-0006 §6).
//!
//! MAVLink 2 over TCP to the program's serial port. The program is in
//! lockstep with our exchange, so nothing here ever waits for an answer:
//! [`MavClient::poll`] reads what is there and returns, and the sends write
//! to a local socket. The caller polls once per exchange, which also keeps
//! the program's send queue from filling (it stops, and both programs hang,
//! when it does).
//!
//! The `mavlink` crate does not appear in this module's public interface:
//! events and arguments are our own types.
//!
//! Two limits of the crate's feature set (`mav2-message-extensions` is off,
//! as ADR-0006 §10 fixes the features): fields that MAVLink 2 appended to a
//! message are not decoded. So a `STATUSTEXT` chunk carries no id and no
//! sequence number, and `GPS_RAW_INT` carries no accuracy in millimetres.

use core::fmt;
use std::collections::BTreeMap;
use std::io;
use std::time::{Duration, Instant};

use mavlink::dialects::ardupilotmega::{
    COMMAND_LONG_DATA, HEARTBEAT_DATA, MavAutopilot, MavFrame, MavMessage, MavModeFlag,
    MavParamType, MavState, MavType, PARAM_REQUEST_READ_DATA, PARAM_SET_DATA,
    PositionTargetTypemask, SET_POSITION_TARGET_LOCAL_NED_DATA,
};
use mavlink::error::MessageReadError;
use mavlink::{MavConnection, MavHeader, MavlinkVersion, Message, MessageData};

/// The most messages one `poll` takes in, so that a flood cannot keep the
/// exchange from going on.
const MAX_PER_POLL: usize = 4096;
/// The length of a `STATUSTEXT` chunk. A chunk of this length may be
/// continued by the next one.
const STATUSTEXT_CHUNK: usize = 50;
/// Message ids that [`MavClient::set_message_interval`] callers use.
pub mod id {
    pub const HEARTBEAT: u32 = 0;
    pub const SYS_STATUS: u32 = 1;
    pub const SYSTEM_TIME: u32 = 2;
    pub const GPS_RAW_INT: u32 = 24;
    pub const RAW_IMU: u32 = 27;
    pub const SCALED_PRESSURE: u32 = 29;
    pub const ATTITUDE: u32 = 30;
    pub const LOCAL_POSITION_NED: u32 = 32;
    pub const EKF_STATUS_REPORT: u32 = 193;
}

/// The type mask of a position target that gives position and yaw and
/// ignores velocity, acceleration and yaw rate. From the MAVLink definition
/// of `POSITION_TARGET_TYPEMASK`: bits 3 to 5 (velocity, 8 + 16 + 32), 6 to 8
/// (acceleration, 64 + 128 + 256) and 11 (yaw rate, 2048) are set; bit 10
/// (yaw ignore, 1024) is clear. `504 + 2048 = 2552`. "Position only, yaw
/// ignored" is this plus 1024, 3576.
pub const TYPE_MASK_POSITION_YAW: u16 = 2552;

/// Why a MAVLink call failed.
#[derive(Debug)]
pub enum MavError {
    /// Nobody listened on the port within the limit.
    Connect {
        port: u16,
        waited: Duration,
        last: Option<io::Error>,
    },
    /// The socket failed.
    Io(io::Error),
    /// The program closed the connection.
    Closed,
    /// An argument that cannot be sent.
    BadArgument { what: String },
}

impl fmt::Display for MavError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Connect { port, waited, last } => {
                write!(
                    f,
                    "nobody listened on 127.0.0.1:{port} within {} ms",
                    waited.as_millis()
                )?;
                if let Some(e) = last {
                    write!(f, " (last error: {e})")?;
                }
                Ok(())
            }
            Self::Io(e) => write!(f, "the MAVLink connection failed: {e}"),
            Self::Closed => write!(f, "the MAVLink connection was closed by the simulator"),
            Self::BadArgument { what } => write!(f, "cannot send: {what}"),
        }
    }
}

impl std::error::Error for MavError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Io(e) => Some(e),
            Self::Connect { last, .. } => last.as_ref().map(|e| e as _),
            _ => None,
        }
    }
}

impl From<io::Error> for MavError {
    fn from(e: io::Error) -> Self {
        Self::Io(e)
    }
}

/// Something the program told us, in our own terms.
#[derive(Clone, Debug, PartialEq)]
pub enum MavEvent {
    /// `HEARTBEAT`. `mode` is the vehicle's own mode number (ArduCopter: 0
    /// STABILIZE, 2 ALT_HOLD, 4 GUIDED, 5 LOITER, 6 RTL, 9 LAND);
    /// `system_status` is MAV_STATE (3 standby, 4 active, 5 critical).
    Heartbeat {
        system_id: u32,
        component_id: u8,
        mode: u32,
        armed: bool,
        system_status: u8,
    },
    /// `STATUSTEXT`, chunks joined. `severity` is the syslog level (0
    /// emergency to 7 debug).
    StatusText { severity: u8, text: String },
    /// `COMMAND_ACK`. `result` is MAV_RESULT, 0 accepted.
    CommandAck { command: u32, result: u8 },
    /// `PARAM_VALUE`, the name without its padding.
    ParamValue {
        name: String,
        value: f32,
        index: u16,
        count: u16,
    },
    /// `GPS_RAW_INT`: degrees times 1e7, millimetres, `eph` is the HDOP times
    /// 100, `vel` the ground speed in cm/s, `fix` MAV GPS_FIX_TYPE (3 = 3D).
    GpsRaw {
        time_usec: u64,
        lat: i32,
        lon: i32,
        alt_mm: i32,
        fix: u8,
        eph: u16,
        epv: u16,
        vel_cms: u16,
        satellites: u8,
    },
    /// `SCALED_IMU` (`id` 1), `SCALED_IMU2` (2) or `SCALED_IMU3` (3):
    /// milli-g, milli-rad/s, milli-gauss.
    ScaledImu {
        id: u8,
        time_boot_ms: u32,
        acc_mg: [i16; 3],
        gyro_mrad_s: [i16; 3],
        mag_mgauss: [i16; 3],
    },
    /// `RAW_IMU`: raw sensor units.
    RawImu {
        time_usec: u64,
        acc: [i16; 3],
        gyro: [i16; 3],
        mag: [i16; 3],
    },
    /// `SCALED_PRESSURE` (`id` 1), `SCALED_PRESSURE2` (2) or `3`: hPa and
    /// centi-degrees Celsius.
    ScaledPressure {
        id: u8,
        time_boot_ms: u32,
        press_abs_hpa: f32,
        press_diff_hpa: f32,
        temperature_cdegc: i16,
    },
    /// `LOCAL_POSITION_NED`: metres and m/s from the vehicle's origin.
    LocalPosition {
        time_boot_ms: u32,
        n: f32,
        e: f32,
        d: f32,
        vn: f32,
        ve: f32,
        vd: f32,
    },
    /// `SYS_STATUS`: millivolts, centi-amps, percent (-1 unknown) and the
    /// MAV_SYS_STATUS_SENSOR bit sets.
    SysStatus {
        voltage_mv: u16,
        current_ca: i16,
        battery_remaining: i8,
        sensors_present: u32,
        sensors_enabled: u32,
        sensors_health: u32,
    },
    /// `EKF_STATUS_REPORT`: the flag bits and the variances (ratios).
    EkfStatus {
        flags: u16,
        velocity_variance: f32,
        pos_horiz_variance: f32,
        pos_vert_variance: f32,
        compass_variance: f32,
        terrain_alt_variance: f32,
    },
    /// `SYSTEM_TIME`: the program's own clock.
    SystemTime {
        time_unix_usec: u64,
        time_boot_ms: u32,
    },
}

/// A `STATUSTEXT` chunk that may be continued.
#[derive(Debug)]
struct PendingText {
    severity: u8,
    text: String,
}

/// The connection.
pub struct MavClient {
    conn: mavlink::Connection<MavMessage>,
    system_id: u8,
    component_id: u8,
    target_system: u8,
    target_component: u8,
    pending_text: Option<PendingText>,
    other: BTreeMap<u32, u64>,
    unparsed: u64,
    received: u64,
}

impl fmt::Debug for MavClient {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("MavClient")
            .field("system_id", &self.system_id)
            .field("component_id", &self.component_id)
            .field("received", &self.received)
            .finish_non_exhaustive()
    }
}

fn bad(what: impl Into<String>) -> MavError {
    MavError::BadArgument { what: what.into() }
}

/// Text up to the first NUL, bytes that are not UTF-8 replaced.
fn text_of(bytes: &[u8]) -> String {
    let end = bytes.iter().position(|&b| b == 0).unwrap_or(bytes.len());
    String::from_utf8_lossy(bytes.get(..end).unwrap_or(&[])).into_owned()
}

fn param_id(name: &str) -> Result<[u8; 16], MavError> {
    if name.is_empty() || name.len() > 16 || !name.bytes().all(|b| b.is_ascii_graphic()) {
        return Err(bad(format!(
            "parameter name {name:?} is not 1 to 16 printable ASCII characters"
        )));
    }
    let mut id = [0u8; 16];
    for (slot, b) in id.iter_mut().zip(name.bytes()) {
        *slot = b;
    }
    Ok(id)
}

impl MavClient {
    /// Connects to `127.0.0.1:port`, trying again until the program listens or
    /// `limit` has passed. `system_id` and `component_id` are ours.
    pub fn connect(
        port: u16,
        system_id: u8,
        component_id: u8,
        limit: Duration,
    ) -> Result<Self, MavError> {
        let start = Instant::now();
        let address = format!("tcpout:127.0.0.1:{port}");
        loop {
            match mavlink::connect::<MavMessage>(&address) {
                Ok(mut conn) => {
                    conn.set_protocol_version(MavlinkVersion::V2);
                    return Ok(Self {
                        conn,
                        system_id,
                        component_id,
                        target_system: 1,
                        target_component: 1,
                        pending_text: None,
                        other: BTreeMap::new(),
                        unparsed: 0,
                        received: 0,
                    });
                }
                Err(e) => {
                    if start.elapsed() >= limit {
                        return Err(MavError::Connect {
                            port,
                            waited: start.elapsed(),
                            last: Some(e),
                        });
                    }
                }
            }
            std::thread::sleep(Duration::from_millis(10));
        }
    }

    /// Sets the system and component the commands are addressed to (default
    /// 1 and 1, ArduCopter's own).
    pub fn set_target(&mut self, system: u8, component: u8) {
        self.target_system = system;
        self.target_component = component;
    }

    /// Messages read so far, those that became events and the others.
    pub fn received(&self) -> u64 {
        self.received
    }

    /// How many messages of each id were read and not kept as events.
    pub fn other_counts(&self) -> &BTreeMap<u32, u64> {
        &self.other
    }

    /// Frames that could not be parsed.
    pub fn unparsed(&self) -> u64 {
        self.unparsed
    }

    /// Reads everything that is there, without blocking, and appends the
    /// events. A closed connection is an error.
    pub fn poll(&mut self, events: &mut Vec<MavEvent>) -> Result<(), MavError> {
        for _ in 0..MAX_PER_POLL {
            match self.conn.try_recv() {
                Ok((header, msg)) => {
                    self.received += 1;
                    self.handle(&header, msg, events);
                }
                Err(MessageReadError::Io(e)) => match e.kind() {
                    io::ErrorKind::WouldBlock | io::ErrorKind::Interrupted => return Ok(()),
                    io::ErrorKind::UnexpectedEof => return Err(MavError::Closed),
                    io::ErrorKind::ConnectionReset
                    | io::ErrorKind::ConnectionAborted
                    | io::ErrorKind::BrokenPipe => return Err(MavError::Closed),
                    _ => return Err(MavError::Io(e)),
                },
                Err(MessageReadError::Parse(_)) => self.unparsed += 1,
            }
        }
        Ok(())
    }

    fn handle(&mut self, header: &MavHeader, msg: MavMessage, out: &mut Vec<MavEvent>) {
        if !matches!(msg, MavMessage::STATUSTEXT(_)) {
            // A chunk is not continued by anything but the next status text:
            // the text so far is whole.
            self.flush_text(out);
        }
        match msg {
            MavMessage::HEARTBEAT(d) => out.push(MavEvent::Heartbeat {
                system_id: header.system_id,
                component_id: header.component_id,
                mode: d.custom_mode,
                armed: d
                    .base_mode
                    .contains(MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED),
                system_status: d.system_status as u8,
            }),
            MavMessage::STATUSTEXT(d) => self.status_text(d.severity as u8, text_of(&*d.text), out),
            MavMessage::COMMAND_ACK(d) => out.push(MavEvent::CommandAck {
                command: d.command as u32,
                result: d.result as u32 as u8,
            }),
            MavMessage::PARAM_VALUE(d) => out.push(MavEvent::ParamValue {
                name: text_of(&*d.param_id),
                value: d.param_value,
                index: d.param_index,
                count: d.param_count,
            }),
            MavMessage::GPS_RAW_INT(d) => out.push(MavEvent::GpsRaw {
                time_usec: d.time_usec,
                lat: d.lat,
                lon: d.lon,
                alt_mm: d.alt,
                fix: d.fix_type as u8,
                eph: d.eph,
                epv: d.epv,
                vel_cms: d.vel,
                satellites: d.satellites_visible,
            }),
            MavMessage::SCALED_IMU(d) => out.push(MavEvent::ScaledImu {
                id: 1,
                time_boot_ms: d.time_boot_ms,
                acc_mg: [d.xacc, d.yacc, d.zacc],
                gyro_mrad_s: [d.xgyro, d.ygyro, d.zgyro],
                mag_mgauss: [d.xmag, d.ymag, d.zmag],
            }),
            MavMessage::SCALED_IMU2(d) => out.push(MavEvent::ScaledImu {
                id: 2,
                time_boot_ms: d.time_boot_ms,
                acc_mg: [d.xacc, d.yacc, d.zacc],
                gyro_mrad_s: [d.xgyro, d.ygyro, d.zgyro],
                mag_mgauss: [d.xmag, d.ymag, d.zmag],
            }),
            MavMessage::SCALED_IMU3(d) => out.push(MavEvent::ScaledImu {
                id: 3,
                time_boot_ms: d.time_boot_ms,
                acc_mg: [d.xacc, d.yacc, d.zacc],
                gyro_mrad_s: [d.xgyro, d.ygyro, d.zgyro],
                mag_mgauss: [d.xmag, d.ymag, d.zmag],
            }),
            MavMessage::RAW_IMU(d) => out.push(MavEvent::RawImu {
                time_usec: d.time_usec,
                acc: [d.xacc, d.yacc, d.zacc],
                gyro: [d.xgyro, d.ygyro, d.zgyro],
                mag: [d.xmag, d.ymag, d.zmag],
            }),
            MavMessage::SCALED_PRESSURE(d) => out.push(MavEvent::ScaledPressure {
                id: 1,
                time_boot_ms: d.time_boot_ms,
                press_abs_hpa: d.press_abs,
                press_diff_hpa: d.press_diff,
                temperature_cdegc: d.temperature,
            }),
            MavMessage::SCALED_PRESSURE2(d) => out.push(MavEvent::ScaledPressure {
                id: 2,
                time_boot_ms: d.time_boot_ms,
                press_abs_hpa: d.press_abs,
                press_diff_hpa: d.press_diff,
                temperature_cdegc: d.temperature,
            }),
            MavMessage::SCALED_PRESSURE3(d) => out.push(MavEvent::ScaledPressure {
                id: 3,
                time_boot_ms: d.time_boot_ms,
                press_abs_hpa: d.press_abs,
                press_diff_hpa: d.press_diff,
                temperature_cdegc: d.temperature,
            }),
            MavMessage::LOCAL_POSITION_NED(d) => out.push(MavEvent::LocalPosition {
                time_boot_ms: d.time_boot_ms,
                n: d.x,
                e: d.y,
                d: d.z,
                vn: d.vx,
                ve: d.vy,
                vd: d.vz,
            }),
            MavMessage::SYS_STATUS(d) => out.push(MavEvent::SysStatus {
                voltage_mv: d.voltage_battery,
                current_ca: d.current_battery,
                battery_remaining: d.battery_remaining,
                sensors_present: d.onboard_control_sensors_present.bits(),
                sensors_enabled: d.onboard_control_sensors_enabled.bits(),
                sensors_health: d.onboard_control_sensors_health.bits(),
            }),
            MavMessage::EKF_STATUS_REPORT(d) => out.push(MavEvent::EkfStatus {
                flags: d.flags.bits(),
                velocity_variance: d.velocity_variance,
                pos_horiz_variance: d.pos_horiz_variance,
                pos_vert_variance: d.pos_vert_variance,
                compass_variance: d.compass_variance,
                terrain_alt_variance: d.terrain_alt_variance,
            }),
            MavMessage::SYSTEM_TIME(d) => out.push(MavEvent::SystemTime {
                time_unix_usec: d.time_unix_usec,
                time_boot_ms: d.time_boot_ms,
            }),
            other => {
                *self.other.entry(other.message_id()).or_insert(0) += 1;
            }
        }
    }

    /// Takes a status text chunk. A chunk shorter than the largest ends its
    /// text; a full one may be continued by the next chunk of the same
    /// severity (the chunk's own id and sequence number are not decoded, see
    /// the module notes) and is held until then.
    fn status_text(&mut self, severity: u8, chunk: String, out: &mut Vec<MavEvent>) {
        if let Some(p) = self.pending_text.take() {
            if p.severity == severity {
                self.pending_text = Some(PendingText {
                    severity,
                    text: p.text + &chunk,
                });
            } else {
                out.push(MavEvent::StatusText {
                    severity: p.severity,
                    text: p.text,
                });
                self.pending_text = Some(PendingText {
                    severity,
                    text: chunk,
                });
            }
        } else {
            self.pending_text = Some(PendingText {
                severity,
                text: chunk,
            });
        }
        let full = self
            .pending_text
            .as_ref()
            .is_some_and(|p| p.text.len() % STATUSTEXT_CHUNK == 0 && !p.text.is_empty());
        if !full {
            self.flush_text(out);
        }
    }

    fn flush_text(&mut self, out: &mut Vec<MavEvent>) {
        if let Some(p) = self.pending_text.take() {
            out.push(MavEvent::StatusText {
                severity: p.severity,
                text: p.text,
            });
        }
    }

    fn send(&self, msg: &MavMessage) -> Result<(), MavError> {
        let header = MavHeader {
            system_id: u32::from(self.system_id),
            component_id: self.component_id,
            sequence: 0,
        };
        self.conn
            .send(&header, msg)
            .map(|_| ())
            .map_err(|e| match e {
                mavlink::error::MessageWriteError::Io(e) => match e.kind() {
                    io::ErrorKind::BrokenPipe
                    | io::ErrorKind::ConnectionReset
                    | io::ErrorKind::ConnectionAborted => MavError::Closed,
                    _ => MavError::Io(e),
                },
                #[allow(unreachable_patterns)]
                other => MavError::BadArgument {
                    what: other.to_string(),
                },
            })
    }

    /// Our own `HEARTBEAT` (a ground station, no autopilot, no mode).
    pub fn heartbeat(&self) -> Result<(), MavError> {
        self.send(&MavMessage::HEARTBEAT(HEARTBEAT_DATA {
            custom_mode: 0,
            mavtype: MavType::MAV_TYPE_GCS,
            autopilot: MavAutopilot::MAV_AUTOPILOT_INVALID,
            base_mode: MavModeFlag::empty(),
            system_status: MavState::MAV_STATE_ACTIVE,
            mavlink_version: 3,
        }))
    }

    /// `MAV_CMD_SET_MESSAGE_INTERVAL`: `message_id` at `hz`; a rate of zero
    /// or less switches the message off.
    pub fn set_message_interval(&self, message_id: u32, hz: f32) -> Result<(), MavError> {
        if !hz.is_finite() {
            return Err(bad("the message rate is not a finite number"));
        }
        let interval_us = if hz > 0.0 { 1.0e6 / hz } else { -1.0 };
        self.command_long(
            511,
            [message_id as f32, interval_us, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
    }

    /// `MAV_CMD_DO_SET_MODE` with the custom-mode flag: the vehicle's own mode
    /// number.
    pub fn set_mode(&self, mode: u32) -> Result<(), MavError> {
        self.command_long(176, [1.0, mode as f32, 0.0, 0.0, 0.0, 0.0, 0.0])
    }

    /// `MAV_CMD_COMPONENT_ARM_DISARM`.
    pub fn arm(&self, arm: bool) -> Result<(), MavError> {
        self.command_long(
            400,
            [if arm { 1.0 } else { 0.0 }, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
    }

    /// `MAV_CMD_NAV_TAKEOFF` to `height_m` above the start point.
    pub fn takeoff(&self, height_m: f32) -> Result<(), MavError> {
        self.command_long(22, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, height_m])
    }

    /// `SET_POSITION_TARGET_LOCAL_NED` in `MAV_FRAME_LOCAL_NED`: north, east
    /// and down in metres from the origin, yaw in radians (0 north, clockwise
    /// positive). The type mask is [`TYPE_MASK_POSITION_YAW`]: position and
    /// yaw are used, the rest ignored.
    pub fn position_target_local_ned(
        &self,
        n: f32,
        e: f32,
        d: f32,
        yaw: f32,
    ) -> Result<(), MavError> {
        if ![n, e, d, yaw].iter().all(|v| v.is_finite()) {
            return Err(bad("a position target is not a finite number"));
        }
        self.send(&MavMessage::SET_POSITION_TARGET_LOCAL_NED(
            SET_POSITION_TARGET_LOCAL_NED_DATA {
                time_boot_ms: 0,
                x: n,
                y: e,
                z: d,
                vx: 0.0,
                vy: 0.0,
                vz: 0.0,
                afx: 0.0,
                afy: 0.0,
                afz: 0.0,
                yaw,
                yaw_rate: 0.0,
                type_mask: PositionTargetTypemask::from_bits_truncate(TYPE_MASK_POSITION_YAW),
                target_system: self.target_system,
                target_component: self.target_component,
                coordinate_frame: MavFrame::MAV_FRAME_LOCAL_NED,
            },
        ))
    }

    /// `PARAM_SET` of a 32-bit float; the program answers with a
    /// `PARAM_VALUE`.
    pub fn param_set(&self, name: &str, value: f32) -> Result<(), MavError> {
        if !value.is_finite() {
            return Err(bad(format!(
                "parameter {name} is set to a non-finite value"
            )));
        }
        self.send(&MavMessage::PARAM_SET(PARAM_SET_DATA {
            param_value: value,
            target_system: self.target_system,
            target_component: self.target_component,
            param_id: param_id(name)?.into(),
            param_type: MavParamType::MAV_PARAM_TYPE_REAL32,
        }))
    }

    /// `PARAM_REQUEST_READ` by name.
    pub fn param_request(&self, name: &str) -> Result<(), MavError> {
        self.send(&MavMessage::PARAM_REQUEST_READ(PARAM_REQUEST_READ_DATA {
            param_index: -1,
            target_system: self.target_system,
            target_component: self.target_component,
            param_id: param_id(name)?.into(),
        }))
    }

    /// `COMMAND_LONG` with MAV_CMD number `command` and its seven parameters.
    /// A number the dialect does not know is refused.
    pub fn command_long(&self, command: u32, params: [f32; 7]) -> Result<(), MavError> {
        let command = u16::try_from(command)
            .map_err(|_| bad(format!("MAV_CMD {command} does not fit in 16 bits")))?;
        // The command goes through the message's own parser, which knows the
        // numbers of the dialect; no other path to the type exists without
        // one more dependency.
        let mut payload = Vec::with_capacity(33);
        for p in params {
            payload.extend_from_slice(&p.to_le_bytes());
        }
        payload.extend_from_slice(&command.to_le_bytes());
        payload.push(self.target_system);
        payload.push(self.target_component);
        payload.push(0);
        let msg = MavMessage::parse(MavlinkVersion::V2, COMMAND_LONG_DATA::ID, &payload)
            .map_err(|_| bad(format!("MAV_CMD {command} is not in the dialect")))?;
        self.send(&msg)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mavlink::MavlinkReader;
    use mavlink::dialects::ardupilotmega::{
        ATTITUDE_DATA, COMMAND_LONG_DATA, EkfStatusFlags, GPS_RAW_INT_DATA, GpsFixType,
        LOCAL_POSITION_NED_DATA, MavCmd, MavSeverity, PARAM_VALUE_DATA, PositionTargetTypemask,
        SCALED_PRESSURE2_DATA, STATUSTEXT_DATA, SYSTEM_TIME_DATA,
    };
    use mavlink::types::CharArray;
    use std::net::{TcpListener, TcpStream};

    const WAIT: Duration = Duration::from_secs(5);

    /// The program's side of the connection, played with the crate itself.
    struct Peer {
        stream: TcpStream,
        reader: MavlinkReader<TcpStream>,
    }

    impl Peer {
        fn send(&mut self, msg: MavMessage) {
            let h = MavHeader {
                system_id: 1,
                component_id: 1,
                sequence: 0,
            };
            mavlink::write_versioned_msg(&mut self.stream, MavlinkVersion::V2, h, &msg)
                .expect("peer write");
        }
        /// The next message the client sent.
        fn recv(&mut self) -> (MavHeader, MavMessage) {
            self.reader
                .read_message::<MavMessage>(MavlinkVersion::V2)
                .expect("peer read")
        }
    }

    fn pair() -> (MavClient, Peer) {
        let listener = TcpListener::bind("127.0.0.1:0").expect("bind");
        let port = listener.local_addr().expect("addr").port();
        let client = MavClient::connect(port, 255, 190, WAIT).expect("connect");
        let (stream, _) = listener.accept().expect("accept");
        stream.set_read_timeout(Some(WAIT)).expect("timeout");
        let reader = MavlinkReader::new(stream.try_clone().expect("clone"));
        (client, Peer { stream, reader })
    }

    /// Polls until `want` events have come, or fails after a wait.
    fn events(c: &mut MavClient, want: usize) -> Vec<MavEvent> {
        let mut out = Vec::new();
        let t = Instant::now();
        while out.len() < want {
            c.poll(&mut out).expect("poll");
            assert!(
                t.elapsed() < WAIT,
                "only {} of {want} events: {out:?}",
                out.len()
            );
            std::thread::sleep(Duration::from_millis(2));
        }
        out
    }

    fn text50(c: u8) -> CharArray<50> {
        CharArray::new([c; 50])
    }

    fn long(peer: &mut Peer) -> COMMAND_LONG_DATA {
        let (h, m) = peer.recv();
        assert_eq!((h.system_id, h.component_id), (255, 190));
        match m {
            MavMessage::COMMAND_LONG(d) => {
                assert_eq!((d.target_system, d.target_component), (1, 1));
                d
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn a_heartbeat_with_the_armed_bit_and_mode_five_arrives_typed() {
        let (mut c, mut peer) = pair();
        peer.send(MavMessage::HEARTBEAT(HEARTBEAT_DATA {
            custom_mode: 5,
            mavtype: MavType::MAV_TYPE_QUADROTOR,
            autopilot: MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
            base_mode: MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED
                | MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            system_status: MavState::MAV_STATE_ACTIVE,
            mavlink_version: 3,
        }));
        let ev = events(&mut c, 1);
        assert!(
            matches!(
                ev[0],
                MavEvent::Heartbeat {
                    mode: 5,
                    armed: true,
                    system_status: 4,
                    system_id: 1,
                    component_id: 1
                }
            ),
            "{ev:?}"
        );
        peer.send(MavMessage::HEARTBEAT(HEARTBEAT_DATA {
            custom_mode: 4,
            base_mode: MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            system_status: MavState::MAV_STATE_STANDBY,
            ..HEARTBEAT_DATA::default()
        }));
        let ev = events(&mut c, 1);
        assert!(
            matches!(
                ev[0],
                MavEvent::Heartbeat {
                    mode: 4,
                    armed: false,
                    system_status: 3,
                    ..
                }
            ),
            "{ev:?}"
        );
    }

    #[test]
    fn a_status_text_in_two_chunks_arrives_as_one_text() {
        let (mut c, mut peer) = pair();
        let mut tail = [0u8; 50];
        tail[..4].copy_from_slice(b"tail");
        for chunk in [text50(b'a'), CharArray::new(tail)] {
            peer.send(MavMessage::STATUSTEXT(STATUSTEXT_DATA {
                severity: MavSeverity::MAV_SEVERITY_INFO,
                text: chunk,
            }));
        }
        let ev = events(&mut c, 1);
        assert_eq!(
            ev,
            [MavEvent::StatusText {
                severity: 6,
                text: format!("{}tail", "a".repeat(50))
            }]
        );
    }

    #[test]
    fn a_short_status_text_arrives_at_once_and_a_held_chunk_ends_at_another_message() {
        let (mut c, mut peer) = pair();
        let mut ready = [0u8; 50];
        ready[..15].copy_from_slice(b"ArduPilot Ready");
        peer.send(MavMessage::STATUSTEXT(STATUSTEXT_DATA {
            severity: MavSeverity::MAV_SEVERITY_INFO,
            text: CharArray::new(ready),
        }));
        let ev = events(&mut c, 1);
        assert_eq!(
            ev,
            [MavEvent::StatusText {
                severity: 6,
                text: "ArduPilot Ready".into()
            }]
        );
        // A full chunk is held; the next message that is not text ends it.
        peer.send(MavMessage::STATUSTEXT(STATUSTEXT_DATA {
            severity: MavSeverity::MAV_SEVERITY_WARNING,
            text: text50(b'x'),
        }));
        peer.send(MavMessage::SYSTEM_TIME(SYSTEM_TIME_DATA {
            time_unix_usec: 0,
            time_boot_ms: 7,
        }));
        let ev = events(&mut c, 2);
        assert!(
            matches!(&ev[0], MavEvent::StatusText { severity: 4, text } if text.len() == 50),
            "{ev:?}"
        );
        assert!(matches!(
            ev[1],
            MavEvent::SystemTime {
                time_boot_ms: 7,
                ..
            }
        ));
    }

    #[test]
    fn a_param_value_arrives_with_its_name_trimmed() {
        let (mut c, mut peer) = pair();
        let mut id = [0u8; 16];
        id[..12].copy_from_slice(b"SIM_BARO_RND");
        peer.send(MavMessage::PARAM_VALUE(PARAM_VALUE_DATA {
            param_value: 0.173,
            param_count: 1200,
            param_index: 65535,
            param_id: CharArray::new(id),
            param_type: MavParamType::MAV_PARAM_TYPE_REAL32,
        }));
        let ev = events(&mut c, 1);
        assert_eq!(
            ev,
            [MavEvent::ParamValue {
                name: "SIM_BARO_RND".into(),
                value: 0.173,
                index: 65535,
                count: 1200
            }]
        );
    }

    #[test]
    fn other_messages_are_counted_and_not_kept() {
        let (mut c, mut peer) = pair();
        for _ in 0..3 {
            peer.send(MavMessage::ATTITUDE(ATTITUDE_DATA::default()));
        }
        peer.send(MavMessage::SYSTEM_TIME(SYSTEM_TIME_DATA {
            time_unix_usec: 5,
            time_boot_ms: 9,
        }));
        let ev = events(&mut c, 1);
        assert_eq!(ev.len(), 1);
        assert_eq!(c.other_counts().get(&30), Some(&3));
        assert_eq!(c.received(), 4);
    }

    #[test]
    fn the_other_events_carry_their_fields() {
        let (mut c, mut peer) = pair();
        peer.send(MavMessage::GPS_RAW_INT(GPS_RAW_INT_DATA {
            time_usec: 11,
            lat: -353_632_610,
            lon: 1_491_652_300,
            alt: 584_000,
            eph: 121,
            epv: 200,
            vel: 31,
            cog: 0,
            fix_type: GpsFixType::GPS_FIX_TYPE_3D_FIX,
            satellites_visible: 10,
        }));
        peer.send(MavMessage::LOCAL_POSITION_NED(LOCAL_POSITION_NED_DATA {
            time_boot_ms: 4,
            x: 1.0,
            y: 2.0,
            z: -3.0,
            vx: 0.5,
            vy: 0.25,
            vz: -0.125,
        }));
        peer.send(MavMessage::SCALED_PRESSURE2(SCALED_PRESSURE2_DATA {
            time_boot_ms: 8,
            press_abs: 950.5,
            press_diff: 0.0,
            temperature: 2100,
        }));
        peer.send(MavMessage::EKF_STATUS_REPORT(
            mavlink::dialects::ardupilotmega::EKF_STATUS_REPORT_DATA {
                flags: EkfStatusFlags::from_bits_truncate(0b101),
                velocity_variance: 0.5,
                ..Default::default()
            },
        ));
        let ev = events(&mut c, 4);
        assert_eq!(
            ev[0],
            MavEvent::GpsRaw {
                time_usec: 11,
                lat: -353_632_610,
                lon: 1_491_652_300,
                alt_mm: 584_000,
                fix: 3,
                eph: 121,
                epv: 200,
                vel_cms: 31,
                satellites: 10
            }
        );
        assert_eq!(
            ev[1],
            MavEvent::LocalPosition {
                time_boot_ms: 4,
                n: 1.0,
                e: 2.0,
                d: -3.0,
                vn: 0.5,
                ve: 0.25,
                vd: -0.125
            }
        );
        assert_eq!(
            ev[2],
            MavEvent::ScaledPressure {
                id: 2,
                time_boot_ms: 8,
                press_abs_hpa: 950.5,
                press_diff_hpa: 0.0,
                temperature_cdegc: 2100
            }
        );
        assert!(
            matches!(ev[3], MavEvent::EkfStatus { flags: 0b101, velocity_variance, .. } if velocity_variance == 0.5),
            "{:?}",
            ev[3]
        );
    }

    #[test]
    fn poll_with_nothing_to_read_returns_at_once_and_adds_nothing() {
        let (mut c, _peer) = pair();
        let mut ev = Vec::new();
        let t = Instant::now();
        for _ in 0..5 {
            c.poll(&mut ev).expect("poll");
        }
        assert!(t.elapsed() < Duration::from_millis(50), "{:?}", t.elapsed());
        assert!(ev.is_empty());
    }

    #[test]
    fn each_send_puts_the_expected_message_on_the_wire() {
        let (c, mut peer) = pair();

        c.heartbeat().expect("heartbeat");
        let (h, m) = peer.recv();
        assert_eq!((h.system_id, h.component_id), (255, 190));
        let MavMessage::HEARTBEAT(d) = m else {
            panic!("{m:?}")
        };
        assert_eq!(d.mavtype, MavType::MAV_TYPE_GCS);
        assert_eq!(d.autopilot, MavAutopilot::MAV_AUTOPILOT_INVALID);

        c.set_message_interval(id::GPS_RAW_INT, 5.0)
            .expect("interval");
        let d = long(&mut peer);
        assert_eq!(d.command, MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL);
        assert_eq!((d.param1, d.param2), (24.0, 200_000.0));
        c.set_message_interval(id::SYSTEM_TIME, 0.0).expect("off");
        let d = long(&mut peer);
        assert_eq!((d.param1, d.param2), (2.0, -1.0));

        c.set_mode(4).expect("mode");
        let d = long(&mut peer);
        assert_eq!(d.command, MavCmd::MAV_CMD_DO_SET_MODE);
        assert_eq!((d.param1, d.param2), (1.0, 4.0));

        c.arm(true).expect("arm");
        let d = long(&mut peer);
        assert_eq!(d.command, MavCmd::MAV_CMD_COMPONENT_ARM_DISARM);
        assert_eq!(d.param1, 1.0);
        c.arm(false).expect("disarm");
        assert_eq!(long(&mut peer).param1, 0.0);

        c.takeoff(10.0).expect("takeoff");
        let d = long(&mut peer);
        assert_eq!(d.command, MavCmd::MAV_CMD_NAV_TAKEOFF);
        assert_eq!(d.param7, 10.0);

        c.command_long(21, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0])
            .expect("land");
        let d = long(&mut peer);
        assert_eq!(d.command, MavCmd::MAV_CMD_NAV_LAND);
        assert_eq!(
            [
                d.param1, d.param2, d.param3, d.param4, d.param5, d.param6, d.param7
            ],
            [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]
        );

        c.position_target_local_ned(1.5, -2.5, -3.0, 0.5)
            .expect("target");
        let (_, m) = peer.recv();
        let MavMessage::SET_POSITION_TARGET_LOCAL_NED(d) = m else {
            panic!("{m:?}")
        };
        assert_eq!((d.x, d.y, d.z, d.yaw), (1.5, -2.5, -3.0, 0.5));
        assert_eq!(d.coordinate_frame, MavFrame::MAV_FRAME_LOCAL_NED);
        assert_eq!(d.type_mask.bits(), 2552);
        // Position and yaw used; velocity, acceleration, yaw rate ignored.
        assert!(!d.type_mask.intersects(
            PositionTargetTypemask::POSITION_TARGET_TYPEMASK_X_IGNORE
                | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_Y_IGNORE
                | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_Z_IGNORE
                | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_YAW_IGNORE
        ));
        assert!(d.type_mask.contains(
            PositionTargetTypemask::POSITION_TARGET_TYPEMASK_VX_IGNORE
                | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_VY_IGNORE
                | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_VZ_IGNORE
                | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_AX_IGNORE
                | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_AY_IGNORE
                | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_AZ_IGNORE
                | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        ));
        assert_eq!((d.target_system, d.target_component), (1, 1));

        c.param_set("SIM_BARO_RND", 0.173).expect("set");
        let (_, m) = peer.recv();
        let MavMessage::PARAM_SET(d) = m else {
            panic!("{m:?}")
        };
        assert_eq!(text_of(&*d.param_id), "SIM_BARO_RND");
        assert_eq!(d.param_value, 0.173);
        assert_eq!(d.param_type, MavParamType::MAV_PARAM_TYPE_REAL32);
        assert_eq!((d.target_system, d.target_component), (1, 1));

        c.param_request("FRAME_CLASS").expect("request");
        let (_, m) = peer.recv();
        let MavMessage::PARAM_REQUEST_READ(d) = m else {
            panic!("{m:?}")
        };
        assert_eq!(text_of(&*d.param_id), "FRAME_CLASS");
        assert_eq!(d.param_index, -1);
    }

    #[test]
    fn a_target_other_than_the_default_is_used() {
        let (mut c, mut peer) = pair();
        c.set_target(7, 3);
        c.arm(true).expect("arm");
        let (_, m) = peer.recv();
        let MavMessage::COMMAND_LONG(d) = m else {
            panic!("{m:?}")
        };
        assert_eq!((d.target_system, d.target_component), (7, 3));
    }

    #[test]
    fn what_cannot_be_sent_is_refused_and_nothing_is_written() {
        let (c, mut peer) = pair();
        assert!(c.param_set("A_NAME_FAR_TOO_LONG_FOR_MAVLINK", 1.0).is_err());
        assert!(c.param_set("", 1.0).is_err());
        assert!(c.param_set("A B", 1.0).is_err());
        assert!(c.param_set("OK", f32::NAN).is_err());
        assert!(c.param_request("").is_err());
        assert!(c.set_message_interval(24, f32::NAN).is_err());
        assert!(
            c.position_target_local_ned(f32::NAN, 0.0, 0.0, 0.0)
                .is_err()
        );
        assert!(c.command_long(70_000, [0.0; 7]).is_err());
        let e = c.command_long(65_000, [0.0; 7]).unwrap_err();
        assert!(matches!(e, MavError::BadArgument { .. }), "{e:?}");
        // The next thing on the wire is the heartbeat sent now.
        c.heartbeat().expect("heartbeat");
        assert!(matches!(peer.recv().1, MavMessage::HEARTBEAT(_)));
    }

    #[test]
    fn connect_to_a_port_nobody_listens_on_fails_within_the_limit_and_names_the_port() {
        let port = {
            let l = TcpListener::bind("127.0.0.1:0").expect("bind");
            l.local_addr().expect("addr").port()
        };
        let t = Instant::now();
        let e = MavClient::connect(port, 255, 190, Duration::from_millis(300)).unwrap_err();
        let took = t.elapsed();
        assert!(took >= Duration::from_millis(300), "{took:?}");
        assert!(took < Duration::from_secs(2), "{took:?}");
        assert!(
            matches!(e, MavError::Connect { port: p, .. } if p == port),
            "{e:?}"
        );
        assert!(e.to_string().contains(&port.to_string()), "{e}");
        assert!(e.to_string().contains("ms"), "{e}");
    }

    #[test]
    fn a_peer_that_closes_makes_the_next_poll_an_error() {
        let (mut c, peer) = pair();
        drop(peer);
        let mut ev = Vec::new();
        let t = Instant::now();
        let e = loop {
            match c.poll(&mut ev) {
                Ok(()) => {
                    assert!(t.elapsed() < WAIT, "the close was never seen");
                    std::thread::sleep(Duration::from_millis(2));
                }
                Err(e) => break e,
            }
        };
        assert!(matches!(e, MavError::Closed), "{e:?}");
        assert!(ev.is_empty());
    }

    #[test]
    fn a_message_already_read_is_delivered_before_the_close_is_reported() {
        let (mut c, mut peer) = pair();
        peer.send(MavMessage::SYSTEM_TIME(SYSTEM_TIME_DATA {
            time_unix_usec: 0,
            time_boot_ms: 1,
        }));
        drop(peer);
        let mut ev = Vec::new();
        let t = Instant::now();
        let e = loop {
            match c.poll(&mut ev) {
                Ok(()) => {
                    assert!(t.elapsed() < WAIT);
                    std::thread::sleep(Duration::from_millis(2));
                }
                Err(e) => break e,
            }
        };
        assert!(matches!(e, MavError::Closed), "{e:?}");
        assert_eq!(ev.len(), 1, "{ev:?}");
    }
}
