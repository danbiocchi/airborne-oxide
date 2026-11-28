// SPDX-License-Identifier: AGPL-3.0-or-later
//! Fault injection and link models (DESIGN §5.4, ADR-0003).
//!
//! A [`Fault`] is a kind plus a time window in simulation seconds. The
//! simulator applies the active faults each tick to the inputs it feeds the
//! core and to the vehicle model; the core never sees the fault list.

use oxide_core::time::Micros;

use crate::SimError;

/// Ground-station heartbeat period, µs (1 Hz).
pub const GCS_HEARTBEAT_PERIOD_US: u64 = 1_000_000;
/// Brain heartbeat period, µs (5 Hz).
pub const BRAIN_HEARTBEAT_PERIOD_US: u64 = 200_000;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FaultKind {
    /// The rotor produces `scale` of its normal thrust and torque (0 = dead).
    MotorLoss {
        rotor: usize,
        scale: f64,
    },
    /// The sensor delivers nothing.
    GpsLoss,
    ImuLoss,
    BaroLoss,
    MagLoss,
    /// Radio link lost: the receiver keeps sending frames holding the last
    /// sticks, with `frame_lost` set, and `failsafe` set as well after
    /// `LinkConfig::rc_failsafe_delay_us`.
    RcLoss,
    /// The receiver sends nothing (unplugged).
    RcSilent,
    /// Ground-station heartbeats stop.
    GcsLoss,
    /// Brain heartbeats and tier-2 setpoints stop.
    BrainCrash,
    /// Brain heartbeats continue with a frozen progress counter; setpoints
    /// stop.
    BrainHang,
    /// The world magnetic field the magnetometer sees is rotated about Down
    /// by `angle_rad` (positive turns the field's north towards east, a
    /// right-handed turn about Down). The angle rises linearly over `ramp_s`
    /// from the start of the window and, when the window has an end, falls
    /// linearly to zero over the last `ramp_s` of it (0 = a step). Rotations
    /// that overlap add.
    MagRotate {
        angle_rad: f64,
        ramp_s: f64,
    },
    /// `north_mps` and `east_mps` are added to the velocity the GPS receiver
    /// reports (measurements taken in the window); the position is not
    /// changed. Offsets that overlap add.
    GpsVelOffset {
        north_mps: f64,
        east_mps: f64,
    },
    /// Extra white noise of `sigma_mps` per horizontal axis on the reported
    /// GPS velocity (measurements taken in the window), drawn from a stream
    /// of its own. While it is active the receiver reports `s_acc_mps` as
    /// its speed accuracy; `None` = it goes on reporting its normal figure.
    GpsVelNoise {
        sigma_mps: f64,
        s_acc_mps: Option<f64>,
    },
}

/// A fault active from `start_s` until `end_s`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Fault {
    pub kind: FaultKind,
    /// Simulation time the fault begins, s.
    pub start_s: f64,
    /// Simulation time it ends, s. `None` = until the end of the run.
    pub end_s: Option<f64>,
}

/// Seconds to whole microseconds, rounded; negative and NaN become 0.
fn secs_to_us(s: f64) -> u64 {
    if s.is_finite() && s > 0.0 {
        (s * 1e6).round() as u64
    } else {
        0
    }
}

impl Fault {
    /// Start time, µs.
    pub fn start_us(&self) -> u64 {
        secs_to_us(self.start_s)
    }

    /// `start ≤ t < end`. A NaN start never starts; a NaN end never ends.
    pub fn active(&self, t: Micros) -> bool {
        if self.start_s.is_nan() || t.0 < self.start_us() {
            return false;
        }
        match self.end_s {
            Some(e) if !e.is_nan() => t.0 < secs_to_us(e),
            _ => true,
        }
    }
}

impl Fault {
    /// The share of a [`FaultKind::MagRotate`]'s angle in force at `t`: 0
    /// outside the window, 1 for a step, and for a ramp the smaller of the
    /// rise since the start and (when the window has an end) the fall to it.
    /// Times are whole microseconds, as `active` reads them.
    pub fn ramp_weight(&self, t: Micros, ramp_s: f64) -> f64 {
        if !self.active(t) {
            return 0.0;
        }
        let ramp_us = secs_to_us(ramp_s);
        if ramp_us == 0 {
            return 1.0;
        }
        let ramp = ramp_us as f64;
        let rise = (t.0 - self.start_us()) as f64 / ramp;
        let fall = match self.end_s {
            Some(e) if !e.is_nan() => (secs_to_us(e) - t.0) as f64 / ramp,
            _ => f64::INFINITY,
        };
        rise.min(fall).min(1.0)
    }
}

/// The active faults at `t` whose kind satisfies `pred`.
pub(crate) fn active_at(
    faults: &[Fault],
    t: Micros,
    pred: fn(&FaultKind) -> bool,
) -> impl Iterator<Item = &Fault> + '_ {
    faults.iter().filter(move |f| f.active(t) && pred(&f.kind))
}

/// Which kinds of fault are active at one instant: what an autopilot behind
/// a bridge needs to translate a fault into its own terms (a GPS loss into
/// its own sensor simulation's parameter, say). Built from the fault list by
/// [`ActiveFaults::at`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ActiveFaults {
    pub gps_loss: bool,
    pub imu_loss: bool,
    pub baro_loss: bool,
    pub mag_loss: bool,
    pub rc_loss: bool,
    pub rc_silent: bool,
    pub gcs_loss: bool,
    pub brain_crash: bool,
    pub brain_hang: bool,
    /// Bit `i` is set while rotor `i` (below 32) has a motor loss active.
    pub motor_loss: u32,
}

impl ActiveFaults {
    /// The kinds active at `t`.
    pub fn at(faults: &[Fault], t: Micros) -> Self {
        let mut a = Self::default();
        for f in faults.iter().filter(|f| f.active(t)) {
            match f.kind {
                FaultKind::MotorLoss { rotor, .. } => {
                    if let Some(bit) = u32::try_from(rotor).ok().and_then(|r| 1u32.checked_shl(r)) {
                        a.motor_loss |= bit;
                    }
                }
                FaultKind::GpsLoss => a.gps_loss = true,
                FaultKind::ImuLoss => a.imu_loss = true,
                FaultKind::BaroLoss => a.baro_loss = true,
                FaultKind::MagLoss => a.mag_loss = true,
                FaultKind::RcLoss => a.rc_loss = true,
                FaultKind::RcSilent => a.rc_silent = true,
                FaultKind::GcsLoss => a.gcs_loss = true,
                FaultKind::BrainCrash => a.brain_crash = true,
                FaultKind::BrainHang => a.brain_hang = true,
                // Sensor value faults act inside the sensor models, which the
                // autopilot behind a bridge does not see.
                FaultKind::MagRotate { .. }
                | FaultKind::GpsVelOffset { .. }
                | FaultKind::GpsVelNoise { .. } => {}
            }
        }
        a
    }

    /// True when no fault is active.
    pub fn none(&self) -> bool {
        *self == Self::default()
    }
}

/// Which links exist to the vehicle, and the receiver's behaviour.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LinkConfig {
    /// A ground station is connected (heartbeat every 1 s).
    pub gcs: bool,
    /// A brain is connected (heartbeat every 200 ms, progress counter +1
    /// each).
    pub brain: bool,
    /// Delay from the start of an RC loss to the receiver's failsafe flag, µs.
    pub rc_failsafe_delay_us: u64,
}

impl Default for LinkConfig {
    /// No ground station, a brain, 1 s receiver failsafe delay.
    fn default() -> Self {
        Self {
            gcs: false,
            brain: true,
            rc_failsafe_delay_us: 1_000_000,
        }
    }
}

impl LinkConfig {
    /// Both heartbeat periods must be whole multiples of the base tick.
    pub fn validate(&self, base_us: u64) -> Result<(), SimError> {
        let whole = |p: u64| base_us != 0 && p.is_multiple_of(base_us);
        if whole(GCS_HEARTBEAT_PERIOD_US) && whole(BRAIN_HEARTBEAT_PERIOD_US) {
            Ok(())
        } else {
            Err(SimError::TimeBase(base_us))
        }
    }
}

/// Refuses faults that could never act: a window that is not finite or ends
/// at or before its start (`end_s: None` is a permanent fault), a motor loss on
/// a rotor beyond the airframe's `rotors` or with a scale outside 0 to 1, and a
/// ground-station or brain fault
/// when `links` has no such link. Such a fault would silently fly undisturbed.
pub fn validate_faults(
    faults: &[Fault],
    rotors: usize,
    links: &LinkConfig,
) -> Result<(), SimError> {
    for (index, f) in faults.iter().enumerate() {
        let refuse = |reason: String| Err(SimError::Fault { index, reason });
        if !f.start_s.is_finite() {
            return refuse(format!("start {} s is not finite", f.start_s));
        }
        if let Some(end) = f.end_s
            && !(end.is_finite() && end > f.start_s)
        {
            return refuse(format!(
                "window {} s to {end} s ends before it starts (or is not finite)",
                f.start_s
            ));
        }
        match f.kind {
            FaultKind::MotorLoss { rotor, .. } if rotor >= rotors => {
                return refuse(format!(
                    "motor loss on rotor {rotor}, but the airframe has {rotors} rotors"
                ));
            }
            FaultKind::MotorLoss { scale, .. } if !(0.0..=1.0).contains(&scale) => {
                return refuse(format!(
                    "motor loss scale {scale} is not a fraction between 0 and 1"
                ));
            }
            FaultKind::MagRotate { angle_rad, ramp_s } => {
                if !angle_rad.is_finite() {
                    return refuse(format!("mag-rotate angle_rad {angle_rad} is not finite"));
                }
                if !(ramp_s.is_finite() && ramp_s >= 0.0) {
                    return refuse(format!(
                        "mag-rotate ramp_s {ramp_s} is not a finite time of at least 0 s"
                    ));
                }
                if let Some(end) = f.end_s
                    && 2.0 * ramp_s > end - f.start_s
                {
                    return refuse(format!(
                        "mag-rotate ramp_s {ramp_s} s is more than half of the window {} s to {end} s",
                        f.start_s
                    ));
                }
            }
            FaultKind::GpsVelOffset {
                north_mps,
                east_mps,
            } if !(north_mps.is_finite() && east_mps.is_finite()) => {
                return refuse(format!(
                    "gps-vel-offset north_mps {north_mps} and east_mps {east_mps} must be finite"
                ));
            }
            FaultKind::GpsVelNoise {
                sigma_mps,
                s_acc_mps,
            } => {
                if !(sigma_mps.is_finite() && sigma_mps >= 0.0) {
                    return refuse(format!(
                        "gps-vel-noise sigma_mps {sigma_mps} is not a finite figure of at least 0"
                    ));
                }
                if let Some(a) = s_acc_mps
                    && !(a.is_finite() && a >= 0.0)
                {
                    return refuse(format!(
                        "gps-vel-noise s_acc_mps {a} is not a finite figure of at least 0"
                    ));
                }
            }
            FaultKind::GcsLoss if !links.gcs => {
                return refuse(
                    "ground-station loss, but the options have no ground station".into(),
                );
            }
            FaultKind::BrainCrash | FaultKind::BrainHang if !links.brain => {
                return refuse(format!("{:?}, but the options have no brain link", f.kind));
            }
            _ => {}
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn f(start: f64, end: Option<f64>) -> Fault {
        Fault {
            kind: FaultKind::GpsLoss,
            start_s: start,
            end_s: end,
        }
    }

    #[test]
    fn active_window_is_half_open() {
        let x = f(1.0, Some(2.0));
        assert!(!x.active(Micros(999_999)));
        assert!(x.active(Micros(1_000_000)));
        assert!(x.active(Micros(1_999_999)));
        assert!(!x.active(Micros(2_000_000)));
        assert!(f(0.5, None).active(Micros(u64::MAX / 2)));
    }

    #[test]
    fn active_faults_follow_the_windows() {
        let faults = [
            f(1.0, Some(2.0)),
            Fault {
                kind: FaultKind::MotorLoss {
                    rotor: 2,
                    scale: 0.0,
                },
                start_s: 1.5,
                end_s: None,
            },
        ];
        assert!(ActiveFaults::at(&faults, Micros(500_000)).none());
        let a = ActiveFaults::at(&faults, Micros(1_000_000));
        assert!(a.gps_loss && a.motor_loss == 0);
        let a = ActiveFaults::at(&faults, Micros(1_600_000));
        assert!(a.gps_loss && a.motor_loss == 0b100);
        let a = ActiveFaults::at(&faults, Micros(2_000_000));
        assert!(!a.gps_loss && a.motor_loss == 0b100);
        assert!(ActiveFaults::at(&[], Micros(0)).none());
    }

    fn mag(angle_rad: f64, ramp_s: f64, start_s: f64, end_s: Option<f64>) -> Fault {
        Fault {
            kind: FaultKind::MagRotate { angle_rad, ramp_s },
            start_s,
            end_s,
        }
    }

    #[test]
    fn the_ramp_weight_is_zero_outside_one_for_a_step_and_triangular_inside() {
        let step = mag(1.0, 0.0, 1.0, Some(2.0));
        assert_eq!(step.ramp_weight(Micros(999_999), 0.0), 0.0);
        assert_eq!(step.ramp_weight(Micros(1_000_000), 0.0), 1.0);
        assert_eq!(step.ramp_weight(Micros(1_999_999), 0.0), 1.0);
        assert_eq!(step.ramp_weight(Micros(2_000_000), 0.0), 0.0);
        let ramp = mag(1.0, 1.0, 10.0, Some(14.0));
        let w = |us| ramp.ramp_weight(Micros(us), 1.0);
        assert_eq!(w(9_999_999), 0.0);
        assert_eq!(w(10_000_000), 0.0);
        assert_eq!(w(10_250_000), 0.25);
        assert_eq!(w(11_000_000), 1.0);
        assert_eq!(w(12_000_000), 1.0);
        assert_eq!(w(13_500_000), 0.5);
        assert_eq!(w(13_999_000), 0.001);
        assert_eq!(w(14_000_000), 0.0);
        // No end: rises and stays.
        let open = mag(1.0, 2.0, 1.0, None);
        assert_eq!(open.ramp_weight(Micros(2_000_000), 2.0), 0.5);
        assert_eq!(open.ramp_weight(Micros(3_000_000), 2.0), 1.0);
        assert_eq!(open.ramp_weight(Micros(86_000_000_000), 2.0), 1.0);
        // A window of exactly twice the ramp peaks at its middle.
        let tri = mag(1.0, 1.0, 0.0, Some(2.0));
        assert_eq!(tri.ramp_weight(Micros(1_000_000), 1.0), 1.0);
        assert_eq!(tri.ramp_weight(Micros(500_000), 1.0), 0.5);
        assert_eq!(tri.ramp_weight(Micros(1_500_000), 1.0), 0.5);
    }

    #[test]
    fn the_new_faults_are_refused_when_a_field_is_not_acceptable() {
        let links = LinkConfig::default();
        let bad = |kind: FaultKind, end_s: Option<f64>, needle: &str| {
            let f = Fault {
                kind,
                start_s: 1.0,
                end_s,
            };
            match validate_faults(&[f], 4, &links) {
                Err(SimError::Fault { index: 0, reason }) => {
                    assert!(reason.contains(needle), "{reason}")
                }
                other => panic!("{kind:?}: {other:?}"),
            }
        };
        let rot = |angle_rad, ramp_s| FaultKind::MagRotate { angle_rad, ramp_s };
        bad(rot(f64::NAN, 0.0), None, "angle_rad");
        bad(rot(f64::INFINITY, 0.0), None, "angle_rad");
        bad(rot(1.0, -1.0), None, "ramp_s");
        bad(rot(1.0, f64::NAN), None, "ramp_s");
        bad(rot(1.0, f64::INFINITY), None, "ramp_s");
        // A ramp that does not fit twice into the window.
        bad(rot(1.0, 2.0), Some(4.9), "ramp_s");
        let off = |north_mps, east_mps| FaultKind::GpsVelOffset {
            north_mps,
            east_mps,
        };
        bad(off(f64::NAN, 0.0), None, "north_mps");
        bad(off(0.0, f64::INFINITY), None, "east_mps");
        let noise = |sigma_mps, s_acc_mps| FaultKind::GpsVelNoise {
            sigma_mps,
            s_acc_mps,
        };
        bad(noise(-0.1, None), None, "sigma_mps");
        bad(noise(f64::NAN, None), None, "sigma_mps");
        bad(noise(1.0, Some(-1.0)), None, "s_acc_mps");
        bad(noise(1.0, Some(f64::INFINITY)), None, "s_acc_mps");
        // The acceptable ones.
        let ok = |kind: FaultKind, end_s| {
            let f = Fault {
                kind,
                start_s: 1.0,
                end_s,
            };
            assert_eq!(validate_faults(&[f], 4, &links), Ok(()), "{kind:?}");
        };
        ok(rot(-3.0, 0.0), Some(2.0));
        ok(rot(1.0, 2.0), Some(5.0));
        ok(rot(1.0, 60.0), None);
        ok(off(-5.0, 2.0), Some(2.0));
        ok(noise(0.0, Some(0.0)), Some(2.0));
        ok(noise(2.0, None), None);
    }

    #[test]
    fn the_new_faults_set_no_flag_the_bridge_reads() {
        let faults = [
            mag(1.0, 0.0, 0.0, None),
            Fault {
                kind: FaultKind::GpsVelOffset {
                    north_mps: 1.0,
                    east_mps: 0.0,
                },
                start_s: 0.0,
                end_s: None,
            },
        ];
        assert!(ActiveFaults::at(&faults, Micros(1_000_000)).none());
    }

    #[test]
    fn bad_heartbeat_period_rejected() {
        let l = LinkConfig::default();
        assert!(l.validate(1_000).is_ok());
        assert!(l.validate(0).is_err());
        assert_eq!(l.validate(300_000), Err(SimError::TimeBase(300_000)));
        assert_eq!(l.validate(400_000), Err(SimError::TimeBase(400_000)));
    }
}
