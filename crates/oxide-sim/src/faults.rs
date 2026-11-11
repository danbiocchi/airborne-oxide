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

/// The active faults at `t` whose kind satisfies `pred`.
pub(crate) fn active_at(
    faults: &[Fault],
    t: Micros,
    pred: fn(&FaultKind) -> bool,
) -> impl Iterator<Item = &Fault> + '_ {
    faults.iter().filter(move |f| f.active(t) && pred(&f.kind))
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
    fn bad_heartbeat_period_rejected() {
        let l = LinkConfig::default();
        assert!(l.validate(1_000).is_ok());
        assert!(l.validate(0).is_err());
        assert_eq!(l.validate(300_000), Err(SimError::TimeBase(300_000)));
        assert_eq!(l.validate(400_000), Err(SimError::TimeBase(400_000)));
    }
}
