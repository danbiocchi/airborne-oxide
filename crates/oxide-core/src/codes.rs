// SPDX-License-Identifier: AGPL-3.0-or-later
//! Fixed numeric codes for everything a log stores as a number (ADR-0005).
//!
//! A log is a format: a number in it must mean the same thing next year.
//! Each code here is a literal in an explicit `match`, never `as u8` on the
//! enum, so reordering or inserting a variant cannot change a code. The
//! current values are the declaration order at the time of writing, from 0.
//! A new variant gets the next unused code; adding a variant without a code
//! does not compile (the `code` matches are exhaustive), and changing a code
//! fails the table tests below.
//!
//! [`Reasons`] and [`PrearmFlags`] already have fixed bit positions; the
//! tests pin them too. [`px4_nav_state`] and [`px4_arming_state`] translate
//! to PX4's numbers for the tools that colour a flight by them.

use crate::config::{ConfigError, EstimatorSource};
use crate::io::{Action, DisarmReason, EventKind, FlightMode, NavState};
use crate::params::ParamError;

impl FlightMode {
    /// The fixed code of this value.
    pub const fn code(self) -> u8 {
        match self {
            FlightMode::Stabilized => 0,
            FlightMode::Altitude => 1,
            FlightMode::Position => 2,
            FlightMode::Offboard => 3,
        }
    }

    /// The value with this code; `None` for a code no value has.
    pub const fn from_code(code: u8) -> Option<Self> {
        match code {
            0 => Some(FlightMode::Stabilized),
            1 => Some(FlightMode::Altitude),
            2 => Some(FlightMode::Position),
            3 => Some(FlightMode::Offboard),
            _ => None,
        }
    }
}

impl NavState {
    /// The fixed code of this value.
    pub const fn code(self) -> u8 {
        match self {
            NavState::Disarmed => 0,
            NavState::Stabilized => 1,
            NavState::Altitude => 2,
            NavState::Position => 3,
            NavState::Offboard => 4,
            NavState::Hold => 5,
            NavState::Rtl => 6,
            NavState::Land => 7,
            NavState::LandNow => 8,
            NavState::Terminated => 9,
        }
    }

    /// The value with this code; `None` for a code no value has.
    pub const fn from_code(code: u8) -> Option<Self> {
        match code {
            0 => Some(NavState::Disarmed),
            1 => Some(NavState::Stabilized),
            2 => Some(NavState::Altitude),
            3 => Some(NavState::Position),
            4 => Some(NavState::Offboard),
            5 => Some(NavState::Hold),
            6 => Some(NavState::Rtl),
            7 => Some(NavState::Land),
            8 => Some(NavState::LandNow),
            9 => Some(NavState::Terminated),
            _ => None,
        }
    }
}

impl Action {
    /// The fixed code of this value.
    pub const fn code(self) -> u8 {
        match self {
            Action::None => 0,
            Action::Hold => 1,
            Action::Rtl => 2,
            Action::Land => 3,
            Action::LandNow => 4,
            Action::Terminate => 5,
        }
    }

    /// The value with this code; `None` for a code no value has.
    pub const fn from_code(code: u8) -> Option<Self> {
        match code {
            0 => Some(Action::None),
            1 => Some(Action::Hold),
            2 => Some(Action::Rtl),
            3 => Some(Action::Land),
            4 => Some(Action::LandNow),
            5 => Some(Action::Terminate),
            _ => None,
        }
    }
}

impl DisarmReason {
    /// The fixed code of this value.
    pub const fn code(self) -> u8 {
        match self {
            DisarmReason::Switch => 0,
            DisarmReason::Kill => 1,
            DisarmReason::Touchdown => 2,
            DisarmReason::Terminated => 3,
        }
    }

    /// The value with this code; `None` for a code no value has.
    pub const fn from_code(code: u8) -> Option<Self> {
        match code {
            0 => Some(DisarmReason::Switch),
            1 => Some(DisarmReason::Kill),
            2 => Some(DisarmReason::Touchdown),
            3 => Some(DisarmReason::Terminated),
            _ => None,
        }
    }
}

impl EstimatorSource {
    /// The fixed code of this value.
    pub const fn code(self) -> u8 {
        match self {
            EstimatorSource::External => 0,
            EstimatorSource::Ekf => 1,
        }
    }

    /// The value with this code; `None` for a code no value has.
    pub const fn from_code(code: u8) -> Option<Self> {
        match code {
            0 => Some(EstimatorSource::External),
            1 => Some(EstimatorSource::Ekf),
            _ => None,
        }
    }
}

impl ConfigError {
    /// The fixed code of this value.
    pub const fn code(self) -> u8 {
        match self {
            ConfigError::BadMassProperties => 0,
            ConfigError::BadRotor => 1,
            ConfigError::UncontrollableLayout => 2,
            ConfigError::BadThrustExpo => 3,
            ConfigError::BadProfile => 4,
            ConfigError::BadEstimator => 5,
            ConfigError::BadFailsafe => 6,
            ConfigError::BadGains => 7,
            ConfigError::BadLimits => 8,
            ConfigError::CannotLift => 9,
            ConfigError::IdleTooHigh => 10,
            ConfigError::DescentLimitTooLow => 11,
            ConfigError::LossOfControlTooTight => 12,
            ConfigError::FilterCutoffTooHigh => 13,
        }
    }

    /// The value with this code; `None` for a code no value has.
    pub const fn from_code(code: u8) -> Option<Self> {
        match code {
            0 => Some(ConfigError::BadMassProperties),
            1 => Some(ConfigError::BadRotor),
            2 => Some(ConfigError::UncontrollableLayout),
            3 => Some(ConfigError::BadThrustExpo),
            4 => Some(ConfigError::BadProfile),
            5 => Some(ConfigError::BadEstimator),
            6 => Some(ConfigError::BadFailsafe),
            7 => Some(ConfigError::BadGains),
            8 => Some(ConfigError::BadLimits),
            9 => Some(ConfigError::CannotLift),
            10 => Some(ConfigError::IdleTooHigh),
            11 => Some(ConfigError::DescentLimitTooLow),
            12 => Some(ConfigError::LossOfControlTooTight),
            13 => Some(ConfigError::FilterCutoffTooHigh),
            _ => None,
        }
    }
}

impl ParamError {
    /// The fixed code of the kind of refusal. [`ParamError::Invalid`] also
    /// carries a [`ConfigError`]: its code is [`ParamError::detail_code`].
    pub const fn code(self) -> u8 {
        match self {
            ParamError::Unknown => 0,
            ParamError::WrongType => 1,
            ParamError::NotFinite => 2,
            ParamError::OutOfRange => 3,
            ParamError::Invalid(_) => 4,
            ParamError::Armed => 5,
        }
    }

    /// The code of the [`ConfigError`] an `Invalid` refusal carries, and 0
    /// for every other kind.
    pub const fn detail_code(self) -> u8 {
        match self {
            ParamError::Invalid(e) => e.code(),
            ParamError::Unknown
            | ParamError::WrongType
            | ParamError::NotFinite
            | ParamError::OutOfRange
            | ParamError::Armed => 0,
        }
    }

    /// The refusal with this code and detail code; `None` for a code no
    /// refusal has, an `Invalid` whose detail is not a [`ConfigError`] code,
    /// or a detail other than 0 on any other kind.
    pub const fn from_code(code: u8, detail: u8) -> Option<Self> {
        match (code, detail) {
            (0, 0) => Some(ParamError::Unknown),
            (1, 0) => Some(ParamError::WrongType),
            (2, 0) => Some(ParamError::NotFinite),
            (3, 0) => Some(ParamError::OutOfRange),
            (4, d) => match ConfigError::from_code(d) {
                Some(e) => Some(ParamError::Invalid(e)),
                None => None,
            },
            (5, 0) => Some(ParamError::Armed),
            _ => None,
        }
    }
}

impl EventKind {
    /// The fixed code of the variant; the payload is the log's business.
    pub const fn code(&self) -> u8 {
        match self {
            EventKind::Armed => 0,
            EventKind::Disarmed(_) => 1,
            EventKind::ArmRefused(_) => 2,
            EventKind::NavChanged { .. } => 3,
            EventKind::ModeDegraded { .. } => 4,
            EventKind::ReasonRaised(_) => 5,
            EventKind::ReasonCleared(_) => 6,
            EventKind::ActionChanged { .. } => 7,
            EventKind::PilotOverride(_) => 8,
            EventKind::Touchdown => 9,
            EventKind::HomeSet => 10,
            EventKind::ParamChanged(_) => 11,
            EventKind::ParamRefused { .. } => 12,
            EventKind::ParamLoadFault(_) => 13,
            EventKind::ParamLoadRecovered(_) => 14,
            EventKind::ParamFaultAcked => 15,
        }
    }
}

/// PX4's `vehicle_status.arming_state` for the armed flag:
/// `ARMING_STATE_DISARMED = 1`, `ARMING_STATE_ARMED = 2`.
pub const fn px4_arming_state(armed: bool) -> u8 {
    if armed { 2 } else { 1 }
}

/// PX4's `vehicle_status.nav_state` for the closest PX4 state.
///
/// The numbers are `NAVIGATION_STATE_*` of PX4's `msg/versioned/VehicleStatus.msg`
/// at PX4-Autopilot commit `99a3048735caa5a2c277d6aa8e17a12a6133db5d`, and the
/// ones Flight Review colours.
///
/// | Oxide | PX4 |
/// |---|---|
/// | `Stabilized` | `STAB` 15 |
/// | `Altitude` | `ALTCTL` 1 |
/// | `Position` | `POSCTL` 2 |
/// | `Offboard` | `OFFBOARD` 14 |
/// | `Hold` | `AUTO_LOITER` 4 |
/// | `Rtl` | `AUTO_RTL` 5 |
/// | `Land` | `AUTO_LAND` 18 |
/// | `LandNow` | `DESCEND` 12 |
/// | `Terminated` | `TERMINATION` 13 |
/// | `Disarmed` | the state of the pilot's `mode`, as PX4 shows the selected mode while disarmed |
pub const fn px4_nav_state(nav: NavState, mode: FlightMode) -> u8 {
    match nav {
        NavState::Disarmed => match mode {
            FlightMode::Stabilized => 15,
            FlightMode::Altitude => 1,
            FlightMode::Position => 2,
            FlightMode::Offboard => 14,
        },
        NavState::Stabilized => 15,
        NavState::Altitude => 1,
        NavState::Position => 2,
        NavState::Offboard => 14,
        NavState::Hold => 4,
        NavState::Rtl => 5,
        NavState::Land => 18,
        NavState::LandNow => 12,
        NavState::Terminated => 13,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::io::{PrearmFlags, Reasons};

    /// Every value of a type with its pinned code. A new variant must be
    /// added here (and to `code`, or nothing compiles).
    macro_rules! pinned {
        ($name:ident: $t:ty = [$(($v:expr, $c:literal)),* $(,)?]) => {
            #[test]
            fn $name() {
                let table: &[($t, u8)] = &[$(($v, $c)),*];
                for (v, c) in table {
                    assert_eq!(v.code(), *c, "{v:?}");
                    assert_eq!(<$t>::from_code(*c), Some(*v), "code {c}");
                }
                // Codes are distinct, and nothing else decodes.
                let mut decoded = 0;
                for c in 0..=255_u8 {
                    if let Some(v) = <$t>::from_code(c) {
                        decoded += 1;
                        assert_eq!(v.code(), c);
                        assert!(table.iter().any(|(t, k)| *t == v && *k == c));
                    }
                }
                assert_eq!(decoded, table.len());
            }
        };
    }

    pinned!(flight_mode_codes: FlightMode = [
        (FlightMode::Stabilized, 0), (FlightMode::Altitude, 1),
        (FlightMode::Position, 2), (FlightMode::Offboard, 3),
    ]);
    pinned!(nav_state_codes: NavState = [
        (NavState::Disarmed, 0), (NavState::Stabilized, 1), (NavState::Altitude, 2),
        (NavState::Position, 3), (NavState::Offboard, 4), (NavState::Hold, 5),
        (NavState::Rtl, 6), (NavState::Land, 7), (NavState::LandNow, 8),
        (NavState::Terminated, 9),
    ]);
    pinned!(action_codes: Action = [
        (Action::None, 0), (Action::Hold, 1), (Action::Rtl, 2),
        (Action::Land, 3), (Action::LandNow, 4), (Action::Terminate, 5),
    ]);
    pinned!(disarm_reason_codes: DisarmReason = [
        (DisarmReason::Switch, 0), (DisarmReason::Kill, 1),
        (DisarmReason::Touchdown, 2), (DisarmReason::Terminated, 3),
    ]);
    pinned!(estimator_source_codes: EstimatorSource = [
        (EstimatorSource::External, 0), (EstimatorSource::Ekf, 1),
    ]);
    pinned!(config_error_codes: ConfigError = [
        (ConfigError::BadMassProperties, 0), (ConfigError::BadRotor, 1),
        (ConfigError::UncontrollableLayout, 2), (ConfigError::BadThrustExpo, 3),
        (ConfigError::BadProfile, 4), (ConfigError::BadEstimator, 5),
        (ConfigError::BadFailsafe, 6), (ConfigError::BadGains, 7),
        (ConfigError::BadLimits, 8), (ConfigError::CannotLift, 9),
        (ConfigError::IdleTooHigh, 10), (ConfigError::DescentLimitTooLow, 11),
        (ConfigError::LossOfControlTooTight, 12), (ConfigError::FilterCutoffTooHigh, 13),
    ]);

    #[test]
    fn param_error_codes() {
        let table: &[(ParamError, u8, u8)] = &[
            (ParamError::Unknown, 0, 0),
            (ParamError::WrongType, 1, 0),
            (ParamError::NotFinite, 2, 0),
            (ParamError::OutOfRange, 3, 0),
            (ParamError::Armed, 5, 0),
        ];
        for (e, c, d) in table {
            assert_eq!((e.code(), e.detail_code()), (*c, *d), "{e:?}");
            assert_eq!(ParamError::from_code(*c, *d), Some(*e));
        }
        // `Invalid` carries a configuration error: code 4, detail its code.
        for d in 0..=255_u8 {
            match ConfigError::from_code(d) {
                Some(ce) => {
                    let e = ParamError::Invalid(ce);
                    assert_eq!((e.code(), e.detail_code()), (4, d));
                    assert_eq!(ParamError::from_code(4, d), Some(e));
                }
                None => assert_eq!(ParamError::from_code(4, d), None),
            }
        }
        // A detail on a kind that has none, and unknown codes, are refused.
        assert_eq!(ParamError::from_code(0, 1), None);
        assert_eq!(ParamError::from_code(5, 3), None);
        for c in 6..=255_u8 {
            assert_eq!(ParamError::from_code(c, 0), None);
        }
    }

    #[test]
    fn event_kind_codes() {
        use crate::io::{NavState as N, Reasons as R};
        use crate::params::{LoadReport, ParamId};
        let load = LoadReport::default();
        let table: &[(EventKind, u8)] = &[
            (EventKind::Armed, 0),
            (EventKind::Disarmed(DisarmReason::Kill), 1),
            (EventKind::ArmRefused(PrearmFlags::KILL), 2),
            (
                EventKind::NavChanged {
                    from: N::Hold,
                    to: N::Rtl,
                },
                3,
            ),
            (
                EventKind::ModeDegraded {
                    requested: FlightMode::Position,
                    flown: N::Altitude,
                },
                4,
            ),
            (EventKind::ReasonRaised(R::RC_LOST), 5),
            (EventKind::ReasonCleared(R::RC_LOST), 6),
            (
                EventKind::ActionChanged {
                    from: Action::None,
                    to: Action::Hold,
                },
                7,
            ),
            (EventKind::PilotOverride(R::RC_LOST), 8),
            (EventKind::Touchdown, 9),
            (EventKind::HomeSet, 10),
            (EventKind::ParamChanged(ParamId(3)), 11),
            (
                EventKind::ParamRefused {
                    id: ParamId(3),
                    why: ParamError::Armed,
                },
                12,
            ),
            (EventKind::ParamLoadFault(load), 13),
            (EventKind::ParamLoadRecovered(load), 14),
            (EventKind::ParamFaultAcked, 15),
        ];
        for (k, c) in table {
            assert_eq!(k.code(), *c, "{k:?}");
        }
        // The payload does not change the code.
        assert_eq!(EventKind::Disarmed(DisarmReason::Switch).code(), 1);
        assert_eq!(EventKind::ArmRefused(PrearmFlags::ALL).code(), 2);
    }

    #[test]
    fn failsafe_reason_bits_are_pinned() {
        let table: &[(Reasons, u16)] = &[
            (Reasons::RC_LOST, 0),
            (Reasons::NO_HUMAN_LINK, 1),
            (Reasons::BRAIN_LOST, 2),
            (Reasons::SETPOINT_STALE, 3),
            (Reasons::BATTERY_LOW, 4),
            (Reasons::BATTERY_CRITICAL, 5),
            (Reasons::GPS_LOST, 6),
            (Reasons::ESTIMATOR_UNHEALTHY, 7),
            (Reasons::IMU_FAILURE, 8),
            (Reasons::GEOFENCE, 9),
            (Reasons::LOSS_OF_CONTROL, 10),
        ];
        for (r, bit) in table {
            assert_eq!(r.0, 1_u16 << bit);
        }
        assert_eq!(Reasons::ALL.0, 0x07ff);
    }

    #[test]
    fn prearm_bits_are_pinned() {
        let table: &[(PrearmFlags, u16)] = &[
            (PrearmFlags::KILL, 0),
            (PrearmFlags::THROTTLE, 1),
            (PrearmFlags::RC, 2),
            (PrearmFlags::NOT_STILL, 3),
            (PrearmFlags::ESTIMATOR, 4),
            (PrearmFlags::POSITION, 5),
            (PrearmFlags::BATTERY, 6),
            (PrearmFlags::CONFIG, 7),
            (PrearmFlags::LINK, 8),
        ];
        for (f, bit) in table {
            assert_eq!(f.0, 1_u16 << bit);
        }
        assert_eq!(PrearmFlags::ALL.0, 0x01ff);
    }

    #[test]
    fn px4_state_numbers_are_pinned() {
        use FlightMode as M;
        use NavState as N;
        assert_eq!(px4_arming_state(false), 1);
        assert_eq!(px4_arming_state(true), 2);
        let table: &[(N, u8)] = &[
            (N::Stabilized, 15),
            (N::Altitude, 1),
            (N::Position, 2),
            (N::Offboard, 14),
            (N::Hold, 4),
            (N::Rtl, 5),
            (N::Land, 18),
            (N::LandNow, 12),
            (N::Terminated, 13),
        ];
        // Armed states do not depend on the pilot's mode.
        for (n, px4) in table {
            for m in [M::Stabilized, M::Altitude, M::Position, M::Offboard] {
                assert_eq!(px4_nav_state(*n, m), *px4, "{n:?} {m:?}");
            }
        }
        // Disarmed shows the pilot's selected mode.
        assert_eq!(px4_nav_state(N::Disarmed, M::Stabilized), 15);
        assert_eq!(px4_nav_state(N::Disarmed, M::Altitude), 1);
        assert_eq!(px4_nav_state(N::Disarmed, M::Position), 2);
        assert_eq!(px4_nav_state(N::Disarmed, M::Offboard), 14);
    }
}
