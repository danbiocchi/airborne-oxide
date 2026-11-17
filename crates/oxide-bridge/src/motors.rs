// SPDX-License-Identifier: AGPL-3.0-or-later
//! Motor pulses to normalised motor commands (ADR-0006 §5).
//!
//! With `A = (pwm - 1000) / 1000`, the command is
//! `u = clamp((A - spin_min) / (spin_max - spin_min), 0, 1)`; a pulse below
//! 1000 µs (including the 0 of an unused channel) is command 0. Output `k`
//! drives our rotor `k`, for `k` in 0 to 3.

use core::fmt;

/// ArduCopter's default `MOT_SPIN_MIN`.
pub const DEFAULT_SPIN_MIN: f32 = 0.15;
/// ArduCopter's default `MOT_SPIN_MAX`.
pub const DEFAULT_SPIN_MAX: f32 = 0.95;

/// The command for one pulse, 0 to 1. Not checked: use [`MotorMap`], whose
/// constructor refuses a `spin_max` that is not above `spin_min`.
pub fn command(pwm: u16, spin_min: f32, spin_max: f32) -> f32 {
    if pwm < 1000 {
        return 0.0;
    }
    let a = f32::from(pwm - 1000) / 1000.0;
    ((a - spin_min) / (spin_max - spin_min)).clamp(0.0, 1.0)
}

/// The spin range is not usable.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MotorError {
    /// `spin_min` or `spin_max` is not a number.
    NotFinite { spin_min: f32, spin_max: f32 },
    /// `spin_max` is not above `spin_min`.
    EmptyRange { spin_min: f32, spin_max: f32 },
    /// A bound is outside 0 to 1 (`spin_min` below 0, `spin_max` above 1).
    OutOfRange { spin_min: f32, spin_max: f32 },
}

impl fmt::Display for MotorError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match *self {
            Self::NotFinite { spin_min, spin_max } => {
                write!(
                    f,
                    "spin_min {spin_min} and spin_max {spin_max} must be numbers"
                )
            }
            Self::EmptyRange { spin_min, spin_max } => {
                write!(f, "spin_max {spin_max} must be above spin_min {spin_min}")
            }
            Self::OutOfRange { spin_min, spin_max } => write!(
                f,
                "spin_min {spin_min} and spin_max {spin_max} must lie within 0 to 1"
            ),
        }
    }
}

impl std::error::Error for MotorError {}

/// The checked spin range and the map of the four outputs.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorMap {
    spin_min: f32,
    spin_max: f32,
}

impl MotorMap {
    /// Refuses a range that is empty, reversed, not a number or outside 0 to 1.
    pub fn new(spin_min: f32, spin_max: f32) -> Result<Self, MotorError> {
        if !spin_min.is_finite() || !spin_max.is_finite() {
            return Err(MotorError::NotFinite { spin_min, spin_max });
        }
        if spin_min < 0.0 || spin_max > 1.0 {
            return Err(MotorError::OutOfRange { spin_min, spin_max });
        }
        if spin_max <= spin_min {
            return Err(MotorError::EmptyRange { spin_min, spin_max });
        }
        Ok(Self { spin_min, spin_max })
    }

    pub fn spin_min(&self) -> f32 {
        self.spin_min
    }
    pub fn spin_max(&self) -> f32 {
        self.spin_max
    }

    /// The commands for rotors 0 to 3 from outputs 0 to 3 (the first four
    /// channels of a packet; fewer than four are read as 0).
    pub fn rotors(&self, pwm: &[u16]) -> [f32; 4] {
        let mut out = [0.0; 4];
        for (u, &p) in out.iter_mut().zip(pwm) {
            *u = command(p, self.spin_min, self.spin_max);
        }
        out
    }
}

impl Default for MotorMap {
    fn default() -> Self {
        Self {
            spin_min: DEFAULT_SPIN_MIN,
            spin_max: DEFAULT_SPIN_MAX,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MIN: f32 = 0.15;
    const MAX: f32 = 0.95;

    #[test]
    fn the_key_pulses_have_their_commands() {
        assert_eq!(command(0, MIN, MAX), 0.0, "an unused channel");
        assert_eq!(command(999, MIN, MAX), 0.0);
        assert_eq!(command(1000, MIN, MAX), 0.0, "disarmed");
        assert_eq!(command(1150, MIN, MAX), 0.0, "the pulse at spin_min");
        assert!((command(1550, MIN, MAX) - 0.5).abs() < 1e-6, "mid-range");
        assert_eq!(command(1950, MIN, MAX), 1.0, "the pulse at spin_max");
        assert_eq!(command(2000, MIN, MAX), 1.0);
        assert_eq!(command(65_535, MIN, MAX), 1.0);
        // A quarter of the way: A = 0.35, (0.35 - 0.15) / 0.8 = 0.25.
        assert!((command(1350, MIN, MAX) - 0.25).abs() < 1e-6);
    }

    #[test]
    fn the_command_never_falls_as_the_pulse_grows_and_stays_in_range() {
        let mut last = 0.0f32;
        for pwm in 0..=2200u16 {
            let u = command(pwm, MIN, MAX);
            assert!((0.0..=1.0).contains(&u), "pwm {pwm}: {u}");
            assert!(u >= last, "pwm {pwm}: {u} after {last}");
            last = u;
        }
        assert_eq!(last, 1.0);
    }

    #[test]
    fn another_range_scales_the_same_way() {
        // spin 0.0 to 1.0: the command is (pwm - 1000) / 1000.
        assert!((command(1250, 0.0, 1.0) - 0.25).abs() < 1e-6);
        // spin_min 0.3: 1300 is the bottom.
        assert_eq!(command(1300, 0.3, 0.9), 0.0);
        assert_eq!(command(1900, 0.3, 0.9), 1.0);
    }

    #[test]
    fn a_range_that_is_not_usable_is_refused_when_the_map_is_built() {
        assert!(MotorMap::new(0.15, 0.95).is_ok());
        assert!(MotorMap::new(0.0, 1.0).is_ok());
        assert!(matches!(
            MotorMap::new(0.5, 0.5),
            Err(MotorError::EmptyRange { .. })
        ));
        assert!(matches!(
            MotorMap::new(0.9, 0.2),
            Err(MotorError::EmptyRange { .. })
        ));
        assert!(matches!(
            MotorMap::new(f32::NAN, 0.9),
            Err(MotorError::NotFinite { .. })
        ));
        assert!(matches!(
            MotorMap::new(0.1, f32::INFINITY),
            Err(MotorError::NotFinite { .. })
        ));
        assert!(matches!(
            MotorMap::new(-0.1, 0.9),
            Err(MotorError::OutOfRange { .. })
        ));
        assert!(matches!(
            MotorMap::new(0.1, 1.1),
            Err(MotorError::OutOfRange { .. })
        ));
        let e = MotorMap::new(0.5, 0.4).expect_err("empty");
        assert!(
            e.to_string().contains("0.4") && e.to_string().contains("0.5"),
            "{e}"
        );
    }

    #[test]
    fn output_k_drives_rotor_k() {
        let m = MotorMap::new(0.0, 1.0).expect("range");
        let r = m.rotors(&[1000, 1250, 1500, 1750, 2000, 2000]);
        assert_eq!(r.len(), 4);
        for (k, want) in [0.0, 0.25, 0.5, 0.75].into_iter().enumerate() {
            assert!((r[k] - want).abs() < 1e-6, "rotor {k}: {}", r[k]);
        }
        assert_eq!(MotorMap::default().spin_min(), 0.15);
        assert_eq!(MotorMap::default().spin_max(), 0.95);
        // Fewer than four pulses: the missing rotors read 0.
        assert_eq!(m.rotors(&[2000]), [1.0, 0.0, 0.0, 0.0]);
    }
}
