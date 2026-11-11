// SPDX-License-Identifier: AGPL-3.0-or-later
//! Airframes are data files, not code (DESIGN §6).

use oxide_core::config::{
    EstimatorConfig, FailsafeParams, Gains, Limits, Rotor, Spin, VehicleConfig,
};
use oxide_core::math::Vec3;
use serde::Deserialize;

use crate::SimError;

/// The reference 5" quad shipped in `airframes/`.
pub const REFERENCE_QUAD_TOML: &str = include_str!("../../../airframes/quad-x-5in.toml");

#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct Airframe {
    pub name: String,
    pub body: Body,
    pub propulsion: Propulsion,
    #[serde(rename = "rotor")]
    pub rotors: Vec<RotorSpec>,
    pub battery: BatterySpec,
    /// Optional `[params]` table of `NAME = value` (ADR-0004), in file order.
    /// Applied before any command-line override.
    #[serde(default, deserialize_with = "ordered_params")]
    pub params: Vec<(String, f64)>,
}

/// A `[params]` table as `(name, value)` pairs in the order written, each
/// value an integer or a float.
pub fn ordered_params<'de, D: serde::Deserializer<'de>>(
    d: D,
) -> Result<Vec<(String, f64)>, D::Error> {
    use serde::de::{MapAccess, Visitor};
    struct Num(f64);
    impl<'de> Deserialize<'de> for Num {
        fn deserialize<D: serde::Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
            struct V;
            impl Visitor<'_> for V {
                type Value = Num;
                fn expecting(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
                    f.write_str("a number")
                }
                fn visit_f64<E>(self, v: f64) -> Result<Num, E> {
                    Ok(Num(v))
                }
                #[allow(clippy::cast_precision_loss)]
                fn visit_i64<E>(self, v: i64) -> Result<Num, E> {
                    Ok(Num(v as f64))
                }
                #[allow(clippy::cast_precision_loss)]
                fn visit_u64<E>(self, v: u64) -> Result<Num, E> {
                    Ok(Num(v as f64))
                }
            }
            d.deserialize_any(V)
        }
    }
    struct M;
    impl<'de> Visitor<'de> for M {
        type Value = Vec<(String, f64)>;
        fn expecting(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            f.write_str("a table of NAME = number")
        }
        fn visit_map<A: MapAccess<'de>>(self, mut m: A) -> Result<Self::Value, A::Error> {
            let mut out = Vec::new();
            while let Some((k, Num(v))) = m.next_entry::<String, Num>()? {
                out.push((k, v));
            }
            Ok(out)
        }
    }
    d.deserialize_map(M)
}

#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct Body {
    pub mass_kg: f64,
    pub inertia_kg_m2: [f64; 3],
    pub drag_area_m2: [f64; 3],
}

#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct Propulsion {
    pub max_thrust_n: f64,
    pub max_rpm: f64,
    pub torque_coeff_m: f64,
    pub motor_time_constant_s: f64,
    pub thrust_expo: f64,
    pub efficiency: f64,
    pub idle_power_w: f64,
    /// Rotor drag, N per (m/s) per unit of summed normalised rotor speed.
    #[serde(default)]
    pub rotor_drag_coeff: f64,
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum SpinSpec {
    Ccw,
    Cw,
}

#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct RotorSpec {
    pub position_m: [f64; 2],
    pub spin: SpinSpec,
}

#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct BatterySpec {
    pub cells: u32,
    pub capacity_mah: f64,
    pub internal_resistance_ohm: f64,
    pub nominal_voltage_v: f64,
    pub initial_state_of_charge: f64,
}

impl Airframe {
    pub fn from_toml(text: &str) -> Result<Self, SimError> {
        let a: Self = toml::from_str(text).map_err(|e| SimError::Airframe(e.to_string()))?;
        a.check()?;
        Ok(a)
    }

    pub fn reference_quad() -> Result<Self, SimError> {
        Self::from_toml(REFERENCE_QUAD_TOML)
    }

    fn check(&self) -> Result<(), SimError> {
        let positive = |v: f64| v.is_finite() && v > 0.0;
        let b = &self.body;
        let p = &self.propulsion;
        let bat = &self.battery;
        let ok = positive(b.mass_kg)
            && b.inertia_kg_m2.iter().all(|v| positive(*v))
            && b.drag_area_m2.iter().all(|v| v.is_finite() && *v >= 0.0)
            && positive(p.max_thrust_n)
            && positive(p.max_rpm)
            && p.torque_coeff_m.is_finite()
            && positive(p.motor_time_constant_s)
            && (0.0..=1.0).contains(&p.thrust_expo)
            && p.efficiency > 0.0
            && p.efficiency <= 1.0
            && p.idle_power_w.is_finite()
            && p.idle_power_w >= 0.0
            && p.rotor_drag_coeff.is_finite()
            && p.rotor_drag_coeff >= 0.0
            && bat.cells > 0
            && positive(bat.capacity_mah)
            && bat.internal_resistance_ohm.is_finite()
            && bat.internal_resistance_ohm >= 0.0
            && positive(bat.nominal_voltage_v)
            && (0.0..=1.0).contains(&bat.initial_state_of_charge)
            && self
                .rotors
                .iter()
                .all(|r| r.position_m.iter().all(|v| v.is_finite()));
        if ok {
            Ok(())
        } else {
            Err(SimError::Airframe(format!(
                "{}: a value is out of range",
                self.name
            )))
        }
    }

    /// Rotors as a fixed-size array; fails if the count isn't `N`.
    pub fn rotor_array<const N: usize>(&self) -> Result<[RotorSpec; N], SimError> {
        self.rotors.clone().try_into().map_err(|v: Vec<RotorSpec>| {
            SimError::Airframe(format!(
                "{}: expected {N} rotors, found {}",
                self.name,
                v.len()
            ))
        })
    }

    /// The configuration the flight core is given. In M1a it matches the
    /// simulated vehicle exactly; later the core gets *believed* values and
    /// the simulator keeps the truth.
    #[allow(clippy::cast_possible_truncation)]
    pub fn core_config<const N: usize>(&self) -> Result<VehicleConfig<N>, SimError> {
        let specs = self.rotor_array::<N>()?;
        let p = &self.propulsion;
        let rotors = specs.map(|r| Rotor {
            x: r.position_m[0] as f32,
            y: r.position_m[1] as f32,
            spin: match r.spin {
                SpinSpec::Ccw => Spin::Ccw,
                SpinSpec::Cw => Spin::Cw,
            },
            max_thrust_n: p.max_thrust_n as f32,
            torque_coeff_m: p.torque_coeff_m as f32,
        });
        let i = self.body.inertia_kg_m2;
        Ok(VehicleConfig {
            mass_kg: self.body.mass_kg as f32,
            inertia: Vec3::new(i[0], i[1], i[2]).cast(),
            rotors,
            thrust_expo: p.thrust_expo as f32,
            nominal_voltage_v: self.battery.nominal_voltage_v as f32,
            gains: Gains::default(),
            limits: Limits::default(),
            estimator: EstimatorConfig::default(),
            failsafe: FailsafeParams::default(),
            battery_cells: u8::try_from(self.battery.cells).unwrap_or(u8::MAX),
            battery_resistance_ohm: self.battery.internal_resistance_ohm as f32,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reference_quad_loads() {
        let a = Airframe::reference_quad().expect("reference airframe parses");
        assert_eq!(a.rotors.len(), 4);
        assert!(a.core_config::<4>().is_ok());
        assert!(a.core_config::<6>().is_err());
    }

    #[test]
    fn unknown_fields_are_rejected() {
        let bad = REFERENCE_QUAD_TOML.replace("[body]", "[body]\nmass_typo = 1.0");
        assert!(Airframe::from_toml(&bad).is_err());
    }
}
