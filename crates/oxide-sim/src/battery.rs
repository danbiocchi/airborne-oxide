// SPDX-License-Identifier: AGPL-3.0-or-later
//! Battery: open-circuit voltage versus state of charge, plus internal
//! resistance, so voltage sags under load and available thrust drops
//! (DESIGN §5.4 S1).

use crate::airframe::BatterySpec;

/// LiPo open-circuit voltage per cell versus state of charge.
const OCV_TABLE: [(f64, f64); 9] = [
    (0.00, 3.30),
    (0.05, 3.50),
    (0.10, 3.60),
    (0.20, 3.70),
    (0.40, 3.78),
    (0.60, 3.87),
    (0.80, 3.98),
    (0.95, 4.12),
    (1.00, 4.20),
];

/// Open-circuit voltage of one cell at state of charge `soc`.
pub fn cell_ocv(soc: f64) -> f64 {
    let s = soc.clamp(0.0, 1.0);
    let mut prev = OCV_TABLE[0];
    for &(x, v) in &OCV_TABLE[1..] {
        if s <= x {
            let (x0, v0) = prev;
            let t = if x > x0 { (s - x0) / (x - x0) } else { 0.0 };
            return v0 + t * (v - v0);
        }
        prev = (x, v);
    }
    prev.1
}

#[derive(Clone, Debug, PartialEq)]
pub struct Battery {
    cells: f64,
    capacity_coulomb: f64,
    resistance_ohm: f64,
    pub state_of_charge: f64,
    pub voltage_v: f64,
    pub current_a: f64,
}

impl Battery {
    pub fn new(spec: &BatterySpec) -> Self {
        let cells = f64::from(spec.cells);
        let soc = spec.initial_state_of_charge;
        Self {
            cells,
            capacity_coulomb: spec.capacity_mah * 3.6,
            resistance_ohm: spec.internal_resistance_ohm,
            state_of_charge: soc,
            voltage_v: cells * cell_ocv(soc),
            current_a: 0.0,
        }
    }

    /// Draws `power_w` for `dt` seconds. Explicit update: the current uses
    /// the voltage from the previous step.
    pub fn draw(&mut self, power_w: f64, dt: f64) {
        let v_prev = self.voltage_v.max(0.1 * self.cells);
        self.current_a = (power_w / v_prev).max(0.0);
        self.state_of_charge =
            (self.state_of_charge - self.current_a * dt / self.capacity_coulomb).max(0.0);
        self.voltage_v = (self.cells * cell_ocv(self.state_of_charge)
            - self.current_a * self.resistance_ohm)
            .max(0.0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ocv_endpoints_and_monotonic() {
        assert!((cell_ocv(1.0) - 4.2).abs() < 1e-12);
        assert!((cell_ocv(0.0) - 3.3).abs() < 1e-12);
        let mut last = 0.0;
        for i in 0..=100 {
            let v = cell_ocv(f64::from(i) / 100.0);
            assert!(v >= last);
            last = v;
        }
    }

    #[test]
    fn sags_under_load() {
        let spec = BatterySpec {
            cells: 6,
            capacity_mah: 1500.0,
            internal_resistance_ohm: 0.025,
            nominal_voltage_v: 22.2,
            initial_state_of_charge: 1.0,
        };
        let mut b = Battery::new(&spec);
        let v0 = b.voltage_v;
        b.draw(1000.0, 0.001);
        assert!(b.voltage_v < v0 && b.current_a > 0.0);
    }
}
