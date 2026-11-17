// SPDX-License-Identifier: AGPL-3.0-or-later
//! Our faults as ArduCopter's parameters (ADR-0006 section 6) and our battery
//! thresholds as ArduCopter's (ADR-0007 section 3).
//!
//! [`FaultTranslator`] is given the faults active at every flight tick and
//! returns the parameter writes for the faults whose state changed at this
//! tick, on and off. A write is decided from the fault list and the tick
//! alone: never from what the program said back (the echo of a write goes to
//! the audit, and nothing waits for it).
//!
//! | Our fault | Written when it starts (when it ends) |
//! |---|---|
//! | GPS lost | `SIM_GPS1_ENABLE 0` (1) |
//! | Barometer lost | `SIM_BARO_DISABLE`, `SIM_BAR2_DISABLE`, `SIM_BAR3_DISABLE`: 1 (0) |
//! | Compass lost | `SIM_MAG1_FAIL`, `SIM_MAG2_FAIL`, `SIM_MAG3_FAIL`: 1 (0) |
//! | IMU lost | `SIM_ACC_FAIL_MSK` and `SIM_GYR_FAIL_MSK`: 3, the two IMUs the program simulates (0) |
//!
//! The names and the number of IMUs were read from the pinned program (V4.7.2,
//! a test of `tests/arducopter_faults.rs` reads each one back): it has three
//! `SIM_BAR*_DISABLE`, three `SIM_MAG*_FAIL` and `SIM_IMU_COUNT` is 2.
//! The other faults need no write: a lost radio leaves the `rc` channels of
//! the reply, the ground station's and the brain's heartbeats and setpoints
//! reach the program only while they arrive, a lost motor is in our physics,
//! and our battery's voltage is in the reply.

use oxide_core::config::FailsafeParams;
use oxide_sim::faults::ActiveFaults;

/// The GPS switch: 1 on, 0 off.
pub const GPS_ENABLE: &str = "SIM_GPS1_ENABLE";
/// The barometers' switches (the program has three names, 1 disables).
pub const BARO_DISABLE: [&str; 3] = ["SIM_BARO_DISABLE", "SIM_BAR2_DISABLE", "SIM_BAR3_DISABLE"];
/// The compasses' failure switches (the program has three names, 1 fails).
pub const MAG_FAIL: [&str; 3] = ["SIM_MAG1_FAIL", "SIM_MAG2_FAIL", "SIM_MAG3_FAIL"];
/// The accelerometer failure mask: bit `i` fails IMU `i`.
pub const ACC_FAIL_MASK: &str = "SIM_ACC_FAIL_MSK";
/// The gyroscope failure mask.
pub const GYR_FAIL_MASK: &str = "SIM_GYR_FAIL_MSK";
/// The number of IMUs the program simulates (`SIM_IMU_COUNT`, read from the
/// pinned binary).
pub const IMU_COUNT: u32 = 2;
/// The failure mask that covers every IMU the program has.
pub const ALL_IMUS_MASK: u32 = (1 << IMU_COUNT) - 1;

/// One parameter write to the program.
#[derive(Clone, Debug, PartialEq)]
pub struct ParamWrite {
    pub name: &'static str,
    /// MAVLink carries a parameter as a 32-bit float.
    pub value: f32,
}

/// Turns the active faults of every tick into the parameter writes of the
/// tick at which a fault starts or ends.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct FaultTranslator {
    gps: bool,
    baro: bool,
    mag: bool,
    imu: bool,
}

impl FaultTranslator {
    /// No fault is active before the first tick.
    pub fn new() -> Self {
        Self::default()
    }

    /// The writes for the faults whose state differs from the last call's,
    /// in the order GPS, barometer, compass, IMU.
    pub fn step(&mut self, faults: &ActiveFaults) -> Vec<ParamWrite> {
        let mut out = Vec::new();
        if faults.gps_loss != self.gps {
            self.gps = faults.gps_loss;
            out.push(ParamWrite {
                name: GPS_ENABLE,
                value: if self.gps { 0.0 } else { 1.0 },
            });
        }
        if faults.baro_loss != self.baro {
            self.baro = faults.baro_loss;
            let value = if self.baro { 1.0 } else { 0.0 };
            out.extend(BARO_DISABLE.iter().map(|&name| ParamWrite { name, value }));
        }
        if faults.mag_loss != self.mag {
            self.mag = faults.mag_loss;
            let value = if self.mag { 1.0 } else { 0.0 };
            out.extend(MAG_FAIL.iter().map(|&name| ParamWrite { name, value }));
        }
        if faults.imu_loss != self.imu {
            self.imu = faults.imu_loss;
            let value = if self.imu { ALL_IMUS_MASK as f32 } else { 0.0 };
            out.extend(
                [ACC_FAIL_MASK, GYR_FAIL_MASK]
                    .iter()
                    .map(|&name| ParamWrite { name, value }),
            );
        }
        out
    }
}

/// The electrical power the reference quad draws in a hover, W (measured on
/// the truth of a hover: a test holds it against our core's own hover).
pub const HOVER_POWER_W: f64 = 193.5;

/// ArduCopter's battery thresholds for the pack voltage at which our core
/// acts: `BATT_LOW_VOLT` and `BATT_CRT_VOLT`, volts.
///
/// How our core decides (`oxide_core::monitor::BatteryMonitor`): by voltage
/// alone, never by charge. Each cell's voltage is the pack voltage plus the
/// drop across the pack's internal resistance (`V + I R`, the current clamped
/// at zero), divided by the number of cells, low-passed (`BAT_FILT_TAU`), and
/// compared with `BAT_LOW_V` and `BAT_CRIT_V` per cell; it must stay below
/// for `BAT_DEBOUNCE`. The compensated voltage is the pack's open-circuit
/// voltage, which does not sag with the load.
///
/// What ArduCopter offers: `BATT_LOW_VOLT` and `BATT_CRT_VOLT` are compared
/// with the pack voltage as it is (`BATT_FS_VOLTSRC 0`, the program's default
/// and what this file keeps), which sags under load, and the voltage must stay
/// below for `BATT_LOW_TIMER` (10 s, left at its default). So the nearest
/// honest mapping is the pack voltage the vehicle has at the moment our
/// core's compensated voltage reaches its threshold, in a hover:
/// `V = ocv - R P / V` with `ocv = cells x threshold` and `P` the hover
/// power ([`HOVER_POWER_W`]), solved for `V`. Without the sag the program
/// would act earlier in the discharge than our core by the time the pack
/// takes to lose about a quarter of a volt (measured in the flights of the
/// tests: about 0.23 V at the low threshold).
///
/// The two timers differ (ours 2 s of filter and 2 s of debounce, the
/// program's 10 s) and so does the filter: a difference of the two systems
/// that is left as it is.
pub fn battery_failsafe_params(
    failsafe: &FailsafeParams,
    cells: u8,
    resistance_ohm: f64,
    hover_power_w: f64,
) -> Vec<(String, f64)> {
    let pack = |per_cell: f32| -> f64 {
        let ocv = f64::from(cells) * f64::from(per_cell);
        // V^2 - ocv V + R P = 0, the larger root: the loaded pack.
        let disc = (ocv * ocv - 4.0 * resistance_ohm * hover_power_w).max(0.0);
        0.5 * (ocv + oxide_core::math::Real::sqrt(disc))
    };
    vec![
        ("BATT_LOW_VOLT".to_string(), pack(failsafe.battery_low_v)),
        (
            "BATT_CRT_VOLT".to_string(),
            pack(failsafe.battery_critical_v),
        ),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxide_core::schedule::F405_LEAN;
    use oxide_sim::scenario::HoverStep;
    use oxide_sim::{Airframe, SimOptions, Simulation};

    fn faults(f: impl FnOnce(&mut ActiveFaults)) -> ActiveFaults {
        let mut a = ActiveFaults::default();
        f(&mut a);
        a
    }

    fn names(w: &[ParamWrite]) -> Vec<&'static str> {
        w.iter().map(|w| w.name).collect()
    }

    #[test]
    fn nothing_is_written_while_nothing_changes() {
        let mut t = FaultTranslator::new();
        for _ in 0..5 {
            assert!(t.step(&ActiveFaults::default()).is_empty());
        }
        let on = faults(|f| f.gps_loss = true);
        assert_eq!(t.step(&on).len(), 1);
        for _ in 0..5 {
            assert!(
                t.step(&on).is_empty(),
                "a fault that goes on writes nothing"
            );
        }
    }

    #[test]
    fn gps_lost_writes_sim_gps1_enable_0_and_its_end_writes_1() {
        let mut t = FaultTranslator::new();
        let w = t.step(&faults(|f| f.gps_loss = true));
        assert_eq!(
            w,
            vec![ParamWrite {
                name: "SIM_GPS1_ENABLE",
                value: 0.0
            }]
        );
        let w = t.step(&ActiveFaults::default());
        assert_eq!(
            w,
            vec![ParamWrite {
                name: "SIM_GPS1_ENABLE",
                value: 1.0
            }]
        );
    }

    #[test]
    fn the_barometer_and_the_compass_write_every_name_they_have() {
        let mut t = FaultTranslator::new();
        let w = t.step(&faults(|f| f.baro_loss = true));
        assert_eq!(
            names(&w),
            ["SIM_BARO_DISABLE", "SIM_BAR2_DISABLE", "SIM_BAR3_DISABLE"]
        );
        assert!(w.iter().all(|w| w.value == 1.0));
        let w = t.step(&ActiveFaults::default());
        assert_eq!(w.len(), 3);
        assert!(w.iter().all(|w| w.value == 0.0));

        let w = t.step(&faults(|f| f.mag_loss = true));
        assert_eq!(
            names(&w),
            ["SIM_MAG1_FAIL", "SIM_MAG2_FAIL", "SIM_MAG3_FAIL"]
        );
        assert!(w.iter().all(|w| w.value == 1.0));
        let w = t.step(&ActiveFaults::default());
        assert_eq!(w.len(), 3);
        assert!(w.iter().all(|w| w.value == 0.0));
    }

    #[test]
    fn the_imu_masks_cover_every_imu_the_program_has() {
        assert_eq!(IMU_COUNT, 2);
        assert_eq!(ALL_IMUS_MASK, 3);
        let mut t = FaultTranslator::new();
        let w = t.step(&faults(|f| f.imu_loss = true));
        assert_eq!(
            w,
            vec![
                ParamWrite {
                    name: "SIM_ACC_FAIL_MSK",
                    value: 3.0
                },
                ParamWrite {
                    name: "SIM_GYR_FAIL_MSK",
                    value: 3.0
                },
            ]
        );
        let w = t.step(&ActiveFaults::default());
        assert_eq!(
            w.iter().map(|w| w.value).collect::<Vec<_>>(),
            vec![0.0, 0.0]
        );
    }

    #[test]
    fn faults_that_need_no_write_write_nothing() {
        let mut t = FaultTranslator::new();
        let all_other = faults(|f| {
            f.rc_loss = true;
            f.rc_silent = true;
            f.gcs_loss = true;
            f.brain_crash = true;
            f.brain_hang = true;
            f.motor_loss = 0b1111;
        });
        assert!(t.step(&all_other).is_empty());
        assert!(t.step(&ActiveFaults::default()).is_empty());
    }

    #[test]
    fn two_faults_that_start_together_and_end_apart_write_each_their_own() {
        let mut t = FaultTranslator::new();
        let both = faults(|f| {
            f.gps_loss = true;
            f.mag_loss = true;
        });
        let w = t.step(&both);
        assert_eq!(
            names(&w),
            [
                "SIM_GPS1_ENABLE",
                "SIM_MAG1_FAIL",
                "SIM_MAG2_FAIL",
                "SIM_MAG3_FAIL"
            ]
        );
        let w = t.step(&faults(|f| f.mag_loss = true));
        assert_eq!(names(&w), ["SIM_GPS1_ENABLE"]);
        assert_eq!(w[0].value, 1.0);
        let w = t.step(&ActiveFaults::default());
        assert_eq!(w.len(), 3);
    }

    #[test]
    fn a_fault_that_is_active_from_the_first_tick_is_written_at_the_first_tick() {
        let mut t = FaultTranslator::new();
        assert_eq!(t.step(&faults(|f| f.gps_loss = true)).len(), 1);
    }

    #[test]
    fn the_battery_thresholds_are_the_pack_voltage_our_compensated_threshold_has_at_a_hover() {
        let fs = FailsafeParams::default();
        assert_eq!(fs.battery_low_v, 3.5);
        assert_eq!(fs.battery_critical_v, 3.3);
        let p = battery_failsafe_params(&fs, 6, 0.025, HOVER_POWER_W);
        assert_eq!(p[0].0, "BATT_LOW_VOLT");
        assert_eq!(p[1].0, "BATT_CRT_VOLT");
        // Open-circuit 21.0 V and 19.8 V; the loaded pack is a quarter volt
        // below, solving V = ocv - R P / V.
        for ((_, v), ocv) in p.iter().zip([21.0, 19.8]) {
            let v = *v;
            let sag = ocv - v;
            assert!(sag > 0.2 && sag < 0.3, "{ocv}: sag {sag}");
            let i = HOVER_POWER_W / v;
            assert!((v + 0.025 * i - ocv).abs() < 1e-5, "{ocv}: {v}");
        }
        assert!(p[0].1 > p[1].1, "low above critical");
        // No load, no sag: the threshold is the pack's.
        let q = battery_failsafe_params(&fs, 6, 0.025, 0.0);
        assert_eq!(q[0].1, 6.0 * f64::from(3.5f32));
        assert_eq!(q[1].1, 6.0 * f64::from(3.3f32));
        // The cell count and the thresholds are the arguments.
        let mut f2 = fs;
        f2.battery_low_v = 3.8;
        f2.battery_critical_v = 3.6;
        let r = battery_failsafe_params(&f2, 4, 0.0, 100.0);
        assert!((r[0].1 - 4.0 * 3.8).abs() < 1e-6 && (r[1].1 - 4.0 * 3.6).abs() < 1e-6);
    }

    #[test]
    fn the_hover_power_is_what_our_core_draws_hovering_the_reference_quad() {
        let airframe = Airframe::reference_quad().expect("airframe");
        let pilot = HoverStep::default();
        let mut sim =
            Simulation::<_, 4>::with_options(&airframe, F405_LEAN, pilot, SimOptions::realistic(1))
                .expect("simulation");
        let (mut sum, mut n) = (0.0, 0u32);
        sim.run_until(oxide_core::time::Micros(20_000_000), |r| {
            if r.t.as_secs_f64() >= 12.0 {
                sum += r.battery_v * r.battery_a;
                n += 1;
            }
        });
        let mean = sum / f64::from(n);
        assert!(
            (mean - HOVER_POWER_W).abs() <= 0.03 * HOVER_POWER_W,
            "hover power {mean} W against the constant {HOVER_POWER_W} W"
        );
    }
}
