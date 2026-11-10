// SPDX-License-Identifier: AGPL-3.0-or-later
//! Health monitors feeding the failsafes (ADR-0003 §2, §3).
//!
//! Small pure helpers the supervisor calls once per run: battery state,
//! touchdown detection, loss-of-control detection and the geometry they
//! need. Nothing here reads a clock; time comes in as `now`. Durations are
//! compared in whole microseconds (as `f64`, exact for any `u64` below
//! 2⁵³ µs), so a timer at exactly its limit has fired.

use crate::config::FailsafeParams;
use crate::io::BatterySample;
use crate::math::{Quat, Real, Vec3};
use crate::time::Micros;

/// Microseconds from `start` to `now`, as `f64`.
fn elapsed_us(now: Micros, start: Micros) -> f64 {
    now.since(start) as f64
}

/// A duration in seconds as microseconds, `f64`.
fn secs_to_us(secs: f32) -> f64 {
    f64::from(secs) * 1_000_000.0
}

/// Tracks how long `cond` has held. Returns true once it has held
/// continuously for at least `secs`. Any false `cond` clears the timer.
fn held_for(since: &mut Option<Micros>, cond: bool, now: Micros, secs: f32) -> bool {
    if !cond {
        *since = None;
        return false;
    }
    let start = *since.get_or_insert(now);
    elapsed_us(now, start) >= secs_to_us(secs)
}

/// Battery state from a filtered, load-compensated cell voltage (ADR-0003 §3).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BatteryMonitor {
    /// Filtered, load-compensated voltage per cell, V. None before the first reading.
    cell: Option<f32>,
    /// Since when the cell voltage has been below the low threshold.
    low_since: Option<Micros>,
    /// Since when the cell voltage has been below the critical threshold.
    critical_since: Option<Micros>,
    low: bool,
    critical: bool,
}

impl BatteryMonitor {
    pub const fn new() -> Self {
        Self {
            cell: None,
            low_since: None,
            critical_since: None,
            low: false,
            critical: false,
        }
    }

    /// `sample`: a fresh battery reading, or None. `dt`: time since the
    /// previous call, s.
    ///
    /// The pack voltage is compensated for the drop across the pack's
    /// internal resistance (`V + I·R`, current clamped at zero) and divided by
    /// `cells`. Zero cells give no reading. Non-finite readings are ignored.
    #[allow(clippy::too_many_arguments)] // The signature is fixed by ADR-0003 and the supervisor.
    pub fn update(
        &mut self,
        now: Micros,
        sample: Option<BatterySample>,
        cells: u8,
        resistance_ohm: f32,
        armed: bool,
        p: &FailsafeParams,
        dt: f32,
    ) {
        if let Some(s) =
            sample.filter(|s| cells > 0 && s.voltage_v.is_finite() && s.current_a.is_finite())
        {
            let pack = s.voltage_v + s.current_a.max(0.0) * resistance_ohm;
            let reading = pack / f32::from(cells);
            if reading.is_finite() {
                self.filter(reading, dt, p.battery_filter_tau_s);
            }
        }

        let cell = self.cell;
        let low_cond = cell.is_some_and(|v| v < p.battery_low_v);
        let crit_cond = cell.is_some_and(|v| v < p.battery_critical_v);
        if held_for(&mut self.low_since, low_cond, now, p.battery_debounce_s) {
            self.low = true;
        }
        if held_for(
            &mut self.critical_since,
            crit_cond,
            now,
            p.battery_debounce_s,
        ) {
            self.critical = true;
        }

        // Latches clear only on the ground, once the pack has clearly recovered.
        if !armed {
            if cell.is_some_and(|v| v > p.battery_low_v + 0.1) {
                self.low = false;
            }
            if cell.is_some_and(|v| v > p.battery_critical_v + 0.1) {
                self.critical = false;
            }
        }
    }

    /// First-order low-pass on the cell voltage, `alpha = dt/(tau + dt)`.
    /// The first reading primes the filter.
    fn filter(&mut self, reading: f32, dt: f32, tau_s: f32) {
        match self.cell {
            None => self.cell = Some(reading),
            Some(v) => {
                let dt = if dt.is_finite() && dt > 0.0 { dt } else { 0.0 };
                let denom = tau_s + dt;
                let alpha = if denom > 0.0 {
                    (dt / denom).clamp(0.0, 1.0)
                } else {
                    1.0
                };
                let next = v + alpha * (reading - v);
                if next.is_finite() {
                    self.cell = Some(next);
                }
            }
        }
    }

    /// Filtered, load-compensated voltage per cell; None before the first reading.
    pub fn cell_voltage(&self) -> Option<f32> {
        self.cell
    }

    pub fn low(&self) -> bool {
        self.low
    }

    pub fn critical(&self) -> bool {
        self.critical
    }

    /// A reading exists and is within 3.0–4.35 V per cell.
    pub fn plausible(&self) -> bool {
        self.cell.is_some_and(|v| (3.0..=4.35).contains(&v))
    }
}

/// Touchdown detection (ADR-0003 §2).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LandDetector {
    /// Since when the touchdown condition has held.
    since: Option<Micros>,
    landed: bool,
}

impl LandDetector {
    pub const fn new() -> Self {
        Self {
            since: None,
            landed: false,
        }
    }

    /// `thrust_n`: commanded collective thrust; `vertical_speed`: estimated,
    /// m/s, None when height is invalid.
    ///
    /// Not armed → landed. Armed: landed once thrust < `touchdown_thrust` ×
    /// hover thrust and (speed unknown or |speed| < `touchdown_speed_mps`)
    /// has held for `touchdown_s`. Any violation clears it. A NaN thrust or
    /// speed counts as a violation.
    pub fn update(
        &mut self,
        now: Micros,
        armed: bool,
        thrust_n: f32,
        hover_thrust_n: f32,
        vertical_speed: Option<f32>,
        p: &FailsafeParams,
    ) -> bool {
        if !armed {
            self.since = None;
            self.landed = true;
            return true;
        }
        let thrust_ok = thrust_n < p.touchdown_thrust * hover_thrust_n;
        let speed_ok = vertical_speed.is_none_or(|s| s.abs() < p.touchdown_speed_mps);
        self.landed = held_for(&mut self.since, thrust_ok && speed_ok, now, p.touchdown_s);
        self.landed
    }

    pub fn landed(&self) -> bool {
        self.landed
    }
}

/// Loss-of-control detection (ADR-0003 §3, LOSS_OF_CONTROL).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ControlMonitor {
    /// Since when the attitude error has exceeded its limit.
    error_since: Option<Micros>,
    /// Since when the tilt has exceeded its limit.
    tilt_since: Option<Micros>,
}

impl ControlMonitor {
    pub const fn new() -> Self {
        Self {
            error_since: None,
            tilt_since: None,
        }
    }

    /// `attitude`: estimated; `target`: commanded. True when the angle between
    /// them has exceeded `loc_error_rad` for `loc_error_s`, or the tilt has
    /// exceeded `loc_tilt_rad` for `loc_tilt_s`. `active = false` (disarmed or
    /// landed) resets both timers and returns false.
    ///
    /// A non-finite attitude counts as over both limits, so a corrupt estimate
    /// trips the check instead of hiding in it.
    pub fn update(
        &mut self,
        now: Micros,
        active: bool,
        attitude: Quat<f32>,
        target: Quat<f32>,
        p: &FailsafeParams,
    ) -> bool {
        if !active {
            self.error_since = None;
            self.tilt_since = None;
            return false;
        }
        let err = attitude_error_angle(attitude, target);
        let tilt = tilt_angle(attitude);
        let err_bad = !err.is_finite() || err > p.loc_error_rad;
        let tilt_bad = !tilt.is_finite() || tilt > p.loc_tilt_rad;
        let by_error = held_for(&mut self.error_since, err_bad, now, p.loc_error_s);
        let by_tilt = held_for(&mut self.tilt_since, tilt_bad, now, p.loc_tilt_s);
        by_error || by_tilt
    }
}

/// Attitude error angle, rad: |Log(target ⊗ conj(attitude))|, in `[0, π]`.
/// NaN if either quaternion is non-finite.
pub fn attitude_error_angle(attitude: Quat<f32>, target: Quat<f32>) -> f32 {
    target.hamilton(attitude.conj()).to_rotation_vector().norm()
}

/// Tilt, rad: angle between body z and world z (NED down), in `[0, π]`.
/// NaN if the attitude is non-finite.
pub fn tilt_angle(attitude: Quat<f32>) -> f32 {
    let zb = attitude.rotate(Vec3::new(0.0, 0.0, 1.0));
    let horizontal = Vec3::new(zb.x, zb.y, 0.0).norm();
    <f32 as Real>::atan2(horizontal, zb.z)
}

/// Outside the cylinder around `home`: horizontal distance > `radius_m` (if
/// `radius_m` > 0) or height above home > `altitude_m` (if `altitude_m` > 0).
/// NED: height above home = `home.z − position.z`. Exactly on a limit is
/// inside. With a limit enabled and a non-finite position or home, the fence
/// reports a breach.
pub fn geofence_breached(
    position: Vec3<f32>,
    home: Vec3<f32>,
    radius_m: f32,
    altitude_m: f32,
) -> bool {
    let radius_on = radius_m > 0.0;
    let altitude_on = altitude_m > 0.0;
    if !radius_on && !altitude_on {
        return false;
    }
    if !position.is_finite() || !home.is_finite() {
        return true;
    }
    let horizontal = Vec3::new(position.x - home.x, position.y - home.y, 0.0).norm();
    let height = home.z - position.z;
    (radius_on && horizontal > radius_m) || (altitude_on && height > altitude_m)
}

#[cfg(test)]
mod tests {
    use super::*;

    const DEG: f32 = 0.017_453_292;

    fn t(ms: u64) -> Micros {
        Micros::from_millis(ms)
    }

    fn params() -> FailsafeParams {
        FailsafeParams::default()
    }

    /// Filter time constant of zero: every reading is taken as it arrives.
    fn snappy() -> FailsafeParams {
        FailsafeParams {
            battery_filter_tau_s: 0.0,
            ..FailsafeParams::default()
        }
    }

    fn bat(voltage_v: f32) -> Option<BatterySample> {
        Some(BatterySample {
            voltage_v,
            current_a: 0.0,
        })
    }

    // ---- BatteryMonitor -------------------------------------------------

    #[test]
    fn battery_first_reading_primes_filter() {
        let p = params();
        let mut m = BatteryMonitor::new();
        assert_eq!(m.cell_voltage(), None);
        assert!(!m.plausible());
        m.update(t(0), bat(12.0), 3, 0.0, false, &p, 0.1);
        assert_eq!(m.cell_voltage(), Some(4.0));
        assert!(m.plausible());
    }

    #[test]
    fn battery_filter_is_first_order_lowpass() {
        let p = params(); // tau = 2 s
        let mut m = BatteryMonitor::new();
        m.update(t(0), bat(4.0), 1, 0.0, false, &p, 0.1);
        m.update(t(100), bat(3.0), 1, 0.0, false, &p, 0.1);
        let alpha = 0.1_f32 / 2.1;
        let expected = 4.0 - alpha;
        assert!((m.cell_voltage().unwrap() - expected).abs() < 1e-5);
        for k in 2..=100 {
            m.update(t(k * 100), bat(3.0), 1, 0.0, false, &p, 0.1);
        }
        // After 10 s (five time constants) the filter is within 1 % of the step.
        let v = m.cell_voltage().unwrap();
        assert!(v > 3.0 && v < 3.02, "filtered {v}");
    }

    #[test]
    fn battery_compensates_for_load() {
        let p = params();
        let mut m = BatteryMonitor::new();
        let s = Some(BatterySample {
            voltage_v: 12.0,
            current_a: 10.0,
        });
        // Pack 12 V + 10 A × 0.05 Ω = 12.5 V over three cells.
        m.update(t(0), s, 3, 0.05, false, &p, 0.1);
        let v = m.cell_voltage().unwrap();
        assert!((v - 12.5 / 3.0).abs() < 1e-5, "cell {v}");
    }

    #[test]
    fn battery_negative_current_is_not_compensated() {
        let p = params();
        let mut m = BatteryMonitor::new();
        let s = Some(BatterySample {
            voltage_v: 12.0,
            current_a: -5.0,
        });
        m.update(t(0), s, 3, 0.05, false, &p, 0.1);
        assert!((m.cell_voltage().unwrap() - 4.0).abs() < 1e-6);
    }

    #[test]
    fn battery_low_latches_at_exact_debounce() {
        let p = params(); // debounce 2 s
        let mut m = BatteryMonitor::new();
        for ms in (0..=1900).step_by(100) {
            m.update(t(ms), bat(10.2), 3, 0.0, true, &p, 0.1); // 3.4 V per cell
        }
        assert!(!m.low());
        m.update(t(2000), bat(10.2), 3, 0.0, true, &p, 0.1);
        assert!(m.low());
        assert!(!m.critical());
    }

    #[test]
    fn battery_debounce_restarts_after_recovery() {
        let p = snappy();
        let mut m = BatteryMonitor::new();
        for ms in [0, 500, 1000, 1500] {
            m.update(t(ms), bat(10.2), 3, 0.0, true, &p, 0.1);
        }
        m.update(t(1600), bat(10.8), 3, 0.0, true, &p, 0.1); // 3.6 V: above low
        for ms in (1700..=3200).step_by(500) {
            m.update(t(ms), bat(10.2), 3, 0.0, true, &p, 0.1);
        }
        assert!(!m.low(), "1.5 s below after recovery must not latch");
        m.update(t(3700), bat(10.2), 3, 0.0, true, &p, 0.1);
        assert!(m.low());
    }

    #[test]
    fn battery_critical_latches_independently() {
        let p = snappy();
        let mut m = BatteryMonitor::new();
        for ms in (0..=2000).step_by(100) {
            m.update(t(ms), bat(9.6), 3, 0.0, true, &p, 0.1); // 3.2 V per cell
        }
        assert!(m.low());
        assert!(m.critical());
    }

    #[test]
    fn battery_latch_holds_while_armed_and_clears_only_disarmed_and_recovered() {
        let p = snappy();
        let mut m = BatteryMonitor::new();
        for ms in (0..=2000).step_by(100) {
            m.update(t(ms), bat(10.2), 3, 0.0, true, &p, 0.1);
        }
        assert!(m.low());
        // Recovered, but armed: stays latched.
        m.update(t(2100), bat(12.0), 3, 0.0, true, &p, 0.1);
        assert!(m.low());
        // Disarmed, but only 3.55 V per cell: below threshold + 0.1 V, stays latched.
        m.update(t(2200), bat(10.65), 3, 0.0, false, &p, 0.1);
        assert!(m.low());
        // Disarmed and above 3.6 V per cell: clears.
        m.update(t(2300), bat(10.83), 3, 0.0, false, &p, 0.1);
        assert!(!m.low());
    }

    #[test]
    fn battery_critical_clears_only_when_disarmed_and_recovered() {
        let p = snappy();
        let mut m = BatteryMonitor::new();
        for ms in (0..=2000).step_by(100) {
            m.update(t(ms), bat(9.6), 3, 0.0, true, &p, 0.1);
        }
        assert!(m.critical());
        m.update(t(2100), bat(12.6), 3, 0.0, false, &p, 0.1); // 4.2 V: recovered, disarmed
        assert!(!m.critical());
    }

    #[test]
    fn battery_nan_readings_are_ignored() {
        let p = params();
        let mut m = BatteryMonitor::new();
        let nan_v = Some(BatterySample {
            voltage_v: f32::NAN,
            current_a: 0.0,
        });
        let nan_i = Some(BatterySample {
            voltage_v: 12.0,
            current_a: f32::NAN,
        });
        m.update(t(0), nan_v, 3, 0.0, false, &p, 0.1);
        m.update(t(100), nan_i, 3, 0.0, false, &p, 0.1);
        assert_eq!(
            m.cell_voltage(),
            None,
            "NaN readings must not prime the filter"
        );
        m.update(t(200), bat(12.0), 3, 0.0, false, &p, 0.1);
        assert_eq!(m.cell_voltage(), Some(4.0));
        m.update(t(300), nan_i, 3, 0.0, false, &p, 0.1);
        assert_eq!(m.cell_voltage(), Some(4.0));
    }

    #[test]
    fn battery_zero_cells_gives_no_reading() {
        let p = params();
        let mut m = BatteryMonitor::new();
        m.update(t(0), bat(12.0), 0, 0.0, false, &p, 0.1);
        assert_eq!(m.cell_voltage(), None);
        assert!(!m.plausible());
    }

    #[test]
    fn battery_plausible_bounds_are_inclusive() {
        let p = snappy();
        let mut m = BatteryMonitor::new();
        m.update(t(0), bat(3.0), 1, 0.0, false, &p, 0.1);
        assert!(m.plausible());
        m.update(t(100), bat(4.35), 1, 0.0, false, &p, 0.1);
        assert!(m.plausible());
        m.update(t(200), bat(2.99), 1, 0.0, false, &p, 0.1);
        assert!(!m.plausible());
        m.update(t(300), bat(4.36), 1, 0.0, false, &p, 0.1);
        assert!(!m.plausible());
    }

    // ---- LandDetector ---------------------------------------------------

    #[test]
    fn land_disarmed_is_landed() {
        let p = params();
        let mut d = LandDetector::new();
        assert!(d.update(t(0), false, 5.0, 1.0, Some(3.0), &p));
        assert!(d.landed());
        // Arming with high thrust: not landed, and the disarmed time does not count.
        assert!(!d.update(t(10), true, 5.0, 1.0, Some(0.0), &p));
    }

    #[test]
    fn land_needs_the_full_time() {
        let p = params(); // touchdown 1 s, thrust < 30 % of hover, speed < 0.25 m/s
        let mut d = LandDetector::new();
        assert!(!d.update(t(0), true, 0.1, 1.0, None, &p));
        assert!(!d.update(t(500), true, 0.1, 1.0, None, &p));
        assert!(!d.update(t(999), true, 0.1, 1.0, None, &p));
        assert!(d.update(t(1000), true, 0.1, 1.0, None, &p));
        assert!(d.landed());
    }

    #[test]
    fn land_interruption_restarts_the_timer() {
        let p = params();
        let mut d = LandDetector::new();
        assert!(!d.update(t(0), true, 0.1, 1.0, Some(0.0), &p));
        assert!(!d.update(t(600), true, 0.5, 1.0, Some(0.0), &p)); // thrust too high
        assert!(!d.update(t(700), true, 0.1, 1.0, Some(0.0), &p));
        assert!(!d.update(t(1600), true, 0.1, 1.0, Some(0.0), &p));
        assert!(d.update(t(1700), true, 0.1, 1.0, Some(0.0), &p));
    }

    #[test]
    fn land_fast_vertical_speed_is_not_landed() {
        let p = params();
        let mut d = LandDetector::new();
        for ms in (0..=5000).step_by(100) {
            assert!(!d.update(t(ms), true, 0.1, 1.0, Some(-0.5), &p));
        }
    }

    #[test]
    fn land_unknown_speed_uses_thrust_only() {
        let p = params();
        let mut d = LandDetector::new();
        assert!(!d.update(t(0), true, 0.0, 1.0, None, &p));
        assert!(d.update(t(1000), true, 0.0, 1.0, None, &p));
    }

    #[test]
    fn land_thrust_at_threshold_is_not_landed() {
        let p = params();
        let mut d = LandDetector::new();
        // Strictly below 30 % of hover thrust is required.
        assert!(!d.update(t(0), true, 0.3, 1.0, Some(0.0), &p));
        assert!(!d.update(t(2000), true, 0.3, 1.0, Some(0.0), &p));
    }

    #[test]
    fn land_nan_inputs_count_as_violation() {
        let p = params();
        let mut d = LandDetector::new();
        assert!(!d.update(t(0), true, f32::NAN, 1.0, Some(0.0), &p));
        assert!(!d.update(t(2000), true, 0.1, 1.0, Some(f32::NAN), &p));
    }

    #[test]
    fn land_violation_clears_landed() {
        let p = params();
        let mut d = LandDetector::new();
        assert!(!d.update(t(0), true, 0.1, 1.0, Some(0.0), &p));
        assert!(d.update(t(1000), true, 0.1, 1.0, Some(0.0), &p));
        assert!(!d.update(t(2000), true, 0.1, 1.0, Some(1.0), &p));
        assert!(!d.landed());
    }

    // ---- ControlMonitor -------------------------------------------------

    #[test]
    fn control_error_trips_at_exact_time() {
        let p = params(); // 45° for 0.5 s
        let mut m = ControlMonitor::new();
        let target = Quat::<f32>::from_euler(0.0, 0.0, 50.0 * DEG);
        for ms in (0..=400).step_by(100) {
            assert!(!m.update(t(ms), true, Quat::identity(), target, &p));
        }
        assert!(m.update(t(500), true, Quat::identity(), target, &p));
    }

    #[test]
    fn control_tilt_trips_before_error_timer() {
        let p = params(); // 75° for 0.25 s; error check would take 0.5 s
        let mut m = ControlMonitor::new();
        let a = Quat::<f32>::from_euler(80.0 * DEG, 0.0, 0.0);
        // Target equal to the attitude: no attitude error, only tilt.
        assert!(!m.update(t(0), true, a, a, &p));
        assert!(!m.update(t(249), true, a, a, &p));
        assert!(m.update(t(250), true, a, a, &p));
    }

    #[test]
    fn control_within_limits_never_trips() {
        let p = params();
        let mut m = ControlMonitor::new();
        let a = Quat::<f32>::from_euler(10.0 * DEG, 5.0 * DEG, 0.0);
        for ms in (0..=10_000).step_by(100) {
            assert!(!m.update(t(ms), true, a, Quat::identity(), &p));
        }
    }

    #[test]
    fn control_recovery_restarts_the_error_timer() {
        let p = params();
        let mut m = ControlMonitor::new();
        let bad = Quat::<f32>::from_euler(0.0, 0.0, 50.0 * DEG);
        for ms in (0..=400).step_by(100) {
            m.update(t(ms), true, Quat::identity(), bad, &p);
        }
        // Recovered at 500 ms.
        assert!(!m.update(t(500), true, Quat::identity(), Quat::identity(), &p));
        for ms in (600..=1000).step_by(100) {
            assert!(
                !m.update(t(ms), true, Quat::identity(), bad, &p),
                "at {ms} ms"
            );
        }
        assert!(m.update(t(1100), true, Quat::identity(), bad, &p));
    }

    #[test]
    fn control_inactive_resets_timers() {
        let p = params();
        let mut m = ControlMonitor::new();
        let bad = Quat::<f32>::from_euler(0.0, 0.0, 50.0 * DEG);
        for ms in (0..=400).step_by(100) {
            m.update(t(ms), true, Quat::identity(), bad, &p);
        }
        assert!(!m.update(t(450), false, Quat::identity(), bad, &p));
        assert!(!m.update(t(500), true, Quat::identity(), bad, &p));
        assert!(!m.update(t(900), true, Quat::identity(), bad, &p));
        assert!(m.update(t(1000), true, Quat::identity(), bad, &p));
    }

    #[test]
    fn control_non_finite_attitude_trips() {
        let p = params();
        let mut m = ControlMonitor::new();
        let nan = Quat::new(f32::NAN, 0.0, 0.0, 0.0);
        assert!(!m.update(t(0), true, nan, Quat::identity(), &p));
        assert!(m.update(t(500), true, nan, Quat::identity(), &p));
    }

    // ---- Geometry -------------------------------------------------------

    #[test]
    fn attitude_error_angle_matches_hand_values() {
        let id = Quat::<f32>::identity();
        let yaw30 = Quat::<f32>::from_euler(0.0, 0.0, 30.0 * DEG);
        assert!((attitude_error_angle(id, yaw30) - 30.0 * DEG).abs() < 1e-5);
        assert!((attitude_error_angle(yaw30, id) - 30.0 * DEG).abs() < 1e-5);
        let roll10 = Quat::<f32>::from_euler(10.0 * DEG, 0.0, 0.0);
        assert!((attitude_error_angle(id, roll10) - 10.0 * DEG).abs() < 1e-5);
        // Both rotated: only the difference matters.
        let yaw10 = Quat::<f32>::from_euler(0.0, 0.0, 10.0 * DEG);
        assert!((attitude_error_angle(yaw10, yaw30) - 20.0 * DEG).abs() < 1e-5);
        assert!(attitude_error_angle(roll10, roll10) < 1e-6);
    }

    #[test]
    fn attitude_error_angle_non_finite_is_not_zero() {
        let nan = Quat::new(f32::NAN, 0.0, 0.0, 0.0);
        assert!(!attitude_error_angle(nan, Quat::identity()).is_finite());
    }

    #[test]
    fn tilt_angle_matches_hand_values() {
        assert!(tilt_angle(Quat::identity()).abs() < 1e-6);
        let roll20 = Quat::<f32>::from_euler(20.0 * DEG, 0.0, 0.0);
        assert!((tilt_angle(roll20) - 20.0 * DEG).abs() < 1e-5);
        let pitch_m30 = Quat::<f32>::from_euler(0.0, -30.0 * DEG, 0.0);
        assert!((tilt_angle(pitch_m30) - 30.0 * DEG).abs() < 1e-5);
        // Yaw alone does not tilt the body z-axis.
        let yaw90 = Quat::<f32>::from_euler(0.0, 0.0, 90.0 * DEG);
        assert!(tilt_angle(yaw90).abs() < 1e-5);
        // Combined: cos(tilt) = cos(roll) · cos(pitch).
        let c = Real::cos(20.0_f64 * f64::from(DEG)) * Real::cos(30.0_f64 * f64::from(DEG));
        let expected = Real::atan2(Real::sqrt(1.0 - c * c), c);
        let combined = Quat::<f32>::from_euler(20.0 * DEG, 30.0 * DEG, 0.0);
        assert!((f64::from(tilt_angle(combined)) - expected).abs() < 1e-5);
    }

    #[test]
    fn geofence_radius_boundary_is_inside() {
        let home = Vec3::zero();
        let on = Vec3::new(3.0, 4.0, 0.0); // exactly 5 m away
        assert!(!geofence_breached(on, home, 5.0, 0.0));
        assert!(geofence_breached(on, home, 4.999, 0.0));
        assert!(!geofence_breached(
            Vec3::new(3.0, 0.0, -50.0),
            home,
            5.0,
            0.0
        ));
    }

    #[test]
    fn geofence_altitude_boundary_is_inside() {
        let home = Vec3::zero();
        // NED: up is negative z. 10 m up is exactly on the limit.
        assert!(!geofence_breached(
            Vec3::new(0.0, 0.0, -10.0),
            home,
            0.0,
            10.0
        ));
        assert!(geofence_breached(
            Vec3::new(0.0, 0.0, -10.1),
            home,
            0.0,
            10.0
        ));
        // Below home is never a breach.
        assert!(!geofence_breached(
            Vec3::new(0.0, 0.0, 50.0),
            home,
            0.0,
            10.0
        ));
    }

    #[test]
    fn geofence_disabled_limits_never_breach() {
        let home = Vec3::zero();
        let far = Vec3::new(1.0e4, -1.0e4, -1.0e4);
        assert!(!geofence_breached(far, home, 0.0, 0.0));
        assert!(!geofence_breached(far, home, -1.0, -1.0));
    }

    #[test]
    fn geofence_each_limit_is_independent() {
        let home = Vec3::new(100.0, 100.0, 0.0);
        let pos = Vec3::new(100.0, 110.0, -5.0); // 10 m out, 5 m up
        assert!(geofence_breached(pos, home, 8.0, 0.0)); // radius only
        assert!(!geofence_breached(pos, home, 12.0, 0.0));
        assert!(geofence_breached(pos, home, 0.0, 4.0)); // altitude only
        assert!(!geofence_breached(pos, home, 0.0, 6.0));
        assert!(geofence_breached(pos, home, 12.0, 4.0)); // altitude breaches alone
    }

    #[test]
    fn geofence_non_finite_position_breaches_when_enabled() {
        let home = Vec3::zero();
        let nan = Vec3::new(f32::NAN, 0.0, 0.0);
        assert!(geofence_breached(nan, home, 10.0, 0.0));
        assert!(!geofence_breached(nan, home, 0.0, 0.0));
    }
}
