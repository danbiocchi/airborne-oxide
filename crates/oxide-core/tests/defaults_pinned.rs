// SPDX-License-Identifier: AGPL-3.0-or-later
//! Pins every default of the tunable configuration, bit for bit. The
//! literals were printed from the code before the parameter table existed;
//! moving the defaults into the table must not change one of them.

use oxide_core::config::{
    EkfParams, EstimatorConfig, EstimatorSource, FailsafeParams, Gains, Limits,
};

/// Float equal by bit pattern, with the name in the failure message.
fn f(name: &str, got: f32, bits: u32) {
    assert_eq!(
        got.to_bits(),
        bits,
        "{name}: got {got} ({:#010x}), pinned {:#010x}",
        got.to_bits(),
        bits
    );
}

#[test]
fn defaults_are_pinned() {
    let g = Gains::default();
    let l = Limits::default();
    let ec = EstimatorConfig::default();
    let e: EkfParams = ec.ekf;
    let fs = FailsafeParams::default();
    assert_eq!(ec.source, EstimatorSource::External);
    f("g.rate_p.x", g.rate_p.x, 0x41e00000);
    f("g.rate_p.y", g.rate_p.y, 0x41e00000);
    f("g.rate_p.z", g.rate_p.z, 0x41400000);
    f("g.rate_i.x", g.rate_i.x, 0x42200000);
    f("g.rate_i.y", g.rate_i.y, 0x42200000);
    f("g.rate_i.z", g.rate_i.z, 0x41200000);
    f("g.rate_d.x", g.rate_d.x, 0x3e99999a);
    f("g.rate_d.y", g.rate_d.y, 0x3e99999a);
    f("g.rate_d.z", g.rate_d.z, 0x00000000);
    f("g.rate_d_cutoff_hz", g.rate_d_cutoff_hz, 0x42700000);
    f("g.gyro_cutoff_hz", g.gyro_cutoff_hz, 0x42a00000);
    f("g.attitude_p.x", g.attitude_p.x, 0x40d00000);
    f("g.attitude_p.y", g.attitude_p.y, 0x40d00000);
    f("g.attitude_p.z", g.attitude_p.z, 0x40400000);
    f("g.velocity_p.x", g.velocity_p.x, 0x40800000);
    f("g.velocity_p.y", g.velocity_p.y, 0x40800000);
    f("g.velocity_p.z", g.velocity_p.z, 0x40800000);
    f("g.velocity_i.x", g.velocity_i.x, 0x3ecccccd);
    f("g.velocity_i.y", g.velocity_i.y, 0x3ecccccd);
    f("g.velocity_i.z", g.velocity_i.z, 0x3f4ccccd);
    f("g.velocity_d.x", g.velocity_d.x, 0x00000000);
    f("g.velocity_d.y", g.velocity_d.y, 0x00000000);
    f("g.velocity_d.z", g.velocity_d.z, 0x00000000);
    f("g.position_p.x", g.position_p.x, 0x3fc00000);
    f("g.position_p.y", g.position_p.y, 0x3fc00000);
    f("g.position_p.z", g.position_p.z, 0x3fc00000);
    f("l.max_tilt_rad", l.max_tilt_rad, 0x3f1c61aa);
    f(
        "l.max_tilt_stabilized_rad",
        l.max_tilt_stabilized_rad,
        0x3f490fdb,
    );
    f("l.max_rate.x", l.max_rate.x, 0x40733333);
    f("l.max_rate.y", l.max_rate.y, 0x40733333);
    f("l.max_rate.z", l.max_rate.z, 0x40600000);
    f("l.max_speed_xy", l.max_speed_xy, 0x40a00000);
    f("l.max_speed_up", l.max_speed_up, 0x40400000);
    f("l.max_speed_down", l.max_speed_down, 0x3fc00000);
    f("l.max_yaw_rate", l.max_yaw_rate, 0x40000000);
    f("l.rate_integral_limit", l.rate_integral_limit, 0x41f00000);
    f(
        "l.velocity_integral_limit",
        l.velocity_integral_limit,
        0x40400000,
    );
    f(
        "l.min_thrust_fraction_of_hover",
        l.min_thrust_fraction_of_hover,
        0x3dcccccd,
    );
    f("l.idle_command", l.idle_command, 0x3d4ccccd);
    assert_eq!(e.delay_horizon_us, 200000, "e.delay_horizon_us");
    f("e.gyro_noise", e.gyro_noise, 0x3c75c28f);
    f("e.accel_noise", e.accel_noise, 0x3eb33333);
    f("e.gyro_bias_noise", e.gyro_bias_noise, 0x3a83126f);
    f("e.accel_bias_noise", e.accel_bias_noise, 0x3b449ba6);
    f("e.gps_pos_noise_m", e.gps_pos_noise_m, 0x3f000000);
    f("e.gps_vel_noise_mps", e.gps_vel_noise_mps, 0x3e99999a);
    f("e.baro_noise_m", e.baro_noise_m, 0x3f000000);
    f("e.yaw_noise_rad", e.yaw_noise_rad, 0x3e19999a);
    f("e.gps_gate", e.gps_gate, 0x40a00000);
    f("e.baro_gate", e.baro_gate, 0x40a00000);
    f("e.yaw_gate", e.yaw_gate, 0x40400000);
    f("e.mag_declination_rad", e.mag_declination_rad, 0x00000000);
    f("e.init_tilt_sigma_rad", e.init_tilt_sigma_rad, 0x3dcccccd);
    f("e.init_yaw_sigma_rad", e.init_yaw_sigma_rad, 0x3e99999a);
    f("e.init_vel_sigma_mps", e.init_vel_sigma_mps, 0x3f000000);
    f("e.init_pos_sigma_m", e.init_pos_sigma_m, 0x3f800000);
    f("e.init_gyro_bias_sigma", e.init_gyro_bias_sigma, 0x3ca3d70a);
    f(
        "e.init_accel_bias_sigma",
        e.init_accel_bias_sigma,
        0x3dcccccd,
    );
    assert_eq!(e.align_time_us, 1000000, "e.align_time_us");
    assert_eq!(e.gps_timeout_us, 5000000, "e.gps_timeout_us");
    assert_eq!(e.pos_timeout_us, 1500000, "e.pos_timeout_us");
    assert_eq!(e.baro_timeout_us, 5000000, "e.baro_timeout_us");
    assert_eq!(e.mag_timeout_us, 30000000, "e.mag_timeout_us");
    f("e.output_tau_s", e.output_tau_s, 0x3e800000);
    assert_eq!(fs.rc_timeout_us, 500000, "fs.rc_timeout_us");
    f("fs.rc_valid_min_s", fs.rc_valid_min_s, 0x3f800000);
    f("fs.rc_loss_hold_s", fs.rc_loss_hold_s, 0x40400000);
    assert_eq!(fs.gcs_timeout_us, 3000000, "fs.gcs_timeout_us");
    f("fs.no_link_rtl_s", fs.no_link_rtl_s, 0x42700000);
    assert_eq!(fs.brain_timeout_us, 2000000, "fs.brain_timeout_us");
    f("fs.brain_loss_rtl_s", fs.brain_loss_rtl_s, 0x41200000);
    assert_eq!(fs.setpoint_timeout_us, 500000, "fs.setpoint_timeout_us");
    f("fs.gps_loss_land_s", fs.gps_loss_land_s, 0x41f00000);
    f("fs.clear_hysteresis_s", fs.clear_hysteresis_s, 0x3f800000);
    f("fs.battery_low_v", fs.battery_low_v, 0x40600000);
    f("fs.battery_critical_v", fs.battery_critical_v, 0x40533333);
    f("fs.battery_debounce_s", fs.battery_debounce_s, 0x40000000);
    f(
        "fs.battery_filter_tau_s",
        fs.battery_filter_tau_s,
        0x40000000,
    );
    f("fs.fence_radius_m", fs.fence_radius_m, 0x00000000);
    f("fs.fence_altitude_m", fs.fence_altitude_m, 0x00000000);
    f("fs.loc_error_rad", fs.loc_error_rad, 0x3f490fdb);
    f("fs.loc_error_s", fs.loc_error_s, 0x3f000000);
    f("fs.loc_tilt_rad", fs.loc_tilt_rad, 0x3fa78d36);
    f("fs.loc_tilt_s", fs.loc_tilt_s, 0x3e800000);
    f("fs.rtl_altitude_m", fs.rtl_altitude_m, 0x41200000);
    f("fs.rtl_speed_mps", fs.rtl_speed_mps, 0x40400000);
    f("fs.land_speed_mps", fs.land_speed_mps, 0x3f333333);
    f("fs.land_blind_thrust", fs.land_blind_thrust, 0x3f666666);
    f("fs.touchdown_thrust", fs.touchdown_thrust, 0x3e99999a);
    f("fs.touchdown_speed_mps", fs.touchdown_speed_mps, 0x3e800000);
    f("fs.touchdown_s", fs.touchdown_s, 0x3f800000);
}
