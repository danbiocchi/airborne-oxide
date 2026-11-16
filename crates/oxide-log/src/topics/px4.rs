// SPDX-License-Identifier: AGPL-3.0-or-later
//! The topics that exist in PX4, under PX4's names, fields and units, and the
//! functions that fill their records (ADR-0005 §3, `inv-px4-topics.md` §6).
//!
//! The format strings are PX4's generated ones (PX4-Autopilot `99a3048`):
//! fields sorted by size, `_padding` fields named as the generator names them.
//! `record_len` is the size without the trailing padding, which `pyulog`
//! refuses in a record.

use super::{Scales, neg_ratio, put_quat, put_vec3, ratio};
use crate::Sink;
use crate::record::{RecordWriter, Topic};
use oxide_core::codes::{px4_arming_state, px4_nav_state};
use oxide_core::control::{HorizontalForm, VerticalForm, YawForm};
use oxide_core::flight::Setpoints;
use oxide_core::io::{
    Action, BaroSample, BatterySample, ImuSample, MagSample, NavState, RcSample, Reasons, Stamped,
    StateSample, Status, Validity,
};
use oxide_core::math::{Quat, Vec3};
use oxide_core::time::Micros;

/// `sensor_combined`.
pub const SENSOR_COMBINED: Topic = Topic {
    name: "sensor_combined",
    format: "uint64_t timestamp;float[3] gyro_rad;uint32_t gyro_integral_dt;\
             int32_t accelerometer_timestamp_relative;float[3] accelerometer_m_s2;\
             uint32_t accelerometer_integral_dt;uint8_t accelerometer_clipping;\
             uint8_t gyro_clipping;uint8_t accel_calibration_count;\
             uint8_t gyro_calibration_count;",
    record_len: 48,
};

/// `vehicle_angular_velocity`.
pub const VEHICLE_ANGULAR_VELOCITY: Topic = Topic {
    name: "vehicle_angular_velocity",
    format: "uint64_t timestamp;uint64_t timestamp_sample;float[3] xyz;float[3] xyz_derivative;",
    record_len: 40,
};

/// `vehicle_rates_setpoint`.
pub const VEHICLE_RATES_SETPOINT: Topic = Topic {
    name: "vehicle_rates_setpoint",
    format: "uint64_t timestamp;float roll;float pitch;float yaw;float[3] thrust_body;\
             bool reset_integral;uint8_t[7] _padding0;",
    record_len: 33,
};

/// `actuator_motors`.
pub const ACTUATOR_MOTORS: Topic = Topic {
    name: "actuator_motors",
    format: "uint64_t timestamp;uint64_t timestamp_sample;float[12] control;\
             uint16_t reversible_flags;uint8_t[6] _padding0;",
    record_len: 66,
};

/// `vehicle_thrust_setpoint`.
pub const VEHICLE_THRUST_SETPOINT: Topic = Topic {
    name: "vehicle_thrust_setpoint",
    format: "uint64_t timestamp;uint64_t timestamp_sample;float[3] xyz;uint8_t[4] _padding0;",
    record_len: 28,
};

/// `vehicle_torque_setpoint`.
pub const VEHICLE_TORQUE_SETPOINT: Topic = Topic {
    name: "vehicle_torque_setpoint",
    format: "uint64_t timestamp;uint64_t timestamp_sample;float[3] xyz;uint8_t[4] _padding0;",
    record_len: 28,
};

/// `vehicle_attitude`.
pub const VEHICLE_ATTITUDE: Topic = Topic {
    name: "vehicle_attitude",
    format: "uint64_t timestamp;uint64_t timestamp_sample;float[4] q;float[4] delta_q_reset;\
             uint8_t quat_reset_counter;uint8_t[7] _padding0;",
    record_len: 49,
};

/// `vehicle_attitude_setpoint`.
pub const VEHICLE_ATTITUDE_SETPOINT: Topic = Topic {
    name: "vehicle_attitude_setpoint",
    format: "uint64_t timestamp;float yaw_sp_move_rate;float[4] q_d;float[3] thrust_body;",
    record_len: 40,
};

/// `vehicle_local_position`.
pub const VEHICLE_LOCAL_POSITION: Topic = Topic {
    name: "vehicle_local_position",
    format: "uint64_t timestamp;uint64_t timestamp_sample;uint64_t ref_timestamp;\
             double ref_lat;double ref_lon;float x;float y;float z;float[2] delta_xy;\
             float delta_z;float vx;float vy;float vz;float z_deriv;float[2] delta_vxy;\
             float delta_vz;float ax;float ay;float az;float heading;float heading_var;\
             float unaided_heading;float delta_heading;float tilt_var;float ref_alt;\
             float dist_bottom;float dist_bottom_var;float delta_dist_bottom;float eph;\
             float epv;float evh;float evv;float vxy_max;float vz_max;float hagl_min;\
             float hagl_max_z;float hagl_max_xy;bool xy_valid;bool z_valid;bool v_xy_valid;\
             bool v_z_valid;uint8_t xy_reset_counter;uint8_t z_reset_counter;\
             uint8_t vxy_reset_counter;uint8_t vz_reset_counter;uint8_t heading_reset_counter;\
             bool heading_good_for_control;bool xy_global;bool z_global;bool dist_bottom_valid;\
             uint8_t dist_bottom_reset_counter;uint8_t dist_bottom_sensor_bitfield;\
             bool dead_reckoning;",
    record_len: 192,
};

/// `vehicle_local_position_setpoint`.
pub const VEHICLE_LOCAL_POSITION_SETPOINT: Topic = Topic {
    name: "vehicle_local_position_setpoint",
    format: "uint64_t timestamp;float x;float y;float z;float vx;float vy;float vz;\
             float[3] acceleration;float[3] thrust;float yaw;float yawspeed;",
    record_len: 64,
};

/// `manual_control_setpoint`.
pub const MANUAL_CONTROL_SETPOINT: Topic = Topic {
    name: "manual_control_setpoint",
    format: "uint64_t timestamp;uint64_t timestamp_sample;uint32_t source_system_id;\
             float roll;float pitch;float yaw;float throttle;float flaps;float aux1;float aux2;\
             float aux3;float aux4;float aux5;float aux6;uint16_t buttons;bool valid;\
             uint8_t data_source;uint8_t source_component_id;bool sticks_moving;\
             uint8_t[2] _padding0;",
    record_len: 70,
};

/// `vehicle_magnetometer`.
pub const VEHICLE_MAGNETOMETER: Topic = Topic {
    name: "vehicle_magnetometer",
    format: "uint64_t timestamp;uint64_t timestamp_sample;uint32_t device_id;\
             float[3] magnetometer_ga;uint8_t calibration_count;uint8_t[7] _padding0;",
    record_len: 33,
};

/// `vehicle_air_data`.
pub const VEHICLE_AIR_DATA: Topic = Topic {
    name: "vehicle_air_data",
    format: "uint64_t timestamp;uint64_t timestamp_sample;uint32_t baro_device_id;\
             float baro_alt_meter;float baro_pressure_pa;float ambient_temperature;float rho;\
             uint8_t temperature_source;uint8_t calibration_count;uint8_t[2] _padding0;",
    record_len: 38,
};

/// `battery_status`.
pub const BATTERY_STATUS: Topic = Topic {
    name: "battery_status",
    format: "uint64_t timestamp;float voltage_v;float current_a;float current_average_a;\
             float discharged_mah;float remaining;float scale;float time_remaining_s;\
             float temperature;float[14] voltage_cell_v;float max_cell_voltage_delta;\
             float full_charge_capacity_wh;float remaining_capacity_wh;float nominal_voltage;\
             float internal_resistance_estimate;float ocv_estimate;float ocv_estimate_filtered;\
             float volt_based_soc_estimate;float voltage_prediction;float prediction_error;\
             float estimation_covariance_norm;uint16_t capacity;uint16_t cycle_count;\
             uint16_t average_time_to_empty;uint16_t manufacture_date;uint16_t state_of_health;\
             uint16_t max_error;uint16_t interface_error;uint16_t faults;\
             uint16_t over_discharge_count;bool connected;uint8_t cell_count;uint8_t source;\
             uint8_t priority;uint8_t id;bool is_powering_off;bool is_required;uint8_t warning;\
             uint8_t[2] _padding0;",
    record_len: 166,
};

/// `vehicle_status`.
pub const VEHICLE_STATUS: Topic = Topic {
    name: "vehicle_status",
    format: "uint64_t timestamp;uint64_t armed_time;uint64_t takeoff_time;\
             uint64_t nav_state_timestamp;uint32_t valid_nav_states_mask;\
             uint32_t can_set_nav_states_mask;uint32_t system_id;uint8_t arming_state;\
             uint8_t latest_arming_reason;uint8_t latest_disarming_reason;\
             uint8_t nav_state_user_intention;uint8_t nav_state;uint8_t executor_in_charge;\
             uint8_t nav_state_display;bool accepts_offboard_setpoints;uint8_t hil_state;\
             uint8_t vehicle_type;bool failsafe;bool failsafe_and_user_took_over;\
             uint8_t failsafe_defer_state;bool gcs_connection_lost;\
             uint8_t gcs_connection_lost_counter;bool high_latency_data_link_lost;bool is_vtol;\
             bool is_vtol_tailsitter;bool in_transition_mode;bool in_transition_to_fw;\
             uint8_t system_type;uint8_t component_id;bool safety_button_available;\
             bool safety_off;bool power_input_valid;bool usb_connected;\
             bool open_drone_id_system_present;bool open_drone_id_system_healthy;\
             bool parachute_system_present;bool parachute_system_healthy;\
             bool traffic_avoidance_system_present;bool rc_calibration_in_progress;\
             bool calibration_enabled;bool pre_flight_checks_pass;uint8_t[2] _padding0;",
    record_len: 78,
};

/// `failsafe_flags`.
pub const FAILSAFE_FLAGS: Topic = Topic {
    name: "failsafe_flags",
    format: "uint64_t timestamp;uint32_t mode_req_angular_velocity;uint32_t mode_req_attitude;\
             uint32_t mode_req_local_alt;uint32_t mode_req_local_position;\
             uint32_t mode_req_local_position_relaxed;uint32_t mode_req_global_position;\
             uint32_t mode_req_global_position_relaxed;uint32_t mode_req_mission;\
             uint32_t mode_req_offboard_signal;uint32_t mode_req_home_position;\
             uint32_t mode_req_wind_and_flight_time_compliance;uint32_t mode_req_prevent_arming;\
             uint32_t mode_req_manual_control;uint32_t mode_req_other;\
             uint32_t mode_not_supported;bool angular_velocity_invalid;bool attitude_invalid;\
             bool local_altitude_invalid;bool local_position_invalid;\
             bool local_position_invalid_relaxed;bool local_velocity_invalid;\
             bool global_position_invalid;bool global_position_invalid_relaxed;\
             bool auto_mission_missing;bool offboard_control_signal_lost;\
             bool home_position_invalid;bool manual_control_signal_lost;bool gcs_connection_lost;\
             uint8_t battery_warning;bool battery_low_remaining_time;bool battery_unhealthy;\
             bool fd_critical_failure;bool fd_esc_arming_failure;bool fd_imbalanced_prop;\
             bool fd_motor_failure;bool fd_motor_stuck_failure;bool fd_alt_loss;\
             bool geofence_breached;bool mission_failure;bool vtol_fixed_wing_system_failure;\
             bool wind_limit_exceeded;bool flight_time_limit_exceeded;bool position_accuracy_low;\
             bool navigator_failure;bool parachute_unhealthy;bool traffic_avoidance_unhealthy;\
             bool remote_id_unhealthy;bool gnss_lost;uint8_t[3] _padding0;",
    record_len: 101,
};

// ---- encoders ----

/// PX4's `RELATIVE_TIMESTAMP_INVALID`.
const RELATIVE_TIMESTAMP_INVALID: i32 = i32::MAX;

/// `t_sample - now` in microseconds as PX4's relative timestamp (the sample
/// time is `timestamp + relative`), or the invalid marker when it does not
/// fit.
fn relative_us(t_sample: Micros, now: Micros) -> i32 {
    let d = i128::from(t_sample.0) - i128::from(now.0);
    match i32::try_from(d) {
        Ok(v) if v != RELATIVE_TIMESTAMP_INVALID => v,
        _ => RELATIVE_TIMESTAMP_INVALID,
    }
}

/// `sensor_combined` from the IMU sample. Gyro (rad/s) and specific force
/// (m/s²) are copied, body FRD, one sample per record. `integral_dt_us` is
/// the sampling period (the recorder derives it from consecutive `t_sample`s;
/// 0 if unknown) and fills both integral times. The accelerometer's time is
/// the sample's: `accelerometer_timestamp_relative = t_sample − now`.
///
/// Unknown, written 0: `accelerometer_clipping`, `gyro_clipping`,
/// `accel_calibration_count`, `gyro_calibration_count`.
pub fn sensor_combined<S: Sink + ?Sized>(
    w: &mut RecordWriter<'_, S>,
    now: Micros,
    imu: &Stamped<ImuSample>,
    integral_dt_us: u32,
) {
    w.u64(now.0);
    put_vec3(w, imu.value.gyro);
    w.u32(integral_dt_us).i32(relative_us(imu.t_sample, now));
    put_vec3(w, imu.value.accel);
    w.u32(integral_dt_us).u8(0).u8(0).u8(0).u8(0);
}

/// `vehicle_angular_velocity`: the estimate's body rate, rad/s FRD.
/// `t_sample` is the time of the data it is based on.
///
/// Unknown, written 0: `xyz_derivative`.
pub fn vehicle_angular_velocity<S: Sink + ?Sized>(
    w: &mut RecordWriter<'_, S>,
    now: Micros,
    t_sample: Micros,
    rate: Vec3<f32>,
) {
    w.u64(now.0).u64(t_sample.0);
    put_vec3(w, rate);
    put_vec3(w, Vec3::new(0.0, 0.0, 0.0));
}

/// `vehicle_rates_setpoint`: the attitude task's rate target, rad/s FRD, and
/// the collective thrust target normalised by the total maximum thrust, on
/// the negative body z axis.
///
/// Unknown, written 0: `reset_integral`.
pub fn vehicle_rates_setpoint<S: Sink + ?Sized>(
    w: &mut RecordWriter<'_, S>,
    now: Micros,
    rate_target: Vec3<f32>,
    thrust_target_n: f32,
    scales: &Scales,
) {
    w.u64(now.0)
        .f32(rate_target.x)
        .f32(rate_target.y)
        .f32(rate_target.z);
    put_vec3(
        w,
        Vec3::new(0.0, 0.0, neg_ratio(thrust_target_n, scales.thrust_max_n)),
    );
    w.bool(false);
}

/// Motor channels in `actuator_motors`.
const MOTOR_CHANNELS: usize = 12;

/// `actuator_motors`: the motor commands as the core gives them, `[0, 1]`
/// (inside PX4's `[-1, 1]`). Channels beyond the motors are NaN (the tools
/// stop counting motors at the first all-NaN channel); more than 12 motors
/// are cut at 12. `t_sample` is the time of the data the commands are based
/// on.
///
/// Unknown, written 0: `reversible_flags`.
pub fn actuator_motors<S: Sink + ?Sized>(
    w: &mut RecordWriter<'_, S>,
    now: Micros,
    t_sample: Micros,
    motors: &[f32],
) {
    w.u64(now.0).u64(t_sample.0);
    for i in 0..MOTOR_CHANNELS {
        w.f32(motors.get(i).copied().unwrap_or(f32::NAN));
    }
    w.u16(0);
}

/// `vehicle_thrust_setpoint`: the rate loop's collective thrust, normalised
/// by the total maximum thrust, on the negative body z axis (x and y are 0).
pub fn vehicle_thrust_setpoint<S: Sink + ?Sized>(
    w: &mut RecordWriter<'_, S>,
    now: Micros,
    t_sample: Micros,
    thrust_n: f32,
    scales: &Scales,
) {
    w.u64(now.0).u64(t_sample.0);
    put_vec3(
        w,
        Vec3::new(0.0, 0.0, neg_ratio(thrust_n, scales.thrust_max_n)),
    );
}

/// `vehicle_torque_setpoint`: the rate loop's torque demand, N·m body FRD,
/// each axis divided by the largest torque the airframe can produce about it
/// ([`Scales::torque_max_nm`]).
pub fn vehicle_torque_setpoint<S: Sink + ?Sized>(
    w: &mut RecordWriter<'_, S>,
    now: Micros,
    t_sample: Micros,
    torque_nm: Vec3<f32>,
    scales: &Scales,
) {
    w.u64(now.0).u64(t_sample.0);
    put_vec3(
        w,
        Vec3::new(
            ratio(torque_nm.x, scales.torque_max_nm.x),
            ratio(torque_nm.y, scales.torque_max_nm.y),
            ratio(torque_nm.z, scales.torque_max_nm.z),
        ),
    );
}

/// `vehicle_attitude`: the estimate's attitude, `w` first, body to world.
///
/// Unknown, written 0: `delta_q_reset`, `quat_reset_counter` (the core has
/// one combined reset counter, in `oxide_estimator`).
pub fn vehicle_attitude<S: Sink + ?Sized>(
    w: &mut RecordWriter<'_, S>,
    now: Micros,
    t_sample: Micros,
    q: Quat<f32>,
) {
    w.u64(now.0).u64(t_sample.0);
    put_quat(w, q);
    put_quat(
        w,
        Quat {
            w: 0.0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
    );
    w.u8(0);
}

/// `vehicle_attitude_setpoint`: guidance's attitude target, and the thrust
/// target normalised by the total maximum thrust on the negative body z axis.
///
/// Unknown, written 0: `yaw_sp_move_rate`.
pub fn vehicle_attitude_setpoint<S: Sink + ?Sized>(
    w: &mut RecordWriter<'_, S>,
    now: Micros,
    attitude_target: Quat<f32>,
    thrust_target_n: f32,
    scales: &Scales,
) {
    w.u64(now.0).f32(0.0);
    put_quat(w, attitude_target);
    put_vec3(
        w,
        Vec3::new(0.0, 0.0, neg_ratio(thrust_target_n, scales.thrust_max_n)),
    );
}

/// `vehicle_local_position` from the published estimate. Position and
/// velocity are NED; `heading` is the attitude's yaw; `z_deriv` is the
/// vertical velocity. `xy_valid`, `z_valid`, `v_xy_valid` and
/// `heading_good_for_control` are the validity flags `position`, `altitude`,
/// `velocity` and `heading`; there is no flag for vertical speed, so
/// `v_z_valid` is `altitude || velocity` (ADR-0003 §2); `dead_reckoning` is
/// "GPS is not aiding".
///
/// Unknown, written 0: `ref_timestamp`, `ref_lat`, `ref_lon`, `ref_alt`
/// (the core has no geodetic reference), `delta_*` and every reset counter
/// (one combined counter exists), `ax`, `ay`, `az`, `heading_var`,
/// `unaided_heading`, `tilt_var`, `dist_bottom*`, `eph`, `epv`, `evh`, `evv`,
/// `vxy_max`, `vz_max`, `hagl_*`, `xy_global`, `z_global`.
pub fn vehicle_local_position<S: Sink + ?Sized>(
    w: &mut RecordWriter<'_, S>,
    now: Micros,
    t_sample: Micros,
    est: &StateSample,
    valid: &Validity,
    gps_aiding: bool,
) {
    let p = est.position;
    let v = est.velocity;
    w.u64(now.0).u64(t_sample.0).u64(0).f64(0.0).f64(0.0);
    w.f32(p.x).f32(p.y).f32(p.z);
    w.f32_array(&[0.0, 0.0]).f32(0.0); // delta_xy, delta_z
    w.f32(v.x).f32(v.y).f32(v.z).f32(v.z); // vx, vy, vz, z_deriv
    w.f32_array(&[0.0, 0.0]).f32(0.0); // delta_vxy, delta_vz
    w.f32_array(&[0.0, 0.0, 0.0]); // ax, ay, az
    w.f32(est.attitude.yaw());
    // heading_var, unaided_heading, delta_heading, tilt_var, ref_alt,
    // dist_bottom, dist_bottom_var, delta_dist_bottom, eph, epv, evh, evv,
    // vxy_max, vz_max, hagl_min, hagl_max_z, hagl_max_xy.
    w.pad(17 * 4);
    w.bool(valid.position)
        .bool(valid.altitude)
        .bool(valid.velocity)
        .bool(valid.altitude || valid.velocity);
    w.u8(0).u8(0).u8(0).u8(0).u8(0); // reset counters
    w.bool(valid.heading).bool(false).bool(false).bool(false);
    w.u8(0).u8(0); // dist_bottom_reset_counter, dist_bottom_sensor_bitfield
    w.bool(!gps_aiding);
}

/// `vehicle_local_position_setpoint` from the setpoints guidance is flying.
/// As PX4 defines it, NaN means "not controlled": a position axis is written
/// only where its form is `Position`, a velocity axis where it is `Position`
/// or `Velocity` (and a velocity setpoint exists), `yaw` only for the
/// `Heading` form; `acceleration` and `yawspeed` are always NaN (the core
/// controls neither), and with no setpoints (`None`) the whole record is NaN.
/// `thrust` is the thrust target normalised by the total maximum thrust, as a
/// vector in the world frame: the target attitude rotating `(0, 0, −n)`.
pub fn vehicle_local_position_setpoint<S: Sink + ?Sized>(
    w: &mut RecordWriter<'_, S>,
    now: Micros,
    setpoints: Option<&Setpoints>,
    attitude_target: Quat<f32>,
    thrust_target_n: f32,
    scales: &Scales,
) {
    const NAN: f32 = f32::NAN;
    let (pos, vel, yaw, thrust) = match setpoints {
        None => ([NAN; 3], [NAN; 3], NAN, [NAN; 3]),
        Some(s) => {
            let horizontal = s.horizontal == HorizontalForm::Position;
            let vertical = s.vertical == VerticalForm::Position;
            let pick = |on: bool, v: f32| if on { v } else { NAN };
            let pos = [
                pick(horizontal, s.position.x),
                pick(horizontal, s.position.y),
                pick(vertical, s.position.z),
            ];
            let vel = match s.velocity {
                None => [NAN; 3],
                Some(v) => {
                    let h = s.horizontal != HorizontalForm::Tilt;
                    let z = s.vertical != VerticalForm::Thrust;
                    [pick(h, v.x), pick(h, v.y), pick(z, v.z)]
                }
            };
            let yaw = pick(s.yaw_form == YawForm::Heading, s.yaw);
            let t = attitude_target.rotate(Vec3::new(
                0.0,
                0.0,
                neg_ratio(thrust_target_n, scales.thrust_max_n),
            ));
            (pos, vel, yaw, [t.x, t.y, t.z])
        }
    };
    w.u64(now.0);
    w.f32_array(&pos).f32_array(&vel);
    w.f32_array(&[NAN; 3]); // acceleration
    w.f32_array(&thrust).f32(yaw).f32(NAN);
}

/// `manual_control_setpoint` from the radio sample. Sticks keep their signs;
/// the throttle is zero-centred as PX4's is, `2·t − 1` (−1 is 0 %, +1 is
/// 100 %). `aux1` is the mode switch (position 0 → −1, 1 → 0, higher → +1),
/// `aux2` the arm switch and `aux3` the kill switch (off −1, on +1). `valid`
/// is `RcSample::is_valid`. `data_source` is PX4's `SOURCE_RC` (1).
///
/// Unknown: `flaps`, `aux4`, `aux5`, `aux6` are NaN (PX4: a channel that is
/// not available is NaN); written 0: `source_system_id`,
/// `source_component_id`, `buttons`, `sticks_moving`.
pub fn manual_control_setpoint<S: Sink + ?Sized>(
    w: &mut RecordWriter<'_, S>,
    now: Micros,
    rc: &Stamped<RcSample>,
) {
    let s = &rc.value;
    let aux1 = match s.mode_switch {
        0 => -1.0,
        1 => 0.0,
        _ => 1.0,
    };
    let sw = |on: bool| if on { 1.0 } else { -1.0 };
    w.u64(now.0).u64(rc.t_sample.0).u32(0);
    w.f32(s.roll)
        .f32(s.pitch)
        .f32(s.yaw)
        .f32(2.0 * s.throttle - 1.0);
    w.f32(f32::NAN); // flaps
    w.f32(aux1)
        .f32(sw(s.arm_switch))
        .f32(sw(s.kill_switch))
        .f32(f32::NAN)
        .f32(f32::NAN)
        .f32(f32::NAN);
    w.u16(0).bool(s.is_valid()).u8(1).u8(0).bool(false);
}

/// `vehicle_magnetometer` from the magnetometer sample: the body-frame field
/// in gauss (the core's µT divided by 100).
///
/// Unknown, written 0: `device_id`, `calibration_count`.
pub fn vehicle_magnetometer<S: Sink + ?Sized>(
    w: &mut RecordWriter<'_, S>,
    now: Micros,
    mag: &Stamped<MagSample>,
) {
    let f = mag.value.field_ut;
    w.u64(now.0).u64(mag.t_sample.0).u32(0);
    put_vec3(w, Vec3::new(f.x / 100.0, f.y / 100.0, f.z / 100.0));
    w.u8(0);
}

/// `vehicle_air_data` from the barometer sample: `baro_alt_meter` is the
/// core's altitude (m, positive up, relative to an arbitrary reference).
///
/// Unknown, written 0: `baro_device_id`, `baro_pressure_pa`,
/// `ambient_temperature`, `rho`, `temperature_source`, `calibration_count`.
pub fn vehicle_air_data<S: Sink + ?Sized>(
    w: &mut RecordWriter<'_, S>,
    now: Micros,
    baro: &Stamped<BaroSample>,
) {
    w.u64(now.0).u64(baro.t_sample.0).u32(0);
    w.f32(baro.value.altitude_m).f32(0.0).f32(0.0).f32(0.0);
    w.u8(0).u8(0);
}

/// PX4's `WARNING_*` for the two battery latches (`LOW` 1, `CRITICAL` 2).
const fn battery_warning(low: bool, critical: bool) -> u8 {
    if critical {
        2
    } else if low {
        1
    } else {
        0
    }
}

/// `battery_status` from the battery sample and the monitor's two latches.
/// `nominal_voltage` is the configuration's, `cell_count` its cell count,
/// `internal_resistance_estimate` the pack resistance per cell (Ω); `id` is 1
/// (PX4 counts from 1), `connected` is true (a sample exists), `warning` is
/// 1 (`LOW`) or 2 (`CRITICAL`).
///
/// Unknown, written as the `@invalid` value of PX4's message: −1 for
/// `current_average_a`, `discharged_mah`, `remaining`, `scale`; NaN for
/// `time_remaining_s`, `temperature`, `remaining_capacity_wh`; 0 for
/// `voltage_cell_v`. Written 0: `max_cell_voltage_delta`,
/// `full_charge_capacity_wh`, the `ocv_*`, `volt_based_soc_estimate`,
/// `voltage_prediction`, `prediction_error`, `estimation_covariance_norm`,
/// `capacity`, `cycle_count`, `average_time_to_empty`, `manufacture_date`,
/// `state_of_health`, `max_error`, `interface_error`, `faults`,
/// `over_discharge_count`, `source`, `priority`, `is_powering_off`,
/// `is_required`.
pub fn battery_status<S: Sink + ?Sized>(
    w: &mut RecordWriter<'_, S>,
    now: Micros,
    b: &BatterySample,
    low: bool,
    critical: bool,
    scales: &Scales,
) {
    w.u64(now.0).f32(b.voltage_v).f32(b.current_a);
    w.f32(-1.0).f32(-1.0).f32(-1.0).f32(-1.0); // current_average_a, discharged_mah, remaining, scale
    w.f32(f32::NAN).f32(f32::NAN); // time_remaining_s, temperature
    w.pad(14 * 4); // voltage_cell_v
    w.f32(0.0).f32(0.0); // max_cell_voltage_delta, full_charge_capacity_wh
    w.f32(f32::NAN); // remaining_capacity_wh
    w.f32(scales.nominal_voltage_v)
        .f32(scales.cell_resistance_ohm);
    w.pad(6 * 4); // ocv_estimate .. estimation_covariance_norm
    w.pad(9 * 2); // capacity .. over_discharge_count
    w.bool(true).u8(scales.battery_cells).u8(0).u8(0).u8(1);
    w.bool(false).bool(false).u8(battery_warning(low, critical));
}

/// `vehicle_status` from the status. `arming_state` and `nav_state` are
/// PX4's numbers (`codes::px4_arming_state`, `codes::px4_nav_state`: a
/// disarmed vehicle shows the pilot's mode); `nav_state_user_intention` is the
/// pilot's mode in the same numbers; `nav_state_display` equals `nav_state`;
/// `failsafe` is "a failsafe action is in force"; `pre_flight_checks_pass`
/// is "no pre-arm check is failing"; `vehicle_type` is `ROTARY_WING` (1) and
/// `system_type` the MAVLink quadrotor (2).
///
/// Unknown, written 0: `armed_time`, `takeoff_time`, `nav_state_timestamp`
/// (derivable from events), `valid_nav_states_mask`, `can_set_nav_states_mask`,
/// `system_id`, `latest_arming_reason`, `latest_disarming_reason`,
/// `executor_in_charge`, `accepts_offboard_setpoints`, `hil_state`,
/// `failsafe_and_user_took_over` (an event, not a state), `failsafe_defer_state`,
/// `gcs_connection_lost`, `gcs_connection_lost_counter`, and every other flag
/// of the message.
pub fn vehicle_status<S: Sink + ?Sized>(w: &mut RecordWriter<'_, S>, now: Micros, s: &Status) {
    let nav = px4_nav_state(s.nav, s.mode);
    let intention = px4_nav_state(NavState::Disarmed, s.mode);
    w.u64(now.0).u64(0).u64(0).u64(0);
    w.u32(0).u32(0).u32(0);
    w.u8(px4_arming_state(s.armed))
        .u8(0)
        .u8(0)
        .u8(intention)
        .u8(nav)
        .u8(0)
        .u8(nav);
    w.bool(false).u8(0).u8(1); // accepts_offboard_setpoints, hil_state, vehicle_type
    w.bool(s.action != Action::None).bool(false).u8(0); // failsafe, took over, defer
    w.bool(false).u8(0).bool(false); // gcs lost, counter, high latency
    w.bool(false).bool(false).bool(false).bool(false); // vtol flags
    w.u8(2).u8(0); // system_type, component_id
    w.pad(11); // safety_button_available .. calibration_enabled
    w.bool(s.prearm.is_empty());
}

/// `failsafe_flags` from the status. Mapped: `angular_velocity_invalid` and
/// `attitude_invalid` from `!valid.attitude`, `local_altitude_invalid`,
/// `local_velocity_invalid`, `local_position_invalid` (and its `_relaxed`
/// twin) from `!valid.altitude`, `!valid.velocity`, `!valid.position`;
/// `home_position_invalid` from `!home_set`; `manual_control_signal_lost`
/// from `RC_LOST`; `offboard_control_signal_lost` from `BRAIN_LOST` or
/// `SETPOINT_STALE`; `battery_warning` 1 for `BATTERY_LOW`, 2 for
/// `BATTERY_CRITICAL`; `fd_critical_failure` from `LOSS_OF_CONTROL`;
/// `geofence_breached` from `GEOFENCE`; `gnss_lost` from `GPS_LOST`.
///
/// Unknown, written 0 (false, no failure): every `mode_req_*`,
/// `mode_not_supported`, `global_position_invalid`(`_relaxed`),
/// `auto_mission_missing`, `gcs_connection_lost`, `battery_low_remaining_time`,
/// `battery_unhealthy`, the other `fd_*`, `mission_failure`,
/// `vtol_fixed_wing_system_failure`, `wind_limit_exceeded`,
/// `flight_time_limit_exceeded`, `position_accuracy_low`, `navigator_failure`,
/// `parachute_unhealthy`, `traffic_avoidance_unhealthy`, `remote_id_unhealthy`.
pub fn failsafe_flags<S: Sink + ?Sized>(w: &mut RecordWriter<'_, S>, now: Micros, s: &Status) {
    let r = |bit: Reasons| s.reasons.intersects(bit);
    let battery = if r(Reasons::BATTERY_CRITICAL) {
        2
    } else {
        u8::from(r(Reasons::BATTERY_LOW))
    };
    w.u64(now.0);
    w.pad(15 * 4); // mode_req_* and mode_not_supported
    w.bool(!s.valid.attitude) // angular_velocity_invalid
        .bool(!s.valid.attitude) // attitude_invalid
        .bool(!s.valid.altitude)
        .bool(!s.valid.position)
        .bool(!s.valid.position) // relaxed
        .bool(!s.valid.velocity)
        .bool(false) // global_position_invalid
        .bool(false) // _relaxed
        .bool(false) // auto_mission_missing
        .bool(r(Reasons::BRAIN_LOST) || r(Reasons::SETPOINT_STALE))
        .bool(!s.home_set)
        .bool(r(Reasons::RC_LOST))
        .bool(false); // gcs_connection_lost
    w.u8(battery).bool(false).bool(false); // warning, low_remaining_time, unhealthy
    w.bool(r(Reasons::LOSS_OF_CONTROL)); // fd_critical_failure
    w.pad(5); // fd_esc_arming_failure .. fd_alt_loss
    w.bool(r(Reasons::GEOFENCE));
    w.pad(9); // mission_failure .. remote_id_unhealthy
    w.bool(r(Reasons::GPS_LOST));
}
