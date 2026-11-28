// SPDX-License-Identifier: AGPL-3.0-or-later
//! Hand-made values that are hard on an encoder: NaN with a payload, negative
//! zero, infinities, subnormals, the largest and smallest times.

use oxide_core::config::{
    EstimatorConfig, FailsafeParams, Gains, Limits, Rotor, Spin, VehicleConfig,
};
use oxide_core::io::{
    Action, BaroSample, BatterySample, BrainHeartbeat, FlightMode, GcsHeartbeat, GpsSample,
    ImuSample, Inputs, MagSample, NavState, ParamSet, PrearmFlags, RcSample, Reasons, Stamped,
    StateSample, Status, TrajectorySetpoint, Validity,
};
use oxide_core::math::{Quat, Vec3};
use oxide_core::params::{self, ParamValue};
use oxide_core::time::Micros;

pub const NAN_PAYLOAD: f32 = f32::from_bits(0x7FC0_1234);
pub const NEG_NAN: f32 = f32::from_bits(0xFFA0_0001);
pub const SUBNORMAL: f32 = f32::from_bits(1);

pub fn v3(a: f32, b: f32, c: f32) -> Vec3<f32> {
    Vec3::new(a, b, c)
}

/// The three kinds of awkward time: smallest, largest, ordinary.
pub fn stamped<T>(t: u64, value: T) -> Stamped<T> {
    Stamped {
        t_sample: Micros(t),
        t_arrival: Micros(t.wrapping_mul(3).wrapping_add(7)),
        value,
    }
}

pub fn imu() -> Stamped<ImuSample> {
    stamped(
        u64::MAX,
        ImuSample {
            gyro: v3(NAN_PAYLOAD, -0.0, f32::INFINITY),
            accel: v3(SUBNORMAL, f32::MIN, f32::MAX),
        },
    )
}

pub fn state() -> Stamped<StateSample> {
    stamped(
        0,
        StateSample {
            position: v3(1.5, -0.0, NEG_NAN),
            velocity: v3(f32::NEG_INFINITY, SUBNORMAL, 3.25),
            attitude: Quat {
                w: NAN_PAYLOAD,
                x: -0.0,
                y: 0.5,
                z: -0.5,
            },
            rate: v3(0.1, -0.2, 0.3),
        },
    )
}

pub fn rc() -> Stamped<RcSample> {
    stamped(
        123_456_789,
        RcSample {
            roll: -0.0,
            pitch: NAN_PAYLOAD,
            yaw: 1.0,
            throttle: SUBNORMAL,
            arm_switch: true,
            kill_switch: false,
            mode_switch: 2,
            failsafe: true,
            frame_lost: false,
        },
    )
}

pub fn rc_other() -> Stamped<RcSample> {
    stamped(
        1,
        RcSample {
            roll: 0.5,
            pitch: -0.5,
            yaw: -1.0,
            throttle: 1.0,
            arm_switch: false,
            kill_switch: true,
            mode_switch: 255,
            failsafe: false,
            frame_lost: true,
        },
    )
}

pub fn battery() -> Stamped<BatterySample> {
    stamped(
        u64::MAX - 1,
        BatterySample {
            voltage_v: f32::from_bits(0x7F80_0001),
            current_a: -0.0,
        },
    )
}

pub fn setpoint() -> Stamped<TrajectorySetpoint> {
    stamped(
        17,
        TrajectorySetpoint {
            position: v3(-0.0, 2.0, NAN_PAYLOAD),
            velocity: v3(SUBNORMAL, f32::INFINITY, -7.0),
            yaw: NEG_NAN,
        },
    )
}

pub fn gps() -> Stamped<GpsSample> {
    stamped(
        99,
        GpsSample {
            position: v3(1.0, 2.0, 3.0),
            velocity: v3(-0.0, NAN_PAYLOAD, f32::NEG_INFINITY),
            h_acc_m: SUBNORMAL,
            v_acc_m: f32::MAX,
            s_acc_mps: f32::MIN_POSITIVE,
            fix_3d: true,
        },
    )
}

pub fn gps_nofix() -> Stamped<GpsSample> {
    let mut g = gps();
    g.value.fix_3d = false;
    g
}

pub fn baro() -> Stamped<BaroSample> {
    stamped(
        5,
        BaroSample {
            altitude_m: NAN_PAYLOAD,
        },
    )
}

pub fn mag() -> Stamped<MagSample> {
    stamped(
        6,
        MagSample {
            field_ut: v3(22.0, -0.0, SUBNORMAL),
        },
    )
}

pub fn gcs() -> Stamped<GcsHeartbeat> {
    stamped(u64::MAX, GcsHeartbeat)
}

pub fn brain() -> Stamped<BrainHeartbeat> {
    stamped(8, BrainHeartbeat { progress: u32::MAX })
}

pub fn set_f32() -> ParamSet {
    ParamSet {
        id: params::find("RATE_ROLL_P").unwrap(),
        value: ParamValue::F32(NAN_PAYLOAD),
    }
}

pub fn set_i32() -> ParamSet {
    let id = params::PARAMS
        .iter()
        .position(|d| matches!(d.default, ParamValue::I32(_)))
        .map(|i| params::ParamId(i as u16))
        .unwrap();
    ParamSet {
        id,
        value: ParamValue::I32(i32::MIN),
    }
}

/// A set that names a parameter outside the table.
pub fn set_unknown() -> ParamSet {
    ParamSet {
        id: params::ParamId(60_000),
        value: ParamValue::F32(-0.0),
    }
}

/// A set whose value has the wrong kind for its parameter (the core refuses it).
pub fn set_wrong_kind() -> ParamSet {
    ParamSet {
        id: params::find("RATE_ROLL_P").unwrap(),
        value: ParamValue::I32(-1),
    }
}

/// Every input present.
pub fn full_inputs() -> Inputs {
    Inputs {
        imu: Some(imu()),
        state: Some(state()),
        rc: Some(rc()),
        battery: Some(battery()),
        setpoint: Some(setpoint()),
        gps: Some(gps()),
        baro: Some(baro()),
        mag: Some(mag()),
        gcs_heartbeat: Some(gcs()),
        brain_heartbeat: Some(brain()),
        param_set: Some(set_f32()),
        param_ack: true,
    }
}

pub const MODES: [FlightMode; 4] = [
    FlightMode::Stabilized,
    FlightMode::Altitude,
    FlightMode::Position,
    FlightMode::Offboard,
];

pub const NAVS: [NavState; 10] = [
    NavState::Disarmed,
    NavState::Stabilized,
    NavState::Altitude,
    NavState::Position,
    NavState::Offboard,
    NavState::Hold,
    NavState::Rtl,
    NavState::Land,
    NavState::LandNow,
    NavState::Terminated,
];

pub const ACTIONS: [Action; 6] = [
    Action::None,
    Action::Hold,
    Action::Rtl,
    Action::Land,
    Action::LandNow,
    Action::Terminate,
];

/// A status with every field at an unusual value.
pub fn busy_status() -> Status {
    Status {
        armed: true,
        killed: true,
        mode: FlightMode::Offboard,
        saturated: true,
        timing_faults: u32::MAX,
        no_estimate: true,
        estimator: oxide_core::io::EstimatorStatus {
            aligned: true,
            gps_aiding: true,
            rejected: u32::MAX - 1,
            resets: 0x8000_0001,
            // Not in the replay record.
            ..oxide_core::io::EstimatorStatus::default()
        },
        nav: NavState::LandNow,
        valid: Validity::ALL,
        reasons: Reasons::ALL,
        action: Action::Terminate,
        prearm: PrearmFlags::ALL,
        home_set: true,
        params_unsaved: true,
    }
}

/// The reference quad as oxide-core's own tests build it.
pub fn quad() -> VehicleConfig<4> {
    let a = 0.08;
    let r = |x: f32, y: f32, spin| Rotor {
        x,
        y,
        spin,
        max_thrust_n: 8.0,
        torque_coeff_m: 0.016,
    };
    VehicleConfig {
        mass_kg: 0.65,
        inertia: Vec3::new(1.6e-3, 1.6e-3, 2.8e-3),
        rotors: [
            r(a, a, Spin::Ccw),
            r(-a, -a, Spin::Ccw),
            r(a, -a, Spin::Cw),
            r(-a, a, Spin::Cw),
        ],
        thrust_expo: 1.0,
        nominal_voltage_v: 22.2,
        gains: Gains::default(),
        limits: Limits::default(),
        estimator: EstimatorConfig::default(),
        failsafe: FailsafeParams::default(),
        battery_cells: 6,
        battery_resistance_ohm: 0.025,
    }
}
