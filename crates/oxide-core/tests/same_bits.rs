// SPDX-License-Identifier: AGPL-3.0-or-later
//! Bit-exact comparison of the core's inputs and outputs (M1c part three,
//! K1 step 5): what a replay compares, where `==` is not enough.

use oxide_core::io::{
    BaroSample, BatterySample, BrainHeartbeat, Event, EventKind, EventList, GcsHeartbeat,
    GpsSample, ImuSample, Inputs, MagSample, Outputs, ParamSet, RcSample, Stamped, StateSample,
    TrajectorySetpoint,
};
use oxide_core::math::{Quat, Vec3};
use oxide_core::params::{ParamId, ParamValue};
use oxide_core::time::Micros;

/// A NaN with a payload `==` cannot see and `to_bits` can.
fn nan(payload: u32) -> f32 {
    f32::from_bits(0x7fc0_0000 | payload)
}

fn st<T>(v: T) -> Option<Stamped<T>> {
    Some(Stamped {
        t_sample: Micros(10),
        t_arrival: Micros(12),
        value: v,
    })
}

fn full_inputs() -> Inputs {
    Inputs {
        imu: st(ImuSample {
            gyro: Vec3::new(0.1, 0.2, 0.3),
            accel: Vec3::new(0.0, 0.0, -9.8),
        }),
        state: st(StateSample {
            position: Vec3::new(1.0, 2.0, 3.0),
            velocity: Vec3::new(0.5, 0.0, 0.25),
            attitude: Quat {
                w: 1.0,
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            rate: Vec3::new(0.0, 0.1, 0.0),
        }),
        rc: st(RcSample {
            roll: 0.1,
            pitch: -0.2,
            yaw: 0.0,
            throttle: 0.5,
            arm_switch: true,
            kill_switch: false,
            mode_switch: 1,
            failsafe: false,
            frame_lost: false,
        }),
        battery: st(BatterySample {
            voltage_v: 22.2,
            current_a: 3.0,
        }),
        setpoint: st(TrajectorySetpoint {
            position: Vec3::new(0.0, 0.0, -5.0),
            velocity: Vec3::new(1.0, 0.0, 0.0),
            yaw: 0.5,
        }),
        gps: st(GpsSample {
            position: Vec3::new(1.0, 2.0, 3.0),
            velocity: Vec3::new(0.0, 0.0, 0.0),
            h_acc_m: 1.0,
            v_acc_m: 1.5,
            s_acc_mps: 0.2,
            fix_3d: true,
        }),
        baro: st(BaroSample { altitude_m: 12.5 }),
        mag: st(MagSample {
            field_ut: Vec3::new(25.0, 0.0, 43.0),
        }),
        gcs_heartbeat: st(GcsHeartbeat),
        brain_heartbeat: st(BrainHeartbeat { progress: 7 }),
        param_set: Some(ParamSet {
            id: ParamId(3),
            value: ParamValue::F32(1.5),
        }),
        param_ack: false,
    }
}

#[test]
fn equal_inputs_are_the_same_bits() {
    let a = full_inputs();
    assert!(a.same_bits(&a));
    assert!(a.same_bits(&full_inputs()));
    assert!(Inputs::default().same_bits(&Inputs::default()));
}

#[test]
fn negative_zero_and_nan_payloads_are_where_eq_and_same_bits_disagree() {
    // -0.0 == 0.0, but not the same bits.
    let a = Inputs {
        baro: st(BaroSample { altitude_m: 0.0 }),
        ..Inputs::default()
    };
    let b = Inputs {
        baro: st(BaroSample { altitude_m: -0.0 }),
        ..Inputs::default()
    };
    assert_eq!(a, b);
    assert!(!a.same_bits(&b));
    // A NaN is not == to itself, but is the same bits as itself.
    let n1 = Inputs {
        baro: st(BaroSample { altitude_m: nan(1) }),
        ..Inputs::default()
    };
    assert_ne!(n1, n1);
    assert!(n1.same_bits(&n1));
    // Two different NaNs are neither == nor the same bits; the same value
    // twice is.
    let n2 = Inputs {
        baro: st(BaroSample { altitude_m: nan(2) }),
        ..Inputs::default()
    };
    assert_ne!(n1, n2);
    assert!(!n1.same_bits(&n2));
    assert!(n2.same_bits(&Inputs {
        baro: st(BaroSample { altitude_m: nan(2) }),
        ..Inputs::default()
    }));
}

/// Changes one float of `full_inputs` through `f` and checks that
/// `same_bits` sees it, for the nudge, the sign of zero and a NaN.
fn each_float_counts(name: &str, zero_it: impl Fn(&mut Inputs), set: impl Fn(&mut Inputs, f32)) {
    let base = full_inputs();
    // A different value.
    let mut v = base;
    set(&mut v, 99.0);
    assert!(!base.same_bits(&v), "{name}: value");
    // The sign of a zero.
    let (mut p, mut m) = (base, base);
    zero_it(&mut p);
    set(&mut m, -0.0);
    assert!(!p.same_bits(&m), "{name}: sign of zero");
    assert_eq!(p, m, "{name}: == cannot tell them apart");
    // Two NaN payloads.
    let (mut a, mut b) = (base, base);
    set(&mut a, nan(1));
    set(&mut b, nan(2));
    assert!(!a.same_bits(&b), "{name}: NaN payload");
    assert!(a.same_bits(&a), "{name}: NaN against itself");
}

#[test]
fn every_float_of_every_input_counts() {
    macro_rules! float {
        ($name:literal, $($path:tt)+) => {
            each_float_counts(
                $name,
                |i| i.$($path)+ = 0.0,
                |i, v| i.$($path)+ = v,
            )
        };
    }
    float!("imu.gyro.x", imu.as_mut().unwrap().value.gyro.x);
    float!("imu.gyro.y", imu.as_mut().unwrap().value.gyro.y);
    float!("imu.gyro.z", imu.as_mut().unwrap().value.gyro.z);
    float!("imu.accel.x", imu.as_mut().unwrap().value.accel.x);
    float!("imu.accel.y", imu.as_mut().unwrap().value.accel.y);
    float!("imu.accel.z", imu.as_mut().unwrap().value.accel.z);
    float!("state.position.x", state.as_mut().unwrap().value.position.x);
    float!("state.position.y", state.as_mut().unwrap().value.position.y);
    float!("state.position.z", state.as_mut().unwrap().value.position.z);
    float!("state.velocity.x", state.as_mut().unwrap().value.velocity.x);
    float!("state.velocity.y", state.as_mut().unwrap().value.velocity.y);
    float!("state.velocity.z", state.as_mut().unwrap().value.velocity.z);
    float!("state.attitude.w", state.as_mut().unwrap().value.attitude.w);
    float!("state.attitude.x", state.as_mut().unwrap().value.attitude.x);
    float!("state.attitude.y", state.as_mut().unwrap().value.attitude.y);
    float!("state.attitude.z", state.as_mut().unwrap().value.attitude.z);
    float!("state.rate.x", state.as_mut().unwrap().value.rate.x);
    float!("state.rate.y", state.as_mut().unwrap().value.rate.y);
    float!("state.rate.z", state.as_mut().unwrap().value.rate.z);
    float!("rc.roll", rc.as_mut().unwrap().value.roll);
    float!("rc.pitch", rc.as_mut().unwrap().value.pitch);
    float!("rc.yaw", rc.as_mut().unwrap().value.yaw);
    float!("rc.throttle", rc.as_mut().unwrap().value.throttle);
    float!("battery.voltage", battery.as_mut().unwrap().value.voltage_v);
    float!("battery.current", battery.as_mut().unwrap().value.current_a);
    float!(
        "setpoint.position.x",
        setpoint.as_mut().unwrap().value.position.x
    );
    float!(
        "setpoint.position.y",
        setpoint.as_mut().unwrap().value.position.y
    );
    float!(
        "setpoint.position.z",
        setpoint.as_mut().unwrap().value.position.z
    );
    float!(
        "setpoint.velocity.x",
        setpoint.as_mut().unwrap().value.velocity.x
    );
    float!(
        "setpoint.velocity.y",
        setpoint.as_mut().unwrap().value.velocity.y
    );
    float!(
        "setpoint.velocity.z",
        setpoint.as_mut().unwrap().value.velocity.z
    );
    float!("setpoint.yaw", setpoint.as_mut().unwrap().value.yaw);
    float!("gps.position.x", gps.as_mut().unwrap().value.position.x);
    float!("gps.position.y", gps.as_mut().unwrap().value.position.y);
    float!("gps.position.z", gps.as_mut().unwrap().value.position.z);
    float!("gps.velocity.x", gps.as_mut().unwrap().value.velocity.x);
    float!("gps.velocity.y", gps.as_mut().unwrap().value.velocity.y);
    float!("gps.velocity.z", gps.as_mut().unwrap().value.velocity.z);
    float!("gps.h_acc", gps.as_mut().unwrap().value.h_acc_m);
    float!("gps.v_acc", gps.as_mut().unwrap().value.v_acc_m);
    float!("gps.s_acc", gps.as_mut().unwrap().value.s_acc_mps);
    float!("baro.altitude", baro.as_mut().unwrap().value.altitude_m);
    float!("mag.x", mag.as_mut().unwrap().value.field_ut.x);
    float!("mag.y", mag.as_mut().unwrap().value.field_ut.y);
    float!("mag.z", mag.as_mut().unwrap().value.field_ut.z);
    // A parameter value.
    let base = full_inputs();
    let with = |v: f32| Inputs {
        param_set: Some(ParamSet {
            id: ParamId(3),
            value: ParamValue::F32(v),
        }),
        ..base
    };
    assert!(!with(0.0).same_bits(&with(-0.0)));
    assert_eq!(with(0.0), with(-0.0));
    assert!(!with(nan(1)).same_bits(&with(nan(2))));
}

#[test]
fn everything_else_of_an_input_counts_too() {
    let base = full_inputs();
    let differs = |f: &dyn Fn(&mut Inputs), what: &str| {
        let mut v = base;
        f(&mut v);
        assert!(!base.same_bits(&v), "{what}");
        assert!(!v.same_bits(&base), "{what} (reversed)");
    };
    differs(&|i| i.imu = None, "imu missing");
    differs(&|i| i.state = None, "state missing");
    differs(&|i| i.rc = None, "rc missing");
    differs(&|i| i.battery = None, "battery missing");
    differs(&|i| i.setpoint = None, "setpoint missing");
    differs(&|i| i.gps = None, "gps missing");
    differs(&|i| i.baro = None, "baro missing");
    differs(&|i| i.mag = None, "mag missing");
    differs(&|i| i.gcs_heartbeat = None, "gcs missing");
    differs(&|i| i.brain_heartbeat = None, "brain missing");
    differs(&|i| i.param_set = None, "param_set missing");
    differs(&|i| i.param_ack = true, "param_ack");
    differs(
        &|i| i.imu.as_mut().unwrap().t_sample = Micros(11),
        "t_sample",
    );
    differs(
        &|i| i.imu.as_mut().unwrap().t_arrival = Micros(13),
        "t_arrival",
    );
    differs(&|i| i.rc.as_mut().unwrap().value.arm_switch = false, "arm");
    differs(&|i| i.rc.as_mut().unwrap().value.kill_switch = true, "kill");
    differs(&|i| i.rc.as_mut().unwrap().value.mode_switch = 2, "mode");
    differs(
        &|i| i.rc.as_mut().unwrap().value.failsafe = true,
        "failsafe",
    );
    differs(
        &|i| i.rc.as_mut().unwrap().value.frame_lost = true,
        "frame lost",
    );
    differs(&|i| i.gps.as_mut().unwrap().value.fix_3d = false, "fix");
    differs(
        &|i| i.brain_heartbeat.as_mut().unwrap().value.progress = 8,
        "progress",
    );
    differs(
        &|i| i.param_set.as_mut().unwrap().id = ParamId(4),
        "param id",
    );
    differs(
        &|i| i.param_set.as_mut().unwrap().value = ParamValue::I32(1),
        "param kind",
    );
    // An integer parameter value compares by value.
    let a = Inputs {
        param_set: Some(ParamSet {
            id: ParamId(1),
            value: ParamValue::I32(-5),
        }),
        ..Inputs::default()
    };
    assert!(a.same_bits(&a));
}

fn outputs() -> Outputs<4> {
    let mut o = Outputs::<4>::off(Micros(1000));
    o.motors = [0.1, 0.2, 0.3, 0.4];
    o.status.armed = true;
    o.status.timing_faults = 2;
    o.events.push(Event {
        t: Micros(5),
        kind: EventKind::Armed,
    });
    o.events.push(Event {
        t: Micros(6),
        kind: EventKind::Touchdown,
    });
    o
}

#[test]
fn outputs_compare_motors_by_bits_and_the_rest_exactly() {
    let a = outputs();
    assert!(a.same_bits(&outputs()));
    for k in 0..4 {
        let mut z = a;
        z.motors[k] = 0.0;
        let mut m = a;
        m.motors[k] = -0.0;
        assert_eq!(z, m);
        assert!(!z.same_bits(&m), "motor {k}: sign of zero");
        let mut n1 = a;
        n1.motors[k] = nan(1);
        let mut n2 = a;
        n2.motors[k] = nan(2);
        assert!(!n1.same_bits(&n2), "motor {k}: NaN payload");
        assert!(n1.same_bits(&n1));
        assert!(!a.same_bits(&n1));
    }
    let mut b = a;
    b.t_valid = Micros(1001);
    assert!(!a.same_bits(&b));
    let mut b = a;
    b.status.armed = false;
    assert!(!a.same_bits(&b));
    let mut b = a;
    b.status.timing_faults = 3;
    assert!(!a.same_bits(&b));
    // Events count, and their order does.
    let mut b = a;
    b.events = EventList::new();
    assert!(!a.same_bits(&b));
    let mut swapped = a;
    swapped.events = EventList::new();
    swapped.events.push(Event {
        t: Micros(6),
        kind: EventKind::Touchdown,
    });
    swapped.events.push(Event {
        t: Micros(5),
        kind: EventKind::Armed,
    });
    assert!(!a.same_bits(&swapped));
    let mut later = a;
    later.events = EventList::new();
    later.events.push(Event {
        t: Micros(7),
        kind: EventKind::Armed,
    });
    later.events.push(Event {
        t: Micros(6),
        kind: EventKind::Touchdown,
    });
    assert!(!a.same_bits(&later));
}
