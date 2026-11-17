// SPDX-License-Identifier: AGPL-3.0-or-later
//! The reply: one line of text per frame, the vehicle's state for the
//! simulator build.
//!
//! The line starts and ends with `\n` (the simulator drops a first reply
//! without the leading one). It finds its keys by a substring scan, so there is
//! no space after a colon, the keys come in the order below, and nothing but
//! these keys is written:
//!
//! `\n{"timestamp":T,"imu":{"gyro":[x,y,z],"accel_body":[x,y,z]},"position":[n,e,d],"quaternion":[w,x,y,z],"velocity":[n,e,d],"battery":{"voltage":V,"current":A},"rc":{"rc_1":p1,...,"rc_8":p8}}\n`
//!
//! Units: seconds, rad/s (body FRD), m/s² specific force (body FRD; reads
//! `(0, 0, -9.80665)` at rest), metres and m/s NED from the start point, the
//! body-to-world quaternion `(w, x, y, z)`, volts and amps, pulse widths in
//! microseconds. `battery` and `rc` are left out entirely when there is nothing
//! to send.

use core::fmt::{self, Write as _};

/// Battery voltage and current, V and A.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BatteryReading {
    pub voltage: f64,
    pub current: f64,
}

/// What one reply carries.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Reply {
    /// The time stamp, whole nanoseconds ([`timestamp_ns`]).
    pub timestamp_ns: u64,
    /// Body rate, rad/s, FRD.
    pub gyro: [f64; 3],
    /// Specific force, m/s², FRD.
    pub accel_body: [f64; 3],
    /// Position, m, NED from the start point.
    pub position: [f64; 3],
    /// Velocity, m/s, NED.
    pub velocity: [f64; 3],
    /// Body-to-world attitude, `(w, x, y, z)`.
    pub quaternion: [f64; 4],
    pub battery: Option<BatteryReading>,
    /// Radio channels 1 to 8, pulse widths in microseconds.
    pub rc: Option<[u16; 8]>,
}

/// Why a reply was not written.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ReplyError {
    /// A number is `NaN` or infinite; the simulator must never see one.
    NonFinite { field: &'static str },
}

impl fmt::Display for ReplyError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NonFinite { field } => {
                write!(
                    f,
                    "the reply's {field} is not a finite number, nothing was sent"
                )
            }
        }
    }
}

impl std::error::Error for ReplyError {}

/// The time stamp of step `k`: `k * (step + 1 ns)`, in whole nanoseconds
/// (never through a float). The extra nanosecond makes the simulator's clock,
/// which adds the differences into whole microseconds and truncates, exact.
pub fn timestamp_ns(step_us: u64, k: u64) -> u64 {
    k.saturating_mul(step_us.saturating_mul(1000).saturating_add(1))
}

impl Reply {
    /// Appends the line to `out`. If a number is not finite, nothing is
    /// appended.
    pub fn write(&self, out: &mut String) -> Result<(), ReplyError> {
        self.check()?;
        let _ = write!(
            out,
            "\n{{\"timestamp\":{}.{:09},\"imu\":{{\"gyro\":",
            self.timestamp_ns / 1_000_000_000,
            self.timestamp_ns % 1_000_000_000
        );
        push_triple(out, &self.gyro);
        out.push_str(",\"accel_body\":");
        push_triple(out, &self.accel_body);
        out.push_str("},\"position\":");
        push_triple(out, &self.position);
        let [w, x, y, z] = self.quaternion;
        let _ = write!(
            out,
            ",\"quaternion\":[{w:.12},{x:.12},{y:.12},{z:.12}],\"velocity\":"
        );
        push_triple(out, &self.velocity);
        if let Some(b) = &self.battery {
            let _ = write!(
                out,
                ",\"battery\":{{\"voltage\":{:.9},\"current\":{:.9}}}",
                b.voltage, b.current
            );
        }
        if let Some(rc) = &self.rc {
            out.push_str(",\"rc\":{");
            for (i, p) in rc.iter().enumerate() {
                if i > 0 {
                    out.push(',');
                }
                let _ = write!(out, "\"rc_{}\":{p}", i + 1);
            }
            out.push('}');
        }
        out.push_str("}\n");
        Ok(())
    }

    /// Every number is finite, or the first that is not is named.
    fn check(&self) -> Result<(), ReplyError> {
        let groups: [(&'static str, &[f64]); 5] = [
            ("gyro", &self.gyro),
            ("accel_body", &self.accel_body),
            ("position", &self.position),
            ("quaternion", &self.quaternion),
            ("velocity", &self.velocity),
        ];
        for (field, v) in groups {
            if v.iter().any(|x| !x.is_finite()) {
                return Err(ReplyError::NonFinite { field });
            }
        }
        if let Some(b) = &self.battery {
            if !b.voltage.is_finite() {
                return Err(ReplyError::NonFinite {
                    field: "battery.voltage",
                });
            }
            if !b.current.is_finite() {
                return Err(ReplyError::NonFinite {
                    field: "battery.current",
                });
            }
        }
        Ok(())
    }
}

/// `[a,b,c]` with nine fractional digits.
fn push_triple(out: &mut String, v: &[f64; 3]) {
    let [a, b, c] = *v;
    let _ = write!(out, "[{a:.9},{b:.9},{c:.9}]");
}

#[cfg(test)]
mod tests {
    use super::*;

    fn full() -> Reply {
        Reply {
            timestamp_ns: 500_001,
            gyro: [0.1, -0.2, 0.3],
            accel_body: [0.0, 0.0, -9.80665],
            position: [1.5, -2.25, -3.0],
            velocity: [0.5, 0.0, -0.125],
            quaternion: [1.0, 0.0, 0.0, 0.0],
            battery: Some(BatteryReading {
                voltage: 16.8,
                current: 12.5,
            }),
            rc: Some([1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000]),
        }
    }

    fn bare() -> Reply {
        Reply {
            timestamp_ns: 0,
            gyro: [0.0, 0.0, 0.0],
            accel_body: [0.0, 0.0, -9.80665],
            position: [0.0, 0.0, 0.0],
            velocity: [0.0, 0.0, 0.0],
            quaternion: [
                core::f64::consts::FRAC_1_SQRT_2,
                0.0,
                core::f64::consts::FRAC_1_SQRT_2,
                0.0,
            ],
            battery: None,
            rc: None,
        }
    }

    fn line(r: &Reply) -> String {
        let mut s = String::new();
        r.write(&mut s).expect("finite");
        s
    }

    #[test]
    fn a_reply_with_every_section_is_these_exact_bytes() {
        let want = concat!(
            "\n{\"timestamp\":0.000500001,",
            "\"imu\":{\"gyro\":[0.100000000,-0.200000000,0.300000000],",
            "\"accel_body\":[0.000000000,0.000000000,-9.806650000]},",
            "\"position\":[1.500000000,-2.250000000,-3.000000000],",
            "\"quaternion\":[1.000000000000,0.000000000000,0.000000000000,0.000000000000],",
            "\"velocity\":[0.500000000,0.000000000,-0.125000000],",
            "\"battery\":{\"voltage\":16.800000000,\"current\":12.500000000},",
            "\"rc\":{\"rc_1\":1500,\"rc_2\":1500,\"rc_3\":1000,\"rc_4\":1500,",
            "\"rc_5\":1000,\"rc_6\":1000,\"rc_7\":1000,\"rc_8\":1000}}\n"
        );
        assert_eq!(line(&full()), want);
    }

    #[test]
    fn a_reply_with_neither_optional_section_is_these_exact_bytes() {
        let want = concat!(
            "\n{\"timestamp\":0.000000000,",
            "\"imu\":{\"gyro\":[0.000000000,0.000000000,0.000000000],",
            "\"accel_body\":[0.000000000,0.000000000,-9.806650000]},",
            "\"position\":[0.000000000,0.000000000,0.000000000],",
            "\"quaternion\":[0.707106781187,0.000000000000,0.707106781187,0.000000000000],",
            "\"velocity\":[0.000000000,0.000000000,0.000000000]}\n"
        );
        assert_eq!(line(&bare()), want);
    }

    #[test]
    fn the_line_starts_and_ends_with_a_newline_and_has_no_space() {
        for r in [full(), bare()] {
            let s = line(&r);
            assert!(s.starts_with("\n{"), "{s:?}");
            assert!(s.ends_with("}\n"), "{s:?}");
            assert_eq!(s.matches('\n').count(), 2, "only the two framing newlines");
            assert!(!s.contains(' '), "no space anywhere: {s:?}");
            let b = s.as_bytes();
            assert!(
                !b.windows(2)
                    .any(|w| w[0].is_ascii_digit() && (w[1] == b'e' || w[1] == b'E')),
                "no exponent: {s:?}"
            );
        }
    }

    #[test]
    fn a_large_or_tiny_number_is_fixed_point_without_an_exponent() {
        let mut r = bare();
        r.position = [123_456_789.5, 1e-12, -1e-12];
        let s = line(&r);
        assert!(
            s.contains("\"position\":[123456789.500000000,0.000000000,-0.000000000]"),
            "{s}"
        );
    }

    #[test]
    fn appends_after_what_is_already_there() {
        let mut s = String::from("keep");
        full().write(&mut s).expect("finite");
        assert!(s.starts_with("keep\n{"));
    }

    #[test]
    fn each_non_finite_field_is_refused_and_leaves_out_untouched() {
        let bad = [f64::NAN, f64::INFINITY, f64::NEG_INFINITY];
        for v in bad {
            let cases: Vec<(&str, Reply)> = vec![
                (
                    "gyro",
                    Reply {
                        gyro: [0.0, v, 0.0],
                        ..full()
                    },
                ),
                (
                    "accel_body",
                    Reply {
                        accel_body: [0.0, 0.0, v],
                        ..full()
                    },
                ),
                (
                    "position",
                    Reply {
                        position: [v, 0.0, 0.0],
                        ..full()
                    },
                ),
                (
                    "velocity",
                    Reply {
                        velocity: [0.0, 0.0, v],
                        ..full()
                    },
                ),
                (
                    "quaternion",
                    Reply {
                        quaternion: [1.0, 0.0, 0.0, v],
                        ..full()
                    },
                ),
                (
                    "battery.voltage",
                    Reply {
                        battery: Some(BatteryReading {
                            voltage: v,
                            current: 1.0,
                        }),
                        ..full()
                    },
                ),
                (
                    "battery.current",
                    Reply {
                        battery: Some(BatteryReading {
                            voltage: 1.0,
                            current: v,
                        }),
                        ..full()
                    },
                ),
            ];
            for (field, r) in cases {
                let mut out = String::from("before");
                let e = r.write(&mut out).expect_err(field);
                assert_eq!(e, ReplyError::NonFinite { field }, "{v}");
                assert_eq!(out, "before", "{field}: nothing was written");
                assert!(e.to_string().contains(field), "{e}");
            }
        }
    }

    #[test]
    fn the_time_stamp_of_step_1_at_500_us_and_of_step_2_000_000() {
        let mut r = bare();
        r.timestamp_ns = timestamp_ns(500, 1);
        assert_eq!(r.timestamp_ns, 500_001);
        assert!(line(&r).starts_with("\n{\"timestamp\":0.000500001,"));
        r.timestamp_ns = timestamp_ns(500, 2_000_000);
        assert!(line(&r).starts_with("\n{\"timestamp\":1000.002000000,"));
        // A float would not do: 2^53 ns is about 104 days, and k * 500.001 us
        // in f64 seconds loses the last digits long before that.
        r.timestamp_ns = timestamp_ns(1000, 3_000_000_000);
        assert!(line(&r).starts_with("\n{\"timestamp\":3000003.000000000,"));
        assert_eq!(timestamp_ns(500, 0), 0);
        assert_eq!(timestamp_ns(500, u64::MAX), u64::MAX, "saturates, no panic");
    }

    /// The exact value, in u128 so that nothing can overflow or round.
    fn exact_ns(step_us: u64, k: u64) -> u128 {
        u128::from(k) * (u128::from(step_us) * 1000 + 1)
    }

    #[test]
    fn the_time_stamp_is_exact_integer_arithmetic_at_large_step_numbers() {
        // 2^40 steps of 500 us is about 6.4 years. A float route (k times
        // 500.001 us in f64 seconds, then times 1e9) differs from the integer
        // value in the last digits there; at 2^40 - 1 it gives
        // 549756913399127872 where the exact value is 549756913399127775.
        let ks = [
            1u64,
            3,
            999_999_937,
            (1 << 32) - 1,
            1 << 32,
            (1 << 33) + 12_345,
            123_456_789_012,
            (1 << 40) - 1,
            1 << 40,
        ];
        for step_us in [500u64, 1000, 250, 4000, 999] {
            for k in ks {
                let want = exact_ns(step_us, k);
                assert!(want < u128::from(u64::MAX), "{step_us} {k}");
                let got = timestamp_ns(step_us, k);
                assert_eq!(u128::from(got), want, "step {step_us} us, k = {k}");
                // And the printed text is those nanoseconds as a decimal.
                let mut r = bare();
                r.timestamp_ns = got;
                let text = format!(
                    "\n{{\"timestamp\":{}.{:09},",
                    want / 1_000_000_000,
                    want % 1_000_000_000
                );
                assert!(line(&r).starts_with(&text), "step {step_us} us, k = {k}");
            }
        }
        assert_eq!(timestamp_ns(500, (1 << 40) - 1), 549_756_913_399_127_775);
    }
}
