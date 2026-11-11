// SPDX-License-Identifier: AGPL-3.0-or-later
//! The guidance task: turns the navigation state, the sticks and the
//! setpoints into an attitude and thrust target (ADR-0003 §2).

use super::{Core, POSITION_LEASH_M, deadband, leash};
use crate::control::{AttitudeTarget, Horizontal, Vertical, Yaw};
use crate::io::{NavState, RcSample, Validity};
use crate::math::{Quat, Real, Vec3, wrap_pi};
use crate::time::Micros;

/// Climb is done within this distance of the return height, m.
const RTL_CLIMB_DONE_M: f32 = 0.5;
/// Within this horizontal distance of home at entry, Rtl skips the climb, m.
const RTL_NEAR_HOME_M: f32 = 1.0;
/// Return is done when the vehicle is this close to the setpoint, m…
const RTL_ARRIVE_M: f32 = 0.5;
/// …and slower than this, m/s.
const RTL_ARRIVE_SPEED_MPS: f32 = 0.5;
/// The moving Rtl setpoint stays within this distance of the vehicle, m.
const RTL_LEASH_M: f32 = 3.0;

/// A descent command of at least this, m/s, counts as a command to move.
///
/// A velocity loop told to descend at 0.5 m/s or more demands at least
/// 1 m/s² downwards, and a vehicle in the air would pass 0.25 m/s
/// (`touchdown_speed_mps`) within a quarter of a second. So when the
/// estimated vertical speed stays below that for `LAND_CONTACT_S` while the
/// command stands, something is holding the vehicle up: it is on the ground.
/// Thrust is no part of the test: on the ground the position loop keeps
/// demanding tilt and with it thrust, and the vertical loop's integral limit
/// stops the thrust at about 40 %, so thrust says nothing about contact.
const CONTACT_DESCENT_MPS: f32 = 0.5;
/// How long the descent command must stand without motion, s.
const LAND_CONTACT_S: f32 = 0.5;
/// A landing with neither height nor vertical speed first descends on the
/// estimator's inertial vertical speed alone. The accelerometer's vertical
/// bias was calibrated by the barometer before it failed, so that speed
/// drifts only by centimetres per second per second, far better than open
/// loop thrust for a short descent. This long, s.
const BLIND_INERTIAL_S: f32 = 30.0;
/// After the inertial descent a landing flies a fixed thrust; after this long
/// it must be on the ground, so the thrust drops to the minimum and
/// touchdown can be detected, s.
const BLIND_LAND_MAX_S: f32 = 60.0;

/// Where Rtl is in its sequence.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(super) enum RtlPhase {
    /// Not decided yet (entered without home or position).
    #[default]
    Pending,
    Climb,
    Return,
    Landing,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(super) struct GuidanceState {
    /// Position setpoint, m NED. In `Rtl` the horizontal part is the moving
    /// setpoint and the vertical part the return height.
    pub(super) position_sp: Vec3<f32>,
    pub(super) yaw_sp: f32,
    last_nav: Option<NavState>,
    pub(super) rtl: RtlPhase,
    /// How long a commanded descent has been standing still, s.
    contact_s: f32,
    /// How long a landing has been flying blind (no height, no vertical
    /// speed), s.
    blind_s: f32,
    /// The validity flags guidance saw on its previous run, to find the edge
    /// at which a sensor returns.
    prev_valid: Validity,
    /// This Rtl has been in its `Landing` phase. Cleared on entering the
    /// state. A phase decided afresh after that never climbs again.
    rtl_landed: bool,
}

/// Horizontal hold at a point, in the best form the flags allow.
fn horizontal_point(v: Validity, sp: Vec3<f32>, vel_ff: [f32; 2]) -> Horizontal {
    if v.position && v.velocity && v.heading {
        Horizontal::Position {
            sp: [sp.x, sp.y],
            vel_ff,
        }
    } else if v.velocity && v.heading {
        Horizontal::Velocity([0.0, 0.0])
    } else {
        Horizontal::Tilt {
            roll: 0.0,
            pitch: 0.0,
        }
    }
}

/// The vertical form the flags allow for `want`, else a fixed thrust.
fn vertical_form(v: Validity, want: Vertical, fallback_thrust_n: f32) -> Vertical {
    if v.altitude {
        return want;
    }
    if v.velocity {
        return match want {
            Vertical::Position { .. } => Vertical::Velocity(0.0),
            other => other,
        };
    }
    Vertical::Thrust(fallback_thrust_n)
}

/// Whether the estimate carries a vertical speed of its own.
fn speed_known(v: Validity) -> bool {
    v.altitude || v.velocity
}

/// The vertical form of a landing descent. With neither height nor speed
/// valid, the first `BLIND_INERTIAL_S` still descend on the inertial vertical
/// speed; after that, a fixed thrust.
fn landing_vertical(
    v: Validity,
    blind_s: f32,
    land_speed_mps: f32,
    fallback_thrust_n: f32,
) -> Vertical {
    let want = Vertical::Velocity(land_speed_mps);
    if !speed_known(v) && blind_s < BLIND_INERTIAL_S {
        return want;
    }
    vertical_form(v, want, fallback_thrust_n)
}

fn yaw_form(v: Validity, yaw_sp: f32) -> Yaw {
    if v.heading {
        Yaw::Heading(yaw_sp)
    } else {
        Yaw::Free
    }
}

fn distance_xy(a: Vec3<f32>, b: Vec3<f32>) -> f32 {
    Vec3::new(a.x - b.x, a.y - b.y, 0.0).norm()
}

impl<const N: usize> Core<N> {
    pub(super) fn run_guidance(&mut self, now: Micros, dt: f32) {
        let sup = self.supervision.visible;
        let est = self.estimate.visible;
        let Some(state) = est.state.filter(|_| est.valid.attitude) else {
            self.position_ctrl.reset();
            self.guide_state.last_nav = None;
            self.guidance.pending = AttitudeTarget::default();
            return;
        };
        let v = est.valid;
        let yaw_now = state.attitude.yaw();
        let flying = sup.armed && !matches!(sup.nav, NavState::Disarmed | NavState::Terminated);
        if !flying {
            self.position_ctrl.reset();
            self.guide_state = GuidanceState {
                position_sp: state.position,
                yaw_sp: yaw_now,
                last_nav: None,
                rtl: RtlPhase::Pending,
                contact_s: 0.0,
                blind_s: 0.0,
                prev_valid: v,
                rtl_landed: false,
            };
            self.guidance.pending = AttitudeTarget {
                attitude: state.attitude,
                thrust_n: 0.0,
            };
            return;
        }

        let mut g = self.guide_state;
        if g.last_nav != Some(sup.nav) {
            // Entering a state: hold where we are.
            g.position_sp = state.position;
            g.yaw_sp = yaw_now;
            g.rtl = RtlPhase::Pending;
            g.contact_s = 0.0;
            g.blind_s = 0.0;
            g.prev_valid = v;
            g.rtl_landed = false;
            self.position_ctrl.reset();
            g.last_nav = Some(sup.nav);
        }
        Self::follow_returning_sensors(&mut g, &state.position, yaw_now, v, sup.nav);

        let l = self.cfg.limits;
        let fs = self.cfg.failsafe;
        let weight = self.cfg.weight_n();
        let hover = weight;
        let landing_thrust = fs.land_blind_thrust * weight;
        let rc = self.valid_rc(now);

        let mut landing_now = false;
        // The descent rate a pilot state's stick commands, m/s (0 if none).
        let mut descent_cmd = 0.0_f32;
        let mut target = match (sup.nav, rc) {
            (NavState::Stabilized, Some(rc)) => {
                g.yaw_sp = wrap_pi(g.yaw_sp + deadband(rc.yaw) * l.max_yaw_rate * dt);
                g.position_sp = state.position;
                let roll = rc.roll.clamp(-1.0, 1.0) * l.max_tilt_stabilized_rad;
                let pitch = -rc.pitch.clamp(-1.0, 1.0) * l.max_tilt_stabilized_rad;
                AttitudeTarget {
                    attitude: Quat::from_euler(roll, pitch, g.yaw_sp),
                    thrust_n: rc.throttle.clamp(0.0, 1.0) * self.cfg.total_max_thrust_n(),
                }
            }
            (NavState::Altitude, Some(rc)) => {
                let roll = rc.roll.clamp(-1.0, 1.0) * l.max_tilt_rad;
                let pitch = -rc.pitch.clamp(-1.0, 1.0) * l.max_tilt_rad;
                let yaw = if v.heading {
                    g.yaw_sp = wrap_pi(g.yaw_sp + deadband(rc.yaw) * l.max_yaw_rate * dt);
                    Yaw::Heading(g.yaw_sp)
                } else {
                    g.yaw_sp = yaw_now;
                    Yaw::Free
                };
                let climb = deadband((rc.throttle - 0.5) * 2.0);
                descent_cmd = (-climb * l.max_speed_down).max(0.0);
                let want = if climb != 0.0 {
                    // The height setpoint follows the vehicle.
                    g.position_sp.z = state.position.z;
                    let vz = if climb > 0.0 {
                        -climb * l.max_speed_up
                    } else {
                        -climb * l.max_speed_down
                    };
                    Vertical::Velocity(vz)
                } else {
                    Vertical::Position {
                        sp: g.position_sp.z,
                        vel_ff: 0.0,
                    }
                };
                self.position_ctrl.update_axes(
                    Horizontal::Tilt { roll, pitch },
                    vertical_form(v, want, hover),
                    yaw,
                    &state,
                    dt,
                    &self.cfg,
                )
            }
            (NavState::Position, Some(rc)) => {
                let vel = self.stick_velocity(&rc, yaw_now);
                descent_cmd = vel.z.max(0.0);
                if v.heading {
                    g.yaw_sp = wrap_pi(g.yaw_sp + deadband(rc.yaw) * l.max_yaw_rate * dt);
                } else {
                    g.yaw_sp = yaw_now;
                }
                let next = g.position_sp + vel * dt;
                g.position_sp = leash(next, state.position, POSITION_LEASH_M);
                self.position_ctrl.update_axes(
                    horizontal_point(v, g.position_sp, [vel.x, vel.y]),
                    vertical_form(
                        v,
                        Vertical::Position {
                            sp: g.position_sp.z,
                            vel_ff: vel.z,
                        },
                        hover,
                    ),
                    yaw_form(v, g.yaw_sp),
                    &state,
                    dt,
                    &self.cfg,
                )
            }
            (NavState::Offboard, _) => {
                let mut vff = Vec3::zero();
                if let Some(sp) = Self::fresh(self.setpoint, now, u64::from(fs.setpoint_timeout_us))
                {
                    g.position_sp = sp.position;
                    g.yaw_sp = wrap_pi(sp.yaw);
                    vff = sp.velocity;
                }
                self.position_ctrl.update_axes(
                    horizontal_point(v, g.position_sp, [vff.x, vff.y]),
                    vertical_form(
                        v,
                        Vertical::Position {
                            sp: g.position_sp.z,
                            vel_ff: vff.z,
                        },
                        hover,
                    ),
                    yaw_form(v, g.yaw_sp),
                    &state,
                    dt,
                    &self.cfg,
                )
            }
            (NavState::Land | NavState::LandNow, _) => {
                landing_now = true;
                descent_cmd = fs.land_speed_mps;
                self.refresh_hold_point(&mut g, &state.position, v);
                self.position_ctrl.update_axes(
                    horizontal_point(v, g.position_sp, [0.0, 0.0]),
                    landing_vertical(v, g.blind_s, fs.land_speed_mps, landing_thrust),
                    yaw_form(v, g.yaw_sp),
                    &state,
                    dt,
                    &self.cfg,
                )
            }
            (NavState::Rtl, _) => {
                let mut vff = [0.0_f32; 2];
                let mut landing = false;
                if let (Some(home), true) = (self.supervision.visible.home, v.position) {
                    match g.rtl {
                        RtlPhase::Pending => {
                            if distance_xy(state.position, home) < RTL_NEAR_HOME_M {
                                g.rtl = RtlPhase::Landing;
                            } else if g.rtl_landed {
                                // The landing was under way and the position
                                // was lost on the way down: fly home at the
                                // height we are at, then land. No climb.
                                g.rtl = RtlPhase::Return;
                                g.position_sp = state.position;
                            } else {
                                g.rtl = RtlPhase::Climb;
                                g.position_sp.x = state.position.x;
                                g.position_sp.y = state.position.y;
                                g.position_sp.z = state.position.z.min(home.z - fs.rtl_altitude_m);
                            }
                        }
                        RtlPhase::Climb => {
                            if v.altitude
                                && Real::abs(state.position.z - g.position_sp.z) < RTL_CLIMB_DONE_M
                            {
                                g.rtl = RtlPhase::Return;
                            }
                        }
                        RtlPhase::Return => {
                            let to_home =
                                Vec3::new(home.x - g.position_sp.x, home.y - g.position_sp.y, 0.0);
                            let dist = to_home.norm();
                            let step = fs.rtl_speed_mps * dt;
                            if dist <= step || dist <= 0.0 {
                                g.position_sp.x = home.x;
                                g.position_sp.y = home.y;
                            } else {
                                let k = step / dist;
                                g.position_sp.x += to_home.x * k;
                                g.position_sp.y += to_home.y * k;
                                vff = [
                                    to_home.x / dist * fs.rtl_speed_mps,
                                    to_home.y / dist * fs.rtl_speed_mps,
                                ];
                            }
                            // Never run far ahead of the vehicle.
                            let kept = leash(g.position_sp, state.position, RTL_LEASH_M);
                            g.position_sp.x = kept.x;
                            g.position_sp.y = kept.y;
                            let at_home = g.position_sp.x == home.x && g.position_sp.y == home.y;
                            let speed = Vec3::new(state.velocity.x, state.velocity.y, 0.0).norm();
                            if at_home
                                && distance_xy(state.position, g.position_sp) < RTL_ARRIVE_M
                                && speed < RTL_ARRIVE_SPEED_MPS
                            {
                                g.rtl = RtlPhase::Landing;
                            }
                        }
                        RtlPhase::Landing => {}
                    }
                    if g.rtl == RtlPhase::Landing {
                        g.rtl_landed = true;
                        g.position_sp.x = home.x;
                        g.position_sp.y = home.y;
                        landing = true;
                    }
                } else {
                    // No home or no position: fly as Hold.
                    self.refresh_hold_point(&mut g, &state.position, v);
                    if g.rtl == RtlPhase::Landing {
                        // Position lost part-way through the landing: hold
                        // the height we are at (ADR-0003 §2). The phase is
                        // decided afresh if the position comes back.
                        g.rtl = RtlPhase::Pending;
                        g.position_sp = state.position;
                    }
                }
                landing_now = landing;
                if landing {
                    descent_cmd = fs.land_speed_mps;
                }
                let vertical = if landing {
                    landing_vertical(v, g.blind_s, fs.land_speed_mps, landing_thrust)
                } else {
                    vertical_form(
                        v,
                        Vertical::Position {
                            sp: g.position_sp.z,
                            vel_ff: 0.0,
                        },
                        hover,
                    )
                };
                self.position_ctrl.update_axes(
                    horizontal_point(v, g.position_sp, vff),
                    vertical,
                    yaw_form(v, g.yaw_sp),
                    &state,
                    dt,
                    &self.cfg,
                )
            }
            // Hold, and a stick state whose RC frame has just gone: hold the
            // point (the failsafe takes over within a supervisor run).
            _ => {
                self.refresh_hold_point(&mut g, &state.position, v);
                self.position_ctrl.update_axes(
                    horizontal_point(v, g.position_sp, [0.0, 0.0]),
                    vertical_form(
                        v,
                        Vertical::Position {
                            sp: g.position_sp.z,
                            vel_ff: 0.0,
                        },
                        hover,
                    ),
                    yaw_form(v, g.yaw_sp),
                    &state,
                    dt,
                    &self.cfg,
                )
            }
        };
        if landing_now && !speed_known(v) && g.blind_s < BLIND_INERTIAL_S {
            // The inertial speed has no barometer behind it: a bias it did
            // not calibrate would make the loop "slow" a descent that is not
            // happening. Never above hover thrust, so the vehicle cannot
            // accelerate upwards. (Not the 0.9 x hover blind thrust: holding
            // a constant descent rate takes about hover thrust.)
            target.thrust_n = target.thrust_n.min(hover);
        }
        // Ground contact (ADR-0003 §2; see `CONTACT_DESCENT_MPS`): a descent
        // is commanded and the vehicle is not moving. A vertical speed is
        // available from the altitude or velocity flags, or, in the first
        // `BLIND_INERTIAL_S` of a blind landing, from the inertial estimate.
        let inertial_blind = landing_now && !speed_known(v) && g.blind_s < BLIND_INERTIAL_S;
        let speed_available = speed_known(v) || inertial_blind;
        if descent_cmd >= CONTACT_DESCENT_MPS
            && speed_available
            && Real::abs(state.velocity.z) < fs.touchdown_speed_mps
        {
            g.contact_s += dt;
        } else {
            // The vehicle moves, or the descent command ends: contact ends.
            g.contact_s = 0.0;
        }
        if landing_now && !speed_known(v) {
            g.blind_s += dt;
        } else {
            g.blind_s = 0.0;
        }
        if g.blind_s >= BLIND_LAND_MAX_S {
            target.thrust_n = l.min_thrust_fraction_of_hover * weight;
        }
        if g.contact_s >= LAND_CONTACT_S {
            // On the ground: minimum thrust, level at the current yaw, and
            // nothing left to wind up against the ground.
            target = AttitudeTarget {
                attitude: Quat::from_euler(0.0, 0.0, yaw_now),
                thrust_n: l.min_thrust_fraction_of_hover * weight,
            };
            self.position_ctrl.reset();
            // The hold point is the ground under the vehicle: a pilot's
            // stick held down must not leave it sunk below the ground, where
            // the next hold would keep commanding a descent.
            g.position_sp.z = state.position.z;
            if sup.nav == NavState::Position {
                g.position_sp.x = state.position.x;
                g.position_sp.y = state.position.y;
            }
        }
        g.prev_valid = v;
        self.guide_state = g;
        self.guidance.pending = target;
    }

    /// Re-bases the setpoints on the estimate when a part of it comes back.
    ///
    /// The estimator resets position and yaw to the sensor when the sensor
    /// returns after its timeout, and the matching flag turns valid on the
    /// same estimate. Every reset happens only while the flag is false (the
    /// timeouts exceed the validity windows), so the edge from invalid to
    /// valid marks it. A setpoint kept from before the gap is then off by the
    /// drift that built up, and the vehicle would fly or turn to it. So on the
    /// edge the setpoint becomes the estimate; while the flag is false the
    /// yaw setpoint follows the yaw. In `Rtl` climb and return the vertical
    /// setpoint is the return height, a target that stays meaningful.
    fn follow_returning_sensors(
        g: &mut GuidanceState,
        position: &Vec3<f32>,
        yaw_now: f32,
        v: Validity,
        nav: NavState,
    ) {
        let was = g.prev_valid;
        if v.position && !was.position {
            g.position_sp.x = position.x;
            g.position_sp.y = position.y;
        }
        if !v.heading || !was.heading {
            g.yaw_sp = yaw_now;
        }
        let return_height =
            nav == NavState::Rtl && matches!(g.rtl, RtlPhase::Climb | RtlPhase::Return);
        if v.altitude && !was.altitude && !return_height {
            g.position_sp.z = position.z;
        }
    }

    /// While a part of the position is not valid the held point follows the
    /// vehicle, so it does not jump when the sensor returns.
    #[allow(clippy::unused_self)]
    fn refresh_hold_point(&self, g: &mut GuidanceState, position: &Vec3<f32>, v: Validity) {
        if !v.position {
            g.position_sp.x = position.x;
            g.position_sp.y = position.y;
        }
        if !v.altitude {
            g.position_sp.z = position.z;
        }
    }

    /// Stick velocity demand in NED for position mode.
    fn stick_velocity(&self, rc: &RcSample, yaw: f32) -> Vec3<f32> {
        let l = &self.cfg.limits;
        let fwd = deadband(rc.pitch) * l.max_speed_xy;
        let right = deadband(rc.roll) * l.max_speed_xy;
        let (s, c) = (Real::sin(yaw), Real::cos(yaw));
        let climb = deadband((rc.throttle - 0.5) * 2.0);
        let vz = if climb > 0.0 {
            -climb * l.max_speed_up
        } else {
            -climb * l.max_speed_down
        };
        Vec3::new(fwd * c - right * s, fwd * s + right * c, vz)
    }
}
