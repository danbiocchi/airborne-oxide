# ADR-0003: Failsafes, flight actions and arming

**Status:** Accepted (M1c) · **Date:** 2026-11-10 · **Design reference:** DESIGN §4.9, §4.10, §4.11

## Context

DESIGN §4.9–§4.11 say *what* the vehicle does when something fails: a table of events and default actions, a ranking, the pilot-override rules and the arming checks. This ADR turns that into a specification precise enough to implement and test, and records the places where the table had to be interpreted. It is also the specification for the failsafe state machine's proofs (DESIGN §9).

M1c is delivered in three parts. This ADR covers the first: failsafes and fault scenarios. Parameters and scenario files, then logging and replay, follow.

## Decision

### 1. The estimate carries validity flags

The estimate is no longer all-or-nothing. It carries five flags, and each controller uses only what is valid.

| Flag | Meaning | EKF sets it while |
|---|---|---|
| `attitude` | Roll, pitch, body rates | the filter is running (IMU fresh, tilt aligned) |
| `heading` | Yaw | heading initialised, and a magnetometer reading accepted within `mag_timeout_us` (30 s) |
| `altitude` | Height, vertical speed | a barometer reading accepted within `baro_timeout_us` (5 s) |
| `velocity` | Horizontal velocity | heading valid, and a GPS fix accepted within `gps_timeout_us` (5 s) |
| `position` | Horizontal position | velocity valid, and a GPS fix accepted within `pos_timeout_us` (1.5 s) |

With the external state source (truth, motion capture) all five are set while the state is fresh.

### 2. Navigation states

What the vehicle is doing is one `NavState`:

| State | Entered by | Behaviour |
|---|---|---|
| `Disarmed` | | Motors off |
| `Stabilized` | Pilot | Sticks set tilt and yaw rate; throttle sets thrust |
| `Altitude` | Pilot fallback | Sticks set tilt and yaw rate; throttle sets climb rate, centred holds height |
| `Position` | Pilot | Sticks set velocity; centred holds position |
| `Offboard` | Pilot | Follows tier-2 trajectory setpoints |
| `Hold` | Failsafe | Holds the point where it was entered |
| `Rtl` | Failsafe | Climbs to `rtl_altitude_m` above home, flies home at `rtl_speed_mps`, then lands |
| `Land` | Failsafe | Descends at `land_speed_mps`, holding horizontal position |
| `LandNow` | Failsafe | Descends in place at once, on attitude alone if it must |
| `Terminated` | Failsafe | Motors off, latched |

**Degradation.** Every state uses the best control its valid flags allow:
- **Horizontal:** position hold needs `position`; otherwise velocity damping (zero velocity target) needs `velocity`; otherwise level attitude.
- **Vertical:** height control needs `altitude`. Without it, climb-rate control still works while `velocity` is valid, because GPS keeps correcting vertical speed. Without either: a fixed descent thrust in landing states and hover thrust elsewhere.
- **Yaw:** holding a heading needs `heading`; otherwise zero yaw rate.

**Pilot modes** degrade the same way (DESIGN §4.9 rule c): a requested `Position` or `Offboard` without `position` becomes `Altitude`; without `altitude` it becomes `Stabilized`. The change is logged as an event.

**Rtl** needs home (set at arming) and `position`. Without them it behaves as `Hold`. Within 1 m of home it skips the climb and lands.

**Touchdown.** The vehicle is landed when, for 1 s, the commanded thrust is below 30 % of hover thrust and the estimated vertical speed is below 0.25 m/s. In `Rtl`, `Land` and `LandNow`, touchdown disarms the vehicle.

**On the ground.** A failsafe action that begins while the vehicle is landed disarms it at once. An armed vehicle sitting on the ground that loses its radio link must not take off to "return".

### 3. Failsafe events and default actions

A *reason* is a condition that is currently true. Each maps to an action, sometimes after a delay. Timings are fields of `FailsafeParams`.

| Reason | Condition | Applies in | Action |
|---|---|---|---|
| `RC_LOST` | No valid RC frame for `rc_timeout_us` (0.5 s) | Stabilized, Altitude, Position | `Hold`; after `rc_loss_hold_s` (3 s), `Rtl` |
| `NO_HUMAN_LINK` | RC lost, and the ground station not heard for `gcs_timeout_us` (3 s) or never connected | Offboard | Continue; after `no_link_rtl_s` (60 s), `Rtl` |
| `BRAIN_LOST` | Brain heartbeat's progress counter unchanged for `brain_timeout_us` (2 s) | Offboard | `Hold`; after `brain_loss_rtl_s` (10 s), `Rtl` |
| `SETPOINT_STALE` | No tier-2 setpoint for 0.5 s | Offboard | `Hold` |
| `BATTERY_LOW` | Filtered, load-compensated cell voltage below `battery_low_v` (3.5 V) for 2 s; latches | Armed | `Rtl` |
| `BATTERY_CRITICAL` | Same, below `battery_critical_v` (3.3 V); latches | Armed | `LandNow` |
| `GPS_LOST` | `position` invalid | Offboard, or while another failsafe is in force | `Hold`; after `gps_loss_land_s` (30 s), `Land` |
| `ESTIMATOR_UNHEALTHY` | `attitude` valid, but `altitude` or `heading` invalid, or the filter reported a numerical fault | Armed | `LandNow` |
| `IMU_FAILURE` | `attitude` invalid | Armed | `Terminated` |
| `GEOFENCE` | Further than `fence_radius_m` from home, or higher than `fence_altitude_m` above it. Zero disables that limit; both default to zero. | Armed, `position` valid | `Rtl` |
| `LOSS_OF_CONTROL` | Attitude error above 45° for 0.5 s, or tilt above 75° for 0.25 s | Armed, not landed | `Terminated` |

**Interpretations of DESIGN §4.10.**
- **RC loss and ground-station loss.** The table's rows overlap. Read together: in a pilot mode, losing RC alone starts Hold then Rtl, whatever the ground station does. In Offboard, losing RC alone changes nothing; losing every human link starts the `no_link_rtl_s` countdown. A ground station that never connected counts as absent, and losing it alone is never a failsafe.
- **GPS loss with a pilot flying.** A pilot in `Position` with valid RC who loses GPS drops to `Altitude` and stays in control (§4.9 rule c). The 30 s landing timer only runs when nobody is flying: Offboard, or another failsafe in force (which includes RC lost). A Hold that is only waiting out its hysteresis does not count; otherwise the lost position would keep the Hold alive after the pilot is back, and land the vehicle under them.
- **Brain loss.** Only the "no stored mission" branch exists (Hold, then Rtl). Missions stored on the core arrive in M3.
- **Estimator unhealthy and IMU failure.** With one IMU and one filter there is no lane to switch to. Losing height or heading while attitude still works is "estimator unhealthy" and lands now. Losing attitude is "IMU failure" and is terminal, as DESIGN §4.10 says for single-IMU boards.
- **Quad motor loss.** A quad cannot fly on three motors. The defined outcome is: the loss-of-control check fires, the event is logged, and the motors stop. A controlled spinning descent is later research.
- **Energy to get home.** The energy check on Rtl is not implemented yet. `BATTERY_CRITICAL` lands wherever the vehicle is, which bounds the risk.

### 4. Arbitration

- **Ranking:** `Terminated` > `LandNow` > `Land` > `Rtl` > `Hold` > none.
- **Highest wins.** The action is the highest-ranked one among the active reasons.
- **Escalation only.** Once the action reaches `Rtl` or higher it never drops while armed, even if every reason clears. A `Hold` ends when its reasons have been clear for `clear_hysteresis_s` (1 s), and the vehicle returns to the pilot's mode.
- **Reset.** Disarming clears everything except the battery latches, which clear only when the battery reads healthy again while disarmed.

### 5. Pilot override (DESIGN §4.9)

- **Kill switch** works at all times, as before.
- **A valid link first.** RC must have been valid for `rc_valid_min_s` (1 s) without a gap.
- **An explicit act.** The override is a *change* of the mode switch while a failsafe action is active. Moving sticks does nothing. RC returning does nothing on its own.
- **The switch position in effect** only updates while RC is settled. A switch moved while the link was down therefore counts as a change once the link has been back for `rc_valid_min_s`.
- **Effect.** The failsafe action ends and the vehicle enters the mode now selected, subject to degradation (§2). The reasons that were active are *acknowledged*: they do not act again until they clear and recur. A new reason, or an acknowledged one that escalates to a higher action, acts as normal.
- **Restricted.** While `BATTERY_CRITICAL`, `ESTIMATOR_UNHEALTHY` or `IMU_FAILURE` is active the pilot can only take over into `Stabilized` or `Altitude`.
- `Terminated` cannot be overridden.

### 6. Arming

Arming needs a rising edge of the arm switch with no pre-arm check failing. Each check has a stable bit in `PrearmFlags`, reported continuously in the status so a ground station can show why.

| Bit | Fails when |
|---|---|
| `KILL` | The kill switch is on or its latch is set |
| `THROTTLE` | Throttle above 10 % |
| `RC` | RC not valid for `rc_valid_min_s` |
| `NOT_STILL` | Body rate above 0.1 rad/s |
| `ESTIMATOR` | `attitude` or `heading` invalid, or `altitude` invalid |
| `POSITION` | The selected mode is Position or Offboard and `position` is invalid |
| `LINK` | The selected mode is Offboard and the brain heartbeat or the tier-2 setpoints are not live |
| `BATTERY` | No fresh battery reading, cell voltage outside 3.0–4.35 V, or `BATTERY_LOW` latched |
| `CONFIG` | The configuration failed validation |

A refused attempt logs the failing bits. Arming is edge-triggered: an attempt that is refused is not retried by holding the switch. On arming, home is set to the current position when `position` is valid. A vehicle armed without a position (Stabilized or Altitude, GPS not yet locked) gets its home at the first run where `position` becomes valid while armed; until then `Rtl` behaves as `Hold`. Home is never overwritten in flight and is cleared on disarm. After `Terminated`, arming stays blocked (the `KILL` bit) until the arm switch is released. DESIGN §4.11's remaining checks (HDOP and satellite count, log storage, RC calibration) arrive with the hardware that reports them.

### 7. Events

The core reports discrete events through its outputs: armed, disarmed and why, arming refused, navigation state changed, mode degraded, failsafe reason raised or cleared, failsafe action changed, pilot override, touchdown, home set. They carry the tick time. The simulator collects them, and the log will store them (M1c part three). "Logged" in DESIGN §4.10 means one of these events.

## Consequences

- The failsafe logic is a pure function of its inputs and a small state, with no clock of its own. That keeps it testable by property tests now and by Kani later.
- Every pairing of RC loss, GPS loss, battery low, battery critical, brain loss, geofence and estimator unhealthy gets a simulator scenario (DESIGN §4.10).
- Controllers must work on partial estimates. The position controller is split into a horizontal and a vertical part that can each run in position, velocity or open-loop form.

## Deferred

- Missions on the core, and "continue the mission" on brain loss (M3).
- The command gateway, strikes and link authority (M3).
- Excessive-vibration failsafe.
- In-flight restart after a core reset (DESIGN §4.16), with the firmware.
- The energy-to-home check for Rtl.
- IMU and estimator lanes on the H743.
