# ADR-0003: Failsafes, flight actions and arming

**Status:** Accepted (M1c) · **Date:** 2026-11-10 · **Design reference:** DESIGN §4.9, §4.10, §4.11

## Context

DESIGN §4.9–§4.11 say *what* the vehicle does when something fails: a table of events and default actions, a ranking, the pilot-override rules and the arming checks. This ADR turns that into a specification precise enough to implement and test, and records the places where the table had to be interpreted. It is also the specification for the failsafe state machine's proofs (DESIGN §9).

M1c is delivered in three parts. This ADR covers the first: failsafes and fault scenarios. The second, parameters and scenario files, is [ADR-0004](0004-parameters-and-scenario-files.md). Logging and replay follow.

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
- **Vertical:** height control needs `altitude`. Without it, climb-rate control still works while `velocity` is valid, because GPS keeps correcting vertical speed. Without either, a landing descends on the estimator's inertial vertical speed for 30 s: the barometer calibrated the accelerometer's vertical bias before it failed, so the speed drifts slowly and is far better than open-loop thrust for a short descent. Thrust is capped at hover in that phase, so a wrong speed can stall the descent but never climb. After 30 s the landing flies a fixed 90 % of hover thrust, and after 60 s minimum thrust. Outside landing states, with neither valid, the vehicle holds hover thrust.
- **Yaw:** holding a heading needs `heading`; otherwise zero yaw rate.

**When a sensor returns.** After a long gap the estimator resets its position, or its yaw, to the sensor, and the estimate jumps by the drift that built up. A setpoint kept from before the gap is then off by that drift, and the vehicle would fly, or turn, to it. So setpoints follow the estimate across the gap:
- **Position:** while `position` is not valid the held point follows the vehicle, and on the run where it is valid again the held point is the new estimate.
- **Resets:** the estimate carries the estimator's reset count. A change in it re-bases the held point and the yaw setpoint as well, because a sensor that returns within one guidance run of its timeout resets the estimate without guidance ever seeing the flag drop.
- **Yaw:** the yaw setpoint does the same for `heading`. `Stabilized` is the exception while the heading is away: it flies on the gyro-integrated yaw, which is smooth without a magnetometer, so the pilot keeps the yaw stick and the setpoint is re-based only at the return.
- **Height:** the height setpoint does the same for `altitude`, except the Rtl return height, which is a target that stays meaningful.

**Pilot modes** degrade the same way (DESIGN §4.9 rule c): a requested `Position` or `Offboard` without `position` becomes `Altitude`; without `altitude` it becomes `Stabilized`. The change is logged as an event.

**Rtl** needs home (set at arming) and `position`. Without them it behaves as `Hold`. Within 1 m of home it skips the climb and lands. An Rtl that has begun its landing never climbs again: if the position is lost on the way down it holds its height, and when the position returns it lands if it is within 1 m of home, and otherwise flies home at the height it is at and then lands.

**Touchdown.** The vehicle is landed when, for 1 s, the commanded thrust is below 30 % of hover thrust and the estimated vertical speed is below 0.25 m/s. In `Rtl`, `Land` and `LandNow`, touchdown disarms the vehicle. The vertical speed is known while `altitude` or `velocity` is valid. When it is not known, low thrust alone counts only in `Rtl`, `Land` and `LandNow`, where the controller lowers the thrust only on ground contact or at the end of a blind landing. Where the pilot commands the thrust (`Stabilized`, `Altitude`, `Position`), an unknown speed means not landed: a throttle chop in the air must never read as a landing.

**Ground contact.** A controller pushing against the ground does not wind its thrust down far enough for touchdown to be detected: the vertical loop's integral limit stops at about 40 % of hover, and the horizontal loop keeps demanding tilt, and with it thrust, to reach a point it cannot get to. Ground contact is therefore judged by motion, not thrust. When a descent of at least twice the touchdown speed is commanded (0.5 m/s with the default 0.25 m/s) and the vehicle has not moved vertically (below the touchdown speed) for 0.5 s, something is holding it up: in the air that command would have it moving within a quarter of a second. The landing speed must reach that threshold too, and a configuration with a slower landing is refused (§6). In contact, guidance commands minimum thrust and a level attitude, resets the position controller so nothing winds up, and moves the held point to where the vehicle is. Contact ends as soon as the vehicle moves. This applies in the landing states and when the pilot holds the throttle down in `Altitude` or `Position`.

**On the ground.** A failsafe action that begins while the vehicle is landed disarms it at once. An armed vehicle sitting on the ground that loses its radio link must not take off to "return". For this rule alone "landed" is wider than touchdown: a pilot state with thrust below half of hover and no vertical speed for 0.2 s also counts. A vehicle in the air at half thrust is accelerating downwards, so it cannot meet that test.

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
| `LOSS_OF_CONTROL` | Tilt error (the angle between the estimated and the commanded thrust axis; the heading is not counted, see the amendment) above 45° for 0.5 s, or tilt above 75° for 0.25 s | Armed, not landed | `Terminated` |

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

- **Kill switch** works at all times. Its position takes effect after three valid frames in a row show it (§6).
- **A valid link first.** RC must have been valid for `rc_valid_min_s` (1 s) without a gap.
- **An explicit act.** The override is a *change* of the mode switch while a failsafe action is active. Moving sticks does nothing. RC returning does nothing on its own.
- **The switch position in effect** only updates while RC is settled. A switch moved while the link was down therefore counts as a change once the link has been back for `rc_valid_min_s`.
- **Effect.** The failsafe action ends and the vehicle enters the mode now selected, subject to degradation (§2). The reasons that were active are *acknowledged*: they do not act again until they clear and recur. Only reasons that were already active before the run of the switch change are acknowledged; one raised on that very run is new. A new reason, or an acknowledged one that escalates to a higher action, acts as normal.
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
| `CONFIG` | A parameter fault is held: the stored parameters were not fully understood at start-up, and no operator has acknowledged it (ADR-0004 §6) |

**Switch confirmation.** The kill switch and the arm switch take effect only after three distinct valid frames in a row show the new position. The supervisor examines each frame once, by its arrival time; it runs at 50 Hz and sees the newest frame of each run. A valid frame showing the other position starts the count again. A frame flagged failsafe or frame-lost is no evidence either way. With no fresh frame at all the counts start again. So one corrupted frame can neither stop the motors in flight, nor disarm, nor arm on the bench, at a cost of about 40 to 60 ms. Arming is the rising edge of the confirmed arm position, and the switch disarms when the confirmed position goes low. Releasing the kill latch or the termination latch needs only one valid frame with both switches off: that can only make arming possible again, and arming still needs its own confirmed edge.

**Configuration.** The core refuses to start with an invalid configuration (`Core::new` returns the error), so an invalid configuration never runs. A parameter change at run time goes through the same validation: one that would break a rule is refused and reported by an event, and the configuration stays as it was (ADR-0004 §3 and §4). The `CONFIG` bit is set only while a parameter fault is held (ADR-0004 §6). Validation covers every failsafe parameter, because a bad one silently disables a protection or makes a landing that never ends:
- every value finite;
- link timeouts between 50 ms and an hour, and delays at most an hour;
- battery thresholds within 3.0 to 4.0 V per cell (the range in which the monitor can act and its latch can clear on a full pack), with critical not above low;
- fence limits not negative (0 turns the fence off);
- loss of control above what the controller itself commands, or a normal manoeuvre switches the motors off: the tilt threshold at least 10° above both tilt limits, the attitude-error threshold at least 20°, and the two timers at least 0.1 s and 0.05 s;
- return speed at least 0.1 m/s, landing speed at least twice the touchdown speed, and the descent limit at least twice the touchdown speed too, because it clamps the landing descent (a slower descent reads as standing on the ground);
- touchdown speed at least 0.1 m/s, because the estimated vertical speed of a landed vehicle is noisy (up to 0.09 m/s in the simulator) and a lower threshold never detects the ground; touchdown time between 0.1 s and 10 s;
- the minimum thrust below the touchdown thrust, or a vehicle on the ground could never read as landed; the blind-landing thrust above it, or a blind landing would read as landed in the air;
- thrust fractions within their range, at least one battery cell, resistance not negative;
- the rotors together can lift the vehicle (this catches a mass in grams);
- the idle motor command gives at most half the hover thrust of a rotor: it is a floor on every motor at all times, so a higher one makes the vehicle climb away;
- the estimator's output time constant at least 0.15 s and not below its delay horizon (measured: below about 0.7 of the horizon the estimate oscillates and the flight is lost);
- the gyro filter's cut-off below 0.45 of the rate-loop frequency, where the filter would otherwise switch itself off.

The rules from the loss-of-control line onwards were added in part two, after a review flew every parameter at both ends of its range. No limit in the core is applied with a clamp that can panic when its bounds are crossed.

A refused attempt logs the failing bits. Arming is edge-triggered: an attempt that is refused is not retried by holding the switch. On arming, home is set to the current position when `position` is valid. A vehicle armed without a position (Stabilized or Altitude, GPS not yet locked) gets its home at the first run where `position` becomes valid while armed; until then `Rtl` behaves as `Hold`. Home is never overwritten in flight and is cleared on disarm. After `Terminated`, arming stays blocked (the `KILL` bit) until the arm switch is released. DESIGN §4.11's remaining checks (HDOP and satellite count, log storage, RC calibration) arrive with the hardware that reports them.

### 7. Events

The core reports discrete events through its outputs: armed, disarmed and why, arming refused, navigation state changed, mode degraded, failsafe reason raised or cleared, failsafe action changed, pilot override, touchdown, home set. Part two added the parameter events: a parameter changed or refused, a load of stored parameters that was not fully understood, and its acknowledgement (ADR-0004). They carry the tick time. The simulator collects them, and the log will store them (M1c part three). "Logged" in DESIGN §4.10 means one of these events.

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
- Switch confirmation on every link frame. The supervisor samples the newest frame at 50 Hz, so on a faster link the frames between two runs are not examined. Counting in the input path would examine all of them.

## Amendment: loss of control is judged on the thrust axis, not on the heading

**Date:** 2026-11-28 · **Changes:** §3 (the `LOSS_OF_CONTROL` row), `crates/oxide-core/src/monitor.rs`, the descriptions of `FS_LOC_ERR` and `FS_LOC_ERR_T`, DESIGN §4.10.

### The fault

The rule of §3 measured the whole rotation between the estimated and the commanded attitude, heading included. A heading setpoint that jumps far enough keeps the heading error above 45° for longer than 0.5 s, because the vehicle turns at its yaw rate limit. A healthy vehicle was then terminated in flight by an ordinary command. Hover at 5 m in Offboard with realistic sensors, then one heading setpoint of 2.5 rad: `ReasonRaised(LOSS_OF_CONTROL)`, `Terminate` and `Disarmed(Terminated)` 0.52 s after the step, and the vehicle fell and met the ground at 9.2 to 9.5 m/s (the same at 3.0 rad; at 2.0 rad it flew on). It was found by a check of the auto-tune experiments, which flew 6 of 6 flights into it on any gains. No scenario file commanded a heading step this large, so no test saw it. The new `yaw-step-large` flies it (2.5 rad and 3.14 rad, and back): before this change all 40 flights of runs 500 to 519 on both profiles were terminated 0.52 s after the first step; after it none is, and their largest tilt is 14.5°.

### The decision

**Loss of control is judged on the direction of the thrust axis.** The error of the rule is the angle between the estimated and the commanded thrust axis (the body's down axis in each attitude): the *tilt error*, `tilt_error_angle` in `monitor.rs`, computed as the arctangent of the norm of the cross product over the dot product of the two axes, so it is exact near 0 and near π. A heading error alone no longer raises `LOSS_OF_CONTROL`. The figures stay: 45° for 0.5 s on the tilt error, 75° for 0.25 s on the tilt, the parameters `FS_LOC_ERR`, `FS_LOC_ERR_T`, `FS_LOC_TILT`, `FS_LOC_TILT_T` with their ranges and cross-field rules, and the action `Terminated`. A non-finite attitude or target still counts as over both limits. The old function `attitude_error_angle` is gone: it returned the whole rotation, and nothing else used it.

### The grounds

- **A multirotor that cannot point its thrust where it is commanded has lost control; one that is turning to a new heading has not.** Its thrust axis, and with it the height and the position, stay under control through a heading step. Switching the motors off in flight guarantees a crash, so the rule must not fire on a healthy vehicle. After the change the 40 heading-step flights of runs 500 to 519 pass the scenario's own limits (horizontal error below 0.8 m, tilt below 30°); the largest tilt is 14.5°.
- **PX4** (BSD-3, checked out at commit `935addf1d6bec6e955b155494a52e7eb4b750eb3`; read, not copied). Its failure detector, `src/modules/commander/failure_detector/FailureDetector.cpp`, `updateAttitudeStatus` (lines 100 to 146), takes the Euler angles of the estimated attitude (`const matrix::Eulerf euler(matrix::Quatf(attitude.q));`, line 106) and judges only roll and pitch: `const bool roll_status = (max_roll > FLT_EPSILON) && (fabsf(roll) > max_roll);` (line 131) and the same for pitch (line 132). Yaw is never read, and neither is the attitude setpoint: PX4 compares the absolute roll and pitch with fixed limits. The limits are `FD_FAIL_R` and `FD_FAIL_P`, 60° by default, with `FD_FAIL_R_TTRI` and `FD_FAIL_P_TTRI`, 0.3 s by default (`failure_detector_params.yaml`, lines 5 to 20 and 37 to 47: "Maximum roll angle before FailureDetector triggers the attitude_failure flag. The flag triggers flight termination (if @CBRK_FLIGHTTERM = 0), which sets outputs to their failsafe values."; "Seconds (decimal) that roll has to exceed FD_FAIL_R before being considered as a failure."). So PX4 judges the direction of the thrust axis (through roll and pitch) and ignores the heading. It does not look at the error to the command; our tilt rule (75° for 0.25 s) is its counterpart, and the error rule is the part PX4 does not have.
- **ArduPilot** (behaviour reference only; no source read). Its crash check, as recorded in [ADR-0009](0009-arducopter-tuning.md), disarms "in cases where the vehicle is likely out of control and has hit the ground", when the lean angle is more than 30° from the wanted one and the vehicle accelerates by less than 3 m/s², for 2 s. A lean angle is the direction of the thrust axis, so this too is a tilt error against the command and says nothing of the heading.

Both references judge the thrust axis and neither counts the heading; ours now judges the thrust axis against the command, as ArduPilot's does and PX4's does not.

### Not chosen

- **Limiting how fast the heading target may move**, so that the error never grows large. It would change every heading step the controller flies and every pinned flight with one, and it treats the symptom: the rule would still count heading. It may be worth doing later for its own sake.
- **A longer time or a larger angle.** It only moves the size of the step that kills the vehicle.
- **A softer reaction to a heading error that persists** (for example a landing). Possibly right, but a new rule: deferred.

### Deferred

- A reaction to a heading error that persists, if one is wanted (a vehicle that cannot turn, for example on a lost yaw authority). Today a vehicle that does not follow its heading command raises nothing.
- A limit on the speed of the heading target (see above).

### Measurements

All on runs 500 to 519 of every scenario file, both profiles, before and after, comparing the class, the verdict, the event timeline and the trace hash of each of the 1760 flights.

- **Only loss-of-control flights changed.** 78 flights differ: the 38 of `motor-loss` that were terminated before and are terminated after (2 of the 40 are bit for bit the same), and the 40 flights of the new `yaw-step-large`, terminated before and flown clean after. The other 42 files, 1680 flights, are unchanged to the trace hash. (`imu-loss` fails its verdict on these runs before and after, identically: a finding outside this change.)
- **Motor loss is still terminated, a little later.** Time from the fault (at 15 s) to the last motor command above zero, runs 500 to 519 (the criterion B3 measure of gate 1):

  | profile | median before | median after | latest before | latest after |
  |---|---|---|---|---|
  | f405-lean | 0.999 s | 1.239 s | 0.999 s | 1.239 s |
  | h743-full | 1.000 s | 1.240 s | 1.000 s | 1.240 s |

  The scenario's own limit (terminated by 17 s, a fault at 15 s) holds on every run; the latest is 1.24 s after the fault. The reason is raised 0.98 s after the fault before, and 0.98 s to 1.22 s after (38 of the 40 flights are later): the heading part of the error no longer counts toward the limit. The median moves by 0.24 s.
- **Gate 1.** Its reports in `docs/gates/` are records and stand as flown: the motors-off figures of our core there (0.999 s on F405, 1.0 s on H743 in `motor-loss`) were flown on the old rule. On the new rule the medians are 1.239 s and 1.240 s. ArduCopter's figure and the verdict of B3 are not touched: our core's median is still far below ArduCopter's 14.9995 s.
- **Pins.** The trace hashes of the three pinned `motor-loss` flights (and nothing else) moved, and the three of `yaw-step-large` are new. The ten `oxide-sim hash` lines: the two `motor-loss` lines moved, the other eight did not.
