# ADR-0007: The criteria of benchmark gate 1

Status: accepted, before the run (DESIGN §8 rule 4). The gate has not been flown.

## Context

Gate 1 measures our flight core against ArduCopter in our simulator, on the reference quad (DESIGN §8). DESIGN gives the criteria as an example: zero crashes, position RMSE in the gust profile at most 1.0 times ArduCopter's, fault-recovery metrics no worse, over 500 seeds. This record makes them exact, so that the result cannot be argued into a pass or a fail afterwards.

How ArduCopter is flown is in [ADR-0006](0006-ardupilot-bridge.md). Four points were the owner's to decide and are recorded here with the reason for each:

1. **Tuning.** ArduCopter gets its standard procedure, automatic tuner included. Our core flies the gains it has today. DESIGN §8 rule 3 asked for our own automatic tuner, which is planned for M2 and does not exist. Waiting for it would put a large piece of work in front of the first measurement, and the choice made favours ArduCopter if it favours anyone.
2. **Faults are judged on the outcome only.** The two autopilots react differently to the same failure by design: on radio loss ours holds for 3 s and then returns, ArduCopter returns at once. Each uses its own response, and only what the physics shows is scored.
3. **Each flies on its own estimator and sensors, made as equal as they can be.** Our core is also flown on the true state, and that is reported but decides nothing.
4. **Both get our GPS error.** ArduCopter's simulated GPS has no horizontal position error and no setting for one, while our GPS model has 0.3 m per sample. A first look showed that this one difference would decide the gust criterion. The bridge therefore adds our GPS model's horizontal error to the position ArduCopter's GPS reads (ADR-0006 §4).

## Decision

### 1. Who flies what

- **Vehicle:** the reference quad, `airframes/quad-x-5in.toml`.
- **ArduCopter:** V4.7.2, the pinned binary of ADR-0006, at a 500 µs step, on its own estimator (EKF3), with the sensor settings of ADR-0006 §4.
- **Our core:** the commit the gate is run on, with the parameters that commit ships. Both chip profiles are flown. The **F405 profile decides**, because a bridged flight has the same wind as the F405 flight of the same seed, sample for sample (ADR-0006 §7). The H743 profile is reported and must meet criterion A1 as well.

### 2. How ArduCopter is tuned

In this order, in calm air on a full pack, by a script that is committed with the report:

1. `FRAME_CLASS 1`, `FRAME_TYPE 1`; `MOT_THST_EXPO` set to the airframe's thrust exponent; `MOT_SPIN_ARM`, `MOT_SPIN_MIN`, `MOT_SPIN_MAX` at their defaults; the battery monitor on; the filter settings of ArduPilot's tuning guide for a 5-inch propeller.
2. Starting gains low enough for a steady hover. The defaults oscillate on this airframe, and the tuner refuses a vehicle that is not steady. The values and how they were found are in the report. This is the manual step the guide puts before the tuner.
3. Hover-thrust learning in a hover.
4. A harmonic notch by the guide's throttle-based method, if a spectrum of the gyro samples of a hover shows a peak above the noise floor. If none does, no notch is set and the report shows the spectrum.
5. AUTOTUNE on roll, pitch and yaw, to its "Success" message, with the gains saved.

Everything the procedure does not touch stays at ArduCopter's default, the position controller included. The resulting parameter file is committed with the report. The report states the simulated flight time the procedure took.

Our core is not tuned for the gate. Its gains were set by hand during development on this airframe in this simulator, and the report says so.

### 3. ArduCopter's failsafes

Its failsafes are switched on, each with the action its documentation gives as the usual one, because a default of "warn only" is not a response:

| Failure | Setting |
|---|---|
| Radio lost | default (`FS_THR_ENABLE 1`: return) |
| Estimator unhealthy (GPS lost) | default (`FS_EKF_ACTION 1`: land) |
| Battery low | `BATT_FS_LOW_ACT 2` (return), at our low-voltage threshold |
| Battery critical | `BATT_FS_CRT_ACT 1` (land), at our critical threshold |
| Geofence | `FENCE_ENABLE 1`, a circle of the scenario's radius, `FENCE_ACTION 1` (return or land) |
| Ground station lost | `FS_GCS_ENABLE 1` (return) |

This file is committed with the report. No other parameter is set to make ArduCopter's response resemble ours.

### 4. The outcome of a flight, from the physics alone

Every flight of either autopilot is classed from the true state, by the same code:

- **Crash:** the vehicle meets the ground at more than 2.0 m/s downward, or tilts more than 60° while armed. Neither autopilot comes near these in a flight under control: ArduCopter limits its tilt to 30° and ours to 35° (45° at full stick in the stabilized mode), and our scenario files allow a touchdown of 1.5 m/s.
- **Fly-away:** the vehicle is more than 100 m from where it armed, or more than 60 m high, at any time. No scenario's own geometry comes near either.
- **Safe:** neither.

A flight that did not start (ArduCopter not ready to arm after the pre-roll, a dead process) is not an outcome. It is flown again once; if it fails to start again, the gate stops until the cause is known.

### 5. Criterion A: holding position in gusts

The scenario is `gust-hold`, seeds 0 to 499. The measure of a flight is the simulator's own: the RMS of the horizontal distance between the true position and the hold point, from 15 s to the end.

- **A1.** Every one of our core's 500 flights is safe, and passes the limits the scenario file already sets for the hold (among them a maximum horizontal error of 1.5 m, an RMS of 0.5 m and a tilt of 35°).
- **A2.** Over the 500 seeds, the **median** of our core's RMS error is at most 1.0 times the median of ArduCopter's, **and** the **95th percentile** of ours is at most 1.0 times ArduCopter's 95th percentile. Only ArduCopter's safe flights enter its statistics; its crashes and fly-aways are reported.

Reported beside it and deciding nothing: our core on the true state; the vertical error and the tilt of both; the same statistics for the H743 profile; the number of seeds in which each was the better.

### 6. Criterion B: failures

**Scored scenarios.** A scenario is scored when every failure in it can be applied to ArduCopter by a route of ADR-0006 §6 and its pilot can be translated. Each is flown on the seeds its file names (10). The list:

- Scored: `gps-loss`, `gps-loss-pilot`, `gps-loss-recovers`, `baro-loss`, `mag-loss`, `rc-loss-position`, `rc-loss-override`, `rc-loss-on-ground`, `rc-loss-offboard-gcs`, `gcs-loss`, `no-human-link`, `battery-low`, `battery-critical`, `geofence`, and every `pair-*` scenario whose two failures are both in this list.
- Scored on the time to motors off, not on the outcome: `motor-loss` and `imu-loss`. A quadrotor cannot fly on with either, and our core's designed response is to stop the motors.
- Scored only if the companion link of ADR-0006 §6 works: `brain-crash`, `brain-hang`, `setpoint-gap` and the pairs with a brain failure.
- Not scored: `kill-in-rtl` (the pilot's kill switch has no agreed ArduCopter counterpart). `hover-step` has no failure and is flown as a check of the bridge.

A scenario may leave the scored list only for a technical reason found while the bridge is built, written into this record before the run.

- **B1. No worse, scenario by scenario.** In every scored scenario, the number of our core's flights that end in a crash or a fly-away is at most ArduCopter's number.
- **B2. Zero crashes.** Our core has no crash and no fly-away in any scored scenario, and every one of its flights passes the scenario file's own expectations, which are stricter (a touchdown under 1.5 m/s, a tilt under 40°).
- **B3. Motors off.** In `motor-loss` and `imu-loss`, the median time from the failure to all motor commands at zero is at most ArduCopter's. A flight in which the motors never stop counts as the length of the flight.

Reported and deciding nothing: for each scenario and autopilot, the time from the failure to touchdown, the touchdown speed, the distance of the landing from the arming point, the greatest tilt, and the modes each went through.

### 7. The verdict

Gate 1 is passed when A1, A2, B1, B2 and B3 all hold.

Before the result is accepted, ten bridged flights chosen before the run (gust-hold seeds 0 to 4 and one seed of five failure scenarios) are flown twice. All ten must repeat bit for bit. If one does not, the result waits until the cause is known.

If the gate fails, the criteria stay. The core is improved and the whole gate is flown again. Changing a criterion takes a new record that gives the reason, before the next run.

### 8. The report

The gate report holds: the verdict, criterion by criterion; ArduCopter's parameter files and the tuning script; the binary's checksum and our commit; every flight's seed, outcome and measures as a table; the distributions as plots; what could not be made equal between the two (ADR-0006 §4), and which way each difference leans.

## Consequences

- DESIGN §8 rule 3 does not hold for gate 1: our core is not tuned by an automatic tuner of its own. DESIGN is changed to say so. The rule stands for gate 2.
- A pass says that our core, as it is, holds position in gusts at least as well as a tuned ArduCopter on equal sensors, and never ends a failure worse. It does not say that our tuning method is as good as ArduCopter's: that was not compared.
- ArduCopter's barometer and compass noise is one fixed sequence in every flight (ADR-0006 §7), so its 500 gust flights vary by wind, IMU noise and GPS error only.
- The outcome classes are coarse on purpose. They cannot say which autopilot chose the wiser response, only that neither response ended badly.

## Deferred

- PX4 (gate 2), and the tuning rule with our own automatic tuner (gate 2).
- Scoring the responses themselves (which action, how fast) and not only their outcomes.
- CPU and RAM (DESIGN §8 rule 5): measured on the chips, not in this gate.
