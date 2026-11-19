# ADR-0007: The criteria of benchmark gate 1

Status: accepted, before the run (DESIGN §8 rule 4). The gate was flown on 18 November 2026 at commit `2c2fedb` and was **not passed**: criterion A2 failed on the 95th percentile. After one change to the estimator, found on other seeds under the rules below, it was flown again on 19 November 2026 at commit `92fdaa2` and **passed**. Both runs are recorded at the end of this record; the reports are [`docs/gates/gate-1/`](../gates/gate-1/README.md) and [`docs/gates/gate-1/run-2/`](../gates/gate-1/run-2/README.md). The criteria did not change.

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
4. A harmonic notch by the guide's throttle-based method, if a spectrum of the gyro samples of a hover shows a peak above the noise floor. If none does, no notch is set and the report shows the spectrum. The rule for "a peak", fixed before the spectrum was taken: a line between 40 Hz and 400 Hz, on any axis, whose power is at least ten times the median power of that band.
5. AUTOTUNE on roll, pitch and yaw, to its "Success" message, with the gains saved.

The guide's values for this vehicle (a 5-inch propeller, six cells): `INS_GYRO_FILTER` 80, the rate controllers' target and derivative filters at half of it, the yaw error filter 2, `INS_ACCEL_FILTER` 10, `MOT_BAT_VOLT_MAX` 25.2 and `MOT_BAT_VOLT_MIN` 19.8 (`crates/oxide-bridge/arducopter/setup.parm`; each of the ten lines changes a default of the program and is read back from it). The tuner runs from ALT_HOLD in calm air with the binary's own settings. On V4.7.2 these read `AUTOTUNE_AGGR` 0.075, `AUTOTUNE_MIN_D` 0.0005 and all three axes. ArduPilot's AutoTune page names 0.1 as the default aggressiveness; the binary's own value is the one used.

**The tuning flight** is one flight by the committed script (`oxide-bridge tune`, seed 0), with the gate's sensing (ADR-0006 §4) and radio (ADR-0006 §6). The pilot arms in ALT_HOLD, climbs to about 10 m and hovers for 60 s with the sticks centred: the hover thrust is learned, and the last 30 s of gyro samples give the spectrum. The pilot then switches to AUTOTUNE, leaves the sticks centred for 480 s and lands: the throttle stick at 0.3, which ArduCopter reads as a slow descent, until the vehicle is on the ground (a true height of 0.05 m or less), then fully low. The vehicle disarms in AUTOTUNE, which is how the program saves its gains. The gains are read back and written to the tuned file. The script decides everything from simulated time, the true height and the motor pulses. What it reads from MAVLink goes into the result file only (and, if a notch were set, the learned hover thrust would go into the notch's reference).

The result is accepted only if all of this holds: every line of the set-up file is read back from the program with the file's value before the flight; each of the three axes reports complete; the tuner reports success and saved gains; every value is read back; the flight is safe by §4's class; and the pack voltage under load, averaged over the last second before the descent begins, is above the low-battery threshold of §3. If one does not hold, no tuned file is committed and the gate is not flown.

From then on every bridged flight loads, in this order: the base file, the set-up file, the tuned file, the failsafes of §3. The starting gains are flown by the tuning flight only.

Three things in this procedure were changed while the script was built, after measurements and before any tuned file existed:

- **The position check.** It was first a box of 20 m out and 30 m up, a figure of ours. Measured: ALT_HOLD has no position hold, and with centred sticks in calm air the vehicle wanders at 0.15 to 0.2 m/s. It was 20 m out after 135 s (66 to 165 s on four other seeds) and 35 m out at most in a whole flight in which the tuner reported success on every axis. That is what the mode does, not a tuning gone wrong, and ArduPilot's AutoTune page asks for a large open area. The check is now §4's class, the limits every gate flight is held to. Not chosen: starting the tuner from LOITER, which makes it hold position weakly (the same page says a tune from AltHold is often the better one), and a pilot who steers back (allowed by the page, but every figure of such a pilot would be ours).
- **The wait in the tuner.** It was first 900 s. The airframe's pack is empty about 650 s into such a flight. The tuner needs about 440 s and ends near 525 s of flight time, with about a fifth of the pack left. With 900 s the vehicle landed on a pack that had been empty for five minutes, and the hover thrust it saved was an artefact of that (0.182, against 0.149 learned in the hover). The wait is 480 s, and the pack check above was added. Measured with 480 s: the tuner reports success after 439 s, the pack is at 21.7 V under load with 14 % of its charge when the descent begins, and the saved hover thrust is 0.150.
- **The landing.** The first script lowered the throttle fully at once. ArduCopter then descends at its full pilot rate, 2.5 m/s, all the way down: the vehicle met the ground at 2.48 m/s, which §4 classes as a crash, and the new position check refused the flight. No pilot lands like that. The script now descends with the stick at 0.3 and lowers it fully on the ground. Everything else in that flight held: it stayed within 27 m and 11 m of where it armed and tilted 16° at most. Measured with the new landing: the descent from 10.5 m takes 15.8 s at 0.67 m/s on average, and the vehicle meets the ground at 0.73 m/s.

**The tuning as flown.** With the three changes in place the procedure's flight was accepted, and its file is committed (`crates/oxide-bridge/arducopter/tuned-gains.parm`). The flight took 597 s of simulated time. The tuner started at 85.8 s and reported roll complete at 185.2 s, pitch at 277.1 s, and yaw and success at 524.6 s, 439 s after it started. The descent began at 565.8 s with the pack at 21.655 V under load (the threshold is 20.767 V) and 14.5 % of its charge; the vehicle was on the ground at 581.6 s and disarmed itself at 583.4 s, still in AUTOTUNE, with the program's message that the gains were saved. It was never more than 28.8 m from where it armed or more than 10.65 m high, and tilted 15.6° at most. A second run of the script, on another instance of the program, writes the same file byte for byte, and a test flies that comparison.

| Parameter | Before the tuner | After the tuner |
|---|---|---|
| `ATC_RAT_RLL_P`, `_I` | 0.05 | 0.0547 |
| `ATC_RAT_RLL_D` | 0.0012 | 0.00117 |
| `ATC_RAT_PIT_P`, `_I` | 0.05 | 0.0549 |
| `ATC_RAT_PIT_D` | 0.0012 | 0.00123 |
| `ATC_RAT_YAW_P` | 0.065 | 0.0483 |
| `ATC_RAT_YAW_I` | 0.0065 | 0.00483 |
| `ATC_RAT_YAW_FLTE` | 2 | 5 |
| `ATC_ANG_RLL_P` | 4.5 | 25.4 |
| `ATC_ANG_PIT_P` | 4.5 | 27.75 |
| `ATC_ANG_YAW_P` | 4.5 | 1.96 |
| `ATC_ACC_R_MAX`, `ATC_ACC_P_MAX` | 1100 | 4875, 5009 |
| `ATC_ACC_Y_MAX` | 270 | 206 |
| `MOT_THST_HOVER` | 0.16 | 0.150 |

The file holds every digit; the other values of the 29 it lists did not change.

**The file is flown as the tuner saved it.** The tuner raised the roll and pitch angle gains about sixfold. ArduPilot's AutoTune page says of this: "AutoTune will attempt to tune each axis as tight as the aircraft can tolerate. In some aircraft, this can be unnecessarily responsive." It gives a guide for most aircraft (angle gains above 10 brought down to 6) and adds that small aerobatic aircraft may prefer to keep them as high as they are. The reference vehicle is a small aerobatic one: a 5-inch quad that hovers at 15 % of its throttle. Nothing is changed by hand after the tuner, so no figure in ArduCopter's file is ours. This was written after the first looks below had been flown with the file. On them the choice does not lean against ArduCopter: in the 100 gust runs it did slightly better with the tuned file than with the starting gains, whose angle gains are 4.5, and no flight's class changed.

The spectrum of the hover (57 averaged segments of 1024 samples; a single unaveraged spectrum of noise has lines above ten times the median by chance) showed its largest line at 1.4 to 1.6 times the band's median on every axis. By the rule of step 4 there is no peak and no notch is set. Our IMU model has no line at the motors' speed (ADR-0006, deferred).

Everything the procedure does not touch stays at ArduCopter's default, the position controller and the tuner's own settings included. The resulting parameter file is committed with the report. The report states the simulated flight time the procedure took.

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

The file is `crates/oxide-bridge/arducopter/failsafes.parm`; the runner adds the two battery thresholds and, for a scenario with a fence, the fence. No other parameter is set to make ArduCopter's response resemble ours.

**The battery thresholds.** Our core compares the pack voltage with the load taken out (voltage plus current times the pack's resistance) against 3.5 V and 3.3 V per cell. ArduCopter compares the voltage as it sags under load. So that both act at the same state of the pack, ArduCopter's thresholds are the loaded pack voltages at which our compensated voltage reaches ours in a hover: `BATT_LOW_VOLT` 20.767 V and `BATT_CRT_VOLT` 19.553 V on this pack, not 21.0 and 19.8 (derived twice, by the code and by hand from our core's own flights). The timers stay each autopilot's own: ArduCopter acts 10 s after the crossing, our core after about 4 s of filtering and debounce.

**Parameters in a scenario file** are our core's and never reach ArduCopter as they are. `FS_FENCE_RAD` becomes ArduCopter's circle fence of that radius. `FS_GPS_LAND` and `FS_NOLINK_RTL` set the timing of our core's own responses and have no counterpart; they are listed in the flight's record as ours only. A scenario with any other parameter is not flown with ArduCopter.

### 4. The outcome of a flight, from the physics alone

Every flight of either autopilot is classed from the true state, by the same code:

- **Crash:** the vehicle meets the ground at more than 2.0 m/s downward, or tilts more than 60° while armed. Neither autopilot comes near these in a flight under control: ArduCopter limits its tilt to 30° and ours to 35° (45° at full stick in the stabilized mode), and our scenario files allow a touchdown of 1.5 m/s.
- **Fly-away:** the vehicle is more than 100 m from where it armed, or more than 60 m high, at any time. No scenario's own geometry comes near either.
- **Safe:** neither.

Made exact when the code was written (`FlightOutcome::class` in the simulator, the same for both autopilots): every time the vehicle comes down onto the ground counts, not only the first, and whatever height it had reached before; a flight whose simulated state stops being a number, or in which the physics reports a fault, is a crash; a flight that is both a crash and a fly-away is a crash. The tilt counts only while armed, which for ArduCopter means while any motor output is above 1000 µs. A vehicle that tumbles after its motors have stopped is caught by how it meets the ground.

A flight that did not start (ArduCopter not ready to arm after the pre-roll, a dead process) is not an outcome. It is flown again once; if it fails to start again, the gate stops until the cause is known.

The scenario files' own expectations are a second, stricter judgement that was written for our core. For ArduCopter, the ones that need our core's states are not judged, and the ones that can be judged from the physics describe our core's designed response (land at home, be disarmed by a given time). ArduCopter is not scored on them: it is scored on the class. In the runner's table its verdict therefore reads "not judged" or "fail" where our core's reads "pass", and nothing of the gate selects ArduCopter's flights by that column.

### 5. Criterion A: holding position in gusts

The scenario is `gust-hold`, runs 0 to 499 (the file itself names 200, its acceptance count from an earlier milestone; the gate flies 500). The measure of a flight is the simulator's own: the RMS of the horizontal distance between the true position and the hold point, from 15 s to the end.

- **A1.** Every one of our core's 500 flights is safe, and passes the limits the scenario file already sets for the hold (among them a maximum horizontal error of 1.5 m, an RMS of 0.5 m and a tilt of 35°).
- **A2.** Over the 500 seeds, the **median** of our core's RMS error is at most 1.0 times the median of ArduCopter's, **and** the **95th percentile** of ours is at most 1.0 times ArduCopter's 95th percentile. Only ArduCopter's safe flights enter its statistics; its crashes and fly-aways are reported. The 95th percentile is taken by linear interpolation between the two nearest ordered values (position 0.95 × (n − 1), counted from 0), and the comparison is made on the values as computed, without rounding. The median of an even number of values is the mean of the two middle ones, computed as (a + b) / 2. If ArduCopter had no safe flight there would be nothing to compare with, and A2 would not hold.

Reported beside it and deciding nothing: our core on the true state; the vertical error and the tilt of both; the same statistics for the H743 profile; the number of seeds in which each was the better.

ArduCopter starts later than our core in this scenario. It holds its motors still for 2.0 s after it arms, so its motors come alive 2.2 s after the arm switch (at 5.2 s; our core's at 3.1 s), and it is at the hold point at about 8.7 s (our core at 5.3 s). Both are there well before the 15 s from which the error is measured (measured on runs 0 to 2 with the bridge). The scenario file's limit that the vehicle be armed from 3.5 s on is our core's own and is not judged for ArduCopter.

### 6. Criterion B: failures

**Scored scenarios.** A scenario is scored when every failure in it can be applied to ArduCopter by a route of ADR-0006 §6 and its pilot can be translated. Each is flown on the seeds its file names (10). The list:

- Scored: `gps-loss`, `gps-loss-pilot`, `gps-loss-recovers`, `baro-loss`, `mag-loss`, `rc-loss-position`, `rc-loss-override`, `rc-loss-on-ground`, `rc-loss-offboard-gcs`, `gcs-loss`, `no-human-link`, `battery-low`, `battery-critical`, `geofence`, and every `pair-*` scenario whose two failures are both in this list.
- Scored on the time to motors off, not on the outcome: `motor-loss` and `imu-loss`. A quadrotor cannot fly on with either, and our core's designed response is to stop the motors.
- Scored, because the companion link of ADR-0006 §6 works (proven on the binary when the bridge was built): `brain-crash`, `brain-hang`, `setpoint-gap` and the pairs with a brain failure.
- Not scored: `kill-in-rtl` (the pilot's kill switch has no agreed ArduCopter counterpart). `hover-step` has no failure and is flown as a check of the bridge.
- Four scored scenarios have a pilot who moves the sticks: `gps-loss-pilot`, `rc-loss-position`, `rc-loss-override` and `rc-loss-on-ground`. Their stick scripts were written for our core, whose motors run at once when it arms: they raise the throttle 0.5 s after the arm switch. ArduCopter keeps its motors still for 2.0 s after it arms, and a throttle raised and centred again in that time lifts nothing. The rule, fixed before these were flown: **for ArduCopter, every event of the pilot later than the arm switch (stick frames and mode-switch changes) is delayed by 2.24 s**, the measured time from the switch to live motors rounded up to a radio period. The arm switch itself, the brain's setpoints and every failure keep their times. The pilot then does for ArduCopter what the script means: raise the throttle once the motors run.
- The same stick means each autopilot's own speed. Measured, one stick at a time: for the same roll, pitch or yaw stick ArduCopter flies 1.65 to 1.8 times as far and turns 1.8 times as fast as our core, and for the same throttle stick it climbs 0.73 times as fast. These are its defaults for how the sticks feel, and ours are our core's. Nothing is scaled. When the failure starts ArduCopter is therefore further out and lower than our core (`rc-loss-position` at 25 s: 24.9 m out and 3.2 m up, against 14.1 m and 4.5 m), so it returns from further away. Both are far inside the limits of §4. An expectation of a file that depends on this, such as the 4 m height `gps-loss-pilot` asks for, is one of those ArduCopter is not scored on (§4).

A scenario may leave the scored list only for a technical reason found while the bridge is built, written into this record before the run.

- **B1. No worse, scenario by scenario.** In every scenario scored on its outcome, the number of our core's flights that end in a crash or a fly-away is at most ArduCopter's number.
- **B2. Zero crashes.** Our core has no crash and no fly-away in any scenario scored on its outcome, and every one of its flights passes the scenario file's own expectations, which are stricter (a touchdown under 1.5 m/s, a tilt under 40°).
- **B3. Motors off.** In `motor-loss` and `imu-loss`, the median time from the failure to all motor commands at zero is at most ArduCopter's. A flight in which the motors never stop counts as the length of the flight. Exactly: the time of the last motor command above zero, less the failure's start; when the motors never stop that is the rest of the flight. The median of the ten flights is the mean of the two middle values, computed as (a + b) / 2, and each of the two scenarios must hold.

Reported and deciding nothing: for each scenario and autopilot, over its runs, the touchdown speed, the distance of the end position from the arming point, the greatest tilt and the time of the disarm (each as a median and a worst value, with the number of flights that touched down and that disarmed). The modes each flight went through are in that flight's own record. This list first named the time from the failure to touchdown; the runner's tables hold no touchdown time, so the report gives the time of the disarm, and each scenario file names when its failure begins. Changed before the gate was flown; it decides nothing.

### 7. The verdict

Gate 1 is passed when A1, A2, B1, B2 and B3 all hold.

Before the result is accepted, ten bridged flights chosen before the run are flown twice: gust-hold runs 0 to 4, and run 0 of `gps-loss`, `battery-low`, `rc-loss-offboard-gcs`, `imu-loss` and `pair-gps-loss+baro-loss`. All ten must repeat bit for bit. If one does not, the result waits until the cause is known: the program then gives no verdict, which is not the same as a gate that was not passed.

The verdict is computed from the runner's tables by a committed program, not by hand (`oxide-bridge gate`). The program flies the gate only from a clean commit, the one it was built at. It gives no verdict when a planned flight is missing or doubled, when a row's seed is not the seed of its scenario and run, or when the tables come from more than one commit, binary or parameter set; and it never passes a plan that is not this record's (fewer runs, a scenario left out, a trial). `hover-step` is flown on the one run its file names. A program that was not built at a commit gives no verdict either: one built on a tree with changes to tracked files, or outside the repository. Files that git does not track do not count, for the program as for the flying. A pair scenario is scored by the failures its file holds, not by its name; one with a failure that is not scored on its outcome stops the gate before it is flown. The ten flights flown twice are extra flights beside the tables: both of each pair must agree with each other and with the same flight in the tables, read from the two flights' own records.

The scenario files and ArduCopter's parameter files that are flown are the commit's own. The program carries their checksums from the commit it was built at; it refuses to fly others as the gate, and it gives no verdict on tables flown with others or at another commit than its own. The plan is the commit's whole set of scenario files: a file left out or added makes the run a trial, which never passes. Every row of a table must equal the flight's own record in its run directory, and a value that cannot be (a negative error or time, a word where a number belongs) is no verdict, in any column. The flight's own record is two files, and the row is held against both: the row the flight wrote, cell for cell, and the flight's audit, for every number of the row, the counts of violated and unjudged expectations with the verdict word they make, the class, the seed and the run, and for ArduCopter the hashes and the directory its simulator was started in. The second flights of the ten flown twice must be the records of their own directories: a copy of the first flight's record is not a repeat.

What the program cannot see is a set of records rewritten together, or a changed program. Against those stands that every flight repeats bit for bit: anyone can fly the gate again from the commit with the pinned binary and compare the hashes in the tables.

If the gate fails, the criteria stay. The core is improved and the whole gate is flown again. Changing a criterion takes a new record that gives the reason, before the next run.

### 8. The report

The gate report holds: the verdict, criterion by criterion; ArduCopter's parameter files and the tuning script; the binary's checksum and our commit; every flight's seed, outcome and measures as a table; the distributions as plots; what could not be made equal between the two (ADR-0006 §4), and which way each difference leans.

## First looks taken before the run

While the bridge was built, flights were flown that show how the two compare. They are recorded here so that nobody has to wonder what was known when. The criteria above were written before them and are unchanged by them.

- **Gusts, 100 runs** (gust-hold, indices 0 to 99, ArduCopter on its hand-set starting gains, before its automatic tuner): our F405 core's RMS error had a median of 0.170 m and a 95th percentile of 0.221 m; ArduCopter's 0.176 m and 0.222 m. Our core had the lower error in 57 runs. All 200 flights were safe. The two are close, and the tuned ArduCopter may come out either side.
- **Failures, run 0 of each file that could then be flown** (38 of 43; the five with a pilot on the sticks came later, below): ArduCopter ended 30 safe, 4 in a fly-away (`baro-loss` and three pairs with it: with every barometer gone it climbs without limit) and 4 in a crash (`imu-loss`, `motor-loss`, and two pairs with a barometer loss that landed at 3.1 and 2.6 m/s). Our core ended every flight that is scored on its outcome safe; in `imu-loss` and `motor-loss`, which are scored on the time to motors off, it stopped its motors 0.14 s and 1.0 s after the failure, ArduCopter 2.0 s after and never.
- **The five files with a pilot on the sticks, run 0** (ArduCopter on its starting gains): `rc-loss-position`, `rc-loss-override`, `rc-loss-on-ground` and `gps-loss-pilot` ended safe. `kill-in-rtl`, which is not scored, ended in a crash for both autopilots, as a kill switch at 10 to 15 m must.
- **A trial of the gate program** (ArduCopter on its starting gains; gust-hold cut to 6 runs, every other file on all of its runs: 417 flights for each autopilot). In the 38 scenarios scored on their outcome, all 380 flights of our F405 core were safe and passed their files' expectations. Of ArduCopter's 380, 60 were not safe: all ten runs of six of the seven files with a barometer loss (not `pair-rc-loss+baro-loss`). Time to motors off, median of ten: `motor-loss` 1.0 s for our core and never for ArduCopter; `imu-loss` 0.14 s and 2.0 s. All ten flights flown twice repeated. So B1, B2 and B3 held on these flights, with ArduCopter not yet tuned. Criterion A cannot be read from six runs.
- **A tuned file that the procedure did not accept.** While the tuning script was built, one tuning flight was flown with the position check switched off; it also landed on the empty pack described in §2. Its file was flown as information: in the same 100 gust runs ArduCopter's median was 0.1759 m and its 95th percentile 0.2214 m (0.1761 m and 0.2218 m on the starting gains; our core 0.1700 m and 0.2211 m), and run 0 of all 43 files ended in the same class as on the starting gains. The two changes to the procedure in §2 were made after these numbers were seen. Neither was chosen for its effect on them: the first keeps the start mode ArduPilot's page recommends, and the second removes an empty pack.
- **The tuned file, flown before the gate** (the file of §2, at the commit that added it). The same 100 gust runs: ArduCopter's median was 0.1761 m, the same as on the starting gains to four digits, and its 95th percentile 0.2212 m (0.2218 m on the starting gains); it had the lower error with the tuned file in 79 of the 100 runs, and no run differed by more than 1.5 mm. Our core, as before: 0.1700 m and 0.2211 m, lower than the tuned ArduCopter in 57 runs. All flights were safe. Run 0 of all 43 files: no class changed (34 safe, 4 fly-aways, 5 crashes). Two flights look different. In `motor-loss` the tuned ArduCopter is on the ground 2.2 s after the failure, 24 m from where it armed, with its motors still commanded (on the starting gains it tumbled for 8.9 s and came down 180 m away); its estimator's failsafe switches it to LAND 3.4 s after the failure, when it is already down. In `imu-loss` it meets the ground at 11.0 m/s (9.4 m/s). Both are crashes as before, and the times to motors off are the same.
- **A second trial of the gate program, on the tuned file** (same reduced plan as the first: gust-hold on 6 runs, 417 flights for each autopilot). ArduCopter's classes were the same as in the first trial, scenario by scenario: 327 safe, 50 crashes, 40 fly-aways, the unsafe ones being all ten runs of six of the seven files with a barometer loss, of `motor-loss`, of `imu-loss` and of `kill-in-rtl`. Our F405 core: all 380 flights of the outcome-scored scenarios safe and passing. Time to motors off as before (1.0 s against never; 0.14 s against 2.0 s). All ten flights flown twice repeated. The program gave "not passed", as it must for a reduced plan.

## The first run

Flown on 18 November 2026, at commit `2c2fedb`, with everything above in place before it. The report, the tables of the plan's 3,233 flights and of the ten flown twice (3,253 flights in all), and what the program printed are in [`docs/gates/gate-1/`](../gates/gate-1/README.md).

| Criterion | Result | Measured |
|---|---|---|
| A1 | holds | 500 of 500 gust flights safe and passing, on each profile |
| A2 | **fails** | median 0.17141 m against ArduCopter's 0.17545 m (holds); 95th percentile 0.22612 m against 0.22087 m (fails, by 2.4 %) |
| B1 | holds | our core 0 crashes or fly-aways in each of the 38 scenarios; ArduCopter 10 of 10 in six of the seven with a barometer loss (not in `pair-rc-loss+baro-loss`) |
| B2 | holds | 380 of 380 flights safe and passing |
| B3 | holds | `motor-loss` 0.999 s against 14.9995 s (never); `imu-loss` 0.139 s against 2.0015 s |
| Repeats | hold | 10 of 10 |

**Gate 1 is not passed.** By §7 the criteria stay, the core is improved, and the whole gate is flown again.

What the run showed about the failure, for whoever improves the core:

- Our core's errors are spread wider than ArduCopter's. It is better up to about the 75th flight in a hundred and worse above. Its worst twentieth is 5 mm worse.
- It is not an accident of the seeds: by hundreds of runs, our 95th percentile is above ArduCopter's in four of five and level in the first, the hundred that the first looks were flown on.
- The error is the estimate, not the control: on the true state the same core holds about 0.030 m at the median.
- The H743 profile, reported only, holds within 0.1570 m at the median and 0.2116 m at the 95th percentile, both below ArduCopter's.

Two rules for the improvement, written now, before any is tried:

1. **The 500 gust seeds of the gate are not a tuning set.** A change to the core is chosen and judged on other seeds of `gust-hold` (runs 500 to 999) and on the rest of the scenario files, and only then is the gate flown again on runs 0 to 499. Otherwise the second run would measure how well the core was fitted to those 500 winds.
2. **A change must be one we would make without the gate:** a fault found, or a setting that is better on its own grounds, with its effect on every other scenario shown. A gain turned until this one number passes is not an improvement.

## The second run

Flown on 19 November 2026 at commit `92fdaa2`, with the criteria, ArduCopter, its parameter files, the binary and the scenario files of the first run. The report is [`docs/gates/gate-1/run-2/`](../gates/gate-1/run-2/README.md).

**The change.** One change to our core: the estimator's process noise is applied per second, not per filter step (ADR-0002 item 12). It was found by a diagnosis on gust-hold runs 500 to 999 and on the other scenario files' runs 500 to 519, as rule 1 of the first run requires; nothing was chosen or judged on the gate's runs, which were flown only to re-pin trace hashes. It stands on PX4's meaning of the parameters and on a filter that behaves the same at every rate DESIGN allows, as rule 2 requires; it was not chosen by its effect on a flight. On runs 500 to 999 it had already met A2 (our F405 core 0.1574 m and 0.2009 m against ArduCopter's 0.1747 m and 0.2137 m) before the gate was flown again.

| Criterion | Result | Measured |
|---|---|---|
| A1 | holds | 500 of 500 gust flights safe and passing, on each profile |
| A2 | holds | median 0.15658 m against ArduCopter's 0.17545 m; 95th percentile 0.20278 m against 0.22087 m |
| B1 | holds | as in the first run |
| B2 | holds | 380 of 380 |
| B3 | holds | `motor-loss` 0.999 s against 14.9995 s (never); `imu-loss` 0.139 s against 2.0015 s |
| Repeats | hold | 10 of 10 |

**Gate 1 is passed.** Our F405 core's error is below ArduCopter's at every percentile from the 1st to the 99th, and its 95th percentile is below ArduCopter's in each of the five hundreds of runs. ArduCopter's 921 rows of the tables (911 flights and the 10 repeats) equal the first run's in every column but the wall-clock time. A second, independent implementation of the verdict gives the same result to the last bit.

What the pass rests on, stated so that it is not read as more: the gate was flown twice, and the second run followed a change chosen on other seeds. The change does not make the estimator well calibrated. Its consistency check found the filter underconfident before and after, and three faults outside the change, all deferred in ADR-0002: a drift through the heading fusion in long flights, the barometer's unmodelled drift, and the simulated GPS reporting twice its true noise.

## Consequences

- DESIGN §8 rule 3 does not hold for gate 1: our core is not tuned by an automatic tuner of its own. DESIGN is changed to say so. The rule stands for gate 2.
- A pass says that our core, as it is, holds position in gusts at least as well as a tuned ArduCopter on equal sensors, and never ends a failure worse. It does not say that our tuning method is as good as ArduCopter's: that was not compared.
- ArduCopter's barometer and compass noise is one fixed sequence in every flight (ADR-0006 §7), so its 500 gust flights vary by wind, IMU noise and GPS error only.
- The outcome classes are coarse on purpose. They cannot say which autopilot chose the wiser response, only that neither response ended badly.

## Deferred

- PX4 (gate 2), and the tuning rule with our own automatic tuner (gate 2).
- Scoring the responses themselves (which action, how fast) and not only their outcomes.
- CPU and RAM (DESIGN §8 rule 5): measured on the chips, not in this gate.
