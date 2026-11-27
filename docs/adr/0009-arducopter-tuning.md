# ADR-0009: ArduCopter's tuning procedure from gate 2 on

Status: **accepted. Proposed on 21 November 2026 and merged on 27 November 2026 on the owner's yes ([#10](https://github.com/danbiocchi/airborne-oxide/pull/10)); it is in force.** It replaces the tuning flight of [ADR-0007](0007-gate-1-criteria.md) §2 for every gate after gate 1. Gate 1's procedure and its tuned file stay as that gate's record, at the commits its two runs were flown on. Nothing here was judged on gust-hold runs 0 to 499, the gate's seeds.

## In short, for the owner

- **The problem:** on the IMU model of [ADR-0008](0008-imu-filter-and-decimation.md), gate 1's tuning flight is accepted on 4 of 20 tuning seeds, and it fails on its own seed. The committed tuned file could no longer be written by the code, and three tests waited.
- **Why it failed:** the flight starts in ALT_HOLD, which holds no position, and the vehicle slides 100 m away in 13 of 20 flights. In 3 more, yaw's tuning does not finish before the tuner's window ends, and the window cannot grow: the pack is nearly empty soon after.
- **The procedure now:** two tuning flights, each started in POSHOLD, which holds position. The first tunes roll and pitch. The second, on a full pack, tunes yaw. The notch is decided once, on the first hover, and the tuned file carries it when one is set.
- **Measured:** accepted on 20 of 20 seeds, where gate 1's flight is accepted on 4. No flight goes further than 4.03 m from where it armed.
- **The comparison with our core does not move.** ArduCopter's gust error on runs 500 to 999 is within 0.0003 m of what it was with gate 1's file, whichever procedure wrote the gains.
- **Both changes are ArduPilot's own.** Its AutoTune page describes starting the tuner from PosHold, and tuning the axes in separate flights when the pack does not last for all of them.

## Context

ADR-0007 §2 tuned ArduCopter in one flight: arm in ALT_HOLD, climb to about 10 m, hover for 60 s, switch to AUTOTUNE for 480 s, land, and disarm in AUTOTUNE so that the gains are saved. That flight was accepted on its seed on the IMU model of the time, and its file was flown in both runs of gate 1.

ADR-0008 changed the IMU model and found that the flight no longer completes on its own seed. Twenty seeds showed that the procedure was fragile on both models: accepted on 6 of 20 on the old one and 4 of 20 on the new. The owner decided that the procedure is changed for gate 2, and that the three tuning tests wait until then.

### Why the flight fails

Measured on tuning seeds 0 to 19 on the IMU of ADR-0008, in calm air. The tuning's seed is its own; none of these is a gust-hold run.

| Outcome of gate 1's procedure | Seeds |
|---|---|
| Accepted | 4 |
| Fly-away: 100 m from where it armed | 13 |
| Yaw not complete when the window ends | 3 |

**The fly-away is a steady drift that begins before the tuner does.**

- In the 60 s hover, with the sticks centred, the vehicle already moves at 0.17 to 0.74 m/s in one direction (median 0.36 m/s). The tuner's twitches add a jitter around that and no net motion.
- The drift follows a small true tilt: 0.14° to 0.72°, at 1.09 m/s for each degree, pointing opposite to the tilt on all 20 seeds.
- The tilt follows the IMU's accelerometer bias at power-on (0.05 m/s² per axis, one sigma, in the model). The size of the horizontal bias correlates with the drift speed at 0.70. ArduCopter's estimator takes part of the bias as tilt, and the controller then holds the wrong level.
- ALT_HOLD has no position loop to correct it. Even the four accepted flights were 61.5 to 67.5 m out.

ADR-0007 §2 measured this drift on the old model (0.15 to 0.2 m/s on five seeds) and judged it harmless within the gate's 100 m limit. Twenty seeds show that it is not.

**Yaw's tuning has a first stage of very uneven length.**

- Over 67 completed flights it took 96 to 254 s (median 138 s). The four stages after it take about 130 s together, with little spread. Roll and pitch take about 92 s each.
- Yaw starts about 270 s into the flight and the window ends at 566 s, so yaw fits only if its first stage is shorter than about 165 s.
- The length does not depend on the start mode, on the drift, or on `AUTOTUNE_AGGR` (0.1 against the binary's 0.075).
- The window cannot grow: the pack is nearly empty 650 s into the flight. The pack itself was never the reason for a refusal (14.5 to 14.6 % of its charge and 21.65 to 21.66 V under load when the descent begins, against a threshold of 20.767 V).

What happens inside that stage was not read: ArduPilot is a behaviour reference here, and only the binary's messages and timing were measured.

### What ArduPilot's documentation gives

From its AutoTune, Tuning Process, and notch-filter pages:

- **The start mode.** AltHold is the first mode named. "AutoTune performs a weak position hold if invoked from Loiter or PosHold flight modes": the vehicle leans up to 10° toward the point where the tuner was started. The page adds that "a better tune can often be achieved by invoking AutoTune from AltHold".
- **The pilot may steer.** "Use the roll, pitch, yaw or throttle stick at any time to reposition the copter if it drifts away." The tune pauses while the sticks are moved.
- **The axes may be tuned in separate flights.** `AUTOTUNE_AXES` is a bitmask (1 roll, 2 pitch, 4 yaw). "This is useful if the vehicle's battery life is not long enough to complete all of the axes in one flight." Gains are saved by landing and disarming in AutoTune.
- **The notch.** Setting up the notch filter is step 4 of the tuning process, before the manual tuning and AutoTune, and the AutoTune page calls it "highly recommended". The throttle-based method needs "the motor noise at the hover throttle level", found from a spectrum of a hover. No page gives a size for a peak worth a notch, says what to do when no clear peak shows, or sends the pilot back to look again after the tune.

## Decision

The procedure of ADR-0007 §2 stays as it is in everything not named here: the frame and motor settings, the guide's filter settings, the starting gains, the hover-thrust learning, the tuner's own settings, the 60 s hover, the 480 s window, the landing, and the rule that the script decides everything from simulated time, the true height, the motor pulses and the gyro samples it gave the program itself.

That rule still holds for everything a flight does while it flies. With more than one flight, three things that the program reported now pass from one flight to the next, and the code names them:

- the hover thrust learned in the first hover, when a peak was found (as in gate 1's script);
- the gains the roll-and-pitch flight saved, which the yaw flight is given as a parameter file;
- whether a flight passed its checks, which decides whether the next flight is flown at all.

### 1. Every tuning flight starts in POSHOLD

The pilot arms in POSHOLD (mode 16) where gate 1's pilot armed in ALT_HOLD. The vehicle then holds its position in the hover, and holds it weakly in the tuner.

### 2. Two tuning flights

- **The roll-and-pitch flight** flies the base file, the set-up file and the starting gains, with `AUTOTUNE_AXES` 3. Its saved gains are read back.
- **The yaw flight** is a new start of the program on a full pack. It flies the same files and then the gains the first flight saved, with `AUTOTUNE_AXES` 4. Its saved gains are read back and are the procedure's result.

Each flight is the flight of ADR-0007 §2 otherwise: arm, climb to about 10 m, hover 60 s, AUTOTUNE for 480 s, descend with the stick at 0.3, land, disarm in AUTOTUNE.

### 3. The notch is decided once, on the first hover

The rule of ADR-0007 §2 step 4 and its threshold are unchanged: a line between 40 Hz and 400 Hz, on any axis, with at least ten times the band's median power, is a peak. It is applied to the hover of the first flight flown, which flies the starting gains. This is the documentation's order: the notch comes before the tune.

- **A peak:** that flight lands after its hover. The roll-and-pitch flight is then flown from the start with the guide's throttle-based notch, and with `MOT_THST_HOVER` set to the hover thrust that hover learned, as gate 1's script did after a peak. The yaw flight flies the same six notch values. Its hover thrust is the one the roll-and-pitch flight saved.
- **No peak:** no flight has a notch.
- The spectrum of every later hover is taken and reported, and decides nothing.

### 4. The tuned file carries the notch

When a notch was set, its six `INS_HNTCH_*` values follow the 29 gains in the tuned file, so that every later flight flies the notch its gains were tuned with. They are written as read back from the program after the yaw flight. When none was set the file has no such line.

Gate 1's script did not do this. Its tuned file listed the gains only, so a notch set for the tuning flight would not have been flown in the gate. It never mattered: gate 1's hover showed no peak.

### 5. Acceptance

Every check of ADR-0007 §2 holds for each tuning flight, on the axes it tunes: the set-up file read back before the flight; each of its axes reported complete; the tuner's success and saved-gains messages; every value read back; the flight safe by the gate's class; the pack above the low-battery threshold before the descent.

Three checks are new:

- **The yaw flight changes nothing but yaw.** Every value read back after the yaw flight equals what the roll-and-pitch flight saved, except the yaw gains, `ATC_ACC_Y_MAX` and `MOT_THST_HOVER`.
- **The program took what each flight was given.** `AUTOTUNE_AXES`, and `MOT_THST_HOVER` where it is set, are read back before the flight and must equal the value given.
- **The notch is the notch that was set.** After each flight that flies a notch, its six values are read back and must equal the values set, as 32-bit numbers.

The first flight, when it lands on a peak, is held to what gate 1's script held it to (the set-up file and the gate's class), and to the second new check.

If one check fails, no later flight is flown and no tuned file is written.

### 6. The committed file

The file is what the procedure writes on seed 0. Every bridged flight loads the base file, the set-up file, the tuned file and the failsafes, in this order, as before.

## How the procedure was chosen

The rule was fixed before any candidate was flown, in this order:

1. **Documented.** A candidate must be a way of tuning that ArduPilot's documentation describes, with nothing of ours in it that the documentation does not give.
2. **Robust.** Accepted on at least 18 of tuning seeds 0 to 19.
3. **Simple.** Among robust candidates, the nearest to gate 1's procedure.
4. **Not by gust result.** ArduCopter's gust error with each candidate's gains was measured, on runs 500 to 999 only, and reported. It chose nothing.

| Candidate | Accepted of 20 | Why the others fail |
|---|---|---|
| Gate 1's: one flight from ALT_HOLD | 4 | 13 fly away, 3 do not finish yaw |
| One flight from LOITER | 13 | yaw not finished |
| One flight from POSHOLD | 14 | yaw not finished |
| Two flights from ALT_HOLD | 1 | fly away |
| Two flights from LOITER | 20 | |
| Two flights from POSHOLD | 20 | |
| One flight from LOITER, `AUTOTUNE_AGGR` 0.1 | 14 | yaw not finished |

- **A position-holding start removes the drift.** From LOITER or POSHOLD no candidate's flight went further than 4.6 m from where it armed.
- **A flight of its own gives yaw the time.** In one flight, yaw is the only failure left once the drift is gone. In two flights, as built, the slowest yaw of the 20 finished with 101 s of its window to spare.
- **Two flights from ALT_HOLD are worse than one.** The roll-and-pitch flight is done at about 275 s and then waits in the tuner, drifting, until its window ends.
- **POSHOLD over LOITER.** Both are documented and both are accepted on 20 of 20. POSHOLD is the nearer to gate 1's procedure: its first hover makes the same notch decisions as the ALT_HOLD hover did (a peak on seeds 9 and 13 only, against 7 of 20 from LOITER), and so it flies fewer flights.

Not chosen:

- **A pilot who steers back, from ALT_HOLD.** The page allows it. But when and how hard such a pilot steers would be figures of ours, as ADR-0007 §2 already said.
- **A more aggressive tuner.** `AUTOTUNE_AGGR` 0.1 did not shorten yaw's first stage, and it changes the gains.
- **A notch always.** The first hover usually shows no line to set it on (below), and the guide gives no frequency without one.
- **No notch ever.** The documentation's tuning process has the step.

### Which way the choice leans

ArduPilot says a tune from AltHold is often the better one, so starting from POSHOLD could lean against ArduCopter. Measured, it does not:

- **The gains hardly differ.** Over the accepted seeds, roll rate P is 0.0549 to 0.0576 from ALT_HOLD and 0.0547 to 0.0577 from POSHOLD in two flights, as built; pitch rate P 0.0549 to 0.0608 and 0.0524 to 0.0579; yaw rate P 0.0497 to 0.0538 and 0.0500 to 0.0555. The pitch angle gain is 27.75 on every accepted seed either way. The roll angle gain is 27.75 on every accepted seed from ALT_HOLD, and 25.4 or 27.75 from POSHOLD.
- **The gust error does not move.** Runs 500 to 999, all 500 flights safe in every row:

| Tuned file | Median | 95th percentile |
|---|---|---|
| Gate 1's committed file (ALT_HOLD, old IMU, seed 0) | 0.17512 m | 0.21722 m |
| Gate 1's procedure on the new IMU, seed 5 | 0.17524 m | 0.21721 m |
| One flight from POSHOLD, seed 0 | 0.17521 m | 0.21718 m |
| Two flights from LOITER, seed 0 | 0.17514 m | 0.21707 m |
| **Two flights from POSHOLD, the notch decided once, seed 0: the committed file's values** | 0.17516 m | 0.21699 m |
| The same procedure in the diagnosis's code, seed 9, with its notch | 0.17506 m | 0.21723 m |
| Seed 9's file with its six notch lines taken out | 0.17509 m | 0.21708 m |

Every file is within 0.0002 m of gate 1's at the median, and within 0.0003 m at the 95th percentile. This agrees with what ADR-0007 found: the tuned gains hardly move ArduCopter's gust result.

### The notch, and what is chance in it

The IMU model of ADR-0008 has a line at the rotors' speed, near 223 Hz in a hover. How far it stands above the band's median in a 30 s spectrum differs from hover to hover: from about 1 to 20 times.

An independent check found what decides it: **the four rotors' lines add or cancel.**

- The model gives each rotor its own line, with a phase that starts at random and advances at that rotor's speed. In a hover the four speeds are within 0.07 Hz of each other, so the gyro sees the sum of four lines of almost one frequency. That sum is anything from nothing to four times one rotor's line, by how their phases happen to stand.
- Measured over the 40 hovers of the twenty seeds: the line's strength follows the distribution of four lines with random phases. Each rotor's own amplitude, the steadiness of the rotor speed and the rest of the band are the same in both hovers of a seed, and the line still changes up to 36 times between them.
- It is not the gains. The line was above the threshold on 2 of 20 first hovers (starting gains) and on 6 of 18 second hovers (tuned roll and pitch gains; seeds 9 and 13 are left out of the 18, because their second hover flies the notch). That difference is within chance (Fisher's exact test, p = 0.12, or p = 0.24 counting all 20 seeds), and on seeds 9 and 13 the line fell on the tuned gains.
- It is not the spectrum's own scatter either: 57 averaged segments scatter by about 13 %, not by a factor of ten.

Not done: the phases were not rebuilt hover by hover from the simulator, so the alignment of each hover is inferred from the model and the distribution. A small real shift on the tuned gains is not excluded by twenty seeds.

So whether a seed's file carries a notch is partly chance. Two things keep that harmless:

- **The decision is taken once**, so a file is never a mix of gains tuned with the notch and gains tuned without it.
- **The notch does not move the gust error** (the last two rows of the table above).

On seed 0, the committed file's seed, the first hover shows no peak (largest line 2.0 to 2.5 times the median), and the file has no notch.

## The procedure as flown

Seed 0, the committed file's seed. The report committed beside the file was written at commit `9f1d890`, on instance 2 of the program; since then the tuning code has changed only in its comments. A second run on another instance of the program gives the same two flights bit for bit and the same file byte for byte, and a test flies that comparison.

| | Roll-and-pitch flight | Yaw flight |
|---|---|---|
| Parameter files | base, set-up, starting gains | the same, then the first flight's gains |
| `AUTOTUNE_AXES`, as read back | 3 | 4 |
| Hover reached | 24.8 s | 24.8 s |
| Largest line of the hover's spectrum | 2.0 to 2.5 times the median: no peak | 16.5 times, at 223.6 Hz: reported, decides nothing |
| Tuner started | 85.8 s | 85.8 s |
| Roll complete | 177.6 s | |
| Pitch complete, success | 269.7 s | |
| Yaw complete, success | | 431.4 s |
| Descent begins | 565.8 s | 565.8 s |
| Pack then, under load | 21.660 V, 14.6 % | 21.678 V, 14.9 % |
| Gains saved at the disarm | 583.6 s | 583.6 s |
| Furthest from where it armed | 2.77 m | 1.57 m |
| Greatest tilt | 15.6° | 3.1° |

The low-battery threshold is 20.767 V. Both flights are safe by the gate's class. The procedure takes 1,195 s of simulated flight.

| Parameter | Before tuning | Gate 1's file | This procedure |
|---|---|---|---|
| `ATC_RAT_RLL_P`, `_I` | 0.05 | 0.0547 | 0.0549 |
| `ATC_RAT_RLL_D` | 0.0012 | 0.00117 | 0.00123 |
| `ATC_RAT_PIT_P`, `_I` | 0.05 | 0.0549 | 0.0577 |
| `ATC_RAT_PIT_D` | 0.0012 | 0.00123 | 0.00123 |
| `ATC_RAT_YAW_P` | 0.065 | 0.0483 | 0.0508 |
| `ATC_RAT_YAW_I` | 0.0065 | 0.00483 | 0.00508 |
| `ATC_RAT_YAW_FLTE` | 2 | 5 | 5 |
| `ATC_ANG_RLL_P` | 4.5 | 25.4 | 27.75 |
| `ATC_ANG_PIT_P` | 4.5 | 27.75 | 27.75 |
| `ATC_ANG_YAW_P` | 4.5 | 1.96 | 2.06 |
| `ATC_ACC_R_MAX`, `ATC_ACC_P_MAX` | 1100 | 4875, 5009 | 4976, 5112 |
| `ATC_ACC_Y_MAX` | 270 | 206 | 220 |
| `MOT_THST_HOVER` | 0.16 | 0.150 | 0.150 |

The file holds every digit. The other values of the 29 it lists did not change, and it has no notch line. As in gate 1, the file is flown as the tuner saved it: nothing is changed by hand.

**The twenty seeds, as built:**

| | |
|---|---|
| Accepted | 20 of 20 |
| Flights | 2 on 18 seeds; 3 on seeds 9 and 13, whose first hover shows a peak (222.7 Hz at 19.3 times the median, and 223.6 Hz at 16.3 times) |
| Roll complete | 177 to 183 s |
| Pitch complete | 268 to 282 s, with 283 to 297 s of the window left |
| Yaw complete | 311 to 465 s, with 101 to 255 s of the window left (the slowest is seed 9) |
| Furthest from where it armed | 1.88 to 4.03 m |
| Pack when each tuning flight's descent begins | 21.658 to 21.678 V, 14.6 to 14.9 % |

On seeds 9 and 13 the file ends with the six notch lines, read back as set. On seed 9 these are `INS_HNTCH_ENABLE` 1, `INS_HNTCH_MODE` 1, `INS_HNTCH_FREQ` 222.65625, `INS_HNTCH_BW` 111.328125, `INS_HNTCH_REF` 0.14950788 and `INS_HNTCH_ATT` 40.

### How it was checked

- **By its builder:** unit tests that fly the whole procedure on made-up flights, and seven tests on the real program.
- **By a reviewer, a separate agent:** it compared the decision above with the code line by line and found no check weaker than gate 1's. It then broke the code on purpose in 28 ways, to see a test fail each time. One break went unnoticed: skipping every acceptance check on the flights that fly a notch. A test now catches it. Ten smaller findings were mended. Among them: the second new check of §5 was added, and the third now holds after every flight that flies a notch, not only the last.
- **By a verifier, a third agent:** it confirmed each mend and flew seed 13. It broke the new guards in 9 more ways: 8 were caught, and the ninth removes one of two guards that each do the job alone.
- **Fresh runs:** seed 0 wrote the committed file byte for byte in every fresh run: on instances 0, 2 and 4, and in the test that compares two instances, which was run again on other instances for this record.
- **The record itself:** two more reviewers traced its figures to the measurements and held the documents against the code. They found the 5 s figure of the motor-loss section below, a false sentence about the angle gains, a stale comment, and smaller slips. All are corrected here.

### What is not shown

- **That ArduCopter's filter applies the notch.** A test shows that a flight started the way a gate flight is, from a file with the six lines, has the six values in the program. That the gyro signal is then filtered is shown only indirectly: taking the lines out changes the flight (the last two rows of the gust table).
- **The gust rows of seed 9** are of the diagnosis's code. As built, the roll-and-pitch flight after a peak also sets the learned hover thrust, so seed 9's gains differ slightly from those rows' (its yaw completes at 465 s, where the diagnosis had 444 s). The seed 0 row is the committed file's own values.
- **The notch's reference thrust and the file's hover thrust differ slightly** on a seed with a notch: about 0.1495 learned in the first hover against about 0.1500 saved by the tuning flights (seed 9: 0.14951 and 0.14993; seed 13: 0.14959 and 0.14998). Gate 1's script would have done the same.
- **When each vehicle met the ground in the motor-loss runs 500 to 519** (below). The runner's tables do not hold it.
- **Windows.** The tests on the real program run on Linux only, as before.

## The pins that moved

ArduCopter's flights now fly other gains, so what single flights are pinned to moved in places. Each new value was seen on two instances of the program before it was pinned, with one exception: the second parameter hash below is computed by the code, was pinned as computed, and was then confirmed by real geofence flights on two instances. No tolerance was widened, and no test was removed or switched off.

| Where | What moved |
|---|---|
| `src/gate/files.rs` | The two parameter hashes the gate expects (the tuned file is part of what is hashed). Each is now compared with the record of a real flight. |
| `arducopter_sticks` | The three hashes of gust-hold 0 and of hover-step 0. In `gps-loss-pilot`, where the blind landing ends, by 0.13 m. |
| `arducopter_faults`, `imu-loss` | The falling vehicle meets the ground at 10.2 m/s, where it was 9.5 m/s. |
| `arducopter_faults`, `motor-loss` | The touchdown speed (7.4 m/s, where it was 6.9 m/s), the greatest tilt, and what the vehicle does once it is down. The test was renamed (next section). |
| `arducopter_tune` | Every pin of the tuning flights: they are other flights now. |

No other pin moved: none in `arducopter_boot`, `arducopter_fly` or `arducopter_scenario` (which gained one assertion: the second parameter hash, compared with a real geofence flight), none in the gate program's two trials, and none in the other tests of the two files above. Our core's ten `oxide-sim hash` lines and its scenario hashes are unchanged.

### Motor loss: a statement that was an observation

The test of `motor-loss` run 0 said that ArduCopter's motors stay on to the end of the flight. On the new gains that flight goes another way:

- The vehicle is on the ground 2.2 s after the failure, as before, at 7.4 m/s (6.9 m/s before).
- The estimator's failsafe switches to LAND 1.2 s after that.
- The program, in LAND, judges the vehicle landed and disarms, 11.3 s after the failure.

**The order of events.**

1. The planner first ruled that "still armed, motors on to the end" was a rule of the test and had to hold unedited. The re-pin stopped on it, as it should.
2. The planner then read gate 1's record and ruled again, from what the planner knew of LAND (the program disarms once it judges the vehicle landed): the statement was an observation. That second ruling still held a figure of the planner's own, that the motors stay on for at least 5 s after the vehicle is down.
3. While the test was re-stated, that knowledge was checked against ArduPilot's Land and crash-check pages, and twenty runs that are not the gate's were flown. Both are below, and both agree with the second ruling's main point.
4. A reviewer then held this record against those twenty runs and found that the 5 s figure fails in two of them. It was a second observation written as a rule. It is gone from the test.

What the three sources say:

- **ArduPilot's documentation.** Its Land page: "Upon reaching the ground the copter will automatically shut-down the motors and disarm the copter if the pilot's throttle is at minimum." Its crash check "disarms the motors in cases where the vehicle is likely out of control and has hit the ground", when the lean angle is more than 30° from the wanted one and the vehicle accelerates by less than 3 m/s², for 2 s. So a disarm on the ground is the program's documented behaviour, by either route.
- **Gate 1's own record.** None of ArduCopter's ten `motor-loss` flights disarmed there.
- **Twenty runs that are not the gate's** (500 to 519), with both tuned files:

| | Gate 1's tuned file | This record's |
|---|---|---|
| Crash | 20 of 20 | 20 of 20 |
| Switched to LAND by the estimator's failsafe | 8 | 8 |
| Disarmed before the flight ended | 4 (3 in LAND, 1 by the crash check) | 1 (by the crash check) |
| Motors on to the end | 16 | 19 |
| Median time to motors off, as criterion B3 counts it | 14.9995 s (never) | 14.9995 s (never) |

Our core, on the same runs: motors off 0.999 s after the failure in all 20.

**What this shows.**

- **"Motors on to the end" was an observation, not a rule.** It was seen in gate 1's ten flights, on the IMU before ADR-0008, and in run 0 on the IMU of ADR-0008. With either file, some flights end disarmed: when the estimator's failsafe happens to come and LAND then completes, or when the vehicle happens to lie still enough for the crash check.
- **Neither file makes ArduCopter answer a lost motor.** In none of the forty flights does the program change mode or disarm in the first 3.2 s after the failure: the LAND comes 3.2 to 6.4 s after it, and the earliest disarm 5.2 s after it. (Its estimator switches lanes 2.7 to 3.1 s after the failure in 25 of the forty, which is no answer to the failure.) In run 0 the vehicle is down after 2.2 s. With this record's file the motors stay on to the end in 19 of 20 flights, and the vehicle gets a median of 103 m from home.
- **Criterion B3 does not move.** Its median is "never" with both files.

The test now pins the course of this one flight. Beside that it states only what held in all three flights of run 0 known to us (gate 1's file on each IMU model, and this record's file): nothing is done before the vehicle is down, the motors are still on when it is down, the flight is a crash, and the scenario's limit (motors off 2 s after the failure) is missed. Of these, the last two could be checked on the forty other flights, and they hold in all forty. In run 0 the new gains happen to favour ArduCopter (11.3 s where it was never). That decides nothing: gate 1 stands as flown, and a later gate counts the median of its own ten flights.

## Consequences

- `crates/oxide-bridge/arducopter/tuned-gains.parm` is replaced by what this procedure writes on seed 0. The run's report is beside it (`tuning-report.txt`).
- The three tuning tests that waited fly again, and three are new: a seed whose first hover shows a peak, a flight that loads a file with a notch, and the twenty seeds (behind a switch, `OXIDE_TUNE_TWENTY=1`, since it takes ten minutes).
- Pinned values of ArduCopter's flights move with the new file (above). Our core's pins do not move: it does not fly ArduCopter's gains.
- Gate 1's reports are unchanged. They are records of runs at their commits, where gate 1's file and script are.
- DESIGN §8 rule 3 already names this procedure for ArduCopter: "the initial-tune parameters, harmonic notch setup and AUTOTUNE in SITL".
- Tuning ArduCopter now takes about 1,200 s of simulated flight in two flights (about 140 s of wall time here), or about 1,300 s in three when the first hover shows a peak.

## Deferred

- **Why yaw's first stage is so uneven.** Only its length was measured. It no longer decides anything: the yaw flight leaves it the whole window.
- **A better detector of the rotor line.** The rule is a threshold on one line of a 30 s spectrum, and it is ours, as it was in ADR-0007. It could be replaced when the IMU model's rotor lines are revisited (ADR-0008, deferred).
- **PX4's tuning and our own automatic tuner** for gate 2 (DESIGN §8 rule 3). They belong to gate 2's criteria record.
