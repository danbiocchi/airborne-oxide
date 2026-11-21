# ADR-0008: The IMU's anti-alias filter and decimation

Status: **accepted by the owner on 20 November 2026 as the simulator's IMU model, to take effect after gate 1's second run. That run has been flown** (19 November 2026 at `92fdaa2`, on the old IMU: passed, [ADR-0007](0007-gate-1-criteria.md)), so the model takes effect when this branch is merged. **Every pin was regenerated on 20 November on the owner's yes, our core's and ArduCopter's; the merge waits for the owner.** The branch is rebased onto the main line of that run (`19c7534`), and every figure of our core below was measured again on it. The model changes what both autopilots sense ([ADR-0006](0006-ardupilot-bridge.md) §4). It changes no criterion. Nothing in it was judged on gust-hold runs 0 to 499, the gate's seeds. The repository's default tests fly a few of those runs, as they always do: runs 0 to 7 against the file's limits (`gust_hold_quick`, which passes) and runs 0 and 1 to compare trace hashes. No figure from those runs is used here.

## In short, for the owner

- **The model:** the IMU samples at 8 kHz on its own clock. Its noise passes a 256 Hz second-order anti-alias filter, and each reader is given every Nth sample. The vibration is a density (6.325e-4 rad/s/√Hz and 1.581e-2 m/s²/√Hz at hover, flat to 4 kHz), equal to what the F405 and ArduCopter streams already carried. The noise a 1 kHz and a 4 kHz reader get agrees within 3 % (tested).
- **Effect, runs 500 to 999, on the main line's estimator:** the F405 does not move beyond the redraw, the H743 rises to the F405 (the gap is gone), and ArduCopter does not move. **The comparison with ArduCopter does not move:** our F405 core's 95th percentile is 0.940 of ArduCopter's on the old IMU and 0.940 on the new one.
- **A problem it creates for the next gate:** ArduCopter's committed tuning procedure fails on the new IMU on its own seed (yaw does not finish in its 480 s window). Twenty seeds show that the procedure is fragile on both models (accepted on 6 of 20 on the old IMU and 4 of 20 on the new), not that the new IMU breaks it.
- **Decided, and what happened:** gate 1's second run was to keep the old IMU, and it did. The decision was taken on 20 November from a checkout that did not yet hold the main line of the 19th; the run had been flown and passed the day before, on the old IMU, after a fix to the estimator (ADR-0002 item 12). The model takes effect when this branch is merged, which waits for the owner. The tuning procedure is changed in gate 2's record.
- **Pins:** on our core's side, 124 of the 129 scenario hashes, 8 of the 10 `oxide-sim hash` lines, the gust-hold 0 pins of two bridge test files, and one bound of CI's bounds sweep. On ArduCopter's side, 13 tests that fail when run here, 3 tuning tests that fail by the procedure's own result, and the stick test's gust-hold 0 hashes. All were regenerated on the owner's yes (20 November), and the three tuning tests wait for gate 2's tuning procedure.

## Context

The simulated IMU's broadband vibration was a fixed rms per sample: 0.02 rad/s on the gyro and 0.5 m/s² on the accelerometer at the hover rotor speed. The white noise and the bias walk were densities, scaled to the sample period. The IMU was read once per tick of whoever read it, with no anti-alias filter and no decimation. A fixed rms per sample is a density that grows with the sample period (rms / √rate). So the F405 profile's 1 kHz stream had twice the vibration density, in amplitude, of the H743 profile's 4 kHz stream. ArduCopter reads the IMU on the F405's 1000 µs grid (ADR-0006 §4), so it had the F405's density too. ADR-0006 §4 described the model as having a fixed noise density. That was true of the white noise and not of the vibration.

A diagnosis on gust-hold runs 500 to 999, on the estimator of gate 1's first run (`6265126`), found that this accounts for about 91 % of the gap between the F405 and the H743 in mean-square position error. With the H743 run at the F405's vibration density, the two medians match (0.1683 m against 0.1707 m). With the F405 run at the H743's density, they match the other way (0.1574 m against 0.1562 m).

A real IMU does not work this way. It samples on its own clock, filters, and hands every reader the latest filtered sample (DESIGN §5.3).

## Decision

The IMU model (`oxide_sim::sensors::ImuModel`, whose documentation gives every value with its reason) becomes:

- **Its own clock: 8 kHz** (125 µs), whatever the step and the reader. Every internal sample draws a bias-walk step and a noise sample. Every simulator step must be a whole multiple of 125 µs (refused otherwise, `SimError::Imu`). The steps in use are 250, 500 and 1000 µs.
- **Anti-alias filter: second-order Butterworth, −3 dB at 256 Hz**, on the noise. The truth, the bias and the rotor-synchronous lines are added unfiltered at the reader's instant. Filtering the signal too would add about 0.88 ms of delay to every autopilot's rate loop. That is a second change to the gate's conditions, and DESIGN §5.4 already lists it for S2 ("output-filter delay").
- **Decimation:** a reader is given the latest filtered sample at its own period. With the same seed, a 1 kHz reader's noise is every fourth sample of a 4 kHz reader's noise, bit for bit.
- **Broadband vibration as a density**, flat from 0 to 4 kHz (the internal Nyquist frequency), at the reference rotor speed: gyro 6.325e-4 rad/s/√Hz, accelerometer 1.581e-2 m/s²/√Hz, scaled with the square of the mean rotor speed as before. These are the old figures as the 1 kHz stream carried them. The F405 and ArduCopter keep, below the filter's cut-off, the density they have flown on until now, and the H743 is raised to it. Because the filter passes only what lies below its cut-off, the 4 kHz band matters only by being far above 256 Hz.
- **The white noise** keeps its densities and passes the filter too.

Unchanged: ArduCopter's 1000 µs IMU grid, the wind, every other sensor, every criterion.

## What was measured, on gust-hold runs 500 to 999 only

Our core was measured twice: first on the estimator of gate 1's first run (`6265126`), and again, after the rebase, on the main line's estimator (`19c7534`, process noise per second, ADR-0002 item 12). The second measurement is the one that counts and is given first. ArduCopter's flights are the same in both: the estimator's fix does not touch the simulator or the bridge, and a sample of ArduCopter's flights flown again after the rebase (runs 500 to 511 on each IMU) equals the earlier tables in all three hashes.

Every flight below was safe and passed the scenario file's own limits. The statistics are those of ADR-0007 §5: the median as (a + b) / 2, and the 95th percentile by linear interpolation at 0.95 × (n − 1). The rms horizontal error is in metres.

| Runs 500 to 999, main line's estimator | Median, old IMU | Median, new IMU | 95th pct., old IMU | 95th pct., new IMU |
|---|---|---|---|---|
| Our core, F405 | 0.15735 | 0.15567 | 0.20086 | 0.20429 |
| Our core, H743 | 0.14780 | 0.15506 | 0.19016 | 0.20516 |
| ArduCopter (tuned file as committed) | 0.17467 | 0.17512 | 0.21373 | 0.21722 |

Reproduced with `oxide-bridge batch --scenario scenarios/gust-hold.toml --indices 500..1000 --autopilot core-f405` (and `core-h743`, `arducopter`) at `19c7534` for the old IMU and on this branch for the new one. The old-IMU figures of our core are those ADR-0002 item 12 records.

Paired by seed (the wind is the same per seed; only the IMU's draws change), with 95 % intervals from 4,000 bootstrap resamples of the seeds:

- **F405: no change beyond the redraw.** Median −0.0017 m (−0.0048 to +0.0027), 95th percentile +0.0034 m (−0.0057 to +0.0093). The flight with the new model was the better in 259 of 500 runs. That is what keeping the F405's density below the cut-off should give: the F405 loses only the noise between about 256 and 500 Hz, most of which its 80 Hz gyro filter removed anyway.
- **H743: worse, to the F405's level.** Median +0.0073 m (+0.0041 to +0.0102), 95th percentile +0.0150 m (+0.0069 to +0.0199). Its vibration density below the cut-off has doubled, to the F405's.
- **The two profiles now fly the same flight, seed by seed.** The mean of the F405-minus-H743 difference was −0.0092 m (standard error 0.0010) on the old IMU and −0.0001 m (standard error 0.00025) on the new one. The medians are 0.15567 m and 0.15506 m. The profiles read the same internal noise stream, and the gap is gone.

The first measurement, on the estimator of gate 1's first run, showed the same three things at that estimator's larger errors: F405 0.17070 → 0.16894 m at the median and 0.23019 → 0.23460 m at the 95th percentile; H743 0.15618 → 0.16997 m and 0.21287 → 0.22985 m.

The model's own test (`noise_density_is_the_same_at_1_khz_and_4_khz` in `crates/oxide-sim/src/sensors.rs`) measures the density a 1 kHz and a 4 kHz reader are given, on independent seeds over 64 s, and requires agreement within 3 % in amplitude. The filter lets 0.4 % fold back into 1 to 100 Hz and 0.8 % into 100 to 250 Hz, and each estimate scatters by about 0.4 %. Measured: gyro 6.43e-4 and 6.34e-4 rad/s/√Hz in 1 to 100 Hz (configured 6.38e-4 with the white noise), ratio 1.013; accelerometer ratio 0.998; in 100 to 250 Hz, 1.013 and 0.995. A second test shows that, with the same seed, a 1 kHz reader's noise is every fourth sample of a 4 kHz reader's, bit for bit.

Cost: 500 F405 flights took 17.9 s with four jobs, against 10.4 s before (about 280 times real time per core, against 480; DESIGN §5.1 asks for 100). The H743 took 33.3 s against 30.1 s. The IMU now draws 12 variates every 125 µs. A bias walk once per millisecond instead would cut the draws by about 40 % (estimated, not measured), and would change nothing a reader can measure.

## What it changes in the comparison with ArduCopter, and which way

**What ArduCopter and our F405 core sense changes alike.** Both read the same stream, on the same 1000 µs grid, with the same seed, before and after. Below about 100 Hz the density is what it was (the filter passes at least 97.7 % of the power there). Between the filter's band and 500 Hz the noise is removed: at hover, the vibration per sample falls from 0.020 to 0.015 rad/s and from 0.50 to 0.38 m/s². Every flight's IMU noise is drawn afresh, so each seed is a new realisation. The turn-on biases are drawn as before and do not change.

**Which way the comparison moves: measured, not at all beyond the redraw.** On runs 500 to 999, on the main line's estimator, our F405 core's error over ArduCopter's is:

| Ours over ArduCopter's | Old IMU | New IMU | Change |
|---|---|---|---|
| Median | 0.901 (0.871 to 0.917) | 0.889 (0.867 to 0.910) | −0.012 (−0.034 to +0.018) |
| 95th percentile | 0.940 (0.915 to 0.988) | 0.940 (0.917 to 0.970) | 0.000 (−0.049 to +0.037) |

ArduCopter's own median moved by +0.0005 m (−0.0026 to +0.0028), and its 95th percentile by +0.0035 m (−0.0022 to +0.0079). Neither autopilot's error moved by more than its seeds' scatter.

Why: the two read one stream, and the density both use is unchanged. The only common change is the noise above the filter's band, and both throw most of it away (each has an 80 Hz gyro filter). The change takes an artefact out of the F405-against-H743 comparison and leaves the F405-against-ArduCopter comparison where it was. **What gate 1's criterion A2 measures holds on these seeds with either IMU:** the 95th-percentile ratio is below 1 in 99.9 % of the resamples on each. (The first measurement, on the first run's estimator, had it at 1.077 and 1.080: that estimator failed A2 on either IMU.)

**The H743** (reported in gate 1; it must meet A1) is worse by 0.007 m at the median and 0.015 m at the 95th percentile. It stays below ArduCopter: 0.885 of its median and 0.944 of its 95th percentile on the new IMU, against 0.846 and 0.890 on the old. All of runs 500 to 999 pass the file's limits. **The H743 figures in the report of gate 1's second run were flown on the old IMU and carry this artefact.** A1 itself is judged on runs 0 to 499, which were not flown here.

**ArduCopter's tuning (ADR-0007 §2) does not survive the change as written.** The committed tuned file was written by the procedure on the old IMU. On this machine the old model's program still writes it byte for byte. With the new model, the same procedure (seed 0, unchanged script) fails its own acceptance: AUTOTUNE completes roll at 176.9 s and pitch at 270.3 s, slightly earlier than before, but yaw is still in its rate-P stage when the fixed 480 s window ends. Roll and pitch gains are saved and yaw's are not. As information only, the procedure was also flown on seeds 1 to 3:

| Tuning flight | Old model | New model |
|---|---|---|
| seed 0 (the procedure's) | success, yaw complete at 524.6 s | yaw not complete by the descent (565.8 s) |
| seed 1 | success, yaw at 526.1 s | fly-away (100 m out in ALT_HOLD at 252.1 s) |
| seed 2 | fly-away (100 m out at 286.5 s) | fly-away (100 m out at 275.7 s) |
| seed 3 | success, yaw at 535.5 s | yaw not complete by the descent |

In these four, yaw's rate-D stage took 110 to 134 s on the old model and 172 to 196 s on the new one, which read as the new IMU slowing the tuner. **Twenty seeds, flown after the owner's decision, show that it does not** (below): the four were too few. The procedure's 480 s window was set on the old model with about 40 s to spare (ADR-0007 §2). The spectrum rule of step 4 still finds no peak (largest line 2.0 to 2.5 times the band's median power, against 1.4 to 1.6 before; the rule is 10), so no notch either way.

Three ways forward were put to the owner:

1. **Keep the committed tuned file** and record that it was tuned on the old IMU. This is measured on gusts: with it, ArduCopter's error on runs 500 to 999 did not move (above). The test that a fresh run writes the committed file then fails until the procedure changes.
2. **Change the tuning procedure** so that it completes on the new IMU, and commit what it writes. One example is yaw in a second flight, which ArduCopter's `AUTOTUNE_AXES` allows. A longer window is not possible: the pack is about empty 650 s into the flight. This is a change to ADR-0007 §2, so it needs this record, and the flights must be on seeds other than the gate's.
3. **Keep the old IMU for gate 1's second run** and adopt this model after it. The F405-against-H743 artefact then stays in that run's H743 figures, which decide nothing.

**The owner chose the third** (20 November 2026), with the reasons below. They are kept as they were given. They were written from a checkout that did not hold the main line of 19 November: the second run had by then been flown, on the old IMU, and passed. The choice and what happened agree; the second and third reasons describe what that run in fact did (ArduCopter's 921 rows equal the first run's, ADR-0007).

- **The model buys gate 1 nothing.** The comparison that decides (F405 against ArduCopter) did not move on runs 500 to 999, and the gap it removes (F405 against H743) decides nothing in gate 1.
- **It would redraw a criterion that failed narrowly.** A2 failed by 0.0053 m at the 95th percentile (0.22612 m against 0.22087 m). Redrawing the IMU's noise alone moved ArduCopter's 95th percentile on runs 500 to 999 by +0.0035 m (−0.0022 to +0.0079) and our core's by +0.0044 m (−0.0109 to +0.0128). A pass after the change could not be told from the redraw.
- **On the old IMU the target stays where it is.** A bridged flight repeats bit for bit when nothing on its side changes (ADR-0006 §8), so ArduCopter's 500 gust flights and its two figures are the first run's, and the second run measures the core alone.
- **The tuning result was not explained** when the decision was taken: on four seeds the tuner looked slower on the new IMU although the density in the band it uses is unchanged. The twenty seeds below explain it as chance, so this reason has fallen away. The first three do not depend on it.

Two things were to follow for gate 1's second run. What became of each:

1. **A change to the core is judged on both IMUs before the gate is flown.** The run was flown before this could be done. It is done here after the fact: the estimator's fix of ADR-0002 item 12 holds on the new IMU as on the old (the table above: 0.15567 m and 0.20429 m against ArduCopter's 0.17512 m and 0.21722 m). The pass does not rest on the noise the new model removes.
2. **The report says that the H743's figures carry the artefact.** The second run's report does not say so. This record does (above). Whether a line is added to that report is the owner's to decide: it is the record of a run.

**The tuning, now that the model is adopted.** The procedure is changed so that it completes on the new IMU (the second way above), in gate 2's record, where the tuning rule is written anew in any case (ADR-0007, deferred). The committed file is not carried on as a file the code can no longer write. What a change must mend is shown by twenty seeds of the procedure on each model (seeds 0 to 19, the committed script, calm air; the tuning's seed is its own, and none of these is a gust-hold run):

| Seeds 0 to 19 | Old model | New model |
|---|---|---|
| Accepted: the tuner's success, gains saved, the flight safe | 6 | 4 |
| Fly-away: 100 m from where it armed, drifting in ALT_HOLD | 9 | 13 |
| Yaw not complete when the 480 s window ends | 5 | 3 |
| Yaw's rate-D stage, where it finished | median 158 s (110 to 276 s, 11 flights) | median 164 s (112 to 244 s, 7 flights) |

- **The procedure is fragile on both models.** The committed file comes from seed 0 on the old model, one of the six seeds in twenty on which the procedure is accepted. ADR-0007 §2 is met by it as written: the procedure is that one flight.
- **Twenty seeds do not tell the two models apart.** Six seeds are accepted on the old model only and four on the new one only (none on both). Yaw's slow stage takes the same time. The seed decides, not the IMU: each seed is a different draw of the noise on each model.
- **The gains are the airframe's.** Where the procedure is accepted it writes the same family of gains on either model: roll and pitch rate P 0.0547 to 0.0608, their angle P 25.4 or 27.75, yaw rate P 0.0483 to 0.0538, hover thrust 0.1499 to 0.1500. The committed file lies inside these ranges, at the low end of several. ADR-0007's first looks found that going from angle gains of 4.5 to 25 moved no gust run by more than 1.5 mm, so this spread should not matter to criterion A (not flown).
- **What a change must mend** is therefore in the procedure and not in the IMU: the drift in ALT_HOLD, which carries the vehicle 100 m away in about half the flights, and yaw's tuning, which does not always fit in what the pack allows.

Criteria A1, A2, B1, B2 and B3 are unchanged in every case. On criterion B's side, our core was flown on every fault file, on the seeds each names and on both profiles (`fault_scenarios_full`), and every flight passes its file's expectations. ArduCopter's fault flights were not flown in bulk, because those are the gate's own flights of criterion B. Run 0 of several, from the binary's own tests, is listed below.

## The pins that move

**Where each stands** (20 November 2026, on the rebased branch):

- **Our core's flights: regenerated, on the owner's yes**, in a commit of their own. The table below is what moved against the main line's pins of 19 November. `POS_VELZ_P` at its minimum is in the bounds sweep's `KNOWN` list. With them the repository's default tests pass, and so do CI's release sweeps.
- **The three tuning tests: waiting**, on the owner's yes. Each prints why and returns, until the tuning procedure is changed in gate 2's record; `OXIDE_TUNE_FLY_WAITING=1` flies them all the same (`crates/oxide-bridge/tests/arducopter_tune.rs`). The fourth, that the hover flight repeats bit for bit on two instances, passes and does not wait.
- **The other tests of ArduCopter's flights: regenerated, on the owner's yes.** The thirteen tests below hold what run 0 of each flight does on the new IMU, the stick test holds gust-hold 0's new hashes, and ADR-0006's rows and read-back figures are those measurements. With them all 74 tests that fly the pinned binary pass, the gate program's two trials among them (one more only measures wall time and was not run). What the redraw of the IMU's noise changed in a single flight, beyond times and distances of a few tenths:
  - **`motor-loss`:** the vehicle is on the ground 2.2 s after the fault as before, but its motors, still on, now drag it along the ground to 237 m from home, with no mode change. With the old IMU it stayed within 24 m and the estimator's failsafe switched to LAND. Both are crashes, and in both the motors never stop, which is what criterion B3 measures.
  - **`gps-loss` and `gps-loss-pilot`:** the estimator's failsafe comes at the same time (8.0 s after the loss). The blind landing ends 13.6 m from home, where it was 21.5 m, and in `gps-loss` the estimator's lane switch is no longer reported.
  - **`imu-loss`:** the vehicle meets the ground at 9.5 m/s, where it was 11.0 m/s.
  - **The IMU read-back** has a truncation correction that holds for a bias near zero: all six axes within 0.09 mrad/s, where the old correction left 0.28 on one (ADR-0006 §4).

**Our core's flights** (the default test run; every one was run, and every one is now regenerated):

| Where | What moves | Test |
|---|---|---|
| `crates/oxide-sim/tests/data/scenario_hashes.txt` | 124 of its 129 trace hashes. They stay only for hover-step on all three cases (ideal sensors) and rc-loss-on-ground on F405 runs 0 and 1 | `scenario_files`: `every_scenario_file_still_flies_its_pinned_flight` (41 of its 43 move) and `…_full` (CI's release sweep) |
| `crates/oxide-sim-cli/src/main.rs`, `PINNED` (what `oxide-sim hash` prints) | 8 of 10 lines: gust-hold#0, rc-loss-position#0, pair-gps-loss+baro-loss#0 and motor-loss#0 on both profiles. The two hover-step lines stay | `hash_prints_its_five_lines_as_before_then_one_per_logged_flight` |
| `crates/oxide-bridge/tests/runner_core.rs` | `GUST_HOLD_0_F405`, `GUST_HOLD_0_H743`, and gust-hold 0's `rms_v_m` (and `max_v_m`, which the test checks after it; `last_motor_on_s` is the flight's end and stays) | `the_core_f405_row_…`, `the_core_h743_row_…`, `the_core_row_of_gust_hold_0_pins_the_vertical_error_…` |
| `crates/oxide-bridge/tests/cli.rs`, lines 246 and 247 | the same two gust-hold 0 hashes | `the_core_routes_of_the_binary_fly_the_profile_they_name` |

One more, from CI's release sweeps (the other three pass on the rebased branch: every fault file on its seeds and both profiles, the replay sweep and the replay damage). `crates/oxide-sim/tests/param_bounds.rs`, `every_bound_flies`, fails on one bound: `POS_VELZ_P` at its minimum, 0, in rc-loss-position on seed 0, climbs to 314 m (the sweep's limit is 200 m). A zero vertical velocity gain leaves the vertical speed uncontrolled on either IMU. Flown on seeds 0 to 19 on the main line's estimator, the old IMU goes above 200 m on 2 of them and is a fly-away by ADR-0007 §4 on 9; the new one on 5 and 11. The old IMU passes the sweep because its seed 0 stays at 114 m. This is the vertical twin of the `POS_VELX_P` entry already in the sweep's `KNOWN` list. It was added there with the pins.

The regeneration: `OXIDE_REGENERATE=1 cargo test --release -p oxide-sim --test scenario_files -- --ignored regenerate_scenario_hashes` for the table (its 124 new values are those the failing test had printed, and its other five rows are untouched), and the new values copied into the three other files from what `oxide-sim hash` and the failing tests print. `crates/oxide-bridge/src/record.rs` uses an old gust-hold 0 hash as sample data in a formatting test. That test passes and needs nothing. One test asserted the old model itself and is rewritten on this branch: `imu_period.rs`'s `the_noise_is_the_1000_us_noise_and_with_period_0_the_500_us_noise`, which is now `the_noise_per_sample_is_the_same_at_1000_us_and_at_the_500_us_step`.

**ArduCopter's flights** (the tests that need the pinned binary, which skip without it). They were run here on the binary of ADR-0006 (checksum checked), on the main line and on this branch, on separate instance ranges, before the rebase and again after it. Both times gave the same thirteen tests and the same first values. Thirteen passed on the main line and failed on this branch before their pins were regenerated. Each stopped at its first value that moved; the table is that first value, kept as the record of what moved:

| Test | First value that moved (on the new IMU, against the old pin) |
|---|---|
| `arducopter_faults`: `a_brain_hang_is_not_noticed_…` | lands 14.99 m from home, the pin reads "15." |
| `…`: `a_lost_radio_in_guided_with_the_brain_alive_…` | disarm at 36.04 s, pin 35.5 ± 0.3 |
| `…`: `battery_critical_arms_returns_at_low_lands_at_critical_…` | disarm at 43.40 s, pin 43.88 ± 0.3 |
| `…`: `battery_low_arms_and_returns_10_s_after_the_threshold_…` | "Battery 1 is low" at 33.82 s, pin 33.52 ± 0.05 |
| `…`: `gps_loss_ends_in_the_estimator_failsafe_8_s_later_…` | 6 status texts after the loss, pin 9 (the estimator failsafe and LAND now come at 23.0 s) |
| `…`: `imu_loss_stops_the_program_…` | touchdown 9.54 m/s, pin 11.0 ± 0.5 (still a crash) |
| `…`: `motor_loss_is_not_answered_…` | modes STABILIZE, GUIDED: the estimator's switch to LAND, 3.4 s after the fault on the old model, does not come |
| `…`: `no_human_link_is_the_radio_failsafe_first_…` | disarm at 36.24 s, pin 35.9 ± 0.3 |
| `arducopter_sticks`: `gps_loss_pilot_ends_in_an_ekf_failsafe_land_…` | the violation texts: 3.40 m reached (3.39), disarm read at 41.41 s (40.73) |
| `…`: `kill_in_rtl_stops_the_motors_…` | touchdown 15.554 m/s, pin 15.466 ± 0.05 |
| `…`: `rc_loss_override_leaves_the_return_for_guided_…` | greatest height 14.851 m, pin 14.791 ± 0.05 |
| `…`: `rc_loss_position_lifts_on_the_delayed_throttle_…` | last motor command at 73.313 s, pin 73.8705 ± 0.003 |
| `arducopter_fly`: `the_imu_arducopter_reads_is_our_models` | seed 2 axis 0 residual 0.448 mrad/s, limit 0.3. That axis's bias is now 0.075 mrad/s after the walk (it was about 0.73), so the reports straddle zero, where the test's truncation correction does not hold. Re-pinning it needs another seed or another correction, not a new number. The figures of ADR-0006 §4 that come from this test (0.05 and 0.28 mrad/s) are measurements of the old model |

The nine start-up tests of `arducopter_boot` pass on both sides when run alone. Run side by side with everything else, two or three of them failed on one side or the other, a different set each time: they depend on the machine's load, not on this change.

The tuning tests, run on the rebased branch: `the_tuner_runs_to_its_messages_and_saves_its_gains_on_landing` and `a_fresh_run_of_the_procedure_writes_the_committed_tuned_file_byte_for_byte_on_two_instances` fail, since yaw's tuning does not complete. `the_hover_learns_the_thrust_and_the_spectrum_has_no_peak` fails on its range for the spectrum's largest line (1.0 to 2.0 times the median; now 2.03 to 2.47, because the filter slopes the 40 to 400 Hz band), though the no-peak verdict and the learned hover thrust (0.1495) hold. The spectrum's rule is part of the procedure (ADR-0007 §2 step 4), so all three wait for it. `the_tuning_flight_repeats_bit_for_bit_on_two_instances` passes.

The tests of `arducopter_fly`, `arducopter_scenario` and `arducopter_sticks` that fly ArduCopter on gust-hold runs 0 to 5 were not run while gate 1 was open. They were run when the pins were regenerated, as the repository's tests, and nothing of them is used as a figure: nine hold limits or repeats and pass unchanged, and the stick test's three gust-hold 0 hashes moved and are regenerated (its hover-step hashes fly ideal sensors and stay). `gate_real`'s two trials of the gate program on a small plan were run too, from the clean commit, and pass. Not run: `gust_hold.rs`' `gust_hold_200_seeds`, which flies our core on runs 0 to 199 against the file's limits.

**Records that stay:** `docs/gates/gate-1/` and `docs/gates/gate-1/run-2/` are the records of the runs at `2c2fedb` and `92fdaa2`, and their hashes are true of those commits.

## Consequences

- Gate 1 is closed on the old IMU: both of its runs flew it. From the merge of this branch every flight, of either autopilot, flies the new one. Every pin is regenerated, and ADR-0006 §4's read-back figures and §6's rows are measured on the new model. ArduCopter's tuning procedure is changed in gate 2's record; until then the committed tuned file stays, as the file the procedure wrote on the old IMU, and the tests that fly the procedure wait for it.
- The F405 and H743 profiles become comparable on the IMU. A difference between them now measures the core's rates, not the IMU's sampling.
- The noise a reader gets no longer depends on its period. A bridged flight could therefore read the IMU at every 500 µs step without the √2 of ADR-0006 §4. The 1000 µs grid is kept anyway, so that this record changes one thing.
- The simulation of our core is slower (F405 by about 70 %, H743 by about 10 %), and still well inside DESIGN §5.1.
- Still not modelled: the filter's delay on the signal (DESIGN §5.4, S2), and filtering of the rotor lines. At the reference quad's full speed (30,000 rpm) the rotor line reaches 500 Hz, the F405 reader's Nyquist frequency, and there a 1 kHz and a 4 kHz reader still see it differently. In gust-hold the rotors run near hover, where the line sits at about 225 Hz.
- The white-noise densities keep the model's convention, standard deviation `D·√f` at an unfiltered rate `f`. MPU-6000-class datasheets quote noise as `D·√B` over a one-sided band `B`, which would be `D·√(f/2)`. The model's white noise is therefore √2 above the datasheet's figure. That was so before this record and is not changed by it.
