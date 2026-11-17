# ADR-0006: The ArduPilot bridge

Status: accepted as the design (M1d). The seam of §2 is built; the bridge is not built yet. The measurements below come from a prototype that was thrown away.

## Context

DESIGN §8 compares our flight core with ArduPilot and PX4 at benchmark gates, in our own simulator, and asks for the semantics of each bridge to be settled before the bridge is written: who owns time, which sensors come from whom, how fast it runs and whether a flight can be repeated.

ArduPilot is GPL-3 and a behaviour reference only (DESIGN §10). It runs as its own program, and nothing of it is copied into this repository. ArduCopter's simulator build (SITL) has an interface for an external physics program, called JSON: SITL sends its motor outputs in a UDP packet, the physics program answers with the vehicle's state as a line of text.

Two spikes produced the facts. The first read ArduPilot's sources at the commit of the pinned release. The second flew our reference quad under the released binary with a prototype that used our physics (`oxide_sim::vehicle`). An independent re-run reproduced the results marked "re-run" below.

**Pinned:** ArduCopter V4.7.2, commit `fb773567896014da007bca58358704d4a0d8516f`, the project's own prebuilt binary `SITL_x86_64_linux_gnu/arducopter` from `firmware.ardupilot.org/Copter/stable-4.7.2/`, 7,235,704 bytes, SHA-256 `caed35e4cb796ec06cac40b09566a18251771486b1cbac5e2bf16b7a5ed5045d`.

What was measured, and shapes the design:

- **It flies.** With only `FRAME_CLASS 1` and `FRAME_TYPE 1` set, ArduCopter arms, takes off, holds, steps and lands our quad. Its motor order and spin directions for a Quad X are ours, rotor for rotor; a swapped order or a reversed spin flips the vehicle at once (re-run).
- **We own time.** SITL's clock is the sum of the time stamps we send, and it waits for our reply before it moves. Its own time is the same at 1, 10 and 100 times real time.
- **A flight repeats bit for bit** (re-run): three flights from fresh directories gave the same 160,055 motor packets, also at another speed and on another instance. The conditions are in §8. With SITL's file log on, the arming moment moves by one loop tick between runs and the flights drift 3 to 6 mm apart.
- **ArduCopter needs 41.2 simulated seconds on the ground** before it will arm in a mode that uses GPS (22.0 s in a mode that does not).
- **Speed.** One simulated second costs about 0.1 s of wall time for one pair of processes, and 0.044 s when both are pinned to one core: the limit is the hand-over between the two processes, not arithmetic. Three pairs side by side gave about 52 simulated seconds per second together (re-run).
- **SITL makes its own GPS, barometer and compass** from the true state we send. Our sensor models do not reach it. Its GPS has no setting for horizontal position error. Its settings for IMU noise do nothing on this interface: the IMU values are the ones we send.
- **ArduCopter's default gains oscillate on this airframe** (thrust-to-weight 5): roll, pitch and yaw rate all show a limit cycle. With reduced gains it flies cleanly, and its automatic tuner then runs to the end in 424 simulated seconds and saves its gains.
- **Its reactions to failures** were observed one by one (§6).

## Decision

### 1. ArduCopter is a separate program, pinned by checksum

The bridge starts the SITL binary as a child process and talks to it over sockets on the local machine. The binary is never committed, bundled or released with this project. The repository records its URL, version, commit and SHA-256; the runner refuses a binary whose checksum is not the recorded one, unless told to accept it, and writes the checksum it found into the flight's record.

### 2. One seam for every autopilot

The simulator's loop calls an `Autopilot` (`crates/oxide-sim/src/autopilot.rs`): one `tick` that sees what the simulator knows at that instant and returns motor commands, with the time from which they are valid. The simulator flies a command from that time on, so each autopilot states its own delay (§3). Our flight core is one implementation, the bridge is another, and PX4's will be a third. The step record carries what every autopilot has (the motor commands and whether it is armed) and, as an optional part, what only our core reports (its status, events and estimate).

A flight is judged from the physics: where the vehicle went, how far it tilted, how hard it landed. Expectations that need our core's own states cannot be judged for another autopilot. They are reported as **not judged**, never as passed, and a flight with an expectation that was not judged does not count as passed.

### 3. Time

- **Lockstep, and the simulator owns the clock.** One reply per motor packet. The physics never advances on a packet whose frame counter repeats.
- **The step of a bridged flight is 500 µs.** ArduCopter's loop is 2500 µs, so it gets five steps per loop. Steps of 250, 500, 1000 and 1250 µs all flew with the same tilt and horizontal error; 2500 µs cannot arm, because ArduCopter demands an IMU rate of 1.8 times its loop rate. 500 µs divides the loop evenly at half the cost of 250 µs. Our core's own profiles keep their steps (1000 µs and 250 µs): no existing flight changes.
- **Time stamps.** The reply's time stamp for step *k* is *k* × (step + 1 ns). SITL adds the differences into a whole number of microseconds and truncates, so exact steps make its clock run 0.04 % slow; the extra nanosecond makes it exact (measured).
- **Framing.** Every reply starts and ends with a newline. SITL drops a first reply that does not start with one.
- **A flight starts with a pre-roll.** For a fixed 45 simulated seconds the vehicle rests on the ground with no wind and no faults while ArduCopter starts its estimator. Scenario time zero is the end of the pre-roll. A flight in which ArduCopter is not ready to arm by then has failed to start and is reported as that, not as a flight.
- **Commands act at once.** The motor packet that answers the state at *t* drives the physics from *t*. This is how ArduPilot's own simulator and its JSON examples work, and it gives ArduCopter no handicap. Our core keeps its rule that a command acts one tick after the tick that computed it (DESIGN §4.2), so it flies with up to 1 ms more delay than ArduCopter.

### 4. What ArduCopter senses

| Quantity | Where it comes from | Setting |
|---|---|---|
| Gyro and accelerometer | **Our IMU model's samples**, sent in the reply: the same noise, bias and vibration model our core flies on | `SIM_DRIFT_SPEED 0` (SITL otherwise adds its own slow gyro drift) |
| GPS | SITL's, from the position and velocity we send | 5 Hz as ours; `SIM_GPS1_LAG_MS 150` (our delay); `SIM_GPS1_VERR 0.1` (our velocity noise) |
| GPS horizontal error | **Ours, added to the horizontal position we send**: 0.3 m per sample, as our GPS model draws it, held between GPS samples. It starts after the pre-roll, so ArduCopter's origin is set without it | none exists in SITL |
| Barometer | SITL's, from the height we send | `SIM_BARO_RND 0.173` (measured: 0.10 m, our noise) |
| Compass | SITL's, from the attitude we send | `SIM_MAG_RND 27` (about 0.5 µT, our noise) |
| Battery | Our pack's voltage and current, sent in the reply | `BATT_MONITOR 4` |
| Radio | Our pilot's sticks, sent in the reply as `rc` channels | |

The GPS error row is the owner's decision: without it, a comparison of position hold measures the GPS models and not the autopilots. In a first look our core held position within 0.035 to 0.044 m RMS in gusts on clean sensors and within 0.15 to 0.22 m on its noisy GPS, while ArduCopter, whose simulated GPS had no horizontal error, held within about 0.10 m. Before the gate is run, a test must show that ArduCopter's reported GPS position carries the added error and that nothing else it senses changed. If that cannot be shown, the gate removes the horizontal GPS error from our core's model instead, and the gate report says so.

What cannot be made equal, and is listed in the gate report: our barometer's random offset at start-up (SITL has none); the height the GPS reports (SITL's error is a slow sine, ours is random); ArduCopter's GPS, barometer and compass are sampled by SITL's code, not ours. Rangefinder and airspeed are not sent: we do not model them, and SITL's rangefinder input on this interface is faulty at the pinned version.

### 5. Motors

ArduCopter's output *k* drives our rotor *k*. With `A = (pwm − 1000) / 1000`, the command is `u = clamp((A − spin_min) / (spin_max − spin_min), 0, 1)` with ArduCopter's `MOT_SPIN_MIN` and `MOT_SPIN_MAX` as set in the committed parameter file, and 0 for a pulse below 1000 µs. `MOT_THST_EXPO` is set to the airframe's own thrust exponent, so that ArduCopter's linearisation of thrust is right for this airframe. While disarmed ArduCopter sends 1000 µs, which is command 0.

### 6. Pilot, commands and faults

- **Stick flying** goes through the `rc` channels of the reply, clocked by our time stamps. The channels are numbered by our own choice (`oxide_bridge::bridge::rc_pulses`): 1 roll, 2 pitch, 3 throttle, 4 yaw (1000 to 2000, 1500 at the centre), 5 mode switch (positions 0, 1, 2 as 1000, 1500, 2000), 6 arm switch (1000 off, 2000 on), 7 kill switch (1000 off, 2000 on), 8 unused (1000). The part that runs the real program maps channels 5 to 8 to ArduCopter's own mode and auxiliary-function parameters; it must not assume other numbers. The pilot's modes map to ArduCopter's: Stabilized to STABILIZE, Altitude to ALT_HOLD, Position to LOITER.
- **Offboard flying** goes over MAVLink in GUIDED: arm, take off, then position targets. Commands are sent from the physics thread at fixed simulated times, and the set of requested telemetry messages is fixed, because both change the instant ArduCopter acts and with it the whole flight.
- **MAVLink is read all the time.** SITL stops when its send queue fills, which would hang both processes.
- **Faults** reach ArduCopter by the route that works, observed in the spike:

| Our fault | How it is applied | What ArduCopter did (defaults unless said) |
|---|---|---|
| GPS lost | `SIM_GPS1_ENABLE 0` | EKF failsafe after 8.0 s: lands (re-run) |
| Barometer lost | every `SIM_BAR*_DISABLE 1` | no failsafe: climbed without limit (157 m in 50 s) |
| Compass lost | every `SIM_MAG*_FAIL 1` | nothing in a hover |
| IMU lost | `SIM_ACC_FAIL_MSK`, `SIM_GYR_FAIL_MSK` | its main loop stops; motors to idle after 2.0 s; the vehicle falls |
| Radio lost | the `rc` channels stop | radio failsafe after 1.0 s: returns and lands. `SIM_RC_FAIL` does nothing on this interface |
| Ground station lost | its heartbeats stop | nothing; with `FS_GCS_ENABLE 1`, returns after 5.1 s |
| Motor lost | our physics gives that rotor no thrust | tumbles; no response that saves or stops it. Needs `SIM_FLOAT_EXCEPT 0`, or SITL aborts |
| Battery low, critical | our pack voltage in the reply | warns only; with `BATT_FS_LOW_ACT 2` returns, with `BATT_FS_CRT_ACT 1` lands |
| Geofence | `FENCE_*` | returns (climbs to 15 m first) or lands; refuses a target outside the fence |

A fault applied by a parameter is written at the fault's time and takes effect when ArduCopter next reads MAVLink, a few milliseconds later; the time of its acknowledgement is recorded.
- **The companion computer** (our "brain") is a second MAVLink link, so that losing the brain and losing the ground station stay different faults. Whether ArduCopter accepts position targets on the second link is to be proven when the bridge is built; if it does not, one link carries both and the scenarios that need the difference are not judged for ArduCopter.

Which of ArduCopter's failsafes are switched on for the gate, and which scenarios are scored, is decided in the gate-criteria record, not here.

### 7. Wind and seeds

A seed gives both autopilots **the same wind, sample for sample**. The wind's turbulence is drawn once per step, so a bridged flight (500 µs) draws its wind on the 1000 µs grid of the F405 profile, and a test holds the two wind traces equal bit for bit.

ArduCopter's own random numbers (its barometer and compass noise) are never seeded: they are the same sequence in every run. Over many seeds ArduCopter therefore sees varying wind, IMU noise and GPS error, which are ours, and one fixed realisation of its barometer and compass noise.

### 8. Repeatability

A bridged flight repeats bit for bit when: SITL's file log is off (`LOG_BACKEND_TYPE 0`); the start time is fixed (`--start-time`); the working directory is new and empty and the stored parameters are wiped (`-w`); the parameter file is the same; every command is sent at a fixed simulated time; the set of requested messages is the same.

The runner can fly any bridged flight twice and compare the trace hashes, and the gate does so for a sample. Bridged flights stay outside the determinism contract between operating systems (DESIGN §5.2): they are as repeatable as the binary is, on the machine it runs on.

### 9. Running many flights

- Each flight is one new SITL process in its own empty directory, with its own ports (`-I n` shifts them by 10 n). SITL is never restarted over MAVLink: that keeps its stored parameters.
- Each simulator-and-SITL pair is pinned to one core.
- SITL runs with a large speed-up and its time synchronisation on, which keeps its default estimator (EKF3).
- A child that dies or stays silent past a wall-clock limit ends that flight as failed, with the reason. The wall clock decides nothing else.

### 10. Where the code lives

A new crate, `oxide-bridge`, holds the JSON exchange, the SITL process runner, the MAVLink client and the translation of pilots and faults. It depends on `oxide-sim` and on the `mavlink` crate, version 0.21.0 with its default features off and `std`, `transport-tcp` and `dialect-ardupilotmega` on (16 new crates in the lock file; MIT or Apache-2.0). `oxide-core`, `oxide-log` and `oxide-sim` gain no dependency.

### 11. What is recorded for each bridged flight

The binary's checksum, version and command line; the parameter file's hash; every parameter written during the flight with the time of its acknowledgement; every MAVLink command sent with its time; a hash of the motor packets and of the replies; ArduCopter's mode changes and status texts with their times; the seed, the scenario file's hash and our commit.

## Consequences

- Gate 1 can be flown. From the measured speed, 500 gust flights of 40 s (85 s with the pre-roll) should take about a quarter of an hour with three pinned pairs, and under an hour and a half with one. This is arithmetic from the measurements; the batch has not been run.
- Half of every bridged flight is pre-roll.
- The comparison is between whole systems: ArduCopter's estimator and controller against ours. The gate also reports our core on true state, to separate the two (gate-criteria record).
- ArduCopter flies with no computation delay and our core with one tick of it; the difference is at most 1 ms.
- ArduCopter must be given gains before it can be tuned: its defaults oscillate on this airframe, and its automatic tuner refuses a vehicle that is not steady. The committed parameter file therefore starts from reduced gains, which is the manual step ArduPilot's own tuning guide puts before the tuner.
- The first MAVLink code in the workspace arrives with the bridge. The flight core's own MAVLink link (DESIGN §4.13) is a later, separate piece of work.

## Deferred

- PX4's bridge (gate 2). PX4 takes every sensor from the simulator, so its bridge sends our sensor models' frames; the seam already carries them.
- Running bridged flights in CI. It needs the binary on the CI machine, which is the owner's decision.
- Other ArduPilot vehicles, and airframes other than the reference quad.
- A notch filter set-up for ArduCopter driven by our vibration model: SITL cannot be told our rotor speeds on this interface.
