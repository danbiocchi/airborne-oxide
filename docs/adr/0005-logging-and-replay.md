# ADR-0005: Logging and replay

Status: accepted (M1c part three)

## Context

DESIGN §4.15 asks for logs in PX4's ULog format with PX4's topic names where a counterpart exists, two levels (normal and replay), a logger that drops and counts instead of blocking, and headers that carry the git hash, the build profile and a parameter hash. "Replay" means that a logged flight can be run again through the flight core, bit for bit.

Before this part the simulator could record a flight three ways, none of them a log: a `StepRecord` per tick in memory, a 38-column CSV that loses precision, and a trace hash over the physics and the motor commands. The trace hash does not cover the status or the events, so it cannot prove a replay.

Three facts shaped the design. They come from reading the format's reference reader (`pyulog`), PX4 Flight Review's source and the core's own code.

- **Existing tools are strict in unexpected places.** `pyulog` treats a data record that includes trailing padding as corrupt and silently drops the topic. Flight Review loads only a fixed list of PX4 topics, and fails the whole page if `vehicle_attitude` or `vehicle_attitude_setpoint` is present without its quaternion.
- **The core is a function of very little.** Its outputs depend only on the configuration and profile it was built with, the report of the stored-parameter load, the time and inputs of every tick, and one call the store makes between ticks. It reads no clock and no static.
- **The core cannot be saved and restored.** A replay therefore starts at start-up and needs every tick, including ticks with no input.

## Decision

### 1. A crate for the log, in two layers

`oxide-log` is a new `no_std` crate without allocation:

- **The format layer** knows ULog and nothing about flight: the file header, the definition messages (flag bits, formats, information, parameters), the data messages (subscription, data, logged string, synchronisation, dropout), and the rules for padding. It writes into a byte sink.
- **The topic layer** knows the flight core's types and maps them to topics (§3 and §5). It depends on `oxide-core`.
- **A reader** (behind the `std` feature) parses any well-formed ULog into formats and records. The replay tool and the tests use it. It is as strict as `pyulog`, and a damaged or hostile file gives an error, never a panic: sizes are checked and nested formats are resolved without recursion.

The firmware will reuse the first two layers unchanged. What stays for hardware bring-up is the SD-card side: preallocated files, the sync interval, and placing the buffer in memory the DMA can reach.

### 2. The file

- **Header and definitions** follow PX4's writer: the flag-bits message first, then every format, the information messages and the initial parameters, and only then the first subscription. A topic's message id is its position in the list of topics, and the normal level's topics keep the same ids at the replay level.
- **`timestamp`** is the first field of every topic, `uint64_t`, microseconds: the time of the tick that produced the record. Where PX4's topic has `timestamp_sample`, it carries the sample's own time.
- **Field order and padding** follow PX4's generator (fields sorted by size, padding fields named `_padding`), and trailing padding is in the format but never in a data record. That is what `pyulog` requires.
- **Information keys:** `sys_name` is `Oxide`; `ver_sw` is the git hash; `ver_hw` names the board, or `SIM` in the simulator. Three keys are our own: `oxide_profile` (the chip profile), `oxide_config_hash` (§6) and `oxide_log_level`. The crate is given the build information by its caller and reads nothing from the environment; an empty string is written as `unknown`.
- **Parameters:** all 98 are written at the start, and again in the data section when one changes.
- **A synchronisation message** every 500 ms, as PX4 does, so that a reader can recover after damage. The interval is counted from the first message written.

### 3. The normal level: a flight any ULog tool can show

Topics that exist in PX4 use PX4's current name, fields and units:

| Topic | From | Rate |
|---|---|---|
| `sensor_combined` | the IMU sample | 100 Hz |
| `vehicle_angular_velocity`, `vehicle_rates_setpoint` | estimate, rate target | 100 Hz |
| `actuator_motors`, `vehicle_thrust_setpoint`, `vehicle_torque_setpoint` | motor commands, the rate loop's output | 100 Hz |
| `vehicle_attitude`, `vehicle_attitude_setpoint` | estimate, attitude target | 50 Hz |
| `vehicle_local_position`, `vehicle_local_position_setpoint` | estimate, guidance target | 20 Hz |
| `manual_control_setpoint` | the radio sample | 10 Hz |
| `vehicle_magnetometer`, `vehicle_air_data` | magnetometer, barometer | 10 Hz |
| `battery_status` | the battery sample and monitor | 5 Hz |
| `vehicle_status`, `failsafe_flags` | the status | on change, and 1 Hz |

What has no PX4 counterpart goes in our own topics: `oxide_status` (validity flags, pre-arm flags, failsafe reasons and action, timing faults; on change, and 1 Hz), `oxide_estimator` (health counters; 5 Hz), `oxide_event` (below) and `oxide_logger` (§7; 1 Hz).

- **Conventions are PX4's.** Quaternions are w first, body to world, which is already the core's. Thrust is normalised by the rotors' total maximum thrust and is negative upward. Torque is normalised by the largest torque the rotors can produce about each axis (for the reference quad: 32 N, and 1.27, 1.27 and 0.26 N·m). The magnetic field is in gauss. The conversions are in one place and tested.
- **Time stamps.** `timestamp` is the tick time. `timestamp_sample`, on topics made from the estimate or the commands, is the time of the newest IMU sample.
- **A PX4 field the core has no value for** gets PX4's own "unknown": the invalid value its message declares, NaN where PX4 says NaN means unknown, otherwise zero. `battery_status.voltage_cell_v` carries the core's one filtered per-cell voltage, repeated for each cell.
- **States are translated, not renumbered.** `vehicle_status.nav_state` carries PX4's number for the closest PX4 state, so Flight Review colours the flight modes. The core's own state is in `oxide_status`, with codes that are fixed in a table and never follow the order of an enum.
- **Events are text and data.** Each event is written as a logged string ("Disarmed: touchdown", "Reason raised: RC_LOST"), which every tool shows, and as an `oxide_event` record for programs.
  - The text is put together from fixed pieces, without formatting code, and carries the event's own time.
  - Its level is emergency for a termination and for loss of control; warning for a raised reason, a refused arming, a refused parameter and a parameter-load fault; information for the rest.
  - The record carries the event exactly: its time, its kind by a fixed code, and its payload as numbers, with a parameter identified by its stable key.
  - A changed parameter is also written as a parameter message with its new value.
- **Decimation is by time, on the tick clock**, so the same flight gives the same file. A topic's next time moves along its own period, so a pause gives one record, not a burst. The first record is written on the first tick that has the data: the topics made from the estimate start with the first estimate, about one second after start-up.
- **A sensor that stops leaves a gap, as in PX4.** The five topics made from an input sample (`sensor_combined`, `manual_control_setpoint`, `vehicle_magnetometer`, `vehicle_air_data`, `battery_status`) are written when they are due and a sample has arrived since their last record. A dead barometer therefore shows as a missing trace, not as a flat, live one. When the samples return, the first is written at once and the topic goes on along its old grid. A receiver that still sends frames marked as lost is a sample, so `manual_control_setpoint` goes on in that case.
- **The tick clock is expected to run forward.** If it steps back by more than a topic's period, the topic goes on from the new time, and one synchronisation message is written at once.
- **"On change" means a change in a field the topic carries.** A status topic is also written one second after its last record.

The core gains one read-only accessor, `Core::telemetry()`, for values it computes and did not expose: the attitude and rate targets, the thrust and torque the rate loop commands, the position and velocity setpoints, the landed flag. Reading it changes nothing.

### 4. What is not in the normal level yet

- **`vehicle_gnss`.** PX4's topic is geodetic and the core's GPS sample is local. It waits for a geodetic origin, which arrives with a real GPS driver. Flight Review's map is therefore empty.
- **`cpuload`.** It needs the firmware's measurement.
- **`estimator_status`.** PX4's topic carries a test ratio for each fused measurement, a reset counter for each axis and bits for specific faults. The estimator reports less: whether it is aligned and aided, how many measurements it rejected and how often it reset (`oxide_estimator`). A log must not carry a zero the core did not compute, so the topic waits for an estimator that reports these values. Flight Review's estimator-flags plot is therefore absent.
- **Consumed charge and state of charge.** The battery monitor works on voltage and has no capacity, so `battery_status.discharged_mah` and `remaining` carry PX4's value for unknown, -1. Flight Review draws them as flat lines.
- **Topics Flight Review can plot and the core has no data for:** `manual_control_switches`, `vehicle_imu_status`, `esc_status`, `vehicle_angular_acceleration`.
- **PX4's event topic with its metadata.** Logged strings do the job.

### 5. The replay level: everything the core saw

A replay log is a normal log plus an exact record of the core's boundary.

- **At the start:**
  - the vehicle description (mass, inertia, rotors, thrust curve, battery), one record per number with its name;
  - the exact microseconds of the eleven duration parameters (`oxide_param_us`). A parameter message holds a duration as `f32` seconds, which names the microsecond only up to 16 s;
  - the profile's name and rates, and the report of the stored-parameter load.

  These are data records, written right after the subscriptions. With the 98 parameters that is the whole configuration. A file whose values do not rebuild exactly one configuration is refused, with the parameter named: a replay never runs against a configuration that is nearly the one flown. The same check runs when a replay-level log is started, so a configuration that could not be replayed is refused before the flight, not after it. A record carries at most eight motors.
- **Every tick, in this order:** a tick record (`oxide_in_tick`) with the time, a sequence number and which inputs are present; one record per input present, with both of its time stamps and its values as the exact bits; the outputs record (`oxide_out`: motor commands, every status field, the number of events); then the tick's events and the normal level's records. The replay topics take the message ids after the normal level's twenty.
- **Events:** the `oxide_event` records of the normal level already carry each event exactly, so the replay level has no second event topic. The outputs record says how many belong to its tick.
- **Between ticks:** the store's "saved" call is a marker on the next tick record. The logger does not intercept the call. It sees that the core's unsaved flag was set after one tick and is clear before the next, and nothing else clears it. A call that changes nothing (the flag already clear, or the vehicle armed) leaves no marker and needs none.
- **Parameter changes** are recorded by the parameter's stable key, not its position in the table. A change to a parameter this build does not have is recorded as such, so the core's refusal replays too.
- **A damaged record is refused, not read as something else:** a flag byte that is neither 0 nor 1, a code no table knows, a set bit no flag owns.

`oxide-sim replay FILE.ulg` rebuilds the core from the file, feeds it the recorded ticks in order, and compares each output with the recorded one bit for bit. It reports the number of ticks replayed and that they were identical, or the first tick that differs, which field, and both values.

- **Exact means bits.** Floats are compared as bit patterns, so a negative zero or a NaN payload counts.
- **A gap ends the proof.** The tool replays up to the gap, says what it found and exits with 1. A gap is a dropout, a missing sequence number, or a tick whose records are not all there or contradict each other. So is a replay record that belongs to no tick, and an event or start record stamped with another time than its tick or its start.
- **A file cut short by a power loss is not a gap.** That is how the log of a crash ends. The ticks before the cut are replayed, and the output says the file ends in a cut. A file that ends in zeros (a preallocated file) is read the same way.
- **Damage is not a cut.** One wrong bit in a message's size makes a reader lose its place, and what it finds next can look like the end of the file. So an apparent end is believed only when it can be one:
  - zeros end the data only when everything after them is zero;
  - a cut message must have the size its kind has (a data message, a synchronisation message and a dropout have a fixed one);
  - a cut message cannot declare more bytes than any message the file can hold.

  A file that fails the first two is refused. One that fails the third is replayed up to that point and reported as damaged, with exit code 1. What cannot be told apart from a power loss is damage inside the last tick's own records: that reads as a cut one tick earlier.
- **A file with no tick proves nothing** and is reported as such.
- **The first line of the output says `identical` only when the replay reached the end of the file or a cut.** A replay that stopped at a gap says `no difference before the replay stopped`, then why.
- **A configuration that does not rebuild is named in a sentence:** `cannot rebuild the configuration: parameter RATE_ROLL_P is refused: its value is out of range`.
- **Replay through changed code is the point.** After a change to the core, a difference is the answer, and the tool shows where the behaviour first departs. `--param NAME=VALUE` replays through a changed setting instead.

### 6. The configuration hash

`oxide_config_hash` is a 64-bit FNV-1a over the whole configuration, the vehicle description included, in a fixed order. Two logs with the same hash and profile were flown with the same settings. A hash over the parameter table alone would miss the mass and the rotors.

The ten durations enter the hash twice: as the seconds a parameter message shows, and as the exact microseconds the core holds. Which parameters are durations is stated once, in the core's parameter table, and the log's `oxide_param_us` records follow that list.

### 7. The logger never blocks the flight

Records go into a fixed ring buffer of whole messages; a lower-priority task drains it to storage.

- **When the buffer is full the record is dropped and counted.** When room returns, a dropout message with the length of the gap is written first, as PX4 does.
- **Events, parameters and status may use the whole buffer;** streamed data stops short of the last quarter, so a burst of samples cannot push out the message that says why the vehicle landed.
- **`oxide_logger`** reports messages written and dropped and the buffer's high-water mark once a second, because PX4's dropout message has a duration and no count. The count for streamed data is exact. A status record that did not fit is tried again on the next tick, so the count for events and status is of attempts. The counters are 32-bit and stop at their maximum instead of wrapping: after 63 days at the normal level, 12 days at the replay level on the F405 profile and under 4 days on the H743 profile.
- A message is in the buffer whole or not at all, and so are the dropout and synchronisation messages written ahead of it.
- The recorder is the buffer plus under 1 kB of state (784 bytes), and a tick on which nothing is due costs a few comparisons (about 20 ns on a desktop; a tick on which everything is due, about 3 µs). On the Cortex-M build it uses no double-precision arithmetic, and a 64-bit division only when a topic is a whole period late or an event's text holds a number.
- A recorder is started once. A second start is refused and changes nothing.

In the simulator the buffer is 64 kB and is drained to the file after every tick, so nothing drops at either level; a test holds that on both profiles. A gap in a log can therefore only come from a real logger, or from a test that drives the recorder with a slow sink. A write error ends the log and is reported when the flight is over. It never changes the flight.

### 8. Commands

- **`--log FILE.ulg`** on `oxide-sim run` and `oxide-sim scenario NAME|PATH` writes the flight's log. `--log-level normal|replay` chooses the level (default `normal`) and is an error without `--log`. `hash` and `montecarlo` do not take them.
- **Output files** (`--log`, `--csv`) are created before the flight, so a path that cannot be written is an error before the first tick, and a command that fails before its flight leaves no file behind. An error names the file: `error: FILE: No space left on device (os error 28)`. If a write fails during the flight, `scenario` still prints its summary and verdict, then the error.
- **Exit codes of every command:** 0 passed; 1 a run the command judged as failed (a scenario's violations, a failing Monte-Carlo run, a replay's difference or gap); 2 an error (a bad argument, an unreadable file, an output that cannot be written). An error takes precedence over a failed run.
- **`ver_sw`** is the git hash the binary was built from, with `-dirty` when the tree had changes, and `unknown` without git. The dirty mark is the state when the build script last ran.
- **`oxide-sim replay FILE.ulg [--param NAME=VALUE ...] [--continue]`** replays a replay-level log (§5). `--param` is applied to the rebuilt configuration before the first tick. `--continue` goes on after a difference and counts the ticks that differ.
- **Exit codes of `replay`:**

  | Code | Meaning |
  |---|---|
  | 0 | Every tick replayed is identical, and there was at least one. Also for a file that ends in a cut. |
  | 1 | A difference, a gap (§5), damage at the end of the file, or a file with no tick. |
  | 2 | The file cannot be replayed: not a ULog file, a file damaged before its end (§5), a normal-level log, a configuration that does not rebuild to the logged hash, an unknown profile, a vehicle that does not have four rotors, a refused `--param`. |

- **`oxide-sim hash`** prints, after the five trace hashes of a profile, the FNV-1a 64 of the bytes of three logs: `rc-loss-position` and `gust-hold` at the normal level and `rc-loss-position` at the replay level. They are written with a fixed `ver_sw`, so the value does not move with the commit. CI compares the whole output between Linux and Windows.

### 9. Proofs

- **The format:** `pyulog` reads the simulator's files at both levels with no corruption flag and finds every topic. For all twenty normal-level topics the values it reads equal, bit for bit, the values the crate's own reader gives. A CI job runs this with `pyulog` 1.3.0, and also checks a log written by the command line.
- **The values:** every field of every normal-level topic equals, bit for bit, what the simulator had at that tick: the input sample, the core's estimate and targets, the status, the events. The test recomputes each conversion itself, over 34 flights on both profiles, and fails if a field is left unchecked or a flag is never seen both set and clear. Nine deliberate wiring mistakes (a zero angular velocity, swapped battery flags, the wrong thrust source, and so on) each fail it.
- **Sensor loss:** in the flights that lose the IMU, the barometer or the magnetometer, that sensor's topic stops within one period, and no record of a sensor topic that carries its sample's time holds a sample older than one topic period plus one sensor period.
- **Determinism:** the same flight gives the same bytes on every run, and the hashes of three logs are compared between Linux and Windows (§8).
- **Logging changes nothing:** a flight with a log has the same trace hash as one without, on three scenarios and both profiles. The ten trace hashes and the 129 pinned flights are what they were before this part.
- **Replay, every scenario:** each of the 43 scenario files, on both chip profiles, is flown with a replay-level log and replayed: 86 flights and 17.5 million ticks, all identical. CI runs it in release (about 30 s).
- **Replay, the boundary outside the inputs:** flights with a parameter change, a refused change, an unclean stored-parameter load with its acknowledgement, a refused arming, a recovered load and a store save all replay identically. With the "saved" marker or the load report edited out of the file, the replay differs: both are needed.
- **Replay detects change:** a core with one gain changed first differs 24 ticks after arming, and the tool names the field. One flipped bit in a recorded gyro sample, or in any field of a recorded output, is found at its tick and named.
- **Replay refuses damage:** a file cut anywhere, a dropout, a missing tick, a record missing from a tick, a record out of place, a changed time stamp, a normal-level file, a file with another configuration hash. None replays as identical beyond what the file holds, and none panics.
- **Replay, every bit:** each of the 281,072 bits of the data section of a 100-tick replay log is flipped in turn and the file replayed. No flip exits 0 with more than the last tick missing: 37,347 are refused, 70,158 stop with exit 1, 39,699 are reported as a difference, 133,838 change nothing the core's outputs depend on in that flight, and 30 fall in the last tick's own records and read as a cut one tick earlier. CI runs it in release. Before the rule in §5, 1,137 flips replayed as a clean, shorter flight.
- **Exact means bits:** a recorded `+0.0` against a replayed `-0.0`, and two NaNs with different payloads, are differences.
- **The buffer:** under any pattern of writes and drains the output is a valid sequence of whole messages, and every dropped message is counted. Streamed data stops at the last quarter of the ring while an event still fits, for two ring sizes.

## Consequences

- A flight in the simulator or on the bench is written for PlotJuggler and Flight Review: PX4's topic names, fields and units, and the fields Flight Review's source needs. `pyulog` reads the files in CI.
- Flight Review's plotting code draws these logs. It was run without its web server, at commit `80e0c0c`, on thirteen simulator logs: six scenarios on both profiles and one replay-level log. It ran to the end on each and drew the same 28 plots, with the flight modes as coloured bands and the events in its table of logged messages. On one log the plotted attitude, rates, position, velocity, battery and barometer values were compared with the log point by point, and they agree. The plots it could not draw are those whose topics §4 lists as absent. A replay-level log gives the same page as a normal one, because Flight Review reads only the topics it knows.
- PlotJuggler has not been run on a log. The source of its ULog loader was read against these files, and nothing in them should stop it.
- A bug seen in a flight can be reproduced on a desk from its log, and a fix can be shown to change the flight at the tick it should.
- A normal-level log is about 41 kB/s on either profile: 3.2 MB for the 80 s of `rc-loss-position`, of which 10 kB are the definitions.
- Replay logs are large. The tick record, an IMU sample and the outputs are 165 bytes a tick. A whole replay-level file of the simulator, with every sensor and the normal level's records, is about 219 kB/s on the F405 profile (1 kHz) and 714 kB/s on the H743 profile (4 kHz): 17.5 MB and 57 MB for those 80 s. Whether an SD card keeps up is measured at bring-up; the outputs record can be packed tighter if it does not.
- Replaying is fast: the 80 s flight replays in about 0.2 s on the F405 profile and 0.7 s on the H743 profile. The tool reads the whole file into memory and needs about four times the file's size, so an hour at the H743 profile's replay level (2.6 GB) wants about 10 GB.
- A log written on the chip is replayed on a desk. That is exact only if the core computes the same bits on both. The simulator's determinism rules are written for that (no library float functions, no fused multiply-add), and CI holds Linux against Windows, but the first replay of a log from real hardware is the test.
- The replay tool flies four rotors, like the rest of this build. The records hold up to eight.
- The codes of states, actions and events in a log are now a format. They are fixed in one table with a test, and a new variant gets a new code.

## Deferred

- The SD-card side of the logger, and making the buffer lock-free between interrupt levels (bring-up).
- Replay of a window that starts in flight. It needs the core's state to be saved and restored, which does not exist. DESIGN §4.15's "short triggered windows" on the F405 can be inspected but not replayed until then.
- `vehicle_gnss`, `cpuload`, `estimator_status`, consumed charge and state of charge in `battery_status`, PX4's event metadata (§4).
- Reading ULog and DataFlash logs of real flights into the simulator for comparison (M2).
- Log rates as parameters.
- Running PlotJuggler on a log, and looking at a log on Flight Review's own web page in a browser: only its plotting code has been run (see Consequences). The automated check stays `pyulog`, the strictest reader of the three.
- Replaying up to damage in the middle of a file. A file damaged before its end is refused whole today; only a cut at the end and a counted dropout are replayed up to.
- Reading a replay log as a stream, so that memory does not grow with the file.
- Keeping a statically allocated recorder out of initialised data. The recorder holds its ring, and its clocks start non-zero, so a static one costs its size in flash and a copy at start-up. At bring-up the ring moves into its own section of DMA-reachable memory anyway.
