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
- **Parameters:** all 93 are written at the start, and again in the data section when one changes.
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
- **Decimation is by time, on the tick clock**, so the same flight gives the same file. A topic's next time moves along its own period, so a pause gives one record, not a burst. The first record is written on the first tick that has the data.
- **"On change" means a change in a field the topic carries.** A status topic is also written one second after its last record.

The core gains one read-only accessor, `Core::telemetry()`, for values it computes and did not expose: the attitude and rate targets, the thrust and torque the rate loop commands, the position and velocity setpoints, the landed flag. Reading it changes nothing.

### 4. What is not in the normal level yet

- **`vehicle_gnss`.** PX4's topic is geodetic and the core's GPS sample is local. It waits for a geodetic origin, which arrives with a real GPS driver. Flight Review's map is therefore empty.
- **`cpuload` and `estimator_status`' innovation ratios.** They need values only the firmware or a later estimator has.
- **PX4's event topic with its metadata.** Logged strings do the job.

### 5. The replay level: everything the core saw

A replay log is a normal log plus an exact record of the core's boundary.

- **At the start:**
  - the vehicle description (mass, inertia, rotors, thrust curve, battery), one record per number with its name;
  - the exact microseconds of the ten duration parameters (`oxide_param_us`). A parameter message holds a duration as `f32` seconds, which names the microsecond only up to 16 s;
  - the profile's name and rates, and the report of the stored-parameter load.

  With the 93 parameters that is the whole configuration. A file whose values do not rebuild exactly one configuration is refused, with the parameter named: a replay never runs against a configuration that is nearly the one flown.
- **Every tick:** a tick record with the time, a sequence number and which inputs are present; then one record per input present, with both of its time stamps and its values as the exact bits; then the outputs record (motor commands, every status field, the number of events).
- **Events:** the `oxide_event` records of the normal level already carry each event exactly, so the replay level has no second event topic. The outputs record says how many belong to its tick.
- **Between ticks:** the store's "saved" call is recorded as a marker on the next tick.
- **Parameter changes** are recorded by the parameter's stable key, not its position in the table. A change to a parameter this build does not have is recorded as such, so the core's refusal replays too.
- **A damaged record is refused, not read as something else:** a flag byte that is neither 0 nor 1, a code no table knows, a set bit no flag owns.

`oxide-sim replay FILE.ulg` rebuilds the core from the header, feeds it the recorded ticks in order, and compares each output with the recorded one bit for bit. It reports the number of ticks replayed and that they were identical, or the first tick that differs, which value, and both versions. It exits non-zero on a difference.

- **Exact means bits.** Floats are compared as bit patterns, so a negative zero or a NaN payload counts.
- **A gap ends the proof.** A replay log with a dropout cannot be replayed past it: the tool replays up to the gap and says so.
- **Replay through changed code is the point.** After a change to the core, a difference is the answer, and the tool shows where the behaviour first departs.

### 6. The configuration hash

`oxide_config_hash` is a 64-bit FNV-1a over the whole configuration, the vehicle description included, in a fixed order. Two logs with the same hash and profile were flown with the same settings. A hash over the parameter table alone would miss the mass and the rotors.

### 7. The logger never blocks the flight

Records go into a fixed ring buffer of whole messages; a lower-priority task drains it to storage.

- **When the buffer is full the record is dropped and counted.** When room returns, a dropout message with the length of the gap is written first, as PX4 does.
- **Events, parameters and status may use the whole buffer;** streamed data stops short of the last quarter, so a burst of samples cannot push out the message that says why the vehicle landed.
- **`oxide_logger`** reports messages written and dropped and the buffer's high-water mark once a second, because PX4's dropout message has a duration and no count. The count for streamed data is exact. A status record that did not fit is tried again on the next tick, so the count for events and status is of attempts.
- A message is in the buffer whole or not at all, and so are the dropout and synchronisation messages written ahead of it.
- The recorder is the buffer plus under 1 kB of state, and a tick on which nothing is due costs a few comparisons.

In the simulator the buffer is drained to a file after every tick, so nothing drops. The tests use a slow sink to make it drop.

### 8. Commands

- `--log FILE.ulg` and `--log-level normal|replay` on `oxide-sim run` and `oxide-sim scenario`.
- `oxide-sim replay FILE.ulg`.

### 9. Proofs

- **The format:** `pyulog` reads every file the simulator writes, with no corruption flag, and the values it reads equal the simulator's. CI runs that.
- **Determinism:** the same flight gives the same bytes on every run and on Linux and Windows.
- **Logging changes nothing:** a flight with the logger on has the same trace hash as with it off.
- **Replay:** every scenario file, on both chip profiles, replays bit for bit. A flight replayed through a core with one gain changed is reported as different.
- **The buffer:** under any pattern of writes and drains the output is a valid sequence of whole messages, and every dropped message is counted.

## Consequences

- A flight in the simulator or on the bench opens in PlotJuggler and Flight Review with the plots a PX4 user expects.
- A bug seen in a flight can be reproduced on a desk from its log, and a fix can be shown to change the flight at the tick it should.
- Replay logs are large. The tick record, an IMU sample and the outputs are 165 bytes, so about 165 kB/s on the F405 profile (1 kHz) and 660 kB/s on the H743 profile (4 kHz), before the other inputs and the normal level's records (about 40 kB/s in a scripted run with every input). Whether an SD card keeps up is measured at bring-up; the outputs record can be packed tighter if it does not.
- The codes of states, actions and events in a log are now a format. They are fixed in one table with a test, and a new variant gets a new code.

## Deferred

- The SD-card side of the logger, and making the buffer lock-free between interrupt levels (bring-up).
- Replay of a window that starts in flight. It needs the core's state to be saved and restored, which does not exist. DESIGN §4.15's "short triggered windows" on the F405 can be inspected but not replayed until then.
- `vehicle_gnss`, `cpuload`, the estimator's innovation ratios, PX4's event metadata (§4).
- Reading ULog and DataFlash logs of real flights into the simulator for comparison (M2).
- Log rates as parameters.
- Checking a real log in Flight Review and PlotJuggler by hand. The automated check is `pyulog`, the strictest reader of the three.
