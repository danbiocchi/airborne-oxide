# Airborne Oxide

*Rust in the air.*

Airborne Oxide is flight software for drones, written in Rust. It has three parts:

1. **A simulator.** It is deterministic, runs much faster than real time, and is calibrated against real flights. The goal is to design flight algorithms *and* airframes in it: generate many airframes, fly them all, and 3D-print the best.
2. **A flight core.** A small, hard-real-time, formally checked flight controller for STM32 chips that flies multirotors, fixed-wings and VTOLs. It stays safe whatever the AI does.
3. **An AI brain.** Computer vision plus a small on-board language model that makes mission decisions. It runs on a companion computer today and will move onto smaller drones as AI hardware shrinks.

> **Status: milestone M1d is done: benchmark gate 1 is passed.** In our simulator the flight core was compared with ArduCopter 4.7.2 by criteria fixed before the run, over 500 gusty position holds and 380 failure flights, and passed on its second run ([ADR-0007](docs/adr/0007-gate-1-criteria.md), [the report](docs/gates/gate-1/run-2/README.md)). The next step is the first run on a real F405 board. The flight core flies a simulated 5" quad on its own state estimate and handles failures: lost radio, lost GPS, a dead barometer, magnetometer or IMU, a crashed brain, a low battery, a geofence breach, a lost motor. The 98 tuning parameters are named, range-checked and changeable in flight. A parameter store that survives a power cut at any moment is written and tested against simulated flash; it meets real flash at F405 bring-up. The scenarios are data files, flown across random seeds on both chip profiles. A flight can be logged in PX4's ULog format, and a logged flight can be replayed through the flight core bit for bit. The SD-card side of the logger waits for F405 bring-up. See the [roadmap](docs/DESIGN.md#13-roadmap).

## Try it

You need [Rust](https://rustup.rs); the pinned toolchain installs itself.

```bash
cargo run --release -p oxide-sim-cli -- run
```

This flies the hover-and-step scenario: arm on the ground, climb to 2 m, step 1 m north, turn 90°, then climb and step east. It prints the result, the speed relative to real time and the trace hash. Other options:

```bash
cargo run --release -p oxide-sim-cli -- run --profile h743 --csv flight.csv
```

`--profile` picks the chip's task schedule (`f405` or `h743`). `--csv` writes every tick to a file you can plot. To watch it in 3D, or fly it with your radio as a joystick, see [`crates/oxide-viewer`](crates/oxide-viewer/README.md).

The M1b scenario holds a point at 2 m in gusty wind on noisy sensors, using the estimator. `--seed` picks the random run:

```bash
cargo run --release -p oxide-sim-cli -- run --scenario gust-hold --seed 7
```

Fly a failure and print what the flight core did about it, event by event (`scenario list` shows them all):

```bash
cargo run --release -p oxide-sim-cli -- scenario rc-loss-position
```

Fly 200 gust-hold seeds, using every CPU core, and print the worst case of each pass criterion (`--profile h743` picks the other chip profile):

```bash
cargo run --release -p oxide-sim-cli -- montecarlo --seeds 200
```

List the tunable parameters, and fly with one changed (all of them are documented in [`docs/PARAMETERS.md`](docs/PARAMETERS.md); `--params FILE.toml` takes a `[params]` table):

```bash
cargo run --release -p oxide-sim-cli -- params list
cargo run --release -p oxide-sim-cli -- run --param POS_X_P=1.2
```

Write a log in PX4's ULog format, then a replay log, and run that flight through the flight core again. The log uses PX4's topic names and units so that PlotJuggler and Flight Review can show it; `pyulog` reads it in CI, Flight Review's plotting code has drawn its plots from simulator logs, and PlotJuggler has not been tried yet. `replay` compares every output of every tick bit for bit and names the first one that differs ([ADR-0005](docs/adr/0005-logging-and-replay.md)):

```bash
cargo run --release -p oxide-sim-cli -- scenario rc-loss-position --log flight.ulg
```

```bash
cargo run --release -p oxide-sim-cli -- scenario rc-loss-position --log replay.ulg --log-level replay
```

```bash
cargo run --release -p oxide-sim-cli -- replay replay.ulg
```

Add `--param ATT_ROLL_P=5` to the last command to replay the same inputs through a changed setting and see where the flight first departs.

`cargo run --release -p oxide-sim-cli -- help` lists every command and its options.

Run the tests:

```bash
cargo test --workspace
```

The full acceptance run is separate, because it needs a release build. It flies every scenario file in `scenarios/` on the seeds and chip profiles its `[run]` table names (the fault scenarios and pairings over 10 seeds, gust-hold over 200), in parallel, and exits non-zero if any run fails:

```bash
cargo run --release -p oxide-sim-cli -- scenarios scenarios
```

`--seeds N` and `--profile f405|h743|both` override the files; `--threads T` limits the cores. The same runs are available as ignored tests (`cargo test --release -p oxide-sim --test fault_scenarios -- --ignored --nocapture`, and `--test gust_hold`).

## Write and fly a scenario file

A scenario is one TOML file: the pilot, the faults, the wind, the parameter overrides, and what must happen. The 43 in [`scenarios/`](scenarios/) define those scenarios; they are compiled into the simulator, and any other file is flown by its path. (`hover-step` and `gust-hold` also keep a pilot written in Rust, which the `run`, `hash` and `montecarlo` commands and the viewer use; a test checks that both forms fly the same flight.) Unknown keys are errors, and heights are up-positive. The fully commented schema is in the documentation of [`scenario_file.rs`](crates/oxide-sim/src/scenario_file.rs).

```toml
name = "gps-loss-short"
about = "GPS lost for 4 s in Offboard: the vehicle holds, then flies on."
duration_s = 40.0

[sim]
sensors = "realistic"
estimator = "ekf"

[wind]
kind = "random"
steady_speed_mps = [0.0, 2.0]
sigma_h_mps = [0.1, 0.4]
tau_s = 2.0

[params]
FS_GPS_LAND = 10.0          # a parameter by name (docs/PARAMETERS.md)

[[fault]]
kind = "gps-loss"
start_s = 15.0
end_s = 19.0

[pilot]
arm_at_s = 3.0
[[pilot.mode]]
t = 0.0
switch = "offboard"
[[pilot.setpoint]]
t = 0.0
position = [0.0, 0.0, 0.0]  # [north, east, up] m
[[pilot.setpoint]]
t = 4.0
position = [0.0, 0.0, 5.0]

[expect]
nav_contains = ["hold", "offboard"]        # in this order
nav_never = ["land", "land-now", "terminated"]
reasons = ["GPS_LOST"]
min_height_m = 4.0
by_s = 40.0
[expect.ending]
kind = "flying"
point = [0.0, 0.0, 5.0]
tolerance_m = 2.0

[run]
seeds = 10
```

Fly it (`--seed` is the run index; the exit code is 1 if an expectation is broken), or fly a whole directory:

```bash
cargo run --release -p oxide-sim-cli -- scenario gps-loss-short.toml --seed 3
cargo run --release -p oxide-sim-cli -- scenarios my-scenarios --seeds 20
```

Putting a file in `scenarios/` adds it to the Monte-Carlo run. To embed it in the simulator (so it can be flown by name, in the viewer too), add it to the list in `crates/oxide-sim/src/fault_scenarios.rs`; a test fails until the list and the directory agree. The run seed is derived from `name`, so renaming a scenario changes its flights.

## Layout

| Path | What it is |
|---|---|
| `crates/oxide-core` | Flight core library (`no_std`, no heap). Builds for `thumbv7em-none-eabihf`. |
| `crates/oxide-store` | Parameter storage on flash (`no_std`): a two-sector log that survives power loss at any point |
| `crates/oxide-log` | Flight logs in PX4's ULog format (`no_std`): the writer, the topics, the logger's ring buffer, the replay records, and a reader |
| `crates/oxide-sim` | Simulator: physics, wind, battery, sensor models, fault injection, scenario-file loader, Monte-Carlo runner, lockstep loop |
| `crates/oxide-bridge` | The bridge that flies another autopilot's simulator build on our physics (ADR-0006); the exchange, the ArduCopter process, MAVLink, an autopilot that flies offboard or the pilot's sticks (a delayed script for the program's arming delay), our faults as its parameter writes, the tuning script (`tune`: two flights from POSHOLD, each with hover, spectrum, the automatic tuner and read-back, ADR-0009; its result is committed as `tuned-gains.parm`, which every bridged flight flies), and the binary `oxide-bridge` that flies a scenario file (`fly`) or many (`batch`) on ArduCopter or on our core and writes one table for both, and flies benchmark gate 1 and computes its verdict from the tables (`gate fly`, `gate verdict`; ADR-0007) |
| `scenarios/` | The scenario files: the 20 failures, the 21 pairings, `hover-step` and `gust-hold` |
| `crates/oxide-sim-cli` | `oxide-sim` command-line runner |
| `tools/ulog_check.py` | Checks a log with `pyulog`, PX4's reference reader (used by CI) |
| `crates/oxide-viewer` | Optional Rerun viewer with joystick input (kept out of the default build) |
| `airframes/` | Airframe data files |
| `docs/DESIGN.md` | Architecture, decisions and roadmap |
| `docs/adr/` | Decision records made after the design sign-off |
| `docs/research/` | Prior-art survey and the design review |
| `docs/hardware/viper.md` | Setup review of the Titan Viper test VTOL |

## License

Copyright © 2026 Daniel Biocchi.

Airborne Oxide is free software, licensed under the **GNU Affero General Public License v3.0 or later**. See [`LICENSE`](LICENSE).
Commercial licenses are available from the copyright holder.

External contributions will be accepted once a contributor license agreement is in place. See [`CONTRIBUTING.md`](CONTRIBUTING.md).
