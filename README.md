# Airborne Oxide

*Rust in the air.*

Airborne Oxide is flight software for drones, written in Rust. It has three parts:

1. **A simulator.** It is deterministic, runs much faster than real time, and is calibrated against real flights. The goal is to design flight algorithms *and* airframes in it: generate many airframes, fly them all, and 3D-print the best.
2. **A flight core.** A small, hard-real-time, formally checked flight controller for STM32 chips that flies multirotors, fixed-wings and VTOLs. It stays safe whatever the AI does.
3. **An AI brain.** Computer vision plus a small on-board language model that makes mission decisions. It runs on a companion computer today and will move onto smaller drones as AI hardware shrinks.

> **Status: milestone M1b.** The flight core flies a simulated 5" quad on its own state estimate: an error-state EKF fed by simulated IMU, GPS, barometer and magnetometer with noise, bias, vibration and GPS latency. It holds position in gusty wind across 200 random seeds on both chip profiles. Fault injection, failsafes and scenario files come next (M1c). See the [roadmap](docs/DESIGN.md#13-roadmap).

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

Fly 200 seeds on every core and print the worst case of each pass criterion:

```bash
cargo run --release -p oxide-sim-cli -- montecarlo --seeds 200
```

Run the tests:

```bash
cargo test --workspace
```

The full 200-seed acceptance test is separate, because it needs a release build:

```bash
cargo test --release -p oxide-sim --test gust_hold -- --ignored --nocapture
```

## Layout

| Path | What it is |
|---|---|
| `crates/oxide-core` | Flight core library (`no_std`, no heap). Builds for `thumbv7em-none-eabihf`. |
| `crates/oxide-sim` | Simulator: physics, wind, battery, sensor models, scenarios, Monte-Carlo runner, lockstep loop |
| `crates/oxide-sim-cli` | `oxide-sim` command-line runner |
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
