# Airborne Oxide

*Rust in the air.*

Airborne Oxide is flight software for drones, written in Rust. It has three parts:

1. **A simulator.** It is deterministic and runs much faster than real time, and it is calibrated against real flights. It is meant to be good enough to design flight algorithms *and* airframes in: generate many airframes, fly them all, 3D-print the best.
2. **A flight core.** A small, hard-real-time, formally checked flight controller for STM32 chips that can fly multirotors, fixed-wings and VTOLs. It stays safe whatever the AI does.
3. **An AI brain.** Computer vision plus a small on-board language model that makes mission decisions. It runs on a companion computer today and on smaller drones as AI hardware shrinks.

> **Status: design phase.** No code yet. The full design is in [`docs/DESIGN.md`](docs/DESIGN.md).

## Planned layout

| Crate | What it is |
|---|---|
| `oxide-core` | Flight core library (`no_std`, no heap) |
| `oxide-fw-h743`, `oxide-fw-f405` | Board firmware |
| `oxide-sim` | The simulator |
| `oxide-lab` | The airframe lab |
| `oxide-brain` | The companion-computer AI |

## Documents

- [`docs/DESIGN.md`](docs/DESIGN.md): architecture, decisions and roadmap
- [`docs/research/prior-art-2026-11.md`](docs/research/prior-art-2026-11.md): survey of existing autopilots, simulators and tools
- [`docs/hardware/viper.md`](docs/hardware/viper.md): setup review of the Titan Viper test VTOL

## License

Copyright © 2026 Daniel Biocchi.

Airborne Oxide is free software, licensed under the **GNU Affero General Public License v3.0 or later**. See [`LICENSE`](LICENSE).
Commercial licenses are available from the copyright holder.

External contributions will be accepted once a contributor license agreement is in place. See [`CONTRIBUTING.md`](CONTRIBUTING.md).
