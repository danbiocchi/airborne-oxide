# Prior-art survey, November 2026

**Question:** does existing software already do what Airborne Oxide plans to do, or pieces we could combine?
**Method:** a web survey on 2026-11-04. Star counts and dates are approximate. Entries marked *(uncertain)* could not be confirmed. The facts below come from the survey and were not each re-verified by hand.

## Conclusion

**Nothing does all or most of it.**
- **The mature stack** is PX4 or ArduPilot, with Gazebo, aerial-autonomy-stack and a Jetson. It covers flight, simulation and perception, but:
  - it is C/C++;
  - it is slow for Monte-Carlo testing;
  - it is not bit-exact deterministic;
  - it has no airframe lab.
- **Rust flight-controller projects** exist, but all are young and none has been shown flying a real vehicle.

**The clearest gaps, where Airborne Oxide can be best in class:**
1. **The simulator.** No open simulator combines all of these in one CPU-first package:
   - multirotor, fixed-wing and tilt-rotor vehicles;
   - realistic sensor and fault models;
   - real terrain;
   - lockstep with our core *and* with ArduPilot and PX4;
   - more than 100× real time in CI;
   - Python bindings for reinforcement learning.

   Today's tools are either GPU reinforcement-learning simulators (mostly quad-only, with little or no autopilot connection) or heavy, Gazebo-class simulators that run at roughly real time.
2. **The airframe lab.** No open tool goes the whole way from a mesh to mass and inertia, aerodynamics, auto-tuning and a manoeuvrability score.
3. **Smaller gaps:** a Rust flight controller proven in real flight, and fitting simulator parameters from ArduPilot logs.

## 1. Rust flight controllers and robotics runtimes

| Project | What it is | License | Activity | Notes |
|---|---|---|---|---|
| copper-rs | Rust robot runtime with deterministic record and replay, `no_std`. Has an example quad flight controller (MicoAir H743) and a Bevy/Avian simulator. | Apache-2.0 | ~1.5k★, v1.3.1 (2026-10) | Quad only, no MAVLink, not shown flying *(uncertain)*. We borrow its replay ideas but don't depend on it in the core (Q32). |
| stable-flight | `no_std` H743 flight core (RTIC 2), MAVLink bridge, deterministic simulator in CI. Covers quad, tailsitter and quadplane. | MIT | 1★, very active | Shaped almost exactly like our plan. Simulation only, and the firmware is a stub. Worth reading. |
| holsatus-flight | Async quad autopilot on Embassy for the RP2040, with a MAVLink server and calibration | Apache-2.0 | 98★ | RP2040, quad only, mid-refactor |
| pulseengine/relay | WebAssembly flight-software components: a PX4-style control cascade with allocation, formal proofs (Verus, Kani, Rocq, Lean) | Apache-2.0 | 2★ | Useful for ideas; not running on hardware |
| protoflight, pro-flight, peterkrull/quad, Peng | Hobby flight controllers and a quad simulator | GPL / MIT/Apache | small | Not flown, archived, or low fidelity |
| dora-rs | Rust dataflow middleware for robots, with Python support | Apache-2.0 | 4.1k★, 1.0 (2026-09) | Soft real-time. A brain middleware candidate for S3. |
| wls-alloc | Constrained least-squares control allocation. No heap, runs on Cortex-M. | MIT as published, but its README calls it a port of tudelft/ActiveSetCtlAlloc, which is **GPL-3.0**. Provenance unresolved. | v0.4.0 | **Not adopted.** We write our own allocator, or port PX4's BSD-3 one (DESIGN §4.5, §10). |
| rust-mavlink / MAVSpec | MAVLink 1/2 with signing and async; embedded mode; code generation | MIT OR Apache-2.0 | 0.21.0 (2026-10) | Adopted |

## 2. Established autopilots

| Autopilot | License | Latest | Notes |
|---|---|---|---|
| PX4 | BSD-3-Clause | v1.17.0 (2026-05), v1.18-rc1 | Permissive, so EKF2 and control allocation can be ported with credit. Has a lockstep simulator interface. |
| ArduPilot | GPL-3.0 | 4.7.2 (2026-10) | Widest vehicle support. Its JSON SITL interface lets an external physics engine drive it. Its code must not be copied (Q31). |
| Betaflight / INAV | GPL-3.0 | Betaflight 2026.6.2 | FPV and racing focus |
| Paparazzi / Crazyflie firmware | GPL-2.0 / GPL-3.0 *(uncertain)* | — | Mostly research use |

## 3. Simulators

| Simulator | License | Speed / determinism / RL | Autopilot SITL |
|---|---|---|---|
| Gazebo (Jetty LTS) | Apache-2.0 | Heavy. Lockstep with PX4. Not built for Monte-Carlo runs. | PX4 natively; ArduPilot through a plugin |
| aerial-autonomy-stack | MIT | Gazebo-based, faster than real time, Gymnasium environment | PX4, ArduPilot, Betaflight |
| Project AirSim | MIT | Unreal Engine 5, plus a lighter mode without rendering | PX4, ArduPilot |
| Colosseum / Flightmare / RotorS / jMAVSim | various | Archived or legacy | — |
| Isaac Sim + Pegasus | Apache-2.0 source; NVIDIA terms for Omniverse | Needs a GPU, good rendering | PX4; ArduPilot experimental |
| Aerial Gym | BSD-3 | Isaac Gym on GPU, thousands of vehicles at once | None |
| OmniDrones | MIT | Isaac Sim 4.1; maintenance is difficult | None |
| gym-pybullet-drones | MIT | Gymnasium, multi-agent RL | Betaflight |
| RotorPy | MIT | Aerodynamics, wind, IMU, motor dynamics; batched | PX4 (optional) |
| Crazyflow | MIT | JAX/MuJoCo, up to ~914M steps/s on an RTX 4090 (total throughput across many parallel environments, not one vehicle); fitted to real Crazyflie data (~30 g airframe); differentiable | None |
| RLtools / learning-to-fly | MIT | C++, trains a hover policy in ~18 s | Deploys to PX4, Crazyflie, ESP32 |
| Genesis | Apache-2.0 | General-purpose physics | None |
| JSBSim | LGPL-2.1 | Fixed-wing flight dynamics, batch mode, Python | ArduPilot, PX4 |

## 4. Airframe design and aerodynamics tools

| Tool | License | Automatable? | Notes |
|---|---|---|---|
| AeroSandbox | MIT | Python, built on automatic differentiation | Vortex lattice, NeuralFoil, weights, optimisation. Adopted for A1 (Q35). |
| NeuralFoil | MIT | Python | ~30× faster than XFOIL per run, ~1000× in batches |
| OpenVSP + VSPAERO | NOSA-1.3 | Headless, Python API | v3.55.0. A2 candidate. Runs as a separate process only. |
| AVL | GPL | Command files | Stability derivatives. Separate process only. |
| XFOIL / XFLR5 | GPL | XFOIL scriptable; XFLR5 mostly a GUI | — |
| SU2 / OpenFOAM | LGPL-2.1 / GPL-3.0 | CLI | Full CFD, for A3 only |
| pyBEMT, SizingLab | MIT / ? | Python | Propeller blade-element theory and multirotor sizing |
| sysid (arplaboratory) | MIT | Jupyter | Quadrotor system identification from PX4 ULog (expects PX4 topic names, DESIGN §4.15). Nothing equivalent found for ArduPilot logs. |

## 5. On-drone AI autonomy

| Stack | License | Notes |
|---|---|---|
| ModalAI VOXL SDK | Varies per repo *(uncertain)* | Tied to VOXL hardware |
| Auterion | Commercial *(uncertain)* | Built on PX4 |
| TypeFly | Apache-2.0 | Turns language into skill-based plans (cloud LLM + YOLO). Pattern adopted (Q37). |
| AerialClaw (arXiv 2026-06) | MIT *(uncertain)* | LLM agent framework; PX4/Gazebo, AirSim; LLM backend can be swapped |
| "Taking Flight with Dialogue" | ? | PX4 + ROS 2 with local models through Ollama |
| VLN-on-the-Fly / OnFly | Code not confirmed | Vision-language model on a Jetson Orin NX, ~0.79 s median per decision. *(Unverified: citation, model size and quantisation still to be added. Orin Nano-class latency will be measured.)* |

## 6. Rust building blocks

| Crate | Notes |
|---|---|
| `nalgebra` 0.35 | Mature, supports `no_std` |
| `rerun` 0.42 | Strong visualisation; pre-1.0 with frequent breaking changes, so kept behind a thin layer |
| `rapier3d` 0.36 | Mature, optional bit-exact cross-platform determinism |
| `embassy-executor` 0.10 | Good STM32 F4/H7 HALs |
| RTIC 2.4 | Priority-based hard real-time scheduling with analysable timing |

## Sources

[copper-rs](https://github.com/copper-project/copper-rs) ·
[stable-flight](https://github.com/wboayue/stable-flight) ·
[holsatus-flight](https://github.com/holsatus/holsatus-flight) ·
[pulseengine/relay](https://github.com/pulseengine/relay) ·
[rust-mavlink](https://github.com/mavlink/rust-mavlink) ·
[dora](https://github.com/dora-rs/dora) ·
[PX4 releases](https://github.com/PX4/PX4-Autopilot/releases) ·
[ArduPilot releases](https://github.com/ArduPilot/ardupilot/releases) ·
[ArduPilot JSON SITL](https://ardupilot.org/dev/docs/sitl-with-JSON.html) ·
[aerial-autonomy-stack](https://github.com/JacopoPan/aerial-autonomy-stack) ·
[Project AirSim](https://github.com/iamaisim/ProjectAirSim) ·
[Pegasus](https://github.com/PegasusSimulator/PegasusSimulator) ·
[Aerial Gym](https://github.com/ntnu-arl/aerial_gym_simulator) ·
[OmniDrones](https://github.com/btx0424/OmniDrones) ·
[Crazyflow](https://github.com/utiasDSL/crazyflow) ·
[RotorPy](https://github.com/spencerfolk/rotorpy) ·
[gym-pybullet-drones](https://github.com/utiasDSL/gym-pybullet-drones) ·
[RLtools](https://github.com/rl-tools/rl-tools) ·
[Genesis](https://github.com/Genesis-Embodied-AI/Genesis) ·
[JSBSim](https://github.com/JSBSim-Team/jsbsim) ·
[AeroSandbox](https://github.com/peterdsharpe/AeroSandbox) ·
[NeuralFoil](https://github.com/peterdsharpe/NeuralFoil) ·
[OpenVSP](https://github.com/OpenVSP/OpenVSP) ·
[AVL](https://web.mit.edu/drela/Public/web/avl/) ·
[XFOIL](https://web.mit.edu/drela/Public/web/xfoil/) ·
[SU2](https://github.com/su2code/SU2) ·
[sysid](https://github.com/arplaboratory/data-driven-system-identification) ·
[TypeFly](https://github.com/typefly/TypeFly) ·
[AerialClaw](https://arxiv.org/abs/2606.12142) ·
[Taking Flight with Dialogue](https://arxiv.org/pdf/2506.07509) ·
[VOXL SDK](https://docs.modalai.com/voxl-sdk/)
