# Design review, 2026-11-04

An independent review of DESIGN.md v0.1 and viper.md v1. Seven reviewers, one per angle, raised 63 findings. Two judges ruled on each one independently: a strict skeptic and an owner's advocate. The resulting changes are summarised in DESIGN.md Appendix B.

**Outcome key:**
- **Applied:** both judges upheld the finding.
- **Applied (merged):** folded into a related finding's fix.
- **Applied (one judge):** upheld by one judge and applied because it was cheap and low-risk.
- **Rejected:** both judges rejected it.

Severities are as the reviewer graded them; the judges' grades follow, as skeptic / advocate.

## Safety and command authority

| # | Finding | Reviewer | Judges (skeptic / advocate) | Outcome |
|---|---|---|---|---|
| safety-1 | No flight-core fault policy (hard fault, watchdog, brownout, stack overflow, panic) | critical | major / major | Applied |
| safety-2 | 'Pilot overrides everything, including a failsafe in progress' is unsafe as written | critical | major / major | Applied |
| safety-3 | Kill-switch semantics undefined for in-flight use; no arming/disarm spec beyond parameters | critical | reject / major | Applied (merged) |
| safety-4 | RTL is the default action in several rows with no requirement for a valid position, home point, or safe altitude | critical | reject / major | Applied (merged) |
| safety-5 | Failsafe table has no arbitration: simultaneous or escalating events are undefined | major | major / major | Applied |
| safety-6 | Brain-loss timings contradict the 'autonomous patrol survives brain reboot' requirement; lost-link patrol policy is unspecified | major | minor / major | Applied |
| safety-7 | Command gateway and authority model underspecified: GCS powers, re-grant of revoked authority, tier-1 vs core limits, mode/command arbitration | major | major / minor | Applied |
| safety-8 | MAVLink signing: no key provisioning, storage, rotation, or replay/failure policy | major | reject / minor | Applied (merged) |
| safety-9 | Failsafe table is multirotor-only, but the stated end aircraft is a tilt-rotor VTOL; geofence and low-battery behaviour in transition/forward flight undefined | minor | reject / reject | Rejected |

## Embedded and real-time feasibility

| # | Finding | Reviewer | Judges (skeptic / advocate) | Outcome |
|---|---|---|---|---|
| embedded-1 | EKF 'predicts at IMU rate' is unaffordable and is not what EKF2 does | major | major / major | Applied |
| embedded-2 | Kani claims overreach for floating-point and generated-estimator code | major | minor / minor | Applied |
| embedded-3 | 'Never panics' has no enforcement mechanism and no defined behaviour if wrong | major | reject / major | Applied (merged) |
| embedded-4 | Bit-for-bit determinism across host and MCU is over-claimed and not actually guaranteed by the libm crate alone | major | reject / major | Applied (merged) |
| embedded-5 | 'CI-enforced timing budgets' cannot be measured on GitHub-hosted runners | major | reject / minor | Applied (merged) |
| embedded-6 | Parameter flash writes can stall flight and wear the sector; no storage design | major | minor / major | Applied |
| embedded-7 | High-rate ULog/replay logging to SD is not feasible on the F405 as specified | major | reject / minor | Applied (one judge) |
| embedded-8 | RTIC 2 + embassy-stm32 integration and DMA/cache memory placement are unaddressed | major | reject / major | Applied (one judge) |
| embedded-9 | Bidirectional DShot is treated as a given but is a custom, resource-constrained driver | minor | reject / minor | Applied (one judge) |
| embedded-10 | Estimator generic over f32/f64 via codegen: F405 has no f64 hardware; numerics and flash size are unspecified | minor | reject / reject | Rejected |

## Simulator architecture

| # | Finding | Reviewer | Judges (skeptic / advocate) | Outcome |
|---|---|---|---|---|
| simulator-1 | Bit-exact determinism claim is under-specified; libm in the core alone does not deliver it | major | major / major | Applied |
| simulator-2 | Physics rate, core rate loop (1-8 kHz) and sensor sampling are not reconciled; the 100x target is unbudgeted | major | reject / major | Applied (merged) |
| simulator-3 | oxide-core/oxide-sim interface is undefined, so 'exact same code' lockstep is not guaranteed; compute latency and jitter are unmodelled | critical | major / critical | Applied |
| simulator-4 | ArduPilot JSON and PX4 SITL interfaces do not fit 'identical scenarios, oxide-sim owns the clock'; sensor models differ per autopilot | major | reject / major | Applied (merged) |
| simulator-5 | S1 sensor model list is incomplete for the S1 milestone, failsafes and gate 1 | major | reject / minor | Applied (one judge) |
| simulator-6 | Calibration plan and Viper airframe cannot be served by S1/S2 physics; S1 calibration data is inadequate | critical | major / critical | Applied |
| simulator-7 | RotorPy/Crazyflow/JSBSim cross-checks have no defined comparison, tolerance or applicability | major | minor / minor | Applied |
| simulator-8 | M1/S1 is too large for a solo developer and mixes sim, full core and hardware bring-up | major | reject / major | Applied (merged) |
| simulator-9 | Airframe-lab A1 (AeroSandbox VLM) cannot supply the aero tables for stall, transition or rotor-airframe interaction; table interface undefined | major | reject / minor | Applied (merged) |

## Licensing

| # | Finding | Reviewer | Judges (skeptic / advocate) | Outcome |
|---|---|---|---|---|
| licensing-1 | cargo-deny allowlist as written would reject normal dependency trees (MPL-2.0, Unicode-3.0, BSL-1.0, CC0, 0BSD, etc.), and 'or similar' is not enforceable | major | reject / minor | Applied (one judge) |
| licensing-2 | README says 'Commercial licenses are available' but nothing exists to license yet | major | reject / reject | Rejected |
| licensing-3 | The vendored 'MIT' wls-alloc is a port of TU Delft control-allocation C code whose upstream license is likely GPL; vendoring also bypasses cargo-deny | major | major / critical | Applied |
| licensing-4 | CLA requirements are underspecified and a CLA alone does not solve third-party code, moral rights, or employer claims | major | reject / minor | Applied (one judge) |
| licensing-5 | Attribution duties for ported PX4 code are mentioned only as 'with credit'; BSD-3 and mixed provenance need concrete rules | major | reject / minor | Applied (merged) |
| licensing-6 | 'Separate process' is a sensible default but is not a complete test for the GPL/LGPL/NOSA tools, and the rule ignores redistribution of those tools | minor | reject / minor | Applied (one judge) |
| licensing-7 | README/DESIGN misstate AGPL obligations, and the PyO3 'RL environment' use case is not addressed | minor | reject / minor | Applied (one judge) |
| licensing-8 | Viper airframe, manual-derived validation data and Titan IP policy conflict with a public repo and public CI | minor | reject / reject | Rejected |
| licensing-9 | Old Apache-2.0 history and 'fresh history' are not reconciled; per-file license marking is missing | minor | reject / reject | Rejected |

## Fact-check

| # | Finding | Reviewer | Judges (skeptic / advocate) | Outcome |
|---|---|---|---|---|
| factcheck-1 | Cross-OS bit-exact determinism is promised, but the plan only specifies libm for the flight core and mentions rapier's mode as a bare switch | major | reject / major | Applied (merged) |
| factcheck-2 | The PX4 and ArduPilot lockstep claims are not supported by the cited docs | major | reject / major | Applied (one judge) |
| factcheck-3 | The Orin NX latency figure is used to justify an Orin Nano-class target and a 4B model, and the source is not cited | major | reject / minor | Applied (one judge) |
| factcheck-4 | The Crazyflow cross-check and headline number do not fit this CI or this vehicle class | minor | reject / minor | Applied (merged) |
| factcheck-5 | Kani proofs on floating-point allocation and control code are asserted without a feasibility check | major | reject / minor | Applied (merged) |
| factcheck-6 | The H743 multi-IMU design assumes a board whose specs are unverified | minor | reject / minor | Applied (one judge) |
| factcheck-7 | The ESC plan depends on AM32 and bidirectional DShot, which is confirmed in general but version-sensitive and unconfirmed for the chosen ESC | minor | reject / minor | Applied (one judge) |

## Viper setup review

| # | Finding | Reviewer | Judges (skeptic / advocate) | Outcome |
|---|---|---|---|---|
| viper-1 | Failsafe and RTL behaviour is unconfigured and the doc never mentions it | critical | critical / critical | Applied |
| viper-2 | Mode-switch finding misses the 3-position-switch mapping and the fixed-wing LOITER slot | critical | major / critical | Applied |
| viper-3 | Radio calibration finding underplays the hazard, and the 'extended endpoints' explanation cannot produce RC3_MAX=2200 | major | reject / minor | Applied (one judge) |
| viper-4 | Battery fix: pins are right but the sequencing, wrong-board risk and tuning side effects are omitted | major | reject / minor | Applied (one judge) |
| viper-5 | Airspeed diagnosis is wrong: ARSPD_TYPE=1 is the I2C MS4525 type, so ARSPD_PIN is not the problem | major | minor / major | Applied |
| viper-6 | Recommended speed limits leave almost no stall margin at the configured 45-degree bank, and the wing area may be overstated | major | reject / major | Applied (merged) |
| viper-7 | Tuning guidance covers only yaw gains; all rate gains are stock and the notch is ordered after autotune | major | minor / major | Applied |
| viper-8 | Transition and forward-flight parameters are unreviewed although the doc says they matter | minor | reject / major | Applied (merged) |
| viper-9 | No compass-versus-throttle check for a high-current tri with an external GPS/compass mast | minor | reject / major | Applied (merged) |

## Consistency and scope

| # | Finding | Reviewer | Judges (skeptic / advocate) | Outcome |
|---|---|---|---|---|
| consistency-1 | M1 is several milestones in one, with no visible demo until the very end | major | major / major | Applied |
| consistency-2 | Typed sensor/actuator interface and crate boundaries between oxide-core, oxide-sim and firmware are unspecified | major | reject / critical | Applied (merged) |
| consistency-3 | Bit-identical determinism across Windows/Linux and MCU/host is not achievable with the stated stack | major | reject / major | Applied (merged) |
| consistency-4 | 'Identical scenarios, one scorecard' across oxide-core, ArduPilot SITL and PX4 SITL does not hold for sensors and faults; gate 1 has no thresholds | major | major / major | Applied |
| consistency-5 | CI claims that cannot run on GitHub runners: F405 timing budgets, Kani on float-heavy code, nightly job limits | major | reject / minor | Applied (merged) |
| consistency-6 | Brain-core protocol is missing time sync, version negotiation and an owner for the custom dialect | major | reject / minor | Applied (one judge) |
| consistency-7 | Development telemetry and log storage/data management are unspecified; replay logging may exceed storage bandwidth; ULog contradiction | major | reject / minor | Applied (one judge) |
| consistency-8 | Sim physics scope does not cover the Viper, yet M2 'done when' and calibration depend on it; real-flight data arrives too late | major | reject / critical | Applied (merged) |
| consistency-9 | No failsafe arbitration for simultaneous failures; position-dependent actions have no fallback | minor | reject / major | Applied (merged) |
| consistency-10 | Body vs Appendix A inconsistencies on heap use, crate contents and rapier precision | minor | minor / critical | Applied |

## Issues the judges added

| Issue | Raised by | Outcome |
|---|---|---|
| F405 'IMU failure → land' is not flyable | skeptic | Applied |
| Time width unspecified; u32 microseconds wraps at 71.6 min | skeptic | Applied |
| ULog 'works with PX4 Flight Review' only if PX4 topic names are used | skeptic | Applied |
| Open-loop replay of logged commands will diverge; the sim-vs-real method is undefined | advocate | Applied |
| Benchmark gates have no tuning-parity rule | advocate | Applied |
| 'IMU failure on the F405: land' cannot be done with one IMU | advocate | Applied |

## Reviewer verdicts on v0.1

- **Safety and command authority:** The architecture (an untrusted brain behind a gateway, RC wired directly to the core) is sound, but the safety layer in §4.8-4.11 is a sketch with several unsafe defaults. Pilot override is unconditional, kill-switch semantics are undefined, RTL is the default with no position/home preconditions, and failsafes have no arbitration. The doc has no section at all on MCU-level faults (watchdog, reset mid-air, brownout, stack overflow), even though the "never panics" principle leans on it. Autonomous patrol (brain-loss timings, lost-link policy) and MAVLink signing key management are named but not specified, so the timings contradict the "mission survives brain reboot" goal. Findings 1-4 should be resolved in the doc before M1 failsafe code is written.
- **Embedded and real-time feasibility:** The architecture is sound and the settled decisions are respected. The embedded sections, however, read as aspirations without numbers: there are no cycle, RAM, flash or log-bandwidth budgets, and several headline claims (EKF at IMU rate, Kani proofs on float code, bit-exact host/MCU maths, CI timing budgets, never-panic) have no workable enforcement or are technically over-stated. The F405 profile is the weakest point: DMA-unreachable CCM, soft-double, flash-erase stalls, SD write stalls and a 1 MB flash are all unaddressed. Each can be fixed with a short budget table or a scoped claim in the doc, and an RTIC+Embassy+SPI-DMA spike on the bench F405 early in M1 would retire the largest feasibility risks.
- **Simulator architecture:** The plan is well structured, but several load-bearing simulator claims lack an interface or a budget. The oxide-core/oxide-sim contract (scheduler ownership, timestamped events, compute latency) and a shared time base for physics and the 4-8 kHz core must be fixed before coding, or lockstep, the 100x target and benchmark gate 1 are not trustworthy. The determinism claim needs an explicit contract (libm everywhere, pinned RNG, ordered containers, a two-OS hash test). The SITL bridges and cross-checks need concrete semantics and tolerances. The largest gap is calibration: physics through S2 cannot represent the tilt-rotor Viper, yet it is the only real data source and the M2 exit criterion. S1/M1 should be sliced into falsifiable sub-milestones, and vibration plus battery, RC and RPM sensor models pulled forward.
- **Licensing:** The headline licensing structure (AGPL-3.0-or-later with a CLA and dual licensing) is internally coherent and the 'no external contributions until a CLA exists' gate is the right safeguard. The weak points are in the details beneath it. The dependency allowlist is not yet enforceable. The README advertises a commercial license that does not exist. The provenance of vendored and ported code (wls-alloc, PX4 files) is unverified and not tracked. The CLA scope and the handling of Titan-derived data need to be written down before any code or docs go public. I made no web lookups, so the Paparazzi/wls-alloc and MAVLink points rest on my recall and should be checked against the upstream headers.
- **Fact-check:** The headline external claims I could check were mostly accurate: BLHeli_32 ending and AM32 bidirectional DShot, stable-flight's description, and the Crazyflow figures, though with context missing. The biggest factual risks are in the determinism and lockstep promises. Those are the core of the flagship goal, and the cited docs either add conditions (rapier, platform libm) or do not confirm the claim (PX4 MAVLink and ArduPilot JSON lockstep). The AI-brain latency justification (Orin NX vs Orin Nano) and the Kani-proof scope are weakly supported. The plan should turn these into short pre-S1 spikes and tighten the wording. I ran 8 lookups, and 2 of them (Matek board specs, ArduPilot board page) returned 404. I did not verify OpenTX 2.3.15 joystick mode, the X9D+ module bay, copper-rs, the ETH Zurich result, signing, or ULog tool support.
- **Viper setup review:** The viper.md review is mostly sound on what it states. Battery pins and scales, the S5-S8 shared timer (TIM2) with the S10 relocation, vectored-yaw Q_TILT_TYPE=2, and the stall arithmetic all check out against the hwdef, ArduPilot docs and a recomputation. The main weaknesses are what it omits and a few diagnoses that are wrong or incomplete. It leaves out the failsafe and RTL configuration (FS_LONG_ACTN, Q_RTL_MODE, battery actions), the 3-position-switch mapping that puts MANUAL on the centre position, and the real hover impact of the stored roll trim. Its ARSPD_PIN reasoning is wrong because ARSPD_TYPE=1 is an I2C sensor type, and it gives no firmware-target or tuning-order guidance. Those gaps matter most because the owner will act on this document, and I would not call the aircraft ready to fly after findings 1-3 alone.
- **Consistency and scope:** The goals and settled decisions are coherent, but the plan is not yet ready to code from. The core/sim/firmware interface, workspace layout, brain-core protocol (time, versioning, dialect owner) and the log pipeline are unspecified. Several CI and verification claims do not hold as written: MCU timing budgets on hosted runners, Kani on a float allocator, cross-platform bit-identical determinism, and equal sensor and fault fidelity across ArduPilot SITL. M1 is far too large for a solo developer and should be split into demo-ending steps. The Viper-centred M2 goal is out of step with the sim physics scope and with when real-flight data arrives.
