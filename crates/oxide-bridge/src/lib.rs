// SPDX-License-Identifier: AGPL-3.0-or-later
//! The bridge that flies another autopilot's simulator build on our physics
//! (ADR-0006): the JSON exchange, the simulator program as a checked child
//! process, the MAVLink client for it, and an autopilot that flies offboard.
//!
//! - [`packet`]: the motor packet the simulator sends (binary, little-endian).
//! - [`reply`]: the line of text we answer with.
//! - [`motors`]: pulse widths to normalised motor commands.
//! - [`link`]: the UDP socket and the lockstep rule.
//! - [`sha256`]: for checking the simulator binary.
//! - [`sitl`]: starts and stops the simulator program (ArduCopter SITL) in its
//!   own new directory, and never leaves it running.
//! - [`mav`]: the MAVLink client: typed events out, small sends in, never
//!   blocking on the program (which only moves while we exchange).
//! - [`bridge`]: the one implementation of the exchange and the reply
//!   ([`bridge::Exchange`]), and [`bridge::JsonBridge`], an
//!   `oxide_sim::Autopilot<4>` that flies the radio's channels.
//! - [`faults`]: our faults as ArduCopter's parameter writes
//!   ([`faults::FaultTranslator`]) and our battery thresholds as its own
//!   ([`faults::battery_failsafe_params`]).
//! - [`parm`]: the committed ArduCopter parameter files (`arducopter/*.parm`)
//!   and their parser.
//! - [`arducopter`]: [`arducopter::ArduCopter`], an `oxide_sim::Autopilot<4>`
//!   that starts the program, runs the 45 s pre-roll, and flies our
//!   quad offboard: our setpoint stream becomes GUIDED commands on a MAVLink
//!   link of its own. [`arducopter::fly`] runs a whole bridged flight and
//!   returns the physics' trace hash and the [`arducopter::FlightAudit`].
//! - [`scenario`]: what of a scenario file ArduCopter can be given (the table
//!   of `[params]`, the refusals) and the file loaded with its hash.
//! - [`runner`]: one flight from a scenario file, through the bridge
//!   ([`runner::fly_scenario`]) or on our own core ([`runner::fly_core`]),
//!   judged and classed the same way.
//! - [`record`]: the flight's record as files (`audit.txt`, `row.csv`) and the
//!   table's row format.
//! - [`batch`]: many flights, each in a child process of its own.
//! - [`tune`]: ArduCopter's tuning procedure as a script (ADR-0007 section 2):
//!   hover and learn, the gyro spectrum and its notch rule, the automatic
//!   tuner, the read-back of the gains, and the check.
//! - [`gate`]: gate 1 (ADR-0007): the plan as data ([`gate::plan`]), `oxide-bridge
//!   gate fly` ([`gate::fly`]) and `oxide-bridge gate verdict`
//!   ([`gate::verdict`], [`gate::report`]).
//! - [`cli`]: the command line of the binary `oxide-bridge` (`fly`, `batch`,
//!   `tune`, `gate`).
//!
//! # The binary
//!
//! `oxide-bridge fly --scenario FILE` flies one run of a scenario file and
//! `oxide-bridge batch --scenario FILE --indices A..B --out DIR` flies many,
//! each in a child process of its own. `--autopilot arducopter` goes through
//! the bridge: the committed parameter files (`base.parm`,
//! `starting-gains.parm`, `failsafes.parm`), our battery thresholds as the
//! program's, the scenario's fence, and our faults as parameter writes
//! ([`faults::FaultTranslator`]). `--autopilot core-f405` and `core-h743`
//! fly our own core on the same file. Both write the same row
//! ([`record::COLUMNS`]) and the same record (`audit.txt`, `row.csv`), judged
//! by the same checker and classed by the same code. `oxide-bridge tune --out
//! DIR` flies the tuning procedure ([`tune`]) and writes `report.txt`,
//! `spectrum.csv` and, when the check passes, `tuned-gains.parm`; the set-up
//! of ArduPilot's tuning guide is `arducopter/setup.parm`. A scenario that
//! needs an airframe other than the reference quad or a parameter without a
//! counterpart is refused with a sentence ([`scenario::translate`]). A pilot
//! who flies the sticks is translated: the radio goes to the program as it
//! is, its set-up in `base.parm` (range 1000 to 2000 us, the pitch channel
//! reversed), the pilot's frames and mode switches after the arm switch are
//! delayed by [`scenario::ARM_DELAY_S`] for the program's arming delay, and
//! the brain acts only while the pilot's switch says offboard.
//!
//! # Gate 1
//!
//! `oxide-bridge gate fly --scenarios DIR --out OUT --binary PATH` flies the
//! plan of ADR-0007 (criterion A on 500 gust seeds with ArduCopter and our
//! core on three estimator and profile choices, criterion B on the failure
//! scenarios, ten flights twice) with the batch machinery, from a commit with no
//! uncommitted changes, and writes `OUT/gate.txt`. `oxide-bridge gate verdict
//! --in OUT [--report FILE]` reads only the tables and `gate.txt` and computes
//! the verdict: exit 0 passed, 4 not passed, 2 the tables are incomplete or
//! inconsistent (it then says what is missing and gives no verdict).
//!
//! # A bridged flight repeats bit for bit
//!
//! Nothing read from MAVLink influences anything sent to the program or to
//! the physics: every command goes out at a simulated time fixed in advance
//! or decided from lockstep data (the truth and the motor pulses). What is
//! read goes into the audit and nowhere else. The ground-station failsafe
//! sees no ground station unless the scenario has one: the measuring
//! instrument sends its heartbeat as system 250, not 255.
//!
//! The tests that fly the real program are `#[ignore]`d: set
//! `OXIDE_ARDUCOPTER` to the program's path and run
//! `cargo test -p oxide-bridge --release -- --ignored` (a flight is 85
//! simulated seconds). The program is a separate GPL process and is never
//! copied into this repository.

pub mod arducopter;
pub mod batch;
pub mod bridge;
pub mod cli;
pub mod faults;
pub mod gate;
pub mod link;
pub mod mav;
pub mod motors;
pub mod packet;
pub mod parm;
pub mod record;
pub mod reply;
pub mod runner;
pub mod scenario;
pub mod sha256;
pub mod sitl;
pub mod tune;
