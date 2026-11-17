// SPDX-License-Identifier: AGPL-3.0-or-later
//! `oxide-bridge`: fly a scenario file through the bridge, or on our own core,
//! once (`fly`) or many times (`batch`). See [`oxide_bridge::cli`].

fn main() -> std::process::ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    oxide_bridge::cli::main_with(&args)
}
