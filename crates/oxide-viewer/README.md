# oxide-viewer

Flies the simulator in real time and shows it in [Rerun](https://rerun.io): the quad, rotor thrust arrows, its trail, the setpoint and the wind in 3D, plus plots of altitude, attitude, battery voltage, motor commands, wind speed and (when the flight controller publishes an estimate) the estimation error. The blue box is where the state estimator thinks the quad is; the orange one is the true pose.

It is kept out of the main workspace because Rerun is large. The flight crates and CI stay small, and the simulator never depends on it.

## Setup (once)

1. Install the Rerun viewer app **at the same version as the SDK, 0.40.0**:

   ```bash
   pip install rerun-sdk==0.40.0
   ```

   or

   ```bash
   cargo install rerun-cli --version 0.40.0 --locked
   ```

   Rerun's viewer collects anonymous usage statistics by default. To turn that off:

   ```bash
   rerun analytics disable
   ```

2. On Linux, the joystick support needs libudev (`sudo apt install libudev-dev`). On Windows nothing extra is needed.

## Run

From the repository root:

```bash
cargo run --release --manifest-path crates/oxide-viewer/Cargo.toml
```

This flies the scripted hover-and-step scenario in real time and opens the viewer.

| Option | Effect |
|---|---|
| `--joystick` | You fly it with a USB joystick instead of the script |
| `--scenario NAME` | Fly a scripted scenario instead of the default `hover-step`: `gust-hold` (realistic sensors, wind, EKF), or any fault scenario or pairing, such as `rc-loss-position` or `pair-rc-loss+gps-loss`. An unknown name lists the available ones |
| `--seed 3` | Run index for `gust-hold` and the fault scenarios (default 0) |
| `--profile h743` | Use the H743 task schedule instead of the F405 one |
| `--speed 4` | Run at 4× real time (`0` = as fast as possible) |
| `--save flight.rrd` | Write a recording file instead of opening the viewer (open it later with `rerun flight.rrd`) |

`--scenario` and `--seed` have no effect with `--joystick`: joystick flights use the true state and calm air.

## Watching a failure

Fault scenarios script a failure (lost RC link, lost GPS, low battery, a crashed brain and so on) and let the flight core's failsafes deal with it. The viewer shows what the core is doing: every event (`Armed`, `RC_LOST raised`, `action None -> Hold`, `nav Position -> Hold`, `Touchdown`, `Disarmed(Touchdown)`) is a line in the `events` text log at its simulation time, and is also printed to the terminal as it happens. The `plots/failsafe_action_rank` plot shows the failsafe action (0 None, 1 Hold, 2 Rtl, 3 Land, 4 LandNow, 5 Terminate), and `plots/valid/*` shows each estimate validity flag (0 or 1). Once the vehicle has armed, a yellow marker shows home in the 3D view, so a return flight can be followed.

```bash
cargo run --release --manifest-path crates/oxide-viewer/Cargo.toml -- --scenario rc-loss-position --speed 2
```

## Flying with the Taranis X9D+

1. Plug the radio into the PC by USB and choose **USB Joystick (HID)** when it asks.
2. In the radio's model, put the controls on channels 1–7 as follows (AETR order):

   | Channel | Control |
   |---|---|
   | 1 | Roll (aileron) |
   | 2 | Pitch (elevator) |
   | 3 | Throttle |
   | 4 | Yaw (rudder) |
   | 5 | Arm switch (up = armed) |
   | 6 | 3-position mode switch: stabilized / position / offboard |
   | 7 | Kill switch (up = kill) |

3. Run with `--joystick`. The first run prints the joystick's axes. If channels come out in the wrong order, remap them without rebuilding, for example to swap channels 1 and 2:

   ```bash
   OXIDE_JOYSTICK_ORDER=1,0,2,3,4,5,6 cargo run --release --manifest-path crates/oxide-viewer/Cargo.toml -- --joystick
   ```

   On Windows PowerShell, set it first with `$env:OXIDE_JOYSTICK_ORDER="1,0,2,3,4,5,6"`.

4. To fly: throttle down, then flip the arm switch. Arming needs the throttle low and a fresh flip of the switch.
   - **Stabilized:** the sticks set tilt and yaw rate, and the throttle sets thrust directly.
   - **Position:** the sticks set velocity, and centred sticks hold position. Mid-throttle holds altitude.

> The joystick path has not been tried with real hardware yet. Expect to adjust the channel mapping on the first run.
