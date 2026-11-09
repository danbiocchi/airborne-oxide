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
| `--scenario gust-hold` | Fly the hold-in-gusts scenario (realistic sensors, wind, EKF) instead of the default `hover-step` |
| `--seed 3` | Run index for `gust-hold` (default 0) |

`--scenario` and `--seed` have no effect with `--joystick`: joystick flights use the true state and calm air.
| `--profile h743` | Use the H743 task schedule instead of the F405 one |
| `--speed 4` | Run at 4× real time (`0` = as fast as possible) |
| `--save flight.rrd` | Write a recording file instead of opening the viewer (open it later with `rerun flight.rrd`) |

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
