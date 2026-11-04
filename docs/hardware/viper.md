# Titan Viper — setup review and help notes

**Aircraft:** Titan Dynamics Viper, a 1.6 m tri-tiltrotor VTOL. The two front motors tilt and the rear motor is fixed.
**Flight controller:** Matek F405-WTE, running ArduPlane (QuadPlane).
**Reviewed from:** the parameter export `Viper_Ardupilot_Nov_5.params`, the Titan Viper build manual (rev 1.0), and ArduPilot's documentation for this board.
**Status:** findings only. Nothing has been changed on the aircraft. We'll work through the fixes together later.

> ⚠️ This review comes from the parameter file alone, with no flight logs. Check every change on the bench with the **props off** before flying. Don't fly again until findings 1–3 are fixed.

---

## Before the next flight

1. Fix **findings 1, 2 and 3** below. These are the safety problems.
2. Redo the radio calibration (**finding 4**).
3. With the props off, run through these bench checks:
   - [ ] Every control surface moves the right way, with about ±35° of throw (the manual's recommendation).
   - [ ] Both tilt servos move the right way and stop at their limits without binding.
   - [ ] Motor order and spin direction are correct (Mission Planner → Setup → Optional Hardware → Motor Test).
   - [ ] The battery voltage in Mission Planner matches a multimeter.
   - [ ] Switching the transmitter off triggers the RC failsafe, and you can see the mode change.
4. First flight: QSTABILIZE or QHOVER, low, in calm air, logging everything. Set `LOG_DISARMED = 1` so the logs also cover the time before arming.

---

## Findings

### 1. ⚠️ The battery monitor reads the wrong pins, and the battery failsafes are off

| Parameter | Yours | ArduPilot's documented default for the F405-WTE |
|---|---|---|
| `BATT_VOLT_PIN` | 10 | **14** |
| `BATT_CURR_PIN` | 11 | **15** |
| `BATT_VOLT_MULT` | 10.95 | **21.0** |
| `BATT_AMP_PERVLT` | 50 | **66.7** |
| `BATT_FS_LOW_ACT` | 0 (none) | set an action |
| `BATT_FS_CRT_ACT` | 0 (none) | set an action |

**What it means:** pins 10/11 are the defaults for the older **F405-Wing** board, not the WTE. These settings were probably carried over from your F405-Wing bench setup, so the voltage and current readings are most likely wrong. On top of that, nothing happens when the battery gets low.

**Fix:**
1. Set the four values in the right-hand column and reboot.
2. Calibrate the voltage against a multimeter, then the current if you have a way to measure it (Setup → Optional Hardware → Battery Monitor).
3. Pick the low and critical battery actions from the list in Mission Planner. QRTL or RTL for low and QLAND for critical are the usual choices on a quadplane.
4. Check the voltage thresholds against your battery's cell count. Your current values (`BATT_LOW_VOLT 14.4`, `BATT_CRT_VOLT 14.0`) are for a 4S LiPo.

### 2. ⚠️ The right V-tail servo shares an output group with the DShot motors

| Output | Function | Signal |
|---|---|---|
| S5 | Motor 1 | DShot150 |
| S6 | Motor 4 (rear) | DShot150 |
| S7 | Motor 2 | DShot150 |
| **S8** | **V-tail right** (`SERVO8_FUNCTION 80`) | needs normal servo PWM |

**What it means:** on this board family, outputs **S5–S8 share one timer**. ArduPilot's docs say that if any output in a group uses DShot, *every* output in that group must use DShot. So the right V-tail servo is probably not getting a usable servo signal.

**Fix:**
1. Move the servo's signal wire from S8 to **S10**. S10 shares a group with S9, which already drives the left V-tail servo with normal PWM.
2. Set `SERVO10_FUNCTION = 80`, and copy across the S8 settings: `MIN 800`, `MAX 2200`, `TRIM 1500`, `REVERSED 0`.
3. Set `SERVO8_FUNCTION = 0`.
4. With the props off, check that the servo moves and goes the right way.

### 3. ⚠️ Flight-mode slot 4 is MANUAL

Your mode switch (channel 6) is set up as:

| Slot | Mode |
|---|---|
| 1 | QSTABILIZE |
| 2 | QHOVER |
| 3 | QAUTOTUNE |
| 4 | **MANUAL** |
| 5 | LOITER |
| 6 | QLOITER |

**What it means:** MANUAL is plain fixed-wing mode, with no stabilisation and no VTOL lift. If you select it by mistake while hovering, the aircraft will most likely fall.

**Fix:** replace it with something safe at hover, such as QLAND or QRTL. Or move MANUAL to a slot you can't reach by accident.

### 4. The radio calibration looks wrong

| Channel | Recorded min / centre / max | Problem |
|---|---|---|
| 1 (roll) | 1112 / **1626** / 2011 | The centre is about 65 µs off the middle. Right roll gets only **~75%** of the travel left roll gets. |
| 3 (throttle) | 988 / 991 / **2200** | A Taranis at normal ±100% endpoints tops out around 2012, so full stick probably registers as only **~84%**. |

**What it means:** roll authority is uneven, and the top of the throttle range may never be used. This usually happens when the calibration is done with a trim or subtrim set on the radio, or with extended endpoints.

**Fix:**
1. In the Taranis "viper" model, set the trims and subtrims on channels 1–4 to zero and the endpoints to 100%.
2. Redo Setup → Mandatory Hardware → Radio Calibration.
3. Check that `RC1_TRIM` ends up close to the midpoint of `RC1_MIN` and `RC1_MAX`, and that `RC3_MAX` is about 2012.

### 5. The yaw gains in hover are still the defaults

- **Yaw method:** `Q_TILT_TYPE = 2` (vectored yaw) with `Q_TILT_YAW_ANGLE = 9°`. This is **correct** for the Viper. With a fixed rear motor, yaw has to come from tilting the front motors differently, and ArduPilot needs type 2 for that. The manual's printed `Q_TILT_TYPE = 0` would make the front motors tilt together, so keep yours.
- **The gains:** `Q_A_RAT_YAW_P 0.18` and `Q_A_RAT_YAW_I 0.018` are ArduPilot's stock values. The manual specifically says to "greatly bump the yaw PIDs to get a less wiggly yaw in hover".

**Fix (after 1–4 are done):** raise the yaw gains in small steps, or run QAUTOTUNE on the yaw axis in calm air. QAUTOTUNE is already on mode slot 3.

### 6. The vibration notch filter is off

`INS_HNTCH_ENABLE = 0`. Big, slow props shake the flight controller, and ArduPilot recommends the harmonic notch filter on VTOLs and multirotors.

**Fix:** needs a hover log first, to find the vibration frequencies. If your ESCs report RPM, use the RPM-based notch; otherwise use the throttle-based one. We'll do this together once there's a log.

### 7. The forward-flight speed limits are below stall

| Parameter | Yours | Problem |
|---|---|---|
| `ARSPD_FBW_MIN` | 9 m/s | Below stall at every weight in the manual |
| `Q_ASSIST_SPEED` | 5 m/s | VTOL motors would only help well after a stall had started |
| `TRIM_ARSPD_CM` | 12 m/s | Below the manual's 45–60 km/h (12.5–16.7 m/s) cruise range |
| `ARSPD_PIN` | 15 | Pin 15 is the WTE's **current sensor**, not an airspeed input |
| `ARSPD_USE` | 0 | No airspeed sensor in use. The manual says one is "highly recommended". |

**Stall speeds**, calculated from the manual's figures (wing area 0.3757 m², C_Lmax 1.3). They match the manual's own plots. The manual's "minimum speed" is 1.2× stall. Your barometer's ground pressure (963.6 hPa) suggests you fly at roughly 420 m elevation, where the air is thinner and the speeds come out about 2% higher:

| All-up weight | Stall (sea level) | Minimum speed (sea level) | Stall (~420 m) | Minimum speed (~420 m) |
|---|---|---|---|---|
| 3.0 kg | 9.9 m/s | 11.9 m/s | 10.1 m/s | 12.1 m/s |
| 3.5 kg | 10.7 | 12.9 | 10.9 | 13.1 |
| 4.0 kg | 11.5 | 13.7 | 11.7 | 14.0 |
| 4.5 kg | 12.1 | 14.6 | 12.4 | 14.9 |
| 5.0 kg | 12.8 | 15.4 | 13.1 | 15.7 |
| 5.5 kg | 13.4 | 16.1 | 13.7 | 16.4 |

**Fix:**
1. Weigh the aircraft.
2. Set `ARSPD_FBW_MIN` to at least the "minimum speed" for that weight.
3. Set `Q_ASSIST_SPEED` just above stall and `TRIM_ARSPD_CM` inside the cruise range.
4. Set `ARSPD_TYPE = 0` until an airspeed sensor is fitted. A digital I2C airspeed sensor is the recommended upgrade.

None of this matters until you fly forward, but it all matters the first time you transition.

### 8. The battery is set up for 4S, but the manual assumes 6S

`Q_M_BAT_VOLT_MAX 16.8` and `Q_M_BAT_VOLT_MIN 13.2` mean 4S. The manual's recommended setup (Brotherhobby Tornado T5 3115 **640KV** with APC 13×8E props) is meant for **6S**. On 4S those motors spin about a third slower and make less than half the static thrust (thrust scales with RPM squared: (14.8/22.2)² ≈ 0.44).

**Fix:** none needed if your motors and props were chosen for 4S. Please note down your motor, prop, ESC, battery and all-up weight. The simulator's Viper model needs them anyway.

---

## What the parameters say about the current setup

| Item | Value |
|---|---|
| Firmware | ArduPlane (QuadPlane). The version isn't recorded in a `.param` file. |
| IMU | ICM-42688-P (SPI), decoded from `INS_ACC_ID` |
| Barometer | SPL06 at I2C address 0x76, decoded from `BARO1_DEVID` |
| Compass | QMC5883L at I2C address 0x0D, external (in the GPS module) |
| Board orientation | Rotated 180° in yaw (`AHRS_ORIENTATION 4`) |
| GPS | `SERIAL3`, 38400 baud, 5 Hz |
| Frame | Tri (`Q_FRAME_CLASS 7`). Motors 1 and 2 tilt (`Q_TILT_MASK 3`). Maximum tilt 45° at 20°/s. |
| Outputs | S1 aileron · S2 left tilt · S3 aileron · S4 right tilt · S5 motor 1 · S6 motor 4 · S7 motor 2 · S8 V-tail right · S9 V-tail left |
| Motor protocol | DShot150 (`Q_M_PWM_TYPE 4`), BLHeli passthrough on S5–S7 |
| Arming | Switch on channel 5 (`RC5_OPTION 153`) |
| Logging | Everything (`LOG_BITMASK 65535`), but not while disarmed |
| History | 101 boots, ~43 min of flight time, ~25 h powered on. The SD card should hold real flight logs. |

## Viper specifications (from the build manual)

- **Size:** span 1608 mm, wing area 3757 cm², aspect ratio 6.88
- **Weight:** maximum takeoff weight 5.5 kg
- **Wing:** airfoil NACA 3412 at root and tip; chord 255 mm at the root, 162 mm at the tip (mean 212 mm); incidence 1° at the root, 0° at the tip; no dihedral or leading-edge sweep; −3° trailing-edge sweep
- **Performance:** maximum L/D 14; cruise 45–60 km/h; efficiency 1.95 Wh/km at 4.0 kg
- **Recommended propulsion:**
  - motors: Brotherhobby Tornado T5 3115 640KV
  - props: APC 13×8E, up to 15". Use 12×6 for longer hover.
  - ESCs: T-Motor 55A 4-in-1, or three 51A BLHeli_32 ESCs
  - batteries: 6S, for example a 5000 mAh LiPo, or a 6S3P Molicel P45B Li-ion pack for maximum endurance
- **Servos:** 20 kg·cm tilt servos, and 4× Emax ES08MAII for the ailerons and ruddervators
- **Setup notes:** about ±35° of throw on every control surface; the CG is marked under the wing; airspeed sensor highly recommended

## Data to collect later (deferred item Q24)

When the simulator's sensor models need real data:

1. Every `.BIN` log from the SD card (or Mission Planner → DataFlash Logs → Download All).
2. A props-off bench log. Set `LOG_DISARMED = 1`, put the aircraft on a still surface, power it up and leave it for 30 minutes or more.
3. All-up weight and CG position.
4. Motor, prop, ESC and battery models.
5. A fresh parameter export after any changes.

## Sources

- ArduPilot docs, "Matek F405-WTE / F405-TE family" (`common-matekf405-te.html`): ADC pins, timer groups, sensors.
- ArduPilot docs, "Matek F405-Wing" (`common-matekf405-wing.html`): the Wing defaults that match the current battery settings.
- ArduPilot source: `AP_InertialSensor_Backend.h` and `AP_Baro_Backend.h` (to decode the device IDs), and `ArduPlane/tiltrotor.cpp` (meaning of `Q_TILT_TYPE` and `Q_TILT_YAW_ANGLE`).
- Titan Dynamics Viper Build & User Manual, revision 1.0.
