# Titan Viper — setup review and help notes

**Aircraft:** Titan Dynamics Viper, a 1.6 m tri-tiltrotor VTOL. The two front motors tilt and the rear motor is fixed.
**Flight controller:** Matek F405-WTE, running ArduPlane (QuadPlane).
**Reviewed from:** the parameter export `Viper_Ardupilot_Nov_5.params`, the Titan Viper build manual (rev 1.0), and ArduPilot's documentation and source for this board.
**Status:** findings only. Nothing has been changed on the aircraft. We'll work through the fixes together later.
**Revision:** v2. Corrected and extended after an independent review on 2026-11-04 (see the end of this document).

> ⚠️ This review comes from the parameter file alone, with no flight logs. Check every change on the bench with the **props off** before flying. **Don't fly again until findings 1, 2, 3 and 9 are fixed.**

---

## Before the next flight

1. **Fix findings 1, 2, 3 and 9.** These are the safety problems.
2. Redo the radio calibration (**finding 4**).
3. Bench checks, **props off**:
   - [ ] Every control surface moves the right way, with about ±35° of throw (the manual's recommendation).
   - [ ] Both tilt servos move the right way and stop at their limits without binding.
   - [ ] Motor order and spin direction are correct (Mission Planner → Setup → Optional Hardware → Motor Test).
   - [ ] The battery voltage shown in Mission Planner matches a multimeter.
   - [ ] Move the mode switch through **every** position and confirm the HUD shows the intended mode at each one.
   - [ ] Every stick moves its bar the right way, including yaw: a right yaw stick should give a nose-right command.
   - [ ] RC failsafe: in QHOVER, switch the transmitter off. The HUD should show **QLAND** and a radio-failsafe message. Repeat in QSTABILIZE.
4. Then follow the **tuning order** below.
5. First flight: QSTABILIZE or QHOVER, low, in calm air, logging everything. Set `LOG_DISARMED = 1` and `INS_LOG_BAT_MASK = 1` first.

Optional, and very effective: load this parameter file into ArduPilot SITL (a QuadPlane tilt-tri model) and trigger each failsafe there first.

---

## Findings

### 1. ⚠️ The battery monitor probably reads the wrong pins, and the battery failsafes are off

| Parameter | Yours | ArduPilot's documented default for the F405-WTE (MatekF405-TE firmware) |
|---|---|---|
| `BATT_VOLT_PIN` | 10 | **14** |
| `BATT_CURR_PIN` | 11 | **15** |
| `BATT_VOLT_MULT` | 10.95 | **21.0** |
| `BATT_AMP_PERVLT` | 50 | **66.7** |
| `BATT_FS_LOW_ACT` | 0 (warn only) | set an action (step 3) |
| `BATT_FS_CRT_ACT` | 0 (warn only) | set an action (step 3) |

**What it means:**
- **Wrong board's defaults:** your pins and voltage scale match the defaults for the older **F405-Wing** board. So unless step 0 below shows otherwise, the voltage and current readings are probably wrong.
- **No action on low battery:** both battery failsafe actions are set to warn only, so the aircraft does nothing when the battery gets low.

**Fix:**
0. **Check before changing anything.** Compare the voltage in Mission Planner with a multimeter, and read the firmware banner in the Messages tab (it should say `MatekF405-TE`). If the voltage already matches, your current pins work for the firmware you have. Stop and confirm the board and firmware before applying the values in the table.
1. Set the four values in the right-hand column and reboot.
2. Calibrate the voltage against the multimeter, then the current if you can measure it (Setup → Optional Hardware → Battery Monitor).
3. Set **both** `BATT_FS_LOW_ACT` and `BATT_FS_CRT_ACT` to **QLand**. On current ArduPlane the battery actions are 0 Warn only, 1 RTL, 2 Land, 3 Terminate, **4 QLand**, 5 Parachute, 6 Loiter to QLand, 7 AUTOLAND or RTL. Your firmware is older, so pick "QLand" from the list Mission Planner shows rather than typing the number.
   - **QRTL is not one of the options.**
   - **Don't choose RTL yet.** RTL is a fixed-wing return (finding 9), so a battery failsafe while hovering would start a forward transition.
4. Check the voltage thresholds against your battery's chemistry and cell count. Your current values (`BATT_LOW_VOLT 14.4`, `BATT_CRT_VOLT 14.0`) suit a 4S LiPo. `BATT_CAPACITY = 18000` mAh suggests a large pack, and if it's Li-ion the per-cell thresholds are different.
5. After the fix, let hover throttle re-learn before any autotune. `Q_M_HOVER_LEARN = 2` is already set.

### 2. ⚠️ The right V-tail servo shares an output group with the DShot motors

| Output | Function | Signal |
|---|---|---|
| S5 | Motor 1 | DShot150 |
| S6 | Motor 4 (rear) | DShot150 |
| S7 | Motor 2 | DShot150 |
| **S8** | **V-tail right** (`SERVO8_FUNCTION 80`) | needs normal servo PWM |

**What it means:** on this board family, outputs **S5–S8 share one timer**. If any output in a group uses DShot, *every* output in that group must use DShot. So the right V-tail servo is probably not getting a usable servo signal.

**Fix:**
1. Move the servo's signal wire from S8 to **S10**. S10 shares a group with S9, which already drives the left V-tail servo with normal PWM.
2. Set `SERVO10_FUNCTION = 80` and copy across the S8 settings: `MIN 800`, `MAX 2200`, `TRIM 1500`, `REVERSED 0`.
3. Set `SERVO8_FUNCTION = 0`.
4. Props off: check that the servo moves the right way.

### 3. ⚠️ The mode switch's centre position is probably MANUAL

Your mode slots (channel 6) are: 1 QSTABILIZE · 2 QHOVER · 3 QAUTOTUNE · 4 **MANUAL** · 5 **LOITER** · 6 QLOITER.

**Which slots can your switch actually reach?** ArduPilot picks the slot from the channel-6 signal:

| Channel 6 signal | Slot |
|---|---|
| below 1231 µs | 1 |
| 1231–1360 µs | 2 |
| 1361–1490 µs | 3 |
| 1491–1620 µs | 4 |
| 1621–1749 µs | 5 |
| 1750 µs and above | 6 |

A plain 3-position switch (about 988 / 1500 / 2012 µs) reaches **only slots 1, 4 and 6**. With your settings those are QSTABILIZE, **MANUAL in the middle position**, and QLOITER. QHOVER and QAUTOTUNE would be unreachable.

**What it means:**
- **MANUAL** is plain fixed-wing mode, with no stabilisation and no VTOL lift. Selecting it while hovering will most likely make the aircraft fall.
- **LOITER** (slot 5) is the *fixed-wing* loiter. Selecting it from a hover starts a forward transition.

**Fix:**
1. Open Mission Planner's Radio Calibration screen and note the channel-6 value in every switch position. (Also check which physical switch, or which radio mix, drives channel 6.)
2. For the hover-only phase, put **only VTOL modes** on every position you can reach. For a 3-position switch, for example:
   - slots 1–3: QSTABILIZE
   - slot 4: QHOVER
   - slots 5–6: QLOITER

   Put QLAND only on a position you choose deliberately.
3. Keep MANUAL, FBWA, LOITER and every other fixed-wing mode off any switch you can reach while hovering. They go on a separate, deliberate switch once forward flight is planned (finding 7).
4. Do the mode-switch bench check above.

### 4. The radio calibration looks wrong

| Channel | Recorded min / centre / max | Problem |
|---|---|---|
| 1 (roll) | 1112 / **1626** / 2011 | The centre is about 65 µs off the middle, and right roll gets only **~75%** of the travel left roll gets |
| 3 (throttle) | 988 / 991 / **2200** | SBUS can't produce 2200 µs (its maximum is about 2150), so this value was typed or imported, not calibrated |

**What it means:**
- **Roll.** If a centred roll stick actually sends about 1500 µs, the stored trim of 1626 turns it into a **~20% left-roll demand, roughly 6° of bank in QSTABILIZE**. The aircraft would lean left unless you hold the stick right.
- **Throttle.** The bad maximum limits full throttle stick to about 84%. It also moves QHOVER's climb-neutral point to about 1594 µs, so a centred throttle stick slowly descends.

**Fix:**
0. **Check first.** With the sticks centred, open Radio Calibration and note channel 1. About 1500 means the stored trim is stale. About 1626 means the radio has a subtrim or mix on that channel.
1. In the Taranis "viper" model, set the trims and subtrims on channels 1–4 to zero and the endpoints to 100%.
2. Redo Setup → Mandatory Hardware → Radio Calibration.
3. Expect about 988 / 1500 / 2012 on every stick channel. `RC1_TRIM` should land near the middle and `RC3_MAX` near 2012.
4. Check that every stick moves its bar the right way. `RC4_REVERSED = 1` (yaw) is unusual, so with the props off confirm that a right yaw stick gives a nose-right command.

### 5. Hover yaw: the setup is right, the gains are stock

- **Yaw method:** `Q_TILT_TYPE = 2` (vectored yaw) with `Q_TILT_YAW_ANGLE = 9°` is **correct** for the Viper. With a fixed rear motor, yaw has to come from tilting the front motors differently, and ArduPilot needs type 2 for that. The manual prints `Q_TILT_TYPE = 0`, which would make the front motors tilt together, so keep yours.
- **The gains:** yaw (`Q_A_RAT_YAW_P 0.18`, `I 0.018`), and roll and pitch too, look like ArduPilot's stock values. The manual says to "greatly bump the yaw PIDs to get a less wiggly yaw in hover".
- **Yaw authority** is limited by the 9° vectoring angle and any slop in the tilt servos. Raising the gain too far can saturate the servos and oscillate.

**Fix:** follow the tuning order below. The notch filter comes *before* autotune.

### 6. The vibration notch filter is off

`INS_HNTCH_ENABLE = 0`. Big, slow props shake the flight controller, and ArduPilot sets up the harmonic notch **before** autotune.

**Fix:** set it from a hover log recorded with the batch sampler on (`INS_LOG_BAT_MASK = 1`).
- **Notch type:** use the throttle-based notch. The RPM-based notch needs ESC telemetry wired to a UART, which isn't configured (`SERIAL4/5` are unused).
- **Order:** see the tuning order below.

### 7. Forward flight: not yet. The limits are below stall and there's no airspeed sensor

| Parameter | Yours | Problem |
|---|---|---|
| `ARSPD_FBW_MIN` | 9 m/s | Below stall at every weight in the manual |
| `Q_ASSIST_SPEED` | 5 m/s | VTOL motors would only help well after a stall has started |
| `TRIM_ARSPD_CM` | 12 m/s | Below the manual's 45–60 km/h (12.5–16.7 m/s) cruise range |
| `ARSPD_TYPE` | 1 | Type 1 is a digital I2C (MS4525-type) sensor, so `ARSPD_PIN` is ignored. `ARSPD_USE = 0`: no airspeed sensor is used. The manual calls one "highly recommended". |
| `Q_TRANS_FAIL` | 0 | A transition that never finishes is never aborted |
| `Q_ASSIST_ALT` | 0 | No altitude-based VTOL assist |

**Stall speeds**, estimated from the manual's figures (wing area 0.3757 m², C_Lmax 1.3), match the manual's own plots. The manual's "minimum speed" is 1.2× stall. Your barometer's ground pressure (963.6 hPa) suggests you fly at roughly 420 m elevation, where the speeds come out about 2% higher:

| All-up weight | Stall (sea level) | Minimum speed (sea level) | Stall (~420 m) | Minimum speed (~420 m) |
|---|---|---|---|---|
| 3.0 kg | 9.9 m/s | 11.9 m/s | 10.1 m/s | 12.1 m/s |
| 3.5 kg | 10.7 | 12.9 | 10.9 | 13.1 |
| 4.0 kg | 11.5 | 13.7 | 11.7 | 14.0 |
| 4.5 kg | 12.1 | 14.6 | 12.4 | 14.9 |
| 5.0 kg | 12.8 | 15.4 | 13.1 | 15.7 |
| 5.5 kg | 13.4 | 16.1 | 13.7 | 16.4 |

**These are estimates, and probably optimistic:**
- **Wing area.** The manual's wing area (3757 cm²) doesn't match its own average chord × span (212 mm × 1608 mm ≈ 3410 cm²). With the smaller area, stall is about **5% higher**.
- **C_Lmax.** C_Lmax 1.3 is the manual's assumption, not a measurement.
- **Turns.** Banking raises stall speed. `STALL_PREVENTION = 1` limits bank only as well as the airspeed reading allows.

**Fix (only when you plan forward flight):**
1. **No forward transition and no fixed-wing mode until a working airspeed sensor is fitted and validated.** Check whether one is fitted by looking for `ARSP` messages in a log.
   - **None fitted:** set `ARSPD_TYPE = 0`.
   - **One fitted:** keep type 1, check `ARSPD_BUS` matches the I2C port it's wired to, calibrate it, and set `ARSPD_USE = 1` only after a ground test and a walking test.
2. Weigh the aircraft. Set `ARSPD_FBW_MIN` to at least **1.3× the stall speed** for that weight. Or keep 1.2× and limit bank with `LIM_ROLL_CD = 3000` for the first fixed-wing flights.
3. Set `TRIM_ARSPD_CM` inside the cruise range and above `ARSPD_FBW_MIN` with a margin, and `Q_ASSIST_SPEED` just above stall.
4. **Before the first transition:**
   - Set `Q_TRANS_FAIL` to about 20–30 s, and confirm in Mission Planner's list that `Q_TRANS_FAIL_ACT` is QLand.
   - Set `Q_ASSIST_ALT` to a safe height.
   - Fly the first transitions at height, over open ground.

### 8. The battery is set up for 4S, but the manual assumes 6S

`Q_M_BAT_VOLT_MAX 16.8` and `Q_M_BAT_VOLT_MIN 13.2` mean 4S. The manual's recommended setup (Brotherhobby Tornado T5 3115 **640KV** with APC 13×8E props) is meant for **6S**. On 4S those motors spin about a third slower and make less than half the static thrust (thrust scales with RPM squared: (14.8/22.2)² ≈ 0.44).

**Fix:** none needed if your motors and props were chosen for 4S. Please note your motor, prop, ESC, battery and all-up weight. The simulator's Viper model needs them too.

### 9. ⚠️ Failsafe and return-to-launch behaviour

**What happens today, according to the export:**
- **RC lost in any Q mode** (QSTABILIZE, QHOVER, QLOITER…): the aircraft goes to **QLAND** and lands where it is (`Q_OPTIONS = 0`). That's acceptable for the first hovers.
- **RC lost in MANUAL, LOITER or another fixed-wing mode:** the aircraft goes to **RTL**. Because `Q_RTL_MODE = 0` and `RTL_AUTOLAND = 0`, RTL flies home and **circles at `ALT_HOLD_RTL` = 100 m without landing**. With the battery failsafes off (finding 1), it circles until the battery is flat.
- **No ground-station failsafe** (`FS_GCS_ENABL = 0`), and **no geofence** (`FENCE_ENABLE = 0`).

**Fix for the hover-only phase:**
1. Battery actions to QLand (finding 1, step 3).
2. Set `Q_RTL_MODE = 1`, so that any RTL ends with a VTOL landing at home instead of circling.
3. Keep the RC failsafe in Q modes as QLAND for now. `Q_OPTIONS` bit 5 (value 32) would make it QRTL; leave that until QRTL has been tested.
4. Keep fixed-wing modes off the reachable switch positions (finding 3), so a fixed-wing RTL can't be triggered while hovering.
5. Optional: a simple altitude-and-radius geofence for the first flights.
6. Test all of this props-off, or in SITL with this parameter file: switch the transmitter off in QHOVER and confirm the mode the aircraft enters.

---

## Tuning order

Do these in order once findings 1–4 and 9 are fixed:

1. Set `INS_LOG_BAT_MASK = 1` (batch sampling on the first IMU) and `LOG_DISARMED = 1`.
2. Fly a short, low hover in QSTABILIZE or QHOVER in calm air, on the current gains.
3. From that log, set up the **throttle-based harmonic notch** (finding 6).
4. **Compass check:** plot compass field strength against throttle in the same log. `COMPASS_MOT` is not calibrated. If the field changes noticeably with throttle, move the GPS mast or run compass-motor calibration.
5. Let hover throttle re-learn.
6. Put QAUTOTUNE on a reachable switch position (finding 3). Run it on roll and pitch, then on yaw.
7. Raise the yaw gains by hand only if the wiggle remains (finding 5).

---

## What the parameters say about the current setup

| Item | Value |
|---|---|
| Firmware | ArduPlane (QuadPlane). The version isn't recorded in a `.param` file; the Messages tab shows it. |
| IMU | ICM-42688-P (SPI), decoded from `INS_ACC_ID` |
| Barometer | SPL06 at I2C address 0x76, decoded from `BARO1_DEVID` |
| Compass | QMC5883L at I2C address 0x0D, external (in the GPS module). Calibrated, but not compensated for motor current. |
| Board orientation | Rotated 180° in yaw (`AHRS_ORIENTATION 4`) |
| GPS | `SERIAL3`, 38400 baud, 5 Hz |
| Frame | Tri (`Q_FRAME_CLASS 7`); motors 1 and 2 tilt (`Q_TILT_MASK 3`). `Q_TILT_MAX 45` is the largest tilt at which VTOL (multicopter) control is still used; the servos' mechanical travel is set by `SERVO2/4 MIN/MAX`. Tilt rate 20°/s. |
| Outputs | S1 aileron · S2 left tilt · S3 aileron · S4 right tilt · S5 motor 1 · S6 motor 4 · S7 motor 2 · S8 V-tail right · S9 V-tail left |
| Motor protocol | DShot150 (`Q_M_PWM_TYPE 4`), BLHeli passthrough on S5–S7 |
| Arming | Switch on channel 5 (`RC5_OPTION 153`) |
| Failsafes | Q modes: RC loss → QLAND. Fixed-wing modes: RC loss → RTL, which circles at 100 m (finding 9). No battery, GCS or geofence actions. |
| Logging | Everything (`LOG_BITMASK 65535`), but not while disarmed, and no IMU batch sampling |
| History | 101 boots, ~43 min of flight time, ~25 h powered. The SD card should hold real flight logs. Logs from before these fixes are "pre-fix" data. |

## Viper specifications (from the build manual)

- **Size:** span 1608 mm; wing area 3757 cm², as stated (but see finding 7); aspect ratio 6.88
- **Weight:** maximum takeoff weight 5.5 kg
- **Wing:** airfoil NACA 3412 at root and tip; chord 255 mm at the root, 162 mm at the tip (mean 212 mm); incidence 1° at the root, 0° at the tip; no dihedral or leading-edge sweep; −3° trailing-edge sweep
- **Performance:** maximum L/D 14; cruise 45–60 km/h; efficiency 1.95 Wh/km at 4.0 kg
- **Recommended propulsion:**
  - motors: Brotherhobby Tornado T5 3115 640KV
  - props: APC 13×8E, up to 15". Use 12×6 for longer hover.
  - ESCs: T-Motor 55A 4-in-1, or three 51A BLHeli_32 ESCs
  - batteries: 6S, for example a 5000 mAh LiPo, or a 6S3P Molicel P45B Li-ion pack for endurance
- **Servos:** 20 kg·cm tilt servos; 4× Emax ES08MAII for the ailerons and ruddervators
- **Setup notes:** about ±35° of throw on every control surface; the CG is marked under the wing; airspeed sensor highly recommended

## Data to collect later (deferred item Q24)

1. Every `.BIN` log from the SD card (or Mission Planner → DataFlash Logs → Download All). Mark them "pre-fix".
2. A props-off bench log: set `LOG_DISARMED = 1` **and** `INS_LOG_BAT_MASK = 1`, put the aircraft on a still surface, power it up and leave it for 30+ minutes (2 hours or more if you can).
3. All-up weight and CG position.
4. Motor, prop, ESC and battery models.
5. A fresh parameter export after the fixes.

## Sources

- **ArduPilot docs:** "Matek F405-WTE / F405-TE family" (`common-matekf405-te.html`) and "Matek F405-Wing" (`common-matekf405-wing.html`).
- **ArduPilot source:**
  - `AP_InertialSensor_Backend.h` and `AP_Baro_Backend.h` (decoding the device IDs);
  - `ArduPlane/tiltrotor.cpp` (`Q_TILT_TYPE`, `Q_TILT_YAW_ANGLE`);
  - `ArduPlane/quadplane.cpp` (`Q_RTL_MODE`, `Q_OPTIONS`, `Q_TRANS_FAIL_ACT`, `Q_TRANSITION_MS`);
  - `ArduPlane/events.cpp` (RC failsafe actions);
  - `AP_BattMonitor_Params.cpp` (battery failsafe actions);
  - `RC_Channel.cpp` (mode-switch PWM bands);
  - `MatekF405-TE` hwdef (pins and timers).
- **Titan Dynamics:** Viper Build & User Manual, revision 1.0.

### Review history

- **v1:** first review from the parameter export.
- **v2 (2026-11-04):** corrected after an independent review. Changes:
  - **battery action:** "QRTL" isn't a valid battery action, so it is now QLand;
  - **mode switch:** a 3-position switch puts MANUAL in the centre;
  - **airspeed:** `ARSPD_TYPE 1` is a digital sensor, so the pin isn't the issue;
  - **calibration cause:** the explanation for `RC3_MAX` was corrected;
  - **added:** failsafe/RTL behaviour (finding 9), the tuning order, the compass check, transition settings, stall-estimate caveats, and "check before changing" steps.
