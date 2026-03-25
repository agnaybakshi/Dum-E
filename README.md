# Vision Arm Teleoperation v1

This project is a first working teleoperation stack for a small 5-servo tabletop arm driven by a single front-facing webcam, Python, and a PCA9685-based controller board. The laptop-side teleop logic is unchanged, and the transport can now be either Arduino serial or ESP32 Wi-Fi UDP.

## Architecture summary

- `main.py` runs the control loop, keyboard safety controls, target generation, and transport output.
- `vision.py` tracks one hand with MediaPipe Hands and extracts stable hand features.
- `hand_mapping.py` converts hand motion into teleop controls:
  - horizontal hand offset -> base yaw rate command
  - vertical hand position -> upper-arm pitch target
  - relative hand size/depth proxy -> lower-arm pitch target
  - upper/lower average -> middle-arm pitch target
  - whole-hand finger curl -> gripper opening
- `filters.py` provides low-pass filters, deadbands, and slew limiting.
- `transport.py` defines the shared teleop packet and transport interface.
- `serial_comm.py` is the serial transport implementation.
- `udp_comm.py` is the UDP transport implementation.
- `send.py` is a UDP diagnostic sender for quick ESP32 firmware testing.
- `calibration.py` loads/saves JSON calibration and exports generated firmware headers.
- `arduino/vision_arm_uno.ino` parses packets, clamps values, applies watchdogs, and drives the servos through the PCA9685.
- `arduino/esp32/vision_arm_esp` is the ESP32 DevKit V1 PlatformIO target using Arduino framework, Wi-Fi UDP, and PCA9685.

## Important design limits

- The base servo is continuous rotation, so v1 uses open-loop base rate control only. Absolute base angle is not known.
- The single RGB camera provides only a relative depth proxy. Reach control is therefore relative and must be tuned experimentally.
- The current teleop version uses direct joint mapping for the three pitch joints:
  - lower joint from relative depth
  - upper joint from hand height
  - middle joint as the average of upper and lower

## Files

- `config/calibration.json`: primary tuning and calibration source
- `arduino/generated_calibration.h`: generated from the JSON for the Uno sketch
- `arduino/vision_arm_uno.ino`: flash this to the Arduino Uno
- `arduino/esp32/vision_arm_esp`: PlatformIO project for ESP32 DevKit V1

## Python setup

1. Create and activate a Python environment.
2. Install dependencies:

```bash
pip install -r requirements.txt
```

3. Edit `config/calibration.json` and select a transport:
   - serial: set `transport.kind` to `serial` and choose `serial.port`
   - UDP: set `transport.kind` to `udp` and use the fixed ESP32 IP from `wifi_secrets.h` for `udp.host` (default in this repo: `192.168.137.50`), then keep `udp.port: 4210`
4. Regenerate the firmware headers:

```bash
python calibration.py export-firmware
```

5. Flash either:
   - `arduino/vision_arm_uno.ino` for the Uno serial path
   - `arduino/esp32/vision_arm_esp` for the ESP32 UDP path
6. Run the teleop app:

```bash
python main.py
```

Optional dry-run without transport:

```bash
python main.py --no-serial
```

Install the Arduino library `Adafruit PWM Servo Driver Library` before building either firmware target.

PCA9685 wiring and channel map for Arduino Uno:
- I2C SDA -> `A4`
- I2C SCL -> `A5`
- Base yaw servo -> channel `0`
- Lower arm pitch -> channel `1`
- Middle arm pitch -> channel `2`
- Upper arm pitch -> channel `3`
- Gripper -> channel `4`

## Keyboard controls

- `q`: quit
- `f` or `space`: freeze/unfreeze teleop
- `x`: toggle emergency stop
- `h`: send home/hold request
- `n`: capture current hand as neutral position and depth reference
- `o`: capture current hand as the open-grip reference
- `p`: capture current hand as the closed-grip reference
- `r`: reload `config/calibration.json`
- `[`: nudge base neutral trim negative
- `]`: nudge base neutral trim positive

## ESP32 UDP migration

What changed:

- The laptop still generates the same teleop command semantics and the same packet format:

```text
T,seq,mode,base_cmd,lower_deg,middle_deg,upper_deg,gripper_open
```

- `main.py` now selects either serial or UDP transport from config.
- The ESP32 firmware receives those exact ASCII packets over UDP and drives the PCA9685 with the same calibration mapping style as the Uno firmware.

Wi-Fi setup:

1. Copy [wifi_secrets.h.example](C:/Users/agnay/Documents/Agnisys/vision_arm/arduino/esp32/vision_arm_esp/include/wifi_secrets.h.example) to `wifi_secrets.h`.
2. Edit [wifi_secrets.h](C:/Users/agnay/Documents/Agnisys/vision_arm/arduino/esp32/vision_arm_esp/include/wifi_secrets.h).
3. Set `VISION_ARM_WIFI_SSID` and `VISION_ARM_WIFI_PASSWORD`.
4. `wifi_secrets.h` is intentionally ignored by Git and should not be committed.
5. Set the static IP values in the same file. This repo currently uses:
   - `VISION_ARM_WIFI_STATIC_IP = 192.168.137.50`
   - `VISION_ARM_WIFI_GATEWAY = 192.168.137.1`
   - `VISION_ARM_WIFI_SUBNET = 255.255.255.0`
6. Make sure that SSID is a `2.4 GHz` network. ESP32 DevKit V1 does not join `5 GHz` Wi-Fi.
7. If you move to a different Wi-Fi subnet later, update both `wifi_secrets.h` and `udp.host` in `config/calibration.json` to match that subnet.

ESP32 wiring to the PCA9685:

- `GPIO21` -> PCA9685 `SDA`
- `GPIO22` -> PCA9685 `SCL`
- `3V3` -> PCA9685 `VCC`
- `GND` -> PCA9685 `GND`
- external servo supply `+` -> PCA9685 `V+`
- external servo supply `-` -> PCA9685 `GND`

The ESP32 ground, PCA9685 ground, and servo power ground must all be common.

Flash the ESP32 with PlatformIO:

```bash
cd arduino/esp32/vision_arm_esp
pio run
pio run --target upload
pio device monitor
```

The ESP32 will request the fixed IP from `wifi_secrets.h` and print that IP plus the UDP listen port after it successfully joins Wi-Fi.

Test the ESP32 without MediaPipe:

```bash
python send.py --host 192.168.137.50 --port 4210 --mode A --base 0.0 --lower 10 --middle 20 --upper 30 --gripper 0.5 --rate 20 --count 0
```

Switch transports in `config/calibration.json`:

- serial:

```json
"transport": { "kind": "serial", "enabled": true }
```

- udp:

```json
"transport": { "kind": "udp", "enabled": true }
```

What remains unchanged:

- camera / MediaPipe hand tracking
- filtering and mapping logic
- teleop modes `A`, `H`, `S`, `M`
- joint-angle and gripper-open command semantics
- calibration JSON as the master source

## Calibration workflow

Use the JSON as the master calibration file, then regenerate the firmware headers whenever firmware constants change.

### Vision and teleop

1. Start with `python main.py --no-serial`.
2. Stand in front of the camera with your operating hand in the intended neutral position.
3. Press `n` to store `neutral_x`, `neutral_y`, and `depth_reference`.
4. Open your hand fully and press `o`.
5. Close/curl your fingers and press `p`.
6. Adjust the teleop region in `config/calibration.json` if you want a larger or smaller active box.
7. Tune `mapping.yaw_deadband` and the firmware joint signs until motion feels natural.

### Servo and base calibration

1. Power the arm safely with the links unloaded or mechanically free to move.
2. Keep `freeze_on_start` set to `true` until calibration is complete.
3. Edit the firmware section in `config/calibration.json`:
   - `base_stop_deg`
   - `base_speed_range_deg`
   - `base_direction_sign`
   - `lower_zero_deg`, `middle_zero_deg`, `upper_zero_deg`
   - `lower_sign`, `middle_sign`, `upper_sign`
   - `gripper_open_servo_deg`, `gripper_closed_servo_deg`
4. Run:

```bash
python calibration.py export-firmware
```

5. Rebuild and reflash the target firmware after header changes.

### Base drift fix

If the base drifts with no hand input:

1. Start the app with teleop still frozen.
2. Check the overlay. `Status` should be `startup hold` or `frozen`, and `Base cmd` should match your current neutral trim.
3. If the base still creeps, tune the continuous-servo neutral stop:

```bash
python calibration.py set-base-stop --deg 91
python calibration.py export-firmware
```

4. Rebuild and reflash the target firmware and test again.
5. Try nearby values such as `89`, `90`, `91`, or `92` until the base truly stops at zero command.

`base_trim_command` is currently used as the neutral stop trim for the continuous-rotation base, including hold modes.

## Teleop protocol

Python sends newline-terminated packets:

```text
T,seq,mode,base_cmd,lower_deg,middle_deg,upper_deg,gripper_open
```

- `mode`:
  - `A` active teleop
  - `H` hold last pose and stop base
  - `S` stop base and hold pose
  - `M` go to configured home pose
- `base_cmd`: normalized continuous-rotation command in `[-1, 1]`
- `lower_deg`, `middle_deg`, `upper_deg`: pitch joint output angles in degrees
- `gripper_open`: normalized gripper openness in `[0, 1]`

## Safety notes

- The arm does not move unless the hand is confidently detected inside the active box and teleop is unfrozen.
- Tracking loss stops the base and holds the last pitch pose.
- The firmware watchdog stops the base if packets stop arriving.
- Both Python and the active firmware target clamp commands before motion.
- Start with small workspace limits and conservative base speed until the mechanism is tuned.
- The PCA9685 servo rail still needs external servo power and a common ground with the Arduino.

## Future upgrades

- Add base encoder feedback and upgrade to full 3D Cartesian IK.
- Add table calibration and workspace visualization in robot coordinates.
- Add logging and replay for tuning hand mappings.
- Add an overhead camera or multi-camera fusion for better depth and pick/place precision.
- Add optional assisted grasp modes after manual teleop is stable.



If you accidentally staged `wifi_secrets.h`, remove it before committing:

```bash
git rm --cached arduino/esp32/vision_arm_esp/include/wifi_secrets.h
```
