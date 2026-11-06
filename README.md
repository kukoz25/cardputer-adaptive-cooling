# Cardputer Adaptive Cooling (prototype v0.2)

A smart cooling controller for indoor cycling on an **M5Stack Cardputer-Adv**.
The application reads heart rate from a BLE sensor (e.g. Polar H10) and power from a BLE trainer (e.g. Wahoo KICKR CORE), then computes a requested 3-speed fan level.

**Important:** This firmware does **not** transmit RF, and it cannot yet control the Vacmaster Cardio54. The CC1101 module is intentionally not initialized. The fan output is simulated on screen and printed over Serial. Radio commands can be implemented after obtaining and testing the original remote signals.

## Implemented features

- Scan and filter nearby Bluetooth LE devices by advertised name or MAC address.
- Choose sensors independently for HR and power, verify GATT service and subscribe to notifications.
- Remember sensor addresses in flash and attempt to reconnect after restart.
- Show live HR and power data, including connection and freshness indicators.
- Settings screen with editable resting HR, maximum HR, FTP and four fan thresholds (%HRR and %FTP).
- Calculate the five Garmin-style cycling HRR bands (34–47, 47–60, 60–73, 73–87, 87–100%) and show them on the device.
- Automatically convert %HRR into bpm and %FTP into watts. Settings are saved using ESP32 Preferences (NVS).
- Adaptive mode picks the higher fan demand from HR and power; HR-only, power-only and manual modes also exist.
- Smooth sensor data, apply threshold hysteresis, minimum switch interval and delayed downshifts.

## Hardware

- M5Stack Cardputer-Adv
- Official M5Stack Cap CC1101 (not used yet)
- Polar H10 or other standard Bluetooth Heart Rate Service sensor
- Wahoo KICKR CORE or other standard Cycling Power Service sensor

The Cardputer subscribes to `0x180D/0x2A37` for HR and `0x1818/0x2A63` for cycling power. It does not control trainer resistance; Zwift can remain the resistance controller.

## Keyboard shortcuts

| Screen | Key | Action |
| --- | --- | --- |
| Home | S | Settings |
| Home | Z | HR zones and calculated fan thresholds |
| Home | B | Bluetooth scan / sensor chooser |
| Home | T | Sensor data test |
| Home | R | Reconnect saved sensors |
| Home | A / H / P | Adaptive / HR / Power mode |
| Home | 1 / 2 / 3 | Set simulated fan level manually |
| Settings | Up / Down (Fn+; / Fn+.) | Select field |
| Settings | Enter | Edit selected field; Enter again saves |
| Settings (editing) | Digits, Backspace | Change numeric value |
| Settings (editing) | Esc | Cancel edit |
| Settings | Z | Show calculated zones |
| Browser | Tab | Switch HR / POWER sensor slot |
| Browser | Type text | Filter by name/MAC |
| Browser | Up / Down, Enter | Select and connect |
| Browser | Fn + Enter | Scan again |
| Test | Tab | Open BLE browser |
| Other screens | Esc (Fn+`) | Return Home |

Names of physical modifier keys may depend on keyboard mapping and library firmware.

## Default configuration

| Variable | Default |
| --- | ---: |
| Resting HR | 49 bpm |
| Max HR | 199 bpm |
| FTP | 228 W |
| Fan 2 threshold (HRR) | 54% = 130 bpm |
| Fan 3 threshold (HRR) | 74% = 160 bpm |
| Fan 2 threshold (FTP) | 60% = 137 W |
| Fan 3 threshold (FTP) | 90% = 206 W |

For example, with 150 bpm and 210 W, adaptive mode chooses the higher requested fan level: **3**. The displayed speed is virtual until RF is added. The HRR band boundaries come from the user's Garmin cycling profile; they are not universal training-zone defaults.

## Compile using PlatformIO

```bash
python -m pip install platformio
pio run -e cardputer_adv
```

Output: `.pio/build/cardputer_adv/firmware.bin`. Builds using the configuration in `platformio.ini`. Copy the `.bin` onto a microSD card and install it through M5Launcher. Device-side testing on Cardputer-Adv is required before declaring a build fully compatible.

## GitHub Actions

Workflow: `.github/workflows/firmware.yml`

- Push to `main` or open a PR to compile and publish a downloadable workflow artifact.
- Push a new `v*` tag (for example `v0.2.0`) to compile and attach `Cardio54-Cardputer-ADV.bin` to that GitHub Release.
- Do not reuse the existing `v0.1.0` tag for new code.

## Run arithmetic tests locally

```bash
g++ -std=c++17 -I include tests/test_cooling_math.cpp -o /tmp/cardio-math-test
/tmp/cardio-math-test
```

## Planned next steps

1. Validate the Settings screen and dual BLE operation with Zwift running.
2. Capture original Vacmaster remote control signals using Bruce and CC1101 when the fan arrives.
3. Identify protocol/modulation and check if replay is supported.
4. Implement an opt-in RF transmitter and test each button independently.

## Sources

- [Cardputer-Adv documentation](https://docs.m5stack.com/en/core/Cardputer-Adv)
- [Cap CC1101 pinout](https://docs.m5stack.com/en/cap/Cap_CC1101)
