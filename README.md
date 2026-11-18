# Cardputer Adaptive Cooling

Control a Vacmaster Cardio54 from an M5Stack Cardputer-Adv during indoor cycling or treadmill running. The firmware reads Bluetooth sensors and sends fan commands through the M5Stack Cap CC1101.

## Get started

1. Install the `Cardio54-Cardputer-ADV.bin` release with M5Launcher on a Cardputer-Adv.
2. Copy your four cleaned RAW captures to a folder named `cardio54` at the root of the microSD card. Keep these names:

   | File | Button |
   | --- | --- |
   | `cardio54_power_clean.sub` | Power |
   | `cardio54_speed1_clean.sub` | Speed I |
   | `cardio54_speed2_clean.sub` | Speed II |
   | `cardio54_speed3_clean.sub` | Speed III |

3. Attach the Cap CC1101 with its 433 MHz antenna and start the app. `RF: ready` means the four files loaded and the radio initialized. The app uses simulation if the card, a file, or the radio is unavailable. Check the serial monitor at 115200 baud for a missing or invalid file path.
4. With the fan in remote-control mode, try keys `1`, `2`, and `3` from Home. Each sends a direct speed command, which can also wake the fan from standby. The `0` key sends the original Power button. From standby, Power starts the fan on speed I.

The firmware loads 433.920 MHz OOK Flipper-format RAW files using the `FuriHalSubGhzPresetOOK650Async` header. Your captures live on the SD card and are ignored by Git. Power is a toggle. The display shows the app's target because the fan provides no state feedback. If you use the original remote during a session, select a speed on the Cardputer to sync its target before relying on auto-off.

## Activity profiles

Press `M` on Home to switch profiles. The Cardputer remembers your selection.

| Profile | Sensors | Cooling |
| --- | --- | --- |
| Indoor Cycling | BLE heart rate and cycling power, such as Polar H10 and KICKR CORE | Adaptive uses the higher fan demand from heart rate and power. HR-only and Power-only are also available. |
| Treadmill Running | BLE heart rate, such as Polar H10 | Speeds follow running HR thresholds. Configure running max HR and thresholds in this profile. |

The app only subscribes to measurements. Zwift can continue controlling trainer resistance. Treadmill running uses heart rate. Pace, WalkingPad, and QZ inputs are currently unavailable.

### Automatic speed reductions

Open Settings with `S` and edit `Downshift delay` to change how long a lower fan speed must be requested before the fan slows down. The setting is shared by both activity profiles and defaults to 20 seconds. Enter 0 to skip this delay. Speed reductions still respect the 6-second interval between command attempts.

### Auto-off during cycling

In automatic cycling modes, the app turns off a fan it has started once the trainer has reported a continuous **20 seconds at 0 W**. A gap or disconnect does not count as zero power. Start pedaling again to resume cooling. The timer is inactive in manual control and treadmill running.

Set `Stop at 0 W` in Cycling Settings to change the delay. Enter `0` to disable it. A longer delay can keep the fan running during stationary recovery intervals.

## Keyboard

| Screen | Key | Action |
| --- | --- | --- |
| Home | `M` | Switch between cycling and treadmill |
| Home | `A` / `H` / `P` | Select Adaptive, HR-only, or Power-only. Treadmill uses HR. |
| Home | `1` / `2` / `3` | Select a speed and enter manual mode |
| Home | `0` | Send the Power button and enter manual mode |
| Home | `B` / `T` / `R` | Scan for sensors, see live readings, or reconnect saved sensors |
| Home | `S` / `Z` | Open settings or view HRR zones and fan thresholds |
| BLE browser | Type, then Up/Down and Enter | Filter by name or address, select, and connect |
| BLE browser | Tab | Switch between the HR and power slot in cycling |
| BLE browser | Fn + Enter | Scan again |
| Settings | Up/Down and Enter | Select a field and edit. Type the new value and press Enter to save. |
| Other screens | Esc | Return Home. During editing, Esc cancels the edit. |

Use Fn + `;` and Fn + `.` for Up and Down. Set resting HR once for both profiles. Each profile has its own max HR and HRR fan thresholds. Cycling also stores FTP and the two power thresholds. The starting thresholds are 54% and 74% HRR, and 60% and 90% FTP.

## Build

```bash
python -m pip install platformio
pio run -e cardputer_adv
```

PlatformIO writes the application image to `.pio/build/cardputer_adv/firmware.bin`. GitHub Actions builds the firmware on pushes to `main` and pull requests. Pushing a new `v*` tag attaches `Cardio54-Cardputer-ADV.bin` to the GitHub Release.

Run the host tests with:

```bash
g++ -std=c++17 -Wall -Wextra -I include tests/test_cooling_math.cpp -o /tmp/cardio-test
/tmp/cardio-test
```

Hardware references: [Cardputer-Adv](https://docs.m5stack.com/en/core/Cardputer-Adv), [Cap CC1101](https://docs.m5stack.com/en/cap/Cap_CC1101).
