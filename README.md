# Cardputer Adaptive Cooling

An open-source smart fan controller for indoor cycling, built for the M5Stack Cardputer-Adv.

The project automatically adjusts fan speed based on heart rate and cycling power, using a control algorithm inspired by the Wahoo KICKR HEADWIND.

## Features

- Bluetooth Low Energy (BLE) support for heart rate and power sensors
- Automatic Bluetooth device discovery and selection
- Manual device name search using the Cardputer keyboard
- Support for simultaneous heart rate and power monitoring
- Automatic heart rate zone calculation using resting and maximum heart rate
- Power zone calculation based on Functional Threshold Power (FTP)
- Adaptive cooling based on both heart rate and cycling power
- Three fan speed levels
- Manual and automatic operating modes
- Configurable thresholds and user settings
- Persistent device and user configuration
- CC1101 radio module support for fan control
- Compatibility with M5Launcher
- Automated firmware builds using GitHub Actions

Some features are planned or under development. The current prototype focuses on BLE connectivity and sensor monitoring.

## Hardware

The project is designed for the following hardware:

| Component | Device |
|---|---|
| Controller | M5Stack Cardputer-Adv |
| Radio module | Official M5Stack Cap CC1101 |
| Heart rate sensor | Polar H10 |
| Smart trainer | Wahoo KICKR CORE |
| Fan | Vacmaster Cardio54 |

Other BLE heart rate sensors and power meters may also work if they support standard Bluetooth services.

## How It Works

The Cardputer connects to a heart rate monitor and a cycling power meter over Bluetooth.

It continuously reads both measurements and calculates the required cooling intensity.

The fan speed is determined by whichever measurement requires more cooling.

For example:

- Low heart rate and low power: Fan speed 1
- Moderate heart rate or power: Fan speed 2
- High heart rate or power: Fan speed 3

This allows the fan to respond quickly to power increases during intervals while maintaining airflow when heart rate remains elevated during recovery.

The controller sends wireless commands to the fan using the CC1101 radio module.

RF transmission requires capturing and validating the original fan remote control signals. This functionality is not yet implemented in the prototype.

## User Configuration

Users can configure their own physiological parameters directly on the Cardputer:

- Resting heart rate (bpm)
- Maximum heart rate (bpm)
- Functional Threshold Power (FTP, watts)

The application will automatically calculate heart rate reserve (HRR), heart rate zones, and power-based cooling thresholds.

All thresholds can be adjusted manually.

## Bluetooth Connectivity

The application supports scanning for nearby Bluetooth devices, filtering devices by name, selecting sensors using the Cardputer keyboard, and saving device selections.

Supported Bluetooth services include:

- Heart Rate Service (0x180D)
- Cycling Power Service (0x1818)

The controller only reads data from the smart trainer. It does not control resistance or interfere with applications such as Zwift.

## Building the Firmware

The project uses PlatformIO and the Arduino framework.

Requirements:

- Visual Studio Code with the PlatformIO extension, or PlatformIO CLI
- ESP32-S3 development platform
- M5Cardputer library
- NimBLE-Arduino library

To build the firmware:

```bash
pio run -e cardputer_adv
```

The compiled firmware is generated at:

```text
.pio/build/cardputer_adv/firmware.bin
```

## Installing with M5Launcher

1. Download the latest firmware from GitHub Releases.
2. Copy the `.bin` file to the Cardputer microSD card.
3. Open M5Launcher.
4. Select the firmware file from the SD card.
5. Install and launch the application.

Firmware compatibility with M5Launcher must be verified on the Cardputer-Adv hardware.

## GitHub Actions

The repository includes an automated build pipeline.

Every push and pull request triggers a firmware compilation.

Successful builds produce a downloadable `.bin` artifact.

Pushing a version tag such as `v0.1.0` automatically creates a GitHub Release containing the compiled firmware.

## Development Status

The project is currently under development.

The initial prototype provides BLE sensor discovery, connection management, and heart rate and power monitoring.

Planned development includes:

- User-editable HR and FTP settings
- Automatic zone calculation
- Improved adaptive cooling with hysteresis and signal filtering
- CC1101 remote signal playback
- Complete fan control
- Improved user interface and diagnostics
- Hardware testing and validation

## License

License to be determined.
