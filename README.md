# Navigational Sunglasses (Raspberry Pi 4, Python)

This repository implements the “navigational sunglasses” runtime for Raspberry Pi OS using:
- 3× VL53L1X ToF sensors on a shared I2C bus with XSHUT-based re-addressing
- 3× vibration motors driven via NPN transistors
- 3× momentary buttons (to GND, internal pull-ups)
- Optional Neo-6M GPS over UART (non-blocking if absent)
- USB speaker audio via ALSA (headless)

No compass. No camera. No GUI.

## Hardware traceability (prompt ↔ wiring guide)

The runtime enforces fixed BCM pin assignments to match the wiring guide’s GPIO table and XSHUT section:

| Function | BCM GPIO | Wiring guide section |
|---|---:|---|
| I2C SDA | 2 | “GPIO Pin Assignment Table” (GPIO 2) |
| I2C SCL | 3 | “GPIO Pin Assignment Table” (GPIO 3) |
| VL53L1X Left XSHUT | 5 | “VL53L1X XSHUT Wiring” |
| VL53L1X Center XSHUT | 6 | “VL53L1X XSHUT Wiring” |
| VL53L1X Right XSHUT | 13 | “VL53L1X XSHUT Wiring” |
| Motor Left (via transistor base) | 17 | “GPIO Pin Assignment Table” |
| Motor Center (via transistor base) | 27 | “GPIO Pin Assignment Table” |
| Motor Right (via transistor base) | 22 | “GPIO Pin Assignment Table” |
| Button1 (to GND, pull-up) | 23 | “GPIO Pin Assignment Table” |
| Button2 (to GND, pull-up) | 24 | “GPIO Pin Assignment Table” |
| Button3 (to GND, pull-up) | 25 | “GPIO Pin Assignment Table” |
| GPS TX → Pi RX | 15 | “Complete Wiring Connection Table” (GPS TX → GPIO15) |
| GPS RX ← Pi TX (optional) | 14 | “Complete Wiring Connection Table” (GPS RX → GPIO14) |

Power and signal requirements reflected in the code behavior:
- VL53L1X VCC is treated as 3.3V-only (do not power sensors from 5V).
- Motors are controlled as GPIO outputs intended for NPN transistor base drive (GPIO does not drive motors directly).
- Buttons are configured as GPIO inputs with internal pull-ups and are expected to short to GND when pressed.

Note: the wiring guide includes a “Compass” row in one table; this project intentionally does not use a compass, and the XSHUT mapping used here is the 3× VL53L1X mapping described in the guide’s dedicated XSHUT section.

## Repository layout

```text
.
├── main.py
├── config.yaml
├── requirements.txt
├── README.md
├── src
│   ├── __init__.py
│   ├── audio.py
│   ├── buttons.py
│   ├── config.py
│   ├── gps_logger.py
│   ├── haptics.py
│   ├── logging_setup.py
│   └── tof_manager.py
├── scripts
│   ├── gpio_selftest.py
│   └── i2c_scan.sh
└── systemd
    └── navigational-sunglasses.service
```

## Setup (Raspberry Pi OS)

1) Enable I2C and UART:
- `sudo raspi-config`
  - Interface Options → I2C → Enable
  - Interface Options → Serial Port → Enable hardware UART; disable shell over serial

2) Install OS packages:
- `sudo apt-get update`
- `sudo apt-get install -y python3-pip i2c-tools alsa-utils`

3) Install Python dependencies:
- `pip3 install -r requirements.txt`

4) Confirm USB speaker:
- `aplay -l`
- `speaker-test -c2 -t wav`

## VL53L1X multi-sensor (XSHUT + addressing)

The startup sequence implemented in [tof_manager.py](file:///d:/Client/Ashley/src/tof_manager.py) matches the wiring guide and prompt:
1) Drive all XSHUT LOW
2) Enable Left only → initialize → set I2C address to 0x30
3) Enable Center only → initialize → set I2C address to 0x31
4) Enable Right only → initialize → set I2C address to 0x32

After that, all three sensors run simultaneously at their unique addresses.

## Runtime behavior

Main loop runs at 5–10 Hz (default 8 Hz). Each cycle:
- Reads left/center/right distance in cm
- Applies median smoothing over the last N=5 samples per channel
- Applies hysteresis to reduce zone jitter (default 4 cm)
- Drives non-blocking haptic patterns per channel:
  - ≤ near threshold: continuous
  - ≤ mid threshold: fast pulses
  - ≤ far threshold: slow pulses
  - else: off

Buttons:
- Button1 toggles obstacle detection + haptics ON/OFF
- Button2 toggles audio playback ON/OFF
- Button3 cycles sensitivity preset: near → normal → far

Audible feedback tones are played for each mode change (via `aplay`).

GPS:
- Runs in a background thread
- Prefers gpsd (TCP 127.0.0.1:2947), falls back to reading NMEA over serial
- Logs `logs/gps.csv` periodically; if GPS is absent, logs a warning and continues

## Systemd service

Copy the unit file, enable, and start:

```bash
sudo mkdir -p /home/pi/navigational-sunglasses
sudo cp -r ./* /home/pi/navigational-sunglasses/
sudo cp systemd/navigational-sunglasses.service /etc/systemd/system/navigational-sunglasses.service
sudo systemctl daemon-reload
sudo systemctl enable navigational-sunglasses
sudo systemctl start navigational-sunglasses
sudo systemctl status navigational-sunglasses
```

Logs:
- App log: `logs/app.log`
- GPS CSV: `logs/gps.csv`
