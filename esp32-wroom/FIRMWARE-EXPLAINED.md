# ESP32 Sunglasses Firmware — Function-by-Function Guide

This document explains `esp32-wroom/esp32-wroom.ino` in small pieces.

## What this firmware does (high level)

1. **Obstacle detection** using 3× VL53L1X distance sensors (Left / Center / Right).
2. **Vibration warning** using a coin vibration motor, based on the *closest* detected object.
3. **Mode button** (1 button) to switch modes.
4. **GPS reading** from Neo‑6M (UART), parsing NMEA messages.
5. **Wi‑Fi + Firebase**: uploads the latest data to Firebase Realtime Database.

---

## Quick hardware mapping (important)

### I2C (shared by all 3 sensors)
- SDA → **GPIO21**
- SCL → **GPIO22**

### XSHUT (one per sensor)
- Left XSHUT → **GPIO25**
- Center XSHUT → **GPIO26**
- Right XSHUT → **GPIO27**

### Vibration motor
- Motor control → **GPIO33**
- Code is **active‑low**: `LOW = ON`, `HIGH = OFF`

### Mode button
- Button → **GPIO14**
- Uses `INPUT_PULLUP`: `HIGH = released`, `LOW = pressed`

### GPS (Neo‑6M on UART2)
- ESP32 RX2 (GPIO16) ← GPS TX
- ESP32 TX2 (GPIO17) → GPS RX (optional)

---

## Data that gets uploaded to Firebase

The firmware sends JSON to `/device` in your RTDB (`/device.json` endpoint). Fields include:

- `left_cm`, `center_cm`, `right_cm` — filtered distances (cm)
- `min_cm` — closest distance (cm)
- `pattern` — `"Off" | "Slow" | "Fast" | "Continuous"`
- `mode` — `"Navigation" | "Music" | "Both"`
- GPS:
  - `lat`, `lng`
  - `gps_valid`
  - `speed_mps` (from RMC)
  - `course_deg` (from RMC)
- `timestamp` — Firebase server timestamp

---

## Important settings (constants)

These are at the top of the file:

- `WIFI_SSID`, `WIFI_PASSWORD` — your hotspot / Wi‑Fi
- `FIREBASE_HOST` — your database base URL
- `FIREBASE_SEND_MS` — upload interval (currently 2000ms)
- `LOOP_HZ` — main loop frequency (currently 5Hz)
- Distance thresholds:
  - `THRESH_NEAR_CM` (continuous vibration)
  - `THRESH_MID_CM` (fast pulse)
  - `THRESH_FAR_CM` (slow pulse)
- `MAX_VALID_CM` — ignores “impossible” sensor values above this

---

# Function-by-function explanation

## `modeName(DeviceMode m)`
Converts the internal enum mode into a readable text:
- `Navigation`, `Music`, or `Both`

Used when sending to Firebase and printing to Serial.

---

## `vibWrite(bool on)`
Turns the vibration motor ON or OFF.

- Because the wiring is **active-low**, the motor is ON when the pin is `LOW`.
- This function hides that detail so the rest of the code can just say “on/off”.

---

## `setXshutAllLow()`
Sets **all three XSHUT pins LOW**.

What that does:
- It forces all sensors into reset (OFF).
- This is required because all sensors start with the same I2C address (0x29).

---

## `initOneSensor(VL53L1X &s, int xshutPin, uint8_t newAddr)`
Initializes **one** VL53L1X sensor.

Steps:
1. Turn that sensor ON (set its XSHUT pin `HIGH`)
2. Call `s.init()` to start the sensor
3. Change its address using `s.setAddress(newAddr)` so it won’t conflict with others
4. Configure it (long range mode, timing budget, continuous measurement)

Return value:
- `true` if initialization worked
- `false` if it failed

---

## `selfTestSensor(VL53L1X &s, const char* name)`
Reads a few measurements from a sensor and prints the result.

Goal:
- Quickly confirm the sensor is actually returning valid distances.

If it cannot get a valid reading, it prints `FAIL`.

---

## `initSensors()`
Initializes **all 3 sensors** safely using XSHUT sequencing.

Why we do this:
- All sensors boot at address `0x29`
- If all 3 are ON at the same time, I2C will conflict

What it does:
1. Create “fresh” sensor objects
2. Set XSHUT pins as outputs
3. Turn all sensors OFF (XSHUT LOW)
4. Turn on **Left**, set address `0x30`
5. Turn on **Center**, set address `0x31`
6. Turn on **Right**, set address `0x32`
7. Run a quick self-test on each

---

## `recoverI2CBus()`
Tries to fix a “stuck I2C bus” problem.

Sometimes, due to wiring noise or resets, the SDA line can get stuck LOW.
This function:
1. Stops I2C (`Wire.end()`)
2. Toggles SCL up to 9 times to “clock out” any stuck bits
3. Sends an I2C “STOP” condition
4. Restarts I2C at **100 kHz** (more stable for longer wires)

---

## `readMm(VL53L1X &s)`
Reads one distance measurement in **millimeters**.

If the sensor times out or returns invalid values (`0` or `65535`), it returns `0` to mean “invalid”.

---

## `mmToCm(uint16_t mm)`
Converts millimeters to centimeters with rounding.

Also applies a safety rule:
- If the value is more than `MAX_VALID_CM`, it returns `0` (invalid).

---

## `patternForMinDistanceCm(uint16_t minCm)`
Decides the vibration pattern based on the closest distance.

- `<= THRESH_NEAR_CM` → Continuous
- `<= THRESH_MID_CM` → Fast pulse
- `<= THRESH_FAR_CM` → Slow pulse
- Otherwise → Off

---

## `vibSetPattern(VibPattern p)`
Changes the current vibration pattern.

When the pattern changes, it resets internal timers so the pulse timing starts cleanly.

---

## `vibUpdate(uint32_t nowMs)`
Runs the vibration motor according to the current pattern.

- Off: motor OFF
- Continuous: motor ON
- Fast/Slow: uses ON/OFF timing to pulse

This is called repeatedly so the pulses keep working.

---

## `buttonPoll(uint32_t nowMs)`
Reads and debounces the button.

Important:
- The code uses `INPUT_PULLUP`
- Pressed means the pin becomes `LOW`
- It changes mode on the **release** event (LOW → HIGH)

Each press cycles:
`Navigation → Music → Both → Navigation → ...`

---

## `connectWiFi()`
Starts Wi‑Fi connection **without blocking** the sensor system.

It calls:
- `WiFi.begin(ssid, password)`

Then returns immediately (so obstacle detection keeps running).

Reconnect logic is handled in `loop()` every ~15 seconds if disconnected.

---

## GPS helpers

### `parseNmeaCoord(const char* s)`
Converts NMEA coordinate format (ddmm.mmmm) into decimal degrees.

### `parseGpgga(const char* sentence)`
Parses `$GPGGA` / `$GNGGA`.

Main purpose:
- check if GPS has a fix (quality field)
- update `lat`, `lng`, and `gps_valid`

### `parseGprmc(const char* sentence)`
Parses `$GPRMC` / `$GNRMC`.

Main purpose:
- check validity with status `A`/`V`
- update `lat`, `lng`
- update:
  - `speed_mps` (speed in meters/second)
  - `course_deg` (direction of travel)

### `gpsPoll()`
Reads bytes from the GPS serial port, builds full NMEA lines, then calls:
- `parseGpgga(...)` or `parseGprmc(...)` when a line is complete.

### `gpsSelfTest(uint32_t timeoutMs)`
At boot, waits up to a few seconds to see if *any* NMEA data arrives.

If nothing arrives, it prints:
- `GPS: FAIL (no NMEA data received)`

This does not stop obstacle detection; it only means GPS wiring or GPS fix is not ready yet.

---

## `vibPatternName(VibPattern p)`
Converts vibration pattern enum to text for Firebase (`Off/Fast/Slow/Continuous`).

---

## `sendToFirebase(leftCm, centerCm, rightCm, minCm)`
Uploads the latest data to Firebase RTDB using HTTP PUT.

Important behavior:
- If Wi‑Fi is not connected, it returns immediately (offline mode still works).
- Sends the `timestamp` using Firebase server timestamp (`.sv`).

---

## `setup()`
Runs once at power‑on.

What it initializes:
1. Serial Monitor
2. I2C at **100 kHz**
3. Motor output (OFF)
4. Button input
5. GPS UART + GPS self-test
6. Sensor init + sensor self-test
7. Wi‑Fi connection start

---

## `loop()`
Runs forever.

Main jobs:
1. Try Wi‑Fi reconnect every ~15 seconds if disconnected
2. Read sensors at `LOOP_HZ`
3. Calculate filtered Left/Center/Right distances and the minimum
4. Choose vibration pattern based on the minimum distance (only in Navigation/Both modes)
5. Update motor pulses
6. Upload to Firebase every `FIREBASE_SEND_MS`
7. Poll GPS continuously

Safety feature:
- If all sensors become invalid for a while, it tries `recoverI2CBus()` and then re-initializes sensors.

---

## Common troubleshooting tips

- If values become `0` randomly:
  - check wiring quality (long wires are sensitive)
  - keep I2C at 100kHz (already set)
  - make sure each XSHUT line is separate and solid
- If GPS always says FAIL:
  - double-check GPS TX → ESP32 RX2 (GPIO16)
  - confirm GPS has power and common GND
- If mode never changes:
  - verify button pins (GPIO14 ↔ GND) and that you used opposite legs of the tactile button
