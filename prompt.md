
**Use this file as context:**  
#Navigational-Sunglasses-Wiring-Guide-UPDATED-NO-COMPASS-XSHUT.md

**Role**  
You are the @Builder / @Agent. Create a complete Raspberry Pi 4 Model B (Raspberry Pi OS) Python project for “navigational sunglasses” based on the wiring guide above. Use BCM numbering only.

**Hard constraints (must match wiring guide exactly)**  
I2C:
- SDA GPIO2, SCL GPIO3

3× VL53L1X:
- Left XSHUT GPIO5
- Center XSHUT GPIO6
- Right XSHUT GPIO13
Shared SDA/SCL/3.3V/GND.

Vibration motors (via NPN transistor drivers):
- Left motor GPIO17
- Center motor GPIO27
- Right motor GPIO22

Buttons (momentary to GND, use internal pull-ups):
- Button1 GPIO23
- Button2 GPIO24
- Button3 GPIO25

GPS Neo-6M UART:
- GPS TX → Pi RX GPIO15
- GPS RX → Pi TX GPIO14 (optional)
Do not block the main loop if GPS is absent.

Audio:
- USB speaker via ALSA (headless).

No compass. No camera. No GUI.

***

# Build plan (do in order, don’t skip)
## Step A — Scaffold the repository
Create this structure:

- `main.py`
- `requirements.txt`
- `config.yaml`
- `README.md`
- `systemd/navigational-sunglasses.service`
- `src/__init__.py`
- `src/config.py`
- `src/logging_setup.py`
- `src/tof_manager.py` (XSHUT + unique addresses + reading)
- `src/haptics.py` (vibration patterns)
- `src/buttons.py` (debounce + events)
- `src/audio.py` (play/stop + mode tones)
- `src/gps_logger.py` (thread + CSV logger)
- `scripts/i2c_scan.sh`
- `scripts/gpio_selftest.py` (motors + buttons quick test)

Then fill each file with complete working code.

## Step B — VL53L1X multi-sensor must work
Implement XSHUT boot + addressing sequence:
1) Set all XSHUT LOW
2) Enable Left only → init → set I2C addr 0x30
3) Enable Center only → init → set I2C addr 0x31
4) Enable Right only → init → set I2C addr 0x32
Then open/read all three by their addresses.

If library support is weak, implement the minimum required init + address change pattern using a known VL53L1X python driver, but keep code clean and documented in README.

## Step C — Runtime behavior
Main loop at 5–10 Hz:
- Read distances (cm) from left/center/right
- Apply smoothing (median last N=5) + hysteresis to reduce jitter
- Trigger vibration:
  - ≤ 20 cm: continuous
  - ≤ 50 cm: fast pulses
  - ≤ 100 cm: slow pulses
  - else: off
Patterns must be non-blocking (do not freeze sensor reads).

## Step D — Buttons / modes
- Button1 toggles obstacle detection + haptics ON/OFF
- Button2 toggles audio playback ON/OFF
- Button3 cycles sensitivity preset: `near`, `normal`, `far` (adjust thresholds live)
Add audible feedback tones for each mode change (simple tone is OK, prefer reliability).

## Step E — GPS logging
Run GPS reading in background thread:
- Prefer gpsd; fallback to serial if gpsd not running
- Every N seconds (configurable), append to `logs/gps.csv`: timestamp, lat, lon, speed (if available)
If GPS unavailable, log a warning and keep the program running.

## Step F — Ops / reliability
- YAML config with validation + defaults
- Rotating file logs in `logs/app.log`
- Clean shutdown on SIGINT/SIGTERM (stop motors, stop audio, stop threads, cleanup GPIO)
- Add a `simulation_mode` config to run without sensors (fake distances per channel)
- Provide systemd service that runs on boot (with WorkingDirectory, Restart=on-failure)

***

# Commands you should ask me to run (and then adapt code based on results)
After generating files, prompt me to run these commands (in TRAE terminal), and wait for results:
1) `sudo raspi-config` steps summary (I2C enable, Serial/UART enable)
2) `bash scripts/i2c_scan.sh`
3) `python3 scripts/gpio_selftest.py`
4) `python3 main.py --once` (single-cycle test mode)

If any command output shows errors, debug and propose patches in-place.

***

# Output rules
- Print the file tree first.
- Then output full content for every created file.
- Don’t leave placeholders like “TODO”.
- Ensure pin numbers are BCM and exactly as listed.

Now implement the entire project.



