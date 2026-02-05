# **Navigational Sunglasses Wiring Guide**

**Raspberry Pi 4 Project - Direct Wiring (No Breadboard)**

Based on Research Project - Assistive Device for Visually Impaired Individuals

---

## 🎯 Project Overview

This smart navigational sunglasses system helps visually impaired individuals navigate safely using obstacle detection and audio feedback:

- **Obstacle detection** - 3× distance sensors detect obstacles left, center, right
- **Haptic feedback** - vibration motors warn user about obstacle location
- **Audio entertainment** - radio/music playback feature
- **Mode selection** - physical buttons let user choose active features
- **GPS location** - tracks current position (optional for future expansion)

---

## 📋 Complete Parts List

### Main Components:
- **1× Raspberry Pi 4 8GB RAM** - main processing unit
- **1× 32GB SD Card** - operating system and programs
- **1× Neo-6M GPS Module with Antenna** - location tracking
- **3× VL53L1X ToF Distance Sensors** - obstacle detection (left, center, right)
- **3× Vibration Motors (coin type)** - haptic feedback
- **1× USB Mini Speaker** - audio output for radio/music
- **3× Push Buttons (momentary)** - feature mode selection
- **1× 5V 3A USB-C Power Supply** - wall power for development
- **1× TP4056 Charging Module (Type-C)** - battery charging
- **1× Li-ion Battery 3.7V 3000mAh** - portable power
- **1× 5V Boost Converter Module (3A)** - battery to Pi power

### Direct Wiring Components:
- **Female-to-female dupont wires** (assorted colors, 40-50 pieces)
- **Screw terminal blocks** (2-way and 3-way: 6-8 pieces)
- **22 AWG jumper wire** (for signal connections)
- **18 AWG wire** (for power distribution)
- **Micro HDMI to HDMI cable** - for development/testing only

### Tools Needed:
- Wire strippers
- Small screwdriver set
- Multimeter
- Electrical tape
- Heat shrink tubing (optional)

---

## 📊 Raspberry Pi GPIO Pin Assignment Table

| **Pi GPIO Pin** | **Device Connected** | **Wire Color** | **Function** |
|-----------------|----------------------|----------------|--------------|
| **5V** | 5V Power Hub (from boost converter) | Red | Main 5V power input |
| **GND** | Ground Hub (all devices) | Black | Common ground |
| **GPIO 2 (SDA)** | I2C SDA Hub (ToF sensors) | Blue | I2C data line |
| **GPIO 3 (SCL)** | I2C SCL Hub (ToF sensors) | Yellow | I2C clock line |
| **GPIO 17** | Vibration Motor Left (via transistor) | Purple | Left obstacle warning |
| **GPIO 27** | Vibration Motor Center (via transistor) | White | Center obstacle warning |
| **GPIO 22** | Vibration Motor Right (via transistor) | Orange | Right obstacle warning |
| **GPIO 23** | Button 1 (Mode Select) | Green | Feature mode button 1 |
| **GPIO 24** | Button 2 (Mode Select) | Brown | Feature mode button 2 |
| **GPIO 25** | Button 3 (Mode Select) | Gray | Feature mode button 3 |
| **GPIO 5** | VL53L1X Left XSHUT | - | Sensor enable (active-low) |
| **GPIO 6** | VL53L1X Center XSHUT | - | Sensor enable (active-low) |
| **GPIO 13** | VL53L1X Right XSHUT | - | Sensor enable (active-low) |
| **3.3V** | 3.3V Power Hub (sensors) | Red | Sensor power reference |

---

## 🔌 Complete Wiring Connection Table

| **Component** | **Component Pin** | **Pi GPIO Pin** | **Wire Color** | **Connection Method** | **Function** |
|---------------|-------------------|-----------------|----------------|----------------------|--------------|
| **VL53L1X ToF #1 (Left)** | VCC | 3.3V Hub | Red | Via 3.3V power hub | Sensor power |
| | GND | GND Hub | Black | Via ground hub | Ground |
| | SDA | GPIO 2 | Blue | Via I2C SDA hub | I2C data |
| | SCL | GPIO 3 | Yellow | Via I2C SCL hub | I2C clock |
| | XSHUT | GPIO 5 | - | Direct wire | Sensor enable (active-low) |
| **VL53L1X ToF #2 (Center)** | VCC | 3.3V Hub | Red | Via 3.3V power hub | Sensor power |
| | GND | GND Hub | Black | Via ground hub | Ground |
| | SDA | GPIO 2 | Blue | Via I2C SDA hub | I2C data |
| | SCL | GPIO 3 | Yellow | Via I2C SCL hub | I2C clock |
| | XSHUT | GPIO 6 | - | Direct wire | Sensor enable (active-low) |
| **VL53L1X ToF #3 (Right)** | VCC | 3.3V Hub | Red | Via 3.3V power hub | Sensor power |
| | GND | GND Hub | Black | Via ground hub | Ground |
| | SDA | GPIO 2 | Blue | Via I2C SDA hub | I2C data |
| | SCL | GPIO 3 | Yellow | Via I2C SCL hub | I2C clock |
| **HMC5883L Compass** | VCC | 3.3V Hub | Red | Via 3.3V power hub | Compass power |
| | GND | GND Hub | Black | Via ground hub | Ground |
| | SDA | GPIO 2 | Blue | Via I2C SDA hub | I2C data |
| | SCL | GPIO 3 | Yellow | Via I2C SCL hub | I2C clock |
| | XSHUT | GPIO 13 | - | Direct wire | Sensor enable (active-low) |
| **Neo-6M GPS** | VCC | 3.3V Hub | Red | Via 3.3V power hub | GPS power |
| | GND | GND Hub | Black | Via ground hub | Ground |
| | TX | GPIO 15 (UART RX) | White | Direct wire | GPS data to Pi |
| | RX | GPIO 14 (UART TX) | Green | Direct wire | Pi to GPS (optional) |
| **Vibration Motor Left** | Positive (+) | GPIO 17 via transistor | Purple | Via NPN transistor | Left warning |
| | Negative (-) | GND Hub | Black | Via ground hub | Ground |
| **Vibration Motor Center** | Positive (+) | GPIO 27 via transistor | White | Via NPN transistor | Center warning |
| | Negative (-) | GND Hub | Black | Via ground hub | Ground |
| **Vibration Motor Right** | Positive (+) | GPIO 22 via transistor | Orange | Via NPN transistor | Right warning |
| | Negative (-) | GND Hub | Black | Via ground hub | Ground |
| **Button 1** | Pin 1 | GPIO 23 | Green | Direct wire | Mode select button |
| | Pin 2 | GND Hub | Black | Via ground hub | Button ground |
| **Button 2** | Pin 1 | GPIO 24 | Brown | Direct wire | Mode select button |
| | Pin 2 | GND Hub | Black | Via ground hub | Button ground |
| **Button 3** | Pin 1 | GPIO 25 | Gray | Direct wire | Mode select button |
| | Pin 2 | GND Hub | Black | Via ground hub | Button ground |
| **USB Mini Speaker** | USB plug | Pi USB port | - | Direct USB connection | Audio output |
| **5V Boost Converter** | IN+ | Battery + (via TP4056) | Red | From battery positive | Battery input |
| | IN- | Battery - | Black | Battery negative | Battery ground |
| | OUT+ | Pi 5V pin | Red | To Pi power | Stable 5V output |
| | OUT- | Pi GND pin | Black | To Pi ground | Power ground |


---

## VL53L1X XSHUT Wiring (Required for 3 Sensors)

Because all VL53L1X sensors share the same default I2C address, you must use the **XSHUT** pins so you can boot and assign each sensor a unique address in software.

### GPIO assignments for XSHUT

- **Left VL53L1X XSHUT → GPIO 5**
- **Center VL53L1X XSHUT → GPIO 6**
- **Right VL53L1X XSHUT → GPIO 13**

### How to wire XSHUT

- Connect each sensor’s **XSHUT** pin to its assigned GPIO above.
- Keep **SDA (GPIO 2)** and **SCL (GPIO 3)** shared for all sensors.
- XSHUT is **active-low**: drive LOW to keep a sensor OFF, drive HIGH to turn it ON.

### Startup sequence (high level)

1. Set all three XSHUT GPIOs LOW (all sensors off)
2. Turn on Left (GPIO 5 HIGH) → initialize → change I2C address (e.g., 0x30)
3. Turn on Center (GPIO 6 HIGH) → initialize → change I2C address (e.g., 0x31)
4. Turn on Right (GPIO 13 HIGH) → initialize → change I2C address (e.g., 0x32)

---

## ⚠️ Important Notes

✅ **All I2C devices share same SDA/SCL lines** - this is normal and correct

🔴 **Vibration motors MUST use transistors** - GPIO pins can't supply enough current directly

✅ **Buttons use internal pull-up resistors** - no external resistors needed

🔴 **VL53L1X sensors need 2.6-3.5V** - connect to 3.3V rail, NEVER 5V

✅ **Common ground CRITICAL** - All components must share common ground

✅ **Boost converter required** - Pi needs stable 5V 3A from 3.7V battery

---

## 🔧 Step-by-Step Building Guide

### **Step 1: Prepare Power System**

**What you need:** TP4056 module, Li-ion battery, 5V boost converter, wires, multimeter

**Battery + Charger Setup:**
1. Connect Li-ion battery to TP4056:
   - Battery **positive (+)** → TP4056 **B+**
   - Battery **negative (-)** → TP4056 **B-**
2. Test charge with USB-C cable connected to TP4056
3. Red LED = charging, Blue LED = fully charged

**Boost Converter Setup:**
1. Connect TP4056 output to boost converter input:
   - TP4056 **OUT+** → Boost converter **IN+** (red wire)
   - TP4056 **OUT-** → Boost converter **IN-** (black wire)
2. Adjust boost converter output voltage:
   - Turn on TP4056 (battery connected)
   - Use multimeter on boost converter output
   - Adjust potentiometer until output reads **5.0-5.1V**
3. Connect boost converter to Raspberry Pi:
   - Boost **OUT+** → Pi **5V pin** (red wire)
   - Boost **OUT-** → Pi **GND pin** (black wire)

**⚠️ Critical Test:**
- Use multimeter: Boost output must read 5.0-5.1V **before** connecting to Pi
- If voltage is wrong, Pi may be damaged!

---

### **Step 2: Create Power and Ground Hubs**

**What you need:** Screw terminal blocks, wires (18 AWG for power, 22 AWG for signals)

**3.3V Power Hub (for sensors):**
1. Use 3-way screw terminal block
2. Connect Pi **3.3V pin** to terminal block
3. This hub will power:
   - 3× VL53L1X sensors
   - Neo-6M GPS

**Ground Hub (BLACK wires - MOST IMPORTANT!):**
1. Use 8-way screw terminal block OR large wire nut
2. Cut 10 black wires (each 6-8 inches long)
3. Strip ½ inch from each end
4. Connect one wire to Pi **GND pin**
5. Connect other wires to:
   - All 3 ToF sensors GND
   - Compass GND
   - GPS GND
   - All 3 vibration motors GND
   - All 3 buttons (one side)
   - Boost converter GND
6. **Test with multimeter:** Continuity between ANY two ground points should beep

---

### **Step 3: Connect I2C Sensors (3× ToF)**

**What you need:** 3× VL53L1X sensors, female dupont wires

**I2C Hub Creation:**
1. Since all I2C devices share SDA/SCL, create I2C hubs:
   - **SDA Hub:** Connect Pi GPIO 2 to all sensors' SDA pins
   - **SCL Hub:** Connect Pi GPIO 3 to all sensors' SCL pins

**VL53L1X ToF Sensor #1 (Left):**
- VCC (red) → 3.3V Hub
- GND (black) → Ground Hub
- SDA (blue) → I2C SDA Hub (GPIO 2)
- SCL (yellow) → I2C SCL Hub (GPIO 3)

**VL53L1X ToF Sensor #2 (Center):**
- VCC (red) → 3.3V Hub
- GND (black) → Ground Hub
- SDA (blue) → I2C SDA Hub (GPIO 2)
- SCL (yellow) → I2C SCL Hub (GPIO 3)

**VL53L1X ToF Sensor #3 (Right):**
- VCC (red) → 3.3V Hub
- GND (black) → Ground Hub
- SDA (blue) → I2C SDA Hub (GPIO 2)
- SCL (yellow) → I2C SCL Hub (GPIO 3)

**⚠️ Note:** VL53L1X sensors have **same I2C address by default**. Use XSHUT pins so you can enable sensors one-by-one and assign unique I2C addresses.

**XSHUT wiring for this build:**
- Left sensor XSHUT → **GPIO 5**
- Center sensor XSHUT → **GPIO 6**
- Right sensor XSHUT → **GPIO 13**

**Test:** Upload I2C scanner code - should detect 3× VL53L1X + compass addresses

---

### **Step 4: Connect GPS Module**

**What you need:** Neo-6M GPS module, antenna, wires

**GPS Wiring:**
- GPS **VCC** → 3.3V Hub (red wire)
- GPS **GND** → Ground Hub (black wire)
- GPS **TX** → Pi **GPIO 15 (UART RX)** (white wire)
- GPS **RX** → Pi **GPIO 14 (UART TX)** (green wire) - optional

**GPS Antenna:**
1. Connect GPS antenna to Neo-6M module connector
2. Place antenna with clear view of sky (for testing)
3. LED on GPS module should blink when searching for satellites

**Test:** Use `gpsd` or `gps3` Python library to read NMEA sentences

---

### **Step 5: Connect Vibration Motors with Transistors**

**What you need:** 3× vibration motors, 3× NPN transistors (2N2222 or BC547), wires

**Why transistors?** Vibration motors draw 50-100mA; Pi GPIO can only safely provide 16mA

**Transistor Circuit for EACH Motor:**

```
Pi GPIO pin → Transistor BASE (middle leg)
Motor (+) → 3.3V Hub
Motor (-) → Transistor COLLECTOR (left leg)
Transistor EMITTER (right leg) → Ground Hub
```

**Vibration Motor Left:**
1. Connect: GPIO 17 → Transistor Q1 BASE
2. Motor (+) purple wire → 3.3V Hub
3. Motor (-) → Q1 COLLECTOR
4. Q1 EMITTER → Ground Hub

**Vibration Motor Center:**
1. Connect: GPIO 27 → Transistor Q2 BASE
2. Motor (+) white wire → 3.3V Hub
3. Motor (-) → Q2 COLLECTOR
4. Q2 EMITTER → Ground Hub

**Vibration Motor Right:**
1. Connect: GPIO 22 → Transistor Q3 BASE
2. Motor (+) orange wire → 3.3V Hub
3. Motor (-) → Q3 COLLECTOR
4. Q3 EMITTER → Ground Hub

**Test:** Set GPIO pin HIGH - motor should vibrate

---

### **Step 6: Connect Mode Selection Buttons**

**What you need:** 3× momentary push buttons, wires

**Button Wiring (Using Internal Pull-Up Resistors):**

Each button connects:
- One side → Pi GPIO pin
- Other side → Ground Hub

**Button 1 (Mode Select):**
- Pin 1 (green wire) → Pi **GPIO 23**
- Pin 2 (black wire) → Ground Hub

**Button 2 (Mode Select):**
- Pin 1 (brown wire) → Pi **GPIO 24**
- Pin 2 (black wire) → Ground Hub

**Button 3 (Mode Select):**
- Pin 1 (gray wire) → Pi **GPIO 25**
- Pin 2 (black wire) → Ground Hub

**Software Setup:** Enable internal pull-up resistors in code:
```python
GPIO.setup(23, GPIO.IN, pull_up_down=GPIO.PUD_UP)
```

**Button reads:**
- Not pressed = HIGH (3.3V)
- Pressed = LOW (0V to ground)

**Test:** Read button state - should change when pressed

---

### **Step 7: Connect USB Mini Speaker**

**What you need:** USB mini speaker with built-in sound card

**Simple Connection:**
1. Plug USB speaker directly into any Pi USB port
2. Pi will auto-detect as audio output device

**Software Setup:**
```bash
# List audio devices
aplay -l

# Set USB speaker as default
sudo nano /etc/asound.conf
# Add: defaults.pcm.card 1
```

**Test:** Play test audio file - sound should come from speaker

---

## ✅ Testing Your Wiring

### **Test 1: Visual Inspection**

- [ ] All wires securely connected (no loose dupont connectors)
- [ ] 3.3V sensors connected to 3.3V hub (NOT 5V!)
- [ ] All grounds connected to common ground hub
- [ ] Vibration motors use transistors (not direct GPIO)
- [ ] Boost converter output adjusted to 5.0-5.1V
- [ ] No short circuits between power rails
- [ ] Battery properly connected to TP4056

### **Test 2: Power Check**

1. Turn on battery (via TP4056 switch if available)
2. Use multimeter to check voltages:
   - Boost converter output: **5.0-5.1V**
   - Pi 3.3V pin: **3.3V**
   - 3.3V Hub: **3.3V**
3. Check ground continuity:
   - Test between Pi GND and boost GND: should beep (0 ohms)
   - Test between any two component GND pins: should beep

### **Test 3: Raspberry Pi Boot Test**

1. Insert SD card with Raspberry Pi OS
2. Connect HDMI cable to monitor (for testing)
3. Power on system
4. Pi should boot normally (green LED activity)
5. Login and open terminal

### **Test 4: I2C Device Detection**

```bash
# Enable I2C interface
sudo raspi-config
# Interface Options → I2C → Enable

# Install I2C tools
sudo apt-get install i2c-tools

# Scan for I2C devices
sudo i2cdetect -y 1
```

**Expected result:** Should show VL53L1X address(es) (often 0x29). If you re-address sensors, you should see multiple addresses.

### **Test 5: GPS Module Test**

```bash
# Install GPS tools
sudo apt-get install gpsd gpsd-clients

# Test GPS data
cgps -s
```

**Expected result:** Should show latitude/longitude after acquiring satellite lock

### **Test 6: Vibration Motor Test**

```python
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)  # Left motor

# Vibrate for 1 second
GPIO.output(17, GPIO.HIGH)
time.sleep(1)
GPIO.output(17, GPIO.LOW)

GPIO.cleanup()
```

**Expected result:** Left vibration motor should vibrate for 1 second

### **Test 7: Button Test**

```python
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.IN, pull_up_down=GPIO.PUD_UP)

print("Press button 1...")
while True:
    if GPIO.input(23) == GPIO.LOW:
        print("Button pressed!")
        break
```

**Expected result:** Message prints when button 1 pressed

### **Test 8: USB Speaker Test**

```bash
# Play test sound
speaker-test -c2 -t wav
```

**Expected result:** Should hear test tones from USB speaker

---

## 🚨 Troubleshooting Guide

### **Problem: Pi Won't Power On**

**Symptoms:**
- No red LED on Pi
- No boot activity

**Check these:**
- Boost converter output voltage = 5.0-5.1V
- Boost converter connected to correct Pi pins (5V + GND)
- Battery charged (check TP4056 LED)
- Common ground between boost converter and Pi

**Solution:**
1. Test boost converter output with multimeter
2. Try powering Pi with official 5V 3A adapter first
3. Check for short circuits

---

### **Problem: I2C Devices Not Detected**

**Symptoms:**
- `i2cdetect` shows no devices
- Sensors don't respond

**Check these:**
- I2C enabled in `raspi-config`
- All sensors connected to 3.3V (NOT 5V!)
- SDA connected to GPIO 2
- SCL connected to GPIO 3
- Common ground

**Solution:**
1. Test one sensor at a time
2. Check wire connections with multimeter continuity
3. Verify sensor power (should be 3.3V)

---

### **Problem: VL53L1X Sensors Conflict**

**Symptoms:**
- Only one sensor detected
- All three show same address (0x29)

**Solution:**
Use XSHUT pins to set different addresses:

```python
# Disable all sensors first
GPIO.setup(4, GPIO.OUT)   # XSHUT for sensor 1
GPIO.setup(5, GPIO.OUT)   # XSHUT for sensor 2
GPIO.setup(6, GPIO.OUT)   # XSHUT for sensor 3

GPIO.output(4, GPIO.LOW)
GPIO.output(5, GPIO.LOW)
GPIO.output(6, GPIO.LOW)

# Enable and change address one by one
GPIO.output(4, GPIO.HIGH)
sensor1.set_address(0x30)

GPIO.output(5, GPIO.HIGH)
sensor2.set_address(0x31)

GPIO.output(6, GPIO.HIGH)
sensor3.set_address(0x32)
```

---

### **Problem: Vibration Motors Not Working**

**Symptoms:**
- No vibration when GPIO set HIGH
- Weak vibration

**Check these:**
- Transistors connected correctly (BASE, COLLECTOR, EMITTER)
- Motor (+) connected to 3.3V
- Motor (-) connected to transistor COLLECTOR
- Transistor EMITTER connected to ground

**Solution:**
1. Test motor directly with 3.3V (briefly)
2. Check transistor with multimeter
3. Verify GPIO pin outputs 3.3V when HIGH

---

### **Problem: GPS Not Getting Fix**

**Symptoms:**
- GPS shows 0 satellites
- No latitude/longitude data

**Check these:**
- GPS antenna connected
- Antenna has clear view of sky
- GPS module powered (3.3V)
- Wait 2-5 minutes for initial fix

**Solution:**
1. Move to outdoor location with clear sky view
2. Check GPS LED (should blink when searching)
3. Verify UART connection (TX → RX, RX → TX)

---

### **Problem: Buttons Not Responding**

**Symptoms:**
- Button press not detected
- Always reads same state

**Check these:**
- Internal pull-up enabled in code
- Button connected between GPIO and GND
- Not using external pull-down resistor

**Solution:**
1. Test button with multimeter continuity
2. Swap to different GPIO pin
3. Check for loose wire connections

---

### **Problem: USB Speaker No Sound**

**Symptoms:**
- No audio output
- System doesn't detect speaker

**Check these:**
- Speaker plugged into USB port
- Speaker appears in `aplay -l`
- Correct audio device selected

**Solution:**
```bash
# Force audio to USB
sudo raspi-config
# System Options → Audio → USB

# Test with speaker-test
speaker-test -c2 -t wav
```

---

## 📊 Step-by-Step Testing Process

**Test Modules One by One (IMPORTANT: Follow this exact order!)**

### **1. Power Test (FIRST!)**
- [ ] Battery charged via TP4056
- [ ] Boost converter outputs 5.0-5.1V
- [ ] Pi 3.3V rail measures ~3.3V
- [ ] All grounds connected to common hub
- [ ] No smoke, sparks, or burning smell

### **2. Raspberry Pi Boot Test**
- [ ] Pi boots successfully
- [ ] Can login via HDMI/SSH
- [ ] Green activity LED flashes

### **3. I2C Sensor Test**
- [ ] I2C enabled in config
- [ ] `i2cdetect` shows sensor addresses
- [ ] Can read distance from each ToF sensor
- [ ] Compass shows heading values

### **4. GPS Module Test**
- [ ] GPS UART enabled
- [ ] `cgps` shows satellite data
- [ ] Latitude/longitude updates
- [ ] Time synchronized

### **5. Vibration Motor Test**
- [ ] Left motor vibrates on command
- [ ] Center motor vibrates on command
- [ ] Right motor vibrates on command
- [ ] Vibration strength adequate
- [ ] No overheating after 1 minute continuous

### **6. Button Test**
- [ ] Button 1 press detected
- [ ] Button 2 press detected
- [ ] Button 3 press detected
- [ ] No false triggers
- [ ] Buttons respond reliably

### **7. USB Speaker Test**
- [ ] Speaker detected by system
- [ ] Test audio plays correctly
- [ ] Volume adjustable
- [ ] No distortion

### **8. Full System Integration Test**
- [ ] All sensors read simultaneously
- [ ] Obstacle detected → correct motor vibrates
- [ ] Buttons switch modes correctly
- [ ] Audio plays while sensors active
- [ ] System runs for 30 minutes without errors
- [ ] Battery life acceptable

---

## 💡 System Optimization Tips

### **For Better Obstacle Detection:**
1. Position sensors at 45° angles (left-forward, center, right-forward)
2. Adjust detection thresholds (20cm = near, 50cm = medium, 100cm+ = far)
3. Different vibration patterns for distance:
   - Continuous = very close
   - Fast pulses = close
   - Slow pulses = medium distance

### **For Better Battery Life:**
1. Reduce sensor polling rate (5-10 Hz instead of continuous)
2. Power down GPS when not needed
3. Use sleep modes between detections
4. Lower CPU frequency: `sudo raspi-config` → Performance Options

### **For Better User Experience:**
1. Different vibration patterns for left/center/right
2. Audio alerts for mode changes
3. Battery level warnings via vibration
4. Calibration routine on startup

---

## 🎓 Project Credits

**Project:** Navigational Sunglasses for Visually Impaired  
**Microcontroller:** Raspberry Pi 4 8GB RAM  
**Assembly:** Direct wiring without breadboard  
**Features:** Obstacle detection, haptic feedback, audio output, mode selection, GPS location  

**Final Assembly Notes:**
- All wiring completed without breadboard
- Power distributed via boost converter with terminal blocks
- Common ground critical for proper operation
- VL53L1X requires 3.3V only (never 5V)
- Vibration motors require transistors (not direct GPIO)
- Test thoroughly before final enclosure assembly

**Good luck with your Navigational Sunglasses project!** 👓🦯✨

---

**Document prepared for:** Ashley Jioson and Team  
**Contract reference:** Service Agreement dated February 1, 2026  
**Programmer:** Jhon Isaac Alegre  
**Target completion:** February 13, 2026