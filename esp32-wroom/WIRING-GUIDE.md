# ESP32‑WROOM Wiring Guide (3× VL53L1X + GPS + 1 Vibration Motor)

This wiring guide matches the pin assignments in `esp32-wroom.ino`.

## Parts
Only use the materials you listed:
- 1× ESP32‑WROOM dev board (ex: ESP32 DevKit V1 / WROOM‑32)
- 3× VL53L1X ToF sensor breakout boards (each must expose **SDA/SCL/XSHUT/VCC/GND**)
- 1× Neo‑6M GPS module (UART)
- 1× micro small size 0720 flat coin vibration motor
- 1× TP4506 charging module
- 1× 7.4V 5000mah battery (red/black wires)
- 1× Buck convert HW-688
- Jumper wires

## Power Rules (Important)
- ESP32 GPIO is **3.3V only**. Do not feed 5V into ESP32 pins.
- Power **VL53L1X sensors at 3.3V** (recommended).
- GPS modules vary: many accept 3.3–5V on VCC, but confirm your exact Neo‑6M board.
- Driving a vibration motor straight from an ESP32 GPIO (jumper wire only) is **high risk**:
  - the motor can draw more current than a GPIO can supply
  - motor electrical noise (“kick”) can reset or damage the ESP32
  - it may work briefly, but it is not a safe or reliable wiring method

## Power Wiring (Battery → Charger → Buck → ESP32)
You have a 7.4V battery, so you must step down for the ESP32.

1) Battery → TP4506:
   - Battery red → TP4506 `B+`
   - Battery black → TP4506 `B-`
2) TP4506 → HW-688 input:
   - TP4506 `OUT+` → HW-688 `IN+`
   - TP4506 `OUT-` → HW-688 `IN-`
3) HW-688 → ESP32 power:
   - Adjust HW-688 output to **5V**
   - HW-688 `OUT+` → ESP32 `5V` / `VIN`
   - HW-688 `OUT-` → ESP32 `GND`

## Pin Assignment Table (ESP32)

### Shared I2C Bus (all 3 VL53L1X share these)
| Function | ESP32 GPIO | Connects to |
|---|---:|---|
| I2C SDA | 21 | VL53L1X SDA (all sensors) |
| I2C SCL | 22 | VL53L1X SCL (all sensors) |
| 3.3V | 3V3 | VL53L1X VCC (all sensors) |
| GND | GND | VL53L1X GND (all sensors) |

### VL53L1X XSHUT (one per sensor)
| Sensor | ESP32 GPIO | Connects to |
|---|---:|---|
| Left XSHUT | 25 | VL53L1X Left XSHUT |
| Center XSHUT | 26 | VL53L1X Center XSHUT |
| Right XSHUT | 27 | VL53L1X Right XSHUT |

### Vibration Motor (single motor output)
| Function | ESP32 GPIO | Connects to |
|---|---:|---|
| Motor control | 33 | Motor (–) lead (direct, jumper-wire only) |

### GPS (Neo‑6M UART)
| Function | ESP32 GPIO | Connects to |
|---|---:|---|
| GPS TX → ESP32 RX | 16 | GPS TX |
| GPS RX ← ESP32 TX (optional) | 17 | GPS RX |
| GPS power | 3V3 (recommended) | GPS VCC |
| GPS ground | GND | GPS GND |

### Mode Button (1 button)
| Function | ESP32 GPIO | Connects to |
|---|---:|---|
| Button | 4 | One leg of button |
| GND | GND | Other leg of button |

## Step‑by‑Step Wiring

### Step 1 — Wire the 3 VL53L1X sensors (shared I2C)
1) Connect **all sensor VCC** to ESP32 **3V3**.  
2) Connect **all sensor GND** to ESP32 **GND** (common ground).  
3) Connect **all sensor SDA** to ESP32 **GPIO21**.  
4) Connect **all sensor SCL** to ESP32 **GPIO22**.  
5) Connect each sensor’s **XSHUT** to its dedicated pin:
   - Left XSHUT → GPIO25
   - Center XSHUT → GPIO26
   - Right XSHUT → GPIO27

Why XSHUT matters: all VL53L1X sensors share the same default I2C address at power‑up, so the firmware holds them in reset and brings them up one‑by‑one to assign unique addresses (0x30/0x31/0x32).

### Step 2 — Wire the GPS module (UART)
1) GPS VCC → ESP32 3V3  
2) GPS GND → ESP32 GND  
3) GPS TX → ESP32 GPIO16 (RX)  
4) (Optional) GPS RX → ESP32 GPIO17 (TX)

### Step 3 — Wire the 0720 vibration motor (3V3, jumper-wire only)
If you are using only the listed materials and no driver parts, this is the simplest direct wiring:

- Motor (+) → ESP32 **3V3** (use 3.3V, not VIN/5V)
- Motor (–) → ESP32 **GPIO33**

Control note:
- Set GPIO33 **LOW** = motor ON (GPIO sinks current)
- Set GPIO33 **HIGH** = motor OFF

If the ESP32 resets when the motor turns on, or the vibration is weak, that is expected with direct wiring.

### Step 4 — Wire the mode button
1) Button one leg → ESP32 **GPIO4**
2) Button other leg → ESP32 **GND**

No external resistor needed (firmware uses internal pull-up).
Short press cycles modes: **Navigation → Music → Both → Navigation ...**

## What You Should See When It's Correct
Open Serial Monitor at **115200**:
- On boot, you should see:
  - `OK: VL53L1X sensors initialized at 0x30/0x31/0x32`
- Then you should see repeating lines like:
  - `Distances cm: left=... center=... right=... | min=...`
- GPS NMEA sentences may also print (lines starting with `$G...`) once the GPS outputs data.

## Vibration Behavior (single motor)
The firmware takes the **minimum distance** seen by any sensor:
- ≤ 20 cm: **continuous vibration**
- ≤ 50 cm: **fast pulses**
- ≤ 100 cm: **slow pulses**
- > 100 cm: **off**

If you want direction (left/center/right) feedback, you need 3 separate motors or a different feedback method (audio or patterned vibrations).

