#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <WiFi.h>
#include <HTTPClient.h>

// ── WiFi credentials (change these) ──
static const char* WIFI_SSID     = "Infinix GT 30 Pro";
static const char* WIFI_PASSWORD = "pangitniaya";

// ── Firebase Realtime Database ──
static const char* FIREBASE_HOST = "https://sunglass-94003-default-rtdb.asia-southeast1.firebasedatabase.app";
static constexpr uint32_t FIREBASE_SEND_MS = 2000;

// ── VL53L1X I2C addresses ──
static constexpr uint8_t ADDR_LEFT   = 0x30;
static constexpr uint8_t ADDR_CENTER = 0x31;
static constexpr uint8_t ADDR_RIGHT  = 0x32;

// ── Pin assignments ──
static constexpr int I2C_SDA_PIN      = 21;
static constexpr int I2C_SCL_PIN      = 22;
static constexpr int XSHUT_LEFT_PIN   = 25;
static constexpr int XSHUT_CENTER_PIN = 26;
static constexpr int XSHUT_RIGHT_PIN  = 27;
static constexpr int VIB_MOTOR_PIN    = 33;
static constexpr bool VIB_ACTIVE_LOW  = true;
static constexpr int BUTTON_PIN       = 14;

// ── GPS (UART2) ──
static constexpr int GPS_UART_NUM  = 2;
static constexpr int GPS_RX_PIN    = 16;
static constexpr int GPS_TX_PIN    = 17;
static constexpr uint32_t GPS_BAUD = 9600;

// ── Timing ──
static constexpr uint32_t LOOP_HZ    = 5;            // 5 Hz — allows 3 blocking reads per tick
static constexpr uint32_t LOOP_DT_MS = 1000 / LOOP_HZ;

// ── Distance thresholds (cm) ──
static constexpr int THRESH_NEAR_CM = 20;
static constexpr int THRESH_MID_CM  = 50;
static constexpr int THRESH_FAR_CM  = 100;
static constexpr uint16_t MAX_VALID_CM = 400;

// ── Median filter ──
static constexpr uint8_t MEDIAN_WINDOW = 5;

struct MedianFilter {
  uint16_t buf[MEDIAN_WINDOW] = {0};
  uint8_t n = 0;
  uint8_t idx = 0;
  uint8_t missCount = 0;

  void push(uint16_t v) {
    buf[idx] = v;
    idx = (idx + 1) % MEDIAN_WINDOW;
    if (n < MEDIAN_WINDOW) n++;
    missCount = 0;
  }

  void miss() {
    missCount++;
    if (missCount >= MEDIAN_WINDOW) {
      n = 0;
      idx = 0;
      missCount = 0;
    }
  }

  uint16_t median() const {
    if (n == 0) return 0;
    uint16_t tmp[MEDIAN_WINDOW];
    for (uint8_t i = 0; i < n; i++) tmp[i] = buf[i];
    for (uint8_t i = 0; i < n; i++) {
      for (uint8_t j = i + 1; j < n; j++) {
        if (tmp[j] < tmp[i]) {
          uint16_t t = tmp[i];
          tmp[i] = tmp[j];
          tmp[j] = t;
        }
      }
    }
    return tmp[n / 2];
  }
};

// ── Vibration ──
enum class VibPattern : uint8_t { Off = 0, Slow = 1, Fast = 2, Continuous = 3 };

struct VibState {
  VibPattern pattern = VibPattern::Off;
  bool on = false;
  uint32_t nextToggleAtMs = 0;
};

// ── GPS parsed data ──
struct GpsData {
  float lat = 0;
  float lng = 0;
  bool valid = false;
};

// ── Device modes ──
enum class DeviceMode : uint8_t { Navigation = 0, Music = 1, Both = 2 };
static constexpr uint8_t MODE_COUNT = 3;

static const char* modeName(DeviceMode m) {
  switch (m) {
    case DeviceMode::Navigation: return "Navigation";
    case DeviceMode::Music:      return "Music";
    case DeviceMode::Both:       return "Both";
    default:                     return "Unknown";
  }
}

// ── Button state ──
struct ButtonState {
  bool lastReading = HIGH;
  uint32_t lastDebounceMs = 0;
  bool stableState = HIGH;
  static constexpr uint32_t DEBOUNCE_MS = 50;
};

// ── Globals ──
static VL53L1X sensorLeft;
static VL53L1X sensorCenter;
static VL53L1X sensorRight;

static MedianFilter filtLeft;
static MedianFilter filtCenter;
static MedianFilter filtRight;

static VibState vib;
static GpsData gpsData;
static DeviceMode currentMode = DeviceMode::Navigation;
static ButtonState btn;
static uint32_t lastFirebaseSendMs = 0;
static uint8_t sensorDeadCount = 0;

static HardwareSerial GPS(GPS_UART_NUM);

// ── Motor helper ──
static void vibWrite(bool on) {
  if (VIB_ACTIVE_LOW) {
    digitalWrite(VIB_MOTOR_PIN, on ? LOW : HIGH);
  } else {
    digitalWrite(VIB_MOTOR_PIN, on ? HIGH : LOW);
  }
}

// ── Sensor init ──
static void setXshutAllLow() {
  digitalWrite(XSHUT_LEFT_PIN, LOW);
  digitalWrite(XSHUT_CENTER_PIN, LOW);
  digitalWrite(XSHUT_RIGHT_PIN, LOW);
}

static bool initOneSensor(VL53L1X &s, int xshutPin, uint8_t newAddr) {
  digitalWrite(xshutPin, HIGH);
  delay(50);                       // 50 ms for reliable boot
  s.setTimeout(500);
  if (!s.init()) return false;
  s.setAddress(newAddr);
  s.setDistanceMode(VL53L1X::Long);
  s.setMeasurementTimingBudget(33000);   // 33 ms budget
  s.startContinuous(50);                 // new measurement every 50 ms
  s.setTimeout(100);                     // 100 ms read timeout for loop
  return true;
}

static bool selfTestOneSensor(VL53L1X &s, const char* name) {
  Serial.print("Self-test ");
  Serial.print(name);
  Serial.print(": ");

  uint16_t lastMm = 0;
  bool ok = false;
  for (int i = 0; i < 3; i++) {
    uint16_t mm = s.read();          // blocking — waits for fresh data
    if (!s.timeoutOccurred() && mm != 0 && mm != 65535) {
      lastMm = mm;
      ok = true;
      break;
    }
  }

  if (!ok) {
    Serial.println("FAIL (no valid readings)");
    return false;
  }

  Serial.print("OK (");
  Serial.print(lastMm);
  Serial.print(" mm / ");
  Serial.print(mmToCm(lastMm));
  Serial.println(" cm)");
  return true;
}

static void selfTestAllSensors() {
  Serial.println("Sensor self-test (quick):");
  const bool okL = selfTestOneSensor(sensorLeft, "LEFT");
  const bool okC = selfTestOneSensor(sensorCenter, "CENTER");
  const bool okR = selfTestOneSensor(sensorRight, "RIGHT");
  if (okL && okC && okR) {
    Serial.println("All sensors OK");
  } else {
    Serial.println("One or more sensors FAILED (check power/I2C/XSHUT)");
  }
}

static bool initSensors() {
  // ── Reset sensor objects so init() talks to the default address (0x29) ──
  // After XSHUT reset, sensors revert to 0x29.  The Pololu library caches
  // the last setAddress() value, so re-creating the objects is essential.
  sensorLeft   = VL53L1X();
  sensorCenter = VL53L1X();
  sensorRight  = VL53L1X();

  pinMode(XSHUT_LEFT_PIN, OUTPUT);
  pinMode(XSHUT_CENTER_PIN, OUTPUT);
  pinMode(XSHUT_RIGHT_PIN, OUTPUT);
  setXshutAllLow();
  delay(50);                        // longer reset pulse for reliable power-down

  Serial.println("Initializing VL53L1X sensors...");
  Serial.print("I2C pins: SDA=");
  Serial.print(I2C_SDA_PIN);
  Serial.print(" SCL=");
  Serial.println(I2C_SCL_PIN);

  Serial.print("Init LEFT (addr 0x");
  Serial.print(ADDR_LEFT, HEX);
  Serial.println(") ...");
  if (!initOneSensor(sensorLeft, XSHUT_LEFT_PIN, ADDR_LEFT)) {
    Serial.println("Init LEFT: FAIL");
    return false;
  }
  Serial.println("Init LEFT: OK");

  Serial.print("Init CENTER (addr 0x");
  Serial.print(ADDR_CENTER, HEX);
  Serial.println(") ...");
  if (!initOneSensor(sensorCenter, XSHUT_CENTER_PIN, ADDR_CENTER)) {
    Serial.println("Init CENTER: FAIL");
    return false;
  }
  Serial.println("Init CENTER: OK");

  Serial.print("Init RIGHT (addr 0x");
  Serial.print(ADDR_RIGHT, HEX);
  Serial.println(") ...");
  if (!initOneSensor(sensorRight, XSHUT_RIGHT_PIN, ADDR_RIGHT)) {
    Serial.println("Init RIGHT: FAIL");
    return false;
  }
  Serial.println("Init RIGHT: OK");

  selfTestAllSensors();
  return true;
}

// ── I2C bus recovery ──
// If a sensor dies mid-transfer it can hold SDA low, bricking the bus.
// Toggle SCL manually to clock out any stuck bits, then re-init Wire.
static void recoverI2CBus() {
  Wire.end();
  delay(5);

  pinMode(I2C_SDA_PIN, INPUT_PULLUP);
  pinMode(I2C_SCL_PIN, OUTPUT);

  // Clock out up to 9 bits to release a stuck SDA
  for (int i = 0; i < 9; i++) {
    digitalWrite(I2C_SCL_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(I2C_SCL_PIN, HIGH);
    delayMicroseconds(5);
    if (digitalRead(I2C_SDA_PIN) == HIGH) break;   // SDA released
  }

  // Send a STOP condition (SDA low→high while SCL high)
  pinMode(I2C_SDA_PIN, OUTPUT);
  digitalWrite(I2C_SDA_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(I2C_SCL_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(I2C_SDA_PIN, HIGH);
  delayMicroseconds(5);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);
}

// ── Sensor read helpers ──
// Uses blocking read — the library waits for fresh data internally.
// Returns: >0 = valid mm, 0 = bad/timeout.
static uint16_t readMm(VL53L1X &s) {
  uint16_t mm = s.read();           // BLOCKING: waits for dataReady, then clears interrupt
  if (s.timeoutOccurred()) return 0;
  if (mm == 0 || mm == 65535) return 0;
  return mm;
}

static uint16_t mmToCm(uint16_t mm) {
  if (mm == 0) return 0;
  uint16_t cm = (mm + 5) / 10;
  if (cm > MAX_VALID_CM) return 0;
  return cm;
}

// ── Vibration logic ──
static VibPattern patternForMinDistanceCm(uint16_t minCm) {
  if (minCm == 0) return VibPattern::Off;
  if (minCm <= THRESH_NEAR_CM) return VibPattern::Continuous;
  if (minCm <= THRESH_MID_CM) return VibPattern::Fast;
  if (minCm <= THRESH_FAR_CM) return VibPattern::Slow;
  return VibPattern::Off;
}

static void vibSetPattern(VibPattern p) {
  if (vib.pattern == p) return;
  vib.pattern = p;
  vib.on = false;
  vib.nextToggleAtMs = 0;
  vibWrite(false);
}

static void vibUpdate(uint32_t nowMs) {
  if (vib.pattern == VibPattern::Off) {
    if (vib.on) { vib.on = false; vibWrite(false); }
    return;
  }
  if (vib.pattern == VibPattern::Continuous) {
    if (!vib.on) { vib.on = true; vibWrite(true); }
    return;
  }

  uint32_t onMs = 0, offMs = 0;
  if (vib.pattern == VibPattern::Fast) { onMs = 80; offMs = 120; }
  else { onMs = 200; offMs = 400; }

  if (vib.nextToggleAtMs == 0) {
    vib.on = true;
    vibWrite(true);
    vib.nextToggleAtMs = nowMs + onMs;
    return;
  }
  if (nowMs < vib.nextToggleAtMs) return;

  if (vib.on) {
    vib.on = false;
    vibWrite(false);
    vib.nextToggleAtMs = nowMs + offMs;
  } else {
    vib.on = true;
    vibWrite(true);
    vib.nextToggleAtMs = nowMs + onMs;
  }
}

// ── Button (debounced, short press cycles mode) ──
static void buttonPoll(uint32_t nowMs) {
  bool reading = digitalRead(BUTTON_PIN);
  if (reading != btn.lastReading) {
    btn.lastDebounceMs = nowMs;
  }
  btn.lastReading = reading;

  if ((nowMs - btn.lastDebounceMs) >= ButtonState::DEBOUNCE_MS) {
    if (reading != btn.stableState) {
      bool prevStable = btn.stableState;
      btn.stableState = reading;
      if (prevStable == LOW && btn.stableState == HIGH) {
        uint8_t next = (static_cast<uint8_t>(currentMode) + 1) % MODE_COUNT;
        currentMode = static_cast<DeviceMode>(next);
        Serial.print("Mode: ");
        Serial.println(modeName(currentMode));
      }
    }
  }
}

// ── WiFi ──
static void connectWiFi() {
  Serial.print("WiFi: connecting to ");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("\nWiFi connected, IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi: connection failed (will retry)");
  }
}

// ── GPS parsing ──
static float parseNmeaCoord(const char* s) {
  float raw = atof(s);
  int deg = (int)(raw / 100);
  float mins = raw - deg * 100.0f;
  return deg + mins / 60.0f;
}

static void parseGpgga(const char* sentence) {
  char buf[128];
  strncpy(buf, sentence, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = 0;

  char* fields[15];
  int fieldCount = 0;
  char* p = buf;
  fields[fieldCount++] = p;
  while (*p && fieldCount < 15) {
    if (*p == ',') {
      *p = 0;
      fields[fieldCount++] = p + 1;
    }
    p++;
  }
  if (fieldCount < 7) return;

  int fix = atoi(fields[6]);
  if (fix == 0) { gpsData.valid = false; return; }

  gpsData.lat = parseNmeaCoord(fields[2]);
  if (fields[3][0] == 'S') gpsData.lat = -gpsData.lat;

  gpsData.lng = parseNmeaCoord(fields[4]);
  if (fields[5][0] == 'W') gpsData.lng = -gpsData.lng;

  gpsData.valid = true;
}

static void gpsPoll() {
  static char line[128];
  static uint8_t idx = 0;
  while (GPS.available() > 0) {
    const char c = static_cast<char>(GPS.read());
    if (c == '\r') continue;
    if (c == '\n') {
      line[idx] = 0;
      idx = 0;
      if (line[0] != '$') continue;
      if (strncmp(line, "$GPGGA", 6) == 0 || strncmp(line, "$GNGGA", 6) == 0) {
        parseGpgga(line);
      }
      continue;
    }
    if (idx + 1 < sizeof(line)) {
      line[idx++] = c;
    } else {
      idx = 0;
    }
  }
}

static void gpsSelfTest(uint32_t timeoutMs) {
  Serial.println("GPS self-test (quick):");
  Serial.print("Waiting for NMEA (");
  Serial.print(timeoutMs);
  Serial.println(" ms) ...");

  const uint32_t start = millis();
  bool sawNmea = false;

  char line[128];
  uint8_t idx = 0;

  while (millis() - start < timeoutMs) {
    while (GPS.available() > 0) {
      const char c = static_cast<char>(GPS.read());
      if (c == '\r') continue;
      if (c == '\n') {
        line[idx] = 0;
        idx = 0;
        if (line[0] == '$') {
          sawNmea = true;
          if (strncmp(line, "$GPGGA", 6) == 0 || strncmp(line, "$GNGGA", 6) == 0) {
            parseGpgga(line);
          }
        }
        continue;
      }
      if (idx + 1 < sizeof(line)) {
        line[idx++] = c;
      } else {
        idx = 0;
      }
    }
    delay(5);
  }

  if (!sawNmea) {
    Serial.println("GPS: FAIL (no NMEA data received)");
    return;
  }

  Serial.println("GPS: OK (NMEA received)");
  if (gpsData.valid) {
    Serial.print("GPS fix: OK (");
    Serial.print(gpsData.lat, 6);
    Serial.print(", ");
    Serial.print(gpsData.lng, 6);
    Serial.println(")");
  } else {
    Serial.println("GPS fix: NO (still acquiring)");
  }
}

// ── Firebase ──
static const char* vibPatternName(VibPattern p) {
  switch (p) {
    case VibPattern::Continuous: return "Continuous";
    case VibPattern::Fast:       return "Fast";
    case VibPattern::Slow:       return "Slow";
    default:                     return "Off";
  }
}

static void sendToFirebase(uint16_t l, uint16_t c, uint16_t r, uint16_t minCm) {
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  String url = String(FIREBASE_HOST) + "/device.json";
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(1500);

  String json = "{";
  json += "\"left_cm\":" + String(l) + ",";
  json += "\"center_cm\":" + String(c) + ",";
  json += "\"right_cm\":" + String(r) + ",";
  json += "\"min_cm\":" + String(minCm) + ",";
  json += "\"pattern\":\"" + String(vibPatternName(vib.pattern)) + "\",";
  json += "\"lat\":" + String(gpsData.lat, 6) + ",";
  json += "\"lng\":" + String(gpsData.lng, 6) + ",";
  json += "\"gps_valid\":" + String(gpsData.valid ? "true" : "false") + ",";
  json += "\"mode\":\"" + String(modeName(currentMode)) + "\",";
  json += "\"timestamp\":{\".sv\":\"timestamp\"}";
  json += "}";

  int code = http.PUT(json);
  if (code > 0) {
    Serial.print("Firebase: ");
    Serial.println(code);
  } else {
    Serial.print("Firebase error: ");
    Serial.println(http.errorToString(code));
  }
  http.end();
}

// ── Setup ──
void setup() {
  Serial.begin(115200);
  delay(100);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);

  pinMode(VIB_MOTOR_PIN, OUTPUT);
  vibWrite(false);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Serial.println("Initializing GPS...");
  GPS.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  gpsSelfTest(3000);

  const bool ok = initSensors();
  if (!ok) {
    Serial.println("ERROR: VL53L1X init failed. Check I2C + XSHUT wiring/pins.");
  } else {
    Serial.println("OK: VL53L1X sensors initialized at 0x30/0x31/0x32");
  }

  connectWiFi();
}

// ── Loop ──
void loop() {
  static uint32_t lastTick = 0;
  const uint32_t now = millis();

  if (WiFi.status() != WL_CONNECTED) {
    static uint32_t lastReconnect = 0;
    if (now - lastReconnect > 15000) {
      lastReconnect = now;
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      Serial.println("WiFi: reconnecting...");
    }
  }

  if (now - lastTick < LOOP_DT_MS) {
    gpsPoll();
    buttonPoll(now);
    vibUpdate(now);
    delay(1);
    return;
  }
  lastTick = now;

  const uint16_t rawLeft   = readMm(sensorLeft);
  const uint16_t rawCenter = readMm(sensorCenter);
  const uint16_t rawRight  = readMm(sensorRight);

  const uint16_t leftCm   = mmToCm(rawLeft);
  const uint16_t centerCm = mmToCm(rawCenter);
  const uint16_t rightCm  = mmToCm(rawRight);

  if (leftCm > 0)   filtLeft.push(leftCm);   else filtLeft.miss();
  if (centerCm > 0) filtCenter.push(centerCm); else filtCenter.miss();
  if (rightCm > 0)  filtRight.push(rightCm);  else filtRight.miss();

  const uint16_t l = filtLeft.median();
  const uint16_t c = filtCenter.median();
  const uint16_t r = filtRight.median();

  uint16_t minCm = 0;
  if (l > 0) minCm = l;
  if (c > 0 && (minCm == 0 || c < minCm)) minCm = c;
  if (r > 0 && (minCm == 0 || r < minCm)) minCm = r;

  if (l == 0 && c == 0 && r == 0) {
    sensorDeadCount++;
    if (sensorDeadCount >= 25) {                       // ~5 s at 5 Hz
      Serial.println("Sensors dead. Recovering I2C bus and re-initializing...");
      recoverI2CBus();                                 // unstick SDA if held low
      delay(50);
      if (!initSensors()) {
        Serial.println("Re-init failed. Will retry next cycle.");
      }
      filtLeft = MedianFilter();
      filtCenter = MedianFilter();
      filtRight = MedianFilter();
      sensorDeadCount = 0;
    }
  } else {
    sensorDeadCount = 0;
  }

  bool navEnabled = (currentMode == DeviceMode::Navigation || currentMode == DeviceMode::Both);
  if (navEnabled) {
    vibSetPattern(patternForMinDistanceCm(minCm));
  } else {
    vibSetPattern(VibPattern::Off);
  }
  vibUpdate(now);
  buttonPoll(now);

  Serial.print("raw mm: ");
  Serial.print(rawLeft);
  Serial.print("/");
  Serial.print(rawCenter);
  Serial.print("/");
  Serial.print(rawRight);
  Serial.print("  cm: L=");
  Serial.print(l);
  Serial.print(" C=");
  Serial.print(c);
  Serial.print(" R=");
  Serial.print(r);
  Serial.print(" min=");
  Serial.println(minCm);

  if (now - lastFirebaseSendMs >= FIREBASE_SEND_MS) {
    lastFirebaseSendMs = now;
    sendToFirebase(l, c, r, minCm);
  }

  gpsPoll();
}
