#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>

static constexpr int I2C_SDA_PIN = 21;
static constexpr int I2C_SCL_PIN = 22;
static constexpr uint32_t I2C_HZ = 100000;

// Optional: if you wired XSHUT, set it here. If not wired, leave it as -1.
static constexpr int XSHUT_PIN = 26;

static VL53L1X sensor;

static void i2cScan() {
  Serial.println("I2C scan:");
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    const uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.print(" - Found device at 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      found++;
    }
  }
  if (found == 0) Serial.println(" - No I2C devices found");
}

static bool initSensor() {
  if (XSHUT_PIN >= 0) {
    pinMode(XSHUT_PIN, OUTPUT);
    digitalWrite(XSHUT_PIN, LOW);
    delay(50);
    digitalWrite(XSHUT_PIN, HIGH);
    delay(50);
  }

  sensor.setTimeout(500);
  if (!sensor.init()) {
    return false;
  }

  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(50000); // 50ms
  sensor.startContinuous(50);               // new reading ~every 50ms
  sensor.setTimeout(200);
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("VL53L1X single-sensor test (ESP32)");

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_HZ);

  i2cScan();

  Serial.println("Initializing sensor at default address (0x29)...");
  if (!initSensor()) {
    Serial.println("ERROR: sensor init failed.");
    Serial.println("Check: VCC=3V3, GND, SDA=GPIO21, SCL=GPIO22.");
    Serial.println("If you used XSHUT, confirm it is on GPIO26 (or edit XSHUT_PIN).");
    return;
  }

  Serial.println("OK: sensor initialized.");
  Serial.println("Move your hand 10-50cm in front of it; values should change.");
}

void loop() {
  const uint16_t mm = sensor.read(); // blocking read for fresh data
  if (sensor.timeoutOccurred() || mm == 0 || mm == 65535) {
    Serial.println("Reading: INVALID/TIMEOUT");
  } else {
    const uint16_t cm = (mm + 5) / 10;
    Serial.print("Reading: ");
    Serial.print(mm);
    Serial.print(" mm (");
    Serial.print(cm);
    Serial.println(" cm)");
  }
  delay(100);
}
