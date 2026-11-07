#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <esp_system.h>

// ======================= CONFIG: EDIT THESE 3 LINES =======================
const char *WIFI_SSID = "exile";                // must be 2.4 GHz
const char *WIFI_PASS = "bovendigoel";
const char *MQTT_HOST = "10.122.243.121";       // your Mac's IP (ipconfig getifaddr en0)
// ==========================================================================

const bool BUZZER_ENABLED = true;   // set false to silence the buzzer hardware (e.g. when debugging power)

const uint16_t MQTT_PORT   = 1883;
const char *TOPIC_DATA     = "cargo/data";     // ESP32 -> backend (10 Hz JSON)
const char *TOPIC_STATUS   = "cargo/status";   // "online" / "offline"
const char *TOPIC_CMD      = "cargo/cmd";      // backend -> ESP32: recal | mute | unmute

// ---- Pins (match the expansion-board wiring) ----
const int PIN_SDA0 = 21, PIN_SCL0 = 22;   // LCD (I2C bus 0)
const int PIN_SDA1 = 16, PIN_SCL1 = 17;   // MPU6050 (I2C bus 1)
const int PIN_DHT = 4;
const int PIN_BUZZER = 18;
const int PIN_PRESSURE = 32;
const int PIN_RED = 25, PIN_YEL = 26, PIN_GRN = 27;

// ---- Rule thresholds (rule-based automation) ----
const float FREEFALL_LIMIT = 4.0;    // m/s^2, below = thrown/dropped (rest is ~9.8)
const float IMPACT_LIMIT   = 25.0;   // m/s^2, above = impact/shake
const float TEMP_LIMIT     = 35.0;   // C, above = too hot (red)
const float TEMP_WARN      = 32.0;   // C, above = warning (yellow)
const int   PRESSURE_DELTA = 400;    // change from baseline (0-4095) = crushed/moved
const unsigned long HOLD_MS = 2000;  // alert stays active this long

const unsigned long SAMPLE_MS  = 10;    // MPU + rules at 100 Hz
const unsigned long PUBLISH_MS = 100;   // MQTT at 10 Hz
const unsigned long LCD_MS     = 250;
const unsigned long DHT_MS     = 2000;
const int BUZZER_FREQ = 2000;

const uint8_t MPU = 0x68;
const float ACC_SCALE  = 9.80665f / 2048.0f;  // +-16 g
const float GYRO_SCALE = 1.0f / 65.5f;        // +-500 dps

LiquidCrystal_I2C lcd(0x27, 16, 2);
DHT dht(PIN_DHT, DHT11);
WiFiClient net;
PubSubClient mqtt(net);

float temp = NAN, hum = NAN;
float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0, mag = 9.8f;
float accMin = 1000, accMax = 0;
int pressure = 0, baseline = 0;
bool muted = false;

const char *alertCode = "none";
const char *alertText = "";
unsigned long alertUntil = 0;
bool alarmOn = false, warmOn = false, buzzerOn = false;

unsigned long lastSample = 0, lastPublish = 0, lastLcd = 0, lastDht = 0, lastNetTry = 0, lastDbg = 0;

// ---------------------------------------------------------------- helpers
void beep(bool on) {
  if (!BUZZER_ENABLED) return;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWriteTone(PIN_BUZZER, on ? BUZZER_FREQ : 0);
#else
  ledcWriteTone(0, on ? BUZZER_FREQ : 0);
#endif
}

void writeReg(uint8_t reg, uint8_t val) {
  Wire1.beginTransmission(MPU);
  Wire1.write(reg);
  Wire1.write(val);
  Wire1.endTransmission();
}

bool readMPU() {
  Wire1.beginTransmission(MPU);
  Wire1.write(0x3B);
  if (Wire1.endTransmission(false) != 0) return false;
  if (Wire1.requestFrom((int)MPU, 14) != 14) return false;
  int16_t raw[7];
  for (int i = 0; i < 7; i++) {
    uint8_t hi = Wire1.read();
    uint8_t lo = Wire1.read();
    raw[i] = (int16_t)((hi << 8) | lo);
  }
  ax = raw[0] * ACC_SCALE; ay = raw[1] * ACC_SCALE; az = raw[2] * ACC_SCALE;
  // raw[3] is the chip temperature, ignored
  gx = raw[4] * GYRO_SCALE; gy = raw[5] * GYRO_SCALE; gz = raw[6] * GYRO_SCALE;
  mag = sqrtf(ax * ax + ay * ay + az * az);
  return true;
}

int readPressure() {
  long sum = 0;
  for (int i = 0; i < 8; i++) sum += analogRead(PIN_PRESSURE);
  return sum / 8;
}

void raiseAlert(const char *code, const char *text) {
  alertCode = code;
  alertText = text;
  alertUntil = millis() + HOLD_MS;
}

void showLine(int row, const char *text) {
  char buf[20];
  snprintf(buf, sizeof(buf), "%-16.16s", text);
  lcd.setCursor(0, row);
  lcd.print(buf);
}

// ---------------------------------------------------------------- MQTT
void onMessage(char *topic, byte *payload, unsigned int len) {
  String m;
  for (unsigned int i = 0; i < len; i++) m += (char)payload[i];
  if (m == "recal") baseline = readPressure();
  else if (m == "mute") muted = true;
  else if (m == "unmute") muted = false;
}

// The alarm rules keep running even if WiFi/MQTT is down.
void maintainConnection() {
  unsigned long now = millis();
  if (WiFi.status() != WL_CONNECTED) {
    if (now - lastNetTry > 20000) {
      lastNetTry = now;
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASS);
    }
    return;
  }
  if (!mqtt.connected() && now - lastNetTry > 5000) {
    lastNetTry = now;
    // short TCP probe first so an unreachable broker only stalls ~0.5 s
    if (net.connect(MQTT_HOST, MQTT_PORT, 500)) {
      if (mqtt.connect("cargo-esp32", nullptr, nullptr, TOPIC_STATUS, 0, true, "offline")) {
        mqtt.publish(TOPIC_STATUS, "online", true);
        mqtt.subscribe(TOPIC_CMD);
      }
    }
  }
}

void fmtNum(char *out, size_t n, float v, int dec) {
  if (isnan(v)) snprintf(out, n, "null");
  else snprintf(out, n, "%.*f", dec, v);
}

void publishData() {
  char t[12], h[12];
  fmtNum(t, sizeof(t), temp, 1);
  fmtNum(h, sizeof(h), hum, 1);
  char buf[420];
  snprintf(buf, sizeof(buf),
    "{\"uptime_ms\":%lu,\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
    "\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,"
    "\"acc_mag\":%.3f,\"acc_min\":%.3f,\"acc_max\":%.3f,"
    "\"temp\":%s,\"hum\":%s,\"pressure\":%d,\"p_base\":%d,"
    "\"buzzer\":%d,\"red\":%d,\"yellow\":%d,\"green\":%d,\"alert\":\"%s\"}",
    millis(), ax, ay, az, gx, gy, gz, mag, accMin, accMax,
    t, h, pressure, baseline,
    (alarmOn && !muted) ? 1 : 0, alarmOn ? 1 : 0,
    (!alarmOn && warmOn) ? 1 : 0, (!alarmOn && !warmOn) ? 1 : 0,
    alarmOn ? alertCode : "none");
  mqtt.publish(TOPIC_DATA, buf);
  accMin = 1000; accMax = 0;
}

// ---------------------------------------------------------------- setup / loop
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.printf("\nreset reason=%d\n", (int)esp_reset_reason());

  // WiFi first, with reduced TX power (same order as the working scan test):
  // mode -> tx power -> begin
  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setBufferSize(512);
  mqtt.setCallback(onMessage);

  pinMode(PIN_RED, OUTPUT);
  pinMode(PIN_YEL, OUTPUT);
  pinMode(PIN_GRN, OUTPUT);

  Wire.begin(PIN_SDA0, PIN_SCL0);
  Wire1.begin(PIN_SDA1, PIN_SCL1);
  Wire1.setClock(400000);
  lcd.init();
  lcd.backlight();
  dht.begin();

  if (BUZZER_ENABLED) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(PIN_BUZZER, BUZZER_FREQ, 8);
#else
    ledcSetup(0, BUZZER_FREQ, 8);
    ledcAttachPin(PIN_BUZZER, 0);
#endif
    beep(false);
  }

  writeReg(0x6B, 0x00);   // wake MPU
  writeReg(0x1A, 0x03);   // low-pass filter ~44 Hz
  writeReg(0x1B, 0x08);   // gyro +-500 dps
  writeReg(0x1C, 0x18);   // accel +-16 g

  analogReadResolution(12);
  pinMode(PIN_PRESSURE, INPUT);

  showLine(0, "Calibrating...");
  showLine(1, "Keep still");
  delay(1500);
  baseline = readPressure();
  Serial.printf("baseline=%d\n", baseline);
}

void loop() {
  unsigned long now = millis();

  if (now - lastDbg > 3000) {
    lastDbg = now;
    Serial.printf("wifi=%d mqtt=%d ip=%s\n", WiFi.status(), mqtt.state(),
                  WiFi.localIP().toString().c_str());
  }

  maintainConnection();
  if (mqtt.connected()) mqtt.loop();

  // DHT11: slow sensor, read every 2 s
  if (now - lastDht >= DHT_MS) {
    lastDht = now;
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    if (!isnan(t) && !isnan(h)) { temp = t; hum = h; }
  }

  // Sensors + rules + actuators at 100 Hz
  if (now - lastSample >= SAMPLE_MS) {
    lastSample = now;
    if (readMPU()) {
      if (mag < accMin) accMin = mag;
      if (mag > accMax) accMax = mag;
      if (mag > IMPACT_LIMIT)        raiseAlert("impact", "IMPACT!");
      else if (mag < FREEFALL_LIMIT) raiseAlert("thrown", "THROWN/DROPPED!");
    }
    pressure = readPressure();
    if (abs(pressure - baseline) > PRESSURE_DELTA) raiseAlert("crushed", "crushed/MOVED!");
    if (!isnan(temp) && temp > TEMP_LIMIT)         raiseAlert("hot", "TOO HOT!");

    alarmOn = now < alertUntil;
    warmOn = !isnan(temp) && temp > TEMP_WARN;

    buzzerOn = alarmOn && !muted && ((now / 200) % 2 == 0);
    beep(buzzerOn);
    digitalWrite(PIN_RED, alarmOn);
    digitalWrite(PIN_YEL, !alarmOn && warmOn);
    digitalWrite(PIN_GRN, !alarmOn && !warmOn);
  }

  // MQTT publish at 10 Hz
  if (now - lastPublish >= PUBLISH_MS) {
    lastPublish = now;
    if (mqtt.connected()) publishData();
  }

  // LCD at 4 Hz
  if (now - lastLcd >= LCD_MS) {
    lastLcd = now;
    char tmp[20], l1[24], l2[20];
    if (isnan(temp)) snprintf(tmp, sizeof(tmp), "Reading DHT...");
    else snprintf(tmp, sizeof(tmp), "%.0fC Hum:%.0f%%", temp, hum);
    char ind = mqtt.connected() ? 'M' : (WiFi.status() == WL_CONNECTED ? 'W' : '-');
    snprintf(l1, sizeof(l1), "%-15.15s%c", tmp, ind);
    lcd.setCursor(0, 0);
    lcd.print(l1);
    if (alarmOn) showLine(1, alertText);
    else { snprintf(l2, sizeof(l2), "OK P:%d", pressure); showLine(1, l2); }
  }
}