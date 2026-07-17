// ================================================================
//  EV-NODE v3 | INA219 + HiveMQ Edition | ESP32-C3
//  INA219 Battery Monitor + WiFi + MQTT Telemetry
//
//  WIRING:
//    INA219 SDA → GPIO4
//    INA219 SCL → GPIO5
//    INA219 VCC → 3.3V
//    INA219 GND → GND
//    WS2812B LED → GPIO8 (onboard RGB on ESP32-C3 DevKit)
//
//  SERIAL: 115200 baud — dashboard refreshes every 2s
//  MQTT:   Publishes every 2s to HiveMQ Cloud
//
//  WS2812B LED STATUS:
//    White  Solid      = Booting / Initializing
//    Blue   Fast Blink = WiFi Connecting
//    Blue   Solid      = WiFi Connected
//    Purple Fast Blink = MQTT Connecting
//    Green  Solid      = MQTT Connected / Idle
//    Green  Pulse      = Charging detected
//    Orange Pulse      = Discharging (load detected)
//    Cyan   Flash      = MQTT Publish Success
//    Red    Fast Blink = Error / WiFi Lost / Sensor Error
//
//  LIBRARIES (Install via Arduino Library Manager):
//    - Adafruit INA219       (by Adafruit)
//    - Adafruit NeoPixel     (by Adafruit)
//    - PubSubClient          (by Nick O'Leary)
//    - Wire                  (built-in)
//    - WiFi / WiFiClientSecure (built-in ESP32 core)
//
//  BOARD: ESP32-C3 Dev Module
// ================================================================

#include <Wire.h>
#include <Adafruit_INA219.h>
#include <Adafruit_NeoPixel.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

// ================================================================
//  INA219 — DO NOT CHANGE THESE LINES
// ================================================================
Adafruit_INA219 ina219;

// ================================================================
//  I2C PINS
// ================================================================
#define I2C_SDA 4
#define I2C_SCL 5

// ================================================================
//  LED CONFIG
// ================================================================
#define LED_PIN    7
#define LED_COUNT  1
#define LED_BRIGHT 30

Adafruit_NeoPixel led(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

#define COLOR_OFF    led.Color(0,   0,   0)
#define COLOR_WHITE  led.Color(255, 255, 255)
#define COLOR_RED    led.Color(255, 0,   0)
#define COLOR_GREEN  led.Color(0,   200, 0)
#define COLOR_BLUE   led.Color(0,   0,   255)
#define COLOR_ORANGE led.Color(255, 80,  0)
#define COLOR_CYAN   led.Color(0,   255, 220)
#define COLOR_PURPLE led.Color(150, 0,   255)

enum LedMode { LED_SOLID, LED_BLINK_FAST, LED_PULSE };

LedMode  currentLedMode  = LED_SOLID;
uint32_t currentLedColor = COLOR_WHITE;
unsigned long ledLastMs  = 0;
bool     ledState        = false;
int      pulseVal        = 0;
int      pulseDir        = 1;

// ================================================================
//  WIFI CREDENTIALS
// ================================================================
#define WIFI_SSID     "Airtel_Zerotouch"
#define WIFI_PASSWORD "Airtel@123"

// ================================================================
//  MQTT CONFIGURATION — HiveMQ Cloud
// ================================================================
const char* MQTT_SERVER = "dd4770b26a7d4a7291371cea6d74c060.s1.eu.hivemq.cloud";
const int   MQTT_PORT   = 8883;
const char* MQTT_USER   = "admin";
const char* MQTT_PASS   = "Pass@123";

const char* TOPIC_TELEMETRY = "ev_vehicle/v3/telemetry";
const char* TOPIC_COMMANDS  = "ev_vehicle/v3/commands";

WiFiClientSecure espClient;
PubSubClient     mqttClient(espClient);

// ================================================================
//  BATTERY CONFIGURATION
// ================================================================
#define BATTERY_CAP_AH          2.2f
#define BATT_MIN_V              9.0f
#define BATT_MAX_V              12.6f

// ================================================================
//  THRESHOLD VALUES
// ================================================================
#define CHARGING_DETECT_VOLTAGE 11.400f
#define CHARGING_CURRENT_MIN    0.10f
#define DISCHARGE_CURRENT_MIN   0.15f
#define CURRENT_DEADBAND        0.012f
#define MOVING_AVG_SIZE         10

// ================================================================
//  SMOOTHING BUFFERS
// ================================================================
float voltageBuffer[MOVING_AVG_SIZE] = {0};
float currentBuffer[MOVING_AVG_SIZE] = {0};
int   bufferIndex = 0;
bool  bufferFull  = false;

// ================================================================
//  SENSOR DATA
// ================================================================
float busVoltage_V    = 0.0f;
float shuntVoltage_mV = 0.0f;
float loadVoltage_V   = 0.0f;
float current_mA      = 0.0f;
float current_A       = 0.0f;
float power_mW        = 0.0f;
float power_W         = 0.0f;
float smoothedVoltage = 0.0f;
float smoothedCurrent = 0.0f;

// Calculated
float shuntResistance = 0.0f;
float voltageDrop     = 0.0f;
float efficiency      = 0.0f;
float noLoadVoltage   = 0.0f;

// Tracking
float maxCurrent_A    = 0.0f;
float minCurrent_A    = 0.0f;
float avgCurrent_A    = 0.0f;
float peakPower_W     = 0.0f;
float totalEnergy_Wh  = 0.0f;
float avgCurrentAcc   = 0.0f;
long  avgCurrentCount = 0;

// Runtime
unsigned long startTime   = 0;
unsigned long lastTime    = 0;
unsigned long lastPublish = 0;
float runtimeSeconds      = 0.0f;
float runtimeMinutes      = 0.0f;
float runtimeHours        = 0.0f;

// Vehicle state
int    soc              = 0;
int    timeToFull       = 0;
int    timeToEmpty      = 0;
bool   chargingDetected = false;
bool   sensorOK         = false;
int    sensorFailCount  = 0;
#define SENSOR_FAIL_LIMIT 5

String vehicleStatus = "Booting";

// ================================================================
//  LED CONTROL
// ================================================================
void setLED(uint32_t color, LedMode mode = LED_SOLID) {
  currentLedColor = color;
  currentLedMode  = mode;
  ledState = true;
  if (mode == LED_SOLID) {
    led.setPixelColor(0, color);
    led.show();
  }
}

void flashLED(uint32_t flashColor) {
  led.setPixelColor(0, flashColor);
  led.show();
  delay(80);
  led.setPixelColor(0, currentLedColor);
  led.show();
}

void updateLED() {
  unsigned long now = millis();
  switch (currentLedMode) {
    case LED_SOLID:
      break;

    case LED_BLINK_FAST:
      if (now - ledLastMs > 150) {
        ledLastMs = now;
        ledState  = !ledState;
        led.setPixelColor(0, ledState ? currentLedColor : COLOR_OFF);
        led.show();
      }
      break;

    case LED_PULSE:
      if (now - ledLastMs > 12) {
        ledLastMs = now;
        pulseVal += pulseDir * 4;
        if (pulseVal >= 255) { pulseVal = 255; pulseDir = -1; }
        if (pulseVal <= 0)   { pulseVal = 0;   pulseDir =  1; }
        uint8_t r = (currentLedColor >> 16) & 0xFF;
        uint8_t g = (currentLedColor >> 8)  & 0xFF;
        uint8_t b =  currentLedColor        & 0xFF;
        led.setPixelColor(0, led.Color(
          (uint8_t)(r * pulseVal / 255),
          (uint8_t)(g * pulseVal / 255),
          (uint8_t)(b * pulseVal / 255)
        ));
        led.show();
      }
      break;
  }
}

// ================================================================
//  WIFI — Connect with watchdog
// ================================================================
void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  Serial.printf("\n[WiFi] Connecting to: %s\n", WIFI_SSID);
  setLED(COLOR_BLUE, LED_BLINK_FAST);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 40) {
    updateLED();
    delay(500);
    Serial.print('.');
    tries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n[WiFi] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
    setLED(COLOR_BLUE, LED_SOLID);
    delay(300);
  } else {
    Serial.println("\n[WiFi] Failed — will retry in loop");
    setLED(COLOR_RED, LED_BLINK_FAST);
  }
}

// ================================================================
//  MQTT CALLBACK — handle incoming commands
// ================================================================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg = "";
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  Serial.printf("[MQTT] Command received: %s\n", msg.c_str());

  if (msg == "REBOOT") {
    Serial.println("[MQTT] REBOOT command — restarting...");
    delay(500);
    ESP.restart();
  }
}

// ================================================================
//  MQTT — Connect with watchdog
// ================================================================
void connectMQTT() {
  if (mqttClient.connected()) return;
  if (WiFi.status() != WL_CONNECTED) return;

  setLED(COLOR_PURPLE, LED_BLINK_FAST);
  Serial.print("[MQTT] Connecting...");

  espClient.setInsecure();   // Skip CA cert for HiveMQ Cloud
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setKeepAlive(30);

  String clientId = "EV_v3_" + String((uint32_t)ESP.getEfuseMac(), HEX);

  if (mqttClient.connect(clientId.c_str(), MQTT_USER, MQTT_PASS)) {
    Serial.println(" Connected!");
    mqttClient.subscribe(TOPIC_COMMANDS);
    setLED(COLOR_GREEN, LED_SOLID);
  } else {
    Serial.printf(" Failed rc=%d\n", mqttClient.state());
    setLED(COLOR_RED, LED_BLINK_FAST);
  }
}

// ================================================================
//  MOVING AVERAGE FILTER
// ================================================================
void pushSample(float v, float i) {
  voltageBuffer[bufferIndex] = v;
  currentBuffer[bufferIndex] = i;
  bufferIndex = (bufferIndex + 1) % MOVING_AVG_SIZE;
  if (bufferIndex == 0) bufferFull = true;

  int count = bufferFull ? MOVING_AVG_SIZE : bufferIndex;
  float vSum = 0, iSum = 0;
  for (int k = 0; k < count; k++) {
    vSum += voltageBuffer[k];
    iSum += currentBuffer[k];
  }
  smoothedVoltage = vSum / count;
  smoothedCurrent = iSum / count;
}

// ================================================================
//  SOC CALCULATION
// ================================================================
int calculateSOC(float v) {
  if (v <= BATT_MIN_V) return 0;
  if (v >= BATT_MAX_V) return 100;
  return (int)constrain(
    map((long)(v * 100), (long)(BATT_MIN_V * 100), (long)(BATT_MAX_V * 100), 0, 100),
    0, 100
  );
}

// ================================================================
//  READ INA219
// ================================================================
bool readINA219() {
  float sv  = ina219.getShuntVoltage_mV();
  float bv  = ina219.getBusVoltage_V();
  float imA = ina219.getCurrent_mA();
  float pmW = ina219.getPower_mW();

  if (bv < 0.0f || bv > 20.0f || imA < -3000.0f || imA > 3000.0f) {
    sensorFailCount++;
    return false;
  }

  sensorFailCount = 0;
  sensorOK = true;

  shuntVoltage_mV = sv;
  busVoltage_V    = bv;
  current_mA      = imA;
  power_mW        = pmW;

  loadVoltage_V = busVoltage_V + (shuntVoltage_mV / 1000.0f);
  current_A     = current_mA / 1000.0f;
  power_W       = power_mW   / 1000.0f;

  if (fabsf(current_A) < CURRENT_DEADBAND) {
    current_A  = 0.0f;
    current_mA = 0.0f;
  }

  pushSample(loadVoltage_V, current_A);

  shuntResistance = (current_mA != 0.0f)
    ? fabsf(shuntVoltage_mV / current_mA) * 1000.0f : 0.0f;

  voltageDrop = (noLoadVoltage > 0.5f) ? (noLoadVoltage - smoothedVoltage) : 0.0f;

  efficiency = (noLoadVoltage > 0.5f)
    ? constrain((smoothedVoltage / noLoadVoltage) * 100.0f, 0.0f, 100.0f)
    : 100.0f;

  unsigned long now = millis();
  float dtH = (now - lastTime) / 3600000.0f;
  totalEnergy_Wh += fabsf(power_W) * dtH;
  lastTime = now;

  runtimeSeconds = (now - startTime) / 1000.0f;
  runtimeMinutes = runtimeSeconds / 60.0f;
  runtimeHours   = runtimeMinutes / 60.0f;

  if (current_A > maxCurrent_A) maxCurrent_A = current_A;
  if (current_A < minCurrent_A) minCurrent_A = current_A;
  if (power_W   > peakPower_W)  peakPower_W  = power_W;

  avgCurrentAcc += current_A;
  avgCurrentCount++;
  avgCurrent_A = avgCurrentAcc / avgCurrentCount;

  return true;
}

// ================================================================
//  UPDATE VEHICLE STATE
// ================================================================
void updateVehicleState() {
  if (!sensorOK || sensorFailCount >= SENSOR_FAIL_LIMIT) {
    vehicleStatus = "Sensor Error";
    setLED(COLOR_RED, LED_BLINK_FAST);
    soc = 0; timeToFull = 0; timeToEmpty = 0;
    return;
  }

  float v = smoothedVoltage;
  float i = smoothedCurrent;

  if (v < 1.0f) {
    vehicleStatus    = "Disconnected";
    chargingDetected = false;
    soc = 0;
    setLED(COLOR_RED, LED_BLINK_FAST);
    return;
  }

  soc = calculateSOC(v);

  bool voltageSign   = (v > CHARGING_DETECT_VOLTAGE);
  bool currentSign   = (i > CHARGING_CURRENT_MIN);
  bool dischargeSign = (i < -DISCHARGE_CURRENT_MIN);

  if (voltageSign || currentSign) {
    chargingDetected = true;
  } else if (!voltageSign && !currentSign) {
    chargingDetected = false;
  }

  float capAh = BATTERY_CAP_AH * (soc / 100.0f);

  if (chargingDetected) {
    float chargeAmps = (i > CHARGING_CURRENT_MIN) ? i : 1.0f;
    timeToFull  = (int)(((BATTERY_CAP_AH - capAh) / chargeAmps) * 60.0f);
    timeToEmpty = 0;
    if (soc >= 99) {
      vehicleStatus = "Fully Charged";
      setLED(COLOR_GREEN, LED_SOLID);
    } else {
      vehicleStatus = "Charging";
      setLED(COLOR_GREEN, LED_PULSE);
    }
  } else if (dischargeSign) {
    timeToEmpty   = (int)((capAh / fabsf(i)) * 60.0f);
    timeToFull    = 0;
    vehicleStatus = "Discharging";
    setLED(COLOR_ORANGE, LED_PULSE);
  } else if (soc <= 10) {
    vehicleStatus = "Low Battery";
    setLED(COLOR_RED, LED_BLINK_FAST);
    timeToFull = timeToEmpty = 0;
  } else {
    vehicleStatus = "Idle";
    setLED(COLOR_GREEN, LED_SOLID);
    timeToFull = timeToEmpty = 0;
  }
}

// ================================================================
//  MQTT PUBLISH — only the 10 parameters the dashboard uses
// ================================================================
void publishTelemetry() {
  if (!mqttClient.connected()) return;

  char buf[256];
  snprintf(buf, sizeof(buf),
    "{"
    "\"vehicle_id\":\"v3\","
    "\"battery_cap_ah\":2.2,"
    "\"voltage\":%.3f,"
    "\"current\":%.4f,"
    "\"power\":%.3f,"
    "\"soc\":%d,"
    "\"status\":\"%s\","
    "\"charging_detected\":%s,"
    "\"time_to_full\":%d,"
    "\"time_to_empty\":%d"
    "}",
    smoothedVoltage,
    smoothedCurrent,
    fabsf(power_W),
    soc,
    vehicleStatus.c_str(),
    chargingDetected ? "true" : "false",
    timeToFull,
    timeToEmpty
  );

  if (mqttClient.publish(TOPIC_TELEMETRY, buf)) {
    flashLED(COLOR_CYAN);
    Serial.printf("[MQTT] Published → V:%.3fV  I:%.2fmA  SOC:%d%%  %s\n",
      smoothedVoltage, smoothedCurrent * 1000.0f, soc, vehicleStatus.c_str());
  } else {
    Serial.println("[MQTT] Publish FAILED");
    setLED(COLOR_RED, LED_BLINK_FAST);
  }
}

// ================================================================
//  SERIAL DASHBOARD HELPERS
// ================================================================
void printRow(const char* label, float val, int dec, const char* unit) {
  Serial.print(F("│  "));
  Serial.print(label);
  int len = strlen(label);
  for (int k = len; k < 18; k++) Serial.print(' ');
  Serial.print(F(": "));
  Serial.print(val, dec);
  Serial.print(' ');
  Serial.println(unit);
}

void printRowStr(const char* label, const String& val) {
  Serial.print(F("│  "));
  Serial.print(label);
  int len = strlen(label);
  for (int k = len; k < 18; k++) Serial.print(' ');
  Serial.print(F(": "));
  Serial.println(val);
}

void printDivider() {
  Serial.println(F("├─────────────────────────────────────────────────────────────┤"));
}

void printHeader(const char* title) {
  Serial.print(F("│  "));
  Serial.println(title);
}

// ================================================================
//  SERIAL DASHBOARD
// ================================================================
void printDashboard() {
  Serial.println();
  Serial.println(F("╔═════════════════════════════════════════════════════════════╗"));
  Serial.println(F("║      ⚡ EV-NODE v3  |  INA219 + HiveMQ  |  ESP32-C3        ║"));
  Serial.println(F("║             5.0Ah  |  WiFi + MQTT Telemetry                ║"));
  Serial.println(F("╚═════════════════════════════════════════════════════════════╝"));

  // DIRECT READINGS
  Serial.println(F("┌─────────────────────────────────────────────────────────────┐"));
  printHeader("📊  DIRECT READINGS");
  printDivider();
  printRow("Bus Voltage",    busVoltage_V,    3, "V");
  printRow("Shunt Voltage",  shuntVoltage_mV, 3, "mV");
  Serial.print(F("│  Current         : "));
  Serial.print(current_mA, 2);
  Serial.print(F(" mA  ("));
  Serial.print(current_A, 4);
  Serial.println(F(" A)"));
  Serial.print(F("│  Power           : "));
  Serial.print(power_mW, 2);
  Serial.print(F(" mW  ("));
  Serial.print(power_W, 4);
  Serial.println(F(" W)"));

  // CALCULATED VALUES
  printDivider();
  printHeader("🧮  CALCULATED VALUES");
  printDivider();
  printRow("Load Voltage",    loadVoltage_V,              3, "V  (Bus + Shunt)");
  printRow("Smoothed Voltage",smoothedVoltage,            3, "V");
  printRow("Smoothed Current",smoothedCurrent * 1000.0f,  2, "mA");
  printRow("Shunt Resistance",shuntResistance,            3, "mΩ  (ideal=100)");
  printRow("Voltage Drop",    voltageDrop,                3, "V");
  printRow("Efficiency",      efficiency,                 2, "%");

  // STATISTICS
  printDivider();
  printHeader("📈  STATISTICS & TRACKING");
  printDivider();
  printRow("Max Current",   maxCurrent_A * 1000.0f, 2, "mA");
  printRow("Min Current",   minCurrent_A * 1000.0f, 2, "mA");
  printRow("Avg Current",   avgCurrent_A * 1000.0f, 2, "mA");
  printRow("Peak Power",    peakPower_W,             3, "W");
  printRow("Total Energy",  totalEnergy_Wh,          4, "Wh");
  Serial.print(F("│  Runtime         : "));
  Serial.print((int)runtimeHours);
  Serial.print(F("h "));
  Serial.print((int)fmod(runtimeMinutes, 60.0f));
  Serial.print(F("m "));
  Serial.print((int)fmod(runtimeSeconds, 60.0f));
  Serial.println(F("s"));

  // BATTERY STATUS
  printDivider();
  printHeader("🔋  BATTERY STATUS");
  printDivider();
  printRow("SOC",              (float)soc,      0, "%");
  printRow("Voltage (smooth)", smoothedVoltage, 3, "V");

  Serial.print(F("│  SOC Bar         : ["));
  int filled = soc / 5;
  for (int k = 0; k < 20; k++) Serial.print(k < filled ? '█' : '░');
  Serial.print(F("] "));
  Serial.print(soc);
  Serial.println(F("%"));

  if (chargingDetected && timeToFull > 0)   printRow("Time to Full",  (float)timeToFull,  0, "min");
  if (!chargingDetected && timeToEmpty > 0) printRow("Time to Empty", (float)timeToEmpty, 0, "min");

  printRowStr("State",    vehicleStatus);
  printRowStr("Charging", chargingDetected ? "YES ✔" : "NO");

  // SYSTEM STATUS
  printDivider();
  printHeader("🖥  SYSTEM STATUS");
  printDivider();

  String wifiStatus = (WiFi.status() == WL_CONNECTED)
    ? ("Connected (" + WiFi.localIP().toString() + ")")
    : "Disconnected";
  String mqttStatus = mqttClient.connected() ? "Connected ✔" : "Disconnected";

  printRowStr("WiFi",     wifiStatus);
  printRowStr("MQTT",     mqttStatus);
  printRowStr("Sensor",   sensorOK ? "INA219 OK ✔" : "ERROR ✘");
  printRow("Free Heap",   (float)(ESP.getFreeHeap() / 1024.0f), 1, "KB");
  printRow("Uptime",      runtimeSeconds, 1, "s");

  Serial.println(F("└─────────────────────────────────────────────────────────────┘"));
  Serial.println();
}

// ================================================================
//  SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  // LED — white = booting
  led.begin();
  led.setBrightness(LED_BRIGHT);
  led.clear();
  led.show();
  setLED(COLOR_WHITE, LED_SOLID);

  Serial.println();
  Serial.println(F("╔══════════════════════════════════════════════════════════════╗"));
  Serial.println(F("║  ⚡ EV-NODE v3  |  INA219 + HiveMQ  |  ESP32-C3            ║"));
  Serial.println(F("║  INA219 SDA→GPIO4  SCL→GPIO5  LED→GPIO8                    ║"));
  Serial.println(F("╚══════════════════════════════════════════════════════════════╝"));

  if (! ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }
  
  ina219.setCalibration_32V_2A();
  // ─────────────────────────────────

  Serial.println(F("[v3] INA219 OK — 32V/2A calibration set"));

  // Capture baseline idle voltage
  delay(200);
  float sumV = 0;
  for (int k = 0; k < 10; k++) {
    sumV += ina219.getBusVoltage_V() + (ina219.getShuntVoltage_mV() / 1000.0f);
    delay(20);
  }
  noLoadVoltage = sumV / 10.0f;
  Serial.printf("[v3] No-load baseline: %.3fV\n", noLoadVoltage);

  // Init tracking
  minCurrent_A  = 999.0f;
  startTime     = millis();
  lastTime      = millis();
  lastPublish   = millis();
  sensorOK      = true;
  vehicleStatus = "Idle";

  // Connect WiFi then MQTT
  connectWiFi();
  connectMQTT();

  Serial.println(F("[v3] Ready — publishing every 2s\n"));
}

// ================================================================
//  LOOP
// ================================================================
void loop() {
  unsigned long now = millis();

  // ── WiFi watchdog ──
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("[WiFi] Lost — reconnecting..."));
    setLED(COLOR_RED, LED_BLINK_FAST);
    connectWiFi();
  }

  // ── MQTT watchdog ──
  if (!mqttClient.connected()) {
    connectMQTT();
  }
  mqttClient.loop();   // Keep MQTT alive + process inbound commands

  // ── Read sensor every 200ms ──
  static unsigned long lastRead = 0;
  if (now - lastRead >= 200) {
    lastRead = now;
    bool ok = readINA219();
    if (!ok && sensorFailCount >= SENSOR_FAIL_LIMIT) {
      sensorOK = false;
    }
  }

  // ── Dashboard + publish every 2s ──
  if (now - lastPublish >= 2000) {
    lastPublish = now;
    updateVehicleState();
    printDashboard();
    publishTelemetry();
  }

  updateLED();
  delay(10);
}

