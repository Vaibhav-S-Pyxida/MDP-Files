/**
 * SolarGuard — Control Flowchart & Thermal Cooling/Cleaning Firmware
 * Target Board : ESP32-S2-DevKitM-1 (Arduino Core for ESP32 v3.x)
 *
 * Control Loop Flowchart Execution:
 *   1. setup()            : Init pins, sensors, LCD 1602 (I2C), Serial, Wi-Fi/NTP
 *   2. Read sensors       : Panel & ambient temp, V/I taps (R1, R3, R5, R7), VBAT, water level
 *   3. Hot and water OK?  :
 *        - Yes -> Pump ON (U15 IN1 LOW -> K1 relay energizes -> water spray cooling)
 *        - No  -> Pump OFF (U15 IN1 HIGH)
 *   4. Cleaning due?      :
 *        - Yes -> Run cleaning sweep (U16 L298N motor forward down, then reverse up)
 *        - No  -> Skip sweep
 *   5. Log and display    : Output telemetry to USB Serial, I2C LCD 1602, and ThingSpeak Cloud
 */

#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <time.h>
#include <esp_task_wdt.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>

// ============================================================================
// 1. NETWORK, CLOUD & DISPLAY CONFIGURATION
// ============================================================================
const char* WIFI_SSID       = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD   = "YOUR_WIFI_PASSWORD";

const char* TS_UPDATE_URL   = "http://api.thingspeak.com/update";
const String TS_API_KEY     = "YOUR_THINGSPEAK_WRITE_API_KEY";

const char* NTP_SERVER      = "pool.ntp.org";
const long  GMT_OFFSET_SEC  = 19800; // UTC+5:30 (IST)
const int   DAYLIGHT_OFFSET = 0;

// LCD 1602 with I2C Backpack (Standard address 0x27 or 0x3F)
#define LCD_I2C_ADDR          0x27
#define LCD_COLS              16
#define LCD_ROWS              2
#define PIN_I2C_SDA           33
#define PIN_I2C_SCL           35

// ============================================================================
// 2. ESP32-S2-DEVKITM-1 GPIO PINOUT
// ============================================================================
// U15 Relay Module (Active-LOW inputs: LOW energizes relay coil)
#define PIN_U15_IN1_K1_PUMP   15  // LOW = Pump P1 ON, HIGH = Pump P1 OFF
#define PIN_U15_IN2_K2_MODE   16  // HIGH = NC (Charge via D1), LOW = NO (Measure RL1 15R)

// U16 L298N Motor Driver (ENA & 5V-EN jumpers removed; +5V from 5V rail)
#define PIN_U16_L298N_ENA     17  // 10 kHz PWM speed
#define PIN_U16_L298N_IN1     18  // IN1=H, IN2=L -> Forward (Down)
#define PIN_U16_L298N_IN2     21  // IN1=L, IN2=H -> Reverse (Up)

#define PWM_FREQ_HZ           10000
#define PWM_RES_BITS          8

// Sensing Layer — ADC1 (GPIO 1-7)
#define PIN_TAP_R1_VPANEL_A   1   // Panel A V tap (R1)
#define PIN_TAP_R3_VPANEL_B   2   // Panel B V tap (R3)
#define PIN_TAP_VBAT_SENSE    3   // VBAT rail divider (9.0 V - 12.6 V)
#define PIN_TAP_R5_IPANEL_A   4   // U4 ACS712-A OUT tap (R5)
#define PIN_TAP_R7_IPANEL_B   5   // ACS712-B OUT tap (R7)
#define PIN_REF_SOLAR_CELL    6   // 0.45 W Mini Solar Panel
#define PIN_RAIN_DETECTOR     7   // Rain / soil moisture sensor

// Digital & 1-Wire Sensors
#define PIN_DS18B20_BUS       8   // DS18B20 x3 panel probes (4.7 kOhm pull-up)
#define PIN_DHT22_DATA        9   // DHT22 ambient sensor
#define PIN_ENDSTOP_HOME      10  // Start/Home limit switch (Active LOW)
#define PIN_ENDSTOP_FAR       11  // End/Far limit switch (Active LOW)
#define PIN_LIQUID_LEVEL      12  // Optical water level sensor

// ============================================================================
// 3. CALIBRATION & THRESHOLDS
// ============================================================================
#define WDT_TIMEOUT_S         20

const float RATIO_R1_PANELA   = 5.54;
const float RATIO_R3_PANELB   = 5.54;
const float RATIO_VBAT        = 4.03;
const float ADC_VREF          = 3.30;
const float ADC_RESOLUTION    = 4095.0;

const float ACS_ZERO_OFFSET_V = 1.667; // 5V ACS712 scaled via 10k/20k divider
const float ACS_SENSITIVITY   = 0.044; // 44 mV/A scaled sensitivity

// Decision Thresholds for Flowchart Branches
const float HOT_PANEL_ON_C    = 45.0;  // Spray cooling turns ON above 45.0 C
const float HOT_PANEL_OFF_C   = 40.0;  // Spray cooling turns OFF below 40.0 C (hysteresis)
const float VBAT_MIN_SAFE     = 9.6;   // Minimum safe 3S battery voltage
const float MIN_SUNLIGHT_V    = 0.80;  // Daylight check via 0.45 W ref cell
const float SOILING_LOSS_PCT  = 15.0;  // Power loss threshold vs Panel B (RL2)
const int   RAIN_ADC_LIMIT    = 2000;

const unsigned long SWEEP_TIMEOUT_MS    = 30000UL;
const unsigned long EVAL_INTERVAL_MS    = 900000UL; // 15 min A/B soiling check
const unsigned long CLOUD_LOG_PERIOD_MS = 60000UL;  // 60 s ThingSpeak interval

// ============================================================================
// 4. GLOBAL OBJECTS & TELEMETRY
// ============================================================================
OneWire oneWire(PIN_DS18B20_BUS);
DallasTemperature panelTempSensors(&oneWire);
DHT ambientDht(PIN_DHT22_DATA, DHT22);
LiquidCrystal_I2C lcd(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);

struct Telemetry {
  float vPanelA, iPanelA, wPanelA;
  float vPanelB, iPanelB, wPanelB;
  float vBat;
  float tPanelMax;
  float tAmbient, hAmbient;
  float sunRefV;
  bool raining;
  bool waterOK;
  bool atHomeStop;
  bool atFarStop;
  bool pumpActive;
  bool sweepRanThisCycle;
} telem;

bool coolingSprayLatched     = false;
int  scheduledCleanDoneToday = 0;
unsigned long lastEvalMs     = 0;
unsigned long lastCloudMs    = 0;

// ============================================================================
// 5. HARDWARE ACTUATOR HELPERS
// ============================================================================
void setPumpK1(bool turnOn) {
  // U15 Active-LOW relay: LOW energizes coil, HIGH turns pump off
  digitalWrite(PIN_U15_IN1_K1_PUMP, turnOn ? LOW : HIGH);
  telem.pumpActive = turnOn;
}

void setModeK2MeasureRL1(bool measureMode) {
  // HIGH = NC (Charge Mode via D1), LOW = NO (Measure Mode RL1 15R)
  digitalWrite(PIN_U15_IN2_K2_MODE, measureMode ? LOW : HIGH);
}

void setMotorM1(uint8_t pwmDuty, int dir) {
  // dir: 1 = Forward (Down), -1 = Reverse (Up), 0 = Stop
  if (dir == 1) {
    digitalWrite(PIN_U16_L298N_IN1, HIGH);
    digitalWrite(PIN_U16_L298N_IN2, LOW);
    ledcWrite(PIN_U16_L298N_ENA, pwmDuty);
  } else if (dir == -1) {
    digitalWrite(PIN_U16_L298N_IN1, LOW);
    digitalWrite(PIN_U16_L298N_IN2, HIGH);
    ledcWrite(PIN_U16_L298N_ENA, pwmDuty);
  } else {
    digitalWrite(PIN_U16_L298N_IN1, LOW);
    digitalWrite(PIN_U16_L298N_IN2, LOW);
    ledcWrite(PIN_U16_L298N_ENA, 0);
  }
}

// ============================================================================
// 6. STEP 2: READ SENSORS (TEMP, VOLTAGE, WATER LEVEL)
// ============================================================================
float readVoltageADC(int pin, float ratio) {
  uint32_t sum = 0;
  for (int i = 0; i < 16; i++) {
    sum += analogRead(pin);
    delay(1);
  }
  return ((sum / 16.0f) / ADC_RESOLUTION) * ADC_VREF * ratio;
}

float readCurrentADC(int pin) {
  uint32_t sum = 0;
  for (int i = 0; i < 32; i++) {
    sum += analogRead(pin);
    delay(1);
  }
  float vOut = ((sum / 32.0f) / ADC_RESOLUTION) * ADC_VREF;
  float amps = (vOut - ACS_ZERO_OFFSET_V) / ACS_SENSITIVITY;
  return (fabs(amps) < 0.08f) ? 0.0f : amps;
}

void readSensors() {
  // Voltages & Currents
  telem.vPanelA = readVoltageADC(PIN_TAP_R1_VPANEL_A, RATIO_R1_PANELA);
  telem.iPanelA = readCurrentADC(PIN_TAP_R5_IPANEL_A);
  telem.wPanelA = telem.vPanelA * telem.iPanelA;

  telem.vPanelB = readVoltageADC(PIN_TAP_R3_VPANEL_B, RATIO_R3_PANELB);
  telem.iPanelB = readCurrentADC(PIN_TAP_R7_IPANEL_B);
  telem.wPanelB = telem.vPanelB * telem.iPanelB;

  telem.vBat    = readVoltageADC(PIN_TAP_VBAT_SENSE, RATIO_VBAT);

  // Panel Temperatures (DS18B20 x3) & Ambient (DHT22)
  panelTempSensors.requestTemperatures();
  float maxT = -100.0f;
  for (int i = 0; i < 3; i++) {
    float t = panelTempSensors.getTempCByIndex(i);
    if (t != DEVICE_DISCONNECTED_C && t > maxT) {
      maxT = t;
    }
  }
  telem.tPanelMax = (maxT > -50.0f) ? maxT : 25.0f;

  float h = ambientDht.readHumidity();
  float tAmb = ambientDht.readTemperature();
  if (!isnan(h)) telem.hAmbient = h;
  if (!isnan(tAmb)) telem.tAmbient = tAmb;

  // Environmental, Water Level & End-Stops
  telem.sunRefV    = readVoltageADC(PIN_REF_SOLAR_CELL, 1.0f);
  telem.raining    = (analogRead(PIN_RAIN_DETECTOR) < RAIN_ADC_LIMIT);
  telem.waterOK    = (digitalRead(PIN_LIQUID_LEVEL) == HIGH);
  telem.atHomeStop = (digitalRead(PIN_ENDSTOP_HOME) == LOW);
  telem.atFarStop  = (digitalRead(PIN_ENDSTOP_FAR) == LOW);
}

// ============================================================================
// 7. STEP 3 & 4: DECISION LOGIC & ACTUATION
// ============================================================================
bool isCleaningDue() {
  // Skip cleaning if raining, dark, or battery low
  if (telem.raining || telem.sunRefV < MIN_SUNLIGHT_V || telem.vBat < (VBAT_MIN_SAFE + 0.5f)) {
    return false;
  }

  // Trigger A: Daily scheduled clean at 08:00 AM
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    if (timeinfo.tm_hour == 8 && timeinfo.tm_min == 0 && scheduledCleanDoneToday == 0) {
      scheduledCleanDoneToday = 1;
      return true;
    }
    if (timeinfo.tm_hour == 0) scheduledCleanDoneToday = 0;
  }

  // Trigger B: Periodic matched-load A/B test via K2 (RL1 15R vs Panel B RL2 15R)
  if (millis() - lastEvalMs >= EVAL_INTERVAL_MS) {
    lastEvalMs = millis();
    setModeK2MeasureRL1(true); // Switch K2 to NO (RL1)
    delay(1200);
    float pA_test = readVoltageADC(PIN_TAP_R1_VPANEL_A, RATIO_R1_PANELA) * readCurrentADC(PIN_TAP_R5_IPANEL_A);
    float pB_ctrl = readVoltageADC(PIN_TAP_R3_VPANEL_B, RATIO_R3_PANELB) * readCurrentADC(PIN_TAP_R7_IPANEL_B);
    setModeK2MeasureRL1(false); // Restore K2 to NC (Charge mode)

    if (pB_ctrl > 3.0f) {
      float lossPct = ((pB_ctrl - pA_test) / pB_ctrl) * 100.0f;
      if (lossPct >= SOILING_LOSS_PCT) {
        return true;
      }
    }
  }
  return false;
}

void runCleaningSweep() {
  Serial.println("[SWEEP] Starting motor forward, then back...");
  telem.sweepRanThisCycle = true;

  // Ensure carriage starts from Home stop
  unsigned long t0 = millis();
  while (digitalRead(PIN_ENDSTOP_HOME) != LOW && (millis() - t0 < SWEEP_TIMEOUT_MS)) {
    esp_task_wdt_reset();
    setMotorM1(190, -1); // Reverse (Up) to Home
    delay(20);
  }
  setMotorM1(0, 0);
  delay(300);

  // 1. Motor Forward (Down) with water spray if tank water is OK
  if (telem.waterOK) setPumpK1(true);
  t0 = millis();
  while (digitalRead(PIN_ENDSTOP_FAR) != LOW && (millis() - t0 < SWEEP_TIMEOUT_MS)) {
    esp_task_wdt_reset();
    setMotorM1(225, 1); // Forward (Down)
    delay(20);
  }
  setMotorM1(0, 0);
  setPumpK1(false);
  delay(400);

  // 2. Motor Reverse (Up) back to Home
  t0 = millis();
  while (digitalRead(PIN_ENDSTOP_HOME) != LOW && (millis() - t0 < SWEEP_TIMEOUT_MS)) {
    esp_task_wdt_reset();
    setMotorM1(225, -1); // Reverse (Up)
    delay(20);
  }
  setMotorM1(0, 0);
  Serial.println("[SWEEP] Completed.");
}

// ============================================================================
// 8. STEP 5: LOG AND DISPLAY (SERIAL, LCD 1602, CLOUD)
// ============================================================================
void logAndDisplay() {
  // A. Serial Console Log
  Serial.printf("T_Panel: %.1fC | VBAT: %.2fV | Water: %s | Pump: %s | PA: %.1fW | PB: %.1fW\n",
                telem.tPanelMax, telem.vBat,
                telem.waterOK ? "OK" : "LOW",
                telem.pumpActive ? "ON" : "OFF",
                telem.wPanelA, telem.wPanelB);

  // B. LCD 1602 I2C Status Display
  lcd.setCursor(0, 0);
  lcd.printf("T:%.1fC B:%.1fV  ", telem.tPanelMax, telem.vBat);
  lcd.setCursor(0, 1);
  lcd.printf("W:%s PMP:%s    ", telem.waterOK ? "OK " : "LOW", telem.pumpActive ? "ON " : "OFF");

  // C. ThingSpeak Cloud Upload (every 60 seconds)
  if (WiFi.status() == WL_CONNECTED && (millis() - lastCloudMs >= CLOUD_LOG_PERIOD_MS)) {
    lastCloudMs = millis();
    HTTPClient http;
    String url = String(TS_UPDATE_URL) + "?api_key=" + TS_API_KEY +
                 "&field1=" + String(telem.wPanelA, 2) +
                 "&field2=" + String(telem.wPanelB, 2) +
                 "&field3=" + String(telem.vBat, 2) +
                 "&field4=" + String(telem.tPanelMax, 1) +
                 "&field5=" + String(telem.waterOK ? 1 : 0) +
                 "&field6=" + String(telem.pumpActive ? 1 : 0) +
                 "&field7=" + String(telem.sweepRanThisCycle ? 1 : 0);
    http.begin(url);
    http.GET();
    http.end();
  }
}

// ============================================================================
// 9. STEP 1: SETUP() — INIT PINS, SENSORS, SERIAL, LCD
// ============================================================================
void setup() {
  Serial.begin(115200);

  // Safe default states for U15 Active-LOW relays
  digitalWrite(PIN_U15_IN1_K1_PUMP, HIGH); // Pump OFF
  digitalWrite(PIN_U15_IN2_K2_MODE, HIGH); // Normal Charge Mode (NC)
  pinMode(PIN_U15_IN1_K1_PUMP, OUTPUT);
  pinMode(PIN_U15_IN2_K2_MODE, OUTPUT);

  // U16 L298N Motor Pins & 10 kHz PWM
  pinMode(PIN_U16_L298N_IN1, OUTPUT);
  pinMode(PIN_U16_L298N_IN2, OUTPUT);
  digitalWrite(PIN_U16_L298N_IN1, LOW);
  digitalWrite(PIN_U16_L298N_IN2, LOW);
  ledcAttach(PIN_U16_L298N_ENA, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcWrite(PIN_U16_L298N_ENA, 0);

  // Digital Input Pins
  pinMode(PIN_ENDSTOP_HOME, INPUT_PULLUP);
  pinMode(PIN_ENDSTOP_FAR, INPUT_PULLUP);
  pinMode(PIN_LIQUID_LEVEL, INPUT_PULLUP);

  // Initialize I2C LCD 1602 & Temperature Sensors
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("SolarGuard Init ");

  panelTempSensors.begin();
  ambientDht.begin();

  // Wi-Fi & NTP Time Sync
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 20) {
    delay(500);
    tries++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET, NTP_SERVER);
  }

  esp_task_wdt_init(WDT_TIMEOUT_S, true);
  esp_task_wdt_add(NULL);
  Serial.println("SolarGuard setup() complete.");
}

// ============================================================================
// 10. MAIN CONTROL LOOP (MATCHES FLOWCHART SEQUENCE)
// ============================================================================
void loop() {
  esp_task_wdt_reset();
  telem.sweepRanThisCycle = false;

  // 1. Read sensors (Temp, voltage, water level)
  readSensors();

  // 2. Decision: Hot and water OK?
  if (telem.tPanelMax >= HOT_PANEL_ON_C) {
    coolingSprayLatched = true;
  } else if (telem.tPanelMax <= HOT_PANEL_OFF_C) {
    coolingSprayLatched = false;
  }

  if (coolingSprayLatched && telem.waterOK && telem.vBat > VBAT_MIN_SAFE) {
    // Yes -> Pump ON (Relay K1 on, water spray cooling)
    setPumpK1(true);
  } else {
    // No -> Pump OFF
    setPumpK1(false);
  }

  // 3. Decision: Cleaning due?
  if (isCleaningDue()) {
    // Yes -> Run cleaning sweep (Motor forward, then back)
    runCleaningSweep();
  }
  // No -> Skip

  // 4. Log and display (Serial, LCD, Cloud)
  logAndDisplay();

  // 5. Repeat loop
  delay(1000);
}