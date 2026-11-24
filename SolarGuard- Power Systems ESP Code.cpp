/**
 * SolarGuard — Automatic Solar Panel Cleaner & Power Path Firmware
 * Target MCU: ESP32-S2-DevKitM-1 (Arduino Framework)
 * Power Path Architecture:
 *   - Panel A (Test) -> F1 (3A) -> ACS712-A -> K2 Relay:
 *       * NC (Default/Low): D1 (SR560) -> Charger PV in -> U2 BMS (3S)
 *       * NO (Active/High): RL1 (15 Ohm / 25 W dummy load)
 *   - Panel B (Control) -> F2 (3A) -> ACS712-B -> RL2 (15 Ohm / 25 W dummy load)
 *   - VBAT Rail (9.0V - 12.6V) -> S1 Master Sw -> F3 (5A) -> U3 LM2596 (5.00V), F4 (K1/P1), F5 (U16/M1)
 */

#include <WiFi.h>
#include <HTTPClient.h>
#include <time.h>
#include <esp_task_wdt.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>

// ==========================================
// 1. NETWORK & CLOUD CONFIGURATION
// ==========================================
const char* WIFI_SSID     = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

const char* TS_SERVER     = "http://api.thingspeak.com/update";
const String TS_API_KEY   = "YOUR_THINGSPEAK_WRITE_API_KEY";

const char* NTP_SERVER    = "pool.ntp.org";
const long  GMT_OFFSET    = 19800; // UTC+5:30 (IST)
const int   DAYLIGHT_OFF  = 0;

// ==========================================
// 2. PIN DEFINITIONS (ESP32-S2-DevKitM-1)
// ==========================================
// Actuation Layer
#define PIN_PUMP_RELAY_K1 15  // K1: F4 (2A) -> Pump P1 + D2 Flyback
#define PIN_MODE_RELAY_K2 16  // K2: LOW = NC (Charge via D1), HIGH = NO (Test Load RL1)
#define PIN_L298N_IN1     17  // U16 Motor Driver IN1 (F5 2A branch)
#define PIN_L298N_IN2     18  // U16 Motor Driver IN2
#define PIN_L298N_ENA     21  // U16 Motor Driver ENA (PWM)

// Sensing Layer (ADC1 Channels GPIO 1-7: Wi-Fi Safe)
#define PIN_TAP_V_PANEL_A 1   // V tap (R1) post-F1 (Panel A Test)
#define PIN_TAP_V_PANEL_B 2   // V tap (R3) post-F2 (Panel B Control)
#define PIN_TAP_VBAT      3   // VBAT rail divider (9.0V - 12.6V post-S1/F3)
#define PIN_TAP_I_PANEL_A 4   // OUT tap (R5) from ACS712-A
#define PIN_TAP_I_PANEL_B 5   // OUT tap (R7) from ACS712-B
#define PIN_REF_CELL      6   // Mini solar cell (0.45W irradiance reference)
#define PIN_RAIN_PROBE    7   // Soil moisture / rain probe

// Environmental & Limit Sensors
#define PIN_ONE_WIRE      8   // DS18B20 x3 Bus (with 4.7k pull-up)
#define PIN_DHT22         9   // DHT22 Ambient Temp & Humidity
#define PIN_ENDSTOP_START 10  // Start/Home limit switch (Active LOW)
#define PIN_ENDSTOP_END   11  // End/Far limit switch (Active LOW)
#define PIN_WATER_LEVEL   12  // Optical liquid level sensor

// ==========================================
// 3. CALIBRATION & SAFETY THRESHOLDS
// ==========================================
#define WDT_TIMEOUT_SEC   15

// Voltage Divider Ratios for V taps (R1, R3) and VBAT
const float V_DIV_R1_PANELA = 5.54;  // Adjust to match R1 divider network
const float V_DIV_R3_PANELB = 5.54;  // Adjust to match R3 divider network
const float V_DIV_VBAT      = 4.03;  // Maps 12.6V max -> ~3.12V ADC
const float ADC_REF_V       = 3.30;
const float ADC_MAX         = 4095.0;

// ACS712-A / ACS712-B via OUT taps (R5, R7)
// Note: 5V ACS712 zero-current (2.50V) scaled by 10k/20k divider = 1.667V
const float ACS_ZERO_V      = 1.667;
const float ACS_SENS_SCALED = 0.044; // 66mV/A (30A module) * (20k / 30k) = 44mV/A

// Operational & Safety Limits
const float VBAT_MIN_SAFE       = 9.6;    // Software cutoff above 9.0V U2 BMS trip
const float VBAT_CHARGE_RESUME  = 11.4;   // Hysteresis recovery threshold
const float MAX_PANEL_TEMP_C    = 65.0;   // Thermal shock protection limit
const float MIN_IRRADIANCE_V    = 0.8;    // Minimum daylight threshold
const float DUMMY_LOAD_OHMS     = 15.0;   // RL1 & RL2 = 15 Ohm, 25 W resistors
const float SOILING_DROP_PCT    = 15.0;   // Trigger clean if Panel A lags Panel B by >= 15%
const int   RAIN_WET_THRESHOLD  = 2000;
const unsigned long MOTOR_TIMEOUT_MS   = 45000;
const unsigned long TEST_SETTLE_MS     = 1500;  // Settle time on RL1 before reading
const unsigned long LOG_INTERVAL_MS    = 60000;

// ==========================================
// 4. STATE MACHINE & TELEMETRY STRUCTURES
// ==========================================
OneWire oneWire(PIN_ONE_WIRE);
DallasTemperature ds18b20(&oneWire);
DHT dht(PIN_DHT22, DHT22);

enum SystemState {
  STATE_IDLE_CHARGING,   // K2 = LOW (NC -> D1 -> Charger)
  STATE_SOILING_EVAL,    // K2 = HIGH (NO -> RL1 15R dummy load comparison)
  STATE_HOMING,          // Returning carriage to Start end-stop
  STATE_CLEAN_FORWARD,   // K1 Pump ON + Forward wipe
  STATE_CLEAN_RETURN,    // K1 Pump OFF + Return squeegee pass
  STATE_FAULT            // Safe shutdown state
};

SystemState currentState = STATE_IDLE_CHARGING;
String faultReason = "NONE";

struct Telemetry {
  float vPanelA, iPanelA, pPanelA;
  float vPanelB, iPanelB, pPanelB;
  float vBat;
  float tempPanels[3];
  float ambientTemp, ambientHum;
  float irradianceV;
  bool isRaining;
  bool waterLevelOK;
  bool atStartStop;
  bool atEndStop;
  bool k2TestLoadActive;
} data;

unsigned long stateStartTime = 0;
unsigned long lastCloudLog   = 0;
unsigned long lastEvalTime   = 0;
int dailyCleanCount          = 0;

// ==========================================
// 5. ACTUATOR & RELAY ROUTING CONTROLS
// ==========================================
void setMotorM1(int speedPWM, int dir) {
  if (dir == 1) {
    digitalWrite(PIN_L298N_IN1, HIGH);
    digitalWrite(PIN_L298N_IN2, LOW);
    analogWrite(PIN_L298N_ENA, speedPWM);
  } else if (dir == -1) {
    digitalWrite(PIN_L298N_IN1, LOW);
    digitalWrite(PIN_L298N_IN2, HIGH);
    analogWrite(PIN_L298N_ENA, speedPWM);
  } else {
    digitalWrite(PIN_L298N_IN1, LOW);
    digitalWrite(PIN_L298N_IN2, LOW);
    analogWrite(PIN_L298N_ENA, 0);
  }
}

void setPumpK1(bool active) {
  // F4 (2A) -> K1 -> Pump P1 (protected by flyback diode D2)
  digitalWrite(PIN_PUMP_RELAY_K1, active ? HIGH : LOW);
}

void setPanelARouteK2(bool routeToDummyLoadRL1) {
  // LOW  = K2 COM->NC -> D1 (SR560) -> Charger PV in (Battery Charging)
  // HIGH = K2 COM->NO -> RL1 (15 Ohm / 25 W matched test load)
  digitalWrite(PIN_MODE_RELAY_K2, routeToDummyLoadRL1 ? HIGH : LOW);
  data.k2TestLoadActive = routeToDummyLoadRL1;
}

void emergencyStop(String reason) {
  setMotorM1(0, 0);
  setPumpK1(false);
  setPanelARouteK2(false); // Revert Panel A to safe charging path (NC)
  faultReason = reason;
  currentState = STATE_FAULT;
  Serial.println("[FAULT] " + reason);
}

void transitionTo(SystemState newState) {
  currentState = newState;
  stateStartTime = millis();
  Serial.printf("[STATE] -> %d\n", newState);
}

// ==========================================
// 6. SENSOR ACQUISITION (R1, R3, R5, R7 TAPS)
// ==========================================
float readTapVoltage(int pin, float ratio) {
  long sum = 0;
  for (int i = 0; i < 16; i++) {
    sum += analogRead(pin);
    delay(1);
  }
  return ((sum / 16.0) / ADC_MAX) * ADC_REF_V * ratio;
}

float readTapCurrent(int pin) {
  long sum = 0;
  for (int i = 0; i < 32; i++) {
    sum += analogRead(pin);
    delay(1);
  }
  float vOut = ((sum / 32.0) / ADC_MAX) * ADC_REF_V;
  float amps = (vOut - ACS_ZERO_V) / ACS_SENS_SCALED;
  return (abs(amps) < 0.08) ? 0.0 : amps;
}

void readAllSensors() {
  // Voltage & Current taps post-F1 and post-F2
  data.vPanelA = readTapVoltage(PIN_TAP_V_PANEL_A, V_DIV_R1_PANELA);
  data.iPanelA = readTapCurrent(PIN_TAP_I_PANEL_A);
  data.pPanelA = data.vPanelA * data.iPanelA;

  data.vPanelB = readTapVoltage(PIN_TAP_V_PANEL_B, V_DIV_R3_PANELB);
  data.iPanelB = readTapCurrent(PIN_TAP_I_PANEL_B);
  data.pPanelB = data.vPanelB * data.iPanelB;

  data.vBat    = readTapVoltage(PIN_TAP_VBAT, V_DIV_VBAT);

  // Temperatures
  ds18b20.requestTemperatures();
  for (int i = 0; i < 3; i++) {
    float t = ds18b20.getTempCByIndex(i);
    data.tempPanels[i] = (t == DEVICE_DISCONNECTED_C) ? 25.0 : t;
  }
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (!isnan(h)) data.ambientHum = h;
  if (!isnan(t)) data.ambientTemp = t;

  // Irradiance, Rain, Water Level & End-stops
  data.irradianceV  = readTapVoltage(PIN_REF_CELL, 1.0);
  data.isRaining    = (analogRead(PIN_RAIN_PROBE) < RAIN_WET_THRESHOLD);
  data.atStartStop  = (digitalRead(PIN_ENDSTOP_START) == LOW);
  data.atEndStop    = (digitalRead(PIN_ENDSTOP_END) == LOW);
  data.waterLevelOK = (digitalRead(PIN_WATER_LEVEL) == HIGH);
}

// ==========================================
// 7. SAFETY RULES & FUSE / LOAD DIAGNOSTICS
// ==========================================
bool checkSafetyRules() {
  // 1. VBAT Undervoltage Guard (Protects 3S pack before U2 BMS 9.0V cutoff)
  if (data.vBat < VBAT_MIN_SAFE && data.vBat > 2.0) {
    emergencyStop("VBAT_UNDERVOLTAGE_LOCKOUT");
    return false;
  }

  // 2. Thermal Shock Guard (Prevent cold spray on overheated glass)
  float maxT = max(data.tempPanels[0], max(data.tempPanels[1], data.tempPanels[2]));
  if (maxT > MAX_PANEL_TEMP_C && (currentState == STATE_CLEAN_FORWARD || currentState == STATE_CLEAN_RETURN)) {
    emergencyStop("THERMAL_SHOCK_PREVENTED");
    return false;
  }

  // 3. Dry-Run Pump Protection (F4 -> K1 -> P1)
  if (!data.waterLevelOK && currentState == STATE_CLEAN_FORWARD) {
    setPumpK1(false);
    Serial.println("[WARN] Tank low — K1 Pump P1 disabled.");
  }

  // 4. Motor M1 Travel Timeout (F5 -> U16 -> M1)
  if ((currentState == STATE_HOMING || currentState == STATE_CLEAN_FORWARD || currentState == STATE_CLEAN_RETURN) &&
      (millis() - stateStartTime > MOTOR_TIMEOUT_MS)) {
    emergencyStop("M1_TRAVEL_TIMEOUT_OR_F5_BLOWN");
    return false;
  }

  return true;
}

// ==========================================
// 8. STATE MACHINE EXECUTION
// ==========================================
void runStateMachine() {
  switch (currentState) {
    case STATE_IDLE_CHARGING:
      setMotorM1(0, 0);
      setPumpK1(false);
      setPanelARouteK2(false); // Panel A charges 3S pack through D1 (SR560)

      // Check scheduled 08:00 AM clean
      struct tm timeinfo;
      if (getLocalTime(&timeinfo)) {
        if (timeinfo.tm_hour == 8 && timeinfo.tm_min == 0 && dailyCleanCount == 0 && !data.isRaining) {
          dailyCleanCount++;
          transitionTo(data.atStartStop ? STATE_CLEAN_FORWARD : STATE_HOMING);
          break;
        }
        if (timeinfo.tm_hour == 0) dailyCleanCount = 0;
      }

      // Periodically switch K2 to RL1 (15R) every 15 mins during daylight to compare Panel A vs Panel B
      if (!data.isRaining && data.irradianceV >= MIN_IRRADIANCE_V && (millis() - lastEvalTime > 900000UL)) {
        lastEvalTime = millis();
        setPanelARouteK2(true); // Switch Panel A onto RL1 (15 Ohm) to match Panel B on RL2 (15 Ohm)
        transitionTo(STATE_SOILING_EVAL);
      }
      break;

    case STATE_SOILING_EVAL:
      // Wait for K2 relay contact bounce and current reading stabilization across RL1 & RL2
      if (millis() - stateStartTime >= TEST_SETTLE_MS) {
        float pA_matched = data.pPanelA; // Measured into RL1 (15 Ohm)
        float pB_matched = data.pPanelB; // Measured into RL2 (15 Ohm)
        setPanelARouteK2(false);         // Immediately restore Panel A charging path via D1

        if (pB_matched > 3.0) {
          float lossPct = ((pB_matched - pA_matched) / pB_matched) * 100.0;
          Serial.printf("[EVAL] Panel A (RL1): %.2fW | Panel B (RL2): %.2fW | Loss: %.1f%%\n",
                        pA_matched, pB_matched, lossPct);
          if (lossPct >= SOILING_DROP_PCT) {
            transitionTo(data.atStartStop ? STATE_CLEAN_FORWARD : STATE_HOMING);
            break;
          }
        }
        transitionTo(STATE_IDLE_CHARGING);
      }
      break;

    case STATE_HOMING:
      setPumpK1(false);
      setPanelARouteK2(false);
      setMotorM1(180, -1);
      if (data.atStartStop) {
        setMotorM1(0, 0);
        delay(400);
        transitionTo(STATE_CLEAN_FORWARD);
      }
      break;

    case STATE_CLEAN_FORWARD:
      setPanelARouteK2(false);
      if (data.waterLevelOK) setPumpK1(true);
      setMotorM1(220, 1);
      if (data.atEndStop) {
        setMotorM1(0, 0);
        setPumpK1(false);
        delay(400);
        transitionTo(STATE_CLEAN_RETURN);
      }
      break;

    case STATE_CLEAN_RETURN:
      setPanelARouteK2(false);
      setPumpK1(false); // Squeegee dry pass
      setMotorM1(220, -1);
      if (data.atStartStop) {
        setMotorM1(0, 0);
        transitionTo(STATE_IDLE_CHARGING);
      }
      break;

    case STATE_FAULT:
      setMotorM1(0, 0);
      setPumpK1(false);
      setPanelARouteK2(false);
      if (faultReason == "VBAT_UNDERVOLTAGE_LOCKOUT" && data.vBat >= VBAT_CHARGE_RESUME) {
        faultReason = "NONE";
        transitionTo(STATE_IDLE_CHARGING);
      }
      break;
  }
}

// ==========================================
// 9. CLOUD TELEMETRY LOGGER (THINGSPEAK)
// ==========================================
void logToCloud() {
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  String url = String(TS_SERVER) + "?api_key=" + TS_API_KEY +
               "&field1=" + String(data.pPanelA, 2) +
               "&field2=" + String(data.pPanelB, 2) +
               "&field3=" + String(data.vBat, 2) +
               "&field4=" + String(data.tempPanels[0], 1) +
               "&field5=" + String(data.ambientTemp, 1) +
               "&field6=" + String(data.irradianceV, 2) +
               "&field7=" + String(currentState) +
               "&field8=" + String(data.k2TestLoadActive ? 1 : 0);

  http.begin(url);
  http.GET();
  http.end();
}

// ==========================================
// 10. SETUP & MAIN LOOP
// ==========================================
void setup() {
  Serial.begin(115200);

  pinMode(PIN_PUMP_RELAY_K1, OUTPUT);
  pinMode(PIN_MODE_RELAY_K2, OUTPUT);
  pinMode(PIN_L298N_IN1, OUTPUT);
  pinMode(PIN_L298N_IN2, OUTPUT);
  pinMode(PIN_L298N_ENA, OUTPUT);

  emergencyStop("BOOT_INIT");
  faultReason = "NONE";
  currentState = STATE_IDLE_CHARGING;

  pinMode(PIN_ENDSTOP_START, INPUT_PULLUP);
  pinMode(PIN_ENDSTOP_END, INPUT_PULLUP);
  pinMode(PIN_WATER_LEVEL, INPUT_PULLUP);

  ds18b20.begin();
  dht.begin();

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 20) {
    delay(500);
    retries++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    configTime(GMT_OFFSET, DAYLIGHT_OFF, NTP_SERVER);
  }

  esp_task_wdt_init(WDT_TIMEOUT_SEC, true);
  esp_task_wdt_add(NULL);
}

void loop() {
  esp_task_wdt_reset();
  readAllSensors();

  if (checkSafetyRules()) {
    runStateMachine();
  }

  if (millis() - lastCloudLog >= LOG_INTERVAL_MS) {
    logToCloud();
    lastCloudLog = millis();
  }

  delay(100);
}