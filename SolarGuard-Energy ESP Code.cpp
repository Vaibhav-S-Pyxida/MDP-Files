/**
 * Automatic Solar Panel Cleaner Firmware
 * Target MCU: ESP32-S2 (Arduino Framework)
 * Architecture: State Machine + Safety Watchdog + ThingSpeak/NTP + Multi-Sensor Array
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

// ThingSpeak Configuration
const char* TS_SERVER     = "http://api.thingspeak.com/update";
const String TS_API_KEY   = "YOUR_THINGSPEAK_WRITE_API_KEY";

// NTP Time Configuration (UTC+5:30 example; adjust offset as needed)
const char* NTP_SERVER    = "pool.ntp.org";
const long  GMT_OFFSET    = 19800; 
const int   DAYLIGHT_OFF  = 0;

// ==========================================
// 2. PIN DEFINITIONS (ESP32-S2)
// ==========================================
// Actuation Layer (Explicitly defined in diagram)
#define PIN_PUMP_RELAY    15  // K1: Water Pump Relay
#define PIN_MODE_RELAY    16  // K2: Mode Switch Relay
#define PIN_L298N_IN1     17  // Motor Driver Direction 1
#define PIN_L298N_IN2     18  // Motor Driver Direction 2
#define PIN_L298N_ENA     21  // Motor Driver PWM Speed (ENA)

// Sensing Layer (ESP32-S2 ADC1/ADC2 & Digital GPIOs)
// Note: ADC1 is GPIO 1-10 on ESP32-S2 (safe to use alongside Wi-Fi)
#define PIN_VOLT_PANEL_A  1   // Voltage divider Panel A
#define PIN_VOLT_PANEL_B  2   // Voltage divider Panel B
#define PIN_VOLT_VBAT     3   // Voltage divider 3S Li-ion Battery (9.0V - 12.6V)
#define PIN_CURR_PANEL_A  4   // ACS712 #1
#define PIN_CURR_PANEL_B  5   // ACS712 #2
#define PIN_REF_CELL      6   // Reference solar cell (Irradiance/Light)
#define PIN_RAIN_PROBE    7   // Analog/Digital rain sensor

#define PIN_ONE_WIRE      8   // DS18B20 x3 Data Bus
#define PIN_DHT22         9   // DHT22 Ambient Temp/Humidity
#define PIN_ENDSTOP_START 10  // Limit switch: Home / Start position (Active LOW)
#define PIN_ENDSTOP_END   11  // Limit switch: Far / End position (Active LOW)
#define PIN_WATER_LEVEL   12  // Tank float switch (LOW = Empty, HIGH = OK)

// ==========================================
// 3. CALIBRATION & SAFETY THRESHOLDS
// ==========================================
#define WDT_TIMEOUT_SEC   15     // Watchdog timer limit

// Voltage Divider Ratios: (R1 + R2) / R2
const float V_DIV_PANEL   = 5.54;  // Adjust based on your resistors (e.g., 10k / 2.2k)
const float V_DIV_VBAT    = 4.03;  // Adjust for 12.6V max -> ~3.1V ADC
const float ADC_REF_V     = 3.30;
const float ADC_MAX       = 4095.0;

// ACS712 5A Module Calibration (185 mV/A, centered at Vcc/2)
const float ACS_ZERO_V    = 1.65;  // Adjust if powered by 5V with divider or 3.3V
const float ACS_SENS      = 0.185; 

// Safety & Operational Thresholds
const float VBAT_MIN_SAFE       = 9.6;    // Cutoff before 9.0V BMS hard shutoff
const float MAX_PANEL_TEMP_C    = 65.0;   // Thermal shock prevention (don't spray cold water on hot glass)
const float MIN_IRRADIANCE_V    = 0.8;    // Minimum daylight threshold to clean
const float EFFICIENCY_DROP_PCT = 15.0;   // Trigger clean if Panel A vs B power deviates > 15%
const int   RAIN_WET_THRESHOLD  = 2000;   // ADC value below this = raining
const unsigned long MOTOR_TIMEOUT_MS = 45000; // Max travel time before jam fault
const unsigned long LOG_INTERVAL_MS  = 60000; // Cloud upload interval

// ==========================================
// 4. OBJECTS & STATE MACHINE DEFINITIONS
// ==========================================
OneWire oneWire(PIN_ONE_WIRE);
DallasTemperature ds18b20(&oneWire);
DHT dht(PIN_DHT22, DHT22);

enum SystemState {
  STATE_IDLE,
  STATE_HOMING,
  STATE_CLEAN_FORWARD,
  STATE_CLEAN_RETURN,
  STATE_FAULT
};

SystemState currentState = STATE_IDLE;
String faultReason = "NONE";

// Sensor Telemetry Struct
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
} data;

unsigned long stateStartTime = 0;
unsigned long lastCloudLog   = 0;
int dailyCleanCount          = 0;

// ==========================================
// 5. HARDWARE ABSTRACTION & ACTUATORS
// ==========================================
void setMotor(int speedPWM, int dir) {
  // dir: 1 = Forward, -1 = Reverse, 0 = Stop
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

void setPump(bool state) {
  // Assumes Active-HIGH relay module; invert if Active-LOW
  digitalWrite(PIN_PUMP_RELAY, state ? HIGH : LOW);
}

void setModeRelay(bool active) {
  digitalWrite(PIN_MODE_RELAY, active ? HIGH : LOW);
}

void emergencyStop(String reason) {
  setMotor(0, 0);
  setPump(false);
  setModeRelay(false);
  faultReason = reason;
  currentState = STATE_FAULT;
  Serial.println("[FAULT] " + reason);
}

void transitionTo(SystemState newState) {
  currentState = newState;
  stateStartTime = millis();
  Serial.printf("[STATE] Transitioned to %d\n", newState);
}

// ==========================================
// 6. SENSOR READING LAYER
// ==========================================
float readVoltage(int pin, float dividerRatio) {
  long sum = 0;
  for (int i = 0; i < 16; i++) {
    sum += analogRead(pin);
    delay(1);
  }
  float adcV = ((sum / 16.0) / ADC_MAX) * ADC_REF_V;
  return adcV * dividerRatio;
}

float readCurrent(int pin) {
  long sum = 0;
  for (int i = 0; i < 32; i++) {
    sum += analogRead(pin);
    delay(1);
  }
  float adcV = ((sum / 32.0) / ADC_MAX) * ADC_REF_V;
  float current = (adcV - ACS_ZERO_V) / ACS_SENS;
  return (abs(current) < 0.05) ? 0.0 : current; // Deadband noise filter
}

void readAllSensors() {
  // 1. Power & Panel V/I
  data.vPanelA = readVoltage(PIN_VOLT_PANEL_A, V_DIV_PANEL);
  data.iPanelA = readCurrent(PIN_CURR_PANEL_A);
  data.pPanelA = data.vPanelA * data.iPanelA;

  data.vPanelB = readVoltage(PIN_VOLT_PANEL_B, V_DIV_PANEL);
  data.iPanelB = readCurrent(PIN_CURR_PANEL_B);
  data.pPanelB = data.vPanelB * data.iPanelB;

  data.vBat    = readVoltage(PIN_VOLT_VBAT, V_DIV_VBAT);

  // 2. Temperatures (DS18B20 x3 + DHT22)
  ds18b20.requestTemperatures();
  for (int i = 0; i < 3; i++) {
    float t = ds18b20.getTempCByIndex(i);
    data.tempPanels[i] = (t == DEVICE_DISCONNECTED_C) ? 25.0 : t;
  }
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (!isnan(h)) data.ambientHum = h;
  if (!isnan(t)) data.ambientTemp = t;

  // 3. Light & Rain
  data.irradianceV = readVoltage(PIN_REF_CELL, 1.0);
  data.isRaining   = (analogRead(PIN_RAIN_PROBE) < RAIN_WET_THRESHOLD);

  // 4. Position & Water Level (Internal Pullups used)
  data.atStartStop  = (digitalRead(PIN_ENDSTOP_START) == LOW);
  data.atEndStop    = (digitalRead(PIN_ENDSTOP_END) == LOW);
  data.waterLevelOK = (digitalRead(PIN_WATER_LEVEL) == HIGH);
}

// ==========================================
// 7. SAFETY RULES & TRIGGER LOGIC
// ==========================================
bool checkSafetyRules() {
  // Rule 1: Battery undervoltage protection (3S Pack)
  if (data.vBat < VBAT_MIN_SAFE && data.vBat > 2.0) {
    emergencyStop("LOW_BATTERY_VBAT");
    return false;
  }

  // Rule 2: Thermal shock protection (Do not spray hot panels)
  float maxPanelT = max(data.tempPanels[0], max(data.tempPanels[1], data.tempPanels[2]));
  if (maxPanelT > MAX_PANEL_TEMP_C && (currentState == STATE_CLEAN_FORWARD || currentState == STATE_CLEAN_RETURN)) {
    emergencyStop("PANEL_OVERTEMP_THERMAL_SHOCK");
    return false;
  }

  // Rule 3: Dry-run pump protection
  if (!data.waterLevelOK && (currentState == STATE_CLEAN_FORWARD || currentState == STATE_CLEAN_RETURN)) {
    setPump(false); // Cut pump immediately, allow dry brush finish or fault
    Serial.println("[WARN] Water level low - Pump disabled.");
  }

  // Rule 4: Motor stall / limit switch timeout
  if ((currentState == STATE_HOMING || currentState == STATE_CLEAN_FORWARD || currentState == STATE_CLEAN_RETURN) &&
      (millis() - stateStartTime > MOTOR_TIMEOUT_MS)) {
    emergencyStop("MOTOR_TRAVEL_TIMEOUT_JAM");
    return false;
  }

  return true;
}

bool shouldStartCleaning() {
  // Do not clean at night or during rain (rain cleans naturally)
  if (data.isRaining || data.irradianceV < MIN_IRRADIANCE_V) return false;
  if (data.vBat < (VBAT_MIN_SAFE + 0.6)) return false;

  // Condition A: Scheduled morning clean (e.g., 08:00 AM via NTP)
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    if (timeinfo.tm_hour == 8 && timeinfo.tm_min == 0 && dailyCleanCount == 0) {
      dailyCleanCount++;
      return true;
    }
    if (timeinfo.tm_hour == 0) dailyCleanCount = 0; // Reset daily flag at midnight
  }

  // Condition B: Soiling detection (Comparing Reference/Clean Panel B vs Dirty Panel A)
  if (data.pPanelB > 5.0) { // Only evaluate when meaningful solar power is generated
    float dropPct = ((data.pPanelB - data.pPanelA) / data.pPanelB) * 100.0;
    if (dropPct >= EFFICIENCY_DROP_PCT) {
      Serial.printf("[TRIGGER] Soiling detected: %.1f%% power drop\n", dropPct);
      return true;
    }
  }

  return false;
}

// ==========================================
// 8. STATE MACHINE EXECUTION
// ==========================================
void runStateMachine() {
  switch (currentState) {
    case STATE_IDLE:
      setMotor(0, 0);
      setPump(false);
      setModeRelay(false);

      if (shouldStartCleaning()) {
        if (!data.atStartStop) {
          transitionTo(STATE_HOMING);
        } else {
          transitionTo(STATE_CLEAN_FORWARD);
        }
      }
      break;

    case STATE_HOMING:
      setPump(false);
      setModeRelay(true);
      setMotor(180, -1); // Move back toward start stop
      if (data.atStartStop) {
        setMotor(0, 0);
        delay(500);
        transitionTo(STATE_CLEAN_FORWARD);
      }
      break;

    case STATE_CLEAN_FORWARD:
      setModeRelay(true);
      if (data.waterLevelOK) setPump(true);
      setMotor(220, 1); // Forward sweep with water spray

      if (data.atEndStop) {
        setMotor(0, 0);
        setPump(false);
        delay(500);
        transitionTo(STATE_CLEAN_RETURN);
      }
      break;

    case STATE_CLEAN_RETURN:
      setModeRelay(true);
      setPump(false);    // Dry/squeegee wipe on return pass
      setMotor(220, -1); // Return to home

      if (data.atStartStop) {
        setMotor(0, 0);
        setModeRelay(false);
        transitionTo(STATE_IDLE);
      }
      break;

    case STATE_FAULT:
      setMotor(0, 0);
      setPump(false);
      setModeRelay(false);
      // Auto-recover if fault was low battery and battery has recharged > 11.2V
      if (faultReason == "LOW_BATTERY_VBAT" && data.vBat > 11.2) {
        faultReason = "NONE";
        transitionTo(STATE_IDLE);
      }
      break;
  }
}

// ==========================================
// 9. DATA LOGGER (THINGSPEAK)
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
               "&field8=" + String(data.isRaining ? 1 : 0);

  http.begin(url);
  int httpCode = http.GET();
  if (httpCode > 0) {
    Serial.printf("[CLOUD] Logged telemetry. HTTP: %d\n", httpCode);
  }
  http.end();
}

// ==========================================
// 10. SETUP & MAIN LOOP
// ==========================================
void setup() {
  Serial.begin(115200);

  // Configure Actuator Pins
  pinMode(PIN_PUMP_RELAY, OUTPUT);
  pinMode(PIN_MODE_RELAY, OUTPUT);
  pinMode(PIN_L298N_IN1, OUTPUT);
  pinMode(PIN_L298N_IN2, OUTPUT);
  pinMode(PIN_L298N_ENA, OUTPUT);
  emergencyStop("BOOT_INIT"); // Ensure safe state on startup
  faultReason = "NONE";
  currentState = STATE_IDLE;

  // Configure Input Pins
  pinMode(PIN_ENDSTOP_START, INPUT_PULLUP);
  pinMode(PIN_ENDSTOP_END, INPUT_PULLUP);
  pinMode(PIN_WATER_LEVEL, INPUT_PULLUP);

  // Initialize Sensors
  ds18b20.begin();
  dht.begin();

  // Initialize Wi-Fi & NTP
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 20) {
    delay(500);
    Serial.print(".");
    retries++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWi-Fi Connected.");
    configTime(GMT_OFFSET, DAYLIGHT_OFF, NTP_SERVER);
  }

  // Initialize Hardware Watchdog Timer
  esp_task_wdt_init(WDT_TIMEOUT_SEC, true);
  esp_task_wdt_add(NULL);
}

void loop() {
  esp_task_wdt_reset(); // Feed watchdog

  readAllSensors();

  if (checkSafetyRules()) {
    runStateMachine();
  }

  if (millis() - lastCloudLog >= LOG_INTERVAL_MS) {
    logToCloud();
    lastCloudLog = millis();
  }

  delay(100); // 10 Hz control loop
}