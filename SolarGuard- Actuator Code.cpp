/**
 * SolarGuard — Documented Power & Actuation Firmware
 * Target Board : ESP32-S2-DevKitM-1 (Arduino Core for ESP32 v3.x)
 *
 * Actuation & Relay Specification (Per Actuation Diagram):
 *   - U15 Relay Module (ACTIVE-LOW: IN LOW energizes coil, IN HIGH de-energizes):
 *       * GPIO 15 -> IN1 (K1): HIGH = OFF, LOW = ON (F4 2A -> K1 COM-NO -> P1 Pump + D2 1N4007)
 *       * GPIO 16 -> IN2 (K2): HIGH = NC (Normal Mode: D1 -> U1 PV+), LOW = NO (Measure Mode: RL1 15R)
 *   - U16 L298N Motor Driver (ENA & 5V-EN jumpers REMOVED; +5V fed from 5V rail):
 *       * GPIO 17 -> ENA (10 kHz PWM speed control)
 *       * GPIO 18 -> IN1 (HIGH/LOW = Forward Down; LOW/HIGH = Reverse Up; equal = Stop)
 *       * GPIO 21 -> IN2
 *       * M1 (25GA-370) across OUT1/OUT2 with C2 (100 nF) snubber; C1 (220 uF / 25 V) on +12V/GND
 */

#include <WiFi.h>
#include <HTTPClient.h>
#include <time.h>
#include <esp_task_wdt.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>

// ============================================================================
// 1. WI-FI, CLOUD & NTP CONFIGURATION
// ============================================================================
const char* WIFI_SSID       = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD   = "YOUR_WIFI_PASSWORD";

const char* TS_UPDATE_URL   = "http://api.thingspeak.com/update";
const String TS_API_KEY     = "YOUR_THINGSPEAK_WRITE_API_KEY";

const char* NTP_SERVER      = "pool.ntp.org";
const long  GMT_OFFSET_SEC  = 19800; // UTC+5:30 (IST)
const int   DAYLIGHT_OFFSET = 0;

// ============================================================================
// 2. ESP32-S2-DEVKITM-1 GPIO PINOUT (MATCHED TO ACTUATION DIAGRAM)
// ============================================================================
// U15 Relay Module (Active-LOW inputs)
#define PIN_U15_IN1_K1_PUMP   15  // LOW = Energize K1 (P1 Pump ON), HIGH = De-energize (OFF)
#define PIN_U15_IN2_K2_MODE   16  // HIGH = NC (Charge Mode), LOW = NO (Measure Mode RL1 15R)

// U16 L298N Motor Driver (Jumpers ENA & 5V-EN removed)
#define PIN_U16_L298N_ENA     17  // 10 kHz PWM Speed Control
#define PIN_U16_L298N_IN1     18  // Direction Bit 1
#define PIN_U16_L298N_IN2     21  // Direction Bit 2

// PWM Configuration for U16 ENA (10 kHz, 8-bit resolution 0-255)
#define PWM_FREQ_HZ           10000
#define PWM_RES_BITS          8

// Sensing Layer — ADC1 (GPIO 1-7)
#define PIN_TAP_R1_VPANEL_A   1   // V tap (R1) post-F1
#define PIN_TAP_R3_VPANEL_B   2   // V tap (R3) post-F2
#define PIN_TAP_VBAT_SENSE    3   // VBAT rail divider (9.0 V - 12.6 V)
#define PIN_TAP_R5_IPANEL_A   4   // U4 ACS712-A OUT tap (R5)
#define PIN_TAP_R7_IPANEL_B   5   // ACS712-B OUT tap (R7)
#define PIN_REF_SOLAR_CELL    6   // 0.45 W Mini Solar Panel
#define PIN_RAIN_DETECTOR     7   // Soil moisture / rain probe

// Environmental & Limit Sensors
#define PIN_DS18B20_BUS       8   // DS18B20 x3 waterproof probes (4.7 kOhm pull-up)
#define PIN_DHT22_DATA        9   // DHT22 Ambient Temp/Humidity
#define PIN_ENDSTOP_HOME      10  // Top/Start limit switch (Active LOW)
#define PIN_ENDSTOP_FAR       11  // Bottom/End limit switch (Active LOW)
#define PIN_LIQUID_LEVEL      12  // Optical water level sensor

// ============================================================================
// 3. CALIBRATION & PROTECTION THRESHOLDS
// ============================================================================
#define WDT_TIMEOUT_S         15

const float RATIO_R1_PANELA   = 5.54;
const float RATIO_R3_PANELB   = 5.54;
const float RATIO_VBAT        = 4.03;
const float ADC_VREF          = 3.30;
const float ADC_RESOLUTION    = 4095.0;

// U4 ACS712-A / ACS712-B via OUT taps R5 & R7 (10k / 20k divider)
const float ACS_ZERO_OFFSET_V = 1.667;
const float ACS_SENSITIVITY   = 0.044;

const float VBAT_MIN_CUTOFF   = 9.6;
const float VBAT_RECOVER_V    = 11.4;
const float MAX_GLASS_TEMP_C  = 65.0;
const float MIN_SUNLIGHT_V    = 0.80;
const float SOILING_LOSS_PCT  = 15.0;
const int   RAIN_ADC_LIMIT    = 2000;

const unsigned long EVAL_INTERVAL_MS    = 900000UL;
const unsigned long RELAY_SETTLE_MS     = 1500UL;
const unsigned long MOTOR_MAX_RUN_MS    = 45000UL;
const unsigned long CLOUD_LOG_PERIOD_MS = 60000UL;

// ============================================================================
// 4. GLOBAL OBJECTS & STATE MACHINE
// ============================================================================
OneWire oneWire(PIN_DS18B20_BUS);
DallasTemperature panelTempSensors(&oneWire);
DHT ambientDht(PIN_DHT22_DATA, DHT22);

enum CleanerState {
  ST_CHARGE_IDLE,       // K2 HIGH (NC -> D1 -> U1 PV+ charge mode)
  ST_MEASURE_MODE_TEST, // K2 LOW  (NO -> RL1 15R measure mode vs Panel B RL2)
  ST_RETURN_UP_HOME,    // IN1=L, IN2=H -> Reverse (Up) to Home stop
  ST_WASH_DOWN_FORWARD, // K1 LOW (Pump ON) + IN1=H, IN2=L -> Forward (Down)
  ST_WIPE_UP_RETURN,    // K1 HIGH (Pump OFF) + IN1=L, IN2=H -> Reverse (Up)
  ST_SYSTEM_FAULT
};

CleanerState state = ST_CHARGE_IDLE;
String activeFault = "NONE";

struct SystemTelemetry {
  float vPanelA, iPanelA, wPanelA;
  float vPanelB, iPanelB, wPanelB;
  float vBat;
  float tPanel[3];
  float tAmbient, hAmbient;
  float sunRefV;
  bool raining;
  bool tankHasWater;
  bool atHomeStop;
  bool atFarStop;
  bool k2MeasureModeActive;
} telem;

unsigned long stateEntryMs   = 0;
unsigned long lastEvalMs     = 0;
unsigned long lastCloudMs    = 0;
int scheduledWashDoneToday   = 0;

// ============================================================================
// 5. ACTUATOR & U15 ACTIVE-LOW RELAY DRIVERS
// ============================================================================
void driveMotorM1(uint8_t dutyCycle, int direction) {
  // Per U16 L298N Diagram:
  // direction ==  1: IN1 = HIGH, IN2 = LOW  -> Forward (Down)
  // direction == -1: IN1 = LOW,  IN2 = HIGH -> Reverse (Up)
  // direction ==  0: IN1 = LOW,  IN2 = LOW  -> Both equal = Stop
  if (direction == 1) {
    digitalWrite(PIN_U16_L298N_IN1, HIGH);
    digitalWrite(PIN_U16_L298N_IN2, LOW);
    ledcWrite(PIN_U16_L298N_ENA, dutyCycle);
  } else if (direction == -1) {
    digitalWrite(PIN_U16_L298N_IN1, LOW);
    digitalWrite(PIN_U16_L298N_IN2, HIGH);
    ledcWrite(PIN_U16_L298N_ENA, dutyCycle);
  } else {
    digitalWrite(PIN_U16_L298N_IN1, LOW);
    digitalWrite(PIN_U16_L298N_IN2, LOW);
    ledcWrite(PIN_U16_L298N_ENA, 0);
  }
}

void setPumpRelayK1(bool pumpOn) {
  // U15 Active-LOW Relay Module: IN1 LOW energizes K1 coil (COM -> NO -> P1 Pump)
  digitalWrite(PIN_U15_IN1_K1_PUMP, pumpOn ? LOW : HIGH);
}

void setModeRelayK2(bool measureModeRL1) {
  // U15 Active-LOW Relay Module:
  // measureModeRL1 == false -> GPIO16 HIGH (De-energized / NC -> D1 -> U1 PV+ Charge Mode)
  // measureModeRL1 == true  -> GPIO16 LOW  (Energized / NO -> RL1 15R Measure Mode)
  digitalWrite(PIN_U15_IN2_K2_MODE, measureModeRL1 ? LOW : HIGH);
  telem.k2MeasureModeActive = measureModeRL1;
}

void triggerFaultLockout(const String& reason) {
  driveMotorM1(0, 0);
  setPumpRelayK1(false); // GPIO15 HIGH (Pump OFF)
  setModeRelayK2(false); // GPIO16 HIGH (Normal Charge Mode NC)
  activeFault = reason;
  state = ST_SYSTEM_FAULT;
  Serial.println("[FAULT LOCKOUT] " + reason);
}

void enterState(CleanerState nextState) {
  state = nextState;
  stateEntryMs = millis();
  Serial.printf("[FSM] State -> %d\n", nextState);
}

// ============================================================================
// 6. SENSOR ACQUISITION
// ============================================================================
float sampleVoltageTap(int gpioPin, float dividerRatio) {
  uint32_t acc = 0;
  for (int i = 0; i < 16; i++) {
    acc += analogRead(gpioPin);
    delay(1);
  }
  return ((acc / 16.0f) / ADC_RESOLUTION) * ADC_VREF * dividerRatio;
}

float sampleCurrentTap(int gpioPin) {
  uint32_t acc = 0;
  for (int i = 0; i < 32; i++) {
    acc += analogRead(gpioPin);
    delay(1);
  }
  float vPin = ((acc / 32.0f) / ADC_RESOLUTION) * ADC_VREF;
  float currentA = (vPin - ACS_ZERO_OFFSET_V) / ACS_SENSITIVITY;
  return (fabs(currentA) < 0.08f) ? 0.0f : currentA;
}

void pollSensors() {
  telem.vPanelA = sampleVoltageTap(PIN_TAP_R1_VPANEL_A, RATIO_R1_PANELA);
  telem.iPanelA = sampleCurrentTap(PIN_TAP_R5_IPANEL_A);
  telem.wPanelA = telem.vPanelA * telem.iPanelA;

  telem.vPanelB = sampleVoltageTap(PIN_TAP_R3_VPANEL_B, RATIO_R3_PANELB);
  telem.iPanelB = sampleCurrentTap(PIN_TAP_R7_IPANEL_B);
  telem.wPanelB = telem.vPanelB * telem.iPanelB;

  telem.vBat    = sampleVoltageTap(PIN_TAP_VBAT_SENSE, RATIO_VBAT);

  panelTempSensors.requestTemperatures();
  for (int i = 0; i < 3; i++) {
    float c = panelTempSensors.getTempCByIndex(i);
    telem.tPanel[i] = (c == DEVICE_DISCONNECTED_C) ? 25.0f : c;
  }
  float hum = ambientDht.readHumidity();
  float tmp = ambientDht.readTemperature();
  if (!isnan(hum)) telem.hAmbient = hum;
  if (!isnan(tmp)) telem.tAmbient = tmp;

  telem.sunRefV      = sampleVoltageTap(PIN_REF_SOLAR_CELL, 1.0f);
  telem.raining      = (analogRead(PIN_RAIN_DETECTOR) < RAIN_ADC_LIMIT);
  telem.atHomeStop   = (digitalRead(PIN_ENDSTOP_HOME) == LOW);
  telem.atFarStop    = (digitalRead(PIN_ENDSTOP_FAR) == LOW);
  telem.tankHasWater = (digitalRead(PIN_LIQUID_LEVEL) == HIGH);
}

// ============================================================================
// 7. SAFETY INTERLOCKS
// ============================================================================
bool evaluateSafetyInterlocks() {
  if (telem.vBat < VBAT_MIN_CUTOFF && telem.vBat > 2.0f) {
    triggerFaultLockout("VBAT_LOW_BATTERY");
    return false;
  }

  float peakGlassC = max(telem.tPanel[0], max(telem.tPanel[1], telem.tPanel[2]));
  if (peakGlassC > MAX_GLASS_TEMP_C && (state == ST_WASH_DOWN_FORWARD || state == ST_WIPE_UP_RETURN)) {
    triggerFaultLockout("GLASS_OVERTEMP_THERMAL_SHOCK");
    return false;
  }

  if (!telem.tankHasWater && state == ST_WASH_DOWN_FORWARD) {
    setPumpRelayK1(false);
    Serial.println("[WARN] Tank level low — K1 Pump P1 disabled.");
  }

  if ((state == ST_RETURN_UP_HOME || state == ST_WASH_DOWN_FORWARD || state == ST_WIPE_UP_RETURN) &&
      (millis() - stateEntryMs > MOTOR_MAX_RUN_MS)) {
    triggerFaultLockout("M1_TIMEOUT_OR_F5_FUSE_OPEN");
    return false;
  }

  return true;
}

// ============================================================================
// 8. FINITE STATE MACHINE
// ============================================================================
void stepStateMachine() {
  switch (state) {
    case ST_CHARGE_IDLE:
      driveMotorM1(0, 0);
      setPumpRelayK1(false);
      setModeRelayK2(false); // GPIO16 HIGH -> NC (Charge Mode via D1)

      struct tm now;
      if (getLocalTime(&now)) {
        if (now.tm_hour == 8 && now.tm_min == 0 && scheduledWashDoneToday == 0 && !telem.raining) {
          scheduledWashDoneToday = 1;
          enterState(telem.atHomeStop ? ST_WASH_DOWN_FORWARD : ST_RETURN_UP_HOME);
          break;
        }
        if (now.tm_hour == 0) scheduledWashDoneToday = 0;
      }

      if (!telem.raining && telem.sunRefV >= MIN_SUNLIGHT_V && (millis() - lastEvalMs >= EVAL_INTERVAL_MS)) {
        lastEvalMs = millis();
        setModeRelayK2(true); // GPIO16 LOW -> NO (Measure Mode RL1 15R)
        enterState(ST_MEASURE_MODE_TEST);
      }
      break;

    case ST_MEASURE_MODE_TEST:
      if (millis() - stateEntryMs >= RELAY_SETTLE_MS) {
        float pTestA    = telem.wPanelA;
        float pControlB = telem.wPanelB;
        setModeRelayK2(false); // GPIO16 HIGH -> Restore Normal Charge Mode (NC)

        if (pControlB > 3.0f) {
          float soilingLoss = ((pControlB - pTestA) / pControlB) * 100.0f;
          Serial.printf("[MEASURE MODE] Panel A (RL1): %.2f W | Panel B (RL2): %.2f W | Loss: %.1f%%\n",
                        pTestA, pControlB, soilingLoss);
          if (soilingLoss >= SOILING_LOSS_PCT) {
            enterState(telem.atHomeStop ? ST_WASH_DOWN_FORWARD : ST_RETURN_UP_HOME);
            break;
          }
        }
        enterState(ST_CHARGE_IDLE);
      }
      break;

    case ST_RETURN_UP_HOME:
      setPumpRelayK1(false);
      setModeRelayK2(false);
      driveMotorM1(185, -1); // IN1=LOW, IN2=HIGH -> Reverse (Up)
      if (telem.atHomeStop) {
        driveMotorM1(0, 0);
        delay(400);
        enterState(ST_WASH_DOWN_FORWARD);
      }
      break;

    case ST_WASH_DOWN_FORWARD:
      setModeRelayK2(false);
      if (telem.tankHasWater) setPumpRelayK1(true); // GPIO15 LOW -> Pump P1 ON
      driveMotorM1(225, 1); // IN1=HIGH, IN2=LOW -> Forward (Down)
      if (telem.atFarStop) {
        driveMotorM1(0, 0);
        setPumpRelayK1(false);
        delay(400);
        enterState(ST_WIPE_UP_RETURN);
      }
      break;

    case ST_WIPE_UP_RETURN:
      setModeRelayK2(false);
      setPumpRelayK1(false); // Dry return wipe
      driveMotorM1(225, -1); // IN1=LOW, IN2=HIGH -> Reverse (Up)
      if (telem.atHomeStop) {
        driveMotorM1(0, 0);
        enterState(ST_CHARGE_IDLE);
      }
      break;

    case ST_SYSTEM_FAULT:
      driveMotorM1(0, 0);
      setPumpRelayK1(false);
      setModeRelayK2(false);
      if (activeFault == "VBAT_LOW_BATTERY" && telem.vBat >= VBAT_RECOVER_V) {
        activeFault = "NONE";
        enterState(ST_CHARGE_IDLE);
      }
      break;
  }
}

// ============================================================================
// 9. CLOUD TELEMETRY UPLOAD
// ============================================================================
void pushTelemetryToCloud() {
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  String endpoint = String(TS_UPDATE_URL) + "?api_key=" + TS_API_KEY +
                    "&field1=" + String(telem.wPanelA, 2) +
                    "&field2=" + String(telem.wPanelB, 2) +
                    "&field3=" + String(telem.vBat, 2) +
                    "&field4=" + String(telem.tPanel[0], 1) +
                    "&field5=" + String(telem.tAmbient, 1) +
                    "&field6=" + String(telem.sunRefV, 2) +
                    "&field7=" + String(state) +
                    "&field8=" + String(telem.k2MeasureModeActive ? 1 : 0);

  http.begin(endpoint);
  http.GET();
  http.end();
}

// ============================================================================
// 10. INITIALIZATION & MAIN LOOP
// ============================================================================
void setup() {
  Serial.begin(115200);

  // Set U15 Active-LOW relay pins HIGH BEFORE configuring OUTPUT to prevent boot glitch
  digitalWrite(PIN_U15_IN1_K1_PUMP, HIGH);
  digitalWrite(PIN_U15_IN2_K2_MODE, HIGH);
  pinMode(PIN_U15_IN1_K1_PUMP, OUTPUT);
  pinMode(PIN_U15_IN2_K2_MODE, OUTPUT);

  // Configure U16 L298N direction pins & 10 kHz PWM on GPIO 17
  pinMode(PIN_U16_L298N_IN1, OUTPUT);
  pinMode(PIN_U16_L298N_IN2, OUTPUT);
  digitalWrite(PIN_U16_L298N_IN1, LOW);
  digitalWrite(PIN_U16_L298N_IN2, LOW);
  ledcAttach(PIN_U16_L298N_ENA, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcWrite(PIN_U16_L298N_ENA, 0);

  pinMode(PIN_ENDSTOP_HOME, INPUT_PULLUP);
  pinMode(PIN_ENDSTOP_FAR, INPUT_PULLUP);
  pinMode(PIN_LIQUID_LEVEL, INPUT_PULLUP);

  panelTempSensors.begin();
  ambientDht.begin();

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    attempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET, NTP_SERVER);
  }

  esp_task_wdt_init(WDT_TIMEOUT_S, true);
  esp_task_wdt_add(NULL);
}

void loop() {
  esp_task_wdt_reset();
  pollSensors();

  if (evaluateSafetyInterlocks()) {
    stepStateMachine();
  }

  if (millis() - lastCloudMs >= CLOUD_LOG_PERIOD_MS) {
    pushTelemetryToCloud();
    lastCloudMs = millis();
  }

  delay(100);
}