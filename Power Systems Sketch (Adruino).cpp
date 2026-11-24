/**
 * SolarGuard — Documented Power Path & Automated Cleaning Firmware
 * Target Board : ESP32-S2-DevKitM-1 (Arduino Core for ESP32)
 *
 * Hardware Power & Signal Topology:
 *   - Panel A (Test)    -> F1 (3 A) -> V tap (R1) -> ACS712-A (OUT tap R5) -> K2 Relay:
 *       * K2 LOW  (NC): D1 (SR560) -> U1 Charger PV in -> U2 BMS (3S 18650)
 *       * K2 HIGH (NO): RL1 (15 Ohm / 25 W dummy load to GND)
 *   - Panel B (Control) -> F2 (3 A) -> V tap (R3) -> ACS712-B (OUT tap R7) -> RL2 (15 Ohm / 25 W)
 *   - Protected Pack    -> U2 BMS -> S1 Master Sw -> F3 (5 A) -> VBAT Rail (9.0 V - 12.6 V):
 *       * U3 LM2596 -> 5.00 V Rail (ESP32-S2, Relay VCC, ACS712 x2, L298N 5V)
 *       * F4 (2 A)  -> K1 Relay -> Pump P1 + Flyback Diode D2
 *       * F5 (2 A)  -> U16 (L298N) -> Wiper Gear Motor M1 + Bulk Cap C1 (220 uF)
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
// 2. ESP32-S2-DEVKITM-1 GPIO PINOUT
// ============================================================================
// Actuation Layer
#define PIN_RELAY_K1_PUMP     15  // K1: F4 (2 A) -> Water Pump P1 + D2
#define PIN_RELAY_K2_MODE     16  // K2: LOW = NC (Charge via D1), HIGH = NO (RL1 15R)
#define PIN_U16_L298N_IN1     17  // U16 Motor Driver IN1 (F5 2 A branch)
#define PIN_U16_L298N_IN2     18  // U16 Motor Driver IN2
#define PIN_U16_L298N_ENA     21  // U16 Motor Driver ENA (PWM Speed)

// Sensing Layer — Strictly ADC1 (GPIO 1-7) to avoid Wi-Fi ADC2 conflict
#define PIN_TAP_R1_VPANEL_A   1   // V tap (R1) post-F1
#define PIN_TAP_R3_VPANEL_B   2   // V tap (R3) post-F2
#define PIN_TAP_VBAT_SENSE    3   // VBAT rail divider (9.0 V - 12.6 V post-S1/F3)
#define PIN_TAP_R5_IPANEL_A   4   // ACS712-A OUT tap (R5)
#define PIN_TAP_R7_IPANEL_B   5   // ACS712-B OUT tap (R7)
#define PIN_REF_SOLAR_CELL    6   // 0.45 W Mini Solar Panel (Irradiance reference)
#define PIN_RAIN_DETECTOR     7   // Soil moisture / rain sensor AO

// Digital & 1-Wire Sensors
#define PIN_DS18B20_BUS       8   // DS18B20 x3 waterproof probes (4.7 kOhm pull-up)
#define PIN_DHT22_DATA        9   // DHT22 Ambient Temp/Humidity
#define PIN_ENDSTOP_HOME      10  // Start/Home limit switch (Active LOW)
#define PIN_ENDSTOP_FAR       11  // End/Far limit switch (Active LOW)
#define PIN_LIQUID_LEVEL      12  // Optical water level sensor

// ============================================================================
// 3. CALIBRATION & PROTECTION THRESHOLDS
// ============================================================================
#define WDT_TIMEOUT_S         15

// Voltage Divider Ratios: (R_top + R_bottom) / R_bottom
const float RATIO_R1_PANELA   = 5.54;  // Calibrate to match R1 divider resistors
const float RATIO_R3_PANELB   = 5.54;  // Calibrate to match R3 divider resistors
const float RATIO_VBAT        = 4.03;  // 100k / 33k divider (12.6 V -> 3.12 V)
const float ADC_VREF          = 3.30;
const float ADC_RESOLUTION    = 4095.0;

// ACS712 30 A Scaling via OUT taps R5 & R7 (10k / 20k divider on 5 V sensor)
// 2.50 V quiescent * (20k / 30k) = 1.667 V; 66 mV/A * (20k / 30k) = 44 mV/A
const float ACS_ZERO_OFFSET_V = 1.667;
const float ACS_SENSITIVITY   = 0.044;

// Safety & Cleaning Trigger Limits
const float VBAT_MIN_CUTOFF   = 9.6;    // Lockout before U2 BMS 9.0 V hard cutoff
const float VBAT_RECOVER_V    = 11.4;   // Hysteresis voltage to clear low-battery fault
const float MAX_GLASS_TEMP_C  = 65.0;   // Thermal shock prevention limit
const float MIN_SUNLIGHT_V    = 0.80;   // Minimum reference cell voltage for daytime eval
const float SOILING_LOSS_PCT  = 15.0;   // Trigger wash if Panel A < Panel B by >= 15%
const int   RAIN_ADC_LIMIT    = 2000;   // Below this ADC count = active rain

// Timing Constants
const unsigned long EVAL_INTERVAL_MS    = 900000UL; // Evaluate RL1 vs RL2 every 15 min
const unsigned long RELAY_SETTLE_MS     = 1500UL;   // K2 contact & ADC settle time
const unsigned long MOTOR_MAX_RUN_MS    = 45000UL;  // Jam / blown F5 fuse timeout
const unsigned long CLOUD_LOG_PERIOD_MS = 60000UL;  // ThingSpeak upload interval

// ============================================================================
// 4. GLOBAL OBJECTS & STATE MACHINE
// ============================================================================
OneWire oneWire(PIN_DS18B20_BUS);
DallasTemperature panelTempSensors(&oneWire);
DHT ambientDht(PIN_DHT22_DATA, DHT22);

enum CleanerState {
  ST_CHARGE_IDLE,      // K2 = LOW (NC -> D1 -> U1 Charger)
  ST_MATCHED_LOAD_TEST,// K2 = HIGH (NO -> RL1 15R vs Panel B RL2 15R)
  ST_RETURN_HOME,      // Homing carriage to Start end-stop
  ST_WASH_FORWARD,     // K1 Pump ON + Forward brush sweep
  ST_SQUEEGEE_RETURN,  // K1 Pump OFF + Reverse dry pass
  ST_SYSTEM_FAULT      // Safe lockout state
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
  bool k2OnDummyLoad;
} telem;

unsigned long stateEntryMs   = 0;
unsigned long lastEvalMs     = 0;
unsigned long lastCloudMs    = 0;
int scheduledWashDoneToday   = 0;

// ============================================================================
// 5. ACTUATOR & RELAY HARDWARE DRIVERS
// ============================================================================
void driveMotorM1(int pwmSpeed, int direction) {
  // direction: 1 = Forward, -1 = Reverse, 0 = Coast/Stop
  if (direction == 1) {
    digitalWrite(PIN_U16_L298N_IN1, HIGH);
    digitalWrite(PIN_U16_L298N_IN2, LOW);
    analogWrite(PIN_U16_L298N_ENA, pwmSpeed);
  } else if (direction == -1) {
    digitalWrite(PIN_U16_L298N_IN1, LOW);
    digitalWrite(PIN_U16_L298N_IN2, HIGH);
    analogWrite(PIN_U16_L298N_ENA, pwmSpeed);
  } else {
    digitalWrite(PIN_U16_L298N_IN1, LOW);
    digitalWrite(PIN_U16_L298N_IN2, LOW);
    analogWrite(PIN_U16_L298N_ENA, 0);
  }
}

void setPumpRelayK1(bool on) {
  digitalWrite(PIN_RELAY_K1_PUMP, on ? HIGH : LOW);
}

void setK2DummyLoadMode(bool connectToRL1) {
  // LOW  = K2 NC -> D1 (SR560) -> U1 Charger PV in
  // HIGH = K2 NO -> RL1 (15 Ohm / 25 W dummy load)
  digitalWrite(PIN_RELAY_K2_MODE, connectToRL1 ? HIGH : LOW);
  telem.k2OnDummyLoad = connectToRL1;
}

void triggerFaultLockout(const String& reason) {
  driveMotorM1(0, 0);
  setPumpRelayK1(false);
  setK2DummyLoadMode(false); // Fail-safe to NC charging path
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
// 6. ANALOG & DIGITAL SENSOR ACQUISITION
// ============================================================================
float sampleVoltageTap(int gpioPin, float dividerRatio) {
  uint32_t acc = 0;
  for (int i = 0; i < 16; i++) {
    acc += analogRead(gpioPin);
    delay(1);
  }
  float vPin = ((acc / 16.0f) / ADC_RESOLUTION) * ADC_VREF;
  return vPin * dividerRatio;
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
  // Power path taps
  telem.vPanelA = sampleVoltageTap(PIN_TAP_R1_VPANEL_A, RATIO_R1_PANELA);
  telem.iPanelA = sampleCurrentTap(PIN_TAP_R5_IPANEL_A);
  telem.wPanelA = telem.vPanelA * telem.iPanelA;

  telem.vPanelB = sampleVoltageTap(PIN_TAP_R3_VPANEL_B, RATIO_R3_PANELB);
  telem.iPanelB = sampleCurrentTap(PIN_TAP_R7_IPANEL_B);
  telem.wPanelB = telem.vPanelB * telem.iPanelB;

  telem.vBat    = sampleVoltageTap(PIN_TAP_VBAT_SENSE, RATIO_VBAT);

  // Thermal sensors
  panelTempSensors.requestTemperatures();
  for (int i = 0; i < 3; i++) {
    float c = panelTempSensors.getTempCByIndex(i);
    telem.tPanel[i] = (c == DEVICE_DISCONNECTED_C) ? 25.0f : c;
  }
  float hum = ambientDht.readHumidity();
  float tmp = ambientDht.readTemperature();
  if (!isnan(hum)) telem.hAmbient = hum;
  if (!isnan(tmp)) telem.tAmbient = tmp;

  // Environmental & mechanical states
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
  // 1. Protect 3S 18650 pack above U2 BMS 9.0 V hard shutoff
  if (telem.vBat < VBAT_MIN_CUTOFF && telem.vBat > 2.0f) {
    triggerFaultLockout("VBAT_LOW_BATTERY");
    return false;
  }

  // 2. Prevent thermal shock from cold water spray on hot glass
  float peakGlassC = max(telem.tPanel[0], max(telem.tPanel[1], telem.tPanel[2]));
  if (peakGlassC > MAX_GLASS_TEMP_C && (state == ST_WASH_FORWARD || state == ST_SQUEEGEE_RETURN)) {
    triggerFaultLockout("GLASS_OVERTEMP_THERMAL_SHOCK");
    return false;
  }

  // 3. Dry-run protection for pump P1 (F4 branch)
  if (!telem.tankHasWater && state == ST_WASH_FORWARD) {
    setPumpRelayK1(false);
    Serial.println("[WARN] Water level low — K1 Pump P1 disabled.");
  }

  // 4. Mechanical stall or blown F5 (2 A) motor fuse protection
  if ((state == ST_RETURN_HOME || state == ST_WASH_FORWARD || state == ST_SQUEEGEE_RETURN) &&
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
      setK2DummyLoadMode(false); // Panel A charges 3S pack via D1 (SR560)

      // Scheduled 08:00 AM daily maintenance clean
      struct tm now;
      if (getLocalTime(&now)) {
        if (now.tm_hour == 8 && now.tm_min == 0 && scheduledWashDoneToday == 0 && !telem.raining) {
          scheduledWashDoneToday = 1;
          enterState(telem.atHomeStop ? ST_WASH_FORWARD : ST_RETURN_HOME);
          break;
        }
        if (now.tm_hour == 0) scheduledWashDoneToday