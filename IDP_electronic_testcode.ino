#include "max6675.h"
#include "mqtt_timing.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "DHT.h"
#include <LiquidCrystal_I2C.h>
#include "pitches.h"
#include <esp_task_wdt.h>
#include <functional>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include "DeviceID.h" 

const int thermoDO = 19;
const int thermoCS1 = 23;
const int thermoCS2 = 25;
const int thermoCLK = 5;
const int lcdColumns = 16;
const int lcdRows = 2;
const int LiqSensorPin = 33; 
const int ledPin = 32;        // for testing only
const int relay1 = 14;
const int relay2 = 12;
const int relay3 = 26;
const int buzzer = 27;

#define DHTPIN 4
#define SEALEVELPRESSURE_HPA (1013.25)
#define DHTTYPE DHT22

// Configuration constants
struct Config {
  static constexpr float TEMP_THRESHOLD = 33.00;
  static constexpr float PRESSURE_THRESHOLD = 0.025;
  static constexpr unsigned long WATER_TIMEOUT = 10UL * 60UL * 1000UL;
  static constexpr unsigned long LCD_INTERVAL = 2000;
  static constexpr unsigned long SENSOR_INTERVAL = 2000;
  static constexpr unsigned long FAST_SENSOR_INTERVAL = 500;
  static constexpr unsigned long RETRY_INTERVAL = 100;
  static constexpr unsigned long STARTUP_GRACE_PERIOD = 30000; // 30 seconds grace period
};

// Hardware objects
MAX6675 thermocouple1(thermoCLK, thermoCS1, thermoDO);
MAX6675 thermocouple2(thermoCLK, thermoCS2, thermoDO);
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows); 
Adafruit_BME280 bme1;
Adafruit_BME280 bme2;
DHT dht(DHTPIN, DHTTYPE);

// Global state variables
unsigned long previousLcdMillis = 0;
int lcdState = 0;
volatile bool waterDetected = false;
volatile bool waterDetectedFlag = false;
unsigned long waterNotDetectedStartTime = 0;  // Start time when water not detected
bool finalWarningSent = false;
bool waterTimerRunning = false;
bool airFlowStatus = false;
unsigned long buzzerPreviousMillis = 0;
int buzzerState = 0;
bool previousState1 = false, currentState1 = false;
bool previousState2 = false, currentState2 = false;
unsigned long lastWaterSensorCheck = 0;
const unsigned long WATER_SENSOR_CHECK_INTERVAL = 100; // Check every 100ms
phh::device::DeviceID deviceId; // Keep the original DeviceID class
bool lastAirFlowStatus = false;
unsigned long airFlowLostStartTime = 0;
const unsigned long AIR_FLOW_LOST_CONFIRMATION_TIME = 3000;

// NEW: System initialization tracking
unsigned long systemStartTime = 0;
bool systemFullyInitialized = false;

// System states for better organization
enum SystemState {
  INITIALIZING,
  NORMAL_OPERATION,
  WATER_WARNING,
  WATER_CRITICAL,
  AIRFLOW_WARNING,
  SENSOR_ERROR,
  EMERGENCY_SHUTDOWN
};

SystemState currentSystemState = INITIALIZING;

void IRAM_ATTR waterSensorInterrupt() {
  waterDetectedFlag = true;  // Just set a flag
}

void setup() {
  Serial.begin(115200);
  
  // Record system start time
  systemStartTime = millis();

  // Check if watchdog is already initialized before initializing
  esp_err_t add_result = esp_task_wdt_add(NULL);
  if (add_result == ESP_OK) {
      Serial.println("Task added to watchdog");
  } else if (add_result == ESP_ERR_INVALID_STATE) {
      Serial.println("Task already added to watchdog");
  }

  // Initialize hardware
  initializeHardware();

  // Initialize sensors with non-blocking error handling
  initializeSensors();

  // Setup complete message
  displaySetupSummary();

  // Initialize MQTT
  setupMQTT();

  // Set up interrupt for water sensor
  attachInterrupt(digitalPinToInterrupt(LiqSensorPin), waterSensorInterrupt, CHANGE);
  delay(500);
  
  // Read initial water sensor state but DON'T start timer yet
  waterDetected = digitalRead(LiqSensorPin) == LOW;
  
  Serial.println("-- Smart Prefilled Humidifier Heater System Ready --");
  Serial.println("-- Starting 30-second initialization grace period --");
}

void initializeHardware() {
  lcd.init();
  lcd.backlight();
  
  pinMode(LiqSensorPin, INPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);
  pinMode(relay3, OUTPUT);
  
  // Ensure all relays start OFF
  digitalWrite(relay1, LOW);
  digitalWrite(relay2, LOW);
  digitalWrite(relay3, LOW);
  
  dht.begin();
}

void initializeSensors() {
  // Initialize with minimal blocking
  bool dhtStatus = testSensor("DHT22", []() { return !isnan(dht.readHumidity()); });
  delay(300);
  bool bme1Status = testSensor("BME280-1", []() { return bme1.begin(0x76); });
  bool bme2Status = testSensor("BME280-2", []() { return bme2.begin(0x77); });
  bool tc1Status = testSensor("MAX6675-1", []() { return !isnan(thermocouple1.readCelsius()); });
  bool tc2Status = testSensor("MAX6675-2", []() { return !isnan(thermocouple2.readCelsius()); });
}

bool testSensor(const char* name, bool (*testFunc)()) {
  for (int attempt = 0; attempt < 3; attempt++) {
    if (testFunc()) {
      return true;
    }
    delay(200);
  }
  Serial.print("Warning: ");
  Serial.print(name);
  Serial.println(" failed to initialize!");
  return false;
}

void displaySetupSummary() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("System Ready");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");
  delay(1000);
}

void updateSystemState() {
  // Check if we're still in initialization period
  if (!systemFullyInitialized) {
    if (millis() - systemStartTime >= Config::STARTUP_GRACE_PERIOD) {
      systemFullyInitialized = true;
      Serial.println("-- System fully initialized --");
      
      // Now start water monitoring if needed
      if (!waterDetected) {
        waterNotDetectedStartTime = millis();
        waterTimerRunning = true;
        Serial.println("-- Water monitoring started --");
      }
    } else {
      currentSystemState = INITIALIZING;
      return; // Stay in initialization state
    }
  }
  
  // Normal system state logic
  if (!areCriticalSensorsWorking()) {
    currentSystemState = SENSOR_ERROR;
  } else if (!waterDetected && finalWarningSent) {
    currentSystemState = EMERGENCY_SHUTDOWN;
  } else if (!waterDetected) {
    currentSystemState = WATER_WARNING;
  } else if (!airFlowStatus) {
    currentSystemState = AIRFLOW_WARNING;
  } else {
    currentSystemState = NORMAL_OPERATION;
  }
}

void handleWaterSensorUpdate() {
    if (waterDetectedFlag) {
        waterDetectedFlag = false;
        waterDetected = digitalRead(LiqSensorPin) == LOW;
        digitalWrite(ledPin, waterDetected ? HIGH : LOW);
        
        if (waterDetected && systemFullyInitialized) {
            waterTimerRunning = false;
            waterNotDetectedStartTime = 0;
            finalWarningSent = false;
        }
    }
}

void handleWaterSafety() {
  // Don't handle water safety during initialization
  if (!systemFullyInitialized) {
    return;
  }
  
  if (!waterDetected) {
    if (!waterTimerRunning) {
      waterNotDetectedStartTime = millis();
      waterTimerRunning = true;
    }

    unsigned long elapsedTime = millis() - waterNotDetectedStartTime;
    
    if (!finalWarningSent && elapsedTime >= Config::WATER_TIMEOUT) {
      playFinalWarning();
      finalWarningSent = true;
    }
  } else {
    waterNotDetectedStartTime = 0;
    finalWarningSent = false;
    waterTimerRunning = false;
  }
}

void playFinalWarning() {
  static unsigned long lastBuzzerChange = 0;
  static int buzzerState = 0;
  static int warningCount = 0;
  
  unsigned long currentMillis = millis();
  
  if (currentMillis - lastBuzzerChange >= 500) {
      lastBuzzerChange = currentMillis;
      
      if (buzzerState % 2 == 0) {
          tone(buzzer, NOTE_B7);
      } else {
          noTone(buzzer);
      }
      
      buzzerState++;
      if (buzzerState >= 10) {  // 5 complete cycles (on/off)
          buzzerState = 0;
          warningCount++;
          if (warningCount >= 3) {  // Repeat 3 times
              warningCount = 0;
              return;
          }
      }
      
      esp_task_wdt_reset();  // Reset watchdog
  }
}

void handleTemperatureControl() {
  // Emergency shutdown takes priority
  if (currentSystemState == EMERGENCY_SHUTDOWN || currentSystemState == SENSOR_ERROR) {
    digitalWrite(relay1, LOW);
    digitalWrite(relay2, LOW);
    digitalWrite(relay3, LOW);
    currentState1 = false;
    currentState2 = false;
    return;
  }

  // During initialization, keep heaters off for safety
  if (currentSystemState == INITIALIZING) {
    digitalWrite(relay1, LOW);
    digitalWrite(relay2, LOW);
    digitalWrite(relay3, LOW);
    currentState1 = false;
    currentState2 = false;
    return;
  }

  // Get sensor readings
  float Tt1 = getThermocouple1();
  float Tt2 = getThermocouple2();

  if (isnan(Tt1) || Tt1 < -50 || Tt1 > 150) {
    // Handle sensor error
    Tt1 = -999.0;  // Error indicator
  }
  if (isnan(Tt2) || Tt2 < -50 || Tt2 > 150) {
    // Handle sensor error
    Tt2 = -999.0;  // Error indicator
  }
  
  // Control heaters based on temperature with safety checks
  if (Tt1 > Config::TEMP_THRESHOLD || Tt1 < 0) { // Include safety check
    digitalWrite(relay1, LOW);
    digitalWrite(relay2, LOW);
    currentState1 = false;
  } else {
    digitalWrite(relay1, HIGH);
    digitalWrite(relay2, HIGH);
    currentState1 = true;
  }

  if (Tt2 > Config::TEMP_THRESHOLD || Tt2 < 0) { // Include safety check
    digitalWrite(relay3, LOW);
    currentState2 = false;
  } else {
    digitalWrite(relay3, HIGH);
    currentState2 = true;
  }
}

void handleAirFlowMonitoring() {
  float BME1p = getPressure1();
  float BME2p = getPressure2();
  
  bool currentMeasuredAirFlow = false;
  // Only calculate if both sensors are working
  if (BME1p > 0 && BME2p > 0) {
    float pressureDiff = abs(BME2p - BME1p);
    currentMeasuredAirFlow = (pressureDiff > Config::PRESSURE_THRESHOLD);
  }

  // Airflow lost confirmation logic
  if (currentMeasuredAirFlow) {
    // Air is flowing, reset timer and state immediately
    airFlowStatus = true;
    airFlowLostStartTime = 0;
  } else {
    // No airflow detected
    if (airFlowLostStartTime == 0) {
      airFlowLostStartTime = millis();
    } else if (millis() - airFlowLostStartTime >= AIR_FLOW_LOST_CONFIRMATION_TIME) {
      airFlowStatus = false;
    }
    // If not yet 3 seconds, keep previous airFlowStatus (don't change to false yet)
    // So, do not set airFlowStatus = false until time is up
  }
}

void printState(bool cS1, bool cS2){
  if (cS1 != previousState1){
    Serial.print("Status update: ");
    Serial.println(cS1 ? "Current across Heater 1&2 Flowing" : "Current across Heater 1&2 NOT Flowing");
    previousState1 = cS1;
  }
  if (cS2 != previousState2){
    Serial.print("Status update: ");
    Serial.println(cS2 ? "Current across Heater 3 Flowing" : "Current across Heater 3 NOT Flowing");
    previousState2 = cS2;
  }
}

void handleBuzzerAlerts() {
  switch (currentSystemState) {
    case INITIALIZING:
      // No buzzer during initialization
      noTone(buzzer);
      break;
      
    case SENSOR_ERROR:
      // Intermittent high-pitched alarm
      tone(buzzer, NOTE_C8);
      break;
      
    case EMERGENCY_SHUTDOWN:
      // Handled in playFinalWarning()
      break;
      
    case AIRFLOW_WARNING:
      tone(buzzer, NOTE_A7);
      break;
      
    case WATER_WARNING:
      warningBuzzerNonBlocking();
      break;
      
    case NORMAL_OPERATION:
    default:
      noTone(buzzer);
      break;
  }
}

void warningBuzzerNonBlocking() {
  unsigned long currentMillis = millis();
  const unsigned long pattern[] = {0, 200, 300, 500, 700};  // Durations
  const int steps = sizeof(pattern) / sizeof(pattern[0]);

  if (buzzerState < steps) {
    if (currentMillis - buzzerPreviousMillis >= pattern[buzzerState]) {
      buzzerPreviousMillis = currentMillis;
      if (buzzerState % 2 == 0) {
        tone(buzzer, NOTE_E7);
      } else {
        noTone(buzzer);
      }
      buzzerState++;
    }
  } else {
    buzzerState = 0;  // Reset pattern
  }
}

void handleLcdDisplay() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousLcdMillis < Config::LCD_INTERVAL) {
    return; // Not time to update yet
  }
  
  previousLcdMillis = currentMillis;
  lcd.clear();

  // Priority display system
  switch (currentSystemState) {
    case INITIALIZING:
      displayInitializationScreen();
      break;
      
    case SENSOR_ERROR:
      lcd.setCursor(0, 0);
      lcd.print("SENSOR ERROR");
      lcd.setCursor(0, 1);
      lcd.print("HEATERS OFF");
      break;
      
    case EMERGENCY_SHUTDOWN:
      lcd.setCursor(0, 0);
      lcd.print("EMERGENCY STOP");
      lcd.setCursor(0, 1);
      lcd.print("NO WATER!");
      break;
      
    case AIRFLOW_WARNING:
      lcd.setCursor(0, 0);
      lcd.print("!! Air Flow !!");
      lcd.setCursor(0, 1);
      lcd.print("   Not Flowing");
      break;
      
    case WATER_WARNING:
      displayWaterWarning();
      break;
      
    case NORMAL_OPERATION:
    default:
      displayNormalCycle();
      break;
  }
}

void displayInitializationScreen() {
  unsigned long elapsed = millis() - systemStartTime;
  unsigned long remaining = Config::STARTUP_GRACE_PERIOD - elapsed;
  
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");
  lcd.setCursor(0, 1);
  lcd.print("Wait: ");
  lcd.print(remaining / 1000);
  lcd.print("s");
}

void displayWaterWarning() {
  unsigned long elapsed = millis() - waterNotDetectedStartTime;
  unsigned long remaining = Config::WATER_TIMEOUT - elapsed;

  lcd.setCursor(0, 0);
  lcd.print("Water Level Low");

  if (remaining > 3 * 60 * 1000) {
    lcd.setCursor(0, 1);
    lcd.print("10 mins left");
  } else if (remaining > 1 * 60 * 1000) {
    lcd.setCursor(0, 1);
    lcd.print("3 mins left");
  } else if (remaining > 0) {
    lcd.setCursor(0, 1);
    lcd.print("1 min left");
  } else {
    lcd.setCursor(0, 1);
    lcd.print("Final Warning!");
  }
}

void displayNormalCycle() {
  float h = getHumidity();
  float Tt1 = getThermocouple1();
  float Tt2 = getThermocouple2();
  const char* h1Status = currentState1 ? "Heating" : "Not Heating";
  const char* h3Status = currentState2 ? "Heating" : "Not Heating";

  switch (lcdState) {
    case 0:
      lcd.setCursor(0, 0);
      lcd.print("Device ID:");
      lcd.setCursor(0, 1);
      lcd.print(deviceId.get());
      break;
    case 1:
      lcd.setCursor(0, 0);
      lcd.print("Humidity:");
      lcd.setCursor(0, 1);
      lcd.print(h >= 0 ? h : 0);
      lcd.print("%");
      break;
    case 2:
      lcd.setCursor(0, 0);
      lcd.print("Heater Temp:");
      lcd.setCursor(0, 1);
      lcd.print(Tt1 != -999.0 ? Tt1 : 0);
      lcd.print("*C");
      break;
    case 3:
      lcd.setCursor(0, 0);
      lcd.print("Final Temp:");
      lcd.setCursor(0, 1);
      lcd.print(Tt2 != -999.0 ? Tt2 : 0);
      lcd.print("*C");
      break;
    case 4:
      lcd.setCursor(0, 0);
      lcd.print("Heater 1 & 2:");
      lcd.setCursor(0, 1);
      lcd.print(h1Status);
      break;
    case 5:
      lcd.setCursor(0, 0);
      lcd.print("Heater 3:");
      lcd.setCursor(0, 1);
      lcd.print(h3Status);
      break;
    case 6:
      lcd.setCursor(0, 0);
      lcd.print("Air Flow:");
      lcd.setCursor(0, 1);
      lcd.print(airFlowStatus ? "Flowing" : "Not Flowing");
      break;
    case 7:
      lcd.setCursor(0, 0);
      lcd.print("Water Level");
      lcd.setCursor(0, 1);
      lcd.print(waterDetected ? "Detected" : "Not Detected");
      break;
  }
  lcdState = (lcdState + 1) % 8;
}

void loop() {
  esp_task_wdt_reset(); // Reset watchdog
  
  // Update sensors efficiently
  updateSensorsIfNeeded();

  handleWaterSensorUpdate();

  // Update airflow status first (needed for system state)
  handleAirFlowMonitoring();
  
  // Determine current system state
  updateSystemState();
  
  // Handle all subsystems
  handleWaterSafety();
  handleTemperatureControl();
  handleBuzzerAlerts();
  handleLcdDisplay();
  handleMQTT();

  // Publish sensor data periodically (only after full initialization)
  if (systemFullyInitialized) {
    unsigned long currentMillis = millis();
    if (currentMillis - lastPublishMillis >= publishInterval) {
        lastPublishMillis = currentMillis;
        
          publishSensorData(
          getThermocouple1(),     // heater_temp
          getThermocouple2(),     // final_temp
          getHumidity(),          // humidity
          getPressure1(),         // pressure1
          getPressure2(),         // pressure2
          waterDetected,          // water_detected
          currentState1,          // heater1_state (relay1)
          currentState1,          // heater2_state (relay2)
          currentState2           // heater3_state (relay3)
        );
    }
  }
  
  // Add small delay to prevent overwhelming the system
  delay(10);
}
